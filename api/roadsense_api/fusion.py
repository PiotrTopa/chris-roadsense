"""Fusion of the map layer (mapd `map` messages) with the camera perception: the api-derived topic `fused`.

Every fact is combined on its own, keeps the unfused inputs next to the result and names its source:

    speed_limit   sign read by the camera > sign held until the next map junction > tagged map limit >
                  implicit map limit (road class + built-up state) > the vision default > unknown
    built_up      area sign > map tag > road class > landuse (with envd) > envd
    place         envd's smoothed scores multiplied by a prior from the matched road
    lanes         lane counts from the map, ego lane and oncoming neighbour with the model's lane lines
    tunnel        map tunnel (with the distance to its end) and / or envd's tunnel label
    lead          the tracked lead vehicle (lead.py; vision + the car's own motion), a copy of the `lead` topic

With no map (no mapd, no pack, no position, or a poor match) every fact falls back to its vision value
and says so; the vision topics themselves are never modified.

Match-confidence hold: the matcher's confidence dips below `min_match_conf` for a message or two where a side
road branches off while the matched road stays the same; without a hold every fact (the speed limit above all)
would fall back to vision for that moment. A low-confidence message is still used, with the last good
confidence, when the map is otherwise available and fresh, it is the same road as the last confident message
(same edge, or the same class with the same name / ref), its map speed limit is unchanged, and it comes within
`match_hold_s` and `match_hold_msgs` of that message. `map.match_hold` says so. Never across a change of the
matched road, an unavailable or stale map (a real loss of position) or a changed map limit.
"""
MAP_MAX_AGE_S = 3.0
PLACES = ("highway", "city_street", "residential", "rural_road", "forest_road", "tunnel", "parking_lot",
          "underground_parking", "gas_station", "bridge")
DEFAULT_CFG = dict(
    min_match_conf=0.5,          # below this the matched road is not used
    hold_cap_m=5000.0,           # a held sign limit ends after this distance at the latest
    hold_min_m=20.0,             # junctions end a sign limit only from this far past the sign
    map_change_min_m=100.0,      # a tagged limit change this far after the sign overrides the sign ...
    map_change_min_conf=0.6,     # ... only from a match this confident and not ambiguous (a match flipping between
    map_change_hold_m=40.0,      #     two roads is not a change of the limit), once the new limit has held this far
    junction_pass_m=10.0,
    lane_prob=0.5, lane_prob_inner=0.3,
    lanes_max_age_s=1.0,         # a `lanes` (tracker) message older than this is not used for the ego lane
    tunnel_env_conf=0.5,
    match_hold_s=2.0,            # a low-confidence match of the same road is used this long after a good one
    match_hold_msgs=3,           # and for at most this many low-confidence map messages in a row
)
IMPLICIT_AREA_RULES = ("built_up", "outside", "outside_dual_2x2")


class Fusion:
    def __init__(self, defaults_kmh=None, cfg=None):
        self.cfg = dict(DEFAULT_CFG, **(cfg or {}))
        self.defaults = defaults_kmh or {"built_up": 50, "outside": 90, "outside_dual_2x2": 100}
        self.seq = 0
        self.hold = None             # the sign limit being followed: value, track, odometers, map state at the read
        self.junctions_passed = 0
        self._pending_j = None       # absolute map odometer of the next real junction ahead
        self._last_map_seq = None
        self._good_match = None      # the last map message matched with confidence: ts, confidence, road identity
        self._low_seqs = set()       # low-confidence map messages since then

    @staticmethod
    def _road_id(m):
        """What has to stay the same for a match-confidence hold: the edge, the road and its map limit."""
        r = m.get("road") or {}
        sl = m.get("speed_limit") or {}
        return {"edge": ((m.get("match") or {}).get("edge"), (m.get("match") or {}).get("dir")),
                "road": (r.get("class"), r.get("name"), r.get("ref")),
                "named": bool(r.get("name") or r.get("ref")),
                "limit": (sl.get("limit_kmh"), sl.get("source"), sl.get("rule"))}

    def _match_hold(self, m, now_mono_ns):
        """The low-confidence `m` as a usable map message with the last good confidence, or None."""
        g = self._good_match
        if g is None or not (m.get("match") or {}).get("matched") or m.get("position") is None:
            return None
        if m.get("ts_mono_ns") is None or (m["ts_mono_ns"] - g["ts"]) / 1e9 > self.cfg["match_hold_s"]:
            return None
        rid = self._road_id(m)
        same_edge = rid["edge"][0] is not None and rid["edge"] == g["id"]["edge"]
        same_road = rid["named"] and rid["road"] == g["id"]["road"]
        if not (same_edge or same_road) or rid["limit"] != g["id"]["limit"]:
            return None
        self._low_seqs.add(m.get("seq"))
        if len(self._low_seqs) > self.cfg["match_hold_msgs"]:
            return None
        held = dict(m, match=dict(m["match"], confidence=g["confidence"]))
        return held, {"messages": len(self._low_seqs), "age_s": round((m["ts_mono_ns"] - g["ts"]) / 1e9, 2),
                      "confidence": g["confidence"]}

    # ---- helpers -----------------------------------------------------------------------------
    def _map(self, latest, now_mono_ns):
        m = latest.get("map")
        if not m:
            return None, {"available": False, "state": None, "reason": "no_mapd", "age_s": None,
                          "match_confidence": None}
        age = (now_mono_ns - m["ts_mono_ns"]) / 1e9 if m.get("ts_mono_ns") is not None else None
        info = {"available": bool(m.get("available")), "state": m.get("state"), "reason": m.get("reason"),
                "age_s": None if age is None else round(age, 2),
                "match_confidence": (m.get("match") or {}).get("confidence")}
        if age is not None and age > MAP_MAX_AGE_S:
            info.update(available=False, reason="stale")
            self._good_match = None                      # a real loss: no hold across it
            return None, info
        if not m.get("available"):
            self._good_match = None
            return None, info
        if (info["match_confidence"] or 0.0) < self.cfg["min_match_conf"]:
            hold = self._match_hold(m, now_mono_ns)
            if hold is not None:
                info["match_hold"] = hold[1]
                return hold[0], info
            self._good_match = None                      # road / limit changed or the hold ran out: no more
            info.update(available=False, reason="low_match_confidence")
            return None, info
        if m.get("ts_mono_ns") is not None:
            self._good_match = {"ts": m["ts_mono_ns"], "confidence": info["match_confidence"],
                                "id": self._road_id(m)}
            self._low_seqs = set()
        return m, info

    def _track_junctions(self, m):
        """Count real junctions (not driveways / service roads) the car passes, from the map look-ahead."""
        if m.get("seq") == self._last_map_seq:
            return
        self._last_map_seq = m.get("seq")
        odo = (m.get("position") or {}).get("odometer_m")
        if odo is None:
            return
        nxt = None
        for j in (m.get("ahead") or {}).get("junctions") or ():
            if j.get("minor") or j.get("kind") not in ("junction", "roundabout"):
                continue
            nxt = odo + j["dist_m"]
            break
        p = self._pending_j
        if p is not None and odo >= p - self.cfg["junction_pass_m"] and (nxt is None or nxt > p + 25.0):
            self.junctions_passed += 1
            p = None
        if nxt is not None and (p is None or abs(nxt - p) < 25.0 or nxt < p):
            p = nxt
        self._pending_j = p

    # ---- facts -------------------------------------------------------------------------------
    def _built_up(self, m, vis):
        area = (vis or {}).get("area") or {}
        v_type, v_src = area.get("type"), area.get("source")
        v_val = True if v_type == "built_up" else False if v_type == "outside" else None
        vision = {"value": v_val, "source": v_src, "confidence": area.get("confidence")} if vis else None
        mp = (m.get("road") or {}).get("built_up") if m else None
        mc = (m.get("match") or {}).get("confidence", 0.0) if m else 0.0
        if v_val is not None and v_src == "sign":
            out = {"value": v_val, "source": "sign", "confidence": area.get("confidence")}
        elif mp and mp["source"] == "tagged":
            out = {"value": mp["value"], "source": "map_tagged", "confidence": round(0.85 * mc, 2)}
        elif mp and mp["source"] == "class":
            out = {"value": mp["value"], "source": "map_class", "confidence": round(0.8 * mc, 2)}
        elif mp:
            agree = v_val is not None and v_val == mp["value"]
            out = {"value": mp["value"], "source": "map_landuse+env" if agree else "map_landuse",
                   "confidence": round((0.7 if agree else 0.45) * mc, 2)}
        elif v_val is not None and v_src == "env":
            out = {"value": v_val, "source": "env", "confidence": area.get("confidence")}
        else:
            out = {"value": None, "source": "none", "confidence": 0.0}
        out["vision"] = vision
        out["map"] = mp
        return out

    def _speed_limit(self, m, vis, built_up):
        c = self.cfg
        mp = m.get("speed_limit") if m else None
        mc = (m.get("match") or {}).get("confidence", 0.0) if m else 0.0
        road = (m.get("road") or {}) if m else {}
        modo = (m.get("position") or {}).get("odometer_m") if m else None
        map_tagged = mp["limit_kmh"] if mp and mp.get("source") == "maxspeed" else None
        vsrc = (vis or {}).get("source") or {}
        vis_sign = bool(vis) and vsrc.get("kind") == "sign" and vis.get("limit_kmh") is not None
        vodo = (vis or {}).get("odometer_m")
        note = None
        # follow the sign the camera read
        if vis_sign:
            key = (vsrc.get("track_id"), vis["limit_kmh"])
            if self.hold is None or self.hold["key"] != key:
                self.hold = {"key": key, "value": vis["limit_kmh"], "sign": vsrc.get("sign"),
                             "track_id": vsrc.get("track_id"), "vodo": vodo, "modo": modo,
                             "map_tagged": map_tagged, "junctions": self.junctions_passed,
                             "built_up": built_up["value"], "ended": None, "had_map": m is not None,
                             "armed": False}
        h = self.hold
        if h is not None and h["ended"] is None:
            travelled = None
            if modo is not None and h["modo"] is not None:
                travelled = modo - h["modo"]
            elif vodo is not None and h["vodo"] is not None:
                travelled = vodo - h["vodo"]
            # the camera reads a sign before the car reaches it, and signs stand right after junctions:
            # junctions count only from hold_min_m past the sign (the vision engine knows where the sign is)
            if not h["armed"]:
                ex = (vis or {}).get("expiry") if vis_sign else None
                past = (ex["distance_cap_m"] - ex["remaining_m"]) if ex and ex.get("remaining_m") is not None \
                    else (travelled - 40.0 if travelled is not None else None)
                if past is not None and past >= c["hold_min_m"]:
                    h["armed"], h["junctions"] = True, self.junctions_passed
            if travelled is not None and travelled < -1.0:
                h["ended"] = "odometer restarted"
            elif m is not None and h["had_map"]:
                limited_access = road.get("class") in ("motorway", "motorway_link") or road.get("motorroad") \
                    or road.get("expressway")
                differs = map_tagged is not None and map_tagged != h["map_tagged"] and map_tagged != h["value"]
                # the new limit has to come from a sure match and to last: on the first recorded drive with a
                # position the match flipped between two roads and ended the sign 24 times in six minutes
                sure = mc >= c["map_change_min_conf"] and not (m.get("match") or {}).get("ambiguous")
                chg = h.get("chg")
                if not differs:
                    h["chg"] = chg = None
                elif sure and (chg is None or chg[0] != map_tagged):
                    h["chg"] = chg = (map_tagged, modo)
                if differs and sure and chg is not None and (travelled or 0.0) >= c["map_change_min_m"] and (
                        modo is None or chg[1] is None or modo - chg[1] >= c["map_change_hold_m"]):
                    h["ended"] = "map limit changed to %d after the sign" % map_tagged
                elif not limited_access and h["armed"] and self.junctions_passed > h["junctions"]:
                    h["ended"] = "junction passed (map)"
                elif built_up["value"] is not None and h["built_up"] is not None \
                        and built_up["value"] != h["built_up"] and built_up["source"] in ("sign", "map_tagged"):
                    h["ended"] = "built-up area boundary"
                elif travelled is not None and travelled > c["hold_cap_m"]:
                    h["ended"] = "distance cap"
            elif not vis_sign:
                h["ended"] = "vision ended it (no map)"        # without a map the vision engine alone decides
        vision = None
        if vis:
            vision = {"limit_kmh": vis.get("limit_kmh"), "kind": vsrc.get("kind"),
                      "sign": vsrc.get("sign"), "rule": vsrc.get("rule"), "confidence": vis.get("confidence")}
        if h is not None and h["ended"] is None:
            agree = map_tagged == h["value"]
            if vis_sign:
                src, conf = "sign", min(1.0, (vis.get("confidence") or 0.0) + (0.1 if agree else 0.0))
            else:
                src, conf = "sign_held", 0.6 + (0.2 if agree else 0.0)
                note = "sign limit kept until the next junction on the map"
            out = {"limit_kmh": h["value"], "source": src, "confidence": round(conf, 2), "rule": None,
                   "sign": h["sign"], "track_id": h["track_id"]}
        else:
            if h is not None and vis_sign and h["ended"]:
                note = "sign limit ended: " + h["ended"]
            if h is not None and not vis_sign:
                self.hold = None
            if map_tagged is not None:
                out = {"limit_kmh": map_tagged, "source": "map", "confidence": round(mp["confidence"] * mc, 2),
                       "rule": None, "detail": mp.get("detail")}
            elif mp and mp.get("limit_kmh") is not None:
                rule, val, conf = mp["rule"], mp["limit_kmh"], mp["confidence"] * mc
                if rule in IMPLICIT_AREA_RULES and built_up["value"] is not None \
                        and built_up["source"] in ("sign",) and built_up["value"] != (rule == "built_up"):
                    # the camera saw the area sign; the pack only inferred the area
                    rule = "built_up" if built_up["value"] else ("outside_dual_2x2" if rule == "outside_dual_2x2" else "outside")
                    val, conf = self.defaults.get(rule, val), (built_up.get("confidence") or 0.5)
                    note = "implicit limit with the built-up state from the area sign"
                out = {"limit_kmh": val, "source": "map_implicit", "confidence": round(conf, 2), "rule": rule}
            elif vis and vis.get("limit_kmh") is not None:
                out = {"limit_kmh": vis["limit_kmh"], "source": "vision_default" if not vis_sign else "sign",
                       "confidence": vis.get("confidence"), "rule": vsrc.get("rule")}
            else:
                out = {"limit_kmh": None, "source": "unknown", "confidence": 0.0, "rule": None}
        mv = mp.get("limit_kmh") if mp else None
        vv = vis.get("limit_kmh") if vis else None
        out["agreement"] = ("none" if mv is None and vv is None else "map_only" if vv is None else
                            "vision_only" if mv is None else "agree" if mv == vv else "conflict")
        out["note"] = note
        out["variable"] = bool(mp and mp.get("variable"))
        out["vision"] = vision
        out["map"] = ({k: mp.get(k) for k in ("limit_kmh", "source", "rule", "detail", "confidence")} if mp else None)
        return out

    @staticmethod
    def _place_prior(road):
        w = dict.fromkeys(PLACES, 1.0)
        cls = road.get("class")
        bu = (road.get("built_up") or {}).get("value")
        if road.get("tunnel"):
            w["tunnel"] *= 20.0
            for k in PLACES:
                if k not in ("tunnel", "underground_parking"):
                    w[k] *= 0.3
            if road.get("parking") or cls == "service":
                w["underground_parking"] *= 3.0
            return w
        w["tunnel"] *= 0.15
        if road.get("parking"):
            under = road.get("covered") or (road.get("layer") or 0) < 0
            w["underground_parking" if under else "parking_lot"] *= 8.0
            return w
        w["underground_parking"] *= 0.3
        if road.get("bridge"):
            w["bridge"] *= 3.0
        if cls in ("motorway", "motorway_link") or road.get("motorroad") or road.get("expressway"):
            w["highway"] *= 6.0
            w["city_street"] *= 0.4
            w["residential"] *= 0.2
            return w
        if cls in ("trunk", "trunk_link") or road.get("dual_carriageway") and not bu:
            w["highway"] *= 2.5
        else:
            w["highway"] *= 0.4
        if road.get("forest"):
            w["forest_road"] *= 5.0
            w["city_street"] *= 0.3
            w["residential"] *= 0.4
        elif bu:
            if cls in ("residential", "living_street", "service"):
                w["residential"] *= 3.0
                w["city_street"] *= 1.5
            else:
                w["city_street"] *= 2.5
                w["residential"] *= 1.2
            w["rural_road"] *= 0.4
            w["forest_road"] *= 0.3
        else:
            w["rural_road"] *= 3.0
            w["city_street"] *= 0.5
            w["residential"] *= 0.6
        return w

    def _place(self, m, env):
        ep = (env or {}).get("place") or {}
        vision = {"label": ep.get("label"), "confidence": ep.get("confidence")} if ep else None
        if not m:
            return {"label": ep.get("label"), "confidence": ep.get("confidence"),
                    "source": "vision" if ep else "none", "scores": ep.get("scores"), "vision": vision, "map": None}
        mc = (m.get("match") or {}).get("confidence", 0.0)
        w = self._place_prior(m["road"])
        map_label = max(w, key=w.get)
        tot = sum(w.values())
        mapinfo = {"label": map_label, "prior": {k: round(v / tot, 3) for k, v in w.items()}}
        scores = ep.get("scores")
        if not scores:
            return {"label": map_label, "confidence": round(w[map_label] / tot * mc, 2), "source": "map",
                    "scores": mapinfo["prior"], "vision": vision, "map": mapinfo}
        post = {k: max(1e-4, float(scores.get(k, 0.0))) * w.get(k, 1.0) ** mc for k in set(scores) | set(w)}
        z = sum(post.values())
        post = {k: v / z for k, v in post.items()}
        label = max(post, key=post.get)
        return {"label": label, "confidence": round(post[label], 3),
                "source": "fused" if label == ep.get("label") == map_label else
                "fused(map)" if label != ep.get("label") else "fused(vision)",
                "scores": {k: round(v, 3) for k, v in post.items()}, "vision": vision, "map": mapinfo}

    def _lanes(self, m, road_msg, lanes_msg=None, now_mono_ns=None, egolane=None):
        c = self.cfg
        left = right = None
        ll = (road_msg or {}).get("lane_lines")
        tracked = False
        if isinstance(lanes_msg, dict) and isinstance(lanes_msg.get("lane_to_left"), bool) and (
                now_mono_ns is None or abs(now_mono_ns - (lanes_msg.get("ts_mono_ns") or 0)) <= c["lanes_max_age_s"] * 1e9):
            # the road-geometry tracker's presence (lanes.py): the same question with hysteresis over time
            left, right, tracked = lanes_msg["lane_to_left"], bool(lanes_msg.get("lane_to_right")), True
        elif ll and len(ll) == 4:
            p = [float(x.get("prob") or 0.0) for x in ll]
            left = p[0] >= c["lane_prob"] and p[1] >= c["lane_prob_inner"]
            right = p[3] >= c["lane_prob"] and p[2] >= c["lane_prob_inner"]
        vision = {"lane_to_left": left, "lane_to_right": right, "tracked": tracked} if (ll or tracked) else None
        # the ego-lane filter (egolane.py, lane L20): probabilities per lane, lane changes, the map count
        est = None
        if isinstance(egolane, dict) and "probs" in egolane:
            est = {k: egolane.get(k) for k in ("probs", "probs_from", "ego_from_right", "confidence", "provenance",
                                               "last_change", "evidence")}
        if not m:
            return {"count": None, "forward": None, "backward": None, "ego_index": None, "oncoming_left": None,
                    "ego_from_right": est["ego_from_right"] if est else None,
                    "probs": est["probs"] if est else None, "probs_from": est["probs_from"] if est else None,
                    "source": "vision" if est and est["ego_from_right"] is not None else "none",
                    "confidence": est["confidence"] if est and est["ego_from_right"] is not None else 0.0,
                    "estimator": est, "vision": vision, "map": None}
        ln = m["road"]["lanes"]
        fwd, bwd = ln["forward"], ln["backward"]
        one_dir = bwd == 0
        mc = (m.get("match") or {}).get("confidence", 0.0)
        if est is not None and est["probs_from"] == "left" and len(est["probs"] or ()) == fwd:
            # the filter ran with this road's tagged count: its answer, or "unknown"
            ego = egolane.get("ego_index")
            oncoming = False if one_dir else (None if ego is None else ego == 0)
            return {"count": ln["total"], "forward": fwd, "backward": bwd, "ego_index": ego, "oncoming_left": oncoming,
                    "ego_from_right": est["ego_from_right"], "probs": est["probs"], "probs_from": "left",
                    "source": "map:" + ln["source"] + "+vision",
                    "confidence": round((est["confidence"] or 0.0) * min(1.0, mc / 0.7), 2) if ego is not None else 0.0,
                    "estimator": est, "vision": vision, "map": ln}
        ego = None
        if fwd == 1:
            ego = 0
        elif right is not None:
            if not right:
                ego = fwd - 1                           # nothing to the right: the rightmost lane
            elif fwd == 2:
                ego = 0
            elif one_dir and left is False:
                ego = 0
        elif one_dir and left is False and fwd >= 2:
            ego = 0
        if one_dir:
            oncoming = False
        elif ego is None:
            oncoming = None
        else:
            oncoming = ego == 0
        base = {"tagged": 0.85, "tagged_total": 0.7, "assumed": 0.4}.get(ln["source"], 0.4)
        return {"count": ln["total"], "forward": fwd, "backward": bwd, "ego_index": ego, "oncoming_left": oncoming,
                "ego_from_right": None if ego is None else fwd - 1 - ego,
                "probs": est["probs"] if est else None, "probs_from": est["probs_from"] if est else None,
                "source": "map:" + ln["source"] + ("+vision" if ego is not None and fwd > 1 else ""),
                "confidence": round(base * mc, 2), "estimator": est, "vision": vision, "map": ln}

    def _tunnel(self, m, env):
        ep = (env or {}).get("place") or {}
        v_in = ep.get("label") == "tunnel" and (ep.get("confidence") or 0.0) >= self.cfg["tunnel_env_conf"]
        vision = {"in_tunnel": v_in, "confidence": ep.get("confidence") if v_in else None} if ep else None
        if not m:
            return {"in_tunnel": v_in if ep else None, "exit_in_m": None, "entry_in_m": None, "length_m": None,
                    "source": "vision" if ep else "none", "vision": vision, "map": None}
        t = (m.get("ahead") or {}).get("tunnel") or {}
        m_in = bool(t.get("in_tunnel"))
        return {"in_tunnel": m_in or v_in, "exit_in_m": t.get("exit_m") if m_in else None,
                "entry_in_m": t.get("entry_m"), "length_m": t.get("length_m"),
                "source": "map+vision" if m_in and v_in else "map" if m_in or not v_in else "vision",
                "vision": vision, "map": t}

    @staticmethod
    def _lead(latest, now_mono_ns):
        """The lead vehicle as the tracker (lead.py, topic `lead`) has it, with the raw vision value alongside.
        `fused` is slow (about 2 per second): a consumer that draws the lead reads the `lead` topic itself."""
        ld = latest.get("lead")
        if not ld or ld.get("ts_mono_ns") is None or now_mono_ns - ld["ts_mono_ns"] > 1.0e9:
            return None
        keys = ("present", "mode", "track_id", "dist_m", "dist_std_m", "lateral_m", "v_rel_mps", "v_lead_mps",
                "a_lead_mps2", "time_gap_s", "ttc_s", "standing", "departed", "source", "vision")
        return {k: ld.get(k) for k in keys}

    # ---- output ------------------------------------------------------------------------------
    def update(self, latest, now_mono_ns, now_real_ns=None):
        m, info = self._map(latest, now_mono_ns)
        vis = latest.get("speed_limit")
        env = latest.get("env")
        if m:
            self._track_junctions(m)
        built_up = self._built_up(m, vis)
        self.seq += 1
        out = {"type": "fused", "seq": self.seq, "ts_mono_ns": now_mono_ns, "ts_real_ns": now_real_ns,
               "map": info,
               "speed_limit": self._speed_limit(m, vis, built_up),
               "built_up": built_up,
               "place": self._place(m, env),
               "lanes": self._lanes(m, latest.get("road"), latest.get("lanes"), now_mono_ns, latest.get("egolane")),
               "tunnel": self._tunnel(m, env),
               "lead": self._lead(latest, now_mono_ns)}
        if m:
            r, a = m["road"], m.get("ahead") or {}
            out["road"] = {k: r.get(k) for k in ("class", "name", "ref", "oneway", "bridge", "lit", "surface",
                                                 "roundabout", "link", "motorroad", "expressway", "dual_carriageway", "toll",
                                                 "turn_lanes")}
            out["ahead"] = {k: a.get(k) for k in ("horizon_m", "path_confidence", "features", "junctions", "curves",
                                                  "speed_limit_changes", "built_up_change", "road_change", "turn_lanes")}
            pos = m.get("position") or {}
            out["position"] = {k: pos.get(k) for k in ("lat", "lon", "heading_deg", "speed_mps", "hacc_m",
                                                       "confidence", "source")}
        else:
            out["road"] = out["ahead"] = out["position"] = None
        return out

    def summary(self, msg):
        """Short dict for /v1/health."""
        sl, pl, ln, tn = msg["speed_limit"], msg["place"], msg["lanes"], msg["tunnel"]
        return {"map_available": msg["map"]["available"], "position_state": msg["map"]["state"],
                "reason": msg["map"]["reason"],
                "speed_limit": {"limit_kmh": sl["limit_kmh"], "source": sl["source"]},
                "place": {"label": pl["label"], "source": pl["source"]},
                "lanes": {"count": ln["count"], "ego_index": ln["ego_index"], "oncoming_left": ln["oncoming_left"]},
                "tunnel": {"in_tunnel": tn["in_tunnel"], "exit_in_m": tn["exit_in_m"]},
                "road": (msg["road"] or {}).get("ref") or (msg["road"] or {}).get("name") if msg["road"] else None}
