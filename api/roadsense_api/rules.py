"""Rules in effect for the car, from the documented `signs` contract (+ `road`, `env`, `lights`).

Generalisation of the L6 speed-limit engine (lane L11): a `RulesInEffect` owns a `SpeedLimit` (unchanged
behaviour, still served as `speed_limit`), feeds it every message and reads its odometer, track objects,
geometry helpers, area / zone / road-class beliefs and junction proxies. Through the engine's track hooks it
judges every other sign class of the rule table `rules/<country>.json` -> `effects`, with the same
confirmation, lateral and plate code, and keeps three lists:

  in_effect  lasting rules (priority road, no stopping / parking, no overtaking, dimension limits, one-way,
             living street, built-up area, motorway / expressway, the speed limit) and momentary signs that
             bind at the next intersection (stop, yield, turn bans, mandatory directions);
  recent     momentary signs just passed (no entry, pedestrian crossing, roundabout, ...), kept 300 m / 120 s;
  ahead      warning signs with the expected distance to the danger (par. 3 ust. 2: 0-100 m where the limit is
             <= 60 km/h, 150-300 m elsewhere), counted down with the odometer.

Junctions ("until the next intersection", par. 32 ust. 2) come from the L6 proxies (junction signs, a >= 50 deg
turn, distance caps) plus the ego traffic light of the `lights` topic: a light seen within 45 m that then
disappears while the car moves on marks the junction at its last position (also fed to the speed limit's
junction list unless `lights_junction` is off). Pure module, no I/O; see docs/lanes/L11-signs-in-effect.md.
"""
import collections
import re

from .speedlimit import SpeedLimit, load_rules, NS

RULES_CFG = {
    "lights_junction": True,          # ego traffic light passed = junction for the rules here
    "lights_junction_speed": True,    # ... and for the speed limit's "until the next intersection" as well
    "early_commit_hits": 12,
}


def _compile(effects):
    out = []
    for e in effects:
        e = dict(e)
        e["_re"] = re.compile(e["match"])
        out.append(e)
    return out


def _warning_name(cls):
    return cls.split("--", 1)[-1].replace("-", " ") + " ahead"


class RulesInEffect:
    def __init__(self, rules=None, cfg=None):
        self.rules = rules or load_rules("pl")
        self.speed = SpeedLimit(self.rules, cfg)
        self.cfg = dict(RULES_CFG, **(cfg or {}))
        self.ecfg = dict(self.rules.get("effects_cfg") or {})
        self.effects = _compile(self.rules.get("effects") or [])
        self.speed.track_hooks.append(self._on_track)
        self.decided = set()              # track ids this engine has judged
        self.active = {}                  # rule id -> active rule (lasting, or momentary until a junction)
        self.recent = collections.deque(maxlen=30)
        self.ahead = []
        self.junctions = []               # [{odo, source, detail}] every junction cue, oldest first
        self._sl_junctions_seen = set()
        self._turn_last_odo = None
        self._light_seen = None           # (odo, dist_m, ts_ns, track_id) of the last close ego light
        self._light_last_junction_odo = None
        self.ignored = collections.deque(maxlen=30)
        self.changes = collections.deque(maxlen=50)
        self.version = 0
        self._last_key = None
        self._since = {}                  # rule id -> (ts_mono_ns, ts_real_ns, odo) when it entered in_effect

    # ---- inputs ------------------------------------------------------------
    @property
    def odo(self):
        return self.speed.odo

    @property
    def now_ns(self):
        return self.speed.now_ns

    def on_message(self, msg):
        t = msg.get("type")
        if t == "lights":
            self.speed._clock(msg)
            self._on_lights(msg)
        elif t in ("road", "signs", "env"):
            self.speed.on_message(msg)
        else:
            return False
        self._sync_junctions()
        if t == "road":
            self._turn_proxy()
        self._light_check()
        self._expire()
        return self._update_output()

    # ---- junction cues -------------------------------------------------------
    def _add_junction(self, odo, source, detail=None, to_speed=False):
        self.junctions.append({"odo": round(odo, 1), "source": source, "detail": detail,
                               "ts_real_ns": self.speed.now_real_ns})
        if to_speed:
            self.speed.junctions.append({"odo": odo, "sign": source, "cls": "lights", "track_id": detail})
        self.junctions = [j for j in self.junctions if j["odo"] > self.odo - 500]

    def _sync_junctions(self):
        for j in self.speed.junctions:
            key = id(j)
            if key not in self._sl_junctions_seen and j.get("cls") != "lights":
                self._sl_junctions_seen.add(key)
                self._add_junction(j["odo"], f"sign {j['sign']}", j.get("track_id"))
        if len(self._sl_junctions_seen) > 500:
            self._sl_junctions_seen = set(id(j) for j in self.speed.junctions)

    def _turn_proxy(self):
        h = self.speed.heading
        if len(h) < 2:
            return
        yaws = [y for _, y in h]
        if max(yaws) - min(yaws) >= self.speed.ex["turn_deg"]:
            if self._turn_last_odo is None or self.odo - self._turn_last_odo > self.ecfg.get("turn_dedupe_m", 60):
                self._turn_last_odo = self.odo
                self._add_junction(self.odo, "turn", round(max(yaws) - min(yaws)))

    def _on_lights(self, msg):
        ego = msg.get("ego")
        if ego and ego.get("dist_m") is not None and ego["dist_m"] <= self.ecfg.get("light_max_dist_m", 45.0):
            self._light_seen = (self.odo, float(ego["dist_m"]), self.now_ns, ego.get("track_id"))

    def _light_check(self):
        s = self._light_seen
        if not s or self.now_ns is None or not self.cfg["lights_junction"]:
            return
        odo0, dist, ts, tid = s
        if (self.now_ns - ts) / NS < self.ecfg.get("light_gone_s", 1.5) or self.odo - odo0 < self.ecfg.get("light_min_move_m", 5.0):
            return
        self._light_seen = None
        pos = odo0 + dist
        if self._light_last_junction_odo is not None and abs(pos - self._light_last_junction_odo) < self.ecfg.get("light_dedupe_m", 40.0):
            return
        self._light_last_junction_odo = pos
        self._add_junction(pos, "traffic light", tid, to_speed=self.cfg["lights_junction_speed"])

    # ---- track evaluation ------------------------------------------------------
    def _effect(self, cls):
        for e in self.effects:
            if e["_re"].match(cls or ""):
                return e
        return None

    def _on_track(self, t, live):
        if t.tid in self.decided:
            return
        cls, vote, q = self.speed.quality(t)
        eff = self._effect(cls)
        if eff is None:
            return
        cat = eff["category"]
        sp = self.speed
        if live and (cat == "momentary" and not eff.get("until_junction") or t.hits < self.cfg["early_commit_hits"]):
            return                    # "passed" signs commit when the track ends (the sign is passed then)
        info = {"track_id": t.tid, "cls": cls, "sign": eff["sign"], "hits": t.hits, "vote": round(vote, 2),
                "score": round(t.mean_score(), 3)}
        if t.suppressed:
            if live:
                return
            self.decided.add(t.tid)
            self.ignored.append(dict(info, reason=f"suppressed ({t.suppressed})", at_odo_m=round(self.odo, 1)))
            return
        kind = "lasting" if cat == "lasting" else "warning" if cat == "warning" else "momentary"   # end signs: 3 hits
        min_hits = self.ecfg.get(f"{kind}_min_hits", 3)
        min_hits = eff.get("min_hits", min_hits)
        reasons = []
        if t.hits < min_hits:
            reasons.append(f"hits {t.hits}<{min_hits}")
        if t.n < 2:                       # the provider's hit count may be 3 on a track we saw in one frame (night, far)
            reasons.append(f"seen in {t.n} frame")
        if vote < sp.cfg["min_vote"]:
            reasons.append(f"vote {vote:.2f}<{sp.cfg['min_vote']}")
        min_score = eff.get("min_score", sp.cfg["min_mean_score"])
        if t.mean_score() < min_score:
            reasons.append(f"score {t.mean_score():.2f}<{min_score}")
        lat, asp = sp._geometry(t)
        info["lateral_m"] = None if lat is None else round(lat, 1)
        if lat is None:
            reasons.append("boxes too small for a position check")
        else:
            if cat == "momentary":
                lo, hi = self.ecfg.get("momentary_lateral_m", [-12.0, 15.0])
            elif cat == "warning":
                lo, hi = self.ecfg.get("warning_lateral_m", [-9.0, 10.0])
            else:
                lo, hi = sp.cfg["lateral_min_m"], sp.cfg["lateral_max_m"]
            lo, hi = eff.get("lateral_min_m", lo), eff.get("lateral_max_m", hi)
            if not (lo <= lat <= hi):
                reasons.append(f"lateral {lat:.1f} m (other roadway / exit lane?)")
        if asp is not None and asp < sp.cfg["min_aspect"]:
            reasons.append(f"aspect {asp:.2f} (turned to a side road)")
        if reasons:
            if live:
                return
            self.decided.add(t.tid)
            self.ignored.append(dict(info, reason="; ".join(reasons), at_odo_m=round(self.odo, 1)))
            return
        self.decided.add(t.tid)
        plate = sp._plate_for(t)
        if plate and plate["action"] == "plate_vehicle" and cat in ("lasting", "momentary") and eff.get("applies_to_car", True):
            self.ignored.append(dict(info, reason=f"vehicle plate {plate['cls']} (not a passenger car)",
                                     at_odo_m=round(self.odo, 1)))
            return
        pos = self.odo + (sp._distance_ahead(t) if live else 0.0)
        self._apply(eff, info, q, pos, plate, lat)

    # ---- state changes -------------------------------------------------------
    def _note(self, what):
        self.changes.append({"ts_real_ns": self.speed.now_real_ns, "odo_m": round(self.odo, 1), "event": what})

    def _end(self, rid, why):
        r = self.active.pop(rid, None)
        if r is None:
            return
        self._since.pop(rid, None)
        self._note(f"{r['sign']} {r['name']} ended: {why}")
        if r.get("until_junction"):           # a momentary sign that bound at the junction: now passed
            self.recent.append(dict(r, passed_odo=r["odo"], passed_ns=r["set_ns"], ended=why))

    def _apply(self, eff, info, q, pos, plate, lat):
        cat = eff["category"]
        rid = eff["id"]
        side = None
        if eff.get("side_specific") and lat is not None:
            side = "left" if lat < -1.5 else "right"
        uncertain = []
        if plate:
            uncertain.append(f"plate {plate['cls']}")
        if eff.get("uncertain"):
            uncertain.append(eff["uncertain"])
        if eff.get("value_unreadable"):
            uncertain.append("value not readable")
        if lat is not None and cat == "momentary" and lat > self.ecfg.get("side_road_lateral_m", 8.0):
            uncertain.append("side road?")
        entry = {"id": rid, "sign": eff["sign"], "name": eff.get("name") or _warning_name(info["cls"]),
                 "category": cat, "odo": pos, "set_ns": self.now_ns, "set_real_ns": self.speed.now_real_ns,
                 "quality": q, "info": info, "plate": plate["cls"] if plate else None, "side": side,
                 "uncertain": uncertain, "applies_to_car": eff.get("applies_to_car", True), "confirmations": 1,
                 "eff": eff}
        for other in eff.get("ends_also") or []:
            if other in self.active and other != rid:
                self._end(other, f"{eff['sign']} ({info['cls']}, track {info['track_id']})")
        if cat == "end":
            for other in eff.get("ends_ids") or []:
                self._end(other, f"{eff['sign']} ({info['cls']}, track {info['track_id']})")
            self.recent.append(dict(entry, passed_odo=pos, passed_ns=self.now_ns))
            self._note(f"{eff['sign']} {entry['name']} (track {info['track_id']})")
        elif cat == "lasting" or (cat == "momentary" and eff.get("until_junction")):
            cur = self.active.get(rid)
            if cur and cur["info"]["track_id"] != info["track_id"]:
                cur.update(odo=pos, quality=max(cur["quality"], q), confirmations=cur["confirmations"] + 1,
                           refreshed_track=info["track_id"], side=side or cur["side"])
                if uncertain and not cur["uncertain"]:
                    cur["uncertain"] = uncertain
                self._note(f"{eff['sign']} {entry['name']} repeated (track {info['track_id']})")
                return
            entry["until_junction"] = bool(eff.get("until_junction"))
            self.active[rid] = entry
            self._since[rid] = (self.now_ns, self.speed.now_real_ns, self.odo)
            self._note(f"{eff['sign']} {entry['name']} (track {info['track_id']})")
        elif cat == "momentary":
            self.recent.append(dict(entry, passed_odo=pos, passed_ns=self.now_ns))
            self._note(f"{eff['sign']} {entry['name']} passed (track {info['track_id']})")
        elif cat == "warning":
            for w in self.ahead:
                if w["id"] == rid and abs(w["odo"] - pos) < 60:
                    w.update(odo=pos, quality=max(w["quality"], q))
                    return
            entry["junction_warning"] = bool(eff.get("junction_warning"))
            self.ahead.append(entry)
            self._note(f"{eff['sign']} {entry['name']} (track {info['track_id']})")

    def _area_key(self):
        rc = self.speed.road_class["type"]
        return rc if rc != "ordinary" else self.speed.area["type"]

    def _cap_m(self, r):
        cap = (r["eff"].get("ends") or {}).get("cap_m", r["eff"].get("cap_m"))
        if isinstance(cap, dict):
            return cap.get(self._area_key(), cap.get("unknown", 1200))
        return cap

    def _junction_after(self, odo, margin_m=None):
        margin = odo + (self.speed.ex["same_pole_margin_m"] if margin_m is None else margin_m)
        for j in self.junctions:
            if j["odo"] > margin and j["odo"] <= self.odo:
                return j
        return None

    def _expire(self):
        for rid, r in list(self.active.items()):
            cap = self._cap_m(r)
            if cap is not None and self.odo - r["odo"] > cap:
                self._end(rid, f"distance cap {cap} m without a repeat")
                continue
            if (r["eff"].get("ends") or {}).get("junction") or r.get("until_junction"):
                # a sign that binds at the next intersection may itself announce it (A-7/B-20: sign + 15 m)
                j = self._junction_after(r["odo"], 0.0 if r.get("until_junction") else None)
                if j:
                    self._end(rid, f"intersection ({j['source']}" + (f", track {j['detail']}" if j["source"].startswith("sign") else "") + ")")
        hold_m, hold_s = self.ecfg.get("recent_hold_m", 300), self.ecfg.get("recent_hold_s", 120)
        while self.recent and (self.odo - self.recent[0]["passed_odo"] > hold_m
                               or (self.now_ns or 0) - (self.recent[0]["passed_ns"] or 0) > hold_s * NS):
            self.recent.popleft()
        keep = []
        key = self._area_key()
        rng = self.ecfg.get("warning_range_m", {}).get(key, [0, 300])
        for w in self.ahead:
            travelled = self.odo - w["odo"]
            if travelled > rng[1] + self.ecfg.get("warning_grace_m", 100):
                self._note(f"{w['sign']} {w['name']} dropped (past {rng[1]} m)")
                continue
            if w["junction_warning"] and self._junction_after(w["odo"]):
                self._note(f"{w['sign']} {w['name']} reached (intersection)")
                continue
            keep.append(w)
        self.ahead = keep

    # ---- output -------------------------------------------------------------
    def _since_of(self, rid, fallback_ns, fallback_odo):
        s = self._since.get(rid)
        ts, real, odo = s if s else (fallback_ns, None, fallback_odo)
        age = (self.now_ns - ts) / NS if self.now_ns and ts else 0.0
        return {"ts_real_ns": real, "age_s": round(age, 1), "distance_m": round(self.odo - odo, 1) if odo is not None else None}

    def _entry(self, r):
        cap = self._cap_m(r)
        travelled = self.odo - r["odo"]
        conf = r["quality"] * (0.8 if r["plate"] else 1.0)
        conf = min(1.0, conf + 0.1 * (r["confirmations"] - 1))
        if cap:
            conf *= max(0.5, 1.0 - 0.5 * max(0.0, travelled) / cap)
        ends = r["eff"].get("ends_text") or ("next intersection (par. 32 ust. 2)" if r.get("until_junction") else None)
        return {"id": r["id"], "sign": r["sign"], "name": r["name"], "category": r["category"],
                "source": dict(r["info"], kind="sign", plate=r["plate"]),
                "since": self._since_of(r["id"], r["set_ns"], r["odo"]),
                "confidence": round(conf, 3), "ends": ends,
                "remaining_m": None if cap is None else round(max(0.0, cap - travelled), 1),
                "uncertain": r["uncertain"], "applies_to_car": r["applies_to_car"], "side": r["side"],
                "value": None, "unit": None}

    def in_effect(self):
        sp = self.speed
        out = []
        val, src, conf, expiry = sp.current()
        if val is not None:
            since = sp._since or {"ts_mono_ns": sp.now_ns, "ts_real_ns": sp.now_real_ns, "odo": sp.odo}
            age = (sp.now_ns - since["ts_mono_ns"]) / NS if sp.now_ns and since["ts_mono_ns"] else 0.0
            out.append({"id": "speed_limit", "sign": src.get("sign") or "B-33" if src["kind"] == "sign" else "default",
                        "name": "speed limit" if src["kind"] == "sign" else f"speed limit ({src.get('rule')})",
                        "category": "lasting", "value": val, "unit": "km/h", "source": src,
                        "since": {"ts_real_ns": since["ts_real_ns"], "age_s": round(age, 1),
                                  "distance_m": round(sp.odo - since["odo"], 1)},
                        "confidence": conf, "ends": expiry["rule"] if expiry else None,
                        "remaining_m": expiry["remaining_m"] if expiry else None,
                        "uncertain": ["plate " + src["plate"]] if src.get("plate") else [], "applies_to_car": True,
                        "side": None})
        a = sp.area
        if a["type"] == "built_up":
            out.append({"id": "built_up_area", "sign": "D-42", "name": "built-up area", "category": "lasting", "value": None,
                        "unit": None, "source": {"kind": a["source"], **({"track_id": a["sign"]["track_id"], "cls": a["sign"]["cls"]}
                                                                        if a["source"] == "sign" and a.get("sign") else {})},
                        "since": self._since_of("built_up_area", None, a["odo"]), "confidence": a["confidence"],
                        "ends": "D-43 end of built-up area", "remaining_m": None,
                        "uncertain": ["entry sign D-42 not observable; area from envd"] if a["source"] == "env" else [],
                        "applies_to_car": True, "side": None})
        if sp.zone["type"] == "residential":
            z = sp.zone
            out.append({"id": "living_street", "sign": "D-40", "name": "living street (20 km/h, pedestrians have priority)",
                        "category": "lasting", "value": 20, "unit": "km/h",
                        "source": dict(z["sign"] or {}, kind="sign"), "since": self._since_of("living_street", None, None),
                        "confidence": 0.7, "ends": "D-41 end of living street", "remaining_m": None, "uncertain": [],
                        "applies_to_car": True, "side": None})
        rc = sp.road_class
        if rc["type"] in ("motorway", "expressway"):
            out.append({"id": rc["type"], "sign": "D-9" if rc["type"] == "motorway" else "D-7",
                        "name": rc["type"], "category": "lasting", "value": None, "unit": None,
                        "source": dict(rc["sign"] or {}, kind=rc["source"]), "since": self._since_of(rc["type"], None, None),
                        "confidence": 0.7, "ends": "D-10 end of motorway" if rc["type"] == "motorway" else "D-8 end of expressway",
                        "remaining_m": None, "uncertain": [], "applies_to_car": True, "side": None})
        for r in sorted(self.active.values(), key=lambda r: r["odo"]):
            out.append(self._entry(r))
        return out

    def recent_list(self):
        out = []
        for r in reversed(self.recent):
            age = (self.now_ns - r["passed_ns"]) / NS if self.now_ns and r["passed_ns"] else 0.0
            out.append({"id": r["id"], "sign": r["sign"], "name": r["name"], "category": r["category"],
                        "source": dict(r["info"], kind="sign", plate=r["plate"]),
                        "passed": {"age_s": round(age, 1), "distance_m": round(self.odo - r["passed_odo"], 1)},
                        "confidence": round(r["quality"] * (0.8 if r["plate"] else 1.0), 3),
                        "uncertain": r["uncertain"], "applies_to_car": r["applies_to_car"],
                        "lateral_m": r["info"].get("lateral_m"), "ended": r.get("ended")})
        return out

    def ahead_list(self):
        key = self._area_key()
        exp = self.ecfg.get("warning_expected_m", {}).get(key, 150)
        rng = self.ecfg.get("warning_range_m", {}).get(key, [0, 300])
        out = []
        for w in self.ahead:
            travelled = self.odo - w["odo"]
            out.append({"id": w["id"], "sign": w["sign"], "name": w["name"], "category": "warning",
                        "source": dict(w["info"], kind="sign", plate=w["plate"]),
                        "expected_in_m": round(max(0.0, exp - travelled), 1), "range_m": [round(max(0.0, rng[0] - travelled), 1),
                                                                                    round(max(0.0, rng[1] - travelled), 1)],
                        "since_sign_m": round(travelled, 1), "confidence": round(w["quality"], 3),
                        "uncertain": w["uncertain"] + (["T-1/T-2 plate"] if w["plate"] else [])})
        out.sort(key=lambda w: w["expected_in_m"])
        return out

    def pending(self):
        out = []
        for t in self.speed.tracks.values():
            if t.tid in self.decided:
                continue
            cls, vote, q = self.speed.quality(t)
            eff = self._effect(cls)
            if not eff:
                continue
            lat, _ = self.speed._geometry(t)
            out.append({"track_id": t.tid, "cls": cls, "sign": eff["sign"], "id": eff["id"], "category": eff["category"],
                        "hits": t.hits, "vote": round(vote, 2), "quality": q, "lateral_m": None if lat is None else round(lat, 1)})
        return out

    def _key(self):
        val, src, _, _ = self.speed.current()
        return (val, src.get("kind"), src.get("track_id"), self.speed.area["type"], self.speed.zone["type"],
                self.speed.road_class["type"], tuple(sorted((rid, r["info"]["track_id"], r["confirmations"]) for rid, r in self.active.items())),
                tuple(r["info"]["track_id"] for r in self.recent), tuple(w["info"]["track_id"] for w in self.ahead),
                len(self.junctions) and self.junctions[-1]["odo"])

    def _update_output(self):
        key = self._key()
        if key != self._last_key:
            self._last_key = key
            self.version += 1
            return True
        return False

    def snapshot(self, full=False):
        sp = self.speed
        last_j = self.junctions[-1] if self.junctions else None
        msg = {
            "type": "rules", "seq": sp.seq, "ts_mono_ns": sp.now_ns, "ts_real_ns": sp.now_real_ns,
            "in_effect": self.in_effect(),
            "recent": self.recent_list(),
            "ahead": self.ahead_list(),
            "junction": {"last": dict(last_j, age_m=round(self.odo - last_j["odo"], 1)) if last_j else None,
                         "count": len(self.junctions)},
            "area": {"type": sp.area["type"], "source": sp.area["source"], "confidence": sp.area["confidence"]},
            "road_class": sp.road_class["type"],
            "zone": {"type": sp.zone["type"], "limit_kmh": sp.zone["limit_kmh"]},
            "v_ego_kmh": None if sp.v_ego is None else round(sp.v_ego * 3.6, 1),
            "odometer_m": round(sp.odo, 1),
            "rules": self.rules["country"],
        }
        if full:
            msg["pending"] = self.pending()
            msg["recent_changes"] = list(self.changes)[::-1]
            msg["ignored"] = list(self.ignored)[::-1]
        return msg

    def summary(self):
        """Short strings for /v1/health and the /preview HUD."""
        def short(e):
            s = e["sign"]
            if e.get("value") is not None:
                s += f" {e['value']}"
            elif e["id"] not in ("speed_limit",):
                s += " " + e["name"].split(" (")[0]
            if e.get("side"):
                s += f" ({e['side'][0].upper()})"
            if e.get("uncertain"):
                s += "?"
            return s
        return {"in_effect": [short(e) for e in self.in_effect()],
                "recent": [short(e) for e in self.recent_list()[:4]],
                "ahead": [f"{w['sign']} {w['name']} ~{w['expected_in_m']:.0f} m" for w in self.ahead_list()[:3]]}
