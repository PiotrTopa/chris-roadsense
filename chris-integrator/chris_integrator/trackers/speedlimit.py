"""Current speed limit from the documented `signs` contract (+ `road` v_ego/pose, `env` place).

Pure state machine, no I/O: feed it messages with on_message(msg) (or on_signs/on_road/on_env)
and read snapshot(). The rule set is data (rules/<country>.json); Polish rules are the default.
Only the documented contract is used: detections[] track_id/cls/score/box(/hits), tracks_ended[]
track_id(/cls), road v_ego/pose.yaw_rate, env place label. See docs/lanes/L6-speed-limit.md.
"""
import collections
import json
import math
import os
import re

from ..contract import LocalEgo

RULES_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "rules")
LIVE_TRACK_TIMEOUT_S = 2.0
NS = 1e9

DEFAULT_CFG = {
    "min_hits": 5,              # confirmed track: at least this many detections seen by us
    "min_vote": 0.6,            # share of the score mass on the winning class
    "min_mean_score": 0.45,
    "early_commit_hits": 12,    # commit while still live once the track is this solid
    "junction_min_hits": 3,
    "sign_diameter_m": 0.8,     # B-33 medium size; used for the lateral offset estimate
    "lateral_min_m": -9.0,      # left of the camera (dual carriageway / one-way duplicates)
    "lateral_max_m": 8.0,       # right of the camera; beyond: separate roadway (exit lane)
    "min_aspect": 0.55,         # w/h below this: sign turned towards a side road
    "max_dt_s": 0.5,            # odometry: a longer gap between road messages counts as this much
    "env_min_stable_s": 20.0,   # the env place vote must hold this long
    "env_min_mass": 0.25,       # smoothed score mass of city_street+residential (or rural+forest)
    "env_mass_ratio": 4.0,      # ... and this many times the opposite mass ("highway" etc. abstain)
    "env_area_conf": 0.45,
    "area_sign_hold_m": 3000,   # an area sign beats the env prior for this distance
    "conflict_window_m": 30,    # two different limits within this distance: keep the stronger
    "focal_px": 690.0, "cx": 960.0, "width": 1920,
    "default_area": "unknown",  # unknown | built_up | outside
}

ENV_BUILT_UP = {"city_street", "residential"}
ENV_OUTSIDE = {"rural_road", "forest_road"}


def load_rules(country="pl"):
    with open(os.path.join(RULES_DIR, f"{country.lower()}.json")) as f:
        rules = json.load(f)
    for s in rules["signs"]:
        s["_re"] = re.compile(s["match"])
    return rules


def _norm_cls(cls):
    return re.sub(r"--g\d+$", "", cls or "")


class _Track:
    __slots__ = ("tid", "votes", "n", "score_sum", "hits", "first_ns", "last_ns", "boxes", "committed", "ended",
                 "suppressed")

    def __init__(self, tid, ts):
        self.tid, self.votes, self.n, self.score_sum = tid, collections.Counter(), 0, 0.0
        self.hits, self.first_ns, self.last_ns, self.boxes = 0, ts, ts, []
        self.committed = self.ended = False
        self.suppressed = None            # reason, when the provider or the api's signctx flagged the track

    def add(self, d, ts):
        self.suppressed = (d.get("reason") or "suppressed") if d.get("suppressed") else None
        s = float(d.get("score") or 0.0)
        self.votes[_norm_cls(d.get("cls"))] += max(s, 1e-3)
        self.n += 1
        self.score_sum += s
        self.hits = max(self.hits + 1, int(d.get("hits") or 0))
        self.last_ns = ts
        b = d.get("box")
        if b and len(b) == 4:
            self.boxes.append(tuple(float(v) for v in b))
            if len(self.boxes) > 60:
                del self.boxes[0]

    def cls(self):
        if not self.votes:
            return None, 0.0
        c, w = self.votes.most_common(1)[0]
        return c, w / sum(self.votes.values())

    def mean_score(self):
        return self.score_sum / self.n if self.n else 0.0


class SpeedLimit:
    def __init__(self, rules=None, cfg=None):
        self.rules = rules or load_rules("pl")
        self.cfg = dict(DEFAULT_CFG, **(cfg or {}))
        ex = self.rules["expiry"]
        self.ex = ex
        self.tracks = {}
        self.odo = 0.0                    # metres, the integrator's ego speed integrated (docs/DESIGN.md)
        self.v_ego = None                 # m/s; None while the ego speed is unknown (the odometer stands then)
        self.ego_source = None            # "vehicle" | "vision": what the speed of the odometer is made from
        self._local = LocalEgo()          # the integrator of an engine that is not handed an ego state
        self.now_ns = None
        self.now_real_ns = None
        self.seq = None
        self._last_road_ns = None
        self.heading = collections.deque()  # (odo, yaw_deg) for the turn proxy
        self.yaw_deg = 0.0
        # area / road class / zone beliefs
        self.area = {"type": self.cfg["default_area"], "source": "config", "confidence": 0.3
                     if self.cfg["default_area"] != "unknown" else 0.0, "odo": None, "sign": None}
        self.road_class = {"type": "ordinary", "source": "default", "sign": None}
        self.zone = {"type": "none", "limit_kmh": None, "sign": None}
        self.env_place = None
        self._env_run = None              # (vote, since_ns)
        self.limit = None                 # active sign limit dict or None
        self.advisory = None
        self.junctions = []               # odo positions of expected/passed junctions (from signs)
        self.ignored = collections.deque(maxlen=20)
        self.changes = collections.deque(maxlen=30)
        self.version = 0
        self._last_key = None
        self._since = None
        self.track_hooks = []             # callables(track, live): other engines judge the classes this one ignores

    # ---- inputs ----------------------------------------------------------
    def on_message(self, msg, ego=None):
        """`ego`: the integrator's `ego_state` for a `road` frame (the api passes it; an engine fed `road` alone,
        or `ego` messages too, builds its own)."""
        t = msg.get("type")
        if t == "ego":
            self._local.feed(msg)
            return False
        if t == "road":
            self.on_road(msg, ego)
        elif t == "signs":
            self.on_signs(msg)
        elif t == "env":
            self.on_env(msg)
        else:
            return False
        return self._update_output()

    def _clock(self, msg):
        ts = msg.get("ts_mono_ns")
        if ts is not None and (self.now_ns is None or ts >= self.now_ns):
            self.now_ns = ts
            self.now_real_ns = msg.get("ts_real_ns", self.now_real_ns)
            self.seq = msg.get("seq", self.seq)

    def on_road(self, msg, ego=None):
        """Odometer and heading of the frame, from the integrator's ego state (vehicle speed and yaw rate when the
        host feeds them, the road model's otherwise; until lane L37 always the road model's). While the speed is
        unknown the odometer does not advance: distances "since the sign" then run late, never ahead."""
        self._clock(msg)
        ts = msg.get("ts_mono_ns")
        if ts is None:
            return
        es = ego if ego is not None else self._local.at(msg)
        sp, yr = (es or {}).get("speed") or {}, (es or {}).get("yaw_rate") or {}
        v = max(0.0, abs(sp["value"])) if sp.get("valid") else None
        self.ego_source = None if v is None else \
            "vehicle" if any(x.startswith("vehicle.") for x in sp.get("sources") or ()) else "vision"
        yaw_rate = -yr["value"] if yr.get("valid") else 0.0          # positive right, as the heading always was
        if self._last_road_ns is not None and ts > self._last_road_ns and v is not None:
            dt = min((ts - self._last_road_ns) / NS, self.cfg["max_dt_s"])
            self.odo += 0.5 * (v + (self.v_ego if self.v_ego is not None else v)) * dt
            self.yaw_deg += math.degrees(yaw_rate) * dt
            self.heading.append((self.odo, self.yaw_deg))
            while self.heading and self.odo - self.heading[0][0] > self.ex["turn_window_m"]:
                self.heading.popleft()
        self._last_road_ns = ts
        self.v_ego = v
        self._check_expiry()

    def on_env(self, msg):
        self._clock(msg)
        p = msg.get("place") or {}
        self.env_place = {"label": p.get("label"), "confidence": p.get("confidence"), "stable_s": p.get("stable_s"),
                          "scores": p.get("scores")}
        self._env_prior()

    def on_signs(self, msg):
        self._clock(msg)
        ts = msg.get("ts_mono_ns") or self.now_ns or 0
        for d in msg.get("detections") or []:
            tid = d.get("track_id")
            if tid is None:
                continue
            t = self.tracks.get(tid)
            if t is None:
                t = self.tracks[tid] = _Track(tid, ts)
            t.add(d, ts)
        for t in list(self.tracks.values()):      # after all boxes of this frame are in (plates)
            if t.last_ns == ts and t.hits >= self.cfg["early_commit_hits"]:
                self._decide(t, live=True)
        for e in msg.get("tracks_ended") or []:
            t = self.tracks.pop(e.get("track_id"), None)
            if t is not None:
                if e.get("suppressed"):
                    t.suppressed = e.get("reason") or "suppressed"
                self._decide(t, live=False)
        for tid in [k for k, t in self.tracks.items() if (ts - t.last_ns) / NS > LIVE_TRACK_TIMEOUT_S]:
            self._decide(self.tracks.pop(tid), live=False)
        self._check_expiry()

    def _decide(self, t, live):
        """Judge a track: this engine's own classes, then every hook (rules.py) with the same track object."""
        if not t.committed:
            self._consider(t, live)
        for cb in self.track_hooks:
            cb(t, live)

    # ---- track evaluation -------------------------------------------------
    def _rule(self, cls):
        for r in self.rules["signs"]:
            m = r["_re"].match(cls or "")
            if m:
                val = int(m.group(1)) if m.groups() and m.group(1) and m.group(1).isdigit() else None
                return r, val
        return None, None

    def _geometry(self, t):
        """Median lateral offset (m, + right) and aspect ratio over the track's boxes."""
        lat, asp = [], []
        for x0, y0, x1, y1 in t.boxes:
            w, h = x1 - x0, y1 - y0
            if h < 12 or w <= 0:
                continue
            lat.append((0.5 * (x0 + x1) - self.cfg["cx"]) * self.cfg["sign_diameter_m"] / h)
            asp.append(w / h)
        if not lat:
            return None, None
        lat.sort(), asp.sort()
        return lat[len(lat) // 2], asp[len(asp) // 2]

    def _distance_ahead(self, t):
        if not t.boxes:
            return 0.0
        x0, y0, x1, y1 = t.boxes[-1]
        h = max(y1 - y0, 1.0)
        return min(self.cfg["focal_px"] * self.cfg["sign_diameter_m"] / h, 80.0)

    def quality(self, t):
        cls, vote = t.cls()
        q = min(1.0, t.hits / 15.0) * 0.4 + vote * 0.3 + min(1.0, t.mean_score()) * 0.3
        return cls, vote, round(q, 3)

    def _consider(self, t, live):
        if t.committed:
            return
        cls, vote, q = self.quality(t)
        rule, val = self._rule(cls)
        if rule is None:
            return
        act = rule["action"]
        pos = self.odo + (self._distance_ahead(t) if live else 0.0)
        info = {"track_id": t.tid, "cls": cls, "sign": rule["sign"], "hits": t.hits, "vote": round(vote, 2),
                "score": round(t.mean_score(), 3)}
        if t.suppressed:          # flagged by the provider or signctx (lamp, traffic light, low score, ...)
            if live:
                return            # the flag may still clear (e.g. "score" as the sign gets closer)
            t.committed = True
            self.ignored.append(dict(info, reason=f"suppressed ({t.suppressed})", at_odo_m=round(self.odo, 1)))
            return
        if act in ("junction", "junction_warning"):
            if t.hits < self.cfg["junction_min_hits"]:
                return
            if live:
                return            # junction signs are placed when the track ends (sign passed)
            t.committed = True
            off = rule.get("offset_m") if act == "junction" else \
                self.rules["junction_warning_offset_m"].get(self.area["type"], 100)
            self.junctions.append({"odo": pos + off, "sign": rule["sign"], "cls": cls, "track_id": t.tid})
            return
        if act.startswith("plate"):
            return                # plates are looked up next to the sign they belong to
        reasons = []
        if t.hits < self.cfg["min_hits"]:
            reasons.append(f"hits {t.hits}<{self.cfg['min_hits']}")
        if vote < self.cfg["min_vote"]:
            reasons.append(f"vote {vote:.2f}<{self.cfg['min_vote']}")
        min_score = rule.get("min_score", self.cfg["min_mean_score"])     # per-sign overrides in the rules
        lat_min = rule.get("lateral_min_m", self.cfg["lateral_min_m"])
        if t.mean_score() < min_score:
            reasons.append(f"score {t.mean_score():.2f}<{min_score}")
        lat, asp = self._geometry(t)
        info["lateral_m"] = None if lat is None else round(lat, 1)
        if lat is None:
            reasons.append("boxes too small for a position check")
        if lat is not None and not (lat_min <= lat <= self.cfg["lateral_max_m"]):
            reasons.append(f"lateral {lat:.1f} m (other roadway / exit lane?)")
        if asp is not None and asp < self.cfg["min_aspect"]:
            reasons.append(f"aspect {asp:.2f} (turned to a side road)")
        if reasons:
            if live:
                return            # may still qualify when the track ends
            t.committed = True
            self.ignored.append(dict(info, reason="; ".join(reasons), at_odo_m=round(self.odo, 1)))
            return
        t.committed = True
        plate = self._plate_for(t)
        if plate and plate["action"] == "plate_vehicle":
            self.ignored.append(dict(info, reason=f"vehicle plate {plate['cls']} (not a passenger car)",
                                     at_odo_m=round(self.odo, 1)))
            return
        self._apply(act, val, info, q, pos, plate)

    def _plate_for(self, t):
        """A complementary plate track directly under the sign's last box, live at the same time."""
        if not t.boxes:
            return None
        sx0, sy0, sx1, sy1 = t.boxes[-1]
        sh = sy1 - sy0
        for o in self.tracks.values():
            if o is t or not o.boxes or abs(o.last_ns - t.last_ns) / NS > 1.0:
                continue
            ocls, _ = o.cls()
            r, _ = self._rule(ocls)
            if not r or not r["action"].startswith("plate"):
                continue
            px0, py0, px1, py1 = o.boxes[-1]
            if min(px1, sx1) - max(px0, sx0) > 0.3 * (sx1 - sx0) and sy1 - 0.2 * sh <= py0 <= sy1 + 1.5 * sh:
                return {"cls": ocls, "action": r["action"], "track_id": o.tid}
        return None

    # ---- state changes ----------------------------------------------------
    def _set_area(self, typ, sign, conf, source="sign"):
        self.area = {"type": typ, "source": source, "confidence": conf, "odo": self.odo, "sign": sign}

    def _end_limit(self, why):
        if self.limit:
            self.limit = None
            self._note(f"limit ended: {why}")

    def _apply(self, act, val, info, q, pos, plate):
        if act == "limit":
            cur = self.limit
            if cur and cur["value_kmh"] != val and abs(pos - cur["odo"]) < self.cfg["conflict_window_m"] \
                    and cur["quality"] > q:
                self.ignored.append(dict(info, reason=f"conflicts with {cur['value_kmh']} on the same spot "
                                                      f"(weaker)", at_odo_m=round(self.odo, 1)))
                return
            zone_suspect = (val <= self.ex["zone_suspect_max_kmh"] and self.area["type"] == "built_up")
            if cur and cur["value_kmh"] == val:
                cur.update(odo=pos, refreshed_track=info["track_id"], quality=max(cur["quality"], q),
                           zone_suspect=cur["zone_suspect"] or zone_suspect)
                cur["confirmations"] += 1
                return
            self.limit = {"value_kmh": val, "odo": pos, "quality": q, "info": info, "plate": plate,
                          "zone_suspect": zone_suspect, "confirmations": 1, "set_ns": self.now_ns}
            self._note(f"B-33 {val}")
        elif act == "end_limit":
            self._end_limit(f"{info['sign']} ({info['cls']})")
        elif act == "end_zone":
            if self.zone["type"] == "speed_zone" or (self.limit and self.limit["zone_suspect"]):
                self._end_limit("B-44 end of speed zone")
            self.zone = {"type": "none", "limit_kmh": None, "sign": None}
        elif act == "residential_start":
            self._end_limit("D-40")
            self.zone = {"type": "residential", "limit_kmh": self.rules["defaults_kmh"]["residential_zone"],
                         "sign": info}
            self._set_area("built_up", info, 0.6)
        elif act == "residential_end":
            self.zone = {"type": "none", "limit_kmh": None, "sign": None}
            self._set_area("built_up", info, 0.6)
        elif act in ("built_up_end", "built_up_start"):
            if self.rules.get("area_signs_end_b33"):
                self._end_limit(info["sign"])
            self.zone = {"type": "none", "limit_kmh": None, "sign": None}
            self._set_area("outside" if act == "built_up_end" else "built_up", info, 0.8)
        elif act in ("motorway_start", "expressway_start"):
            self._end_limit(info["sign"])
            self.road_class = {"type": act.split("_")[0], "source": "sign", "sign": info}
            self._set_area("outside", info, 0.8)
        elif act in ("motorway_end", "expressway_end"):
            self._end_limit(info["sign"])
            self.road_class = {"type": "ordinary", "source": "sign", "sign": info}
        elif act == "advisory":
            self.advisory = {"value_kmh": val, "odo": pos, "info": info}
        elif act == "ignore_vehicle":
            self.ignored.append(dict(info, reason="applies to trucks only", at_odo_m=round(self.odo, 1)))

    def _env_vote(self, e):
        """built_up / outside / None from envd's smoothed place scores (label fallback)."""
        sc = e.get("scores")
        if sc:
            b = sum(sc.get(k, 0.0) for k in ENV_BUILT_UP)
            o = sum(sc.get(k, 0.0) for k in ENV_OUTSIDE)
            r = self.cfg["env_mass_ratio"]
            if b >= self.cfg["env_min_mass"] and b >= r * o:
                return "built_up"
            if o >= self.cfg["env_min_mass"] and o >= r * b:
                return "outside"
            return None
        if (e.get("confidence") or 0) < self.cfg["env_min_mass"]:
            return None
        lab = e.get("label")
        return "built_up" if lab in ENV_BUILT_UP else "outside" if lab in ENV_OUTSIDE else None

    def _env_prior(self):
        """envd place as a weak prior for the area (D-42 is not in the classifier's classes).

        The vote must hold for env_min_stable_s; it never overrides an area sign seen within
        area_sign_hold_m, nor a motorway/expressway sign."""
        typ = self._env_vote(self.env_place or {})
        if typ != (self._env_run or (None,))[0]:
            self._env_run = (typ, self.now_ns)
        if typ is None or self.now_ns is None or (self.now_ns - self._env_run[1]) / NS < self.cfg["env_min_stable_s"]:
            return
        if self.road_class["type"] != "ordinary":
            return
        a = self.area
        if a["source"] == "sign" and a["odo"] is not None and self.odo - a["odo"] < self.cfg["area_sign_hold_m"]:
            return
        if a["type"] != typ or a["source"] != "env":
            self._set_area(typ, {"env_place": (self.env_place or {}).get("label")}, self.cfg["env_area_conf"],
                           source="env")

    def _check_expiry(self):
        lim = self.limit
        if lim:
            rc = self.road_class["type"]
            key = rc if rc != "ordinary" else self.area["type"]
            cap = self.ex["zone_suspect_cap_m"] if lim["zone_suspect"] else self.ex["distance_cap_m"].get(key, 1200)
            if self.odo - lim["odo"] > cap:
                self._end_limit(f"distance cap {cap} m without a repeat (proxy for a missed intersection)")
            elif rc == "ordinary" and not lim["zone_suspect"]:
                margin = lim["odo"] + self.ex["same_pole_margin_m"]
                for j in self.junctions:
                    if j["odo"] > margin and j["odo"] <= self.odo:
                        self._end_limit(f"intersection after {j['sign']} ({j['cls']}, track {j['track_id']})")
                        break
                else:
                    if self.heading and self.odo - lim["odo"] > self.ex["same_pole_margin_m"]:
                        yaws = [y for o, y in self.heading if o >= margin]
                        if yaws and max(yaws) - min(yaws) >= self.ex["turn_deg"]:
                            self._end_limit(f"turn of {max(yaws) - min(yaws):.0f} deg within "
                                            f"{self.ex['turn_window_m']} m (proxy: turned at an intersection)")
        self.junctions = [j for j in self.junctions if j["odo"] > self.odo - 50]
        if self.advisory and self.odo - self.advisory["odo"] > self.ex["advisory_hold_m"]:
            self.advisory = None

    def _note(self, what):
        self.changes.append({"ts_real_ns": self.now_real_ns, "odo_m": round(self.odo, 1), "event": what})

    # ---- output -------------------------------------------------------------
    def default_limit(self):
        d = self.rules["defaults_kmh"]
        rc = self.road_class["type"]
        if rc == "motorway":
            return d["motorway"], "motorway", 0.7
        if rc == "expressway":
            return d["expressway"], "expressway", 0.6
        if self.zone["type"] == "residential":
            return d["residential_zone"], "residential_zone", 0.7
        a = self.area
        if a["type"] == "built_up":
            return d["built_up"], "built_up", a["confidence"]
        if a["type"] == "outside":
            return d["outside"], "outside", a["confidence"]
        return None, "unknown_area", 0.0

    def current(self):
        lim = self.limit
        if lim:
            conf = lim["quality"] * (0.8 if lim["plate"] else 1.0)
            conf = min(1.0, conf + 0.1 * (lim["confirmations"] - 1))
            rc = self.road_class["type"]
            key = rc if rc != "ordinary" else self.area["type"]
            cap = self.ex["zone_suspect_cap_m"] if lim["zone_suspect"] else self.ex["distance_cap_m"].get(key, 1200)
            travelled = self.odo - lim["odo"]
            conf *= max(0.5, 1.0 - 0.5 * max(0.0, travelled) / cap)   # fades towards the cap
            src = dict(lim["info"], kind="sign")
            if lim["plate"]:
                src["plate"] = lim["plate"]["cls"]
            expiry = {"rule": "zone (B-43?) until B-44 / another limit" if lim["zone_suspect"]
                      else "next intersection (par. 32 ust. 2)" if rc == "ordinary" else "until B-34/B-33/D-10/D-8",
                      "distance_cap_m": cap, "remaining_m": round(max(0.0, cap - travelled), 1)}
            return lim["value_kmh"], src, round(conf, 3), expiry
        val, rule, conf = self.default_limit()
        src = {"kind": "default" if val is not None else "unknown", "rule": rule,
               "area_source": self.area["source"]}
        return val, src, round(conf, 3), None

    def pending(self):
        out = []
        for t in self.tracks.values():
            if t.committed:
                continue
            cls, vote, q = self.quality(t)
            rule, val = self._rule(cls)
            if not rule or rule["action"].startswith(("plate", "junction")):
                continue
            lat, _ = self._geometry(t)
            out.append({"track_id": t.tid, "cls": cls, "sign": rule["sign"], "action": rule["action"],
                        "value_kmh": val, "hits": t.hits, "vote": round(vote, 2), "quality": q,
                        "lateral_m": None if lat is None else round(lat, 1)})
        return out

    def _update_output(self):
        val, src, conf, _ = self.current()
        key = (val, src.get("kind"), src.get("track_id"), src.get("rule"), self.area["type"],
               self.zone["type"], self.road_class["type"])
        if key != self._last_key:
            if self._last_key is None or val != self._last_key[0]:
                self._since = {"ts_mono_ns": self.now_ns, "ts_real_ns": self.now_real_ns, "odo": self.odo}
            self._last_key = key
            self.version += 1
            return True
        return False

    def snapshot(self, full=False):
        val, src, conf, expiry = self.current()
        since = self._since or {"ts_mono_ns": self.now_ns, "ts_real_ns": self.now_real_ns, "odo": self.odo}
        age = (self.now_ns - since["ts_mono_ns"]) / NS if self.now_ns and since["ts_mono_ns"] else 0.0
        msg = {
            "type": "speed_limit", "seq": self.seq, "ts_mono_ns": self.now_ns, "ts_real_ns": self.now_real_ns,
            "limit_kmh": val, "source": src, "confidence": conf,
            "since": {"ts_real_ns": since["ts_real_ns"], "age_s": round(age, 1),
                      "distance_m": round(self.odo - since["odo"], 1)},
            "expiry": expiry,
            "area": {"type": self.area["type"], "source": self.area["source"],
                     "confidence": self.area["confidence"]},
            "road_class": self.road_class["type"],
            "zone": {"type": self.zone["type"], "limit_kmh": self.zone["limit_kmh"]},
            "advisory_kmh": self.advisory["value_kmh"] if self.advisory else None,
            "pending": self.pending(),
            "v_ego_kmh": None if self.v_ego is None else round(self.v_ego * 3.6, 1),
            "ego_source": self.ego_source,
            "odometer_m": round(self.odo, 1),
            "rules": self.rules["country"],
        }
        if full:
            msg["recent_changes"] = list(self.changes)[::-1]
            msg["ignored"] = list(self.ignored)[::-1]
        return msg
