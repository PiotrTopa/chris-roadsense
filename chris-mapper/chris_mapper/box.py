"""The map box: pose in, map match and map context out.

    in    `ego_state`   the integrator's ego estimate: position (lat, lon, east / north covariance, how fresh
                        its last absolute fix is), heading with its std, speed, odometer, time of validity
    out   `map`         every `period_s` of input time:
                          match        the matched point, the road's bearing, confidence, ambiguity: geometry of
                                       the map, which the estimator takes as a measurement (cross-track to the
                                       road = 0, heading = the road's bearing), never the pose that came in
                          position     the pose that was used, with its state (`good`: an absolute fix within
                                       the estimator's freshness window; `dead_reckoning`; `unknown`; `no_source`)
                          road, speed_limit, ahead (features, junctions, curves, limit changes, turn lanes,
                          `junction_view`)

The box holds no position filter and reads no sensor: where the car is, is the estimator's statement, taken
with its uncertainty. What the box adds is which road that is, and what the map says about it.

No I/O besides the pack file and no wall clock: all times are the input's `ts_mono_ns`.
"""
import math
import os

from . import API_VERSION, context
from .matcher import MapMatcher
from .rsmap import Pack

NS = 1e9
DEFAULTS = dict(
    pack=None,                  # path of the map pack; None: CHRIS_MAP_PACK or /var/lib/chris/maps/current.rsmap
    period_s=0.45,              # a match (and a `map` message) this often
    horizon_m=1500.0, shape_m=600.0,
    max_hacc_m=150.0,           # a pose less certain than this is not matched
    stand_speed=0.3,            # standing with a match and a fresh fix: the match is kept, not recomputed
    min_hacc_m=3.0,             # the pose's stated uncertainty is not taken below this for the search
    country="pl",
    matcher=None,               # overrides of matcher.DEFAULT_CFG
    cache_tiles=24,
    pack_check_s=5.0,           # the pack file is looked at this often (a new pack is loaded without a restart)
)
DEFAULT_PACK = "/var/lib/chris/maps/current.rsmap"

# the declarations of the plug-in contract (also sent in the hello line of the out-of-process form)
NAME = "mapper"
INPUTS = [{"name": "ego_state", "trigger": True}]
OUTPUTS = [{"name": "map", "kind": "context", "time": "periodic", "public": True,
            "doc": "map match (a measurement for the estimator) and map context"}]


class MapBox:
    def __init__(self, cfg=None, pack=None):
        self.cfg = dict(DEFAULTS)
        self.cfg.update({k: v for k, v in (cfg or {}).items() if k in DEFAULTS})
        self.limits = context.load_limits(self.cfg["country"])
        self.pack_path = None
        self.pack = None
        self.matcher = None
        self._pack_id = self._pack_t = None
        if pack is not None:
            self.set_pack(pack)
        else:
            self.pack_path = self.cfg["pack"] or os.environ.get("CHRIS_MAP_PACK") or DEFAULT_PACK
        self.view_memo = {}             # orientation of the last roundabout view (context.junction_view)
        self.receiver = None            # the estimator's entry for its receiver (`ego_state.sources.gnss`)
        self.seq = 0
        self.counts = {"in": 0, "matches": 0, "errors": 0}
        self.last_error = None
        self.reset()

    # ---- pack --------------------------------------------------------------------------------------
    def set_pack(self, pack):
        self.pack = pack
        self.matcher = MapMatcher(pack, self.cfg.get("matcher")) if pack is not None else None
        self.match = None

    def check_pack(self, t):
        """(Re)load the pack when the file behind the path appeared or changed (a new symlink target too)."""
        if self.pack_path is None or (self._pack_t is not None and 0.0 <= t - self._pack_t < self.cfg["pack_check_s"]):
            return
        self._pack_t = t
        try:
            st = os.stat(self.pack_path)
            ident = (st.st_dev, st.st_ino, st.st_size, st.st_mtime_ns)
        except OSError:
            ident = None
        if ident == self._pack_id:
            return
        self._pack_id = ident
        old = self.pack
        if ident is None:
            self.set_pack(None)
        else:
            try:
                self.set_pack(Pack(self.pack_path, cache_tiles=self.cfg["cache_tiles"]))
            except (OSError, ValueError) as e:
                self.last_error = "cannot load pack %s: %s" % (self.pack_path, e)
                self.counts["errors"] += 1
                return
        if old is not None:
            old.close()

    def reset(self, reason=None):
        self.match = None
        self.t_match = None
        self.odo_match = None
        self.state = "no_source"
        self.had_position = False
        self.frame = None               # the estimator's local frame id: a re-anchored frame is a new position
        self.t_out = None
        if self.matcher is not None:
            self.matcher.reset()

    def close(self):
        if self.pack is not None and self.pack_path is not None:
            self.pack.close()
            self.pack = None

    # ---- the pose as the estimator states it ---------------------------------------------------------
    @staticmethod
    def pose(es):
        """(state, pose or None) from an `ego_state`: pose = dict(lat, lon, heading_deg or None, heading_std,
        speed, hacc: RMS radial error, cross_sigma: its part across the heading, fix_age_s, odometer_m)."""
        p = es.get("position") or {}
        if not p.get("valid"):
            return ("no_source" if p.get("reason") in ("no_measurement", "no_absolute_source") else "unknown"), None
        cov = p.get("cov_en") or [0.0, 0.0, 0.0]
        cee, cen, cnn = cov
        hd = es.get("heading") or {}
        heading = hd.get("value") if hd.get("valid") else None
        sp = es.get("speed") or {}
        speed = sp.get("value") if sp.get("valid") else 0.0
        cross = None
        if heading is not None:
            h = math.radians(heading)                   # course: clockwise from north; across = (cos h, -sin h) in (e, n)
            ce, cn = math.cos(h), -math.sin(h)
            cross = math.sqrt(max(0.0, ce * ce * cee + 2.0 * ce * cn * cen + cn * cn * cnn))
        return ("good" if p.get("mode") == "fused" else "dead_reckoning"), {
            "lat": p["lat"], "lon": p["lon"], "heading_deg": heading, "heading_std": hd.get("std"),
            "speed": speed or 0.0, "hacc": math.sqrt(max(0.0, cee + cnn)), "cross_sigma": cross,
            "fix_age_s": p.get("fix_age_s"), "odometer_m": es.get("odometer_m") or 0.0,
            "frame": (p.get("local") or {}).get("frame"), "sources": p.get("sources") or []}

    # ---- one input -----------------------------------------------------------------------------------
    def on_ego_state(self, es):
        """Feed an `ego_state`; returns a `map` message when one is due, else None."""
        ts = es.get("ts_mono_ns")
        if not isinstance(ts, int):
            return None
        self.counts["in"] += 1
        t = ts / NS
        if self.t_out is not None and 0.0 <= t - self.t_out < self.cfg["period_s"]:
            return None
        self.t_out = t
        self.check_pack(t)
        self.receiver = ((es.get("sources") or {}).get("gnss") or None)
        c = self.cfg
        prev = self.state
        self.state, pose = self.pose(es)
        if pose is not None and pose["hacc"] > c["max_hacc_m"]:
            self.state, pose = "unknown", None
        if pose is not None:
            if self.frame is not None and pose["frame"] != self.frame and self.matcher is not None:
                self.matcher.reset()            # the estimator anchored its position anew: not a continuation
                self.match = None
            self.frame = pose["frame"]
            self.had_position = True
        elif self.had_position and self.state == "no_source":
            self.state = "unknown"
        if pose is None:
            if prev in ("good", "dead_reckoning") and self.matcher is not None:
                self.matcher.reset()
            self.match = None
        elif self.matcher is not None:
            standing = abs(pose["speed"]) < c["stand_speed"] and self.match is not None and self.state == "good"
            if not standing:
                travelled = 0.0 if self.odo_match is None else max(0.0, pose["odometer_m"] - self.odo_match)
                hacc = max(pose["hacc"], c["min_hacc_m"])
                try:
                    self.match = self.matcher.update(pose["lat"], pose["lon"], pose["heading_deg"], pose["speed"], hacc,
                                                     travelled, reverse=pose["speed"] < -0.3,
                                                     cross_sigma=pose["cross_sigma"])
                except Exception as e:           # a damaged tile must not stop the box
                    self.counts["errors"] += 1
                    self.last_error = "%s: %s" % (type(e).__name__, e)
                    self.match = None
                self.odo_match = pose["odometer_m"]
                self.t_match = t
                self.counts["matches"] += 1
        return self._message(ts, es.get("ts_real_ns"), pose)

    def _message(self, ts, real_ns, pose):
        c = self.cfg
        self.seq += 1
        m = self.match if pose is not None else None
        if self.pack is None:
            reason = "no_pack"
        elif self.state == "no_source":
            reason = "no_position"
        elif self.state == "unknown":
            reason = "position_unknown"
        elif m is None:
            reason = "off_road"
        else:
            reason = None
        out = {"type": "map", "seq": self.seq, "ts_mono_ns": ts, "ts_real_ns": real_ns, "api": API_VERSION,
               "state": self.state, "available": reason is None, "reason": reason}
        man = self.pack.manifest if self.pack is not None else None
        out["pack"] = {"region": man.get("region"), "osm_timestamp": man.get("osm_timestamp"),
                       "built": man.get("built"), "format": man.get("format")} if man else None
        # what the pose is made from is the estimator's statement (`ego_state.sources`, `ego_state.position`)
        # what the pose is made from is the estimator's statement; its receiver's tag is passed on, so that a
        # consumer can tell a made-up position (a source tagged "synthetic...") from a real one
        rx = self.receiver or {}
        tag = rx.get("receiver")
        out["input"] = {"pose": "ego_state",
                        "gnss": None if pose is None and not rx else {
                            "source": tag, "fix": rx.get("fix"), "hacc_m": rx.get("hacc_m"),
                            "age_s": None if pose is None else pose["fix_age_s"]},
                        "motion": {"source": "integrator", "tag": None, "age_s": 0.0},
                        "errors": self.counts["errors"], "last_error": self.last_error,
                        "synthetic": isinstance(tag, str) and tag.strip().lower().startswith("synthetic")}
        if pose is not None:
            hacc = pose["hacc"]
            if self.state == "good":
                conf = max(0.0, min(1.0, 1.0 - hacc / 60.0))
            else:
                conf = max(0.0, min(0.9, 1.0 - hacc / c["max_hacc_m"]))
            out["position"] = {
                "lat": pose["lat"], "lon": pose["lon"],
                "heading_deg": None if pose["heading_deg"] is None else round(pose["heading_deg"], 1),
                "heading_acc_deg": None if pose["heading_deg"] is None else pose["heading_std"],
                "speed_mps": round(pose["speed"], 2), "hacc_m": round(hacc, 1), "confidence": round(conf, 2),
                "source": "gnss" if self.state == "good" else "dead_reckoning",
                "fix_age_s": pose["fix_age_s"], "dr_distance_m": None,
                "motion_source": "integrator", "odometer_m": round(pose["odometer_m"], 1)}
        else:
            out["position"] = None
        if reason is None:
            e = m.edge
            out["match"] = m.as_dict()
            out["road"] = context.road_info(e, m.dir)
            out["speed_limit"] = context.speed_limit(e, m.dir, self.limits)
            out["ahead"] = context.look_ahead(self.pack, m, self.limits, c["horizon_m"], c["shape_m"],
                                               memo=self.view_memo)
        else:
            out["match"] = {"matched": False} if pose is not None and self.pack is not None else None
            out["road"] = out["speed_limit"] = out["ahead"] = None
        return out

    def health(self):
        if self.pack is None:
            return {"state": "idle", "reason": self.last_error or "no map pack at %s" % self.pack_path,
                    "pack": None, **self.counts}
        man = self.pack.manifest
        return {"state": "ok", "reason": None, "position": self.state, "matched": self.match is not None,
                "pack": {"region": man.get("region"), "osm_timestamp": man.get("osm_timestamp")}, **self.counts}
