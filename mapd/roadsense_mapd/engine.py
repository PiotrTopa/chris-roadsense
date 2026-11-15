"""mapd core: position input (GNSS + motion) -> filter -> map matching -> the `map` message.

No I/O and no wall clock in here: the daemon (and the tests, and the replay tools) feed messages and call
step(now_ns). All times are CLOCK_MONOTONIC nanoseconds of this host.

Position states (`state` in the message):
    no_source       no usable fix has ever arrived
    good            a fix was used within fix_timeout_s
    dead_reckoning  the fix is older, the position is carried by motion input (vehicle or vision) and the map
    unknown         the fix is lost and dead reckoning ran out (time, distance, uncertainty, or no motion input)
"""
import math

from . import API_VERSION, context, geo
from .matcher import MapMatcher
from .posfilter import PositionFilter

NS = 1e9
DEFAULTS = dict(
    fix_timeout_s=3.0,          # a fix older than this is "lost"
    motion_timeout_s=1.5,       # motion input older than this is absent
    coast_s=5.0,                # dead reckoning without any motion input (constant velocity)
    dr_max_s={"vehicle": 900.0, "vision": 120.0},
    dr_max_m={"vehicle": 25000.0, "vision": 3000.0},
    dr_max_hacc_m=150.0,
    max_fix_hacc_m=100.0,       # worse fixes are ignored
    default_hacc_m=10.0,
    max_input_age_s=5.0, max_input_future_s=0.5,
    match_period_s=0.45,
    horizon_m=1500.0, shape_m=600.0,
    wheelbase_m=2.70, steering_ratio=19.1, track_m=1.48,      # Toyota Prius (XW20) nominal values
    vision_speed_sigma=(0.5, 0.1), vision_yaw_sigma_deg=2.0,
    feedback_min_conf=0.5, feedback_cross_m=4.0, feedback_heading_deg=6.0,
    feedback_max_dist_m=10.0, feedback_max_heading_deg=10.0,
    country="pl",
)


def _num(m, k, lo=None, hi=None, required=False):
    v = m.get(k)
    if v is None:
        if required:
            raise ValueError("missing field %s" % k)
        return None
    if isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v):
        raise ValueError("%s must be a finite number" % k)
    if (lo is not None and v < lo) or (hi is not None and v > hi):
        raise ValueError("%s out of range" % k)
    return float(v)


class Engine:
    def __init__(self, pack=None, cfg=None):
        self.cfg = dict(DEFAULTS, **(cfg or {}))
        self.limits = context.load_limits(self.cfg["country"])
        self.filter = PositionFilter(self.cfg.get("filter"))
        self.pack = None
        self.matcher = None
        self.set_pack(pack)
        self.t_filter = None            # time the filter state refers to
        self.last_fix = None            # ts of the last fix used
        self.last_gnss = None           # last gnss message (any quality) with its ts
        self.fix_lost = False           # the feeder said fix "none"
        self.motion = None              # {"ts", "source", "v", "yaw_cw", "reverse", "tag"}
        self.odo = 0.0
        self.odo_fix = 0.0
        self.odo_match = 0.0
        self.match = None
        self.t_match = None
        self.state = "no_source"
        self.seq = 0
        self.counts = {"gnss": 0, "gnss_used": 0, "gnss_ignored": 0, "motion": 0, "vision": 0, "errors": 0}
        self.last_error = None
        self._anchor = None

    def set_pack(self, pack):
        self.pack = pack
        self.matcher = MapMatcher(pack, self.cfg.get("matcher")) if pack is not None else None
        self.match = None

    # ---- input -------------------------------------------------------------------------------
    def on_input(self, msg, now_ns):
        """One message of the position input API. Returns None, or the reason it was rejected."""
        try:
            if not isinstance(msg, dict):
                raise ValueError("not a JSON object")
            t = msg.get("type")
            if t == "hello":
                return None
            if t not in ("gnss", "motion"):
                raise ValueError("unknown type %r" % (t,))
            ts = msg.get("ts_mono_ns")
            if ts is None:
                ts = now_ns
            elif isinstance(ts, bool) or not isinstance(ts, int):
                raise ValueError("ts_mono_ns must be an integer")
            elif ts < now_ns - self.cfg["max_input_age_s"] * NS:
                raise ValueError("ts_mono_ns is %.1f s old (CLOCK_MONOTONIC of this host expected)"
                                 % ((now_ns - ts) / NS))
            elif ts > now_ns + self.cfg["max_input_future_s"] * NS:
                raise ValueError("ts_mono_ns is in the future (CLOCK_MONOTONIC of this host expected)")
            if t == "gnss":
                self._on_gnss(msg, ts)
            else:
                self._on_motion(msg, ts)
            return None
        except ValueError as e:
            self.counts["errors"] += 1
            self.last_error = str(e)
            return str(e)

    def _advance(self, ts, yaw_cw, yaw_sigma, measured):
        """Predict the filter to ts with the given yaw rate; integrates the odometer."""
        f = self.filter
        if not f.initialised:
            self.t_filter = ts
            return
        if self.t_filter is None:
            self.t_filter = ts
            return
        dt = (ts - self.t_filter) / NS
        if dt <= 0:
            return
        if dt > 1.0:                                  # a gap in the input: no yaw information for it
            f.predict(min(dt, 5.0), None)
        else:
            f.predict(dt, yaw_cw, yaw_sigma, speed_measured=measured)
        self.odo += abs(f.speed) * min(dt, 5.0)
        self.t_filter = ts

    def _on_motion(self, m, ts, source="vehicle"):
        v = _num(m, "speed_mps", 0.0, 120.0, required=True)
        gear = m.get("gear")
        if gear is not None and gear not in ("P", "R", "N", "D", "B"):
            raise ValueError("gear must be one of P R N D B")
        reverse = bool(m.get("reverse")) or gear == "R"
        vs = -v if reverse else v
        yaw_cw = None
        yr = _num(m, "yaw_rate_rps", -3.0, 3.0)
        ws = m.get("wheel_speeds_mps")
        sw = _num(m, "steering_wheel_deg", -1000.0, 1000.0)
        if yr is not None:
            yaw_cw = -yr
        elif isinstance(ws, list) and len(ws) == 4 and all(isinstance(x, (int, float)) for x in ws):
            yaw_cw = -(ws[3] - ws[2]) / self.cfg["track_m"] * (-1.0 if reverse else 1.0)
        elif sw is not None:
            delta = math.radians(sw / self.cfg["steering_ratio"])
            yaw_cw = -vs * math.tan(delta) / self.cfg["wheelbase_m"]
        self.counts["motion" if source == "vehicle" else "vision"] += 1
        if source == "vision":
            a, b = self.cfg["vision_speed_sigma"]
            sig_v, sig_y = a + b * v, math.radians(self.cfg["vision_yaw_sigma_deg"])
        else:
            sig_v, sig_y = None, None
        self._advance(ts, yaw_cw, sig_y, True)
        self.filter.update_speed(vs, sig_v)
        self.motion = {"ts": ts, "source": source, "v": vs, "yaw_cw": yaw_cw, "reverse": reverse,
                       "tag": m.get("source")}

    def on_vision(self, road, now_ns):
        """A modeld `road` message as a motion source of last resort (used only while no vehicle motion arrives)."""
        mo = self.motion
        if mo and mo["source"] == "vehicle" and now_ns - mo["ts"] < self.cfg["motion_timeout_s"] * NS:
            return
        v = road.get("v_ego")
        pose = road.get("pose") or {}
        if v is None:
            v = pose.get("vx")
        ts = road.get("ts_mono_ns")
        if not isinstance(v, (int, float)) or not isinstance(ts, int) or not math.isfinite(v):
            return
        if ts < now_ns - self.cfg["max_input_age_s"] * NS or ts > now_ns + self.cfg["max_input_future_s"] * NS:
            return
        yr = pose.get("yaw_rate")                      # device frame, z down: positive = clockwise
        m = {"speed_mps": min(120.0, max(0.0, float(v))), "source": "modeld"}
        try:
            self._on_motion(m, ts, source="vision")
        except ValueError:
            return
        if isinstance(yr, (int, float)) and math.isfinite(yr):
            self.motion["yaw_cw"] = float(yr)

    def _on_gnss(self, g, ts):
        self.counts["gnss"] += 1
        fix = g.get("fix", "3d")
        if fix not in ("none", "2d", "3d", "dgps", "rtk", "dr"):
            raise ValueError("fix must be one of none 2d 3d dgps rtk dr")
        meta = {"ts": ts, "fix": fix, "source": g.get("source"), "sats": g.get("sats"), "hdop": g.get("hdop"),
                "hacc_m": None, "utc_ms": g.get("utc_ms")}
        if fix == "none":
            self.last_gnss = meta
            self.fix_lost = True
            return
        lat = _num(g, "lat", -90.0, 90.0, required=True)
        lon = _num(g, "lon", -180.0, 180.0, required=True)
        hacc = _num(g, "hacc_m", 0.0, 1e6) or self.cfg["default_hacc_m"]
        speed = _num(g, "speed_mps", 0.0, 120.0)
        speed_acc = _num(g, "speed_acc_mps", 0.0, 100.0)
        course = _num(g, "course_deg", 0.0, 360.0)
        course_acc = _num(g, "course_acc_deg", 0.0, 360.0)
        meta["hacc_m"] = hacc
        self.last_gnss = meta
        if hacc > self.cfg["max_fix_hacc_m"]:
            self.counts["gnss_ignored"] += 1
            return
        f = self.filter
        mo = self.motion
        motion_fresh = mo is not None and ts - mo["ts"] < self.cfg["motion_timeout_s"] * NS
        reverse = bool(mo and motion_fresh and mo["reverse"])
        moving = speed is not None and speed >= 2.5
        if not f.initialised or self.state in ("unknown", "no_source"):
            f.init(lat, lon, hacc, speed=(mo["v"] if motion_fresh else speed or 0.0),
                   course_deg=course if (moving and course is not None and not reverse) else None)
            self.t_filter = ts
            used = True
            self.state = "good"
            self._anchor = (lat, lon)
            if self.matcher:
                self.matcher.reset()
        else:
            if not motion_fresh:
                self._advance(ts, None, None, False)
            lag = (self.t_filter - ts) / NS if self.t_filter is not None else 0.0
            if 0.0 < lag <= 2.0 and f.heading_known:
                # the fix is older than the filter state (motion arrived since): carry it forward along the heading
                h = math.radians(f.heading_deg)
                lat, lon = geo.offset(lat, lon, f.speed * lag * math.cos(h), f.speed * lag * math.sin(h))
            used = f.update_position(lat, lon, hacc)
            if used:
                if speed is not None and not reverse:
                    f.update_speed(speed, max(speed_acc or 0.5, 0.3))
                if course is not None and moving and not reverse:
                    f.update_course(course, max(course_acc or 0.0, math.degrees(math.atan2(0.5, speed)) + 2.0))
                elif not f.heading_known and not reverse and self._anchor is not None:
                    # no course from the receiver: take the first heading from the displacement between fixes
                    d = geo.dist_m(self._anchor[0], self._anchor[1], lat, lon)
                    if d > max(12.0, 2.5 * hacc):
                        f.update_course(geo.bearing_deg(self._anchor[0], self._anchor[1], lat, lon), 20.0)
        if used:
            self.counts["gnss_used"] += 1
            self.last_fix = ts
            self.fix_lost = False
            self.odo_fix = self.odo
        else:
            self.counts["gnss_ignored"] += 1

    # ---- output ------------------------------------------------------------------------------
    def _motion_source(self, now_ns):
        mo = self.motion
        if mo is None or now_ns - mo["ts"] > self.cfg["motion_timeout_s"] * NS:
            return "none"
        return mo["source"]

    def _position_state(self, now_ns, src):
        c = self.cfg
        if not self.filter.initialised or self.last_fix is None:
            return "no_source"
        age = (now_ns - self.last_fix) / NS
        if age <= c["fix_timeout_s"] and not self.fix_lost:
            return "good"
        if self.state == "unknown":
            return "unknown"                          # only a new fix leaves this state
        if self.filter.hacc > c["dr_max_hacc_m"]:
            return "unknown"
        if src == "none":
            return "dead_reckoning" if age <= c["coast_s"] else "unknown"
        if age > c["dr_max_s"][src] or self.odo - self.odo_fix > c["dr_max_m"][src]:
            return "unknown"
        return "dead_reckoning"

    def step(self, now_ns, real_ns=None):
        """Advance to now_ns and return the `map` message."""
        c = self.cfg
        f = self.filter
        src = self._motion_source(now_ns)
        if f.initialised and src == "none" and self.state in ("good", "dead_reckoning") and self.t_filter is not None:
            self._advance(now_ns, None, None, False)   # coast at constant velocity
        prev = self.state
        self.state = self._position_state(now_ns, src)
        if self.state == "unknown" and prev != "unknown" and self.matcher:
            self.matcher.reset()
            self.match = None
        located = self.state in ("good", "dead_reckoning")
        if located and self.matcher is not None:
            due = self.t_match is None or (now_ns - self.t_match) / NS >= c["match_period_s"]
            standing = abs(f.speed) < 0.3 and self.match is not None and self.state == "good"
            if due and not standing:
                lat, lon = f.latlon
                mo = self.motion
                reverse = bool(mo and src != "none" and mo["reverse"])
                self.match = self.matcher.update(lat, lon, f.heading_deg if f.heading_known else None, f.speed,
                                                 f.hacc, self.odo - self.odo_match, reverse=reverse,
                                                 cross_sigma=f.cross_sigma)
                self.odo_match = self.odo
                self.t_match = now_ns
                m = self.match
                # Map feedback while dead reckoning. It may only nudge: a match the filter does not already
                # agree with (far off, or at an angle) is not fed back, or a wrong branch at a fork would
                # drag the position along with it.
                if m is not None and self.state == "dead_reckoning" and m.confidence >= c["feedback_min_conf"] \
                        and not m.ambiguous and m.dist <= c["feedback_max_dist_m"] \
                        and (m.heading_diff is None or abs(m.heading_diff) <= c["feedback_max_heading_deg"]):
                    f.update_on_road(m.lat, m.lon, m.bearing, c["feedback_cross_m"], 1000.0)
                    yaw = abs(mo["yaw_cw"] or 0.0) if mo and src != "none" else 0.0
                    if abs(f.speed) > 2.0 and yaw < math.radians(3.0):
                        f.update_course((m.bearing + 180.0) % 360.0 if reverse else m.bearing,
                                        c["feedback_heading_deg"], gate_deg=c["feedback_max_heading_deg"])
        elif not located:
            self.match = None
        return self._message(now_ns, real_ns, src)

    def _message(self, now_ns, real_ns, src):
        c = self.cfg
        f = self.filter
        self.seq += 1
        located = self.state in ("good", "dead_reckoning")
        m = self.match if located else None
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
        out = {"type": "map", "seq": self.seq, "ts_mono_ns": now_ns, "ts_real_ns": real_ns, "api": API_VERSION,
               "state": self.state, "available": reason is None, "reason": reason}
        man = self.pack.manifest if self.pack is not None else None
        out["pack"] = {"region": man.get("region"), "osm_timestamp": man.get("osm_timestamp"),
                       "built": man.get("built"), "format": man.get("format")} if man else None
        g = self.last_gnss
        gn = None
        if g:
            gn = {"source": g["source"], "fix": g["fix"], "age_s": round((now_ns - g["ts"]) / NS, 2),
                  "hacc_m": g["hacc_m"], "sats": g["sats"], "hdop": g["hdop"],
                  "used": self.counts["gnss_used"], "ignored": self.counts["gnss_ignored"],
                  "rejected": f.rejected_total, "resets": f.resets}
        mo = self.motion
        out["input"] = {"gnss": gn, "motion": {"source": src, "tag": mo["tag"] if mo and src != "none" else None,
                                               "age_s": round((now_ns - mo["ts"]) / NS, 2) if mo else None},
                        "errors": self.counts["errors"], "last_error": self.last_error}
        if located:
            lat, lon = f.latlon
            age = (now_ns - self.last_fix) / NS
            dr_m = self.odo - self.odo_fix
            if self.state == "good":
                conf = max(0.0, min(1.0, 1.0 - f.hacc / 60.0))
            else:
                conf = max(0.0, min(0.9, 1.0 - f.hacc / c["dr_max_hacc_m"]))
            out["position"] = {
                "lat": round(lat, 7), "lon": round(lon, 7),
                "heading_deg": round(f.heading_deg, 1) if f.heading_known else None,
                "heading_acc_deg": round(f.heading_sigma_deg, 1) if f.heading_known else None,
                "speed_mps": round(f.speed, 2), "hacc_m": round(f.hacc, 1), "confidence": round(conf, 2),
                "source": "gnss" if self.state == "good" else "dead_reckoning",
                "fix_age_s": round(age, 2),
                "dr_distance_m": round(dr_m, 1) if self.state == "dead_reckoning" else 0.0,
                "motion_source": src, "odometer_m": round(self.odo, 1)}
        else:
            out["position"] = None
        if reason is None:
            e = m.edge
            out["match"] = m.as_dict()
            out["road"] = context.road_info(e, m.dir)
            out["speed_limit"] = context.speed_limit(e, m.dir, self.limits)
            out["ahead"] = context.look_ahead(self.pack, m, self.limits, c["horizon_m"], c["shape_m"])
        else:
            out["match"] = {"matched": False} if located and self.pack is not None else None
            out["road"] = out["speed_limit"] = out["ahead"] = None
        return out
