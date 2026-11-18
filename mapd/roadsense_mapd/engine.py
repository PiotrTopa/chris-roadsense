"""mapd core: vehicle input (GNSS, motion, controls, body, range sensors) -> filter -> map matching -> the
`map` message, plus the car as the host described it (`ego`, `vehicle`, `objects`; vehicle.py).

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
from .vehicle import (HOLD_S, TURN_SIGNALS, Calibration, VehicleStore, parse_description, parse_quality,
                      parse_signals)

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
    # Vehicle description: the feeder's `vehicle` message > the "vehicle" object here > these older top-level
    # keys > nominal defaults (vehicle.py DESCRIPTION_DEFAULTS).
    wheelbase_m=None, steering_ratio=None, track_m=None, vehicle=None,
    pair_s=0.3,                 # a yaw signal of another motion message is used with a speed this close in time
    use_calibration=True,       # apply roadsense's own scale estimates to signals declared "uncalibrated"
    gnss_lag_s=0.0,             # a receiver that reports late: its fixes are this much older than their timestamp
    calibration=None,           # overrides of vehicle.Calibration.CFG
    vision_speed_sigma=(0.5, 0.1), vision_yaw_sigma_deg=2.0,
    feedback_min_conf=0.5, feedback_cross_m=4.0, feedback_heading_deg=6.0,
    feedback_max_dist_m=10.0, feedback_max_heading_deg=10.0,
    # Receiver accuracy (lane L20): a receiver's own hacc can be far too optimistic (the reference car's head
    # unit reported 1-4 m while its fixes scattered 10-15 m across the road with 3-5 satellites). The ratio of
    # the fixes' innovations to what hacc promises is tracked over the last fixes and the hacc in use is
    # scaled by it (never below the receiver's own value, at most gnss_scale_max times it).
    gnss_adaptive=True, gnss_scale_n=20.0, gnss_scale_max=6.0,
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
        self.view_memo = {}             # orientation of the last roundabout view (context.junction_view)
        self.set_pack(pack)
        self.t_filter = None            # time the filter state refers to
        self.last_fix = None            # ts of the last fix used
        self.last_gnss = None           # last gnss message (any quality) with its ts
        self.fix_lost = False           # the feeder said fix "none"
        self.motion = None              # {"ts", "source", "v", "yaw_cw", "reverse", "tag"}
        self.ego = None                 # the last vehicle `motion` as an `ego` message (relayed by the daemon)
        self.ego_seq = 0
        self.vehicle = VehicleStore(self.cfg.get("vehicle"),
                                    {k: self.cfg.get(k) for k in ("wheelbase_m", "steering_ratio", "track_m")})
        self.cal = Calibration(self.cfg.get("calibration"))
        self.objects_out = []           # `objects` messages accepted since the daemon last took them (relay)
        self.odo = 0.0
        self.odo_fix = 0.0
        self.odo_match = 0.0
        self.match = None
        self.t_match = None
        self.state = "no_source"
        self.seq = 0
        self.counts = {"gnss": 0, "gnss_used": 0, "gnss_ignored": 0, "motion": 0, "vision": 0, "errors": 0,
                       "controls": 0, "body": 0, "vehicle": 0, "objects": 0}
        self.last_error = None
        self._anchor = None
        self.synthetic = False          # the position state was fed by a source tagged synthetic since the last reset
        self._fed_synth = None          # kind of the GNSS source that fed the state: True synthetic, False real, None nothing yet
        self.resets = 0
        self.last_reset = None          # {"ts", "reason"}
        self.gnss_nis = 1.0             # running mean of (innovation / promised sigma)^2 per axis (1 = honest)

    @property
    def gnss_scale(self):
        """Factor applied to the receiver's hacc (1 = taken as reported)."""
        if not self.cfg["gnss_adaptive"]:
            return 1.0
        return min(self.cfg["gnss_scale_max"], math.sqrt(max(1.0, self.gnss_nis)))

    def set_pack(self, pack):
        self.pack = pack
        self.matcher = MapMatcher(pack, self.cfg.get("matcher")) if pack is not None else None
        self.match = None

    @staticmethod
    def is_synthetic(tag):
        """A `source` tag that marks made-up input: "synthetic", or anything starting with it ("synthetic-drive")."""
        return isinstance(tag, str) and tag.strip().lower().startswith("synthetic")

    def reset(self, ts, reason=None):
        """Forget the position: filter, fix, match. The map goes back to `no_source` until a new fix arrives.
        Motion input (and with it the `ego` relay) is not touched; the odometer keeps counting."""
        self.filter = PositionFilter(self.cfg.get("filter"))
        self.t_filter = None
        self.last_fix = None
        self.last_gnss = None
        self.fix_lost = False
        self.match = None
        self.t_match = None
        if self.matcher:
            self.matcher.reset()
        self.state = "no_source"
        self._anchor = None
        self.odo_fix = self.odo_match = self.odo
        self.synthetic = False
        self._fed_synth = None
        self.resets += 1
        self.last_reset = {"ts": ts, "reason": reason}

    # ---- input -------------------------------------------------------------------------------
    TYPES = ("gnss", "motion", "reset", "controls", "body", "vehicle", "objects")

    def on_input(self, msg, now_ns):
        """One message of the vehicle input API. Returns None, or the reason it was rejected."""
        try:
            if not isinstance(msg, dict):
                raise ValueError("not a JSON object")
            t = msg.get("type")
            if t == "hello":
                return None
            if t not in self.TYPES:
                raise ValueError("unknown type %r" % (t,))
            ts = msg.get("ts_mono_ns")
            age = msg.get("age_ms")
            if ts is None and age is not None:
                # the sample's age at the moment of sending: for a feeder that relays from another device
                # and knows the age but shares no clock with this host
                if isinstance(age, bool) or not isinstance(age, (int, float)) or not math.isfinite(age) or age < 0:
                    raise ValueError("age_ms must be a number >= 0")
                if age > self.cfg["max_input_age_s"] * 1e3:
                    raise ValueError("age_ms: the sample is %.1f s old" % (age / 1e3))
                ts = now_ns - int(age * 1e6)
            elif ts is None:
                ts = now_ns
            elif isinstance(ts, bool) or not isinstance(ts, int):
                raise ValueError("ts_mono_ns must be an integer")
            elif ts < now_ns - self.cfg["max_input_age_s"] * NS:
                raise ValueError("ts_mono_ns is %.1f s old (CLOCK_MONOTONIC of this host expected)"
                                 % ((now_ns - ts) / NS))
            elif ts > now_ns + self.cfg["max_input_future_s"] * NS:
                raise ValueError("ts_mono_ns is in the future (CLOCK_MONOTONIC of this host expected)")
            if t == "reset":
                reason = msg.get("reason")
                if reason is not None and not isinstance(reason, str):
                    raise ValueError("reason must be a string")
                self.reset(ts, (reason or "requested")[:80])
            elif t == "gnss":
                self._on_gnss(msg, ts)
            elif t == "motion":
                self._on_motion(msg, ts)
            elif t == "vehicle":
                self.vehicle.describe(parse_description(msg))
                self.counts["vehicle"] += 1
            elif t == "objects":
                self.objects_out.append(self.vehicle.objects(msg, ts, msg.get("source")))
                del self.objects_out[:-8]
                self.counts["objects"] += 1
            else:
                self._on_signals(t, msg, ts)
            return None
        except ValueError as e:
            self.counts["errors"] += 1
            self.last_error = str(e)
            return str(e)

    def _on_signals(self, group, m, ts):
        """A `controls` or `body` message: stored for the `vehicle` topic (and the `ego` relay's turn signal)."""
        sig = parse_signals(group, m)
        if not sig:
            raise ValueError("no known signal in the %s message" % group)
        q, dq = parse_quality(m)
        self.vehicle.update(group, sig, ts, q, dq, m.get("source"))
        self.counts[group] += 1

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

    def _recent(self, name, ts, max_age_s, any_quality=False):
        """A usable vehicle signal for the motion message of time ts: (value, quality). The message's own, or
        one that a message without a speed delivered within max_age_s (a gyroscope on another bus, say). What an
        earlier complete message carried is never reused: a complete message stands for itself, as in v1."""
        s = self.vehicle.sig.get(name)
        if s is None or s[2] == "invalid" or (not any_quality and s[2] not in ("good", "uncalibrated")):
            return None, None
        if s[1] != ts and not (s[4] and abs(ts - s[1]) <= max_age_s * NS):
            return None, None
        return s[0], s[2]

    def _on_motion(self, m, ts, source="vehicle"):
        c = self.cfg
        tag = m.get("source")
        turn = None
        if source == "vehicle":
            sig = parse_signals("motion", m)
            turn = m.get("turn_signal")                  # its place is `controls`; accepted here since v1
            if turn is not None and turn not in TURN_SIGNALS:
                raise ValueError("turn_signal must be one of off left right both")
            if not sig and turn is None:
                raise ValueError("missing field speed_mps")
            q, dq = parse_quality(m)
            self.vehicle.update("motion", sig, ts, q, dq, tag, partial="speed_mps" not in sig)
            if turn is not None:
                self.vehicle.update("controls", {"turn_signal": turn}, ts, q, dq, tag)
            v, vq = self._recent("speed_mps", ts, 0.0) if "speed_mps" in sig else (None, None)
            if v is None:
                self.counts["motion"] += 1
                return                                   # stored; the filter moves with messages that carry a speed
            gear, _ = self._recent("gear", ts, HOLD_S["motion"])
            rev, _ = self._recent("reverse", ts, HOLD_S["motion"])
            yr, yq = self._recent("yaw_rate_rps", ts, c["pair_s"])
            ws, _ = self._recent("wheel_speeds_mps", ts, c["pair_s"])
            sw, _ = self._recent("steering_wheel_deg", ts, c["pair_s"])
        else:
            v, vq = _num(m, "speed_mps", 0.0, 120.0, required=True), "good"
            gear = rev = yr = yq = ws = sw = None
        reverse = bool(rev) or gear == "R"
        d = self.vehicle.desc
        standstill = v == 0.0 and (ws is None or not any(ws))
        yaw_wheels = None if ws is None else (ws[3] - ws[2]) / d["track_m"] * (-1.0 if reverse else 1.0)
        if source == "vehicle":
            # the estimators also get a yaw rate that is not used ("unverified"): its scale against GNSS is
            # what tells the host whether to trust it
            yr_any, _ = self._recent("yaw_rate_rps", ts, c["pair_s"], any_quality=True)
            self.cal.set_tag(tag)
            self.cal.on_motion(ts, v, standstill, yr_any, yaw_wheels)
            if vq == "uncalibrated" and c["use_calibration"] and self.cal.speed_scale() is not None:
                v *= self.cal.speed_scale()
        vs = -v if reverse else v
        yaw_cw, yaw_src = None, None
        if yr is not None:
            yaw = yr - self.cal.bias
            if yq == "uncalibrated" and c["use_calibration"] and self.cal.yaw_scale() is not None:
                yaw *= self.cal.yaw_scale()
            yaw_cw, yaw_src = -yaw, "yaw_rate"
        elif yaw_wheels is not None:
            yaw_cw, yaw_src = -yaw_wheels, "wheel_speeds"
        elif sw is not None:
            delta = math.radians(sw / d["steering_ratio"])
            yaw_cw, yaw_src = -vs * math.tan(delta) / d["wheelbase_m"], "steering"
        if standstill and yaw_cw is not None:
            yaw_cw = 0.0                                 # a standing car does not turn, whatever the sensor says
        self.counts["motion" if source == "vehicle" else "vision"] += 1
        if source == "vision":
            a, b = self.cfg["vision_speed_sigma"]
            sig_v, sig_y = a + b * v, math.radians(self.cfg["vision_yaw_sigma_deg"])
        else:
            sig_v, sig_y = None, None
        self._advance(ts, yaw_cw, sig_y, True)
        self.filter.update_speed(vs, sig_v)
        self.motion = {"ts": ts, "source": source, "v": vs, "yaw_cw": yaw_cw, "reverse": reverse,
                       "tag": tag}
        if self.filter.initialised and self.is_synthetic(tag):
            self.synthetic = True                     # a real fix carried on by made-up motion is made up too
        if source == "vehicle":
            if turn is None:
                turn = self.vehicle.get("turn_signal", ts)
            self.ego_seq += 1
            self.ego = {"type": "ego", "seq": self.ego_seq, "ts_mono_ns": ts, "api": API_VERSION,
                        "speed_mps": vs, "gear": gear, "reverse": reverse,
                        "standstill": standstill,
                        "wheel_speeds_mps": ws,
                        "yaw_rate_rps": None if yaw_cw is None else round(-yaw_cw, 5),
                        "yaw_source": yaw_src,
                        "turn_signal": turn,
                        "source": "vehicle", "tag": tag}
            for k in ("steering_wheel_deg", "accel_long_mps2", "accel_lat_mps2"):
                val, _ = self._recent(k, ts, c["pair_s"])
                if val is not None:
                    self.ego[k] = val

    def vehicle_message(self, now_ns, real_ns=None, feeders=None):
        """The `vehicle` message: every signal the host sent, as roadsense holds it now."""
        ego = self.ego
        motion = None
        if ego is not None and now_ns - ego["ts_mono_ns"] <= self.cfg["motion_timeout_s"] * NS:
            motion = {k: ego[k] for k in ("speed_mps", "standstill", "gear", "reverse", "yaw_rate_rps", "yaw_source")}
        return self.vehicle.message(now_ns, real_ns, motion, self.cal.report(), feeders)

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
        synth = self.is_synthetic(g.get("source"))
        if self._fed_synth is not None and synth != self._fed_synth:
            # a synthetic session ended and the real receiver speaks (or the other way round): the position of
            # the one must not live on as the position of the other
            self.reset(ts, "gnss source changed from %s to %s" % (("real", "synthetic") if synth else ("synthetic", "real")))
        if self.cfg["gnss_lag_s"] and fix != "none":
            ts -= int(self.cfg["gnss_lag_s"] * NS)
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
        raw = (lat, lon)
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
            if self.state == "good" and self.last_fix is not None and ts - self.last_fix <= 2.5 * NS:
                # receiver accuracy: only between consecutive fixes (after dead reckoning the innovation is the
                # dead-reckoning error, not the receiver's)
                inn = f.innovation(lat, lon, hacc)
                if inn is not None:
                    n = self.cfg["gnss_scale_n"]
                    self.gnss_nis += (min(inn[0] / 2.0, 36.0) - self.gnss_nis) / n
            hacc = hacc * self.gnss_scale
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
            self.cal.on_fix(ts, raw[0], raw[1], hacc, speed, course if moving and not reverse else None, geo.dist_m)
            self.counts["gnss_used"] += 1
            self.last_fix = ts
            self.fix_lost = False
            self.odo_fix = self.odo
            self._fed_synth = synth
            if synth:
                self.synthetic = True
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
                  "rejected": f.rejected_total, "resets": f.resets,
                  "hacc_scale": round(self.gnss_scale, 2)}     # lane L20: factor on the receiver's hacc in use
        mo = self.motion
        out["input"] = {"gnss": gn, "motion": {"source": src, "tag": mo["tag"] if mo and src != "none" else None,
                                               "age_s": round((now_ns - mo["ts"]) / NS, 2) if mo else None},
                        "errors": self.counts["errors"], "last_error": self.last_error,
                        "synthetic": self.synthetic, "resets": self.resets,
                        "last_reset": None if self.last_reset is None else {
                            "reason": self.last_reset["reason"],
                            "age_s": round((now_ns - self.last_reset["ts"]) / NS, 2)}}
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
            out["ahead"] = context.look_ahead(self.pack, m, self.limits, c["horizon_m"], c["shape_m"],
                                               memo=self.view_memo)
        else:
            out["match"] = {"matched": False} if located and self.pack is not None else None
            out["road"] = out["speed_limit"] = out["ahead"] = None
        return out
