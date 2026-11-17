"""Vehicle input: what a host tells roadsense about the car (docs/VEHICLE_API.md).

Three things live here, all without I/O and without a wall clock (times are CLOCK_MONOTONIC ns of this host):

    schema        the signals of the `motion`, `controls` and `body` messages with their units and ranges, the
                  static `vehicle` description and the `objects` lists of range sensors: validation only
    VehicleStore  the latest value of every signal with its time, quality and source tag, the description
                  (feeder > mapd.json > defaults) and the last scan of every range sensor; it writes the
                  `vehicle` and `objects` messages that mapd relays on ego.sock
    Calibration   what roadsense can estimate and a feeder cannot: yaw-rate bias (standstill) and scale (against
                  the GNSS course over whole turns, and against the wheel-speed yaw), wheel-yaw scale, and the
                  scale of the vehicle speed against GNSS

Frames. Input uses the vehicle frame of ISO 8855: x forward, y to the left, z up, origin on the ground under
the centre of the rear axle; angles and yaw rates are positive to the left (counter-clockwise seen from above).
roadsense's output topics use the camera frame of the model (x forward, y to the RIGHT, origin at the camera):
the `objects` relay converts, so a consumer of the output never mixes the two.
"""
import bisect
import json
import math
import os

NS = 1e9
QUALITIES = ("good", "uncalibrated", "unverified", "invalid")
USABLE = ("good", "uncalibrated")      # roadsense acts on these; `unverified` is reported only, `invalid` is absent
TURN_SIGNALS = ("off", "left", "right", "both")
GEARS = ("P", "R", "N", "D", "B")
CRUISE = ("off", "standby", "active")
WIPERS = ("off", "intermittent", "low", "high")
SENSOR_KINDS = ("radar", "lidar", "ultrasonic", "camera", "other")
OBJECT_CLASSES = ("car", "truck", "motorcycle", "bicycle", "pedestrian", "unknown")
MAX_OBJECTS = 256

# signal -> spec: ("num", lo, hi) | ("bool",) | ("enum", values) | ("wheels",)
MOTION = {
    "speed_mps": ("num", 0.0, 120.0),
    "gear": ("enum", GEARS),
    "reverse": ("bool",),
    "yaw_rate_rps": ("num", -3.0, 3.0),
    "wheel_speeds_mps": ("wheels",),
    "steering_wheel_deg": ("num", -1500.0, 1500.0),
    "accel_long_mps2": ("num", -30.0, 30.0),
    "accel_lat_mps2": ("num", -30.0, 30.0),
}
CONTROLS = {
    "turn_signal": ("enum", TURN_SIGNALS),
    "brake": ("bool",),
    "brake_pressure": ("num", 0.0, 1.0),
    "accelerator": ("num", 0.0, 1.0),
    "cruise": ("enum", CRUISE),
    "cruise_set_speed_mps": ("num", 0.0, 120.0),
    "parking_brake": ("bool",),
}
BODY = {
    "low_beam": ("bool",),
    "high_beam": ("bool",),
    "parking_lights": ("bool",),
    "fog_front": ("bool",),
    "fog_rear": ("bool",),
    "wipers": ("enum", WIPERS),
    "rain": ("num", 0.0, 1.0),
    "ambient_light_lux": ("num", 0.0, 200000.0),
    "outside_temp_c": ("num", -60.0, 70.0),
    "door_open": ("bool",),
}
GROUPS = {"motion": MOTION, "controls": CONTROLS, "body": BODY}
GROUP_OF = {k: g for g, spec in GROUPS.items() for k in spec}
# A signal older than this is reported as null (unknown). The rate guidance follows from it: motion at 10-50 Hz,
# controls on every change and once a second, body on every change and every 5 s.
HOLD_S = {"motion": 1.5, "controls": 3.0, "body": 15.0}

DESCRIPTION_NUM = {
    "wheelbase_m": (1.0, 8.0), "track_m": (0.8, 3.0), "track_front_m": (0.8, 3.0), "steering_ratio": (5.0, 40.0),
    "length_m": (1.0, 25.0), "width_m": (0.5, 3.5), "height_m": (0.5, 5.0),
    "rear_axle_to_front_m": (0.5, 20.0), "wheel_speed_min_mps": (0.0, 3.0),
}
# Nominal values of a compact car. They only matter for a feeder that sends neither a yaw rate nor a `vehicle`
# message; `description.sources` says "default" for them.
DESCRIPTION_DEFAULTS = {"wheelbase_m": 2.70, "track_m": 1.48, "steering_ratio": 19.1}
MOUNT_NUM = {"x_m": (-10.0, 25.0), "y_m": (-3.0, 3.0), "z_m": (0.0, 5.0)}
SENSOR_NUM = dict(MOUNT_NUM, yaw_deg=(-180.0, 180.0), fov_deg=(0.0, 360.0), max_range_m=(0.0, 1000.0))
OBJECT_NUM = {
    "range_m": (0.0, 1000.0), "azimuth_deg": (-180.0, 180.0), "x_m": (-1000.0, 1000.0), "y_m": (-1000.0, 1000.0),
    "range_rate_mps": (-150.0, 150.0), "vx_mps": (-150.0, 150.0), "vy_mps": (-150.0, 150.0),
    "rcs_dbsm": (-100.0, 100.0), "prob": (0.0, 1.0), "width_m": (0.0, 30.0), "length_m": (0.0, 50.0),
}


def _fin(v):
    return isinstance(v, (int, float)) and not isinstance(v, bool) and math.isfinite(v)


def _number(name, v, lo, hi):
    if not _fin(v):
        raise ValueError("%s must be a finite number" % name)
    if v < lo or v > hi:
        raise ValueError("%s out of range" % name)
    return float(v)


def parse_signals(group, msg):
    """The signals of one `motion` / `controls` / `body` message: {signal: value}. A signal that is absent or
    null is left out; a malformed one raises ValueError. Unknown fields are ignored."""
    out = {}
    for name, spec in GROUPS[group].items():
        v = msg.get(name)
        if v is None:
            continue
        kind = spec[0]
        if kind == "num":
            out[name] = _number(name, v, spec[1], spec[2])
        elif kind == "bool":
            if not isinstance(v, bool):
                raise ValueError("%s must be true or false" % name)
            out[name] = v
        elif kind == "enum":
            if v not in spec[1]:
                raise ValueError("%s must be one of %s" % (name, " ".join(spec[1])))
            out[name] = v
        else:                                       # wheels: four speeds; anything else is ignored (as in v1)
            if isinstance(v, list) and len(v) == 4 and all(_fin(x) and 0.0 <= x <= 120.0 for x in v):
                out[name] = [float(x) for x in v]
    return out


def parse_quality(msg, default="good"):
    """The `quality` object of a message: {signal: quality}, with "*" for every signal of the message."""
    q = msg.get("quality")
    if q is None:
        return {}, default
    if not isinstance(q, dict):
        raise ValueError("quality must be an object {signal: quality}")
    for k, v in q.items():
        if v not in QUALITIES:
            raise ValueError("quality of %s must be one of %s" % (k, " ".join(QUALITIES)))
    return {k: v for k, v in q.items() if k != "*"}, q.get("*", default)


def _mount(name, m, spec):
    if not isinstance(m, dict):
        raise ValueError("%s must be an object" % name)
    out = {}
    for k, (lo, hi) in spec.items():
        if m.get(k) is not None:
            out[k] = _number("%s.%s" % (name, k), m[k], lo, hi)
    return out


def parse_description(msg):
    """A `vehicle` message (or the "vehicle" object of mapd.json): the keys it sets, validated."""
    out = {}
    if msg.get("name") is not None:
        if not isinstance(msg["name"], str):
            raise ValueError("name must be a string")
        out["name"] = msg["name"][:80]
    for k, (lo, hi) in DESCRIPTION_NUM.items():
        if msg.get(k) is not None:
            out[k] = _number(k, msg[k], lo, hi)
    if msg.get("camera") is not None:
        out["camera"] = _mount("camera", msg["camera"], MOUNT_NUM)
    if msg.get("sensors") is not None:
        if not isinstance(msg["sensors"], list) or len(msg["sensors"]) > 32:
            raise ValueError("sensors must be a list of at most 32 objects")
        sensors = []
        for s in msg["sensors"]:
            if not isinstance(s, dict) or not isinstance(s.get("id"), str) or not s["id"]:
                raise ValueError("every sensor needs an id (string)")
            kind = s.get("kind", "other")
            if kind not in SENSOR_KINDS:
                raise ValueError("sensor kind must be one of %s" % " ".join(SENSOR_KINDS))
            sensors.append(dict(_mount("sensor " + s["id"], s, SENSOR_NUM), id=s["id"][:40], kind=kind))
        out["sensors"] = sensors
    return out


def parse_objects(msg):
    """The object list of an `objects` message, validated: polar (`range_m`, `azimuth_deg`) or Cartesian
    (`x_m`, `y_m`) in the sensor's frame, at least one of the two per object."""
    objs = msg.get("objects")
    if not isinstance(objs, list):
        raise ValueError("objects must be a list (an empty list = the sensor sees nothing)")
    if len(objs) > MAX_OBJECTS:
        raise ValueError("more than %d objects" % MAX_OBJECTS)
    out = []
    for i, o in enumerate(objs):
        if not isinstance(o, dict):
            raise ValueError("objects[%d] is not an object" % i)
        d = {}
        for k, v in o.items():
            spec = OBJECT_NUM.get(k)
            if spec is not None:
                if v is None:
                    continue
                if v.__class__ is not float and v.__class__ is not int or v != v or not spec[0] <= v <= spec[1]:
                    _number("objects[%d].%s" % (i, k), v, spec[0], spec[1])      # raises with the reason
                d[k] = v
            elif k == "id":
                if v is not None:
                    if isinstance(v, bool) or not isinstance(v, (int, str)):
                        raise ValueError("objects[%d].id must be an integer or a string" % i)
                    d["id"] = v
            elif k == "class" and v is not None:
                if v not in OBJECT_CLASSES:
                    raise ValueError("objects[%d].class must be one of %s" % (i, " ".join(OBJECT_CLASSES)))
                d["class"] = v
        if "range_m" not in d and "x_m" not in d:
            raise ValueError("objects[%d] needs range_m or x_m" % i)
        out.append(d)
    return out


# ---- calibration ---------------------------------------------------------------------------------------------

class _Ratio:
    """Least-squares slope through the origin of y against x (y = scale * x), with a slow forgetting so that it
    follows a change (new tyres) and a state that can be saved."""

    def __init__(self, min_n, max_std, n_cap=2000.0):
        self.sxx = self.sxy = self.syy = 0.0
        self.n = 0.0
        self.min_n, self.max_std, self.n_cap = min_n, max_std, n_cap

    def add(self, x, y, w=1.0):
        if self.n >= self.n_cap:                 # forget: keep the weight of the last n_cap samples
            k = 1.0 - 1.0 / self.n_cap
            self.sxx *= k
            self.sxy *= k
            self.syy *= k
            self.n *= k
        self.sxx += w * x * x
        self.sxy += w * x * y
        self.syy += w * y * y
        self.n += 1.0

    @property
    def value(self):
        return self.sxy / self.sxx if self.sxx > 0.0 else None

    @property
    def std(self):
        if self.n < 3 or self.sxx <= 0.0:
            return None
        s = self.sxy / self.sxx
        res = max(self.syy - s * self.sxy, 0.0) / (self.n - 1.0)
        return math.sqrt(res / self.sxx)

    @property
    def converged(self):
        sd = self.std
        return self.n >= self.min_n and sd is not None and sd <= self.max_std

    def report(self, ref):
        v, sd = self.value, self.std
        return {"value": None if v is None else round(v, 4), "std": None if sd is None else round(sd, 4),
                "n": int(self.n), "converged": self.converged, "ref": ref}

    def dump(self):
        return [self.sxx, self.sxy, self.syy, self.n]

    def load(self, v):
        if isinstance(v, list) and len(v) == 4 and all(_fin(x) for x in v) and v[0] >= 0.0 and v[3] >= 0.0:
            self.sxx, self.sxy, self.syy, self.n = (float(x) for x in v)


class Calibration:
    """Estimators fed by the engine with every accepted `motion` message and every GNSS fix it used.

    yaw-rate bias     mean of `yaw_rate_rps` while the car stands (speed and wheels exactly zero for
                      `still_s`). Always applied: a standing car does not turn.
    yaw-rate scale    (a) against the GNSS course: between two fixes on straights (little turning around the
                      fix, speed above `course_min_mps`, at most `turn_max_s` apart) the integral of the yaw
                      rate has to equal the change of the course; one sample per turn, least squares through
                      the origin. (b) against the yaw of the rear wheel pair: sample by sample while turning.
    wheel-yaw scale   the same as (a) for the wheel-speed yaw: it checks `track_m`.
    speed scale       GNSS speed over ground against the vehicle speed while cruising straight; without a
                      receiver speed, the straight-line distance between two fixes against the distance
                      driven.
    GNSS lag          how late the receiver's positions are: the speed implied by two successive fixes is
                      compared with the vehicle speed 0, 0.25 .. 3 s earlier; the delay with the smallest
                      squared error wins. A receiver that smooths its track shows up as a lag (and as a
                      `speed_ratio` below 1: it under-reads motion). Reported only; `gnss_lag_s` in mapd.json
                      applies a delay by hand.

    Estimates against GNSS are absolute; one counts as `converged` from `min_turns` turns / `min_speed_n`
    samples with a small standard error. The engine applies a converged scale only to a signal the feeder
    declared `uncalibrated`. The sums are kept per source tag in a small state file so that a new drive starts
    where the last one ended.
    """

    CFG = dict(still_s=1.0, bias_tau_s=3.0,
               course_min_mps=5.0, straight_rps=0.02, turn_max_s=40.0, turn_min_rad=0.35, max_fix_hacc_m=15.0,
               min_turns=12, max_scale_std=0.015,
               wheels_min_rps=0.05, wheels_min_mps=3.0,
               speed_min_mps=8.0, speed_max_accel=0.5, min_speed_n=60, max_speed_std=0.004,
               chord_min_m=150.0, chord_max_turn_rad=0.05,
               lag_step_s=0.25, lag_max_s=3.0, lag_min_n=60, lag_min_gain=0.15)

    def __init__(self, cfg=None):
        self.c = dict(self.CFG, **(cfg or {}))
        c = self.c
        self.tag = None
        self.bias = 0.0
        self.bias_n = 0
        self._still_since = None
        self._bias_ts = None
        self.gyro_gnss = _Ratio(c["min_turns"], c["max_scale_std"], 400.0)
        self.wheel_gnss = _Ratio(c["min_turns"], c["max_scale_std"], 400.0)
        self.gyro_wheels = _Ratio(200, 0.01, 20000.0)
        self.speed = _Ratio(c["min_speed_n"], c["max_speed_std"], 5000.0)
        # running integrals since the start; an anchor (a fix on a straight) remembers their values
        self.ts = None
        self.i_gyro = self.i_wheel = self.i_dist = 0.0
        self.has_gyro = self.has_wheels = False
        self._yaw_abs = 0.0            # low-passed |yaw rate| (whichever source), for "on a straight"
        self._v = None
        self._accel = 0.0
        self._anchor = None            # {"ts","course","lat","lon","gyro","wheel","dist","gyro_ok","wheel_ok"}
        self._chord = None             # start of a straight stretch: (lat, lon, distance, yaw integral)
        # GNSS lag: vehicle speed history and, per candidate delay, the squared error of the fix-to-fix speed
        self._vt, self._vv = [], []
        self._lags = [i * c["lag_step_s"] for i in range(int(c["lag_max_s"] / c["lag_step_s"]) + 1)]
        self._lag_sse = [0.0] * len(self._lags)
        self._lag_sxy = [0.0] * len(self._lags)
        self._lag_sxx = [0.0] * len(self._lags)
        self._lag_n = 0
        self._prev_fix = None          # (ts, lat, lon)
        self._gap = False
        self.dirty = False

    # ---- inputs ----
    def on_motion(self, ts, v, standstill, yaw_gyro, yaw_wheels):
        """`yaw_gyro`: the feeder's yaw rate (rad/s, positive left) or None; `yaw_wheels`: from the rear pair."""
        c = self.c
        dt = None if self.ts is None else (ts - self.ts) / NS
        self.ts = ts
        if dt is None or dt <= 0.0 or dt > 0.5:
            self._gap = True                       # an integral with a hole cannot be compared with a course
            dt = 0.0
        if yaw_gyro is not None:
            if standstill:
                if self._still_since is None:
                    self._still_since = ts
                if (ts - self._still_since) / NS >= c["still_s"]:
                    k = 1.0 if self.bias_n == 0 else min(1.0, max(dt, 0.02) / c["bias_tau_s"])
                    self.bias += k * (yaw_gyro - self.bias)
                    self.bias_n += 1
            else:
                self._still_since = None
            self.i_gyro += (yaw_gyro - self.bias) * dt
            self.has_gyro = True
        else:
            self.has_gyro = False
        if yaw_wheels is not None:
            self.i_wheel += yaw_wheels * dt
            self.has_wheels = True
            if yaw_gyro is not None and abs(yaw_wheels) >= c["wheels_min_rps"] and abs(v) >= c["wheels_min_mps"]:
                self.gyro_wheels.add(yaw_gyro - self.bias, yaw_wheels)      # true (wheels) = scale * gyro
        else:
            self.has_wheels = False
        self.i_dist += abs(v) * dt
        self._vt.append(ts)
        self._vv.append(abs(v))
        if len(self._vt) > 1200:
            del self._vt[:600], self._vv[:600]
        yaw = yaw_gyro - self.bias if yaw_gyro is not None else yaw_wheels
        if yaw is not None:
            self._yaw_abs += min(1.0, max(dt, 0.02) / 0.7) * (abs(yaw) - self._yaw_abs)
        if self._v is not None and dt > 0.0:
            self._accel += min(1.0, dt / 0.7) * (abs(v - self._v) / dt - self._accel)
        self._v = v

    def on_fix(self, ts, lat, lon, hacc, speed, course, dist_fn):
        """A fix the filter used. `course` in degrees clockwise from north or None; `dist_fn(lat1, lon1, lat2,
        lon2)` gives metres."""
        c = self.c
        v = self._v
        pf, self._prev_fix = self._prev_fix, (ts, lat, lon)
        if pf is not None and 0.5 * NS <= ts - pf[0] <= 2.5 * NS and hacc <= c["max_fix_hacc_m"] and self._vt:
            self._lag_sample(0.5 * (ts + pf[0]), dist_fn(pf[1], pf[2], lat, lon) / ((ts - pf[0]) / NS))
        if v is None or self.ts is None or abs(ts - self.ts) > 0.5 * NS or hacc > c["max_fix_hacc_m"]:
            return
        straight = self._yaw_abs <= c["straight_rps"]
        if speed is not None and straight and v >= c["speed_min_mps"] and self._accel <= c["speed_max_accel"]:
            self.speed.add(v, speed)               # true (GNSS) = scale * vehicle
            self.dirty = True
        if not straight or v < c["course_min_mps"]:
            self._chord = None
            return
        a, self._anchor = self._anchor, {"ts": ts, "course": course, "lat": lat, "lon": lon, "gyro": self.i_gyro,
                                         "wheel": self.i_wheel, "dist": self.i_dist,
                                         "gyro_ok": self.has_gyro, "wheel_ok": self.has_wheels}
        gap, self._gap = self._gap, False
        if speed is None:                          # no receiver speed: straight-line distance against distance driven
            turn = self.i_gyro if self.has_gyro else self.i_wheel
            ch = self._chord
            if ch is None or gap or abs(turn - ch[3]) > c["chord_max_turn_rad"]:
                self._chord = (lat, lon, self.i_dist, turn)
            else:
                chord, driven = dist_fn(ch[0], ch[1], lat, lon), self.i_dist - ch[2]
                if chord >= c["chord_min_m"]:
                    if driven > 0.0 and abs(chord / driven - 1.0) < 0.2:
                        self.speed.add(1.0, chord / driven, w=400.0)      # weighted like a sample at 20 m/s
                        self.dirty = True
                    self._chord = (lat, lon, self.i_dist, turn)
        if a is None or gap or (ts - a["ts"]) / NS > c["turn_max_s"]:
            return
        d_gyro, d_wheel = self.i_gyro - a["gyro"], self.i_wheel - a["wheel"]
        ref = d_gyro if (a["gyro_ok"] and self.has_gyro) else d_wheel
        if course is not None and a["course"] is not None:
            # the course turns clockwise, the yaw integral counter-clockwise; unwrap around the integral
            d_course = -math.radians(course - a["course"])
            d_course = ref + (d_course - ref + math.pi) % (2.0 * math.pi) - math.pi
            if abs(ref) >= c["turn_min_rad"] and abs(d_course - ref) < 0.5:
                if a["gyro_ok"] and self.has_gyro:
                    self.gyro_gnss.add(d_gyro, d_course)
                if a["wheel_ok"] and self.has_wheels:
                    self.wheel_gnss.add(d_wheel, d_course)
                self.dirty = True

    def _lag_sample(self, t_mid, v_gnss):
        """The speed between two fixes against the vehicle speed `lag` earlier, for every candidate lag."""
        vt, vv = self._vt, self._vv
        if t_mid - self._lags[-1] * NS < vt[0] or t_mid > vt[-1] + 0.5 * NS:
            return
        vals = []
        for lag in self._lags:
            i = bisect.bisect_left(vt, t_mid - lag * NS)
            i = min(max(i, 0), len(vt) - 1)
            if abs(vt[i] - (t_mid - lag * NS)) > 0.6 * NS:
                return
            vals.append(vv[i])
        if max(vals) < 2.0 and v_gnss < 2.0:
            return                                     # standing: says nothing about a delay
        for k, x in enumerate(vals):
            self._lag_sse[k] += (v_gnss - x) ** 2
            self._lag_sxy[k] += v_gnss * x
            self._lag_sxx[k] += x * x
        self._lag_n += 1
        self.dirty = True

    def gnss_lag(self):
        """{"lag_s", "speed_ratio", "n", "converged"}: the delay that explains the fixes best. `converged` needs
        `lag_min_n` fix pairs and an error clearly below the one of "no delay" or of the longest delay tried
        (a drive at constant speed says nothing)."""
        n = self._lag_n
        if n < 5:
            return {"lag_s": None, "speed_ratio": None, "n": n, "converged": False}
        k = min(range(len(self._lags)), key=lambda i: self._lag_sse[i])
        worst = max(self._lag_sse[0], self._lag_sse[-1])
        gain = 1.0 - self._lag_sse[k] / worst if worst > 0.0 else 0.0
        ratio = self._lag_sxy[k] / self._lag_sxx[k] if self._lag_sxx[k] > 0.0 else None
        return {"lag_s": self._lags[k], "speed_ratio": None if ratio is None else round(ratio, 3), "n": n,
                "converged": n >= self.c["lag_min_n"] and gain >= self.c["lag_min_gain"]}

    def set_tag(self, tag):
        """The source tag of the motion input: another car (another tag) starts from nothing."""
        if tag != self.tag:
            self.__init__(self.c)
            self.tag = tag

    # ---- outputs ----
    def yaw_scale(self):
        return self.gyro_gnss.value if self.gyro_gnss.converged else None

    def speed_scale(self):
        return self.speed.value if self.speed.converged else None

    def report(self):
        return {"yaw_rate": {"bias_rps": round(self.bias, 5) if self.bias_n else None, "bias_n": self.bias_n,
                             "scale": self.gyro_gnss.report("gnss_course"),
                             "scale_vs_wheels": self.gyro_wheels.report("wheel_speeds")},
                "wheel_yaw": {"scale": self.wheel_gnss.report("gnss_course")},
                "speed": {"scale": self.speed.report("gnss")},
                "gnss": self.gnss_lag()}

    def dump(self):
        return {"tag": self.tag, "gyro_gnss": self.gyro_gnss.dump(), "wheel_gnss": self.wheel_gnss.dump(),
                "gyro_wheels": self.gyro_wheels.dump(), "speed": self.speed.dump()}

    def load(self, d):
        if not isinstance(d, dict):
            return
        self.tag = d.get("tag")
        for k in ("gyro_gnss", "wheel_gnss", "gyro_wheels", "speed"):
            getattr(self, k).load(d.get(k))


def load_calibration(path, cal):
    try:
        with open(path) as f:
            cal.load(json.load(f))
    except (OSError, ValueError):
        pass


def save_calibration(path, cal):
    try:
        tmp = path + ".tmp"
        with open(tmp, "w") as f:
            json.dump(cal.dump(), f)
        os.replace(tmp, path)
        return True
    except OSError:
        return False


# ---- the store -----------------------------------------------------------------------------------------------

class VehicleStore:
    def __init__(self, cfg=None, legacy=None):
        """`cfg`: the "vehicle" object of mapd.json; `legacy`: mapd.json's older top-level `wheelbase_m`,
        `steering_ratio`, `track_m`."""
        self.sig = {}                   # signal -> [value, ts, quality, source tag, sent without a speed]
        self.desc = dict(DESCRIPTION_DEFAULTS)
        self.desc_src = {k: "default" for k in self.desc}
        for src in (legacy or {}), parse_description(cfg or {}):
            for k, v in src.items():
                if v is not None:
                    self.desc[k], self.desc_src[k] = v, "config"
        self.sensors = {}               # id -> {"ts", "n", "kind", "count", "hz", "msg"}
        self.seq = 0
        self.obj_seq = 0
        self.changed = False            # a discrete signal, the description or the set of sensors changed
        self.counts = {"controls": 0, "body": 0, "vehicle": 0, "objects": 0}

    # ---- input ----
    def update(self, group, values, ts, quality, default_q, tag, partial=False):
        """Store the signals of one message. Returns the ones roadsense may act on: {signal: value} of quality
        `good` / `uncalibrated`, with the quality next to it in a second dict."""
        usable, qual = {}, {}
        for k, v in values.items():
            q = quality.get(k, default_q)
            old = self.sig.get(k)
            if GROUPS[group][k][0] != "num" and k != "wheel_speeds_mps" and (old is None or old[0] != v or old[2] != q):
                self.changed = True
            if old is None:
                self.changed = True
            self.sig[k] = [v, ts, q, tag, partial]
            if q in USABLE:
                usable[k], qual[k] = v, q
        if group in self.counts:
            self.counts[group] += 1
        return usable, qual

    def describe(self, desc):
        for k, v in desc.items():
            if self.desc.get(k) != v or self.desc_src.get(k) != "feeder":
                self.changed = True
            self.desc[k], self.desc_src[k] = v, "feeder"
        self.counts["vehicle"] += 1

    def get(self, name, now_ns, usable=True):
        """The value of a signal when it is fresh (and, with `usable`, of a quality roadsense acts on)."""
        s = self.sig.get(name)
        if s is None or (now_ns - s[1]) / NS > HOLD_S[GROUP_OF[name]]:
            return None
        if s[2] == "invalid" or (usable and s[2] not in USABLE):
            return None
        return s[0]

    def sensor(self, sid):
        for s in self.desc.get("sensors") or []:
            if s["id"] == sid:
                return s
        return None

    def objects(self, msg, ts, tag):
        """One scan of a range sensor -> the `objects` message of the relay (positions in the camera frame when
        the mounts of the sensor and of the camera are known, else in the sensor's own frame)."""
        sid = msg.get("sensor")
        if not isinstance(sid, str) or not sid:
            raise ValueError("objects needs a sensor id (string)")
        sid = sid[:40]
        objs = parse_objects(msg)
        mount = self.sensor(sid)
        kind = msg.get("kind") or (mount or {}).get("kind") or "other"
        if kind not in SENSOR_KINDS:
            raise ValueError("kind must be one of %s" % " ".join(SENSOR_KINDS))
        tracked = msg.get("tracked")
        if tracked is not None and not isinstance(tracked, bool):
            raise ValueError("tracked must be true or false")
        cam = self.desc.get("camera")
        to_cam = mount is not None and "x_m" in mount and cam is not None and "x_m" in cam
        if to_cam:
            psi = math.radians(mount.get("yaw_deg", 0.0))
            cp, sp = math.cos(psi), math.sin(psi)
            ox, oy = mount["x_m"] - cam["x_m"], mount.get("y_m", 0.0) - cam.get("y_m", 0.0)
        out = []
        for o in objs:
            if "x_m" in o:
                xs, ys = o["x_m"], o.get("y_m", 0.0)
            else:
                az = math.radians(o.get("azimuth_deg", 0.0))
                xs, ys = o["range_m"] * math.cos(az), o["range_m"] * math.sin(az)
            d = o                                   # parse_objects made it: only known keys, validated
            vx, vy = o.get("vx_mps"), o.get("vy_mps")
            if to_cam:                             # sensor frame (y left) -> vehicle frame -> camera frame (y right)
                d["x_m"], d["y_m"] = round(ox + xs * cp - ys * sp, 2), round(-(oy + xs * sp + ys * cp), 2)
                if vx is not None:
                    vy0 = vy or 0.0
                    d["vx_mps"], d["vy_mps"] = round(vx * cp - vy0 * sp, 2), round(-(vx * sp + vy0 * cp), 2)
            else:
                d["x_m"], d["y_m"] = round(xs, 2), round(-ys, 2)
                if vx is not None:
                    d["vx_mps"], d["vy_mps"] = vx, -(vy or 0.0)
            if vx is None:
                d.pop("vy_mps", None)              # a lateral speed alone is not a velocity
            out.append(d)
        s = self.sensors.get(sid)
        if s is None:
            s = self.sensors[sid] = {"ts": None, "n": 0, "hz": None}
            self.changed = True
        if s["ts"] is not None and ts > s["ts"]:
            hz = NS / (ts - s["ts"])
            s["hz"] = hz if s["hz"] is None else s["hz"] + 0.1 * (hz - s["hz"])
        s.update(ts=ts, n=s["n"] + 1, kind=kind, count=len(out))
        self.counts["objects"] += 1
        self.obj_seq += 1
        return {"type": "objects", "seq": self.obj_seq, "ts_mono_ns": ts, "api": 1, "sensor": sid, "kind": kind,
                "frame": "camera" if to_cam else "sensor", "tracked": tracked, "source": tag, "objects": out}

    # ---- output ----
    def message(self, now_ns, real_ns, motion, calibration, feeders):
        """The `vehicle` message. `motion`: what the engine made of the motion input (signed speed, standstill,
        the yaw rate in use and where it comes from), or None without fresh motion."""
        self.seq += 1
        self.changed = False
        groups = {g: {} for g in GROUPS}
        signals = {}
        fresh = False
        for name, g in GROUP_OF.items():
            s = self.sig.get(name)
            val = None
            if s is not None:
                age = (now_ns - s[1]) / NS
                if age <= HOLD_S[g] and s[2] != "invalid":
                    val, fresh = s[0], True
                signals[name] = {"age_s": round(age, 2), "quality": s[2], "source": s[3]}
            groups[g][name] = val
        mo = groups["motion"]
        if motion is not None:
            mo.update(motion)
        else:
            mo.update(speed_mps=None, standstill=None, yaw_rate_rps=None, yaw_source=None)
        desc = dict(self.desc, sources=dict(self.desc_src))
        sensors = [{"id": sid, "kind": s.get("kind"), "age_s": round((now_ns - s["ts"]) / NS, 2),
                    "objects": s.get("count"), "hz": None if s["hz"] is None else round(s["hz"], 1), "scans": s["n"],
                    "mounted": self.sensor(sid) is not None}
                   for sid, s in sorted(self.sensors.items())]
        return {"type": "vehicle", "seq": self.seq, "ts_mono_ns": now_ns, "ts_real_ns": real_ns, "api": 1,
                "present": fresh, "motion": mo, "controls": groups["controls"], "body": groups["body"],
                "signals": signals, "sensors": sensors, "description": desc, "calibration": calibration,
                "feeders": feeders or []}
