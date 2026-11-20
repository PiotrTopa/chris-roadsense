"""The vehicle input store: what the car's host tells the integrator (vehicle input API 1.1, docs/VEHICLE_INPUT.md).

Ported from the legacy mapd's `vehicle.py` / `engine.py` (lane L43-V): the validation of every feeder message with
the same error reasons, the latest value of every signal with its time, quality and source tag, the car's static
description, the range sensors, and the `vehicle` topic. What mapd did with the input beyond that (its position
filter and its calibration estimators) lives in the estimator now: nothing of it is repeated here. The store turns
accepted messages into the inputs the estimator's sources read:

    motion    -> `ego`      signed speed, gear, standstill, the yaw rate in use and where it comes from, wheels,
                            steering, accelerations: only signals of quality `good` / `uncalibrated`
    gnss      -> `gnss`     the fix as it came (known fields only), stamped
    controls, body, vehicle -> `vehicle` (the topic: every signal with age, quality and source)
    objects   -> `objects`  the scan validated, in the sensor's frame, with the sensor's mount in the car frame
    reset     -> `reset`    routed to the stream `position_reset` (the estimator and the map box forget the position)

No I/O and no wall clock in here: times are CLOCK_MONOTONIC ns of this host, handed in by the caller.

Frames. Input uses the vehicle frame of ISO 8855: x forward, y to the left, z up, origin on the ground under the
centre of the rear axle; angles and yaw rates positive to the left. The estimator's car frame has its origin at the
reference camera, x forward, y to the left: a sensor's mount is given relative to the camera's.
"""
import math

NS = 1e9
API_VERSION, API_MINOR = 1, 1
QUALITIES = ("good", "uncalibrated", "unverified", "invalid")
USABLE = ("good", "uncalibrated")      # acted on; `unverified` is reported only, `invalid` is absent
TURN_SIGNALS = ("off", "left", "right", "both")
GEARS = ("P", "R", "N", "D", "B")
CRUISE = ("off", "standby", "active")
WIPERS = ("off", "intermittent", "low", "high")
FIXES = ("none", "2d", "3d", "dgps", "rtk", "dr")
SENSOR_KINDS = ("radar", "lidar", "ultrasonic", "camera", "other")
OBJECT_CLASSES = ("car", "truck", "motorcycle", "bicycle", "pedestrian", "unknown")
MAX_OBJECTS = 256
TYPES = ("gnss", "motion", "reset", "controls", "body", "vehicle", "objects")

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
# A signal older than this is reported as null (unknown).
HOLD_S = {"motion": 1.5, "controls": 3.0, "body": 15.0}
SIGNALS = {"motion": sorted(MOTION) + ["turn_signal"], "controls": sorted(CONTROLS), "body": sorted(BODY)}

DESCRIPTION_NUM = {
    "wheelbase_m": (1.0, 8.0), "track_m": (0.8, 3.0), "track_front_m": (0.8, 3.0), "steering_ratio": (5.0, 40.0),
    "length_m": (1.0, 25.0), "width_m": (0.5, 3.5), "height_m": (0.5, 5.0),
    "rear_axle_to_front_m": (0.5, 20.0), "wheel_speed_min_mps": (0.0, 3.0),
}
# Nominal values of a compact car: they matter only for a feeder that sends neither a yaw rate nor a description
# (`description.sources` says "default" for them).
DESCRIPTION_DEFAULTS = {"wheelbase_m": 2.70, "track_m": 1.48, "steering_ratio": 19.1}
MOUNT_NUM = {"x_m": (-10.0, 25.0), "y_m": (-3.0, 3.0), "z_m": (0.0, 5.0)}
SENSOR_NUM = dict(MOUNT_NUM, yaw_deg=(-180.0, 180.0), fov_deg=(0.0, 360.0), max_range_m=(0.0, 1000.0))
OBJECT_NUM = {
    "range_m": (0.0, 1000.0), "azimuth_deg": (-180.0, 180.0), "x_m": (-1000.0, 1000.0), "y_m": (-1000.0, 1000.0),
    "range_rate_mps": (-150.0, 150.0), "vx_mps": (-150.0, 150.0), "vy_mps": (-150.0, 150.0),
    "rcs_dbsm": (-100.0, 100.0), "prob": (0.0, 1.0), "width_m": (0.0, 30.0), "length_m": (0.0, 50.0),
}
GNSS_KEYS = ("fix", "lat", "lon", "hacc_m", "speed_mps", "speed_acc_mps", "course_deg", "course_acc_deg", "alt_m",
             "vacc_m", "sats", "hdop", "utc_ms", "age_ms", "source")

DEFAULTS = dict(
    max_input_age_s=5.0,        # older stamps are rejected (None: not checked, e.g. a replay on another clock)
    max_input_future_s=0.5,     # stamps further in the future are rejected (None: not checked)
    max_fix_hacc_m=100.0,       # worse fixes are accepted and not passed on (docs: "fixes above 100 m are ignored")
    pair_s=0.3,                 # a yaw / steering / acceleration of a message without a speed joins one this close
    motion_timeout_s=1.5,       # the `vehicle` topic's derived motion is null after this
    period_s=0.5,               # the `vehicle` topic at least this often while something feeds or ticks
    change_s=0.05,              # ... and at once on a change of a switch-like signal, at most this often
    vehicle=None,               # the description's defaults for this car (keys of the `vehicle` message)
)


def _fin(v):
    return isinstance(v, (int, float)) and not isinstance(v, bool) and math.isfinite(v)


def _number(name, v, lo, hi):
    if not _fin(v):
        raise ValueError("%s must be a finite number" % name)
    if v < lo or v > hi:
        raise ValueError("%s out of range" % name)
    return float(v)


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


def parse_signals(group, msg):
    """The signals of one `motion` / `controls` / `body` message: {signal: value}. A signal that is absent or null
    is left out; a malformed one raises ValueError. Unknown fields are ignored."""
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
    """A `vehicle` message (or the configured defaults): the keys it sets, validated."""
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
    """The object list of an `objects` message, validated: polar (`range_m`, `azimuth_deg`) or Cartesian (`x_m`,
    `y_m`) in the sensor's frame, at least one of the two per object. Only known keys are kept."""
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


def is_synthetic(tag):
    """A `source` tag that marks made-up input: "synthetic", or anything starting with it ("synthetic-drive")."""
    return isinstance(tag, str) and tag.strip().lower().startswith("synthetic")


def hello(now_ns=None):
    """The reply to a feeder's hello (and the greeting on connect): vehicle input API 1.1."""
    h = {"type": "hello", "api": API_VERSION, "minor": API_MINOR, "server": "chris-integrator", "accepts": list(TYPES),
         "signals": {k: list(v) for k, v in SIGNALS.items()}, "clock": "CLOCK_MONOTONIC"}
    if now_ns is not None:
        h["now_mono_ns"] = now_ns
    return h


def calibration_report(cal):
    """The estimator's calibration (`ego_state.calibration`) in the shape of the vehicle input API's
    `vehicle.calibration` (section 9): `value` is the factor a fed signal is multiplied by to be true. The estimator
    learns states, not sample counts: `n` is null, `converged` is the state's `known`, `ref` is "estimator". What
    the estimator does not estimate is null (`yaw_rate.bias_n`, `yaw_rate.scale_vs_wheels`, `gnss.speed_ratio`)."""
    cal = cal or {}
    veh, gn = cal.get("vehicle") or {}, cal.get("gnss") or {}

    def factor(st):                                 # signal = (1 + s) x true  ->  true = signal / (1 + s)
        if not isinstance(st, dict) or not _fin(st.get("value")):
            return {"value": None, "std": None, "n": None, "converged": False, "ref": "estimator"}
        k = 1.0 + st["value"]
        sd = st.get("std")
        return {"value": round(1.0 / k, 4) if k > 0 else None,
                "std": round(sd / (k * k), 4) if _fin(sd) and k > 0 else None,
                "n": None, "converged": bool(st.get("known")), "ref": "estimator"}
    bg = veh.get("b_g") if isinstance(veh.get("b_g"), dict) else {}
    tau = gn.get("tau") if isinstance(gn.get("tau"), dict) else {}
    return {"source": "estimator" if cal else None,
            "yaw_rate": {"bias_rps": bg.get("value") if bg.get("known") else None, "bias_std_rps": bg.get("std"),
                         "bias_n": None, "scale": factor(veh.get("s_g")),
                         "scale_vs_wheels": None},
            "wheel_yaw": {"scale": factor(veh.get("s_wy"))},
            "speed": {"scale": factor((cal.get("odometry") or {}).get("speed_scale"))},
            "gnss": {"lag_s": tau.get("value"), "lag_std_s": tau.get("std"), "speed_ratio": None, "n": None,
                     "converged": bool(tau.get("known"))}}


class VehicleStore:
    """The latest value of every signal with its time, quality and source tag; the description (feeder > configured
    defaults > nominal); the range sensors seen."""

    def __init__(self, cfg=None):
        self.sig = {}                   # signal -> [value, ts, quality, source tag, sent without a speed]
        self.desc = dict(DESCRIPTION_DEFAULTS)
        self.desc_src = {k: "default" for k in self.desc}
        for k, v in parse_description(cfg or {}).items():
            self.desc[k], self.desc_src[k] = v, "config"
        self.sensors = {}               # id -> {"ts", "n", "kind", "count", "hz"}
        self.seq = 0
        self.obj_seq = 0
        self.changed = False            # a discrete signal, the description, the sensors or the feeders changed

    def update(self, group, values, ts, quality, default_q, tag, partial=False):
        for k, v in values.items():
            q = quality.get(k, default_q)
            old = self.sig.get(k)
            if old is None or (GROUPS[group][k][0] not in ("num", "wheels") and (old[0] != v or old[2] != q)):
                self.changed = True
            self.sig[k] = [v, ts, q, tag, partial]

    def describe(self, desc):
        for k, v in desc.items():
            if self.desc.get(k) != v or self.desc_src.get(k) != "feeder":
                self.changed = True
            self.desc[k], self.desc_src[k] = v, "feeder"

    def get(self, name, now_ns, usable=True):
        """The value of a signal when it is fresh (and, with `usable`, of a quality that is acted on)."""
        s = self.sig.get(name)
        if s is None or (now_ns - s[1]) / NS > HOLD_S[GROUP_OF[name]]:
            return None
        if s[2] == "invalid" or (usable and s[2] not in USABLE):
            return None
        return s[0]

    def recent(self, name, ts, max_age_s):
        """A usable signal for the motion message of time ts: the message's own, or one that a message without a
        speed delivered within max_age_s. What an earlier complete message carried is never reused."""
        s = self.sig.get(name)
        if s is None or s[2] not in USABLE:
            return None
        if s[1] != ts and not (s[4] and abs(ts - s[1]) <= max_age_s * NS):
            return None
        return s[0]

    def sensor(self, sid):
        for s in self.desc.get("sensors") or []:
            if s["id"] == sid:
                return s
        return None

    def objects(self, msg, ts, tag):
        """One scan of a range sensor -> the `objects` message: the objects as validated, in the sensor's frame
        (x along the boresight, y to the left), with the sensor's `mount` in the estimator's car frame (relative
        to the camera, y left) when the description places both the sensor and the camera."""
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
        s = self.sensors.get(sid)
        if s is None:
            s = self.sensors[sid] = {"ts": None, "n": 0, "hz": None}
            self.changed = True
        if s["ts"] is not None and ts > s["ts"]:
            hz = NS / (ts - s["ts"])
            s["hz"] = hz if s["hz"] is None else s["hz"] + 0.1 * (hz - s["hz"])
        s.update(ts=ts, n=s["n"] + 1, kind=kind, count=len(objs))
        self.obj_seq += 1
        out = {"type": "objects", "seq": self.obj_seq, "ts_mono_ns": ts, "api": API_VERSION, "sensor": sid, "kind": kind,
               "frame": "sensor", "tracked": tracked, "source": tag, "mounted": mount is not None, "objects": objs}
        if mount is not None and "x_m" in mount and cam is not None and "x_m" in cam:
            out["mount"] = {"x_m": round(mount["x_m"] - cam["x_m"], 3),
                            "y_m": round(mount.get("y_m", 0.0) - cam.get("y_m", 0.0), 3),
                            "z_m": round(mount.get("z_m", 0.0) - cam.get("z_m", 0.0), 3),
                            "yaw_deg": mount.get("yaw_deg", 0.0)}
        return out

    def message(self, now_ns, real_ns, motion, calibration, feeders, extra=None):
        """The `vehicle` topic. `motion`: the derived motion of the last `ego` handed to the estimator (signed speed,
        standstill, gear, reverse, the yaw rate in use and its source) or None without fresh motion."""
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
        out = {"type": "vehicle", "seq": self.seq, "ts_mono_ns": now_ns, "ts_real_ns": real_ns, "api": API_VERSION,
               "present": fresh, "motion": mo, "controls": groups["controls"], "body": groups["body"],
               "signals": signals, "sensors": sensors, "description": desc, "calibration": calibration,
               "feeders": feeders or []}
        if extra:
            out.update(extra)
        return out


class VehicleInput:
    """Feeder messages in, estimator inputs and the `vehicle` topic out.

        vi = VehicleInput(cfg, calibration=lambda: bus_latest_ego_state_calibration)
        outs, err = vi.on_message(msg, now_ns)      # err: None, or the reason of the error reply
        msg = vi.tick(now_ns)                       # the periodic `vehicle` topic, or None when not due

    `outs` are messages for the stack (`ego`, `gnss`, `vehicle`, `objects`, `reset`). A rejected message changes
    nothing and produces nothing."""

    def __init__(self, cfg=None, calibration=None):
        self.cfg = dict(DEFAULTS, **(cfg or {}))
        self.store = VehicleStore(self.cfg.get("vehicle"))
        self.calibration = calibration          # callable -> the estimator's `ego_state.calibration`, or None
        self.ego = None                         # the last `ego` built
        self.seq = 0
        self.gnss_seq = 0
        self.counts = {t: 0 for t in TYPES}
        self.counts.update(errors=0, gnss_ignored=0)
        self.last_error = None
        self.resets = 0
        self.last_reset = None                  # {"ts_mono_ns", "reason", "auto"}
        self._fed_synth = None                  # the receiver that fed the position: True synthetic, False real
        self.feeders = {}                       # connection key -> {"name", "provides", "since_ns", "messages"}
        self._vehicle_t = None                  # ts of the last `vehicle` topic message

    # ---- feeders (connections) ---------------------------------------------------------------------------
    def connect(self, key, now_ns):
        self.feeders[key] = {"name": None, "provides": None, "since_ns": now_ns, "messages": 0}
        self.store.changed = True

    def hello(self, key, msg, now_ns):
        """A feeder's hello: its name and what it will send (shown in `feeders`). Returns the reply."""
        f = self.feeders.get(key)
        if f is not None:
            if isinstance(msg.get("name"), str):
                f["name"] = msg["name"][:60]
            prov = msg.get("provides")
            if isinstance(prov, dict) and len(repr(prov)) <= 4096:
                f["provides"] = prov
            self.store.changed = True
        return hello(now_ns)

    def disconnect(self, key):
        if self.feeders.pop(key, None) is not None:
            self.store.changed = True

    # ---- input -------------------------------------------------------------------------------------------
    def on_message(self, msg, now_ns, real_ns=None, key=None):
        """One message of the vehicle input API: (messages for the stack, None) or ([], reason)."""
        try:
            outs = self._input(msg, now_ns, real_ns)
        except ValueError as e:
            self.counts["errors"] += 1
            self.last_error = str(e)
            return [], str(e)
        if key is not None and key in self.feeders:
            self.feeders[key]["messages"] += 1
        v = self._vehicle_due(now_ns, real_ns)
        if v is not None:
            outs.append(v)
        return outs, None

    def tick(self, now_ns, real_ns=None):
        """The `vehicle` topic when it is due (every `period_s`, sooner on a change), else None."""
        return self._vehicle_due(now_ns, real_ns)

    def _stamp(self, msg, now_ns):
        c = self.cfg
        ts, age = msg.get("ts_mono_ns"), msg.get("age_ms")
        if ts is None and age is not None:
            # the sample's age at the moment of sending: a feeder relaying from another device knows the age but
            # shares no clock with this host
            if isinstance(age, bool) or not isinstance(age, (int, float)) or not math.isfinite(age) or age < 0:
                raise ValueError("age_ms must be a number >= 0")
            if c["max_input_age_s"] is not None and age > c["max_input_age_s"] * 1e3:
                raise ValueError("age_ms: the sample is %.1f s old" % (age / 1e3))
            return now_ns - int(age * 1e6)
        if ts is None:
            return now_ns
        if isinstance(ts, bool) or not isinstance(ts, int):
            raise ValueError("ts_mono_ns must be an integer")
        if c["max_input_age_s"] is not None and ts < now_ns - c["max_input_age_s"] * NS:
            raise ValueError("ts_mono_ns is %.1f s old (CLOCK_MONOTONIC of this host expected)" % ((now_ns - ts) / NS))
        if c["max_input_future_s"] is not None and ts > now_ns + c["max_input_future_s"] * NS:
            raise ValueError("ts_mono_ns is in the future (CLOCK_MONOTONIC of this host expected)")
        return ts

    def _input(self, msg, now_ns, real_ns):
        if not isinstance(msg, dict):
            raise ValueError("not a JSON object")
        t = msg.get("type")
        if t == "hello":
            return []
        if t not in TYPES:
            raise ValueError("unknown type %r" % (t,))
        ts = self._stamp(msg, now_ns)
        if t == "reset":
            reason = msg.get("reason")
            if reason is not None and not isinstance(reason, str):
                raise ValueError("reason must be a string")
            out = [self._reset(ts, (reason or "requested")[:80])]
        elif t == "gnss":
            out = self._gnss(msg, ts, now_ns, real_ns)
        elif t == "motion":
            out = self._motion(msg, ts)
        elif t == "vehicle":
            self.store.describe(parse_description(msg))
            out = []
        elif t == "objects":
            out = [self.store.objects(msg, ts, msg.get("source"))]
        else:
            sig = parse_signals(t, msg)
            if not sig:
                raise ValueError("no known signal in the %s message" % t)
            q, dq = parse_quality(msg)
            self.store.update(t, sig, ts, q, dq, msg.get("source"))
            out = []
        self.counts[t] += 1
        if real_ns is not None:
            for m in out:
                m.setdefault("ts_real_ns", real_ns - (now_ns - m["ts_mono_ns"]))
        return out

    def _reset(self, ts, reason, auto=False):
        self.seq += 1
        self.resets += 1
        self.last_reset = {"ts_mono_ns": ts, "reason": reason, "auto": auto}
        self._fed_synth = None
        self.store.changed = True
        return {"type": "reset", "seq": self.seq, "ts_mono_ns": ts, "reason": reason, "auto": auto}

    def _gnss(self, g, ts, now_ns, real_ns):
        fix = g.get("fix", "3d")
        if fix not in FIXES:
            raise ValueError("fix must be one of %s" % " ".join(FIXES))
        if fix != "none":
            _num(g, "lat", -90.0, 90.0, required=True)
            _num(g, "lon", -180.0, 180.0, required=True)
            hacc = _num(g, "hacc_m", 0.0, 1e6)
            _num(g, "speed_mps", 0.0, 120.0)
            _num(g, "speed_acc_mps", 0.0, 100.0)
            _num(g, "course_deg", 0.0, 360.0)
            _num(g, "course_acc_deg", 0.0, 360.0)
        else:
            hacc = None
        out = []
        synth = is_synthetic(g.get("source"))
        if self._fed_synth is not None and synth != self._fed_synth:
            # a synthetic session ended and the real receiver speaks (or the other way round): the position of the
            # one must not live on as the position of the other (as mapd did)
            out.append(self._reset(ts, "gnss source changed from %s to %s" % (
                ("real", "synthetic") if synth else ("synthetic", "real")), auto=True))
        if fix != "none":
            self._fed_synth = synth
        if hacc is not None and hacc > self.cfg["max_fix_hacc_m"]:
            self.counts["gnss_ignored"] += 1
            return out
        self.gnss_seq += 1
        m = {k: g[k] for k in GNSS_KEYS if k in g}
        m.update(type="gnss", fix=fix, seq=self.gnss_seq, ts_mono_ns=ts, arrival_mono_ns=now_ns)
        out.append(m)
        return out

    def _motion(self, m, ts):
        c = self.cfg
        st = self.store
        tag = m.get("source")
        sig = parse_signals("motion", m)
        turn = m.get("turn_signal")                  # its place is `controls`; accepted here since v1
        if turn is not None and turn not in TURN_SIGNALS:
            raise ValueError("turn_signal must be one of off left right both")
        if not sig and turn is None:
            raise ValueError("missing field speed_mps")
        q, dq = parse_quality(m)
        st.update("motion", sig, ts, q, dq, tag, partial="speed_mps" not in sig)
        if turn is not None:
            st.update("controls", {"turn_signal": turn}, ts, q, dq, tag)
        v = st.recent("speed_mps", ts, 0.0) if "speed_mps" in sig else None
        if v is None:
            return []                                # stored; the estimator gets messages that carry a speed
        gear = st.recent("gear", ts, HOLD_S["motion"])
        rev = st.recent("reverse", ts, HOLD_S["motion"])
        yr = st.recent("yaw_rate_rps", ts, c["pair_s"])
        ws = st.recent("wheel_speeds_mps", ts, c["pair_s"])
        sw = st.recent("steering_wheel_deg", ts, c["pair_s"])
        reverse = bool(rev) or gear == "R"
        d = st.desc
        standstill = v == 0.0 and (ws is None or not any(ws))
        vs = -v if reverse else v
        if yr is not None:
            # the sensor's reading as it came: the estimator learns its bias (at standstill) and scale itself
            yaw, ysrc = yr, "yaw_rate"
        elif ws is not None:
            yaw, ysrc = (ws[3] - ws[2]) / d["track_m"] * (-1.0 if reverse else 1.0), "wheel_speeds"
        elif sw is not None:
            yaw, ysrc = vs * math.tan(math.radians(sw / d["steering_ratio"])) / d["wheelbase_m"], "steering"
        else:
            yaw, ysrc = None, None
        if standstill and yaw is not None and ysrc != "yaw_rate":
            yaw = 0.0                                # a standing car does not turn
        if turn is None:
            turn = st.get("turn_signal", ts)
        self.seq += 1
        ego = {"type": "ego", "seq": self.seq, "ts_mono_ns": ts, "api": API_VERSION, "speed_mps": vs, "gear": gear,
               "reverse": reverse, "standstill": standstill, "wheel_speeds_mps": ws,
               "yaw_rate_rps": None if yaw is None else round(yaw, 5), "yaw_source": ysrc, "turn_signal": turn,
               "source": "vehicle", "tag": tag}
        for k in ("steering_wheel_deg", "accel_long_mps2", "accel_lat_mps2"):
            val = st.recent(k, ts, c["pair_s"])
            if val is not None:
                ego[k] = val
        self.ego = ego
        return [ego]

    # ---- output ------------------------------------------------------------------------------------------
    def _vehicle_due(self, now_ns, real_ns):
        c, last = self.cfg, self._vehicle_t
        if last is not None and now_ns >= last:
            dt = (now_ns - last) / NS
            if dt < c["change_s"] or (dt < c["period_s"] and not self.store.changed):
                return None
        return self.vehicle_message(now_ns, real_ns)

    def vehicle_message(self, now_ns, real_ns=None):
        """The `vehicle` topic now."""
        self._vehicle_t = now_ns
        ego = self.ego
        motion = None
        if ego is not None and now_ns - ego["ts_mono_ns"] <= self.cfg["motion_timeout_s"] * NS:
            motion = {k: ego[k] for k in ("speed_mps", "standstill", "gear", "reverse", "yaw_rate_rps", "yaw_source")}
        cal = None
        if self.calibration is not None:
            try:
                cal = self.calibration()
            except Exception:                        # a report must never stop the input
                cal = None
        feeders = [{"name": f["name"], "provides": f["provides"], "connected_s": round((now_ns - f["since_ns"]) / NS, 1),
                    "messages": f["messages"]} for f in self.feeders.values()]
        extra = {"input": {"counts": dict(self.counts), "last_error": self.last_error, "resets": self.resets,
                           "last_reset": self.last_reset}}
        return self.store.message(now_ns, real_ns, motion, calibration_report(cal), feeders, extra)
