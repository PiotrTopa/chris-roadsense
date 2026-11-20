"""Fault injection into any message stream: the simulator's or a real recording's (docs/DESIGN.md section 5a).

A fault is (target, kind, window, parameters). The target names a source's quantity (`TARGETS`), the kind is one
of the taxonomy's fault classes (`KINDS`); `apply(messages, faults)` yields the stream with the faults in it.
Nothing here knows the integrator: the campaign (chris-roadsense tools/fault_campaign.py) runs the faulted stream through it and
scores what it reported against what was injected.

    for m in apply(msgs, [Fault("vehicle.speed", "scale", 60.0, 120.0, k=0.7)]): ...

Times are seconds since the stream's first stamp. Messages are never modified in place.
"""
import bisect
import copy
import math
import random

NS = 1e9
R_EARTH = 6378137.0

# target -> (message type, [field paths], group of the integrator that should be blamed, quantity affected)
# a path "a.b" is msg["a"]["b"]; "[*]" applies to every element of a list
TARGETS = {
    "vehicle.speed":      ("ego", ["speed_mps", "wheel_speeds_mps[*]"], "vehicle", "speed"),      # the whole speed chain
    "vehicle.speed_only": ("ego", ["speed_mps"], "vehicle", "speed"),                             # the speed signal, wheels healthy
    "vehicle.wheels":     ("ego", ["wheel_speeds_mps[*]"], "wheels", "speed"),
    "vehicle.gyro":       ("ego", ["yaw_rate_rps"], "vehicle", "yaw_rate"),
    "vehicle.accel":      ("ego", ["accel_long_mps2"], "vehicle", "accel"),
    "vehicle.feed":       ("ego", None, "vehicle", "speed"),
    "vision.speed":       ("road", ["pose.vx", "v_ego"], "vision", "speed"),
    "vision.yaw":         ("road", ["pose.yaw_rate"], "vision", "yaw_rate"),
    "vision.std":         ("road", ["pose.vx_std"], "vision", "speed"),
    "vision.feed":        ("road", None, "vision", "speed"),
    "vision.mount":       ("road", None, "vision", "mount"),
    "vision.lead":        ("road", ["leads[0].x[0]"], "vision", "lead"),
    "gnss.pos":           ("gnss", None, "gnss", "position"),
    "gnss.hacc":          ("gnss", ["hacc_m"], "gnss", "position"),
    "gnss.feed":          ("gnss", None, "gnss", "position"),
    "map.match":          ("map", None, "map", "position"),
    "radar.range":        ("objects", ["objects[*].range_m"], "radar_front", "lead"),
    "radar.rate":         ("objects", ["objects[*].range_rate_mps"], "radar_front", "speed"),
    "radar.mount":        ("objects", ["objects[*].azimuth_deg"], "radar_front", "lead"),
    "radar.feed":         ("objects", None, "radar_front", "lead"),
    "clock":              (None, None, None, "time"),
}

KINDS = ("bias", "scale", "drift", "stuck", "repeat", "dropout", "noise", "outliers", "delay", "stamp", "epoch",
         "stale", "sign", "unit", "frame", "mount", "ghost", "lie")


class Fault:
    """target: a key of TARGETS; kind: one of KINDS; t0, t1: window (s since the stream's start); params:
    bias b | scale k | drift rate (per s, of the value; `rel=True`: of the scale) | repeat n | noise sigma |
    outliers p, mag | delay d (the content is d s old, the stamp is fresh) | stamp d (the stamp is wrong by d) |
    epoch back (every stamp of the stream steps back by `back` s at t0) | stale n (silence over the window, then
    n samples that repeat the last value before it) | unit k | mount pitch (rad), collapse | ghost x | lie k."""

    def __init__(self, target, kind, t0, t1=1e9, **params):
        assert target in TARGETS and kind in KINDS, (target, kind)
        self.target, self.kind, self.t0, self.t1, self.p = target, kind, t0, t1, params
        self.mtype, self.paths, self.group, self.quantity = TARGETS[target]
        self.rng = random.Random(params.get("seed", 97))
        self.held = None
        self.hist = []
        self.count = 0
        self.stale_left = 0

    def name(self):
        return "%s %s%s" % (self.target, self.kind, "".join(" %s=%s" % kv for kv in sorted(self.p.items()) if kv[0] != "seed"))


def _walk(msg, path):
    """[(container, key)] the path addresses (several for [*])."""
    cur = [msg]
    parts = path.replace("[", ".[").split(".")
    for i, part in enumerate(parts):
        last = i == len(parts) - 1
        nxt = []
        for c in cur:
            if part == "[*]":
                if isinstance(c, list):
                    nxt += [(c, k) for k in range(len(c))] if last else list(c)
            elif part.startswith("["):
                k = int(part[1:-1])
                if isinstance(c, list) and len(c) > k:
                    nxt.append((c, k) if last else c[k])
            elif isinstance(c, dict) and part in c and c[part] is not None:
                nxt.append((c, part) if last else c[part])
        cur = nxt
    return cur


def _num(v):
    return isinstance(v, (int, float)) and not isinstance(v, bool)


def _shift_latlon(m, de, dn):
    if _num(m.get("lat")) and _num(m.get("lon")):
        m["lat"] = m["lat"] + math.degrees(dn / R_EARTH)
        m["lon"] = m["lon"] + math.degrees(de / (R_EARTH * math.cos(math.radians(m["lat"]))))


def _value(f, v, rel, key):
    k, p = f.kind, f.p
    if k == "bias":
        return v + p["b"]
    if k == "scale":
        return v * p["k"]
    if k == "unit":
        return v * p.get("k", 3.6)
    if k == "sign":
        return -v
    if k == "drift":
        return v * (1.0 + p["rate"] * (rel - f.t0)) if p.get("rel") else v + p["rate"] * (rel - f.t0)
    if k == "noise":
        return v + f.rng.gauss(0.0, p["sigma"])
    if k == "outliers":
        return v + (f.rng.choice((-1, 1)) * p["mag"] if f.rng.random() < p["p"] else 0.0)
    if k == "lie":
        return v * p["k"]
    if k in ("stuck", "repeat"):
        if f.held is None:
            f.held = {}
        if k == "repeat" and f.count % p.get("n", 5) == 0:
            f.held[key] = v
        return f.held.setdefault(key, v)
    return v


def _one(f, m, rel, out):
    """Apply fault f to message m (already a private copy); append what is delivered to `out`."""
    k, p = f.kind, f.p
    inside = f.t0 <= rel < f.t1
    if k == "epoch":                                        # every stamp steps back at t0 and stays there
        if rel >= f.t0:
            m["ts_mono_ns"] = int(m["ts_mono_ns"] - p.get("back", 600.0) * NS)
        return out.append(m)
    if m.get("type") != f.mtype or (f.target.startswith("radar") and m.get("kind") != "radar"):
        return out.append(m)
    if k == "stale":
        if inside:
            f.stale_left = p.get("n", 3)
            return None
        if f.stale_left > 0 and f.held is not None and rel >= f.t1:
            f.stale_left -= 1
            return out.append(dict(copy.deepcopy(f.held), ts_mono_ns=m["ts_mono_ns"], seq=m.get("seq")))
        if rel < f.t0:
            f.held = m
        return out.append(m)
    if k == "delay":                                        # content of d seconds ago under a fresh stamp
        f.hist.append((rel, m))
        if len(f.hist) > 4000:
            del f.hist[:2000]
        if not inside:
            return out.append(m)
        i = bisect.bisect_right([h[0] for h in f.hist], rel - p["d"]) - 1
        if i < 0:
            return None
        old = copy.deepcopy(f.hist[i][1])
        old["ts_mono_ns"] = m["ts_mono_ns"]
        if "seq" in m:
            old["seq"] = m["seq"]
        return out.append(old)
    if not inside:
        return out.append(m)
    f.count += 1
    if k == "dropout":
        return None
    if k == "stamp":
        m["ts_mono_ns"] = int(m["ts_mono_ns"] + p["d"] * NS)
        return out.append(m)
    if f.target == "gnss.pos":
        if k == "bias":
            _shift_latlon(m, p.get("e", 0.0), p.get("n", 0.0))
        elif k == "drift":
            _shift_latlon(m, p.get("e", 0.0) * (rel - f.t0), p.get("n", 0.0) * (rel - f.t0))
        elif k == "noise":
            _shift_latlon(m, f.rng.gauss(0, p["sigma"]), f.rng.gauss(0, p["sigma"]))
        elif k == "outliers" and f.rng.random() < p["p"]:
            a = f.rng.uniform(0, 2 * math.pi)
            _shift_latlon(m, p["mag"] * math.cos(a), p["mag"] * math.sin(a))
        elif k in ("stuck", "repeat"):
            if f.held is None or (k == "repeat" and f.count % p.get("n", 5) == 1):
                f.held = {q: m.get(q) for q in ("lat", "lon", "speed_mps", "course_deg", "alt_m")}
            m.update(f.held)
        elif k == "frame":                                  # latitude and longitude swapped about the first fix
            if f.held is None:
                f.held = (m["lat"], m["lon"])
            m["lat"], m["lon"] = f.held[0] + (m["lon"] - f.held[1]), f.held[1] + (m["lat"] - f.held[0])
        return out.append(m)
    if f.target == "map.match":
        mt = m.get("match")
        if isinstance(mt, dict) and k == "bias":            # the matcher sits on a parallel road
            b = math.radians(90.0 - (mt.get("bearing_deg") or 0.0))
            _shift_latlon(mt, -math.sin(b) * p["b"], math.cos(b) * p["b"])
        elif isinstance(mt, dict) and k == "frame":         # a crossing road: the bearing turned
            mt["bearing_deg"] = ((mt.get("bearing_deg") or 0.0) + p.get("deg", 90.0)) % 360.0
        return out.append(m)
    if f.target == "vision.mount":
        pose = m.get("pose")
        if isinstance(pose, dict) and _num(pose.get("vx")):
            dp = p["pitch"] if k == "mount" else p.get("rate", 0.0) * (rel - f.t0)
            if p.get("collapse"):
                # what the road model does far outside its range (measured on the reference car with the camera
                # tilted by about 16 degrees): the speed collapses, its stated std explodes, the direction of the
                # pose returns to "aligned", a phantom lead sits a few metres ahead
                v = pose["vx"]
                pose["vx"] = p.get("speed_factor", 0.15) * v + f.rng.gauss(0, 0.3 + 0.1 * abs(v))
                pose["vx_std"] = 0.5 + 0.75 * abs(pose["vx"])
                pose["vy"], pose["vz"] = f.rng.gauss(0, 0.02), f.rng.gauss(0, 0.02)
                m["v_ego"] = pose["vx"]
                if m.get("leads"):
                    m["leads"] = [{"prob": 0.93, "x": [round(5.2 + f.rng.gauss(0, 0.6), 2)], "y": [0.0], "v": [round(pose["vx"], 2)],
                                   "a": [0.0], "x_std": [1.0], "y_std": [0.3], "v_std": [0.5], "a_std": [1.0]}]
            elif _num(pose.get("vz")):
                pose["vz"] = pose["vz"] - pose["vx"] * dp
                for ld in m.get("leads") or ():             # the model places things by the horizon it believes
                    if ld.get("x"):
                        ld["x"] = [x * (1.0 + 6.0 * dp) for x in ld["x"]]
        return out.append(m)
    if k == "ghost":
        if f.mtype == "objects":
            objs = m.get("objects") or []
            m["objects"] = objs + [dict(o, range_m=2.0 * o["range_m"], range_rate_mps=2.0 * o.get("range_rate_mps", 0.0), synthetic="ghost")
                                   for o in objs[:2] if _num(o.get("range_m"))] + \
                [{"range_m": p.get("x", 25.0) + f.rng.gauss(0, 0.3), "azimuth_deg": f.rng.gauss(0, 0.3), "range_rate_mps": f.rng.gauss(0, 0.2),
                  "synthetic": "ghost"}]
        elif m.get("leads") is not None:
            m["leads"] = [{"prob": 0.9, "x": [p.get("x", 12.0) + f.rng.gauss(0, 0.4)], "y": [0.0], "v": [(m.get("pose") or {}).get("vx", 0.0)],
                           "a": [0.0], "x_std": [1.0], "y_std": [0.3], "v_std": [0.5], "a_std": [1.0]}]
        return out.append(m)
    if k == "frame" and f.target == "radar.mount":
        for c, key in _walk(m, "objects[*].azimuth_deg"):
            c[key] = -c[key]
        return out.append(m)
    for path in f.paths or ():
        for c, key in _walk(m, path):
            if _num(c[key]):
                c[key] = _value(f, c[key], rel, (path, key))
    if f.target in ("vehicle.speed", "vehicle.speed_only") and _num(m.get("speed_mps")):
        if m["speed_mps"] < 0 and k != "sign":
            m["speed_mps"] = 0.0
        if m.get("standstill") and m["speed_mps"] != 0.0:
            m["standstill"] = False
    return out.append(m)


def apply(messages, faults):
    """Yield `messages` (dicts in arrival order, or (arrival, dict) pairs) with the faults applied."""
    faults = list(faults)
    t_first = None
    for item in messages:
        pair = isinstance(item, tuple)
        m = item[1] if pair else item
        ts = m.get("ts_mono_ns") if isinstance(m, dict) else None
        if not isinstance(ts, int) or not faults:
            yield item
            continue
        if t_first is None:
            t_first = ts / NS
        rel = ts / NS - t_first
        cur = [m]
        for f in faults:
            if f.mtype is not None and m.get("type") != f.mtype:
                continue
            nxt = []
            for c in cur:
                _one(f, copy.deepcopy(c), rel, nxt)
            cur = nxt
        for c in cur:
            yield (item[0], c) if pair else c
