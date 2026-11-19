"""Road users seen by a camera's class detector: where they are, from the box and the camera's calibration, and
what they are, as an annotation of the tracks (lane L42; non-critical).

The detector (roadsense-signs `signd --lights-road-users`: the traffic-light detector's own output, eight
BDD100K road classes) publishes boxes in the picture; it knows nothing about the car. This file turns a box
into a direction and, where the picture allows it, a distance, both with their uncertainty:

* bearing (left positive) from the box's horizontal centre through the camera's calibration (pitch, yaw, roll,
  intrinsics of the `calib` message); 1 sigma = a quarter of the box width (where on the object the tracked
  point is) plus 0.3 deg of calibration;
* ground position from the box's bottom edge on a flat road (the contact point), only when the box is not cut
  at the bottom of the detector's region (`clip` "b": the wide view ends at 62 % of the frame height, i.e. at
  about 14 m for this camera, the second view at about 24 m) and the bottom lies below the horizon by enough
  pixels to mean a distance. Range uncertainty from the pitch (0.6 deg), the bottom edge (3 px + 4 % of the box
  height) and the camera height (5 cm), propagated through d = f h / (v - v_horizon): it grows with the square
  of the distance (about 15 % at 20 m, 30 % at 40 m for f = 975 px, h = 1.22 m);
* no position from the box size: a vehicle's width depends on whether it is seen from behind or from the
  side, so a size-based range would be confidently wrong on every crossing car. A box without a ground point
  carries a bearing only.

The `road_users` messages (stream `road_users`, produced by the `road_users` plug-in from `lights_raw`) are read
by the `road_users` source kind: each road user is attached to at most one track (and each track gets at most
one road user per message) by bearing and, where both have one, distance within 3 sigma, and its class is
folded into that track's class vote (core/classvote.py). As with `object_classes`: nothing is started, moved,
confirmed or kept alive by it, the lead is not chosen by it, the sensor is no object source (`annotates_only`),
and the tracks, the lead and the ego state are the same, number for number, with and without it.
"""
import math

from .objects import Scan, SensorModel, X, Y, classify
from .registry import NS, Source, fin

# signd class names -> the contract's labels (core/classvote.py folds the same aliases)
LABELS = {"person": "human", "rider": "bike", "car": "car", "truck": "truck", "bus": "bus", "train": "tram",
          "motor": "motorbike", "bike": "bike"}

PROJ_CFG = dict(
    pitch_std_deg=0.6,           # calibration pitch uncertainty (online calibration's spread on the car: 0.3-0.6)
    bearing_cal_std_deg=0.3,     # calibration yaw uncertainty
    bottom_std_px=3.0, bottom_std_rel=0.04,    # where the box's bottom edge is: px + share of the box height
    height_std_m=0.05,
    min_below_px=4.0,            # a bottom edge closer to the horizon than this gives no distance
    max_range_m=80.0,
    clipped_weight=0.7,          # vote weight of a box cut by the detector's region
)


class Camera:
    """The pinhole camera of a `calib` message: back-projection of picture points into the car frame."""

    def __init__(self, calib=None, size=None):
        c = calib or {}
        w = float(c.get("width") or 1280)
        h = float(c.get("height") or 720)
        s = 1.0 if not size or not size[0] else float(size[0]) / w     # boxes are in the frame the detector saw
        self.f = float(c.get("focal_px") or 910.0) * s
        self.cx = float(c.get("cx") or w / 2.0) * s
        self.cy = float(c.get("cy") or h / 2.0) * s
        self.height = float(c.get("height_m") or 1.22)
        r, p, y = (float(c.get(k) or 0.0) for k in ("roll", "pitch", "yaw"))
        cr, sr, cp, sp, cy_, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
        # device <- calibrated car frame (x forward, y right, z down): Rz(yaw) Ry(pitch) Rx(roll), as roadsense
        R = [[cy_ * cp, cy_ * sp * sr - sy * cr, cy_ * sp * cr + sy * sr],
             [sy * cp, sy * sp * sr + cy_ * cr, sy * sp * cr - cy_ * sr],
             [-sp, cp * sr, cp * cr]]
        self.Rt = [[R[j][i] for j in range(3)] for i in range(3)]     # car <- device

    def ray(self, u, v):
        """Unit-free ray of a pixel in the car frame (x forward, y right, z down)."""
        d = (1.0, (u - self.cx) / self.f, (v - self.cy) / self.f)       # device frame: forward, right, down
        Rt = self.Rt
        return tuple(Rt[i][0] * d[0] + Rt[i][1] * d[1] + Rt[i][2] * d[2] for i in range(3))

    def bearing(self, u):
        """Direction of a picture column, rad, left positive (at the horizon row)."""
        r = self.ray(u, self.cy)
        return -math.atan2(r[1], r[0])

    def horizon_v(self, u):
        """Row of the horizon at column u (where the ground ray is parallel to the road)."""
        lo, hi = self.cy - 0.5 * self.f, self.cy + 0.5 * self.f
        for _ in range(30):                                            # z of the ray rises with v: bisection
            mid = 0.5 * (lo + hi)
            if self.ray(u, mid)[2] > 0.0:
                hi = mid
            else:
                lo = mid
        return 0.5 * (lo + hi)

    def ground(self, u, v):
        """(x forward, y right) of a picture point on a flat road, or None above the horizon."""
        r = self.ray(u, v)
        if r[2] <= 1e-6:
            return None
        k = self.height / r[2]
        return r[0] * k, r[1] * k


def project(obj, cam, cfg=PROJ_CFG):
    """A detector box -> {"bearing_deg", "bearing_std_deg", "range_m", "range_std_m", "x_m", "y_m" (left),
    "method": "ground" | None}; range and position None without a ground point."""
    x0, y0, x1, y1 = (float(v) for v in obj["box"])
    u = 0.5 * (x0 + x1)
    b = cam.bearing(u)
    half = math.atan2(0.5 * (x1 - x0), cam.f)
    out = {"bearing_deg": round(math.degrees(b), 2),
           "bearing_std_deg": round(math.degrees(0.5 * half) + cfg["bearing_cal_std_deg"], 2),
           "range_m": None, "range_std_m": None, "x_m": None, "y_m": None, "method": None}
    if "b" in (obj.get("clip") or ""):
        return out
    vh = cam.horizon_v(u)
    below = y1 - vh
    if below < cfg["min_below_px"]:
        return out
    g = cam.ground(u, y1)
    if g is None:
        return out
    gx, gy = g
    rng = math.hypot(gx, gy)
    if not rng < cfg["max_range_m"]:
        return out
    # d = f h / dv  ->  sigma_d = d / dv * sigma_v (+) d * sigma_h / h; sigma_v from the pitch and the bottom edge
    sv = math.hypot(cam.f * math.radians(cfg["pitch_std_deg"]), cfg["bottom_std_px"] + cfg["bottom_std_rel"] * (y1 - y0))
    sd = math.hypot(rng / below * sv, rng * cfg["height_std_m"] / cam.height)
    out.update(range_m=round(rng, 2), range_std_m=round(sd, 2), x_m=round(gx, 2), y_m=round(-gy, 2), method="ground")
    return out


class RoadUserModel(SensorModel):
    """Annotation only: a road user's class goes to the track it points at (one each way per message)."""
    name = "road_users"
    initiates = False
    annotates_only = True
    gate_sigma = 3.0
    min_bearing_gate = math.radians(1.5)
    track_halfwidth_m = 1.2      # a vehicle's half width: the bearing gate of a near track widens with it

    def process(self, layer, scan, ego):
        st = layer.stats[scan.src]
        pairs = []
        for j, d in enumerate(scan.dets):
            b, sb, r, sr = d["bearing"], d["bearing_std"], d["range"], d["range_std"]
            for tr in layer.tracks:
                x, y = float(tr.x[X]), float(tr.x[Y])
                if x <= 0.5:
                    continue
                rt = math.hypot(x, y)
                e = abs((math.atan2(y, x) - b + math.pi) % (2.0 * math.pi) - math.pi)
                gate = max(self.min_bearing_gate, self.gate_sigma * sb, math.atan2(self.track_halfwidth_m, rt))
                if e > gate:
                    continue
                cost = e / gate
                if r is not None:
                    srt = math.sqrt(max(float(tr.P[X, X]), 0.0) + sr * sr) if hasattr(tr, "P") else sr
                    dr = abs(rt - r) / max(srt, 0.5)
                    if dr > self.gate_sigma:
                        continue
                    cost += dr / self.gate_sigma
                pairs.append((cost, -d["size"], j, tr))
        pairs.sort(key=lambda p: (p[0], p[1]))
        used_d, used_t = set(), set()
        for cost, _, j, tr in pairs:
            if j in used_d or tr.id in used_t:
                continue
            used_d.add(j)
            used_t.add(tr.id)
            d = scan.dets[j]
            classify(tr, d["attrs"], scan.src, scan.t, d["weight"])
            st["assigned"] += 1


class RoadUserSource(Source):
    """`{"kind": "road_users", "id": "<id>"}`: reads the `road_users` stream (its messages' `sensor` must be the
    id, or the id is "*")."""
    rank = 0
    types = ("road_users",)
    stale_s = 2.0
    witness = True
    info = "road users of a camera's class detector (annotation of tracks; no measurement)"

    def __init__(self, sensor="*", **kw):
        self.id = "roadusers" if sensor == "*" else sensor
        self.sensor = sensor
        self.model = RoadUserModel()
        super().__init__(**kw)

    def scan(self, msg, ctx):
        if self.sensor != "*" and msg.get("sensor") != self.sensor:
            return None
        objs = msg.get("objects")
        if not isinstance(objs, list) or not isinstance(msg.get("ts_mono_ns"), int):
            return None
        dets = []
        for o in objs:
            if not isinstance(o, dict) or not isinstance(o.get("class"), str) or not fin(o.get("bearing_deg")):
                continue
            rng = o.get("range_m") if fin(o.get("range_m")) else None
            box = o.get("box") or (0, 0, 0, 0)
            dets.append({"bearing": math.radians(o["bearing_deg"]),
                         "bearing_std": math.radians(o["bearing_std_deg"]) if fin(o.get("bearing_std_deg")) else math.radians(1.0),
                         "range": rng, "range_std": float(o["range_std_m"]) if rng is not None and fin(o.get("range_std_m")) else None,
                         "size": float(box[3]) - float(box[1]) if len(box) == 4 else 0.0,
                         "weight": float(o.get("weight", 1.0)) if fin(o.get("weight")) else 1.0,
                         "attrs": {"class": o["class"], "class_conf": o.get("class_conf")}})
        ctx.mem["n"] = ctx.mem.get("n", 0) + 1
        ts = msg["ts_mono_ns"]
        return Scan(self.id, self.group, ts / NS, dets, self.model, ts)

    def report(self, ctx):
        return {"scans": ctx.mem.get("n", 0)}


def from_lights(msg, calib, sensor="cam", cfg=PROJ_CFG):
    """A `lights` message with `road_users` (signd --lights-road-users) -> a `road_users` message, or None when
    the message carries none (key absent: off or not decoded this frame). `"road_users": null` = shed by the load
    governor: the message says so (`shed`) with no objects."""
    if "road_users" not in msg:
        return None
    ru = msg.get("road_users")
    frame = msg.get("frame") if isinstance(msg.get("frame"), list) else None
    cam = Camera(calib, frame)
    out = {"type": "road_users", "sensor": sensor, "seq": msg.get("seq"), "ts_mono_ns": msg.get("ts_mono_ns"),
           "ts_real_ns": msg.get("ts_real_ns"), "view": msg.get("view"), "roi": msg.get("roi"),
           "shed": ru is None, "calib": "message" if calib else "nominal", "objects": []}
    for o in ru or ():
        if not isinstance(o, dict) or not isinstance(o.get("cls"), str) or not isinstance(o.get("box"), list) or len(o["box"]) != 4:
            continue
        if not all(fin(v) for v in o["box"]):
            continue
        label = LABELS.get(o["cls"], o["cls"])
        p = project(o, cam, cfg)
        clip = o.get("clip") or ""
        out["objects"].append(dict(p, **{"class": label, "class_conf": o.get("score"), "det_class": o["cls"],
                                         "rider": bool(o.get("rider")), "clip": clip, "box": o["box"],
                                         "weight": cfg["clipped_weight"] if clip else 1.0}))
    return out


def _factory(spec):
    return RoadUserSource(spec.get("id", "*"))


def register():
    """Make `{"kind": "road_users"}` known to the sources' configuration (`sources.build`)."""
    from ..sources import KINDS
    KINDS.setdefault("road_users", _factory)
