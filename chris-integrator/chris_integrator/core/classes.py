"""Road-user classes as an input of their own: a detector that says *what* an object is, without measuring it.

    {"type": "objects", "kind": "classes", "sensor": "<id>", "ts_mono_ns": ...,
     "objects": [{"class": "truck", "class_conf": 0.8, "x_m": 31.0, "y_m": -0.4}            position in the car frame
                 {"class": "human", "class_conf": 0.6, "bearing_deg": 12.0, "range_m": 18}  or a bearing (left
                 ...]}                                                                       positive), range optional

registered in the configuration as `{"kind": "object_classes", "id": "<id>"}`. Each object is attached to the
track it points at (nearest in bearing inside a gate; in range too when a range is given) and folded into that
track's class attribute (`objects.classify`). Nothing else happens: no track is started, moved, confirmed or
kept alive by it, the lead is not chosen by it, and the sensor does not count as an object source. A detector
that also measures positions well enough to track by is a `camera_objects` source instead; its `class` field
feeds the same attribute.
"""
import math

from .objects import Scan, SensorModel, X, Y, classify
from .registry import NS, Source, fin


class ClassModel(SensorModel):
    name = "classes"
    initiates = False
    annotates_only = True            # not an object source: it carries no measurement
    bearing_gate = math.radians(2.0)
    lateral_gate_m = 1.5
    range_gate = (3.0, 0.3)          # m + share of the range

    def process(self, layer, scan, ego):
        st = layer.stats[scan.src]
        for d in scan.dets:
            b, r = d["bearing"], d["range"]
            best, best_e = None, None
            for tr in layer.tracks:
                x, y = float(tr.x[X]), float(tr.x[Y])
                if x <= 0.5:
                    continue
                rt = math.hypot(x, y)
                e = abs((math.atan2(y, x) - b + math.pi) % (2.0 * math.pi) - math.pi)
                if e > max(self.bearing_gate, math.atan2(self.lateral_gate_m, rt)):
                    continue
                if r is not None and abs(rt - r) > self.range_gate[0] + self.range_gate[1] * r:
                    continue
                key = (e, rt)
                if best is None or key < best_e:
                    best, best_e = tr, key
            if best is not None:
                classify(best, d["attrs"], scan.src, scan.t)
                st["assigned"] += 1


class ClassSource(Source):
    rank = 0
    types = ("objects",)
    stale_s = 2.0
    witness = True                   # never fused, never starts a fault
    info = "road-user classes (annotation of tracks; no measurement)"

    def __init__(self, sensor, **kw):
        self.id = sensor
        self.sensor = sensor
        self.model = ClassModel()
        super().__init__(**kw)

    def scan(self, msg, ctx):
        if msg.get("sensor") != self.sensor or msg.get("kind") != "classes" or not isinstance(msg.get("objects"), list):
            return None
        dets = []
        for o in msg["objects"]:
            if not isinstance(o, dict) or not isinstance(o.get("class"), str):
                continue
            if fin(o.get("x_m")) and fin(o.get("y_m")) and o["x_m"] > 0.5:
                b, r = math.atan2(o["y_m"], o["x_m"]), math.hypot(o["x_m"], o["y_m"])
            elif fin(o.get("bearing_deg")):
                b, r = math.radians(o["bearing_deg"]), (float(o["range_m"]) if fin(o.get("range_m")) else None)
            else:
                continue
            dets.append({"bearing": b, "range": r, "attrs": {k: o[k] for k in ("class", "class_conf", "prob") if k in o}})
        ctx.mem["n"] = ctx.mem.get("n", 0) + 1
        ts = msg["ts_mono_ns"]
        return Scan(self.id, self.group, ts / NS, dets, self.model, ts)

    def report(self, ctx):
        return {"scans": ctx.mem.get("n", 0)}


def _factory(spec):
    return ClassSource(spec["id"])


def register():
    """Make `{"kind": "object_classes"}` known to the sources' configuration (`sources.build`)."""
    from ..sources import KINDS
    KINDS.setdefault("object_classes", _factory)
