"""A simulated second camera with its own object detector: per frame the vehicles in its field of view in its
own frame (good bearing, poor range), missed detections, a late stamp. Emitted as `objects` messages of kind
"camera"."""
import math
import random

from .world import NS


class CameraObjectsSim:
    def __init__(self, sensor="cam_wide", mount=(0.0, 0.0, 0.0), seed=61, hz=10.0, latency=0.12, age=0.10, fov_deg=60.0,
                 max_range=70.0, range_noise=(0.3, 0.05), cross_noise=(0.08, 0.005), p_d=0.88, false_rate=0.02, off=()):
        self.__dict__.update(locals())

    def generate(self, w):
        rng = random.Random(self.seed)
        mx, my, yaw = self.mount
        c, s = math.cos(yaw), math.sin(yaw)
        t = w.t0 + 0.09
        while t < w.t0 + w.T:
            rel = t - w.t0
            if not any(a <= rel < b for a, b in self.off):
                objs = []
                for o in w.targets_at(t - self.age):
                    dx, dy = o["x"] - mx, o["y"] - my
                    x, y = c * dx + s * dy, -s * dx + c * dy
                    r = math.hypot(x, y)
                    if r < 2.0 or r > self.max_range or abs(math.atan2(y, x)) > math.radians(self.fov_deg) or rng.random() > self.p_d:
                        continue
                    er = rng.gauss(0, self.range_noise[0] + self.range_noise[1] * r)
                    ec = rng.gauss(0, self.cross_noise[0] + self.cross_noise[1] * r)
                    ux, uy = x / r, y / r
                    objs.append({"x_m": round(x + er * ux - ec * uy, 2), "y_m": round(y + er * uy + ec * ux, 2), "class": "car",
                                 "prob": 0.9, "truth": o["id"]})
                if rng.random() < self.false_rate:
                    objs.append({"x_m": round(rng.uniform(5, self.max_range), 2), "y_m": round(rng.uniform(-8, 8), 2), "class": "unknown", "prob": 0.4})
                yield t + self.latency, {"type": "objects", "sensor": self.sensor, "kind": "camera", "tracked": False,
                                         "ts_mono_ns": int(round(t * NS)), "objects": objs}
            t += 1.0 / self.hz + rng.uniform(-0.004, 0.004)
