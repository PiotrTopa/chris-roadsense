"""A simulated radar: per scan the detections of every vehicle and roadside object in its field of view
(range, azimuth, range rate, with noise and missed detections), clutter, and double-bounce ghosts of the
nearest vehicle ahead; emitted as `objects` messages of the vehicle input API, polar in the sensor's frame.
Mount anywhere on the car (x, y in the car frame, yaw of the boresight); `misalign` is a true yaw error the
integrator has to find."""
import math
import random

from .world import CAM_TO_AXLE, NS


class RadarSim:
    def __init__(self, sensor="radar_front", mount=(2.2, 0.0, 0.0), seed=51, hz=17.0, latency=0.05, fov_deg=50.0,
                 max_range=160.0, sigma=(0.2, 0.4, 0.12), p_d=0.92, p_d_static=0.7, clutter=2.0, ghost=0.3,
                 misalign_deg=0.0, blackout=(), with_mount=True):
        self.__dict__.update(locals())

    def _det(self, o, tr, rng, mx, my, yaw):
        dx, dy = o["x"] - mx, o["y"] - my
        r = math.hypot(dx, dy)
        az = math.atan2(dy, dx) - yaw
        az = (az + math.pi) % (2.0 * math.pi) - math.pi
        if r < 1.0 or r > self.max_range or abs(az) > math.radians(self.fov_deg):
            return None
        vsx, vsy = tr["v"] - tr["w"] * my, tr["w"] * (mx + CAM_TO_AXLE)
        rr = (dx * (o["vx"] - vsx) + dy * (o["vy"] - vsy)) / r
        return r, az, rr

    def generate(self, w):
        rng = random.Random(self.seed)
        mx, my, myaw = self.mount
        yaw = myaw + math.radians(self.misalign_deg)          # where the sensor really points
        t = w.t0 + 0.07
        sr, sa, sv = self.sigma[0], math.radians(self.sigma[1]), self.sigma[2]
        while t < w.t0 + w.T:
            rel = t - w.t0
            if not any(a <= rel < b for a, b in self.blackout):
                tr = w.truth_at(t)
                objs = []
                near = None
                for o in w.targets_at(t) + w.statics_at(t):
                    d = self._det(o, tr, rng, mx, my, yaw)
                    if d is None or rng.random() > (self.p_d_static if o["kind"] == "static" else self.p_d):
                        continue
                    r, az, rr = d
                    objs.append((r + rng.gauss(0, sr), az + rng.gauss(0, sa), rr + rng.gauss(0, sv), o["id"]))
                    if o["kind"] != "static" and abs(o["y"]) < 1.5 and o["x"] > 0 and (near is None or r < near[0]):
                        near = d
                if near is not None and 2.0 * near[0] < self.max_range and rng.random() < self.ghost:
                    objs.append((2.0 * near[0] + rng.gauss(0, 2 * sr), near[1] + rng.gauss(0, sa), 2.0 * near[2] + rng.gauss(0, 2 * sv), -1))
                for _ in range(rng.choice([0, 1, 2, 3, 4]) if self.clutter else 0):
                    if rng.random() < self.clutter / 2.0:
                        objs.append((rng.uniform(3.0, self.max_range), rng.uniform(-1, 1) * math.radians(self.fov_deg),
                                     rng.uniform(-35.0, 10.0), -2))
                msg = {"type": "objects", "sensor": self.sensor, "kind": "radar", "tracked": False, "ts_mono_ns": int(round(t * NS)),
                       "objects": [{"range_m": round(r, 2), "azimuth_deg": round(math.degrees(az), 3),
                                    "range_rate_mps": round(rr, 3), "truth": i} for r, az, rr, i in objs]}
                if self.with_mount:
                    msg["mount"] = {"x_m": mx, "y_m": my, "yaw_deg": math.degrees(myaw)}
                yield t + self.latency + rng.uniform(0, 0.01), msg
            t += 1.0 / self.hz + rng.uniform(-0.003, 0.003)
