"""A world with known truth, and sensors that emit the stack's real message types (docs/DESIGN.md
section 10). The road is the ego car's path; other vehicles move along it at a lane offset; roadside objects
stand beside it. Everything is seeded and deterministic.

    w = World(seed=1, scenario="town")
    for t_arrival, msg in w.messages():      # in arrival order, as the integrator would receive them
        ...
    w.truth_at(t)                             # ego truth; w.targets_at(t): targets in the car frame
"""
import bisect
import heapq
import math
import random

import numpy as np

NS = 1_000_000_000
G = 9.80665
LAT0, LON0 = 54.4000, 18.5000
R_EARTH = 6378137.0
CAM_TO_AXLE = 1.5                # the car frame's origin (reference camera) is this far ahead of the rear axle


def geo(e, n):
    return LAT0 + math.degrees(n / R_EARTH), LON0 + math.degrees(e / (R_EARTH * math.cos(math.radians(LAT0))))


SCENARIOS = {
    # (duration s, speed plan [(t, target speed)], curvature plan [(arc length m, curvature 1/m)])
    "town": dict(T=240.0, speed=[(0, 0.0), (5, 13.0), (45, 13.0), (52, 0.0), (64, 0.0), (66, 9.0), (100, 9.0), (104, 14.0),
                                 (150, 14.0), (158, 0.0), (170, 0.0), (173, 11.0), (240, 11.0)],
                 curv=[(0, 0.0), (300, 0.0), (300, 1 / 60.0), (390, 1 / 60.0), (390, 0.0), (700, 0.0), (700, -1 / 35.0),
                       (755, -1 / 35.0), (755, 0.0), (1300, 0.0), (1300, 1 / 120.0), (1500, 1 / 120.0), (1500, 0.0)]),
    "highway": dict(T=180.0, speed=[(0, 20.0), (30, 31.0), (100, 31.0), (110, 24.0), (140, 24.0), (150, 33.0), (180, 33.0)],
                    curv=[(0, 0.0), (800, 0.0), (800, 1 / 600.0), (1600, 1 / 600.0), (1600, 0.0), (3000, 0.0),
                          (3000, -1 / 800.0), (4000, -1 / 800.0), (4000, 0.0)]),
}


class Target:
    """A vehicle on the road: arc length ahead of the ego at t0, a speed plan [(t, speed)] and a lateral plan
    [(t, offset left of the ego's lane centre)]."""

    def __init__(self, tid, gap0, speed, lateral, t_on=0.0, t_off=1e9, kind="car"):
        self.id, self.gap0, self.speed, self.lateral, self.t_on, self.t_off, self.kind = tid, gap0, speed, lateral, t_on, t_off, kind


def _interp_plan(plan, t):
    ts = [p[0] for p in plan]
    return float(np.interp(t, ts, [p[1] for p in plan]))


class World:
    def __init__(self, seed=1, scenario="town", dt=0.01, targets="default", accel_lim=2.5, statics=True,
                 grade_amp=0.03, speed_scale=0.0):
        sc = SCENARIOS[scenario] if isinstance(scenario, str) else scenario
        self.rng = random.Random(seed)
        self.seed, self.dt, self.T = seed, dt, sc["T"]
        self.speed_scale = speed_scale       # the car's speed signal reads (1 + speed_scale) x ground speed
        n = int(self.T / dt) + 1
        self.t = np.arange(n) * dt + 1000.0   # a monotonic clock that does not start at zero
        self.t0 = float(self.t[0])
        # ego speed: follow the plan with bounded acceleration and jerk
        v = np.zeros(n)
        a = np.zeros(n)
        vc, ac = _interp_plan(sc["speed"], 0.0), 0.0
        for i in range(n):
            tgt = _interp_plan(sc["speed"], i * dt)
            want = max(-accel_lim * 1.4, min(accel_lim, (tgt - vc) / 1.2))
            ac += max(-3.0 * dt, min(3.0 * dt, want - ac))
            vc = max(0.0, vc + ac * dt)
            if vc == 0.0 and tgt == 0.0:
                ac = 0.0
            v[i], a[i] = vc, ac
        a = np.gradient(v, dt)
        self.v, self.a = v, a
        self.s = np.concatenate([[0.0], np.cumsum(0.5 * (v[1:] + v[:-1]) * dt)])
        # the road: curvature over arc length (smoothed), far beyond the ego's own travel
        self.ds = 0.5
        S = np.arange(0.0, self.s[-1] + 600.0, self.ds)
        cs = [p[0] for p in sc["curv"]]
        k = np.interp(S, cs, [p[1] for p in sc["curv"]])
        k = np.convolve(k, np.ones(31) / 31.0, mode="same")
        hd = np.concatenate([[0.6], 0.6 + np.cumsum(0.5 * (k[1:] + k[:-1]) * self.ds)])
        self.S, self.kappa, self.heading = S, k, hd
        self.px = np.concatenate([[0.0], np.cumsum(np.cos(0.5 * (hd[1:] + hd[:-1])) * self.ds)])
        self.py = np.concatenate([[0.0], np.cumsum(np.sin(0.5 * (hd[1:] + hd[:-1])) * self.ds)])
        self.grade_S = grade_amp * np.sin(S / 180.0) if grade_amp else np.zeros(len(S))
        self.alt_S = 40.0 + np.concatenate([[0.0], np.cumsum(np.sin(0.5 * (self.grade_S[1:] + self.grade_S[:-1])) * self.ds)])
        self.psi = np.interp(self.s, S, hd)
        self.w = v * np.interp(self.s, S, k)
        self.pe, self.pn = np.interp(self.s, S, self.px), np.interp(self.s, S, self.py)
        self.th = np.interp(self.s, S, self.grade_S)
        self.z = np.interp(self.s, S, self.alt_S)
        # other vehicles
        self.targets = self._default_targets(scenario) if targets == "default" else list(targets or [])
        self._tgt_s = {}
        for tg in self.targets:
            vt = np.array([_interp_plan(tg.speed, i * dt) for i in range(n)])
            self._tgt_s[tg.id] = (tg.gap0 + CAM_TO_AXLE + np.concatenate([[0.0], np.cumsum(0.5 * (vt[1:] + vt[:-1]) * dt)]), vt)
        # roadside objects (stationary): arc length, lateral offset
        self.statics = []
        if statics:
            sj = 20.0
            while sj < S[-1] - 50.0:
                self.statics.append((sj, self.rng.choice([-1, 1]) * self.rng.uniform(4.0, 9.0)))
                sj += self.rng.uniform(12.0, 45.0)
        self.statics_s = [x[0] for x in self.statics]

    def _default_targets(self, scenario):
        if scenario == "highway":
            return [Target(1, 45.0, [(0, 20.0), (30, 30.0), (95, 30.0), (108, 23.0), (140, 23.0), (150, 34.0), (180, 34.0)], [(0, 0.0)]),
                    Target(2, 20.0, [(0, 24.0), (180, 36.0)], [(0, 3.5)]),
                    Target(3, 90.0, [(0, 27.0), (180, 27.0)], [(0, -3.5), (60, -3.5), (64, 0.0), (120, 0.0), (124, -3.5)])]
        return [
            # the lead: follows the same rhythm a little ahead, stops at the same queues
            Target(1, 22.0, [(0, 0.0), (4, 13.0), (44, 13.0), (50, 0.0), (62, 0.0), (64, 9.5), (100, 9.5), (103, 14.5),
                             (120, 14.5), (240, 16.0)], [(0, 0.0), (118, 0.0), (122, 3.4)]),
            # a car that cuts in at 130 s and brakes to the second stop
            Target(2, 44.0, [(0, 0.0), (5, 12.9), (45, 12.9), (52, 0.0), (64, 0.0), (66, 9.0), (100, 9.0), (104, 13.8),
                             (148, 13.8), (156, 0.0), (168, 0.0), (171, 11.5), (240, 11.5)], [(0, 3.4), (128, 3.4), (132, 0.0)]),
            # oncoming-side / next-lane traffic that is never the lead
            Target(3, 35.0, [(0, 6.0), (240, 12.0)], [(0, -3.4)]),
        ]

    # ---- truth -----------------------------------------------------------------------------------------
    def _i(self, t):
        return min(max(int(round((t - self.t0) / self.dt)), 0), len(self.t) - 1)

    def truth_at(self, t):
        f = (t - self.t0) / self.dt
        i = min(max(int(f), 0), len(self.t) - 2)
        u = min(max(f - i, 0.0), 1.0)

        def L(arr):
            return float(arr[i] + u * (arr[i + 1] - arr[i]))
        return {"t": t, "pe": L(self.pe), "pn": L(self.pn), "psi": L(self.psi), "v": L(self.v), "a": L(self.a),
                "w": L(self.w), "z": L(self.z), "th": L(self.th), "s": L(self.s)}

    def _road_xy(self, s, d):
        hd = float(np.interp(s, self.S, self.heading))
        return (float(np.interp(s, self.S, self.px)) - d * math.sin(hd), float(np.interp(s, self.S, self.py)) + d * math.cos(hd), hd)

    def _to_car(self, tr, x, y):
        """World point -> the car frame (origin the reference camera, x forward, y left)."""
        dx, dy = x - tr["pe"], y - tr["pn"]
        c, s = math.cos(tr["psi"]), math.sin(tr["psi"])
        return c * dx + s * dy - CAM_TO_AXLE, -s * dx + c * dy

    def targets_at(self, t):
        """Vehicles in the car frame: [{id, x, y, vx, vy (over ground, car axes), kind}]."""
        tr = self.truth_at(t)
        i = self._i(t)
        out = []
        for tg in self.targets:
            if not (tg.t_on <= t - self.t0 <= tg.t_off):
                continue
            ss, vt = self._tgt_s[tg.id]
            s, v = float(ss[i]) + float(self.s[0]), float(vt[i])
            d = _interp_plan(tg.lateral, t - self.t0)
            x, y, hd = self._road_xy(s, d)
            cx, cy = self._to_car(tr, x, y)
            dh = hd - tr["psi"]
            out.append({"id": tg.id, "x": cx, "y": cy, "vx": v * math.cos(dh), "vy": v * math.sin(dh), "kind": tg.kind,
                        "d": d, "s": s})
        return out

    def statics_at(self, t, ahead=160.0, behind=20.0):
        tr = self.truth_at(t)
        lo, hi = bisect.bisect_left(self.statics_s, tr["s"] - behind), bisect.bisect_right(self.statics_s, tr["s"] + ahead)
        out = []
        for j in range(lo, hi):
            sj, dj = self.statics[j]
            x, y, _ = self._road_xy(sj, dj)
            cx, cy = self._to_car(tr, x, y)
            out.append({"id": 1000 + j, "x": cx, "y": cy, "vx": 0.0, "vy": 0.0, "kind": "static"})
        return out

    def lead_at(self, t, half_lane=1.5):
        """The true lead: the nearest vehicle ahead within the ego's lane."""
        best = None
        for o in self.targets_at(t):
            if o["x"] > 0.5 and abs(o["d"]) <= half_lane and (best is None or o["x"] < best["x"]):
                best = o
        return best

    def path_ahead(self, t, xs):
        """The road ahead in the car frame at forward distances xs: lateral offsets (left positive)."""
        tr = self.truth_at(t)
        out = []
        for x in xs:
            px, py, _ = self._road_xy(tr["s"] + CAM_TO_AXLE + x, 0.0)
            out.append(self._to_car(tr, px, py)[1])
        return out

    # ---- messages --------------------------------------------------------------------------------------
    def messages(self, sensors):
        """All sensors' messages merged in arrival order: yields (arrival time s, message dict)."""
        heap = []
        for k, sn in enumerate(sensors):
            for arr, msg in sn.generate(self):
                heap.append((arr, k, id(msg), msg))
        heap.sort(key=lambda x: (x[0], x[1]))
        for arr, _, _, msg in heap:
            yield arr, msg
