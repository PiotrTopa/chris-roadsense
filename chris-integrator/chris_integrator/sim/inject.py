"""Synthetic sensors injected into a real recording, derived consistently from it (docs/INTEGRATOR.md
section 10). The car has no radar and one camera; this makes the recording into what such a car would have
recorded:

  * the car's motion is the recording's own (CAN speed and gyro, dead-reckoned into a path);
  * the lead vehicle is where the recording's camera-only lead track says it is, smoothed non-causally (a
    pseudo-truth: good for association, continuity and fault behaviour; not an accuracy reference);
  * roadside objects are invented but stand still in the world the car drives through, so their range rates
    are consistent with the car's motion; clutter and double-bounce ghosts as in sim/radar.py.

    msgs = list(read_session(...))
    for m in inject(msgs, radars=[{"sensor": "radar_front", "mount": (2.2, 0.0, 0.0)}]): ...
"""
import bisect
import math
import random

import numpy as np

NS = 1e9
AXLE = 1.5


class _Ego:
    """The recording's own motion: speed and yaw rate over time (vehicle feed, else the road model), and the
    dead-reckoned pose."""

    def __init__(self, msgs):
        t, v, w = [], [], []
        for m in msgs:
            if m.get("type") == "ego" and isinstance(m.get("speed_mps"), (int, float)) and isinstance(m.get("ts_mono_ns"), int):
                t.append(m["ts_mono_ns"] / NS), v.append(float(m["speed_mps"])), w.append(float(m.get("yaw_rate_rps") or 0.0))
        self.vehicle = len(t) > 50
        if not self.vehicle:
            for m in msgs:
                p = m.get("pose") if m.get("type") == "road" else None
                if p and isinstance(p.get("vx"), (int, float)):
                    t.append(m["ts_mono_ns"] / NS - 0.125), v.append(float(p["vx"])), w.append(-float(p.get("yaw_rate") or 0.0))
        o = np.argsort(t)
        self.t, self.v, self.w = np.array(t)[o], np.array(v)[o], np.array(w)[o]
        dt = np.diff(self.t)
        dt = np.where(dt > 1.5, 0.0, dt)              # a silence of the feed: the pose is not carried through it
        self.psi = np.concatenate([[0.0], np.cumsum(0.5 * (self.w[1:] + self.w[:-1]) * dt)])
        ds = 0.5 * (self.v[1:] + self.v[:-1]) * dt
        pm = 0.5 * (self.psi[1:] + self.psi[:-1])
        self.x = np.concatenate([[0.0], np.cumsum(ds * np.cos(pm))])
        self.y = np.concatenate([[0.0], np.cumsum(ds * np.sin(pm))])
        self.s = np.concatenate([[0.0], np.cumsum(np.abs(ds))])
        self.gap = np.concatenate([[False], np.diff(self.t) > 1.5])

    def at(self, t):
        """(v, w, x, y, psi, s) or None inside a silence of the motion source."""
        i = bisect.bisect_right(self.t, t)
        if i == 0 or i >= len(self.t) or self.gap[i]:
            return None
        u = (t - self.t[i - 1]) / max(self.t[i] - self.t[i - 1], 1e-6)
        L = lambda a: float(a[i - 1] + u * (a[i] - a[i - 1]))
        return L(self.v), L(self.w), L(self.x), L(self.y), L(self.psi), L(self.s)


def lead_truth(msgs, integrator=None):
    """The recording's camera-only lead track, smoothed non-causally per track: lists (t, gap, lateral left, lead
    speed over ground), one list per track id."""
    from ..core.integrator import Integrator
    I = integrator or Integrator()
    rows = {}
    for m in msgs:
        if m.get("type") == "road":
            es, tr, ld = I.frame(m)
            if ld and ld.get("present") and ld.get("valid"):
                rows.setdefault(ld["track_id"], []).append((m["ts_mono_ns"] / NS, ld["dist_m"], -ld["lateral_m"], ld["v_lead_mps"]))
        else:
            I.on_message(m)
    out = []
    for tid, r in rows.items():
        if len(r) < 13:
            continue
        a = np.array(r)
        k = 13                                         # about 1 s of frames
        ker = np.ones(k) / k
        pad = lambda c: np.convolve(np.pad(c, k // 2, mode="edge"), ker, mode="valid")
        out.append(np.column_stack([a[:, 0], pad(a[:, 1]), pad(a[:, 2]), pad(a[:, 3])]))
    return out


def inject(msgs, radars=(), camera2=None, seed=7, hz=17.0, latency=0.05, lead=None, statics=True, clutter=1.0,
           ghost=0.3, sigma=(0.2, 0.4, 0.12), p_d=0.92, speed_fault=None, integrator=None):
    """Yield the recording's messages with synthetic `objects` scans merged in at their arrival times.
    radars: [{"sensor", "mount": (x, y, yaw rad), "fov_deg", "max_range"}]. camera2: {"sensor", "mount", ...}:
    a second camera's object list. speed_fault (t0, t1, factor) in session seconds: the recording's vehicle speed
    and wheel speeds are multiplied by factor in that window (a decoding fault of the speed signal)."""
    rng = random.Random(seed)
    ego = _Ego(msgs)
    tracks = lead_truth(msgs, integrator) if lead is None else lead
    t0 = min(m["ts_mono_ns"] for m in msgs if isinstance(m.get("ts_mono_ns"), int)) / NS
    t1 = max(m["ts_mono_ns"] for m in msgs if isinstance(m.get("ts_mono_ns"), int)) / NS
    # roadside objects along the dead-reckoned path
    stat = []
    if statics and len(ego.s):
        sj = 15.0
        while sj < ego.s[-1] + 150.0:
            stat.append((sj, rng.choice([-1, 1]) * rng.uniform(4.0, 9.0)))
            sj += rng.uniform(10.0, 40.0)
    stat_s = [a for a, _ in stat]

    def lead_at(t):
        for a in tracks:
            if a[0, 0] <= t <= a[-1, 0]:
                i = min(bisect.bisect_left(a[:, 0].tolist(), t), len(a) - 1)
                if abs(a[i, 0] - t) < 0.3:
                    return float(a[i, 1]), float(a[i, 2]), float(a[i, 3])
        return None

    def world_statics(e):
        v, w, x, y, psi, s = e
        lo, hi = bisect.bisect_left(stat_s, s - 10.0), bisect.bisect_right(stat_s, s + 160.0)
        out = []
        c, sn = math.cos(psi), math.sin(psi)
        for j in range(lo, hi):
            sj, dj = stat[j]
            # where the path will be at arc length sj: straight ahead of the current pose is good enough for a
            # roadside object's place (it only has to stand still once placed): place it once, in the world
            key = j
            if key not in placed:
                ahead = sj - s
                placed[key] = (x + c * ahead - sn * dj, y + sn * ahead + c * dj)
            px, py = placed[key]
            dx, dy = px - x, py - y
            out.append((c * dx + sn * dy - AXLE, -sn * dx + c * dy, 0.0, 0.0))
        return out
    placed = {}
    scans = []
    for k, rd in enumerate(list(radars) + ([dict(camera2, _cam=True)] if camera2 else [])):
        mx, my, yaw = rd.get("mount", (2.2, 0.0, 0.0))
        fov, rmax = math.radians(rd.get("fov_deg", 60.0 if rd.get("_cam") else 50.0)), rd.get("max_range", 70.0 if rd.get("_cam") else 160.0)
        rate = rd.get("hz", 10.0 if rd.get("_cam") else hz)
        t = t0 + 0.07 + 0.013 * k
        sr, sa, sv = sigma[0], math.radians(sigma[1]), sigma[2]
        while t < t1:
            e = ego.at(t)
            if e is not None:
                v, w = e[0], e[1]
                objs = []
                ld = lead_at(t)
                things = []
                if ld is not None:
                    things.append((ld[0], ld[1], ld[2], 0.0, True))
                if not rd.get("_cam"):
                    things += [(a, b, c, d, False) for a, b, c, d in world_statics(e)]
                near = None
                for x, y, vx, vy, veh in things:
                    dx, dy = x - mx, y - my
                    r = math.hypot(dx, dy)
                    az = (math.atan2(dy, dx) - yaw + math.pi) % (2.0 * math.pi) - math.pi
                    if r < 1.0 or r > rmax or abs(az) > fov or rng.random() > (p_d if veh else 0.7):
                        continue
                    if rd.get("_cam"):
                        er, ec = rng.gauss(0, 0.3 + 0.05 * r), rng.gauss(0, 0.08 + 0.005 * r)
                        cx, cy = r * math.cos(az), r * math.sin(az)
                        objs.append({"x_m": round(cx + er * cx / r - ec * cy / r, 2), "y_m": round(cy + er * cy / r + ec * cx / r, 2),
                                     "class": "car", "prob": 0.9})
                        continue
                    vsx, vsy = v - w * my, w * (mx + AXLE)
                    rr = (dx * (vx - vsx) + dy * (vy - vsy)) / r
                    objs.append({"range_m": round(r + rng.gauss(0, sr), 2), "azimuth_deg": round(math.degrees(az + rng.gauss(0, sa)), 3),
                                 "range_rate_mps": round(rr + rng.gauss(0, sv), 3), "synthetic": "lead" if veh else "static"})
                    if veh and (near is None or r < near[0]):
                        near = (r, az, rr)
                if not rd.get("_cam"):
                    if near is not None and 2.0 * near[0] < rmax and rng.random() < ghost:
                        objs.append({"range_m": round(2.0 * near[0], 2), "azimuth_deg": round(math.degrees(near[1]), 3),
                                     "range_rate_mps": round(2.0 * near[2], 3), "synthetic": "ghost"})
                    for _ in range(rng.choice([0, 1, 2, 3])):
                        if rng.random() < clutter / 1.5:
                            objs.append({"range_m": round(rng.uniform(3.0, rmax), 2), "azimuth_deg": round(math.degrees(rng.uniform(-fov, fov)), 3),
                                         "range_rate_mps": round(rng.uniform(-35.0, 10.0), 3), "synthetic": "clutter"})
                msg = {"type": "objects", "sensor": rd["sensor"], "kind": "camera" if rd.get("_cam") else "radar", "tracked": False,
                       "ts_mono_ns": int(round(t * NS)), "synthetic": True, "objects": objs}
                if not rd.get("_cam"):
                    msg["mount"] = {"x_m": mx, "y_m": my, "yaw_deg": math.degrees(yaw)}
                scans.append((t + latency, msg))
            t += 1.0 / rate + rng.uniform(-0.003, 0.003)
    scans.sort(key=lambda x: x[0])
    k = 0
    last = None
    for m in msgs:
        ts = m.get("ts_mono_ns")
        if isinstance(ts, int):
            tt = ts / NS
            last = tt if last is None else max(last, tt)
            while k < len(scans) and scans[k][0] <= last:
                yield scans[k][1]
                k += 1
        if speed_fault and m.get("type") == "ego" and isinstance(ts, int) and speed_fault[0] <= ts / NS - t0 < speed_fault[1] \
                and isinstance(m.get("speed_mps"), (int, float)):
            m = dict(m, speed_mps=m["speed_mps"] * speed_fault[2])
            if isinstance(m.get("wheel_speeds_mps"), list):
                m["wheel_speeds_mps"] = [x * speed_fault[2] for x in m["wheel_speeds_mps"]]
        yield m
