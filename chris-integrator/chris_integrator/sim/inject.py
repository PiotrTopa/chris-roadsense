"""Synthetic sensors injected into a real recording, derived consistently from it (docs/INTEGRATOR.md
section 10). The car has no radar and one camera; this makes the recording into what such a car would have
recorded:

  * the car's motion is the recording's own (CAN speed and gyro, dead-reckoned into a path);
  * the lead vehicle is where the recording's camera-only lead track says it is, smoothed non-causally (a
    pseudo-truth: good for association, continuity and fault behaviour; not an accuracy reference). Camera
    tracks that are one vehicle seen with a gap (the next track starts where the previous one, carried on by
    its own closing speed, would be; `link_tracks`) are joined and the vehicle is injected through the gap:
    a vehicle does not vanish because the camera lost it for a second;
  * roadside objects are invented but stand still in the world the car drives through, so their range rates
    are consistent with the car's motion. They are placed beside the road actually driven: at their lateral
    offset from the recording's whole dead-reckoned path (known in advance offline), and dropped where another
    part of the path passes closer than `roadside_min_m` (a junction, a hairpin). (Before L41 they were placed
    straight ahead of the pose at the moment they came into range, which in a bend put some on the road;
    `statics_mode="ahead"` keeps that for comparison.) Clutter and double-bounce ghosts as in sim/radar.py.

Every injected detection carries `synthetic` (lead / static / ghost / clutter) and, for the vehicle and the
roadside objects, `synthetic_id`: for the evaluation only; no source reads them. `world` (a dict) receives the
`Injected` description (vehicles and objects over time) for the evaluation.

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


def lead_truth(msgs, integrator=None, split=True, jump_m=1.5, jump_rel=0.08, unsplit=None):
    """The recording's camera-only lead track, smoothed non-causally per track: lists (t, gap, lateral left, lead
    speed over ground), one list per track id. split: a track is cut where its distance steps by more than its
    own relative speed explains (more than max(jump_m, jump_rel x distance) between two frames, or a silence of
    0.5 s): the camera's track keeps its id when it re-measures the same vehicle at another distance
    ("relocated"), and smoothing across such a step would inject a vehicle moving at tens of m/s, which no
    tracker should follow. Each piece is then a vehicle of its own (`link_tracks` joins the consistent ones).
    unsplit: a list that receives the tracks smoothed without splitting (what L38 used), for comparison."""
    from ..core.integrator import Integrator
    I = integrator or Integrator()
    rows = {}
    for m in msgs:
        if m.get("type") == "road":
            es, tr, ld = I.frame(m)
            if ld and ld.get("present") and ld.get("valid"):
                rows.setdefault(ld["track_id"], []).append((m["ts_mono_ns"] / NS, ld["dist_m"], -ld["lateral_m"], ld["v_lead_mps"],
                                                             ld.get("v_rel_mps") if ld.get("v_rel_mps") is not None else 0.0))
        else:
            I.on_message(m)
    pieces = []
    for tid, r in rows.items():
        cur = [r[0]]
        for p, q in zip(r, r[1:]):
            dt = q[0] - p[0]
            step = abs(q[1] - p[1] - 0.5 * (p[4] + q[4]) * dt)
            if split and (dt > 0.5 or step > max(jump_m, jump_rel * q[1])):
                pieces.append(cur)
                cur = []
            cur.append(q)
        pieces.append(cur)
    def smooth(parts):
        out = []
        for r in parts:
            if len(r) < 13:
                continue
            a = np.array(r)
            k = 13                                     # about 1 s of frames
            ker = np.ones(k) / k
            pad = lambda c: np.convolve(np.pad(c, k // 2, mode="edge"), ker, mode="valid")
            out.append(np.column_stack([a[:, 0], pad(a[:, 1]), pad(a[:, 2]), pad(a[:, 3])]))
        out.sort(key=lambda a: a[0, 0])
        return out
    if unsplit is not None:
        unsplit.extend(smooth(rows.values()))
    return smooth(pieces)


def link_tracks(tracks, max_gap_s=3.0, tol_m=1.5, tol_rel=0.08, tol_rate=0.5, tol_lat_m=1.5, tol_v=3.0):
    """Join camera tracks that are one vehicle seen with a gap: the next track starts within max_gap_s after the
    previous one ended, where the previous one's distance carried on by its own rate of change (over its last
    second) would be (within max(tol_m, tol_rel x distance), the step `lead_truth` splits at, plus tol_rate per
    second of gap), at the same lateral place and speed. Returns one array (t, gap, lateral left, speed) per
    vehicle, the gaps filled by linear interpolation, and per vehicle the indices of the tracks it joined."""
    tr = sorted((a for a in tracks if len(a)), key=lambda a: a[0, 0])
    groups = []
    for k, a in enumerate(tr):
        g = groups[-1] if groups else None
        if g is not None:
            b = g[-1][1]
            gap = a[0, 0] - b[-1, 0]
            m = b[:, 0] >= b[-1, 0] - 1.0
            slope = 0.0
            if m.sum() >= 3:
                slope = float(np.polyfit(b[m, 0] - b[-1, 0], b[m, 1], 1)[0])
            pred = b[-1, 1] + slope * max(gap, 0.0)
            if -0.3 <= gap <= max_gap_s and abs(a[0, 1] - pred) <= max(tol_m, tol_rel * max(pred, 0.0)) + tol_rate * max(gap, 0.0) \
                    and abs(a[0, 2] - b[-1, 2]) <= tol_lat_m and abs(a[0, 3] - b[-1, 3]) <= tol_v:
                g.append((k, a))
                continue
        groups.append([(k, a)])
    out, members = [], []
    for g in groups:
        parts = []
        for n, (k, a) in enumerate(g):
            if n:
                b = parts[-1]
                t0, t1 = b[-1, 0], a[0, 0]
                if t1 - t0 > 0.1:
                    ts = np.arange(t0 + 1.0 / 13.0, t1 - 0.02, 1.0 / 13.0)
                    if len(ts):
                        u = ((ts - t0) / (t1 - t0))[:, None]
                        parts.append(np.column_stack([ts, (b[-1, 1:] * (1 - u) + a[0, 1:] * u)]))
                a = a[a[:, 0] > parts[-1][-1, 0]]
            if len(a):
                parts.append(a)
        out.append(np.vstack(parts))
        members.append([k for k, _ in g])
    return out, members


class Injected:
    """What was injected, for the evaluation: the vehicles (car frame, from the pseudo-truth) and the roadside
    objects (fixed in the dead-reckoned world) at any time."""

    def __init__(self, ego, vehicles, stat_world, stat_s, members=None):
        self.ego, self.vehicles, self.members = ego, vehicles, members
        self.stat_world = stat_world          # [(x, y)] world positions (None where dropped / not yet placed)
        self.stat_s = stat_s
        self._vt = [v[:, 0] for v in vehicles]
        self.path = (ego.x, ego.y)

    def vehicle_at(self, t, max_dt=0.3):
        """[(vehicle index, gap, lateral left, speed)] of the vehicles present at t."""
        out = []
        for k, (a, ta) in enumerate(zip(self.vehicles, self._vt)):
            if ta[0] <= t <= ta[-1]:
                i = min(bisect.bisect_left(ta, t), len(ta) - 1)
                if abs(ta[i] - t) < max_dt:
                    out.append((k, float(a[i, 1]), float(a[i, 2]), float(a[i, 3])))
        return out

    def statics_at(self, t, e=None, lo_m=-10.0, hi_m=170.0):
        """[(object index, x, y)] in the car frame of the roadside objects placed so far around the pose at t."""
        e = e if e is not None else self.ego.at(t)
        if e is None:
            return []
        v, w, x, y, psi, s = e
        c, sn = math.cos(psi), math.sin(psi)
        lo, hi = bisect.bisect_left(self.stat_s, s + lo_m), bisect.bisect_right(self.stat_s, s + hi_m)
        out = []
        for j in range(lo, hi):
            p = self.stat_world[j]
            if p is None:
                continue
            dx, dy = p[0] - x, p[1] - y
            out.append((j, c * dx + sn * dy - AXLE, -sn * dx + c * dy))
        return out

    def path_distance(self, j):
        """Distance of roadside object j from the whole driven path (m)."""
        p = self.stat_world[j]
        if p is None:
            return None
        return float(np.min(np.hypot(self.path[0] - p[0], self.path[1] - p[1])))


def _place_statics(ego, rng, mode, roadside_min_m):
    """Arc lengths and lateral offsets of the roadside objects; for mode "path" also their world positions."""
    stat = []
    if len(ego.s):
        sj = 15.0
        end = ego.s[-1] + (150.0 if mode == "ahead" else 0.0)
        while sj < end:
            stat.append((sj, rng.choice([-1, 1]) * rng.uniform(4.0, 9.0)))
            sj += rng.uniform(10.0, 40.0)
    world = [None] * len(stat)
    if mode == "path" and stat:
        # the path by arc length (s is non-decreasing; keep the first sample of each arc length)
        s, idx = np.unique(ego.s, return_index=True)
        px, py, ps = ego.x[idx], ego.y[idx], ego.psi[idx]
        # the path thinned to about 1 m for the distance test
        keep = np.concatenate([[0], np.nonzero(np.diff(np.floor(s)) > 0)[0] + 1])
        tx, ty = px[keep], py[keep]
        for j, (sj, dj) in enumerate(stat):
            x, y, psi = (float(np.interp(sj, s, a)) for a in (px, py, ps))
            qx, qy = x - math.sin(psi) * dj, y + math.cos(psi) * dj
            if float(np.min(np.hypot(tx - qx, ty - qy))) < roadside_min_m:
                continue                       # another stretch of the road passes here
            world[j] = (qx, qy)
    return stat, world


def inject(msgs, radars=(), camera2=None, seed=7, hz=17.0, latency=0.05, lead=None, statics=True, clutter=1.0,
           ghost=0.3, sigma=(0.2, 0.4, 0.12), p_d=0.92, speed_fault=None, integrator=None, statics_mode="path",
           roadside_min_m=3.5, link=True, world=None):
    """Yield the recording's messages with synthetic `objects` scans merged in at their arrival times.
    radars: [{"sensor", "mount": (x, y, yaw rad), "fov_deg", "max_range"}]. camera2: {"sensor", "mount", ...}:
    a second camera's object list. speed_fault (t0, t1, factor) in session seconds: the recording's vehicle speed
    and wheel speeds are multiplied by factor in that window (a decoding fault of the speed signal).
    statics_mode: "path" (beside the driven road) or "ahead" (the L38 placement). link: join camera tracks of one
    vehicle across gaps (`link_tracks`). world: a dict that receives "injected" (an `Injected`)."""
    rng = random.Random(seed)
    ego = _Ego(msgs)
    tracks = lead_truth(msgs, integrator) if lead is None else lead
    if link:
        tracks, members = link_tracks(tracks)
    else:
        members = [[k] for k in range(len(tracks))]
    t0 = min(m["ts_mono_ns"] for m in msgs if isinstance(m.get("ts_mono_ns"), int)) / NS
    t1 = max(m["ts_mono_ns"] for m in msgs if isinstance(m.get("ts_mono_ns"), int)) / NS
    # roadside objects along the dead-reckoned path
    stat, stat_world = _place_statics(ego, rng, statics_mode, roadside_min_m) if statics else ([], [])
    stat_s = [a for a, _ in stat]
    W = Injected(ego, tracks, stat_world, stat_s, members)
    if world is not None:
        world["injected"] = W

    def world_statics(e):
        v, w, x, y, psi, s = e
        c, sn = math.cos(psi), math.sin(psi)
        if statics_mode == "ahead":
            lo, hi = bisect.bisect_left(stat_s, s - 10.0), bisect.bisect_right(stat_s, s + 160.0)
            for j in range(lo, hi):
                if stat_world[j] is None:
                    # straight ahead of the current pose, placed once in the world when it comes into range
                    sj, dj = stat[j]
                    ahead = sj - s
                    stat_world[j] = (x + c * ahead - sn * dj, y + sn * ahead + c * dj)
        return [(xc, yc, 0.0, 0.0, j) for j, xc, yc in W.statics_at(None, e, hi_m=160.0)]
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
                things = [(gx, gy, gv, 0.0, ("lead", vid)) for vid, gx, gy, gv in W.vehicle_at(t)]
                if not rd.get("_cam"):
                    things += [(a, b, c, d, ("static", j)) for a, b, c, d, j in world_statics(e)]
                near = None
                for x, y, vx, vy, (tag, oid) in things:
                    veh = tag == "lead"
                    dx, dy = x - mx, y - my
                    r = math.hypot(dx, dy)
                    az = (math.atan2(dy, dx) - yaw + math.pi) % (2.0 * math.pi) - math.pi
                    if r < 1.0 or r > rmax or abs(az) > fov or rng.random() > (p_d if veh else 0.7):
                        continue
                    if rd.get("_cam"):
                        er, ec = rng.gauss(0, 0.3 + 0.05 * r), rng.gauss(0, 0.08 + 0.005 * r)
                        cx, cy = r * math.cos(az), r * math.sin(az)
                        objs.append({"x_m": round(cx + er * cx / r - ec * cy / r, 2), "y_m": round(cy + er * cy / r + ec * cx / r, 2),
                                     "class": "car", "prob": 0.9, "synthetic": tag, "synthetic_id": oid})
                        continue
                    vsx, vsy = v - w * my, w * (mx + AXLE)
                    rr = (dx * (vx - vsx) + dy * (vy - vsy)) / r
                    objs.append({"range_m": round(r + rng.gauss(0, sr), 2), "azimuth_deg": round(math.degrees(az + rng.gauss(0, sa)), 3),
                                 "range_rate_mps": round(rr + rng.gauss(0, sv), 3), "synthetic": tag, "synthetic_id": oid})
                    if veh and (near is None or r < near[0]):
                        near = (r, az, rr, oid)
                if not rd.get("_cam"):
                    if near is not None and 2.0 * near[0] < rmax and rng.random() < ghost:
                        objs.append({"range_m": round(2.0 * near[0], 2), "azimuth_deg": round(math.degrees(near[1]), 3),
                                     "range_rate_mps": round(2.0 * near[2], 3), "synthetic": "ghost", "synthetic_id": near[3]})
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
