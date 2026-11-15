"""Synthetic drives over a map pack: a route through the road graph, a simple vehicle that follows it,
and the position-input messages (GNSS + motion) a feeder would send for that drive, with noise, dropouts
and biases. Used by the tests and by tools/synth_drive.py; nothing in the daemon depends on it."""
import heapq
import math
import random

import numpy as np

from . import context, geo
from . import rsmap as R

CLASS_SPEED = {"motorway": 33.0, "motorway_link": 20.0, "trunk": 28.0, "trunk_link": 18.0, "primary": 20.0,
               "primary_link": 14.0, "secondary": 17.0, "secondary_link": 12.0, "tertiary": 14.0,
               "tertiary_link": 11.0, "unclassified": 11.0, "residential": 8.0, "living_street": 4.0,
               "service": 4.0, "track": 3.0}


def snap(pack, lat, lon, radius=300.0, classes=None):
    """Nearest edge to a point: (edge, s along it) or None."""
    best = None
    for e in pack.query(lat, lon, radius):
        if classes and e.cls_name not in classes:
            continue
        d, s, _ = e.project(lat, lon)
        if best is None or d < best[0]:
            best = (d, e, s)
    return (best[1], best[2]) if best and best[0] <= radius else None


def route(pack, start, goal, classes=None, max_nodes=400000):
    """Fastest route between two (lat, lon) points: [(edge, dir, s_from, s_to)]. Raises ValueError if none."""
    a, b = snap(pack, *start, classes=classes), snap(pack, *goal, classes=classes)
    if a is None or b is None:
        raise ValueError("no road near the %s point" % ("start" if a is None else "goal"))
    ea, sa = a
    eb, sb = b
    glat, glon = goal

    def h(nref):
        lat, lon = pack.node_pos(nref)
        return geo.dist_m(lat, lon, glat, glon) / 36.0

    def cost(e, length):
        return length / CLASS_SPEED.get(e.cls_name, 8.0)

    if ea.ref == eb.ref and (sb >= sa or not ea.oneway):
        return [(ea, 1 if sb >= sa else -1, sa, sb)]
    heap, best, prev = [], {}, {}
    n = 0
    starts = [(ea.b, cost(ea, ea.length - sa), 1)] + ([] if ea.oneway else [(ea.a, cost(ea, sa), -1)])
    for node, c0, d in starts:
        best[node] = c0
        prev[node] = (None, ea, d)
        heapq.heappush(heap, (c0 + h(node), n, c0, node))
        n += 1
    targets = {eb.a: 1}
    if not eb.oneway:
        targets[eb.b] = -1
    done = None
    final = {}
    expanded = 0
    while heap:
        _, _, c, node = heapq.heappop(heap)
        if c > best.get(node, 1e18):
            continue
        if node in targets:
            d = targets[node]
            tot = c + cost(eb, sb if d > 0 else eb.length - sb)
            if done is None or tot < done[0]:
                done = (tot, node, d)
            final[node] = tot
            if len(final) == len(targets):
                break
        if done is not None and c > done[0]:
            break
        expanded += 1
        if expanded > max_nodes:
            break
        for ref, end in pack.node_edges(node):
            e = pack.edge(ref)
            if e is None:
                continue
            d = 1 if end == 0 else -1
            if d < 0 and e.oneway:
                continue
            nxt = e.b if d > 0 else e.a
            c2 = c + cost(e, e.length) + (30.0 if e.flags & R.F_ACCESS_RESTRICTED else 0.0)
            if c2 < best.get(nxt, 1e18):
                best[nxt] = c2
                prev[nxt] = (node, e, d)
                heapq.heappush(heap, (c2 + h(nxt), n, c2, nxt))
                n += 1
    if done is None:
        raise ValueError("no route")
    _, node, d = done
    path = [(eb, d, 0.0 if d > 0 else eb.length, sb)]
    while node is not None:
        pnode, e, dd = prev[node]
        if pnode is None:
            path.append((e, dd, sa, e.length if dd > 0 else 0.0))
        else:
            path.append((e, dd, 0.0 if dd > 0 else e.length, e.length if dd > 0 else 0.0))
        node = pnode
    path.reverse()
    return path


def path_from(pack, lat, lon, heading_deg, length_m, limits=None):
    """A path that simply follows the most likely continuation from a point (no destination)."""
    best = None
    for e in pack.query(lat, lon, 80.0):
        d, s, brg = e.project(lat, lon)
        for direction in ((1,) if e.oneway else (1, -1)):
            b = brg if direction > 0 else (brg + 180.0) % 360.0
            sc = d + abs(geo.ang_diff(heading_deg, b))
            if best is None or sc < best[0]:
                best = (sc, e, direction, s)
    if best is None:
        raise ValueError("no road near the start point")
    segs, _ = context.most_likely_path(pack, best[1], best[2], best[3], length_m)
    return [(e, d, s0, s1) for e, d, _, s0, s1, _ in segs]


class Truth:
    __slots__ = ("t", "lat", "lon", "heading", "v", "yaw_ccw", "edge", "dir", "tunnel", "steer_deg", "path_s")


class Drive:
    """A vehicle following a path with pure pursuit; iterate to get Truth samples every dt seconds."""

    def __init__(self, pack, path, dt=0.05, limits=None, speed_factor=0.95, v0=None, max_v=None, lateral_m=0.0,
                 wheelbase_m=2.70, steering_ratio=19.1):
        self.dt = dt
        self.limits = limits or context.load_limits("pl")
        self.wheelbase, self.ratio = wheelbase_m, steering_ratio
        lat, lon, owner = [], [], []
        self.targets = []
        for i, (e, d, s0, s1) in enumerate(path):
            lo, hi = min(s0, s1), max(s0, s1)
            i0 = int(np.searchsorted(e.s, lo, side="right"))
            i1 = int(np.searchsorted(e.s, hi, side="left"))
            la = [e.at(lo)[0]] + e.lat[i0:i1].tolist() + [e.at(hi)[0]]
            lo_ = [e.at(lo)[1]] + e.lon[i0:i1].tolist() + [e.at(hi)[1]]
            if d < 0:
                la.reverse()
                lo_.reverse()
            lat += la
            lon += lo_
            owner += [i] * len(la)
            lim = context.speed_limit(e, d, self.limits)["limit_kmh"]
            vt = (lim / 3.6 if lim else CLASS_SPEED.get(e.cls_name, 10.0)) * speed_factor
            if e.flags & R.F_ROUNDABOUT:
                vt = min(vt, 7.0)
            self.targets.append(min(vt, max_v) if max_v else vt)
        self.path = path
        self.local = geo.Local(lat[0], lon[0])
        x = (np.array(lon) - self.local.lon0) * self.local.mlon
        y = (np.array(lat) - self.local.lat0) * self.local.mlat
        keep = np.concatenate([[True], np.hypot(np.diff(x), np.diff(y)) > 0.02])
        self.x, self.y, self.owner = x[keep], y[keep], np.array(owner)[keep]
        if lateral_m:                                   # drive offset to the right of the centre line
            dx, dy = np.gradient(self.x), np.gradient(self.y)
            nrm = np.hypot(dx, dy)
            nrm[nrm == 0] = 1.0
            self.x, self.y = self.x + lateral_m * dy / nrm, self.y - lateral_m * dx / nrm
        self.s = np.concatenate([[0.0], np.cumsum(np.hypot(np.diff(self.x), np.diff(self.y)))])
        self.length = float(self.s[-1])
        # turn severity at every vertex, for slowing down before corners
        hd = np.arctan2(np.diff(self.x), np.diff(self.y))
        self.turn = np.concatenate([[0.0], np.abs((np.diff(hd) + math.pi) % (2 * math.pi) - math.pi), [0.0]])
        self.v = self.targets[0] if v0 is None else v0

    def _at(self, s):
        s = min(max(s, 0.0), self.length)
        i = min(len(self.s) - 2, max(0, int(np.searchsorted(self.s, s, side="right")) - 1))
        seg = self.s[i + 1] - self.s[i]
        t = (s - self.s[i]) / seg if seg > 0 else 0.0
        return (self.x[i] + t * (self.x[i + 1] - self.x[i]), self.y[i] + t * (self.y[i + 1] - self.y[i]), i)

    def __iter__(self):
        px, py, i = self._at(0.0)
        th = math.atan2(self.x[i + 1] - self.x[i], self.y[i + 1] - self.y[i])     # clockwise from north
        s, t, v = 0.0, 0.0, self.v
        dt = self.dt
        while s < self.length - 1.0:
            ld = max(6.0, 0.9 * v)
            tx, ty, _ = self._at(s + ld)
            alpha = geo.wrap_pi(math.atan2(tx - px, ty - py) - th)
            kappa = 2.0 * math.sin(alpha) / max(1.0, math.hypot(tx - px, ty - py))      # 1/m, positive = right
            _, _, i = self._at(s)
            vt = self.targets[int(self.owner[i + 1])]        # a segment belongs to the edge of its end vertex
            j0 = int(np.searchsorted(self.s, s))
            j1 = int(np.searchsorted(self.s, s + max(15.0, v * v / 4.0)))
            sharp = float(self.turn[j0:j1 + 1].max()) if j1 >= j0 else 0.0
            if sharp > math.radians(25.0):
                vt = min(vt, max(3.5, 9.0 - 5.0 * (sharp - math.radians(25.0))))
            if abs(kappa) > 1e-4:
                vt = min(vt, math.sqrt(2.5 / abs(kappa)))
            a = max(-3.0, min(2.0, (vt - v) / 1.0))
            v = max(0.5, v + a * dt)
            yaw_cw = v * kappa
            th += yaw_cw * dt
            px += v * math.sin(th) * dt
            py += v * math.cos(th) * dt
            # progress along the path: project onto the neighbourhood of the previous progress
            best = None
            for ds in np.arange(-2.0, max(4.0, v * dt * 3.0), 0.25):
                qx, qy, _ = self._at(s + ds)
                d2 = (qx - px) ** 2 + (qy - py) ** 2
                if best is None or d2 < best[0]:
                    best = (d2, s + ds)
            s = max(s, best[1])
            t += dt
            o = Truth()
            o.t = t
            o.lat, o.lon = self.local.to_ll(px, py)
            o.heading = math.degrees(th) % 360.0
            o.v, o.yaw_ccw = v, -yaw_cw
            _, _, i = self._at(s)
            e, d, _, _ = self.path[int(self.owner[i + 1])]
            o.edge, o.dir = e, d
            o.tunnel = bool(e.flags & R.F_TUNNEL)
            o.steer_deg = -math.degrees(math.atan(self.wheelbase * kappa)) * self.ratio   # positive = left
            o.path_s = s
            yield o


def feed(truth, t0_ns=0, seed=1, gnss_hz=1.0, motion_hz=20.0, motion="yaw", gnss_sigma_m=3.0, gnss_tau_s=20.0,
         gnss_white_m=0.5, hacc_m=5.0, tunnel_blocks_gnss=True, dropouts=(), explicit_none=False,
         bias_m=None, yaw_bias_dps=0.15, yaw_noise_dps=0.2, speed_scale=1.01, speed_noise=0.05,
         gnss_course=True, source="synthetic", tick_hz=0.0):
    """Turn Truth samples into (t_ns, message, truth) tuples of the position input API.

    tick_hz > 0 also yields (t_ns, None, truth) at that rate, so that a consumer can step its engine while no
    message is due (no motion input and no fix, e.g. in a tunnel).

    motion: "yaw" (speed + yaw_rate_rps), "steer" (speed + steering_wheel_deg), "wheels" (speed + four wheel
    speeds) or "none". dropouts: [(t_from, t_to)] seconds without a fix (also inside tunnels when
    tunnel_blocks_gnss). bias_m: f(t) -> (east, north) metres added to the GNSS position."""
    rnd = random.Random(seed)
    ge = gn = 0.0
    next_g = next_m = next_tick = 0.0
    last_t = None
    for o in truth:
        dt = o.t - last_t if last_t is not None else 0.0
        last_t = o.t
        if gnss_tau_s > 0 and dt > 0:
            k = math.exp(-dt / gnss_tau_s)
            q = gnss_sigma_m * math.sqrt(1 - k * k)
            ge, gn = ge * k + rnd.gauss(0, q), gn * k + rnd.gauss(0, q)
        ts = t0_ns + int(o.t * 1e9)
        if tick_hz > 0 and o.t >= next_tick:
            next_tick = o.t + 1.0 / tick_hz
            yield ts, None, o
        if motion != "none" and o.t >= next_m:
            next_m = o.t + 1.0 / motion_hz
            m = {"type": "motion", "ts_mono_ns": ts, "speed_mps": round(max(0.0, o.v * speed_scale + rnd.gauss(0, speed_noise)), 3),
                 "gear": "D", "source": source}
            if motion == "yaw":
                m["yaw_rate_rps"] = round(o.yaw_ccw + math.radians(yaw_bias_dps + rnd.gauss(0, yaw_noise_dps)), 5)
            elif motion == "steer":
                m["steering_wheel_deg"] = round(o.steer_deg + rnd.gauss(0, 0.5), 2)
            elif motion == "wheels":
                half = 0.5 * 1.48 * o.yaw_ccw
                m["wheel_speeds_mps"] = [round(o.v - half, 3), round(o.v + half, 3), round(o.v - half, 3), round(o.v + half, 3)]
            yield ts, m, o
        if gnss_hz > 0 and o.t >= next_g:
            next_g = o.t + 1.0 / gnss_hz
            blocked = (tunnel_blocks_gnss and o.tunnel) or any(a <= o.t < b for a, b in dropouts)
            if blocked:
                if explicit_none:
                    yield ts, {"type": "gnss", "ts_mono_ns": ts, "fix": "none", "source": source}, o
                continue
            be, bn = bias_m(o.t) if bias_m else (0.0, 0.0)
            lat, lon = geo.offset(o.lat, o.lon, gn + bn + rnd.gauss(0, gnss_white_m), ge + be + rnd.gauss(0, gnss_white_m))
            g = {"type": "gnss", "ts_mono_ns": ts, "fix": "3d", "lat": round(lat, 7), "lon": round(lon, 7),
                 "hacc_m": hacc_m, "speed_mps": round(max(0.0, o.v + rnd.gauss(0, 0.15)), 2), "sats": 14, "source": source}
            if gnss_course:
                g["course_deg"] = round((o.heading + rnd.gauss(0, 1.0 + 3.0 / max(1.0, o.v))) % 360.0, 1)
                g["course_acc_deg"] = round(1.5 + 4.0 / max(1.0, o.v), 1)
            yield ts, g, o
