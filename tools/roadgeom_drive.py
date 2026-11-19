#!/usr/bin/env python3
"""Synthetic drive for the fused road geometry (`view.road`; api 1: topic `roadgeom`), without hardware and without daemons.

A vehicle is driven along a route (mapd's simulator); its position input goes through the real map engine,
a synthetic camera produces `road` messages from the true lane geometry, the real road-geometry tracker and
the real fusion turn them into `lanes` and `view` (with `view.road`). The result is compared with the truth and can be
written as a recording for a consumer to render.

    # the built-in test network: a bend, a signal-controlled crossroads, a roundabout, a side road
    tools/roadgeom_drive.py --network tour --out /tmp/tour.jsonl.gz
    # a route on a real pack
    tools/roadgeom_drive.py --pack P --from LAT,LON --to LAT,LON --out /tmp/drive.jsonl.gz
    # a turn signal: T0:T1:left|right (seconds of the drive)
    tools/roadgeom_drive.py --network tour --turn 62:70:right

The synthetic camera is deliberately limited: it sees the ego lane for `--see` metres at most and not round
a bend sharper than its field of view, its far points are noisy, and it loses the lane lines on a roundabout
and in the last metres before it (the planned path stays). The recording holds `t<TAB>json` lines (gzip) of
the topics `map`, `fused`, `ego`, `road`, `lanes`, `view`; positions are those of the synthetic network
or of the route given.
"""
import argparse
import gzip
import json
import math
import os
import random
import sys
import tempfile

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(REPO, "mapd"))
sys.path.insert(0, os.path.join(REPO, "chris-integrator"))

import numpy as np                                        # noqa: E402

from chris_integrator.trackers.fusion import Fusion                   # noqa: E402
from chris_integrator.trackers.lanes import RoadGeometryTracker       # noqa: E402
from chris_integrator.trackers.roadgeom import RoadGeometryFusion     # noqa: E402
from chris_integrator.trackers.egolane import EgoLaneEstimator        # noqa: E402
from chris_integrator.trackers.view import ViewBuilder                # noqa: E402
from roadsense_mapd import geo, rsmap as R, sim           # noqa: E402
from roadsense_mapd.engine import Engine                  # noqa: E402

ORIGIN = (52.0, 19.0)       # an arbitrary point; the network is made up
X_IDXS = [192.0 * (i / 32.0) ** 2 for i in range(33)]


def ll(x, y):
    return geo.offset(ORIGIN[0], ORIGIN[1], y, x)


def _edge(a, b, xy, cls, **kw):
    lat, lon = zip(*[ll(x, y) for x, y in xy])
    return R.RawEdge(a, b, R.pts_from_latlon(lat, lon), R.CLASS_ID[cls], **kw)


class Walk:
    """Lay out a road by walking: straight pieces and arcs from a pose (x east, y north, heading from east, ccw)."""

    def __init__(self, x, y, heading_deg):
        self.x, self.y, self.h = x, y, math.radians(heading_deg)

    def here(self):
        return (self.x, self.y)

    def straight(self, length, step=50.0):
        pts = [self.here()]
        n = max(1, int(length / step))
        for _ in range(n):
            self.x += length / n * math.cos(self.h)
            self.y += length / n * math.sin(self.h)
            pts.append(self.here())
        return pts

    def arc(self, radius, angle_deg, step_deg=6.0):
        """angle > 0 turns left."""
        pts = [self.here()]
        n = max(1, int(abs(angle_deg) / step_deg))
        da = math.radians(angle_deg) / n
        for _ in range(n):
            chord = 2.0 * radius * math.sin(abs(da) / 2.0)
            self.h += da / 2.0
            self.x += chord * math.cos(self.h)
            self.y += chord * math.sin(self.h)
            self.h += da / 2.0
            pts.append(self.here())
        return pts

    def side(self, angle_deg, length):
        """End point of a road leaving here at angle_deg from the heading (positive = left)."""
        a = self.h + math.radians(angle_deg)
        return [self.here(), (self.x + 0.5 * length * math.cos(a), self.y + 0.5 * length * math.sin(a)),
                (self.x + length * math.cos(a), self.y + length * math.sin(a))]


def tour_network():
    """(edges, start, goal): 2.2 km of made-up road with one of everything the picture has to show."""
    FT = R.FEATURE_ID
    main = dict(name="Main", ref="R1", lanes=2)
    ed = []
    w = Walk(-700.0, 0.0, 0.0)
    start = w.here()
    p = w.straight(320.0) + w.arc(180.0, 55.0)[1:] + w.straight(180.0)[1:]
    ed.append(_edge("a0", "x1", p, "secondary", feats=[(len(p) - 1, FT["traffic_signals"], R.DIR_BOTH)], **main))
    ed.append(_edge("x1", "x1l", w.side(88.0, 220.0), "tertiary", name="Cross"))
    ed.append(_edge("x1", "x1r", w.side(-92.0, 220.0), "tertiary", name="Cross"))
    ed.append(_edge("x1", "x1s", w.side(-40.0, 60.0), "service", name="Yard"))
    p = w.straight(260.0)
    ed.append(_edge("x1", "rw", p, "secondary", feats=[(len(p) - 1, FT["give_way"], R.DIR_FORWARD)], **main))
    # roundabout, radius 14 m, centre straight ahead; nodes at the entry (w), then south, east, north in driving order
    r = 14.0
    cx, cy = w.x + r * math.cos(w.h), w.y + r * math.sin(w.h)
    h0 = math.degrees(w.h)

    def on_ring(a):                       # a: degrees from the entry direction, counter-clockwise seen from above
        t = math.radians(h0 + 180.0 + a)
        return (cx + r * math.cos(t), cy + r * math.sin(t))

    def ring_arc(a0, a1):
        n = max(2, int((a1 - a0) / 12.0))
        return [on_ring(a0 + (a1 - a0) * k / n) for k in range(n + 1)]

    ring = dict(flags=R.F_ONEWAY | R.F_ROUNDABOUT, name="Rondo")
    names = ["rw", "rs", "re", "rn"]
    for k in range(4):
        ed.append(_edge(names[k], names[(k + 1) % 4], ring_arc(90.0 * k, 90.0 * (k + 1)), "secondary", **ring))

    def spoke(a, length):
        t = math.radians(h0 + 180.0 + a)
        x0, y0 = on_ring(a)
        return [(x0, y0), (x0 + 0.5 * length * math.cos(t), y0 + 0.5 * length * math.sin(t)),
                (x0 + length * math.cos(t), y0 + length * math.sin(t))]

    ed.append(_edge("rs", "rs1", spoke(90.0, 200.0), "residential", name="South"))
    ed.append(_edge("rn", "rn1", spoke(270.0, 200.0), "tertiary", name="North"))
    w.x, w.y = on_ring(180.0)
    p = w.straight(200.0) + w.arc(110.0, -70.0)[1:] + w.straight(220.0)[1:]
    ed.append(_edge("re", "t1", p, "secondary", **main))
    ed.append(_edge("t1", "t1r", w.side(-85.0, 180.0), "residential", name="Side"))
    p = w.straight(330.0)
    ed.append(_edge("t1", "end", p, "secondary", **main))
    return ed, start, w.here()


def build_network(name):
    if name != "tour":
        sys.exit("unknown network %r (known: tour)" % name)
    ed, start, goal = tour_network()
    path = os.path.join(tempfile.mkdtemp(prefix="roadgeom-drive-"), name + ".rsmap")
    R.build_pack(path, ed, {"region": "synthetic", "osm_timestamp": "", "built": "roadgeom_drive",
                            "attribution": "made-up test network"})
    return R.Pack(path), ll(*start), ll(*goal)


class Camera:
    """The synthetic camera: `road` messages from the true lane centre ahead of the car."""

    def __init__(self, drive, see_m=70.0, seed=3, lane_w=3.4):
        self.d, self.see, self.w = drive, see_m, lane_w
        self.rnd = random.Random(seed)
        self.seq = 0
        self.wob = [0.0, 0.0]
        self.ss = np.arange(0.0, drive.length, 1.0)          # the driven line, a point every metre
        self.px, self.py = np.interp(self.ss, drive.s, drive.x), np.interp(self.ss, drive.s, drive.y)

    def centre(self, o):
        """True lane centre ahead in the car's frame: (x forward, y right) arrays along the path."""
        d = self.d
        i0 = max(int(o.path_s) - 3, 0)
        i1 = int(o.path_s + 460.0) + 1
        loc = d.local
        px = (o.lon - loc.lon0) * loc.mlon
        py = (o.lat - loc.lat0) * loc.mlat
        th = math.radians(o.heading)                     # clockwise from north
        ex, ny = self.px[i0:i1] - px, self.py[i0:i1] - py
        return ex * math.sin(th) + ny * math.cos(th), ex * math.cos(th) - ny * math.sin(th)

    def road(self, o, ts_ns, on_ring, ring_in):
        fx, ry = self.centre(o)
        ok = fx > -1.0
        fx, ry = fx[ok], ry[ok]
        # visible as far as the road runs forward within the field of view (+-35 degrees) and within `see`
        n = 1
        while n < len(fx) and fx[n] > fx[n - 1] + 0.05 and abs(ry[n]) < 0.7 * max(fx[n], 6.0) + 3.0 and fx[n] < self.see:
            n += 1
        vis = float(fx[n - 1]) if n >= 2 else 0.0
        x = np.asarray(X_IDXS)
        if n >= 2:
            y = np.interp(x, fx[:n], ry[:n])
            slope = (ry[n - 1] - ry[n - 2]) / max(fx[n - 1] - fx[n - 2], 0.1)
            far = x > vis
            y[far] = ry[n - 1] + slope * (x[far] - vis) * 0.5      # beyond what is seen: a guess that flattens out
        else:
            y = np.zeros(len(x))
        std = 0.12 + 0.010 * x + np.where(x > vis, 0.6 + 0.05 * (x - vis), 0.0)
        for k in range(2):                               # slow wobble of the far end, as a network's output has
            self.wob[k] = 0.9 * self.wob[k] + self.rnd.gauss(0.0, 0.1)
        y = y + self.wob[0] * (x / 100.0) + self.wob[1] * (x / 100.0) ** 2
        lost = on_ring or (ring_in is not None and ring_in < 14.0) or vis < 12.0
        p_in = 0.06 if lost else 0.93
        hw = 0.5 * self.w

        def line(off, prob, extra=0.0):
            noise = np.array([self.rnd.gauss(0.0, 0.03 + 0.004 * v) for v in x])
            return {"x": [round(v, 2) for v in X_IDXS], "y": np.round(y + off + noise, 3).tolist(),
                    "y_std": np.round(std + extra, 3).tolist(), "prob": prob}

        plan_n = max(12.0, min(8.0 * o.v, 120.0))
        m = fx <= plan_n
        k = max(int(m.sum()), 2)
        px = np.linspace(0.0, float(fx[min(k, len(fx)) - 1]), 33)
        self.seq += 1
        return {"type": "road", "seq": self.seq, "ts_mono_ns": ts_ns, "ts_real_ns": ts_ns, "valid": True,
                "v_ego": round(o.v, 3), "model_ms": 0.0, "frames_skipped": 0,
                "pose": {"vx": round(o.v, 3), "vy": 0.0, "vz": 0.0, "yaw_rate": round(-o.yaw_ccw, 4)},
                "lane_lines": [line(-hw - self.w, 0.75 if not lost else 0.05), line(-hw, p_in), line(hw, p_in),
                               line(hw + self.w, 0.04)],
                "road_edges": [line(-hw - self.w - 0.6, 1.0, 0.3), line(hw + 0.7, 1.0, 0.3)],
                "leads": [],
                "plan": {"x": np.round(px, 2).tolist(), "y": np.round(np.interp(px, fx[:k], ry[:k]), 3).tolist(),
                         "z": [0.0] * 33}}, vis


def truth_error(g, cam, o):
    """Lateral distance (m) of the fused centre line from the true lane centre, by distance along it."""
    c = g.get("centre")
    if not c:
        return {}
    fx, ry = cam.centre(o)
    out = {}
    for s, x, y in zip(c["s"], c["x"], c["y"]):
        d = float(np.min(np.hypot(fx - x, ry - y)))
        b = 50 * int(s // 50)
        out.setdefault(b, []).append(d)
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--network", help="a built-in made-up network: tour")
    ap.add_argument("--pack")
    ap.add_argument("--from", dest="start", metavar="LAT,LON")
    ap.add_argument("--to", dest="goal", metavar="LAT,LON")
    ap.add_argument("--out", help="recording (t<TAB>json lines, gzip)")
    ap.add_argument("--see", type=float, default=70.0, help="how far the synthetic camera sees the lane (m)")
    ap.add_argument("--lateral", type=float, default=1.7, help="the car drives this far right of the way's centre line (m)")
    ap.add_argument("--gnss-sigma", type=float, default=3.0)
    ap.add_argument("--no-gnss", action="store_true", help="no position source: the camera's part alone")
    ap.add_argument("--turn", action="append", default=[], metavar="T0:T1:SIDE", help="turn signal left|right from T0 to T1 s")
    ap.add_argument("--max-v", type=float, default=None, help="cap of the driving speed (m/s)")
    ap.add_argument("--seed", type=int, default=1)
    ap.add_argument("--timeline", action="store_true", help="print one line per second")
    a = ap.parse_args()
    if a.network:
        pack, start, goal = build_network(a.network)
    elif a.pack and a.start and a.goal:
        pack = R.Pack(a.pack, cache_tiles=600)
        start, goal = (tuple(float(v) for v in s.split(",")) for s in (a.start, a.goal))
    else:
        sys.exit("give --network NAME or --pack P --from LAT,LON --to LAT,LON")
    turns = [(float(t0), float(t1), side) for t0, t1, side in (t.split(":") for t in a.turn)]
    path = sim.route(pack, start, goal)
    drive = sim.Drive(pack, path, lateral_m=a.lateral, max_v=a.max_v)
    cam = Camera(drive, a.see, a.seed + 2)
    eng = Engine(pack)
    tracker, fusion, fuse = RoadGeometryTracker(), RoadGeometryFusion(), Fusion(None)
    egolane, view = EgoLaneEstimator(), ViewBuilder()
    out = gzip.open(a.out, "wt") if a.out else None
    t0 = 10 ** 12
    latest = {}
    next_map = next_frame = 0.0
    err = {}
    worst = [0.0, 0.0, 0]
    stats = {"frames": 0, "used": 0, "why": {}, "ring": 0, "branches": 0, "intent": 0}

    def rec(t, msg):
        if out:
            out.write("%.3f\t%s\n" % (t, json.dumps(msg, separators=(",", ":"))))

    kw = dict(seed=a.seed, gnss_sigma_m=a.gnss_sigma, source="synthetic")
    if a.no_gnss:
        kw["gnss_hz"] = 0.0
    for ts, msg, o in sim.feed(drive, t0_ns=t0, tick_hz=20.0, **kw):
        if msg is not None:
            if msg["type"] == "motion":
                side = next((s for t_a, t_b, s in turns if t_a <= o.t < t_b), None)
                if turns:
                    msg["turn_signal"] = side or "off"
            e = eng.on_input(msg, ts)
            if e:
                print("rejected:", e, file=sys.stderr)
            if msg["type"] == "motion" and eng.ego is not None:
                ego = dict(eng.ego, ts_real_ns=ts)
                tracker.on_ego(ego)
                fusion.on_ego(ego)
                egolane.on_ego(ego)
                view.on_ego(ego)
                rec(o.t, ego)
            continue
        if o.t >= next_map:
            next_map = o.t + 0.5
            m = eng.step(ts, ts)
            latest["map"] = m
            tracker.on_map(m)
            fusion.on_map(m)
            egolane.on_map(m)
            view.on_map(m)
            rec(o.t, m)
            f = fuse.update(latest, ts, ts)
            rec(o.t, f)
        if o.t >= next_frame:
            next_frame = o.t + 0.1
            on_ring = bool(o.edge.flags & R.F_ROUNDABOUT)
            mg = (latest.get("map") or {}).get("ahead") or {}
            rg = (mg.get("geometry") or {}).get("ring") or {}
            road, vis = cam.road(o, ts, on_ring, rg.get("entry_m"))
            latest["road"] = road
            rec(o.t, road)
            ln = tracker.on_road(road)
            if ln is None:
                continue
            latest["lanes"] = ln
            rec(o.t, ln)
            g = fusion.on_lanes(ln, road)
            latest["egolane"] = egolane.on_lanes(ln, road)
            rec(o.t, view.on_frame(road, None, ln, g, latest["egolane"]))      # the geometry goes out inside view.road
            stats["frames"] += 1
            stats["used"] += bool(g["map"]["used"])
            why = g["map"]["why"] or "used"
            stats["why"][why] = stats["why"].get(why, 0) + 1
            stats["ring"] += g["ring"] is not None
            stats["branches"] += bool(g["branches"])
            stats["intent"] += (g.get("way") or {}).get("source") == "intent"
            near_ring = g["ring"] is not None and (g["ring"]["on_ring"] or g["ring"]["entry_m"] < 100.0)
            for b, v in (truth_error(g, cam, o).items() if o.path_s < drive.length - 460.0 else ()):
                err.setdefault((near_ring, b), []).extend(v)
                if max(v) > worst[0]:
                    worst[:] = [max(v), o.t, b]
            if a.timeline and abs(o.t - round(o.t)) < 0.05:
                c = g["centre"] or {"s": [0], "src": ""}
                r = g["ring"]
                print("%6.1f s %5.1f km/h  cam %-5s %3.0f m  map %-22s centre to %4.0f m (%s)  branches %d%s%s" % (
                    o.t, o.v * 3.6, g["vision"]["from"], g["vision"]["reach_m"], why, c["s"][-1],
                    "".join(k for i, k in enumerate(c["src"]) if i == 0 or c["src"][i - 1] != k),
                    len(g["branches"]),
                    "" if r is None else "  ring %s r %.0f at %+.0f,%+.0f" % (
                        "ON" if r["on_ring"] else "in %3.0f m" % r["entry_m"], r["radius_m"], r["centre"][0], r["centre"][1]),
                    "" if (g.get("way") or {}).get("source") != "intent" else "  intent %s -> branch %s" % (
                        g["way"]["turn"], g["way"]["branch"])))
    n = max(stats["frames"], 1)
    print("drive %.0f s, %.2f km, %d frames" % (o.t, drive.length / 1000.0, stats["frames"]))
    print("map part used in %.1f %% of the frames; %s" % (100.0 * stats["used"] / n, ", ".join(
        "%s %.1f %%" % (k, 100.0 * v / n) for k, v in sorted(stats["why"].items(), key=lambda kv: -kv[1]))))
    print("frames with a ring %.1f %%, with side roads %.1f %%, with an intent choice %.1f %%" % (
        100.0 * stats["ring"] / n, 100.0 * stats["branches"] / n, 100.0 * stats["intent"] / n))
    print("fused centre line against the true lane centre (lateral, m), by distance along it:")
    for ring, b in sorted(err):
        v = sorted(err[(ring, b)])
        print("  %-9s %3d-%3d m: median %.2f  p95 %.2f  max %.2f  (n=%d)" % (
            "at a ring" if ring else "open road", b, b + 50, v[len(v) // 2], v[int(0.95 * (len(v) - 1))], v[-1], len(v)))
    print("largest error %.1f m at %.1f s (%d m along the line)" % tuple(worst))
    if out:
        out.close()


if __name__ == "__main__":
    main()
