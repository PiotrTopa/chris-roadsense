#!/usr/bin/env python3
"""Replay recorded messages through the road-geometry tracker and the road-geometry fusion (api `lanes` and
`roadgeom`) and report what the fused geometry is made of.

    tools/roadgeom_replay.py SESSION.jsonl[.gz] [--out ROADGEOM.jsonl] [--every N]

The input is a drive recorder session (`road`, `ego`, `map` lines; a gzip stream that ends in a broken block
is read up to its last complete line). Without a position source in the recording the result is the camera's
part alone: the report then says how far the camera's centre line reaches and where it comes from.
"""
import argparse
import collections
import json
import os
import sys
import time
import zlib

sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "chris-integrator"))

from chris_integrator.trackers.lanes import RoadGeometryTracker       # noqa: E402
from chris_integrator.trackers.roadgeom import RoadGeometryFusion     # noqa: E402


def lines(path):
    """JSON lines of a plain or gzip file; a truncated gzip stream yields what is complete."""
    with open(path, "rb") as f:
        if path.endswith(".gz"):
            d, buf = zlib.decompressobj(31), b""
            while True:
                chunk = f.read(1 << 20)
                if not chunk:
                    break
                try:
                    buf += d.decompress(chunk)
                except zlib.error:
                    break
                *full, buf = buf.split(b"\n")
                yield from full
        else:
            yield from f


def pct(v, q):
    v = sorted(v)
    return v[min(len(v) - 1, int(q * len(v)))] if v else float("nan")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("session")
    ap.add_argument("--out")
    ap.add_argument("--every", type=int, default=1, help="use every Nth road message")
    a = ap.parse_args()
    tracker, fusion = RoadGeometryTracker(), RoadGeometryFusion()
    out = open(a.out, "w") if a.out else None
    n = 0
    src, why = collections.Counter(), collections.Counter()
    reach, moving_reach, cost = [], [], []
    used = 0
    for raw in lines(a.session):
        if raw[:1] == b"{" or b"\t" not in raw[:24]:
            body = raw
        else:
            body = raw.split(b"\t", 1)[1]                # "t<TAB>json" recordings
        try:
            m = json.loads(body)
        except ValueError:
            continue
        t = m.get("type")
        if t == "ego":
            tracker.on_ego(m)
            fusion.on_ego(m)
        elif t == "map":
            tracker.on_map(m)
            fusion.on_map(m)
        elif t == "road":
            n += 1
            if n % a.every:
                continue
            ln = tracker.on_road(m)
            if ln is None:
                continue
            t0 = time.perf_counter()
            g = fusion.on_lanes(ln, m)
            cost.append((time.perf_counter() - t0) * 1e3)
            if g is None:
                continue
            src[g["vision"]["from"]] += 1
            why[g["map"]["why"] or "used"] += 1
            used += bool(g["map"]["used"])
            reach.append(g["vision"]["reach_m"])
            if g["motion"]["speed_mps"] > 3.0:
                moving_reach.append(g["vision"]["reach_m"])
            if out:
                out.write(json.dumps(g, separators=(",", ":")) + "\n")
    tot = sum(src.values()) or 1
    print("road messages %d, geometry frames %d" % (n, tot))
    print("camera centre from: " + ", ".join("%s %.1f %%" % (k, 100.0 * v / tot) for k, v in src.most_common()))
    print("camera reach (m): median %.0f, p10 %.0f, p90 %.0f; while driving: median %.0f, p10 %.0f, p90 %.0f"
          % (pct(reach, 0.5), pct(reach, 0.1), pct(reach, 0.9), pct(moving_reach, 0.5), pct(moving_reach, 0.1),
             pct(moving_reach, 0.9)))
    print("map part: used in %.1f %%; otherwise: %s" % (100.0 * used / tot, ", ".join(
        "%s %.1f %%" % (k, 100.0 * v / tot) for k, v in why.most_common() if k != "used") or "-"))
    print("fusion cost per message (ms): median %.3f, p95 %.3f" % (pct(cost, 0.5), pct(cost, 0.95)))


if __name__ == "__main__":
    main()
