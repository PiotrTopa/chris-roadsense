#!/usr/bin/env python3
"""Who was right at 45 m: the overlay picks judged by what both models see later at 5-15 m (lane L24-M).

    python tools/vision_eval/adjudicate.py ROAD_DIR OVERLAY_DIR [--a sc.13.8] [--b vis.13.8]

For every pick in OVERLAY_DIR/*.csv with a `lineN` reason: the tracked line N of each model at 45 m is carried
with the common ego motion (mean pose of sc.13.8 and vis.20, as in evaluate.py) until it is ~10 m ahead, and
compared with the near-range line there (mean of both models' tracked line where both have it, else the one
present). The near range is where the two models agree (median 0.1-0.2 m at 10 m), so this is a reference
neither model owns. Prints per pick |error| of A and B and the winner (> 0.25 m apart).
"""
import argparse
import csv
import glob
import math
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import evaluate as ev  # noqa: E402


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("road_dir"); ap.add_argument("ov_dir")
    ap.add_argument("--a", default="sc.13.8"); ap.add_argument("--b", default="vis.13.8")
    a = ap.parse_args()
    picks = {}
    for f in glob.glob(os.path.join(a.ov_dir, "*.csv")):
        ride = os.path.basename(f)[:-4].rstrip("L")
        for r in csv.DictReader(open(f)):
            if r["why"].startswith("line"):
                picks.setdefault(ride, set()).add((int(r["seq"]), int(r["why"][4])))
    wins = {a.a: 0, a.b: 0, "tie": 0}
    for ride, ps in sorted(picks.items()):
        names = {a.a, a.b, "sc.13.8", "vis.20" if os.path.exists(os.path.join(a.road_dir, f"{ride}.vis.20.jsonl.gz"))
                 else "vis.17"}
        S = {s: ev.load(os.path.join(a.road_dir, f"{ride}.{s}.jsonl.gz")) for s in names}
        t_e, Xe, Ye, Pe, se = ev.ego_track([S.get("vis.20", S.get("vis.17")), S["sc.13.8"]])
        for seq, line in sorted(ps):
            res = {}
            for name in (a.a, a.b):
                M = S[name]
                i = int(np.argmin(np.abs(M["seq"] - seq)))
                y = M["tr_y"][i, line]
                if not np.isfinite(y).all():
                    res[name] = None
                    continue
                ti = M["t"][i]
                y0 = float(np.interp(45.0, ev.X, y))
                s0 = np.interp(ti, t_e, se)
                k = int(np.searchsorted(se, s0 + 35.0))
                if k >= len(t_e):
                    res[name] = None
                    continue
                tj = t_e[k]
                P0, P1 = np.interp(ti, t_e, Pe), Pe[k]
                wx = np.interp(ti, t_e, Xe) + 45.0 * math.cos(P0) - y0 * math.sin(P0)
                wy = np.interp(ti, t_e, Ye) + 45.0 * math.sin(P0) + y0 * math.cos(P0)
                dx, dy = wx - Xe[k], wy - Ye[k]
                xp = dx * math.cos(P1) + dy * math.sin(P1); yp = -dx * math.sin(P1) + dy * math.cos(P1)
                near = []
                for nm in (a.a, a.b):
                    N = S[nm]
                    j = int(np.argmin(np.abs(N["t"] - tj)))
                    yy = N["tr_y"][j, line]
                    if abs(N["t"][j] - tj) < 0.1 and np.isfinite(yy).all():
                        near.append(float(np.interp(xp, ev.X, yy)))
                res[name] = abs(np.mean(near) - yp) if near else None
            ea, eb = res[a.a], res[a.b]
            if ea is None or eb is None:
                w = "n/a"
            elif abs(ea - eb) <= 0.25:
                w = "tie"
            else:
                w = a.a if ea < eb else a.b
            if w in wins:
                wins[w] += 1
            print(f"{ride} seq {seq} line{line}: |err| {a.a} {ea if ea is None else round(ea, 2)}  "
                  f"{a.b} {eb if eb is None else round(eb, 2)}  -> {w}")
    print("wins:", wins)


if __name__ == "__main__":
    main()
