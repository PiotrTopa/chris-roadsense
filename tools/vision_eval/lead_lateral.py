#!/usr/bin/env python3
"""Lateral place of the model's lead against its own lane lines and plan, by distance (lane L32).

  usage: lead_lateral.py ROAD.jsonl[.gz] ... [--prob 0.7] [--line-prob 0.5] [--bins 15,30,45,60,75,90,105]
                         [--straight K] [--json OUT]

For every `road` message with leads[0].prob >= --prob: the lead's lateral place y (m, + right) minus
  * the centre of the ego lane (lines 1 and 2, both prob >= --line-prob) at the lead's distance,
  * the plan's y at the lead's distance (where the plan reaches that far),
  * nothing (the raw y).
Reported per file: the median per distance bin and the angle (deg) = median of atan(dy / x) over the frames
(+ = the lead is to the right of the reference), with a bootstrap over 10 s blocks for its spread, the direction
of the model's `pose` on the runs modeld's online calibration accepts (the calibration's yaw error), and the
per-ride split by the lane's curvature at the lead (--straight K: only frames where the lane centre departs
from its tangent at 0 m by less than K m at the lead's distance).
"""
import argparse
import gzip
import json
import math
import random
import sys


def opn(p):
    return gzip.open(p, "rt") if p.endswith(".gz") else open(p)


def interp(xs, ys, x):
    if x < xs[0] or x > xs[-1]:
        return None
    lo, hi = 0, len(xs) - 1
    while hi - lo > 1:
        m = (lo + hi) // 2
        if xs[m] <= x:
            lo = m
        else:
            hi = m
    if xs[hi] == xs[lo]:
        return ys[lo]
    f = (x - xs[lo]) / (xs[hi] - xs[lo])
    return ys[lo] + f * (ys[hi] - ys[lo])


def med(v):
    v = sorted(v)
    n = len(v)
    if not n:
        return float("nan")
    return v[n // 2] if n % 2 else 0.5 * (v[n // 2 - 1] + v[n // 2])


POSE = {}  # path -> direction of `pose` (deg) on the runs modeld's online calibration would accept


def rows(path, prob, line_prob):
    out = []
    pd = POSE.setdefault(path, [])
    for ln in opn(path):
        if '"road"' not in ln[:24]:
            continue
        m = json.loads(ln)
        if m.get("type") != "road":
            continue
        q = m.get("pose")
        if q and q["vx"] >= 6.7 and abs(q["yaw_rate"]) <= 0.01745 and math.atan2(q["vy_std"], q["vx"]) <= 0.00873:
            pd.append(math.degrees(math.atan2(q["vy"], q["vx"])))
        L = (m.get("leads") or [None])[0]
        if not L or L["prob"] < prob:
            continue
        x, y = L["x"][0], L["y"][0]
        if x < 5:
            continue
        r = {"t": m["ts_mono_ns"] * 1e-9, "x": x, "y": y, "v": m.get("v_ego") or 0.0, "lane": None, "plan": None,
             "bend": None, "ystd": L["y_std"][0]}
        ll = m.get("lane_lines") or []
        if len(ll) >= 3 and ll[1]["prob"] >= line_prob and ll[2]["prob"] >= line_prob:
            a = interp(ll[1]["x"], ll[1]["y"], x)
            b = interp(ll[2]["x"], ll[2]["y"], x)
            if a is not None and b is not None:
                c = 0.5 * (a + b)
                r["lane"] = y - c
                # departure of the lane centre from its tangent over the first 10 m, at the lead's distance
                c0 = 0.5 * (ll[1]["y"][0] + ll[2]["y"][0])
                c10 = 0.5 * (interp(ll[1]["x"], ll[1]["y"], 10.0) + interp(ll[2]["x"], ll[2]["y"], 10.0))
                r["bend"] = c - (c0 + (c10 - c0) * x / 10.0)
                r["lane_c"] = c
        p = m.get("plan")
        if p and p.get("x"):
            py = interp(p["x"], p["y"], x)
            if py is not None:
                r["plan"] = y - py
                r["plan_y"] = py
        out.append(r)
    return out


def angle_stats(rs, key, rng):
    v = [(r["t"], math.degrees(math.atan2(r[key], r["x"]))) for r in rs if r.get(key) is not None]
    if len(v) < 20:
        return None
    a = med([d for _, d in v])
    blocks = {}
    for t, d in v:
        blocks.setdefault(int(t // 10), []).append(d)
    bl = list(blocks.values())
    bs = []
    for _ in range(200):
        s = []
        for _ in bl:
            s += rng.choice(bl)
        bs.append(med(s))
    bs.sort()
    return {"deg": a, "lo": bs[int(0.05 * len(bs))], "hi": bs[int(0.95 * len(bs))], "n": len(v), "blocks": len(bl)}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("files", nargs="+")
    ap.add_argument("--prob", type=float, default=0.7)
    ap.add_argument("--line-prob", type=float, default=0.5)
    ap.add_argument("--bins", default="15,30,45,60,75,90,105")
    ap.add_argument("--min-x", type=float, default=20.0, help="angle from leads at least this far (m)")
    ap.add_argument("--straight", type=float, default=None)
    ap.add_argument("--min-v", type=float, default=None)
    ap.add_argument("--json")
    a = ap.parse_args()
    edges = [float(x) for x in a.bins.split(",")]
    rng = random.Random(1)
    res = {}
    for f in a.files:
        rs = rows(f, a.prob, a.line_prob)
        if a.straight is not None:
            rs = [r for r in rs if r["bend"] is not None and abs(r["bend"]) < a.straight]
        if a.min_v is not None:
            rs = [r for r in rs if r["v"] >= a.min_v]
        name = f.split("/")[-1].replace(".jsonl.gz", "").replace(".jsonl", "")
        out = {"n": len(rs), "bins": [], "angle": {}}
        print(f"== {name}: {len(rs)} lead frames")
        print("   bin m      n   y-lane  n   y-plan  n    y     |bend|")
        for lo, hi in zip(edges, edges[1:]):
            b = [r for r in rs if lo <= r["x"] < hi]
            l = [r["lane"] for r in b if r["lane"] is not None]
            p = [r["plan"] for r in b if r["plan"] is not None]
            y = [r["y"] for r in b]
            bd = [abs(r["bend"]) for r in b if r["bend"] is not None]
            out["bins"].append({"lo": lo, "hi": hi, "n_lane": len(l), "lane": med(l), "n_plan": len(p), "plan": med(p),
                                "n": len(y), "y": med(y)})
            print(f"   {lo:3.0f}-{hi:3.0f} {len(l):5d} {med(l):+7.2f} {len(p):5d} {med(p):+7.2f} {len(y):5d} {med(y):+6.2f} {med(bd):6.2f}")
        far = [r for r in rs if r["x"] >= a.min_x]
        for key in ("lane", "plan", "y"):
            s = angle_stats(far, key, rng)
            out["angle"][key] = s
            if s:
                print(f"   angle lead - {key:4s} (x >= {a.min_x:.0f} m): {s['deg']:+.2f} deg  [{s['lo']:+.2f}, {s['hi']:+.2f}] 90 % block bootstrap, "
                      f"{s['n']} frames, {s['blocks']} blocks of 10 s")
        if len(POSE[f]) >= 100:
            out["pose_dir"] = med(POSE[f])
            print(f"   direction of pose on straight fast runs: {out['pose_dir']:+.2f} deg ({len(POSE[f])} runs): "
                  f"the calibration's yaw is off by this; expected lead - lane = {-0.95 * out['pose_dir']:+.2f} deg (vision)")
        res[name] = out
    if a.json:
        json.dump(res, open(a.json, "w"), indent=1)


if __name__ == "__main__":
    sys.exit(main())
