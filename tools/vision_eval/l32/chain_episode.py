#!/usr/bin/env python3
"""Pull-away episodes in the MFD rows of a chain run: standing behind a lead (>= 3 s under 0.3 m/s, lead drawn), the lead
leaves, then the own car moves. Prints one line per second: own speed, drawn lead distance, the lead's own lateral, the drawn
road centre at the lead's distance and where it comes from (b both lines, l / r one line, p path), the offset (m, + right).
  usage: chain_episode.py ROWS.jsonl.gz [ROWS2.jsonl.gz ...] [--list] [--t0 S --t1 S]"""
import sys, json, gzip, argparse
ap = argparse.ArgumentParser(); ap.add_argument("rows", nargs="+"); ap.add_argument("--list", action="store_true")
ap.add_argument("--t0", type=float); ap.add_argument("--t1", type=float)
a = ap.parse_args()
def at(xs, ys, x):
    if ys is None: return None
    n = min(len(xs), len(ys))
    for i in range(n - 1):
        if xs[i] <= x <= xs[i + 1] and ys[i] is not None and ys[i + 1] is not None:
            return ys[i] + (x - xs[i]) / (xs[i + 1] - xs[i]) * (ys[i + 1] - ys[i])
def load(p): return [json.loads(l) for l in gzip.open(p, "rt")]
R = [load(p) for p in a.rows]
if a.list:
    rows = R[0]; i = 0; n = len(rows)
    while i < n:
        if rows[i]["v"] < 0.3 and rows[i]["lead"]:
            j = i
            while j < n and rows[j]["v"] < 0.3: j += 1
            dur = rows[j - 1]["t"] - rows[i]["t"]
            ds = [r["lead"][0] for r in rows[i:j] if r["lead"]]
            k = j
            while k < n and rows[k]["t"] < rows[j - 1]["t"] + 15 and rows[k]["v"] < 5: k += 1
            if dur >= 3 and ds and max(ds) - ds[0] > 5 and k < n and rows[k]["v"] >= 5:
                print(f"standing {rows[i]['t']:.0f}-{rows[j-1]['t']:.0f} s, lead from {ds[0]:.0f} m to {max(ds):.0f} m while standing; moving >= 5 m/s at {rows[k]['t']:.0f} s")
            i = j
        else: i += 1
    sys.exit(0)
names = [p.split("/")[-1].replace(".rows.jsonl.gz", "") for p in a.rows]
print("t s    " + " | ".join(f"{nm[-22:]:>40s}" for nm in names)); print("       " + " | ".join("   v m/s  lead m  lead y  centre src  offset" for _ in names))
t = a.t0
while t <= a.t1:
    cells = []
    for rows in R:
        r = min(rows, key=lambda r: abs(r["t"] - t)) if rows else None
        if r is None or abs(r["t"] - t) > 0.2 or not r["lead"]:
            cells.append(f"{(r['v'] if r else 0):8.1f}       -       -       -   -       -"); continue
        d, lat = r["lead"]; c = at(r["xs"], r["cen"], d)
        k = min(range(len(r["xs"])), key=lambda i: abs(r["xs"][i] - d)); src = r["cen_src"][k] if k < len(r["cen_src"]) else "-"
        cells.append(f"{r['v']:8.1f} {d:7.1f} {-lat:+7.2f} " + (f"{-c:+7.2f}   {src}  {-(lat - c):+6.2f}" if c is not None else "      -   -       -"))
    print(f"{t:6.0f} " + " | ".join(cells)); t += 1.0
