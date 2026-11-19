#!/usr/bin/env python3
"""Effect of --policy-hz on the plan: the plan held since the last policy run against the plan of this run, as a
path y(x) at 10 / 30 / 60 m; raw hold and hold carried with the model's own motion (pose vx, yaw rate).

    python tools/vision_eval/plan_hold.py ROAD_DIR RIDE [RIDE ...]     (ROAD_DIR: <ride>.vis.20.jsonl.gz, lane L27)"""
import gzip, json, sys, math
import numpy as np
D = sys.argv[1]
XS = (10.0, 30.0, 60.0)
def load(ride):
    out = []
    with gzip.open(f"{D}/{ride}.vis.20.jsonl.gz", "rt") as f:
        for ln in f:
            m = json.loads(ln)
            if m.get("type") == "road" and "plan" in m:
                p = m["plan"]
                out.append((m["ts_mono_ns"]*1e-9, np.array(p["x"]), np.array(p["y"]), m["pose"]["vx"], m["pose"]["yaw_rate"]))
    return out
def yat(px, py, x):
    return np.interp(x, px, py) if px[-1] >= x else np.nan
res = {}
for ride in sys.argv[2:]:
    R = load(ride)
    for hz in (1, 2, 5, 10):
        held = None; nxt = -1
        for (t, px, py, v, yr) in R:
            if v < 3: held = None; continue
            if held is None or t >= nxt - 0.025:
                held = [t, px, py, 0.0, 0.0, 0.0]; nxt = t + 1.0/hz; last = t; continue
            dt = t - last; last = t
            # integrate ego motion in the held plan's frame (x fwd, y right, yaw positive right)
            held[5] += yr*dt; held[3] += v*math.cos(held[5])*dt; held[4] += v*math.sin(held[5])*dt
            age = t - held[0]
            c, s = math.cos(held[5]), math.sin(held[5])
            hx, hy = held[1] - held[3], held[2] - held[4]
            cx, cy = hx*c + hy*s, -hx*s + hy*c
            for x in XS:
                f = yat(px, py, x)
                if not np.isfinite(f): continue
                res.setdefault((hz, x, "raw"), []).append(abs(yat(held[1], held[2], x) - f))
                res.setdefault((hz, x, "carried"), []).append(abs(np.interp(x, cx, cy) - f))
print("| policy-hz | plan held as is, |dy| at 10 / 30 / 60 m: median (p95) m | carried with the model's motion: median (p95) m |")
print("|---|---|---|")
for hz in (10, 5, 2, 1):
    row = []
    for kind in ("raw", "carried"):
        row.append(" / ".join(f"{np.nanmedian(res[(hz,x,kind)]):.3f} ({np.nanpercentile(res[(hz,x,kind)],95):.2f})" for x in XS))
    print(f"| {hz} | {row[0]} | {row[1]} |")
# frame-to-frame change of the fresh plan itself (20/s) as the noise floor
