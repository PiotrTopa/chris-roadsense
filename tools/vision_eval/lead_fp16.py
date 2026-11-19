#!/usr/bin/env python3
"""lead_fp16.py REF.jsonl[.gz] RUN.jsonl[.gz] [--name RIDE] [--json OUT]      (lane L28)

The new road model's lead through two arithmetic paths on the same frames (modeld_replay --rates streams of the
same decoded frames: REF = fp32, RUN = fp16 as deployed), before and after the api's lead tracker with the
model's own profile (`road_model` "vision" selects it).

Before the tracker (leads[0] at t=0, frames where both have prob >= 0.5): |x_run - x_ref| mean / p95 / max (m) and
relative to the distance; |v_run - v_ref| of the lead speed; presence (prob >= 0.5) disagreement.
After the tracker, on the frames both streams ran:
  dist error   |d_run - d_ref| where both present: mean / median / p95 / max (m), and relative (%)
  vrel error   the same for the relative speed (m/s)
  presence     % of runs on which one shows a lead and the other not (run only / ref only)
  per stream   on/off and identity changes, distance jumps (> max(1.5 m, 8 %)) per minute, per-frame step after the
               relative speed (rms, p95, over 0.5 m per minute), distance noise per band (lead_eval.py's measures)
  id events    identity changes of RUN with no identity change of REF within +-0.5 s, and the reverse
"""
import gzip
import json
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import lead_eval  # noqa: E402
from lead_eval import DEFAULT_CFG  # noqa: E402


def read(path):
    op = gzip.open if path.endswith(".gz") else open
    out = []
    with op(path, "rt") as f:
        for ln in f:
            if not ln.startswith("{"):
                continue
            m = json.loads(ln)
            if m.get("type") == "road":
                out.append(m)
    return out


def stats(v):
    v = np.asarray(v, float)
    v = v[np.isfinite(v)]
    if not len(v):
        return {"n": 0}
    return {"n": int(len(v)), "mean": round(float(v.mean()), 3), "median": round(float(np.median(v)), 3),
            "p95": round(float(np.percentile(v, 95)), 3), "max": round(float(v.max()), 3)}


def id_events(outs, t):
    ev, last = [], None
    for o, ti in zip(outs, t):
        if o["present"]:
            if last is not None and o["track_id"] != last:
                ev.append(ti)
            last = o["track_id"]
    return np.array(ev)


def unmatched(a, b, tol=0.5):
    return int(sum(1 for x in a if not len(b) or np.min(np.abs(b - x)) > tol))


def main():
    args = [a for a in sys.argv[1:]]
    name = args[args.index("--name") + 1] if "--name" in args else ""
    jout = args[args.index("--json") + 1] if "--json" in args else None
    files = [a for i, a in enumerate(args) if not a.startswith("--") and (i == 0 or args[i - 1] not in ("--name", "--json"))]
    ref, run = read(files[0]), read(files[1])
    rs = {m["seq"]: m for m in run}
    pairs = [(a, rs[a["seq"]]) for a in ref if a["seq"] in rs]
    ra = [p[0] for p in pairs]
    rb = [p[1] for p in pairs]
    res = {"ride": name, "frames": len(pairs), "ref_only": len(ref) - len(pairs), "run_only": len(run) - len(pairs)}
    # before the tracker
    dx, dxr, dv, dis, both = [], [], [], 0, 0
    for a, b in pairs:
        la, lb = a["leads"][0], b["leads"][0]
        pa, pb = la["prob"] >= 0.5, lb["prob"] >= 0.5
        dis += pa != pb
        if pa and pb:
            both += 1
            xa, xb = la["x"][0], lb["x"][0]
            dx.append(abs(xb - xa))
            dxr.append(100.0 * abs(xb - xa) / max(1.0, xa))
            dv.append(abs(lb["v"][0] - la["v"][0]))
    res["raw"] = {"dist_m": stats(dx), "dist_pct": stats(dxr), "lead_speed_mps": stats(dv),
                  "presence_disagree_pct": round(100.0 * dis / max(1, len(pairs)), 2), "both_present": both}
    # after the tracker
    oa = lead_eval.run(ra, DEFAULT_CFG)
    ob = lead_eval.run(rb, DEFAULT_CFG)
    t = np.array([m["ts_mono_ns"] * 1e-9 for m in ra])
    de, der, ve, only_a, only_b = [], [], [], 0, 0
    for x, y in zip(oa, ob):
        if x["present"] and y["present"]:
            de.append(abs(y["dist_m"] - x["dist_m"]))
            der.append(100.0 * abs(y["dist_m"] - x["dist_m"]) / max(1.0, x["dist_m"]))
            ve.append(abs(y["v_rel_mps"] - x["v_rel_mps"]))
        elif x["present"]:
            only_a += 1
        elif y["present"]:
            only_b += 1
    n = max(1, len(oa))
    res["tracked"] = {"dist_m": stats(de), "dist_pct": stats(der), "vrel_mps": stats(ve),
                      "presence_disagree_pct": round(100.0 * (only_a + only_b) / n, 2),
                      "run_only_pct": round(100.0 * only_b / n, 2), "ref_only_pct": round(100.0 * only_a / n, 2)}
    sa, _ = lead_eval.score(ra, oa)
    sb, _ = lead_eval.score(rb, ob)
    keep = ("onoff", "id", "steps", "fstep_rms", "fstep_p95", "fstep_over", "present_pct", "unconf_pct",
            "d_noise_15-30", "d_noise_30-60", "d_noise_60-200", "v_noise", "follow_pct", "follow_p90_pct")
    mins = sa["minutes"]
    per = {}
    for tag, s in (("ref", sa), ("run", sb)):
        d = {}
        for k in keep:
            v = s[k]
            if k in ("onoff", "id", "steps", "fstep_over"):
                v = v / mins
            d[k] = None if v != v else round(float(v), 3)
        per[tag] = d
    res["minutes"] = round(mins, 2)
    res["per_stream"] = per
    ea, eb = id_events(oa, t), id_events(ob, t)
    res["id_events"] = {"ref": int(len(ea)), "run": int(len(eb)), "run_not_in_ref": unmatched(eb, ea),
                        "ref_not_in_run": unmatched(ea, eb)}
    print(json.dumps(res, indent=1))
    if jout:
        with open(jout, "w") as f:
            json.dump(res, f, indent=1)


if __name__ == "__main__":
    main()
