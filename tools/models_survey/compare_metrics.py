#!/usr/bin/env python3
"""Stability / agreement metrics from replay_compare.py outputs (no ground truth).

usage: python -I compare_metrics.py OUT.json RIDE.npz [...]
Metrics per ride x model (20 Hz outputs, first 2 s skipped):
  lane_flick_ego / _outer : presence on/off changes per minute at prob 0.5 (mean over ego lines 1,2 / outer 0,3)
  lane_on_ego             : fraction of frames with ego line prob > 0.5
  lane_j10 / lane_j30     : RMS second difference of ego-line y at x=10 m / 30 m (m), frames where the line is
                            present at t-1, t, t+1 (motion is smooth at 20 Hz, so this is mostly frame noise)
  lane_d10                : median |dy| frame to frame at 10 m (m)
  edge_j10 / edge_j30     : same RMS second difference for the two road edges (no gating)
  lead_flick              : lead_prob[0] on/off changes per minute at 0.5; lead_on: fraction present
  lead_jx                 : RMS second difference of lead-0 distance x (m) while present 3 frames in a row
  lead_jrel               : median |dx|/x frame to frame while present
Agreement (sc097 vs each other model): ego-lane presence agreement, median |dy| at 10/30 m when both present,
edge |dy| at 10 m, lead presence agreement, median |dx| when both see a lead.
"""
import json
import sys

import numpy as np

X = 192.0 * (np.arange(33) / 32.0) ** 2
SKIP = 40


def at(arr, x):  # arr (..., 33) -> value at x m
    i = np.searchsorted(X, x) - 1
    w = (x - X[i]) / (X[i + 1] - X[i])
    return arr[..., i] * (1 - w) + arr[..., i + 1] * w


def flick(on, minutes):
    return np.sum(on[1:] != on[:-1]) / minutes


def j2(y, mask=None):
    d2 = y[2:] - 2 * y[1:-1] + y[:-2]
    if mask is not None:
        m = mask[2:] & mask[1:-1] & mask[:-2]
        d2 = d2[m]
    return float(np.sqrt(np.mean(d2 ** 2))) if d2.size else float("nan")


def model_metrics(z, m):
    lane = z[f"{m}/lane"][SKIP:, :, :, 0]  # (T,4,33) y
    lp = z[f"{m}/lprob"][SKIP:]
    edge = z[f"{m}/edge"][SKIP:, :, :, 0]
    lead = z[f"{m}/lead"][SKIP:, 0]  # lead 0 (t offset 0): x y v a
    ldp = z[f"{m}/lead_prob"][SKIP:, 0]
    T = len(lp)
    minutes = T / 20.0 / 60.0
    on = lp > 0.5
    r = {}
    r["lane_flick_ego"] = float(np.mean([flick(on[:, i], minutes) for i in (1, 2)]))
    r["lane_flick_outer"] = float(np.mean([flick(on[:, i], minutes) for i in (0, 3)]))
    r["lane_on_ego"] = float(on[:, 1:3].mean())
    for x in (10, 30):
        r[f"lane_j{x}"] = float(np.nanmean([j2(at(lane[:, i], x), on[:, i]) for i in (1, 2)]))
        r[f"edge_j{x}"] = float(np.mean([j2(at(edge[:, i], x)) for i in (0, 1)]))
    d = []
    for i in (1, 2):
        y = at(lane[:, i], 10)
        mm = on[1:, i] & on[:-1, i]
        d.append(np.abs(np.diff(y))[mm])
    d = np.concatenate(d)
    r["lane_d10"] = float(np.median(d)) if d.size else float("nan")
    lon = ldp > 0.5
    r["lead_flick"] = float(flick(lon, minutes))
    r["lead_on"] = float(lon.mean())
    r["lead_jx"] = j2(lead[:, 0], lon)
    mm = lon[1:] & lon[:-1]
    rel = np.abs(np.diff(lead[:, 0]))[mm] / np.maximum(lead[1:, 0][mm], 1.0)
    r["lead_jrel"] = float(np.median(rel)) if rel.size else float("nan")
    r["frames"] = T
    return r


def agreement(z, a, b):
    la, lb = z[f"{a}/lane"][SKIP:, :, :, 0], z[f"{b}/lane"][SKIP:, :, :, 0]
    pa, pb = z[f"{a}/lprob"][SKIP:] > 0.5, z[f"{b}/lprob"][SKIP:] > 0.5
    r = {"ego_presence_agree": float((pa[:, 1:3] == pb[:, 1:3]).mean()),
         "ego_on_only_" + a: float((pa[:, 1:3] & ~pb[:, 1:3]).mean()),
         "ego_on_only_" + b: float((~pa[:, 1:3] & pb[:, 1:3]).mean())}
    for x in (10, 30):
        ds = [np.abs(at(la[:, i], x) - at(lb[:, i], x))[pa[:, i] & pb[:, i]] for i in (1, 2)]
        ds = np.concatenate(ds)
        r[f"ego_dy{x}_med"] = float(np.median(ds)) if ds.size else float("nan")
        r[f"ego_dy{x}_p90"] = float(np.percentile(ds, 90)) if ds.size else float("nan")
    ea, eb = z[f"{a}/edge"][SKIP:, :, :, 0], z[f"{b}/edge"][SKIP:, :, :, 0]
    r["edge_dy10_med"] = float(np.median(np.abs(at(ea, 10) - at(eb, 10))))
    qa, qb = z[f"{a}/lead_prob"][SKIP:, 0] > 0.5, z[f"{b}/lead_prob"][SKIP:, 0] > 0.5
    r["lead_presence_agree"] = float((qa == qb).mean())
    both = qa & qb
    xa, xb = z[f"{a}/lead"][SKIP:, 0, 0], z[f"{b}/lead"][SKIP:, 0, 0]
    r["lead_dx_med"] = float(np.median(np.abs(xa - xb)[both])) if both.any() else float("nan")
    r["lead_dx_rel_med"] = float(np.median((np.abs(xa - xb) / np.maximum(xa, 1))[both])) if both.any() else float("nan")
    return r


def main():
    res = {}
    for p in sys.argv[2:]:
        z = np.load(p)
        meta = json.loads(str(z["meta"]))
        ride = meta["ride"]
        res[ride] = {"meta": meta, "models": {}, "agree": {}}
        for m in meta["models"]:
            res[ride]["models"][m] = model_metrics(z, m)
        for m in meta["models"]:
            if m != "sc097":
                res[ride]["agree"][f"sc097~{m}"] = agreement(z, "sc097", m)
        if "v0111" in meta["models"] and "v0111bl" in meta["models"]:
            res[ride]["agree"]["v0111~v0111bl"] = agreement(z, "v0111", "v0111bl")
    json.dump(res, open(sys.argv[1], "w"), indent=1)
    keys = ["lane_on_ego", "lane_flick_ego", "lane_flick_outer", "lane_d10", "lane_j10", "lane_j30", "edge_j10",
            "edge_j30", "lead_on", "lead_flick", "lead_jx", "lead_jrel"]
    print("ride\tmodel\t" + "\t".join(keys))
    for ride, r in res.items():
        for m, mm in r["models"].items():
            print(f"{ride}\t{m}\t" + "\t".join(f"{mm[k]:.3g}" for k in keys))
    print()
    for ride, r in res.items():
        for pair, a in r["agree"].items():
            print(f"{ride}\t{pair}\t" + "  ".join(f"{k}={v:.3g}" for k, v in a.items()))


if __name__ == "__main__":
    main()
