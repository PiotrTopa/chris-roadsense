#!/usr/bin/env python3
"""Ego-motion of the two road models: scatter of pose.vx and pose.yaw_rate about a centred 1 s mean, while moving.

    python tools/vision_eval/pose_noise.py LEAD_DIR RIDE [RIDE ...]    (LEAD_DIR: lead_eval.py extract, lane L27)"""
import gzip, json, sys
import numpy as np
def cen(t, v, half=0.5):
    out = np.full(len(v), np.nan); lo = np.searchsorted(t, t-half); hi = np.searchsorted(t, t+half, side="right")
    for i in range(len(v)):
        w = v[lo[i]:hi[i]]
        if len(w) >= 5: out[i] = np.mean(w)
    return out
rsd = lambda v: 1.4826*np.nanmedian(np.abs(v-np.nanmedian(v)))
print("| ride | vx noise sc / vis, m/s | yaw-rate noise sc / vis, mrad/s | v_ego (previous run) - vx (this run): sd sc / vis, m/s |")
print("|---|---|---|---|")
tot = []
for ride in sys.argv[2:]:
    row = []
    for s in ("sc.13.8", "vis.13.8"):
        with gzip.open(f"{sys.argv[1]}/{ride}.{s}.jsonl.gz", "rt") as f: ms = [json.loads(l) for l in f]
        t = np.array([m["ts_mono_ns"]*1e-9 for m in ms]); vx = np.array([m["pose"]["vx"] for m in ms]); yr = np.array([m["pose"]["yaw_rate"] for m in ms]); ve = np.array([m["v_ego"] for m in ms])
        mv = vx > 3
        row.append((rsd((vx-cen(t,vx))[mv]), 1e3*rsd((yr-cen(t,yr))[mv]), rsd((ve-vx)[mv])))
    tot.append(row)
    print(f"| {ride} | {row[0][0]:.3f} / {row[1][0]:.3f} | {row[0][1]:.2f} / {row[1][1]:.2f} | {row[0][2]:.3f} / {row[1][2]:.3f} |")
a = np.array(tot).mean(axis=0)
print(f"| mean | {a[0][0]:.3f} / {a[1][0]:.3f} | {a[0][1]:.2f} / {a[1][1]:.2f} | {a[0][2]:.3f} / {a[1][2]:.3f} |")
