#!/usr/bin/env python3
"""Side-by-side overlays (left: supercombo v0.9.7, right: another model) from a replay_compare.py npz.

usage: python -I overlay.py RIDE_DIR RIDE.npz OUT_DIR [--model v0111] [--frames i,j,k | --auto 3]
--auto N picks frames with the largest disagreement (ego-lane presence/position, lead presence) plus one
random frame. Lane lines: green (ego) / cyan (outer), alpha by probability, drawn only for prob > 0.3;
road edges red; lead 0 box (1.8 x 1.5 m) yellow if prob > 0.5. Projection uses the ride's K and rpy and
the model's own lane z (road surface below the camera).
"""
import argparse
import glob
import io
import json
import os
import sys

import numpy as np
from PIL import Image, ImageDraw

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import replay_compare as rc  # noqa: E402

X = rc.X_IDXS


def project(K, R, pts):  # pts (N,3) calibrated frame (x fwd, y right, z down)
    d = (R @ pts.T).T
    v = (rc.VIEW_FROM_DEVICE @ d.T).T
    ok = v[:, 2] > 0.5
    uv = (K @ v.T).T
    uv = uv[:, :2] / np.maximum(uv[:, 2:3], 1e-6)
    return uv, ok


def draw(img, K, R, z, m, i, label, height):
    dr = ImageDraw.Draw(img, "RGBA")
    lane = z[f"{m}/lane"][i]; lp = z[f"{m}/lprob"][i]; edge = z[f"{m}/edge"][i]
    lead = z[f"{m}/lead"][i, 0]; ldp = z[f"{m}/lead_prob"][i, 0]
    sel = X <= 120
    for j in range(4):
        if lp[j] < 0.3:
            continue
        pts = np.stack([X[sel], lane[j, sel, 0], lane[j, sel, 1]], 1)
        uv, ok = project(K, R, pts)
        col = (0, 255, 0) if j in (1, 2) else (0, 220, 255)
        dr.line([tuple(p) for p in uv[ok]], fill=col + (int(255 * lp[j]),), width=4)
    for j in range(2):
        pts = np.stack([X[sel], edge[j, sel, 0], edge[j, sel, 1]], 1)
        uv, ok = project(K, R, pts)
        dr.line([tuple(p) for p in uv[ok]], fill=(255, 40, 40, 220), width=3)
    if ldp > 0.5:
        x, y = lead[0], lead[1]
        c = np.array([[x, y - 0.9, height], [x, y + 0.9, height], [x, y + 0.9, height - 1.5], [x, y - 0.9, height - 1.5]])
        uv, ok = project(K, R, c)
        if ok.all():
            dr.polygon([tuple(p) for p in uv], outline=(255, 220, 0, 255), width=3)
            dr.text(tuple(uv[2] + [0, -14]), f"{x:.0f} m p{ldp:.2f}", fill=(255, 220, 0, 255))
    dr.rectangle([0, 0, 640, 24], fill=(0, 0, 0, 160))
    dr.text((6, 5), f"{label}  lane p {' '.join(f'{p:.2f}' for p in lp)}  lead p {ldp:.2f}", fill=(255, 255, 255, 255))
    return img


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("ride"); ap.add_argument("npz"); ap.add_argument("out")
    ap.add_argument("--model", default="v0111")
    ap.add_argument("--frames", default="")
    ap.add_argument("--auto", type=int, default=3)
    a = ap.parse_args()
    z = np.load(a.npz)
    meta = json.loads(str(z["meta"]))
    K = np.array(meta["K"]); R = rc.rot_from_euler(*meta["rpy"])
    cal = json.load(open(os.path.join(a.ride, "calib.json")))
    height = cal.get("height_m", 1.22)
    m = a.model
    T = len(z["sc097/lprob"])
    if a.frames:
        idx = [int(x) for x in a.frames.split(",")]
    else:
        pa, pb = z["sc097/lprob"] > 0.5, z[f"{m}/lprob"] > 0.5
        dis_pres = (pa[:, 1:3] != pb[:, 1:3]).sum(1)
        dy = np.nan_to_num(np.abs(z["sc097/lane"][:, 1:3, 8, 0] - z[f"{m}/lane"][:, 1:3, 8, 0]).max(1) * (pa[:, 1:3] & pb[:, 1:3]).all(1))
        ld = (z["sc097/lead_prob"][:, 0] > 0.5) != (z[f"{m}/lead_prob"][:, 0] > 0.5)
        score = dis_pres * 2 + np.minimum(dy, 3) + ld * 1.5
        score[:40] = -1
        idx = []
        for k in np.argsort(-score):
            if all(abs(k - j) > 100 for j in idx):
                idx.append(int(k))
            if len(idx) >= a.auto - 1:
                break
        idx.append(int(np.random.default_rng(0).integers(40, T)))
    mj = sorted(glob.glob(os.path.join(a.ride, "*.mjpeg")))[0]
    offs = rc.jpeg_offsets(mj)
    os.makedirs(a.out, exist_ok=True)
    for i in idx:
        f = meta["start_frame"] + i
        with open(mj, "rb") as fh:
            fh.seek(offs[f]); b = fh.read(offs[f + 1] - offs[f])
        base = Image.open(io.BytesIO(b)).convert("RGB")
        L = draw(base.copy(), K, R, z, "sc097", i, f"supercombo v0.9.7  {meta['ride']} f{f}", height)
        Rr = draw(base.copy(), K, R, z, m, i, f"{m} vision  {meta['ride']} f{f}", height)
        out = Image.new("RGB", (1920, 540))
        out.paste(L.resize((960, 540)), (0, 0)); out.paste(Rr.resize((960, 540)), (960, 0))
        p = os.path.join(a.out, f"{meta['ride']}-f{f}-sc097-vs-{m}.jpg")
        out.save(p, quality=88)
        print(p)


if __name__ == "__main__":
    main()
