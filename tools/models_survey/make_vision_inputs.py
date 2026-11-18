#!/usr/bin/env python3
"""Build real v0.10+/v0.11 vision input pairs (frames t-4, t; nearest warp as v0.11.1) from rides.

usage: python -I make_vision_inputs.py OUT.npz RIDE_DIR:START_S[:N[:STEP_S]] [...]
Each sample: img, big_img uint8 [12,128,256]; sc_img, sc_big = v0.9.7 pair (t-1, t), bilinear. Default N=15 samples spaced STEP_S=2 s.
"""
import glob
import json
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import replay_compare as rc  # noqa: E402


def main():
    out = sys.argv[1]
    imgs, bigs, tags, scs, scb = [], [], [], [], []
    for spec in sys.argv[2:]:
        parts = spec.split(":")
        ride, start = parts[0], float(parts[1])
        n = int(parts[2]) if len(parts) > 2 else 15
        step = float(parts[3]) if len(parts) > 3 else 2.0
        cam = json.load(open(os.path.join(ride, "camera-replay.json")))
        cal = json.load(open(os.path.join(ride, "calib.json")))
        K = np.array([[cam["focal_px"], 0, cam["cx"]], [0, cam["focal_px"], cam["cy"]], [0, 0, 1.0]])
        rpy = (cal.get("roll", 0.0), cal.get("pitch", 0.0), cal.get("yaw", 0.0))
        (wy, wuv) = rc.make_warps(rc.warp_matrix(K, rpy, rc.MED_K), "nearest")
        (by, buv) = rc.make_warps(rc.warp_matrix(K, rpy, rc.SBIG_K), "nearest")
        (sy, suv) = rc.make_warps(rc.warp_matrix(K, rpy, rc.MED_K), "bilinear")
        (sby, sbuv) = rc.make_warps(rc.warp_matrix(K, rpy, rc.SBIG_K), "bilinear")
        mj = sorted(glob.glob(os.path.join(ride, "*.mjpeg")))[0]
        fps = cam.get("fps", 20)
        for i in range(n):
            f0 = int(round((start + i * step) * fps))
            fr = list(rc.frames(mj, f0, 5))
            a, b = fr[0], fr[4]
            imgs.append(np.concatenate([rc.pack(wy, wuv, *a), rc.pack(wy, wuv, *b)]))
            bigs.append(np.concatenate([rc.pack(by, buv, *a), rc.pack(by, buv, *b)]))
            c = fr[3]  # supercombo v0.9.7 pair (t-1, t), bilinear warp
            scs.append(np.concatenate([rc.pack(sy, suv, *c), rc.pack(sy, suv, *b)]))
            scb.append(np.concatenate([rc.pack(sby, sbuv, *c), rc.pack(sby, sbuv, *b)]))
            tags.append(f"{os.path.basename(ride.rstrip('/'))}@{f0}")
    np.savez_compressed(out, img=np.array(imgs), big_img=np.array(bigs), sc_img=np.array(scs), sc_big=np.array(scb), tags=np.array(tags))
    print("saved", out, len(imgs))


if __name__ == "__main__":
    main()
