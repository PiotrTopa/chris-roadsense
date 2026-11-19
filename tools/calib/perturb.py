#!/usr/bin/env python3
"""Turn the camera of a recording by a known angle (validation of tools/calib/mount.py; needs OpenCV).

    ffmpeg ... -f rawvideo -pix_fmt nv12 - | perturb.py --size 1280x720 --focal F [--cx X --cy Y]
        --calib calib.json --pitch-deg DP --yaw-deg DY [--roll-deg DR] [--save-frames DIR --save-every 20]
        | modeld_replay /dev/stdin --width 1280 --height 720 ...

Reads raw NV12 frames on stdin and writes them to stdout as a camera would have seen the same drive had its
mounting been (pitch + DP, yaw + DY, roll + DR) instead of calib.json's angles: a pure rotation about the lens, so
the picture is re-projected exactly (x' ~ K V R_new R_old^T V^T K^-1 x) and no depth is needed. Luma and chroma
are warped in their own planes, nothing is colour-converted. With all angles 0 the frames pass through untouched.
`--save-frames` keeps every N-th output frame as DIR/<seq>.jpg (seq = frame number from 1, as modeld_replay
numbers them), which together with modeld_replay's messages makes a folder mount.py reads like a recorder session.
"""
import argparse
import math
import os
import sys

import cv2
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import geom  # noqa: E402


def homography(intr, calib, d_pitch, d_yaw, d_roll):
    k = np.array([[intr["focal_px"], 0, intr["cx"]], [0, intr["focal_px"], intr["cy"]], [0, 0, 1.0]])
    r_old = geom.rot_from_euler(calib.get("roll", 0.0), calib.get("pitch", 0.0), calib.get("yaw", 0.0))
    r_new = geom.rot_from_euler(calib.get("roll", 0.0) + d_roll, calib.get("pitch", 0.0) + d_pitch,
                                calib.get("yaw", 0.0) + d_yaw)
    v = geom.VIEW_FROM_DEVICE
    return k @ v @ r_new @ r_old.T @ v.T @ np.linalg.inv(k)


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--size", default="1280x720")
    ap.add_argument("--focal", type=float, required=True)
    ap.add_argument("--cx", type=float)
    ap.add_argument("--cy", type=float)
    ap.add_argument("--calib", help="calib.json of the recording (the mounting the angles are added to)")
    ap.add_argument("--pitch-deg", type=float, default=0.0)
    ap.add_argument("--yaw-deg", type=float, default=0.0)
    ap.add_argument("--roll-deg", type=float, default=0.0)
    ap.add_argument("--save-frames")
    ap.add_argument("--save-every", type=int, default=20)
    a = ap.parse_args(argv)
    w, h = (int(x) for x in a.size.lower().split("x"))
    intr = dict(focal_px=a.focal, cx=a.cx if a.cx is not None else w / 2.0, cy=a.cy if a.cy is not None else h / 2.0)
    calib = geom.load_json(a.calib) if a.calib else {}
    ident = a.pitch_deg == 0 and a.yaw_deg == 0 and a.roll_deg == 0
    hm = homography(intr, calib, math.radians(a.pitch_deg), math.radians(a.yaw_deg), math.radians(a.roll_deg))
    s = np.diag([0.5, 0.5, 1.0])
    # chroma plane: half size, samples at the centres of 2x2 luma blocks: u_c = (u - 0.5) / 2
    t = np.array([[0.5, 0, -0.25], [0, 0.5, -0.25], [0, 0, 1.0]])
    hc = t @ hm @ np.linalg.inv(t)
    del s
    if a.save_frames:
        os.makedirs(a.save_frames, exist_ok=True)
    fin, fout = sys.stdin.buffer, sys.stdout.buffer
    n = 0
    ysz, csz = w * h, w * h // 2
    while True:
        buf = fin.read(ysz + csz)
        if len(buf) < ysz + csz:
            break
        n += 1
        y = np.frombuffer(buf, np.uint8, ysz).reshape(h, w)
        uv = np.frombuffer(buf, np.uint8, csz, ysz).reshape(h // 2, w // 2, 2)
        if not ident:
            y = cv2.warpPerspective(y, hm, (w, h), flags=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT, borderValue=0)
            uv = cv2.warpPerspective(uv, hc, (w // 2, h // 2), flags=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT,
                                     borderValue=(128, 128))
        try:
            fout.write(y.tobytes())
            fout.write(uv.tobytes())
        except BrokenPipeError:
            break
        if a.save_frames and (n - 1) % a.save_every == 0:
            cv2.imwrite(os.path.join(a.save_frames, "%d.jpg" % n), y, [cv2.IMWRITE_JPEG_QUALITY, 90])
    return 0


if __name__ == "__main__":
    sys.exit(main())
