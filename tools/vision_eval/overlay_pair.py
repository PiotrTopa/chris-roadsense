#!/usr/bin/env python3
"""Frames where the two road models disagree after the trackers, both drawn side by side (lane L24-M hand check).

    python tools/vision_eval/overlay_pair.py RIDE_DIR ROAD_DIR OUT_DIR [--a sc.13.8] [--b vis.13.8] [--n 5]

Picks the N moments (at least 4 s apart, moving) where the tracked inner lane lines of the two streams differ
most at 30-60 m or where the lead differs (present in one only, or distance off by > 25 %), and writes
OUT_DIR/<ride>_<seq>.jpg: left panel stream A, right panel stream B. Drawn: tracked lane lines (`lanes`,
solid; green = inner, yellow = outer), road edges (orange), the raw model lines (thin grey), the tracked lead
(red cross at its distance and lateral offset, label = distance). OUT_DIR/<ride>.csv lists the picks and why.
Frames are shown under the ride's CC BY 3.0 licence (SOURCE.txt) for a private check only.
"""
import argparse
import glob
import json
import os
import subprocess
import sys

import numpy as np
from PIL import Image, ImageDraw

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))
sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(HERE)), "api"))
from replay_lib import Pipeline, read_lines  # noqa: E402
from roadsense_api.overlay import Projector  # noqa: E402

X = np.array([192.0 * (i / 32.0) ** 2 for i in range(33)])


def run(path):
    p = Pipeline()
    out = {}
    for m in read_lines(path):
        outs = p.feed(m)
        if m.get("type") == "road":
            d = {o["type"]: o for o in outs}
            if "lanes" in d:
                out[m["seq"]] = (m, d["lanes"], d.get("lead"))
    return out


def frame(mj, seq):
    n = seq - 1
    r = subprocess.run(["ffmpeg", "-loglevel", "error", "-f", "mjpeg", "-framerate", "20", "-i", mj, "-vf",
                        f"select=eq(n\\,{n})", "-frames:v", "1", "-f", "image2pipe", "-vcodec", "png", "-"],
                       capture_output=True, check=True)
    import io
    return Image.open(io.BytesIO(r.stdout)).convert("RGB")


def draw(img, proj, m, ln, ld, title):
    d = ImageDraw.Draw(img)
    z = proj.height

    def poly(xs, ys, col, w, maxx=100.0):
        sel = (xs <= maxx) & np.isfinite(ys)
        pts = proj.project(xs[sel], ys[sel], np.full(sel.sum(), z))
        pts = [p for p in pts if -2000 < p[0] < 4000 and -2000 < p[1] < 4000]
        if len(pts) >= 2:
            d.line(pts, fill=col, width=w)
    for l in m["lane_lines"]:
        poly(X, np.asarray(l["y"]), (150, 150, 150), 1)
    for e in m["road_edges"]:
        poly(X, np.asarray(e["y"]), (110, 110, 110), 1)
    for i, l in enumerate(ln["lines"]):
        if l.get("present") and l.get("y"):
            poly(np.asarray(ln["x"]), np.asarray(l["y"]), (0, 255, 0) if i in (1, 2) else (255, 230, 0), 3)
    for e in ln.get("edges") or []:
        if e.get("present") and e.get("y"):
            poly(np.asarray(ln["x"]), np.asarray(e["y"]), (255, 140, 0), 3)
    if ld and ld.get("present") and ld.get("dist_m"):
        pts = proj.project([ld["dist_m"]], [ld.get("lateral_m") or 0.0], [z - 0.7])
        if pts:
            u, v = pts[0]
            d.line([(u - 14, v - 14), (u + 14, v + 14)], fill=(255, 0, 0), width=4)
            d.line([(u - 14, v + 14), (u + 14, v - 14)], fill=(255, 0, 0), width=4)
            d.text((u + 16, v - 8), f"{ld['dist_m']:.1f} m", fill=(255, 0, 0))
    d.rectangle([0, 0, 420, 18], fill=(0, 0, 0))
    d.text((4, 3), title, fill=(255, 255, 255))
    return img


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("ride"); ap.add_argument("road_dir"); ap.add_argument("out")
    ap.add_argument("--a", default="sc.13.8"); ap.add_argument("--b", default="vis.13.8")
    ap.add_argument("--n", type=int, default=5)
    a = ap.parse_args()
    slug = os.path.basename(os.path.normpath(a.ride))
    A = run(os.path.join(a.road_dir, f"{slug}.{a.a}.jsonl.gz"))
    B = run(os.path.join(a.road_dir, f"{slug}.{a.b}.jsonl.gz"))
    cands = []
    for s in sorted(set(A) & set(B)):
        (ma, la, da), (mb, lb, db) = A[s], B[s]
        if (ma.get("v_ego") or 0) < 3:
            continue
        score, why = 0.0, ""
        for i in (1, 2):
            ya, yb = la["lines"][i], lb["lines"][i]
            if ya.get("present") and yb.get("present"):
                dy = abs(np.interp(45.0, X, ya["y"]) - np.interp(45.0, X, yb["y"]))
                if dy > score:
                    score, why = dy, f"line{i} dy45={dy:.2f}m"
            elif ya.get("present") != yb.get("present"):
                if 1.0 > score:
                    score, why = 1.0, f"line{i} present {a.a}={ya.get('present')} {a.b}={yb.get('present')}"
        pa, pb = bool(da and da.get("present")), bool(db and db.get("present"))
        if pa != pb:
            if 1.5 > score:
                score, why = 1.5, f"lead present {a.a}={pa} {a.b}={pb}"
        elif pa and pb and da.get("dist_m") and db.get("dist_m"):
            r = abs(da["dist_m"] - db["dist_m"]) / max(da["dist_m"], db["dist_m"])
            if r > 0.25 and 1.2 + r > score:
                score, why = 1.2 + r, f"lead dist {da['dist_m']:.1f} vs {db['dist_m']:.1f}"
        if score > 0.3:
            cands.append((score, s, why))
    cands.sort(reverse=True)
    picks = []
    for sc, s, why in cands:
        if all(abs(s - p[1]) >= 80 for p in picks):
            picks.append((sc, s, why))
        if len(picks) >= a.n:
            break
    os.makedirs(a.out, exist_ok=True)
    cam = json.load(open(os.path.join(a.ride, "camera-replay.json")))
    cal = json.load(open(os.path.join(a.ride, "calib.json")))
    mj = sorted(glob.glob(os.path.join(a.ride, "*.mjpeg")))[0]
    with open(os.path.join(a.out, f"{slug}.csv"), "w") as f:
        f.write("seq,t_s,score,why\n")
        for sc, s, why in sorted(picks, key=lambda p: p[1]):
            img = frame(mj, s)
            proj = Projector((cam["focal_px"], cam.get("cx", 640), cam.get("cy", 360), 1280, 720), cal, img.size)
            L = draw(img.copy(), proj, *A[s], f"{slug} t={(s - 1) / 20:.1f}s  {a.a}")
            R = draw(img.copy(), proj, *B[s], f"{slug} t={(s - 1) / 20:.1f}s  {a.b}")
            both = Image.new("RGB", (img.width * 2, img.height))
            both.paste(L, (0, 0)); both.paste(R, (img.width, 0))
            both.resize((img.width * 3 // 2, img.height * 3 // 4)).save(os.path.join(a.out, f"{slug}_{s}.jpg"), quality=85)
            f.write(f"{s},{(s - 1) / 20:.2f},{sc:.2f},{why}\n")
    print(f"{slug}: {len(picks)} picks -> {a.out}")


if __name__ == "__main__":
    main()
