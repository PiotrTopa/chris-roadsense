#!/usr/bin/env python3
"""Random moments where only one road model reports a lead (after the trackers), for a hand check (lane L24-M).

    python tools/vision_eval/lead_presence_sample.py RIDE_DIR ROAD_DIR OUT_DIR [--n 3] [--seed 1]

Unlike overlay_pair.py (worst disagreements), this samples uniformly among the moving runs where
lead.present differs between sc.13.8 and vis.13.8: N moments where only the vision model has a lead and N where
only supercombo has one (at least 3 s apart). Each sheet OUT_DIR/<ride>_<which>.jpg stacks the moments, each as
a side-by-side pair (left supercombo, right vision; red cross = the tracked lead at its distance). OUT_DIR/
<ride>_presence.csv lists them; the judgement (vehicle in the ego lane ahead at about that distance: real /
false / unclear) is made by eye on the sheet.
"""
import argparse
import glob
import json
import os
import random
import sys

from PIL import Image

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import overlay_pair as op  # noqa: E402


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("ride"); ap.add_argument("road_dir"); ap.add_argument("out")
    ap.add_argument("--n", type=int, default=3); ap.add_argument("--seed", type=int, default=1)
    a = ap.parse_args()
    slug = os.path.basename(os.path.normpath(a.ride))
    A = op.run(os.path.join(a.road_dir, f"{slug}.sc.13.8.jsonl.gz"))
    B = op.run(os.path.join(a.road_dir, f"{slug}.vis.13.8.jsonl.gz"))
    only = {"visonly": [], "sconly": []}
    for s in sorted(set(A) & set(B)):
        (ma, _, da), (_, _, db) = A[s], B[s]
        if (ma.get("v_ego") or 0) < 1:
            continue
        pa, pb = bool(da and da.get("present")), bool(db and db.get("present"))
        if pb and not pa:
            only["visonly"].append(s)
        elif pa and not pb:
            only["sconly"].append(s)
    rng = random.Random(a.seed)
    cam = json.load(open(os.path.join(a.ride, "camera-replay.json")))
    cal = json.load(open(os.path.join(a.ride, "calib.json")))
    mj = sorted(glob.glob(os.path.join(a.ride, "*.mjpeg")))[0]
    os.makedirs(a.out, exist_ok=True)
    with open(os.path.join(a.out, f"{slug}_presence.csv"), "w") as f:
        f.write("which,seq,t_s,runs_in_class\n")
        for which, seqs in only.items():
            pool = seqs[:]
            rng.shuffle(pool)
            picks = []
            for s in pool:
                if all(abs(s - p) >= 60 for p in picks):
                    picks.append(s)
                if len(picks) >= a.n:
                    break
            rows = []
            for s in sorted(picks):
                img = op.frame(mj, s)
                proj = op.Projector((cam["focal_px"], cam.get("cx", 640), cam.get("cy", 360), 1280, 720), cal, img.size)
                L = op.draw(img.copy(), proj, *A[s], f"{slug} t={(s - 1) / 20:.1f}s sc")
                R = op.draw(img.copy(), proj, *B[s], f"{slug} t={(s - 1) / 20:.1f}s vis")
                pair = Image.new("RGB", (img.width * 2, img.height))
                pair.paste(L, (0, 0)); pair.paste(R, (img.width, 0))
                rows.append(pair.resize((img.width, img.height // 2)))
                f.write(f"{which},{s},{(s - 1) / 20:.2f},{len(seqs)}\n")
            if rows:
                sheet = Image.new("RGB", (rows[0].width, sum(r.height for r in rows)))
                y = 0
                for r in rows:
                    sheet.paste(r, (0, y)); y += r.height
                sheet.save(os.path.join(a.out, f"{slug}_{which}.jpg"), quality=85)
    print(slug, {k: len(v) for k, v in only.items()})


if __name__ == "__main__":
    main()
