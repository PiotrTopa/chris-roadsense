#!/usr/bin/env python3
"""Per-frame envd embeddings + cheap frame thumbnails of a ride (lane L48-E, rate study).

Decodes a ride MJPEG (20 fps) with ffmpeg at --thumb-every frames (default 5 = 4 Hz), keeps a 32x18 Y/U/V
block-mean thumbnail of every decoded frame (the input of a change detector) and feeds every
--emb-every-th frame (default 10 = 2 Hz: two 1 Hz phases, offset 0 and offset 10) as full-range NV12 to the
real C++ pipeline (`envd_replay --nv12 ... --emb --reset-each`), whose per-frame embeddings are kept, so any
class file can be scored afterwards and any schedule / smoother replayed offline (rate_sim.py).

    rate_extract.py --ride ~/datasets/rides/krakow-2024/krakow-25-35.mjpeg --out DIR/krakow-2024 \
        --replay build-l48e/envd_replay --model models/envd-image.onnx --classes models/envd-classes.json

Writes DIR/<name>.npz: emb [N,512] float32, emb_frame [N] (source frame index), thumb [M,3,18,32] uint8,
thumb_frame [M], fps.
"""
import argparse
import json
import os
import subprocess
import sys
import threading

import numpy as np

W, H = 1280, 720
TW, TH = 32, 18


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--ride", required=True)
    ap.add_argument("--out", required=True, help="output path without .npz")
    ap.add_argument("--replay", required=True)
    ap.add_argument("--model", required=True)
    ap.add_argument("--classes", required=True)
    ap.add_argument("--threads", type=int, default=3)
    ap.add_argument("--thumb-every", type=int, default=5)
    ap.add_argument("--emb-every", type=int, default=10)
    ap.add_argument("--fps", type=float, default=20.0)
    ap.add_argument("--max-frames", type=int, default=0, help="source frames (testing)")
    a = ap.parse_args()
    assert a.emb_every % a.thumb_every == 0
    sel = "select=not(mod(n\\,%d))" % a.thumb_every
    ff = ["ffmpeg", "-v", "error", "-i", a.ride, "-vf", sel, "-fps_mode", "passthrough"]
    if a.max_frames:
        ff += ["-frames:v", str(a.max_frames // a.thumb_every)]
    ff += ["-pix_fmt", "yuvj420p", "-f", "rawvideo", "-"]
    dec = subprocess.Popen(ff, stdout=subprocess.PIPE)
    jl = a.out + ".jsonl"
    fout = open(jl, "w")
    rep = subprocess.Popen([a.replay, "--model", a.model, "--classes", a.classes, "--threads", str(a.threads),
                            "--reset-each", "--emb", "--no-timing", "--nv12", "%dx%d" % (W, H), "/dev/stdin"],
                           stdin=subprocess.PIPE, stdout=fout, stderr=subprocess.DEVNULL)
    ysz, csz = W * H, (W // 2) * (H // 2)
    thumbs, thumb_frame, emb_frame = [], [], []
    nv = np.empty(ysz + 2 * csz, np.uint8)
    k = 0
    while True:
        buf = dec.stdout.read(ysz + 2 * csz)
        if len(buf) < ysz + 2 * csz:
            break
        a8 = np.frombuffer(buf, np.uint8)
        y = a8[:ysz].reshape(H, W)
        u = a8[ysz:ysz + csz].reshape(H // 2, W // 2)
        v = a8[ysz + csz:].reshape(H // 2, W // 2)
        src = k * a.thumb_every
        t = np.stack([y.reshape(TH, H // TH, TW, W // TW).mean(axis=(1, 3)),
                      u.reshape(TH, H // 2 // TH, TW, W // 2 // TW).mean(axis=(1, 3)),
                      v.reshape(TH, H // 2 // TH, TW, W // 2 // TW).mean(axis=(1, 3))])
        thumbs.append(np.round(t).astype(np.uint8))
        thumb_frame.append(src)
        if src % a.emb_every == 0:
            nv[:ysz] = a8[:ysz]
            uv = nv[ysz:].reshape(H // 2, W)
            uv[:, 0::2] = u
            uv[:, 1::2] = v
            rep.stdin.write(nv.tobytes())
            emb_frame.append(src)
        k += 1
    rep.stdin.close()
    rep.wait()
    dec.wait()
    fout.close()
    embs = []
    with open(jl) as f:
        for line in f:
            embs.append(json.loads(line)["emb"])
    emb = np.asarray(embs, np.float32)
    if len(emb) != len(emb_frame):
        sys.exit("mismatch: %d embeddings for %d frames" % (len(emb), len(emb_frame)))
    np.savez_compressed(a.out + ".npz", emb=emb, emb_frame=np.asarray(emb_frame), thumb=np.asarray(thumbs),
                        thumb_frame=np.asarray(thumb_frame), fps=a.fps)
    os.remove(jl)
    print("%s: %d embeddings, %d thumbnails" % (a.out, len(emb), len(thumbs)))


if __name__ == "__main__":
    main()
