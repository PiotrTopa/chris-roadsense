#!/usr/bin/env python3
"""Full-resolution frames of a ride at given ride times (s), by direct JPEG indexing of the MJPEG file (20 fps).
usage: frame.py SLUG T [T ...]  -> ~/l11/frames/SLUG/full/t<T>.jpg (prints the paths). Index cached in full/index.npy"""
import glob, os, sys
import numpy as np
slug = sys.argv[1]; times = [float(t) for t in sys.argv[2:]]
src = glob.glob(os.path.expanduser(f"~/datasets/rides/{slug}/*.mjpeg"))[0]
out = os.path.expanduser(f"~/l11/frames/{slug}/full"); os.makedirs(out, exist_ok=True)
idx_path = f"{out}/index.npy"
if os.path.exists(idx_path):
    idx = np.load(idx_path)
else:
    offs, pos, chunk = [], 0, 64 << 20
    with open(src, "rb") as f:
        tail = b""
        while True:
            buf = f.read(chunk)
            if not buf:
                break
            data = tail + buf
            base = pos - len(tail)
            i = 0
            while True:
                j = data.find(b"\xff\xd8\xff", i)
                if j < 0 or j >= len(data) - 3:
                    break
                offs.append(base + j); i = j + 3
            tail = data[-3:]; pos += len(buf)
    idx = np.array(offs + [os.path.getsize(src)], dtype=np.int64)
    np.save(idx_path, idx)
n = len(idx) - 1
with open(src, "rb") as f:
    for t in times:
        k = min(n - 1, max(0, int(round(t * 20))))
        f.seek(idx[k]); data = f.read(idx[k + 1] - idx[k])
        p = f"{out}/t{t:07.1f}.jpg"
        open(p, "wb").write(data); print(p)
print(f"{slug}: {n} frames = {n / 20:.1f} s", file=sys.stderr)
