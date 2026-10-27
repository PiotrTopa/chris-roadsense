#!/usr/bin/env python3
"""Contact sheets of a ride from direct MJPEG indexing (20 fps): one frame every STEP s (default 2), 640x360, 12 per
sheet (4x3), ride time printed on each tile.  usage: sheets.py SLUG [STEP] -> ~/l11/frames/SLUG/sheet_NNN.jpg"""
import glob, io, os, sys
import numpy as np
from PIL import Image, ImageDraw, ImageFont
slug = sys.argv[1]; step = float(sys.argv[2]) if len(sys.argv) > 2 else 2.0
src = glob.glob(os.path.expanduser(f"~/datasets/rides/{slug}/*.mjpeg"))[0]
out = os.path.expanduser(f"~/l11/frames/{slug}"); os.makedirs(f"{out}/full", exist_ok=True)
for old in glob.glob(f"{out}/sheet_*.jpg"):
    os.remove(old)
idx_path = f"{out}/full/index.npy"
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
            data = tail + buf; base = pos - len(tail); i = 0
            while True:
                j = data.find(b"\xff\xd8\xff", i)
                if j < 0 or j >= len(data) - 3:
                    break
                offs.append(base + j); i = j + 3
            tail = data[-3:]; pos += len(buf)
    idx = np.array(offs + [os.path.getsize(src)], dtype=np.int64); np.save(idx_path, idx)
n = len(idx) - 1
times = [k * step for k in range(int(n / 20 / step))]
font = ImageFont.load_default(size=22)
with open(src, "rb") as f:
    for si in range(0, len(times), 12):
        sheet = Image.new("RGB", (4 * 640, 3 * 360), (0, 0, 0)); d = ImageDraw.Draw(sheet)
        for k, t in enumerate(times[si:si + 12]):
            fr = int(round(t * 20)); f.seek(idx[fr]); im = Image.open(io.BytesIO(f.read(idx[fr + 1] - idx[fr]))).convert("RGB")
            im = im.resize((640, 360)); x, y = (k % 4) * 640, (k // 4) * 360; sheet.paste(im, (x, y))
            label = f"{slug} {int(t // 60):d}:{t % 60:04.1f}"
            d.rectangle([x, y, x + 12 + 11 * len(label), y + 28], fill=(0, 0, 0)); d.text((x + 6, y + 3), label, fill=(255, 255, 0), font=font)
        sheet.save(f"{out}/sheet_{si // 12:03d}.jpg", quality=80)
print(slug, n, "frames", f"{n/20:.0f} s,", (len(times) + 11) // 12, "sheets")
