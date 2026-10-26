#!/usr/bin/env python3
"""Image embeddings of a labelled image list with an open_clip model (evaluation / probe training).

    embed_dataset.py --model ViT-B-32 --pretrained openai --list list.csv --root DIR --out emb.npz
                     [--preproc clip|squash] [--size N] [--threads 8] [--batch 32]

list.csv: header `file,...` (any extra label columns are carried into the npz as string arrays).
--preproc clip   = the model's own transform (resize short side, centre crop) -- the reference;
--preproc squash = whole frame resized to size x size with no crop (what envd does on the phone,
                   see envd/envd.cc: the dashcam frame is 16:9 and the sides carry the scene).
Output npz: files, emb (float32 [N,D], L2-normalised), <label columns>, model, pretrained, preproc.
Weights are fetched into the Hugging Face cache (~/.cache/huggingface) by open_clip.
"""
import argparse
import csv
import os
import sys
import time

import numpy as np
import torch
from PIL import Image


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--model", required=True)
    ap.add_argument("--pretrained", required=True, help="open_clip tag or a local checkpoint path")
    ap.add_argument("--list", required=True)
    ap.add_argument("--root", required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--preproc", default="clip", choices=("clip", "squash"))
    ap.add_argument("--size", type=int, default=0, help="squash: side length (default: the model's input size)")
    ap.add_argument("--threads", type=int, default=8)
    ap.add_argument("--batch", type=int, default=32)
    ap.add_argument("--limit", type=int, default=0)
    ap.add_argument("--config-dir", default=os.environ.get("OPEN_CLIP_EXTRA_CONFIGS", ""),
                    help="extra open_clip model_configs directory (e.g. the TinyCLIP JSONs)")
    a = ap.parse_args()

    import open_clip
    if a.config_dir:
        open_clip.add_model_config(a.config_dir)
    torch.set_num_threads(a.threads)
    model, _, preprocess = open_clip.create_model_and_transforms(a.model, pretrained=a.pretrained)
    model.eval()
    cfg = open_clip.get_model_config(a.model)
    size = a.size or (cfg["vision_cfg"]["image_size"] if cfg else 224)
    if isinstance(size, (list, tuple)):
        size = size[0]
    # normalisation constants of this model (preprocess is a torchvision Compose ending in Normalize)
    norm = [t for t in preprocess.transforms if t.__class__.__name__ == "Normalize"][-1]
    mean = torch.tensor(norm.mean).view(3, 1, 1)
    std = torch.tensor(norm.std).view(3, 1, 1)

    with open(a.list) as f:
        rows = list(csv.DictReader(f))
    if a.limit:
        rows = rows[: a.limit]
    cols = [c for c in rows[0].keys() if c != "file"]

    def load(path):
        im = Image.open(path).convert("RGB")
        if a.preproc == "clip":
            return preprocess(im)
        # Replica of envd/prep.cc: the frame as NV12 (full-range BT.601, 4:2:0 chroma = 2x2 means),
        # box-downscaled to size x size with INTEGER block bounds (ox*W//size .. (ox+1)*W//size),
        # the block mean converted to RGB afterwards. Parity with the C++: docs/lanes/L2-envd.md.
        w, h = im.size[0] & ~1, im.size[1] & ~1
        ycc = np.asarray(im.crop((0, 0, w, h)).convert("YCbCr"), dtype=np.float64)
        Y = ycc[:, :, 0]
        C = np.round(ycc[:, :, 1:].reshape(h // 2, 2, w // 2, 2, 2).mean(axis=(1, 3))) - 128.0
        bx = (np.arange(size) * w + size - 1) // size  # first source column with floor(x*size/w) == ox
        by = (np.arange(size) * h + size - 1) // size
        ny = np.diff(np.append(by, h))[:, None] * np.diff(np.append(bx, w))[None, :]
        Ym = np.add.reduceat(np.add.reduceat(Y, by, axis=0), bx, axis=1) / ny
        cx = np.searchsorted(np.arange(0, w, 2) * size // w, np.arange(size))  # chroma col -> output col
        cy = np.searchsorted(np.arange(0, h, 2) * size // h, np.arange(size))
        nc = np.diff(np.append(cy, h // 2))[:, None] * np.diff(np.append(cx, w // 2))[None, :]
        Cm = np.add.reduceat(np.add.reduceat(C, cy, axis=0), cx, axis=1) / nc[:, :, None]
        U, V = Cm[:, :, 0], Cm[:, :, 1]
        rgb = np.stack([Ym + 1.402 * V, Ym - 0.344136 * U - 0.714136 * V, Ym + 1.772 * U]).clip(0, 255)
        x = torch.from_numpy((rgb / 255.0).astype(np.float32))
        return (x - mean) / std

    embs = []
    t0 = time.time()
    with torch.no_grad():
        for i in range(0, len(rows), a.batch):
            batch = torch.stack([load(os.path.join(a.root, r["file"])) for r in rows[i : i + a.batch]])
            e = model.encode_image(batch)
            e = e / e.norm(dim=-1, keepdim=True)
            embs.append(e.float().numpy())
            if (i // a.batch) % 20 == 0:
                print(f"{i + len(batch)}/{len(rows)} {time.time() - t0:.0f} s", file=sys.stderr, flush=True)
    emb = np.concatenate(embs)
    out = {"files": np.array([r["file"] for r in rows]), "emb": emb, "model": a.model, "pretrained": a.pretrained,
           "preproc": a.preproc, "size": size}
    for c in cols:
        out[c] = np.array([r[c] for r in rows])
    np.savez(a.out, **out)
    print(f"{a.out}: {emb.shape} in {time.time() - t0:.0f} s ({len(rows) / (time.time() - t0):.1f} img/s)")


if __name__ == "__main__":
    main()
