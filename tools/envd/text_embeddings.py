#!/usr/bin/env python3
"""Class text embeddings of the envd taxonomy for one open_clip model, as the JSON file envd ships.

    text_embeddings.py --model ViT-B-32 --pretrained openai --out models/envd-classes.json [--config-dir DIR]

Output: {"model", "pretrained", "embed_dim", "image_size", "mean", "std", "logit_scale",
         "groups": {"place": {"classes": [...], "emb": [[...], ...]}, ...}}
Each class embedding is the L2-normalised mean of the normalised prompt embeddings (tools/envd/prompts.py).
"""
import argparse
import json
import os
import sys

import numpy as np
import torch

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from prompts import GROUPS, prompts_for  # noqa: E402


def class_embeddings(model, tokenizer):
    out = {}
    with torch.no_grad():
        for g, classes in GROUPS.items():
            embs = []
            for c in classes:
                tok = tokenizer(prompts_for(g, c))
                e = model.encode_text(tok).float()
                e = e / e.norm(dim=-1, keepdim=True)
                m = e.mean(0)
                embs.append((m / m.norm()).numpy())
            out[g] = {"classes": classes, "emb": np.stack(embs)}
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--model", required=True)
    ap.add_argument("--pretrained", required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--config-dir", default="")
    a = ap.parse_args()
    import open_clip
    if a.config_dir:
        open_clip.add_model_config(a.config_dir)
    model, _, preprocess = open_clip.create_model_and_transforms(a.model, pretrained=a.pretrained)
    model.eval()
    tokenizer = open_clip.get_tokenizer(a.model)
    norm = [t for t in preprocess.transforms if t.__class__.__name__ == "Normalize"][-1]
    cfg = open_clip.get_model_config(a.model)
    size = cfg["vision_cfg"]["image_size"]
    if isinstance(size, (list, tuple)):
        size = size[0]
    ce = class_embeddings(model, tokenizer)
    doc = {"model": a.model, "pretrained": os.path.basename(a.pretrained), "embed_dim": int(cfg["embed_dim"]),
           "image_size": int(size), "mean": [float(x) for x in norm.mean], "std": [float(x) for x in norm.std],
           "logit_scale": float(model.logit_scale.exp().item()),
           "groups": {g: {"classes": v["classes"], "emb": [[round(float(x), 6) for x in row] for row in v["emb"]]}
                      for g, v in ce.items()}}
    with open(a.out, "w") as f:
        json.dump(doc, f, separators=(",", ":"))
    print(f"{a.out}: {sum(len(v['classes']) for v in ce.values())} classes, dim {doc['embed_dim']}, "
          f"logit_scale {doc['logit_scale']:.2f}")


if __name__ == "__main__":
    main()
