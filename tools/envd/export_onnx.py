#!/usr/bin/env python3
"""Export the image tower of an open_clip model to ONNX for envd (input [1,3,S,S] normalised NCHW,
output [1,D] projected embedding, unnormalised) and check it against PyTorch on random inputs.

    export_onnx.py --model ViT-B-32 --pretrained openai --out models/envd-image.onnx [--config-dir DIR] [--opset 17]
"""
import argparse
import os

import numpy as np
import torch


class ImageTower(torch.nn.Module):
    def __init__(self, model):
        super().__init__()
        self.model = model

    def forward(self, x):
        return self.model.encode_image(x)


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--model", required=True)
    ap.add_argument("--pretrained", required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--config-dir", default="")
    ap.add_argument("--opset", type=int, default=17)
    a = ap.parse_args()
    import open_clip
    if a.config_dir:
        open_clip.add_model_config(a.config_dir)
    model, _, _ = open_clip.create_model_and_transforms(a.model, pretrained=a.pretrained)
    model.eval()
    cfg = open_clip.get_model_config(a.model)
    size = cfg["vision_cfg"]["image_size"]
    if isinstance(size, (list, tuple)):
        size = size[0]
    tower = ImageTower(model).eval()
    x = torch.randn(1, 3, size, size)
    with torch.no_grad():
        ref = tower(x).numpy()
    torch.onnx.export(tower, x, a.out, input_names=["image"], output_names=["embedding"], opset_version=a.opset,
                      dynamo=False, do_constant_folding=True)
    import onnxruntime as ort
    s = ort.InferenceSession(a.out, providers=["CPUExecutionProvider"])
    err = 0.0
    for _ in range(3):
        x = torch.randn(1, 3, size, size)
        with torch.no_grad():
            ref = tower(x).numpy()
        got = s.run(None, {"image": x.numpy()})[0]
        cos = float((ref * got).sum() / (np.linalg.norm(ref) * np.linalg.norm(got)))
        err = max(err, float(np.abs(ref - got).max()))
    print(f"{a.out}: {os.path.getsize(a.out) / 1e6:.1f} MB, input {size}x{size}, output {ref.shape}, "
          f"max |diff| vs torch {err:.2e}, cos {cos:.6f}")


if __name__ == "__main__":
    main()
