#!/usr/bin/env python3
"""Called by tools/ncnn/make_vision_ncnn.sh: fix_vision_ncnn.py IN.param OUT.param

pnnx converts the v0.11.1 driving_vision graph (uint8 inputs) with two `Tensor.to` (uint8 -> float) layers in
front of the first Concat; ncnn has no such layer and its Mats are float anyway. Drop them and feed the inputs
(float 0..255, which is what modeld's packer produces) straight into the Concat. Nothing else needs fixing:
every layer has a Vulkan kernel, the 2-D Gemms already come out as InnerProduct, and the largest activation on
real frames is 583 (fp16 limit 65,504; docs/lanes/L21-budget-and-models.md section 5).
"""
import sys

pin, pout = sys.argv[1:3]
L = open(pin).read().splitlines()
out, dropped = [], 0
for line in L[2:]:
    f = line.split()
    if f and f[0] == "Tensor.to":
        dropped += 1
        continue
    if f and f[0] == "Concat" and f[1] == "cat_0":
        f[4:6] = ["in0", "in1"]
        line = " ".join(f)
    out.append(line)
if dropped != 2:
    sys.exit(f"fix_vision_ncnn: expected 2 Tensor.to layers, found {dropped}")
nl, nb = map(int, L[1].split())
open(pout, "w").write("\n".join([L[0], f"{nl - dropped} {nb - dropped}"] + out) + "\n")
