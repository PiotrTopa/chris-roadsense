#!/usr/bin/env python3
"""Debug helper: compare named ncnn blobs with ONNX intermediates on one real sample.
usage: python -I ncnn_blobdiff.py PARAM BIN SIM.onnx INPUTS.npz ncnnblob=onnxtensor [...]"""
import sys, numpy as np, onnx, onnxruntime as ort, ncnn
param, bin_, onx, inp = sys.argv[1:5]
pairs = [a.split("=") for a in sys.argv[5:]]
z = np.load(inp); img, big = z["img"][0], z["big_img"][0]
imgf, bigf = np.ascontiguousarray(img, dtype=np.float32), np.ascontiguousarray(big, dtype=np.float32)  # kept alive for ncnn.Mat
m = onnx.load(onx)
for _, t in pairs:
    m.graph.output.append(onnx.helper.make_empty_tensor_value_info(t))
s = ort.InferenceSession(m.SerializeToString(), providers=["CPUExecutionProvider"])
names = [o.name for o in s.get_outputs()]
R = dict(zip(names, s.run(None, {"img": img[None], "big_img": big[None]})))
net = ncnn.Net(); net.opt.use_vulkan_compute = False; net.load_param(param); net.load_model(bin_)
for b, t in pairs:
    ex = net.create_extractor()
    ex.input("in0", ncnn.Mat(imgf))
    ex.input("in1", ncnn.Mat(bigf))
    _, o = ex.extract(b); a = np.array(o).copy(); r = R[t]
    print(f"{b:>6} {str(a.shape):18s} vs {t:28s} {str(r.shape):20s}", end=" ")
    if a.size == r.size:
        d = np.abs(a.ravel() - r.ravel()); print(f"maxdiff {d.max():.4g} ref max {np.abs(r).max():.4g}")
    else:
        print("size mismatch")
