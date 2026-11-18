#!/usr/bin/env python3
"""ncnn accuracy/latency check of a converted openpilot model vs ORT fp32, on real inputs.

usage:
  python -I ncnn_check.py vision PARAM BIN ONNX-fp32 INPUTS.npz [--gpu 1]
  python -I ncnn_check.py sc097  PARAM BIN ONNX-fp32 INPUTS.npz [--gpu 1]

vision: v0.10/v0.11 driving_vision converted by pnnx; the two `Tensor.to` (uint8 -> float) layers that pnnx
leaves in the param (ncnn has no such layer) are removed first (float 0..255 is fed instead), written next
to PARAM as *.fixed.param.
sc097: roadsense's models/supercombo-ncnn.{param,bin} (tools/ncnn/make_ncnn.sh); recurrent inputs zero
(single-run error only), v_ego 15 m/s.

Variants: cpu-fp32, vk-fp32, vk-fp16p (fp16 packed storage, fp32 arithmetic, no fp16 storage
= the mode ncnn uses on the Adreno 630 under turnip, docs/HARDWARE.md), vk-fp16s (+fp16 storage).
The laptop GPU (Quadro P3200) reports fp16 arithmetic = 0 like the Adreno; it is a proxy for fp16
storage rounding, not for turnip's synchronisation bug or Adreno speed.
"""
import argparse
import os
import time

import numpy as np
import ncnn
import onnxruntime as ort

X_IDXS = 192.0 * (np.arange(33) / 32.0) ** 2
NEAR = X_IDXS <= 100.0


def fix_vision_param(p):
    L = open(p).read().splitlines()
    out = []
    dropped = 0
    for line in L[2:]:
        f = line.split()
        if f and f[0] == "Tensor.to":
            dropped += 1
            continue
        if f and f[0] == "Concat" and f[1] == "cat_0":
            f[4:6] = ["in0", "in1"]
            line = " ".join(f)
        out.append(line)
    nl, nb = map(int, L[1].split())
    q = p.replace(".param", ".fixed.param")
    open(q, "w").write("\n".join([L[0], f"{nl - dropped} {nb - dropped}"] + out) + "\n")
    return q, dropped


def sigmoid(x):
    return 1 / (1 + np.exp(-x))


def parse(kind, r):
    r = r.astype(np.float64).ravel()
    if kind == "vision":
        lane = r[117:381].reshape(4, 33, 2)[:, :, 0]; lp = sigmoid(r[645:653])[1::2]
        edge = r[653:785].reshape(2, 33, 2)[:, :, 0]
        lead = r[917:989].reshape(3, 6, 4)[0, 0]; ldp = sigmoid(r[1061])
        hid = r[1064:1576]
    else:
        lane = r[4955:5219].reshape(4, 33, 2)[:, :, 0]; lp = sigmoid(r[5483:5491])[1::2]
        edge = r[5491:5623].reshape(2, 33, 2)[:, :, 0]
        h = int(np.argmax([r[5755 + 51 * k + 48] for k in range(2)]))
        lead = r[5755 + 51 * h:5755 + 51 * h + 4]; ldp = sigmoid(r[5857])
        hid = r[5992:6504]
    return lane, lp, edge, lead, ldp, hid


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("kind", choices=["vision", "sc097"])
    ap.add_argument("param"); ap.add_argument("bin"); ap.add_argument("onnx"); ap.add_argument("inputs")
    ap.add_argument("--gpu", type=int, default=1)
    ap.add_argument("--variants", default="cpu-fp32,vk-fp32,vk-fp16p,vk-fp16s")
    a = ap.parse_args()
    z = np.load(a.inputs)
    if a.kind == "vision":
        param, dropped = fix_vision_param(a.param)
        print(f"removed {dropped} Tensor.to layers -> {param}")
        imgs, bigs = z["img"], z["big_img"]
    else:
        param = a.param
        imgs, bigs = z["sc_img"], z["sc_big"]
    N = len(imgs)
    so = ort.SessionOptions(); so.intra_op_num_threads = 4
    s = ort.InferenceSession(a.onnx, so, providers=["CPUExecutionProvider"])
    ref = []
    for i in range(N):
        if a.kind == "vision":
            o = s.run(None, {"img": imgs[i:i + 1], "big_img": bigs[i:i + 1]})[0]
        else:
            feeds = {"input_imgs": imgs[i:i + 1].astype(np.float32), "big_input_imgs": bigs[i:i + 1].astype(np.float32),
                     "desire": np.zeros((1, 100, 8), np.float32), "traffic_convention": np.array([[1, 0]], np.float32),
                     "lateral_control_params": np.array([[15.0, 0.4]], np.float32),
                     "prev_desired_curv": np.zeros((1, 100, 1), np.float32),
                     "features_buffer": np.zeros((1, 99, 512), np.float32)}
            o = s.run(None, feeds)[0]
        ref.append(parse(a.kind, o))
    for var in a.variants.split(","):
        net = ncnn.Net()
        vk = var.startswith("vk")
        net.opt.use_vulkan_compute = vk
        if vk:
            net.set_vulkan_device(a.gpu)
        net.opt.use_fp16_packed = "fp16" in var
        net.opt.use_fp16_storage = var.endswith("fp16s")
        net.opt.use_fp16_arithmetic = False
        net.opt.use_bf16_storage = False
        net.opt.num_threads = 4
        net.load_param(param); net.load_model(a.bin)
        errs = {k: [] for k in ("lane_mean", "lane_max_100m", "lane_max_all", "lprob", "edge_max_100m", "lead_x", "lead_p", "hid_cos")}
        tms = []
        for i in range(N):
            ex = net.create_extractor()
            # keep the numpy buffers alive: ncnn.Mat(ndarray) wraps the memory without owning it
            keep = [np.ascontiguousarray(imgs[i], dtype=np.float32), np.ascontiguousarray(bigs[i], dtype=np.float32)]
            ex.input("in0", ncnn.Mat(keep[0]))
            ex.input("in1", ncnn.Mat(keep[1]))
            if a.kind == "sc097":
                keep += [np.zeros((100, 8), np.float32), np.array([1, 0], np.float32),
                         np.array([15.0, 0.4], np.float32), np.zeros((100, 1), np.float32), np.zeros((8, 512), np.float32)]
                for j in range(5):
                    ex.input(f"in{j + 2}", ncnn.Mat(keep[2 + j]))
            t0 = time.perf_counter()
            r, out = ex.extract("out0")
            o = np.array(out).copy()
            tms.append((time.perf_counter() - t0) * 1e3)
            lane, lp, edge, lead, ldp, hid = parse(a.kind, o)
            R = ref[i]
            d = np.abs(lane - R[0])
            errs["lane_mean"].append(d.mean()); errs["lane_max_100m"].append(d[:, NEAR].max()); errs["lane_max_all"].append(d.max())
            errs["lprob"].append(np.abs(lp - R[1]).max())
            errs["edge_max_100m"].append(np.abs(edge - R[2])[:, NEAR].max())
            if R[4] > 0.5:
                errs["lead_x"].append(abs(lead[0] - R[3][0]))
            errs["lead_p"].append(abs(ldp - R[4]))
            errs["hid_cos"].append(float(np.dot(hid, R[5]) / (np.linalg.norm(hid) * np.linalg.norm(R[5]) + 1e-12)))
        msg = [f"{var:9s} N={N} ms med {np.median(tms[1:]):.1f}"]
        for k, v in errs.items():
            v = np.array(v) if v else np.array([np.nan])
            if k == "hid_cos":
                msg.append(f"{k} min {v.min():.4f}")
            else:
                msg.append(f"{k} mean {v.mean():.3g} p95 {np.percentile(v, 95):.3g} max {v.max():.3g}" + (f" (n={len(v)})" if k == "lead_x" else ""))
        print(" | ".join(msg), flush=True)
        del net


if __name__ == "__main__":
    main()
