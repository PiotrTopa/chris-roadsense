#!/usr/bin/env python3
"""Run supercombo v0.9.7 and newer openpilot vision models on the same stretch of a roadsense ride.

usage: python -I replay_compare.py RIDE_DIR START_S DUR_S OUT.npz [--models sc097,v0111,v0101] [--threads 3]

* frames: RIDE_DIR/<slug>.mjpeg (1280x720, 20 fps), decoded by ffmpeg to yuvj420p (full range, as the
  roadsense ring), intrinsics from camera-replay.json, rpy from calib.json (openpilot rpyCalib convention).
* sc097 = models/supercombo-fp32.onnx (scripts/fetch-models.sh; $SC097 overrides): 20 Hz every frame, image pair
  (t-1, t), bilinear warp (openpilot v0.9.7 transform.cl), desire 0, traffic RHT, v_ego = own pose vx,
  steer delay 0.4 s, features_buffer / prev_desired_curv feedback (docs/SUPERCOMBO_SPEC.md).
* v0111 = v0.11.1 driving_vision (fp32-converted): image pair (t-4, t) = 200 ms apart, nearest-neighbour
  warp as v0.11.1 compile_modeld.py (round + clip), UV matrix M*[[1,1,.5],[1,1,.5],[2,2,1]]; no state.
  v0111bl = the same model with the bilinear warp (sensitivity check).
* v0101 = v0.10.1 driving_vision: pair (t-4, t), bilinear warp (v0.10.x still used the OpenCL warp).
All models run on every frame (20 Hz) so the frame-to-frame metrics are comparable.
Saves per-frame parsed outputs (lane y/z 4x33, lane prob 4, edges 2x33, lead0..2 xyva at t=0, lead prob,
pose) per model.
"""
import argparse
import json
import mmap
import os
import subprocess
import sys
import time

import numpy as np
import onnxruntime as ort

W, H = 1280, 720
MED_K = np.array([[910.0, 0, 256.0], [0, 910.0, 47.6], [0, 0, 1]])
SBIG_K = np.array([[455.0, 0, 256.0], [0, 455.0, 151.8], [0, 0, 1]])
VIEW_FROM_DEVICE = np.array([[0, 1, 0], [0, 0, 1], [1, 0, 0]], dtype=np.float64)
D = os.environ.get("OP_MODELS", os.path.expanduser("~/datasets/openpilot-models"))   # downloaded releases, <tag>/driving_*.onnx
MODELS = {
    "sc097": os.environ.get("SC097", os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", "models", "supercombo-fp32.onnx")),
    "v0111": f"{D}/v0.11.1/driving_vision-fp32.onnx",
    "v0111bl": f"{D}/v0.11.1/driving_vision-fp32.onnx",
    "v0101": f"{D}/v0.10.1/driving_vision-fp32.onnx",
}
X_IDXS = 192.0 * (np.arange(33) / 32.0) ** 2


def rot_from_euler(r, p, y):
    cr, sr, cp, sp, cy, sy = np.cos(r), np.sin(r), np.cos(p), np.sin(p), np.cos(y), np.sin(y)
    Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    return Rz @ Ry @ Rx


def warp_matrix(K_cam, rpy, model_K):
    calib_from_model = np.linalg.inv(model_K @ VIEW_FROM_DEVICE)
    return K_cam @ VIEW_FROM_DEVICE @ rot_from_euler(*rpy) @ calib_from_model


def src_coords(M, w, h):
    xs, ys = np.meshgrid(np.arange(w, dtype=np.float64), np.arange(h, dtype=np.float64))
    X = M[0, 0] * xs + M[0, 1] * ys + M[0, 2]
    Y = M[1, 0] * xs + M[1, 1] * ys + M[1, 2]
    Wd = M[2, 0] * xs + M[2, 1] * ys + M[2, 2]
    return X / Wd, Y / Wd


class Warp:
    """Precomputed static warp (calibration fixed for the ride). kind: 'bilinear' | 'nearest'."""

    def __init__(self, M, kind, sw, sh, dw, dh):
        X, Y = src_coords(M, dw, dh)
        self.kind = kind
        if kind == "nearest":
            xi = np.clip(np.round(X), 0, sw - 1).astype(np.int64)
            yi = np.clip(np.round(Y), 0, sh - 1).astype(np.int64)
            self.idx = (yi * sw + xi).ravel()
        else:
            # openpilot transform.cl: fixed point 1/32 px, replicate border
            Xf = np.rint(X * 32).astype(np.int64); Yf = np.rint(Y * 32).astype(np.int64)
            sx, sy = Xf >> 5, Yf >> 5
            ax, ay = (Xf & 31) / 32.0, (Yf & 31) / 32.0
            x0 = np.clip(sx, 0, sw - 1); x1 = np.clip(sx + 1, 0, sw - 1)
            y0 = np.clip(sy, 0, sh - 1); y1 = np.clip(sy + 1, 0, sh - 1)
            self.i00 = (y0 * sw + x0).ravel(); self.i01 = (y0 * sw + x1).ravel()
            self.i10 = (y1 * sw + x0).ravel(); self.i11 = (y1 * sw + x1).ravel()
            self.w00 = ((1 - ay) * (1 - ax)).ravel(); self.w01 = ((1 - ay) * ax).ravel()
            self.w10 = (ay * (1 - ax)).ravel(); self.w11 = (ay * ax).ravel()
        self.shape = (dh, dw)

    def __call__(self, plane):
        f = plane.ravel()
        if self.kind == "nearest":
            return f[self.idx].reshape(self.shape)
        f = f.astype(np.float32)
        v = f[self.i00] * self.w00 + f[self.i01] * self.w01 + f[self.i10] * self.w10 + f[self.i11] * self.w11
        return np.clip(np.rint(v), 0, 255).astype(np.uint8).reshape(self.shape)


def make_warps(M, kind):
    if kind == "nearest":  # v0.11.1 compile_modeld.py
        Muv = M * np.array([[1.0, 1.0, 0.5], [1.0, 1.0, 0.5], [2.0, 2.0, 1.0]])
    else:  # v0.9.7 / v0.10.x common/mat.h transform_scale_buffer(M, 0.5)
        S_out = np.array([[2, 0, 0.5], [0, 2, 0.5], [0, 0, 1.0]])
        S_in = np.array([[0.5, 0, -0.25], [0, 0.5, -0.25], [0, 0, 1.0]])
        Muv = S_in @ M @ S_out
    return Warp(M, kind, W, H, 512, 256), Warp(Muv, kind, W // 2, H // 2, 256, 128)


def pack(wy, wuv, y, u, v):
    Y = wy(y)
    U = wuv(u)
    V = wuv(v)
    return np.stack([Y[0::2, 0::2], Y[1::2, 0::2], Y[0::2, 1::2], Y[1::2, 1::2], U, V])  # (6,128,256) uint8


def jpeg_offsets(path):
    with open(path, "rb") as fh:
        mm = mmap.mmap(fh.fileno(), 0, access=mmap.ACCESS_READ)
        offs = []
        i = mm.find(b"\xff\xd8\xff")
        while i >= 0:
            offs.append(i)
            i = mm.find(b"\xff\xd8\xff", i + 3)
        offs.append(len(mm))
        mm.close()
    return offs


def frames(path, start, n):
    offs = jpeg_offsets(path)
    a, b = offs[start], offs[min(start + n, len(offs) - 1)]
    p = subprocess.Popen(["ffmpeg", "-loglevel", "error", "-f", "mjpeg", "-i", "-", "-pix_fmt", "yuvj420p",
                          "-f", "rawvideo", "-"], stdin=subprocess.PIPE, stdout=subprocess.PIPE)
    import threading

    def feed():
        with open(path, "rb") as fh:
            fh.seek(a)
            left = b - a
            while left > 0:
                c = fh.read(min(left, 1 << 22))
                if not c:
                    break
                p.stdin.write(c)
                left -= len(c)
        p.stdin.close()

    threading.Thread(target=feed, daemon=True).start()
    fs = W * H * 3 // 2
    while True:
        buf = p.stdout.read(fs)
        if len(buf) < fs:
            break
        a8 = np.frombuffer(buf, np.uint8)
        yield a8[:W * H].reshape(H, W), a8[W * H:W * H * 5 // 4].reshape(H // 2, W // 2), \
            a8[W * H * 5 // 4:].reshape(H // 2, W // 2)
    p.wait()


def sess(path, threads):
    so = ort.SessionOptions()
    so.intra_op_num_threads = threads
    so.inter_op_num_threads = 1
    so.execution_mode = ort.ExecutionMode.ORT_SEQUENTIAL
    so.graph_optimization_level = ort.GraphOptimizationLevel.ORT_ENABLE_ALL
    return ort.InferenceSession(path, so, providers=["CPUExecutionProvider"])


def sigmoid(x):
    return 1.0 / (1.0 + np.exp(-np.clip(x, -50, 50)))


def parse_sc097(o):
    r = o.astype(np.float64)
    lane = r[4955:5219].reshape(4, 33, 2)
    lprob = sigmoid(r[5483:5491])[1::2]
    edge = r[5491:5623].reshape(2, 33, 2)
    leads = np.zeros((3, 4)); lstd = np.zeros((3, 4))
    for i in range(3):
        h = int(np.argmax([r[5755 + 51 * k + 48 + i] for k in range(2)]))
        base = 5755 + 51 * h
        tr = r[base:base + 24].reshape(6, 4)
        leads[i] = tr[0]
        lstd[i] = np.exp(r[base + 24:base + 48].reshape(6, 4)[0])
    lead_prob = sigmoid(r[5857:5860])
    pose = r[5948:5954]
    edge_std = np.exp(r[5623:5755].reshape(2, 33, 2)[:, 0, 0])
    return dict(lane=lane, lprob=lprob, edge=edge, edge_std=edge_std, lead=leads, lead_std=lstd,
                lead_prob=lead_prob, pose=pose, hidden=r[5992:6504].astype(np.float32))


def parse_vision(o):  # v0.10.x / v0.11.x driving_vision slices (onnx metadata output_slices)
    r = o.astype(np.float64)
    lane = r[117:381].reshape(4, 33, 2)
    lprob = sigmoid(r[645:653])[1::2]
    edge = r[653:785].reshape(2, 33, 2)
    edge_std = np.exp(r[785:917].reshape(2, 33, 2)[:, 0, 0])
    lead = r[917:989].reshape(3, 6, 4)[:, 0, :]
    lstd = np.exp(r[989:1061].reshape(3, 6, 4)[:, 0, :])
    lead_prob = sigmoid(r[1061:1064])
    pose = r[87:93]
    return dict(lane=lane, lprob=lprob, edge=edge, edge_std=edge_std, lead=lead, lead_std=lstd,
                lead_prob=lead_prob, pose=pose)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("ride")
    ap.add_argument("start", type=float)
    ap.add_argument("dur", type=float)
    ap.add_argument("out")
    ap.add_argument("--models", default="sc097,v0111,v0101")
    ap.add_argument("--threads", type=int, default=3)
    a = ap.parse_args()
    slug = os.path.basename(os.path.normpath(a.ride))
    cam = json.load(open(os.path.join(a.ride, "camera-replay.json")))
    cal = json.load(open(os.path.join(a.ride, "calib.json")))
    K = np.array([[cam["focal_px"], 0, cam["cx"]], [0, cam["focal_px"], cam["cy"]], [0, 0, 1.0]])
    rpy = (cal.get("roll", 0.0), cal.get("pitch", 0.0), cal.get("yaw", 0.0))
    M_med = warp_matrix(K, rpy, MED_K)
    M_big = warp_matrix(K, rpy, SBIG_K)
    names = a.models.split(",")
    warps = {}
    for kind in ("bilinear", "nearest"):
        warps[kind] = (make_warps(M_med, kind), make_warps(M_big, kind))
    kind_of = {"sc097": "bilinear", "v0111": "nearest", "v0111bl": "bilinear", "v0101": "bilinear"}
    S = {n: sess(MODELS[n], a.threads) for n in names}
    # state
    sc_in = None
    if "sc097" in names:
        sc_in = {"input_imgs": np.zeros((1, 12, 128, 256), np.float32),
                 "big_input_imgs": np.zeros((1, 12, 128, 256), np.float32),
                 "desire": np.zeros((1, 100, 8), np.float32),
                 "traffic_convention": np.array([[1, 0]], np.float32),
                 "lateral_control_params": np.array([[0.0, 0.4]], np.float32),
                 "prev_desired_curv": np.zeros((1, 100, 1), np.float32),
                 "features_buffer": np.zeros((1, 99, 512), np.float32)}
    rings = {n: {"img": [np.zeros((6, 128, 256), np.uint8)] * 5, "big": [np.zeros((6, 128, 256), np.uint8)] * 5}
             for n in names if n != "sc097"}
    rec = {n: {} for n in names}
    tms = {n: [] for n in names}
    fps = cam.get("fps", 20)
    start = int(round(a.start * fps)); n = int(round(a.dur * fps))
    t0 = time.time()
    k = 0
    import glob
    mj = sorted(glob.glob(os.path.join(a.ride, "*.mjpeg")))[0]
    for y, u, v in frames(mj, start, n):
        packed = {}
        for kind in set(kind_of[m] for m in names):
            (wy, wuv), (by, buv) = warps[kind]
            packed[kind] = (pack(wy, wuv, y, u, v), pack(by, buv, y, u, v))
        for m in names:
            img, big = packed[kind_of[m]]
            ts = time.perf_counter()
            if m == "sc097":
                sc_in["input_imgs"][0, :6] = sc_in["input_imgs"][0, 6:]
                sc_in["input_imgs"][0, 6:] = img
                sc_in["big_input_imgs"][0, :6] = sc_in["big_input_imgs"][0, 6:]
                sc_in["big_input_imgs"][0, 6:] = big
                o = S[m].run(None, sc_in)[0][0]
                p = parse_sc097(o)
                sc_in["features_buffer"][0, :-1] = sc_in["features_buffer"][0, 1:]
                sc_in["features_buffer"][0, -1] = p.pop("hidden")
                sc_in["prev_desired_curv"][0, :-1] = sc_in["prev_desired_curv"][0, 1:]
                sc_in["prev_desired_curv"][0, -1, 0] = o[5990]
                sc_in["lateral_control_params"][0, 0] = float(np.clip(p["pose"][0], 0, 70))
            else:
                r = rings[m]
                r["img"] = r["img"][1:] + [img]
                r["big"] = r["big"][1:] + [big]
                feeds = {"img": np.concatenate([r["img"][0], r["img"][-1]])[None],
                         "big_img": np.concatenate([r["big"][0], r["big"][-1]])[None]}
                o = S[m].run(None, feeds)[0][0]
                p = parse_vision(o)
            tms[m].append((time.perf_counter() - ts) * 1e3)
            for kk, vv in p.items():
                rec[m].setdefault(kk, []).append(vv)
        k += 1
        if k % 500 == 0:
            print(f"{slug}: {k}/{n} frames, {time.time() - t0:.0f} s, " +
                  ", ".join(f"{m} {np.median(tms[m]):.1f} ms" for m in names), flush=True)
    out = {"meta": json.dumps(dict(ride=slug, start=a.start, dur=a.dur, frames=k, start_frame=start,
                                   rpy=rpy, K=K.tolist(), models=names, kind=kind_of,
                                   ms={m: float(np.median(tms[m])) for m in names}))}
    for m in names:
        for kk, vv in rec[m].items():
            out[f"{m}/{kk}"] = np.array(vv, np.float32)
    np.savez_compressed(a.out, **out)
    print("saved", a.out, k, "frames", flush=True)


if __name__ == "__main__":
    main()
