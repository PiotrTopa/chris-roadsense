#!/usr/bin/env python3
"""C++ vision path (modeld_replay --road-model vision) against the Python reference of this directory.

usage: python -I check_cpp_vision.py RIDE_DIR DUMP_DIR ROAD.jsonl [--frames N]

Run first (same frames: the ride's first N frames, every pair):
  ffmpeg -f mjpeg -framerate 20 -i RIDE.mjpeg -frames:v N -vf scale=in_range=full:out_range=full,format=nv12 \
    -f rawvideo - | modeld_replay /dev/stdin --width 1280 --height 720 --fps 20 --focal F --calib calib.json \
    --road-model vision --model driving_vision-fp32.onnx --policy driving_policy-fp32.onnx \
    --dump-input DUMP_DIR --dump-runs N --dump-tensors > ROAD.jsonl

Checks per run: (1) packer: the C++ input tensors (DUMP_DIR/runNNN_in0/in1.f32) against replay_compare.py's
nearest warp + pack of the same decoded frames (pair = frame k-4, k); (2) network: the C++ raw outputs
(runNNN_out.f32) against ONNX Runtime in Python on the Python inputs; (3) parser: the JSON fields against
replay_compare.parse_vision of the C++ raw outputs; (4) policy: the JSON plan against the policy run in
Python on the same 5 Hz feature history (zeros where the history is not yet filled).
"""
import argparse
import glob
import json
import os
import sys

import numpy as np
import onnxruntime as ort

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import replay_compare as rc  # noqa: E402

D = os.environ.get("OP_MODELS", os.path.expanduser("~/datasets/openpilot-models"))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("ride"); ap.add_argument("dump"); ap.add_argument("road")
    ap.add_argument("--frames", type=int, default=30)
    a = ap.parse_args()
    cam = json.load(open(os.path.join(a.ride, "camera-replay.json")))
    cal = json.load(open(os.path.join(a.ride, "calib.json")))
    K = np.array([[cam["focal_px"], 0, cam["cx"] if "cx" in cam else 640], [0, cam["focal_px"], cam.get("cy", 360)], [0, 0, 1.0]])
    rpy = (cal.get("roll", 0.0), cal.get("pitch", 0.0), cal.get("yaw", 0.0))
    (wy, wuv), (by, buv) = rc.make_warps(rc.warp_matrix(K, rpy, rc.MED_K), "nearest"), \
        rc.make_warps(rc.warp_matrix(K, rpy, rc.SBIG_K), "nearest")
    mj = sorted(glob.glob(os.path.join(a.ride, "*.mjpeg")))[0]
    packed = [(rc.pack(wy, wuv, y, u, v), rc.pack(by, buv, y, u, v)) for y, u, v in rc.frames(mj, 0, a.frames)]
    road = [m for m in map(json.loads, open(a.road)) if m.get("type") == "road"]
    sv = rc.sess(f"{D}/v0.11.1/driving_vision-fp32.onnx", 4)
    sp = rc.sess(f"{D}/v0.11.1/driving_policy-fp32.onnx", 1)
    hist = []  # (seq, hidden) of C++ raw outputs, 20 Hz
    worst = dict(pack=0, net=0.0, parse=0.0, plan=0.0)
    n = 0
    for i, m in enumerate(road):
        k = m["seq"] - 1
        base = os.path.join(a.dump, f"run{i:03d}")
        if not os.path.exists(base + "_in0.f32"):
            break
        cin0 = np.fromfile(base + "_in0.f32", np.float32).reshape(12, 128, 256)
        cin1 = np.fromfile(base + "_in1.f32", np.float32).reshape(12, 128, 256)
        raw = np.fromfile(base + "_out.f32", np.float32)
        assert m["frame_seq_prev"] - 1 == k - 4, (m["frame_seq_prev"], k)
        pin0 = np.concatenate([packed[k - 4][0], packed[k][0]]).astype(np.float32)
        pin1 = np.concatenate([packed[k - 4][1], packed[k][1]]).astype(np.float32)
        worst["pack"] = max(worst["pack"], int((cin0 != pin0).sum() + (cin1 != pin1).sum()))
        ref = sv.run(None, {"img": cin0.astype(np.uint8)[None], "big_img": cin1.astype(np.uint8)[None]})[0][0]
        worst["net"] = max(worst["net"], float(np.abs(ref - raw).max()))
        refp = sv.run(None, {"img": pin0.astype(np.uint8)[None], "big_img": pin1.astype(np.uint8)[None]})[0][0]
        lp = rc.parse_vision(refp)["lane"][:, :, 0]
        worst["pack_lane"] = max(worst.get("pack_lane", 0.0), float(np.abs(lp - rc.parse_vision(raw)["lane"][:, :, 0])[:, :24].max()))
        p = rc.parse_vision(raw)
        d = [np.abs(np.array([l["y"] for l in m["lane_lines"]]) - p["lane"][:, :, 0]).max(),
             np.abs(np.array([l["z"] for l in m["lane_lines"]]) - p["lane"][:, :, 1]).max(),
             np.abs(np.array([l["prob"] for l in m["lane_lines"]]) - p["lprob"]).max(),
             np.abs(np.array([e["y"] for e in m["road_edges"]]) - p["edge"][:, :, 0]).max(),
             np.abs(np.array([[L["x"][0], L["y"][0], L["v"][0], L["a"][0]] for L in m["leads"]]) - p["lead"]).max(),
             np.abs(np.array([L["prob"] for L in m["leads"]]) - p["lead_prob"]).max(),
             abs(m["pose"]["vx"] - p["pose"][0])]
        rel = max(float(x) / 1.0 for x in d)
        worst["parse"] = max(worst["parse"], rel)
        # policy on the 20 Hz history: rows k, k-4, ..., zeros before the first run
        hist.append((k, raw[1064:1576].copy()))
        byk = dict(hist)
        feat = np.zeros((1, 25, 512), np.float32)
        for r in range(25):
            kk = k - 4 * r
            if kk in byk:
                feat[0, 24 - r] = byk[kk]
        po = sp.run(None, {"desire_pulse": np.zeros((1, 25, 8), np.float32),
                           "traffic_convention": np.array([[1, 0]], np.float32), "features_buffer": feat})[0][0]
        plan = po[:495].reshape(33, 15)
        dp = max((np.abs(np.array(m["plan"]["x"]) - plan[:, 0]) / (1e-3 + np.abs(plan[:, 0]))).max(),
                 (np.abs(np.array(m["plan"]["y"]) - plan[:, 1]) / (1e-3 + np.abs(plan[:, 1]))).max())
        if os.environ.get("VERBOSE"):
            print(i, k, float(dp), np.abs(np.array(m["plan"]["x"]) - plan[:, 0]).max(), plan[-1, 0])
        worst["plan"] = max(worst["plan"], float(dp))
        worst["plan_bad"] = worst.get("plan_bad", 0) + int(dp > 1e-3)
        n += 1
    print(f"{n} runs: packer differing input values (max per run) {worst['pack']}; network |C++ - Python ORT| max "
          f"{worst['net']:.3g} (same inputs); lane y up to 100 m from the Python-packed inputs vs C++ {worst['pack_lane']:.3g} m; parser max abs diff {worst['parse']:.3g} (JSON prints 5 significant digits); "
          f"policy plan x/y max relative diff {worst['plan']:.3g}, runs above 1e-3: {worst['plan_bad']}")


if __name__ == "__main__":
    main()
