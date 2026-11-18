#!/usr/bin/env python3
"""ORT CPU latency of ONNX models with random realistic inputs (uint8 images 0..255, small floats).

usage: python -I bench_ort.py --threads 2,4 --runs 30 MODEL.onnx [...]
Prints median / p10 / p90 ms per thread count. Sequential execution, ORT_ENABLE_ALL, like openpilot's
onnxmodel.py. Also: --dep checks which outputs change when the recurrent state inputs are perturbed
(for the merged driving_supercombo graphs).
"""
import argparse
import os
import sys
import time

import numpy as np
import onnxruntime as ort

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

NP = {"tensor(float)": np.float32, "tensor(float16)": np.float16, "tensor(uint8)": np.uint8,
      "tensor(int64)": np.int64}


def make_inputs(sess, rng):
    feeds = {}
    for i in sess.get_inputs():
        dt = NP[i.type]
        shp = [d if isinstance(d, int) else 1 for d in i.shape]
        if dt == np.uint8:
            feeds[i.name] = rng.integers(0, 256, size=shp, dtype=np.uint8)
        elif "img" in i.name:
            feeds[i.name] = rng.integers(0, 256, size=shp).astype(dt)
        else:
            feeds[i.name] = (rng.standard_normal(shp) * 0.1).astype(dt)
    return feeds


def bench(path, threads, runs):
    so = ort.SessionOptions()
    so.intra_op_num_threads = threads
    so.inter_op_num_threads = 1
    so.execution_mode = ort.ExecutionMode.ORT_SEQUENTIAL
    so.graph_optimization_level = ort.GraphOptimizationLevel.ORT_ENABLE_ALL
    s = ort.InferenceSession(path, so, providers=["CPUExecutionProvider"])
    feeds = make_inputs(s, np.random.default_rng(0))
    for _ in range(3):
        s.run(None, feeds)
    t = []
    for _ in range(runs):
        t0 = time.perf_counter()
        s.run(None, feeds)
        t.append((time.perf_counter() - t0) * 1e3)
    t = np.array(t)
    return np.median(t), np.percentile(t, 10), np.percentile(t, 90)


def dep(path, slices):
    """Perturb each non-image input; report max |delta| per output slice."""
    s = ort.InferenceSession(path, providers=["CPUExecutionProvider"])
    rng = np.random.default_rng(1)
    base = make_inputs(s, rng)
    o0 = s.run(["outputs"], base)[0].astype(np.float64)[0]
    for name in base:
        if "img" in name and "state" not in name and "new" not in name:
            continue
        f = dict(base)
        x = base[name]
        if x.dtype == np.uint8:
            f[name] = rng.integers(0, 256, size=x.shape, dtype=np.uint8)
        else:
            f[name] = (x + rng.standard_normal(x.shape).astype(x.dtype) * 0.5).astype(x.dtype)
        o1 = s.run(["outputs"], f)[0].astype(np.float64)[0]
        d = {k: float(np.max(np.abs(o1[v] - o0[v]))) for k, v in slices.items() if k != "pad"}
        print(f"  perturb {name:20s}: " + ", ".join(f"{k}={v:.3g}" for k, v in d.items()))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("models", nargs="+")
    ap.add_argument("--threads", default="2,4")
    ap.add_argument("--runs", type=int, default=30)
    ap.add_argument("--dep", action="store_true")
    ap.add_argument("--interleave", action="store_true")
    a = ap.parse_args()
    print("onnxruntime", ort.__version__)
    if a.interleave:
        # all sessions alive, round-robin 5-run blocks, so background load hits every model equally
        for th in [int(x) for x in a.threads.split(",")]:
            sess = {}
            for p in a.models:
                so = ort.SessionOptions()
                so.intra_op_num_threads = th
                so.inter_op_num_threads = 1
                so.execution_mode = ort.ExecutionMode.ORT_SEQUENTIAL
                so.graph_optimization_level = ort.GraphOptimizationLevel.ORT_ENABLE_ALL
                s = ort.InferenceSession(p, so, providers=["CPUExecutionProvider"])
                f = make_inputs(s, np.random.default_rng(0))
                for _ in range(3):
                    s.run(None, f)
                sess[p] = (s, f, [])
            for _ in range(a.runs // 5):
                for p, (s, f, t) in sess.items():
                    for _ in range(5):
                        t0 = time.perf_counter(); s.run(None, f); t.append((time.perf_counter() - t0) * 1e3)
            for p, (s, f, t) in sess.items():
                t = np.array(t)
                print(f"{p}\tthreads={th}\tmin={t.min():.1f}\tmedian={np.median(t):.1f} ms\tp10={np.percentile(t,10):.1f}"
                      f"\tp90={np.percentile(t,90):.1f}\tn={len(t)}", flush=True)
        return
    for p in a.models:
        if a.dep:
            import onnx
            from safe_meta import onnx_meta
            sl = onnx_meta(onnx.load(p, load_external_data=False))["output_slices"]
            print(p)
            dep(p, sl)
            continue
        for th in [int(x) for x in a.threads.split(",")]:
            med, p10, p90 = bench(p, th, a.runs)
            print(f"{p}\tthreads={th}\tmedian={med:.1f} ms\tp10={p10:.1f}\tp90={p90:.1f}", flush=True)


if __name__ == "__main__":
    main()
