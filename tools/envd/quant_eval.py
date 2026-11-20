#!/usr/bin/env python3
"""Same scores at a lower cost? An image-tower variant (int8 / fp16 ONNX) against the fp32 tower, both through
the real C++ pipeline (`envd_replay --reset-each --emb`) on the same inputs (lane L48-E, candidate B3).

    quant_eval.py --replay build-l48e/envd_replay --ref models/envd-image.onnx --cand vitb32_int8.onnx \
        --classes envd-classes-probe.json --bdd-npz sq_bdd_vitb32_openai.npz --bdd-root ~/datasets/dashcam/bdd100k_val_fo \
        --n 2000 [--nv12 frames.nv12 --nv12-size 1280x720] --out result.json

Reports the embedding cosine (min / p1 / mean), per-group frame-label agreement and, on BDD split B (the probe's
held-out half), the macro recall of each tower against the BDD labels (zeroshot_eval.py metric).
"""
import argparse
import json
import os
import subprocess
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from zeroshot_eval import MAP, evaluate, scores, split_of  # noqa: E402

COLS = {"weather": "weather", "light": "timeofday", "place": "scene"}


def run(replay, model, classes, args, threads):
    p = subprocess.run([replay, "--model", model, "--classes", classes, "--threads", str(threads), "--reset-each",
                        "--emb", "--no-timing"] + args, capture_output=True, text=True, check=True)
    return np.array([json.loads(l)["emb"] for l in p.stdout.splitlines() if l.startswith("{")], np.float32)


def compare(e0, e1, doc):
    cos = (e0 * e1).sum(1) / np.linalg.norm(e0, axis=1) / np.linalg.norm(e1, axis=1)
    r = {"n": len(e0), "cos_min": float(cos.min()), "cos_p1": float(np.percentile(cos, 1)), "cos_mean": float(cos.mean())}
    for g in doc["groups"]:
        _, p0 = scores(e0, doc, g)
        _, p1 = scores(e1, doc, g)
        r["agree_" + g] = float((p0.argmax(1) == p1.argmax(1)).mean())
        r["maxprob_diff_" + g] = float(np.abs(p0 - p1).max())
    return r


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--replay", required=True)
    ap.add_argument("--ref", required=True)
    ap.add_argument("--cand", required=True)
    ap.add_argument("--classes", required=True)
    ap.add_argument("--bdd-npz")
    ap.add_argument("--bdd-root")
    ap.add_argument("--n", type=int, default=2000)
    ap.add_argument("--nv12")
    ap.add_argument("--nv12-size", default="1280x720")
    ap.add_argument("--threads", type=int, default=4)
    ap.add_argument("--out", required=True)
    a = ap.parse_args()
    doc = json.load(open(a.classes))
    res = {"ref": a.ref, "cand": a.cand, "classes": a.classes}
    if a.nv12:
        args = ["--nv12", a.nv12_size, a.nv12]
        e0, e1 = run(a.replay, a.ref, a.classes, args, a.threads), run(a.replay, a.cand, a.classes, args, a.threads)
        res["rides"] = compare(e0, e1, doc)
        print("rides", res["rides"], flush=True)
    if a.bdd_npz:
        z = np.load(a.bdd_npz, allow_pickle=True)
        files = [str(f) for f in z["files"]]
        sel = [i for i, f in enumerate(files) if split_of(f) == "B"][: a.n]
        paths = [os.path.join(a.bdd_root, files[i]) for i in sel]
        e0, e1 = run(a.replay, a.ref, a.classes, paths, a.threads), run(a.replay, a.cand, a.classes, paths, a.threads)
        res["bdd"] = compare(e0, e1, doc)
        for g, col in COLS.items():
            labs = [str(z[col][i]) for i in sel]
            keep = [k for k, l in enumerate(labs) if l in MAP["bdd"][col]]
            lab = [MAP["bdd"][col][labs[k]] for k in keep]
            for tag, e in (("ref", e0), ("cand", e1)):
                cls, p = scores(e[keep], doc, g)
                _, macro, micro = evaluate(None, lab, cls, p)
                res["bdd"]["%s_macro_%s" % (tag, g)] = macro
                res["bdd"]["%s_micro_%s" % (tag, g)] = micro
        print("bdd", res["bdd"], flush=True)
    json.dump(res, open(a.out, "w"), indent=1)


if __name__ == "__main__":
    main()
