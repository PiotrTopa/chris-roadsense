#!/usr/bin/env python3
"""fp16 range scan: run an fp32 ONNX vision model with every intermediate tensor exposed as an output,
on real inputs, and list tensors whose max |value| exceeds fp16 limits (65504) or comes close.

usage: python -I range_scan.py MODEL-fp32.onnx INPUTS.npz [--limit 16384]
"""
import argparse
import collections

import numpy as np
import onnx
import onnxruntime as ort


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("model")
    ap.add_argument("inputs")
    ap.add_argument("--limit", type=float, default=16384)
    a = ap.parse_args()
    m = onnx.load(a.model)
    m = onnx.shape_inference.infer_shapes(m)
    producer = {}
    for n in m.graph.node:
        for o in n.output:
            producer[o] = (n.op_type, n.name)
    known = {o.name for o in m.graph.output}
    vis = {v.name: v for v in m.graph.value_info}
    for n in m.graph.node:
        for o in n.output:
            if o in known or not o:
                continue
            if o in vis and vis[o].type.tensor_type.elem_type == onnx.TensorProto.FLOAT:
                m.graph.output.append(vis[o])
                known.add(o)
    so = ort.SessionOptions()
    so.graph_optimization_level = ort.GraphOptimizationLevel.ORT_DISABLE_ALL
    so.intra_op_num_threads = 4
    s = ort.InferenceSession(m.SerializeToString(), so, providers=["CPUExecutionProvider"])
    z = np.load(a.inputs)
    names = [o.name for o in s.get_outputs()]
    mx = collections.defaultdict(float)
    for i in range(len(z["img"])):
        outs = s.run(None, {"img": z["img"][i:i + 1], "big_img": z["big_img"][i:i + 1]})
        for nme, v in zip(names, outs):
            if v.dtype.kind == "f" and v.size:
                mx[nme] = max(mx[nme], float(np.max(np.abs(v))))
    print(f"{len(z['img'])} samples, {len(mx)} float tensors; overall max {max(mx.values()):.4g}")
    rows = sorted(((v, k) for k, v in mx.items() if v > a.limit), reverse=True)
    print(f"tensors above {a.limit:g}: {len(rows)}  (fp16 max 65504)")
    for v, k in rows:
        print(f"  {v:12.4g}  {k:40s}  produced by {producer.get(k, ('?', '?'))}")


if __name__ == "__main__":
    main()
