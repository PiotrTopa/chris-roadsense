#!/usr/bin/env python3
"""Inspect openpilot driving ONNX files: I/O tensors, metadata output_slices, op histogram,
parameter count, MACs (Conv/Gemm/MatMul from shape inference), opset/domains.

usage: python -I inspect_models.py MODEL.onnx [...]   (prints JSON-ish text)
Loads files only with onnx (no pickle except the restricted reader in safe_meta.py).
"""
import collections
import json
import os
import sys

import numpy as np
import onnx
from onnx import shape_inference, numpy_helper

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from safe_meta import onnx_meta  # noqa: E402

DT = {v: k for k, v in onnx.TensorProto.DataType.items()}


def vi_desc(v):
    t = v.type.tensor_type
    dims = [d.dim_value if d.HasField("dim_value") else d.dim_param for d in t.shape.dim]
    return v.name, dims, DT[t.elem_type]


def macs(model):
    m = shape_inference.infer_shapes(model, data_prop=True)
    shapes = {}
    for v in list(m.graph.value_info) + list(m.graph.input) + list(m.graph.output):
        shapes[v.name] = [d.dim_value for d in v.type.tensor_type.shape.dim]
    for i in m.graph.initializer:
        shapes[i.name] = list(i.dims)
    tot = collections.Counter()
    unknown = 0
    for n in m.graph.node:
        try:
            if n.op_type == "Conv":
                w = shapes[n.input[1]]
                o = shapes[n.output[0]]
                tot["Conv"] += int(np.prod(o)) * int(np.prod(w[1:]))
            elif n.op_type == "Gemm":
                a = shapes[n.input[0]]; o = shapes[n.output[0]]
                at = {x.name: x.i for x in n.attribute}
                k = a[0] if at.get("transA", 0) else a[-1]
                tot["Gemm"] += int(np.prod(o)) * k
            elif n.op_type == "MatMul":
                a = shapes[n.input[0]]; o = shapes[n.output[0]]
                tot["MatMul"] += int(np.prod(o)) * a[-1]
        except (KeyError, IndexError):
            unknown += 1
    return tot, unknown


def main():
    for path in sys.argv[1:]:
        m = onnx.load(path)
        md = onnx_meta(m)
        print("=" * 100)
        print(path, os.path.getsize(path), "bytes")
        print("ir", m.ir_version, "opsets", [(o.domain or "ai.onnx", o.version) for o in m.opset_import],
              "producer", m.producer_name, m.producer_version)
        print("metadata keys:", sorted(md))
        for k, v in md.items():
            if k != "output_slices":
                s = str(v)
                print(f"  meta {k} = {s[:300]}")
        inits = {i.name for i in m.graph.initializer}
        print("inputs:")
        for v in m.graph.input:
            if v.name not in inits:
                print("  ", vi_desc(v))
        print("outputs:")
        for v in m.graph.output:
            print("  ", vi_desc(v))
        if "output_slices" in md:
            print("output_slices:")
            for k, s in md["output_slices"].items():
                ln = (s.stop - s.start) if s.stop is not None else None
                print(f"   {k:28s} {s.start:6d} .. {('end' if s.stop is None else s.stop - 1)!s:>6}  len {ln}")
        hist = collections.Counter(n.op_type for n in m.graph.node)
        print("nodes", len(m.graph.node), "op histogram:", dict(hist.most_common()))
        dts = collections.Counter(DT[i.data_type] for i in m.graph.initializer)
        npar = sum(int(np.prod(i.dims)) for i in m.graph.initializer)
        print("initializers", len(m.graph.initializer), "params", npar, "dtypes", dict(dts))
        # constant nodes params too
        try:
            t, unk = macs(m)
            print("MACs:", {k: f"{v/1e9:.3f} G" for k, v in t.items()}, "total %.3f GMAC" % (sum(t.values()) / 1e9),
                  "(nodes with unknown shapes: %d)" % unk)
        except Exception as e:  # noqa
            print("MACs failed:", e)
        # Conv kernel/group mix
        convs = collections.Counter()
        for n in m.graph.node:
            if n.op_type == "Conv":
                at = {a.name: (list(a.ints) if a.ints else a.i) for a in n.attribute}
                convs[(tuple(at.get("kernel_shape", [])), "dw" if at.get("group", 1) > 1 else "g1",
                       tuple(at.get("strides", [1, 1])))] += 1
        if convs:
            print("conv kinds (kernel, group, stride):", dict(convs.most_common()))


if __name__ == "__main__":
    main()
