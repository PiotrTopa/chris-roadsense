#!/usr/bin/env python3
# Convert every float16 initializer/Constant/Cast/value_info in an ONNX graph to float32.
# ONNX Runtime CPU on aarch64 has few fp16 kernels: supercombo v0.9.7 runs 300 ms as fp16
# and 54 ms as fp32 on the Poco F1 (4 threads on the Cortex-A75 cores).
# usage: onnx_fp16_to_fp32.py in.onnx out.onnx   (needs: pip install onnx numpy)
import sys, numpy as np, onnx
from onnx import numpy_helper, TensorProto
m = onnx.load(sys.argv[1])
g = m.graph
n_init = 0
for t in g.initializer:
    if t.data_type == TensorProto.FLOAT16:
        a = numpy_helper.to_array(t).astype(np.float32)
        t.CopyFrom(numpy_helper.from_array(a, t.name)); n_init += 1
n_node = 0
for node in g.node:
    if node.op_type == "Cast":
        for at in node.attribute:
            if at.name == "to" and at.i == TensorProto.FLOAT16:
                at.i = TensorProto.FLOAT; n_node += 1
    if node.op_type == "Constant":
        for at in node.attribute:
            if at.name == "value" and at.t.data_type == TensorProto.FLOAT16:
                a = numpy_helper.to_array(at.t).astype(np.float32)
                at.t.CopyFrom(numpy_helper.from_array(a, at.t.name)); n_node += 1
def fix(vi):
    tt = vi.type.tensor_type
    if tt.elem_type == TensorProto.FLOAT16: tt.elem_type = TensorProto.FLOAT; return 1
    return 0
n_vi = sum(fix(v) for v in list(g.input) + list(g.output) + list(g.value_info))
print(f"initializers {n_init} nodes {n_node} value_infos {n_vi} opset {[ (o.domain,o.version) for o in m.opset_import]}")
onnx.checker.check_model(m)
onnx.save(m, sys.argv[2])
