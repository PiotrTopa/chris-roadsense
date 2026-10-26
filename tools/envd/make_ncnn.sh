#!/bin/sh
# Convert the envd image tower (ONNX from tools/envd/export_onnx.py) to ncnn for `envd --backend ncnn`.
#   usage: tools/envd/make_ncnn.sh IN.onnx OUT_STEM [PYTHON_VENV]
#   writes OUT_STEM.param + OUT_STEM.bin (input blob in0, output out0, fp32 weights; envd packs fp16 on the GPU)
# Steps: onnxsim -> pnnx (ONNX -> ncnn). Needs onnx, onnxsim, pnnx (+ torch) in PYTHON_VENV
# (default .venv-models, created by scripts/fetch-models.sh). The input shape is read from the ONNX graph.
set -eu
[ $# -ge 2 ] || { sed -n '2,7p' "$0"; exit 2; }
IN=$(realpath "$1")
OUT=$(realpath -m "$2")
HERE=$(cd "$(dirname "$0")" && pwd)
VENV=$(realpath "${3:-$HERE/../../.venv-models}")
W=$(mktemp -d "${TMPDIR:-/tmp}/envd-ncnn.XXXXXX")
trap 'rm -rf "$W"' EXIT
SHAPE=$("$VENV/bin/python" -I - "$IN" <<'PY'
import sys, onnx
g = onnx.load(sys.argv[1]).graph
print("[" + ",".join(str(d.dim_value or 1) for d in g.input[0].type.tensor_type.shape.dim) + "]")
PY
)
cp "$IN" "$W/m.onnx"
(cd "$W" && "$VENV/bin/python" -I -m onnxsim m.onnx m-sim.onnx >/dev/null &&
    "$VENV/bin/pnnx" m-sim.onnx inputshape="$SHAPE" fp16=0 >pnnx.log 2>&1) || { tail -20 "$W/pnnx.log"; exit 1; }
cp "$W/m_sim.ncnn.param" "$OUT.param"
cp "$W/m_sim.ncnn.bin" "$OUT.bin"
echo "wrote $OUT.param $OUT.bin (input $SHAPE)"
grep -c . "$OUT.param" | sed 's/^/layers+2: /'
