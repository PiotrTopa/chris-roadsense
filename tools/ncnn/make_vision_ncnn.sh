#!/bin/sh
# Convert openpilot v0.11.1 driving_vision (fp32 ONNX) into the ncnn model for `modeld --road-model vision
# --backend ncnn`.
#   usage: tools/ncnn/make_vision_ncnn.sh driving_vision-fp32.onnx OUT_STEM [PYTHON_VENV]
#   writes OUT_STEM.param and OUT_STEM.bin (e.g. driving_vision-ncnn.{param,bin})
# Steps: onnxsim -> pnnx (uint8 inputs, fp32 weights) -> tools/ncnn/fix_vision_ncnn.py (drops the two
# uint8 -> float casts). Tools as for make_ncnn.sh (onnx 1.23.2, onnxsim 0.8.1, pnnx 20261020 in
# PYTHON_VENV, default .venv-models). The fp32 ONNX comes from the release's fp16 graph through
# tools/onnx_fp16_to_fp32.py.
set -eu
[ $# -ge 2 ] || { sed -n '2,9p' "$0"; exit 2; }
HERE=$(cd "$(dirname "$0")" && pwd)
ONNX=$(realpath "$1")
OUT=$(realpath -m "$2")
VENV=$(realpath "${3:-$HERE/../../.venv-models}")
W=$(mktemp -d "${TMPDIR:-/tmp}/dv-ncnn.XXXXXX")
trap 'rm -rf "$W"' EXIT
cd "$W"
cp "$ONNX" dv.onnx
"$VENV/bin/python" -I -m onnxsim dv.onnx dv-sim.onnx >/dev/null
"$VENV/bin/pnnx" dv-sim.onnx inputshape="[1,12,128,256]u8,[1,12,128,256]u8" fp16=0 >pnnx.log 2>&1 \
    || { tail -20 pnnx.log; exit 1; }
"$VENV/bin/python" -I "$HERE/fix_vision_ncnn.py" dv_sim.ncnn.param "$OUT.param"
cp dv_sim.ncnn.bin "$OUT.bin"
