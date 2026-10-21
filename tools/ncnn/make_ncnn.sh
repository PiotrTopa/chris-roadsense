#!/bin/sh
# Convert supercombo v0.9.7 (fp32 ONNX) into the ncnn model used by `modeld --backend ncnn`.
#   usage: tools/ncnn/make_ncnn.sh MODEL-fp32.onnx OUT_STEM [PYTHON_VENV]
#   writes OUT_STEM.param and OUT_STEM.bin (e.g. models/supercombo-ncnn.{param,bin})
# Steps: onnxsim -> pnnx (ONNX -> ncnn, fp32 weights) -> tools/ncnn/fix_ncnn.py (three graph fixes,
# see there). Needs onnx, onnxsim, pnnx (+ torch, which pnnx links against) in PYTHON_VENV
# (default .venv-models, created by scripts/fetch-models.sh). Deterministic: the same inputs and
# tool versions (onnx 1.23.2, onnxsim 0.8.1, pnnx 20261020) give byte-identical outputs.
set -eu
[ $# -ge 2 ] || { sed -n '2,8p' "$0"; exit 2; }
HERE=$(cd "$(dirname "$0")" && pwd)
ONNX=$(realpath "$1")
OUT=$(realpath -m "$2")
VENV=$(realpath "${3:-$HERE/../../.venv-models}")
W=$(mktemp -d "${TMPDIR:-/tmp}/sc-ncnn.XXXXXX")
trap 'rm -rf "$W"' EXIT
cd "$W"
cp "$ONNX" sc.onnx
"$VENV/bin/python" -I -m onnxsim sc.onnx sc-sim.onnx >/dev/null
# pnnx names its outputs after the input file (sc-sim -> sc_sim.ncnn.*)
"$VENV/bin/pnnx" sc-sim.onnx inputshape="[1,12,128,256],[1,12,128,256],[1,100,8],[1,2],[1,2],[1,100,1],[1,99,512]" fp16=0 >pnnx.log 2>&1 \
    || { tail -20 pnnx.log; exit 1; }
"$VENV/bin/python" -I "$HERE/fix_ncnn.py" sc-sim.onnx sc_sim.ncnn.param sc_sim.ncnn.bin "$OUT.param" "$OUT.bin"
