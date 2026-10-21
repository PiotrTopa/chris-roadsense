#!/bin/sh
# Download the model files roadsense needs into ./models (not committed).
set -eu
cd "$(dirname "$0")/.."
mkdir -p models
fetch() { # url sha256 dest
    if [ -f "$3" ] && [ "$(sha256sum "$3" | cut -d' ' -f1)" = "$2" ]; then
        echo "ok   $3"; return 0
    fi
    echo "get  $3"
    curl -fsSL -o "$3.part" "$1"
    got=$(sha256sum "$3.part" | cut -d' ' -f1)
    [ "$got" = "$2" ] || { echo "sha256 mismatch for $3: $got"; rm -f "$3.part"; exit 1; }
    mv "$3.part" "$3"
}
# openpilot driving model, MIT (comma.ai), openpilot tag v0.9.7
fetch "https://github.com/commaai/openpilot/raw/v0.9.7/selfdrive/modeld/models/supercombo.onnx" \
      b31b504bc0b440d3bc72967507a00eb4f112285626fbfb3135011500325ee6d6 \
      models/supercombo.onnx

# ONNX Runtime CPU needs the fp32 graph (see tools/onnx_fp16_to_fp32.py). The conversion
# needs the Python `onnx` package, which has no musl wheel: run this script on the dev box
# and copy models/ to the phone (scripts/push-models.sh).
if [ ! -f models/supercombo-fp32.onnx ]; then
    PY=python3
    if ! $PY -c 'import onnx' 2>/dev/null; then
        [ -x .venv-models/bin/python ] || python3 -m venv .venv-models
        .venv-models/bin/pip -q install onnx numpy
        PY=.venv-models/bin/python
    fi
    $PY -I tools/onnx_fp16_to_fp32.py models/supercombo.onnx models/supercombo-fp32.onnx
fi

# ncnn (Vulkan) model for `modeld --backend ncnn`: onnxsim + pnnx + graph fixes, see
# tools/ncnn/make_ncnn.sh. pnnx links against torch (CPU wheel, ~200 MB download) and needs
# an x86_64/aarch64 glibc box, so this also runs on the dev box. Skip with NCNN=0.
if [ "${NCNN:-1}" != 0 ] && [ ! -f models/supercombo-ncnn.param ]; then
    [ -x .venv-models/bin/python ] || python3 -m venv .venv-models
    if ! .venv-models/bin/python -c 'import onnx, onnxsim, pnnx, torch' 2>/dev/null; then
        .venv-models/bin/pip -q install torch==2.14.1 --index-url https://download.pytorch.org/whl/cpu
        .venv-models/bin/pip -q install onnx==1.23.2 onnxsim==0.8.1 pnnx==20261020 numpy
    fi
    tools/ncnn/make_ncnn.sh models/supercombo-fp32.onnx models/supercombo-ncnn .venv-models
fi
sha256sum models/*.onnx models/*.param models/*.bin 2>/dev/null || true
