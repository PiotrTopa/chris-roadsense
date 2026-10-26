#!/bin/sh
# Build the envd model files into ./models (not committed except the small class file):
#   models/envd-image.onnx      the image tower of the chosen open_clip model (ONNX, fp32)
#   models/envd-classes.json    class text embeddings + preprocessing constants (tools/envd/text_embeddings.py)
# Run on a dev box (glibc x86_64/aarch64; the venv pulls torch CPU ~200 MB + open_clip); copy
# models/ to the phone with scripts/push-models.sh. Weights come from the Hugging Face hub into
# ~/.cache/huggingface. Model choice and its licence: docs/lanes/L2-envd.md, docs/LICENSES.md.
#
#   scripts/fetch-envd-model.sh                        # the shipped model
#   ENVD_MODEL=ViT-B-32 ENVD_PRETRAINED=openai scripts/fetch-envd-model.sh   # another open_clip model
#   NCNN=1 scripts/fetch-envd-model.sh                 # also convert to ncnn (envd --backend ncnn)
set -eu
cd "$(dirname "$0")/.."
ENVD_MODEL=${ENVD_MODEL:-ViT-B-32}
ENVD_PRETRAINED=${ENVD_PRETRAINED:-openai}
VENV=${VENV:-.venv-models}
mkdir -p models
if ! "$VENV/bin/python" -c 'import open_clip, onnxruntime, torch' 2>/dev/null; then
    [ -x "$VENV/bin/python" ] || python3 -m venv "$VENV"
    "$VENV/bin/pip" -q install torch==2.14.1 --index-url https://download.pytorch.org/whl/cpu
    "$VENV/bin/pip" -q install open_clip_torch==3.3.0 torchvision onnx==1.23.2 onnxruntime numpy pillow \
        --extra-index-url https://download.pytorch.org/whl/cpu
fi
[ -f models/envd-image.onnx ] || "$VENV/bin/python" -I tools/envd/export_onnx.py --model "$ENVD_MODEL" \
    --pretrained "$ENVD_PRETRAINED" --out models/envd-image.onnx
"$VENV/bin/python" -I tools/envd/text_embeddings.py --model "$ENVD_MODEL" --pretrained "$ENVD_PRETRAINED" \
    --out models/envd-classes.json
if [ "${NCNN:-0}" = 1 ] && [ ! -f models/envd-image-ncnn.param ]; then
    tools/envd/make_ncnn.sh models/envd-image.onnx models/envd-image-ncnn "$VENV"
fi
sha256sum models/envd-image.onnx models/envd-classes.json models/envd-image-ncnn.* 2>/dev/null || true
