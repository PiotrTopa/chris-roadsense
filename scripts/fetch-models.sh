#!/bin/sh
# Download the model files roadsense needs into ./models (not committed), or into $MODELS_DIR.
#   road model, default since lane L27: openpilot v0.11.1 driving_vision + driving_policy (MIT, comma.ai)
#   road model, selectable (`modeld --road-model supercombo`) and modeld's fallback: openpilot v0.9.7 supercombo
# Every download is pinned by URL (a release tag) and sha256; the converted files are checked against the sha256
# this toolchain produced (a mismatch is reported, not fatal: another onnx / pnnx version writes other bytes).
# env: NCNN=0 skips the ncnn conversions; ROAD_MODELS="vision supercombo" (default both) selects.
set -eu
cd "$(dirname "$0")/.."
M=${MODELS_DIR:-models}
mkdir -p "$M"
ROAD_MODELS=${ROAD_MODELS:-vision supercombo}
want() { case " $ROAD_MODELS " in *" $1 "*) return 0 ;; esac; return 1; }
check() { # file sha256 : report a converted file whose bytes differ from the pinned conversion
    [ -f "$1" ] || return 0
    got=$(sha256sum "$1" | cut -d' ' -f1)
    [ "$got" = "$2" ] && echo "ok   $1" || echo "NOTE $1: sha256 $got differs from the pinned conversion $2 (toolchain version?)"
}
venv() { # the conversion tools (python onnx has no musl wheel: run on the dev box, then scripts/push-models.sh)
    [ -x .venv-models/bin/python ] || python3 -m venv .venv-models
    if ! .venv-models/bin/python -c 'import onnx, onnxsim, pnnx, torch' 2>/dev/null; then
        .venv-models/bin/pip -q install torch==2.14.1 --index-url https://download.pytorch.org/whl/cpu
        .venv-models/bin/pip -q install onnx==1.23.2 onnxsim==0.8.1 pnnx==20261020 numpy
    fi
}
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
# --- default road model: openpilot v0.11.1 driving_vision + driving_policy, MIT (comma.ai), tag v0.11.1 ---------
if want vision; then
    OP=https://github.com/commaai/openpilot/raw/v0.11.1/selfdrive/modeld/models
    fetch "$OP/driving_vision.onnx" ee29ee5bce84d1ce23e9ff381280de9b4e4d96d2934cd751740354884e112c66 "$M/driving_vision.onnx"
    fetch "$OP/driving_policy.onnx" 78477124cbf3ffe30fa951ebada8410b43c4242c6054584d656f1d329b067e15 "$M/driving_policy.onnx"
    # fp32 graphs (ONNX Runtime CPU: the policy always, the vision model with --backend ort)
    for n in driving_vision driving_policy; do
        if [ ! -f "$M/$n-fp32.onnx" ]; then
            venv
            .venv-models/bin/python -I tools/onnx_fp16_to_fp32.py "$M/$n.onnx" "$M/$n-fp32.onnx"
        fi
    done
    check "$M/driving_vision-fp32.onnx" 183f63637d300eb1242294c64b9c529b3e09c266a65414622cc227f783a2de57
    check "$M/driving_policy-fp32.onnx" be1235de2eab03297f4d93f38ace8b5aa2f725fe7adee6ad2fcdc6becec0042a
    # ncnn (Vulkan) vision model for `modeld --backend ncnn` (tools/ncnn/make_vision_ncnn.sh). The policy stays on
    # ONNX Runtime: it does not convert to ncnn (docs/lanes/L24-M-vision-model.md).
    if [ "${NCNN:-1}" != 0 ] && [ ! -f "$M/driving_vision-ncnn.param" ]; then
        venv
        tools/ncnn/make_vision_ncnn.sh "$M/driving_vision-fp32.onnx" "$M/driving_vision-ncnn" .venv-models
    fi
    check "$M/driving_vision-ncnn.param" 480f638a051bdae651432000b0d33105196d44da7dec70b6f64f5379db8b827f
    check "$M/driving_vision-ncnn.bin" 9e37fe46323a9cf41a4cea82b1ec15a85f89e42b9e24789e4cb81049fbce1fbc
fi

# --- selectable road model and modeld's fallback: openpilot v0.9.7 supercombo, MIT (comma.ai), tag v0.9.7 --------
if want supercombo; then
    fetch "https://github.com/commaai/openpilot/raw/v0.9.7/selfdrive/modeld/models/supercombo.onnx" \
          b31b504bc0b440d3bc72967507a00eb4f112285626fbfb3135011500325ee6d6 \
          "$M/supercombo.onnx"
    if [ ! -f "$M/supercombo-fp32.onnx" ]; then
        venv
        .venv-models/bin/python -I tools/onnx_fp16_to_fp32.py "$M/supercombo.onnx" "$M/supercombo-fp32.onnx"
    fi
    # ncnn: onnxsim + pnnx + graph fixes, see tools/ncnn/make_ncnn.sh
    if [ "${NCNN:-1}" != 0 ] && [ ! -f "$M/supercombo-ncnn.param" ]; then
        venv
        tools/ncnn/make_ncnn.sh "$M/supercombo-fp32.onnx" "$M/supercombo-ncnn" .venv-models
    fi
fi
sha256sum "$M"/*.onnx "$M"/*.param "$M"/*.bin 2>/dev/null || true
