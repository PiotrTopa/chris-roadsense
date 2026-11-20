#!/bin/sh
# Reference deployment: copy the downloaded/converted models to the phone (target: the argument, else $ROADSENSE_PHONE, else user@phone).
set -eu
cd "$(dirname "$0")/.."
T=${1:-${ROADSENSE_PHONE:-user@phone}}
ssh "$T" 'mkdir -p ~/roadsense/models'
# the default road model (v0.11.1 vision + policy) and supercombo (selectable, and modeld's fallback)
for f in models/driving_vision-fp32.onnx models/driving_policy-fp32.onnx models/driving_vision-ncnn.param models/driving_vision-ncnn.bin \
         models/supercombo-fp32.onnx models/supercombo-ncnn.param models/supercombo-ncnn.bin models/envd-image.onnx models/envd-classes.json models/envd-image-ncnn.param models/envd-image-ncnn.bin; do [ -f "$f" ] && scp "$f" "$T":~/roadsense/models/; done
echo "models on $T:~/roadsense/models (systemd units expect /var/lib/roadsense/models; scripts/install-phone.sh moves them)"
