#!/bin/sh
# Reference deployment: copy the downloaded/converted models to the phone (default target user@192.168.0.86).
set -eu
cd "$(dirname "$0")/.."
T=${1:-user@192.168.0.86}
ssh "$T" 'mkdir -p ~/roadsense/models'
scp models/supercombo-fp32.onnx "$T":~/roadsense/models/
for f in models/supercombo-ncnn.param models/supercombo-ncnn.bin; do [ -f "$f" ] && scp "$f" "$T":~/roadsense/models/; done
echo "models on $T:~/roadsense/models (systemd units expect /var/lib/roadsense/models; scripts/install-phone.sh moves them)"
