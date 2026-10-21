#!/bin/sh
# Copy the downloaded/converted models to the phone (default target user@192.168.0.86).
set -eu
cd "$(dirname "$0")/.."
T=${1:-user@192.168.0.86}
ssh "$T" 'mkdir -p ~/pocopilot/models'
scp models/supercombo-fp32.onnx "$T":~/pocopilot/models/
[ -f models/signs.onnx ] && scp models/signs.onnx models/signs-classes.txt "$T":~/pocopilot/models/ || true
echo "models on $T:~/pocopilot/models (systemd units expect /var/lib/pocopilot/models; scripts/install-phone.sh moves them)"
