#!/bin/sh
# fp16 accuracy gate of the v0.11.1 vision model on the phone: ncnn Vulkan (fp16-packed, as modeld runs it)
# against ONNX Runtime fp32 on the same frames, one command (lane L24-M).
#   usage: tools/ncnn/vision_gate.sh FRAMES.nv12 WIDTH HEIGHT [MODEL_DIR] [N_FRAMES] [OUT_DIR]
#   MODEL_DIR (default /var/lib/roadsense/models): driving_vision-fp32.onnx, driving_vision-ncnn.{param,bin},
#   driving_policy-fp32.onnx. Uses modeld_replay from PATH (or $MODELD_REPLAY). Extra modeld_replay arguments
#   (e.g. --focal/--calib) in $REPLAY_ARGS.
# Prints compare_replay.py's table for the vision outputs and ACCEPT/REJECT (docs/HARDWARE.md thresholds:
# mean lane error < 0.05 m, max < 0.2 m, lead x within 1 m, lane probs within 0.05), then the split (pipelined)
# ncnn run against the one-stage ncnn run (must be identical), and the mean model ms of each run.
set -eu
[ $# -ge 3 ] || { sed -n '2,10p' "$0"; exit 2; }
F=$1; W=$2; H=$3; D=${4:-/var/lib/roadsense/models}; N=${5:-400}; O=${6:-$(mktemp -d)}
HERE=$(cd "$(dirname "$0")" && pwd)
R=${MODELD_REPLAY:-modeld_replay}
mkdir -p "$O"
A="--width $W --height $H --road-model vision --policy $D/driving_policy-fp32.onnx --max-frames $N ${REPLAY_ARGS:-}"
$R "$F" $A --model "$D/driving_vision-fp32.onnx" --threads 4 > "$O/ort.jsonl" 2> "$O/ort.log"
$R "$F" $A --backend ncnn --model "$D/driving_vision-ncnn.param" > "$O/ncnn.jsonl" 2> "$O/ncnn.log"
$R "$F" $A --backend ncnn --model "$D/driving_vision-ncnn.param" --split > "$O/ncnn_split.jsonl" 2> "$O/ncnn_split.log"
grep -h "modeld: ncnn on" "$O/ncnn.log" | head -1
echo "== ncnn fp16 vs ORT fp32 =="
python3 "$HERE/compare_replay.py" "$O/ort.jsonl" "$O/ncnn.jsonl" --csv "$O/gate.csv" || true
echo "== ncnn split (pipelined) vs one-stage =="
python3 "$HERE/compare_replay.py" "$O/ncnn.jsonl" "$O/ncnn_split.jsonl" | grep -E "lane_y_max|plan_xy|ACCEPT|REJECT"
tail -n 1 "$O/ort.log" "$O/ncnn.log" "$O/ncnn_split.log"
echo "outputs in $O"
