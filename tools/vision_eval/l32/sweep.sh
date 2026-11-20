#!/bin/sh
# usage: sweep.sh RIDE START_S DUR_S MODEL(vision|supercombo) TAG DYAW_DEG DCX_PX [DPITCH_DEG] [extra modeld_replay args]
set -eu
RIDE=$HOME/datasets/rides/$1; S=$2; DUR=$3; MODEL=$4; TAG=$5; DYAW=$6; DCX=$7; DP=${8:-0}
shift 7; [ $# -gt 0 ] && shift
B=${B:-$HOME/datasets/l32/build/modeld}
OUT=${OUT:-$HOME/datasets/l32/sweep}
M=$HOME/datasets/openpilot-models/v0.11.1
slug=$(basename "$RIDE")
mj=$(ls "$RIDE"/*.mjpeg | head -1)
set -- $(python3 - "$RIDE" "$DYAW" "$DCX" "$DP" <<'PY'
import json,sys,math
r=sys.argv[1]; d=json.load(open(r+'/camera-replay.json')); c=json.load(open(r+'/calib.json'))
print(d['focal_px'], d.get('cx',640)+float(sys.argv[3]), d.get('cy',360), c.get('pitch',0)+math.radians(float(sys.argv[4])), c.get('yaw',0)+math.radians(float(sys.argv[2])), c.get('roll',0), c.get('height_m',1.22))
PY
) "$@"
F=$1; CX=$2; CY=$3; P=$4; Y=$5; R=$6; H=$7; shift 7
s0=$((S*20)); n=$((DUR*20))
if [ "$MODEL" = vision ]; then MA="--road-model vision --model $M/driving_vision-fp32.onnx --policy $M/driving_policy-fp32.onnx --policy-hz 0"
else MA="--road-model supercombo --model $HOME/projects/roadsense/models/supercombo-fp32.onnx"; fi
o="$OUT/$slug.$MODEL.$TAG.jsonl"
ffmpeg -nostdin -loglevel error -f mjpeg -framerate 20 -i "$mj" -vf "trim=start_frame=$s0:end_frame=$((s0 + n)),scale=in_range=full:out_range=full,format=nv12" -f rawvideo - \
 | "$B/modeld_replay" /dev/stdin --width 1280 --height 720 --fps 20 --focal "$F" --cx "$CX" --cy "$CY" \
     --pitch "$P" --yaw "$Y" --roll "$R" --height-m "$H" --threads 2 $MA "$@" > "$o" 2> "$o.log"
gzip -f "$o"
