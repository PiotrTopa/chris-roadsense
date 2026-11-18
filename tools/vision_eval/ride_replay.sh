#!/bin/sh
# Road streams of one ride stretch from both road models, at the run rates of the phone (lane L24-M).
#   usage: tools/vision_eval/ride_replay.sh RIDE_DIR START_S DUR_S OUT_DIR [BUILD_DIR]
#   env:   MODELS (default ~/datasets/openpilot-models), SC_MODEL (supercombo-fp32.onnx), THREADS (2),
#          SC_RATES (13.8), VIS_RATES (13.8,17,20)
# Writes OUT_DIR/<slug>.sc.<rate>.jsonl.gz (supercombo v0.9.7, its own recurrent state per rate) and
# OUT_DIR/<slug>.vis.<rate>.jsonl.gz (v0.11.1 driving_vision + driving_policy; one vision run per frame shared by
# the rates, policy history per rate). seq = frame number from the ride start (START_S * 20 + 1, ...).
# Frames: the ride's MJPEG decoded by ffmpeg to full-range NV12 (as camerad-replay), modeld_replay --rates.
# Run it under systemd-run --user -p MemoryMax=... -p CPUQuota=... (heavy).
set -eu
[ $# -ge 4 ] || { sed -n '2,10p' "$0"; exit 2; }
RIDE=$(realpath "$1"); S=$2; DUR=$3; OUT=$(realpath -m "$4")
HERE=$(cd "$(dirname "$0")" && pwd)
B=${5:-$HERE/../../build}
MODELS=${MODELS:-$HOME/datasets/openpilot-models}
SC_MODEL=${SC_MODEL:-$HOME/projects/roadsense/models/supercombo-fp32.onnx}
THREADS=${THREADS:-2}
SC_RATES=${SC_RATES:-13.8}
VIS_RATES=${VIS_RATES:-13.8,17,20}
slug=$(basename "$RIDE")
mj=$(ls "$RIDE"/*.mjpeg | head -1)
F=$(python3 -c "import json,sys;d=json.load(open(sys.argv[1]));print(d['focal_px'],d.get('cx',640),d.get('cy',360))" "$RIDE/camera-replay.json")
set -- $F
mkdir -p "$OUT"
s0=$(python3 -c "print(int(round($S*20)))"); n=$(python3 -c "print(int(round($DUR*20)))")
dec() { ffmpeg -loglevel error -f mjpeg -framerate 20 -i "$mj" -vf "trim=start_frame=$s0:end_frame=$((s0 + n)),scale=in_range=full:out_range=full,format=nv12" -f rawvideo -; }
run() {  # tag model-args...
    tag=$1; shift
    dec | "$B/modeld_replay" /dev/stdin --width 1280 --height 720 --fps 20 --focal "$F1" --cx "$F2" --cy "$F3" \
        --calib "$RIDE/calib.json" --threads "$THREADS" --out-prefix "$OUT/$slug.$tag." "$@" 2> "$OUT/$slug.$tag.log"
}
F1=$1; F2=$2; F3=$3
run sc --road-model supercombo --model "$SC_MODEL" --rates "$SC_RATES" &
run vis --road-model vision --model "$MODELS/v0.11.1/driving_vision-fp32.onnx" \
    --policy "$MODELS/v0.11.1/driving_policy-fp32.onnx" --rates "$VIS_RATES" &
wait
# seq offset: modeld_replay numbers frames from 1 at the stretch start
for f in "$OUT/$slug".sc.*.jsonl "$OUT/$slug".vis.*.jsonl; do
    python3 -c "
import json,sys
off=$s0
with open(sys.argv[1]) as fi, open(sys.argv[1]+'.tmp','w') as fo:
    for ln in fi:
        m=json.loads(ln)
        if 'seq' in m: m['seq']+=off
        if 'frame_seq_prev' in m: m['frame_seq_prev']+=off
        if isinstance(m.get('ts_mono_ns'),int) and m.get('type')=='road': m['ts_mono_ns']+=off*50000000; m['ts_real_ns']=m['ts_mono_ns']
        fo.write(json.dumps(m,separators=(',',':'))+'\n')
" "$f" && mv "$f.tmp" "$f" && gzip -f "$f"
done
tail -n 1 "$OUT/$slug.sc.log" "$OUT/$slug.vis.log"
