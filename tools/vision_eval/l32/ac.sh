#!/bin/sh
# usage: ac.sh RIDE START_S DUR_S TAG OUT_DIR DYAW_DEG DPITCH_DEG [extra modeld_replay args]
# vision + policy, ncnn fp32 on the laptop GPU, 13.8 runs/s; writes OUT_DIR/RIDE.TAG.13.8.jsonl.gz (seq from the ride start)
set -eu
RIDE=/home/piotr/datasets/rides/$1; S=$2; DUR=$3; TAG=$4; OUT=$5; DY=$6; DP=$7
shift 7
B=${B:-/home/piotr/datasets/l32/build-ncnn/modeld}
slug=$(basename "$RIDE"); mj=$(ls "$RIDE"/*.mjpeg | head -1); mkdir -p "$OUT"
set -- $(python3 - "$RIDE" "$DY" "$DP" <<'PY'
import json,sys,math
r=sys.argv[1]; d=json.load(open(r+'/camera-replay.json')); c=json.load(open(r+'/calib.json'))
print(d['focal_px'], d.get('cx',640), d.get('cy',360), c.get('pitch',0)+math.radians(float(sys.argv[3])), c.get('yaw',0)+math.radians(float(sys.argv[2])), c.get('roll',0), c.get('height_m',1.22))
PY
) "$@"
F=$1; CX=$2; CY=$3; P=$4; Y=$5; R=$6; H=$7; shift 7
s0=$((S*20)); n=$((DUR*20))
ffmpeg -nostdin -loglevel error -f mjpeg -framerate 20 -i "$mj" -vf "trim=start_frame=$s0:end_frame=$((s0 + n)),scale=in_range=full:out_range=full,format=nv12" -f rawvideo - \
 | "$B/modeld_replay" /dev/stdin --width 1280 --height 720 --fps 20 --focal "$F" --cx "$CX" --cy "$CY" \
     --pitch "$P" --yaw "$Y" --roll "$R" --height-m "$H" --backend ncnn --ncnn-fp32 --models-dir /mnt/win/linux-scratch/l27/models \
     --road-model vision --rates 13.8 --out-prefix "$OUT/$slug.$TAG." "$@" 2> "$OUT/$slug.$TAG.log"
f="$OUT/$slug.$TAG.13.8.jsonl"
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
