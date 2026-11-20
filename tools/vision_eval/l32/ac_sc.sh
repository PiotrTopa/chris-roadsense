#!/bin/sh
# supercombo (ncnn fp32, 13.8 runs/s) on the rides with stops: the ride's calibration and the car-like wrong one
set -eu
B=$HOME/datasets/l32/build-ncnn/modeld; O=$HOME/datasets/l32/ac
run() { # ride start dur tag dyaw dpitch
RIDE=$HOME/datasets/rides/$1; mj=$(ls "$RIDE"/*.mjpeg | head -1)
set -- "$@" $(python3 - "$RIDE" "$5" "$6" <<'PY'
import json,sys,math
r=sys.argv[1]; d=json.load(open(r+'/camera-replay.json')); c=json.load(open(r+'/calib.json'))
print(d['focal_px'], d.get('cx',640), d.get('cy',360), c.get('pitch',0)+math.radians(float(sys.argv[3])), c.get('yaw',0)+math.radians(float(sys.argv[2])), c.get('roll',0), c.get('height_m',1.22))
PY
)
s0=$(($2*20)); n=$(($3*20)); slug=$1; tag=$4
ffmpeg -nostdin -loglevel error -f mjpeg -framerate 20 -i "$mj" -vf "trim=start_frame=$s0:end_frame=$((s0 + n)),scale=in_range=full:out_range=full,format=nv12" -f rawvideo - \
 | "$B/modeld_replay" /dev/stdin --width 1280 --height 720 --fps 20 --focal "$7" --cx "$8" --cy "$9" --pitch "${10}" --yaw "${11}" --roll "${12}" --height-m "${13}" \
   --backend ncnn --ncnn-fp32 --road-model supercombo --model $HOME/projects/roadsense/models/supercombo-ncnn.param --rates 13.8 --out-prefix "$O/$slug.$tag." 2> "$O/$slug.$tag.log"
f="$O/$slug.$tag.13.8.jsonl"
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
}
run warsaw-city 0 900 scoff 0 0
run warsaw-city 0 900 scpoff 3.4 -1.4
run krakow-2024 60 240 scoff 0 0
run krakow-2024 60 240 scpoff 3.4 -1.4
echo sc done
