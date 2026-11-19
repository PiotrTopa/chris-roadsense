#!/bin/sh
# Lane L28, job B (root): the new road model's lead through both arithmetic paths on the phone's GPU, same decoded
# frames: ncnn Vulkan fp16 packed (as deployed) and ncnn Vulkan fp32 (--ncnn-fp32), plus ORT fp32 on the CPU on the
# first 400 frames of the first stretch (checks that ncnn fp32 is a valid reference). No policy (the lead tracker
# reads leads, v_ego, pose). Streams at the deployed run rate (15/s) and at 13.8/s (L27's tables).
L=/home/user/l28; O=$L/fp16; M=$L/models; R=$L/build-rs/modeld/modeld_replay; mkdir -p $O
python3 $L/host.py roadsense_mode '{"mode":"on"}'; python3 $L/host.py set_fan '{"pct":100}'
echo "B start $(date '+%F %T') hottest $(sh $L/hot.sh)" >> $O/log
for spec in "warsaw-city 0 6000" "s7-elblag-lowsun 1200 4800" "a1-night 0 6000"; do
  set -- $spec; ride=$1; s0=$2; n=$3; D=/home/user/rides/$ride; MJ=$(ls $D/*.mjpeg | head -1)
  set -- $(python3 -c "import json,sys;d=json.load(open(sys.argv[1]));print(d['focal_px'],d.get('cx',640),d.get('cy',360))" $D/camera-replay.json)
  A="--width 1280 --height 720 --fps 20 --focal $1 --cx $2 --cy $3 --calib $D/calib.json --road-model vision --policy none --rates 13.8,15"
  rm -f $O/f1; mkfifo $O/f1
  $R $O/f1 $A --backend ncnn --model $M/driving_vision-ncnn.param --out-prefix $O/$ride.f16. > /dev/null 2> $O/$ride.f16.log &
  python3 $L/mjpeg_nv12.py $MJ $s0 $n $O/f1 - 2> $O/$ride.dec.log | $R /dev/stdin $A --backend ncnn --ncnn-fp32 --model $M/driving_vision-ncnn.param --out-prefix $O/$ride.f32. > /dev/null 2> $O/$ride.f32.log
  wait
  echo "$ride done $(date +%T) hottest $(sh $L/hot.sh)" >> $O/log
  if [ $ride = warsaw-city ]; then
    python3 $L/mjpeg_nv12.py $MJ $s0 400 - 2>/dev/null | taskset -c 4-7 $R /dev/stdin $A --backend ort --threads 4 --model $M/driving_vision-fp32.onnx --out-prefix $O/$ride.ort. > /dev/null 2> $O/$ride.ort.log
    echo "$ride ort done $(date +%T)" >> $O/log
  fi
done
rm -f $O/f1
python3 $L/host.py fan_auto; python3 $L/host.py roadsense_mode '{"mode":"auto"}'
echo B-DONE $(date +%T) >> $O/log
