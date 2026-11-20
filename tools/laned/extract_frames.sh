#!/bin/sh
# extract_frames.sh RIDE START_S DUR_S : every 4th frame (5 fps) of the stretch as full-range NV12 1280x720
# -> ${L29_FRAMES:-${SCRATCH:-$HOME/scratch}/l29/frames}/RIDE.nv12 ; frame k is ride frame n = START*20 + 4k (ts = n * 0.05 s)
set -eu
R=$1; S=$2; D=$3
mj=$(ls ${RIDES_DIR:-$HOME/datasets/rides}/$R/*.mjpeg | head -1)
s0=$((S*20)); n=$((D*20))
ffmpeg -loglevel error -y -f mjpeg -framerate 20 -i "$mj" -vf "trim=start_frame=$s0:end_frame=$((s0+n)),select='not(mod(n-$s0\,4))',scale=in_range=full:out_range=full,format=nv12" -vsync 0 -f rawvideo ${L29_FRAMES:-${SCRATCH:-$HOME/scratch}/l29/frames}/$R.nv12
echo "$R $s0 4 $(($(stat -c %s ${L29_FRAMES:-${SCRATCH:-$HOME/scratch}/l29/frames}/$R.nv12)/1382400))" > ${L29_FRAMES:-${SCRATCH:-$HOME/scratch}/l29/frames}/$R.txt
