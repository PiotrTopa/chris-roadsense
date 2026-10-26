#!/bin/sh
# Make a ride file for camerad-replay from any video ffmpeg can read (laptop side; the phone has no ffmpeg).
#   tools/make_ride.sh INPUT OUT.mjpeg [START [DURATION]]     e.g. ... ride.webm ride.mjpeg 00:12:00 600
# Output: concatenated baseline JPEGs (ffmpeg -f mjpeg), 1280x720, YCbCr 4:2:0 full range (JFIF = the ring's
# BT.601 full range), FPS (default 20) frames per second, quality Q (ffmpeg -q:v, default 4: ~100-150 KB/frame).
# Heavy: run inside a memory-capped scope, e.g.
#   systemd-run --user --scope -p MemoryMax=8G nice tools/make_ride.sh ...
# Env: FPS (20), SIZE (1280:720), Q (4), THREADS (8). See docs/lanes/L5-ride-replay.md.
set -eu
[ $# -ge 2 ] || { sed -n 2,8p "$0"; exit 2; }
IN=$1; OUT=$2; START=${3:-0}; DUR=${4:-}
FPS=${FPS:-20}; SIZE=${SIZE:-1280:720}; Q=${Q:-4}; THREADS=${THREADS:-8}
set -- -hide_banner -loglevel error -nostats -threads "$THREADS" -ss "$START"
[ -n "$DUR" ] && set -- "$@" -t "$DUR"
ffmpeg "$@" -i "$IN" -an -sn -dn \
    -vf "fps=$FPS,scale=$SIZE:flags=area:out_range=full,format=yuvj420p" \
    -c:v mjpeg -q:v "$Q" -huffman optimal -f mjpeg -y "$OUT"
ls -l "$OUT"
