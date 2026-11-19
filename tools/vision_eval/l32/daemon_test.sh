#!/bin/sh
D=/mnt/win/linux-scratch/l32/daemon; B=/home/piotr/datasets/l32/build-ncnn
export ROADSENSE_RUNDIR=$D/run ROADSENSE_CONFIG_DIR=$D/cfg ROADSENSE_RING=/dev/shm/roadsense-l32-test
for pass in 1 2; do
  $B/camerad-replay/camerad-replay --file /home/piotr/datasets/rides/s7-elblag-lowsun/s7-elblag-lowsun.mjpeg --start 70 --no-preview > $D/camerad.$pass.log 2>&1 &
  CP=$!
  sleep 2
  $B/modeld/modeld --backend ncnn --ncnn-fp32 --models-dir /mnt/win/linux-scratch/l27/models --cpus none --calib-state $D/state.json --stats 10 > $D/modeld.$pass.log 2>&1 &
  MP=$!
  sleep 8
  timeout $([ $pass = 1 ] && echo 75 || echo 20) python3 -c "
import socket,sys
s=socket.socket(socket.AF_UNIX); s.connect('$D/run/modeld.sock'); f=s.makefile()
for ln in f:
    if ln.startswith('{\"type\":\"calib\"'): sys.stdout.write(ln); sys.stdout.flush()
    elif ln.startswith('{\"type\":\"road\"'): sys.stdout.write('R\n')
" > $D/client.$pass.txt
  kill $MP $CP; wait
done
rm -f /dev/shm/roadsense-l32-test
