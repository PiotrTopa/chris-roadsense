#!/bin/sh
# stack.sh up PROFILE | down | status     (root, on the phone)
# Runs a whole roadsense stack as transient systemd units (l21-*) from candidate binaries with its
# own run directory, frame ring, config and data directory, next to the installed stack, which must
# be stopped (`systemctl stop roadsense.target`). The api listens on the installed port so that the
# host's consumer (the car backend's scene feed and its health watchdog) talks to the candidate.
# A profile is a shell fragment overriding the variables below (defaults = the installed stack).
L=${L21_HOME:-/home/user/l21}
RUN=/run/l21; RING=/dev/shm/l21-frames; CFG=$L/etc; DATA=$L/data
UNITS="l21-camerad l21-modeld l21-signd l21-envd l21-mapd l21-api"
M=/var/lib/roadsense/models

RIDE=/home/user/rides/krakow-2024
CAMERAD_BIN=/usr/local/bin/camerad-replay
CAMERAD_ARGS=""
MODELD_BIN=/usr/local/bin/modeld
MODELD_ARGS="--backend ncnn --cpus 4-7 --prep-cpus 0-3 --max-hz 20 --throttle-temp 95000 --gpu-yield 80 --gpu-lead 2 --cpu-latency-us 0 --stats 10"
SIGND_BIN=/usr/local/bin/signd
SIGND_ARGS="--det-backend ncnn --cls-backend ort --threads 2 --cpus 4-7 --max-hz 5 --gpu-sync turn --stats 10 --lights --lights-backend ncnn-cpu --lights-threads 2 --lights-cpus 4-7 --lights-hz 3 --lights-roi2 0.125,0.125,0.875,0.569 --lights-thr-keep 0.12 --lights-coast 2 --lights-state $M/lights_state_bdd_48x24.onnx --verifier $M/sign_verifier_32.onnx --class-min-score regulatory--no-entry=0.71 --cls $M/mtsd_signcls_l12_64.onnx --cls-names $M/mtsd_signcls_l12_classes.txt --dist-match-new 2.0"
ENVD_BIN=/usr/local/bin/envd
ENVD_ARGS="--backend ort --threads 2 --cpus 4-7 --max-hz 1 --stats 60"
API_PY=/usr/local/lib/roadsense/api
API_ARGS=""
API_PROPS=""
MAPD_PY=/usr/local/lib/roadsense/mapd
MAPD_ARGS="--hz 2 --stats 60"
MAPD_PROPS="-p CPUAffinity=0-3 -p Nice=10"
PORT=8090
CLOCKS=pin            # pin = the installed modeld unit's clock pins (all cores and the GPU at max)
EXTRA_ENV=""

pin_clocks() {
  for c in 4 5 6 7; do f=/sys/devices/system/cpu/cpu$c/cpufreq; cat $f/cpuinfo_max_freq > $f/scaling_max_freq; cat $f/cpuinfo_max_freq > $f/scaling_min_freq; done
  for c in 0 1 2 3; do cat /sys/devices/system/cpu/cpu$c/cpufreq/cpuinfo_max_freq > /sys/devices/system/cpu/cpu$c/cpufreq/scaling_min_freq; done
  cat /sys/class/devfreq/5000000.gpu/max_freq > /sys/class/devfreq/5000000.gpu/min_freq
}
unpin_clocks() {
  for c in 0 1 2 3 4 5 6 7; do cat /sys/devices/system/cpu/cpu$c/cpufreq/cpuinfo_min_freq > /sys/devices/system/cpu/cpu$c/cpufreq/scaling_min_freq 2>/dev/null; done
  cut -d" " -f1 /sys/class/devfreq/5000000.gpu/available_frequencies > /sys/class/devfreq/5000000.gpu/min_freq
}
down() {
  systemctl stop $UNITS 2>/dev/null
  systemctl reset-failed $UNITS 2>/dev/null
  # transient units are only gone once collected; a new one of the same name cannot start before
  for i in $(seq 1 40); do
    systemctl list-units --all --no-legend $(for u in $UNITS; do echo $u.service; done) 2>/dev/null | grep -q . || break
    sleep 0.5
  done
  unpin_clocks
  rm -rf $RUN $RING
}
case "$1" in
up)
  [ -n "$2" ] && . "$2"
  # The host's watchdog restarts the installed target when its api has been silent for 3 minutes
  # (between two candidate runs): stop it again. Stopping is not a change of the installation.
  systemctl stop roadsense.target roadsense-signd.service roadsense-api.service 2>/dev/null
  down
  mkdir -p $RUN $CFG $DATA $L/rec
  for f in envd.json mapd.json camera-dvr.json; do [ -e $CFG/$f ] || cp /etc/roadsense/$f $CFG/; done
  [ -e $CFG/recorder.json ] || printf '{"dir": "%s/rec", "max_mb": 1500, "frame_every_s": 1.0}\n' $L > $CFG/recorder.json
  cp $RIDE/camera-replay.json $CFG/camera-replay.json; cp $RIDE/calib.json $CFG/calib.json
  [ "$CLOCKS" = pin ] && pin_clocks
  E="--setenv=ROADSENSE_RUNDIR=$RUN --setenv=ROADSENSE_RING=$RING --setenv=ROADSENSE_CONFIG_DIR=$CFG --setenv=ROADSENSE_DATADIR=$DATA $EXTRA_ENV"
  S="--collect -q -p Slice=roadsense.slice"
  systemd-run $S --unit l21-camerad -p Nice=-5 $E $CAMERAD_BIN --file $(ls $RIDE/*.mjpeg | head -1) $CAMERAD_ARGS
  sleep 1
  systemd-run $S --unit l21-modeld -p Nice=-5 $E $MODELD_BIN $MODELD_ARGS
  [ -n "$SIGND_BIN" ] && systemd-run $S --unit l21-signd -p Nice=-5 $E $SIGND_BIN $SIGND_ARGS
  [ -n "$ENVD_BIN" ] && systemd-run $S --unit l21-envd -p Nice=5 $E $ENVD_BIN $ENVD_ARGS
  [ -n "$MAPD_PY" ] && systemd-run $S --unit l21-mapd $MAPD_PROPS -p MemoryMax=384M $E --setenv=PYTHONPATH=$MAPD_PY /usr/bin/python3 -m roadsense_mapd --calibration-file $DATA/vehicle-calibration.json $MAPD_ARGS
  systemd-run $S --unit l21-api $API_PROPS $E --setenv=PYTHONPATH=$API_PY /usr/bin/python3 -m roadsense_api --host 0.0.0.0 --port $PORT $API_ARGS
  ;;
down) down ;;
status) systemctl is-active $UNITS ;;
*) echo "usage: stack.sh up PROFILE | down | status" >&2; exit 2 ;;
esac
