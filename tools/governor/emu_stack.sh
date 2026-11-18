#!/bin/sh
# emu_stack.sh RUNDIR CFGDIR MJPEG PORT   (dev box; started by emu.py inside one systemd unit)
# The whole stack from a build tree, CPU backends only: camerad-replay, modeld (ORT), signd (ORT, signs and
# lights), envd (ORT), api. Paths from the environment: RS_BUILD, SIGNS_BUILD, RS_SRC, MODELS (supercombo-fp32.onnx,
# envd-image.onnx, envd-classes.json), SIGN_MODELS (mtsd_*), LIGHT_DET, LIGHT_STATE, PYTHON, MODELD_HZ (the cap
# on road runs: the rate the emulated platform reaches at full clock, as the phone's road rate is bounded by its GPU).
RUN=$1; CFG=$2; MJPEG=$3; PORT=$4
export ROADSENSE_RUNDIR=$RUN ROADSENSE_RING=$RUN/frames ROADSENSE_CONFIG_DIR=$CFG ROADSENSE_DATADIR=$RUN/data
mkdir -p $RUN/data
trap 'kill $(jobs -p) 2>/dev/null' TERM INT EXIT
$RS_BUILD/camerad-replay/camerad-replay --file $MJPEG > $RUN/camerad.log 2>&1 &
sleep 1
$RS_BUILD/modeld/modeld --backend ort --model $MODELS/supercombo-fp32.onnx --threads 2 --cpus none --max-hz ${MODELD_HZ:-14} --stats 10 > $RUN/modeld.log 2>&1 &
$SIGNS_BUILD/signd --det-backend ort --cls-backend ort --models-dir $SIGN_MODELS --threads 2 --cpus none --max-hz 5 --stats 10 \
  --lights --lights-backend ort --lights-model $LIGHT_DET --lights-threads 2 --lights-hz 5 \
  --lights-roi2 0.125,0.125,0.875,0.569 --lights-thr-keep 0.12 --lights-coast 3 --lights-state $LIGHT_STATE > $RUN/signd.log 2>&1 &
$RS_BUILD/envd/envd --backend ort --model $MODELS/envd-image.onnx --classes $MODELS/envd-classes.json --threads 2 --cpus none --max-hz 1 --stats 30 > $RUN/envd.log 2>&1 &
PYTHONPATH=$RS_SRC/api $PYTHON -m roadsense_api --host 127.0.0.1 --port $PORT > $RUN/api.log 2>&1 &
wait
