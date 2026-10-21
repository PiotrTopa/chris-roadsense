#!/bin/sh
# Build ncnn (tag 20261020, BSD-3-Clause) with Vulkan for `modeld --backend ncnn`.
#   usage: scripts/build-ncnn.sh DIR        (e.g. ~/ncnn; then build roadsense with
#          cmake -DROADSENSE_NCNN=ON -DNCNN_DIR=DIR)
# Clones into DIR (with the bundled glslang submodule) and builds static libncnn.a + glslang in
# DIR/build (unmodified ncnn). Needs git, cmake, a C++ compiler, the Vulkan headers/loader
# (pmOS: vulkan-headers, vulkan-loader-dev) and OpenMP (libgomp).
#
# WARNING: on the Poco F1 this takes ~45 min at -j6 and heats the SoC to 90 C+; the script pauses
# the compiler for 60 s whenever the hottest zone exceeds 90 C. Run it inside a held-open ssh
# session (or tmux/screen): `nohup ... &` does NOT survive, the session scope is killed at logout.
set -eu
[ $# -eq 1 ] || { sed -n '2,12p' "$0"; exit 2; }
D=$(realpath -m "$1")
TAG=20261020
JOBS=${JOBS:-6}
if [ ! -d "$D/.git" ]; then
    git clone --depth 1 --branch "$TAG" https://github.com/Tencent/ncnn.git "$D"
fi
cd "$D"
git submodule update --init --depth 1 glslang
mkdir -p build
cd build
cmake -DCMAKE_BUILD_TYPE=Release -DNCNN_VULKAN=ON -DNCNN_SHARED_LIB=OFF -DNCNN_BUILD_TOOLS=ON \
      -DNCNN_BUILD_EXAMPLES=OFF -DNCNN_BUILD_BENCHMARK=ON -DNCNN_BUILD_TESTS=OFF -DNCNN_PYTHON=OFF ..
done_flag=$(mktemp)
rm -f "$done_flag"
( while [ ! -e "$done_flag" ]; do
      t=$(cat /sys/class/thermal/thermal_zone*/temp 2>/dev/null | sort -n | tail -1)
      if [ "${t:-0}" -gt 90000 ]; then
          echo "build-ncnn: $((t / 1000)) C, pausing the compiler 60 s"
          pkill -STOP -f "cc1plus.*$D" || true; sleep 60; pkill -CONT -f "cc1plus.*$D" || true
      fi
      sleep 10
  done ) &
rc=0
make -j"$JOBS" || rc=$?
touch "$done_flag"
wait
rm -f "$done_flag"
[ $rc -eq 0 ] && echo "build-ncnn: done: $D/build/src/libncnn.a" || echo "build-ncnn: make failed ($rc)"
exit $rc
