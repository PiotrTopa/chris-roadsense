#!/bin/sh
# Build and install pocopilot on the phone. Run ON THE PHONE as root from a repo checkout
# (e.g. ~/pocopilot after `scripts/sync-phone.sh`). Models are expected in ./models
# (scripts/fetch-models.sh + scripts/push-models.sh on the dev box).
set -eu
cd "$(dirname "$0")/.."
[ "$(id -u)" = 0 ] || { echo "run as root (sudo)"; exit 1; }
BUILD=${BUILD:-build}
cmake -S . -B "$BUILD" -DCMAKE_BUILD_TYPE=Release >/dev/null
cmake --build "$BUILD" -j"$(nproc)"
install -m 755 "$BUILD/camerad/camerad" "$BUILD/modeld/modeld" "$BUILD/signd/signd" /usr/local/bin/
install -m 755 "$BUILD/tools/ringdump" /usr/local/bin/ 2>/dev/null || true
for t in modeld_replay signd_replay; do [ -x "$BUILD/tools/$t" ] && install -m 755 "$BUILD/tools/$t" /usr/local/bin/ || true; done
mkdir -p /usr/local/lib/pocopilot /var/lib/pocopilot/models /etc/pocopilot
rm -rf /usr/local/lib/pocopilot/api && cp -r api /usr/local/lib/pocopilot/api
for m in models/*.onnx models/*.txt; do [ -f "$m" ] && install -m 644 "$m" /var/lib/pocopilot/models/; done
for c in camera.json calib.json modeld.json signd.json; do
    [ -f "/etc/pocopilot/$c" ] || { [ -f "config/$c" ] && install -m 644 "config/$c" /etc/pocopilot/; }
done
install -m 644 systemd/pocopilot-*.service systemd/pocopilot.target /etc/systemd/system/
systemctl daemon-reload
systemctl enable pocopilot.target >/dev/null
echo "installed. start:  systemctl start pocopilot.target   status: systemctl status 'pocopilot-*'"
