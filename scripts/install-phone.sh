#!/bin/sh
# Reference deployment (Poco F1 / postmarketOS, systemd): build and install roadsense on the
# phone. Run ON THE PHONE as root from a repo checkout (e.g. ~/roadsense after
# `scripts/sync-phone.sh`). Models are expected in ./models (scripts/fetch-models.sh +
# scripts/push-models.sh on the dev box). A sign provider (roadsense-signs) installs separately.
set -eu
cd "$(dirname "$0")/.."
[ "$(id -u)" = 0 ] || { echo "run as root (sudo)"; exit 1; }
BUILD=${BUILD:-build}
# ncnn backend: CMAKE_ARGS="-DROADSENSE_NCNN=ON -DNCNN_DIR=/path/to/ncnn" (scripts/build-ncnn.sh)
cmake -S . -B "$BUILD" -DCMAKE_BUILD_TYPE=Release ${CMAKE_ARGS:-} >/dev/null
cmake --build "$BUILD" -j"$(nproc)"
install -m 755 "$BUILD/camerad/camerad" "$BUILD/modeld/modeld" /usr/local/bin/
for t in modeld/modeld_replay tools/ringdump camerad-dvr/camerad-dvr envd/envd envd/envd_replay; do [ -x "$BUILD/$t" ] && install -m 755 "$BUILD/$t" /usr/local/bin/ || true; done
mkdir -p /usr/local/lib/roadsense /var/lib/roadsense/models /etc/roadsense
rm -rf /usr/local/lib/roadsense/api && cp -r api /usr/local/lib/roadsense/api
for m in models/*.onnx models/*.param models/*.bin models/*.txt models/*.json; do [ -f "$m" ] && install -m 644 "$m" /var/lib/roadsense/models/; done
for c in camera.json calib.json modeld.json envd.json; do
    [ -f "/etc/roadsense/$c" ] || { [ -f "config/$c" ] && install -m 644 "config/$c" /etc/roadsense/; }
done
# API port/bind: API_PORT=N / API_HOST=ADDR write /etc/roadsense/api.env (kept afterwards; an
# unset one keeps its current value). The car computer uses 8090 on 127.0.0.1: its own backend
# serves 8080 and re-publishes this API, so nothing outside the phone talks to it directly.
# The nftables rule follows api.env: opened for the port (default 8080) unless the API is
# loopback-only, in which case a previously installed rule is removed.
OLD_PORT=$(sed -n 's/^ROADSENSE_API_PORT=//p' /etc/roadsense/api.env 2>/dev/null | tail -1)
OLD_HOST=$(sed -n 's/^ROADSENSE_API_HOST=//p' /etc/roadsense/api.env 2>/dev/null | tail -1)
if [ -n "${API_PORT:-}" ] || [ -n "${API_HOST:-}" ]; then
    { printf 'ROADSENSE_API_PORT=%s\n' "${API_PORT:-${OLD_PORT:-8080}}"
      [ -n "${API_HOST:-$OLD_HOST}" ] && printf 'ROADSENSE_API_HOST=%s\n' "${API_HOST:-$OLD_HOST}"
    } > /etc/roadsense/api.env
fi
PORT=$(sed -n 's/^ROADSENSE_API_PORT=//p' /etc/roadsense/api.env 2>/dev/null | tail -1); PORT=${PORT:-8080}
HOST=$(sed -n 's/^ROADSENSE_API_HOST=//p' /etc/roadsense/api.env 2>/dev/null | tail -1); HOST=${HOST:-0.0.0.0}
if [ -d /etc/nftables.d ]; then
    case "$HOST" in
        127.*|localhost|::1) rm -f /etc/nftables.d/50_roadsense.nft ;;
        *) sed "s/8080/$PORT/" phone/50_roadsense.nft > /etc/nftables.d/50_roadsense.nft ;;
    esac
    systemctl try-restart nftables
fi
install -m 644 systemd/roadsense-*.service systemd/roadsense.target systemd/roadsense.slice /etc/systemd/system/
systemctl daemon-reload
# boot autostart is opt-in (AUTOSTART=1): the reference phone overheats without cooling
[ "${AUTOSTART:-0}" = 1 ] && systemctl enable roadsense.target >/dev/null || true
echo "installed. start:  systemctl start roadsense.target   status: systemctl status 'roadsense-*'"
