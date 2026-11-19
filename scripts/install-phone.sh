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
for t in modeld/modeld_replay tools/ringdump camerad-dvr/camerad-dvr envd/envd envd/envd_replay laned/laned laned/laned_replay; do [ -x "$BUILD/$t" ] && install -m 755 "$BUILD/$t" /usr/local/bin/ || true; done
mkdir -p /usr/local/lib/roadsense /var/lib/roadsense/models /etc/roadsense
rm -rf /usr/local/lib/roadsense/api && cp -r api /usr/local/lib/roadsense/api
# mapd, the legacy position daemon (Python, needs py3-numpy; it uses the map box's modules), and its tools.
# Map packs live in /var/lib/roadsense/maps and are installed separately (scripts/roadsense-maps install,
# docs/MAPS.md), never from this tree. In the chris stack the unit stays off: the map box runs inside
# chris-integrator (chris-integrator/scripts/install.sh installs both).
rm -rf /usr/local/lib/roadsense/mapd && mkdir -p /usr/local/lib/roadsense/mapd /var/lib/roadsense/maps
cp -r mapd/roadsense_mapd chris-mapper/chris_mapper /usr/local/lib/roadsense/mapd/
install -m 755 tools/position_client.py /usr/local/bin/roadsense-position
install -m 755 tools/vehicle_client.py /usr/local/bin/roadsense-vehicle
for m in models/*.onnx models/*.param models/*.bin models/*.txt models/*.json; do [ -f "$m" ] && install -m 644 "$m" /var/lib/roadsense/models/; done
for c in camera.json calib.json modeld.json envd.json mapd.json laned.json; do
    [ -f "/etc/roadsense/$c" ] || { [ -f "config/$c" ] && install -m 644 "config/$c" /etc/roadsense/; }
done
# Debug face port/bind: API_PORT=N / API_HOST=ADDR write /etc/roadsense/api.env (kept afterwards; an
# unset one keeps its current value). Since api 3 this is one camera's debug face (default 8081 on
# loopback); the consumer API is chris-integrator's (its own install script and port).
# The nftables rule follows api.env: opened for the port unless the face is loopback-only, in which
# case a previously installed rule is removed.
OLD_PORT=$(sed -n 's/^ROADSENSE_API_PORT=//p' /etc/roadsense/api.env 2>/dev/null | tail -1)
OLD_HOST=$(sed -n 's/^ROADSENSE_API_HOST=//p' /etc/roadsense/api.env 2>/dev/null | tail -1)
if [ -n "${API_PORT:-}" ] || [ -n "${API_HOST:-}" ]; then
    { printf 'ROADSENSE_API_PORT=%s\n' "${API_PORT:-${OLD_PORT:-8081}}"
      [ -n "${API_HOST:-$OLD_HOST}" ] && printf 'ROADSENSE_API_HOST=%s\n' "${API_HOST:-$OLD_HOST}"
    } > /etc/roadsense/api.env
fi
PORT=$(sed -n 's/^ROADSENSE_API_PORT=//p' /etc/roadsense/api.env 2>/dev/null | tail -1); PORT=${PORT:-8081}
HOST=$(sed -n 's/^ROADSENSE_API_HOST=//p' /etc/roadsense/api.env 2>/dev/null | tail -1); HOST=${HOST:-127.0.0.1}
if [ -d /etc/nftables.d ]; then
    case "$HOST" in
        127.*|localhost|::1) rm -f /etc/nftables.d/50_roadsense.nft ;;
        *) sed "s/8080/$PORT/" phone/50_roadsense.nft > /etc/nftables.d/50_roadsense.nft ;;
    esac
    systemctl try-restart nftables
fi
# A masked unit (symlink to /dev/null, e.g. roadsense-camerad.service after
# scripts/install-camerad-dvr.sh --enable) stays masked: install(1) would replace the symlink.
for u in systemd/roadsense-*.service systemd/roadsense.target systemd/roadsense.slice; do
    d=/etc/systemd/system/$(basename "$u")
    if [ -L "$d" ] && [ "$(readlink "$d")" = /dev/null ]; then echo "kept masked: $(basename "$u")"; continue; fi
    install -m 644 "$u" /etc/systemd/system/
done
systemctl daemon-reload
# boot autostart is opt-in (AUTOSTART=1): the reference phone overheats without cooling
[ "${AUTOSTART:-0}" = 1 ] && systemctl enable roadsense.target >/dev/null || true
# laned (lane lines for the display) is optional and stays off: systemctl enable roadsense-laned.service
echo "installed. start:  systemctl start roadsense.target   status: systemctl status 'roadsense-*'"
