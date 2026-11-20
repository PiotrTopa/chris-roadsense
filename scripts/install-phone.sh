#!/bin/sh
# Reference deployment (Poco F1 / postmarketOS, systemd): build and install roadsense on the
# phone, then chris-integrator (the consumer API and the map box) from chris-integrator/scripts/install.sh.
# Run ON THE PHONE as root from a repo checkout (e.g. ~/roadsense after `scripts/sync-phone.sh`).
# Models are expected in ./models (scripts/fetch-models.sh + scripts/push-models.sh on the dev box).
# A sign provider (roadsense-signs) installs separately.
#
#   sh scripts/install-phone.sh [--no-systemctl]
#   DESTDIR=/scratch/dir SKIP_BUILD=1 sh scripts/install-phone.sh    files only, into a scratch prefix
# Environment (optional): BUILD (build dir), CMAKE_ARGS, SKIP_BUILD=1 (install what BUILD already holds),
#   DEBUG_API_PORT / DEBUG_API_HOST (the debug face, default 8091 on loopback; under their earlier names,
#   API_PORT / API_HOST, they are still read here and mean the same), CHRIS_API_PORT / CHRIS_API_HOST (the
#   consolidated API of chris-integrator, default 8090 on loopback; passed on to its install script, which
#   never sees this script's API_PORT / API_HOST), AUTOSTART=1 (enable roadsense.target at boot),
#   WITH_CHRIS=0 (skip chris-integrator), CHRIS_CPUS (its CPUAffinity, default 0-2 on this phone),
#   PYTHON (passed to the chris install's checks).
set -eu
cd "$(dirname "$0")/.."
D=${DESTDIR:-}
D=${D%/}
SYSTEMCTL=1
for a in "$@"; do
    case "$a" in
        --no-systemctl) SYSTEMCTL=0 ;;
        *) echo "install-phone.sh: unknown argument $a" >&2; exit 2 ;;
    esac
done
[ -n "$D" ] && SYSTEMCTL=0
[ -n "$D" ] || [ "$(id -u)" = 0 ] || { echo "run as root (sudo), or DESTDIR=/some/dir for a scratch install"; exit 1; }
# The debug face's port / bind as given to this script (DEBUG_API_PORT / DEBUG_API_HOST; API_PORT / API_HOST are
# their earlier names). API_PORT / API_HOST leave the environment here: chris-integrator's install script,
# run from this one, would read them as the consolidated API's and refuse the collision; its own are
# CHRIS_API_PORT / CHRIS_API_HOST. The integrator's link to this face follows the port when one was given.
GIVEN_PORT=${DEBUG_API_PORT:-${API_PORT:-}}
GIVEN_HOST=${DEBUG_API_HOST:-${API_HOST:-}}
unset API_PORT API_HOST
if [ -n "$GIVEN_PORT" ]; then ROADSENSE_HTTP=${ROADSENSE_HTTP:-http://127.0.0.1:$GIVEN_PORT}; export ROADSENSE_HTTP; fi
# the chris part's prerequisites first, so that a missing module stops the install before anything changed
if [ "${WITH_CHRIS:-1}" = 1 ]; then
    if [ "$SYSTEMCTL" = 1 ]; then sh chris-integrator/scripts/install.sh --check
    else DESTDIR=$D sh chris-integrator/scripts/install.sh --check --no-systemctl; fi
fi
BUILD=${BUILD:-build}
if [ "${SKIP_BUILD:-0}" != 1 ]; then
    # ncnn backend: CMAKE_ARGS="-DROADSENSE_NCNN=ON -DNCNN_DIR=/path/to/ncnn" (scripts/build-ncnn.sh)
    # shellcheck disable=SC2086
    cmake -S . -B "$BUILD" -DCMAKE_BUILD_TYPE=Release ${CMAKE_ARGS:-} >/dev/null
    cmake --build "$BUILD" -j"$(nproc)"
fi
mkdir -p "$D/usr/local/bin" "$D/usr/local/lib/roadsense" "$D/var/lib/roadsense/models" "$D/etc/roadsense" "$D/etc/systemd/system"
for t in camerad/camerad modeld/modeld modeld/modeld_replay tools/ringdump camerad-dvr/camerad-dvr envd/envd envd/envd_replay laned/laned laned/laned_replay; do
    if [ -x "$BUILD/$t" ]; then install -m 755 "$BUILD/$t" "$D/usr/local/bin/"
    else case "$t" in camerad/camerad|modeld/modeld) [ "${SKIP_BUILD:-0}" = 1 ] || { echo "missing $BUILD/$t"; exit 1; } ;; esac; fi
done
rm -rf "$D/usr/local/lib/roadsense/api" && cp -r api "$D/usr/local/lib/roadsense/api"
# mapd, the legacy position daemon (Python, needs py3-numpy; it uses the map box's modules), and its tools.
# Map packs live in /var/lib/roadsense/maps and are installed separately (scripts/roadsense-maps install,
# docs/MAPS.md), never from this tree. In the chris stack the unit stays off and out of roadsense.target:
# the map box runs inside chris-integrator (chris-integrator/scripts/install.sh installs both and
# disables mapd).
rm -rf "$D/usr/local/lib/roadsense/mapd" && mkdir -p "$D/usr/local/lib/roadsense/mapd" "$D/var/lib/roadsense/maps"
cp -r mapd/roadsense_mapd chris-mapper/chris_mapper "$D/usr/local/lib/roadsense/mapd/"
find "$D/usr/local/lib/roadsense" -name __pycache__ -type d -prune -exec rm -rf {} \;
install -m 755 tools/position_client.py "$D/usr/local/bin/roadsense-position"
install -m 755 tools/vehicle_client.py "$D/usr/local/bin/roadsense-vehicle"
for m in models/*.onnx models/*.param models/*.bin models/*.txt models/*.json; do [ -f "$m" ] && install -m 644 "$m" "$D/var/lib/roadsense/models/"; done
for c in camera.json calib.json modeld.json envd.json mapd.json laned.json; do
    [ -f "$D/etc/roadsense/$c" ] || { [ -f "config/$c" ] && install -m 644 "config/$c" "$D/etc/roadsense/"; }
done
# Debug face port/bind: /etc/roadsense/api.env. Since api 3 this is one camera's debug face, on 8091 on
# loopback; the consumer API is chris-integrator's (port 8090, chris-integrator/scripts/install.sh).
# DEBUG_API_PORT=N / DEBUG_API_HOST=ADDR replace a value; otherwise the file's own values are kept, except the old
# consolidated port 8090 (now chris-integrator's), which becomes 8091. Other keys are kept; the old file
# is kept once as api.env.pre-chris. The nftables rule follows api.env: opened for the port unless the
# face is loopback-only, in which case a previously installed rule is removed.
AE=$D/etc/roadsense/api.env
OLD_PORT=$(sed -n 's/^ROADSENSE_API_PORT=//p' "$AE" 2>/dev/null | tail -n 1)
OLD_HOST=$(sed -n 's/^ROADSENSE_API_HOST=//p' "$AE" 2>/dev/null | tail -n 1)
[ "$OLD_PORT" = 8090 ] && OLD_PORT=
PORT=${GIVEN_PORT:-${OLD_PORT:-8091}}
HOST=${GIVEN_HOST:-${OLD_HOST:-127.0.0.1}}
if [ -f "$AE" ] && [ ! -e "$AE.pre-chris" ]; then cp -p "$AE" "$AE.pre-chris"; fi
[ -f "$AE" ] || : > "$AE"
for kv in "ROADSENSE_API_PORT=$PORT" "ROADSENSE_API_HOST=$HOST"; do
    k=${kv%%=*}
    if grep -q "^$k=" "$AE"; then
        [ "$(sed -n "s/^$k=//p" "$AE" | tail -n 1)" = "${kv#*=}" ] || { sed -i "s|^$k=.*|$kv|" "$AE"; echo "debug face: $kv"; }
    else
        printf '%s\n' "$kv" >> "$AE"; echo "debug face: $kv"
    fi
done
if [ "$SYSTEMCTL" = 1 ] && [ -d /etc/nftables.d ]; then
    case "$HOST" in
        127.*|localhost|::1) rm -f /etc/nftables.d/50_roadsense.nft ;;
        *) sed "s/8080/$PORT/" phone/50_roadsense.nft > /etc/nftables.d/50_roadsense.nft ;;
    esac
    systemctl try-restart nftables
fi
# A masked unit (symlink to /dev/null, e.g. roadsense-camerad.service after
# scripts/install-camerad-dvr.sh --enable) stays masked: install(1) would replace the symlink.
for u in systemd/roadsense-*.service systemd/roadsense.target systemd/roadsense.slice; do
    d=$D/etc/systemd/system/$(basename "$u")
    if [ -L "$d" ] && [ "$(readlink "$d")" = /dev/null ]; then echo "kept masked: $(basename "$u")"; continue; fi
    install -m 644 "$u" "$D/etc/systemd/system/"
done
[ "$SYSTEMCTL" = 1 ] && systemctl daemon-reload
# boot autostart is opt-in (AUTOSTART=1): the reference phone overheats without cooling
if [ "${AUTOSTART:-0}" = 1 ] && [ "$SYSTEMCTL" = 1 ]; then systemctl enable roadsense.target >/dev/null; fi
# laned (lane lines for the display) is optional and stays off: systemctl enable roadsense-laned.service
echo "roadsense installed${D:+ into $D}: daemons in /usr/local/bin, the debug face (port $PORT, bind $HOST) and the legacy mapd in /usr/local/lib/roadsense, units in /etc/systemd/system"
if [ "${WITH_CHRIS:-1}" = 1 ]; then
    # chris-integrator on the little cores 0-2 (cpu 3 is the car backend's, 4-7 run the models); the
    # integrator's link to this face (ROADSENSE_HTTP) was set above when a debug port was given
    echo "--- chris-integrator"
    if [ "$SYSTEMCTL" = 1 ]; then CPUS=${CHRIS_CPUS:-0-2} sh chris-integrator/scripts/install.sh
    else CPUS=${CHRIS_CPUS:-0-2} DESTDIR=$D sh chris-integrator/scripts/install.sh --no-systemctl; fi
else
    echo "chris-integrator skipped (WITH_CHRIS=0): consumers then have no consolidated API; see chris-integrator/docs/INSTALL.md"
    echo "start:  systemctl start roadsense.target   status: systemctl status 'roadsense-*'"
fi
