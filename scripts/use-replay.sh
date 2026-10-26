#!/bin/sh
# Switch the stack's frame source to a recorded ride (camerad-replay) and back. Run as root from a repo
# checkout after a build (default build dir ./build, see install-phone.sh; BUILD=... overrides).
#   scripts/use-replay.sh --enable /path/ride.mjpeg   install camerad-replay + unit, stop the current camera
#                                                     (camerad-dvr or camerad), switch calib.json to the ride's,
#                                                     start the replay (loops)
#   scripts/use-replay.sh --disable                   stop the replay, restore calib.json and the camera that
#                                                     ran before (default camerad-dvr)
#   scripts/use-replay.sh --status
# Per-ride settings: camera-replay.json (focal_px, cx, cy, fps) and calib.json next to the ride file are used if
# present, else config/camera-replay.json and config/calib-replay.json. See docs/lanes/L5-ride-replay.md.
set -eu
cd "$(dirname "$0")/.."
[ "$(id -u)" = 0 ] || { echo "run as root (sudo)"; exit 1; }
BUILD=${BUILD:-build}
ETC=/etc/roadsense
STATE=/var/lib/roadsense/replay-prev-source     # camera unit to restore: dvr | camerad
CALIB_BAK=$ETC/calib.json.pre-replay

# Ring readers built before the RingReader reopen fix (2026-10-26) never follow a replaced ring: restart the
# running ones after every switch (no-op for stopped units).
restart_readers() { systemctl try-restart roadsense-modeld.service roadsense-envd.service roadsense-signd.service || true; }

case "${1:-}" in
--status)
    systemctl --no-pager -n 0 status roadsense-camerad-replay.service 2>/dev/null | sed -n 1,3p || true
    [ -f $ETC/camera-replay.env ] && cat $ETC/camera-replay.env
    [ -f $STATE ] && echo "restores on --disable: $(cat $STATE)"
    exit 0 ;;
--disable)
    systemctl disable --now roadsense-camerad-replay.service 2>/dev/null || true
    [ -f $CALIB_BAK ] && mv -f $CALIB_BAK $ETC/calib.json
    prev=$(cat $STATE 2>/dev/null || echo dvr)
    rm -f $STATE
    if [ "$prev" = camerad ]; then
        systemctl unmask roadsense-camerad.service 2>/dev/null || true
        install -m 644 systemd/roadsense-camerad.service /etc/systemd/system/
        systemctl daemon-reload
        systemctl start roadsense-camerad.service
        restart_readers
        echo "replay off; switched back to camerad"
    else
        systemctl daemon-reload
        systemctl enable --now roadsense-camerad-dvr.service
        restart_readers
        echo "replay off; switched back to camerad-dvr"
    fi
    exit 0 ;;
--enable) [ $# -ge 2 ] || { echo "usage: $0 --enable FILE.mjpeg"; exit 2; } ;;
*) echo "usage: $0 --enable FILE.mjpeg | --disable | --status"; exit 2 ;;
esac

FILE=$(readlink -f "$2")
[ -r "$FILE" ] || { echo "$FILE not readable"; exit 1; }
DIR=$(dirname "$FILE")
if [ -x "$BUILD/camerad-replay/camerad-replay" ]; then
    install -m 755 "$BUILD/camerad-replay/camerad-replay" /usr/local/bin/
elif [ ! -x /usr/local/bin/camerad-replay ] || [ camerad-replay/main.cc -nt /usr/local/bin/camerad-replay ]; then
    # one translation unit + libjpeg: build it directly (seconds) instead of a full cmake build
    echo "building camerad-replay"
    nice c++ -std=c++17 -O2 -Wall -Wextra -Icamerad camerad-replay/main.cc -ljpeg -o /var/tmp/camerad-replay.new
    install -m 755 /var/tmp/camerad-replay.new /usr/local/bin/camerad-replay && rm -f /var/tmp/camerad-replay.new
fi
[ -x /usr/local/bin/camerad-replay ] || { echo "camerad-replay missing: build first (needs libjpeg-turbo)"; exit 1; }
mkdir -p $ETC /var/lib/roadsense
install -m 644 systemd/roadsense-camerad-replay.service /etc/systemd/system/

# Remember which camera to restore (only on the first --enable; a second --enable just changes the ride).
if ! systemctl -q is-active roadsense-camerad-replay.service || [ ! -f $STATE ]; then
    if systemctl -q is-active roadsense-camerad.service; then echo camerad > $STATE; else echo dvr > $STATE; fi
fi
[ -f $CALIB_BAK ] || { [ -f $ETC/calib.json ] && cp -p $ETC/calib.json $CALIB_BAK || true; }

if [ -f "$DIR/camera-replay.json" ]; then install -m 644 "$DIR/camera-replay.json" $ETC/camera-replay.json
else install -m 644 config/camera-replay.json $ETC/camera-replay.json; fi
if [ -f "$DIR/calib.json" ]; then install -m 644 "$DIR/calib.json" $ETC/calib.json
else install -m 644 config/calib-replay.json $ETC/calib.json; fi     # modeld re-reads calib.json within 2 s
printf 'REPLAY_FILE=%s\nREPLAY_ARGS=%s\n' "$FILE" "${REPLAY_ARGS:-}" > $ETC/camera-replay.env

systemctl daemon-reload
systemctl disable --now roadsense-camerad-dvr.service 2>/dev/null || true
systemctl stop roadsense-camerad.service 2>/dev/null || true
# roadsense.target and modeld Want camerad: keep it masked (as install-camerad-dvr.sh does), or a modeld restart
# would pull it in and its Conflicts= would stop the replay.
if [ "$(readlink /etc/systemd/system/roadsense-camerad.service 2>/dev/null)" != /dev/null ]; then
    rm -f /etc/systemd/system/roadsense-camerad.service
    systemctl mask roadsense-camerad.service
fi
systemctl enable roadsense-camerad-replay.service
systemctl restart roadsense-camerad-replay.service
sleep 1
restart_readers
echo "switched to camerad-replay: $FILE (back: $0 --disable; restores $(cat $STATE))"
