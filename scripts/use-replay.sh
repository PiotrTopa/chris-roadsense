#!/bin/sh
# Switch the stack's frame source to a recorded ride (camerad-replay) and back. Run as root from a repo
# checkout after a build (default build dir ./build, see install-phone.sh; BUILD=... overrides).
#   scripts/use-replay.sh --enable /path/ride.mjpeg   install camerad-replay + unit, stop the current camera
#                                                     (camerad-dvr or camerad), switch calib.json to the ride's,
#                                                     start the replay (loops)
#   scripts/use-replay.sh --playlist [DIR] [--shuffle] every ride under DIR (default ~$SUDO_USER/rides, else
#                                                     /home/user/rides) one after another, looping: order of the
#                                                     catalogue in docs/RIDES.md, then any other ride dirs by
#                                                     name; the list goes to /etc/roadsense/replay-playlist.txt
#                                                     (edit it freely: re-read at every wrap). camerad-replay
#                                                     installs each ride's calib.json itself (modeld follows).
#   scripts/use-replay.sh --disable                   stop the replay, restore calib.json and the camera that
#                                                     ran before (default camerad-dvr)
#   scripts/use-replay.sh --status
# Per-ride settings: camera-replay.json (focal_px, cx, cy, fps) and calib.json next to the ride file are used if
# present, else config/camera-replay.json and config/calib-replay.json. See docs/lanes/L5-ride-replay.md and
# docs/lanes/L10-playlist.md. Switching between --enable and --playlist keeps the pre-replay calib.json backup
# and the camera to restore from the first switch.
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
    grep -q '^REPLAY_MODE=--playlist' $ETC/camera-replay.env 2>/dev/null && \
        echo "playlist: $(grep -cv '^#' $ETC/replay-playlist.txt) rides in $ETC/replay-playlist.txt"
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
--enable) [ $# -ge 2 ] || { echo "usage: $0 --enable FILE.mjpeg"; exit 2; }
    MODE=--file
    FILE=$(readlink -f "$2")
    [ -r "$FILE" ] || { echo "$FILE not readable"; exit 1; }
    DIR=$(dirname "$FILE") ;;
--playlist)
    MODE=--playlist
    shift
    RIDES=""
    SHUFFLE=""
    for a in "$@"; do
        case "$a" in --shuffle) SHUFFLE=--shuffle ;; *) RIDES=$a ;; esac
    done
    if [ -z "$RIDES" ]; then
        RIDES=/home/user/rides
        [ -n "${SUDO_USER:-}" ] && [ -d "/home/$SUDO_USER/rides" ] && RIDES=/home/$SUDO_USER/rides
    fi
    RIDES=$(readlink -f "$RIDES")
    [ -d "$RIDES" ] || { echo "$RIDES: not a directory"; exit 1; } ;;
*) echo "usage: $0 --enable FILE.mjpeg | --playlist [DIR] [--shuffle] | --disable | --status"; exit 2 ;;
esac
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

if [ "$MODE" = --file ]; then
    if [ -f "$DIR/camera-replay.json" ]; then install -m 644 "$DIR/camera-replay.json" $ETC/camera-replay.json
    else install -m 644 config/camera-replay.json $ETC/camera-replay.json; fi
    if [ -f "$DIR/calib.json" ]; then install -m 644 "$DIR/calib.json" $ETC/calib.json
    else install -m 644 config/calib-replay.json $ETC/calib.json; fi     # modeld re-reads calib.json within 2 s
    ARGS=${REPLAY_ARGS:-}
else
    # Playlist in the catalogue order of docs/RIDES.md (rows of the "## Catalogue" table), then the rest by name.
    LIST=$ETC/replay-playlist.txt
    {
        echo "# written by scripts/use-replay.sh --playlist $RIDES ($(date -Iseconds)); one ride dir per line,"
        echo "# re-read by camerad-replay at every wrap"
        listed=" "
        for slug in $(sed -n '/^## Catalogue/,/^## /p' docs/RIDES.md 2>/dev/null | sed -n 's/^| \([a-z0-9][a-z0-9-]*\) |.*/\1/p'); do
            if [ -d "$RIDES/$slug" ] && ls "$RIDES/$slug"/*.mjpeg >/dev/null 2>&1; then echo "$RIDES/$slug"; listed="$listed$slug "; fi
        done
        for d in "$RIDES"/*/; do
            slug=$(basename "$d")
            case "$listed" in *" $slug "*) continue ;; esac
            ls "$d"*.mjpeg >/dev/null 2>&1 && echo "$RIDES/$slug"
        done
    } > $LIST.new
    n=$(grep -cv '^#' $LIST.new || true)
    [ "$n" -gt 0 ] || { rm -f $LIST.new; echo "no rides (dirs with an .mjpeg) under $RIDES"; exit 1; }
    mv -f $LIST.new $LIST
    FILE=$LIST
    ARGS="$SHUFFLE ${REPLAY_ARGS:-}"
fi
printf 'REPLAY_MODE=%s\nREPLAY_FILE=%s\nREPLAY_ARGS=%s\n' "$MODE" "$FILE" "$ARGS" > $ETC/camera-replay.env

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
if [ "$MODE" = --playlist ]; then
    echo "switched to camerad-replay playlist: $n rides from $FILE $SHUFFLE (back: $0 --disable; restores $(cat $STATE))"
else
    echo "switched to camerad-replay: $FILE (back: $0 --disable; restores $(cat $STATE))"
fi
