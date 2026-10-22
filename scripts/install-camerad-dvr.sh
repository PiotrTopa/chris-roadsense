#!/bin/sh
# Install camerad-dvr (USB DVR camera 1902:3241 as the frame source) on a systemd host.
# Run as root from a repo checkout, after a build (default build dir ./build, see install-phone.sh).
#   scripts/install-camerad-dvr.sh            install binary, unit, default config (nothing enabled)
#   scripts/install-camerad-dvr.sh --enable   also switch the stack to the DVR camera: mask
#                                             roadsense-camerad.service, enable + start this unit
#   scripts/install-camerad-dvr.sh --disable  switch back to the phone camera (camerad)
# Re-run with --enable after scripts/install-phone.sh (it reinstalls the camerad unit file).
set -eu
cd "$(dirname "$0")/.."
[ "$(id -u)" = 0 ] || { echo "run as root (sudo)"; exit 1; }
BUILD=${BUILD:-build}
case "${1:-}" in
--disable)
    systemctl disable --now roadsense-camerad-dvr.service || true
    rm -f /etc/systemd/system/roadsense-camerad.service
    install -m 644 systemd/roadsense-camerad.service /etc/systemd/system/
    systemctl daemon-reload
    systemctl start roadsense-camerad.service
    echo "switched to camerad"; exit 0 ;;
--enable|"") ;;
*) echo "usage: $0 [--enable|--disable]"; exit 2 ;;
esac
[ -x "$BUILD/camerad-dvr/camerad-dvr" ] || { echo "$BUILD/camerad-dvr/camerad-dvr missing: build first (needs libjpeg-turbo)"; exit 1; }
install -m 755 "$BUILD/camerad-dvr/camerad-dvr" /usr/local/bin/
mkdir -p /etc/roadsense
[ -f /etc/roadsense/camera-dvr.json ] || install -m 644 config/camera-dvr.json /etc/roadsense/
install -m 644 systemd/roadsense-camerad-dvr.service /etc/systemd/system/
systemctl daemon-reload
if [ "${1:-}" = --enable ]; then
    # roadsense.target and modeld Want camerad; a masked camerad is skipped, so only camerad-dvr runs.
    systemctl stop roadsense-camerad.service 2>/dev/null || true
    rm -f /etc/systemd/system/roadsense-camerad.service
    systemctl mask roadsense-camerad.service
    systemctl enable --now roadsense-camerad-dvr.service
    echo "switched to camerad-dvr (back: $0 --disable)"
else
    echo "installed. switch the stack to the DVR camera: $0 --enable"
fi
