#!/bin/sh
# Install chris-integrator (and the map box chris-mapper, when its tree is next to this one) on a host with
# systemd. Run as root from a checkout. Needs python3, numpy and aiohttp on the host.
#   API_PORT=N API_HOST=ADDR ROADSENSE_HTTP=URL   are written to /etc/chris/integrator.env (kept afterwards)
set -eu
cd "$(dirname "$0")/.."
[ "$(id -u)" = 0 ] || { echo "run as root (sudo)"; exit 1; }
mkdir -p /usr/local/lib/chris /etc/chris /var/lib/chris/maps
rm -rf /usr/local/lib/chris/chris_integrator && cp -r chris_integrator /usr/local/lib/chris/
if [ -d ../chris-mapper/chris_mapper ]; then
    rm -rf /usr/local/lib/chris/chris_mapper && cp -r ../chris-mapper/chris_mapper /usr/local/lib/chris/
fi
find /usr/local/lib/chris -name __pycache__ -prune -exec rm -rf {} +
if [ -n "${API_PORT:-}" ] || [ -n "${API_HOST:-}" ] || [ -n "${ROADSENSE_HTTP:-}" ]; then
    { [ -n "${API_PORT:-}" ] && printf 'CHRIS_API_PORT=%s\n' "$API_PORT"
      [ -n "${API_HOST:-}" ] && printf 'CHRIS_API_HOST=%s\n' "$API_HOST"
      [ -n "${ROADSENSE_HTTP:-}" ] && printf 'CHRIS_ROADSENSE_HTTP=%s\n' "$ROADSENSE_HTTP"
      true
    } > /etc/chris/integrator.env
fi
install -m 644 systemd/chris-integrator.service /etc/systemd/system/
systemctl daemon-reload
echo "installed. start:  systemctl start chris-integrator   status: curl -s http://127.0.0.1:${API_PORT:-8080}/v2/mixer"
