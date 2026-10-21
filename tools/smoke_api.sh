#!/bin/sh
# Smoke test: fake daemons + api, every endpoint (no hardware, no curl needed).
#   tools/smoke_api.sh                 # free port
#   POCOPILOT_TEST_PORT=8081 OVERLAY_OUT=/tmp/overlay.jpg tools/smoke_api.sh
set -e
cd "$(dirname "$0")/.."
PY=${PYTHON:-python3}
exec "$PY" -m unittest discover -s api/tests -t . -v
