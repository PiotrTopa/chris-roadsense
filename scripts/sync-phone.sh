#!/bin/sh
# Copy the working tree (no build dirs, no models) to the phone's ~/pocopilot. rsync is not on the phone.
set -eu
cd "$(dirname "$0")/.."
T=${1:-user@192.168.0.86}
git ls-files -z | tar czf - --null -T - | ssh "$T" 'mkdir -p ~/pocopilot && tar xzf - -C ~/pocopilot'
echo "synced to $T:~/pocopilot"
