#!/bin/sh
# Reference deployment: copy the working tree (no build dirs, no models) to the phone's ~/roadsense. rsync is not on the phone.
set -eu
cd "$(dirname "$0")/.."
T=${1:-${ROADSENSE_PHONE:-user@phone}}
# The tracked files, and chris-integrator's native core as built for the target (native/dist/, made on this
# machine by chris-integrator/native/cross-build.sh; not tracked): its install script uses the artefact that
# fits the phone and passes its self-test there, and installs pure Python when there is none.
{ git ls-files -z; find chris-integrator/native/dist -type f -print0 2>/dev/null || true; } \
    | tar czf - --null -T - | ssh "$T" 'mkdir -p ~/roadsense && tar xzf - -C ~/roadsense'
echo "synced to $T:~/roadsense"
if [ -d chris-integrator/native/dist ]; then
    echo "  with the native core for: $(ls chris-integrator/native/dist | tr '\n' ' ')"
else
    echo "  without a built native core (chris-integrator/native/cross-build.sh makes one): chris-integrator will run pure Python"
fi
