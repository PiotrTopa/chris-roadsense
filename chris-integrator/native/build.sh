#!/bin/sh
# Build chris-integrator's native core for THIS machine: one C file -> one shared object.
#
#   sh native/build.sh                 -> chris_integrator/native/_chris_native.abi3.so   (used at the next import)
#   OUT=/some/dir sh native/build.sh   -> /some/dir/_chris_native.abi3.so                 (an artefact to ship)
#
# Needs a C compiler and the Python headers (Alpine: gcc musl-dev python3-dev; Debian: gcc python3-dev).
# Environment: CC (default cc), PYTHON (default python3), CFLAGS (added), SANITIZE=1 (an address-sanitizer
# build for the tests: run them with LD_PRELOAD=$(cc -print-file-name=libasan.so)).
# The module uses the CPython limited API: the binary loads in every CPython >= 3.11 of the same platform
# (machine and C library), whatever version built it. docs/NATIVE.md has the cross build for a phone.
set -eu
SRC=$(cd "$(dirname "$0")/.." && pwd)
CC=${CC:-cc}
PYTHON=${PYTHON:-python3}
OUT=${OUT:-$SRC/chris_integrator/native}
command -v "$CC" >/dev/null 2>&1 || { echo "build.sh: no C compiler ($CC)" >&2; exit 1; }
INC=$("$PYTHON" -c 'import sysconfig; print(sysconfig.get_paths()["include"])')
[ -f "$INC/Python.h" ] || { echo "build.sh: no Python headers in $INC (Alpine: apk add python3-dev)" >&2; exit 1; }
# -ffp-contract=off: no fused multiply-add (it would change the last bit of results that are otherwise
# numpy's); no -ffast-math, no -march: the arithmetic is IEEE in the order written.
FLAGS="-std=c99 -O3 -fPIC -shared -ffp-contract=off -fno-fast-math -fvisibility=hidden -Wall -Wextra -Wno-unused-parameter"
[ "${SANITIZE:-0}" = 1 ] && FLAGS="$FLAGS -O1 -g -fsanitize=address,undefined -fno-omit-frame-pointer"
mkdir -p "$OUT"
# shellcheck disable=SC2086
"$CC" $FLAGS ${CFLAGS:-} -I"$INC" "$SRC/chris_integrator/native/src/chris_native.c" -o "$OUT/_chris_native.abi3.so.tmp" -lm
mv "$OUT/_chris_native.abi3.so.tmp" "$OUT/_chris_native.abi3.so"
echo "built $OUT/_chris_native.abi3.so ($("$CC" --version | head -n 1); headers of $("$PYTHON" -V 2>&1))"
if [ "$OUT" = "$SRC/chris_integrator/native" ] && [ "${SANITIZE:-0}" != 1 ]; then
    (cd "$SRC" && "$PYTHON" -m chris_integrator.native)
fi
