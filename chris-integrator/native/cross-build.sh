#!/bin/sh
# Build the native core for another Linux platform in a container (docker; other machines through
# qemu-user binfmt), without installing anything on the target: for the reference deployment, a
# postmarketOS / Alpine phone (aarch64, musl), whose packages are pinned.
#
#   sh native/cross-build.sh                    -> native/dist/linux-aarch64-musl/_chris_native.abi3.so (+ BUILD.txt)
#   PLATFORM=linux/amd64 sh native/cross-build.sh       -> native/dist/linux-x86_64-musl/
#   IMAGE=debian:stable-slim PLATFORM=linux/arm64 ...   -> native/dist/linux-aarch64-glibc/
#   TEST=0 ...                                  skip the test run in the container (pytest tests/test_native.py)
#
# The container builds with the image's gcc against the image's Python headers and runs the self-test and
# the native tests there (emulated: correct, slow). The binary uses the CPython limited API, so the image's
# Python version need not be the target's: any CPython >= 3.11 on the same machine type and C library loads
# it. `scripts/install.sh` picks the artefact from native/dist/ by the target's machine and C library.
set -eu
SRC=$(cd "$(dirname "$0")/.." && pwd)
PLATFORM=${PLATFORM:-linux/arm64}
IMAGE=${IMAGE:-alpine:edge}
TEST=${TEST:-1}
command -v docker >/dev/null 2>&1 || { echo "cross-build.sh: docker is needed" >&2; exit 1; }
case "$IMAGE" in
    alpine*) PREP="apk add --no-cache gcc musl-dev python3-dev py3-numpy py3-pytest >/dev/null" ;;
    *) PREP="apt-get update -qq >/dev/null && apt-get install -y -qq gcc libc6-dev python3-dev python3-numpy python3-pytest >/dev/null" ;;
esac
OUTROOT=$SRC/native/dist
mkdir -p "$OUTROOT"
docker run --rm --platform "$PLATFORM" -e TEST="$TEST" -e HOST_UID="$(id -u)" -e HOST_GID="$(id -g)" \
    -v "$SRC":/src:ro -v "$OUTROOT":/out "$IMAGE" sh -euc "
$PREP
libc=glibc; ldd --version 2>&1 | grep -qi musl && libc=musl
tag=linux-\$(uname -m)-\$libc
mkdir -p /work /out/\$tag
cp -r /src/chris_integrator /src/tests /src/native /work/
rm -f /work/chris_integrator/native/_chris_native*.so
cd /work
sh native/build.sh
cp chris_integrator/native/_chris_native.abi3.so /out/\$tag/
{
  echo \"platform: \$tag\"
  echo \"image: $IMAGE ($PLATFORM)\"
  echo \"compiler: \$(cc --version | head -n 1)\"
  echo \"python headers: \$(python3 -V 2>&1)\"
  echo \"numpy in the test run: \$(python3 -c 'import numpy; print(numpy.__version__)')\"
  echo \"source sha256: \$(sha256sum chris_integrator/native/src/chris_native.c | cut -d' ' -f1)\"
  echo \"binary sha256: \$(sha256sum chris_integrator/native/_chris_native.abi3.so | cut -d' ' -f1)\"
} > /out/\$tag/BUILD.txt
if [ \"\$TEST\" = 1 ]; then
  python3 -m pytest -q -p no:cacheprovider tests/test_native.py 2>&1 | tail -n 3 | tee -a /out/\$tag/BUILD.txt
fi
chown -R \$HOST_UID:\$HOST_GID /out/\$tag
cat /out/\$tag/BUILD.txt
"
