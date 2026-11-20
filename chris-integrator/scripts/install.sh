#!/bin/sh
# Install chris-integrator (and the map box chris-mapper, when its tree is next to this one or MAPPER_SRC
# names it) on a host with systemd, from a source tree. POSIX sh and busybox tools only; no network, no pip.
# Re-runnable: a second run changes nothing that the first one set up. docs/INSTALL.md has the details.
#
#   sh scripts/install.sh [--no-systemctl] [--check]       as root, on the host
#   DESTDIR=/scratch/dir sh scripts/install.sh              into a scratch prefix (no root, no systemctl):
#                                                           DESTDIR/run.sh then starts the daemon from there
# Environment (all optional):
#   CHRIS_API_PORT, CHRIS_API_HOST, ROADSENSE_HTTP
#                                        the consolidated API's port / bind address and the camera instance's
#                                        debug face; written to <etc>/chris/integrator.env (defaults 8090,
#                                        127.0.0.1, http://127.0.0.1:8091). An existing file is kept: only
#                                        missing keys are added, a key given here explicitly is replaced.
#                                        (API_PORT / API_HOST, the names before L46, are read when the CHRIS_
#                                        ones are not set. roadsense's scripts/install-phone.sh uses its own
#                                        names for the debug face and does not pass those two on.)
#   MAP_PACK                             CHRIS_MAP_PACK in integrator.env (default: <data>/maps/current.rsmap)
#   CPUS                                 CPUAffinity of the unit (a drop-in), e.g. CPUS=0-2; unset: none
#   PYTHON                               the interpreter checked and used by run.sh (default python3); the unit
#                                        always runs /usr/bin/python3
#   MAPPER_SRC                           the chris-mapper tree (default ../chris-mapper next to this tree)
#   STOP_LEGACY=0                        do not stop / disable a legacy roadsense mapd (default: stop + disable)
set -eu

SRC=$(cd "$(dirname "$0")/.." && pwd)
D=${DESTDIR:-}
D=${D%/}
SYSTEMCTL=1
CHECK_ONLY=0
for a in "$@"; do
    case "$a" in
        --no-systemctl) SYSTEMCTL=0 ;;
        --check) CHECK_ONLY=1 ;;
        -h|--help) sed -n '2,24p' "$0"; exit 0 ;;
        *) echo "install.sh: unknown argument $a" >&2; exit 2 ;;
    esac
done
[ -n "$D" ] && SYSTEMCTL=0
PYTHON=${PYTHON:-python3}
MAPPER_SRC=${MAPPER_SRC:-$SRC/../chris-mapper}
[ -d "$MAPPER_SRC/chris_mapper" ] && MAPPER_SRC=$(cd "$MAPPER_SRC" && pwd) || MAPPER_SRC=
DONE=""
note() { DONE="$DONE
  - $*"; }
die() { echo "install.sh: $*" >&2; exit 1; }

# ---- 1. prerequisites: nothing is changed before all of them hold ----------------------------------------
[ -n "$D" ] || [ "$(id -u)" = 0 ] || die "run as root (or set DESTDIR=/some/dir for a scratch install)"
[ -f "$SRC/chris_integrator/__init__.py" ] || die "no chris_integrator package in $SRC"
[ -f "$SRC/systemd/chris-integrator.service" ] || die "no systemd/chris-integrator.service in $SRC"
command -v "$PYTHON" >/dev/null 2>&1 || die "no $PYTHON (needs Python 3.10 or newer)"
"$PYTHON" -c 'import sys; sys.exit(0 if sys.version_info >= (3, 10) else 1)' \
    || die "$("$PYTHON" -V 2>&1) is too old: Python 3.10 or newer is needed"
# third-party modules imported at run time (an import scan of both packages): numpy (the estimator core and
# the map box: required), aiohttp (the consolidated HTTP / WebSocket API: required for a daemon that serves
# consumers). Nothing else outside the standard library.
for m in numpy aiohttp; do
    "$PYTHON" -c "import $m" 2>/dev/null || die "python module '$m' is missing for $PYTHON (Alpine: apk add py3-$m; Debian: apt install python3-$m); nothing was changed"
done
# the whole import chain of the daemon with this interpreter, from the source tree (no byte code written)
PYTHONDONTWRITEBYTECODE=1 PYTHONPATH="$SRC${MAPPER_SRC:+:$MAPPER_SRC}" "$PYTHON" -c '
import chris_integrator.daemon, chris_integrator.io.http, chris_integrator.stack
import sys
if len(sys.argv) > 1: import chris_mapper.box' ${MAPPER_SRC:+mapper} \
    || die "the daemon does not import with $PYTHON (see the error above); nothing was changed"
if [ "$SYSTEMCTL" = 1 ]; then
    command -v systemctl >/dev/null 2>&1 || die "no systemctl (use --no-systemctl, or DESTDIR=...)"
fi
if [ -n "${CPUS:-}" ]; then
    case "$CPUS" in *[!0-9,-]*) die "CPUS=$CPUS: a CPU list like 0-2 or 0,1,2" ;; esac
fi
API_PORT=${CHRIS_API_PORT:-${API_PORT:-}}
API_HOST=${CHRIS_API_HOST:-${API_HOST:-}}
# the effective ports (what integrator.env will say): the API's and the camera debug face's must differ
envval() { [ -f "${D}/etc/chris/integrator.env" ] && sed -n "s/^$1=//p" "${D}/etc/chris/integrator.env" | tail -n 1; true; }
EFF_PORT=${API_PORT:-$(envval CHRIS_API_PORT)}; EFF_PORT=${EFF_PORT:-8090}
EFF_RS=${ROADSENSE_HTTP:-$(envval CHRIS_ROADSENSE_HTTP)}; EFF_RS=${EFF_RS:-http://127.0.0.1:8091}
hp=${EFF_RS#*://}; hp=${hp%%/*}
[ "${hp##*:}" = "$EFF_PORT" ] && die "the camera's debug face ($EFF_RS) and the consolidated API (port $EFF_PORT) collide; nothing was changed"
PYV=$("$PYTHON" -c 'import sys, numpy, aiohttp; print("python %d.%d.%d, numpy %s, aiohttp %s" % (sys.version_info[:3] + (numpy.__version__, aiohttp.__version__)))')
echo "prerequisites ok: $PYV${MAPPER_SRC:+, chris-mapper tree $MAPPER_SRC}"
[ -n "$MAPPER_SRC" ] || echo "note: no chris-mapper tree found: installing without the map box (the integrator runs and says so)"
[ "$CHECK_ONLY" = 1 ] && exit 0

LIB=$D/usr/local/lib/chris
ETC=$D/etc/chris
DATA=$D/var/lib/chris
UNITS=$D/etc/systemd/system
RS_ETC=$D/etc/roadsense
RS_DATA=$D/var/lib/roadsense

# ---- 2. code -------------------------------------------------------------------------------------------
mkdir -p "$LIB" "$ETC" "$DATA/maps" "$UNITS"
put_pkg() {   # $1 source package dir, $2 name: copied next to the old one, then swapped
    rm -rf "$LIB/$2.new"
    cp -r "$1" "$LIB/$2.new"
    find "$LIB/$2.new" -name __pycache__ -type d -prune -exec rm -rf {} \;
    rm -rf "$LIB/$2"
    mv "$LIB/$2.new" "$LIB/$2"
    note "code: $LIB/$2"
}
put_pkg "$SRC/chris_integrator" chris_integrator
if [ -n "$MAPPER_SRC" ]; then put_pkg "$MAPPER_SRC/chris_mapper" chris_mapper; fi

# ---- 3. integrator.env: the car defaults, an existing file kept --------------------------------------
ENVF=$ETC/integrator.env
[ -f "$ENVF" ] || { : > "$ENVF"; note "created $ENVF"; }
set_key() {   # $1 key, $2 value, $3 1 = explicitly given (replaces), 0 = default (only when missing)
    if grep -q "^$1=" "$ENVF"; then
        if [ "$3" = 1 ] && [ "$(sed -n "s/^$1=//p" "$ENVF" | tail -n 1)" != "$2" ]; then
            sed -i "s|^$1=.*|$1=$2|" "$ENVF"
            note "$ENVF: $1=$2 (replaced)"
        fi
    else
        printf '%s=%s\n' "$1" "$2" >> "$ENVF"
        note "$ENVF: $1=$2 (added)"
    fi
}
set_key CHRIS_API_PORT "${API_PORT:-8090}" "$([ -n "${API_PORT:-}" ] && echo 1 || echo 0)"
set_key CHRIS_API_HOST "${API_HOST:-127.0.0.1}" "$([ -n "${API_HOST:-}" ] && echo 1 || echo 0)"
set_key CHRIS_ROADSENSE_HTTP "${ROADSENSE_HTTP:-http://127.0.0.1:8091}" "$([ -n "${ROADSENSE_HTTP:-}" ] && echo 1 || echo 0)"
[ -n "${MAP_PACK:-}" ] && set_key CHRIS_MAP_PACK "$MAP_PACK" 1
RS_URL=$(sed -n 's/^CHRIS_ROADSENSE_HTTP=//p' "$ENVF" | tail -n 1)
CH_PORT=$(sed -n 's/^CHRIS_API_PORT=//p' "$ENVF" | tail -n 1)

# ---- 4. migration from a roadsense install (copies; the originals stay for a rollback) ----------------
copy_if_absent() {   # $1 from, $2 to
    if [ -e "$1" ] && [ ! -e "$2" ]; then
        cp -p "$1" "$2"
        note "migrated $1 -> $2"
    elif [ -e "$1" ]; then
        note "kept $2 (already there; $1 not copied)"
    fi
}
for n in recorder integrator mixer road_users lead lanes egolane view roadgeom lights rules fused rates; do
    copy_if_absent "$RS_ETC/$n.json" "$ETC/$n.json"
done
# the legacy mapd's mapd.json: the keys the map box reads go to mapper.json; the others are reported
if [ -f "$RS_ETC/mapd.json" ] && [ ! -e "$ETC/mapper.json" ] && [ -f "$LIB/chris_mapper/box.py" ]; then
    PYTHONDONTWRITEBYTECODE=1 PYTHONPATH="$LIB" "$PYTHON" - "$RS_ETC/mapd.json" "$ETC/mapper.json" <<'EOF'
import json, sys
from chris_mapper.box import DEFAULTS
src = json.load(open(sys.argv[1]))
keep = {k: v for k, v in src.items() if k in DEFAULTS and k != "pack"}   # the pack: the symlink of step 5
skip = sorted(k for k in src if k not in keep)
with open(sys.argv[2], "w") as f:
    f.write(json.dumps(keep, indent=2) + "\n")
print("mapper.json: %s; not carried: %s" % (", ".join(sorted(keep)) or "nothing",
      ", ".join(skip) or "nothing"))
print("  (pack: via <data>/maps/current.rsmap; hz: the box matches every 0.45 s of input time; wheelbase_m, "
      "steering_ratio, track_m, vision_odometry: read by nothing in chris; the wheel-speed yaw uses a "
      "track of 1.48 m fixed in chris_integrator/sources/vehicle.py)")
EOF
    note "wrote $ETC/mapper.json from $RS_ETC/mapd.json"
elif [ -f "$RS_ETC/mapd.json" ] && [ -e "$ETC/mapper.json" ]; then
    note "kept $ETC/mapper.json (mapd.json not read again)"
fi
# state carried across restarts (faults, calibration) and the sign history
copy_if_absent "$RS_DATA/integrator.json" "$DATA/integrator.json"
copy_if_absent "$RS_DATA/signs.jsonl" "$DATA/signs.jsonl"
# the map pack: a symlink to the file roadsense's current pack resolves to (a DESTDIR is followed inside)
resolve() {   # print the final target of $1 within $D, or nothing when it does not resolve to a file
    p=$1; i=0
    while [ -L "$p" ] && [ $i -lt 20 ]; do
        t=$(readlink "$p")
        case "$t" in /*) p=$D$t ;; *) p=$(dirname "$p")/$t ;; esac
        i=$((i + 1))
    done
    [ -f "$p" ] && echo "$p"
}
PACK=$DATA/maps/current.rsmap
if [ -e "$PACK" ] || [ -L "$PACK" ]; then
    note "kept $PACK -> $(readlink "$PACK" 2>/dev/null || echo '(a file)')"
elif [ -e "$RS_DATA/maps/current.rsmap" ] || [ -L "$RS_DATA/maps/current.rsmap" ]; then
    T=$(resolve "$RS_DATA/maps/current.rsmap" || true)
    if [ -n "$T" ]; then
        ln -s "$T" "$PACK"
        note "map pack: $PACK -> $T"
    else
        note "WARNING: $RS_DATA/maps/current.rsmap does not resolve to a file: no map pack linked (the integrator runs without the map and says so)"
    fi
else
    note "no map pack found: the integrator runs without the map and says so (install one as $PACK)"
fi
if [ -f "$ETC/recorder.json" ]; then
    RDIR=$("$PYTHON" -c 'import json,sys; print(json.load(open(sys.argv[1])).get("dir") or "")' "$ETC/recorder.json" 2>/dev/null || true)
    note "recorder: sessions go to ${RDIR:-(no dir in recorder.json)} as named in $ETC/recorder.json"
fi
# roadsense's api becomes the camera's debug face behind the integrator: its port and bind address follow
# CHRIS_ROADSENSE_HTTP (default 127.0.0.1:8091); the old file is kept once as api.env.pre-chris
if [ -d "$RS_ETC" ]; then
    hp=${RS_URL#*://}; hp=${hp%%/*}
    RS_HOST=${hp%:*}; RS_PORT=${hp##*:}
    [ "$RS_PORT" = "$hp" ] && RS_PORT=80
    case "$RS_HOST" in localhost|127.*) RS_BIND=127.0.0.1 ;; *) RS_BIND=0.0.0.0 ;; esac
    AE=$RS_ETC/api.env
    if [ -f "$AE" ] && [ ! -e "$AE.pre-chris" ]; then cp -p "$AE" "$AE.pre-chris"; note "backup $AE.pre-chris"; fi
    [ -f "$AE" ] || : > "$AE"
    setkv() {   # $1 key, $2 value
        if grep -q "^$1=" "$AE"; then
            [ "$(sed -n "s/^$1=//p" "$AE" | tail -n 1)" = "$2" ] || { sed -i "s|^$1=.*|$1=$2|" "$AE"; note "$AE: $1=$2"; }
        else
            printf '%s=%s\n' "$1" "$2" >> "$AE"; note "$AE: $1=$2"
        fi
    }
    setkv ROADSENSE_API_PORT "$RS_PORT"
    # the bind address: loopback unless it is already set to something the integrator can reach
    OLD_BIND=$(sed -n 's/^ROADSENSE_API_HOST=//p' "$AE" | tail -n 1)
    case "$RS_HOST:$OLD_BIND" in
        *:) setkv ROADSENSE_API_HOST "$RS_BIND" ;;
        localhost:127.*|127.*:127.*|*:0.0.0.0|*:::) ;;
        *) setkv ROADSENSE_API_HOST "$RS_BIND" ;;
    esac
fi

# ---- 5. units ------------------------------------------------------------------------------------------
U=$UNITS/chris-integrator.service
if [ -L "$U" ] && [ "$(readlink "$U")" = /dev/null ]; then
    note "kept masked: chris-integrator.service (systemctl unmask chris-integrator.service to use it)"
else
    if ! cmp -s "$SRC/systemd/chris-integrator.service" "$U" 2>/dev/null; then
        install -m 644 "$SRC/systemd/chris-integrator.service" "$U"
        note "unit: $U"
    fi
fi
if [ -n "${CPUS:-}" ]; then
    mkdir -p "$U.d"
    printf '# written by chris-integrator/scripts/install.sh (CPUS=%s)\n[Service]\nCPUAffinity=%s\n' "$CPUS" "$CPUS" > "$U.d/50-cpus.tmp"
    if cmp -s "$U.d/50-cpus.tmp" "$U.d/50-cpus.conf"; then rm -f "$U.d/50-cpus.tmp"
    else mv "$U.d/50-cpus.tmp" "$U.d/50-cpus.conf"; note "drop-in: $U.d/50-cpus.conf (CPUAffinity=$CPUS)"; fi
fi
# membership of roadsense.target: [Install] WantedBy=roadsense.target, i.e. the symlink
# roadsense.target.wants/chris-integrator.service. It works whether or not the target itself is enabled:
# `systemctl start roadsense.target` pulls the unit in (Wants), `systemctl stop roadsense.target` stops it
# (PartOf). An earlier version was WantedBy=multi-user.target: that link is removed.
if [ "$SYSTEMCTL" = 1 ]; then
    systemctl daemon-reload
    systemctl reenable chris-integrator.service >/dev/null 2>&1 || die "systemctl reenable chris-integrator.service failed"
else
    rm -f "$UNITS/multi-user.target.wants/chris-integrator.service"
    mkdir -p "$UNITS/roadsense.target.wants"
    ln -sfn /etc/systemd/system/chris-integrator.service "$UNITS/roadsense.target.wants/chris-integrator.service"
fi
note "chris-integrator.service: wanted by roadsense.target (roadsense.target.wants/), PartOf it; not enabled at boot by itself"
# the legacy mapd estimates a position of its own: out of roadsense.target, disabled and stopped (not
# masked: `systemctl enable --now roadsense-mapd.service` undoes it)
T=$UNITS/roadsense.target
if [ -f "$T" ] && grep -q '^Wants=.*roadsense-mapd\.service' "$T"; then
    [ -e "$T.pre-chris" ] || cp -p "$T" "$T.pre-chris"
    sed -i '/^Wants=/ s/[[:space:]]*roadsense-mapd\.service//' "$T"
    note "removed roadsense-mapd.service from Wants= of $T (backup $T.pre-chris)"
fi
for f in "$UNITS"/roadsense.target.d/*.conf; do
    [ -f "$f" ] && grep -q 'roadsense-mapd' "$f" && note "WARNING: $f mentions roadsense-mapd: check it does not pull mapd in"
done
if [ "${STOP_LEGACY:-1}" = 1 ]; then
    if [ "$SYSTEMCTL" = 1 ]; then
        if systemctl cat roadsense-mapd.service >/dev/null 2>&1; then
            systemctl daemon-reload
            systemctl disable roadsense-mapd.service >/dev/null 2>&1 || true
            if systemctl is-active -q roadsense-mapd.service; then
                systemctl stop roadsense-mapd.service
                note "stopped roadsense-mapd.service"
            fi
            note "disabled roadsense-mapd.service (not masked)"
        fi
    elif [ -e "$UNITS/roadsense.target.wants/roadsense-mapd.service" ] || [ -L "$UNITS/roadsense.target.wants/roadsense-mapd.service" ]; then
        rm -f "$UNITS/roadsense.target.wants/roadsense-mapd.service"
        note "removed roadsense.target.wants/roadsense-mapd.service"
    fi
fi
[ "$SYSTEMCTL" = 1 ] && systemctl daemon-reload

# ---- 6. a scratch prefix: run.sh starts the daemon from it ----------------------------------------------
if [ -n "$D" ]; then
    PYABS=$(command -v "$PYTHON")
    cat > "$D/run.sh" <<EOF
#!/bin/sh
# Start chris-integrator as installed into this prefix (written by chris-integrator/scripts/install.sh).
# The unit's environment with the prefix's run / config / data dirs, then <etc>/chris/integrator.env.
# Override: CHRIS_ROADSENSE_RUNDIR (the camera instance's sockets), PYTHON; further arguments go to the daemon.
P=\$(cd "\$(dirname "\$0")" && pwd)
set -a
. "\$P/etc/chris/integrator.env"
set +a
export CHRIS_RUNDIR="\$P/run/chris" CHRIS_DATADIR="\$P/var/lib/chris" CHRIS_CONFIG_DIR="\$P/etc/chris"
export CHRIS_MAP_PACK="\${CHRIS_MAP_PACK:-\$P/var/lib/chris/maps/current.rsmap}"
export PYTHONPATH="\$P/usr/local/lib/chris" PYTHONDONTWRITEBYTECODE=1
mkdir -p "\$CHRIS_RUNDIR"
[ \${#CHRIS_RUNDIR} -le 90 ] || echo "run.sh: warning: \$CHRIS_RUNDIR is too long for Unix socket paths (108 bytes): install into a shorter DESTDIR or call run.sh through a short symlink" >&2
exec "\${PYTHON:-$PYABS}" -m chris_integrator --roadsense "\${CHRIS_ROADSENSE_RUNDIR:-\$P/run/roadsense}" "\$@"
EOF
    chmod 755 "$D/run.sh"
    note "scratch prefix: $D/run.sh starts the daemon from the prefix"
fi

# ---- 7. what was done, what is next --------------------------------------------------------------------
echo "chris-integrator installed${D:+ into $D}:$DONE"
cat <<EOF

next steps:
  - roadsense's own install (scripts/install-phone.sh of chris-roadsense) must have put the camera's debug
    face (api 3) in place; its port / bind address are in /etc/roadsense/api.env (now $(sed -n 's/^ROADSENSE_API_PORT=//p' "$RS_ETC/api.env" 2>/dev/null | tail -n 1) / $(sed -n 's/^ROADSENSE_API_HOST=//p' "$RS_ETC/api.env" 2>/dev/null | tail -n 1)).
  - start:   systemctl restart roadsense.target      (the api moves to its new port, chris-integrator starts,
                                                      the legacy mapd is no longer pulled in)
             or, without a target: systemctl start chris-integrator.service
  - check:   curl -s http://127.0.0.1:$CH_PORT/v2/health ; curl -s http://127.0.0.1:$CH_PORT/v2/mixer
             ls -l /run/chris/position.sock
  - the car backend (the car computer's own deployment, not done here):
             BACKEND_POSITION_RUNDIR=/run/chris  BACKEND_ROADSENSE_API_PORT=$CH_PORT
             unit list: roadsense.target (it carries chris-integrator.service), without roadsense-mapd.service
  - rollback: docs/INSTALL.md, section "Rollback"
EOF
if [ -n "$D" ]; then
    echo "  - scratch run:  CHRIS_ROADSENSE_RUNDIR=<dir with the camera sockets> $D/run.sh"
fi
