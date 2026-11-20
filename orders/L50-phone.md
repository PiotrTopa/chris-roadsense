# Order L50-phone: the native core of chris-integrator on the phone

Placed by lane L50, 2026-11-20. Background: `docs/lanes/L50-native.md` (section 6), `chris-integrator/docs/NATIVE.md`.
Phone: the car computer (Poco F1) on the bench, reached and handled as in its CMDB entry; root through the vault
(`dotinfra vault exec poco_user_password -- ssh HOST 'sudo -S -p "" CMD'`, password on stdin only). One lane on
the phone at a time. Nothing is installed with `apk`; nothing is compiled on the phone. Never reboot, never
touch USB / VBUS. Stop the stack the moment a thermal zone passes 95 C. Times in CET.

A replay pollutes learned state: before step 2 copy `/var/lib/chris/integrator.json`, `/var/lib/chris/signs.jsonl`,
`/var/lib/roadsense/{camera-calibration.json,signs.jsonl,vehicle-calibration.json}`, `/etc/roadsense/calib.json`
with their checksums and the list of `/var/lib/roadsense/rec`, and restore them at the end (L46 section 7,
`~user/l46/restore_l46.sh` is the pattern).

## 1. Trees and the self-test (as user; nothing is changed on the system)

```sh
# laptop, in the roadsense worktree of v2-l50 (the artefact is made by: sh chris-integrator/native/cross-build.sh)
git archive fae9098 chris-integrator chris-mapper | ssh HOST 'mkdir -p ~/l50/base && tar x -C ~/l50/base'
{ git ls-files -z chris-integrator chris-mapper; find chris-integrator/native/dist -type f -print0; } \
    | tar czf - --null -T - | ssh HOST 'mkdir -p ~/l50/final && tar xzf - -C ~/l50/final'
# phone
cd ~/l50/final/chris-integrator
cp native/dist/linux-aarch64-musl/_chris_native.abi3.so chris_integrator/native/
sha256sum chris_integrator/native/_chris_native.abi3.so; cat native/dist/linux-aarch64-musl/BUILD.txt
python3 -m chris_integrator.native            # must print: self-test ok ...; exit status 0
DESTDIR=/home/user/l50/inst sh scripts/install.sh --no-systemctl | grep 'native core'     # IN USE (...), under busybox ash
```

**Stop here if the self-test fails**: report its lines; the phone stays as it is.

## 2. The per-message path off line, quiet little core (stack off through the backend: `roadsense_mode off`)

Inputs: L46's, on the phone: `~user/l46/bench.cap.gz` (the phone's own input, 3,035 road frames) and
`~user/l46/car113201-slice` (a drive with vehicle feed; with the map pack). As user, `sh ~/l50/quiet.sh`:

```sh
#!/bin/sh
cd ~/l50 || exit 1
rm -f quiet.done
B=final/chris-integrator/tools/cost_bench.py
P=/home/user/roadsense-maps/poland-20261114-f2.rsmap
CAP=$HOME/l46/bench.cap.gz; CAR=$HOME/l46/car113201-slice
C=${CORE:-1}
cost() { CHRIS_NATIVE=$2 PYTHONHASHSEED=0 taskset -c $C python3 $B run $4 $5 --tree $3 --json $1.json > $1.out 2>&1; }
dump() { CHRIS_NATIVE=$2 PYTHONHASHSEED=0 taskset -c $C python3 $B run $4 $5 --tree $3 --dump $1.jsonl.gz > $1.out 2>&1; }
for pass in 1 2; do
    cost bench-base-$pass 0 base "$CAP" "";            cost bench-py-$pass 0 final "$CAP" "";            cost bench-nat-$pass 1 final "$CAP" ""
    cost car-base-$pass 0 base "$CAR" "--pack $P";     cost car-py-$pass 0 final "$CAR" "--pack $P";     cost car-nat-$pass 1 final "$CAR" "--pack $P"
done
taskset -c $C python3 final/chris-integrator/tools/native_bench.py "$CAP" --json unit.json > unit.out 2>&1
for k in bench car; do
    [ $k = bench ] && { I=$CAP; X=""; } || { I=$CAR; X="--pack $P"; }
    dump d-$k-base 0 base "$I" "$X"; dump d-$k-py 0 final "$I" "$X"
    dump d-$k-exact json,round,interp,lanes final "$I" "$X"; dump d-$k-nat 1 final "$I" "$X"
done
for k in bench car; do
    echo "== $k: base against this branch on the reference"; python3 $B diff d-$k-base.jsonl.gz d-$k-py.jsonl.gz --show 0 | tail -n 1
    echo "== $k: reference against the exact native parts"; python3 $B diff d-$k-py.jsonl.gz d-$k-exact.jsonl.gz --show 0 | tail -n 1
    echo "== $k: reference against all native"; python3 $B diff d-$k-py.jsonl.gz d-$k-nat.jsonl.gz --show 3 | grep -v 'identical *[0-9]*$'
done > diff.out 2>&1
echo done > quiet.done
```

About 50 minutes on one little core. Record: `ms_per_road_frame` and the stages of the twelve cost runs,
`unit.out`, `diff.out`, the hottest zone.

**Pass**: (a) base against the reference: IDENTICAL on both inputs; (b) reference against the exact parts:
IDENTICAL on both; (c) reference against all native: every stream identical but `ego_state`, whose differing
messages differ only by the sign of a zero (list any other field with its two values: `--show`); (d) native
cheaper than the reference by at least a quarter on both inputs. **If (a) or (b) fails, or (c) shows a
difference beyond rounding, stop: the phone stays on pure Python**, and the differing lines go into the report.

## 3. Deployment on the bench (authorised by the owner) and the live cost

```sh
# root, stack off
R=/home/user/rollback-20261120-l50; mkdir -p $R; chmod 700 $R
tar czf $R/chris-lib-before.tar.gz -C / usr/local/lib/chris/chris_integrator usr/local/lib/chris/.deployed-commit-chris_integrator
LIB=/usr/local/lib/chris; SRC=/home/user/l50/final/chris-integrator/chris_integrator
cp -r $SRC $LIB/chris_integrator.l50new && find $LIB/chris_integrator.l50new -name __pycache__ -type d -prune -exec rm -rf {} \;
chown -R root:root $LIB/chris_integrator.l50new; chmod -R go-w $LIB/chris_integrator.l50new
mv $LIB/chris_integrator $R/chris_integrator-replaced; mv $LIB/chris_integrator.l50new $LIB/chris_integrator
(cd $LIB && python3 -m chris_integrator.native) || rm -f $LIB/chris_integrator/native/_chris_native*.so   # self-test where it runs
echo <head of v2-l50> > $LIB/.deployed-commit-chris_integrator
# RESTORE.txt in $R: stack off; mv $LIB/chris_integrator away; tar xzf $R/chris-lib-before.tar.gz -C /; mode auto.
# Softer undo, no files: CHRIS_NATIVE=0 in /etc/chris/integrator.env and a restart of the unit.
```

(The install script is not run on the live system: it would re-create the unit's `WantedBy` link that the
car's provisioning removes, L46 section 7. Its native step is the three lines above.)

Live, replay playlist (`sh scripts/use-replay.sh --enable ...` as L46), four runs alternating
`CHRIS_NATIVE=0` (a line in `/etc/chris/integrator.env`) and native (the line removed), each started fresh:
stack on, 200 s of warm-up, `python3 tools/cost_live.py --seconds 240 --json ...`, `GET /v2/health` (its
`native` entry must say what was meant), stack off, cool-down to the same start temperature. Record per run:
share of one core, road frames a second, CPU ms per road frame (user + system), latency of `road` / `lead` /
`lanes`, resident memory, peak temperature, plug-in errors. Check live once with the native core: the backend
receives its topics at the camera's rate, a WebSocket client gets what it asks for, `integrator.sock` greets,
the recorder's session reads back, the daemon stops cleanly and saves its state.

**Pass**: native at least a fifth below pure Python in CPU per road frame, 0 plug-in errors, latency not
worse. Otherwise leave `CHRIS_NATIVE=0` in `integrator.env` and say so.

## 4. As found, and the record

Replay off, DVR source enabled, learned state restored by checksum, the bench's recorder sessions moved out
of the recordings folder into the rollback folder, backend mode `auto`. `~/.infra/servers/poco.md` updated in
place (package, module checksum, `native` in health, cost, rollback, the `CHRIS_NATIVE` switch), `dotinfra
lint` clean. Results into `docs/lanes/L50-native.md` sections 0, 4 and 6.
