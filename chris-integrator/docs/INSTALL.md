# Installing chris-integrator on a host

`scripts/install.sh` installs chris-integrator (and the map box chris-mapper, when its source tree is next to
this one or `MAPPER_SRC` names it) on a Linux host with systemd, from a source tree, as root. It is POSIX `sh`
with busybox-compatible tools only, fetches nothing from the network, uses no pip, and can be run again: a second
run changes nothing the first one set up. On a host that already runs a chris-roadsense instance with its legacy
`roadsense-mapd`, it also moves that host over (section "Migration") and leaves a way back (section "Rollback").

## Requirements

| what | why | Alpine / postmarketOS | Debian / Ubuntu |
|---|---|---|---|
| Python 3.10 or newer | the daemon | `python3` | `python3` |
| numpy | the estimator core (`chris_integrator/core`), the map box (`chris_mapper/rsmap.py`, `context.py`): required | `py3-numpy` | `python3-numpy` |
| aiohttp | the consolidated HTTP / WebSocket API (`chris_integrator/io/http.py`): required for a daemon that serves consumers | `py3-aiohttp` | `python3-aiohttp` |
| systemd | the unit; not needed for a scratch install | | |

Nothing else outside the standard library is imported at run time. Evidence: a scan of every `import` in
`chris_integrator` and `chris_mapper` finds exactly `numpy`, `aiohttp` and `chris_mapper` (optional: without it
the stack runs without the map box and says so); a run of the installed daemon under `python3 -X importtime`
loads, besides the standard library, `numpy`, `aiohttp` and aiohttp's own dependencies (`multidict`, `yarl`,
`frozenlist`, `aiosignal`, `propcache`, `aiohappyeyeballs`, `attrs`, `idna`), which the distribution packages of
aiohttp bring along.

`install.sh` checks all of this, and that the daemon's whole import chain loads with the host's interpreter,
**before** it changes anything; a missing module stops it with the package to install.

### The native core (optional)

The package is pure Python and complete as such. A compiled module with the hot path's arithmetic and text
work (`docs/NATIVE.md`) makes the daemon about twice as cheap; `install.sh` uses one when it is at hand for the
host's machine type and C library and passes its self-test on the host, and says which it did:

| situation | what the script does | its line in the summary |
|---|---|---|
| `native/dist/linux-<machine>-<libc>/_chris_native.abi3.so` in the source tree (made off the host by `native/cross-build.sh`), or a module built in the tree (`native/build.sh`), or `NATIVE=/path/to/_chris_native.abi3.so` | copies it into the installed package, runs `python3 -m chris_integrator.native` (loads? gives the pure-Python results on this machine?) | `native core: IN USE (linux-aarch64-musl, sha256 ..., self-test ok)` |
| the self-test fails or the module does not load (another machine type, another C library) | removes the module again | `native core: NOT used, the compiled module ... failed here and was removed: pure Python (...)` |
| no module at hand | nothing | `native core: no compiled module for linux-... at hand (...): pure Python` |
| `NATIVE=0` | removes a module left by an earlier run | `native core: not installed (NATIVE=0): pure Python` |

The script never compiles and installs no package. On a running system the choice is visible in `GET
/v2/health` (`native`) and in the daemon's log at start (`hot path NATIVE (...)` / `hot path pure Python: ...`);
`CHRIS_NATIVE=0` in `/etc/chris/integrator.env` switches the native core off without touching files.

## Install

```sh
sh scripts/install.sh                     # as root, from the source tree (chris-mapper next to it)
sh scripts/install.sh --check             # the checks only
```

Environment (all optional):

| variable | effect | default |
|---|---|---|
| `CHRIS_API_PORT`, `CHRIS_API_HOST` | the consolidated API's port and bind address (the same keys in `integrator.env`). `API_PORT` / `API_HOST` are read when these are not set; roadsense's `scripts/install-phone.sh` names the debug face `DEBUG_API_PORT` / `DEBUG_API_HOST` and never passes its own `API_PORT` on | `8090`, `127.0.0.1` |
| `ROADSENSE_HTTP` | the camera instance's debug face, to which `/preview`, `/v2/frame.jpg`, `/v2/camera`, `/v2/governor` are forwarded (`CHRIS_ROADSENSE_HTTP`) | `http://127.0.0.1:8091` |
| `MAP_PACK` | `CHRIS_MAP_PACK` in `integrator.env` | `/var/lib/chris/maps/current.rsmap` |
| `CPUS` | `CPUAffinity=` of the unit, as a drop-in | none |
| `PYTHON` | the interpreter that is checked (the unit always runs `/usr/bin/python3`) | `python3` |
| `MAPPER_SRC` | the chris-mapper tree | `../chris-mapper` |
| `STOP_LEGACY=0` | leave a legacy `roadsense-mapd.service` alone | stop and disable it |

An existing `/etc/chris/integrator.env` is kept: a key that is missing is added with its default, a key given
explicitly in the environment is replaced, every other line stays.

### Files

| path | content |
|---|---|
| `/usr/local/lib/chris/chris_integrator`, `/usr/local/lib/chris/chris_mapper` | the code (replaced on every run) |
| `/etc/chris/integrator.env` | `CHRIS_API_PORT`, `CHRIS_API_HOST`, `CHRIS_ROADSENSE_HTTP` (and anything the host adds) |
| `/etc/chris/<plug-in>.json`, `mixer.json`, `recorder.json`, `mapper.json` | optional configuration, one file per plug-in (`chris_integrator/stack.py`) |
| `/var/lib/chris/` | state carried across restarts (`integrator.json`: faults, calibration; one file per plug-in that keeps state), `signs.jsonl` (ended sign tracks) |
| `/var/lib/chris/maps/current.rsmap` | the map pack (a symlink; a changed pack is loaded without a restart) |
| `/run/chris/` | `position.sock` and `feed.sock` (the vehicle input API: the same server under two names), `integrator.sock` (JSON lines of the public streams) |
| `/etc/systemd/system/chris-integrator.service` (+ `chris-integrator.service.d/50-cpus.conf` with `CPUS`) | the unit |

### Unit and ports

* `chris-integrator.service`: `Slice=roadsense.slice`, `PartOf=roadsense.target`, `[Install]
  WantedBy=roadsense.target`. `install.sh` enables it into `roadsense.target` (the link
  `/etc/systemd/system/roadsense.target.wants/chris-integrator.service`; a link under `multi-user.target.wants`
  from an earlier version is removed). This works whether or not the target itself is enabled at boot:
  `systemctl start roadsense.target` pulls the integrator in (`Wants=` through the `.wants/` link),
  `systemctl stop roadsense.target` and `restart` carry it (`PartOf=`). The unit is not started at boot by itself.
  A host that adds its own drop-ins with the same `PartOf=` / `Slice=` lines, or a `roadsense.target.d/` drop-in
  with `Wants=chris-integrator.service`, duplicates these harmlessly (list settings are merged, `Slice=` is the
  same value; `systemd-analyze verify` is clean with such drop-ins), and either mechanism alone is enough.
* Ports: the consolidated API on **8090** (loopback by default), the camera instance's debug face on **8091**.
  These are the installed values (`CHRIS_API_PORT` / `CHRIS_API_HOST` in the unit and in `integrator.env`). The
  daemon's built-in default, used only when it is started by hand without that environment
  (`python3 -m chris_integrator`), is port **8080 on all interfaces**: the port a stand-alone roadsense API always
  had, right for a development machine and a single-purpose box, not for a host whose own service owns 8080 (the
  reference car: its backend); there the unit's environment decides, never the built-in default.
* No watchdog, no start timeout a hot and throttled CPU could trip; `Restart=always` every 2 s, and
  `StartLimitIntervalSec=0` so that a restart loop (e.g. while another process still holds the port during a
  switch-over) never gives up. `MemoryMax=768M` is a backstop against a leak, not a throttle (no `MemoryHigh`):
  the daemon is about 70-75 MB resident with the map box and the recorder (measured on the reference phone).
* `systemctl stop` sends SIGTERM; the daemon then saves its carried state and exits with status 0.

### CPU placement and priority

The integrator is in the measurement path: it produces `lead` (and `lanes`, `view`) for every camera frame, about
3-4 ms of one laptop core per frame for the whole stack; on the reference phone about 25 ms of a little core per
frame with a recorder and one WebSocket consumer, 39 % of that core at 15 frames a second (docs/DESIGN.md 14.7). Choices for a host that shares
its cores with a camera pipeline:

* **Nice -2** (in the unit). systemd does not enable the cpu controller inside `roadsense.slice` unless a unit in
  it asks for a CPU weight, so the slice's services compete as one group in which nice orders them: -2 puts the
  integrator ahead of background analysers at nice 5 that share its cores (about 4.7 to 1 in CPU share under
  contention) and behind the camera pipeline's own threads at nice -5 (the frames it consumes). A
  `CPUWeight=` on this unit would switch the cpu controller on in the slice and re-weight every sibling service
  instead, so it is not used.
* **CPU affinity** is per host (`CPUS=`): on a big.LITTLE phone whose big cores carry the models (and throttle
  first under heat), the little cores are the place for this light Python process; keep it off any core the host
  reserves for itself.

What to measure on the host to confirm: `GET /v2/mixer` (per plug-in `ms_per_call` and `max_ms`, the
estimator's in particular, against the camera's frame interval), `GET /v2/health` (`governor.topics.*.hz` and
`latency_ms` of `lead` / `lanes`, `rate.degraded`), the process's CPU share and resident memory, under the
heat the host really runs at.

### A scratch prefix (no root)

```sh
DESTDIR=/tmp/chris-prefix sh scripts/install.sh       # systemctl is skipped (also: --no-systemctl)
CHRIS_ROADSENSE_RUNDIR=/run/roadsense /tmp/chris-prefix/run.sh
```

The same files land under the prefix, and `run.sh` starts the daemon with the unit's environment pointed at the
prefix's run, configuration and data directories. Unix socket paths are limited to 108 bytes: use a short prefix,
or call `run.sh` through a short symlink.

## Migration from a legacy roadsense host

When `/etc/roadsense` and `/var/lib/roadsense` exist, `install.sh` copies (never moves, never overwrites; the
originals stay for a rollback):

* `/etc/roadsense/{recorder, integrator, mixer, road_users, lead, lanes, egolane, view, roadgeom, lights, rules,
  fused, rates}.json` -> `/etc/chris/`, those that exist and are not there yet; recorder sessions keep going to
  the `dir` named in `recorder.json`;
* `/etc/roadsense/mapd.json` -> `/etc/chris/mapper.json`: the keys the map box reads (`horizon_m`, `country`, and
  any other key of `chris_mapper.box.DEFAULTS`). Not carried, and reported: `pack` (the symlink below), `hz` (the
  box matches every 0.45 s of input time), `vision_odometry`, `wheelbase_m`, `steering_ratio`, `track_m` (read by
  nothing in chris; the wheel-speed yaw rate uses a track width fixed in `chris_integrator/sources/vehicle.py`);
* `/var/lib/roadsense/integrator.json` (carried faults and calibration) and `signs.jsonl` -> `/var/lib/chris/`;
* the map pack: `/var/lib/chris/maps/current.rsmap` -> the file `/var/lib/roadsense/maps/current.rsmap` resolves
  to (when not there yet);
* `/etc/roadsense/api.env`: the debug face moves to the port of `CHRIS_ROADSENSE_HTTP` (8091) and loopback
  (unless it is already bound to all addresses); the old file is kept once as `api.env.pre-chris`;
* `roadsense-mapd.service` (it estimates a position of its own: running both would estimate it twice) is removed
  from `Wants=` of an installed `roadsense.target` (backup `roadsense.target.pre-chris`), disabled (its
  `roadsense.target.wants/` link goes) and stopped. It is **not masked**, so the way back is one command.

The camera instance's own install (chris-roadsense `scripts/install-phone.sh`) puts its debug face (api 3) in
place on 8091 and runs this script at its end.

The host's consumer, if it was written for the legacy layout, needs: the vehicle feed at
`/run/chris/position.sock` (instead of `/run/roadsense/position.sock`), the consolidated API on 8090, and
`roadsense.target` (which now carries `chris-integrator.service`) without `roadsense-mapd.service` in any unit list
it manages. `install.sh` prints this at the end; it does not change the consumer.

## Rollback

Back to the legacy layout (roadsense-mapd estimating the position, roadsense's api 2 serving the consolidated
topics on 8090). Take a backup **before** installing: the camera instance's install replaces its api code with the
debug face, which serves no consolidated topics.

```sh
# before the install
tar czf /root/pre-chris.tgz /etc/roadsense /etc/systemd/system/roadsense* /usr/local/lib/roadsense

# rollback, as root
systemctl stop roadsense.target                       # or: systemctl stop chris-integrator.service
systemctl disable chris-integrator.service            # removes it from roadsense.target.wants/
rm -f /etc/systemd/system/chris-integrator.service
rm -rf /etc/systemd/system/chris-integrator.service.d
# (also remove any drop-in the host added that wants chris-integrator.service, e.g. in roadsense.target.d/)
tar xzf /root/pre-chris.tgz -C /                      # the old api code, api.env (port 8090), roadsense.target
# without that backup: cp -p /etc/roadsense/api.env.pre-chris /etc/roadsense/api.env;
#   cp -p /etc/systemd/system/roadsense.target.pre-chris /etc/systemd/system/roadsense.target (if it exists);
#   and reinstall the old roadsense tree for its api 2 code
systemctl enable roadsense-mapd.service               # WantedBy=roadsense.target: back into the target
systemctl daemon-reload
systemctl start roadsense.target
```

Point the consumer back at `/run/roadsense/position.sock`. `/etc/chris`, `/var/lib/chris` and
`/usr/local/lib/chris` can stay (nothing reads them without the unit) or be removed.
