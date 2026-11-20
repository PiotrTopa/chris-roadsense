# L43: integration of the mixer (L39), the estimator and fault detection (L41) and road users (L42) into one deployable state

Lane L43, 2026-11-20, laptop only. The phone was read (python, packages, units, configs) and nothing was written
there; the car repo and the car-side lane's worktree (L44) were read and run, never edited. Branch `v2-l43`
from `v2-l39` (3c5c419) with `v2-l42` (c734669) and `v2-l41` (8ceb0c0) merged; two sub-lanes on their own
branches, merged back: L43-V (vehicle input store, `v2-l43-veh`) and L43-D (install, units, tools, public split,
`v2-l43-dep`). Not pushed, not merged into `main`.

Public descriptions: `chris-integrator/docs/DESIGN.md` 5b, `docs/FAULTS.md` ("Which detectors gate"),
`docs/INSTALL.md`, `docs/VEHICLE_INPUT.md`, `docs/TOPICS.md`; `chris-mapper/docs/MAP.md`, `PACK.md`.

## 0. Result in one page

**Done and working (laptop):**

* **One tree.** The three branches merge without textual conflict and with one structural one (1.1). Tests:
  chris-integrator 331 (with the car repo present; 3 skip without it), chris-mapper 22, roadsense api 22, mapd 81; the car repo's two end-to-end
  test files against this tree: 4 passed (with L44's snapshot of this branch: 2 passed, 1 xfailed, 1 failed).
* **Detectors gate or report, per detector, in the configuration**, with the default of each taken from the
  healthy drives (section 2). On the two healthy vehicle-fed drives nothing is set aside and neither speed, yaw
  rate, lead nor position is published invalid by any detector; on the glass (the car repo's own replay)
  `CAMERA UNSURE` on 104128 goes from 14 episodes / 6.1 % of the driving to **0 / 0 %**, on 113201 0; the
  incident drives stay at 98.9 % and 100 % of the driving with one episode each.
* **Why the detectors fired on healthy data** was found for each and fixed at the cause in eight cases (2.2);
  three detectors cannot be made quiet on this car's sensors and report (2.1).
* **Position**: the "worse than L38" under receiver faults is a scoring artefact of the campaign tool (3.1);
  the regression under map faults was real and has two causes, one removed and one bounded (3.2); a lost
  position is no longer published as unknown and no longer stays unknown while parked (3.3); the car's own
  synthetic drive becomes usable in 4 s and its map state switches 3 times in 27 s instead of 15 (3.4).
* **The vehicle input store** is ported; the car's feeder, its error handling and its synthetic drive with
  `reset` work against chris-integrator as against mapd (section 4).
* **Install**: one command on the phone produces the agreed interface; tested in a scratch prefix under busybox
  sh and dash, twice in a row, and the whole stack was run from that prefix (sections 5, 6).

**Not good, stated first:**

* **The incident drive 151146 goes unknown 0.6 s later than in L38** (3.7 s after the car starts moving against
  3.1 s; 98.8 % of the driving faulted against 99.0 %). The car pulls out in a tight turn, and the camera's speed
  error in turns is now modelled (the cause of most false camera alarms). 153636: identical (0.1 s, 100 %).
* **Position on the poor-reception drive through the map box is still worse than legacy mapd**
  (held-out fixes 63 m against 24 m, map available 83.5 % against 93.3 %; L39 measured 52 m and 88 %): not
  fixed (6.5). On the three other drives the map is available 97-98.5 % of the driving (L39: 95-96.6 %).
* **With the default modes the integrator isolates less than L41 showed**: faults that only a reporting
  detector sees (a 10 % speed scale error, a gyro bias, a frozen speed) are found and named, and the source
  stays in the estimate. Section 6.4 has what that costs, in simulation and on the recording.
* **The stated position uncertainty is optimistic on the car** (normalised 5-7 where 2 is expected): two runs
  of the same recording that re-anchor on different fixes differ by more than three of their stated sigmas.
  This is also why the position columns of the fault table on the recording are noisy (3.1).
* **Margins are thin where they matter most**: `self_check`, the detector that carries the incident, reached
  0.65 of its level on a healthy drive. Everything rests on two vehicle-fed healthy drives of one day.
* Nothing ran on the phone.

## 1. Merge notes

1.1 `git merge v2-l42`, then `git merge v2-l41`: no textual conflict. One structural conflict: L41's sign-row
source reads the message type `signs`; on the bus that name is the sign-context box's output, downstream of the
estimator: a loop the mixer refuses. The estimator plug-in now maps a source's message type to its stream
(`STREAM_OF = {"signs": "signs_raw"}`, a constructor argument; the replay pipeline keeps the recorded `signs`):
the new source is a declared plug-in input, nothing bypasses the contract. `gnss_track` reads `gnss`, already a
channel stream. L41's renamed row classes (`mount_yaw`, `mount_pitch`) and L39's class slot in
`core/objects.py` / `core/lead.py` merged without overlap; L42's `core/classvote.py` and the road-users box
were untouched by L41.

1.2 Semantic changes forced by the merge, each visible in a test:

* `tests/test_mapper_box.py`: the simulated receiver is tagged `receiver` when it lags (the tag `synthetic` now
  means "stamped at its time of validity", 3.4); the position limit is 9 m instead of 6 m with a check that the
  error is inside the stated uncertainty (L41's along-track covariance: 7.8 m against 5.5 m with L38's
  estimator, stated hacc about 9 m).
* `tests/test_faults.py`, `tests/test_radar.py`: tests of isolation by small differences and by vote run with
  every detector gating (`GATE_ALL`); new tests cover the defaults (a report names the fault and sets nothing
  aside; an invalid mode is refused).

1.3 All three suites and L39's tooling run on the merged tree: `tools/mixer_equiv.py` (`run`, `diff`, new
`stats`), `tools/mapper_eval.py`, `tools/stack_e2e.py` (new `--installed`), L41's `tools/fault_campaign.py`
(`--cfg`, `--gate-all`, scoring fixed, 3.1) and `tools/integrator_eval.py` (`--cfg`), L42's
`tools/road_users_eval.py` (import checked, not re-run: it needs frames). New: `tools/detector_audit.py`.

## 4. What was ported (sub-lane L43-V)

`chris_integrator/io/vehicle_input.py` is the port of mapd's vehicle store (same owner, MIT): every check with
mapd's error reasons (stamps, ranges, enums, quality, description, sensors, objects), per-signal value, age,
quality and source tag, the car's description (feeder > `vehicle_input.json` > nominal), the sensor table, the
feeder registry. The socket (`position.sock` = `feed.sock`) behaves line for line as mapd's: a hello for a
hello (`api: 1, minor: 1, server: "chris-integrator"`, `accepts`, `signals`), nothing for an accepted line,
`{"type": "error", "line": n, "reason": ...}` for a rejected one, the connection kept. `POST /v2/vehicle`
answers `{"api": 1, "accepted": n, "rejected": [{"index", "reason"}]}`. The `vehicle` topic has every key of
mapd's message (motion, controls, body, `signals{age_s, quality, source}`, sensors, description, calibration,
feeders) plus `input{counts, last_error, resets, last_reset}`, twice a second, also without a feeder.

`reset` goes through the mixer's contract (a declared stream `position_reset`, read by the estimator and the
map box). It drops: the local frame, position, heading and altitude with their covariance, the receiver's and
the map's states and memories, the position monitors; the map box's match (state `no_source` at once). It keeps:
speed, yaw rate, accelerations, every calibration state of the car's sensors and cameras (checked to the bit),
faults, the object layer, the odometer, the map pack. The same happens by itself when the receiver's tag
changes (`Integrator.new_position_source`).

Quality tags: only `good` / `uncalibrated` motion signals reach the estimator; an `unverified` gyro is left out
and the yaw rate falls back to the rear wheels, then the steering angle (mapd's relay semantics); the tags are
carried in `vehicle.signals`. The gyro now passes as read (not zeroed at standstill, not bias-corrected): the
estimator learns its bias.

Deliberately not carried (each explicit null or documented in `docs/VEHICLE_INPUT.md`): mapd's own calibration
estimators and counters (the estimator owns calibration; reported from `ego_state.calibration` in mapd's shape
with `ref: "estimator"`), `use_calibration`, `gnss_lag_s`, vision odometry.

Carried over from the removed `test_map_e2e`: raw socket lines and error replies on both socket names, HTTP
accepted / rejected, the signal table, sensors and objects, `/ws` vehicle + objects, `health.vehicle`, reset
over HTTP, roadsense's reference clients (`tests/test_vehicle_e2e.py`, 5 tests; `tests/test_vehicle_input.py`,
9 tests with 36 error cases). Against the real consumer (`tests/test_car_feeder.py`, 3 tests, skipped without
`CHRIS_TEST_CAR_REPO`; the car code runs in its own interpreter): hello, acceptance, error replies, a
synthetic session with the reset at its start and at its end (map `no_source` within 0.1 s, the car's own
`SyntheticTaint` cleared, `ego_state.position` invalid): passed on the car repo's `main` (407016b) and on L44's
worktree. L44's reset case, an xfail against their snapshot, passes.

Open in this part: a synthetic drive's motion and fixes still feed the car's calibration states (mapd kept
calibration per source tag); an `unverified` `controls.brake` still reaches `lead.ego.brake`; `position_reset`
is neither recorded nor replayed; an out-of-process map box does not declare `position_reset` (it goes to
`unknown`, not `no_source`); `health.daemons.mapd.connected` is true whenever the map box exists.

## 5. Install and rollback on the phone (sub-lane L43-D)

Read from the phone today: Alpine / postmarketOS, Python 3.14.8, `py3-aiohttp` 3.14.3, `py3-numpy` 2.5.4 (apk);
`/etc/roadsense/api.env` has `ROADSENSE_API_PORT=8090`; `roadsense.target` wants mapd; the pack is a symlink in
`/var/lib/roadsense/maps`; `roadsense.slice` is restricted to cpus 0-2,4-7 by the host's drop-in (cpu 3 is the
car backend's) and has no cpu controller inside (`cgroup.subtree_control = memory pids`).

**Required packages** (all present on the phone; nothing is fetched): `python3` >= 3.10, `py3-numpy` (required:
the estimator core and the map pack reader import it), `py3-aiohttp`; the debug face also `py3-pillow`. A static
import scan of both packages finds only `numpy`, `aiohttp` and the optional `chris_mapper`; a run under
`PYTHONPROFILEIMPORTTIME` loads nothing else. The script checks all of this before it changes anything.

**Install** (as root, from the source tree on the phone, stack stopped; re-runnable):

```sh
tar czf /root/pre-chris.tgz /etc/roadsense /etc/systemd/system/roadsense* /usr/local/lib/roadsense   # for the rollback
systemctl stop roadsense.target
cd ~/roadsense && sh scripts/install-phone.sh          # builds and installs roadsense, then runs chris-integrator/scripts/install.sh CPUS=0-2
systemctl start roadsense.target                       # or the car backend's mode switch
curl -s 127.0.0.1:8090/v2/health; curl -s 127.0.0.1:8090/v2/mixer; ls -l /run/chris/position.sock
systemctl show -p PartOf,Slice,CPUAffinity,Nice chris-integrator
```

`chris-integrator/scripts/install.sh` alone (the integrator and the map box only) takes `API_PORT`, `API_HOST`,
`ROADSENSE_HTTP`, `MAP_PACK`, `CPUS`, `STOP_LEGACY=0`, `DESTDIR`, `--no-systemctl`, `--check`.

What it produces:

| | |
|---|---|
| code | `/usr/local/lib/chris/{chris_integrator, chris_mapper}` |
| unit | `/etc/systemd/system/chris-integrator.service`: `PartOf=roadsense.target`, `WantedBy=roadsense.target` (a `.wants` link: started and stopped with the target, which itself stays disabled), `Slice=roadsense.slice`, `Nice=-2`, `Restart=always`, `RestartSec=2`, `StartLimitIntervalSec=0`, `MemoryMax=768M`, no watchdog, no start timeout; drop-in `50-cpus.conf` `CPUAffinity=0-2` |
| config | `/etc/chris/integrator.env`: `CHRIS_API_PORT=8090`, `CHRIS_API_HOST=127.0.0.1`, `CHRIS_ROADSENSE_HTTP=http://127.0.0.1:8091` (an existing file is kept, missing keys added, a key given on the command line replaced) |
| migrated (copied if absent, never overwritten, originals kept) | `/etc/roadsense/{recorder, integrator, mixer, road_users, lead, lanes, egolane, view, roadgeom, lights, rules, fused, rates}.json` -> `/etc/chris/`; `mapd.json` -> `/etc/chris/mapper.json` (the keys the box knows: `horizon_m`, `country`; `wheelbase_m`, `steering_ratio`, `track_m`, `vision_odometry`, `hz` are printed as unused); `/var/lib/roadsense/{integrator.json, signs.jsonl}` -> `/var/lib/chris/`; `/var/lib/chris/maps/current.rsmap` -> what the roadsense pack link resolves to; the recorder's `dir` as named in the moved `recorder.json` |
| debug face | `/etc/roadsense/api.env`: port 8091, host 127.0.0.1 if none was set (backup `api.env.pre-chris`) |
| legacy mapd | removed from the installed target's `Wants=` (backup `roadsense.target.pre-chris`), disabled and stopped, not masked |
| run time | `/run/chris/{position.sock, feed.sock, integrator.sock}`, `/var/lib/chris/integrator.json` (faults and calibration, saved on a stop: the daemon now handles SIGTERM; before, a `systemctl stop` saved nothing) |

**Affinity: cpus 0-2, nice -2.** Little cores: the process is light Python (laptop: 3.2-3.9 ms per road frame,
70 MB); cpu 3 belongs to the car backend; the big cores run the models and are the ones that throttle at
83-90 C. It is in the measurement path (it produces `lead` per camera frame), so it must not wait behind the
slow analysers on the same cores: the slice has no cpu controller inside, so nice is what orders its services;
-2 against envd / laned at +5 is about 4.7 to 1 under contention, and it stays behind the model's frame
preparation (-5), which produces what it consumes. A `CPUWeight=` would switch the controller on for the whole
slice and re-weight every sibling. Nothing kills it when the cores throttle (no watchdog); the memory bound is
a backstop ten times its size. **To confirm on the phone** (not measured): `/v2/mixer` estimator `ms_per_call`
and `max_ms` against the 50-75 ms frame interval, `/v2/health` `governor.topics.lead / lanes` rate and latency,
the process's CPU share at heat with envd and laned running. If the little cores do not hold the frame rate:
`CPUS=0-2,4-7`.

**The car side must set** (L44's job, printed by the script): `BACKEND_POSITION_RUNDIR=/run/chris`,
`BACKEND_ROADSENSE_API_PORT=8090`, `chris-integrator.service` in its unit list, `roadsense-mapd.service` out.

**Rollback**:

```sh
systemctl stop roadsense.target
systemctl disable chris-integrator.service
rm -f /etc/systemd/system/chris-integrator.service; rm -rf /etc/systemd/system/chris-integrator.service.d
# and any host drop-in that wants chris-integrator (L44: roadsense.target.d/50-prius-chris.conf)
tar xzf /root/pre-chris.tgz -C /          # the old api code, api.env with 8090, the old roadsense.target
# without the backup: cp api.env.pre-chris and roadsense.target.pre-chris back and reinstall the old roadsense tree
systemctl unmask roadsense-mapd.service 2>/dev/null; systemctl enable roadsense-mapd.service
systemctl daemon-reload && systemctl start roadsense.target
```
`/etc/chris`, `/var/lib/chris`, `/usr/local/lib/chris` may stay; nothing reads them.

Checked against L44's assumptions: `API_PORT` / `API_HOST` / `ROADSENSE_HTTP` work; their drop-ins repeating
`PartOf` and `Slice` are harmless (`systemd-analyze verify` clean with them); `/v2/frame.jpg`, `/v1/frame.jpg`
and `/preview` are forwarded to the debug face on 8091 (tested from the installed prefix with the fake daemons
and the real debug face); `position.sock` in the run dir; `health.daemons.camerad / modeld`, `view.integrity`,
`lead.valid`, `lanes.valid` exist. One difference to know: the daemon's built-in default port is still 8080;
8090 comes from the unit and `integrator.env`.

**The two tools L39 left broken** are restored, not retired: `tools/budget/api_profile.py` (runs the stack with
the api subscriber and the recorder; 3.62 % of one laptop core on 113201) and `tools/vision_eval/l32/chain_api.py`
(the stack by default, `--api DIR` for a pre-L39 tree; old and new give identical `lead` distance and validity
on 9771 of 9771 frames of 113201 with `--ego`). The tools L39 listed as not run were run once each; only
`speedlimit_replay` needed a fix (it read plain JSONL only).

Not verified: everything on the phone (the scripts as root under the phone's busybox, real `systemctl`, CPU and
latency at heat); `shellcheck` was not available.

## 2. The detectors on the real car: what gates, what reports

Mechanism: `chris-integrator/docs/DESIGN.md` 5b. Configuration: `integrator.json` `{"detectors": {name: "gate" |
"report"}}`; defaults in `core/integrator.py` `DETECTORS`. A reporting detector publishes
`ego_state.diagnostics[]` entries of kind `reported` (detector, who would be set aside, by which rule, the
numbers) and is listed under `sources.<g>.reported`; it sets nothing aside, has no vote, and nobody waits for it.
`ego_state.checks[].peak` is the largest value a test's statistic reached as a fraction of its firing level.

### 2.1 The table (`tools/detector_audit.py`; 104128 / 113201 healthy with vehicle, receiver, map: 8843 / 1725 moving frames; 065629 camera only; 135622, 145155 parked; 151146, 153636 incident)

| detector | default | healthy 104128 / 113201: peak / level; share of the driving with a verdict; spells | incident 151146 / 153636 | basis for the default |
|---|---|---|---|---|
| `speed.share` | **gate** | 0.13 / 0.09; 0; 0 | 2.0 / 2.0 (fires), held throughout | margin 7 |
| `self_check` | **gate** | 0.65 / 0.64 (camera-only 0.62, parked 0); 0; 0 | 2.9 / 3.4 (fires); 94 % / 90 % | margin 1.5: thin; L38's detector, unchanged |
| `yaw_rate.share` | **gate** | 0.67 / 0.12; 0; 0 | 0.48 / 0.20 | margin 1.5 on the gyro-wheels pair; by vote only; can take one source's yaw rows out, never an output |
| `signal.noise`, `signal.invalid` | **gate** | 0.18 / 0.18; 0; 0 | 0.40 / 0.37 | margin 5 |
| `signal.stuck` | report | never approached | never | a constant true value is indistinguishable (found on the car's synthetic drive: vehicle and wheels were faulted for good at constant speed, 3.4) |
| `speed.cusum` | report | 1.95 / 0.99; 0.5 % / 0; 2 / 0 | 10 / 9.6 | fires on healthy data (two tight turns); before the turn term 8 spells |
| `yaw_rate.cusum` | report | 3.9 / -; 16 % / 0; 15 / 0 | 2.4 / - | the wheels' slow offset (2.2) is above its limit |
| `speed.track` | report | 4.4 / 0.87; 68 % / 0; 15 / 0 | 28 / 6.9 | the receiver's track is a 1-3 m/s source on this receiver (2.2) |
| `innovation` | gate | fires: 20 / 11 spells (position rows 14 % / 11 % of the driving, map cross-track 9 % / 28 %, mount yaw 12 % / 4 %) | fires | sets a row class aside while it lasts; no output goes invalid by it |
| `position.lost` | gate | 9 / 3 times | 2 / 1 | does not invalidate any more (3.3) |
| `map.conflict` | gate | fires: 21 % / 16 %; 5 / 1 spells (peak 2.3 / 2.8) | 29 % / 0 | holds only the map's rows; it is what bounds a wrong map (3.2); position with it is not worse (held-out fixes 15.3 / 49.9 m with, 17.7 / 53.1 m without, 104128) |

**Outputs on the healthy drives with these defaults**: no source and no quantity set aside, nothing disputed;
speed, yaw rate, lead never invalid by a detector (one frame at the very start of a session, before the first
vehicle sample: `no_measurement`); position invalid 0 % / 1.8 % of the moving frames (113201: before the first
anchor and three dead-reckoning run-outs with 3-5 satellites); heading unknown 1.7 % / 8.4 % (its own std above
45 degrees after a position loss). Source health `suspect`: camera 11 % / 4 % of the moving frames (the mount's
yaw row set aside by `innovation`; L41: 20.5 % / 3.9 %, L38: 0.3 % / 2.4 %), vehicle 0 / 1 %.

**False camera faults**: L41 0.8 % / 0.7 % of the moving frames -> **0 / 0**. Wheel-difference yaw set aside:
up to 15 % -> **0** (reported 8.7 % of the driving on 104128 by `yaw_rate.cusum`). The receiver's track
"faulted" 27 % -> never faulted, reported.

**The incident drives** (first camera fault after the car starts moving; share of the moving frames faulted):

| | L38 | this branch |
|---|---|---|
| 151146 | 3.1 s; 99.0 % | 3.7 s; 98.8 % |
| 153636 | 0.1 s; 100 % | 0.1 s; 100 % |
| 161325 (camera only, dusk) | not measured here | faulted by `self_check` for 79 % of the session |

The 0.6 s on 151146 is the turn term of 2.2: the car pulls out through a turn of 0.2-0.5 rad/s, where a healthy
camera errs by 1-3 m/s. Without the term the detection is at L38's 3.1 s and the healthy drives have eight
false verdicts. Not recovered.

### 2.2 Why the detectors fired on healthy data

Fixed at the cause (each measured on the recordings; scripts were one-off, the effects are reproduced by the
audit above):

1. **Wheel-speed sensors have a floor.** A wheel reads exactly 0 below about 0.35 m/s (smallest non-zero value
   0.34-0.35 m/s on four drives) and the four come alive one at a time: 100-320 samples per drive of a moving
   car with a wheel at zero, up to a speed signal of 1.1 m/s. One rear wheel at 0.4 m/s beside one at zero is a
   wheel-difference "yaw rate" of 0.3-0.6 rad/s; the mean of the wheels reads low. This was every false
   wheel-yaw verdict at low speed and while reversing (the sign handling in reverse is right: ratio to the gyro
   -0.98 before the flip). Now: no wheel measurement unless every wheel reads at least 0.6 m/s.
2. **The rear wheels' difference has a slow offset** against gyro and camera, which agree with each other to
   0.001-0.003 rad/s on straights: medians over two minutes between -0.022 and +0.003 rad/s on four drives, not
   proportional to speed. Its sigma now carries 0.012 rad/s in quadrature. The offset itself is not modelled as
   a state: with the gyro out, a heading from the wheels alone drifts by up to 1 degree a second and the stated
   uncertainty does not say so (not done).
3. **The camera's speed is worse in turns**: robust sigma against the vehicle 0.36-0.50 m/s below 0.2 rad/s,
   0.5-0.8 at 0.2-0.3, 0.8-0.9 at 0.3-0.6 rad/s; 5-14 % of the frames above 0.2 rad/s beyond 4 straight-road
   sigmas against 0.1-0.7 % below; errors of 1-4 m/s for 2-3 s, either sign, in every false speed verdict
   looked at (seven on 104128, one on 113201). The characterised sigma has a turn term, 2 m/s per rad/s in
   quadrature, taken from the estimate's yaw rate; the camera's own uncertainty check keeps the straight level.
4. **A CUSUM verdict could not lapse while the car stood**: one stayed for 18 minutes of parking, another 6.
   A CUSUM-only verdict lapses after 30 s without a sample; one the share test confirms becomes the share
   test's (without this the incident drives lost 5-12 % of their faulted time once the CUSUM reported).
5. **A re-anchoring published one frame with speed, yaw rate and lead unknown** (the filter's buffer is
   restarted and with it the record of when each quantity was last measured): 3 frames on 104128, each a 5 s
   `CAMERA UNSURE` on the glass.
6. **Verdict texts were wrong**: a CUSUM verdict said "100 % of N samples beyond 4 sigma".

Not fixable on this car, demoted with the numbers:

* **The receiver's track as third opinion** (`speed.track`): against the odometer over 3, 6 and 10 fixes its
  speed error is 1.4-1.6, 1.0-1.4 and 0.9-1.2 m/s robust and 3.2, 2.9, 2.5 m/s rms on 104128, beyond 10 % in
  45-58 % of the chords, whatever delay is assumed (0-3 s) and whether path or chord is used; on 113201 (3-5
  satellites) the track is 20-25 % short; 46 % of the fixes of 104128 repeat the previous position. Its error
  is correlated over tens of seconds, so a mean test accumulates it. With it reporting, a wrong vehicle speed
  is decided by rank as in L38 (the camera is blamed), not by vote as in L41's table.
* **`yaw_rate.cusum`**: item 2. **`speed.cusum`**: two spells remain in tight turns after item 3.
* **`signal.stuck`**: a constant true value.

## 3. Position

### 3.1 "Worse than L38 under receiver faults" is a scoring artefact

`fault_campaign.py real` compared a faulted run's position with the unfaulted run's only while both had the
same local frame id. A run that re-anchored once more than its reference was never scored again. L38
re-anchors under a receiver jump, drift or delay, so its "0 s" were positions 30-50 m off that were not looked
at. Scored whenever both runs share the origin (seconds valid and wrong by more than 15 m and 3 sigma, 104128
slice, fault 55 s; measured with a one-off script on the trees themselves, before the changes of 3.2 / 3.3):

| fault | L38: as reported, as it is | L41: as reported, as it is |
|---|---|---|
| receiver: jump 30 m | 0, **48** | 6, 10 |
| receiver: slow drift 1 m/s | 0, **24** | 31, 31 |
| receiver: noise burst 15 m | 0, 17 | 0, 0 |
| receiver: outliers 20 % of 60 m | 0, 23 | 2, 2 |
| receiver: frozen fix / drop-out | 26, 26 | 0, **38** |
| receiver: lat / lon swapped | 0, 32 | 0, 44 |
| receiver: fixes 2 s older | 0, **44** | 42, 42 |
| map: parallel road 25 m | 1, 1 | 40, 40 |
| map: bearing turned 90 deg | 20, 20 | 36, 36 |

So under receiver faults L41 was better on three of these rows, equal on one, worse on three; neither survives
a persistent receiver fault (one absolute source). The tool is fixed (and `--tree` on recordings, which since L39 silently
ran this tree's integrator: found when "L38" gave this branch's numbers to the digit).

The comparison has a weakness that the fix exposes: two runs of the same recording that re-anchor on different
fixes differ by more than 15 m and 3 stated sigmas, because the stated position uncertainty is optimistic on
this receiver (held-out fixes: normalised 5-7 where 2 is expected). Position columns on the recording are
therefore noisy also for faults that have nothing to do with position (6.4).

### 3.2 Map faults: a real regression, two causes

* L41 believed the receiver at 8 x its stated accuracy after every re-anchoring; a wrong map was then alone
  with the position. Off by default (`reanchor_widen`): turned map bearing 36 s -> 0 s on the slice (one-off
  script); position invalid 7-14 s -> 1-2 s per receiver fault.
* A matcher on a parallel road pulls the position over, the receiver's adaptive noise then explains the
  receiver away, and nothing objects. L38 was pulled 7-17 m of the 25 m and stayed under the scoring threshold
  ("1 s"). New, raw against raw: the signed distance of each new receiver fix from the matched road's line,
  median over 8 fixes; beyond 12 m the map's rows are held back (`map.conflict`, the map source then reads
  `suspect` with the offset). Parallel road: 41 s -> 12 s. It also holds the map back on healthy driving 21 % /
  16 % of the time (this receiver's median sits 12-27 m beside the road then); position is not worse for it
  (2.1). A parallel road nearer than 12 m is not seen.

### 3.3 A lost position

Before: after 8 s of absolute positions set aside the position was declared unknown until the next usable fix
(9 and 4 times on the healthy drives, about 1 s each, then 2 s of `dead_reckoning_ran_out` from the 8 x
widening), and a position lost just before parking stayed unknown for as long as the car stood (104128:
position valid 46.5 % of all frames). Now (`reanchor: soft`): it stays published under a new frame id with its
uncertainty opened by 40 m, and the next usable fix places position and heading as a first anchoring does.
Position valid: 104128 93.5 % of all frames, 100 % of the moving ones. A first variant that let the filter pull
the old state in was measured and dropped (151146: held-out fixes 31 m p50 / 339 m p90 against 9 / 29 m).

### 3.4 The car's synthetic drive (L44's items 2 and 3)

Three causes, found by running the car repo's own feeder and tests against this tree:

* **A feed without a gyro had no yaw sensor**: the wheel difference was used only beside a gyro, and L39's
  converter had dropped mapd's derived yaw rate. The car's synthetic motion (and its default CAN set) has wheel
  speeds and steering only: the position left the road at the first bend. Fixed twice over: the store derives
  the yaw rate as mapd did (L43-V), and the wheels source fuses its difference whenever the feed's own yaw rate
  is not that very difference.
* **`signal.stuck` faulted vehicle and wheels for good** when the synthetic speed settled at a constant 5 m/s;
  the speed then came from the camera of the fake daemons. Now reports.
* **The receiver's delay prior (2.0 +- 1.5 s, this car's head unit) applied to a source stamped at its time of
  validity**; at constant speed nothing corrects it and it drifted to 2.7-5.9 s. A receiver that names itself
  may have its own prior (`gnss_delay`); `synthetic`, the API's tag, is 0 +- 0.05 s; a change of the tag
  forgets the position and anchors anew. (L39's 5.3, "a receiver without delay", is the same thing: such a
  receiver needs a tag and a line of configuration.)
* Flicker: `position.mode` was judged fresh against the fix's time of validity, 2-3 s before its arrival, on a
  3 s window: it flipped every second. Freshness now counts from arrival (1.6 s: one missed beat of a 1 Hz
  receiver), which also gives `dead_reckoning` within 1.6 s of a lost fix as legacy mapd did.

Result on the car backend's synthetic drive (their `test_position_mapd_e2e.py`, drop-out 15-20 s): position
valid 3.2 s after the first fix, map `good` from 4.2 s, `dead_reckoning` through the drop-out, `good` again at
20.2 s: **3 switches in 27 s** (15 before), no `unknown`. Their four tests pass (`test_alerts_mapd_e2e.py`
included).

## 6. Whole-stack proof and regression tables

Commands in section 9. Laptop; the tables were produced while other jobs of this lane ran (12 cores, load 6-18),
so absolute costs scatter; where a cost matters the quieter run is named.

### 6.1 Whole stack, the daemon as installed

`tools/stack_e2e.py --installed PREFIX --install`: `chris-integrator/scripts/install.sh` into a scratch prefix,
the daemon started by the prefix's `run.sh` (port 8090, run dir with `position.sock`), a recording replayed on
real sockets as a roadsense instance, the car feed through `position.sock`, the car repo's unmodified
`RoadsenseSceneFeed` on the WebSocket. 104128 from 900 s for 240 s, 2938 road frames:

| | L39 (in tree, its report) | this branch, as installed |
|---|---|---|
| scenes built by the car's feed | 2138 = 8.9 / s | 2139-2147 = 8.9 / s |
| scenes with a lead / a lane line / a speed limit | 23.8 / 36.9 / 90.1 % | 25.9 / 38.8 / 90.0 % |
| map layer present / usable (`map.ok`) / road picture (`geo`) | - / 68.8 / 21.3 % | 100 / 52.6-57.4 / 6.3-7.6 % |
| display channel in use, lead source tracked | 99.8, 99.9 % | 99.8, 99.9 % |
| `road` line in -> its `view` line out, p50 / p90 / p99 | 7.1 / 16.9 / 60.8 ms | 4.3 / 8.5 / 26.4 ms (quiet machine); 8.8 / 20.4 / 62.6 ms (loaded) |
| daemon CPU, memory | 10.6 % of a core, 70 MB | 7.2 % (6.1 ms per road frame, everything included; loaded machine 14.2 %), 70 MB |
| health | | five daemons and `mapd` connected; vehicle, wheels, map `ok`; 12 plug-ins, 0 errors; clean stop on SIGTERM with the carried state saved |

**`map.ok` and the road picture are lower than L39's on this slice** (slow traffic, the car standing for much
of it): 53-57 % against 69 %, 6-8 % against 21 %; the sub-lane's run of the tree before the estimator changes
of this lane had 57 % and 20.5 %. Over the whole drive with the map box the road picture has its map part in
28.0 % of the frames against 25.1 % for L39 and branches in 26.8 % against 23.8 %
(`mixer_equiv.py run --mapper box`, both trees), so it is not a general loss; what happens on this slice was
not diagnosed.

The car repo's own end-to-end tests against this tree (`PRIUS_ROADSENSE_REPO`): `test_position_mapd_e2e.py`
and `test_alerts_mapd_e2e.py`, 4 passed.

### 6.2 Frame by frame against v2-l39 (`tools/mixer_equiv.py run` in both trees, `stats`)

Same trackers, another estimator underneath: the messages are not identical (lead 4-26 %, lanes 7-69 %
identical), the decisions are.

| | 104128 (28 288 frames) | 113201 (9746) | 065629 camera only (3143) |
|---|---|---|---|
| `lead.valid` differs | 3 frames, all invalid -> valid | 1 frame, invalid -> valid | 0 |
| `lead.present` differs | 8 frames (none -> track / coast) | 9 frames (none -> track) | 0 |
| `lanes.valid` differs | 0 | 0 | 0 |
| lead distance: frames differing; p50 / p95 / max of the difference | 5.0 %; 0.01 / 0.16 / 5.6 m | 3.8 %; 0.01 / 0.52 / 11.3 m | 76 %; 0.03 / 0.13 / 12.1 m |
| relative speed | 1.9 %; 0.01 / 0.07 / 1.0 m/s | 2.5 %; 0.02 / 1.44 / 11.6 m/s | 76 %; 0.03 / 0.16 / 4.7 m/s |
| ego speed in the lead message | 20 %; 0.001 / 0.004 / 0.94 m/s | 16 %; 0.001 / 0.13 / 9.1 m/s | 89 %; 0.32 / 1.26 / 2.2 m/s |
| ego yaw rate in `lanes` | 37 %; 0.0014 / 0.021 rad/s | 27 %; 0.0012 / 0.022 | 89 %; 0.0021 / 0.026 |

Where from: (a) the invalid -> valid frames are the frames after a re-anchoring that were published unknown
(2.2 item 5); (b) the yaw rate now includes the wheel difference with another weight and the gyro's scale as a
consider state (L41), 1-2 mrad/s; (c) with a vehicle feed the speed is the vehicle's and differs in the third
decimal, except in the feed's silences: on 113201, 22 of the 37 frames with a speed difference above 0.5 m/s
lie inside a silence of the vehicle feed (the camera carries the speed there, with L41's noise model and scale
handling), on 104128 2 of 2; the other 15 were not looked at one by one; (d) the camera-only drive differs in
most frames by L41's camera-speed model (0.3 m/s median: the bias L41 removed). The large lead differences
(5-12 m, one to a few frames per drive) follow the ego speed in those frames; not examined individually.
`view` differs through the same quantities and through the map alignment (0.02-0.1 degree median).

### 6.3 Incident drives, false alarms, the glass

Section 2.1. On the glass (the car repo's `tools/mfd_chain_replay.py --glass`, its own scene builder and
director with the 5 s hold):

| drive | `CAMERA UNSURE` while driving: L44's snapshot (424ef8e) | this branch | on / off switches |
|---|---|---|---|
| 104128 healthy | 6.1 %, 14 episodes | **0 %, 0 episodes** | 14 -> 0 |
| 113201 healthy | not measured there | 0 %, 0 episodes | 0 |
| 151146 incident | 99.3 % | 98.9 % | 1 |
| 153636 incident | not measured there | 100 % | 1 |

Raw-fallback scenes: 0 on all four.

### 6.4 The fault table

Simulation (town, seed 1, 59 faults; `chris-integrator/docs/FAULTS.md` has every row for the all-gating case):

| | L38 | this branch, default modes | every detector gating |
|---|---|---|---|
| injected source taken out, or named at fault level | 12 | 20 | 38 |
| no output confidently wrong | 29 | 30 | 33 |
| time an output was valid and confidently wrong | 1657 s | 1355 s | 737 s |
| ... speed / yaw rate / heading / position / lead | 530 / 154 / 179 / 691 / 104 | 424 / 56 / 119 / 655 / 102 | 131 / 34 / 95 / 376 / 101 |
| alarms on 12 healthy simulated drives (38 min) | (L41's report: receiver suspect 0.7 %, map 5.5 %) | one finding for 0.3 s | the same |

Accuracy in simulation is L41's, unchanged (town position rms 1.18 / 2.29 / 1.60 m, motorway 4.28 / 6.77 /
9.59 m over three seeds; NEES 0.36-1.23 and 0.74-3.96). With a radar (8 radar faults, all gating): 3 isolated,
4 with no output wrong, 26 s (L41: 3, 4, 29 s).

On the recording (104128, seconds 260-560, fault 355-410 s; the reference is each tree's own unfaulted run;
positions scored across re-anchorings, 3.1; L38 re-run with the same tool):

| 59 faults on the recording | L38 | default modes | every detector gating |
|---|---|---|---|
| injected source taken out, or named at fault level | 18 | 25 | 31 |
| no output confidently wrong | 18 | 22 | 22 |
| time an output was valid and confidently wrong | 1728 s | 1493 s | 953 s |
| ... speed / yaw rate / heading / position / lead | 432 / 124 / 228 / 881 / 62 | 405 / 39 / 234 / 754 / 61 | 140 / 11 / 99 / 642 / 62 |

Rows where the default modes are worse than L38 on the recording (seconds valid and wrong): vehicle speed
signal x 0.7 with the wheels right: heading 59 (L38 0); camera pitch step 1.7 degrees: position 43, heading 8
(0); vehicle speed sign: position 64, heading 17 (53, 9); receiver frozen / drop-out: position 46 / 38 (25 /
25); lat / lon swapped 49 (32); slow drift 30 (23); map bearing turned 39 (19); parallel road 11 (1, under the
threshold while pulled 7-17 m). Better: receiver jump 12 (48), noise burst 0 (17), outliers 0 (23), lying
accuracy 4 (21), the gyro faults (yaw rate 124 -> 39 s in all), camera scale 0.3 and camera drop-out (position
14 / 10 -> 0). The position rows carry the noise described in 3.1: a camera pitch step does not move the
position, a different re-anchoring fix does. The speed rows are L38's with the default modes, as intended: the
vote that fixes them is the receiver's track.

With every detector gating on the healthy 104128: the vehicle is falsely set aside by vote for 0.26 % of the
driving, the wheels' yaw rate for 7 %, the receiver's track for 23 %, the speed is `disputed` for 2.7 %; the
camera is never faulted any more. That is what the defaults keep off the car.

### 6.5 Position against legacy mapd

Held-out fixes (every second one, car moving), distance to the estimate, p50 / p90:

| drive | estimator, recorded map as input | estimator without the map | mapd's position filter | L41 (with map) |
|---|---|---|---|---|
| 104128 | 14.7 / 70.7 m | 10.9 / 48.9 | 11.2 / 63.6 | 15.9 / 59.6 |
| 113201 poor reception | 27.5 / 56.3 | 29.5 / 94.4 | 23.1 / 91.5 | 22.0 / 62.6 |
| 151146 | 9.4 / 28.9 | 6.0 / 20.1 | 5.4 / 23.8 | 7.2 / 19.0 |
| 153636 | 9.2 / 17.7 | 7.6 / 13.9 | 4.3 / 22.8 | 9.2 / 17.2 |

Position valid: 93.5 / 76.2 / 85.0 / 91.2 % of all frames (the rest is before the first anchor). Stated
uncertainty: normalised distance 5.4 / 7.5 / 5.8 / 2.5 where 2 is expected: optimistic on three of four.

Through the map box (`tools/mapper_eval.py real`; the box's own match fed back), car moving:

| drive | system | map available | distance to the matched road p50 / p90 | held-out fixes against the position 2 s before their stamp, p50 / p90 |
|---|---|---|---|---|
| 104128 | legacy / box | 98.2 % / 97.8 % (L39: 95.2) | 8.8 / 24.8, 3.2 / 20.7 m | 28.8 / 57.4, 19.5 / 71.3 m |
| 113201 | legacy / box | 93.3 % / **83.5 %** (L39: 88.0) | 12.4 / 22.1, 2.2 / 34.8 | 24.1 / 69.1, **63.0 / 136.1** (L39: 51.9 / 119.6) |
| 151146 | legacy / box | 98.6 % / 98.5 % (L39: 95.4) | 10.4 / 25.6, 2.3 / 21.6 | 15.9 / 35.4, 14.1 / 27.3 |
| 153636 | legacy / box | 100 % / 97.0 % (L39: 96.6) | 5.0 / 20.8, 2.5 / 6.4 | 11.5 / 28.5, 11.0 / 18.1 |

Three drives: availability now within 0.1-3 points of legacy (the lost position is no longer unknown), position
level with or better than legacy at the receiver's age. **113201 is worse than legacy and worse than L39
measured**: with 3-5 satellites and long gaps the receiver-delay state walks to 4.6 s in the loop with the box
(1.2-2.0 s on the other drives; 2.0 s prior), i.e. the map's rows and sparse fixes are reconciled by the
delay, and the position runs ahead along the road. Without the box in the loop the same drive is at 27.5 m.
Not fixed; the lead is the delay state's freedom under sparse fixes (a bound on its rate, or no delay learning
while the map's rows are the majority).

Simulated drives on a real map pack with truth (2 s receiver lag): on route 100 / 100 / 99.5 / 100 % of the
steps (L39: 100 / 100 / 95.2 / 100), position error with a fix 1.1-3.9 m p50 against legacy's 25-53 m; the
180 m dead-reckoning excursion of L39's 5.2 is gone (17 m, 2 steps).

### 6.6 Camera-only speed honesty (`integrator_eval.py heldout`, the vehicle held out)

| drive | n | error / stated std (rms) | inside 2 sigma | filtered rms / raw camera rms |
|---|---|---|---|---|
| 104128 | 9970 | 0.73 (L41 0.75, L38 1.56) | 0.984 (0.980, 0.796) | 0.588 / 0.651 m/s |
| 113201 | 2174 | 0.86 (0.90, 1.39) | 0.978 (0.972, 0.868) | 0.600 / 0.629 |

L41's result holds on the merged tree. Yaw rate: error / std 1.20 and 1.64, as in L41 (stated std too small).

### 6.7 Cost per road frame (laptop, one process, the whole consolidated side with the recorded map)

| | v2-l39 | this branch |
|---|---|---|
| 104128 | 3.64 ms | 3.86-3.94 ms |
| 113201 | 3.09 ms | 3.20-3.23 ms |
| 065629 (camera only) | - | 2.46 ms |
| the estimator alone, per call | 1.01 ms (L39's report) | 0.98 ms (104128) |

Measured alternately on a loaded machine: 4-8 % more than L39, inside the scatter of such a machine but in the
same direction on both drives (one more input per sign message, the detectors' bookkeeping, the road-users
box). The installed daemon at real pace: 6.1 ms per road frame including sockets, HTTP and the map box.
Phone: not measured.

## 7. The two public directories

`chris-integrator/` and `chris-mapper/` were each extracted alone (`git archive` of the directory) and checked
(L43-D): chris-integrator 293 tests pass and 19 skip (16 of `test_api.py` need roadsense's fake daemons and
debug face, 3 need chris-mapper beside it); chris-mapper 21 pass, 1 skips (it needs roadsense's pack builder);
no test needs a recording (`tests/data` holds two CSVs of numbers from CC-BY rides). Both build a wheel and an
sdist offline with their package data; the installed integrator starts without the mapper ("no map box", 11
plug-ins) and with it (12). Both carry an MIT `LICENSE`; no AGPL text, no code from roadsense-signs (mentions of
`signd` are protocol references), no dependency on it. No private address, hostname, local path or recording
name in either (the example source tag `prius-can` in two moved tables and a place name were replaced; the
references to a `docs/INTEGRATOR.md` that exists only as a pointer in roadsense now say `docs/DESIGN.md`).

The topic field tables moved out of roadsense's `docs/ARCHITECTURE.md` (806 lines, verbatim but for links and
ports): consolidated topics -> `chris-integrator/docs/TOPICS.md`; the `map` message -> `chris-mapper/docs/MAP.md`;
the pack -> `chris-mapper/docs/PACK.md`. ARCHITECTURE.md keeps the raw per-camera topics and pointers.

What the split still needs:

* `tests/test_api.py` starts roadsense's `tools/fake_daemons.py` and `api/`: vendor a small fake-daemon set and
  a debug-face stub, or keep it as an integration test of the roadsense repository.
* The three `test_mapper_box` tests and chris-mapper's builder test need both repositories: a CI job that
  checks out both. There are no CI files.
* Cross-references that leave the directory: roadsense `docs/VEHICLE_API.md` (now also described in
  `docs/VEHICLE_INPUT.md`), `docs/ARCHITECTURE.md` (raw topics), roadsense `tools/*` and `docs/lanes/*` named in
  docstrings and tables as the place where a number was measured.
* `chris_integrator/replay.py` `load_pos` reads one host's trip-log format (with its time zone); it belongs in
  that host's tools. `tests/test_car_feeder.py` names the reference car's backend repository and runs only
  when pointed at it.
* The README's "Reference deployment" paragraph names the car and the phone (the roadsense README does too):
  the owner's call. The MIT copyright line carries the owner's name, as in roadsense.
* `TOPICS.md` has no row for the `road_user` objects of `view` (described in `trackers/roadusers_view.py`).
* The mixer's default API port in code is 8080; packaging installs 8090.

## 8. Not done, or not verified

* Nothing ran on the phone: the install scripts as root under its busybox, the unit under real `systemctl`,
  CPU on the little cores, latency at 83-90 C, the sockets against the real daemons. The phone estimate stays
  L38's (4-6 x the laptop per operation): 13-23 ms per road frame, unmeasured.
* The incident drive 151146: 0.6 s slower than L38 (2.1).
* Position through the map box on 113201 (6.5); the stated position uncertainty (3.1); a persistent receiver
  fault is detected and not survived (one absolute source), as before.
* With the default modes a wrong vehicle speed is isolated by rank (the camera is blamed), as in L38; L41's
  vote needs `speed.track` in gate, which this receiver does not support (2.2).
* The wheel-difference offset is in the sigma, not a state (2.2 item 2).
* A gentle re-aiming of the camera at standstill is still not detected (L41 section 8); the sign rows now flow
  through the mixer (`signs_raw`) and were not re-evaluated here.
* Calibration states are fed by a synthetic drive (section 4); `unverified` controls pass.
* The `suspect` share of the camera on healthy drives (11 % / 4 %) comes from the mount's yaw row
  (`innovation`); not looked into (L41: "the yaw residual is not constant on the real car").
* `map.ok` and the road picture on the end-to-end slice (6.1).
* `tools/road_users_eval.py` and L42's measurements were not re-run on the merged tree (frames needed); L42's
  tests pass.
* Thin data everywhere: every default of section 2 and every position number rests on two vehicle-fed healthy
  drives (38 and 15 minutes) of one car on one day; the incident numbers on two drives; the fault table on the
  recording on one 5-minute slice.

## 9. Reproducing

```sh
cd ~/projects/wt/roadsense-l43; D=<datasets>/roadsense-rec; P=$D/trip-pos/pos_20261119.txt; O=/tmp/l43; mkdir -p $O
L38=~/projects/wt/roadsense-l38; L39=~/projects/wt/roadsense-l39; CAR=~/projects/cyberpank-prius-gen2-computer
PP=$(python3 -c "import pytest,os;print(os.path.dirname(os.path.dirname(pytest.__file__)))"); PY=~/projects/roadsense/.venv-models/bin/python
# tests
(cd chris-integrator && CHRIS_TEST_CAR_REPO=$CAR PYTHONPATH=$PP $PY -m pytest -q -p no:cacheprovider tests)
(cd chris-mapper && PYTHONPATH=$PP $PY -m pytest -q -p no:cacheprovider tests); PYTHONPATH=$PP $PY -m pytest -q -p no:cacheprovider api/tests
(cd mapd && PYTHONPATH=$PP $PY -m pytest -q -p no:cacheprovider tests)
# the car repo's end-to-end tests against this tree (read-only there)
(cd ~/projects/wt/prius-l44 && PYTHONDONTWRITEBYTECODE=1 PRIUS_ROADSENSE_REPO=$PWD/../roadsense-l43 $CAR/.venv/bin/python -m pytest -p no:cacheprovider -q -s tests/test_alerts_mapd_e2e.py tests/test_position_mapd_e2e.py)
# everything below inside: systemd-run --user --collect -p MemoryMax=24G --wait -- bash -c "..."
export PYTHONHASHSEED=0 PYTHON=$PY
python3 tools/detector_audit.py $D/20261119-{104128,113201,065629,135622,145155,151146,153636,161325} --pos $P > $O/audit.txt          # 2.1
python3 tools/detector_audit.py $D/20261119-{104128,113201} --pos $P --gate-all > $O/audit_gateall.txt                               # 6.4, last paragraph
for s in 104128 113201 065629; do                                                                                                     # 6.2, 6.7
  python3 tools/mixer_equiv.py run $D/20261119-$s --pos $P --out $O/${s}_new.jsonl.gz
  (cd $L39 && python3 tools/mixer_equiv.py run $D/20261119-$s --pos $P --out $O/${s}_l39.jsonl.gz)
  python3 tools/mixer_equiv.py stats $O/${s}_l39.jsonl.gz $O/${s}_new.jsonl.gz --fields 40; done
python3 tools/integrator_eval.py real $D/20261119-{104128,113201,151146,153636} --pos $P                                              # 6.5
python3 tools/integrator_eval.py position $D/20261119-{104128,113201,151146,153636} --pos $P
python3 tools/integrator_eval.py heldout $D/20261119-{104128,113201}                                                                  # 6.6
python3 tools/mapper_eval.py real $D/20261119-{104128,113201,151146,153636} --pos $P; python3 tools/mapper_eval.py synth --lag 2.0
python3 tools/fault_campaign.py sim --out $O/sim_default.json; python3 tools/fault_campaign.py sim --gate-all --out $O/sim_gate.json   # 6.4
python3 tools/fault_campaign.py sim --tree $L38 --out $O/sim_l38.json; python3 tools/fault_campaign.py show $O/sim_l38.json $O/sim_gate.json
python3 tools/fault_campaign.py healthy
for m in "" "--gate-all" "--tree $L38"; do python3 tools/fault_campaign.py real $D/20261119-104128 --pos $P --from 260 --to 560 $m --out $O/real_${m// /_}.json; done
# whole stack as installed (6.1); the prefix must be a short path (socket names)
systemd-run --user --collect -q -p MemoryMax=8G --wait --pipe -E CHRIS_TEST_CAR_REPO=$CAR -- $PY $PWD/tools/stack_e2e.py $D/20261119-104128 --pos $P --start 900 --seconds 240 --installed /tmp/chris-e2e --install --pack ~/roadsense-maps/packs/poland-20261114-f2.rsmap
# the glass (6.3): the car repo's replay, its frontend interpreter with numpy on the path
NP=~/projects/roadsense/.venv-models/lib/python3.14/site-packages
(cd ~/projects/wt/prius-l44 && PYTHONDONTWRITEBYTECODE=1 PYTHONPATH=$NP $CAR/.venv-fe/bin/python tools/mfd_chain_replay.py $D/20261119-104128 --roadsense ~/projects/wt/roadsense-l43 --glass)
```

The incident onset times of 2.1 and the tables of 3.1 / 3.2 came from one-off scripts (first frame with the
camera `fault` after the first frame with the vehicle above 3 m/s; the campaign's faults applied to the slice
with the position compared across frame ids); `detector_audit.py --episodes` lists the same spells, and the
fixed `fault_campaign.py real` reproduces the position scoring.

## 10. Branch

`v2-l43`; head: see the lane's hand-back. Sub-lane branches `v2-l43-veh` (db4d63c) and `v2-l43-dep` (d6b6f2e)
are merged in. Not pushed, not merged into `main`.
