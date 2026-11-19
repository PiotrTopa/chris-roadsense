# L38: chris-integrator, the all-data integrator as its own component

Lane L38, 2026-11-19, laptop only (phone and car read-only; nothing deployed). Branch `v2-l38` from `v2-l37`
(d0be5ce). The component: `chris-integrator/` (package `chris_integrator`, MIT, publishable as it stands; its
README and `docs/DESIGN.md` are the public description). Builds on the audit (L35) and replaces the
api-internal integrator of L37.

Three rulings of the owner arrived during the lane and are folded in: the integrator is the **top component**
and roadsense one of its sources; the stack is named **chris** (`chris-integrator`, `chris_integrator`,
`CHRIS_*`); the component is to be a **public MIT repository**.

## 0. Result in one page

**Built and working** (tests: 27 in `chris-integrator/tests`, 255 in `api/tests`, 94 in `mapd/tests`):

* One component with a hard boundary: no import from roadsense; inputs are messages, outputs are three
  messages (`ego_state`, `tracks`, `lead`). Library, and daemon (`python3 -m chris_integrator`) with the car's
  direct feed (`feed.sock`, vehicle input API). The api's trackers read only its outputs.
* One ego estimator with the full state: position, heading, speed, acceleration, yaw rate, altitude, grade,
  speed scale, and 15-19 calibration states declared by the sources (gyro bias and scale, accelerometer
  offsets, camera delay / scale / coloured error / **mount angles**, receiver delay and correlated error, map
  lane offset, per radar mount yaw and delay). Late and out-of-order data by insert-and-re-run; fixed-lag
  smoothing; Student-t updates; consider states; three layers of fault detection with isolation by vote.
* An object layer: one multi-target track list, per-sensor models, assignment, existence, merging, ghosts, the
  lead by the path. The lead tracker's measured rules (L14, L27, L29, L32) moved into it.
* A front radar, a second radar and a second camera added as sources **without touching the core** in their
  commits; a simulator with truth; an injector of synthetic sensors into real recordings.

**Measured, in short** (section 3):

* Lead and lanes on the three healthy drives against L37, frame by frame: lead present in the same frames
  but 43 / 9 / 0 of 28287 / 9745 / 3143 (0.15 % / 0.09 % / 0 %), distance within 0.24-0.60 m (p99); lane
  presence pattern identical in 100 % of the frames; `view` identical.
* The incident: camera faulted in 99.0 % and 100 % of the moving frames (as L37), its lead `valid: false` in
  99.4 % / 100 %, nothing drawn by the car's own scene builder while faulted.
* Simulation, town: speed 0.05 m/s rms, yaw rate 0.0035 rad/s, heading 0.3-0.4 degrees, position 1-5 m, with
  NEES at or below its expectation except the position on one seed of three; every calibration state found
  within its stated uncertainty (speed scale 0.0126-0.0155 against 0.015; camera mount to 0.0001 rad).
* **The camera's mounting is measured on the car to 0.001 rad**: on both healthy drives the estimator says the
  camera sits at pitch 0.054-0.055, yaw -0.058 to -0.061 against the calibration in use (0, 0), i.e. it
  suggests the very calibration that was applied before the incident drives (0.0561, -0.0621). Finding 3.6.
* With a radar (simulation): lead distance error 0.13 m rms against 4.0 m for the camera with an 8 % range
  error; a knocked camera and a wrong vehicle speed are both isolated by vote.

**Not working, stated plainly** (section 5):

* **Position on a motorway in simulation**: the receiver's delay and the along-track position are not told
  apart at constant speed; the filter becomes overconfident (17-22 m rms, NEES 12-23, heading 2.3 degrees with
  NEES 17-22). Speed, yaw rate, grade and the lead are unaffected.
* **Position on the car** is usable on three drives (held-out fixes 7.6-15.7 m median from the estimate, stated
  uncertainty optimistic by 1.2-2.8x in variance) and poor on the bad-reception drive (62 m median, valid in
  34 % of the frames).
* **A wrong vehicle speed injected into a real drive is not isolated in time**: by vote only after 88 s of a
  90 s fault, the camera blamed first (by rank). In simulation the same fault is isolated in 12 s.
* Camera-only speed on the car: the stated std is too small by 1.4-1.6x; the filter is not better than the raw
  value.
* With an injected radar on a real drive the lead's track ids churn (112 against 39).

**Not done** (section 4): the trackers other than the lead, the HTTP / WebSocket API and the map matcher were
not physically moved into `chris-integrator/` (they read only its outputs; the move is specified); mapd still
runs its own position filter; phone not measured.

## 1. Design in one page

Full text: `chris-integrator/docs/DESIGN.md`.

**Architecture** (owner's ruling). roadsense (chris-roadsense) analyses one camera's video and publishes its
outputs; the integrator takes roadsense instance(s), the car's motion and signals, GNSS, radars directly,
owns fusion, object layer, map side, the trackers and decisions that combine sources or need ego motion, and
the consolidated API. The rule for the line: a module stays in roadsense when it needs one camera's pixels or
one camera's model output and nothing else. The table of every module of `api/roadsense_api/` and `mapd/` is in
DESIGN.md section 9; the decisions asked for: lane-line tracker, rules / speed limit, ego-light selection,
view builder, recorder -> integrator; governor, `/preview` overlay, raw topic debug -> roadsense.

**State.** One EKF: `pe, pn, psi, v, a, w, al, z, th, sv` plus source-declared calibration states. `v` is in
the scale of the car's speed signal (known to cm/s, what consumers want); ground distance is `v / (1 + sv)`.

**Measurements** are scalar rows with `h(x) -> (prediction, sparse Jacobian)` from the source's model, in two
noise classes (exact / robust). Time delays are states: a row is inserted at `stamp - tau` and its Jacobian
carries the state's rate.

**Time.** Time-ordered buffer (10 s), late rows inserted and the filter re-run from there (exact for
out-of-sequence data: test `test_out_of_sequence_rows_give_the_in_order_result`, equal to 1e-9); output at
the frame time by an RTS pass over what has arrived since; clock restarts are epochs (calibration kept, faults
carried).

**Fault detection and isolation.** Raw pair monitors (L37), self monitors (L37), innovation consistency per
row class; a class with no partner is set aside while its innovations are far off; isolation by rank or by
vote, with the vote allowed to form.

**Objects.** Tracks `[x, y, vx, vy, ax]` in the car frame with the ego motion as input; sensors are models;
global nearest neighbour; Bayes existence; covariance intersection when merging; the lead is the nearest
confirmed track on the path (the road model's plan, else the ego arc).

**What was compared and rejected**: UKF, factor-graph smoother, one-step OOSM updates, plain gating, noise
inflation instead of coloured-noise states, IMM for the ego state, JPDA, MHT (DESIGN.md section 11, with the
reasons; openpilot's `locationd` is the production precedent for buffer-and-rewind).

**What the build taught** (DESIGN.md section 2): a scale state must not learn from a regressor that is
mostly its own noise (the gyro scale walked to -0.4 on a straight road); states a row cannot tell apart must
be consider states (the camera's scale without a speed reference); the speed is better kept in the wheel
signal's scale with the ground scale as a state than the other way round.

## 2. What is built, per layer

| layer | files | state |
|---|---|---|
| time-ordered EKF | `core/ekf.py` | built, unit-tested (out-of-sequence exactness, smoothing, robust rows and anchor rule, consider states) |
| ego model, calibration states | `core/ego.py` | built |
| registry, monitors, isolation, outputs | `core/registry.py`, `monitors.py`, `integrator.py`, `output.py` | built; `monitors.py` is L37's with the NIS monitor added |
| object layer, lead | `core/objects.py`, `core/lead.py` | built |
| sources of the stack | `sources/vehicle.py`, `vision.py`, `gnss.py`, `mapmatch.py`, `camera_lead.py` | built |
| radar (any number, any mount) | `sources/radar.py` | built; simulation and injection only |
| further camera's object list | `sources/camera_objects.py` | built; simulation and injection only |
| simulator, injector | `sim/world.py`, `sensors.py`, `radar.py`, `camera2.py`, `inject.py` | built |
| daemon, direct feed | `daemon.py`, `feed.py`, `fastjson.py`, `systemd/`, `pyproject.toml` | built, socket test; not run on the phone |
| api on the integrator | `api/roadsense_api/integrator/__init__.py` (a shim), `lead.py` (stand-alone face), `state.py` (in-process or daemon mode), `server.py` (`/v2/tracks`), `recorder.py` | built |
| tools | `tools/integrator_eval.py`, `tools/replay_lib.py`, `chris-integrator/tools/hop_cost.py` | built |

**Extensibility in the commit history.**

```
41c0b1a  a second camera's object list as a source      sources/camera_objects.py, sim/camera2.py, a test
0eda7b4  a second radar (corner mount)                  two test files; one constant in sources/radar.py
7bb2df6  a front radar as a source                      sources/radar.py, sim/radar.py, a test; api adapter and tests
```

None of the three touches `core/`. Honest footnotes: (a) two general changes to the core were made in the
commits right before the radar (bf72bc8 isolation timing, 620ffc4 naming in the lead message) and one to the
camera-lead source (8d6bc3f: adopting a track another sensor holds); they are not radar code, but building
the radar is what showed they were needed. (b) After the three, running the radar and the second camera
*together* exposed a bug in the core's ordering check (it was kept per source, not per sensor stream, so two
object sensors sharing the `objects` type dropped each other's older scans); fixed in the core in the last
commit, with a test. So: adding each sensor needed no core change; making two object sensors coexist needed
one bug fix.

**Consumers.** Every api tracker reads `ego_state` as before (helper signatures unchanged). `lead` comes from
the object layer with all its former fields plus `exist`, `sources`, `on_path`, `path_offset_m`,
`lateral_std_m`. New topic `tracks` (`/v2/tracks`, recorded). Without a vehicle feed or a receiver the
estimate stands on the camera, says `unchecked`, has no absolute position (`reason: no_absolute_source`) and
keeps a local dead-reckoned pose. A frame the integrator does not answer (daemon mode, 80 ms) goes out as
`lead.valid: false`, reason `integrator_timeout`.

One behaviour change, by design: the lead is the track on the path whoever sees it, so a moving vehicle on
the path held by a radar alone is a lead (L14: only the camera could say which vehicle is the lead). The old
rule is the switch `objects.lead_radar_only: false`. The L14 radar tests were adapted and say so.

## 3. Measurements

Commands in section 7. Recordings: `~/datasets/roadsense-rec/20261119-*`, `road_model` restored to `vision`,
the car's receiver lines merged from the trip logs. Healthy: 104128, 113201 (vehicle feed), 065629 (camera
only, another calibration). Incident: 151146, 153636, 161325 (camera only).

### 3.1 No regression of lead and lanes (L37 against L38, frame by frame)

`integrator_replay.py products` in both trees, then `compare`:

| session | frames | lead present L37 / L38 | in one only (L37 / L38) | distance difference where both, p50 / p90 / p99 | v_rel difference p99 | tracks, longest | lane presence pattern identical | view lead, lines |
|---|---|---|---|---|---|---|---|---|
| 104128 | 28287 | 0.769 / 0.769 | 13 / 30 | 0.00 / 0.01 / 0.24 m | 0.04 m/s | 39 / 39, 241 s both | 100 % | 0.773, 0.309 both |
| 113201 | 9745 | 0.671 / 0.670 | 9 / 0 | 0.00 / 0.00 / 0.35 m | 0.07 m/s | 23 / 26, 77.2 / 75.5 s | 100 % | 0.674, 0.185 both |
| 065629 camera only | 3143 | 0.960 / 0.960 | 0 / 0 | 0.01 / 0.10 / 0.60 m | 0.49 m/s | 5 / 5, 91 s both | 100 % | |

Lead speed change per frame (jitter) p50 / p90 / p99: 0.01 / 0.13 / 0.46 m/s in both on 104128. The
differences come from the filter now having a lateral state coupled through the yaw rate and from the ego
speed's real uncertainty entering the track. Reading: matched, not beaten; three more track ids on 113201.

### 3.2 The incident

`integrator_eval.py real`, `integrator_replay.py summary` and `mfd`:

| session | moving frames with the camera faulted | lead present, vehicle >= 5 m/s | lead `valid: false` there | lanes invalid there | through the car's SceneBuilder while faulted |
|---|---|---|---|---|---|
| 151146 | 99.0 % (suspect 0.9 %) | 0.004 (one track of 0.8 s before the fault) | 0.994 | 0.994 | lead 0.000, any lane line 0.000 (9726 scenes) |
| 153636 | 100 % | 0.000 | 1.000 | 1.000 | |
| 161325 camera only | 79.2 % of all frames (self monitor) | 0.000 | 0.792 of all | 0.792 | |

No false alarm: camera `ok` in 99.7 % (suspect 0.3 %) and 96.6 % (suspect 2.4 %, unchecked 1.0 %) of the
moving frames of the healthy drives; never faulted. The camera's calibration state is `unknown` throughout
the incident (the mount rows are not used while the camera is faulted).

### 3.3 Ego state against simulated truth

`integrator_eval.py sim` (town 240 s, motorway 180 s; every sensor with a bias or scale to find; after the
first 30 s). NEES is per scalar (expected 1), the position's has 2 degrees of freedom (expected 2):

| scenario, seed | speed rms, NEES, within 2 sigma | yaw rate rms, NEES | heading rms, NEES | position rms, NEES | grade NEES |
|---|---|---|---|---|---|
| town 1 | 0.046 m/s, 0.17, 1.00 | 0.0035 rad/s, 0.28 | 0.32 deg, 0.32 | 1.02 m, 0.30 | 0.26 |
| town 2 | 0.048, 0.18, 1.00 | 0.0032, 0.28 | 0.28, 0.22 | 5.02 m, **8.60** | 0.28 |
| town 3 | 0.056, 0.17, 1.00 | 0.0034, 0.29 | 0.38, 0.41 | 2.43 m, 2.53 | 0.23 |
| motorway 1 | 0.124, 0.18, 1.00 | 0.0058, 0.72 | **2.25 deg, 16.9** | **17.0 m, 12.2** | 0.86 |
| motorway 2 | 0.134, 0.19, 1.00 | 0.0058, 0.72 | **2.27 deg, 22.0** | **22.0 m, 23.2** | 0.63 |
| town 1 with radar | 0.040, 0.26, 1.00 | 0.0043, 0.29 | 0.40, 0.36 | 1.12 m, 0.72 | 0.26 |
| town 2 with radar | 0.030, 0.22, 1.00 | 0.0039, 0.29 | 0.38, 0.25 | 4.27 m, **15.1** | 0.28 |

Reading: speed, yaw rate, heading (town) and grade are honest to conservative (NEES 0.2-0.9: the stated std is
2-5 times the variance of the error; conservative, not tuned down). The position is honest on two town seeds
and optimistic on one; **on the motorway the position and the heading are wrong and overconfident** (5.1).

Calibration states found (town, three seeds; truth in brackets): speed scale 0.0155 / 0.0126 / 0.0136 +- 0.003
(0.015), with a radar 0.0155 / 0.0154 +- 0.0004; gyro bias 0.0045 (0.004); camera mount yaw -0.0599, pitch
0.0500 +- 0.0003 (-0.06, 0.05); camera delay 0.124-0.127 +- 0.026 s (0.125); receiver delay 2.11 / 2.18 / 2.28
+- 0.10 s (2.2); radar mount yaw 0.92 / 1.03 deg (1.0). Gyro scale 0.028 +- 0.029 (0.04): within its
uncertainty, weakly observable in 240 s. Unit tests assert these within tolerance and within 4 sigma.

Other behaviours under test (`tests/test_ego.py`): vehicle feed silent for 7 s with stale samples on resume
(camera carries the speed, error below 1 m/s, NEES below 3, stale samples not believed); camera-only car;
receiver outage of 60 s (dead reckoning, uncertainty grows, NEES below 6) and a 50 m excursion (moves the
estimate by less than half); the matcher on a parallel road 25 m away for 20 s (pull below 12 m, the map rows
are set aside and reported); a clock restart; a source registered at run time.

### 3.4 Ego state on the car

`integrator_eval.py real`: first-pass innovations per row class (scaled median; 1 = stated uncertainty right),
and receiver fixes **held out** (every second fix compared with the estimate, never fused; car moving):

| session | vehicle speed / yaw rate innovations | camera speed / mount | receiver position innovations | map cross-track | held-out fixes: distance to the estimate p50 / p90 | normalised (expect 2), inside the 95 % ellipse | position valid (fused) |
|---|---|---|---|---|---|---|---|
| 104128 | 0.18 / 0.02 | 0.00 / 0.32 | 1.01 | 0.86 | 15.7 / 55.8 m | 5.5, 0.77 | 91 % (17 %) |
| 113201 | 0.20 / 0.00 | 0.00 / 0.36 | 2.50 | 2.14 | **61.9 / 213.9 m** | **73.6, 0.46** | 34 % (3 %) |
| 151146 | 0.11 / 0.02 | faulted | 0.78 | 0.38 | 7.6 / 18.7 m | 3.2, 0.83 | 80 % (36 %) |
| 153636 | | faulted | 0.57 | 0.15 | 9.2 / 16.9 m | 2.5, 0.89 | 91 % (36 %) |

(`fused` = a fix within 3 s; the car stands for more than half of every recording and fixes are not used
while standing.) The innovations far below 1 for the vehicle and camera rows say their stated noise is
conservative (the medians are dominated by standstill frames). Reading of the position: 5.2.

Vehicle held out entirely (`heldout`; the camera alone against the recorded vehicle, vehicle >= 1 m/s):

| session | n | speed error robust sigma, rms, p99 | error / stated std (rms), within 2 sigma | raw camera at its own time: sigma, rms | yaw rate error sigma, error / std |
|---|---|---|---|---|---|
| 104128 | 9970 | 0.62, 0.75, 2.12 m/s | **1.56**, 0.80 | 0.51, 0.65 | 0.0089 rad/s, **2.20** |
| 113201 | 2174 | 0.41, 0.59, 1.66 | **1.39**, 0.87 | 0.42, 0.63 | 0.0078, **2.34** |

Reading: 5.4.

### 3.5 The receiver's speed bias: settled

`integrator_eval.py gnss`:

| session | position repeated: moving / standing | distance over chords >= 100 m against the odometer (median, n) | displacement speed between fixes: best lag, ratio, sigma | speed FIELD per 60 s: best lag p10 / p50 / p90, ratio p10 / p50 / p90 |
|---|---|---|---|---|
| 104128 | 0.00 / 0.72 | 1.025 (50); >= 300 m: 1.030 (15) | 1.75 s, 1.025, 1.59 m/s | 4.3 / 6.6 / 8.8 s; 0.00 / 0.83 / 1.02 |
| 113201 | 0.00 / 0.07 | 0.70 (3) | 7.25 s, 0.75, 2.33 | 1.6 / 8.6 / 8.8; 0.27 / 0.82 / 1.07 |
| 151146 | 0.01 / 0.86 | 1.029 (25) | 2.50 s, 0.970, 1.29 | 1.5 / 3.0 / 5.9; 0.00 / 0.82 / 1.03 |
| 153636 | 0.00 / 0.83 | 1.050 (12) | 1.75 s, 1.028, 0.90 | 2.0 / 6.1 / 7.9; 0.43 / 0.97 / 1.07 |

The receiver has no scale bias: its **positions** agree with the odometer to +2-5 % over distance (three
usable drives) and move with the car at a lag of 1.75-2.5 s. Its **speed field** is another output: delayed by
1.5-9 s within one drive and low. L35 (4-10 % low at the best lag per drive) and L37 (half, at a fixed 3 s)
both measured that field. The repeated positions are standstill. Consequence in the build: positions fused
with an estimated delay, the speed field a witness that is never fused. 113201 is a poor-reception drive.

### 3.6 Finding: the camera's mounting, and what the incident drives show

The estimator's mount states (`calibration.vision`), from `pose.vy`, `pose.vz` at the *integrator's* speed:

| session | calibration in use | residual yaw, pitch (+- std) | state | suggested calibration |
|---|---|---|---|---|
| 104128 healthy | pitch 0, yaw 0 | -0.0583, +0.0554 (+- 0.0011) | `offset` | pitch 0.0554, yaw -0.0583 |
| 113201 healthy | 0, 0 | -0.0605, +0.0543 (+- 0.0007) | `offset` | pitch 0.0543, yaw -0.0605 |
| 151146, 153636 incident | pitch 0.0561, yaw -0.0621 | not estimated (camera faulted) | `unknown` | |

Facts: on the healthy drives the camera sat 3.4 degrees in yaw and 3.2 degrees in pitch off the calibration
in use, and the road model worked (speed ratio 1.00). The incident drives ran with the calibration that
removes exactly that offset; in their raw data the direction of the pose reads about 0.00 / +0.01 (aligned)
while the model's speed is 0.2-0.4 of the true one (pre-design analysis, DESIGN.md section 1). So on these
recordings the incident coincides with the *new calibration being applied*, and the direction cue does not
show an 8-degree misalignment. Whether the camera was also knocked, or the warp with the new angles is what
the model cannot digest, the recordings do not decide: **unverified**, and worth a check on the car before
anything else (run the same camera once with the calibration at 0, 0).

### 3.7 Object layer against simulated truth, and with injected sensors

Simulation (`sim --radar --cam2`, town, two seeds; the camera's range 8 % long, radar with clutter and
double-bounce ghosts, a second camera with its own detections):

| | camera only | radar + second camera, seed 1 | seed 2 |
|---|---|---|---|
| lead tracked where truth has one | 0.999-1.000 | 0.999 | 0.999 |
| lead where truth has none (share of frames) | 0.002 | 0.007 | 0.003 |
| lead distance error rms | 3.99-4.01 m | 0.13 m | 0.13 m |
| lead relative speed error rms | 0.23-0.24 m/s | 0.12 | 0.13 |
| vehicles in the ego lane: tracked, duplicate, id switches, rms x / y | - | 1.000, 0.002, 0, 0.14 / 0.08 m | 1.000, 0.003, 0, 0.13 / 0.08 m |
| vehicles in the next lanes | - | 0.956, 0.033, 11 switches, 0.37 / 0.22 m | 1.000, 0.014, 4, 0.39 / 0.23 m |
| confirmed tracks on no true object, per frame | - | 0.002 | 0.003 |
| ghost-flagged tracks per frame; a ghost as the lead | - | 0.15; never (test) | 0.14; never |

The id switches in the next lanes (4-11 over 240 s for one vehicle) happen at the hand-over between sensors'
fields of view; the ego-lane vehicle keeps its id. Sensors' own consistency on confirmed tracks (mean
normalised innovation squared per dimension, 1 = honest): radar 0.39-0.46, second camera 0.40-0.45
(conservative).

Tests (`test_radar.py`, `test_second_radar.py`, `test_second_camera.py`): fused lead below 0.5 m rms where the
camera alone is off by more than 2 m; the camera's range scale learned (1 / 1.10 +- 0.02); the radar's mount
misalignment found to 0.2 degrees, also for a corner radar at 40 degrees; one track per vehicle across two
radars' fields of view (duplicates below 6 % of frames, at the hand-over); the second camera tracks next-lane
vehicles the lead output never reports, with a range std that says how poor its range is; isolation by vote
of a knocked camera (the radar keeps the lead, nothing of the camera is used) and of a vehicle speed reading
30 % low (vehicle faulted by vote within 14 s, nobody blamed by rank meanwhile, comes back when it agrees);
a radar blackout of 30 s (same track id through it).

Injected into real drives (`integrator_eval.py inject`; the lead of the recording, smoothed non-causally, is
the pseudo-truth: it tests association and continuity, not accuracy):

| | 104128 as recorded | 104128 + radar | 113201 + two radars + second camera |
|---|---|---|---|
| lead present (vehicle >= 5 m/s) | 0.492 | 0.517 | 0.844 (as recorded 0.798) |
| tracked against the pseudo-truth | 1.000 | 0.985 | 0.990 (as recorded 0.999) |
| difference where both p50 / p90 / p99 | 0.09 / 0.41 / 1.14 m | 0.09 / 0.32 / 3.68 m | 0.07 / 0.23 / 0.78 m |
| lead present where the pseudo-truth has none | 0.0001 | 0.0148 | 0.0119 (as recorded 0.0030) |
| track ids | 39 | **112** | 41 (as recorded 26) |
| lead by source | vision+ego 0.70, vision 0.07 | radar+vision+ego 0.67, vision 0.07, radar+ego 0.02 | camera+radar+vision+ego 0.60, vision 0.03, radar+vision+ego 0.02 |
| radar mount yaw found (injected 0), speed scale | | -0.01 +- 0.04 deg, +0.0001 +- 0.0012 | front +0.01 +- 0.11, corner +0.59 +- 0.21 deg; -0.0004 +- 0.0009 |

Incident drive 151146 with an injected radar (no lead injected): camera faulted at 211.0 s as without it
(by rank; the label changes to `vote` at 446 s when the radar's pair has enough evidence), `lead.valid`
false in 0.2 % of the frames instead of 85 % (the object layer keeps running on the radar), **but a lead is
reported in 8.4 % of the frames from the radar alone, at 8.6 / 60.7 / 134.6 m (p10 / p50 / p90), where no
vehicle was injected** (5.5).

Vehicle speed x 0.7 injected into 113201 from 430 to 520 s: 5.3.

### 3.8 Cost

Laptop (one job at a time), `integrator_eval.py cost`, `chris-integrator/tools/hop_cost.py`:

| | value |
|---|---|
| integrator, 104128 (38 min, vehicle + camera + receiver + map) | 2.14 ms per road frame all inputs included = 2.6 % of one core (road 1.73 ms each, vehicle sample 0.38 ms, receiver fix 2.35 ms: its re-run) |
| + injected radar at 17 Hz, 113201 | 3.26 ms per road frame = 3.5 % of one core (a scan 0.22 ms) |
| L37's integrator, for scale | 0.35 ms per road frame |
| daemon over sockets, simulated drive at real pace | 3.1 ms per road frame, 3.7 % of one core, resident memory 42 MB |
| added latency of the process boundary (road line written -> its `lead` line read) | p50 2.3 ms, p90 2.5 ms, p99 3.0 ms |
| outputs per frame | `ego_state` 5.4 kB (a consumer's parse 97 us), `tracks` 0.6 kB (21 us), `lead` 1.0 kB (19 us) |
| partial parse of a `road` line (5.2 kB) against a full parse | 66 us against 88 us |

The partial parse saves a quarter, less than hoped (the scan for the members to cut is itself in Python).
With the integrator on top the question it answered is mostly gone: it is the one consumer that parses
`road`. `ego_state` grew to 5.4 kB (calibration and checks every frame); for the MFD a slimmer topic or a
lower rate of the two tables is the obvious next step.

Phone: **not measured**. Estimate: the api's Python there runs about 4-6 times slower per operation than this
laptop (L37's basis, itself an estimate), which gives 9-13 ms per road frame, 11-17 % of one core at 13 frames
a second without a radar and 15-25 % with one. The filter core (27 x 27 matrices in numpy, about 60 % of the
time in `predict` and the re-run) is the part to compile if that is too much.

## 4. What is not built

* **The physical move of the remaining modules.** `api/roadsense_api/{lanes, egolane, roadgeom, roadview, view,
  speedlimit, rules, lights, signctx, fusion, recorder, state, server, sockets}.py` and
  `mapd/{matcher, rsmap, context, limits}` still live in roadsense and are served by roadsense's api process.
  They depend only on messages and on the integrator's outputs, so the move is mechanical, but it was not done
  in this lane: it renames every import of 255 tests, collides with the unmerged `v2-l36` edits in
  `roadview.py`, and deserved a lane of its own after the ruling arrived mid-way. Until then the consolidated
  API is roadsense's api with the integrator inside it (or beside it in daemon mode), not yet the integrator's.
* **mapd's position filter is not absorbed.** The integrator fuses the receiver itself and takes `map.match` as
  a measurement, but that match still comes from mapd's own filter: the position is estimated twice. The
  re-seat (matcher reads the integrator's position) is designed (DESIGN.md section 9), not built.
* **A second roadsense instance** (a second camera's road model: pose, lead, lanes) as a source: the second
  camera built here is an object list. A second pose needs the instance tagged in its `road` messages.
* **Sign and light tracks in the object layer**: not built; they stay with their detectors and the view.
* **Feedback of the mount estimate to modeld**: the estimate and the suggestion are published; nothing writes
  the calibration.
* **`gnss` relay of the car's receiver when the car feeds the integrator directly**: `feed.sock` accepts it;
  not exercised with the real feeder.
* **IMM for targets, JPDA, a compiled filter core**: considered, not built (DESIGN.md section 11).

## 5. What does not work, or works worse than it should

5.1 **Motorway position (simulation).** At a steady 30 m/s on long gentle curves the receiver's delay state
drifts (2.42-2.45 s against 2.2, stated +- 0.10) and with it the along-track position (17-22 m rms); the yaw
scale states drift too. Cause as far as diagnosed: at constant speed a delay and an along-track offset are
the same thing to a position fix, and the EKF's linearisation gives it information it does not have. Town
driving, with its speed changes, does not show it. Candidates: fix the delay at speed and learn it only under
acceleration; an observability-constrained or invariant EKF; a sliding-window smoother for the position
states. Not attempted.

5.2 **Position on the car.** On 104128, 151146 and 153636 the estimate is 7.6-15.7 m (median) from held-out
fixes whose own scatter is 10-15 m (L20), with a stated uncertainty that is optimistic (normalised 2.5-5.5
against 2). On 113201 (3-5 satellites, 4 usable chords) it is 62 m and the position is declared unknown in two
thirds of the frames: bounded by saying "unknown", not solved. The receiver is not allowed to teach the speed
and yaw scales (`gnss_absolute: false`), because when it was, the scales walked off on 113201. The integrator
was not compared against mapd's own filter on the same fixes: whether it is better or worse than what the car
has today is **not established**.

5.3 **A wrong vehicle speed on a real drive.** Speed x 0.7 for 90 s injected into 113201 with an injected
radar: the camera is blamed first, by rank, 41 s into the fault; the vehicle is isolated by vote only at
87.7 s; the speed is wrong by 3.4 m/s (median) for practically the whole window (0.3 % of the frames within
1 m/s). In simulation the same fault is isolated in 12-14 s. The difference: on this drive the radar's own
velocity fit succeeds on 23 % of the scans (few stationary returns were injected) and the car stands for long
stretches, so the radar's pair gathers evidence too slowly and the "vote is forming" hold does not cover the
wait. The mechanism is right; its timing on sparse evidence is not good enough.

5.4 **Camera-only speed on the car.** With the vehicle held out, the error is 1.4-1.6 times the stated std
(80-87 % inside 2 sigma instead of 95 %) and the filtered value is not better than the raw one (sigma 0.62
against 0.51 on 104128). L37 had 1.01-1.06 with its floor. The coloured-noise state's parameters (0.30 m/s,
3 s) were set from the characterised sigma and from the simulator, not fitted to the car's residuals; they
should be. The yaw rate's stated std is too small by 2.2-2.3x in the same test.

5.5 **Track continuity and radar-only leads with injected radar.** On 104128 the lead's track ids go from 39
to 112 with the radar, and 1.5 % of the frames have a lead where the camera-only pseudo-truth has none; on
the incident drive a radar-only lead appears in 8.4 % of the frames with no vehicle injected. Part of it is
the injector (roadside objects are placed straight ahead of the pose at the moment they are spawned, so in a
bend some end up on the road and are honest stationary obstacles on the path; the pseudo-truth ends where the
camera's track ended while the injected radar still sees the vehicle), part is the tracker (ids change when
the camera drops and re-designates a track a radar holds). The two are **not separated**. In simulation, where
truth is known, false leads are 0.2-0.7 % of the frames and tracks on no object 0.004-0.008 per frame.

5.6 **Before the first evidence** the camera is `unchecked` and believed, as in L37 (2552 frames on 151146
with a lead at 14-17 m while parked).

## 6. Unverified

* Everything on the phone: CPU, memory, the daemon's sockets against the real modeld and the real feeder, the
  unit file. Nothing was deployed.
* Radar and second camera on real hardware: none exists. Simulation and injection only.
* The cause of the incident (3.6).
* Thresholds of the monitors: as L37 (two healthy vehicle-fed drives, town, daylight). None was tuned to the
  drives in this lane; the new ones (innovation monitor 6 / 3 over 8 s, re-anchor after 8 s, scale regressor
  floors 0.05 rad/s and 3 m/s) were set on the simulator and not moved after seeing the recordings, except
  `gnss_absolute`, which was switched off *because of* 113201.
* 065629 (another calibration and focal length, camera only): the camera is `suspect` in 69 % of the frames
  and the mount state says `moved`; not looked into.
* `tools/integrator_replay.py` of L37: `products`, `compare`, `summary`, `mfd` were used; its `detect`,
  `accuracy`, `characterise`, `cost` were not re-run on the new integrator (`integrator_eval.py` replaces them).
* Recovery from a camera fault on real data: never seen (no recording contains one), simulation only.

## 7. Reproducing

```sh
cd ~/projects/wt/roadsense-l38; D=~/datasets/roadsense-rec; P=$D/trip-pos/pos_20261119.txt; O=/tmp/l38; mkdir -p $O
# tests (aiohttp lives in the models venv on this machine; pytest in the system python); one suite per call
PP=$(python3 -c "import pytest,os;print(os.path.dirname(os.path.dirname(pytest.__file__)))"); PY=~/projects/roadsense/.venv-models/bin/python
PYTHONPATH=$PP $PY -m pytest -q -p no:cacheprovider chris-integrator/tests     # 27 passed, about 4 min
PYTHONPATH=$PP $PY -m pytest -q -p no:cacheprovider api/tests                  # 255 passed
(cd mapd && PYTHONPATH=$PP $PY -m pytest -q -p no:cacheprovider tests)         # 94 passed
# heavy runs inside a memory-capped unit
systemd-run --user --collect -p MemoryMax=12G --wait -- bash -c "cd $PWD &&
  python3 tools/integrator_eval.py sim --seeds 1 2 3 > $O/sim_town.txt;
  python3 tools/integrator_eval.py sim --scenario highway --seeds 1 2 > $O/sim_highway.txt;
  python3 tools/integrator_eval.py sim --radar --cam2 --seeds 1 2 > $O/sim_radar.txt;
  python3 tools/integrator_eval.py real $D/20261119-104128 $D/20261119-113201 $D/20261119-065629 $D/20261119-151146 $D/20261119-153636 $D/20261119-161325 --pos $P > $O/real.txt;
  python3 tools/integrator_eval.py heldout $D/20261119-104128 $D/20261119-113201 > $O/heldout.txt;
  python3 tools/integrator_eval.py gnss $D/20261119-104128 $D/20261119-113201 $D/20261119-151146 $D/20261119-153636 --pos $P > $O/gnss.txt;
  python3 tools/integrator_eval.py inject $D/20261119-104128 --pos $P > $O/inject_104128.txt;
  python3 tools/integrator_eval.py inject $D/20261119-151146 --pos $P --no-lead > $O/inject_151146.txt;
  python3 tools/integrator_eval.py inject $D/20261119-113201 --pos $P --fault 430 520 0.7 > $O/inject_fault.txt;
  python3 tools/integrator_eval.py inject $D/20261119-113201 --pos $P --cam2 --two > $O/inject_all.txt;
  python3 tools/integrator_eval.py cost $D/20261119-104128 --pos $P > $O/cost.txt;
  python3 tools/integrator_eval.py cost $D/20261119-113201 --pos $P --radar >> $O/cost.txt;
  for s in 104128 113201 065629 151146 153636 161325; do
    python3 tools/integrator_replay.py products $D/20261119-\$s --pos $P --out $O/\${s}_l38.jsonl.gz;
    python3 tools/integrator_replay.py products $D/20261119-\$s --pos $P --tree ~/projects/wt/roadsense-l37 --out $O/\${s}_l37.jsonl.gz; done;
  for s in 104128 113201 065629; do python3 tools/integrator_replay.py compare $O/\${s}_l37.jsonl.gz $O/\${s}_l38.jsonl.gz --label L37 L38; done > $O/compare.txt;
  python3 tools/integrator_replay.py summary $O/151146_l38.jsonl.gz $O/153636_l38.jsonl.gz $O/161325_l38.jsonl.gz > $O/summary.txt;
  python3 tools/integrator_replay.py mfd $D/20261119-151146 --car-repo ~/projects/cyberpank-prius-gen2-computer > $O/mfd.txt"
(cd chris-integrator && python3 tools/hop_cost.py --seconds 45)
```

The sim table of 3.3 for the motorway was made with `gnss_absolute` on (the tool's setting for the simulated
receiver); with the default (off) it is worse still (heading 3.3 degrees, position 22 m on seed 1).

## 8. What the car repo has to change (`~/projects/cyberpank-prius-gen2-computer`, read-only here)

**Nothing has to change to stay connected to this branch.** The api keeps its port, paths and topics; `lead`,
`lanes`, `view`, `fused`, `ego_state` keep their fields (additions only). The changes below are for the new
architecture and for showing "unknown".

*Repointing, when the consolidated API has moved into chris-integrator (section 4; not yet):*

1. `roadsense_url` / the proxy under `/api/v1/roadsense` and its WebSocket (`ws://127.0.0.1:8090/ws`, checked
   by `tests/test_roadsense_front.py`): point at the integrator's API (same paths and topics; the port is to be
   assigned then). `/preview`, `/frame.jpg` and the raw topic dump stay with roadsense's thin api: the proxy
   needs two upstreams, or the integrator forwards them.
2. `cyberpunk_computer/backend/position_feeder.py`: connect to chris-integrator's `feed.sock`
   (`$CHRIS_RUNDIR/feed.sock`, default `/run/chris/feed.sock`) instead of roadsense's `position.sock`. The
   protocol is the same vehicle input API (`hello` line with `accepts`, then `motion`, `gnss`, `controls`,
   `body`, `objects`). *This one can be done today*: the daemon accepts the feed now. While mapd still needs
   the position for the map, feed both.
3. Topics for the scene feed (`roadsense_scene.py`, `TOPICS`): add `tracks` when the MFD is to draw more than
   the lead (next-lane vehicles once a radar or a second camera exists); `tracks.tracks[]` is in the car frame,
   x forward, **y left** (the `lead` and `view` topics keep y right as before).

*From L37, still open (unchanged file and line references in `docs/lanes/L37-integrator.md` section 4):*

4. `SceneBuilder`: never fall back to raw `road.leads[0]` / raw lane lines while `ego_state.sources.vision.usable`
   is false or `lead.valid` is false; show `unknown` with the reason instead of an empty road.
5. `alerts/inputs.py` `_scene_lead`: "unknown" distinct from "no lead".
6. The motion feed's silences (19-25 per drive, about 7 s) and its stale first samples after them.
7. Receiver: `age_ms` or `utc_ms`, each `Location` once, `speed_acc_mps`, `course_acc_deg`. With honest timing
   the delay state would start near its truth; the motorway problem (5.1) is exactly an unknown delay.
8. The turn signal.

*New from this lane:*

9. `lead.source` still names kinds (`vision`, `vision+ego`, `radar+vision+ego`); new values appear with new
   sensors (`camera+radar+vision+ego`). Anything that string-matches `source` should use `lead.sources[]`.
10. `ego_state` is now 5.4 kB per frame: subscribe to it at a low rate, or read `/v2/health.integrator`.
11. The mount suggestion (`ego_state.calibration.vision.suggested`, `state`) is worth a line on a service
    screen: it is the calibration procedure's "straight drive" done continuously.

## 9. Does the component stand on its own?

Yes, and it is laid out for it. `chris-integrator/` imports nothing from roadsense (the api imports *it*,
through one shim file), has its own tests that need no recording (the simulator), its own README, design
document, `pyproject.toml`, unit file and MIT `LICENSE`. What is general is nearly all of it: the estimator,
the registry, monitors, the object layer, the simulator. What is specific to this car is confined to the
noise constants inside five source files and is named as such.

The move to its own repository needs: `git subtree split --prefix=chris-integrator` (history kept) or a plain
copy; in roadsense, the shim `api/roadsense_api/integrator/__init__.py` finds the package installed instead
of in the tree (it already tries the import first); `tools/replay_lib.py` and `tools/integrator_eval.py` take
the package from the environment; the lane documents stay in roadsense. Licence: MIT is consistent with
roadsense (MIT); roadsense-signs (AGPL) is a separate process whose messages are read, none of its code is
here. Checked for private material: no addresses, host names, keys or local paths in the directory; no
recordings.

Two things I would settle before publishing: the motorway position (5.1) should be fixed or the README's
limits should name it (they name the position as weak, not that case), and the consolidated API should have
moved (section 4), since the README describes the integrator as the component consumers talk to.

## 10. Branch

`v2-l38`, head: see the lane's hand-back (this file is in the last commit). Commits, oldest first:
a69774c design · 8c2436f core, stack sources, simulator, api on it · 8d6bc3f object-layer speed, adoption ·
bf72bc8 isolation timing · 620ffc4 lead naming · 7bb2df6 radar · 0eda7b4 second radar · 41c0b1a second camera ·
08e0daa design: integrator on top · ab48171 chris-integrator, daemon, feed, evaluation · the last: per-stream
ordering fix, design as built, this report.
