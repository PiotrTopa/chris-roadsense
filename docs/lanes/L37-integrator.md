# L37: the data integrator

Lane L37, 2026-11-19, laptop only (phone and car read-only; nothing deployed). Branch `v2-l37` from `v2-l35`
(be05cf0). Design: `docs/INTEGRATOR.md`. Contract: `docs/ARCHITECTURE.md` ("The data integrator: api-derived
topic `ego_state`"). Builds on the audit `docs/lanes/L35-ego-fusion-audit.md`.

## 0. Result in one page

* **Built:** `api/roadsense_api/integrator/`: a source registry (input description + measurement model per
  source), a time-ordered ego estimator (two small Kalman filters, late measurements inserted at their time of
  validity, smoothed state at the frame time), consistency monitors between sources and of a source against
  its own stated uncertainty, fault isolation by rank or vote, and the output `ego_state` with value / std /
  valid / reason / mode / sources per quantity and a health state per source. Every tracker reads it; none
  selects a raw source any more. mapd relays raw `gnss`; the recorder keeps `road_model`, `gnss`, `ego_state`.
* **The incident class is caught.** On the two bad drives the camera is faulted **3.0 s after the car first
  moves at 3 m/s** (151146) and **while still parked, 17.5 s into the session** (153636), and stays faulted to
  the end: 99.0 % and 100 % of the moving frames. Tracked lead with the vehicle above 5 m/s: before L37
  present in 93.4 % / 100 % of the frames at 5.0 / 6.4 / 8.7 m and 4.7 / 5.8 / 7.8 m (p10 / p50 / p90), one
  track up to 413 s; after L37 present in **0.4 % / 0 %**, `valid: false` with the reason in 99.4 % / 100 %.
  Lane lines and the path go out as unknown in the same frames. On a session without any vehicle feed and the
  same knocked camera (161325) the camera's own stated uncertainty faults it (79 % of the session).
* **No false alarm** on the three healthy drives (two with a vehicle feed: 716 s + 159 s of moving evidence;
  one camera-only: 233 s) and the two parked sessions. Margins: share of frames beyond 4 sigma over 5 s at
  most 0.20 / 0.12 against a threshold of 0.5 (bad drives: 0.93 / 0.86 typical); own-uncertainty ratio at most
  2.62 / 2.55 / 2.49 against 4 (bad: 7.4 / 8.3 / 12.9 typical).
* **No regression:** on the three healthy drives lead presence is identical frame by frame to the code before
  L37 (0 of 41175 frames differ), tracked distance within 0.01 m (p99) with a vehicle feed and 0.24 m
  camera-only, lane presence patterns identical in 100 % of the frames, `view` identical.
* **Found on the way** (each changed the build): the vehicle feed sends stale values for up to 0.28 s after
  its silences (7 of 49 silences, up to 7.4 m/s wrong; the old trackers used them); the head-unit GNSS is
  wrong by half on the healthy drives and cannot even be a voting witness; a single-sample gate locked a lone
  camera out of its own filter (9 m/s error) until the gate was tied to a trusted anchor; the knocked camera
  passed a sigma-normalised check by inflating its own `vx_std`, so monitors use the characterised sigma.
* **Not done / not proven:** no object layer for radar targets (designed, section 3 of the design; no radar
  data exists), recovery from a fault never seen on real data, CPU on the phone not measured, thresholds rest
  on two healthy vehicle-fed drives. Section 6.

## 1. Design in short

**State.** Ego motion: longitudinal [speed, acceleration, accelerometer offset] and yaw [yaw rate, its
derivative], two linear Kalman filters on one time axis. Objects stay in the lead tracker, which is now the
object layer *behind* the integrator (it gets its ego speed, standstill, brake and "may the camera be
believed" from it). Position stays in mapd (it needs the map); mapd relays the raw `gnss`.

**Sources** (`integrator/sources.py`; noise measured on the car):

| source (group, rank) | messages | measurements | role |
|---|---|---|---|
| vehicle (3) | `ego`, `vehicle` | speed 0.03 + 0.002 v m/s; standstill (v = 0 exactly); accelerometer 0.25 m/s2 (+ offset state); yaw rate by its origin (gyro 0.004 + 0.03 r); turn signal, brake | fused, never gated; not believed for 0.35 s after a silence > 1 s |
| wheels (2, family vehicle) | `ego.wheel_speeds_mps` | mean wheel speed, 0.10 + 0.04 v | monitor: checks the speed signal's decoding; no vote (same sensors) |
| vision (1) | `road.pose` | speed and yaw rate 0.125 s before the frame stamp; speed sigma max(0.25 + 0.035 v, stated `vx_std`); yaw 0.007 + 0.25 r; stated / characterised sigma; standstill detector for camera-only cars | fused, gated only while a vehicle measurement anchors the state |
| gnss (0, witness) | `gnss` (new mapd relay) | speed over ground, repeats dropped | reported; never fused, no vote |

**Time.** One clock (the phone's CLOCK_MONOTONIC). Each measurement has a time of validity; a 2 s buffer
keeps them sorted; a late one (the camera: 85-130 ms late, valid another 125 ms earlier) is inserted and the
filter re-run from there. The state for a frame is the fixed-lag smoothed state at the frame time (vehicle
samples on both sides of it have arrived by then). A source's clock stepping back by more than 2 s starts a
new epoch (boot); faults are carried across it and across api restarts.

**Fault detection and isolation.** Never against the fused state. Pair monitors compare two sources' raw
measurements at a common time, normalised by their characterised sigmas, over 5 s of evidence (either speed
>= 3 m/s; standing still is no evidence): `disagree` at >= 15 samples with >= 50 % beyond 4 sigma, back to
`agree` at <= 25 % over 10 s. A self monitor faults a source whose stated uncertainty is 4x its characterised
one (5 s median; back below 3x over 10 s). A disagreeing pair faults the lower-ranked source unless
independent third sources side with it against the higher one (vote). A faulted source leaves the estimate
at once and everything made from it goes out as unknown with the reason.

**Output.** `ego_state` per road frame (section "The data integrator" of ARCHITECTURE.md): `speed`, `accel`,
`yaw_rate` each with `value`, `std`, `valid`, `reason`, `mode` (fused / coast / none), `age_s`, `sources`;
`standstill`; pass-through `signals` with age; `sources` (state `absent | stale | unchecked | ok | suspect |
fault`, reason, isolation rule, since, age, rate); `checks`; counters. `lead`, `lanes` carry `valid`,
`reason`, `inputs`; `view` carries `integrity`.

**Filter family, and what was rejected** (design section 2): a linear Kalman filter because every source
observes the state directly; no EKF / UKF / particle filter (nothing non-linear or multi-modal), no single big
state with position (it would import the weak GNSS into the trackers), no innovation test against the fused
state as the fault detector (the dominant source would test itself; the audit's z-share rule had a thin
margin), no fusing of GNSS, no separate process yet (a second JSON hop per message for no gain; the package
has no dependency on the api).

## 2. What was built, and which consumers moved

| file | what |
|---|---|
| `api/roadsense_api/integrator/core.py` | `Integrator`, `SourceSpec`, `Measurement`; registry, epochs, isolation, health, `ego_state` |
| `.../filters.py` | `Channel`: time-ordered Kalman filter, insert + re-run, RTS smoothing, anchored gate, std floor |
| `.../monitors.py` | `Pair` (share and ratio modes), `SelfCheck`, `Series` |
| `.../sources.py` | this stack's sources and their noise |
| `.../contract.py` | `validity`, `usable`, `signal`, `LocalEgo` (a tracker used alone builds its own integrator: one code path) |
| `api/roadsense_api/state.py` | integrator first in the fan-out; `ego_state` emitted per frame; faults saved to / carried from `<data dir>/integrator.json`; `health.integrator` |
| `api/roadsense_api/server.py` | `GET /v2/ego_state`; status page line; api 2.1.0 |
| `mapd/roadsense_mapd/__main__.py` | raw `gnss` relayed on `ego.sock` |
| `api/roadsense_api/recorder.py` | `road` keeps `road_model` (audit finding 6) and the other scalars; `gnss`, `ego_state` recorded; `ego` at the relay rate |
| `tools/replay_lib.py` | `Pipeline` with the integrator; `read_session(road_model=, pos=)` restores the model name and merges trip-log GNSS by wall time |
| `tools/integrator_replay.py` | `characterise`, `detect`, `accuracy`, `products`, `summary`, `compare`, `mfd`, `cost` |
| tests | `api/tests/test_integrator.py` (25), `mapd/tests/test_daemon.py` (gnss relay), three existing tests adapted |

Raw reads of the audit's table 1.2, and where they went:

| consumer | before | now |
|---|---|---|
| lead (`lead.py`) | own vehicle-within-0.5 s-else-model rule; brake from `vehicle`; no check of the camera | ego speed, standstill, brake from `ego_state`; no track while vision is faulted or the speed unknown (`valid: false`, reason, event `invalid`, `vision: null`) |
| lanes (`lanes.py`) | same rule again; pose yaw rate | `ego_state`; all lines absent with `valid: false` while vision is faulted |
| view (`view.py`) | same rule again | `ego_state`; no lead / lines / path and `integrity.valid: false` while vision is faulted |
| road geometry, ego lane | speed from `lanes.ego`, yaw / turn signal from raw `ego` | `ego_state` (yaw rate, turn signal with its age) |
| speed limit / rules odometer | always the road model's speed and yaw rate | `ego_state`: the vehicle's when fed; stands while the speed is unknown; `rules.ego_source` |
| sign context | raw `leads[0]` | the tracked, validated `lead` |
| ego light | raw plan always | the plan only while vision is not faulted |
| fused | raw lane lines as a fallback | not while vision is faulted; `fused.lead` carries `valid`, `reason` |
| /preview overlay | raw | unchanged (debug picture); the status page shows the verdict |
| mapd EKF, modeld calibrator | | unchanged (design section 3) |

Not modelled as sources: `map`, `signs`, `lights`, `env` (detector and map outputs, not ego motion; they go to
their consumers as before) and `objects` (no radar exists; it still reaches `lead.py`'s radar code, off by
default). The integrator gates what is made from the camera's geometry.

## 3. Measurements

Sessions `<datasets>/roadsense-rec/20261119-*`; `road_model` restored to `vision` in every replay. Commands in
section 5. Good: 104128, 113201 (vehicle feed), 065629 (camera only). Bad: 151146, 153636 (vehicle feed),
161325 (camera only, same knocked camera). Parked only: 135622, 145155.

### 3.1 Detection (`integrator_replay.py detect`)

| session | moving frames (vehicle >= 3 m/s) | first vision fault | by | moving frames in fault | pair share beyond 4 sigma p50 / p99 / max | own-uncertainty ratio p50 / p99 / max |
|---|---|---|---|---|---|---|
| 104128 good | 8842 (716 s) | never | | 0 % (suspect 0 %) | 0.00 / 0.14 / 0.20 | 0.87 / 1.90 / 2.62 |
| 113201 good | 1724 (159 s) | never | | 0 % (suspect 0 %) | 0.06 / 0.07 / 0.12 | 1.41 / 2.45 / 2.55 |
| 065629 good, camera only | - | never | | | no pair | 0.96 / 2.43 / 2.49 |
| 135622, 145155 parked | 0 | never (`unchecked` throughout) | | | no evidence | |
| 151146 bad | 4229 (330 s) | 206.7 s = **3.0 s after the first moving frame** | pair, rank | **99.0 %** (suspect 0.9 %); after the first fault 100 % | 0.93 / 1.00 / 1.00 | 7.35 / 11.28 / 11.45 |
| 153636 bad | 2779 (215 s) | 17.5 s, **parked, before moving** | self, then pair | **100 %** | 0.86 / 1.00 / 1.00 | 8.26 / 13.02 / 13.41 |
| 161325 bad, camera only | - | 130.3 s (the self monitor only counts frames where the model's own speed is >= 1 m/s) | self | 79.2 % of all frames | no pair | 12.88 / 15.01 / 15.01 |

Thresholds: pair 0.5, self 4. The fault on both vehicle-fed bad drives is one interval to the end of the
session. Other checks: vehicle / wheels never disagree (ratio 0.98, share 0.00-0.02); the yaw-rate pair
(camera against gyro) never disagrees, also not in the incident (share at most 0.20; it only makes
`suspect`). Vehicle feed `stale` in 7.3 % / 5.2 % of the frames of the good drives (its silences).

GNSS witness (30 s ratio of medians, band 0.6-1.6), share of frames with verdict `disagree`:

| session | vehicle vs gnss | camera vs gnss | ratio gnss / vehicle p50 |
|---|---|---|---|
| 104128 good | 52 % | 6 % | 0.57 |
| 113201 good | 59 % | 62 % | 0.42 |
| 151146 bad | 12 % | 84 % | 0.88 |
| 153636 bad | 0 % | 74 % | 0.96 |

It sided with the vehicle in the incident and against it on the healthy drives: as a voter it would have been
harmful. It is recorded, reported and marks itself `suspect`; it has no vote.

### 3.2 What lead / lanes / view put out (`products`, `summary`; L35 = code before this lane, same data)

Vehicle above 5 m/s:

| session | code | raw `leads[0]` p >= 0.5: share, dist p10/50/90 | tracked lead present, dist p10/50/90 | longest track | lead `valid: false` | both inner lane lines | view shows lead / lines |
|---|---|---|---|---|---|---|---|
| 151146 | L35 | 0.73, 4.8 / 7.1 / 11.7 m | **0.934**, 5.0 / 6.4 / 8.7 m | 256.5 s | - | 0.037 | 0.939 / 0.230 |
| 151146 | L37 | same | **0.004** (11 frames before the fault: one track of 0.8 s at 5.5 m) | 0.8 s | 0.994 | 0.000 | 0.006 / 0.000 |
| 153636 | L35 | 0.96, 4.3 / 6.4 / 11.5 m | **1.000**, 4.7 / 5.8 / 7.8 m | 413.4 s | - | 0.307 | 1.000 / 0.372 |
| 153636 | L37 | same | **0.000** | 0 | 1.000 | 0.000 | 0.000 / 0.000 |

All frames, while vision is faulted (15058 and 7026 frames): lead present 0.000, lanes invalid 1.000, view
integrity false 1.000, view lead 0.000-0.001 (the fade-out frames).

What is left out of reach: before the first evidence the camera is `unchecked` and its products go out. On
151146 that is 2554 frames (the car parked for 200 s; a lead at 14.2 / 15.8 / 16.9 m in 84.5 % of them, which
may well be a real parked car: raw probability >= 0.5 in 3 % only) and the 3.0 s before the fault. On 153636
it is the first 17.4 s (a lead at 3.8 m in 233 of 234 frames). With the fault carried over from the previous
run (what the api does on the car, `integrator.json`) the first 60 s of 153636 have a lead in 0 of 802 frames
instead of 233.

Through the car computer's own scene builder (`mfd`; `roadsense_scene.py` imported read-only, unchanged):
151146, vehicle >= 5 m/s: scene lead in 93.9 % of the scenes at 5.0 / 6.5 / 8.8 m with fault detection off,
**0.0 %** (drawn lead 0.2 %) with the integrator; lane lines 21.9 % -> 0.0 %. On 104128 scenes are built as
before (lead in 47.6 % of the scenes at >= 5 m/s, dist 10.9 / 35.7 / 60.4 m).

### 3.3 No regression on the good drives (`compare` L35 against L37, frame by frame)

| session | frames | lead present L35 / L37 | present in one only | distance difference where both, p50 / p90 / p99 | v_rel difference p50 / p90 / p99 | lead speed change per frame p50 / p90 / p99, L35 -> L37 | lane presence pattern identical | view lead, lines |
|---|---|---|---|---|---|---|---|---|
| 104128 | 28287 | 0.769 / 0.769 | 0 / 0 | 0.00 / 0.00 / 0.01 m | 0.00 / 0.01 / 0.05 m/s | 0.01 / 0.13 / 0.46 -> same | 100 % | 0.773, 0.309 both |
| 113201 | 9745 | 0.671 / 0.671 | 0 / 0 | 0.00 / 0.00 / 0.01 m | 0.00 / 0.01 / 0.09 m/s | 0.01 / 0.10 / 0.45 -> 0.01 / 0.10 / 0.46 | 100 % | 0.674, 0.185 both |
| 065629 camera only | 3143 | 0.960 / 0.960 | 0 / 0 | 0.01 / 0.05 / 0.24 m | 0.10 / 0.54 / 1.43 m/s | 0.10 / 0.39 / 0.83 -> 0.08 / 0.34 / 0.76 | 100 % | 0.963, 0.653 both |

Same number of tracks and the same longest track in every case (39 / 241 s, 23 / 77 s, 5 / 91 s). Split by
"vehicle >= 5 m/s" and "vehicle stands" the picture is the same (the tool prints both). On the camera-only
drive the lead's speed is a little steadier: the ego speed added to the pair's relative speed is the
filtered one instead of the per-frame one.

### 3.4 Ego-state accuracy against held-out sources (`accuracy`)

Speed error against the recorded vehicle speed, frames with the vehicle above 1 m/s:

| test | session | n | robust sigma | rms | p99 | error / reported std (rms) | raw camera at the frame: sigma, rms |
|---|---|---|---|---|---|---|---|
| vehicle held out entirely (camera only) | 104128 | 10162 | 0.510 | 0.629 | 1.93 | 1.01 | 0.521, 0.652 |
| | 113201 | 2321 | 0.411 | 0.579 | 1.68 | 1.06 | 0.423, 0.593 |
| every other vehicle sample held out, frames nearest a held-out one | 104128 | 5078 | 0.025 | 0.055 | 0.15 | 0.58 | 0.522, 0.646 |
| | 113201 | 1159 | 0.023 | 0.366 | 0.87 | 1.53 | 0.424, 0.613 |
| | 151146 | 3894 | 0.017 | 0.030 | 0.11 | 0.45 | 2.85, 6.23 |
| | 153636 | 2117 | 0.015 | 0.025 | 0.09 | 0.39 | 3.75, 5.22 |
| 2 s vehicle gaps every 20 s, inside the gaps | 104128 | 913 | 0.469 | 0.592 | 1.72 | 0.97 | 0.518, 0.610 |
| | 113201 | 213 | 0.543 | 0.664 | 1.78 | 1.10 | 0.573, 0.701 |
| | 151146 (camera faulted) | 685, valid 38 % | 0.138 | 0.314 | 1.36 | 0.46 | 2.42, 5.55 |
| | 153636 (camera faulted) | 387, valid 36 % | 0.067 | 0.100 | 0.21 | 0.20 | 4.13, 5.05 |

Reading:
* With the vehicle feed the estimate is the vehicle speed at the frame time, to 0.02-0.03 m/s where a sample
  was held out. The heavy tail on 113201 (rms 0.37, p99 0.87) is the reference, not the estimate: its stale
  samples after feed silences, which the integrator no longer follows.
* **Camera only, the filter is no better than the raw value** (sigma 0.51 against 0.52, 0.41 against 0.42;
  rms 3 % lower). The camera's error is correlated over seconds; a filter that assumes white noise cannot
  remove it. The reported std was 1.44 times too small until it was floored at the camera's own sigma; it is
  now honest (1.01, 1.06). On a car without a vehicle feed the integrator therefore adds checking and
  metadata, not accuracy.
* Inside feed gaps with a healthy camera the error is the camera's (0.47-0.54); with a faulted camera the
  estimate coasts on the accelerometer state for 1 s (0.07-0.14 sigma) and is then unknown (valid 36-38 % of
  the gap frames), as designed.
* Camera only on the bad drives the self monitor withholds 92 % / 91 % of the moving frames; in the rest the
  speed is published and wrong (median error -1.6 / -2.3 m/s). That is the residual risk of a car without a
  vehicle feed.
* Yaw rate with all sources: the gyro's (difference to it 0.0004-0.0007 rad/s). Camera only: sigma 0.009
  rad/s (raw 0.008). Whether the gyro or the wheels have the right scale (wheel-difference gain 0.947
  against the gyro) is not decided by this data.

### 3.5 Sources as measured (`characterise`)

| | 104128 | 113201 | 151146 | 153636 |
|---|---|---|---|---|
| vehicle speed, one sample left out: robust sigma (m/s) | 0.020 | 0.017 | 0.012 | 0.012 |
| wheel speed / speed signal (per wheel) | 0.979-0.981 | 0.979-0.981 | 0.979-0.981 | 0.978-0.981 |
| rear wheel-difference yaw against gyro: gain in turns, sigma (rad/s) | 0.947, 0.019 | 0.947, 0.014 | 0.942, 0.015 | 0.946, 0.010 |
| camera speed at lag 0 / 0.125 / 0.25 s: robust sigma (m/s) | 0.570 / 0.554 / 0.563 | 0.482 / 0.481 / 0.501 | 4.39 (any) | 3.72 (any) |
| camera yaw rate at 0.125 s: sigma, gain in turns, rms in turns | 0.0064, 0.962, 0.031 | 0.0062, 0.922, 0.071 | 0.0057, 0.637, 0.114 | 0.0050, 0.854, 0.039 |
| accelerometer - d(speed)/dt: median, sigma (m/s2) | -0.02, 0.25 | +0.10, 0.20 | +0.19, 0.24 | +0.19, 0.23 |
| bridging 1 s: hold the speed -> speed + accelerometer (rms, m/s) | 0.634 -> 0.351 | 0.746 -> 0.476 | 0.750 -> 0.451 | 0.474 -> 0.312 |
| vehicle feed silences > 1 s (median length) | 19 (7.0 s) | 25 (7.0 s) | 5 (6.3 s) | 0 |
| resumes whose first 0.3 s are > 1 m/s off where the feed settles (worst) | 2 (4.2 m/s) | 3 (7.4 m/s) | 2 (4.1 m/s) | 0 |
| camera's stated `vx_std` / characterised sigma, p50 / p95 / p99 | 0.78 / 1.92 / 2.91 | 0.82 / 2.12 / 3.17 | 7.07 / 12.3 / 14.6 | 8.85 / 13.8 / 15.7 |

(065629: 0.92 / 2.53 / 3.91; 161325: 12.8 / 16.0 / 17.1.) The lag minimum of the camera speed is flat; 0.125 s
is used (audit: 0.10-0.15).

### 3.6 CPU (laptop, one job at a time; `cost`, `products`)

| | per road frame |
|---|---|
| integrator alone, vehicle-fed drive (104128), all its inputs included | 0.350 ms (0.43 % of one core at the drive's rate) |
| integrator alone, camera-only drive (065629) | 0.161 ms (0.15 %) |
| whole pipeline (integrator, lead, lanes, ego lane, road geometry, view, fused), 113201 | 0.967 ms |
| the same before L37 | 0.580 ms |

The pipeline costs 0.39 ms more per frame, 0.35 ms of it the integrator (numpy 2x2 / 3x3 operations in
Python). The phone was not touched: the api there uses about 9 % of the little and 11-12 % of the big cores
(L28); if the phone is 4-6 times slower per Python operation than this laptop, the integrator is 2-3 % of one
core at 13 frames a second. That is an estimate, not a measurement.

## 4. What the car repo has to change (`~/projects/cyberpank-prius-gen2-computer`, read-only here)

Nothing has to change for roadsense 2.1 to be safe to connect: every addition is additive, and a faulted
camera reaches an unchanged MFD as "no lead, no lines" (measured through its `SceneBuilder`, 3.2). To show
"unknown" instead of an empty road, and to close the fallbacks:

**`cyberpunk_computer/backend/roadsense_scene.py`**
1. `TOPICS` (line 60): add `"ego_state"` (about 1.5 kB per road frame; or read `view.integrity` only and skip
   the topic).
2. `SceneBuilder.feed` (line 1002): `elif mtype == "ego_state":` keep
   `(now, msg["sources"]["vision"]["usable"], msg["sources"]["vision"].get("reason"))`; for `lead` (line 1027)
   also keep `msg.get("valid") is not False` and `msg.get("reason")`; for `lanes` the same.
3. `SceneBuilder.build`, the path without `view` (line 1076, `road_to_scene(road, self.cfg)`), and the lead
   block at line 1088: **never take the raw `road.leads[0]` / raw `lane_lines` while roadsense says the camera
   is not usable.** Concretely: when the last `ego_state` (not older than 1 s) has `vision.usable == false`,
   or the last `lead` message had `valid == false` and is not older than 5 s, set `scene["lead"] = None`,
   `scene["lanes"] = [None] * 4`, `scene["edges"] = [None] * 2`, drop `scene["path"]`, also when `lead` /
   `lanes` are older than `lead_max_age_s` / `lanes_max_age_s` (today that age-out falls back to raw).
4. New scene keys: `"vis": {"ok": false, "why": "<reason>"}` from `view.integrity` (view mode:
   `view_to_scene`, line 910, pass `msg["integrity"]`) or from `ego_state`; `"lead_unknown": true` when
   `lead.valid` is false. The screen shows a "camera not believable" mark with the reason instead of an empty
   road; `lead_departed` must not be raised from a lead that went invalid.
5. `cyberpunk_computer/alerts/inputs.py` `_scene_lead` (lines 240-262): when `scene["lead_unknown"]` (or
   `vis.ok` false) return "unknown" distinct from "no lead" (today `(None, True)`), so that nothing that
   means "road ahead is clear" is derived from it; the third fallback (raw `leads[0]`, scene `lead` without
   `lead_src`) must not be taken when `vis.ok` is false.

**`cyberpunk_computer/backend/position_feeder.py` / the panda reader**
6. The motion feed falls silent for about 7 s 19-25 times per drive (2026-11-19, 104128 and 113201), and in 7
   of 49 resumes its first 2-3 messages (up to 0.28 s) carry the values held before the silence (0 m/s and
   standstill at a true 5.3 m/s; 11.3 at a true 6.0). Find the cause of the silences; after one, send a
   `motion` message only from CAN frames received after it, with their receipt time. roadsense now ignores a
   feed for 0.35 s after a silence of more than 1 s, which covers the observed cases and costs 0.35 s of
   vehicle speed per resume.
7. GNSS (as in the audit): `age_ms` or `utc_ms`, each `Location` once, `speed_acc_mps`, `course_acc_deg`. With
   honest timing the receiver could become a ranked source; as it is it disagrees with the car's own speed
   in half of the healthy frames.
8. The turn signal (`controls.turn_signal`): the hook exists, nothing feeds it; road geometry and the
   ego-lane filter read it through the integrator.

Message additions the car may read: topic `ego_state` (ARCHITECTURE.md), `lead.valid` / `reason` / `inputs` /
`ego.std_mps` / event `invalid`, `lanes.valid` / `reason` / `inputs`, `view.integrity`, `fused.lead.valid` /
`reason`, `rules.ego_source`, `/v2/health.integrator`, raw `gnss` on `/ws`.

## 5. Reproducing

```sh
cd ~/projects/wt/roadsense-l37; D=<datasets>/roadsense-rec; O=/tmp/l37; mkdir -p $O
ALL="$D/20261119-104128 $D/20261119-113201 $D/20261119-065629 $D/20261119-151146 $D/20261119-153636 $D/20261119-161325"
systemd-run --user --collect -p MemoryMax=8G --wait -- bash -c "cd $PWD &&
  python3 tools/integrator_replay.py characterise $ALL > $O/characterise.txt;
  python3 tools/integrator_replay.py detect $ALL $D/20261119-135622 $D/20261119-145155 --pos $D/trip-pos/pos_20261119.txt > $O/detect.txt;
  python3 tools/integrator_replay.py accuracy $D/20261119-104128 $D/20261119-113201 $D/20261119-151146 $D/20261119-153636 > $O/accuracy.txt;
  for s in 104128 113201 065629 151146 153636 161325; do
    python3 tools/integrator_replay.py products $D/20261119-\$s --pos $D/trip-pos/pos_20261119.txt --out $O/\${s}_l37.jsonl.gz;
    python3 tools/integrator_replay.py products $D/20261119-\$s --tree ~/projects/wt/roadsense-l35 --out $O/\${s}_l35.jsonl.gz; done;
  for s in 104128 113201 065629; do python3 tools/integrator_replay.py compare $O/\${s}_l35.jsonl.gz $O/\${s}_l37.jsonl.gz --label L35 L37; done > $O/compare.txt;
  python3 tools/integrator_replay.py summary $O/151146_l35.jsonl.gz $O/151146_l37.jsonl.gz $O/153636_l35.jsonl.gz $O/153636_l37.jsonl.gz $O/161325_l37.jsonl.gz > $O/summary.txt;
  python3 tools/integrator_replay.py mfd $D/20261119-151146 --car-repo ~/projects/cyberpank-prius-gen2-computer > $O/mfd.txt;
  python3 tools/integrator_replay.py mfd $D/20261119-151146 --car-repo ~/projects/cyberpank-prius-gen2-computer --no-gate >> $O/mfd.txt;
  python3 tools/integrator_replay.py cost $D/20261119-104128 > $O/cost.txt"
# tests (aiohttp lives in the models venv on this machine; pytest in the system python)
PP=$(python3 -c "import pytest,os;print(os.path.dirname(os.path.dirname(pytest.__file__)))")
PYTHONPATH=$PP ~/projects/roadsense/.venv-models/bin/python -m pytest -q -p no:cacheprovider api/tests           # 280 passed
(cd mapd && PYTHONPATH=$PP ~/projects/roadsense/.venv-models/bin/python -m pytest -q -p no:cacheprovider tests)  # 94 passed
```
`--tree` runs the pipeline of another checkout (here the code before this lane) on the same data, with
`road_model` restored in both. The carried-fault number of 3.2 is `Pipeline().integrator.carry_fault("vision",
...)` before feeding 153636 (unit test: `ApiState.test_topics_and_carried_fault`).

## 6. Not verified, open

* **Thresholds** rest on two healthy vehicle-fed drives (875 s of moving evidence), one healthy camera-only
  drive, town speeds (no motorway), daylight, no rain, and one kind of incident. The margins are wide on this
  data (0.20 against 0.5 against 0.86; 2.6 against 4 against 7.4); their behaviour at night, in rain (a
  camera that honestly states a large uncertainty) and at motorway speeds is unknown. Nothing was tuned to
  make a number pass; the 0.5 / 4 thresholds were set from the first look at the distributions and not moved.
* **Recovery** from a fault was never seen on real data (the camera was not put back within a recording);
  only the synthetic tests exercise it. The same for isolation by vote (no third independent ranked source
  exists on the car) and for a wrong vehicle speed.
* **Before the first evidence** of a drive the camera is `unchecked` and believed (3.2). The carried fault
  covers the case of 153636; a camera knocked while parked with no fault on file is not caught until 1.2-3 s
  into the first motion (or by the self monitor).
* **Camera-only cars:** the self monitor is the only check; it rests on this road model's `vx_std` behaviour
  in one incident, and let 8-9 % of the moving frames through with a wrong speed marked valid.
* **CPU on the phone**, the live path on the phone (mapd's gnss relay into the api, the fault file under
  `/var/lib/roadsense`), and the recorder's new topics on the phone's disk budget are not measured (unit and
  socket tests only).
* **Object layer:** not built. `objects` have no source model; `lead.py`'s radar association is as it was
  (off). A radar's ego speed and a second camera plug in through the registry (tests with synthetic sources);
  their detections need the object layer of design section 3.
* **Static objects in `view`** (lights, signs) are still drawn while the camera is faulted, positioned from
  the image without the plan; `view.integrity` is the only flag on them. The `/preview` overlay draws raw.
* **modeld's online calibrator** and mapd's `vision_odometry` do not read the verdict (no path from the api to
  them). On 2026-11-19 the calibrator said `collecting` throughout; the integrator's fault is now the signal
  that says "the camera moved or the model is lost" (it cannot tell the two apart).
* **GNSS** reaches the integrator and the recorder but adds nothing to the estimate; whether the phone's own
  modem is better was not measured.
* **Older replay tools** (`lead_replay.py`, `lanes_replay.py`, ...) run the new trackers through a private
  integrator and still do not restore `road_model` on sessions recorded before this lane: use
  `integrator_replay.py products` or `read_session(road_model="vision")` for those.
* **Merge with L36:** this lane edited the heads of `view.py` (`ViewBuilder` inputs, `on_frame` up to the
  returned dict, one added key `integrity`) and `roadgeom.py` (`on_ego`, the first lines of `on_lanes`);
  `roadview.py` is untouched. A textual conflict is possible where L36 touches the same lines.

## 7. A component of its own?

Not yet. What is general is small and already separable: `core.py`, `filters.py`, `monitors.py`,
`contract.py` (about 950 lines, no import from the rest of the api, messages in and dicts out). What made it
work on this car is not general: the measured noise of each source, the 0.125 s of the image pair, the
warm-up of this feeder, the self monitor on this model's `vx_std`, the decision that this GNSS has no vote.
Three things would have to be true before a separate repository is worth its overhead: an object layer that
fuses a real radar's targets through the same registry without touching the core; a second ranked ego-speed
source, so that isolation by vote runs on data and not only in tests; and a second deployment (another car or
another camera model) whose source file can be written without editing the core. The radar is the test: if
adding it is a source file and an object-layer module, extract `integrator/` then; if the core has to be
reworked for it, it was roadsense-specific and stays in.
