# L41: estimation quality of chris-integrator

Lane L41, 2026-11-19 / 20, laptop only (phone and car read-only; the phone was unreachable for the whole lane, so
nothing was read from it either). Branch `v2-l41` from `v2-l38` (75285f9). Object layer: sub-lane L41-O
(branch `v2-l41-obj`, cherry-picked here). Camera-mount evidence: sub-lane L41-M (investigation, no code).
Public description of what changed: `chris-integrator/docs/DESIGN.md` (sections 2, 5a), `docs/FAULTS.md`.

"Before" is always L38's integrator (`v2-l38`), measured in this lane with the same tools and the same scoring.

## 0. Result in one page

| | before (L38) | after (L41) |
|---|---|---|
| fault campaign, 59 faults in simulation: detected / isolated correctly / wrong source blamed | 46 / 12 / 6 | 55 / 38 / 2 |
| ... time an output was published valid and confidently wrong, all faults | 1657 s | 748 s |
| ... alarms on 12 healthy simulated drives (38 min) | gnss suspect 0.7 %, map suspect 5.5 % of the time | vehicle and camera suspect once for 0.3 s (0.01 %) |
| motorway (simulation, 3 seeds): position rms, NEES (2 dof) | 17.0-22.0 m, 12.2-23.2 | 4.3-9.7 m, 0.74-4.0 |
| ... heading rms, NEES | 2.25-2.27 deg, 16.9-22.0 | 0.30-0.66 deg, 0.28-1.16 |
| town (simulation, 3 seeds): position rms, NEES | 1.0-5.0 m, 0.30-8.6 | 1.2-2.3 m, 0.35-1.22 |
| camera-only speed on the car: error / stated std (rms); inside 2 sigma | 1.56, 1.39; 0.80, 0.87 | 0.75, 0.90; 0.98, 0.97 |
| ... filtered error rms against the raw camera's | 0.75 / 0.65, 0.59 / 0.63 m/s | 0.59 / 0.65, 0.60 / 0.63 m/s |
| position on the car against mapd's filter | not compared | about equal in the median, shorter tail (section 3) |
| camera pitch step of 1.7 deg (simulation) | mount `moved`, never followed | found in 0.6 s, reported as +1.83 deg, followed |
| the incident drives | camera faulted; mount `unknown` | the same, plus "model out of range" and, from sign rows, "camera pitch changed by about 16-17 deg up" |
| lead track ids with an injected radar, 104128 (39 as recorded) | 112 | 60 (54 on the sub-lane's own branch) |
| radar-only lead on the incident drive, no vehicle injected | 8.4 % of frames | 0.05 % (0 on the sub-lane's branch) |
| cost, laptop, 600 s of driving of 104128 | 2.15 ms per road frame | 2.2-2.3 ms (L38 re-measured alongside: 2.0-2.4): the same |
| tests: chris-integrator / api / mapd | 27 / 255 / 94 | 39 / 255 / 94 |

**What is not good, stated first** (details in section 8):

* **False alarms on the real healthy drives went up.** On 104128 / 113201 the camera is falsely faulted in
  0.8 % / 0.7 % of the moving frames (L38: never) and marked suspect in 20.5 % / 3.9 % (L38: 0.3 % / 2.4 %). The new
  pair tests and the receiver's track are the cause; their limits were set on the simulator, then loosened
  after seeing the two drives (said in the code), and are still not quiet.
* **The receiver's track, the third opinion that isolates a wrong vehicle speed, is unreliable on this car's
  receiver**: on 104128 it is itself in disagreement with the vehicle in about 40 % of the moving frames of
  the second half of the drive. The isolation shown in simulation is therefore available on the car only part
  of the time (section 2.3).
* **Position on the car is not better than mapd's filter in the median**, and its stated uncertainty is still
  optimistic on 104128 and 113201 (section 3).
* **A gentle re-aiming of the camera at standstill is not detected**; the one recorded (14:40) is seen only at
  the next drive. Its size cannot be estimated from the road model's outputs at all, only from sign rows.
* Receiver faults (jump, drift, delayed fixes, swapped axes) are detected and named but the position stays
  confidently wrong for 35-47 s of a 55 s fault: there is one absolute source. On the real recording the
  position under receiver and map faults is *worse* than L38's (section 2.3): not diagnosed.
* Cost is not lower than L38's (section 7).

## 1. What was wrong in the estimator, and what was changed

Found while reproducing L38's numbers (each with the experiment that showed it; scripts in section 10):

1. **The motorway failure was mostly two defects, not the delay ambiguity itself.** (a) The simulated receiver
   emitted fixes that describe a time before the simulated world starts; `truth_at` clamps, so the first fixes
   were 0.4 s old instead of 2.2 s, and the anchor put the car 32 m ahead. (b) `_anchor` gave the position
   7 m of uncorrelated uncertainty although it is "fix + speed x delay": with a delay prior of +- 1.5 s at
   20 m/s that is 30 m along the track, fully correlated with the delay state. The filter then resolved the
   32 m by moving the delay to 3.8 s (stated +- 0.3) and stayed there. Fixed: the simulator, and the anchor
   covariance (delay along the track, heading across, with their correlations). The delay and the along-track
   position are now uncertain *together* until the speed changes, and the covariance says so.
2. **Scale states dropped their uncertainty below a fixed floor.** Below 0.05 rad/s the gyro scale's Jacobian was
   zeroed: the heading in a long motorway bend (0.03-0.05 rad/s) did not contain the scale's uncertainty. The
   regressor is now shrunk by its own uncertainty (zero inside 3 sigma or 0.02 rad/s).
3. **All yaw scales walked together** (to -0.25, stated +- 0.03) when no absolute source may teach them (the
   real car's configuration): nothing in a yaw sensor says what its scale is. The gyro's scale is now a
   consider state of the gyro's own row; the others are learned relative to it.
4. **The camera "learned" its own scale without a reference**, by two routes: the mount rows used the speed
   *state* as regressor (`vy = v x dyaw` is a speed measurement), and standstill rows cashed in correlations
   built at speed. Camera-only speed was biased low by 0.5-0.9 m/s at 8-16 m/s (section 4). Fixed: mount rows
   use the pose's own `vx`; standstill rows do not correct calibration states; "a reference" means another
   source holding the speed of a *moving* car.
5. **The camera's speed error is not white**: correlation 0.80 at one frame, 0.42 at 1 s, 0.21 at 3 s. The
   model had 75 % of the sigma white. Refitted (two Gauss-Markov states, 0.45 m/s / 3.5 s and 0.38 m/s /
   0.35 s) and the fused sigma covers the tails (x 1.35).
6. **Isolation was speed-only, rank-or-vote, group-wide.** Now: self-evident faults need no rank; faults are per
   (source, quantity); the wheel difference is a yaw sensor of its own (three yaw opinions); the receiver's
   track is a third speed opinion; a pending vote publishes the speed as `disputed` with a std that covers both
   readings instead of believing rank at once; noise bursts are weighted by their measured noise.
7. **The mount model did not follow the hardware** (section 5).

Dead ends, kept in the code comments: a consider state below the floor with the plain regressor (the scales
drifted through the cross-covariances); coloured-noise states scaled by the speed-dependent sigma (a second
scale factor: the camera's scale came out wrong by 8 %); a handled-camera detector at 0.01 rad/s (3-4 false
events per healthy drive).

## 2. Fault detection and isolation

### 2.1 What was built

* `chris_integrator/sim/faults.py`: fault injection into any message stream (simulator or recording): bias,
  scale, drift, stuck, repeat, drop-out, noise, outliers, delay of the content, stamp offset, epoch, stale after
  silence, sign, unit, frame, mount change, ghosts, a lying std; any combination.
* `tools/fault_campaign.py`: `sim` (known truth), `real` (the unfaulted run of the same recording is the
  reference), `healthy` (false alarms), `show` (before / after). `--tree` runs another checkout under the same
  faults.
* `chris_integrator/core/fdi.py` and the changes of `monitors.py` / `integrator.py`: DESIGN.md section 5a.
* `chris-integrator/tests/test_faults.py` (7 tests) is the regression subset; `chris-integrator/docs/FAULTS.md`
  is the full table (generated).

### 2.2 The table (simulation, town, seed 1; fault from 95 s to 150 s)

Columns as in `docs/FAULTS.md`: seconds to the first alarm; isolation (`ok`, `named (warning)`, `none`,
`WRONG`); seconds an output was valid and wrong beyond its tolerance and beyond 3 of its own sigma.

| fault | before: detected after (s) | isolated | wrong-but-valid | after: detected after (s) | isolated | wrong-but-valid | largest such error | recovered after (s) |
|---|---|---|---|---|---|---|---|---|
| vehicle speed: bias +2 m/s | 12.4 | none | position 32 s, speed 55 s | 2.9 | ok + gnss_track | position 20 s, speed 6 s | position 40.3, speed 2.09 | 32 |
| vehicle speed: scale 0.7 | 1.9 | WRONG: vision | position 43 s, speed 55 s | 0.6 | ok + gnss_track | position 10 s, speed 1 s | position 27.6, speed 2.83 | 36 |
| vehicle speed: scale 0.9 | 26.4 | none | position 20 s, speed 55 s | 1.9 | ok + gnss_track | position 40 s, speed 5 s | position 26.3, speed 1.45 | 35 |
| vehicle speed: slow scale drift to +17 % | 57.5 | none | position 14 s, speed 44 s | 31.0 | ok + gnss_track | position 5 s, speed 30 s | position 16.1, speed 1.81 | 32 |
| vehicle speed: stuck | 10.0 | WRONG: vision | position 45 s, speed 49 s | 1.6 | ok + gnss_track | position 6 s | position 33.5 | - |
| vehicle speed: each sample repeated 5x | - | none | 0 | 2.5 | named (warning) | 0 | - | 29 |
| vehicle speed: noise burst 1 m/s | 4.5 | named (warning) | speed 18 s | 0.4 | ok | speed 3 s | speed 1.47 | - |
| vehicle speed: outliers 10 % of 5 m/s | 15.6 | named (warning) | speed 16 s | 0.6 | ok | speed 13 s | speed 3.62 | - |
| vehicle speed: unit (km/h read as m/s) | 1.8 | WRONG: vision | heading 2 s, position 53 s, speed 57 s, yaw_rate 1 s | 0.5 | ok + gnss_track | position 12 s, speed 0 s | position 251, speed 19.7 | - |
| vehicle speed: sign | 1.8 | WRONG: vision | heading 3 s, position 48 s, speed 57 s | 0.5 | ok + gnss_track | position 12 s, speed 0 s | position 205, speed 18.2 | - |
| vehicle speed signal only: scale 0.7 (wheels right) | 1.7 | ok + vision | heading 0 s, position 2 s, speed 3 s | 0.6 | ok + gnss_track | position 6 s, speed 1 s | position 27.6, speed 2.83 | 36 |
| vehicle wheels only: scale 0.8 | 1.7 | ok | 0 | 1.0 | ok + gnss_track | 0 | - | 35 |
| gyro: bias 0.02 rad/s | 14.2 | none | heading 21 s, position 10 s, yaw_rate 2 s | 1.4 | ok | heading 14 s, yaw_rate 2 s | heading 9.36, yaw_rate 0.02 | 1 |
| gyro: scale 1.3 | 43.8 | none | heading 19 s, position 12 s, yaw_rate 14 s | 36.9 | ok | heading 19 s, position 16 s, yaw_rate 14 s | heading 17.3, position 22.9, yaw_rate 0.04 | 12 |
| gyro: slow bias drift to 0.03 rad/s | 33.8 | none | heading 20 s, position 13 s, yaw_rate 5 s | 21.0 | ok | heading 18 s, position 15 s | heading 13.6, position 24.4 | 39 |
| gyro: stuck | 37.9 | none | heading 18 s, position 14 s, yaw_rate 15 s | 2.1 | ok | heading 14 s, position 14 s, yaw_rate 1 s | heading 46.5, position 157, yaw_rate 0.11 | - |
| gyro: noise burst 0.03 rad/s | 6.2 | none | heading 17 s, position 13 s, yaw_rate 34 s | 0.4 | named (warning) | yaw_rate 1 s | yaw_rate 0.06 | 7 |
| gyro: sign | 36.4 | named (warning) | heading 17 s, position 14 s, yaw_rate 15 s | 34.8 | ok | yaw_rate 1 s | yaw_rate 0.13 | - |
| gyro: unit (deg/s read as rad/s) | 1.8 | named (warning) | heading 57 s, lead 2 s, position 50 s, yaw_rate 54 s | 0.3 | ok | 0 | - | - |
| accelerometer: bias 1 m/s2 | 27.4 | none | 0 | 31.5 | none | 0 | - | - |
| accelerometer: sign | 10.8 | named (warning) | 0 | 10.9 | named (warning) | 0 | - | 45 |
| accelerometer: stuck | - | none | 0 | 2.1 | named (warning) | 0 | - | 6 |
| vehicle feed: drop-out | 0.6 | ok | 0 | 0.6 | ok | 0 | - | 0 |
| vehicle feed: silent 7 s, 3 stale samples on return | 0.6 | ok | 0 | 0.6 | ok | 0 | - | 18 |
| vehicle feed: content 0.5 s old (delay jump) | - | none | speed 3 s, yaw_rate 3 s | 49.9 | named (warning) | speed 3 s, yaw_rate 3 s | speed 0.7, yaw_rate 0.07 | 0 |
| vehicle feed: stamps 0.5 s late | 0.5 | WRONG: vision | speed 13 s, yaw_rate 12 s | 0.5 | WRONG: vision | speed 13 s, yaw_rate 12 s | speed 0.62, yaw_rate 0.06 | 10 |
| vehicle feed: stamps 0.5 s early | - | none | 0 | 0.7 | named (warning) | 0 | - | 4 |
| camera speed: scale 0.3 | 1.8 | ok | 0 | 0.5 | ok + gnss_track | 0 | - | 35 |
| camera speed: scale 0.85 | - | none | 0 | 7.8 | ok | 0 | - | 1 |
| camera speed: bias +1.5 m/s | - | none | 0 | 1.1 | ok | 0 | - | 5 |
| camera speed: slow scale drift to -17 % | - | none | 0 | 30.6 | named (warning) | 0 | - | 1 |
| camera speed: stuck | 10.5 | ok | 0 | 1.5 | ok + gnss_track | 0 | - | 35 |
| camera speed: noise burst 2 m/s | 5.2 | named (warning) | 0 | 3.4 | named (warning) | 0 | - | 0 |
| camera speed: noise 1.5 m/s and stated std x 0.2 (lies) | 5.3 | named (warning) | 0 | 6.6 | named (warning) | 0 | - | 0 |
| camera yaw rate: sign | 36.4 | named (warning) | 0 | 34.8 | ok | 0 | - | - |
| camera yaw rate: scale 1.5 | 38.6 | named (warning) | heading 5 s | 40.1 | named (warning) | 0 | - | 0 |
| camera feed: drop-out | 0.5 | ok | 0 | 0.7 | ok | 0 | - | 0 |
| camera feed: content 0.3 s old (delay jump) | - | none | 0 | - | none | 0 | - | 0 |
| camera feed: stamps 0.3 s late | - | none | 0 | 0.9 | named (warning) | 0 | - | 5 |
| camera mount: pitch step 1.7 deg | 3.1 | named (warning) | 0 | 0.6 | named (warning) | 0 | - | 0 |
| camera mount: pitch step 8 deg (model out of range) | 1.8 | ok | lead 2 s | 0.4 | ok | lead 2 s | lead 52.1 | 0 |
| camera mount: slow pitch creep 0.03 deg/s | 49.0 | named (warning) | 0 | 29.3 | named (warning) | 0 | - | 9 |
| camera lead: range scale 1.3 | - | none | lead 44 s | - | none | lead 44 s | lead 100 | 0 |
| camera lead: phantom lead at 12 m | - | none | lead 55 s | - | none | lead 55 s | lead 64.7 | 1 |
| receiver: position jump 30 m | 4.4 | named (warning) | position 48 s | 0.4 | ok | position 47 s | position 28.3 | 32 |
| receiver: slow position drift 1 m/s | 39.8 | none | position 26 s | 36.4 | ok | position 35 s | position 40.6 | 32 |
| receiver: noise burst 15 m | 7.3 | named (warning) | 0 | 0.4 | ok | 0 | - | - |
| receiver: outliers 20 % of 60 m | - | none | 0 | 0.4 | ok | 0 | - | - |
| receiver: frozen fix while moving | 5.4 | ok | 0 | 5.4 | ok | 0 | - | 35 |
| receiver: lat / lon swapped (frame) | 5.4 | named (warning) | position 42 s | 3.4 | ok | position 39 s | position 508 | - |
| receiver: noise 15 m and stated accuracy x 0.3 (lies) | 7.3 | named (warning) | 0 | 0.4 | ok | 0 | - | - |
| receiver: drop-out | 4.4 | ok | 0 | 4.3 | ok | 0 | - | 0 |
| receiver: fixes 2 s older (delay jump) | 5.4 | named (warning) | position 38 s | 6.4 | ok | position 36 s | position 30.5 | 35 |
| map: matcher on a parallel road 25 m away | 4.3 | named (warning) | 0 | 4.3 | named (warning) | 0 | - | 24 |
| map: bearing turned 90 deg (frame) | - | none | 0 | - | none | 0 | - | 0 |
| clock: epoch change (all stamps step back 600 s) | -0.1 | WRONG: vehicle,wheels | 0 | -0.1 | ok + vehicle,wheels | 0 | - | 0 |
| two together: vehicle and camera speed x 0.7 | 2.2 | named (warning) | position 43 s, speed 55 s | 1.9 | WRONG: gnss_track | position 43 s, speed 55 s | position 66.7, speed 4.3 | 32 |
| two together: vehicle speed stuck, receiver out | 4.4 | ok + vision | position 54 s, speed 49 s | 1.6 | ok | position 6 s | position 32.9 | - |
| two together: camera out of range, receiver jump 30 m | 1.8 | ok | lead 2 s, position 58 s | 0.4 | ok | position 57 s | position 28.3 | 0 |

before: 59 faults: detected 46, isolated correctly 12, wrong source blamed 6, no output confidently wrong 29; confidently-wrong output time in all 1657 s

after: 59 faults: detected 55, isolated correctly 38, wrong source blamed 2, no output confidently wrong 31; confidently-wrong output time in all 748 s

With a front radar (radar faults only; the radar rows were not worked on in this lane):

| fault | before: detected after (s) | isolated | wrong-but-valid | after: detected after (s) | isolated | wrong-but-valid | largest such error | recovered after (s) |
|---|---|---|---|---|---|---|---|---|
| radar: range bias 3 m | - | none | 0 | - | none | 0 | - | 0 |
| radar: range scale 1.1 | - | none | lead 1 s | - | none | lead 1 s | lead 84.8 | 0 |
| radar: range rate bias 1 m/s | 3.7 | ok | lead 4 s | 2.1 | ok | 0 | - | 33 |
| radar: mount knocked 3 deg | 8.8 | named (warning) | 0 | 8.8 | named (warning) | 0 | - | 0 |
| radar: azimuth sign (frame) | 37.9 | named (warning) | lead 10 s | 37.8 | named (warning) | lead 8 s | lead 79.1 | 2 |
| radar: drop-out | 0.4 | ok | 0 | 0.6 | ok | 0 | - | 15 |
| radar: content 0.2 s old | 13.7 | named (warning) | lead 1 s | 13.4 | named (warning) | lead 2 s | lead 79.2 | 0 |
| radar: ghosts | 3.6 | ok | lead 29 s | 1.6 | ok | lead 18 s | lead 77.2 | 53 |

before: 8 faults: detected 6, isolated correctly 3, wrong source blamed 0, no output confidently wrong 3; confidently-wrong output time in all 45 s

after: 8 faults: detected 6, isolated correctly 3, wrong source blamed 0, no output confidently wrong 4; confidently-wrong output time in all 29 s

With a radar, "vehicle speed x 0.7" is isolated in 0.6 s with 1 s of wrong speed (vote of camera and radar).

### 2.3 On the real recordings

Faults injected into 104128, seconds 260-560, fault from 355 s to 410 s (the car moves throughout the fault);
the reference is the unfaulted run of the same slice. This tests detection and the damage to the outputs on
real noise; it rests on one 5-minute slice of one drive.

| 59 faults on the recording | before (L38) | after (L41) |
|---|---|---|
| the injected source taken out | 18 | 31 |
| no output confidently wrong | 25 | 29 |
| time an output was valid and confidently wrong, all faults | 1276 s | 664 s |
| ... by output: speed / yaw rate / heading / position / lead | 432 / 124 / 228 / 430 / 62 s | 135 / 9 / 93 / 366 / 62 s |

The gain is in speed, yaw rate and heading. **Position under receiver and map faults is worse than before on
this slice**: a 30 m jump 0 -> 6 s, a slow drift 0 -> 31 s, fixes 2 s older 0 -> 42 s, the matcher on a
parallel road 1 -> 40 s, a turned map bearing 20 -> 36 s. Not diagnosed; the candidates are the wider anchor
covariance and the re-anchoring (the estimate now follows the receiver more readily). The lead is untouched
(one sensor). The detection-time and "wrong source" columns of the tool are not given for the recording: on
this slice the receiver's speed witness goes stale 3 s into the window in every faulted run, the scoring
counts that as the first alarm and as a blamed source, and the unfaulted run does not show it (not
understood); "the injected source taken out" does not depend on it.

Rows where either run had an output confidently wrong (the other 20 faults had none in both):

| fault | before: injected source taken out | wrong-but-valid | after: injected source taken out | wrong-but-valid |
|---|---|---|---|---|
| vehicle speed: bias +2 m/s | no | position 32 s, speed 55 s | no | position 30 s, speed 18 s |
| vehicle speed: scale 0.7 | no | position 18 s, speed 55 s | yes | heading 5 s, position 26 s, speed 4 s |
| vehicle speed: scale 0.9 | no | position 13 s, speed 55 s | no | position 21 s, speed 21 s |
| vehicle speed: slow scale drift to +17 % | no | position 1 s, speed 44 s | no | position 14 s, speed 28 s |
| vehicle speed: stuck | no | speed 4 s | yes | speed 1 s |
| vehicle speed: noise burst 1 m/s | no | speed 20 s | yes | speed 2 s |
| vehicle speed: outliers 10 % of 5 m/s | no | position 10 s, speed 15 s | yes | speed 12 s |
| vehicle speed: unit (km/h read as m/s) | no | position 17 s, speed 58 s | yes | position 13 s, speed 0 s |
| vehicle speed: sign | no | heading 9 s, position 17 s, speed 57 s | yes | position 13 s, speed 0 s |
| vehicle speed signal only: scale 0.7 (wheels right) | yes | position 1 s, speed 3 s | yes | position 4 s, speed 1 s |
| gyro: bias 0.02 rad/s | no | heading 30 s, position 20 s, yaw_rate 8 s | yes | heading 13 s, position 8 s |
| gyro: scale 1.3 | no | heading 14 s, position 16 s | no | 0 |
| gyro: slow bias drift to 0.03 rad/s | no | heading 22 s, position 14 s, yaw_rate 10 s | yes | heading 7 s, position 6 s |
| gyro: stuck | no | heading 20 s, position 13 s, yaw_rate 10 s | yes | heading 12 s, position 8 s, yaw_rate 1 s |
| gyro: noise burst 0.03 rad/s | no | heading 29 s, position 37 s, yaw_rate 30 s | no | heading 24 s, position 18 s, yaw_rate 4 s |
| gyro: sign | no | heading 24 s, position 19 s, yaw_rate 17 s | yes | heading 10 s, position 10 s, yaw_rate 3 s |
| gyro: unit (deg/s read as rad/s) | no | heading 44 s, lead 1 s, position 23 s, yaw_rate 48 s | yes | heading 0 s, yaw_rate 1 s |
| vehicle feed: drop-out | yes | speed 5 s | yes | 0 |
| vehicle feed: silent 7 s, 3 stale samples on return | yes | speed 1 s | yes | 0 |
| vehicle feed: content 0.5 s old (delay jump) | no | yaw_rate 1 s | no | 0 |
| vehicle feed: stamps 0.5 s late | no | yaw_rate 1 s | no | 0 |
| camera speed: scale 0.3 | yes | heading 7 s, position 14 s | yes | 0 |
| camera yaw rate: scale 1.5 | no | heading 18 s, position 14 s | no | heading 10 s, position 10 s |
| camera feed: drop-out | no | position 10 s | no | 0 |
| camera mount: pitch step 8 deg (model out of range) | yes | heading 9 s, lead 2 s, position 14 s | yes | lead 3 s |
| camera mount: slow pitch creep 0.03 deg/s | no | 0 | no | position 1 s |
| camera lead: range scale 1.3 | no | lead 0 s | no | lead 0 s |
| camera lead: phantom lead at 12 m | no | lead 56 s | no | lead 55 s |
| receiver: position jump 30 m | yes | 0 | yes | position 6 s |
| receiver: slow position drift 1 m/s | yes | 0 | yes | position 31 s |
| receiver: outliers 20 % of 60 m | yes | 0 | yes | position 2 s |
| receiver: frozen fix while moving | yes | position 26 s | yes | 0 |
| receiver: drop-out | yes | position 26 s | yes | 0 |
| receiver: fixes 2 s older (delay jump) | yes | 0 | yes | position 42 s |
| map: matcher on a parallel road 25 m away | no | position 1 s | no | position 40 s |
| map: bearing turned 90 deg (frame) | no | heading 2 s, position 20 s | no | position 36 s |
| two together: vehicle and camera speed x 0.7 | no | position 18 s, speed 55 s | no | heading 12 s, position 25 s, speed 46 s |
| two together: vehicle speed stuck, receiver out | yes | position 34 s, speed 4 s | yes | speed 1 s |
| two together: camera out of range, receiver jump 30 m | yes | heading 1 s, lead 2 s | yes | lead 3 s |

Source states on the whole healthy drives (`integrator_eval.py real`), share of the frames with the vehicle
>= 3 m/s:

| session | camera: fault / suspect / ok | vehicle: suspect | receiver's track: fault / ok / silent or unchecked | L38: camera fault / suspect |
|---|---|---|---|---|
| 104128 healthy | **0.8 %** / **20.5 %** / 78.7 % | 12.1 % | 27.2 % / 67.5 % / 5.3 % | 0 / 0.3 % |
| 113201 healthy | **0.7 %** / 3.9 % / 94.2 % | 1.2 % | 0 / 50.6 % / 49.4 % | 0 / 2.4 % |
| 151146 incident | 99.4 % / 0.5 % / 0.1 % | 0.1 % | 11.6 % / 72.3 % / 16.0 % | 99.0 % / 0.9 % |
| 153636 incident | 100 % / 0 / 0 | 0 | 0 / 69.1 % / 30.9 % | 100 % / 0 |

Reading: the incident is detected as before. On the healthy drives the camera is now falsely faulted in under
1 % of the moving frames and suspected far more often than in L38; the reasons seen on 104128 are the yaw-rate
pair between the wheel difference and the camera (a monitor that did not exist), the mount's yaw residual, and
three short false faults (one by the vote of the receiver's track, one by the CUSUM against the wheels, one by
the old share test). The receiver's track is itself "faulted" (in disagreement with the vehicle, blamed by
rank) in 27 % of the moving frames of 104128: on this receiver it is a weak third opinion.

### 2.4 Not detectable with the present sensors

| fault | why | what would make it detectable |
|---|---|---|
| which of two is wrong, when only two measure it (camera-less or receiver-less car; any lead range) | no third opinion | a third source |
| vehicle and camera speed wrong the same way | they outvote the truth; the receiver's track is blamed (table: WRONG) | a second absolute speed (radar on stationary objects) |
| a persistent offset, slow drift or delay change of the receiver | one absolute source; detected, but the position follows it | a second receiver; landmarks on the map |
| swapped / mirrored receiver axes | distances stay right | as above |
| camera lead range scale, phantom lead | one sensor reports the lead | a radar |
| a gyro frozen within its noise of zero, a gyro sign error, on a straight road | nothing turns: found at the first bend (heading wrong by then) | nothing; time |
| a delay of the whole vehicle feed under fresh stamps | camera and vehicle disagree in time, not which is late (named after 50 s by a trend) | `age_ms` from the feeder |
| the vehicle feed's stamps 0.5 s late | the same; here the *camera* is blamed (table: WRONG) | as above |
| accelerometer bias | it is road grade | an altitude source |
| radar range bias / scale | the camera's range is learned from the radar | a second radar or lidar |

## 3. Position

Simulation (`integrator_eval.py sim`, after the first 30 s; NEES of the position has 2 degrees of freedom):

| scenario, seed | before: heading rms, NEES | position rms, NEES | after: heading rms, NEES | position rms, NEES |
|---|---|---|---|---|
| town 1 | 0.32 deg, 0.32 | 1.02 m, 0.30 | 0.45, 0.24 | 1.17, 0.35 |
| town 2 | 0.28, 0.22 | 5.02, 8.60 | 0.26, 0.16 | 2.29, 1.22 |
| town 3 | 0.38, 0.41 | 2.43, 2.53 | 0.35, 0.24 | 1.59, 0.77 |
| motorway 1 | 2.25, 16.9 | 17.0, 12.2 | 0.30, 0.28 | 4.26, 0.74 |
| motorway 2 | 2.27, 22.0 | 22.0, 23.2 | 0.54, 1.16 | 6.75, 2.00 |
| motorway 3 | (not run in L38) | | 0.66, 0.88 | 9.66, 4.02 |

The motorway rms contains the honest first stretch (along-track uncertainty 30 m until the speed changes).
Calibration states are now inside their stated uncertainty in both scenarios (gyro scale 0.038-0.056 +- 0.025
in town against 0.04; in L38's motorway runs it read -0.06 +- 0.02). Still open in simulation: motorway seed 3
(NEES 4.0); in the real car's configuration (`gnss_absolute` off) the motorway heading is 1.2-1.3 deg with
NEES 4-6 and the wheel and camera yaw scales read -0.2 +- 0.03 (experiment of section 10, not in a table tool).

On the car (`integrator_eval.py real`: every second fix held out; `integrator_eval.py position`: the same
fixes to mapd's engine without a map pack and to the integrator without its map source):

| session | L38: held-out fix to the estimate p50 / p90, normalised (expect 2) | L41, with map | L41 without map: p50 / p90, inside the 95 % bound | mapd's filter: p50 / p90, inside the 95 % bound |
|---|---|---|---|---|
| 104128 | 15.7 / 55.8 m, 5.5 | 15.9 / 59.6 m, 8.7 | 10.3 / 45.3 m, 0.71 | 11.2 / 63.6 m, 0.72 |
| 113201 | 61.9 / 213.9 m, 73.6 | 22.0 / 62.6 m, 12.9 | 20.3 / 58.8 m, 0.54 | 23.1 / 91.5 m, 0.56 |
| 151146 | 7.6 / 18.7 m, 3.2 | 7.2 / 19.0 m, 3.1 | 5.8 / 18.5 m, 0.88 | 5.4 / 23.8 m, 0.82 |
| 153636 | 9.2 / 16.9 m, 2.5 | 9.2 / 17.2 m, 2.8 | 7.6 / 14.3 m, 0.89 | 4.3 / 22.8 m, 0.77 |

Reading. Against mapd's filter: the same in the median on two drives, better on the poor-reception drive,
worse on 153636 (7.6 against 4.3 m); the tail (p90) is shorter on all four; honesty is equal on two and better
on two. So: **not worse, better in the tail, not better in the median**. There is no truth on the car: this
measures who predicts a held-out fix of the same receiver, and a filter that follows the receiver closely
(mapd's) is favoured by that. Caveat: mapd's engine rejected 60-363 of the converted motion samples per drive (on 153636: 156 of 4346,
all "speed_mps out of range": the relay's negative speeds while reversing). The stated uncertainty remains optimistic on 104128 and 113201 (normalised 9-13 against 2). The
receiver-delay state does not settle to one value across drives on the car (1.3, 3.7, 0.9, 1.3 s; the
displacement lag measured in L38 was 1.75-2.5 s): on this receiver the delay is not one number, and the
position inherits that.

## 4. Camera-only speed (vehicle held out; `integrator_eval.py heldout`, `characterise`)

| session | n | L38: error / stated std (rms), inside 2 sigma, filtered rms / raw rms | L41 | yaw rate error / std: L38, L41 |
|---|---|---|---|---|
| 104128 | 9970 | 1.56, 0.796, 0.753 / 0.651 m/s | 0.75, 0.980, 0.589 / 0.651 | 2.20, 1.18 |
| 113201 | 2174 | 1.39, 0.868, 0.587 / 0.629 | 0.90, 0.972, 0.601 / 0.629 | 2.34, 1.69 |

The stated uncertainty is now honest to conservative, and the filtered value is better than the raw one on
both drives. Most of what L38 lost was a bias: the camera's scale state had walked to +0.06-0.07 and the
coloured state to 0.6 sigma (defect 4 of section 1), so the estimate read 0.5-0.9 m/s low at 8-16 m/s. The
noise model is fitted on the same two drives it is checked on (12 000 frames in all; there is no third
vehicle-fed healthy drive to hold out). The yaw rate's stated std is still too small by 1.2-1.7 x: its error
is heavy-tailed (rms three times the robust sigma) and only its floor was raised.

## 5. The camera mount

### 5.1 What the data says happened (L41-M, checked here where it steers a statement)

| time | what | evidence |
|---|---|---|
| 10:41-11:47, two drives | calibration in use (0, 0); the camera sits at pitch +0.054-0.056, yaw -0.060 against it; the model works (speed ratio 1.00) | estimator, pose direction; lane-line direction agrees |
| 13:06-13:14, parked | calibration (0.0561, -0.0621) set: exactly that offset | config; not recorded |
| 13:56-14:40, parked | model looks as before (lead at 6.4-7 m, p 0.83) | parked data only |
| **14:40:00-14:40:26, parked** | **the camera is handled**: pitch rate up to 0.038 rad/s in 3-4 jerks (elsewhere in that 54-minute session below 0.0011 rad/s in 99.9 % of the frames), lead lost (7 m -> 16 m, p 0.85 -> 0.31), stated stds doubled, image brightness steps | `road.pose`, `env` |
| 14:51-15:11, parked | the degraded state continues | |
| 15:13-15:16 | the car is manoeuvred (gear D and R, speeds up to 0.5 m/s and more in the vehicle feed) | L41-M read this as a second handling "while the car stands"; the vehicle feed says the car moved, so **a second handling is not established** |
| from 15:17:47, two drives | model speed 0.08-0.28 of the vehicle's from the first moving window; phantom lead at 5 m; pose direction reads "aligned" (+0.009, -0.001) | |
| the same drives | **sign detections sit 284-290 px lower in the 1280x720 frame than in the morning, at equal box size** (three size classes, 75 tracks; one sign location seen in both periods: +273 to +304 px, horizontally 0 to +18 px) | `signs` boxes, raw frame, independent of the road model |

So: the camera was re-aimed upwards. 290 px at focal length 975 px is **16.6 degrees**. The owner's 140 px
between the 13:57 and the 14:52 pictures is 8.2 degrees if measured on the full 1280x720 frame. The two numbers
do not agree and the recordings do not say why: either the pictures were measured at half size, or about half
of the change happened outside what they bracket (nothing recorded between 11:47 and 13:56 while the phone was
worked on; a second handling after 14:52 is not established, see above). The frames on the phone
(`.../20261119-151146/frames` against the morning's) would settle it; the phone was unreachable. Yaw: no sign
of a change (horizontal shift of the same sign 0 to +18 px, median +1 px: under one degree, one location).
Roll: the data does not decide.

The incident is therefore the re-aiming, not the calibration applied at 13:11 (that calibration removes the
morning's offset, and the model ran with that very offset at ratio 1.00). With the true horizon 16 degrees
off, the model's input window lies at or above it; what the model does there (collapsed speed, exploded std,
a pose direction that returns to "aligned") is measured, not predicted by anything in the repository: no
perturbation experiment beyond 2 degrees exists.

### 5.2 What the estimator should have reported, and what it reports now

`integrator_eval.py mount`: the day's sessions through one integrator (a restart is a clock epoch; calibration
states and the sign-row reference are kept).

| stage | should have | L41 reports |
|---|---|---|
| morning drives | residual pitch +0.055, yaw -0.060 against calibration (0, 0); suggest that calibration | pitch +0.0562 +- 0.0011 / yaw -0.0598 (104128), +0.0540 +- 0.0007 / -0.0604 (113201); no pitch step, no handling |
| calibration changed (13:11) | the residual carried over: about 0 against the new calibration; unverified until driven | -0.0021 / +0.0017 +- 0.0055, `converging` (L38 reset the residual to its prior here) |
| 14:40 handling | "camera handled, pitch unknown" | **nothing** (section 8) |
| from 15:17:47 | camera faulted; mount unknown; "model out of range, looks like a large pitch change"; then the size | camera faulted in 99.4 % / 100 % of the moving frames; mount `unknown`; finding `model_out_of_range` at the first moving frames; from sign rows, once 8 tracks per size class are in (455 s into 151146): **"signs sit 290 px lower ...: camera pitch changed by about 17 deg up"**; 284 px / 16 deg on 153636 |

A pitch step the model survives (simulation, 0.03 rad = 1.7 deg at 9 m/s): found after 0.6 s, state
`pitch_changed`, then "camera pitch changed by +1.83 deg (+0.0319 rad, +- 0.15 deg)", the yaw residual
unmoved; a creep of 0.03 deg/s is named by a trend after 29 s. A step of the incident's size is detected and
described, and estimated only through the sign rows, which have their own limits: about 1.5 degrees of
scatter between the reference and later healthy driving (25-27 px on the morning drives), ten tracks per
size class before they speak, a reference that must have been learned before the change, and no check on
anything but the two days' roads. They are not in the simulator.

## 6. Object layer on real drives (sub-lane L41-O)

Injector artefact and tracker fault are separated (`tools/integrator_objects_eval.py`; every lead-id change
and every radar-only lead frame is classified by cause from the injected detections' tags, which the tracker
never reads).

| | L38 | injector fixed, tracker as it was | both fixed (sub-lane's branch) | on this branch | as recorded |
|---|---|---|---|---|---|
| 104128 + radar: lead track ids | 112 | 107 | 54 | 60 | 39 |
| ... lead where no vehicle was injected | 1.48 % | 1.16 % | 0.56 % | 0.73 % | 0.01 % |
| 151146 + radar, no vehicle: radar-only lead | 8.4 % of frames | 2.2 % | 0 | 0.05 % (9 frames at 120 m) | |
| 113201 + two radars + second camera: ids | 41 | 57 | 24 | (not re-run) | 26 |

Of the 8.4 %: 955 frames were the injector (roadside objects placed straight ahead of the pose at spawn time
ended up on the road in bends), 536 the tracker (a sticky "has moved" flag, and a gap between the stationary
and the moved thresholds). Of the 112 ids: about 70 changes involved a roadside object, 3 were one vehicle
getting a new id. Injector: roadside objects placed beside the whole dead-reckoned path; the pseudo-truth cut
at the camera's distance steps and joined across gaps of up to 3 s. Tracker: "has moved" needs 2 sigma above
the threshold for 0.5 s with the ego-motion errors counted; a track no camera designates is a lead candidate
only if it moved or a camera's road model once called it a vehicle; on a merge the lead's id survives; the
camera adopts the track a radar holds also on re-acquisition; a camera track takes its first radar range only
within max(3 m, 25 %). Simulation (`sim --radar --cam2`, 2 seeds): tracked 0.999, false leads 0.7 % / 0.3 %
-> 0.5 % / 0.3 %, everything else identical. The difference between the sub-lane's branch and this one
(54 -> 60 ids, 0 -> 0.05 %) comes from the changed ego estimate under the same tracker; not looked into.
Open: a cut-in adopting a stationary radar track (16 frames); the camera range scale learned from wrong pairs
at standstill (1.3); the pseudo-truth is the camera's own smoothed lead, so this shows association and
continuity, not accuracy; no real radar.

## 7. Cost

Laptop, one process, `time.process_time` over seconds 700-1300 of 104128 (600 s of driving; vehicle, camera,
receiver, map, signs; 7280 road frames; every input included), the two trees measured alternately. The machine
was shared with other lanes (load average 5), so the scatter is about 15 %:

| | L38 | L41 |
|---|---|---|
| ms per road frame, runs in the order measured | 2.38, 2.03 | 2.28, 2.23 |
| state size | 25 | 26 |

Earlier in the lane, on a quieter machine: 2.15 (L38) against 2.10 (L41, before the last detectors). The whole
drive with `integrator_eval.py cost` was only measured under load (4.3 ms) and is not a number.

Where the time goes (profile of the same slice): the filter's row loop and `predict` about half, building
`ego_state` about a quarter before the change and less after it. Removed: dense transition products (the
transition is a 10 x 10 block and a diagonal), element-wise process noise, a symmetrisation per row, medians
on every innovation, rebuilding the health / check / calibration tables every frame (now five times a second
and whenever a fault changes; the quantities themselves are fresh every frame; `tables_age_s` says how old
the tables are). Added: one state (the camera's fast coloured error), the wheel-yaw monitor, the detectors.
Net: no saving against L38.

Phone (Poco F1): **not measured** (unreachable). Basis for an estimate, as in L37 / L38: the api's Python on
the phone runs 4-6 times slower per operation than this laptop core (itself an estimate from L37's budget
measurements, not re-measured). That gives 9-14 ms per road frame, i.e. 12-18 % of one core at
13 frames a second. The row loop is what a compiled core would replace.

## 8. What remains wrong or unverified

* False alarms on the real healthy drives (section 0, 2.3). Not acceptable as a final state: the limits of
  the pair CUSUMs, of the trend detector and of the receiver-track vote need more healthy driving than two
  drives, or the camera's false faults have to be traced one by one (three were looked at: one by vote of
  the receiver's track, one by the CUSUM against the wheels, one by the old share test at 15 samples).
* The receiver's track as a voter on this car (section 0). A source that is itself set aside now has no vote
  (last change of the lane; the sim table and the real runs were repeated with it).
* The wheel-difference yaw is set aside by vote in up to 15 % of the moving frames of 104128 (gyro and camera
  against it): a false isolation on the real car, without effect on the outputs as long as the gyro is right.
* The stream-clock detector warns about the map stream on 113201 (31 % of the moving frames): the map's rate
  is irregular.
* A gentle re-aiming at standstill (14:40) is not detected. The threshold that would catch it fired 3-4
  times per healthy drive (the model reports small rotations while the car stands in traffic).
* Mount `moved` at the end of 104128: the yaw residual is not constant on the real car (as in L38); not
  looked into.
* Position: section 3. Receiver faults: detected, not survived.
* `gyro: stuck` in the table: a value frozen within its noise of zero is not self-evident (it had to be
  exempted: real and test streams hold near-zero values legitimately); the heading is wrong for 14 s.
* Sensitivity lost to the loosened limits: a gyro bias of 0.02 rad/s is voted out after 12 s (0.04: 2 s); a
  10 % vehicle speed error keeps the position wrong for 40 s.
* Radar faults: untouched; range bias and scale are not detected.
* Two faults at once that agree: section 2.4.
* Nothing ran on the phone; frames were not looked at; the cost estimate for the phone has no new basis.
* 065629 and 161325 (camera only, other calibration / dusk) were not run.
* L37's `integrator_replay.py` checks: their equivalents were run (`detect` -> sections 2.3 and 5.2;
  `accuracy` -> section 4; `characterise` -> `integrator_eval.py characterise`; `cost` -> section 7). The
  `products` / `compare` frame-by-frame comparison of lead and lanes against L38 was **not** re-run on this
  branch; the api's 255 tests pass.

## 9. Files touched outside the lane's own paths

Own: `core/ekf.py`, `core/ego.py`, `core/monitors.py`, `sources/*`, `sim/*`, `tools/integrator_eval.py`, new
files (`core/fdi.py`, `sim/faults.py`, `sources/signrows.py`, `tools/fault_campaign.py`,
`tools/integrator_objects_eval.py`, `tests/test_faults.py`, `tests/test_objects_*.py`, `docs/FAULTS.md`).

Outside, for the merge with L39:

| file | what |
|---|---|
| `chris_integrator/core/integrator.py` | the largest: `_anchor` (covariance), `_isolate` rewritten (per-quantity faults, self-evident findings, pending vote, `_indep`, `_critical`, `_third_pending`), `_innovation`, `_aux_ix`, hooks for `fdi` in `on_message` / `_monitor` / `_new_epoch` / `_reanchor`, `noise_scale` in `_measure`, standstill rows with `freeze`, pair configuration from the source, new configuration keys (`fdi`, `vote_*`, pair CUSUM limits, `quarantine` names) |
| `chris_integrator/core/output.py` | `diagnostics`, `set_aside`, speed `mode: disputed`, tables cached at 5 Hz (`tables_age_s`), calibration entries for sources without states |
| `chris_integrator/core/objects.py` | L41-O: moved flag, lead candidates, identity on merge (`core/lead.py`, `core/registry.py` untouched) |
| `chris-integrator/tests/simrun.py`, `test_ego.py`, `test_radar.py` | truth in the estimator's frame; the source list; the vote test adapted (without a radar the vehicle is now isolated by the receiver's track: a change by design) |
| `chris-integrator/docs/DESIGN.md`, `README.md` | sections 2, 5a; limits |

New output fields, additive: `ego_state.diagnostics[]`, `ego_state.tables_age_s`, `sources.<g>.set_aside[]`,
`speed.mode = "disputed"`, `calibration.vision.{pitch_steps, last_pitch_step, handled, freedom}`,
`calibration.vision.mf`, `calibration.signrows`, sources `gnss_track` and `signrows`. Row classes `mount` ->
`mount_yaw`, `mount_pitch`. `sources.wheels` now also carries the fused wheel-difference yaw rate.

## 10. Reproducing

```sh
cd ~/projects/wt/roadsense-l41; D=~/datasets/roadsense-rec; P=$D/trip-pos/pos_20261119.txt; L38=~/projects/wt/roadsense-l38; O=/tmp/l41; mkdir -p $O
PP=$(python3 -c "import pytest,os;print(os.path.dirname(os.path.dirname(pytest.__file__)))"); PY=~/projects/roadsense/.venv-models/bin/python
PYTHONPATH=$PP $PY -m pytest -q -p no:cacheprovider chris-integrator/tests     # 39 passed
PYTHONPATH=$PP $PY -m pytest -q -p no:cacheprovider api/tests                  # 255 passed
(cd mapd && PYTHONPATH=$PP $PY -m pytest -q -p no:cacheprovider tests)         # 94 passed
systemd-run --user --collect -p MemoryMax=24G --wait -- bash -c "cd $PWD &&
  python3 tools/fault_campaign.py sim --out $O/after_sim.json; python3 tools/fault_campaign.py sim --tree $L38 --out $O/before_sim.json;
  python3 tools/fault_campaign.py show $O/before_sim.json $O/after_sim.json > $O/table.md;
  python3 tools/fault_campaign.py sim --radar --only radar --out $O/after_radar.json; python3 tools/fault_campaign.py sim --radar --only radar --tree $L38 --out $O/before_radar.json;
  python3 tools/fault_campaign.py healthy > $O/healthy.txt; python3 tools/fault_campaign.py healthy --tree $L38 > $O/healthy_before.txt;
  python3 tools/fault_campaign.py real $D/20261119-104128 --pos $P --from 260 --to 560 --out $O/real.json;
  python3 tools/fault_campaign.py real $D/20261119-104128 --pos $P --from 260 --to 560 --tree $L38 --out $O/real_before.json;
  python3 tools/integrator_eval.py sim --seeds 1 2 3 > $O/sim_town.txt; python3 tools/integrator_eval.py sim --scenario highway --seeds 1 2 3 > $O/sim_highway.txt;
  python3 tools/integrator_eval.py sim --radar --cam2 --seeds 1 2 > $O/sim_radar.txt;
  python3 tools/integrator_eval.py real $D/20261119-104128 $D/20261119-113201 $D/20261119-151146 $D/20261119-153636 --pos $P > $O/real.txt;
  python3 tools/integrator_eval.py position $D/20261119-104128 $D/20261119-113201 $D/20261119-151146 $D/20261119-153636 --pos $P > $O/position.txt;
  python3 tools/integrator_eval.py heldout $D/20261119-104128 $D/20261119-113201 > $O/heldout.txt;
  python3 tools/integrator_eval.py characterise $D/20261119-104128 $D/20261119-113201 > $O/characterise.txt;
  python3 tools/integrator_eval.py mount $D/20261119-104128 $D/20261119-113201 $D/20261119-135622 $D/20261119-145155 $D/20261119-151146 $D/20261119-153636 --pos $P > $O/mount.txt;
  python3 tools/integrator_objects_eval.py inject $D/20261119-104128 --pos $P --baseline > $O/obj_104128.txt;
  python3 tools/integrator_objects_eval.py inject $D/20261119-151146 --pos $P --no-lead > $O/obj_151146.txt;
  python3 tools/integrator_eval.py cost $D/20261119-104128 --pos $P > $O/cost.txt"
```

The "before" columns of sections 3 and 4 are the same `integrator_eval.py` commands in `$L38`. The
experiments of section 1 (ablation of row classes, the anchor trace, who moves the camera's scale, the
real-configuration motorway) and L41-M's scripts were one-off scripts outside the repository and are not kept
in it; what they showed is reproduced by the commands above before and after. The cost of the 600 s slice:
the integrator timed over seconds 700-1300 of 104128 (`time.process_time`, one job at a time).

## 11. Branch

`v2-l41`, head: see the lane's hand-back. Not pushed, not merged. `v2-l41-obj` (L41-O) is contained in it by
cherry-pick.
