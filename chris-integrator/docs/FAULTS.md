# The fault table

Every fault class of the taxonomy (DESIGN.md section 5a) injected into the simulated town drive (seed 1, 240 s,
the fault from 95 s to 150 s unless it is a permanent change), through the integrator as built. Reproduce with
`tools/fault_campaign.py sim` of the chris-roadsense repository; `tests/test_faults.py` asserts a subset.

Columns: **detected after**: seconds from the onset to the first alarm that the unfaulted run does not have (a
source `fault` / `suspect` / `stale`, a finding in `ego_state.diagnostics`). **isolated**: `ok` the injected
source (or one that shares its sensor) was taken out; `named (warning)` a finding names it and nothing was taken
out; `none`; `WRONG` another source was taken out. **wrong-but-valid**: seconds during which an output was
published valid and was wrong by more than its tolerance (speed 0.5 m/s, yaw rate 0.02 rad/s, position 15 m,
heading 6 degrees, lead distance 4 m) *and* by more than 3 of its own stated sigma, beyond what the unfaulted
run shows: the cost that matters. **recovered after**: seconds from the end of the fault until alarms and errors
are gone (`-`: not within the run).

"Before" is the integrator before this work (the same faults, the same scoring).

## Which detectors gate

Every detector that can set something aside runs in one of two modes (DESIGN.md section 5b): `gate` (its verdict
takes effect) or `report` (its verdict is published with the evidence and nothing is set aside). The default of
each follows one rule: **a detector gates only if it was quiet, with a stated margin, on the reference car's
healthy drives**. The table is that evidence: two drives with the car's own speed, gyro, wheels, receiver and
map (38 and 15 minutes, 8843 and 1725 camera frames with the car moving), one camera-only drive, two parked
sessions. `peak` is the largest value the detector's statistic reached as a fraction of its firing level
(1 = it fires; 0.5 = a margin of 2). It is thin data: two drives of one car on one day; a detector that gates
here has earned it on those and nothing more. Reproduce: `tools/detector_audit.py SESSION...` of the
chris-roadsense repository.

| detector | default | healthy drives: peak / level, verdicts | why |
|---|---|---|---|
| `speed.share` | gate | 0.13, 0.09; never | margin 7; the test that takes a camera out when its road model has left its range (ratio 0.04-0.4 to the vehicle: level reached within 3-4 s of driving) |
| `self_check` | gate | 0.65, 0.64 (camera-only drive 0.62); never | margin 1.5: the thinnest of the gating ones; unchanged since it was first quiet on these drives; on the three drives with a re-aimed camera it holds 79-94 % of the driving |
| `yaw_rate.share` | gate | 0.67 (gyro against wheels), 0.3 and 0.1 the other pairs; never | margin 1.5 on one pair; isolates by vote only, and can only take one source's yaw-rate rows out, never a source or an output |
| `signal.noise`, `signal.invalid` | gate | 0.18; never | margin 5 |
| `signal.stuck` | report | never approached | a constant true value looks the same as a frozen sensor: a simulated drive at constant speed froze "stuck" for good, a cruise at one quantisation step would. With a second source the pair tests decide |
| `speed.cusum` | report | 1.95, 0.99: fired twice on one drive (0.5 % of the driving) | after the camera's turn error was modelled (below) it still fires in two tight turns; before, 8 times |
| `yaw_rate.cusum` | report | 3.9: 15 spells on one drive (16 % of the driving), none on the other | the rear wheels' difference carries a slow offset of up to 0.022 rad/s against gyro and camera (which agree to 0.003): above this test's 0.015 rad/s |
| `speed.track` | report | 4.4: 15 spells, held 68 % of the driving on one drive | this receiver's track is not a speed at this level: against the odometer over 3-10 fixes its error is 1-1.6 m/s (robust), 2.5-3.2 m/s rms, beyond 10 % in half of the chords, correlated over tens of seconds; with 3-5 satellites the track is 20-25 % short; 46 % of the fixes repeat the position |
| `innovation` | gate | fires: position rows 14 % / 11 % of the driving, map cross-track 9 % / 28 % | sets a row class aside while it lasts, never a source; no output becomes invalid by it (the position coasts and says so) |
| `position.lost` | gate | 9 and 3 times | does not invalidate: the position stays published with its uncertainty opened by 40 m until the next usable fix places it (`reanchor: soft`) |
| `map.conflict` | gate | (before L47, fixed limit of 12 m) fired: 21 % / 16 % of the driving (the receiver's median of 8 fixes lies 12-27 m beside the matched road); since L47 the limit follows the receiver's own error, see "The receiver distrusted" | holds only the map's rows back; the position then rests on the receiver alone. It is what bounds a matcher on a parallel road (below) |

What was found behind the alarms on healthy data, and fixed at the cause instead of by a wider limit:

* **Wheel-speed sensors have a floor.** A wheel reads exactly 0 below about 0.35 m/s and the four come alive
  one at a time: 126-320 samples per drive of a moving car with a wheel at zero. One wheel at 0.4 m/s beside one
  at 0 is a "yaw rate" of 0.3-0.6 rad/s, and a mean of the wheels that reads low. Below 0.6 m/s on any wheel
  the wheels are not offered as a measurement.
* **The rear wheels' difference has a slow offset** (above). Its sigma says so (0.012 rad/s in quadrature);
  what remains is why `yaw_rate.cusum` reports.
* **The camera's speed is worse in turns**: robust sigma against the vehicle 0.4-0.5 m/s below 0.2 rad/s,
  0.8-0.9 m/s at 0.3-0.6 rad/s; 5-14 % of the frames above 0.2 rad/s were beyond 4 of the straight-road
  sigmas. The characterised sigma has a turn term (2 m/s per rad/s, from the estimate's yaw rate); the camera's
  own uncertainty check keeps the straight-road level.
* **A CUSUM verdict never lapsed while the car stood** (no samples, nothing to clear it): one held for 18
  minutes of parking. It lapses after 30 s without a sample; one the share test confirms becomes the share
  test's.
* **A re-anchoring published one frame with speed, yaw rate and lead unknown** ("no_measurement": the filter's
  buffer was restarted and with it the memory of when each quantity was last measured).
* **After a re-anchoring the receiver was believed at 8 x its stated accuracy**, which left a wrong map alone
  with the position. Off by default.
* **Freshness of the position counted from the fix's time of validity**, not its arrival: with a 2 s delay and
  one fix a second the mode flipped between `fused` and `coast` every second.
* **A feed without a gyro had no yaw sensor at all** (the wheels' difference was only used beside a gyro).

What `report` costs is in the summary lines under the tables: with the default modes, a fault that only a
reporting detector sees is found and named, and its source stays in the estimate.

## The receiver distrusted (L47)

One absolute source. What can be known about the position without it, and what the estimator does once the
receiver's own motion says it is not to be believed:

* **What is left.** Across the road: the matched road's line (the lane offset, 2.5 m, and the spread of the
  candidate roads). Along the road: the odometer, whose error grows with the distance driven (the speed scale
  is known to 1-2 % without a receiver to learn it from), reset at every turn of the matched road, where the
  new piece's line constrains what was along-track before. The heading: the gyro, and the road's bearing on
  straights.
* **Found out raw against raw**, without asking the estimate (which may be the wrong party): the receiver's
  displacement from fix to fix against the distance the car's speed sensors measured and the direction of
  the receiver's own earlier track turned by the car's yaw. A single step beyond 8 m + half the car's own
  step is a jump; two steps in three beyond 2.5 m/s a receiver that is not moving as the car does (few
  satellites: a smoothed track that crawls and catches up); mirrored coordinates turn the wrong way at the
  first bend. Not while the car's own speed sources disagree with each other: then the receiver is not the
  one to blame.
* **Held back.** What the receiver moved and the car did not is subtracted from its fixes (decaying over 45 s)
  and counted into its stated error; a jump also widens the position's own covariance by the step (the
  receiver may have returned to the truth, and the position that followed it is then the wrong one). The
  position goes on by dead reckoning with the map across the road, `position.mode: coast`, the receiver
  `suspect` with the reason. Beyond 250 m held back the receiver is not used at all.
* **Re-acquired** when its fixes lie within the estimate's reach again; or, once dead reckoning has run out
  (stated uncertainty beyond 75 m: the position is then published unknown), when the receiver has moved as
  the car does ten fixes running: the position is placed on it anew.
* **What this cannot do.** A slow drift (1 m/s) is inside this class of receiver's ordinary disagreement with
  the odometer (its chords over 100 m differ from the odometer's by 20 % either way on the reference car): it
  is followed along the road; across the road the map holds it. A step that stays is followed after the best
  part of a minute. A parallel road nearer than about 3 of the receiver's own sigmas (12 m for a 3 m
  receiver, 35-40 m for the reference car's with 3-5 satellites) cannot be told from the receiver's error:
  the answer is the stated uncertainty across the road (the matcher's candidates and the odds of a wrong
  match are in it), not a detection.

Dead reckoning with the map, in simulation with truth (town drive, a receiver like the reference car's,
gone from 60 s to 230 s: 1.7 km): error along the road at most 1.4 m (median per 20 s), across 1.0 m; stated
uncertainty (1 sigma, major axis) 2.2 m after each bend, growing to 8-13 m before the next one. The same
without the map: 18-32 m, stated 31-44 m. (`tests/test_position_loop.py`; the simulated map is always right:
what a wrong match costs is the map rows of the table.)

Rows of the table below that this changed (simulation, default modes, seconds an output was valid and wrong
beyond its tolerance and 3 of its own sigmas; `tools/fault_campaign.py sim`, before = the tree before L47):

| fault | before | after |
|---|---|---|
| receiver: position jump 30 m | position 41 s | 0 |
| receiver: slow position drift 1 m/s | position 35 s | 0 (followed along the road, inside the stated uncertainty) |
| receiver: lat / lon swapped | heading 18 s, position 50 s | heading 21 s, position 18 s |
| receiver: fixes 2 s older | position 33 s | position 3 s |
| camera out of range and receiver jump 30 m | lead 2 s, position 41 s | lead 2 s |
| map: parallel road 25 m, bearing turned 90 degrees | 0, 0 | 0, 0 |
| vehicle speed: stuck / sign | position 46 / 58 s (sign: heading 12 s) | position 29 / 28 s |
| vehicle speed: bias +2 m/s / scale 0.9 / slow scale drift | position 39 / 40 / 13 s | position 48 / 50 / 36 s |
| vehicle and camera speed x 0.7 | position 42 s | position 49 s |
| gyro: scale 1.3 | heading 17 s, position 8 s | heading 23 s, position 19 s |
| all 59 faults | 1355 s: position 655, heading 119, speed 424, yaw rate 56, lead 102 | 1198 s: position 493, heading 124, speed 424, yaw rate 56, lead 102 |

Worse where the car's own speed or yaw rate is wrong and (with the default modes) stays in the estimate: the
receiver's rows correct the position along the road more slowly than before, because its slowly varying
error is no longer taken for white noise. That is the price of the stated uncertainty being right on the car
(DESIGN.md section 3a); with those detectors gating the wrong sensor leaves the estimate instead. Alarms after
a receiver fault now last as long as something is held back (up to a minute after the fault ends).

## The reference car's sensor set (vehicle, camera, receiver, map)

(The table below is the one of before L47; its receiver and map rows are superseded by the section above.)

Every detector gating (`tools/fault_campaign.py sim --gate-all`): what the detectors can do on a car whose
sensors support them. "Before" is the integrator before the fault-detection work. Positions are scored whenever
both runs share the local origin (until L43 only while they shared the frame id, which hid positions after a
re-anchoring; the "before" column is re-scored the same way).

| fault | before: detected after (s) | isolated | wrong-but-valid | after: detected after (s) | isolated | wrong-but-valid | largest such error | recovered after (s) |
|---|---|---|---|---|---|---|---|---|
| vehicle speed: bias +2 m/s | 12.4 | none | position 32 s, speed 55 s | 2.9 | ok + gnss_track | position 25 s, speed 6 s | position 40.3, speed 2.09 | 35 |
| vehicle speed: scale 0.7 | 1.9 | WRONG: vision | position 43 s, speed 55 s | 0.6 | ok + gnss_track | position 4 s, speed 1 s | position 25.7, speed 2.83 | 35 |
| vehicle speed: scale 0.9 | 26.4 | none | position 20 s, speed 55 s | 1.9 | ok + gnss_track | position 40 s, speed 5 s | position 26.3, speed 1.45 | 35 |
| vehicle speed: slow scale drift to +17 % | 57.5 | none | position 14 s, speed 44 s | 31.0 | ok + gnss_track | position 5 s, speed 30 s | position 16.1, speed 1.81 | 32 |
| vehicle speed: stuck | 10.0 | WRONG: vision | position 45 s, speed 49 s | 1.6 | ok + gnss_track | 0 | - | - |
| vehicle speed: each sample repeated 5x | - | none | 0 | 2.2 | named (warning) | 0 | - | 39 |
| vehicle speed: noise burst 1 m/s | 4.5 | named (warning) | speed 18 s | 0.4 | ok | speed 3 s | speed 1.47 | - |
| vehicle speed: outliers 10 % of 5 m/s | 15.6 | named (warning) | speed 16 s | 0.6 | ok | speed 13 s | speed 3.62 | - |
| vehicle speed: unit (km/h read as m/s) | 1.8 | WRONG: vision | heading 2 s, position 53 s, speed 57 s, yaw_rate 1 s | 0.5 | ok + gnss_track | position 6 s, speed 0 s | position 145, speed 19.7 | - |
| vehicle speed: sign | 1.8 | WRONG: vision | heading 3 s, position 48 s, speed 57 s | 0.5 | ok + gnss_track | position 6 s, speed 0 s | position 114, speed 18.2 | 35 |
| vehicle speed signal only: scale 0.7 (wheels right) | 1.7 | ok + vision | heading 0 s, position 2 s, speed 3 s | 0.6 | ok + gnss_track | speed 1 s | speed 2.83 | 35 |
| vehicle wheels only: scale 0.8 | 1.7 | ok | 0 | 1.0 | ok + gnss_track | 0 | - | 35 |
| gyro: bias 0.02 rad/s | 14.2 | none | heading 21 s, position 10 s, yaw_rate 2 s | 1.4 | ok | heading 21 s, position 10 s, yaw_rate 2 s | heading 13.8, position 36.6, yaw_rate 0.02 | 1 |
| gyro: scale 1.3 | 43.8 | none | heading 19 s, position 12 s, yaw_rate 14 s | 37.3 | ok | heading 24 s, position 16 s, yaw_rate 14 s | heading 17.2, position 26.4, yaw_rate 0.04 | 12 |
| gyro: slow bias drift to 0.03 rad/s | 33.8 | none | heading 20 s, position 13 s, yaw_rate 5 s | 21.0 | ok | heading 18 s, position 15 s | heading 13.6, position 24.3 | 38 |
| gyro: stuck | 37.9 | none | heading 18 s, position 14 s, yaw_rate 15 s | 2.1 | ok | heading 14 s, position 14 s, yaw_rate 1 s | heading 46.5, position 157, yaw_rate 0.11 | - |
| gyro: noise burst 0.03 rad/s | 6.2 | none | heading 17 s, position 13 s, yaw_rate 34 s | 0.4 | named (warning) | yaw_rate 1 s | yaw_rate 0.06 | 7 |
| gyro: sign | 36.4 | named (warning) | heading 17 s, position 14 s, yaw_rate 15 s | 34.8 | ok | yaw_rate 1 s | yaw_rate 0.15 | - |
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
| receiver: position jump 30 m | 4.4 | named (warning) | position 48 s | 0.4 | ok | position 41 s | position 28.3 | 32 |
| receiver: slow position drift 1 m/s | 39.8 | none | position 26 s | 36.4 | ok | position 35 s | position 42 | 32 |
| receiver: noise burst 15 m | 7.3 | named (warning) | 0 | 0.4 | ok | 0 | - | - |
| receiver: outliers 20 % of 60 m | - | none | 0 | 0.4 | ok | 0 | - | - |
| receiver: frozen fix while moving | 5.4 | ok | 0 | 5.4 | ok | 0 | - | 35 |
| receiver: lat / lon swapped (frame) | 5.4 | named (warning) | position 42 s | 3.4 | ok | heading 18 s, position 50 s | heading 42, position 554 | - |
| receiver: noise 15 m and stated accuracy x 0.3 (lies) | 7.3 | named (warning) | 0 | 0.4 | ok | 0 | - | - |
| receiver: drop-out | 4.4 | ok | 0 | 4.3 | ok | 0 | - | 0 |
| receiver: fixes 2 s older (delay jump) | 5.4 | named (warning) | position 38 s | 6.4 | ok | position 33 s | position 32.6 | 35 |
| map: matcher on a parallel road 25 m away | 4.3 | named (warning) | 0 | 3.8 | named (warning) | 0 | - | 5 |
| map: bearing turned 90 deg (frame) | - | none | 0 | 3.8 | named (warning) | 0 | - | 5 |
| clock: epoch change (all stamps step back 600 s) | -0.1 | WRONG: vehicle,wheels | 0 | -0.1 | ok + vehicle,wheels | 0 | - | 0 |
| two together: vehicle and camera speed x 0.7 | 2.2 | named (warning) | position 43 s, speed 55 s | 1.9 | WRONG: gnss_track | position 37 s, speed 55 s | position 67.8, speed 4.3 | 35 |
| two together: vehicle speed stuck, receiver out | 4.4 | ok + vision | position 54 s, speed 49 s | 1.6 | ok | 0 | - | - |
| two together: camera out of range, receiver jump 30 m | 1.8 | ok | lead 2 s, position 58 s | 0.4 | ok | position 41 s | position 28.3 | 0 |
before: 59 faults: detected 46, isolated correctly 12, wrong source blamed 6, no output confidently wrong 29; confidently-wrong output time in all 1657 s
after:  59 faults: detected 56, isolated correctly 38, wrong source blamed 2, no output confidently wrong 33; confidently-wrong output time in all 737 s

**With the default modes** (`tools/fault_campaign.py sim`; `speed.cusum`, `yaw_rate.cusum`, `speed.track`,
`signal.stuck` report): 59 faults, the injected source taken out or named at fault level in 20 (all gating: 38;
before: 12), no output confidently wrong in 30 (33; 29), confidently-wrong output time 1355 s (737 s; 1657 s),
of which speed 424 s (131; 530), position 655 s (376; 691), heading 119 s (95; 179), yaw rate 56 s (34; 154),
lead 102 s (101; 104). What the reporting detectors would have decided is where the difference is: a wrong
vehicle speed (bias, scale 0.7 / 0.9, drift, unit, sign) is isolated by vote of the receiver's track when it
gates, and by rank (the camera is blamed, or nobody) when it reports; a gyro bias or drift is voted out by the
CUSUMs when they gate and named when they report. In the all-gating table the receiver faults counted as
"isolated" are the receiver's track being marked (it is never fused): the position columns are the same in
both modes. On 12 healthy simulated drives (38 minutes) either mode raises one finding for 0.3 s.

## With a front radar (radar faults)

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
