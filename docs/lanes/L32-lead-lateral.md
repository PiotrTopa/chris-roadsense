# L32: the lead drawn beside its lane on the MFD (new road model)

Lane of the 2.0 campaign, 2026-11-19, Fable. Branch `v2-l32` from main (bef6b77). Laptop only; the phone was not
touched and nothing was fetched from it. All tools and drivers: `tools/vision_eval/l32/` (README there),
`tools/vision_eval/lead_lateral.py`, `tools/vision_eval/calib_from_pose.py`; outputs in `~/datasets/l32`.

The owner's observations this report has to explain, as given: (1) the lead is drawn shifted to the right, more
the further it is; (2) standing still, a lead that pulls away is drawn straight ahead, without a shift; the
shift starts when the own car moves; (3) it did not happen with the version installed before the 2.0
deployment of 2026-11-19.

## Summary

| question | answer | status |
|---|---|---|
| Is the shift reproduced on the laptop? | Yes, with replayed rides whose calibration is made wrong by the car's measured error (yaw 3.4 deg, pitch 1.4 deg), through the whole chain to the MFD director: +1.1 m at 15-30 m, +3.1 m at 30-60 m, +4.9 m beyond 60 m while driving; +0.1 m while standing behind a lead that is 15-30 m away (section 1) | measured in replay |
| Which stage adds it? | None of api, view, backend scene feed, MFD director. The offset is in modeld's raw `road` message and passes every stage unchanged to within 0.3 m (section 1) | measured |
| Which change brought it (regression bisect)? | The road model. The chain from before 2.0 with supercombo: no shift. 2.0 code with supercombo: no shift. 2.0 code with the vision model: shift. The 2.0 backend with the old Pi frontend: the same shift. The chain from before 2.0 fed with the vision model: the same shift (section 2) | measured, two rides |
| Why does it depend on own speed? | Two things, both measured. (a) Standing, the vision model reports no lane lines (with the car-like calibration both inner lines in 1 % of standing runs; 91 % at 11-15 m/s), so the MFD draws its road straight from the path and the lead sits on it; when the car moves the lines appear, turned by the calibration error. (b) Leads are near when slow (median 6 m standing, 60-70 m at speed) and the offset is an angle (section 3) | measured |
| Does anything move the LEAD with speed? | No. The lead's own lateral value is the same standing and driving (within 0.25 m of zero at every speed and distance on the test ride). The lines are what is wrong (section 3) | measured |
| Host motion (yaw-rate bias, steering-angle offset) through the chain? | No effect on what is drawn, to two decimals at every speed and distance: no stage uses the yaw rate to place lines or lead (`line_motion` is off) (section 4) | measured |
| What is the underlying cause? | With a wrong camera calibration the vision model's outputs split: lane lines and road edges turn with the calibration's yaw, degree for degree; the lead does not turn (0.01-0.1 per degree); the plan turns in part. supercombo turned lines and lead together by 0.2-0.5 per degree, which hid the same error before (section 5) | measured on three rides |
| Is the car's calibration wrong by that much? | Yes: its own recordings of 12-17 November give yaw -3.2 to -3.5 deg and pitch +1.3 deg against the configured 0 / 0 (section 6). The car's recorded supercombo output shows the pattern the replay shows for supercombo with that error: lead and lines both off the axis, together | measured on car recordings (supercombo) |
| The vision model on the car itself | No recording of it exists on the laptop. That the car's shift is this effect is an inference: the replay with the car's calibration error reproduces all three observations | **not proven on the car** |
| Fix | At the source: modeld estimates the camera's pitch and yaw from the model's own ego-motion and uses it for the warp, reported in `calib` and `/v2/health` (section 7). On the replay with the car-like error the drawn offset goes from +1.1 / +3.1 / +4.9 m to +0.2 / +0.2 / +0.3 m | built, replayed; not run on the car |
| Does the fix cost anything? | Lead tables, lane flicker and jitter the same; far-range lane error better on four rides; the lead's distance noise at 15-30 m higher (0.09 -> 0.12 m, mean of the held-out rides); a ride whose camera aim changes is followed with a lag of 5 blocks (section 7) | measured |

What would settle it on the car: one recorder session of a drive with the vision model (the recorder already
writes every `road` and `calib` message). `python3 tools/vision_eval/l32/byspeed.py SESSION/messages.jsonl.gz`
gives the raw offset by speed, `tools/vision_eval/calib_from_pose.py SESSION` the calibration error; the chain
trace runs on it unchanged.

## 1. Reproduction through the whole chain, by own speed

Ride: warsaw-city, 900 s of town driving with traffic-light stops (two pull-aways behind a lead that leaves
first). Frames -> `modeld_replay` (vision + policy, 13.8 runs/s) -> the api's own `State` (the code that runs
on the phone, fed message by message, its clock the stream's) -> the car backend's `SceneBuilder` -> the MFD
frontend's `read_inputs` + `SceneDirector` at 20 Hz (`tools/vision_eval/l32/chain.sh`). Two calibrations:
`ride` = the ride's own `calib.json`; `car` = that made wrong by the car's measured error (yaw +3.4 deg, pitch
-1.4 deg relative to the truth, section 6). Code: roadsense main (bef6b77), car repo main (c2cec5a; the backend
code equals the deployed ddee72a).

Measure at every stage: the lead's lateral place minus the centre of the ego lane (both inner lines) at the
lead's distance, + = lead to the right, median per own-speed bin (`chain_table.py`). At the MFD also against
the renderer's own road centre (`render.py` `_set_centre`: both inner lines, else one line -+ 1.75 m, else the
path), which is what the picture shows.

**Vision model, car-like calibration (`warsaw-city.C_v2_vis.car`): offset m / frames used**

| stage | standing (< 0.3 m/s) | 0.3-3 | 3-7 | 7-11 | 11-15 | 15+ m/s |
|---|---|---|---|---|---|---|
| raw `road` (modeld) | +0.25 / 15 | +0.44 / 382 | +0.83 / 435 | +1.93 / 738 | +3.90 / 2469 | +4.33 / 1557 |
| topics `lead` / `lanes` | +0.33 / 1220 | +0.42 / 526 | +0.82 / 567 | +1.70 / 961 | +4.12 / 3044 | +4.42 / 1765 |
| `view` | +0.34 / 1226 | +0.42 / 526 | +0.81 / 567 | +1.63 / 988 | +4.25 / 3078 | +4.31 / 1784 |
| backend scene, tracked part | +0.33 / 883 | +0.43 / 381 | +0.83 / 412 | +1.70 / 694 | +4.10 / 2217 | +4.42 / 1269 |
| backend scene, `vw` (view) part | +0.34 / 887 | +0.42 / 381 | +0.81 / 412 | +1.61 / 714 | +4.25 / 2241 | +4.31 / 1287 |
| MFD director, both lines | +0.34 / 1771 | +0.42 / 767 | +0.82 / 816 | +1.60 / 1422 | +4.23 / 4433 | +4.30 / 2587 |
| MFD, the renderer's centre | +0.14 / 4267 | +0.36 / 1089 | +0.66 / 1384 | +1.40 / 1887 | +4.16 / 4606 | +4.30 / 2601 |
| median lead distance, m | 6 | 7 | 14 | 24 | 65 | 71 |

The offset is there in the raw model output and is handed on unchanged; the largest difference between two
of the first six rows of one column is 0.33 m. As an angle it is 2.8-3.7 deg in every moving bin: the metres grow with speed
because the lead is further away at speed. With the ride's own calibration the same table reads +0.2 to +0.5 m
in every stage (`warsaw-city.C_v2_vis.ride`, section 2).

**What the MFD draws, by lead distance and own speed (car-like calibration):** lead minus the drawn centre |
the lead's own lateral value | direction of the drawn road centre between 5 and 20 m | share of frames where
the centre comes from lane lines

| lead distance | own speed | frames | lead - drawn centre, m | lead's own lateral, m | drawn centre direction, deg | centre from lines |
|---|---|---|---|---|---|---|
| 15-30 m | **standing** | 988 | **+0.09** | +0.09 | 0.00 | 0 % |
| 15-30 m | 3-7 m/s | 550 | +0.92 | -0.09 | -3.41 | 65 % |
| 15-30 m | 7-11 m/s | 1022 | +1.13 | -0.23 | -3.80 | 85 % |
| 15-30 m | 11-15 m/s | 230 | +1.47 | -0.03 | -3.47 | 93 % |
| 30-60 m | 7-11 m/s | 673 | +2.80 | +0.12 | -3.33 | 89 % |
| 30-60 m | 11-15 m/s | 1767 | +3.06 | +0.15 | -3.71 | 97 % |
| 30-60 m | 15+ m/s | 565 | +3.38 | +0.13 | -3.75 | 100 % |
| 60+ m | 11-15 m/s | 2609 | +4.88 | +0.17 | -3.76 | 99 % |
| 60+ m | 15+ m/s | 2022 | +4.52 | +0.23 | -3.65 | 100 % |

This is the owner's observation in numbers. Standing with the lead 15-30 m ahead (it has pulled away): the lead
is drawn on the road centre (+0.09 m), the drawn road is straight (0.00 deg) and none of it comes from lane
lines. Driving: the drawn road runs 3.3-3.8 deg to the left, the lead's own value has not changed (within
+-0.25 m of zero in every row), so it stands 1 to 5 m to the right of the drawn lane.

One pull-away, second by second (`chain_episode.py warsaw-city.C_v2_vis.car.rows.jsonl.gz --t0 210 --t1 230`;
this ride turns after the stop, which moves both the lead and the road in the car's frame):

| t, s | own speed, m/s | drawn lead distance, m | lead lateral, m | drawn centre at the lead, m | centre from | offset, m |
|---|---|---|---|---|---|---|
| 211 | 0.0 | 10.3 | -0.03 | 0.00 | nothing (straight) | -0.03 |
| 213 | 0.0 | 14.8 | +0.21 | 0.00 | nothing | +0.21 |
| 215 | 0.0 | 18.0 | +0.19 | 0.00 | nothing | +0.19 |
| 217 | 0.0 | 20.2 | +0.19 | 0.00 | nothing | +0.19 |
| 221 | 4.1 | 11.4 | -2.14 | -1.52 | path | -0.62 |
| 224 | 6.9 | 26.9 | -2.59 | -3.37 | path | +0.78 |
| 226 | 8.5 | 36.2 | -0.35 | -3.52 | one line | +3.17 |
| 228 | 10.2 | 45.8 | +4.92 | -0.04 | one line | +4.96 |
| 230 | 10.9 | 52.7 | +6.94 | +2.12 | both lines | +4.83 |

(The same seconds with the ride's own calibration: offset +0.1 to +0.2 m standing, +1.0 to +1.7 m at 226-230 s,
where this ride's own small calibration error and the turn show.)

## 2. Regression bisect by configuration

Same ride, same frames, the chain assembled from different versions (`bisect.sh`, tables by `bisect_table.py`).
Before 2.0: roadsense 3da2b63 (the last main before the v2 work; api 0.1.0, no `view`) and the car repo at
0da4b79 (first parent of the v2 merge 62e3cfb). The exact commit that was installed before the deployment is in
the phone's rollback notes, which this lane did not read; these are the versions the repositories had then.

**What the MFD draws: lead minus the drawn road centre, m; in brackets the direction of the drawn centre, deg.
warsaw-city, car-like calibration**

| configuration | standing, lead 15-30 m | 3-7 m/s, lead < 15 m | 7-11 m/s, lead 15-30 m | 11-15 m/s, lead 30-60 m | 11-15 m/s, lead 60+ m | 15+ m/s, lead 60+ m |
|---|---|---|---|---|---|---|
| A: chain before 2.0, supercombo | - | -0.17 (-2.43) | -0.45 (-2.25) | -0.03 (-1.19) | -0.23 (-1.07) | +0.16 (-0.61) |
| B: 2.0 code, supercombo | - | -0.12 (-2.51) | -0.40 (-2.24) | -0.05 (-1.15) | -0.24 (-1.12) | +0.03 (-0.70) |
| **C: 2.0 code, vision (installed default)** | +0.09 (0.00) | **+0.58** (-3.99) | **+1.13** (-3.80) | **+3.06** (-3.71) | **+4.88** (-3.76) | **+4.52** (-3.65) |
| M: 2.0 api + backend, Pi frontend from before 2.0, vision | +0.08 (0.00) | +0.59 (-3.60) | +1.14 (-3.87) | +3.09 (-3.73) | +4.67 (-3.79) | +4.62 (-3.63) |
| N: the same mix, supercombo | - | -0.17 (-2.43) | -0.45 (-2.25) | -0.03 (-1.19) | -0.23 (-1.07) | +0.16 (-0.61) |
| V: chain before 2.0 fed with the vision model | +0.10 (0.00) | +0.59 (-3.58) | +1.16 (-3.87) | +3.09 (-3.72) | +4.66 (-3.79) | +4.61 (-3.64) |
| F: C with the online calibration (section 7), from a cold start | -0.02 (0.00) | +0.12 (-2.22) | +0.17 (-0.31) | +0.20 (-0.07) | +0.28 (-0.09) | +0.08 (+0.05) |

("-": fewer than 8 such frames; supercombo keeps the standing car in front as its lead, it does not report the
one that leaves.)

**The same with the ride's own calibration** (the code changes alone, on a camera that is about right)

| configuration | 3-7 m/s, lead < 15 m | 7-11 m/s, lead 15-30 m | 11-15 m/s, lead 30-60 m | 11-15 m/s, lead 60+ m | 15+ m/s, lead 60+ m |
|---|---|---|---|---|---|
| A: before 2.0, supercombo | -0.02 | -0.10 | 0.00 | -0.01 | +0.06 |
| B: 2.0 code, supercombo | 0.00 | -0.10 | -0.01 | +0.02 | +0.04 |
| C: 2.0 code, vision | +0.13 | +0.13 | +0.32 | +0.62 | +0.39 |
| M: old Pi frontend, vision | +0.12 | +0.10 | +0.35 | +0.58 | +0.45 |
| V: before 2.0, vision | +0.12 | +0.10 | +0.36 | +0.58 | +0.46 |

**Stage by stage at 11-15 m/s, car-like calibration, m**

| configuration | raw `road` | `lead` / `lanes` | `view` | scene, tracked | scene, `vw` | director, both lines | renderer's centre |
|---|---|---|---|---|---|---|---|
| A | -0.09 | -0.14 | (none) | -0.15 | (none) | -0.12 | -0.11 |
| B | -0.09 | -0.14 | -0.12 | -0.15 | -0.11 | -0.11 | -0.12 |
| C | +3.90 | +4.12 | +4.25 | +4.10 | +4.25 | +4.23 | +4.16 |
| M | +3.90 | +4.12 | +4.25 | +4.10 | +4.25 | +3.87 | +3.82 |
| V | +3.90 | +4.07 | (none) | +4.06 | (none) | +3.90 | +3.85 |

Second ride, krakow-2024 (240 s, town; noisier, fewer frames per cell), car-like calibration, the renderer's
centre: A -0.10 / +0.08 / +0.06 m, B -0.10 / +0.10 / +0.10 m, C +0.38 / +0.86 / +1.95 m, M +0.44 / +0.87 /
+1.93 m, V +0.44 / +0.86 / +1.96 m (3-7 m/s lead < 15 m / 7-11 m/s lead 15-30 m / 11-15 m/s lead 30-60 m); at
15+ m/s with the lead beyond 60 m: A +0.30, B -0.01, C +4.05 m.

Reading:

* **A against B: nothing.** The 2.0 code (the `view` channel and how the MFD consumes it, the trackers, the
  scene feed, the director's view path) does not put the lead off its lane: with supercombo both chains agree to
  0.1 m in every cell.
* **B against C: this is the pair that first shows it.** The same code with the vision model: +0.6 to +4.9 m.
* **M (the mixed installation): no contract mismatch.** The old Pi frontend on the new backend draws what the
  new frontend draws (within 0.25 m); with supercombo (N) it draws what A draws.
* **V: the model alone suffices.** The chain from before 2.0 fed with the vision model shows the same shift.
* With a calibration that is about right (second table) the vision model's remaining offset is +0.1 to +0.6 m:
  the ride's own smaller calibration error (section 5).

## 3. Why own speed: what changes between standing and driving

Raw `road` messages of the same ride, by own speed (`byspeed.py`; lead prob >= 0.7; "both lines" = both inner
lines with prob >= 0.5):

| own speed | vision, car-like calibration: runs with both lines | lead's own y, m | direction of the lane centre 5-30 m, deg | vision, ride calibration: both lines | lead y | lane direction | supercombo, car-like: both lines | lead y | lane direction |
|---|---|---|---|---|---|---|---|---|---|
| standing | **1 %** | +0.05 | -3.31 (few) | 25 % | -0.07 | -0.30 | 33 % | -0.36 | -4.07 |
| 0.3-3 m/s | 47 % | -0.21 | -3.72 | 50 % | -0.09 | -0.42 | 38 % | -0.70 | -3.17 |
| 3-7 | 39 % | -0.19 | -3.53 | 42 % | -0.15 | -0.48 | 14 % | -1.08 | -1.88 |
| 7-11 | 57 % | -0.19 | -3.52 | 64 % | -0.16 | -0.33 | 48 % | -1.22 | -0.55 |
| 11-15 | 91 % | +0.15 | -3.65 | 86 % | +0.23 | -0.26 | 79 % | -0.86 | -1.04 |
| 15+ | 94 % | +0.20 | -3.63 | 95 % | +0.31 | -0.19 | 80 % | -0.56 | -0.55 |

* **The lead does not move with speed or with the calibration** in the vision model: its y is within 0.3 m of
  zero in every bin, with either calibration. (supercombo's lead moves with the calibration: -0.4 to -1.2 m.)
* **The lines do**: with the car-like calibration they run 3.3-3.7 deg to the left whenever they are present, at
  every speed.
* **Standing, the vision model has no lane lines** with the car-like calibration (1 % of standing runs; the
  median probability of the weaker inner line is 0.13 standing against 0.88 at 11-15 m/s). The MFD then has no
  lines to draw the lane from; its road centre comes from the path, which is straight ahead while standing, and
  the lead, which is also straight ahead, sits on it. That is observation (2).
* Response of each raw output to the car-like calibration error, by own speed (the same frames with and without
  the error, `sens_speed.py`; change of direction in degrees for +3.4 deg of yaw): lead -0.40 standing, -0.70
  creeping, -0.05 to -0.09 above 7 m/s; lines -2.1 standing, -2.8 to -3.4 moving; plan -1.05 standing, -2.2
  creeping, -0.4 to -0.5 at speed. The lines carry the error at every speed; the lead carries a tenth of it or
  less while driving.

So nothing in the chain is a function of speed. What depends on the own car's motion is the model's content:
lines only while moving, and far leads only while moving.

## 4. Host motion injected

`inject.sh`: the same chain (2.0 code, vision, the ride's calibration) with a host `ego` message per frame as
mapd relays it (`speed_mps`, `yaw_rate_rps`, source `vehicle`): (i) exact (the model's own motion), (ii) a
yaw-rate bias of 0.02 rad/s, (iii) a yaw-rate error proportional to speed, 0.002 rad/s per m/s (a steering-angle
zero offset of about 0.3 deg at the wheels). The api took the feed (`view.ego.source`, `lead.ego.source`,
`lanes.ego.source` = `vehicle` on all 12418 frames).

| run | standing, lead 15-30 m | 7-11 m/s, lead 15-30 m | 11-15 m/s, lead 30-60 m | 11-15 m/s, lead 60+ m | stage values at 11-15 m/s (raw / topics / view / director) |
|---|---|---|---|---|---|
| no host feed | +0.15 | +0.13 | +0.32 | +0.62 | +0.45 / +0.50 / +0.52 / +0.50 |
| exact | +0.16 | +0.13 | +0.32 | +0.62 | +0.45 / +0.50 / +0.52 / +0.50 |
| yaw-rate bias 0.02 rad/s | +0.16 | +0.13 | +0.32 | +0.62 | +0.45 / +0.50 / +0.52 / +0.50 |
| speed-proportional yaw-rate error | +0.16 | +0.13 | +0.32 | +0.62 | +0.45 / +0.50 / +0.52 / +0.50 |

A wrong host yaw rate changes nothing in the place of the lead or the lines: `view` carries lines by the car's
motion only with `line_motion` on (it is off by default; not checked whether the car's configuration sets it),
the lanes tracker takes the speed only, the MFD extrapolates each line with the line's own measured rate. The
host feed is not the cause.

## 5. The cause in the model: which output follows the calibration

The same ride stretch replayed with the calibration's yaw changed by +-1 and +-2 deg, the principal point `cx`
moved by the same angle, the pitch by +-1 deg (`sweep.sh`, ORT fp32, every frame; compared run by run against
the unchanged replay: `sens.py ~/datasets/l32/sweep RIDE MODEL`). Median change of each output's direction in
degrees, divided by the size of the calibration change (first value: yaw increased, second: decreased):

| output | vision, a1-night (yaw +1 / -1) | vision, warsaw-city (+2 / -2) | vision, tunnels-s3 (+2 / -2) | supercombo, a1-night (+2 / -2) | supercombo, warsaw-city (+2 / -2) |
|---|---|---|---|---|---|
| direction of `pose` (atan vy/vx) | -0.94 / +0.86 | -1.02 / +0.83 | -0.94 / +0.84 | -0.95 / +0.95 | -1.00 / +0.90 |
| lane centre at 10 / 30 / 60 m (yaw increased) | -0.97 / -0.96 / -0.95 | -1.04 / -1.02 / -0.98 | -0.95 / -0.97 / -0.96 | -0.38 / -0.21 / -0.18 | -0.49 / -0.37 / -0.42 |
| lead | -0.01 / +0.03 | +0.05 / +0.11 | +0.04 / +0.05 | -0.22 / +0.47 | -0.36 / +0.72 |
| plan at 30 / 60 m (yaw increased; decreased) | -0.03 / -0.09; +0.24 / +0.31 | -0.04 / -0.11; +0.55 / +0.54 | -0.12 / -0.26; +0.21 / +0.24 | -0.34 / -0.23; +0.07 / +0.18 | -0.37 / -0.47; +0.23 / +0.36 |
| lead minus lane centre | +0.93 / -0.87 | +1.11 / -0.80 | +1.05 / -0.85 | -0.03 / +0.20 | -0.08 / +0.22 |

* **Vision: the lane lines turn by the full angle, the lead by almost nothing**, the plan by 0.05-0.5 depending
  on the scene. The model places the lead (and in part the plan) by the car's actual motion, which it sees in
  the frame pair; the lines are geometry of the warped image.
* **supercombo: lines and lead both turn by 0.2-0.5 of the angle**, together; their difference changes by a
  tenth to a fifth of the angle. That is why the same camera error showed nothing before the new model.
* A shift of the principal point is the same as a yaw (`cx` moved by 1.1 deg on a1-night: pose -1.03, lanes
  -1.05, lead 0.00).
* Pitch (vision, a1-night, +1 / -1 deg): the direction of `pose` follows (vz/vx +0.91 / -0.96 deg); nothing
  moves sideways; **the lead's distance does** (median 73 m: -7.5 m with the camera believed 1 deg further down,
  -1.6 m with 1 deg further up).

The other hypotheses of the brief:

* (a) the two inputs: `modeld/warp.cc` builds both frames from one camera with one rotation and one intrinsic
  matrix; the constants equal upstream's `common/transformations/model.py` (recomputed: the forward axis lands
  at (256, 47.6) and (256, 151.8)), the matrix is upstream's `get_warp_matrix`, and upstream also uses the same
  `rpyCalib` for both and ignores `wideFromDeviceEuler` in the warp (`selfdrive/modeld/modeld.py` v0.11.1, lines
  259-262). No inconsistency found. Not tested: moving one input alone.
* (c) frames: with a wrong calibration the lines are in the calibrated (warped image) frame and the lead in the
  frame of the car's motion; with a right calibration these are one frame. No transform is missing in our code.
* (d) decode: the lead slices and the sign are the same for both models; a lead that does not follow the warp
  is not something a decode error produces.

On the rides with their own (estimated) calibrations the same effect is there in small
(`lead_lateral.py ~/datasets/openpilot-models/l24/road/*.vis.13.8.jsonl.gz`):

| ride | vision: lead - lane centre by distance 15-30 / 30-45 / 45-60 / 60-75 / 75-90 / 90-105 m, m | vision: angle lead - lane, deg [90 %] | vision: lead - plan | supercombo: lead - lane | vision: direction of `pose`, deg | expected lead - lane (-0.95 x) |
|---|---|---|---|---|---|---|
| a1-night | +0.37 / +0.44 / +0.54 / +0.78 / +0.82 / +0.84 | +0.63 [+0.51, +0.72] | +0.28 | +0.26 | -0.47 | +0.45 |
| krakow-2024 | -0.14 / -0.18 / -0.82 / -1.10 / -1.69 / - | -0.79 [-0.94, -0.12] | -0.16 | +0.47 | +0.71 | -0.68 |
| s7-elblag-lowsun | - / - / +0.39 / +0.22 / +0.00 / +0.24 | +0.18 [+0.04, +0.34] | +0.12 | +0.02 | -0.05 | +0.05 |
| tunnels-s3 | -0.02 / +0.14 / +0.13 / +0.26 / +0.32 / +0.02 | +0.18 [+0.11, +0.23] | +0.02 | +0.12 | +0.10 | -0.10 |
| warsaw-city (first 300 s) | +0.06 / +0.18 / +0.43 / +0.46 / +0.38 / - | +0.29 [+0.15, +0.39] | +0.22 | -0.08 | -0.11 | +0.10 |

The angle differs from ride to ride in size and sign and follows each ride's calibration error (last two
columns). A remainder of about +0.15 deg to the right is left on four of five rides (0.2 m at 75 m), not
explained here.

## 6. The car's calibration from its own recordings

The recorder sessions on the laptop (`~/roadsense-rec`, supercombo at the time) hold every `road` message, so
the model's `pose` is there at full rate. The direction of `pose` on straight fast runs is the calibration's
error (section 5, first row: 0.83-1.02 per degree, both models).
`python3 tools/vision_eval/calib_from_pose.py --per-file ~/roadsense-rec/2026111*` (configured in all sessions:
pitch 0, yaw 0, focal 975 px at 1280x720):

| session | accepted runs / blocks | pitch, deg (sigma of the blocks) | yaw, deg (sigma of the blocks) |
|---|---|---|---|
| 20261112-141112 | 1710 / 17 | +1.31 (0.13) | -3.17 (0.16) |
| 20261117-101013 | 7839 / 78 | **-1.06** (0.11) | -3.34 (0.07) |
| 20261117-145921 | 1064 / 10 | +1.39 (0.06) | -3.26 (0.05) |
| 20261117-163836 | 2901 / 29 | +1.29 (0.08) | -3.48 (0.12) |
| 145921 + 163836 together | 3965 / 39 | +1.33 (0.08) | -3.42 (0.13) |

(The other sessions have no straight run above 24 km/h.)

* **Yaw: -3.2 to -3.5 deg in all four sessions over five days.** The camera looks 3.3 deg to the right of the
  direction of travel, or its principal point is 57 px right of the assumed centre, or a mix (the two cannot be
  told apart and are corrected together). Within a session 0.05-0.15 deg, between sessions 0.3 deg.
* **Pitch: +1.3 deg** in three sessions, -1.06 deg in the morning session of 17 November (not explained: the
  mounting or the load differed in that one session).
* **The car's recorded supercombo output shows what the replay shows for supercombo with this error**
  (`byspeed.py ~/roadsense-rec/20261117-163836/messages.jsonl.gz`): lead minus lane centre -0.01 to +0.36 m at
  every speed (no shift, observation 3), while the lead itself is 0.4-0.7 m left of the axis in three of four speed bins and
  the lane centre runs 0.1-2.4 deg to the left: both off the axis, together. Replay, supercombo, car-like
  calibration: lead minus lane -0.17 to +0.22 m, lead 0.4-1.2 m left, lanes 0.6-4 deg left.
* A weak direct look at the car's frames with the vision model (the recordings have one frame a second; the
  model fed the same frame twice believes the car stands and finds lines and leads far less often: a hint, not
  a measurement; `still.py`): session 163836, configured calibration: lead +0.93 m right of its lane centre at
  15-30 m (6 frames with both lines); with the estimated calibration +0.18 m (51 frames), and both inner lines
  in 98 instead of 9 of 694 frames.

Prediction for the car with the vision model and the configured calibration: lines 3.2 deg to the left, lead in
place: 0.8 m at 15 m, 1.7 m at 30 m, 3.4 m at 60 m to the right while driving; nothing visible while standing.

## 7. The fix: online pitch / yaw in modeld

The lead is the output that is right; the lines are wrong by the calibration. So `lead` is not touched and no
display correction was built (it would have moved the right output to the wrong one). The calibration is
corrected at the source: `modeld/calibrator.{h,cc}`, contract in docs/ARCHITECTURE.md "Calibration".

* Observation per model run: `yaw = yaw_used + atan2(vy, vx)`, `pitch = pitch_used - atan2(vz, vx)` from `pose`
  (openpilot calibrationd's observation; signs and factor measured in section 5).
* Gates: `pose.vx` >= 6.7 m/s; |yaw rate| <= 1 deg/s (openpilot: 2); `atan2(vy_std, vx)` <= 0.5 deg (openpilot:
  0.25: on the car's recordings the model's own figure is 0.3-0.5 deg, 0.25 passes 0-15 % of the straight runs
  and 0.5 about 90 %, with the same estimate: -3.20 / -3.45 against -3.34 / -3.48 deg on two sessions).
* 100 accepted runs = one block (median); estimate = median of the last 50 blocks. Valid from 5 blocks while the
  standard error of the median is <= 0.3 deg (stays valid to 0.45 deg), the blocks' sigma <= 1.5 deg and the
  estimate within 0.15 rad of the configured value; else `collecting` / `unstable` / `out_of_range` and the
  configured value is in the warp. When the newest 5 blocks all leave the estimate by more than 1.5 deg the
  camera was moved and the older blocks are dropped.
* The warp follows a valid estimate in steps of at least 0.25 deg; a step drops one frame pair (0.2 s without a
  `road` message with the vision model).
* Kept between starts (`/var/lib/roadsense/camera-calibration.json`), tied to the `calib.json` values and the
  camera's intrinsics. Roll, height and focal length are not estimated.
* Reported: `calib.source` `"auto"`, `calib.online` (state, estimate, sigma, standard error, blocks, configured
  values); `/v2/health` `calibration` (with `off_config_deg`). `auto_calib`: `on` (default), `observe`, `off`.
  `modeld_replay` has it behind `--auto-calib` (off by default there).

### Effect on the rides

Each ride stretch of L24-M / L27 replayed (`ac_all.sh`; vision + policy, ncnn fp32 on the laptop GPU, 13.8
runs/s): `off` = the ride's `calib.json` as it is; `cold` = the estimator from nothing; `warm` = again with the
estimate known from the start; `poff` = the calibration made wrong by the car's error, no estimator; `pcold` =
that with the estimator from nothing (the car's first drive with the fix). krakow-2024 was rerun with the
final validity rule (`ac_krakow.sh`); for the other rides the earlier rule (sigma <= 0.5 deg) gives the same
decisions.

**The lead against its lane: angle lead minus lane centre, deg (leads from 20 m, median, + = lead right;
`lead_lateral.py`, tables by `tables.py`)**

| ride | off | warm | cold | poff (config wrong by the car's error) | pcold |
|---|---|---|---|---|---|
| a1-night | +0.70 | +0.11 | +0.01 | +3.87 | +0.15 |
| warsaw-city | +0.38 | +0.14 | +0.15 | +3.66 | +0.16 |
| krakow-2024 | -0.79 | +0.13 | -0.43 | +2.43 | +1.97 |
| s7-elblag-lowsun | +0.18 | +0.18 | +0.18 | +3.44 | +0.36 |
| tunnels-s3 | +0.18 | +0.16 | +0.47 | +3.52 | +1.57 |

**By distance, m (median per bin, bins with at least 20 frames)**

| ride, run | 15-30 | 30-45 | 45-60 | 60-75 | 75-90 | 90-105 m |
|---|---|---|---|---|---|---|
| a1-night, off | +0.37 | +0.51 | +0.68 | +0.80 | +0.88 | +0.95 |
| a1-night, warm | +0.17 | +0.15 | +0.09 | +0.05 | +0.09 | +0.05 |
| a1-night, poff | +1.67 | +2.59 | +3.42 | +4.53 | +5.43 | +6.00 |
| a1-night, pcold | +0.19 | +0.20 | +0.14 | +0.08 | +0.13 | +0.16 |
| warsaw-city, off | +0.14 | +0.28 | +0.38 | +0.43 | +0.54 | +0.73 |
| warsaw-city, warm | +0.09 | +0.12 | +0.16 | +0.14 | +0.17 | +0.27 |
| warsaw-city, poff | +1.25 | +2.49 | +3.39 | +4.30 | +5.11 | - |
| warsaw-city, pcold | +0.16 | +0.15 | +0.17 | +0.15 | +0.16 | +0.27 |
| krakow-2024, off | -0.14 | -0.18 | -0.82 | -1.10 | -1.69 | - |
| krakow-2024, warm | +0.20 | +0.21 | +0.07 | -0.05 | -0.31 | -0.94 |
| krakow-2024, poff | +1.16 | +1.86 | +2.05 | +2.40 | +2.60 | - |
| krakow-2024, pcold | +0.44 | +1.39 | +1.99 | +2.40 | +2.60 | - |
| s7-elblag-lowsun, off = warm | - | - | +0.39 | +0.22 | +0.00 | +0.24 |
| s7-elblag-lowsun, poff | - | - | +3.36 | +3.75 | +4.85 | +5.88 |
| s7-elblag-lowsun, pcold | - | - | +3.33 | +3.74 | +0.50 | +0.32 |
| tunnels-s3, off | -0.02 | +0.14 | +0.13 | +0.26 | +0.32 | +0.02 |
| tunnels-s3, warm | +0.03 | +0.12 | +1.37 | +0.12 | +0.29 | +0.26 |
| tunnels-s3, poff | +1.72 | +2.46 | +3.08 | +4.65 | +5.36 | +5.41 |
| tunnels-s3, pcold | +0.03 | +0.12 | +1.40 | +0.13 | +3.68 | +1.59 |

**The estimator on each ride** (from the `calib` lines of the streams)

| ride, run | first valid after | steps of the warp | end state: blocks, sigma yaw / pitch, deg | estimate against the configured yaw, deg |
|---|---|---|---|---|
| a1-night, cold | 41 s | 2 (41 s, 114 s) | valid, 50, 0.19 / 0.12 | -0.8 |
| a1-night, pcold (config wrong by 3.4 / 1.4 deg) | 42 s | 2 | valid, 50, 0.13 / 0.11 | -4.1 (the same estimate as cold within 0.13 deg) |
| warsaw-city, cold | 82 s | 1 | valid, 50, 0.03 / 0.03 | -0.3 |
| warsaw-city, pcold | 85 s | 1 | valid, 50, 0.04 / 0.04 | -3.7 (within 0.02 deg of the cold run's) |
| s7-elblag-lowsun, pcold | 51 s | 1 | valid, 28, 0.02 / 0.02 | -3.3 |
| s7-elblag-lowsun, cold | 55 s | 0 (estimate within 0.25 deg of the configured value) | valid, 28, 0.03 / 0.07 | -0.05 |
| krakow-2024, cold | 54 s | 1 | valid, 13, 0.52 / 0.29 | +0.8 |
| krakow-2024, pcold | 57 s | 2 | valid, 13, 0.79 / 0.24 | -2.5 |
| tunnels-s3, cold | 36 s | 6 (the ride changes its camera aim by 2 deg four times, section 9) | valid, 16, 0.46 / 0.39 | +1.8 at the end |
| warsaw-storm, cold and pcold | never (200 accepted runs in 240 s: rain, slow) | 0 | collecting, 2 blocks | not applied |
| gdansk-night-snow, cold | never (blocks scatter 0.76 deg, 9 blocks) | 0 | unstable | not applied |

Reading:

* Where the estimate is valid from the start (`warm`) the lead is within 0.1-0.3 m of its lane centre at every
  distance on a1-night and warsaw-city (before: up to 0.95 m, growing with distance), and krakow-2024 goes from
  -0.79 to +0.13 deg.
* **With the car's error in the configuration (`poff`) every ride shows the car's symptom: 1.2-1.7 m at 15-30 m,
  5-6 m beyond 90 m.** With the estimator from nothing (`pcold`) a1-night and warsaw-city are at 0.1-0.3 m; the
  other rides are corrected only for the part after the first valid estimate (s7: the far bins; krakow-2024:
  the near leads come early in that ride). That is the cold start, once per camera position.
* Rides where the estimator does not get valid (rain, snow, town at low speed) keep the configured calibration:
  nothing changes there, and `calib.online.state` says so.
* tunnels-s3 gets worse in one bin (+1.37 m at 45-60 m): the ride changes its camera aim and the estimator is
  5 blocks behind each change.

**Lead tables of L27 / L29** (`lead_eval.py score`, mean of the rides; supercombo's stream of L24-M for
reference)

| after the tracker | held-out rides (s7, a1-night, warsaw-city): off | warm | cold | tuning rides (krakow-2024, warsaw-storm, gdansk, tunnels-s3): off | warm | cold |
|---|---|---|---|---|---|---|
| lead on/off changes / min | 1.87 | 1.96 | 1.89 | 1.94 | 1.94 | 1.94 |
| identity changes / min | 0.88 | 0.89 | 0.87 | 0.84 | 0.84 | 0.84 |
| distance jumps / min | 0.41 | 0.44 | 0.61 | 0.55 | 0.50 | 0.50 |
| per-frame step rms / p95, m | 0.72 / 1.39 | 0.74 / 1.39 | 0.73 / 1.38 | 0.36 / 0.64 | 0.36 / 0.67 | 0.37 / 0.69 |
| distance noise 15-30 / 30-60 / 60+ m | 0.09 / 0.17 / 0.37 | 0.12 / 0.20 / 0.35 | 0.12 / 0.21 / 0.38 | 0.04 / 0.09 / 0.19 | 0.06 / 0.09 / 0.19 | 0.06 / 0.10 / 0.21 |
| relative-speed noise, m/s | 0.77 | 0.72 | 0.70 | 0.20 | 0.21 | 0.21 |
| follows the raw distance, median / p90 % | 1.80 / 6.65 | 1.88 / 7.05 | 1.81 / 7.08 | 3.06 / 9.91 | 3.05 / 9.93 | 3.06 / 9.87 |

(The held-out set here is three rides: krakow-night-tunnel was not replayed. These streams are ncnn fp32 on the
laptop GPU, L27's were ORT: compare the columns with each other, not with L27's table.) The lead tables are the
same within the spread between rides; the distance noise at 15-30 m is 0.02-0.03 m higher with the estimator
(0.09 -> 0.12 m), the one figure that is worse.

**Lanes, far range, ego lane** (`evaluate.py` of L24-M on the same streams; far range = a tracked inner line point
seen at that distance against the same line when the car is 5-15 m from it, |dy| median, m; ego lane against L20's
labels where they exist; `~/datasets/l32/res/eval.json`)

| ride | measure | off | warm | cold | poff | pcold |
|---|---|---|---|---|---|---|
| a1-night | far range 20-40 / 40-70 / 70-100 m | 0.17 / 0.38 / 0.68 | 0.11 / 0.21 / 0.36 | 0.17 / 0.34 / 0.56 | 1.28 / 2.87 / 4.98 | 0.12 / 0.21 / 0.33 |
| | inner-line presence changes / min; line jitter at 50 m | 0.8; 0.048 | 0.8; 0.053 | 0.8; 0.050 | 0.8; 0.051 | 0.8; 0.052 |
| | ego lane right / wrong and confident, % | 98.6 / 0.0 | 98.6 / 0.0 | 98.6 / 0.0 | 98.4 / 0.0 | 98.7 / 0.0 |
| | lead distance noise 15-30 / 30-60 / 60+ m | 0.14 / 0.22 / 0.47 | 0.19 / 0.30 / 0.46 | 0.19 / 0.33 / 0.48 | 0.12 / 0.18 / 0.40 | 0.16 / 0.28 / 0.44 |
| warsaw-city | far range | 0.11 / 0.23 / 0.48 | 0.09 / 0.18 / 0.39 | 0.10 / 0.20 / 0.43 | 1.21 / 2.67 / 4.64 | 0.18 / 0.49 / 2.23 |
| | inner-line changes / min; jitter 50 m | 4.0; 0.056 | 3.9; 0.053 | 4.0; 0.053 | 3.7; 0.056 | 4.1; 0.054 |
| | ego lane right / wrong and confident, % | 81.1 / 2.3 | 81.1 / 2.3 | 80.6 / 2.2 | **58.2 / 3.6** | 75.6 / 2.4 |
| | lead distance noise | 0.03 / 0.05 / 0.28 | 0.04 / 0.06 / 0.24 | 0.04 / 0.05 / 0.30 | 0.03 / 0.06 / 0.15 | 0.03 / 0.04 / 0.24 |
| krakow-2024 | far range | 0.43 / 0.94 / 1.59 | 0.38 / 0.78 / 1.41 | 0.43 / 0.93 / 1.70 | 0.89 / 1.95 / 3.37 | 0.57 / 1.28 / 2.37 |
| | inner-line changes / min; jitter 50 m | 7.0; 0.084 | 7.8; 0.082 | 8.0; 0.084 | 4.3; 0.083 | 7.0; 0.082 |
| tunnels-s3 | far range | 0.19 / 0.37 / 0.59 | 0.13 / 0.27 / 0.45 | 0.29 / 0.68 / 1.21 | 0.82 / 1.97 / 3.82 | 0.29 / 0.67 / 1.20 |
| | inner-line changes / min; jitter 50 m | 0.0; 0.049 | 0.0; 0.045 | 0.0; 0.049 | 0.0; 0.044 | 0.0; 0.046 |
| | lead distance noise | 0.04 / 0.16 / 0.30 | 0.13 / 0.15 / 0.31 | 0.12 / 0.18 / 0.35 | 0.06 / 0.04 / 0.22 | 0.15 / 0.18 / 0.33 |
| warsaw-storm (estimator never valid) | far range | 0.20 / 0.53 / 1.04 | the same | the same | 0.81 / 2.06 / 4.35 | 0.81 / 2.06 / 4.35 |

* **The lane lines get better where the estimate is in use**: the far-range error falls on all four rides (70-100 m:
  0.68 -> 0.36, 0.48 -> 0.39, 1.59 -> 1.41, 0.59 -> 0.45 m). This is the lines agreeing with themselves over
  time, which a yaw error breaks (a far point is displaced by distance times the error).
* **With the car's error the lines are far off (3.4-5 m at 70-100 m) and the ego lane suffers** (warsaw-city:
  right 81 % -> 58 %, wrong and confident 2.3 -> 3.6 %). With the estimator from nothing most of it comes back
  within the ride (0.33 m on a1-night; 2.2 m and 75.6 % on warsaw-city, where the first 85 s are uncorrected).
  This is a second reason the car needs the calibration, beyond the picture: the measurement topics `lanes`,
  the ego lane and what depends on them are wrong by the same angle today.
* Lane flicker and jitter: unchanged (krakow-2024: 7.0 -> 7.8 inner-line changes a minute, the one that moved).
* **Worse: the lead's distance noise at 15-30 m** (0.14 -> 0.19 m on a1-night, 0.04 -> 0.13 m on tunnels-s3,
  0.03 -> 0.04 m on warsaw-city). On tunnels-s3 it comes with the six steps of the warp; on a1-night there is
  no step after the start in `warm`, so the changed pitch (0.7 deg on that ride) is the likely reason; not
  examined further.
* From a cold start on a ride whose camera changes (tunnels-s3 `cold`) the far range is worse than without the
  estimator (1.21 against 0.59 m at 70-100 m): the price of following a camera that was moved.



## 8. To check on the car (not proven here)

1. Record one drive with the vision model before changing anything (the recorder as it is). On the laptop:
   `byspeed.py` on it must show the lead's own y near zero at every speed and the lane direction near -3 deg while
   driving; `calib_from_pose.py` must give yaw about -3.4 deg. If it does not, this explanation is wrong for the
   car and the session says where.
2. With the new modeld: `/v2/health` -> `calibration.online`: `state` `valid`, `applied` true, `off_config_deg`
   near pitch +1.3, yaw -3.4; `error_deg` under 0.3. The first valid estimate needs 500 accepted runs: under a
   minute on an open road; in town longer (session 145921: 10 blocks in 70 min of town driving). The state file
   makes this a one-time wait per camera position.
3. That the vision model's certainty about its motion on the DVR camera passes the 0.5 deg gate (`accepted` /
   `seen` in the `calib` message grow while driving straight). Only supercombo's figure is known from the car.
4. The MFD: the lead in its lane while driving; the lane lines parallel to the car on a straight road.
5. The lead's distance changes by several percent with the pitch correction (section 5); a parked car at a
   measured distance would check it.
6. Immediate relief without the new build (only if the camera was not moved since 17 November):
   `/etc/roadsense/calib.json` `{"pitch": 0.0233, "yaw": -0.0597, "roll": 0.0, "height_m": 1.22}`.

## 9. Limits and open points

* The car with the vision model was not measured (see the summary). The replay's "car-like" condition is a
  dashcam ride with a rotated calibration, not the car's camera.
* The versions for "before 2.0" are the repositories' state before the v2 merges, not a read-out of what was
  installed.
* The standstill behaviour (no lane lines while standing) is the model's; with the corrected calibration it is
  still there (25 % of standing runs with both lines on this ride). Whether the display should hold the last
  lines while standing is a display question, not addressed.
* A remainder of about +0.15 deg (lead right of the lane centre) after the correction on most rides.
* The estimate assumes the car moves along its own axis on straights. On tunnels-s3 the direction of `pose` and
  the direction of the lane lines both change by 2 deg between sections of the ride (44-51 s, 181 s, 247 s,
  276 s): that ride changes its camera aim, and the estimator follows each change after 5 blocks, being wrong
  by 2 deg in between. On a car whose camera does not move this does not arise; if the direction of `pose` can
  be biased by a scene (not seen on the other rides or the car's recordings), the same would happen.
* Town rides: blocks scatter by 0.7-0.9 deg (krakow-2024); the estimate needs about 14 blocks there.
* On a cold start the first 5 blocks are uncorrected, and those are the frames with far leads.
* fp16 on the phone: all numbers here are fp32.
* The api's projections into the image (ego light by the path, sign context, overlay) use the `calib` message:
  with the corrected values the plan and the lead are projected with the true angles. Not measured.

## 10. Checks

| check | command | result |
|---|---|---|
| roadsense api | `PYTHON=~/projects/roadsense/.venv-models/bin/python sh tools/smoke_api.sh` | 255 passed (252 on main + 3 in `api/tests/test_calibration.py`) |
| C++ | whole tree, Release, with `-DROADSENSE_NCNN=ON` (`~/datasets/l32/build-ncnn`) and without (`~/datasets/l32/build2`) | builds |
| estimator | `build/modeld/calibrator_test` | all passed (closed loop to 0.005 deg, gates, observe mode, disagreeing blocks, a town ride, a moved camera, out of range, state file kept / ignored / reset) |
| warp | `build/modeld/warp_test` | 10 cases identical (untouched) |
| `modeld_replay` unchanged without `--auto-calib` | 200 frames of a1-night, vision + policy and supercombo, ORT fp32, against the binary built from main (`calib` and `road` lines without the timing fields) | identical for both models |
| the daemon itself | `camerad-replay` + `modeld --backend ncnn` on the laptop, s7-elblag-lowsun from 70 s, `calib.json` wrong by yaw +3.4 / pitch -1.4 deg (`/mnt/win/linux-scratch/l32/daemon/run.sh`) | 19.6-19.9 runs/s; `collecting` for 4 blocks, at block 5 (29 s) `source: "auto"`, yaw 5.92 -> 2.68 deg (the ride's own value 2.52), pitch -4.26 -> -3.14 (-2.86); after a restart of modeld the estimate is `restored` and in use from the first frames |
| car suite | not run: the car repo is not edited and the api change is one more key in `/health` | - |

Not pushed. Worktrees left for the bisect: `~/projects/wt/roadsense-l32-pre2` (3da2b63), `~/projects/wt/prius-l32-pre2`
(0da4b79), both detached and unchanged.


## 11. Reproduce

* Chain trace: `sh tools/vision_eval/l32/chain.sh NAME ROAD.jsonl.gz API_DIR BACKEND_REPO FRONTEND_REPO
  [--ego --ego-yaw-bias R --ego-yaw-per-speed K]` -> `~/datasets/l32/chain/NAME.{ws,scenes,rows}.jsonl.gz`,
  `NAME.txt`, `NAME.json`. Bisect: `bisect.sh RIDE`, tables `bisect_table.py RIDE CONFIG...`. Injection:
  `inject.sh RIDE`. Episode: `chain_episode.py ROWS... [--list | --t0 S --t1 S]`.
* Road streams: `ac.sh RIDE START DUR TAG OUT DYAW_DEG DPITCH_DEG [modeld_replay args]` (vision), `ac_sc.sh`
  (supercombo); `ac_all.sh`, `score.sh`, `tables.py`.
* By own speed: `byspeed.py ROAD...`, `sens_speed.py A B`. Sensitivity: `sweep.sh`, `sweep2.sh`, `sens.py`.
* The car: `calib_from_pose.py --per-file ~/roadsense-rec/2026111*`, `byspeed.py SESSION/messages.jsonl.gz`.
* Unit test: `build/modeld/calibrator_test`; api: `api/tests/test_calibration.py`.
