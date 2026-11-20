# L35: ego-motion and input audit (for the all-data integrator)

Lane L35, 2026-11-19, laptop only (phone and car repo read-only; nothing deployed). Times are Warsaw local
(CET). Scope as changed by the owner during the lane: **audit only**; the integrator is built by a later lane
from this report. A first prototype of an ego-state estimator with a vision gate, written before the scope
change, is parked unintegrated on branch `l35-prototype-estimator` (commit 88afd13, `api/roadsense_api/egostate.py`;
not for merge, mentioned in section 7 only because its replay numbers inform the thresholds).

Data: recorder sessions copied from the phone to `<datasets>/roadsense-rec/20261119-*` (good: 104128, 113201,
065629; bad: 151146, 153636; also 135622, 145155 (parked, standstill only), 161325 (no vehicle feed)), and the
feeder's GNSS messages extracted from the car computer's trip logs (`grep '"dir":"POS"'
~/cyberpunk/logs/trips/trip_20261119_*.ndjson` on the phone -> `<datasets>/roadsense-rec/trip-pos/pos_20261119.txt`,
7172 lines). Analysis tool: `tools/ego_audit.py` (added in this lane; analysis only, nothing in the stack uses it).

## 0. Findings in one page

1. **There is no integrator.** Each consumer takes the ego motion on its own: vehicle `ego` when a sample is
   within 0.5 s of the frame, else the road model's `v_ego` / `pose`. Three independent copies of that rule
   (lead, lanes, view), two consumers that inherit the lanes tracker's choice (roadgeom, egolane; roadgeom adds its
   own yaw-rate choice), and one consumer that never takes the vehicle at all
   (the speed-limit / rules odometer, `speedlimit.py:141-159`, integrates the road model's `v_ego` and yaw rate
   even when CAN speed is there). No source is ever checked against another; nothing has a notion of a sick
   source. mapd's EKF fuses GNSS and vehicle motion but only for the position, and its output never reaches the
   trackers.
2. **The 8-degree camera incident is trivially visible in the residuals.** Healthy drives: road-model speed /
   vehicle speed = 1.00 (5 s median, p1-p99 0.90-1.14), noise sigma 0.25 + 0.035 v m/s, lag 0.10-0.15 s.
   Bad drives: 5 s median ratio 0.09 (151146) and 0.42 (153636), 86-95 % of the evidence time outside any
   healthy bound; a 5 s window rule fires **0.9-3.1 s after the first frame at >= 3 m/s** on the bad drives and
   **never** in the 819 s of moving evidence of the good ones. GNSS speed alone (head unit, poor) also shows it:
   vision / GNSS median ratio 0.08 on 151146 against 0.84-1.08 on good drives.
3. **The road model's yaw rate stayed usable in the incident** (residual sigma 0.005 rad/s, lag 0.15 s) but its
   gain in turns fell from 0.91-0.96 to 0.64-0.85: speed is the clear failure signal, yaw rate a secondary one.
   The model's own `vx_std` also exploded (p95 0.7-0.9 m/s healthy, 5.6-7.9 m/s bad): a free third cue.
4. **The head-unit GNSS is a weak source:** about 1 message per second, 36-61 % of messages repeat the previous
   speed and course, 3-49 % repeat the position, timestamps are arrival times, and the best-fitting lag against
   CAN speed is 2-7 s (inconsistent between drives); speed residual robust sigma 0.45-2.4 m/s; its course does not
   follow the yaw rate (gain 0.04 in turns on 104128). It is good enough to catch a 10x speed error, not to
   steer anything fine. mapd's GNSS-lag estimator only searches 0-3 s (`vehicle.py:297`).
5. **The car sends everything it has for ego motion** (speed, gear, four wheel speeds, steering angle, yaw-rate
   sensor, longitudinal and lateral acceleration, brake, accelerator, cruise, turn signal hook) at 10 Hz with
   per-signal quality. What is missing is GNSS quality (fix time, repeats) and a better GNSS source; the
   recorder caps `ego` at 10 Hz and does not record the raw `gnss` messages at all.
6. **Replay fidelity bug:** the recorder's `slim_road` (`recorder.py:64-66`) drops `road_model`, so every replay
   of a recorded session through `lead.py` / `egolane.py` runs the **supercombo profile** although the car ran the
   vision model (and `lead._measure` takes `v_ego` instead of `pose.vx` for the relative speed). Any builder
   measurement on recordings must restore it (all recordings of 2026-11-19 are vision-model).

## 1. Data-flow map as it is today

```
camera (USB DVR) ─ camerad-dvr ─> frame ring (NV12 1280x720, ts = CLOCK_MONOTONIC when the frame became readable)
   ├─ modeld (driving_vision + driving_policy | supercombo) ── road.sock ── road, calib ──┐
   ├─ signd (roadsense-signs) ───────────────────────────── signd.sock ── signs ─────────┤
   ├─ signd --lights ──────────────────────────────────────── lightd.sock ─ lights ──────┤
   ├─ envd ────────────────────────────────────────────────── envd.sock ── env ─────────┤
   └─ laned (display only) ─────────────────────────────────── laned.sock ─ display_lanes ┤
car computer feeder ─ position.sock ─> mapd ─ mapd.sock: map (2/s) ─────────────────────┤
  (gnss, motion, controls, body,        ├─ ego.sock: ego (vehicle motion, <= 20/s) ─────┤
   vehicle, objects, reset)             └─ vehicle.sock: vehicle (2/s + on change), objects┤
                     modeld.sock road (only with vision_odometry on) ─> mapd EKF        │
                                                                                         v
api (one Python process): state.py on_message  ── raw inputs straight into every tracker ──>
   lead (lead.py) · lanes (lanes.py) · ego lane (egolane.py) · road geometry (roadgeom.py) · ego light (lights.py)
   · sign context (signctx.py) · speed limit + rules (speedlimit.py, rules.py) · fused (fusion.py) · view (view.py,
   roadview.py) · /preview overlay (overlay.py) · recorder (recorder.py)      ──> /ws, /v2/* ──> MFD (car computer)
```

### 1.1 Inputs: rate, clock, latency, units, frame, uncertainty

Rates and gaps measured with `tools/ego_audit.py rates` (section 6 has the command); everything else from the
code and docs/ARCHITECTURE.md / docs/VEHICLE_API.md.

| input (topic) | producer | rate on the car (measured) | timestamp: clock and meaning | latency to the api | units / frame | uncertainty fields |
|---|---|---|---|---|---|---|
| frames | camerad-dvr | camera ~20/s (the ring); the recorder keeps 1 `camera` msg/s | CLOCK_MONOTONIC right after the info-sector poll that showed a new frame (poll ~210/s, ~5 ms resolution, ARCHITECTURE "camerad-dvr"); the camera's own exposure-to-USB delay is unknown and not in it | - | NV12 1280x720; intrinsics from the ring header / `calib` (`focal_px` 975 on 104128/151146, 690 on 065629) | none |
| `road` (vision model) | modeld (`supercombo.cc:403-500` writes the message for both models) | 10.9-13.5/s (dt p50 0.053 s, p99 0.156 s); `frames_skipped` p50 1-9 | `ts_mono_ns` = the camera time of the newer frame of the 200 ms pair (`main.cc:647`); `pair_dt_ms`, `frame_seq_prev`; `ts_real_ns` from the offset | camera -> /ws median 84-108 ms, p90 94-130 ms (L28-phone-proofs.md:101, :238); recorder file order bounds it from below: p95 76-94 ms | device frame x fwd, y RIGHT, z DOWN, origin camera; m, m/s, rad/s | `pose.*_std`; `leads[0].x_std/y_std/v_std/a_std`; `lane_lines[].y_std`, `prob`; `road_edges[].y_std`; `plan.x_std/y_std`; `leads[].prob`; `valid` (finite outputs) |
| `road.pose` | idem | per frame | describes the 200 ms pair: measured to sit **0.10-0.15 s** behind the vehicle speed / yaw rate (section 6.2) | as `road` | vx m/s, yaw_rate rad/s **positive right** (z down) | `vx_std` 0.2-1.3 m/s healthy, up to 8 in the incident |
| `road.v_ego` | idem | per frame | vision model: = `pose.vx` of the run; supercombo: the speed fed into the run | | m/s | none |
| `road.leads[0..2]` | idem | per frame; recorder keeps `leads[0]` only (`recorder.py:80-82`) | frame time; `t` = +0/+2/+4 s hypotheses | | x,y m; v,a over ground (the model adds its own ego speed) | `prob`, `x_std`, `y_std`, `v_std`, `a_std` |
| `road.lane_lines`, `road_edges` | idem | per frame | frame time | | y at 33 fixed x (m), right positive | `prob` (lines), `y_std` |
| `road.plan` | driving_policy at `policy_hz` 5 (held between runs: `plan.held`, `age_ms`) | 5/s new, every frame re-sent | the frame the plan was made on is `age_ms` earlier | | 33 points in time (t), x,y,z, v, a, rates | `x_std`, `y_std`; recorder keeps x,y,z only |
| `calib` | modeld | at start, on block end / warp change | frame time | | rad, m, px | `online.spread_*`, `se_*`, `state`, `accepted/seen` |
| `signs` | signd | 5-10/s (L28) | frame time | detector 85 ms + queue | boxes in frame px; classes | `score`; track id |
| `lights` | signd --lights | about 3/s | frame time | `lights_ms` ~160 ms | boxes px; api adds `dist_m` (housing size), `lateral_m` (from the plan) | per-track `conf`, `agree`, `amb` |
| `env` | envd | ~1/s | frame time | `env_ms` | class labels | per-group score / confidence |
| `display_lanes` | laned | per its frame | frame time | | as `lanes` | `y_std` optional |
| feeder `motion` -> `ego` | car computer -> mapd relay (`engine.py:235-315`, `__main__.py:127-139`) | feeder 10/s (`motion_hz`), relay <= 20/s; **recorder caps at 10/s** -> 6.6-8.2/s recorded, gaps 8-119 s when the feeder stops | `ts_mono_ns` = panda receipt of the 0x0B4 frame (feeder docstring; bus-to-USB a few ms, <= 20 ms read timeout) | ~ms | m/s signed along heading (negative in reverse); yaw rad/s **positive left**; accel m/s2 | no std; `quality` per signal arrives in `vehicle.signals` (speed `good`; yaw rate, wheels, accels `uncalibrated`); `standstill` = speed and every wheel exactly 0 |
| feeder `gnss` | head unit -> car computer -> mapd (`engine.py:348-430`) | ~1/s (0.84-1.0 on 2026-11-19, gaps up to 41 s) | head unit sends no `age_ms`/`utc_ms` in these logs -> arrival time minus 20 ms; best-fit lag vs CAN speed **2-7 s** | not relayed to the api at all; only `map.input.gnss` (fix, age, hacc, sats) and the EKF output `map.position` | lat/lon deg, speed m/s, course deg clockwise from north | `hacc_m` (1-8 m claimed; mapd scales it up to 6x by its innovations, ARCHITECTURE "Receiver accuracy"), `speed_acc_mps` / `course_acc_deg` not sent |
| `map` | mapd | 2/s | step time | 0.5 s period | position, heading deg cw, speed (EKF), road context | `hacc_m`, `heading_acc_deg`, `confidence`, match confidence |
| `vehicle` | mapd | 2/s + on change | step time | | every host signal | `signals.*.age_s`, `quality`; calibration estimates with `n`, `converged` |
| `objects` | mapd relay of range sensors | none on this car | scan time | | camera frame when mounts known | per object as fed |

Clock: everything is the phone's CLOCK_MONOTONIC (`ts_mono_ns`), which **restarts at every boot** (the phone
rebooted at least at ~14:49 and ~15:35 on 2026-11-19). A replay that joins logs of different producers must
match boots by wall time (the trip-log join in `tools/ego_audit.py load_pos` does).

### 1.2 Where raw inputs enter logic today (each marked line would read the integrator instead)

Routing: `api/roadsense_api/state.py:205-224` (`ego`, `vehicle`, `objects`, `map` fan out to the trackers),
`:241-281` (per `road` message: lead, lanes, ego lane, road geometry, view, fused). Offline mirror:
`tools/replay_lib.py:91-140` (`Pipeline.feed`).

| consumer | raw read (file:line) | what | integrator should supply |
|---|---|---|---|
| lead tracker | `lead.py:364-376` `on_ego` buffer; `:378-399` `_ego_at` (vehicle within 0.5 s, else `road.v_ego` / `pose.vx`, vision standstill after 1 s under 0.15 m/s) | ego speed + standstill | **-> integrator** ego speed, std, standstill, source health |
| lead tracker | `lead.py:402-422` `_measure`: `leads[0]` x, y, v, a, stds; relative speed = `v[0] - pose.vx` (vision) / `- v_ego` (supercombo) | lead measurement, relative speed **from the same broken model** | **-> integrator** vision health / validity gate; the lead's own measurement stays raw |
| lead tracker | `lead.py:286-291` brake from `vehicle`; `:275-284`, `:293-362` radar `objects` | brake, radar | **-> integrator** (radar fusion belongs there per the owner's requirement) |
| lanes tracker | `lanes.py:165-179` `on_ego` / `_speed` (vehicle within 0.5 s, else `v_ego`); `:245-247` `pose.yaw_rate`, `pose.vy` for the ego-motion compensation (off by default) | ego speed, yaw | **-> integrator** |
| lanes tracker | `lanes.py:184-206` lane lines / edges with `y_std`, `prob` | lines | stays raw, plus a vision validity flag |
| view | `view.py:210-215` `on_ego`; `:389-395` vehicle within 0.5 s else model; `:397-407` `on_road`: `v_ego`, `pose.yaw_rate`, `plan` (-> `view.path`) | ego motion for carrying objects; path | **-> integrator** ego; plan validity |
| view.road | `roadview.py:106-116` speed from view; `:160` `map.position` | distance driven | **-> integrator** |
| road geometry | `roadgeom.py:103-104` `on_ego`; `:229-240` speed from `lanes.ego`, yaw from vehicle `ego` else the model's, turn signal; `:118` `map.position` | ego motion, intent | **-> integrator** |
| ego lane | `egolane.py:124-126` turn signal; `:229` speed from `lanes.ego` | speed gate (2 m/s) | **-> integrator** |
| speed limit / rules | `speedlimit.py:141-159` odometer and heading from `road.v_ego` / `pose.vx` and `pose.yaw_rate` **always** (never the vehicle); `rules.py:471` reports it as `v_ego_kmh` | odometer for "sign in effect" distances, turn detection | **-> integrator** (today wrong by 6-10x in the incident, and noisier than CAN on every drive) |
| ego light | `lights.py:96`, `:155`, `:227` the plan for lane relevance; `state.py:185` latest `road`, `calib` | path | **-> integrator** plan validity |
| sign context | `signctx.py:44` raw `leads[0]` (prob >= threshold) to explain a red round sign by a lead's lights | raw lead | **-> integrator** (should read the tracked, validated lead) |
| fused | `fusion.py:346` raw `lane_lines`; `:448` lead summary; `:456-457` `map.position` | | **-> integrator** |
| /preview, status page | `overlay.py:80-98` raw plan, lines, edges, `leads[0]` | debug display | may stay raw (debug), but should show the health |
| mapd EKF | `engine.py:235-315` `motion`; `:348-430` `gnss`; `:325-345` `on_vision` (only with `vision_odometry`, `__main__.py:231-256`) | position | already a fusion, for position only; the integrator's ego state and mapd's EKF should share one motion estimate (or mapd consumes the integrator's) |
| modeld online calibrator | `modeld/calibrator.h:41-46` gates on its own `pose.vx` >= 6.7 m/s, yaw rate, `vy_std` | camera pitch / yaw | needs the integrator's speed / health; today it accepted 100 garbage samples in the incident (`calib.online` accepted 100 of 19108 on 151146, 100 of 4975 on 153636, state `collecting`) and never said "camera moved" |
| MFD (car computer, read-only) | `roadsense_scene.py:60` topics; `:141-198` raw `road.leads[0]` / lines **when `lead` is older than 0.5 s** (`:85`, `:1088`); `:242-251` `lead_to_scene` (present false -> no lead); `:910-954` `view` objects; alerts `alerts/inputs.py:240-250` tracked lead, else `fused.lead`, else raw `leads[0]` | | the integrator's validity must also reach `view` and `lead`, and the MFD must not fall back to raw `leads[0]` while roadsense says "invalid" (car-side change) |

## 2. What the Prius backend sends (read-only: `cyberpunk_computer/backend/position_feeder.py`)

| signal | CAN | sent? | quality declared | note |
|---|---|---|---|---|
| `speed_mps` | 0x0B4 | yes, 10 Hz | good | true speed (speedo reads 12 % high) |
| `gear` P/R/N/D/B | 0x120 | yes | good (B unobserved) | |
| `wheel_speeds_mps` x4 | 0x0B1 / 0x0B3 | yes (API 1.1 full mode) | uncalibrated (2.2 % below 0x0B4; L/R labels third-party) | |
| `steering_wheel_deg` | 0x025 | yes | good | ratio speed-dependent 21-29 |
| `yaw_rate_rps` | 0x022 | yes (API 1.1), offset learned at standstill | uncalibrated (scale 0.0044 rad/s/count, not on a full circle) | mapd uses it first: `yaw_source: "yaw_rate"` in every recorded `ego` |
| `accel_long_mps2`, `accel_lat_mps2` | 0x023 | yes | uncalibrated (r 0.93 / 0.95; long includes grade, lat +0.35 m/s2) | relayed in `ego`; **no consumer reads them** |
| `brake`, `accelerator`, `cruise`, `cruise_set_speed_mps` | 0x230, 0x244, 0x5C8, 21D3 | yes (`controls`) | good / unverified / good / uncalibrated | lead reads `brake` only |
| lamps, door, outside temperature | 0x57F, ... | yes (`body`) | good | |
| turn signal | hook only | **no** (`turn_signal: null` in every recorded `ego`) | | roadgeom / egolane intent unused |
| `brake_pressure`, `parking_brake`, `wipers`, `rain`, `ambient_light_lux`, `fog_rear` | | **no** (null in `vehicle`) | | wipers would explain vision drop-outs |
| GNSS | head unit HTTP push (or phone modem, `phone` source) | yes, ~1/s, `fix`, lat/lon, `hacc_m`, `speed_mps`, `course_deg`, `alt_m`, `sats`; **no `age_ms`, no `utc_ms`, no `speed_acc_mps` / `course_acc_deg`** | | arrival-time stamps, repeated fixes (section 6.3) |

Car-side requests that follow from this audit (to be specified to the car repo, not changed here):
1. Head unit: send `age_ms` (Android `elapsedRealtimeNanos - Location.getElapsedRealtimeNanos`) or `utc_ms`, and
   send each Location once (no re-post of the last one); add `speed_acc_mps`/`course_acc_deg`
   (`Location.getSpeedAccuracyMetersPerSecond`, `getBearingAccuracyDegrees`). Alternatively prefer the phone's
   own modem GNSS (`phone` source) if its timing is better: not measured.
2. The turn signal (the feeder has a hook, nothing feeds it) would serve roadgeom / egolane.
3. The MFD must honour a future "invalid" lead / lanes and not fall back to raw `road.leads[0]` while the
   `lead` topic is fresh (`roadsense_scene.py:1088`); the alerts' third fallback likewise.

## 3. Existing filters and estimators

| estimator | where | state | inputs | assumptions | overlaps |
|---|---|---|---|---|---|
| position EKF | `mapd/roadsense_mapd/posfilter.py` (class at :23), driven by `engine.py:205-222` | east, north, heading, speed, yaw-rate bias | yaw rate as control (vehicle: gyro > wheel-speed difference > steering; vision when switched on); speed measurement (vehicle 0.3 m/s; vision `a + b v`); GNSS position (gated 99.9 %, `hacc` scaled), speed, course; map cross-track / bearing while dead reckoning | the vehicle speed is truth-like; GNSS `hacc` may lie (adaptive scale) | the only fused speed in the stack, but published only as `map.position.speed_mps` at 2 Hz and used by nobody for ego motion |
| vehicle calibration | `mapd/roadsense_mapd/vehicle.py:266-400` (`Calibration`) | yaw-rate bias (standstill mean), gyro scale vs wheels and vs GNSS course, wheel-yaw scale, speed scale vs GNSS, GNSS lag (0-3 s grid) | `motion`, GNSS fixes | GNSS speed and course are good references; lag <= 3 s | on 2026-11-19: speed scale vs GNSS 0.83 (n 9, not converged), yaw scale vs GNSS course 0.75 (n 1): consistent with the poor GNSS of section 6.3; lag search too short for the measured 2-7 s |
| lead Kalman | `api/roadsense_api/lead.py` (`_KF` :180, tracker :219) | distance, lead speed, lead accel; ego speed an input | `leads[0]` x, v - pose.vx, stds; ego speed (vehicle or model); radar objects; brake | the camera's lead and its relative speed are sane; nothing checks the ego motion the model used | duplicates the ego selection rule; contains a radar fusion (off) that would belong to an integrator |
| lanes tracker | `api/roadsense_api/lanes.py` | per line and point: lateral offset + variance, presence with hysteresis | lane lines / edges with `y_std`, `prob`; ego speed (vehicle or model), pose yaw/vy (compensation off); map lane count prior | lines are in the car frame and stay there between frames | ego selection duplicated |
| ego lane HMM | `api/roadsense_api/egolane.py` | discrete distribution over lanes | `lanes`, map lanes, turn signal | | speed from `lanes.ego` |
| road geometry | `api/roadsense_api/roadgeom.py` | joined camera / map centre line | `lanes`, plan, `map.ahead.geometry`, ego yaw / speed | | ego selection duplicated |
| ego light selector | `api/roadsense_api/lights.py:374-406` | ego light track with hysteresis | `lights`, plan, `lanes`, map signals | the plan is valid | |
| view filters | `api/roadsense_api/view.py` | per object a Kalman filter in the car frame; line smoothing | measurement topics + ego motion | ego motion from vehicle or model | ego selection duplicated |
| speed-limit odometer | `api/roadsense_api/speedlimit.py:141-159` | odometer, heading integral | `road.v_ego`, `pose.yaw_rate` | the model's speed is right | ignores CAN speed and mapd's odometer (`map.position.odometer_m`) |
| online calibrator | `modeld/calibrator.h`, `calibrator.cc` | block medians of pitch / yaw from the pose direction | `pose` vx, vy, vz, yaw rate, `vy_std` | the model's own speed decides what is "fast" | needs an outside speed; cannot see a moved camera that kills the model's motion sense |
| signd / lights trackers | roadsense-signs (SORT, light tracker) | image tracks | detections | | outside the api |

## 4. Measured source characterisation (car recordings, 2026-11-19)

### 4.1 Streams (rates, gaps)

| session | road /s | road gaps > 0.5 s (max) | ego recorded /s | ego gaps (max) | GNSS (feeder) /s | GNSS fix in `map` | calib |
|---|---|---|---|---|---|---|---|
| 104128 good, 38 min | 12.4 | 20 (11.7 s) | 7.5 | 19 (118.8 s) | 0.92, max gap 41 s | 3d 91 %, none 8 % | pitch 0, yaw 0, f 975 |
| 113201 good, 15 min | 10.9 | 25 (12.0 s) | 6.6 | 25 (20.4 s) | 0.84, max gap 7.9 s | 3d 49 %, none 51 % | pitch 0, yaw 0 |
| 065629 good, 4 min | 13.5 | 0 | **no vehicle feed** | | 0 | none 99 % | pitch 0, f 690 |
| 151146 bad, 23 min | 12.8 | 5 (7.5 s) | 8.0 | 5 (8.2 s) | 0.98, max gap 7.0 s | 3d 98 % | pitch 0.0561, yaw -0.0621, online `collecting` |
| 153636 bad, 9 min | 12.9 | 0 | 8.2 | 0 | 1.00 | 3d 99 % | as 151146 |

Road-model latency: the newest vehicle sample already written when a `road` line was written is 9-15 ms (median)
and 76-94 ms (p95) past the frame's time: a lower bound of the road message's delay (the recorder caps `ego` at
10/s). L28 measured camera -> /ws 84-108 ms median.

### 4.2 Road-model pose against the vehicle (healthy drives 104128 + 113201)

`tools/ego_audit.py vision` (vehicle sample taken 0.1 s before the frame):

| vehicle speed | n | bias vision - vehicle (m/s) | robust sigma (m/s) | ratio (median) | model `vx_std` p50 / p95 |
|---|---|---|---|---|---|
| standstill | 16004 / 6958 | 0.003 / 0.018 | 0.001 / 0.011 | - | 0.06 / 0.18 |
| 0-3 | 1418 / 629 | 0.10 / 0.12 | 0.23 / 0.20 | 1.06 / 1.09 | 0.29 / 0.69 |
| 3-6 | 2491 / 659 | 0.08 / 0.31 | 0.39 / 0.45 | 1.02 / 1.07 | 0.33 / 0.94 |
| 6-9 | 2664 / 578 | 0.11 / 0.07 | 0.49 / 0.39 | 1.01 / 1.01 | 0.40 / 0.78 |
| 9-12 | 1037 / 339 | 0.06 / -0.26 | 0.74 / 0.45 | 1.01 / 0.98 | 0.45 / 0.82 |
| 12-15 | 1163 / 145 | -0.07 / -0.27 | 0.84 / 0.53 | 1.00 / 0.98 | 0.52 / 0.72 |
| >= 15 | 1485 / - | -0.07 | 0.78 | 1.00 | 0.57 / 0.74 |

* Speed noise model that fits both: **sigma = 0.25 + 0.035 v m/s**, bias within +-0.3 m/s; tails heavier than
  Gaussian (p1/p99 about +-1.5-2 m/s at 3-15 m/s). The model's `vx_std` is of the right size (0.3-0.8 m/s).
* Yaw rate (v >= 3 m/s): bias 0.0016 rad/s, robust sigma 0.0062-0.0065 rad/s, p1/p99 -0.05/+0.05; gain in turns
  (|vehicle yaw| > 0.08 rad/s) 0.963 / 0.908 (the gyro's scale is `uncalibrated`, so part of this is the gyro).
* Lag (smallest robust residual spread over lags -0.3..+0.6 s): speed **0.15 / 0.10 s**, yaw rate **0.10 /
  0.125 s** (vision behind the vehicle). Consistent with the 200 ms image pair (centre 0.1 s before the frame)
  plus a few tens of ms of camera delay. A plain high-passed cross-correlation is not reliable here (speed
  correlation 0.2-0.6 with lags of opposite sign); the yaw-rate correlation gives 0.075 s at r 0.86 on 104128.
* GNSS speed against vision: vision / GNSS median ratio 1.075 (104128), 0.839 (113201), sigma 2.2-4.2 m/s
  (dominated by the GNSS, below).

### 4.3 GNSS (head unit, via the feeder's trip-log lines)

`tools/ego_audit.py gnss SESSION... --pos <datasets>/roadsense-rec/trip-pos/pos_20261119.txt` and the
repeat / displacement check in section 6:

| session | msgs | repeated speed / course / position | best lag vs CAN speed (speed field) | bias, robust sigma at that lag (m/s) | displacement speed: lag, sigma | GNSS speed while the car stands p95 / p99 | course-rate gain in turns |
|---|---|---|---|---|---|---|---|
| 104128 | 2020 | 60 / 61 / 46 % | 5.2-6.1 s | -0.6, 1.4 | 1.8 s, 1.6 | 1.1 / 6.3 m/s | 0.04 (n 83) |
| 113201 | 414 | 36 / 36 / 3 % | 3.0-7.6 s (flat) | -2.6..-2.9, 2.4-2.8 | 7.6 s, 2.5 | 6.2 / 6.3 | -0.04 (n 12) |
| 151146 | 1343 | 59 / 62 / 49 % | 3.0-4.2 s | -0.6, 0.9-1.0 | 2.0 s, 1.2 | 1.7 / 3.6 | 0.55 (n 18) |
| 153636 | 535 | 46 / 50 / 36 % | 1.8-5.3 s | -0.1..-0.2, 0.45-0.7 | 3.4 s, 0.8 | 2.9 / 5.7 | - |

Reading: the head unit re-posts its last Location (repeats), the feeder stamps arrival time (no `age_ms`), so
the effective delay is seconds and varies; speed reads 4-10 % low at the best lag; the course carries almost no
turn information at 1 s differences. Healthy residual of GNSS course rate vs yaw rate: sigma 0.02-0.04 rad/s
without turn gain, i.e. noise. As a reference it is usable only for coarse checks over windows of tens of
seconds. Fix share: 3d in 49-99 % of the `map` messages; `hacc_m` claimed 1-8 m.

### 4.4 The camera incident in each residual

`tools/ego_audit.py incident` (evidence frame = vehicle >= 3 m/s; "bad" = |vision - vehicle| > 3 x (0.25 +
0.035 v); 5 s windows of evidence time, at least 15 frames):

| session | evidence s | 5 s median ratio p1 / p50 / p99 | bad share p50 | rule ratio5s outside [0.7, 1.4] | rule bad share >= 0.6 | rule ratio5s < 0.5 | raw `leads[0]` p >= 0.5: share, distance p50 |
|---|---|---|---|---|---|---|---|
| 104128 good | 684 | 0.90 / 1.00 / 1.14 | 0.00 (p99 0.33) | never | never (one window at 0.6 exactly at 1051.9 s, 0 s duration) | never | 67 %, 12.8 m |
| 113201 good | 135 | 0.92 / 1.01 / 1.12 | 0.00 (p99 0.24) | never | never | never | 43 %, 11.7 m |
| 065629 good | - | no vehicle feed: no check possible | | | | | |
| 151146 bad | 333 | -0.03 / 0.09 / 0.90 | 0.97 | **2.5 s** after the first evidence frame, true 88 % of the evidence time | 3.1 s, 90 % | 3.0 s, 86 % | 69 %, **5.5 m** |
| 153636 bad | 218 | -0.01 / 0.42 / 2.45 | 0.89 | **0.9 s**, 84 % | 0.9 s, 95 % | 5.2 s, 60 % | 78 %, **5.1 m** |

Other residuals in the incident (`tools/ego_audit.py vision` on 151146 / 153636):

| residual | healthy | incident |
|---|---|---|
| vision speed / vehicle speed, per bin | 0.98-1.09 | 0.03-0.20 (151146), 0.10-0.87 (153636); wild both ways (p99 ratio 2.4-4.4) |
| vision - vehicle speed at 12-15 m/s | -0.07 +- 0.84 | **-12.3** +- 2.0 |
| model `vx_std` p95 | 0.7-0.9 | 4.1-8.2 |
| vision yaw - vehicle yaw | 0.0016 +- 0.0063 | -0.0015 +- 0.0058 / -0.0002 +- 0.0052 (still fine) |
| yaw gain in turns | 0.91-0.96 | 0.64 / 0.85 |
| vision / GNSS speed ratio | 0.84-1.08 | 0.08 (151146), 0.40 (153636) |
| online calibrator | (not run on these) | 1 block from 100 accepted of 19108 frames (151146), `collecting`; never flagged anything |

What the stack did with it (lead tracker replay, `tools/lead_replay.py SESSION --csv`; **supercombo profile, see
finding 6**): with the vehicle above 5 m/s a lead was "present" in 93 % / 100 % of the frames at a tracked
distance p10/p50/p90 of 4.8/7.0/10.6 m and 4.3/6.1/10.2 m, longest single track 178 s / 104 s; on the good drives
51 % / 80 % at 11/37/62 m and 10/16/50 m. The prototype gate (branch `l35-prototype-estimator`, two Kalman filters,
the same bad-share rule with 15 frames / 5 s to fire and 50 frames at <= 15 % over 10 s to recover) fired 3.1 s and
0.9 s after the first evidence frame and stayed in `fault` for 1155 s and 544 s; on the good drives it never
fired, but its windowed bad share reached 0.50 (104128) and 0.47 (113201) against the 0.6 threshold, so a z-score
rule alone has thin margin; the 5 s median ratio has a wide one (healthy 0.90-1.14, incident 0.09 / 0.42).

## 5. Gaps against the requirement, and constraints for the builder

Gaps (the owner's requirement: all inputs into one integrator first, trackers / map / display after it, with
metadata; extensible to more cameras and radars):

1. No shared ego state (speed, acceleration, yaw rate with uncertainties) and no per-source health; three
   independent copies of "vehicle within 0.5 s else model" plus two inheriting ones (section 1.2) and one
   consumer on vision only.
2. No consistency check anywhere: vehicle vs vision vs GNSS are never compared online; the measured healthy
   statistics above make such a check cheap and decisive for gross failures.
3. Raw GNSS never reaches the api (only mapd); `gnss` is neither relayed on `ego.sock` nor recorded.
4. The vehicle accelerations are relayed and unused; acceleration would also give exact interpolation of the
   10 Hz vehicle speed to frame times (vision lag 0.1-0.15 s at up to 3 m/s2 = 0.3-0.45 m/s).
5. Camera-derived outputs carry no validity: `lead`, `lanes`, `view` have no "valid / reason" fields; the MFD
   falls back to raw `leads[0]` when `lead` goes stale.
6. Camera calibration health is invisible: the online calibrator judges "fast" by the model's own speed;
   `/v2/health.calibration` says `collecting` while the camera is 8 degrees off.
7. Radar fusion lives inside the lead tracker (`lead.py` radar part) instead of an integrator; multi-camera
   inputs have no place at all (ring, modeld and `road` are single-camera; `view` / `lead` assume one camera frame).
8. Replay fidelity: `road_model` dropped by the recorder; `ego` capped at 10/s; `lead`, `lanes` not recorded (by
   design, recomputed); plan stds, `leads[1..2]`, `meta`, `desire_*` not recorded; GNSS only in the car's trip
   logs, on a different file and boot-dependent clock.

Constraints a builder must respect:

* **Process boundaries.** modeld (C++), mapd (Python, owns the vehicle input API and everything about position),
  api (one Python asyncio process; all trackers, `view`, recorder). Inputs reach the api over Unix stream
  sockets, JSON lines, newest-never-dropped-in-order, slow clients cut at 1 MiB (ARCHITECTURE "JSON sockets").
  An integrator in the api sees `road`, `ego`, `map`, `vehicle`, `objects`, `signs`, `lights`, `env`; GNSS needs
  a relay from mapd (or the integrator lives in mapd, which then needs every `road` message: today it reads
  `modeld.sock` at most 10/s and only with `vision_odometry`). A separate process is possible (one more socket
  hop, ~ms) and matches "may become its own component".
* **Clock / ordering.** One CLOCK_MONOTONIC; `road` arrives 85-130 ms after its frame time, vehicle samples
  within ms, GNSS seconds late (arrival-stamped): the integrator must handle out-of-sequence measurements and
  publish frame-stamped estimates (measurement topics refer to the frame, no prediction to "now":
  ARCHITECTURE `lead`).
* **Measurement vs display (Guidelines 4, 5).** Measurement topics fresh and unsmoothed for looks, held /
  coasted values marked, "unknown" with a reason instead of a guess; `view` is the only smoothed channel and never
  feeds back.
* **CPU on the phone.** The api uses about 9 % of the little and 11-12 % of the big cores already
  (L28-phone-proofs.md:235); the trackers cost 0.1-0.2 ms per frame each on the laptop; the load governor sheds
  in a stated order and must keep measurement outputs last. A 4-state Kalman filter per input is negligible (the
  prototype: 12 s CPU for a 38-min session on the laptop, ~0.15 ms per road frame including the vehicle and
  GNSS updates).
* **Contracts.** docs/ARCHITECTURE.md (`road`, `ego`, `lead`, `lanes`, `view`, `/v2/health`), docs/VEHICLE_API.md
  (feeder API 1.1, `ego` fields, quality vocabulary). Additive fields keep api 2; changes of meaning need api 3 with
  the MFD kept connected (Guideline 6). The MFD reads `road`, `lead`, `lanes`, `view`, `fused`, `map`
  (`roadsense_scene.py:60`) and draws `lead.present`, lane `present`; it falls back to raw `road.leads[0]` after
  0.5 s without `lead`.
* **Replay tooling.** `tools/replay_lib.py` (`Pipeline`) mirrors `state.py`; `lead_replay.py`, `lanes_replay.py`,
  `view_replay.py`, `map_replay.py`: an integrator must be added to both `state.py` and `Pipeline`, and its
  numbers must be measured with `road_model` restored (finding 6).
* **Car-agnostic.** Other cars and replays have no vehicle feed and no GNSS (065629 here): the integrator must
  degrade to today's behaviour (vision-only, verdict "unchecked") and never require a source.

## 6. Reproducing

```sh
# copy (phone read-only)
for d in 20261119-104128 20261119-065629 20261119-113201 20261119-151146 20261119-153636; do
  mkdir -p <datasets>/roadsense-rec/$d; scp user@phone:/var/lib/roadsense/rec/$d/messages.jsonl.gz <datasets>/roadsense-rec/$d/; done
ssh user@phone 'cd ~/cyberpunk/logs/trips && for f in trip_20261119_*.ndjson; do grep "\"dir\":\"POS\"" $f | sed "s/^/$f /"; done' \
  > <datasets>/roadsense-rec/trip-pos/pos_20261119.txt
D=<datasets>/roadsense-rec; S="$D/20261119-104128 $D/20261119-065629 $D/20261119-113201 $D/20261119-151146 $D/20261119-153636"
systemd-run --user --collect -p MemoryMax=3G --wait -- bash -c "cd ~/projects/wt/roadsense-l35 &&
  python3 tools/ego_audit.py rates $S > /tmp/rates.txt; python3 tools/ego_audit.py vision $S > /tmp/vision.txt;
  python3 tools/ego_audit.py incident $S > /tmp/incident.txt;
  python3 tools/ego_audit.py gnss $S --pos $D/trip-pos/pos_20261119.txt > /tmp/gnss.txt"
python3 tools/lead_replay.py $D/20261119-151146/messages.jsonl.gz --csv /tmp/lead_151146.csv   # supercombo profile (finding 6)
```
The repeat and displacement-speed columns of 4.3 are printed by `ego_audit.py gnss` (displacement lag scan
-2..8 s in 0.2 s steps).

## 7. Not verified / open

* The camera delay itself (exposure -> USB poll) is not separable from the 200 ms pair: only their sum
  (0.10-0.15 s vs CAN) is measured. CAN receipt delay assumed a few ms (feeder docs), not measured here.
* GNSS lag is poorly determined (flat residual surface, 2-7 s); the phone modem source (`phone`) was not in use
  and is not characterised.
* Thresholds of section 4.4 rest on two good drives with a vehicle feed (819 s of evidence, no motorway above
  ~17 m/s, no rain, no night): a builder should re-check on more drives; the ratio rule's margin is wide, the
  z-share rule's is thin.
* The lead numbers of 4.4 were replayed with the supercombo profile (recorder drops `road_model`).
* Lane lines under the moved camera: lane-line probability fell (median min-prob of the two inner lines 0.27
  good vs 0.06 / 0.19 bad) while the width at 9 m stayed plausible (3.0-3.1 m vs 3.2-3.3 m); whether the lines'
  geometry was wrong is not established without ground truth. The plan collapsed (|plan y| at its end p50 0.2-0.5 m
  vs 3.4-4.4 m good), consistent with a model that believes it barely moves.
* Prototype estimator / gate (branch `l35-prototype-estimator`, 88afd13): not integrated, no tests, not reviewed;
  numbers quoted only as an existence proof.
