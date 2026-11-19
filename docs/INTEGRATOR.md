# The data integrator

Lane L37, 2026-11-19. Design of the component every input passes through before any logic reads it. Builds on
the audit `docs/lanes/L35-ego-fusion-audit.md` (data flow, source characterisation); measured results of what
was built are in `docs/lanes/L37-integrator.md`. Code: `api/roadsense_api/integrator/`.

```
vehicle motion (`ego`)  ─┐                       ┌─ ego state at a requested time (frame time):
vehicle signals          │   source registry      │    speed, acceleration, yaw rate, standstill, each with
GNSS (`gnss`, new relay) ├─> measurement models ─>│    value, std, valid / reason, mode (fused | coast), sources
road model (`road`)      │   time-ordered buffer  ├─ per-source health: ok | unchecked | suspect | fault | stale | absent
range sensors (`objects`)┘   estimator + checks   ├─ pairwise consistency table, faults with reason and isolation
future: camera 2, radars                          └─ gated products: may the camera's lead / lanes / plan be used
                                                         │
        lead · lanes · ego lane · road geometry · speed limit / rules · lights · sign context · fused · view
        (read the integrator's state; none of them picks a raw source any more)
```

## 1. What the data says (the facts the design rests on)

Measured on the car's recordings of 2026-11-19 (audit section 4, re-checked with `tools/integrator_replay.py
characterise`; numbers in the L37 report):

* **Vehicle speed (CAN 0x0B4) is close to exact at 10 Hz.** Leaving one sample out and interpolating its
  neighbours reproduces it with a robust sigma of 0.012-0.020 m/s. The four wheel speeds sit at 0.979-0.981 of
  it with a residual sigma of 0.010-0.018 m/s. The vision model's speed has sigma 0.25 + 0.035 v m/s with heavy
  tails. An optimal filter therefore gives the camera a weight below 1 % whenever the vehicle speaks: with a
  vehicle feed, fusion of the speed is almost a pass-through, and its value lies elsewhere: bridging the feed's
  gaps (8-119 s on these drives, recorded rate 6.6-8.2 /s), exact values at frame times, cars without a feed, and
  above all **checking**.
* **Yaw rate:** the gyro (zeroed at standstill by mapd) is the reference; the rear wheel-speed difference
  follows it with gain 0.947 and sigma 0.010-0.019 rad/s; the vision model's yaw rate is good near straight
  (sigma 0.005-0.007 rad/s, bias 0.0015) and poor in turns (gain 0.85-0.96, rms error 0.03-0.11 rad/s where
  |yaw rate| > 0.08).
* **Longitudinal accelerometer:** equals d(speed)/dt with a slowly moving offset (0-0.2 m/s2: grade,
  calibration) and sigma 0.20-0.25 m/s2; holding it bridges a 0.5-2 s gap of the speed with 30-45 % less error
  than holding the speed.
* **Latency:** the vision pose describes the 200 ms image pair: 0.10-0.15 s before the frame's stamp (flat
  minimum; 0.125 s used). The `road` message arrives 85-130 ms after the frame time, vehicle samples within
  milliseconds: when a frame is processed, vehicle samples *after* its time of validity already exist.
* **Head-unit GNSS:** arrival-stamped, seconds late with a varying lag (2-7 s), 36-61 % repeated messages. A
  per-message test of its speed against the envelope of the vehicle speed over the previous 8 s fails on
  5-52 % of the messages of healthy drives. It can witness a gross error over tens of seconds; it cannot be
  fused into the ego motion and cannot outvote anything.
* **The camera incident** (8 degrees off): against the vehicle, the share of frames whose speed is more than 4
  sigma off, over 5 s of driving, is 0.00 typical / 0.32 at most on the healthy drives and 0.86-0.96 typical /
  0.67-0.73 at the 5th percentile on the bad ones. The model also knows it by itself: its own `vx_std` over
  the characterised sigma, 5 s median, never exceeds 2.6 on three healthy drives (one of them another camera)
  and is 7-13 typically on the bad ones (session 161325, no vehicle feed, same knocked camera: never below
  11.8). Two independent detectors with wide margins; the second works on cars without any vehicle feed.

## 2. Structure

Three layers in one component, all pure functions of the messages (no I/O, no wall clock):

1. **Sources.** A source is described by a `SourceSpec` (id, group, rank, expected rate, stale time, latency
   model, role per quantity) and a **measurement model**: a function from the source's message to a list of
   `Measurement(quantity, t_valid_ns, value, var, flags)`. Quantities today: `speed`, `accel`, `yaw_rate`,
   `standstill` (a flag), plus discrete pass-through signals (`turn_signal`, `brake`) and, for the object
   layer, object detections. Roles: `fuse` (enters the estimate), `monitor` (checked against the others,
   never fused: wheel speeds, wheel-difference yaw rate, GNSS speed), `self` (the source's own stated
   uncertainty, checked against its characterised noise).
2. **Ego estimator.** Two small linear Kalman filters on a common time axis:
   * longitudinal: x = [v, a, b], v signed speed along the heading, a its derivative, b the accelerometer's
     offset; v' = a, a' = jerk noise, b' = slow random walk. Measurements: vehicle speed, vision speed
     (at frame time - 0.125 s), accelerometer (a + b), standstill (v = 0, a = 0 exactly);
   * yaw: x = [r, r'] (ISO 8855: positive left). Measurements: gyro, vision yaw rate (noise growing with
     |r|), standstill (r = 0).
   Measurements are kept in a time-ordered buffer (horizon 2 s). A late one (vision: 85-130 ms late and valid
   another 125 ms earlier) is inserted at its time of validity and the filter is re-run from there (two or
   three updates). A request for the state at time T (the frame time of a `road` message) returns the
   **fixed-lag smoothed** state at T: forward filter through everything in the buffer, one RTS backward pass
   to T. No delay is added: it uses what has arrived by the time the frame is processed, which for the vehicle
   means samples on both sides of T.
   Outliers: a fused measurement whose innovation exceeds 5 sigma is not applied (counted, reported as
   `rejected`); that is single-sample protection, not the fault detector.
3. **Consistency and isolation.** Independent of the filter and never against the fused state (a source must
   not vote for itself):
   * **pair monitors:** for two groups that measure the same quantity, the residual of their raw measurements
     at a common time of validity (the denser source interpolated), normalised by both declared sigmas. Kept
     over a window of *evidence time* (5 s; a sample is evidence when the quantity is large enough to show a
     scale error: either speed >= 3 m/s). Verdict `disagree` when >= 15 samples and the share beyond 4 sigma is
     >= 0.5; back to `agree` when over 10 s of evidence the share is <= 0.25; `unknown` without evidence.
     Standing still is no evidence in either direction: the verdict holds.
   * **self monitors:** the 5 s median of (stated sigma / characterised sigma) above 4 = `self_fault`, back
     below 3 over 10 s.
   * **witness monitors** (GNSS): the ratio of medians over 30 s of evidence; `disagree` outside [0.6, 1.6].
   * **isolation:** a disagreeing pair (A, B) faults the lower-ranked group, unless a third group agrees with
     the lower and disagrees with the higher one (then the higher one is faulted: "by vote"). The fault
     record says which rule decided (`rank`, `vote`, `self`). Ranks: vehicle 3, radar 2 (future), vision 1,
     GNSS 0 (witness only: it can tip a vote, never start a fault).
   * **effect:** a faulted group's measurements leave the estimate at once (the filter is re-run without
     them over the buffer), and every product declared as depending on that group goes out as unknown with
     the reason. Recovery is by evidence only (the pair agrees again, the self monitor is quiet), never by
     time.

Health per source: `absent` (never seen) / `stale` (silent longer than its stale time) / `unchecked` (alive,
nothing to check it against, or no evidence yet) / `ok` (checked and agreeing) / `suspect` (a monitor is
between its thresholds, or a witness disagrees) / `fault`.

### Filter family: why this one

* Linear Kalman filter, two decoupled channels, RTS smoothing over a short buffer. The state is observed
  directly and linearly by every source; nothing here is non-linear enough for an EKF/UKF, and nothing is
  multi-modal enough for a particle filter. The heavy tails of the vision speed are handled where they
  matter (robust window statistics for the verdict, innovation gating for the single sample) instead of a
  heavy-tailed filter, because the vision weight is below 1 % with a vehicle feed.
* **Rejected: one big state with position, heading, speed, biases (mapd's EKF extended).** Position needs the
  map and the GNSS, lives in mapd, and its only absolute sensor is the weak head-unit GNSS; coupling the ego
  motion to it would import that weakness into the trackers.
* **Rejected: a bank of solo filters per source with innovation tests against the fused state.** With a
  dominant source the fused state is that source; a test against it is a test against the vehicle anyway, and
  a sick dominant source would hide. Raw-vs-raw pair monitors give the same answer without contamination and
  extend to any number of groups.
* **Rejected: chi-square (NIS) window test as the fault detector.** The audit showed its thin margin (healthy
  windows reach 0.5 of the frames beyond 3 sigma against a 0.6 threshold); the share beyond 4 sigma has
  healthy max 0.32 against incident p5 0.67.
* **Rejected: applying late measurements through first-order back-propagation (the L35 prototype).** Insert
  and re-run is exact, costs two or three 3x3 updates, and gives the smoothed state for free.
* **Rejected: fusing GNSS speed.** See section 1. It is recorded, characterised, used as a witness.
* **Rejected: a separate process now.** One more JSON hop per message (parse + dump of every `road` message a
  second time) for no gain while all consumers live in the api process. The package has no dependency on the
  api (messages in, dicts out); moving it behind a socket later is a wrapper.

## 3. Objects (lead, radar targets) and the existing estimators

| estimator | decision | why |
|---|---|---|
| lead Kalman (`lead.py`) | **object layer, fed and gated by the integrator.** It keeps its filter and life cycle; it no longer selects the ego speed, reads the brake or decides whether the camera is believable: the ego state (speed, std, standstill), the pass-through signals and the admission of each sensor's detections come from the integrator. | Its model and tuning carry four lanes of measured work (L14, L27, L29, L32); nothing in the incident was wrong with the filter, everything with its inputs. |
| radar association (inside `lead.py`) | stays where it is in this lane, admission through the integrator's source health; **target design:** an object layer with one track list in the car frame, per-sensor measurement models (camera: x, y, relative speed; radar: range, azimuth, range rate), association per sensor, and the lead chosen from the tracks by the path. | No radar exists on the car: a multi-target layer cannot be measured yet. The seam (source spec + measurement model + admission) is in place; section 8 says what a radar adds. |
| lanes tracker (`lanes.py`) | fed (speed, yaw rate) and gated (vision validity) by the integrator | geometry in the camera's frame; only the camera measures it |
| ego-lane HMM, road geometry, speed-limit / rules odometer, view | read the integrator's state (the odometer no longer integrates the model's speed) | |
| mapd position EKF | **left in mapd; a source.** mapd keeps position, heading and map matching and relays the raw `gnss` to the integrator. | Position needs the map (cross-track updates while dead reckoning) and lives upstream of the api. Its speed input is the same vehicle speed. Open: with `vision_odometry` on, mapd should honour the vision verdict (it has no vehicle feed in that case, so only the self monitor applies). |
| modeld online calibrator | left in modeld; its state is reported next to the vision verdict | It needs an outside speed to judge "fast"; that needs an input path api -> modeld (not built). The integrator's verdict is what tells a consumer "the camera is not believable"; the calibrator cannot. |

## 4. Time

* One clock: the phone's CLOCK_MONOTONIC (`ts_mono_ns`) on every input. Each source spec carries the offset
  from its stamp to the time of validity (vision pose: -0.125 s; vehicle: 0; GNSS: unknown 0-8 s, which is why
  it is a witness over long windows only).
* Arrival order is not time order: buffer, insert, re-run (section 2). A measurement older than the horizon
  (2 s) is counted as `late` and used by the monitors only.
* **Epochs.** The clock restarts at boot and a replay may join sessions: when any source's stamp goes back by
  more than 2 s, or the newest time jumps forward by more than 30 s with nothing in between, the integrator
  starts a new epoch: buffers and filters are dropped, verdicts are kept as `carried` (a fault survives; `ok`
  falls back to `unchecked`), `epoch` in the output counts up.
* The state is published for a time of validity (the frame time), never predicted to "now"; a consumer that
  needs "now" has the std and the derivative.
* Prediction beyond the newest measurement of a quantity is marked `mode: "coast"` with `age_s`; after
  `max_coast_s` (1 s for speed and yaw rate) the quantity is `valid: false, reason: "no_measurement"`.

## 5. Output contract

Topic `ego_state`, one per `road` message, stamped with the frame time (and at the vehicle's rate while the
camera is silent for more than 0.3 s):

```
{"type":"ego_state","seq":N,"ts_mono_ns":T,"ts_real_ns":...,"epoch":0,
 "speed":    {"value":12.31,"std":0.02,"valid":true,"reason":null,"mode":"fused","age_s":0.04,
              "sources":["vehicle.speed","vision.pose"]},
 "accel":    {... same shape, m/s2},
 "yaw_rate": {... same shape, rad/s, positive LEFT},
 "standstill": {"value":false,"for_s":0.0,"source":"vehicle"},
 "signals":  {"turn_signal":{"value":null,"age_s":null},"brake":{"value":false,"age_s":0.2}},
 "sources":  {"vehicle":{"state":"ok","reason":null,"age_s":0.04,"rate_hz":9.8,"tag":"prius-can"},
              "vision": {"state":"fault","reason":"speed disagrees with vehicle (ratio 0.09, 96 % of 62 frames beyond 4 sigma); own speed std 7.1x its normal",
                         "since_s":412.0,"isolation":"rank","age_s":0.0,"rate_hz":12.8,"usable":false},
              "gnss":   {"state":"unchecked", ...}},
 "checks":   [{"a":"vehicle","b":"vision","quantity":"speed","verdict":"disagree","n":62,"share":0.96,
               "ratio":0.09,"evidence_s":5.0}, {"a":"vision","kind":"self", ...}, ...],
 "rejected": {"vision.pose":3}, "late": {}}
```

Derived measurement topics (`lead`, `lanes`, and the display channel `view`) gain

```
"valid": true | false, "reason": null | "vision: <the fault's reason>" | "ego speed unknown: ...",
"inputs": {"vision":"ok","vehicle":"ok"}          # the health of what the product was made from
```

and when `valid` is false they carry no value: `lead.present` false with `mode: "none"`, every lane line
`present` false, `view` without lead and lines and with `integrity.valid` false. "Unchecked" is not invalid:
a car without a vehicle feed behaves as before and says `inputs.vision: "unchecked"`. Raw `road` stays a
source's raw output on /ws; a consumer that reads it takes its validity from `ego_state.sources.vision`.

## 6. Where it runs, cost

Inside the api process, called from the fan-out (`state.py`) before the trackers; the same class is driven by
`tools/replay_lib.py`. Inputs: the sockets the api already has, plus raw `gnss` relayed by mapd on `ego.sock`.
Budget: a handful of 2x2 / 3x3 updates per message and one smoothing pass per frame; measured cost in the
L37 report. It is not shed by the load governor: it is the cheapest part of the measurement chain and what
makes the rest trustworthy.

Stand-alone use: every tracker that is not handed an ego state builds a private integrator and feeds it the
same messages (`integrator.LocalEgo`), so a tracker in a test or a tool runs the same code path as in the api.

## 7. Replay and tests

* `tools/integrator_replay.py`: sessions through the integrator alone (`characterise`, `detect`, `accuracy`,
  `cost`) and through the whole pipeline (`products`: lead / lanes / view during and outside faults), with
  `--pos` joining the car's trip-log GNSS by wall time for the sessions recorded before `gnss` was.
* The recorder now writes `road_model` (and the other scalar keys the trackers read), raw `gnss`, `ego_state`,
  and `ego` at 20 /s; `replay_lib.read_session(..., road_model=...)` restores the model name for sessions
  recorded before.
* Unit tests (`api/tests/test_integrator.py`): filter against synthetic truth, out-of-order equals in-order,
  smoothing, epochs, fault detection / isolation by rank and by vote / recovery, self monitor, staleness, a
  radar-like source added by a spec and a model only; consumer tests for the invalid outputs.

## 8. How a radar and a second camera plug in

A front radar (object list with range, azimuth, range rate; relayed as `objects`):
1. `SourceSpec(id="radar_front", group="radar_front", rank=2, ...)` and a measurement model that yields
   (a) an **ego speed** measurement from its stationary targets (the mode of -range_rate / cos(azimuth)), which
   joins the speed pair monitors as a third ranked group: with it a wrong vehicle speed is isolated by vote,
   and the camera is checked on cars without a CAN feed; (b) its **objects** for the object layer.
2. Nothing changes in the consumers: they read `ego_state` and the lead; the lead's `inputs` gains `radar_front`.
A second radar is another spec with its mount; a rear one contributes ego speed with the sign turned.

A second camera (another modeld instance publishing `road` with a `camera` id): another spec in group
`vision:<id>` with the same pose model. Its speed and yaw rate join the monitors (two cameras that agree
outvote a vehicle feed; one that disagrees is isolated by vote without any vehicle data), and its detections
enter the object layer through its mount transform. Products name the camera they depend on.

What is general and what is not: the registry, buffer, filters, monitors and isolation know nothing about
cars or cameras. The measurement models and their noise numbers are this car's and this model's (0x0B4 speed,
the 200 ms pair, `vx_std` semantics); they live in `integrator/sources.py` and are meant to be measured again
per source, not abstracted.

## 9. Limits known at design time

* Thresholds rest on two healthy drives with a vehicle feed (about 14 min of moving evidence, no motorway, no
  rain, no night) and one incident. The margins are wide (section 1); their tails are not known.
* A camera that is wrong while the car stands (before the first metres of a drive) cannot be told from a
  healthy one by motion: state `unchecked` until evidence arrives. A fault is remembered across restarts
  (`carried`) so that a drive that starts with yesterday's knocked camera starts as `fault`.
* A vehicle speed that is wrong with no second ranked source: the camera is blamed (rank), the product goes
  unknown. That errs to the safe side but names the wrong culprit; the reason text says "isolated by rank".
