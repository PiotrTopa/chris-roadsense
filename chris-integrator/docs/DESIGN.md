# The data integrator

Lane L38, 2026-11-19. Design of the component every input passes through before any logic reads it: one box
in the architecture, with a full ego state, an object layer, and sources that plug in without touching the
core. It replaces the design of lane L37 (ego motion only, inside the api); what is kept from L37 is listed in
section 12. Facts about the inputs: `docs/lanes/L35-ego-fusion-audit.md` and section 1. Measured results of
what was built: `docs/lanes/L38-integrator.md`. Sections marked "(as built)" were corrected after the code and
the replays; the report says what is not built.

```
 inputs (any process, any rate, any latency)          THE INTEGRATOR  (package chris_integrator,          consumers (downstream only)
                                                       daemon chris-integrator, or a library)     
 vehicle motion `ego`, `vehicle` ─┐                ┌────────────────────────────────────────────┐
 receiver fixes `gnss`            │  input         │ timebase: clock epochs, per-source delay   │  `ego_state`  position, heading, speed,
 road model `road` (pose, leads,  ├─ contract ───> │ ego estimator: one EKF, full state +       ├─>  accel, yaw rate, grade, calibration,
   plan), `calib`                 │  (JSON lines,  │   per-source calibration states, time-     │    per-source health, checks
 map matcher `map`                │   one socket   │   ordered buffer, fixed-lag smoothing      │  `tracks`     one multi-target list in the
 radars `radar` (N of them)       │   per producer)│ monitors + fault isolation (raw pairs, NIS)│    car frame, existence, sources
 further cameras `camera_objects` ┘                │ object layer: association, track life      ├─> `lead`       the track on the path
                                                   │   cycle, existence, lead by the path       │   all with time of validity, std,
                                                   └────────────────────────────────────────────┘   valid / reason, held / coasted marks
                                                                                                       │
        api: lanes · ego lane · road geometry · speed limit / rules · lights · sign context · fused · view · recorder · MFD
        mapd: map matcher (reads the integrator's position; its match comes back as an input)
```

## 0. Requirements this design answers

1. **One box, on top**: its own component (`chris-integrator/`, imports nothing from roadsense), an input contract
   and an output contract, a daemon and a library; since the owner's ruling during the lane it is the top
   component of the car's perception stack and roadsense is one of its sources (section 9).
2. **One coherent ego estimate, full state**: position, heading, speed, acceleration, yaw rate, grade, sensor
   biases and scale factors, per-sensor time delays, camera mounting angles (sections 2-4).
3. **Object layer inside the box**: one multi-target track list, per-sensor measurement models and
   association, life cycle, existence probability, cross-sensor consistency, the lead picked by the path
   (section 6).
4. **Top-tier machinery**, chosen against the literature (section 11).
5. **Extensibility proven**: front radar, second radar, second camera added as source files only, driven by a
   simulator with truth and by injection into real recordings (sections 7, 10).
6. **Metadata on everything that leaves** (section 8). 7. **Consumers stay connected** (section 9).

"Not needed on today's data" is not a reason used anywhere below. Where the car has no sensor, the part is
built and proven on the simulator and on synthetic sensors injected into the real recordings.

## 1. What the data says

From the audit and L37 (recordings of 2026-11-19), plus two things settled in this lane before the design
(`tools/integrator_eval.py gnss`, `calib`; numbers in the report):

* **Vehicle speed (CAN 0x0B4)**: 10 Hz, robust sigma 0.012-0.020 m/s against its own neighbours; wheel speeds
  at 0.979-0.981 of it. The feed falls silent for about 7 s 19-25 times a drive and its first samples after a
  silence are stale (up to 0.28 s, up to 7 m/s wrong).
* **Gyro** zeroed at standstill; the rear wheel difference and the camera's yaw rate both read 0.91-0.96 of it
  in turns: three sources, one scale question, which an estimator with scale states can hold.
* **Accelerometers**: longitudinal = dv/dt + slow offset (grade, calibration), sigma 0.2-0.25 m/s2.
* **Road model pose**: speed sigma 0.25 + 0.035 v, heavy tails, error correlated over seconds, valid 0.10-0.15 s
  before the frame stamp (the 200 ms image pair). Arrives 85-130 ms after the frame.
* **The direction of the pose is a precise mount-angle measurement.** On the healthy drives (calibration pitch
  0, yaw 0 in use) `pose.vy / pose.vx` = -0.062 and `pose.vz / pose.vx` = -0.056 with a robust spread of
  0.003-0.005 per frame at speed on straights; the lane lines' direction says the same (-0.065). The camera
  sat 3.5 degrees in yaw and 3.2 degrees in pitch off the calibration in use, and the model worked. On the two
  incident drives the calibration in use was pitch 0.0561, yaw -0.0621 (those very offsets), the direction
  reads 0.00 / +0.01, and the model's speed is 0.2-0.4 of the true one. So in the incident the *direction* cue
  says "aligned" while the *scale* is lost: the fault is visible in the speed against the vehicle, not in the
  direction. (What exactly broke on the car is not established by the recordings; the report states it.)
* **GNSS (head unit), the bias question settled.** Positions: no repeats while moving (the 37-50 % repeated
  positions are standstill), distance over chords >= 100 m is 1.02-1.05 times the CAN odometer on the three
  usable drives, displacement speed between fixes is 0.97-1.03 of the CAN speed at a lag of 1.75-2.5 s with
  sigma 0.9-1.6 m/s. The **speed field** is a different, more delayed and smoothed output: its best lag per
  60 s window ranges 1.5-8.75 s within one drive and it reads 0.82-0.97 of the CAN speed in the median with
  windows near 0. L35 (4-10 % low at the best lag per
  drive) and L37 (half, at a fixed 3 s lag) both looked at this field. Conclusion: the receiver is not biased
  in scale; its positions are usable with their real noise and a lag of about 2 s, its speed field is not a
  measurement of the current speed and is kept as a reported witness only. 113201 is a poor-reception drive
  (4 chords, ratio 0.7-0.8): the filter must survive it by weighting, not by trusting.

## 2. The ego estimator: state

One extended Kalman filter on one time axis. The kinematic state is planar with altitude (a car without a
6-axis IMU does not observe roll, and observes pitch only as road grade through the accelerometer):

| core state | meaning | process |
|---|---|---|
| `pe`, `pn` | position, local east / north (m) about an origin set by the first absolute fix | integrates speed along heading |
| `psi` | heading, rad counter-clockwise from east; innovations wrapped (the filter is an error-state EKF on SE(2): the heading error is the small quantity) | integrates yaw rate |
| `v` | speed along the heading, signed (m/s) | integrates `a` |
| `a` | longitudinal acceleration (m/s2) | random walk, jerk PSD |
| `w` | yaw rate, positive left (rad/s) | integrates `al` |
| `al` | yaw acceleration (rad/s2) | random walk |
| `z` | altitude (m) | integrates `v sin(th)` |
| `th` | road grade (rad) | random walk |
| `sv` | scale of the speed state against the ground (as built): `v` is the speed as the car's own speed signal scales it, known to cm/s, which is what consumers and the object layer want; distance over ground is `v / (1 + sv)` | random walk, tiny |

Everything else in the state belongs to a source and is **declared by the source** (`AuxState`: name, initial
value and sigma, process: random walk with a PSD or first-order Gauss-Markov with a time constant). The core
only appends them to the state vector and propagates them. This stack's sources declare:

| source | calibration states (each estimated, each published with its std) |
|---|---|
| vehicle | gyro bias `b_g` and scale `s_g`; wheel-difference yaw scale `s_wy`; accelerometer offsets `b_ax` (longitudinal, with grade), `b_ay` (lateral: bank and bias) |
| vision | time delay `tau` (the image pair, prior 0.125 s); speed scale `s`; yaw-rate scale `s_w`; two Gauss-Markov errors of the speed, `m` (0.45 m/s, 3.5 s) and `mf` (0.38 m/s, 0.35 s), fitted to the autocorrelation of the car's residuals (0.80 at one frame, 0.42 at 1 s, 0.21 at 3 s: hardly any of the error is white); mount yaw and pitch against the calibration in use (`dyaw`, `dpitch`) |
| gnss | time delay `tau` (prior 2 s, sigma 1.5 s); east / north error correlated over 20 s (Gauss-Markov) |
| map match | the car's offset from the matched road's centre line (Gauss-Markov, 120 s): a thousand matches must not pretend to know the position across the road to centimetres |
| radar (each) | mount yaw misalignment `dyaw`; time delay `tau` |

Observability, stated rather than assumed: `sv` is observable only against an absolute source (GNSS
positions over distance, a radar's stationary targets); with neither it keeps its prior and says so through
its std. `b_ax` and `th` are separable only with an altitude source: without one their sum is observable and
the published grade carries the bias prior as uncertainty. The absolute yaw scale is split between `s_g`,
`s_wy` and the camera by their priors until a heading reference (GNSS track over turns, map bearing) decides.
Time delays are observable only under excitation (acceleration for a speed delay, speed for a position
delay); their process noise is tiny and the std says how far they are known.

**What the build taught about calibration states (as built).** (1) A scale state's Jacobian is the quantity
it multiplies; near zero that quantity is its own estimation noise and teaches the scale nonsense (the
errors-in-variables bias of an EKF: the gyro scale walked to -0.4 on a straight road in simulation). The
regressor is therefore shrunk towards zero by its own uncertainty (zero inside 3 sigma or 0.02 rad/s, the
value itself far outside). A fixed floor (0.05 rad/s, the first build) also removed the scale's *uncertainty*
from every gentle bend: an unlearned gyro scale of 8 % at 0.04 rad/s is 3 degrees of heading in a long
motorway bend that the covariance did not contain. (1a) Nothing in a yaw-rate sensor says what its scale is:
the gyro's scale is a consider state of the gyro's own row, corrected only through heading references; the
other yaw sensors' scales are learned relative to it. Without that gauge all yaw scales walked together
(to -0.25, stated +- 0.03) whenever no absolute source was allowed to teach them. (1b) A standstill says what
the motion is and nothing about a calibration: standstill rows do not correct calibration states, or the
correlations built at speed are cashed in at v = 0, where they mean nothing. (1c) A coloured-noise state
scaled by a speed-dependent sigma is a second scale factor; the filter split the camera's scale between the
two and got it wrong by 8 %: coloured-noise states have a constant sigma. (2) The same holds for the delay of the camera's yaw rate, whose Jacobian is the poorly known yaw
acceleration: the camera's delay is learned from the speed rows only. (3) States a row cannot tell apart are
**consider states** for it (Schmidt-Kalman: their uncertainty counts, they are not corrected): the camera's
rows never correct another source's calibration; the camera's own scale is frozen while no other source holds
the speed; the receiver's and the map's rows do not teach the speed and yaw scales unless the receiver is
declared good (`gnss_absolute`; the reference car's is not).

**Unobservable directions have to be in the covariance.** A position placed on a fix is that fix carried
forward by speed x delay along the heading: at constant speed the receiver's delay and the along-track
position are one quantity. The anchor therefore gives the position the delay's uncertainty along the track
and the heading's across it, correlated with those states; the filter learns the delay when the speed
changes and the stated position uncertainty is honest until it does. (The first build anchored with an
uncorrelated 7 m and became overconfident by 15-35 m along the track.)

**Time delays as states.** A measurement stamped `t` by a source with delay `tau` is `h(x(t - tau))`. It is
inserted into the buffer at `t - tau_hat` (the current estimate) and its Jacobian carries
`dh/dtau = -H f(x)` (the state's rate), so the filter corrects `tau` whenever the state moves.

**Camera mounting (revised in L41).** The mount model follows the hardware: on the reference car the mount has
one degree of freedom, pitch (it is re-aimed by hand); yaw and roll are fixed. So `dyaw` is a constant to be
found once, and `dpitch` is a constant *between steps*.

* The rows are about the direction of the pose: `pose.vy`, `pose.vz` against the pose's *own* `vx` (plus the
  yaw-rate lever arm). The regressor is the measured `vx`, not the speed state: with the state there the rows
  were a speed measurement in disguise, and a camera without a reference "learned" its own scale from them.
  They are used only at speed (judged by the integrator's speed, not the camera's) and never while the camera
  is faulted.
* Two row classes, `mount_yaw` and `mount_pitch`, judged separately. A two-sided CUSUM on the pitch row's
  residual finds a step in a few frames; the state is then put at the step's estimate with an uncertainty
  that says so, converges again within seconds, and the step is reported with its size ("camera pitch
  changed by +1.8 deg"). The yaw row has no such mechanism: a persistent yaw residual is reported as a trend
  of the camera, not followed as a movement of the mount.
* A calibration change does not move the camera: the residual against the new calibration is the old
  residual plus what the calibration changed by.
* The pose moving while another source knows the car stands is somebody handling the camera: the pitch is
  free again (prior uncertainty) until the next drive.
* Far outside its range (measured: a camera tilted by about 16 degrees) the road model returns a collapsed
  speed, an exploded stated std, a phantom near lead, and a pose direction that reads "aligned". That direction
  is not a mount measurement: the camera is faulted, the mount is `unknown`, and the finding says what this
  looks like. The size of such a change cannot be estimated from the model's outputs.
* `sources/signrows.py`: the image rows of sign detections at equal box size, against a reference learned
  while the camera was believed. Rows / focal length is a pitch change that does not pass through the road
  model; coarse (about 2 degrees), needs ten sign tracks per size class, keeps working when the model is lost.

Published as `calibration.vision` (angles, std, `state`: `converging`, `ok`, `offset`, `pitch_changed`, `moved`,
`unknown`; `pitch_steps`, `last_pitch_step`, `handled`) and `calibration.signrows`.

**Standstill** (vehicle: speed and every wheel exactly 0) is a set of exact pseudo-measurements (`v`, `a`,
`w`, `al` = 0): the gyro then measures its bias, the accelerometer its offset plus grade.

## 3. The ego estimator: measurements

Every measurement is a scalar row: value, sigma, time of validity, and a function `h(x) -> (prediction,
sparse Jacobian)` supplied by the source's model. Rows carry a noise class:

* `exact`: applied as is (vehicle speed, standstill).
* `robust`: heavy-tailed source. The update is a Student-t / Huber re-weighting: the row's variance is
  inflated by `max(1, (|nu| / (k sqrt(S)))^2)`-type factor, so a wild sample moves the state by a bounded
  amount instead of being either believed or thrown away; beyond a hard gate (chi-square 0.9999) it is not
  applied and counted. A robust row is only gated while an exact row anchors the same quantity (L37's lesson:
  a lone camera must be followed, not locked out by its own filter). As built: the Student-t weight (4 degrees
  of freedom) scales the whole innovation variance, so the state moves by weight x the Kalman step however
  small the row's own noise is.
* `monitor`: not fused (a witness); it feeds the monitors only.

| source | rows (noise) |
|---|---|
| vehicle | speed `v` (0.03 + 0.002 v, exact); gyro `(1+s_g) w + b_g` (0.004 + 0.03 abs(w)); rear wheel difference `(1+s_wy) w` (0.02 + 0.05 abs(w), robust); accelerometer long `a + g sin(th) + b_ax` (0.25); lateral `v w + b_ay` (0.5); standstill rows; mean wheel speed (monitor); warm-up after a feed silence (L37) |
| vision | speed `(1+s) v + m` at `t - tau` (max of characterised and stated sigma for the white part, robust); yaw rate `w` (0.007 + 0.25 abs(w), robust); `vy`, `vz` (mount rows, robust, at speed) |
| gnss | position east, north at `t - tau` (hacc x adaptive scale, robust); course while moving straight (robust, 15 degrees); altitude (robust); speed field (monitor) |
| map match | cross-track distance to the matched road = 0 (sigma: lane geometry, at least 2.5 m, robust); heading = road bearing while moving (8 degrees, robust); only when matched, not ambiguous, confidence >= 0.5 |
| radar | ego speed and yaw rate from the range rates of stationary detections (robust fit per scan; section 6) |

**Adaptive noise.** A source whose stated accuracy is not to be believed (the head unit's `hacc`) gets a scale
on its variance from its own windowed normalised innovations (innovation-based adaptive estimation), bounded
(1-8) and published.

**Correlated sources.** Two measurements derived from the same physical sensor (mean wheel speed and the CAN
speed) are never both fused: one is fused, the other monitors (family rule, L37). Errors correlated in time
are modelled by Gauss-Markov states (camera speed). Where the object layer fuses two tracks of unknown
cross-correlation (track-to-track across sensors that share the camera's ego motion) it uses covariance
intersection (section 6).

## 4. Time

* **Clocks.** Every input is stamped on the phone's CLOCK_MONOTONIC, which restarts at boot. A source whose
  stamp steps back by more than 2 s starts a new **epoch**: the buffer, the monitors' windows and the tracks
  are dropped, the calibration states (biases, scales, delays, mount) and their covariance are kept, faults
  are carried, the position keeps its value with its uncertainty inflated by the unknown time parked.
  Wall-clock time is never used for ordering.
* **Buffer and out-of-sequence measurements.** Measurements are kept in a time-ordered buffer (horizon 10 s:
  the receiver's fixes are seconds old). A late one is inserted at its time of validity and the filter is
  re-run from there with every later measurement re-applied: the exact solution of the out-of-sequence
  problem (the same result as if it had arrived in order), which the approximate one-step algorithms
  (Bar-Shalom's A1/B1) only approach. Cost: one re-run of at most the lag times the input rate, per late
  measurement; measured in the report. Older than the horizon: dropped and counted.
* **Output time.** The state for a frame is the **fixed-lag smoothed** state at the frame's time of validity
  given everything that has arrived (a Rauch-Tung-Striebel pass over the entries newer than the frame, at
  most 0.6 s): by the time a road message arrives, vehicle samples on both sides of its frame time exist.
  Nothing is delayed for it and nothing is predicted to "now": measurement topics refer to the frame.
* **Linearisation.** The process model is mildly non-linear (heading into position); the re-run re-linearises
  at the current estimate on every pass (an iterated fixed-lag filter in effect).

## 5. Fault detection and isolation

Three layers, because each catches what the others cannot:

1. **Raw pair monitors** (L37, kept): two sources' raw measurements of one quantity at a common time,
   normalised by their *characterised* sigmas, over 5 s of evidence. Never against the fused state (the
   dominant source would test itself). A disagreeing pair faults the lower-ranked source unless independent
   sources side with it against the higher one: with three or more independent sources of a quantity (vehicle,
   camera, a radar's ego speed) the isolation is a **vote**, and a wrong *vehicle* speed is isolated too.
   Groups that share sensors form a family and cast one vote.
2. **Self monitors** (L37, kept): a source's stated uncertainty against its characterised one.
3. **Innovation consistency (NIS)** of every fused row class per source: the windowed mean of
   `nu^2 / S` against its chi-square bounds. It adapts noise where the source allows it (GNSS), marks a
   source `suspect`, and for sources with no raw pair (position, heading, mount angles) it is the fault
   detector: a persistent excess faults the source if a higher-ranked one anchors the quantity.

As built: innovation monitors use the scaled median over 8 s. A row class whose innovations are far beyond the
stated uncertainty (`pos`, `heading`, `map_cross`, `map_heading`, `mount`, `alt`) is **set aside**: its rows
are still evaluated, so it returns by itself, but they are not applied. Absolute positions set aside for 8 s
mean the position is lost: the local frame is anchored again on the next usable fix (`reanchors`). Blame by
rank waits while a vote is forming (an independent third source agrees with the lower-ranked one and drifts
away from the higher), and a disagreement held only by its recovery hysteresis blames nobody anew.

A faulted source leaves the estimate at once (the buffer is re-run without it), its calibration states freeze,
its detections leave the object layer, and everything made from it goes out `valid: false` with the reason.
Recovery needs evidence of agreement (hysteresis as in L37). Faults survive epochs and restarts (carried).

## 5a. Fault detection as a general capability (L41)

Section 5 is isolation between sources. Below it sits detection that knows nothing about any particular
source or incident (`core/fdi.py`), and above it the campaign that scores all of it (`sim/faults.py`,
`tools/fault_campaign.py` in chris-roadsense, `tests/test_faults.py`).

**Taxonomy** (the campaign injects each into every source and quantity it applies to): bias, scale error, slow
drift, stuck value, repeated samples, drop-out, noise burst, outliers, delay and delay jump of the content,
time-stamp error, clock epoch change, stale-after-silence, sign / unit / frame error, mount change of a camera
or a radar, ghosts and mis-association in the object layer, a source lying about its uncertainty, two sources
failing together, slowly developing faults.

**Detectors.**

| family | sees | needs |
|---|---|---|
| signal integrity of one raw stream (`Stream`) | stuck, repeated samples, noise burst (robust sigma of the second difference against the characterised one), outliers, not-a-number | nothing: *self-evident*, the culprit is known whatever its rank |
| timing integrity of one message stream (`Clock`) | a step of its stamps against the other streams' stamps at arrival (stamp offset, latency jump); staleness and epochs as before | the other streams as a clock |
| pair monitors (`monitors.Pair`) | bias, scale, sign, unit between two raw sources: a share-of-bad-samples test for gross faults, a CUSUM for what never puts one sample beyond the gate, a CUSUM in units of the pair's own learned spread for slow noisy sources; the kind of the disagreement is named from the samples | a second source |
| innovation consistency (`NisMonitor`, `Trend`) | a source's stated uncertainty against what its innovations show; a signed innovation that keeps its sign (bias, drift, a calibration that moved) | the filter |
| source-specific (`sources/`) | camera pitch step, handled camera, road model out of range, sign rows | that source's physics |

**Responses**, by what is known: a self-evident fault takes the quantity out at once; a noise burst below the
fault level stays in with the weight its measured noise deserves (adaptive measurement noise); a
disagreement of two is decided by vote when an independent third exists, and while the third has not yet
spoken the quantity is published `mode: disputed` with a std that covers both readings; by rank only when no
third exists. Faults are per (source, quantity) unless the quantity is the source's main signal: a gyro
fault sets the gyro's rows aside and keeps the vehicle's speed.

**Independent opinions on today's car.** Speed: the vehicle's speed signal (wheels: same family), the camera,
and the receiver's *track* (`gnss_track`: path length over three fixes; slow, noisy, no scale of its own).
Yaw rate: gyro, rear wheel difference, camera. Position, heading: the receiver against dead reckoning and the
map's geometry.

**What cannot be detected with the present sensors** (the campaign shows each as such):

| fault | why | what would make it detectable |
|---|---|---|
| which of two is wrong, when only two measure the quantity | no third opinion | a third source (speed: radar ego-speed, a second receiver) |
| two sources wrong the same way (vehicle and camera speed both x 0.7) | they outvote the truth | a second absolute source of the speed (radar on stationary objects) |
| a persistent offset or slow drift of the receiver's positions | the only absolute source; the map gives the cross-track part only | a second receiver, or map matching of landmarks |
| mirrored / swapped receiver coordinates | distances stay right; only turns disagree | as above |
| the camera's lead range or a phantom lead | one sensor reports the lead | a radar (the object layer then cross-checks) |
| a gyro sign error on a straight road | nothing turns | time: it shows at the first bend |
| a delay of the whole vehicle feed with fresh stamps | camera and vehicle disagree in time, not which one is late | honest time stamps from the feeder (`age_ms`) |
| an accelerometer bias | it is the same thing as road grade | an altitude source |

## 6. The object layer

**Track state** in the car frame (x forward, y left, origin the reference camera; z ignored):
`[x, y, vx, vy, ax]`, the target's position relative to the car and its velocity and longitudinal acceleration
*over the ground*, expressed in the car's axes. Process: the car's own motion from the ego estimator at that
time is the input,

    x' = vx - v_ego + w y      y' = vy - w x      vx' = ax + w vy      vy' = -w vx      ax' = -ax / tau + noise

so "we both stand" and "the gap closes" are different hypotheses (the lead filter's insight, kept), a turn
of the car does not look like lateral motion of the target, and the ego speed's uncertainty enters the
position's. A track in the standing regime (car and target stand) switches its process noise and is watched
by a CUSUM on its speed (the lead filter's measured behaviour: the `departed` cue).

**Sensors are models, not code paths.** An object source supplies per message a scan: time of validity
(minus its estimated delay), detections, and a sensor model with
`h(track) -> z, H`, `R(det)`, `initiate(det) -> track`, `p_detect(track)` (field of view, range, occlusion),
clutter density, the association gate, and its life-cycle policy. The core does association and bookkeeping
only. Built in this lane:

| object source | measurement | notes |
|---|---|---|
| camera lead (`road.leads`) | x, y, relative speed along x | reports at most the model's own lead hypotheses; noise and evidence rules are the lead filter's measured ones (L14, L27, L29, L32) |
| radar (front; any mount) | range, azimuth, range rate (non-linear `h`, EKF) | many detections per scan, clutter, multipath ghosts; stationary detections also give the ego speed |
| second camera (any mount) | bearing-dominant x, y in its own frame | detections converted through its mount |

**Association.** Per scan: gates (chi-square on the innovation with the source's own extra rules), then a
global nearest neighbour assignment (optimal assignment on the gated cost matrix; the scans here have at most
tens of detections). A detection assigned to no track may start a tentative track (if its sensor may
initiate). JPDA and MHT were considered and rejected for this sensor set (section 11).

**Existence.** Each track carries an existence probability updated by Bayes per scan from the sensor's
detection probability at the track's place and its clutter density (integrated probabilistic data
association's existence recursion, with hard assignment): a hit multiplies the odds by
`P_D L / lambda_c`, a miss inside the sensor's coverage by `1 - P_D`, a miss outside changes nothing.
`tentative -> confirmed -> coast -> deleted` are thresholds on it together with the source's policy (the
camera's measured acquire / weak-evidence / coast rules are that source's policy).

**Cross-sensor consistency.** Tracks started by different sensors that are statistically the same object
(position and velocity within a gate for 0.3 s) are merged by covariance intersection and keep the older id.
Per sensor pair the layer keeps the residual statistics of jointly seen tracks: camera range against radar
range (the camera's range scale, applied to the camera's measurements while a radar taught it: L14's
`vision_scale`), radar azimuth bias (the radar's mount yaw state). Ghost rules: a radar track at about twice
the range and twice the range rate of a nearer one on the same bearing is marked `ghost` (double bounce); a
track seen only by one radar, never moving, is `stationary` and not a lead candidate on its own.

**The lead** is the nearest confirmed, non-ghost track on the path. The path is the road model's plan while
the camera is usable, else the arc of the ego state (`v`, `w`). A track designated by the camera's lead
output is on the path by the model's own judgement; a radar-only track is on the path when its lateral offset
from the path is inside the corridor and it moves or has moved. The `lead` output keeps every field the api
served (`present`, `mode`, `standing`, `departed`, events, raw `vision`, `radar`, `vision_scale`) plus
validity and sources.

**Out-of-sequence scans.** The layer runs on scan time. A scan older than the layer's time by less than
0.3 s is applied with the track retrodicted to the scan's time by its own velocity (one-step retrodiction);
older scans are dropped and counted.

## 7. Extensibility: what "adding a sensor" is

A file in `chris_integrator/sources/` (or outside the package) that defines:

1. an **input description**: message types, group, rank, family, staleness;
2. a **measurement model**: message -> ego measurement rows and / or an object scan with its sensor model;
3. **mount and calibration**: a `Mount` (position, yaw in the car frame) and the `AuxState`s it wants estimated;

and one `register()` call (or a line in `integrator.json`). No change to `core/` or to any consumer: tracks
carry `sources`, the health table grows a row. Proven in the commit history of this lane: the front radar,
the second radar and the second camera were each added by commits that touch `sources/`, `sim/`, tests and
tools only.

## 8. Output contract (`contract.py`; ARCHITECTURE.md has the field tables)

Three messages per road frame (and per vehicle sample while no camera frames arrive), all stamped with the
frame's `ts_mono_ns` = time of validity:

* **`ego_state`** (api 3; a superset of L37's): `speed`, `accel`, `yaw_rate`, `position` (lat, lon, local
  east / north, covariance, `hacc_m`), `heading`, `grade`, `odometer_m`, `standstill`; each quantity
  `{value, std, valid, reason, mode: fused | coast | none, age_s, sources}`. `calibration`: per source every
  estimated bias / scale / delay / mount angle with std and state. `sources`: per group `state` (`absent |
  stale | unchecked | ok | suspect | fault`), `usable`, reason, isolation rule, age, rate, NIS. `checks`: every
  monitor's verdict. `signals`: pass-through values with age. Counters (late, dropped, rejected).
* **`tracks`**: `frame: "car"`, every track with id, `state` (`tentative | confirmed | coast`), `exist`,
  position / velocity / acceleration with std, `age_s`, `meas_age_s` per source, `sources`, `flags`
  (`standing`, `ghost`, `stationary`, `on_path`), and per-sensor health of the object sources.
* **`lead`**: section 6.

Rules: `valid: false` always carries a `reason`; a value that is not a fresh fusion is marked (`mode: coast`,
`meas_age_s`); nothing is held. Display smoothing stays in `view` (downstream).

## 9. Architecture: the integrator is the top component (owner's ruling, 2026-11-19, during the lane)

"roadsense being just what analyses the video; its output is one of the sources to the integrator; the
integrator is what talks with the MFD and presents the API of consolidated data. With a new camera a new
model will just add more inputs to the integrator, and the integrator is where the decisions are taken."

```
 roadsense instance (per camera)        car backend (CAN, head unit)      radars, further roadsense instances
   camerad -> modeld, signd, lightd,      motion, controls, body, gnss       objects / road / signs / lights
   envd, laned; thin debug api                    |                                   |
        road, calib, signs, lights, env           | feed.sock (vehicle input API)     |
                     \                            v                                   v
                      +------------------>  INTEGRATOR  <------------------------------+
                         ego estimator · object layer · map side (matcher) · trackers and decisions
                         (lanes, ego lane, road geometry, rules / speed limit, ego light, sign context,
                         fused, view) · recorder · consolidated API (/v1, /v2, /ws) -> MFD
```

**The rule for the line.** A module stays in roadsense when it needs one camera's pixels or one camera's
model output and nothing else: one frame of reference, no ego motion from elsewhere, no other source, no
map, no memory across sources. A module belongs to the integrator when it combines sources, needs the ego
motion or the position, carries state across the gaps of a source, takes a decision a consumer acts on, or
is the consumer's interface.

| today | goes to | why |
|---|---|---|
| `camerad*`, `modeld`, `signd` / lights provider (roadsense-signs), `envd`, `laned` | roadsense | pixels of one camera |
| modeld's online calibrator | roadsense keeps the warp; the **mount estimate** is the integrator's (section 2), fed back as a `calib` suggestion | it needs the car's speed |
| `api/overlay.py` (`/preview`), the status page of raw topics, frame ring readers | roadsense (thin debug api) | draws one camera's raw output on its own picture |
| `api/governor.py` | roadsense | it sheds this phone's perception load by its heat; its status is an input (`governor`) the consolidated API shows |
| `api/lead.py` | integrator: object layer (done in this lane) | fusion across sensors, ego motion |
| `api/lanes.py` (lane-line tracker) | integrator | ego speed and yaw rate, the map's lane count prior, later a second camera's lines |
| `api/egolane.py`, `api/roadgeom.py`, `api/roadview.py`, `api/view.py` | integrator | map + camera + ego motion; `view` is the consumer's picture |
| `api/speedlimit.py`, `api/rules.py`, `api/rules/` | integrator | signs (camera) + map + odometer: a decision |
| `api/lights.py` (ego light selection) | integrator | plan + lanes + map signals; the per-light detection stays in the provider |
| `api/signctx.py` | integrator | explains a sign by the lead and the lights: cross-source |
| `api/fusion.py` (`fused`) | integrator | by definition |
| `api/recorder.py` | integrator | it records the consolidated inputs and outputs; the video DVR stays with the camera |
| `api/state.py`, `api/server.py`, `api/sockets.py` (HTTP / WebSocket / topics) | integrator: the consolidated API | the consumer's interface |
| `mapd/posfilter.py`, `mapd/vehicle.py` calibration | absorbed by the ego estimator (position, heading, scales, GNSS delay are its states) | position is estimated once |
| `mapd/engine.py` input handling (`position.sock`, vehicle input API 1.1), `VehicleStore` | integrator: the feed input (the car backend feeds the integrator directly) | not video analysis |
| `mapd/matcher.py`, `rsmap.py`, `context.py`, `limits/` (the map) | integrator: map side; the matcher reads the integrator's position, its match is a measurement (below) | needs the position |

What remains of roadsense's api: the sockets it already publishes (`road`, `calib`, `signs`, `lights`, `env`,
`camera`, `display_lanes`), the frame ring, and a small HTTP face for stand-alone use and debugging
(`/preview`, raw topic dump, health of its own daemons, the governor). It serves no tracked or fused topic.

**A second camera** is a second roadsense instance with its own run directory; its `road` / `signs` /
`lights` are further sources (`sources/` descriptions with a mount), nothing else changes.

**Inputs reach the integrator directly.** The car backend connects to the integrator's `feed.sock` and speaks
the vehicle input API as it does to mapd today (`gnss`, `motion`, `controls`, `body`, `vehicle`, `objects`,
`reset`; docs/VEHICLE_API.md). The sources accept both the feeder's `motion` / `gnss` and mapd's relayed
`ego` / `gnss`, so the switch is a socket path on the car.

**Process and the cost of crossing it.** One daemon (`python3 -m chris_integrator`): it reads each
roadsense instance's sockets and the feed, runs estimator, object layer, trackers and the API. With the
integrator on top there is no second parse of the `road` message any more (L37's objection to a separate
process): the integrator is the only consumer that parses it, roadsense's own api does not. Library form
for replay and tests: the same `Integrator` object without sockets.

**The position is estimated once.** The matcher is a module of the integrator that reads the ego state's
position and returns `match` as a constraint measurement. What returns is the *map's geometry* (cross-track
to a road, the road's bearing), not the position that was sent, so the loop is not incestuous; lock-in on a
wrong road is bounded by the robust weighting, by a lane-offset state, by using only unambiguous matches and
by setting the map rows aside while their innovations are far off (section 5).

**Consumers through the transition.** The consolidated API keeps today's message shapes, paths and topic
names (additive fields; `ego_state` api 3, new topic `tracks`). The MFD repoints an endpoint and the car
backend a feed socket; the report lists both.

**Its own repository.** The directory imports nothing from roadsense and speaks to it over sockets only, so
the split is a move of `chris-integrator/` plus packaging (`pyproject`, a unit file, its tests and tools). Licence:
roadsense is MIT; the integrator can be MIT as well (it contains no code of roadsense-signs, which is AGPL
and stays a separate provider process whose *messages* it reads).

**Built in this lane, and the order of the rest.** See the report: the estimator, object layer, sources,
simulator and the daemon are in `chris-integrator/`; the api's trackers already read only the integrator's outputs.
The physical move of the tracker modules, the consolidated HTTP / WebSocket server and the map side from
`api/` and `mapd/` into `chris-integrator/` is a move of files that already depend only on messages; where it was
not completed in this lane the report says so file by file.

## 10. Simulator and test plan

`chris_integrator/sim/`: a world with known truth (ego trajectory with speed, curvature, grade profiles;
vehicles with lane offsets and speed profiles, cut-ins, a standing queue; roadside stationary objects) and
sensor models that emit the stack's real message types with noise, bias, scale, latency, jitter, drop-outs
and faults (vehicle feed silences with stale resumes, a knocked camera, a wrong vehicle speed scale, GNSS
lag / outage / jumps, radar clutter and double-bounce ghosts, a radar blackout). `sim/inject.py` adds
synthetic radars and a second camera to a *real* recording, derived consistently from it (the car's own
motion from the CAN speed and gyro, targets from the non-causally smoothed lead of the recording).

Shown (commands in the report): ego accuracy and NEES / coverage against simulated truth and against
held-out real sources (position against fixes not yet used; speed with the vehicle held out); delays, scales
and mount angles converging to simulated truth and to plausible values on the car; the incident detected and
its products unknown; isolation by vote with three sources (simulation, and injected radar on real drives:
camera fault and vehicle-speed fault); lead and lanes on the good drives against L37 frame by frame; tracks
against simulated truth (continuity, id switches, false tracks, ghosts, position error) and with injected
radar on real drives; behaviour under delays and drop-outs; CPU and memory.

## 11. Machinery chosen, and what it was compared with

* **EKF with error-state heading, sequential scalar updates** against a **UKF**: the only non-linearities are
  heading into position and the radar's polar measurement, both mild at these update rates; the UKF's sigma
  points cost 2n+1 propagations of a ~25-state vector per step for no accuracy gain here, and scalar
  sequential updates make per-row robust weighting, gating and NIS accounting natural. An invariant EKF
  (SE(2)) would improve consistency under large heading uncertainty at start; the heading starts unknown here
  and is handled by a wide prior and wrapped innovations; noted as a possible refinement.
* **Buffered re-run + fixed-lag RTS** against a **factor graph / sliding-window smoother** (GTSAM / iSAM2,
  what a survey-grade fusion would use) and against **one-step OOSM updates**: the re-run is exact for
  out-of-sequence data and re-linearises; a factor graph gives the same estimate for a linear-Gaussian window
  and better ones under strong non-linearity, at the price of a sparse solver dependency and tens of ms per
  solve in Python; openpilot's `locationd` (error-state EKF, buffered rewind for late measurements) is the
  production precedent for the choice made here.
* **Robust (Student-t / Huber) updates and chi-square gates** against plain gating: a gate alone discards
  information and locks out a lone source; a heavy-tailed likelihood is the principled model of the camera's
  measured tails.
* **Coloured-noise states (Gauss-Markov)** against inflating R or flooring the std (L37's patch).
* **IMM** (interacting multiple models): for the ego state the manoeuvre is measured directly at 10 Hz
  (speed, gyro), so a second ego model has nothing to add. For targets the standing / moving switch with a
  CUSUM change detector (quickest detection) is used; a full IMM over constant-velocity / constant-acceleration
  target models is the alternative, not adopted because the measured lead filter already beats its own
  earlier variants on the recordings and its two regimes are that switch.
* **GNN association with existence probabilities (IPDA-style)** against **JPDA** (soft assignment: better in
  dense clutter with unresolved targets, blurs well-separated road targets) and **MHT** (delayed decisions:
  best for crossing targets, an order more code and CPU). The sensor set here gives sparse, well-separated
  targets with strong range-rate discrimination.
* **Covariance intersection** for track-to-track fusion of unknown correlation, against naive fusion (double
  counting) and against keeping separate track lists (the consumer would have to associate).
* **Consistency**: NEES on simulation and NIS on real data are part of the test suite, not an afterthought;
  **FDI** uses parity between raw sources (pairs, vote) plus innovation monitoring.

## 12. From L37: kept, replaced

Kept: the source registry idea (input description + measurement model), families and ranks, the raw pair and
self monitors (`monitors.py`, unchanged in logic), isolation by rank or vote, the insert-and-re-run buffer
and fixed-lag smoothing (generalised from two linear channels to one EKF), the vehicle feed's warm-up rule,
the vision standstill detector, the `ego_state` quantity format and the consumers' helper functions, the
recorder and replay fixes, carried faults.

Replaced: two decoupled linear channels -> one EKF with position, heading, grade and calibration states;
GNSS as a non-fused witness -> positions fused with estimated delay and adaptive noise (the speed field stays
a witness, for the measured reason in section 1); the std floor -> a coloured-noise state; no object layer ->
section 6, with the lead tracker moved into it; a package inside the api -> a component with a daemon;
position only in mapd -> position in the integrator, the map as a measurement.

## 13. Limits known at design time

* No 6-axis IMU: roll, bank and pitch dynamics are not states; bank hides in `b_ay`, grade and accelerometer
  bias are separable only with altitude.
* Sideslip is not modelled (no-slip kinematics): fine for town driving, wrong on ice.
* The camera reports its lead hypotheses only (the recorder keeps `leads[0]`): multi-target evidence from the
  camera on real recordings is limited to that; the multi-target machinery is exercised by the simulator and
  the injected radars.
* Thresholds of the monitors rest on the drives of one day (L37 section 6); the simulator covers what the car
  cannot show, and no threshold is tuned to the drives without the report saying so.
* Python + numpy: the cost on the phone is an estimate from laptop measurements until measured there.
