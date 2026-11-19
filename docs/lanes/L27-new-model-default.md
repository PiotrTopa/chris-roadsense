# L27: the openpilot v0.11.1 model line (vision + policy) as the default road model

Lane of the 2.0 campaign, 2026-11-19, Fable. Branch `v2-l27` from `v2` (558ffa5). Owner's ruling of
2026-11-19: build around the new models now, do not make the switch depend on other steps; lane lines are a
separate, later step and a nice-to-have (they are for drawing). So lane-line quality does not gate this lane,
and L24-M's finding (the new model's own lane and lead outputs are worse than supercombo's after the trackers)
is the starting point, not a veto.

## What is the default now

`modeld` without a named road model runs `driving_vision` (GPU, ncnn; or ORT) and `driving_policy` (CPU, ORT,
at most 5 runs a second) of openpilot v0.11.1. supercombo v0.9.7 stays selectable (`--road-model supercombo`,
`"road_model": "supercombo"` in `modeld.json`). Nothing is installed on the phone by this lane.

| piece | state |
|---|---|
| default switch, model resolution, fail-safe combinations | built, exercised on the laptop (section 1) |
| model files: pinned URL + sha256, fp32 and ncnn conversion reproducible | built, run end to end (section 1) |
| plan from the policy for the lights, the view path and the road geometry; `--policy-hz 5` | built; effect on the plan measured (section 2) |
| ego-motion of the new model | measured steadier; used by every consumer of `pose` / `v_ego` (section 2) |
| lead tracker for this model | built, tuned on 4 rides, scored on 4 others (section 3) |
| lane outputs flowing; ego lane with "unknown" preferred | measured; commit threshold 0.9 for this model (section 4) |
| seam for a separate display lane source | built (`view.py`, `display_lanes`; section 4) |
| governor: the policy's rate before the road model's | built, tests and tables updated (section 5) |
| MFD draws from `view` with the new default | checked with the car repo's replay tools; car repo not edited (section 7) |
| phone: fp16 gate, GPU time, policy cost, budget | section 6 |

## 1. The switch and how mixed installations fail safe

`resolve_road_model()` (`modeld/supercombo.cc`) is the one rule, used by `modeld` and `modeld_replay`:

| started with | result |
|---|---|
| no road model named (the unit in `systemd/`), vision + policy files installed | vision + policy |
| no road model named, a vision or the policy file missing, supercombo installed | supercombo, `road_model_fallback: true` in every `road` line, `health.road_model.fallback`, a log line that names the missing file |
| no road model named, nothing installed | exit 1 with both paths in the message |
| `--road-model vision` / config, the policy file missing | exit 1 (`--policy none` runs without a plan, deliberately) |
| `--road-model supercombo` / config | supercombo |
| `--model PATH` alone | the file's name decides (`driving_vision*` -> vision, else supercombo) |
| unknown model name | exit 1 |

All seven were run (`modeld_replay --models-dir DIR ...` on directories holding all, only supercombo, vision
without the policy, nothing; 60 frames of `krakow-2024`): the result of each row is what the table says.

* **Old binary, new unit.** The unit's command line is unchanged (it names no road model on purpose): a modeld
  from before L27 runs supercombo exactly as before; its files stay installed (`fetch-models.sh` and
  `push-models.sh` keep supercombo).
* **New binary, old unit.** The same command line: vision + policy if their files are there, else the supercombo
  fallback, flagged in every message and in health. `--throttle-temp` is still accepted and ignored.
* **New api, old modeld** (no `road_model` in `road`): the trackers keep supercombo's settings, which is right.
  **Old api, new modeld**: works, with supercombo's tracker settings on the vision model's output (L24-M's
  numbers: lead on/off and identity changes about twice as frequent).
* A plan that is not refreshed is never served for long: `plan.held` / `plan.age_ms` mark it, and after 1 s
  without a policy run the `plan` key is gone.

Model files (`scripts/fetch-models.sh`, run end to end into a scratch directory: all `ok`):

| file | source / how | sha256 |
|---|---|---|
| `driving_vision.onnx` | `https://github.com/commaai/openpilot/raw/v0.11.1/selfdrive/modeld/models/driving_vision.onnx` | ee29ee5bce84d1ce23e9ff381280de9b4e4d96d2934cd751740354884e112c66 |
| `driving_policy.onnx` | same directory, `driving_policy.onnx` | 78477124cbf3ffe30fa951ebada8410b43c4242c6054584d656f1d329b067e15 |
| `driving_vision-fp32.onnx` | `tools/onnx_fp16_to_fp32.py` | 183f63637d300eb1242294c64b9c529b3e09c266a65414622cc227f783a2de57 |
| `driving_policy-fp32.onnx` | `tools/onnx_fp16_to_fp32.py` | be1235de2eab03297f4d93f38ace8b5aa2f725fe7adee6ad2fcdc6becec0042a |
| `driving_vision-ncnn.param` | `tools/ncnn/make_vision_ncnn.sh` (onnxsim 0.8.1, pnnx 20261020) | 480f638a051bdae651432000b0d33105196d44da7dec70b6f64f5379db8b827f |
| `driving_vision-ncnn.bin` | same | 9e37fe46323a9cf41a4cea82b1ec15a85f89e42b9e24789e4cb81049fbce1fbc |

The converted files are checked against these sums and a difference is reported, not fatal (another onnx or
pnnx version writes other bytes). The policy does not convert to ncnn (L24-M) and stays on ORT CPU.

ncnn and the prefetch (pipelined) path with the new build, laptop GPU (Quadro P3200, fp16 packed), 300 frames
of `krakow-2024`, `tools/ncnn/vision_gate.sh`: lane y mean 0.0027 m, max 0.149 m; lead x max 0.131 m; lane prob
max 0.011; plan max 0.12 m: ACCEPT; split against one-stage identical. A proxy for the phone (section 6).

## 2. The stack around it

**Plan.** The policy's plan is in `road.plan` as supercombo's was; `lights.py` (path-based ego light),
`view.py` (the `path` object, light positions), `roadgeom.py` (geometry where no line is tracked) and the
overlay read it unchanged (L24-M measured the geometry-from-plan shares; contract tests in
`api/tests/test_vision_road.py`, `test_new_model_default.py`).

**`--policy-hz`.** The policy's history is 25 rows 200 ms apart, kept by camera time for every vision run; how
often the policy itself runs only decides how fresh the plan is. Effect of holding the plan, from the eight
rides' 20 runs/s streams (the plan of the last policy run against the plan of this run as a path y(x), while
moving; `tools/vision_eval/plan_hold.py`):

| policy-hz | |dy| at 10 / 30 / 60 m: median (p95), m |
|---|---|
| 10 (= one run held: the plan's own run-to-run change) | 0.005 (0.09) / 0.026 (0.49) / 0.055 (0.85) |
| **5** | 0.006 (0.12) / 0.034 (0.64) / 0.069 (1.05) |
| 2 (the governor's cap) | 0.009 (0.18) / 0.049 (0.94) / 0.100 (1.63) |
| 1 | 0.011 (0.26) / 0.068 (1.49) / 0.134 (2.57) |

Carrying the held plan with the model's own motion (speed, yaw rate) did not help (median worse at every
distance, p95 better only from 2 Hz down), so the held plan is published as it is. Cost on the laptop (one
core, ORT, 1 thread): 6.0 ms per policy run; per vision run on average 4.6-6.0 ms at `--policy-hz 0`, 1.1 ms
at 5. **Chosen: 5.** At 5 the plan differs from an every-run plan by 0.03 m (median) at 30 m, which is the
size of the plan's own change from one run to the next (p95 0.64 against 0.49 m), for a fifth to a quarter of
the cost; the plan feeds the light relevance (lane-width decisions), the display path and the geometry
fallback, none of which resolves 3 cm. `policy_hz: 0` runs it on every vision run. Phone cost: section 6.

**Ego-motion.** The vision model's pose comes from the 200 ms pair. Scatter about a centred 1 s mean, nine
rides, 13.8 runs/s, while moving (`tools/vision_eval/pose_noise.py`):

| | supercombo | vision |
|---|---|---|
| speed `pose.vx`, m/s (mean of the rides) | 2.70 | 0.71 |
| yaw rate, mrad/s | 3.38 | 1.65 |
| worst rides, speed (tunnels-s3 / krakow-night-tunnel / a1-night) | 6.3 / 7.4 / 4.3 | 0.66 / 0.58 / 1.9 |
| one ride where vision is worse (s7-elblag-lowsun), speed | 0.73 | 0.98 |

It is better on eight of nine rides and by a factor of 2-4 on average. Everything in the api that falls back to
the model's motion (the lead tracker's ego speed without a vehicle feed, the view's carry of objects and
lines, the lanes tracker) reads `road.v_ego` / `road.pose`, so it gets the steadier values with the switch.
Changed on purpose: for the vision model `v_ego` is `pose.vx` of this run (supercombo: of the previous run, the
value fed into the network), and the lead tracker forms the relative speed from the lead speed and the pose of
the same run (the pair's own difference). A vehicle feed (`ego` from mapd) still wins where a host sends one.

`road_model` is in `road`, `lead`, `lanes`, `view`; health has `road_model` (`model`, `fallback`, `plan`,
`plan_age_ms`, `policy_hz`, `policy_ms`).

## 3. Lead

The tracker's settings were measured on supercombo. What this model is, measured on the tuning rides (raw
`leads[0]`, frames with prob >= 0.3, scatter about a centred 1 s median):

| distance | distance scatter sc / vis, m | relative-speed scatter sc / vis, m/s | the model's own v_std (vis) |
|---|---|---|---|
| 15-30 m (prob >= 0.8) | 0.32 / 0.50 | 0.66 / 0.44 | 0.8 |
| 30-50 m | 0.86 / 0.93 | 1.26 / 0.78 | 1.2 |
| 50-80 m | 2.30 / 2.24 | 3.89 / 0.90 | 1.8 |
| 80 m and beyond | 3.11 / 2.44 | 3.40 / 0.82 | 2.0 |

Its distance is no steadier than supercombo's (worse near), its relative speed is 2-4 times steadier beyond
30 m, and its own `v_std` overstates the speed's scatter by about two. The supercombo settings gave that speed
a sigma of about 10 m/s at 60 m: it was not used where it is this model's strength.

Built (`api/roadsense_api/lead.py`, `PROFILES["vision"]`, selected by `road.road_model`; contract in
docs/ARCHITECTURE.md, topic `lead`):

1. the relative speed is trusted (sigma max(0.15, 0.5 v_std) + 0.01 d), so the filter's speed carries the
   distance through gaps;
2. weak evidence: a frame under the drop threshold (0.3) but from 0.1 on, whose distance (max(2 m, 8 %)) and
   speed (2.5 m/s) agree with the track, is a measurement with 2.5 times the sigma. The message says
   `mode: "coast"`, `evidence: "weak"`; the track ends 2.5 s after its last confirming frame;
3. a frame that agrees with nothing is `evidence: "none"`: prediction only, 0.8 s, then the track is dropped;
4. acquisition by accumulated evidence: the probabilities of successive frames that agree with each other
   through the reported speed add up to 2.5; two agreeing frames at 0.9 start at once;
5. distant leads found and dropped: the same vehicle found again within 3 s where the lost track would be
   keeps its `track_id` (`reacquired`); a replacing hypothesis at another distance with the same speed and
   lateral position keeps it too (`relocated`: the model re-estimated the distance; the distance still jumps);
6. the distance gets a random walk of (3 % of d)^2 per second and its noise is taken at 0.4 x_std.

Point 6 is a correction made on the way. With the speed trusted and the distance on pure kinematics, the track
stopped following the model's distance (it stayed at 48.8 m while the model went from 52 to 61 m with a
reported relative speed near zero) and then jumped as a "changed" lead: the model's distance is not the
integral of its own speed. Identity changes went up (2.3 -> 2.6 a minute) until the walk was added.

Nothing is held: every state without a confirming frame is marked (`coast`, `evidence`, `meas_age_s`) and
bounded in time.

### Method

`tools/vision_eval/lead_eval.py` (`extract`, then `score`) on L24-M's road streams (nine rides, supercombo at
13.8 runs/s, vision at 13.8 / 17 / 20; winter-snow-rural has no lead in either). Tuned on **krakow-2024,
warsaw-storm, gdansk-night-snow, tunnels-s3**; scored on **s7-elblag-lowsun, krakow-night-tunnel, a1-night,
warsaw-city** (L24-M tuned on the first two only: 7-25 events per ride were too few to tune on). Every choice
was made on the tuning rides. One exception, stated: the per-frame step measure (`fstep`) was added after the
display replay of section 7 showed it on warsaw-city, a scoring ride; the two settings it led to (`rx_scale`,
`dist_walk_rel`) were then chosen on the tuning rides alone.

Measures (after the tracker): on/off and identity changes per minute as in L24-M; `steps` = jumps of the
tracked distance between two messages over max(1.5 m, 8 %), whatever the id does; `fstep` = the distance's
step from one message to the next after the track's own relative speed is taken out (what a display sees);
noise about a centred 1 s median; `follow` = |tracked - centred median of the raw distance| / distance;
`release` = how long `present` stays up after the raw probability fell under 0.1 for good.

### Result on the four scoring rides (5 min each)

`python tools/vision_eval/lead_eval.py score LEAD_DIR --rides s7-elblag-lowsun,krakow-night-tunnel,a1-night,warsaw-city`
(`--no-profile` for the second column):

| after the tracker, mean of 4 rides | supercombo 13.8/s | vision, supercombo's settings (L24-M's state) 13.8/s | **vision, its own profile 13.8/s** | 17/s | 20/s (3 rides) |
|---|---|---|---|---|---|
| lead on/off changes / min | 1.65 | 3.15 | **1.75** | 1.95 | 2.07 |
| identity changes / min | 0.95 | 2.81 | **0.69** | 0.79 | 0.80 |
| distance jumps over max(1.5 m, 8 %) / min | 0.63 | 2.38 | **0.50** | 0.55 | 0.53 |
| per-frame step after the relative speed: rms / p95, m | 1.04 / 1.93 | 0.82 / 1.63 | **0.73 / 1.26** | 0.62 / 1.10 | 0.56 / 0.93 |
| per-frame steps over 0.5 m / min | 84 | 108 | **108** | 118 | 130 |
| distance noise 15-30 / 30-60 / 60+ m, m | 0.21 / 0.22 / 0.49 | 0.19 / 0.38 / 0.56 | **0.09 / 0.15 / 0.26** | 0.12 / 0.28 / 0.39 | 0.05 / 0.09 / 0.17 |
| relative-speed noise, m/s | 3.31 | 1.23 | **0.69** | 0.71 | 0.71 |
| follows the raw distance: median / p90 error, % | 5.8 / 11.9 | 2.1 / 7.1 | **1.9 / 7.2** | 1.9 / 6.9 | 1.9 / 7.4 |
| lead present, % of runs | 40.8 | 44.2 | **46.3** | 47.4 | 54.5 |
| of it not confirmed (`coast`), % | 16.9 | 12.6 | **17.7** (weak evidence 6.4, none 11.3) | 21.1 | 27.9 |
| release after the lead is gone, median / longest, s | 0.22 / 0.22 | 0.30 / 0.40 | **0.92 / 1.45** | 1.01 / 1.76 | 1.02 / 1.68 |
| presence agrees with supercombo's, % of runs (only here / only there) | - | 91.2 (6.1 / 2.8) | **90.3 (7.6 / 2.1)** | 89.1 (8.7 / 2.2) | 86.3 (12.7 / 1.0) |

On the four tuning rides (in-sample, for the record): on/off 2.00 (supercombo) against 3.67 -> 1.81, identity
0.89 against 2.30 -> 0.71, jumps 1.39 against 4.24 -> 0.80, per-frame step rms 0.84 against 0.66 -> 0.36,
relative-speed noise 2.10 against 0.76 -> 0.21; acquisition of a clear lead 0.05 s (supercombo) against 0.10 s.

Per scoring ride, 13.8/s (supercombo / vision with its profile):

| ride | on/off / min | identity / min | jumps / min | per-frame step rms, m | steps over 0.5 m / min | relative-speed noise |
|---|---|---|---|---|---|---|
| s7-elblag-lowsun | 2.0 / 2.0 | 1.0 / 0.75 | 0.5 / 0.0 | 0.78 / 0.71 | 83 / 104 | 1.17 / 0.70 |
| krakow-night-tunnel | 1.2 / 1.2 | 0.4 / 0.4 | 0.4 / 0.2 | 2.08 / 0.54 | 10 / 6 | 5.83 / 0.47 |
| a1-night | 1.6 / 2.0 | 1.2 / 0.6 | 1.0 / 1.0 | 1.02 / 1.22 | 208 / 239 | 6.02 / 1.51 |
| warsaw-city | 1.8 / 1.8 | 1.2 / 1.0 | 0.6 / 0.8 | 0.28 / 0.44 | 35 / 83 | 0.21 / 0.10 |

### Reading, honestly

* **Flicker.** On/off 1.75 a minute against supercombo's 1.65 at the same rate (L24-M: 3.0 against 1.6), at
  17-20 runs/s 1.95-2.07. Identity changes and distance jumps are now below supercombo's. With 6-10 events per
  ride these differences are inside the spread between rides: the claim is "about the same", not "better".
  The doubled flicker of L24-M is gone on rides the settings were not tuned on.
* **Still worse: close leads in town.** On warsaw-city the tracked distance moves more from frame to frame
  than supercombo's (step rms 0.44 against 0.28 m, steps over 0.5 m 83 against 35 a minute), and that reaches
  the display: section 7, drawn lead step rms 0.16 against 0.09 m, 64 against 7 steps over 0.5 m in 3 minutes.
  The model's raw distance at 15-30 m scatters 1.5 times more than supercombo's and this tracker does not
  remove all of it. Over all four rides the count of per-frame steps over 0.5 m is 29 % higher (108 against 84
  a minute) while their rms and p95 are lower.
* **Release.** A lead that is gone stays for 0.9 s (longest 1.5 s) against 0.2 s, marked `coast` the whole
  time. That is the price of carrying dropouts; `coast_s` 0.6 brought it to 0.7 s for about 5 % more on/off
  changes on the tuning rides (measured on an earlier state of the profile).
* **More presence.** 46 % of runs against 41 %; 7.6 % of runs have a lead only here. L24-M's hand check of such
  moments found about half real, a fifth false, the rest undecidable; no lead labels exist, so this lane adds
  nothing to that. The acquisition rule (evidence 2.5) took the tuning rides from 54 to 48 % present.
* **Better.** Relative-speed noise 0.7 against 3.3 m/s (L24-M: 1.0 against 2.9), distance noise at every range,
  and the track follows the raw distance three times closer.
* **Limits.** No ground truth for the lead; 5-minute stretches; one tuning pass; the s7 ride has no 20/s stream.
  The numbers are from ORT fp32 streams of the laptop; the phone runs ncnn fp16 (section 6).

## 4. Lanes

Unchanged: the model's lane lines and edges go through the same trackers with the model's own probabilities;
`lanes`, `fused.lanes`, `view` work (tests; section 7). L24-M's findings about their quality stand (far range
1.6-1.9 times further off, inner-line presence changes 3.6 against 2.1 a minute): not addressed here.

**Ego lane against L20's labels** (three rides, first 300 s, map count from the label; mean of the rides;
`tools/egolane_replay.py`, threshold scan with the filter's own confidence):

| | right | wrong and confident | unknown |
|---|---|---|---|
| supercombo 13.8/s, commit 0.8 | 90.2 % | 3.4 % | 6.3 % |
| vision 13.8/s, commit 0.8 (the switch alone) | 87.9 % | 5.5 % | 6.6 % |
| **vision 13.8/s, commit 0.9 (adopted)** | **86.8 %** | **3.3 %** | **9.9 %** |
| vision 13.8/s, commit 0.95 | 83.5 % | 2.9 % | 13.6 % |
| vision 17 / 20 runs/s, commit 0.9 | 87.0 / 87.0 % | 3.2 / 3.2 % | 9.8 / 9.8 % |

Per ride at 0.9 (right / wrong / unknown): a1-night 98.0 / 0.0 / 1.9 (supercombo 98.6 / 0.2 / 1.3);
krakow-night-tunnel 83.1 / 3.0 / 13.8 (89.5 / 2.6 / 7.8; at 0.8 it was 84.2 / 9.3 / 6.6); warsaw-city
79.1 / 6.9 / 14.0 (82.6 / 7.5 / 9.9). The switch alone raised wrong-and-confident from 3.4 to 5.5 %, all of it
on the night expressway; with the threshold at 0.9 for this model (`egolane.PROFILES`) it is 3.3 %, paid with
3.4 points of "right" and 3.6 more of "unknown" against supercombo. The threshold was chosen on the same three
rides it is reported on (no other labels exist; a split by time leaves one ride without labelled frames in a
half): in-sample, and it should be re-checked when more labels exist.

**The seam.** One place decides which lines the display draws: `ViewBuilder._display_lanes()` in
`api/roadsense_api/view.py`. A provider sends `{"type":"display_lanes", ...}` (the shape of `lanes`) on
`<rundir>/laned.sock`; while fresh (0.5 s) `view.lines` is built from it and `view.lines_source` names it,
otherwise from the model's `lanes` (`"model"`). The measurement topics never see it. Contract:
docs/ARCHITECTURE.md "Display lane source"; test `test_display_lane_source_seam`. No provider exists.

## 5. Governor

The policy is a CPU load on the model thread (big cores). New step `plan` (cap `policy_hz` 2) directly before
the road model's own rate:

| level | step | cap |
|---|---|---|
| 1-7 | unchanged | |
| 8 | `plan` | `policy_hz` 2: the plan held up to 0.5 s (table of section 2: 0.05 m median, 0.94 m p95 at 30 m), road / lead / lanes untouched |
| 9 | `road` (last resort, as before) | `road_hz` 10 |

It is not a last resort (shed on the forecast like the detectors) and marked `quick` (it saves little: three
policy runs a second). `road` is not marked `shed` for it: the plan says `held` itself. **Renumbering: the road
model's step was level 8 and is 9 now**; `max_level` 8 keeps the road model's rate, 7 also the plan's. The
runbook of docs/lanes/L24-quality-max.md, section 5 step 3, forces levels by number: `8` there now means the
plan step and the road step is `9`. `tools/governor/emu.py` has the step; `camerad/governor.h` documents the
key; tests in `api/tests/test_governor.py` (15).

## 6. Phone

PENDING at the time of this commit (the phone part follows the laptop commit; see the end of this report).

## 7. MFD (the car repo, not edited)

* Car suite in `wt/prius-v2` against this branch: `PRIUS_ROADSENSE_REPO=<this worktree> SDL_VIDEODRIVER=dummy
  .venv-fe/bin/python -m pytest -q tests`: section 8.
* Display chain on a ride replayed through both models with this build (warsaw-city 0-180 s; supercombo at
  13.8 runs/s through the api of `v2`, the new default at 17 runs/s through this branch;
  `tools/view_replay.py ROAD --ws WS`, then the car repo's `tools/scene_feed_replay.py WS --strip 60:6:PNG`):

  | what the MFD draws | before (supercombo) | after (new default) |
  |---|---|---|
  | scenes published | 9.9/s | 10.0/s |
  | lane lines at 10 / 30 m, step rms | 0.012 / 0.020 m | 0.010 / 0.023 m (left), 0.008 / 0.018 m (right); no step over 0.3 m in either |
  | lead, step after its relative speed: rms / p95 / over 0.5 m | 0.089 / 0.193 m / 7 of 2794 | 0.162 / 0.313 m / 64 of 3026 |
  | lead read-out changes while standing | 1 in 20 s | 0 in 21 s |

  The MFD draws lanes, path and lead from `view` with the new default. The drawn lead is about twice as
  unsteady on this ride (section 3, "still worse").
  Screenshots (six consecutive drawn frames from 60 s): `/home/piotr/datasets/l27/mfd/mfd_before.png`,
  `/home/piotr/datasets/l27/mfd/mfd_after.png`. In that moment "before" also shows the lane to the left and the
  lane boxes; "after" shows the ego lane only: the vision model's outer line was not present there.
* `tools/mfd_chain_replay.py` on a recorded car drive (20261117-145921, junction labels) against `v2` and
  against this branch: identical output (section 8); a recording carries supercombo's `road` without
  `road_model`, so it runs on supercombo's settings in both. (The tool needs numpy and pygame in one interpreter; `.venv-fe` has
  no numpy, so the system numpy was put on `PYTHONPATH`. That is the same on `v2`.)

## 8. Tests

| suite | command | result |
|---|---|---|
| roadsense api | `PYTHON=~/projects/roadsense/.venv-models/bin/python sh tools/smoke_api.sh` | 242 passed (228 on `v2` + 13 in `api/tests/test_new_model_default.py` + 1 governor test) |
| mapd | `~/roadsense-maps/.venv/bin/python -m unittest discover -s mapd/tests -t mapd` | 93 passed |
| C++ | whole tree, Release, `-DROADSENSE_NCNN=ON` (ncnn 20261020), `modeld/warp_test` | builds; warp test identical |
| model resolution | section 1 (seven start-up cases with `modeld_replay --models-dir`) | as specified |
| fp16 gate, laptop GPU | `tools/ncnn/vision_gate.sh` | ACCEPT; split = one-stage |
| car suite (`wt/prius-v2`, not edited) | `PRIUS_ROADSENSE_REPO=<this worktree> SDL_VIDEODRIVER=dummy .venv-fe/bin/python -m pytest -q tests` | 1711 passed, 9 skipped, 1 failed: `tests/test_battery_health.py::test_service_tap_to_stored_ride` (the battery-health service, no roadsense code; the laptop's load was about 20 from other lanes). That file alone: 25 passed, against this branch and against `v2` |
| MFD chain replay on a recorded car drive | car repo `tools/mfd_chain_replay.py ~/roadsense-rec/20261117-145921 --labels car-20261117-145921.junctions.csv --roadsense <v2 \| this worktree>` | output identical for `v2` and this branch (2361 driving scenes, lane boxes on 1166, 4 labelled junctions: the same stages reached) |
| MFD scene feed replay, new default | section 7 | draws; screenshots there |

## 9. Reproduce

* lead: `python tools/vision_eval/lead_eval.py extract ~/datasets/openpilot-models/l24/road LEAD_DIR`, then the
  `score` commands above (`--per-ride`, `--no-profile`, `--set KEY=JSON`).
* ego lane: `tools/egolane_replay.py ROAD --labels LANES.csv --map-count label [--set commit_p=0.9]` per ride
  and stream (the estimator takes the profile from the stream's `road_model`).
* plan hold: `python tools/vision_eval/plan_hold.py ~/datasets/openpilot-models/l24/road RIDE ...`; pose noise:
  `python tools/vision_eval/pose_noise.py LEAD_DIR RIDE ...`.
* model resolution: `modeld_replay FRAMES.nv12 --width W --height H --models-dir DIR [--road-model M]`.
* fp16 gate: `tools/ncnn/vision_gate.sh`.

## 10. Not done, and open

* Lane lines: nothing beyond the seam and the threshold (by the ruling).
* The lead in town at close range (section 3): the remaining gap. Candidates not tried: the model's `leads[1]`
  (+2 s) as a second look at the same vehicle, a near-range distance from the lead's image box.
* The ego-lane threshold is in-sample.
* No on-car run: everything here is replayed rides; the car's camera has not seen this model.
* Old recordings carry no `road_model`: replays of them use supercombo's settings, which is what recorded them.
