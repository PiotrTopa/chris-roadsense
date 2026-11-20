# L24-M: openpilot v0.11.1 `driving_vision` as a second road model

Sub-lane of L24 (roadsense 2.0), 2026-11-18 21:10-00:30 CET, laptop only (the phone was not touched; after the
22:28 restart it was not reachable either). Branch `v2-l24m` from `v2` (89798e7). Question: is the v0.11.1
vision model (+ its policy) better than the installed supercombo v0.9.7 after the 2.0 trackers, or the same
quality for less budget (owner's rule: quality first; same quality and cheaper = switch; a measurement-quality
regression is not traded for speed).

## Verdict

**Not adopted. Kept behind the switch (`modeld --road-model vision`), default stays supercombo.**

Nine rides (krakow-2024, winter-snow-rural, warsaw-storm, gdansk-night-snow, tunnels-s3, s7-elblag-lowsun: 4-5 min
windows; a1-night, krakow-night-tunnel, warsaw-city: their first 5 min, the stretch with L20's labels), after the
api's trackers, supercombo at 13.8 runs/s against vision at 13.8 / 17 / 20 runs/s. "same" = within the spread
between rides (method in section 2). Full table: section 3.

| metric (after the trackers) | supercombo 13.8/s | vision 13.8/s | vision 20/s | verdict |
|---|---|---|---|---|
| far range, inner line seen at 20-40 / 40-70 / 70-100 m vs the same line later at 5-15 m, median m | 0.13 / 0.30 / 0.57 | 0.20 / 0.47 / 0.81 (x1.8 / x1.8 / x1.6) | 0.23 / 0.53 / 0.91 | **worse** at every range and rate |
| same, p90 m | 0.45 / 1.14 / 2.06 | 0.55 / 1.28 / 2.48 | 0.62 / 1.45 / 2.75 | **worse** (40-70 m: same) |
| lines picked where the models disagree most at 45 m, judged by what both see later at 5-15 m (17 picks) | 8 right | 6 right (3 ties) | | **same** (mixed: supercombo 5-0 on krakow / tunnels-s3 / warsaw-storm, vision 6-3 on s7 / krakow-night-tunnel / a1-night) |
| ego lane against L20's labels, right / wrong and confident (map count from the label, 3 rides) | 90.2 / 3.4 % | 87.9 / 5.5 % | 88.4 / 4.7 % | **same** overall; krakow-night-tunnel **worse** (89.5 / 2.6 -> 84.2 / 9.3 %) |
| inner lane-line presence changes / min | 2.1 | 3.6 | 3.5 | **worse** |
| inner-line jitter at 10 / 50 m (tracked), m | 0.022 / 0.064 | 0.029 / 0.067 | 0.030 / 0.074 | **worse** |
| lead on/off changes / min | 1.6 | 3.0 | 3.2 | **worse** (x2) |
| lead identity changes / min | 0.8 | 2.3 | 2.2 | **worse** (x3) |
| lead present, % of runs | 38 % | 42 % | 46 % | more, about half of it real (hand check: 10 real, 4 false, 7 unclear of 21) |
| lead relative-speed noise (tracked), m/s | 2.9 | 1.0 | 0.9 | **better** (the 200 ms pair) |
| lead distance noise per band (tracked) | | | | **same** (worse at 17/s, better at 20/s 30-60 m) |
| road geometry without the policy (`plan` absent) | | | | on unmarked roads none at all: winter-snow-rural 100 %, gdansk-night-snow 81 %, warsaw-storm 78 % of runs; the policy (6 ms/run on the laptop CPU) is needed |
| + temporal stage in front of the trackers, tuned on krakow-2024 / warsaw-storm, scored on the 6 others | 1.7 lead on/off, 0.9 identity, 1.9 inner-line changes / min | 2.1, 1.8, 2.7 | 2.4, 1.7, 2.5 | flicker still **worse**; jitter same / better; far range still **worse** (x1.8) |
| GPU per run | 47 ms (Adreno, L21) | 28-38 ms (estimate; laptop GPU ratio 0.73) | | **cheaper**, phone measurement pending |

**Recommendation: keep supercombo.** The vision model is cheaper on the GPU and its lead speed is steadier, but its
far-range lane geometry is 1.6-1.9 times further off on every ride type and rate, the ego-lane estimate on the
night expressway is worse, and lead / line flicker is two to three times higher; a temporal stage in front of
the trackers closes the jitter but neither the flicker (held-out rides) nor the far-range gap, which no filter
can close. Under the owner's rule (same quality shown after the trackers, then cheaper) this is not adoptable.

What would change it: a far-range accuracy check on the car's own camera with a real calibration (the rides'
calibrations are estimates checked on supercombo overlays, see Limits), or a newer openpilot vision model
whose far-range geometry is at least supercombo's. The flicker gap alone is closable (section 4).

## 1. What was built

| piece | files |
|---|---|
| second road model in modeld: `--road-model supercombo|vision` (config key `road_model`), default supercombo | `modeld/main.cc` (option parsing, model-path defaults, the prep thread's pair choice; the rate-cap / thermal block is untouched) |
| image pair (frame 200 ms earlier, current): `PairPicker` keeps the selected frames of the last 300 ms and takes the one within 30 ms of t - 200 ms; reset on every stream gap/restart, so a pair never spans one; no pair (no run) after a dropped frame, as openpilot's `prepare_only` | `modeld/pipeline.{h,cc}` |
| warp as v0.11.1 `compile_modeld.py`: nearest neighbour, float32 source point, round half to even, clip, U/V matrix `M * [[1,1,.5],[1,1,.5],[2,2,1]]`; one gather per pixel | `modeld/warp.{h,cc}` (`WarpKind::kNearestV011`) |
| uint8 inputs: the packed float frames (0..255) are converted in the ORT backend; ncnn takes them as float (the two casts are removed from the graph) | `modeld/ort_backend.cc`, `modeld/ncnn_backend.cc` |
| parser of the 1,576 outputs into `RoadOutputs` / the `road` JSON: lane lines, edges, 3 leads (single hypothesis), pose, road transform, wide-from-device euler, desire_pred; meta rearranged into the v0.9.7 layout, so FCW and the JSON stay the same | `modeld/vision.cc`, `modeld/vision_layout.h` |
| `road_model` field in every `road` line (`"supercombo"` / `"vision"`) | `modeld/supercombo.cc` |
| fields the vision model does not produce: `desired_curvature`, `desired_curvature_std` = null; `desire_state` = null and **no `plan` key** without the policy (absent, not null: `lights.py` reads `road.get("plan", {})`) | `modeld/supercombo.cc`, contract test `api/tests/test_vision_road.py` |
| optional `driving_policy` (ORT CPU) for `plan` + `desire_state`: 25 x 512 feature history at 5 Hz kept by camera time (row k = stored run within 50 ms of t - 0.2 k s; zeros before the history starts, as openpilot's queue), desire zero, traffic convention as for supercombo; `--policy PATH|none`, `--policy-threads`, `--policy-hz F` (run it at most F times a second, the plan is held in between); `plan.prob` = null (single hypothesis) | `modeld/vision.{h,cc}`, `modeld/supercombo.cc` |
| ncnn backend: the existing two-stage (prefetch) path works unchanged. For this graph the stage-1 analysis finds 233 of 236 layers with the frontier = `out0` itself, so the prefetch thread records the whole next run while the current one runs and submits it on `go`; stage 2 is only the download. One-stage and split runs give identical outputs | `modeld/ncnn_backend.cc` |
| `modeld_replay`: `--road-model`, `--policy`, `--policy-hz`, `--rates R1,R2,.. --out-prefix P` (simulates the phone's run rate: at each scheduled time the newest pair runs, pairs in between are skipped; one output per rate, each with its own state; for the vision model one network run per frame is shared by the rates) | `tools/modeld_replay.cc` |
| conversion to ncnn, reproducible from the fp32 ONNX (byte-identical to the L21 survey's files) | `tools/ncnn/make_vision_ncnn.sh`, `tools/ncnn/fix_vision_ncnn.py` |
| phone fp16 gate in one command | `tools/ncnn/vision_gate.sh` (+ `compare_replay.py` now takes either model) |
| C++ against the Python reference | `tools/models_survey/check_cpp_vision.py` |
| evaluation after the trackers | `tools/vision_eval/{ride_replay.sh, evaluate.py, summarize.py, overlay_pair.py, adjudicate.py, lead_presence_sample.py}` |

supercombo is unchanged: `modeld_replay` of the branch and of 89798e7 on 60 frames of `krakow-2024` give identical
`road` lines apart from `effective_hz` (the replay now writes the simulated rate there).

### Unit check: C++ against the Python reference (40-60 frames of `krakow-2024`)

`python -I tools/models_survey/check_cpp_vision.py <datasets>/rides/krakow-2024 DUMP ROAD.jsonl --frames 60`
(the C++ side: the command in the script's docstring, `modeld_replay --dump-tensors`):

| stage | result |
|---|---|
| packer (C++ float32 vs the survey's float64 nearest warp) | up to 38 of 786,432 input values per run differ (pixels on a rounding boundary); effect on the lane lines up to 100 m: max 0.035 m |
| network (C++ ORT vs Python ORT on the C++ inputs) | identical (max abs diff 0) |
| parser (JSON vs `replay_compare.parse_vision` of the C++ raw outputs) | max 5e-4 (the JSON prints 5 significant digits) |
| policy (C++ plan vs the policy run in Python on the 20 Hz history) | max relative diff 4.8e-4 on 56 of 56 runs |

### ncnn and fp16 (laptop)

ncnn 20261020 built on the laptop (`<datasets>/openpilot-models/l24/ncnn`, static, Vulkan via simplevk), modeld
with `-DROADSENSE_NCNN=ON -DNCNN_DIR=...`. On the laptop's Quadro P3200 (fp16 packed + storage, fp32 arithmetic,
the closest the laptop has to the Adreno's fp16-packed mode), 200 frames of `krakow-2024`, ncnn against ORT fp32
(`tools/ncnn/compare_replay.py ort.jsonl ncnn.jsonl`): lane y error mean 0.0036 m, max 0.055 m; lead x max
0.020 m; lane prob max 0.004; plan max 0.39 m (the policy amplifies the vision difference at 10 s); **ACCEPT**.
Split (pipelined) against one-stage: identical. This is a proxy: it does not exercise turnip's synchronisation
bug (flushall stays on) or the Adreno's speed. **Pending (phone): the real gate, below.**

### Freed budget

| | supercombo v0.9.7 | v0.11.1 vision | source |
|---|---|---|---|
| GPU per run, Quadro P3200 ncnn fp16 (ratio) | 10.4 ms | 7.6 ms (0.73) | `modeld_replay --backend ncnn`, 200 frames |
| GPU per run on the Adreno 630 | 47 ms (L21) | **28-38 ms estimated** (L21: 0.60-0.77 x); not measured | pending (phone) |
| CPU ORT, laptop, per run | 46 ms (L21, 2 threads) | 35 ms (L21, 2 threads) | |
| prep (warp + pack, both inputs) per frame, laptop | 3.5 ms | 1.4 ms (nearest, one gather) | `modeld_replay` stats |
| prep on the phone (A55) | 10 ms per frame = 19 % of a little core at 20 frames/s (L21) | ~4 ms per frame (laptop ratio 0.39), about 11 % of a little core freed; estimate | |
| policy (ORT CPU, 1 thread) per run, laptop | - | 6.0 ms (weights 28 MB, memory bound); with `--policy-hz 5`: 1.25 ms average per run | `modeld_replay` stats |
| policy on the phone | - | not measured; expected 10-20 ms on a big core per run (~15-35 % of a big core at 17 runs/s), a quarter of that at `--policy-hz 5` | pending (phone) |
| policy on ncnn | - | does not convert cleanly (pnnx leaves `torch.where`, `torch.unbind`, `Tensor.reshape`, `GatherND`); kept on ORT CPU | `pnnx dp-sim.onnx inputshape=[1,25,8],[1,2],[1,25,512]` |

Net, if the phone confirms the estimate: 9-19 ms of GPU per run (at 13.8 runs/s 12-26 % of the GPU's time, or
17-20 runs/s at today's GPU share), 8 % of a little core from the warp, minus the policy's big-core share.

## 2. Method of the comparison

Ten rides (seven complete; see the table of rides), each replayed through both models at the run rate each would
have on the phone: supercombo at 13.8 runs/s (its own recurrent state advancing only on the runs it gets), vision
at 13.8, 17 and 20 runs/s (`tools/vision_eval/ride_replay.sh RIDE START DUR OUT BUILD`, ORT fp32 on the laptop,
each in a `systemd-run --user -p MemoryMax=8G -p CPUQuota=400%` unit). Every stream went through the api's code
(`tools/replay_lib.Pipeline`: LeadTracker, RoadGeometryTracker, EgoLaneEstimator, RoadGeometryFusion,
ViewBuilder; no map, no vehicle speed) and `tools/vision_eval/evaluate.py` measured, per stream:

* flicker per minute: presence changes of the tracked inner lines, outer lines, edges; lead on/off and identity
  changes;
* jitter: robust sd of the tracked (and raw) inner lines at 10 m and 50 m about their centred 1 s mean;
* lead noise: robust sd of the tracked distance about its centred 1 s median per band; of the relative speed;
* **far range**: a tracked inner-line point seen at 20-40 / 40-70 / 70-100 m is carried with the car's motion
  until it is 5-15 m ahead and compared with the line seen there then (median and p90 of |dy|). The car's motion
  is the mean of the two models' own pose (vx, yaw rate), so neither model is scored on its own odometry. A
  check with either model's odometry alone gives the same ranking (`EGO=sc|vis`; tunnels-s3 40-70 m: supercombo
  0.16-0.17 m, vision 0.35-0.39 m with either);
* ego lane against L20's hand labels (`tools/egolane_replay.py`, map count from the label, and without a map),
  where labels exist (a1-night, krakow-night-tunnel, warsaw-city, first 300 s);
* the share of runs whose road geometry (`view.road.vision.from`) comes from the plan, and the same stream with
  the plan removed (`vis.13.8~noplan`).

`tools/vision_eval/summarize.py RESULTS.json` turns the per-ride values into the verdict table: mean over rides,
median per-ride ratio, a bootstrap 90 % interval of the mean per-ride difference; **better / worse** only when the
interval excludes zero and the median ratio is more than 10 % from 1, else **same**. The replays are
deterministic: the spread that decides "same" is the spread between rides, not run-to-run noise.

| ride | conditions | stretch | rates | labels |
|---|---|---|---|---|
| krakow-2024 | day city, overcast (tuning) | 60-300 s | 13.8 / 13.8, 17, 20 | - |
| warsaw-storm | thunderstorm, city (tuning) | 60-300 s | same | - |
| winter-snow-rural | snow, unmarked road (tuning; no lines, no leads from either model) | 120-360 s | same | - |
| gdansk-night-snow | night + snow, city | 60-300 s | same | - |
| tunnels-s3 | expressway, tunnels | 0-300 s | same | - |
| s7-elblag-lowsun | low sun, expressway | 60-300 s | 13.8 / 13.8, 17 (the 20/s stream was lost in the 22:28 restart; odometry from 17/s) | - |
| a1-night | night motorway | 0-300 s | 13.8 / 13.8, 17, 20 | L20 lanes |
| krakow-night-tunnel | night expressway, tunnel | 0-300 s | same | L20 lanes |
| warsaw-city | day city, 3 lanes | 0-300 s | same | L20 lanes |

a1-s8-rain (rain motorway) was dropped: after the restart the laptop was shared with other lanes' replays (load
30+), the 15-minute rides were cut to their first 5 minutes and one ride had to go.

## 3. Results

Default trackers, all nine rides (`python3 tools/vision_eval/summarize.py res_all.json`; results file built by
`evaluate.py ROAD_DIR --out res.json` per ride and merged; `ROAD_DIR` = `<datasets>/openpilot-models/l24/road`):

| metric | rides | sc.13.8 | vis.13.8 (ratio, 90% CI of diff, verdict) | vis.17 (ratio, 90% CI of diff, verdict) | vis.20 (ratio, 90% CI of diff, verdict) |
|---|---|---|---|---|---|
| inner lane-line presence changes / min (tracked) | 8 | 2.12 | 3.57 (xnan, [+0.923, +2.53], **worse**) | 3.61 (xnan, [+0.933, +2.55], **worse**) | 3.53 (xnan, [+0.738, +2.08], **worse**) |
| outer lane-line presence changes / min (tracked) | 8 | 1.61 | 1.52 (xnan, [-0.439, -0.0167], **better**) | 1.57 (xnan, [-0.379, +0.0111], same) | 1.39 (xnan, [-0.45, -0.05], **better**) |
| road-edge presence changes / min (tracked) | 8 | 3.1 | 2.54 (xnan, [-0.733, +0.337], same) | 2.59 (xnan, [-0.609, +0.336], same) | 2.92 (xnan, [-0.673, +0.38], same) |
| both inner lines present, % of runs | 8 | 51.1 | 56.1 (xnan, [-3.53, +2.08], -) | 55.9 (xnan, [-3.56, +1.8], -) | 51 (xnan, [-3.4, +2.53], -) |
| ego-lane index changes / min (no map) | 8 | 0 | 0 (xnan, [+0, +0], same) | 0 (xnan, [+0, +0], same) | 0 (xnan, [+0, +0], same) |
| ego lane named, % of runs (no map) | 8 | 0 | 0 (xnan, [+0, +0], -) | 0 (xnan, [+0, +0], -) | 0 (xnan, [+0, +0], -) |
| inner-line jitter at 10 m, m (tracked) | 7 | 0.0221 | 0.0287 (x1.23, [+0.00438, +0.0107], **worse**) | 0.0287 (x1.23, [+0.0045, +0.0104], **worse**) | 0.0297 (x1.28, [+0.00443, +0.011], **worse**) |
| inner-line jitter at 50 m, m (tracked) | 7 | 0.0644 | 0.067 (x1.11, [+0.00125, +0.0135], **worse**) | 0.069 (x1.15, [+0.00224, +0.0166], **worse**) | 0.0744 (x1.13, [+0.00129, +0.0186], **worse**) |
| inner-line jitter at 10 m, m (raw model) | 8 | 0.0539 | 0.0591 (x1.20, [+0.00322, +0.0146], **worse**) | 0.0592 (x1.19, [+0.00344, +0.0147], **worse**) | 0.0625 (x1.18, [+0.002, +0.0145], **worse**) |
| inner-line jitter at 50 m, m (raw model) | 8 | 0.179 | 0.175 (x1.08, [-0.0112, +0.0403], same) | 0.173 (x1.08, [-0.0124, +0.0371], same) | 0.191 (x1.00, [-0.0156, +0.0459], same) |
| far-range error 20-40 m, median m (tracked) | 7 | 0.13 | 0.204 (x1.80, [+0.0462, +0.126], **worse**) | 0.203 (x1.82, [+0.0439, +0.127], **worse**) | 0.226 (x1.94, [+0.0524, +0.138], **worse**) |
| far-range error 40-70 m, median m (tracked) | 7 | 0.297 | 0.471 (x1.80, [+0.111, +0.303], **worse**) | 0.471 (x1.79, [+0.111, +0.302], **worse**) | 0.525 (x1.91, [+0.132, +0.322], **worse**) |
| far-range error 70-100 m, median m (tracked) | 7 | 0.573 | 0.813 (x1.64, [+0.146, +0.443], **worse**) | 0.795 (x1.64, [+0.119, +0.432], **worse**) | 0.907 (x1.69, [+0.167, +0.489], **worse**) |
| far-range error 20-40 m, p90 m (tracked) | 7 | 0.446 | 0.553 (x1.28, [+0.0359, +0.295], **worse**) | 0.552 (x1.26, [+0.0336, +0.298], **worse**) | 0.616 (x1.32, [+0.0447, +0.329], **worse**) |
| far-range error 40-70 m, p90 m (tracked) | 7 | 1.14 | 1.28 (x1.09, [-0.04, +0.592], same) | 1.29 (x1.12, [-0.0219, +0.602], same) | 1.45 (x1.08, [-0.00789, +0.653], same) |
| far-range error 70-100 m, p90 m (tracked) | 7 | 2.06 | 2.48 (x1.24, [+0.292, +1.05], **worse**) | 2.43 (x1.18, [+0.226, +0.987], **worse**) | 2.75 (x1.24, [+0.312, +1.11], **worse**) |
| far-range error 20-40 m, median m (raw) | 8 | 0.227 | 0.282 (x1.42, [+0.0441, +0.108], **worse**) | 0.279 (x1.42, [+0.0411, +0.104], **worse**) | 0.308 (x1.42, [+0.048, +0.113], **worse**) |
| far-range error 40-70 m, median m (raw) | 8 | 0.456 | 0.58 (x1.36, [+0.0938, +0.239], **worse**) | 0.577 (x1.35, [+0.0942, +0.232], **worse**) | 0.641 (x1.40, [+0.108, +0.252], **worse**) |
| far-range error 70-100 m, median m (raw) | 8 | 0.864 | 1.04 (x1.42, [+0.166, +0.362], **worse**) | 1.04 (x1.42, [+0.161, +0.355], **worse**) | 1.16 (x1.42, [+0.196, +0.386], **worse**) |
| lead on/off changes / min (tracked) | 8 | 1.58 | 3.03 (xnan, [+0.651, +2.22], **worse**) | 3.02 (xnan, [+0.722, +2.1], **worse**) | 3.15 (xnan, [+0.832, +2.29], **worse**) |
| lead identity changes / min (tracked) | 8 | 0.794 | 2.27 (xnan, [+0.655, +2.45], **worse**) | 2.11 (xnan, [+0.706, +1.93], **worse**) | 2.24 (xnan, [+0.781, +2.22], **worse**) |
| lead present, % of runs | 8 | 37.6 | 41.9 (xnan, [+0.0333, +9.79], -) | 43.3 (xnan, [+1.07, +12.2], -) | 45.8 (xnan, [+2.89, +15.3], -) |
| lead distance noise 0-15 m, m (tracked) | 4 | 0.0297 | 0.0817 (xnan, [+0, +0.145], same) | 0.104 (xnan, [+0.022, +0.126], **worse**) | 0.063 (xnan, [-0.0075, +0.0751], same) |
| lead distance noise 15-30 m, m (tracked) | 5 | 0.17 | 0.197 (x2.18, [-0.0612, +0.114], same) | 0.247 (x1.84, [+0.0034, +0.149], **worse**) | 0.133 (x0.91, [-0.0741, -0.0032], same) |
| lead distance noise 30-60 m, m (tracked) | 6 | 0.313 | 0.334 (xnan, [-0.0464, +0.103], same) | 0.436 (xnan, [+0.0451, +0.213], **worse**) | 0.26 (xnan, [-0.0967, -0.01], **better**) |
| lead distance noise 60+ m, m (tracked) | 7 | 0.536 | 0.591 (x1.13, [-0.0544, +0.236], same) | 0.757 (x1.63, [+0.0736, +0.427], **worse**) | 0.614 (x0.90, [-0.149, +0.339], same) |
| lead relative-speed noise, m/s (tracked) | 7 | 2.92 | 0.998 (x0.54, [-2.94, -0.569], **better**) | 1.06 (x0.57, [-2.74, -0.487], **better**) | 0.916 (x0.40, [-3.23, -0.744], **better**) |
| ego lane right, % of labelled frames (map count from label) | 3 | 90.2 | 87.9 (x0.98, [-3.97, -0.633], same) | 88.3 (x1.00, [-3.57, -0.167], same) | 88.4 (x1.00, [-3.57, -0.1], same) |
| ego lane wrong and confident, % (map count from label) | 3 | 3.43 | 5.53 (x0.96, [-0.233, +4.43], same) | 5.1 (x0.95, [-0.3, +3.63], same) | 4.73 (x0.95, [-0.3, +2.9], same) |
| ego lane right, % of labelled frames (no map) | 3 | 0 | 0 (xnan, [+0, +0], same) | 0 (xnan, [+0, +0], same) | 0 (xnan, [+0, +0], same) |
| ego lane wrong and confident, % (no map) | 3 | 0 | 0 (xnan, [+0, +0], same) | 0 (xnan, [+0, +0], same) | 0 (xnan, [+0, +0], same) |


Per ride, ego lane against the labels (map count from the label; frames / right / wrong and confident):
a1-night 3949 / 98.6 / 0.2 % (supercombo) vs 98.3 / 0.1 % (vision 13.8/s); krakow-night-tunnel 1631 / 89.5 / 2.6 % vs
84.2 / 9.3 %; warsaw-city 1700 / 82.5 / 7.5 % vs 81.2 / 7.2 %. The rate makes little difference for the vision model
(13.8 vs 20 runs/s are within a few percent on every metric): its weaknesses are per frame, not of the run rate.

The far-range ranking does not come from the odometry used: with supercombo's or with the vision model's own pose
alone (`EGO=sc|vis evaluate.py ...`) tunnels-s3 40-70 m is 0.16-0.17 m (supercombo) against 0.35-0.39 m (vision),
krakow-2024 0.55-0.60 against 0.91-0.99 m.

### Hand checks

**Lane lines.** `overlay_pair.py --kind lines` picked the moments where the two models' tracked inner lines differ
most at 45 m (17 picks with both lines present, 6 rides). Judged by eye on the frames first: at 45 m the eye cannot
tell a 1-2 m lateral difference near the vanishing point reliably, and the first eye judgements (vision better on
the motorway frames of tunnels-s3) were contradicted by the measurement below. So the picks were judged by
`adjudicate.py`: the 45 m point carried with the car's motion until it is about 10 m ahead, against the near-range
line there (mean of both models; at 10 m they agree to 0.1-0.2 m). Result: supercombo right 8, vision right 6,
ties 3 (krakow-2024, tunnels-s3, warsaw-storm: supercombo 5, vision 0, ties 3; s7-elblag-lowsun,
krakow-night-tunnel, a1-night: vision 6, supercombo 3). On the frames of greatest disagreement neither model is reliably right; the medians over
all frames (table above) favour supercombo.

**Lead presence (coordinator question 1).** `lead_presence_sample.py` drew, per ride, three random moments
(seed 1, moving, at least 3 s apart) where only the vision model has a tracked lead and three where only supercombo
has one, on eight rides (the snow ride has no lead in either). No lead labels exist (L20's labels are lanes), so
each frame was judged by eye: real = a vehicle in the ego lane at about the reported distance; false = nothing
there, an oncoming headlight, a vehicle in the next lane or on a crossing road; unclear = a vehicle far away
whose lane cannot be told.

| | moments | real | false | unclear |
|---|---|---|---|---|
| only vision has a lead | 21 | 10 | 4 | 7 |
| only supercombo has a lead | 14 | 8 | 3 | 3 |

How often each case happens (runs in the 4-5 min stretches): vision-only 36-483 runs per ride (a1-night 483,
warsaw-city 201), supercombo-only 13-197 (s7-elblag-lowsun 197). The extra presence of the vision model is mostly
distant leads (50 m and beyond) that are real about half the time; its false ones are the same kind as
supercombo's (oncoming headlights at night, cars in the next lane at a junction). It is not a clear gain, and the
distant leads are what the tracker drops and re-acquires (the doubled on/off and identity rates). Separately, the
worst-disagreement picks (`overlay_pair.py`, 10 lead picks) had supercombo right on 2, vision on 1, both wrong on
2 (a lead on an empty road at a junction), unclear 5; in one close case (krakow-2024 175.9 s) the vision model
put a car at 10 m at 26 m.

## 4. A temporal stage in front of the trackers (tuned on two rides, scored on the others)

Coordinator question 2. The candidate stage (vision streams only, supercombo untouched):

* in front of the trackers (`evaluate.py --pre PROB_TAU,GEO_TAU`, class `PreStage`): an exponential low-pass by
  message time of the presence probabilities (lane lines, lead 0) and of the geometry (line / edge y, lead x and v);
* the trackers' own hysteresis and filters (`--lanes-set`, `--lead-set`, config keys that exist already).

Six variants were run on the tuning rides krakow-2024, warsaw-storm, winter-snow-rural (`<datasets>/openpilot-models/
l24/tune/P0..P5.json`; winter-snow-rural carries no lines or leads, so the tuning in effect used two rides):

| variant | lead on/off / identity / inner-line changes per min, krakow + warsaw (supercombo: 1.75, 2.5 / 0.75, 1.25 / 4.5, 3.0) | jitter 50 m | far 40-70 m (krakow) |
|---|---|---|---|
| P0 none | 2.25, 6.26 / 1.75, 3.0 / 7.0, 3.5 | 0.084, 0.138 | 0.99 |
| P1 prob tau 0.3 s | 1.75, 2.75 / 1.5, 1.25 / 5.0, 3.5 | 0.081, 0.133 | 0.99 |
| P2 prob 0.3 s + geometry 0.1 s | 1.75, 2.75 / 1.0, 1.25 / 5.0, 3.5 | 0.070, 0.108 | 1.00 |
| P3 trackers: lead acquire 4 frames, coast 1.0 s, change confirm 0.8 s; lanes conf tau 1.0 s | 1.25, 2.0 / 1.0, 0.75 / 3.5, 3.5 | 0.083, 0.134 | 1.03 |
| **P4 = P2 + P3 (chosen)** | 1.75, 2.25 / 0.75, 1.0 / 3.5, 3.5 | 0.069, 0.097 | 1.04 |
| P5 stronger (0.5 / 0.2 s, acquire 5, coast 1.2 s, on/off 0.5/0.15) | 1.25, 1.5 / 0.5, 0.75 / 2.75, 6.0 | 0.056, 0.065 | 1.04 |

P4 reaches supercombo's flicker on the tuning rides (P5 overshoots into line dropouts on warsaw-storm). Scored on
the six other rides (`evaluate.py ROAD_DIR --rides gdansk-night-snow,tunnels-s3,s7-elblag-lowsun,krakow-night-tunnel,
a1-night,warsaw-city --streams sc.13.8,vis.13.8,vis.17,vis.20 $(cat P4.args)`, P4.args = `--pre 0.3,0.1 --tracker-on
vis --lead-set acquire_frames=4 --lead-set coast_s=1.0 --lead-set change_confirm_s=0.8 --lanes-set conf_tau_s=1.0`;
`summarize.py res_P4_heldout.json`):

| held-out, after the trackers | supercombo (default) | vision + P4, 13.8/s | vision + P4, 20/s | verdict |
|---|---|---|---|---|
| lead on/off changes / min | 1.67 | 2.14 | 2.35 | **worse** |
| lead identity changes / min | 0.87 | 1.82 | 1.74 | **worse** |
| inner lane-line presence changes / min | 1.9 | 2.7 | 2.5 | worse / same |
| inner-line jitter 10 / 50 m, m | 0.019 / 0.050 | 0.019 / 0.038 | 0.019 / 0.039 | same / better |
| far range 20-40 / 40-70 / 70-100 m, median m | 0.10 / 0.21 / 0.40 | 0.14 / 0.34 / 0.62 | 0.16 / 0.39 / 0.73 | **worse** |
| lead distance noise 15-30 / 30-60 m | 0.21 / 0.27 | 0.12 / 0.20 | 0.05 / 0.12 | better |

Cost of the stage: computation negligible (a few exponential filters per `road` message in the api); freshness:
the geometry low-pass lags a ramp by its 0.1 s time constant, presence appears and goes about 0.3 s later, and a
new lead needs 4 frames instead of 2 (0.29 s instead of 0.15 s at 13.8 runs/s). It does not bring lead and
inner-line stability back to supercombo's level on rides it was not tuned on, and it cannot touch the far-range
error (a geometry bias, not noise). So no net freed budget is claimed; the stage is not proposed.

## 5. The plan gap

Without the policy the `road` message has no `plan`. What uses it: `roadgeom.py` (road geometry when no lane line
is tracked), `view.py` (the `path` object, light positions relative to the path), `lights.py` (path-based ego light;
falls back to "image" mode), `overlay.py`. Measured (`view.road.vision.from`, share of runs; stream with the plan
removed = `vis.13.8~noplan`):

| ride | supercombo: geometry from plan | vision + policy: from plan | vision without plan: no geometry at all |
|---|---|---|---|
| winter-snow-rural (unmarked snow) | 99.7 % | 100 % | **100 %** |
| gdansk-night-snow | 49.6 % | 67.2 % | **80.5 %** |
| warsaw-storm | 44.4 % | 48.7 % | **78.1 %** |
| krakow-2024 | 13.6 % | 12.0 % | 26.7 % |
| tunnels-s3, s7-elblag-lowsun (marked expressway) | 0 % | 0-3 % | 0-3 % |

So the policy is needed wherever lines are missing. Its cost: 6.0 ms per run on one laptop core (memory bound:
28 MB of fp32 weights per run), 1.25 ms per run on average with `--policy-hz 5` (plan held between policy runs,
up to ~1 m different at the 10 s end of the plan); on the phone not measured (pending); it does not convert to
ncnn (unsupported layers), so it stays on ORT CPU. Deriving a path from the lane lines instead is what
`roadgeom.py` already does when lines exist; it gives nothing where the plan matters (no lines).

## 6. Latency of the 200 ms pair

The vision model sees the frame 200 ms earlier and the current one; its output belongs to the current frame like
supercombo's, so the delay to the output is the same (and its run is shorter). What moves is the motion estimate:
its baseline is centred 100 ms back instead of 25 ms. Cross-correlation of the first differences against
supercombo (`lag_vs_ref_ms` in the results): the model's own speed (pose vx) lags by +30 to +140 ms (weak
correlation, 0.13-0.38), the tracked lead distance by 0 ms (correlation 0.5-0.9). In exchange the lead's relative
speed is 2-3 times less noisy after the tracker (table). Nothing in the api showed a timing problem from it.

## 7. Phone steps (pending: the phone was not reachable)

All of these need only the files below; nothing was copied or run on the phone by this lane.

Files to copy (from the laptop; models are not in the repository):

| file | size | from |
|---|---|---|
| `driving_vision-ncnn.param` | 19,656 B | `<datasets>/openpilot-models/l24/ncnn-model/` (or `tools/ncnn/make_vision_ncnn.sh driving_vision-fp32.onnx driving_vision-ncnn`) |
| `driving_vision-ncnn.bin` | 78,572,356 B | same |
| `driving_vision-fp32.onnx` (ORT reference for the gate) | 92,985,140 B | `<datasets>/openpilot-models/v0.11.1/` |
| `driving_policy-fp32.onnx` | 27,959,695 B | same |

Build on the phone from branch `v2-l24m` (same as today): `CMAKE_ARGS="-DROADSENSE_NCNN=ON -DNCNN_DIR=<phone-home>/ncnn"`
with the usual `cmake -S . -B build ...; cmake --build build -j4` (targets `modeld`, `modeld_replay`).

1. fp16 gate (ncnn Vulkan fp16-packed against ORT fp32, plus the pipelined path against one-stage), one command:
   `REPLAY_ARGS="--calib /etc/roadsense/calib.json" MODELD_REPLAY=build/modeld/modeld_replay sh tools/ncnn/vision_gate.sh ljkr.nv12 1920 1080 MODEL_DIR 400 ~/l24/gate`
   (MODEL_DIR = where the four files are; prints ACCEPT/REJECT with the docs/HARDWARE.md thresholds and the mean
   model ms of each run).
2. GPU time per run: the `model ... ms avg` line of the gate's ncnn run (single process, no sign provider); for the
   stack, `modeld --road-model vision --backend ncnn --model MODEL_DIR/driving_vision-ncnn.param --policy
   MODEL_DIR/driving_policy-fp32.onnx [--policy-hz 5]` in place of the installed modeld arguments (the rest
   unchanged), and `tools/budget/run.sh` as in L21 for W, GPU busy and `md-*` thread CPU (the policy runs on
   `md-main`).
3. Policy cost on the A75: the `policy` figure in the same `modeld_replay` line (`--policy-threads 1`).

## 8. Limits

* No ground truth. The far-range measure is self-consistency (a far prediction against the later near view); a
  model that is consistently wrong in the same way near and far would not be caught, and the near reference
  itself mixes both models where it is used for adjudication.
* The rides' calibrations (pitch / yaw / focal) are estimates, and their overlays were judged plausible with
  supercombo. A calibration that suits one model's training distribution better could favour it; both models use
  the same warp intrinsics, so the bias, if any, is second order, but it cannot be excluded without the car's
  calibrated camera.
* Labels: 3 rides x 5 min for the ego lane; no lead labels (hand checks of 35 + 10 frames, one judge).
* The laptop's numbers are ORT fp32; the phone runs ncnn fp16 (laptop GPU gate: lane error mean 0.004 m, so the
  comparison holds for the phone up to that).
* s7-elblag-lowsun has no 20/s stream; a1-s8-rain was not run (time, after the restart).

## 9. Tests and commits

* C++ unit check against the Python reference: section 1.
* api tests: `PYTHON=~/projects/roadsense/.venv-models/bin/python sh tools/smoke_api.sh` (unittest; that
  venv has aiohttp; no interpreter on the laptop has both aiohttp and pytest, the suite does not need pytest):
  **214 tests, OK** (211 before + 3 in `api/tests/test_vision_road.py`: the pipeline and the ego-light selection
  with a vision `road` message without `plan`, and with a policy plan).
* No api code changed (no tracker-side fix was proposed: section 4).
* Commits on `v2-l24m`: 789f0cf (modeld vision path, tools), ea1d8eb (evaluation, contract test), e595760
  (temporal stage options, adjudication, presence sampler), and the commit of this report.
