# L21: where the phone's budget goes, what was wasted, what the recovered part buys

Lane of the 2.0 campaign, 2026-11-18 18:45-20:50 CET, Fable. Phone on the bench (no camera, no gateway),
reached over its own access point; rides replayed by `camerad-replay`. Branches `v2-l21` in roadsense (from
`v2-l19`) and in roadsense-signs. Nothing was installed on the phone: candidates ran from `~/l21` as
transient units next to the stopped installed stack. Times CET.

Owner's ruling during the lane (2026-11-18): latency to the display does not matter; quality and freshness
of the measurement topics do. The display path is a place to save, not to spend.

## Summary

| | installed today | envd moved (no output change) | **proposed: + lights at 5 Hz, new state model** |
|---|---|---|---|
| phone rail, fan 100 % | 6.67 W | 6.19 W | 7.01 W |
| big cores busy (of 400 %) / their temperature | 146 % / 70 C | 99 % / 62 C | 137 % / 68 C |
| road messages, latency camera -> /ws median / p90 | 13.7/s, 120 / 140 ms | 13.9/s, 108 / 129 ms | 13.8/s, 110 / 131 ms |
| lights | 3.0/s | 3.0/s | 5.0/s |
| light-time reported on the replays, day / night | 95.5 / 93.6 % | same | 98.3 / 98.9 % |
| lights under 12 px reported, day / night | 81 / 81 % | same | 93 / 96 % |
| lit lights of the car's drives given their colour / called dark | 70 / 29 % | same | 91 / 9 % (cross-validated) |

* The stack costs 5.6 W above the idle phone (1.06 W). The GPU is busy 91 % of the time and cool (57 C);
  the big cores are the hot part. So the GPU is free in heat and full in time: nothing more fits on it
  without taking road runs away. The big cores are where waste hurts.
* The largest waste was envd: one scenery label a second took 61 % of a big core, 1.25 W (19 % of the
  phone's power) and 15 C on the big-core sensors. On the little cores it gives the same labels for 0.5 W
  less.
* The recovered big-core time went to the traffic lights (5 Hz instead of 3), and the state classifier was
  retrained for what the car's camera does to a lit lamp. Together they cost 0.3 W more than today's stack
  at the same big-core temperature.
* A newer openpilot model (v0.11.1 vision alone, MIT) is a real candidate for a build lane: no recurrent
  state, converts to ncnn with every layer on Vulkan, no fp16 overflow, an estimated 28-38 ms of GPU per
  run against 47 ms. Not built: it needs a new parser and a presence filter, and its cost on the Adreno is
  an estimate.

## 1. Method

`tools/budget/` (roadsense):

* `stack.sh up PROFILE | down`: a whole stack as transient units `l21-*` in `roadsense.slice`, own run
  directory, ring, config and data directory; a profile overrides binaries and arguments, the default is
  the installed stack with its arguments and clock pins. The api listens on the installed port (8090): the
  car backend's watchdog restarts the installed target when that port is silent for 3 minutes, and its scene
  feed is then the real consumer of the candidate.
* `measure.py NAME SECONDS`: CPU per unit and per thread from `schedstat`, split into little and big cores
  by the core each thread was last seen on (10 Hz); GPU time per unit from the DRM `fdinfo`
  (`drm-engine-gpu`); GPU clock residency; rail power (`qcom-battery` voltage x current, 5 Hz); every thermal
  zone at 1 Hz; stops the units above 95 C.
* `wsprobe.py`: a `/ws` consumer on every topic: rate and latency from the frame's camera timestamp.
* `run.sh NAME PROFILE [SECONDS] [SETTLE]`: up, settle, budget with only the host's consumer connected, 45 s
  with the probe, the daemons' stats lines, down. `table.py` prints the tables below from the result files.
* `tap.py` / `api_profile.py`: the daemons' socket streams recorded on the phone and fed through the api's
  `State` on the dev box, per input type and with cProfile.

modeld and signd name their threads (`md-prep`, `md-prefetch`, `md-main`, `sg-signs`, `sg-lights`; pools
inherit the name of the thread that created them), which is what makes the per-stage rows possible.

All runs: ride `krakow-2024` from its start, fan 100 % unless stated, host in roadsense mode `on` (8 cores
online), 240-300 s measured after 60 s. Reproduce a row: `sudo sh ~/l21/run.sh NAME ~/l21/profiles/NAME.sh 240 60`.
One run per configuration; the spread between two runs of one configuration (A0 and the same stack with
named threads) was 0.07 W and 3 C.

## 2. Where it goes today

CPU in percent of one core; GPU in percent of wall time.

| | installed stack (240 s) | the same, lane build with named threads |
|---|---|---|
| phone rail, W (mean / last 60 s) | 6.67 / 6.56 | 6.74 / 6.88 |
| temperature last 60 s, C: big cores / little / GPU | 70 / 63 / 57 | 67 / 61 / 58 |
| hottest zone, max over the run, C | 84.4 | 85.0 |
| CPU, % of one core: little cores / big cores | 60 / 146 | 59 / 146 |
| GPU busy, % of wall (sum of the daemons' GPU time) | 91.1 | 90.8 |
| camerad: little / big / GPU | 16.4 / 0.2 / 0.0 | 16.6 / 0.1 / 0.0 |
| modeld: little / big / GPU | 20.2 / 18.9 / 65.5 | 19.1 / 18.8 / 65.4 |
| &nbsp;&nbsp;thread `md-prep`: little / big | - | 19.1 / 0.0 |
| &nbsp;&nbsp;thread `md-prefetch`: little / big | - | 0.0 / 11.5 |
| &nbsp;&nbsp;thread `md-main`: little / big | - | 0.0 / 7.3 |
| signd: little / big / GPU | 0.0 / 65.0 / 25.6 | 0.0 / 65.0 / 25.4 |
| &nbsp;&nbsp;thread `sg-lights`: little / big | - | 0.0 / 54.3 |
| &nbsp;&nbsp;thread `sg-signs`: little / big | - | 0.0 / 10.7 |
| envd: little / big / GPU | 0.0 / 61.0 / 0.0 | 0.0 / 61.2 / 0.0 |
| mapd: little / big / GPU | 0.2 / 0.0 / 0.0 | 0.2 / 0.0 / 0.0 |
| api: little / big / GPU | 23.3 / 0.9 / 0.0 | 23.3 / 0.7 / 0.0 |

Per stage:

| daemon, stage | cores | cost | from |
|---|---|---|---|
| camerad-replay: JPEG decode of 20 frames a second to NV12, preview file every 2nd frame (the camera's own JPEG, no re-encode) | little | 16 % | budget |
| modeld prep: warp + pack of every camera frame for both model inputs, 10 ms | little | 19 % | thread `md-prep` |
| modeld prefetch: recording the next run's vision stage, 6.5 ms, 13.7 + 6.3 re-records a second | big | 12 % | thread `md-prefetch` |
| modeld main: policy stage, CPU layers, parse, JSON (1 ms) | big | 7 % | thread `md-main` |
| modeld on the GPU: 47 ms per run | GPU | 65 % | fdinfo |
| signd signs: letterbox, classifier and verifier on ORT, tracker | big | 11 % | thread `sg-signs` |
| signd sign detector, 51 ms per run, 5 a second | GPU | 25 % | fdinfo |
| signd lights: detector on the CPU (ncnn, 2 threads, 93 ms), 3 a second, state CNN | big | 54 % | thread `sg-lights` |
| envd: one frame a second, 320 ms on 2 threads | big | 61 % | budget |
| api: 5,600 messages in 120 s; of its time `road` parse 16 %, lane tracker 27 %, `view` 16 %, JSON encode of the derived topics 11 %, roadgeom, lead, rules and lights 5 % each | little | 23 % | budget, `api_profile.py` |
| mapd without a position source | little | 0.2 % | budget |

Watts by subtraction (150 s runs, same ride and fan):

| stack | rail W | big cores C | road/s |
|---|---|---|---|
| idle phone, 8 cores online, no pins | 1.06 | 29 | - |
| idle + CPU and GPU clock pins of the modeld unit | 1.28 | 30 | - |
| idle + pins + CPU wake-up latency hold | 1.62 | 31 | - |
| modeld alone (camera, api, mapd running) | 4.23 | 42 | 19.1 |
| + signd (signs and lights) | 5.41 | 55 | 13.8 |
| + envd = installed | 6.67 | 70 | 13.8 |
| installed, modeld capped at 10 runs a second | 6.07 | 61 | 9.8 |
| installed, without the wake-up latency hold | 6.58 | 62 | 13.3 |
| installed, without the clock pins | 6.17 | 69 | 12.8 (road latency 151 / 174 ms) |

Readings:

* A fully busy big core is about 2 W; the GPU at 90 % about 2.3 W; the little cores are an order cheaper
  per unit of work done (envd's 61 % of a big core became 168 % of little cores for 0.6 W less).
* The GPU is saturated in time. modeld alone takes 90 % of it at 19 runs a second; with the sign detector
  it is 65 % + 25 %. Every detector run added to the GPU costs about one road run (51 ms against 47 ms).
  The premise "GPU work is comparatively free" holds for heat and does not hold for time.
* The clock pins and the latency hold are not waste: 0.5 W buys 30 ms of road latency and 0.9 road runs a
  second; kept under the freshness ruling.

## 3. Waste removed

### envd on the little cores (shipped on the branch)

`systemd/roadsense-envd.service`, `config/envd.json`: `--threads 3 --cpus 0-3` instead of `--threads 2 --cpus 4-7`.

| | big cores, 2 threads | little cores, 3 threads |
|---|---|---|
| time per frame, rate | 326 ms, 1.0/s | 681 ms, 1.0/s |
| rail | 6.67 W | 6.19 W |
| big cores busy / temperature / hottest zone max | 146 % / 70 C / 84 C | 99 % / 62 C / 76 C |
| road latency median / p90 | 120 / 140 ms | 108 / 129 ms |
| `env` latency | 371 ms | 723 ms |

Accuracy check: `envd_replay` on 60 frames of `krakow-lights` (every 100th of the first 6000), big cores with
2 threads against little cores with 3: smoothed and per-frame labels identical on 60 of 60, every score
identical to the three decimals printed (`~/l21/envchk.sh`). The label is smoothed over 10 s, so 0.35 s more
latency does not show. The road latency falls because camerad and the api now find a big core free.

### Tried and rejected

| change | result |
|---|---|
| compact warp table in modeld's prep (6 bytes per pixel instead of 24, weights from a lookup; bit-identical on 10 camera / calibration cases) | slower on the A55 (4.3 against 3.7 ms per packed frame in the same process): the cost is the source gathers, not the table. Reverted; `modeld/warp_test.cc` (the table against the definition, and its cost) stays. |
| modeld's prefetch thread on the little cores | 12 % of a big core saved, road 13.7 -> 13.25 runs a second, envd down to 0.96/s. Rejected, option removed. |
| modeld without the wake-up latency hold, or without clock pins | see the table above: slower for 0.1 / 0.5 W. Kept as installed. |
| api: lazy `view`, cheaper serialisation | the api is 23 % of a little core, about 0.1 W; `view` is 16 % of that. Under the ruling it is the place to save, but there is next to nothing to save; not touched (its trackers are other lanes' surfaces). |
| camerad preview path | already the camera's own JPEG, no encode; the recorder copies one a second. Nothing to remove. |

Binaries: the lane's modeld and signd give the same output as the installed ones (`modeld_replay` 40 of 40
lines, `signd_replay` 60 of 60 lines identical apart from timing fields, on `ljkr.nv12`).

## 4. Budget spent on quality

### Traffic lights at 5 Hz (shipped on the branch)

roadsense-signs unit: `--lights-hz 5 --lights-coast 3` (was 3 and 2; the two views alternate, so each is now
looked at 2.5 times a second; coast 3 keeps the 0.6 s a missed light stays listed).

`lights_replay` on the phone's own backend (ncnn CPU, fp16) on the first 5 minutes of two rides, scored by
roadsense-signs `tools/lights/replay_score.py` against lane L15's reference files (`~/l21/lightseval.sh`):

| ride (lit reference lights, light-seconds) | lights per second | light-time reported | under 12 px | 12-24 px | 24-48 px | drop-outs per min of light-time | reported light-frames without a reference box |
|---|---|---|---|---|---|---|---|
| krakow-lights, day (46, 844 s) | 2.9 (installed) | 95.5 % | 81.2 % | 94.3 % | 99.0 % | 1.7 | 3.1 % |
| | **5** | **98.3 %** | 92.5 % | 97.9 % | 99.5 % | 0.2 | 3.2 % |
| | 6.7 | 98.6 % | 92.7 % | 98.5 % | 99.8 % | 0.1 | 2.8 % |
| krakow-night-tunnel (25, 75 s) | 2.9 (installed) | 93.6 % | 81.1 % | 97.5 % | 91.5 % | 2.4 | 7.1 % |
| | **5** | **98.9 %** | 95.9 % | 99.6 % | 100 % | 0.0 | 9.9 % |
| | 6.7 | 98.7 % | 94.6 % | 99.6 % | 100 % | 0.0 | 11.2 % |

(The installed row reproduces L15's 95.8 / 93.9 % measured with ORT fp32 on the dev box.) 5 Hz takes most of
what there is; 6.7 Hz adds nothing. Cost: `sg-lights` 54 -> 90 % of a big core. The reference is the same
detector at its best, so the shares are upper bounds, as in L15.

### State classifier for the car's own camera (script and unit on the branch, weights local)

The known weak point (L16: 95 of 107 observations of one green light reported `off`). Measured on the two
drives of 2026-11-17 the car recorded (frames once a second): of 1,076 detections at score >= 0.25, a
stratified sample of 320 labelled by eye. Against a bright sky the DVR camera shows a lit lamp as a white
disc with a thin coloured rim on a dark housing; the model trained on BDD100K calls that `none`.

| model | lit lights: right colour / wrong colour / `none` | others called lit | BDD100K test half, recall red / yellow / green / none |
|---|---|---|---|
| installed | 70.2 / 1.0 / 28.8 % (through signd on the phone 69.7 / 1.0 / 29.3 %) | 14.2 % | 87.1 / 61.0 / 95.8 / 88.2 |
| retrained with clipped-highlight augmentation, no car picture | 83.8 / 1.0 / 15.2 % | 15.4 % | 81.0 / 76.0 / 95.2 / 85.2 |
| + fine-tuned on the car sample, cross-validated | **90.8 / 0.5 / 8.7 %** | 10.7 % | 85-87 / 60-63 / 95 / 90 |

Details, the control runs and the limits (one camera, two sunny days, one labeller, in-sample design of the
augmentation, 4 yellow phases) in roadsense-signs `docs/LIGHT_MODELS.md`. The classifier costs the same
(1 ms per crop). `signd --lights-state A.onnx,B.onnx` takes the first file installed.

Not measured: what this does to the ego light over a drive (the recorder keeps one frame a second, the
tracker needs the detector's own rate). The next recorded drive with the new model is that measurement.

## 5. Newer openpilot models (evaluated on the dev box, not built)

A sub-lane (Opus, dev box only; scripts in `tools/models_survey/`, files in `<datasets>/openpilot-models/`).

| | supercombo v0.9.7 (installed) | v0.11.1 `driving_vision.onnx` |
|---|---|---|
| licence | MIT (repository licence, no separate model licence) | same |
| outputs | lanes, edges, leads, pose, plan, meta from one recurrent graph | lanes, edges, leads (3, one hypothesis), pose, road transform, meta from the vision model alone; plan from a separate 7 M-parameter policy |
| inputs | two fp16 image pairs 50 ms apart, desire, features buffer and previous curvature fed back | two uint8 image pairs 200 ms apart; no state |
| ncnn layers / on the CPU | 499 / 5 | 236 / 0 |
| largest activation (fp16 limit 65,504) | 1.3e7 (needs the normalisation fix) | 583 |
| fp16-packed against fp32, 40 single runs: lane error mean / max up to 100 m | 0.0074 / 0.060 m | 0.0074 / 0.054 m |
| ONNX Runtime CPU, 2 threads | 46.0 ms | 35.3 ms |
| ncnn fp16-packed on a CPU Vulkan driver | 2031 ms | 1218 ms |
| ego-line jitter at 10 m, four rides | 0.10-0.21 m | 0.08-0.14 m (lower on three) |
| ego-lane on/off changes per minute at 0.5 | 0-41 | 2-69 |
| lead on/off changes per minute | 6-28 | 20-81 |

* v0.9.8 / v0.9.9: lanes and leads come from the policy with a 5 Hz feature history: no gain here.
  v0.10.1: same structure as v0.11.1, steadier presence, but supercombo-like cost and activations to 4e7.
  v0.11.2 and master: one stateful graph again; master has 42 MatMul layers, which ncnn cannot place on
  Vulkan.
* Expected GPU cost of the v0.11.1 vision model: 28-38 ms per run (47 ms scaled by the measured ratios
  0.60-0.77); the multiply-accumulate count is equal, the saving is in elementwise layers, so this is an
  estimate until it runs on the Adreno. At 13.7 runs a second that would free 9-19 % of the GPU's time: room
  for the sign detector at 7-8 Hz or a second, full-scale sign view.
* What it would need: a parser for the 1,576-value output (slice table in the sub-lane's hand-back, kept in
  `tools/models_survey/` scripts), a ring of five warped frames per input, the phone accuracy gate
  (`tools/ncnn/compare_replay.py`) for the new graph, and a check that the api's lane and lead trackers absorb
  the extra flicker of a model without memory. The comparison shows stability and agreement, not
  correctness: there is no ground truth, and the rides' calibrations are guesses.

Output of the v0.11.1 vision model (`[1,1576]`, read from the ONNX metadata `output_slices`; every log-std
block is followed by exp; the X and T grids are those of v0.9.7):

| block | indices | layout |
|---|---|---|
| meta | 0..54 | engaged 0; gas / brake disengage, steer override, hard brake 3 / 4 / 5 for 2..10 s; gas, brake, left and right blinker for 0..10 s; sigmoid |
| desire_pred | 55..86 | 4 x 8, softmax per row |
| pose | 87..98 | mean 87..92 (velocity m/s, rotation rates rad/s), log-std 93..98 |
| wide_from_device_euler | 99..104 | mean, log-std |
| road_transform | 105..116 | mean 105..110 (index 2 = camera height), log-std |
| lane_lines | 117..644 | mean 117..380 = 4 lines x 33 points x (y, z); log-std 381..644 |
| lane_lines_prob | 645..652 | line i = sigmoid(raw[645 + 2 i + 1]) |
| road_edges | 653..916 | mean 653..784 = 2 x 33 x 2; log-std 785..916 |
| lead | 917..1060 | mean 917..988 = 3 leads (now, +2 s, +4 s) x 6 times x (x, y, v, a); log-std 989..1060 |
| lead_prob | 1061..1063 | sigmoid |
| hidden_state | 1064..1575 | 512 values, the policy's input |

Inputs `img`, `big_img`: uint8 `[1,12,128,256]`, channels 0..5 the frame 200 ms earlier, 6..11 the current
one, the same six-channel YUV packing and warp matrices as today; normalisation is inside the graph.
Conversion: onnxsim, pnnx with `inputshape=[1,12,128,256]u8,[1,12,128,256]u8`, then the two `Tensor.to`
layers removed (`tools/models_survey/ncnn_check.py`, `fix_vision_param`).

## 6. Before and after

Fan 100 %, ride `krakow-2024`, 240 s (300 s for the proposed stack) after 60 s.

| | installed | envd on the little cores | proposed (envd little, lights 5 Hz, car state model; 300 s) |
|---|---|---|---|
| phone rail, W (mean / last 60 s) | 6.67 / 6.56 | 6.19 / 6.30 | 7.01 / 6.83 |
| temperature last 60 s, C: big cores / little / GPU | 70 / 63 / 57 | 62 / 58 / 58 | 68 / 63 / 58 |
| hottest zone, max over the run, C | 84.4 | 75.5 | 79.8 |
| CPU, % of one core: little cores / big cores | 60 / 146 | 197 / 99 | 203 / 137 |
| GPU busy, % of wall (sum of the daemons' GPU time) | 91.1 | 91.7 | 91.3 |
| camerad: little / big / GPU | 16.4 / 0.2 / 0.0 | 3.9 / 6.1 / 0.0 | 3.7 / 6.7 / 0.0 |
| modeld: little / big / GPU | 20.2 / 18.9 / 65.5 | 19.2 / 19.2 / 66.3 | 20.7 / 19.7 / 65.9 |
| &nbsp;&nbsp;thread `md-prep`: little / big | - | 19.2 / 0.0 | 20.7 / 0.0 |
| &nbsp;&nbsp;thread `md-prefetch`: little / big | - | 0.0 / 11.7 | 0.0 / 12.0 |
| &nbsp;&nbsp;thread `md-main`: little / big | - | 0.0 / 7.5 | 0.0 / 7.7 |
| signd: little / big / GPU | 0.0 / 65.0 / 25.6 | 0.0 / 64.8 / 25.4 | 0.0 / 101.1 / 25.4 |
| &nbsp;&nbsp;thread `sg-lights`: little / big | - | 0.0 / 54.0 | 0.0 / 89.7 |
| &nbsp;&nbsp;thread `sg-signs`: little / big | - | 0.0 / 10.8 | 0.0 / 11.4 |
| envd: little / big / GPU | 0.0 / 61.0 / 0.0 | 168.4 / 0.0 / 0.0 | 172.4 / 0.0 / 0.0 |
| mapd: little / big / GPU | 0.2 / 0.0 / 0.0 | 0.2 / 0.0 / 0.0 | 0.2 / 0.0 / 0.0 |
| api: little / big / GPU | 23.3 / 0.9 / 0.0 | 5.6 / 9.2 / 0.0 | 5.8 / 9.5 / 0.0 |

| topic: msg/s, latency median / p90 ms | installed | envd on the little cores | proposed (envd little, lights 5 Hz, car state model; 300 s) |
|---|---|---|---|
| camera | 20.0/s, 13.3 / 24.9 | 20.0/s, 6.4 / 15.3 | 20.0/s, 5.7 / 13.9 |
| env | 1.0/s, 370.8 / 395.0 | 1.0/s, 722.6 / 745.4 | 1.0/s, 736.1 / 752.6 |
| fused | 2.1/s, 4.7 / 7.1 | 2.1/s, 2.3 / 5.5 | 2.1/s, 2.3 / 5.2 |
| lanes | 13.7/s, 122.9 / 142.2 | 13.9/s, 109.9 / 130.9 | 13.8/s, 108.8 / 130.0 |
| lead | 13.7/s, 122.4 / 141.7 | 13.9/s, 109.6 / 130.5 | 13.8/s, 108.5 / 129.8 |
| lights | 3.0/s, 144.2 / 168.7 | 3.0/s, 130.7 / 154.0 | 5.0/s, 138.7 / 152.4 |
| map | 2.0/s, 3.1 / 17.4 | 2.0/s, 1.6 / 3.8 | 2.0/s, 2.3 / 11.2 |
| road | 13.7/s, 119.8 / 139.5 | 13.9/s, 108.2 / 128.6 | 13.8/s, 107.0 / 128.8 |
| roadgeom | 13.7/s, 123.6 / 142.6 | 13.9/s, 110.1 / 131.2 | 13.8/s, 109.0 / 130.1 |
| rules | 1.1/s, 129.2 / 156.7 | 1.1/s, 112.6 / 139.0 | 1.3/s, 124.2 / 149.5 |
| signs | 5.0/s, 140.9 / 171.2 | 5.0/s, 131.5 / 165.0 | 5.0/s, 133.0 / 160.3 |
| speed_limit | 1.0/s, 126.0 / 149.1 | 1.0/s, 115.9 / 139.9 | 1.0/s, 114.4 / 148.6 |
| vehicle | 2.0/s, 3.3 / 18.5 | 2.0/s, 1.5 / 3.8 | 2.0/s, 2.4 / 11.2 |

| daemon stats (mean of the 10 s lines) | installed | envd on the little cores | proposed (envd little, lights 5 Hz, car state model; 300 s) |
|---|---|---|---|
| modeld runs/s | 13.74 | 13.91 | 13.72 |
| modeld model ms mean / p90 | 54.0 / 57.31 | 53.58 / 56.33 | 54.17 / 57.31 |
| modeld prep ms (per camera frame) | 10.05 | 9.4 | 10.17 |
| signd signs/s, detect ms (detector) | 4.99, 98.54 (56.19) | 4.99, 96.45 (55.59) | 4.99, 96.95 (55.42) |
| signd lights/s, lights ms (detector) | 3.0, 104.31 (92.66) | 3.0, 98.34 (86.83) | 4.99, 97.83 (86.58) |
| envd frames/s, ms per frame | 1.0, 325.94 | 1.0, 680.94 | 1.0, 706.37 |

With the fan in automatic mode (the car's setting; bench, 300 s after 90 s):

| | installed | proposed |
|---|---|---|
| phone rail, W (mean / last 60 s) | 6.83 / 6.85 | 6.96 / 6.93 |
| temperature last 60 s, C: big cores / little / GPU | 63 / 59 / 59 | 70 / 66 / 62 |
| hottest zone, max over the run, C | 89.0 | 83.7 |
| CPU, % of one core: little cores / big cores | 61 / 147 | 203 / 138 |
| GPU busy, % of wall (sum of the daemons' GPU time) | 91.0 | 91.4 |

| topic: msg/s, latency median / p90 ms | installed | proposed |
|---|---|---|
| lights | 3.0/s, 147.0 / 178.3 | 5.0/s, 126.4 / 148.7 |
| road | 13.6/s, 120.4 / 139.9 | 13.7/s, 108.1 / 130.1 |

Indicative only: the fan duty was not logged, the policy moves it with the temperature (40 % at the end of
the second run), and the installed stack ran first, straight after the fan left 100 %. What it shows: both
stacks stay under 90 C on the bench with the fan left to itself, and the difference at the rail is 0.13 W.

## 7. Open

* **Sign detector.** Signs under about 15 px are still missed and the detector still runs 5 times a second.
  It is on the GPU, which is full: a second, full-scale view on alternate frames (as for the lights) costs
  nothing but halves each view's rate, and the tracker already loses small edge signs at 5 Hz (L12). More
  runs cost road runs one for one. The v0.11.1 model is the way to make room.
* **Lights detector on the CPU** is now the largest big-core load (90 %). int8 through ncnn's quantisation
  tools (built on the phone) may halve it; needs the accuracy check on BDD100K val, and YOLO11's attention
  block is a known risk for int8. The BDD-trained YOLOv8n (all layers on Vulkan) is on the dev box unloaded:
  loading that checkpoint was refused by the permission system in L7 and was not retried here.
* **The detector's `traffic sign` class** (BDD100K class 9) is computed in every lights run, at a larger scale
  than the sign detector sees, and thrown away. Whether it finds Polish signs the MTSD detector misses is a
  replay on the six labelled rides; not done.
* **Arrows** are not a state. It needs labelled arrow crops; the car's recordings hold a few (five green
  arrows among the 320 labelled crops).
* **The state model** has seen two sunny drives. Night, rain and yellow need recorded drives; the scan and
  labelling tools take a session directory.
* **`/preview` and far-range lane accuracy** (item d of the brief): not addressed.
* The measurement has no camera: `camerad-dvr` reading the USB camera was not in any run (notebook fold 0: about
  3 % of a core at 19.5 frames a second).

## 8. Phone

Left as found: transient units stopped, clock pins released, fan automatic, roadsense mode `auto`. The
lane's files are in `<phone-home>/l21` (sources, builds, profiles, results in `out/`, replay frames in `lf/`,
1.4 GB). Side effects: every start of the host's mode `on` and every watchdog restart of the installed target
opened a bench recorder session in `/var/lib/roadsense/rec` (five, `20261118-185123` to `20261118-202334`; left in place).
No reset, no drop from the network; hottest reading 85 C at fan 100 %, 89 C with the fan in automatic mode.
