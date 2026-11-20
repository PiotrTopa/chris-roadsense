# L48-E envd: the scene analyser, cheaper at the same quality?

Sub-lane of L48 (whole-system optimisation), 2026-11-20 07:45-09:30 CET, laptop only (the phone was held by
another lane). Rule: same quality SHOWN after the consumers, never assumed.

## 0. Cost today
- One ViT-B/32 image tower (ORT CPU fp32, 3 threads on cpu0-3) per frame. Measured on the car (recordings of
  2026-11-19, `env_ms`): **656-1138 ms per frame** (median per drive; 1.1 s on the two longest drives, the
  bench figure 680 ms), so envd ran at 0.88-1.0 frames/s instead of 1. CPU last night: 70-195 % of one core
  (about half of the little cluster).
- Prep (box squash + stats) and scoring: **17-18 ms per frame** (`env_ms - encode_ms`), 1.5-2.7 % of the
  frame (candidate B4: nothing to gain there).
- Savings below are given as the share of envd's CPU removed: envd's CPU is the tower's, so it scales with the
  inference rate (inference rate x CPU per inference; the republish copies cost nothing measurable).

## 1. Method (candidate A: rate)
- Per-frame embeddings by the real C++ pipeline (`envd_replay --nv12 --emb`, probe class file as installed on
  the phone) on all **21 rides** of the library (246 min) on a 2 Hz grid (frames 0, 10, 20, ... of 20 fps),
  plus 32x18 Y/U/V thumbnails at 4 Hz (`tools/envd/rate_extract.py`); cache `<datasets>/envd-cache/l48/`.
- The car's **own recordings** (8 drives of 2026-11-19; 6 finished in the simulation, 70 min with frames,
  the two longest drives 20261119-104128 / -135622 not finished): the phone's own 1 Hz per-frame scores,
  replayed with the recorded `road` / `signs` / `ego` / `lights` / `map` streams.
- `tools/envd/rate_sim.py`: replica of `envd/smoother.h` (checked: reproduces the phone's smoothed labels on
  100 % of the recorded messages for place and weather and 99.4-100 % for light), every schedule, then the
  consumers: the real `RulesInEffect` / speed-limit engine (env area prior, vision limit), the real `Fusion`
  methods (built_up, place, tunnel, limit, with the recorded map), replicas of the car's tunnel rule (vision
  path: tunnel >= 0.5 held 3 s / 5 s out), the curve alert's "wet" and the MFD 5 s staleness rule.
  Rides have no road stream: a neutral clock (empty `signs` at 2 Hz) drives the speed engine.
- Metric: time share of disagreement with the 1 Hz baseline (phase 0) on a 0.5 s grid, label changes matched
  with their delay; a schedule is run at every phase its period allows (2 s: 4 phases per ride, 10 s: 20).
  `tools/envd/rate_report.py` makes the tables.
- **Noise floor**: (a) the 1 Hz baseline at frame offset 10 (phase 1, independent frames); (b) "phone jitter":
  1 Hz with the intervals drawn from the 8668 recorded env intervals of the car (4 seeds).
- The smoother window is already in seconds of frame time (`window_s`, samples older than 10 s dropped), so a
  longer period simply leaves fewer samples in it (5 at 2 s, 3 at 5 s, 2 at 10 s). Also tested: window 3 x
  period (15 s at 5 s, 30 s at 10 s): worse everywhere (more lag), not proposed.

## 2. Results, rides (21 rides, 246 min; disagreement in % of time)
| candidate | inf/s | place | weather | light | area prior | car tunnel rule | wet | long area flips (>5 % of a ride) |
|---|---|---|---|---|---|---|---|---|
| noise: 1 Hz, offset 10 frames | 1.00 | 3.05 | 1.88 | 4.04 | 0.17 | 0.36 | 1.14 | 0 / 21 runs |
| noise: 1 Hz, phone jitter | 1.00 | 1.57 | 1.27 | 2.00 | 0.37 | 0.22 | 0.82 | 1 / 84 |
| **fixed 2 s** | 0.50 | 3.18 | 2.28 | 3.84 | 0.72 | 0.33 | 1.48 | 2 / 84 |
| fixed 3 s | 0.33 | 4.04 | 2.94 | 4.80 | 1.47 | 0.46 | 1.93 | 6 / 126 |
| fixed 5 s | 0.20 | 5.98 | 4.06 | 6.31 | 2.70 | 0.73 | 2.61 | 14 / 210 |
| fixed 10 s | 0.10 | 9.60 | 6.11 | 8.90 | 4.07 | 1.36 | 3.97 | 47 / 420 |
| event E1 (5 s + trigger dy20/dh.35/dc6, burst 10 s) | 0.38 | 5.37 | 3.46 | 5.90 | 2.00 | 0.51 | 2.43 | 2 / 21 |
| event E3 (5 s + trigger dy30/dh.5/dc10, burst 6 s) | 0.25 | 5.92 | 3.55 | 5.16 | 1.80 | 0.76 | 2.52 | 2 / 21 |
| event E5 (3 s + trigger, burst 6 s) | 0.37 | 4.13 | 2.56 | 3.82 | 1.70 | 0.69 | 1.91 | 1 / 21 |

Label-change delays (median / p90 / worst s; full tables `<datasets>/envd-cache/l48/report_rides.md`):
place: noise 0.5/2.5/56.5, 2 s 0.5/3.5/57.5, 3 s 0.5/5.0/58.5, 5 s 2.5/13.5/59.5. **Tunnel entry** (place):
noise 0.5/0.5/1.5, 2 s 1.0/2.0/7.0, 5 s 2.0/5.5/30, E3 -1.2/2.8/3.5, E5 -1.0/2.7/12. Car tunnel rule entry:
2 s 0.5/2.0/7.0, E1 -1.5/0.8/2.0 (the trigger makes it *earlier* than 1 Hz). Area prior: 2 s every change
matched, 0 extra, delay 0.0/2.0/5.0; its excess disagreement comes from 2 of 84 runs where a borderline
"outside" vote (s1-roadworks at the start, s7-elblag) held its 20 s with fewer env messages and the area was
set minutes earlier than at 1 Hz (fewer messages = fewer chances to break the 20 s run). 1 Hz with the phone's
own jitter does the same once in 84 runs: the prior is fragile on borderline votes by itself.

The trigger thresholds were declared before scoring (E1-E5); none was tuned on the results. Triggers per
15-min ride: 0-53 (E1), 0-21 (E2/E3); wipers, sun/shade and night glare fire them, so the event schedules spend
most of their inferences in bursts and do not beat a fixed 2-3 s period on the slow transitions.

## 3. Results, the car's recordings (6 drives, 70 min with frames)
| candidate | inf/s | place | weather | light | area prior | wet | fused place | fused built_up | fused limit | fused tunnel |
|---|---|---|---|---|---|---|---|---|---|---|
| fixed 2 s | 0.44 | 1.66 | 1.08 | 1.31 | 1.86 | 0.90 | 2.18 | 0.04 | 0.06 | 0.00 |
| fixed 3 s | 0.27 | 2.62 | 3.40 | 1.26 | 2.08 | 3.26 | 3.84 | 0.16 | 0.18 | 0.00 |
| fixed 5 s | 0.17 | 4.77 | 4.25 | 1.62 | 3.30 | 4.13 | 6.58 | 0.75 | 0.78 | 0.00 |
| fixed 10 s | 0.10 | 6.28 | 6.48 | 2.96 | 2.66 | 6.09 | 9.62 | 0.42 | 0.45 | 0.00 |

No phase-offset noise floor on the recordings (one 1 Hz phase only; no frames to re-run). The area-prior
excess at 2 s is one drive (20261119-153636: 14.2 % of the time, while the map was in use 95 % of it): there
the fused built_up / limit (what the MFD and the alerts read) follow the map and moved by 0.0 %.
No tunnel on these drives.

## 4. Candidate C: existence (recordings, env removed entirely, `existence` in the JSON)
| | map in use | no map |
|---|---|---|
| share of the drive time | 0-95 % per drive (113201 33 %, 151146 56 %, 153636 95 %) | the rest |
| fused built_up changes without env | **0.0 %** of the time | 0-99 % (env is the only source) |
| fused speed limit changes without env | **0.0-0.07 %** | 0-99 % (the vision default from the env area) |
| fused tunnel | unchanged on these drives (no tunnel; vision is OR-ed with the map) | unknown instead of false |
| fused place (display) | 16-42 % (map prior alone) | no place |

Which groups decide something (code reading, chris-integrator and the car computer):
- **place**: decides the area prior -> vision implicit limit and fused built_up **only without a map**; the
  tunnel label is OR-ed into fused tunnel (map or vision) and feeds the car's tunnel recirculation
  (actuation; vision path needs 3 s), with or without a map; otherwise display (MFD scenery, fused place).
- **weather**: decides the car's curve notice threshold (`wet` = rain / snow / fog); display.
- **light**: display only (MFD backdrop). No consumer decides on it.
- Observation (not a finding of this lane): on the car's November daytime drives light was `dusk_dawn` 85 %
  of the time and place `parking_lot` 57 % (one 50-min drive entirely): the labels on the DVR frames look
  doubtful; cannot be checked without frames.
- Speculation, not built: light from the sun elevation (GNSS position + clock) and frame luma would replace
  the only display-only group at zero cost; with a usable map the place prior is redundant for every decision
  except the tunnel OR, so a map-gated envd rate (e.g. one frame in 10 s while the map is in use and no
  tunnel ahead) would be the order-of-magnitude cut. It needs a consumer-side signal (governor `env_hz` cap set
  by the integrator/mapper when the map is usable): proposal for the integrator owners, not built here.

## 5. Candidate B: cost per inference
| candidate | laptop speed (3 thr, `parity20.nv12`) | quality vs fp32 (744 ride frames / 2000 BDD split-B images) | verdict |
|---|---|---|---|
| B1 ncnn/Vulkan on the GPU | - | laptop: `extract out0 failed` on the NVIDIA Vulkan device (fp16 and fp32); phone: hung the GPU (L3 row D) | **rejected** (hang; the GPU is ~84 % busy taking turns between road and signs) |
| B2 TinyCLIP-40M (L2 tables) | 3x cheaper (L2), phone 434 vs 812 ms | zero-shot macro -4 points; no probe for it exists | **rejected** as measured (would need a probe + this after-consumer test) |
| B3a ORT int8 dynamic (MatMul/Gemm) | 63 vs 88 ms (1.4x) | cos mean 0.987 / min 0.936; frame labels agree 93-95 %; BDD macro weather 79.0->78.4, light 85.1->83.3, place 74.0->72.7 | **rejected** (measured loss) |
| B3b ORT int8 dynamic per-channel | 61 ms | broken (cos 0.29-0.34) | rejected (quantizer output wrong; not pursued) |
| B3c fp16 weights (ORT, keep fp32 I/O) | 117 ms on x86 (no fp16 kernels: slower) | cos >= 0.99999; labels 100 / 100 / 99.7 % (rides), BDD macro equal within 0.1 | **same quality; speed on the A55 unknown** -> phone order row |
| B4 prep / scoring | 17-18 ms of 656-1138 | - | nothing to gain |

## 6. Recommendation
1. **`--period 2 --republish 1`** (envd every 2 s, last message re-sent every second so the MFD's 5 s age
   rule and the speed engine's per-arrival evaluation see a 1 Hz stream): **-50 % of envd's CPU** (with the
   measured 70-195 % of a core: 35-100 % of a little core freed; inferred, not measured on the phone).
   Quality: within the phase-offset noise floor for place, light, tunnel (entry p90 2.0 s vs 0.5 s; worst 7 s)
   and, on the car recordings, for every map-backed decision (fused built_up / limit 0.04-0.06 %);
   above it for weather / wet (+0.3-0.4 points, delays p90 2.5 s) and for the env area prior (0.72 % vs
   0.17-0.37 %, driven by 2/84 vs 1/84 borderline 20-s votes: not statistically separable, not shown equal).
   So: **not a clean pass**; it is the only rate cut whose excess is of the order of the noise floor. The phone
   order decides it with the installed configuration side by side.
2. `--trigger` (scene-change trigger, implemented, parity with the study's detector exact): useful only for
   the tunnel (car tunnel rule entry earlier than at 1 Hz in the rides) but fires on wipers / glare; with
   period 3 s (E5) it costs 0.37 inf/s for quality close to fixed 2 s. Not recommended as the default; kept
   as an option for a map-gated low-rate mode.
3. fp16 tower: same scores; measure on the phone (order row C). If ORT's ARM fp16 path is faster on the A55
   it stacks with 1.
4. The order-of-magnitude cut is not available inside envd at the same quality (10 s: place 9.6 % vs 3.05 %,
   area 4.1 % vs 0.17-0.37 %). It is available consumer-side: with a usable map, env decides nothing but the
   tunnel OR and the curve "wet" (section 4) -> map-gated rate (proposal).

## 7. Implemented (envd, defaults unchanged)
- `envd/main.cc`: `--period S` / `period` (one frame every S s; overrides `--max-hz`; the governor cap still
  applies), `--republish S` / `republish` (re-send the last line unchanged every S s; not while paused by the
  governor, not when the last frame is older than 2.5 periods), `--trigger` (+ `--look-hz`, `--trigger-luma`,
  `--trigger-hist`, `--trigger-chroma`, `--burst`, `--burst-period`; config keys likewise). Without the new
  options every branch is skipped: the loop, the rate (`max_hz`) and the messages are as before; `pipeline`,
  `smoother`, `output` and `envd_replay` are untouched, so `envd_replay` output is the same by construction.
- `envd/trigger.h` (thumbnail, change score, schedule), `envd/trigger_test.cc` (unit checks pass;
  `--nv12` dumps thumbnails: **0 of 100 ride frames differ** from `rate_sim.py`'s thumbnails).
- Daemon smoke test (laptop, `tools/ring_replay.py` with 60 s of tunnels-s3 around the first portal):
  default: 57 frames in 56 s, `tunnel` from frame 531; `--period 5 --republish 1 --trigger`: 34 frames,
  59 messages (23 re-sent), trigger at the portal (frame 415), `tunnel` from frame 496.
- Tools: `tools/envd/rate_extract.py`, `rate_sim.py`, `rate_report.py`, `quant_eval.py`.
- `config/envd.json` and the unit are **not** changed (the phone order decides).

## 8. Unverified / not finished
- Phone: nothing measured on the phone; CPU savings are inferred from the rate.
- Recording simulation of the two longest drives (20261119-104128, -135622, 91 min) stopped unfinished.
- fp16 speed on the A55; int8 on ARM (rejected on quality anyway).
- The rides' area prior runs without real signs / road class (neutral clock); the recordings have no tunnel and
  no phase-offset noise floor.
- The car's label quality on its own frames (section 4 observation).
