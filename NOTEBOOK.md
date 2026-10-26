# roadsense notebook

## Fold 0 (2026-10-23): baseline
Facts (measured on the phone, DVR camera behind the hub, see docs/PHONE_DVR_DEPLOY.md):
1. camerad-dvr 19.5 fps into the ring, ~3 % of a core.
2. modeld on the Adreno 630 (ncnn/Vulkan, fp16-packed, `TU_DEBUG=flushall`): 71-79 ms per run,
   9.3-11.5 runs/s (12/s at its best on 2026-10-21), die 63-70 C without signd.
3. signd on 4 big cores (ONNX Runtime, MTSD YOLOv8n 384x640 + 64x64 classifier): 170-200 ms,
   4.8-5.6 fps; drives the die to a ~95 C plateau without a fan, ~86 C with the user's
   improved cooling (23:13 CEST).
Open question that opens L1: can signs move to the GPU, or is the GPU already saturated by modeld?

## Lanes
| Lane | Model | State | Report |
|---|---|---|---|
| L1 signs-vulkan | Fable | done (2026-10-24 ~01:00 CEST) | docs/lanes/L1-signs-vulkan.md |
| L1-soak | Opus | superseded by L3 (speed ruling) | orders/L1-soak.md |
| L2 envd (environment/weather analyser, 1 fps) | Fable (user ruling 2026-10-26) | done (2026-10-26 ~18:20 CET) | docs/lanes/L2-envd.md |
| L2-envd-phone | Opus | folded into L3 | orders/L2-envd-phone.md |
| L3 speed-tune + envd on phone | Opus | done (2026-10-26 21:40 CET) | docs/lanes/L3-speed.md |
| L4 GPU turn-taking (signd/modeld) | Opus | running (2026-10-26 22:15 CET) | docs/lanes/L4-gpu-sched.md |

## Fold 1 (2026-10-24): signs on the GPU (L1, Fable)
- [FAV] signd's detector runs on the Adreno 630 (ncnn/Vulkan) next to modeld: 5.0 signs/s at
  9.7-9.9 road/s with the die at 67 C (shipped unit, `--max-hz 5`), vs 5.6 signs/s at 94 C and
  rising on the big cores (ORT). Uncapped: 8.7 signs/s and 8.7 road/s at 72 C.
- [FAV] Accuracy: all 21 detections on 3 Polish photos + 60 Ljubljana frames identical to ORT fp32
  (boxes within 2 px, scores within 0.01); deterministic with `TU_DEBUG=flushall`.
- [UNFAV] The classifier is slower on the GPU (10-15 ms/crop vs 3 ms on CPU), so it stays on ORT;
  ncnn computes the YOLOv8 head's final Concat wrongly (worked around with separate outputs).
- [INSTR] GPU headroom measured from the devfreq tracepoint: modeld alone uses 56 % of the GPU
  (~48 ms GPU of each ~76 ms run); capping modeld to 10 Hz frees little (devfreq drops the clock).
- [INSTR] Side finding: modeld ran 14-15 runs/s when the little cores were busy, because devfreq held
  the GPU at 710 MHz; a GPU min-frequency pin may give modeld about +20 %.
Facts: 4. signd det-ncnn + cls-ORT, 2 threads, cpus 4-7: numbers above (lane report, 70 s runs).
5. modeld GPU busy 56 % uncapped, 51 % at --max-hz 10.
Changes: roadsense-signs c16d572 (ncnn runner, --max-hz, conversion tools, unit on the GPU split);
roadsense 753d2ae (lane report, soak order). Phone: signd binary, ncnn models and unit reinstalled,
signd left stopped.
Roadmap: next L1-soak (Opus, 15-min plateaus of the shipped and uncapped configs, does modeld's
80 C throttle ever fire); candidate L2 = GPU min-frequency pin for modeld; then mount/calibration.

## Fold 2 (2026-10-26): environment analyser envd (L2, Fable)
- [FAV] envd works with zero-shot CLIP ViT-B/32 (OpenAI weights, MIT) + a linear probe: held-out macro
  recall weather 78.9 % (BDD), light 86.2 %, place 74.0 % (BDD) / 73 % (Places365 proxy incl.
  underground parking 98 %, forest road 83 %). Smoothed labels: 0-2 extra flips at 20 % wrong frames,
  5-6 s to follow a real change.
- [UNFAV] Zero-shot alone is not enough (BDD weather macro 56.9 %, place 58.3 %; highway 35 %,
  overcast 35 %, snow 30 %); fog is barely testable on BDD (13 images; ~10 %). The probe is trained on
  research-licensed data (BDD100K, DAWN CC BY-NC, Places365), so it is not committed; the repo ships
  the zero-shot class file.
- [INSTR] Encoder choice by pre-registered zero-shot rule: ViT-B/32 openai 71.1 > SigLIP2 68.1 >
  TinyCLIP-40M 67.0 > ViT-B/32 laion2b 65.4; MobileCLIP excluded (Apple research-only). C++ path
  bit-exact to Python (cos 1.00000), smoother identical on 360/360 frames. Laptop cost 89 ms/frame at
  2 threads. ncnn conversion done but unverified (x86 ncnn wheel segfaults), so ORT on the little cores
  is the default.
Deviations: two disclosed in the lane report (anchored probe variant; extra datasets split A/B after a
BDD-only probe could not learn the Places-only classes). The pre-registration section was first
committed together with results (2c03711), so its precedence is the lane's statement, not a git fact.
Not measured: phone cost (the user's stress test ran the whole lane); order L2-envd-phone.
Changes: roadsense 2c03711..460b5ef (envd/, tools, api env topic + /v1/env, unit, docs; 11 api tests pass).
Roadmap: L2-envd-phone (Opus: build, install with the probe class file, cost rows, 30-min run) once
the stress test ends; L1-soak still open; candidates: GPU min-clock pin, boost auto-recovery, mount
and calibration, in-car recordings as own training data for envd.

## Fold 3 (2026-10-26): speed profile + envd live on the phone (L3, Opus)
- [FAV] Road rate 9.6 -> 13.2 runs/s (15-min run, 13.0-13.3 in every window), model 95 -> 74 ms,
  frame-to-client latency 185 -> 144 ms median; signs unchanged at 5.0/s (detect 107 -> 81 ms);
  envd live at 1 Hz, 323 ms per frame on the big cores. Die 61-64 C after the first 4 min (max 82 C).
  Verified live 21:39 CET: road 12.7/s, signs 4.6/s, env connected.
- [FAV] The lever was CPU clocks, not the GPU: modeld's main thread on the big cores, little and big
  cores pinned at max (big-core 2.09 GHz cap removed); GPU min-freq pin and throttle-temp had no
  measurable effect (kept as guard / raised to 95 C).
- [UNFAV] The phone hard-reset by itself twice under high load (20:14 and ~21:03 CET; also 16:38 in the
  user's stress test). No oops, no thermal event, temps < 66 C, boost healthy afterwards; suspected
  supply brown-out (battery eliminator + VBUS boost feeding hub and camera; peaks >= 2.8 A with the
  speed profile). Not proven. Fallback: drop the big-core pin (12.6 road/s).
- [UNFAV] envd on ncnn/Vulkan hung the GPU (kernel hangcheck), leaving modeld failing every run until
  restart; modeld now exits after 20 consecutive failures (built, not yet exercised by a real hang).
  envd at 2 Hz heats 60 -> 88 C for no gain.
- [INSTR] Signs cost ~3.8 road runs/s (16.3/s without signd): modeld runs take ~105 ms when they overlap
  a detector run on the GPU (bimodal). Per-stage road latency: camera ~38 ms, prep 11 ms, model 56 ms
  median, ~20 ms queue/JSON, api +4 ms.
- [INSTR] envd live on a dark bench: weather clear, light night; place flips between underground
  parking and gas station (known probe confusion, conf ~0.4-0.6).
Changes: roadsense 0e531ef (units, envd.json, modeld lost-device exit, report), roadsense-signs
cb9087e (comment). Phone: speed profile installed and running; autostart disabled; old modeld and api
kept as backups.
Roadmap / candidates: (a) supply check or soak for the resets (user hardware: eliminator rating,
wire gauge, powered hub); (b) code lane: interleave/serialise signd and modeld on the GPU to win back
part of the 3.8 road/s; (c) envd place smoothing or a gas-station/underground-parking disambiguation;
(d) ledlat glass-to-API; (e) boost auto-recovery; (f) mount + calibration; (g) own recordings.

### Fold 3 correction (2026-10-26, user)
The resets at 16:38, 20:14 and ~21:03 CET were caused by the user's own hardware testing, not by the
stack or a supply brown-out. The [UNFAV] reset item of fold 3 is withdrawn; roadmap item (a) is dropped;
the big-core pin stays.
