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
| L3 speed-tune + envd on phone | Opus | running (2026-10-26) | docs/lanes/L3-speed.md |

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
