# L1 signs-vulkan: can signd run on the Adreno next to modeld?

Lane opened 2026-10-23 23:50 CEST (Fable). Question: can the sign stack (roadsense-signs `signd`,
MTSD YOLOv8n 384x640 detector + 400-class 64x64 classifier, ONNX Runtime CPU today) run on the GPU
through ncnn/Vulkan like modeld, or is the GPU already saturated by modeld?

Phone: Poco F1 (SD845, Adreno 630, turnip, no VK_KHR_16bit_storage), stack as deployed:
camerad-dvr (USB DVR camera, 19.5 fps) + modeld (`--backend ncnn --cpus 0-3 --prep-cpus 4-7
--max-hz 20 --throttle-temp 80000`) + api; signd stopped by the user (heat). Times CEST.

## 1. GPU headroom with modeld as deployed

Method: the kernel tracepoint `devfreq:devfreq_monitor` for `5000000.gpu` logs, per devfreq poll
(50 ms), the GPU frequency and `load` = 100 x busy/total cycles of that poll (msm/a6xx GMU power
counter). Script `~/l1/phone/gpubusy.sh SECONDS LABEL` on the phone (root), copy in this lane's
scratch; it enables the event, sleeps, and averages the polls. Honest figure = busy % of wall time;
polling stops while the GPU is idle/suspended, so when polled time < wall the GPU was idle for the
difference. The load figure is only valid while the GPU is continuously active (see the 5 Hz row).

| run (2026-10-23 23:5x) | modeld rate / model_ms | polls covering wall | GPU busy (of wall) | mean GPU clock |
|---|---|---|---|---|
| modeld as deployed (max-hz 20), 30 s | 11.8 runs/s, 76 ms | 97 % | **56.4 %** (mean poll load 57.9 %) | 682 MHz, 78 % of polls at 710 |
| modeld `--max-hz 10`, 30 s | 9.5 runs/s, 83 ms | 97 % | **51.1 %** | 634 MHz, 66 % at 710 |
| modeld `--max-hz 5`, 30 s | 4.9 runs/s, 100 ms | 56 % | (load metric invalid: GPU idles between runs; devfreq reports >100 % per poll after idle) | 524 MHz |
| modeld as deployed again, 20 s | 11.5 runs/s, 78 ms | 97 % | 56.3 % | 675 MHz |

Commands: `sudo sh ~/l1/phone/gpubusy.sh 30 modeld-deployed`; capped runs via
`~/l1/phone/modeld_capped.sh HZ 45` (stops the unit, runs `/usr/local/bin/modeld` by hand with the
unit's arguments and `--max-hz HZ`, measures, restarts the unit).

Reading: at 11.8 runs/s the Adreno is busy 56 % of the time, i.e. about **48 ms of GPU per 76 ms
run**; the other ~28 ms of `model_ms` are CPU-side (ncnn's five CPU layers, upload/download,
`TU_DEBUG=flushall` cache flushes, fence waits). Roughly **44 % of the GPU is free** with modeld
uncapped, 49 % at a 10 Hz cap. Capping to 10 Hz frees only 5 % more GPU because the devfreq
governor then lowers the clock (634 MHz) and each run takes longer (83 ms); the freed GPU time is
real but small. The GPU clock is not pinned; a second GPU client would push it back to 710 MHz.

## 2. Detector + classifier on ncnn/Vulkan

### Conversion (laptop, `.venv-models` of roadsense: onnx 1.23.2, onnxsim 0.8.1, pnnx 20261020)

`onnxsim` -> `pnnx inputshape=[1,3,384,640] fp16=0` (detector) / `[1,3,64,64]` (classifier).
Both graphs convert without unsupported layers; every layer type (Convolution, Swish, Split,
BinaryOp, Concat, Slice, Reshape, Sigmoid, Pooling, MemoryData, Interp, Softmax, Permute,
InnerProduct) has a Vulkan kernel, so unlike supercombo nothing falls back to the CPU.
Detector: 213 layers, 4.9 GFLOPs; classifier: 88 layers.

Findings on the laptop (ncnn 1.0.20261020 Python wheel, ORT 1.31 as reference, three Polish
photos from `~/pocopilot/signd-test` on the phone, letterbox as signd does it):

1. **Final Concat of the detector output is wrong in ncnn.** The YOLOv8 head ends with
   `Concat(boxes[4,5040], scores[1,5040], axis 0) -> out0 [5,5040]`; the ncnn output of that layer
   is garbage (integer-looking values, scores 1.0 at wrong anchors) on both CPU and Vulkan while
   its two inputs (`257` boxes, `236` scores) are correct (boxes within 7e-4 px of ORT in fp32). A
   stand-alone Concat of the same shapes works, so it is an interaction with the graph (not
   investigated further). Fix in `tools/ncnn/fix_ncnn.py` (roadsense-signs): drop `cat_16`, name
   the box blob `out0` and the score blob `out1`; the backend extracts both.
2. **fp16 range is safe.** Largest activation in the detector graph (CPU fp32 scan of all 258
   blobs on a real photo): 638 (the boxes, pixels), 159 elsewhere; classifier: 12.2. Nothing near
   fp16's 65504, so no supercombo-style rescaling is needed.
3. Laptop Vulkan fp16-packed (Intel UHD 630) vs ORT fp32, patched graph: detector max |dscore|
   0.0029, max |dbox| 2.2 px in 640-space (the raw `cxcywh` rows, including anchors with no sign);
   classifier top-1 identical, max |dp| 0.0125 (p 0.587 vs 0.599).
4. Null: the ncnn **CPU** path gives a wrong class branch on x86 (scores permuted relative to
   boxes; blob `180` = Reshape of the 1-channel class conv does not match its input). Not our
   target (the CPU reference stays ORT), recorded in case anyone tries ncnn CPU.

### Implementation (roadsense-signs, this lane)

`signd --backend ncnn` (and `--det-backend` / `--cls-backend` per model), mirroring modeld's
structure: `signd/runner.h` (ModelRunner interface, RunnerOptions), `signd/ort_runner.cc` (the
reference, unchanged behaviour), `signd/ncnn_runner.cc` (Vulkan, fp16-packed, TU_DEBUG=flushall
added unless `--ncnn-no-flushall`, both models share ncnn's GPU instance). `yolo.cc` only calls
`run(in, out)`. CMake `-DROADSENSE_NCNN=ON -DNCNN_DIR=...` as in roadsense. Conversion:
`tools/ncnn/make_ncnn.sh MODELS_DIR` (+ `fix_ncnn.py`), wired into `scripts/fetch-models.sh`
(skip with `NCNN=0`) and `scripts/install-phone.sh` (`NCNN_DIR=... `). Checks: `tools/ncnn/
compare_models.py` (laptop, ORT vs ncnn on photos), `tools/ncnn/compare_replay.py` (two
`signd_replay --raw` outputs). `signd_replay` gained `--backend...`, multi-frame raw NV12 input
(`--every N`, `--max-frames N`) and a per-frame timing summary.

Build on the phone (user-owned tree, the old `/var/tmp/roadsense-signs-build` is root-owned):
`cmake -S ~/roadsense-signs -B ~/l1/build-signs -DROADSENSE_DIR=~/roadsense -DROADSENSE_NCNN=ON
-DNCNN_DIR=~/vulkan-work/ncnn && make -j4` (00:14 CEST, clean, no warnings).

### Accuracy on the phone (turnip), ORT fp32 vs ncnn fp16-packed, 2026-10-24 00:1x CEST

Inputs: the three Polish photos (`~/pocopilot/signd-test/*.jpg`, 1920 px) and 60 frames of the
Ljubljana drive (`~/rs-replay/ljkr.nv12`, 1920x1080 NV12, every 2nd of the first 120). Command
(per backend): `signd_replay --models-dir ~/roadsense-signs/models --backend X --raw --reset-each
--no-timing PHOTOS` and `... --raw --size 1920x1080 --every 2 --max-frames 60 ljkr.nv12`, then
`tools/ncnn/compare_replay.py ort.jsonl ncnn.jsonl` (matching by IoU >= 0.5, frame pixels).
modeld was running on the GPU during all of these.

| input | ref dets | ncnn dets | matched / missed / extra | box err px mean / max | dscore mean / max | cls top-1 same | dcls_prob mean / max |
|---|---|---|---|---|---|---|---|
| 3 photos | 7 | 7 | 7 / 0 / 0 | 0.76 / 1.20 | 0.0010 / 0.0020 | 7/7 | 0.013 / 0.091 |
| ljkr 60 frames | 14 | 14 | 14 / 0 / 0 | 0.89 / 1.90 | 0.0053 / 0.0090 | 14/14 | 0.049 / 0.229 |

Verdict: the fp16-packed GPU path reproduces the ORT detections (same boxes within 2 px of 1920,
same scores within 0.01, same class on every box); the classifier probability moves by up to 0.23
on one small far sign in the drive (mean 0.05), which the `cls_gate` 0.6 vote can flip on
borderline crops but does not change the class.
Determinism: two ncnn runs (`ncnn` vs `ncnn2`) are bit-identical on both inputs with
`TU_DEBUG=flushall`. Without flushall (`--ncnn-no-flushall`) the 3 photos were also identical to
the flushall run and the detector took 61 instead of 90 ms; see section 3 for the 60-frame check
that decides whether this graph needs the turnip workaround at all.

### Latency, replay on the phone (median / p90 over frames; `signd_replay` timing summary)

| config (modeld running on the GPU at ~11.8/s) | input | pre ms | det ms | cls ms | total ms |
|---|---|---|---|---|---|
| ORT, 4 threads, cpus 4-7 (deployed) | photos | 12.7 / 13.4 | 148 / 153 | 8.2 / 12.6 (2-3 crops) | 174 / 174 |
| ncnn det + ncnn cls, cpus 0-3 | photos | 32.8 / 35.3 | 90 / 96 | 29.9 / 52.3 | 153 / 156 |
| ncnn, cpus 0-3, no flushall | photos | 32.1 / 34.4 | 61 / 65 | 21.2 / 59.0 | 115 / 156 |
| ORT, 4 threads, cpus 4-7 | ljkr | 12.6 / 12.7 | 168 / 172 | 0 / 5.7 | 181 / 188 |
| ncnn, cpus 0-3 | ljkr | 39.7 / 45.8 | 79 / 84 | 0 / 14.3 | 119 / 134 |

Observations: (1) the detector itself is 1.9-2.1x faster on the GPU than on four A75s, even with
modeld sharing the GPU; (2) the letterbox preprocessing (`pre`) costs 33-40 ms on the A55s vs
13 ms on the A75s at 1920x1080 (the DVR camera is 1280x720, so less live); (3) the classifier is
*slower* on the GPU: ~10-15 ms per 64x64 crop (dispatch + flushall overhead on a tiny net) vs
~3 ms per crop on ORT, which motivates the mixed configuration (detector ncnn, classifier ORT).

### flushall: still needed, cheap here

60-frame ljkr replays, `--ncnn-no-flushall`, twice, vs the flushall run (`compare_replay.py`):
run 1 bit-identical detections; run 2 one detection's score off by 0.006. So turnip's
synchronisation problem shows on this graph too, only much more mildly than on supercombo
(tiny score drift instead of garbage), and the flushall cost on these small nets is within noise
(det 77.5 vs 79.0 ms with modeld running; 61.3 vs 62.4 ms alone). Decision: keep flushall on
(default), as modeld does.

## 3. Phone measurements: alone, with modeld, live (2026-10-24 00:2x-00:4x CEST)

Replay (`signd_replay`, ljkr 1920x1080, 60 frames, median / p90 ms; `~/l1/phone/run_suite.sh`):

| config | modeld | pre | det | cls | total |
|---|---|---|---|---|---|
| ORT 4 thr cpus 4-7 | stopped | 11.1 / 11.5 | 147 / 150 | 0 / 5.5 | 158 / 164 |
| ncnn det+cls, cpus 0-3 | stopped | 38.9 / 43.3 | 62 / 91 | 0 / 8.5 | 107 / 133 |
| ncnn det+cls, no flushall, cpus 0-3 | stopped | 39.3 / 43.0 | 61 / 81 | 0 / 8.1 | 103 / 121 |
| ncnn det + ORT cls (2 thr), cpus 0-3 | stopped | 37.3 / 41.9 | 67 / 95 | 0 / 14.7 | 108 / 134 |
| ORT 4 thr cpus 4-7 | running | 12.6 / 12.7 | 168 / 172 | 0 / 5.7 | 181 / 188 |
| ncnn det+cls, cpus 0-3 | running | 39.7 / 45.8 | 79 / 84 | 0 / 14.3 | 119 / 134 |
| ncnn det + ORT cls (2 thr), cpus 0-3 | running | 38.9 / 39.7 | 80 / 85 | 0 / 14.6 | 119 / 133 |
| ncnn det+cls, cpus 4-7 | running | 23.8 / 25.6 | 95 / 96 | 0 / 6.7 | 119 / 126 |
| ORT 4 thr cpus 0-3 | running | 31.7 / 32.4 | 365 / 376 | 0 / 12.0 | 398 / 417 |

The GPU detector costs 62 ms alone and 79-95 ms with modeld sharing the GPU (+27-50 %); ORT on the
A75s costs 147 alone / 168 with modeld (its prep threads). Note the ORT-alone run heated the die
from 55 to 79 C in the 60 frames (10 s); the ncnn runs stayed at 52-55 C.

Live, 70 s each (`~/l1/phone/bench_live.sh NAME 70 ARGS`, L1 build of signd on the real ring at
1280x720 next to the deployed camerad-dvr + modeld + api; GPU busy from the devfreq tracepoint
over the last 60 s; temps = hottest zone / cpu7 / gpu-top before -> after; signd stats mean over
the 5 s windows; modeld rate from its journal lines in the window):

| signd config | signs fps | detect_ms | modeld runs/s (model_ms) | GPU busy | signd CPU | temps before -> after (C) |
|---|---|---|---|---|---|---|
| ORT 4 thr cpus 4-7 (the stopped unit's config) | 5.64 | 177 | 9.7 (78.5) | 51 % | 36 % (of 8 cores = ~3 big cores) | 58/52/59 -> **94/90/76** (still rising) |
| ncnn det+cls, 1 thr, cpus 0-3 | 8.14 | 122 | 8.1 (116) | 80 % | 4-5 % | 80/78/69 -> 71/60/72 (cooling from the ORT run) |
| ncnn det + ORT cls, 2 thr, cpus 0-3 | 8.15 | 122 | 8.4 (113) | 79 % | 2-5 % | 66/62/66 -> 73/60/72 |
| ncnn det+cls, 1 thr, cpus 4-7 | 8.74 | 114 | 8.7 (105) | 80 % | 2-3 % | 69/60/69 -> 75/61/75 |
| ORT 4 thr cpus 0-3 | 2.55 | 392 | 14.2 (65) | (metric invalid) | 34 % | 69/62/70 -> 74/67/75 |
| ORT 2 thr cpus 0-3 | 1.79 | 559 | 15.0 (63) | 67 % | 23 % | 75/65/74 -> 71/66/72 |

Readings:
1. The GPU is not saturated: modeld alone uses 56 %; with signd's detector on it the Adreno runs
   at 80 % busy and both get ~8-9 runs/s each (modeld 11.8 -> 8.1-8.7 runs/s, model_ms 76 -> 105-116;
   signd 8.1-8.7 fps). The sum of work is what the GPU delivers at 710 MHz: 8.7 x 48 ms (modeld)
   + 8.7 x ~45 ms (signd) = 0.81 s per s.
2. ORT signd on the big cores (the stopped unit) gives 5.6 fps and 9.7 road/s but drives the die
   to 94 C within 70 s on the bench (the reason the user stopped it); the GPU configurations hold
   71-75 C with the same airflow, with the big cores idle (cpu7 60 C).
3. ORT on the little cores is useless (2.5 fps at 4 threads, 1.8 at 2), and modeld's own rate
   rises to 14-15 runs/s when the little cores are busy: devfreq keeps the GPU at 710 MHz the whole
   time, whereas with the GPU briefly idle between modeld runs the clock dips (634-682 MHz mean in
   section 1). A GPU min-frequency pin would likely give modeld +20 % for free (not this lane's
   question; noted for the roadmap).
4. The classifier on the GPU brings nothing (10-15 ms per crop vs ~3 ms on a CPU thread); detector
   on ncnn + classifier on ORT is the better split, with equal fps and less GPU traffic.
5. Placement of signd's CPU side: cpus 4-7 (big, otherwise idle, 24 ms pre) gives 8.7 fps vs 8.1
   on cpus 0-3 (39 ms pre) at the same temperature; the A75s are cheap when only one thread works.

## 4. Duty cycle and recommendation (live, 70 s each, 00:4x-00:5x CEST, `~/l1/phone/run_suite2.sh`)

`--max-hz` added to signd for this (process at most X frames/s, newest frame each time).

| signd config | signs fps | detect_ms | modeld runs/s (model_ms) | GPU busy | temps before -> after (max/cpu7/gpu, C) |
|---|---|---|---|---|---|
| ncnn det + ORT cls, 2 thr, cpus 4-7 | 8.72 | 114 | 8.7 (105) | 80 % | 62/59/63 -> 72/61/72 |
| same, `--max-hz 6` | 5.98 | 98 | 8.7 (105) | 71 % | 72/61/70 -> 69/58/68 |
| ncnn det + ORT cls, 2 thr, cpus 0-3, `--max-hz 6` | 5.97 | 136 | 8.9 (104) | 73 % | 65/60/63 -> 70/58/70 |
| ncnn det + ORT cls, 2 thr, cpus 4-7, `--max-hz 4` | 3.99 | 98 | 9.6 (95) | 66 % | 67/59/66 -> 66/57/65 |
| (section 3) ORT 4 thr cpus 4-7, the old unit | 5.64 | 177 | 9.7 (78) | 51 % | 58/52/59 -> 94/90/76 |

modeld's rate does not come back linearly when signd is capped: at 6 signs/s modeld still runs
8.7/s (each 45 ms detector dispatch stalls whatever modeld run overlaps it), at 4 signs/s 9.6/s.
Interpolating, `--max-hz 5` should give ~5 signs/s and ~9.1 road/s at ~68 C.

### Answer
Yes, the sign stack runs on the GPU, and it pays: modeld leaves 44 % of the Adreno free, the
detector needs ~45 ms of it per frame, and both fit. Signs fps per degree:

| configuration | signs fps | road runs/s | die after 70 s |
|---|---|---|---|
| ORT on four A75s (unit until now) | 5.6 | 9.7 | 94 C and rising (unusable without the fan) |
| detector ncnn + classifier ORT, cpus 4-7, uncapped | 8.7 | 8.7 | 72 C |
| same, `--max-hz 5` (**recommended default**) | ~5 (4.0-6.0 measured at 4/6 Hz) | ~9.1 (9.6-8.7) | ~68 C |

Recommendation: ship `signd --det-backend ncnn --cls-backend ort --threads 2 --cpus 4-7 --max-hz 5`
(the unit file now says so): the same signs rate as the CPU path, the road rate kept at ~9/s, and
~25 C less die temperature; when the fan is on and signs matter more than the last road message,
drop `--max-hz` for 8.7/8.7. The classifier stays on the CPU (GPU is slower for 64x64 crops) and
both backends keep ORT as the exact reference. The 15-minute plateaus of these configurations go
to the execution lane: `orders/L1-soak.md`.

Side finding for modeld (not this lane): with the little cores busy, modeld ran 14-15 runs/s
instead of 11.8 because devfreq kept the GPU at 710 MHz; pinning `min_freq` of
`/sys/class/devfreq/5000000.gpu` would likely give modeld +20 % at no cost.

### Smoke test of the shipped unit (00:5x CEST, 45 s, `systemctl start roadsense-signd`, then stopped)
signd 4.99-5.03 fps, detect_ms 107-113 (det 87-94); modeld 9.7-9.9 runs/s (model_ms 92-94);
hottest zone 67 C, cpu7 57 C, gpu 66 C. Matches the interpolation (5 signs/s, >= 9 road/s, <70 C).

## 5. Phone state at hand-back
camerad-dvr + modeld + api running as found; roadsense-signd stopped as found, but its binary,
the *-ncnn models (`/var/lib/roadsense/models/mtsd_*-ncnn.{param,bin}`) and the unit file were
reinstalled from the L1 tree with `NCNN_DIR=/home/user/vulkan-work/ncnn BUILD=/var/tmp/roadsense-signs-build
scripts/install-phone.sh` (see below), so `systemctl start roadsense-signd` now starts the
recommended GPU configuration. Scratch: `~/l1/` (scripts, user build, replay outputs), `~/roadsense-signs`
synced to the committed tree.
