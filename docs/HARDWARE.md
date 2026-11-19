# Reference deployment: Poco F1 on postmarketOS

roadsense itself is device-agnostic (see the README); this file records the
platform it was developed and measured on.

Measured on 2026-10-21 on the bench phone (postmarketOS edge, systemd, phosh,
kernel 7.1.0-rc1-sdm845, Tianma kernel variant with the FocalTech touch patch,
USB in host mode with VBUS sourced by the phone).

## Compute

| Item | Value |
|---|---|
| SoC | Snapdragon 845: 4x Cortex-A75 @ 2.65 GHz (cpu4-7) + 4x Cortex-A55 (cpu0-3), schedutil |
| RAM | 5.5 GiB usable, 8 GiB zram swap |
| Storage | 109 GiB root, ~100 GiB free |
| GPU | Adreno 630, Mesa 26.2 freedreno (GLES 3.2) + turnip Vulkan ICD present |
| DSP/NPU | Hexagon remoteprocs boot (adsp/cdsp/slpi) but no userspace path for inference on mainline. Ignore. |
| supercombo v0.9.7 | ORT CPU, 4 threads pinned to the big cores: **fp16 graph as shipped: 300 ms**; converted to fp32 (`tools/onnx_fp16_to_fp32.py`): **54 ms** (18 fps). 8 threads is slower than 4 (little cores drag). Temperature after 10 s of inference: 64-74 C. |
| Inference | ONNX Runtime 1.30 (Alpine `onnxruntime`, CPU EP, C++ API only: no Python bindings on musl). No ncnn/TFLite/PyTorch packages. GPU: ncnn 20261020 + Vulkan built from source (`scripts/build-ncnn.sh`, ~45 min), `modeld --backend ncnn`, see below. |

Thermal (measured 2026-10-21): modeld at 10 Hz on cpu4-7 reaches 90-95 C within a minute on the bench (no airflow) and the model slows from 56 to 110-150 ms; on cpu0-3 it runs at 3.5 Hz / 276 ms at 63-71 C. The sensor zones swing ~20 C between runs, so guards must average. Sustained 8-thread inference throttles. Measure `thermal_zone*/temp`
and prefer the 4 big cores for the model, small cores for capture/IO.

### GPU inference: ncnn + Vulkan on turnip (measured 2026-10-21)

Driver facts: Mesa 26.2 turnip, "Turnip Adreno (TM) 630", Vulkan compute queue,
subgroup 64-128. No `VK_KHR_16bit_storage` (ncnn reports fp16 packed/storage/
arithmetic = 1/0/0), so ncnn's only fast mode is **fp16-packed**: fp16 storage
of blobs and weights, fp32 arithmetic. GPU fp32 everywhere is exact (max output
diff vs ORT 2e-4) but takes 320 ms. GPU clock moves between 257 and 710 MHz
(`/sys/class/devfreq/*gpu*`). Five small layers (two MatMul, ExpandDims,
Squeeze of the temporal part) have no Vulkan kernel and run on the CPU.

| supercombo v0.9.7, ncnn fp16-packed | Value |
|---|---|
| model latency (replay, phone otherwise busy with the root stack) | 58-64 ms per run (+~5 ms for `TU_DEBUG=flushall`), first run (pipeline build) 0.1-1.5 s |
| sustained, scbench alone | 16.8 runs/s for 60 s, ~15 % of one CPU core, SoC 76-80 C (CPU ORT at the same duty: 93-94 C) |
| live modeld on cpu0-3 next to the running root stack | see below |

Two problems had to be solved before the outputs were usable:

1. **Random corruption (driver/ncnn synchronisation).** With ncnn's defaults
   about half the runs on the same input return different, wrong outputs (lane
   errors from 0.1 m to hundreds of metres, no error code); the very first run
   after start is usually right, which is why a one-frame check passed.
   `TU_DEBUG=flushall` (turnip flushes and invalidates all GPU caches around each
   dispatch) makes every run bit-identical and correct at ~+5 ms. Not enough:
   `TU_DEBUG=syncdraw`, ncnn light mode off, packing layout off, subgroup ops
   off, shader local memory off, and a patch that forces a real barrier on
   every recycled blob buffer. The ncnn backend therefore adds `flushall` to
   `TU_DEBUG` before Vulkan starts (`--ncnn-no-flushall` to skip).
2. **fp16 overflow in the vision-feature L2 normalisation.** The sum of squares
   (ONNX `/summarizer/ReduceSum`, ncnn `sum_237`) reaches ~1.6e5 on real frames,
   above fp16's 65504; with fp16 storage it saturates and the normalised
   feature, which also feeds the recurrent `features_buffer`, comes out up to
   1.5x too small (lane lines off by up to 5 m on single frames).
   `tools/ncnn/fix_ncnn.py` scales one factor of the square by 2^-8 and the
   norm back by 2^4 (exact in fp32). The other out-of-range blobs (up to 1.3e7)
   are the cubic term of tanh-GELU, where saturation does not change the result.
   Per-layer fp32 through ncnn's `featmask` does not work on Vulkan in this
   version (outputs inf/garbage), so the remaining fp16 error cannot be fenced
   off layer by layer; an ORT emulation (fp16 rounding of every tensor) puts
   about half of it in the temporal attention block, spread over many ops.

Accuracy over the whole Ljubljana replay (119 runs, 1920x1080 NV12, both
backends keeping their own recurrent state, `tools/ncnn/compare_replay.py`),
ncnn fp16-packed + flushall + fix 2 against ORT fp32:

| metric (per run, then over the replay) | mean | p95 | max | gate |
|---|---|---|---|---|
| lane-line y, mean abs over 4 lines x 33 points (m) | 0.018 | 0.039 | 0.055 | mean < 0.05: pass |
| lane-line y, max abs (m) | 0.078 | 0.171 | 0.276 | max < 0.2: **fail** (only at x = 192 m; <= 0.062 m up to 50 m, <= 0.104 m up to 100 m) |
| lane-line prob | 0.006 | 0.014 | 0.041 | < 0.05: pass |
| lead 0 x at t=0 (m) | 0.26 | 0.77 | 1.86 | < 1 m: **fail** (leads at 50-105 m, plus seq 120) |
| lead 0 v at t=0 (m/s) | 0.12 | 0.26 | 5.54 | (one run, seq 120, prob 0.96: 27.6 vs 33.2) |
| lead 0 prob | 0.002 | 0.008 | 0.025 | |
| plan x/y, max abs over 33 points (m) | 1.05 | 3.01 | 9.0 | |

No drift: the first and last thirds of the replay have the same error. Running
the temporal part and heads on the CPU in fp32 (featmask CPU) only reduces the
mean lane error to 0.014 m with the same outliers, at 500+ ms per run. ORT on
the fp16-weight graph as shipped (weights fp16, compute fp32) stays within
0.009 m on the same frames, so the residual is ncnn's fp16 activation storage.
Because the gate is not met, the shipped unit keeps `--backend ort`.

Live, 120 s (2026-10-21 ~21:55 CEST), own modeld `--backend ncnn --max-hz 20`
(default cpus 0-3, 1 CPU thread) reading the system ring next to the running
root stack (camerad, ORT modeld at 5 Hz on cpu4-7): the phone was already at
88 C (hottest zone), so the thermal guard halved the rate at once; 9.6 road
messages/s (1154 in 120 s), model_ms 79-94 ms average (max 138; prep 18-21 ms on
the A55s), the process used 57 % of one A55 (prep + dispatch + CPU layers), GPU
zones 65 -> 75-85 C, CPU zones 85 -> 92-95 C, GPU clock mostly 710 MHz. These
temperatures are dominated by the CPU stack running alongside; a GPU-only stack
was not measured live.

## Budget of the running stack (measured 2026-11-18, lane L21)

Bench, ride replay, fan 100 %, all eight cores online, rail power from `qcom-battery` (voltage x current);
method and every row in [docs/lanes/L21-budget-and-models.md](lanes/L21-budget-and-models.md)
(`tools/budget/`).

| | rail W | big cores busy / C | GPU busy / C |
|---|---|---|---|
| idle (no pins) | 1.06 | - / 29 | - |
| idle with the modeld unit's clock pins and CPU wake-up latency hold | 1.62 | - / 31 | - |
| modeld alone, 19 road runs a second | 4.23 | 20 % / 42 | 90 % / 50 |
| the stack as installed until 2026-11-18 (envd on the big cores, lights 3 Hz) | 6.67 | 146 % / 70 | 91 % / 57 |
| envd on the little cores | 6.19 | 99 % / 62 | 92 % / 58 |
| envd on the little cores, lights 5 Hz | 7.01 | 137 % / 68 | 91 % / 58 |

* A fully busy A75 is about 2 W, the Adreno at 90 % about 2.3 W. The A55s do the same work for roughly half
  the energy at a third of the speed.
* The GPU is the cool part and the full part: 47 ms per road run and 51 ms per sign-detector run fill 91 %
  of its time at 710 MHz. More GPU work means fewer road runs.
* The clock pins and the wake-up latency hold of the modeld unit cost 0.5 W together and buy 0.9 road runs a
  second and 30 ms of road latency.
* Per-process GPU time is in `/proc/<pid>/fdinfo/*` (`drm-engine-gpu`); the daemons name their threads
  (`md-prep`, `md-prefetch`, `md-main`, `sg-signs`, `sg-lights`), so `top -H` shows the stages.

### Budget with the new default road model (measured 2026-11-19, lane L27)

The default road model is openpilot v0.11.1 `driving_vision` (GPU) + `driving_policy` (CPU, 5 runs a second).
Same bench, same session, the same sign / light / scenery daemons (L21's proposed stack: envd on the little
cores, lights 5 Hz), candidates run from `~/l27`, nothing installed
([docs/lanes/L27-new-model-default.md](lanes/L27-new-model-default.md) section 6):

| | supercombo | vision + policy at 5 Hz (default) | vision + policy after every run |
|---|---|---|---|
| rail W | 6.83 | 6.59 | 6.73 |
| GPU busy (modeld's part) | 91 % (66 %) | 84 % (59 %) | 83 % (57 %) |
| big cores busy / C | 139 % / 71 | 137 % / 68 | 146 % / 70 |
| little cores busy; of it frame prep | 204 %; 20.7 % | 192 %; 11.8 % | 192 %; 11.7 % |
| model thread on a big core (GPU dispatch, parsing, policy) | 7.5 % | 8.6 % | 17.0 % |
| road runs a second; model step | 13.8; 54 ms | 15.2; 44 ms | 14.8; 50 ms |
| `road` latency camera -> consumer, median / p90 | 111 / 131 ms | 85 / 97 ms | 101 / 121 ms |

* Single process (`modeld_replay --backend ncnn`, clocks pinned): 46.8 ms per run against supercombo's 60.5 ms;
  frame prep 1.8-4.7 ms against 12.9 ms; the policy 7.9 ms per run on an A75 (18.0 ms on an A55).
* The GPU is no longer full in time (84 %): 7 points are free, and the road rate is now set by the turn-taking
  with the sign detector, not by the GPU.
* fp16 on the Adreno against ORT fp32 for this model: lines that are present are within 0.007 m (to 30 m) and
  0.019 m (30-100 m) on average, like supercombo's; the lead distance differs by 0.37 m on average (0.86 m p95,
  about 1 % of the distance), twice supercombo's 0.19 m. The strict script of the "GPU inference" section
  (maximum over all four lines to 192 m) rejects both models on the bench clip; see the lane report.
* Governor step `plan` (the policy at 2 runs a second): three policy runs a second less, about 0.7 % of a big
  core; it is in the order for completeness, not for its saving.

### Thermal headroom and the load governor (lane L24, 2026-11-18)

Limits of this phone as the kernel exposes them: CPU and cluster zones trip `passive` at 90 C (cpufreq
cooling starts to lower the clocks), `hot` at 95 C (where the hardware limiter holds the SoC), critical 110 C;
GPU zones 85 / 90 / 110 C. Cooling devices: `cpufreq-cpu0` (17 states), `cpufreq-cpu4` (23), the GPU's devfreq
(6). The governor's default probe reads exactly these (docs/ARCHITECTURE.md "Load governor"); nothing in
roadsense names the phone.

Path to the air, from the bench point "10.0 W at 94-95 C hottest zone in 22 C air, fan 100 %": about 7.3 K/W
(linear model, hottest zone, fan at 100 %, the air at the phone taken as the cabin air). The governor starts to
shed at 86 C (4 K under the passive trip, on a 4 s average), so the power that runs with nothing shed at an
air temperature Ta is (86 - Ta) / 7.3 W:

| air at the phone | 25 C | 30 C | 35 C | 40 C | 45 C | 50 C | 55 C |
|---|---|---|---|---|---|---|---|
| power with nothing shed, fan 100 % | 8.4 W | 7.7 W | 7.0 W | 6.3 W | 5.6 W | 4.9 W | 4.2 W |

Against the stack's budget (7.0 W as proposed by L21; the per-step savings are estimates from L21's
subtraction table until the on-phone runbook of docs/lanes/L24-quality-max.md has been run):

| governor level | stack W (estimate) | nothing more shed up to | road / lead / lanes |
|---|---|---|---|
| 0: everything | 7.0 | 35 C | full |
| 3: + scenery once in 5 s | 6.5 | 38 C | full |
| 6: + signs and lights at half rate | 5.7 | 44 C | full |
| 7: + scenery paused, detectors at a quarter | 5.1 | 49 C | full |
| 8: + the plan at 2 runs a second (new default road model, L27) | about 5.0 | 49 C | full; `road.plan` held up to 0.5 s |
| 9: + road model at 10 runs a second | 4.2 | 55 C | half rate, same latency |
| above | - | - | the platform limiter cuts clocks; the topics say `limited`, `slow`, `late` |

With a lower fan duty the path is longer and every temperature in the tables drops accordingly: the fan has to
be at 100 % before the governor sheds anything (what the host's fan policy must do: full duty from 80 C on the
hottest zone, or whenever `health.governor.level` > 0 or `health.governor.headroom_k` < 10).

## Camera

| Item | Value |
|---|---|
| Sensor | Sony IMX363 rear, `imx363 16-0010`, 1920x1080 SRGGB10 packed (only mode exposed by the driver), through `qcom-camss` |
| libcamera | 0.7.2, pipeline `simple` + software ISP. Debayer runs on the GPU (EGL/GLES, `debayer_egl.cpp`), output XRGB8888 only (YUV420 requests are adjusted to XRGB8888). |
| AE/AWB | Work (the IPA warns "Failed to create camera sensor helper for imx363" but exposure/gain are still driven: observed ExposureTime 22-44 ms, AnalogueGain 480 indoors, balanced colour means). Tuning file `/usr/share/libcamera/ipa/simple/imx363.yaml` has black level, AWB and CCM tables. |
| Controls exposed | only Gamma, Contrast, Saturation. Manual exposure/gain via libcamera controls is NOT available; `v4l2-ctl -d /dev/v4l-subdev19 --set-ctrl exposure=..,analogue_gain=..` works but AE overrides it. |
| Throughput | 1920x1080 XRGB8888: 30 fps (30.26 at libcamera's default vblank 112; lower rates via VBLANK, see below); 1280x720 / 640x360 (scaled by the ISP): 22.5 fps. Prefer capturing native 1080p and scaling in camerad. |
| Front camera | not in the device tree. |
| Python | `py3-libcamera` works (used for `tools/camtest.py`). No `cv2` module (Alpine `opencv` ships no Python bindings). |

### Sensor frame timing (VBLANK), measured 2026-10-21

libcamera's configure puts the IMX363 in a mode reporting hblank 4800 and
pixel_rate 508.8 MHz (before any libcamera use the subdev reports 6912 /
256.8 MHz) and sets vertical_blanking = 112. The nominal V4L2 formula
`(1080 + vblank) * (1920 + hblank) / pixel_rate` predicts 63 fps; the real rate
is 30.26 fps. vblank written after configure sticks through `start()` (libcamera
does not touch it again), and a sweep 112..4000 (fps from SensorTimestamps)
fits a straight line:

    T = (3045 + vblank) * 10.468 us     (vblank 112: 30.26 fps, 216: 29.35, 1000: 23.63, 2706: 16.60, 4000: 13.56)

i.e. the mode reads ~3045 lines (full sensor height) at 10.468 us per line.
camerad `--fps N` uses this model: 20 fps -> vblank 1731 (measured 20.01),
25 -> 776 (25.03), 15 -> 3324 (14.98), 10 -> 6508 (9.98). Writing vblank needs
only the `video` group (`/dev/v4l-subdev*` is root:video 0660).

ExposureTime metadata from the simple IPA is not trustworthy without a sensor
helper (it reported a constant 54262 us while the sensor's `exposure` control
was 3126 lines = 32.7 ms); AnalogueGain reports the raw code (480 indoors).

### camerad cost (1080p, measured 2026-10-21)

| Stage | Cortex-A55 (camerad default) | Cortex-A75 (taskset 4-7) |
|---|---|---|
| XRGB8888 -> NV12 into the ring (NEON + software prefetch) | 12-13 ms/frame | 7.7 ms/frame |
| same, from a cached malloc buffer (microbenchmark) | 6.6 ms | 2.5 ms |
| 2x downscale for preview | 5-6 ms | 1.6 ms |
| JPEG 960x540 q80, built-in encoder (in camerad / tight loop at full clock) | 28-30 ms / 12.8 ms | 18 ms / 6.3 ms |

Without software prefetch the in-order A55 needs ~40 ms for the conversion
(stalls on every miss), with GCC auto-vectorisation or hand NEON alike. The
software-ISP output buffers read slower than ordinary cached memory (7.7 vs
2.5 ms on the A75). In camerad the stages run at bursty load, so schedutil keeps
the clocks low; the "full clock" numbers are tight-loop benchmarks.

Whole camerad process (incl. libcamera's SWIspWorker, ~35 % of a core, which
drives the GPU debayer and computes AE/AWB statistics), all on cpu0-3: 92 % of
one A55 at 20 fps (worker 30 %, preview 20 %), 135 % at 30 fps. No
`sensor_drops` in any run.

Busybox `nc` has no `-U`: use a python3 socket one-liner (or install
`socat` / `netcat-openbsd`) to watch a socket on the phone.

Known artifact: ISP-scaled 720p output shows a fine Bayer-grid moiré
(nearest-neighbour scaling in the GPU debayer). Native 1080p + area downscale
in camerad avoids it.

## Power

With `poco-f1-usb-host` applied the phone sources VBUS on USB-C, so it cannot
be charged over that port while in host mode. For in-car use this needs either
a role switch (device mode) when parked, or a different charging path. Battery
reported Full on the bench with nokia1 also driving the port; do not rely on
that.

## Network / access

Wi-Fi only (`ssh user@192.168.0.86` on the home network). `ath10k` needs the
retry service on boot. In the car there is no network unless a hotspot or USB
Ethernet (host mode) is provided.
