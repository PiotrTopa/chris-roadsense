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
| Inference | ONNX Runtime 1.30 (Alpine `onnxruntime`, CPU EP, C++ API only: no Python bindings on musl). No ncnn/TFLite/PyTorch packages. ncnn+Vulkan on turnip is the only realistic GPU route and would have to be built from source. |

Thermal (measured 2026-10-21): modeld at 10 Hz on cpu4-7 reaches 90-95 C within a minute on the bench (no airflow) and the model slows from 56 to 110-150 ms; on cpu0-3 it runs at 3.5 Hz / 276 ms at 63-71 C. The sensor zones swing ~20 C between runs, so guards must average. Sustained 8-thread inference throttles. Measure `thermal_zone*/temp`
and prefer the 4 big cores for the model, small cores for capture/IO.

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
