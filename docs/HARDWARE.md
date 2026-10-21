# Hardware and platform notes (Poco F1 / postmarketOS)

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

Thermal: sustained 8-thread inference throttles. Measure `thermal_zone*/temp`
and prefer the 4 big cores for the model, small cores for capture/IO.

## Camera

| Item | Value |
|---|---|
| Sensor | Sony IMX363 rear, `imx363 16-0010`, 1920x1080 SRGGB10 packed (only mode exposed by the driver), through `qcom-camss` |
| libcamera | 0.7.2, pipeline `simple` + software ISP. Debayer runs on the GPU (EGL/GLES, `debayer_egl.cpp`), output XRGB8888 only (YUV420 requests are adjusted to XRGB8888). |
| AE/AWB | Work (the IPA warns "Failed to create camera sensor helper for imx363" but exposure/gain are still driven: observed ExposureTime 22-44 ms, AnalogueGain 480 indoors, balanced colour means). Tuning file `/usr/share/libcamera/ipa/simple/imx363.yaml` has black level, AWB and CCM tables. |
| Controls exposed | only Gamma, Contrast, Saturation. Manual exposure/gain via libcamera controls is NOT available; `v4l2-ctl -d /dev/v4l-subdev19 --set-ctrl exposure=..,analogue_gain=..` works but AE overrides it. |
| Throughput | 1920x1080 XRGB8888: 30 fps; 1280x720 / 640x360 (scaled by the ISP): 22.5 fps. Prefer capturing native 1080p and scaling in camerad. |
| Front camera | not in the device tree. |
| Python | `py3-libcamera` works (used for `tools/camtest.py`). No `cv2` module (Alpine `opencv` ships no Python bindings). |

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
