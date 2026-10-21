# Roadmap

## Phase 0: feasibility (in progress)
- [x] libcamera capture with AE/AWB at 1080p30 (GPU debayer)
- [x] ONNX Runtime C++ available on the phone; supercombo v0.9.7 loads
- [x] supercombo latency on 4 big cores: 54 ms as fp32 (300 ms as fp16)
- [x] sign detector chosen (MTSD YOLOv8n + 400-class classifier), licences recorded, 114 ms cold / 145-180 ms sustained on big cores, ~320 ms on little cores (moved to roadsense-signs, 2026-10-21)

## Phase 1: pipeline on the bench (running on the reference phone since 2026-10-21, `systemctl start roadsense.target`)
- [x] camerad: capture, frame ring in shared memory, timestamps, JPEG preview
- [x] modeld: frame pairing, warp to model intrinsics, YUV 6-channel packing, supercombo run, output parsing into physical units, JSON publish
- [x] signd: detector + tracker (SORT-style), JSON publish (now in the separate AGPL-3.0 repo roadsense-signs; roadsense keeps the `signs` socket contract)
- [x] api: aiohttp server, /v1/road /v1/signs /v1/camera /v1/health, /ws stream, /preview MJPEG with overlays
- [x] systemd units, one-command install on the phone (`scripts/install-phone.sh`), nftables rule for :8080
- [x] replay mode (`modeld_replay`, and `signd_replay` in roadsense-signs; a whole-stack replay from a video is still open): run the whole pipeline from a recorded video instead of the camera

## Phase 2: in-car data gathering
- [ ] mount + intrinsics/extrinsics calibration (focal length from EXIF/sensor, pitch/yaw from the model's own calibration outputs like openpilot's calibrationd)
- [ ] rolling recorder (video + JSON) with ring buffer and USB storage
- [ ] sign track aggregation (one event per physical sign with best crop and GPS when available)
- [ ] thermal/battery management, charging strategy in host mode

## Later
- [ ] car control integration (separate project; this API is the input)
- [ ] GPU inference via ncnn/Vulkan if CPU fps is insufficient
- [ ] newer openpilot vision/policy split models

## Model choice note (2026-10-21)
supercombo v0.9.7 is used first: single model, 54 ms fp32 on the phone, outputs lane lines, road edges, leads, plan, pose directly (docs/SUPERCOMBO_SPEC.md). openpilot v0.9.8–v0.11.1 split it into `driving_vision.onnx` + `driving_policy.onnx` with a 5 Hz history, which would allow running every 4th frame (less CPU) at the cost of a new parser; current master merged them again. Revisit when CPU budget becomes the problem.

## Known limits after phase 1 (2026-10-21)
- Thermal: on the bench with no airflow the SoC reaches 90-95 C with modeld on the big cores; the unit runs modeld at `--max-hz 5 --throttle-temp 75000`, which gives about 2.5-5 road messages/s; signd (roadsense-signs) on the little cores gives 1.5-2 fps. In-car mounting needs airflow or a lower duty cycle. 20 Hz like openpilot is not reachable on this SoC on CPU.
- Camera orientation: no rotation is applied; decide at mounting time (camerad option to add).
- Exposure metadata from the soft ISP is unreliable (no sensor helper for IMX363); image quality itself is fine.
- Calibration is a static config file; auto pitch/yaw from the model's pose is phase 2.
- api and daemons run as root (phase 1); a system user with RemoveIPC=no comes later.
