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
- [ ] GPU inference via ncnn/Vulkan: integrated 2026-10-21 (`modeld --backend ncnn`, ~60 ms/run, ~15 % of a core). Two blockers solved (turnip cache flush, fp16 overflow in the feature normalisation); full-replay accuracy still misses the gate on far-range outliers (lane y max 0.28 m at 192 m, lead x 1.9 m at 100 m; HARDWARE.md), so the unit stays on ORT. Open: reduce the fp16 activation error (ncnn per-layer fp32 is broken on Vulkan) or relax the gate to the ranges that matter
- [ ] newer openpilot vision/policy split models

## Model choice note (2026-10-21)
supercombo v0.9.7 is used first: single model, 54 ms fp32 on the phone, outputs lane lines, road edges, leads, plan, pose directly (docs/SUPERCOMBO_SPEC.md). openpilot v0.9.8–v0.11.1 split it into `driving_vision.onnx` + `driving_policy.onnx` with a 5 Hz history, which would allow running every 4th frame (less CPU) at the cost of a new parser; current master merged them again. Revisit when CPU budget becomes the problem.

## Known limits after phase 1 (2026-10-21)
- Thermal: on the bench with no airflow the SoC reaches 90-95 C with modeld on the big cores; the unit runs modeld at `--max-hz 5 --throttle-temp 75000`, which gives about 2.5-5 road messages/s; signd (roadsense-signs) on the little cores gives 1.5-2 fps. In-car mounting needs airflow or a lower duty cycle. 20 Hz like openpilot is not reachable on this SoC on CPU.
- Camera orientation: no rotation is applied; decide at mounting time (camerad option to add).
- Exposure metadata from the soft ISP is unreliable (no sensor helper for IMX363); image quality itself is fine.
- Calibration is a static config file; auto pitch/yaw from the model's pose is phase 2.
- api and daemons run as root (phase 1); a system user with RemoveIPC=no comes later.
- int8 quantisation tried and rejected (2026-10-21, docs/QUANTIZATION.md): the FastViT backbone's heavy-tailed activations make every int8 variant fail the lane-accuracy gate, and on ARM the QDQ overhead eats the gain anyway. fp32 stays; big cores capped at 2.09 GHz in the modeld unit.

## GPU backend decision (2026-10-21)
The ncnn/Vulkan fp16-packed backend misses the original accuracy gate only at the far end of the lane lines (x = 192 m: max 0.28 m) and for leads beyond 50 m (up to 1.9 m, i.e. about 3 %). Within 100 m the lane error stays under 0.11 m and the error does not accumulate through the recurrent state. For data gathering that is acceptable, and the GPU path runs at 60-90 ms per run on about half a little core instead of four big cores at 90 C, so the systemd unit now uses `--backend ncnn --max-hz 20`. The ORT fp32 path stays as the exact reference (`--backend ort`, `tools/ncnn/compare_replay.py`). Open: the residual fp16 error sits in the temporal attention block; turnip needs `TU_DEBUG=flushall` (set by modeld) or results are randomly corrupted, which is a driver bug worth reporting upstream.
