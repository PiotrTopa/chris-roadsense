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
- [x] host control (2026-11-04): the stack runs on the single Prius car computer (same Poco F1 as the car backend) and is started/stopped by that backend on ACC through `roadsense.target`; `roadsense.slice` + `ROADSENSE_API_PORT` added for the shared machine (docs/ARCHITECTURE.md "Running under a host controller")

## Map layer (mapd + fusion; built 2026-11-15, verified on synthetic position only)
- [x] position input API v1 (`gnss`, `motion` over `<rundir>/position.sock` or `POST /v1/position`), reference client `tools/position_client.py`
- [x] mapd: EKF over GNSS + vehicle motion (yaw rate, wheel speeds or steering angle), vision odometry as fallback motion source, dead reckoning with map feedback, states `no_source` / `good` / `dead_reckoning` / `unknown`
- [x] map matching with heading and continuity, confidence, parallel-road protection
- [x] map context: road attributes, tagged or Polish statutory speed limit with its source, look-ahead along the most likely path (shape, curves, junctions, roundabouts, crossings, signals, tunnel entry / exit, built-up and limit changes)
- [x] api topic `fused` (speed limit, built-up, place, lanes, tunnel with provenance), `/v1/map`, `/v1/fused`, status page
- [x] `scripts/roadsense-maps` (fetch, build, info, verify, install); Poland pack built on the dev box (docs/MAPS.md, docs/lanes/L13-mapd.md)
- [ ] a real position source: no feeder exists yet (the car's head-unit GNSS and the Prius CAN motion are to be fed by the car backend); every number so far comes from `tools/synth_drive.py`
- [ ] mapd on the phone: not deployed, CPU and memory there not measured; the pack is not installed on the phone
- [ ] vehicle constants (`wheelbase_m`, `steering_ratio`, `track_m`) are nominal Prius values, not calibrated; vision odometry speed depends on the camera calibration
- [ ] built-up area is inferred from landuse where no tag states it (D-42 / D-43 sign nodes are not used); turn restrictions, turn lanes and lane-level geometry are not in the pack
- [ ] the most likely path ignores the driver's intent (blinkers, a navigation route)
- [ ] the recorded rides carry no position (video only), so no ride replays the map layer; a ride with a GNSS track is needed for a camera + map replay
- [ ] pack updates are manual (re-run fetch / build / install)

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
- int8 quantisation tried and rejected (2026-10-21, docs/QUANTIZATION.md): the FastViT backbone's heavy-tailed activations make every int8 variant fail the lane-accuracy gate, and on ARM the QDQ overhead eats the gain anyway. fp32 stays. (The 2.09 GHz big-core cap that came out of it was removed on 2026-10-26 under the speed-first ruling: modeld runs on the GPU and the fan holds the die near 60 C; docs/lanes/L3-speed.md.)

## GPU backend decision (2026-10-21)
The ncnn/Vulkan fp16-packed backend misses the original accuracy gate only at the far end of the lane lines (x = 192 m: max 0.28 m) and for leads beyond 50 m (up to 1.9 m, i.e. about 3 %). Within 100 m the lane error stays under 0.11 m and the error does not accumulate through the recurrent state. For data gathering that is acceptable, and the GPU path runs at 60-90 ms per run on about half a little core instead of four big cores at 90 C, so the systemd unit now uses `--backend ncnn --max-hz 20`. The ORT fp32 path stays as the exact reference (`--backend ort`, `tools/ncnn/compare_replay.py`). Open: the residual fp16 error sits in the temporal attention block; turnip needs `TU_DEBUG=flushall` (set by modeld) or results are randomly corrupted, which is a driver bug worth reporting upstream.
- Live GPU stack on the phone (2026-10-21 22:05 CEST, 2 min, root units, no heatsink, display disconnected): 12 road messages/s at 76 ms per run, averaged die 76 C (cpu7 72 C, GPU 73 C), modeld 8 % CPU, camerad 6 %, signd 29 % on the little cores, big cores idle. Compare the CPU path: 2.5-5/s at 87-94 C.
