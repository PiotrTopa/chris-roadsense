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
- [x] mapd: EKF over GNSS + vehicle motion (yaw rate, wheel speeds or steering angle), dead reckoning with map feedback, states `no_source` / `good` / `dead_reckoning` / `unknown`; vision odometry as fallback motion source exists but is **off by default since 2026-11-15** (`"vision_odometry": true` in `mapd.json` once the camera is calibrated; vehicle motion over the position input is the normal source)
- [x] position input `reset` (2026-11-15): a feeder can make mapd forget the position; a change between a `synthetic` and a real GNSS source resets on its own; `map.input.synthetic` flags a position fed by synthetic input
- [x] map matching with heading and continuity, confidence, parallel-road protection
- [x] map context: road attributes, tagged or Polish statutory speed limit with its source, look-ahead along the most likely path (shape, curves, junctions, roundabouts, crossings, signals, tunnel entry / exit, built-up and limit changes)
- [x] api topic `fused` (speed limit, built-up, place, lanes, tunnel with provenance), `/v1/map`, `/v1/fused`, status page
- [x] `scripts/roadsense-maps` (fetch, build, info, verify, install); Poland pack built on the dev box (docs/MAPS.md, docs/lanes/L13-mapd.md)
- [ ] a real position source: no feeder exists yet (the car's head-unit GNSS and the Prius CAN motion are to be fed by the car backend); every number so far comes from `tools/synth_drive.py`
- [x] mapd on the phone (2026-11-15): deployed with `install-phone.sh` (`roadsense-mapd.service` in `roadsense.target`, started/stopped by the car backend with the rest of the stack); Poland pack (262 MB, OSM 2026-11-14) at `/var/lib/roadsense/maps/current.rsmap` -> `~user/roadsense-maps/poland-20261114.rsmap`, sha256 checked. Measured on the phone (little cores 0-2, Nice 10): idle 0.35 % of one core, 37 MB RSS; live synthetic drives (GNSS 1 Hz + motion 20 Hz) 8.2 % (S2 tunnel) / 9.6 % (Warsaw city) of one core, RSS max 46 / 52 MB, step 15 / 22 ms mean (EMA, max 40 ms), `map` message 2.9 / 5.6 kB, api +~2 % of one core with the new topics; tile load 10.6 ms cold / 4.4 ms warm (p50 of the 20 largest tiles + city centres, max 14 ms); offline replay results (on-route %, dead-reckoning error, tunnel announcement) identical to the laptop
- [ ] mapd step time on the phone is 20-30x the laptop's (laptop 0.7 ms) while one tile decode is only ~2.5x slower: probably low little-core clocks at an 8 % duty cycle (not verified); fine at 2 Hz, worth a look before raising `hz`
- [ ] vehicle constants (`wheelbase_m`, `steering_ratio`, `track_m`) are nominal Prius values, not calibrated; vision odometry speed depends on the camera calibration
- [ ] built-up area is inferred from landuse where no tag states it (D-42 / D-43 sign nodes are not used); turn restrictions, turn lanes and lane-level geometry are not in the pack
- [ ] the most likely path ignores the driver's intent (blinkers, a navigation route)
- [ ] the recorded rides carry no position (video only), so no ride replays the map layer; a ride with a GNSS track is needed for a camera + map replay
- [ ] pack updates are manual (re-run fetch / build / install)

## Lead vehicle (tracker in the api; built 2026-11-15, not deployed)
- [x] topic `lead` / `/v1/lead` / `fused.lead` (`api/roadsense_api/lead.py`, docs/lanes/L14-lead.md): Kalman track of `road.leads[0]` with the ego speed as input, a standstill hold (frozen distance while both stand), gating of inconsistent frames (cut-in, lead change, flicker between two vehicles), `departed` cue, uncertainty, the raw vision value alongside; `tools/lead_replay.py`
- [x] the car's own motion for it: mapd relays accepted vehicle `motion` as `ego` on `<rundir>/ego.sock`; without a feeder the tracker runs on vision alone
- [x] measured on replayed rides (laptop, ORT fp32, 20 fps and every 2nd frame) and on synthetic scenes with the measured noise; numbers in docs/lanes/L14-lead.md
- [ ] not verified on the car: no drive with the lead output and the CAN motion recorded together exists (the 2026-11-13 trip log has CAN but no roadsense messages); the ego path ran on synthetic scenes and with a stand-in wheel speed derived from the model's own speed
- [ ] the phone runs the ncnn fp16 backend at 10-14 Hz with skipped frames; the noise figures come from the fp32 backend on every frame
- [ ] distance is from the camera (`camera_to_front_m` unset): `gap_m` and a bumper-to-bumper time gap need the camera position; the DVR camera's focal / pitch / height are uncalibrated, which scales every distance
- [ ] `/preview` still draws the raw `leads[0]` box

## Stability of what a consumer draws (lane L15, 2026-11-16; docs/lanes/L15-stability.md)
- [x] topic `lanes` / `/v1/lanes` (`api/roadsense_api/lanes.py`): the four lane lines and two road edges as tracks, presence with hysteresis on a filtered probability, geometry filtered per point with the network's own uncertainty, coasting through short drop-outs, re-initialisation when the network relabels its lines; `fused.lanes` takes the ego lane from it; `tools/lanes_replay.py`
- [x] soft map prior for the two outer lines from the matched road's tagged lane count; neutral without a position source
- [x] ego traffic light over time (`EgoLightSelector` in `lights.py`): hysteresis on the current ego track, coasting tracks of the provider, a 1.2 s hold marked `held`; `tools/lights_ego_replay.py`
- [x] drive recorder (`recorder.py`, opt-in by `recorder.json`): messages and one camera JPEG a second per session, capped on disk; the first source of perception data from the car itself
- [ ] not verified on the car: every number comes from replayed ride videos (other cameras, fp32 at 20 fps); the first recorded drive is the check
- [ ] the map prior has only run on tests and with a constant synthetic lane count (no live GNSS fix on the car yet)
- [ ] the ego-motion compensation of the line filter is implemented and off (it made the tracks trail on the rides); worth a second look with a calibrated camera and wheel speed
- [ ] `/preview` still draws the raw lane lines
- [ ] traffic lights are found late: the detector needs a housing of about 25-30 px in its input, which is 20-30 m with the car's camera (docs/lanes/L15-stability.md); a detector with a better small-object recall, or a higher-resolution pass, is the open item

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
