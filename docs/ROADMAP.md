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
  - [x] pitch / yaw online from the model's ego-motion (L32, `auto_calib`, docs/ARCHITECTURE.md Calibration); replayed rides and the car's recordings only, **not yet run on the car**
  - [ ] focal length, camera height, roll: still nominal
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
- [x] topic `lead` / `/v1/lead` / `fused.lead` (`api/roadsense_api/lead.py`, docs/lanes/L14-lead.md): Kalman track of `road.leads[0]` with the ego speed as input, a standstill hold (frozen distance while both stand; replaced by a live standing regime in 2.0, lane L19), gating of inconsistent frames (cut-in, lead change, flicker between two vehicles), `departed` cue, uncertainty, the raw vision value alongside; `tools/lead_replay.py`
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
- [x] traffic lights (roadsense-signs `signd --lights`, in its unit): a second, full-scale view around the horizon on every other light frame, weak boxes that continue a track, coasting reports; on the replays the reported share of the light-time went from 82-85 % to 94-96 % (lights under 12 px: 39-56 % to 81-83 %) and the ego light stopped blinking
- [ ] traffic lights on the car's own camera at night and in rain are unmeasured (no picture of one exists yet; the recorder collects them); if lights are still missed, the open item is a detector with a better recall on small lights or a full-resolution pass, which the phone at 83-89 C has no headroom for

## The car's own traffic light, junction view, start-up (lane L16, 2026-11-17; docs/lanes/L16-my-lights-junctions.md)
- [x] first recorded drive from the car (29 min, day, sun): lane-line flicker 82 on/off changes per minute raw -> 2.8 tracked, 2120 spans shorter than a second -> 0 (the L15 tracker, checked on the car's own data); ego light shown 18.3 % of the time, 3 short gaps
- [x] lane assignment of the ego light (`lights.py`): stop and go groups, slot of every head in lane widths from the tracked lane lines, the group over the ego lane wins, `assign` / `amb` / `alt` in the message; on the recorded drive 5 conflict spans (21 s), the right group shown in all of them, before and after
- [ ] the assignment has not changed an outcome yet: on this drive the lateral window alone was right too. It is the missed-ego-head case and wider junctions that need it, and no recording has one
- [ ] arrow signals are not recognised (no arrow state in the provider); lit lights are often reported `off` against the sun or far away (95 of 107 observations of one green light): the state classifier on the car's camera is the weak part now
- [x] `map.ahead.junction_view`: arms, control, roundabout ring and exit numbers from the pack graph; verified on a hand-made network and on synthetic positions in Opole
- [ ] the junction view has never run with a real position (no GNSS source on the car); lane counts per arm are tagged values only, no turn lanes
- [x] start-up delay explained: the DVR camera leaves the USB bus 23-39 s after power-up, for 4-5 s or for 122 s (2 of 5 drives, both the first after a long stand); not a GNSS wait
- [ ] why 122 s is not known (camera firmware; power, temperature and the head unit are candidates); the USB tree also re-enumerates once per drive at the camera port ("cannot reset, err -71"), 6 s without frames

## Later
- [ ] car control integration (separate project; this API is the input)
- [ ] GPU inference via ncnn/Vulkan: integrated 2026-10-21 (`modeld --backend ncnn`, ~60 ms/run, ~15 % of a core). Two blockers solved (turnip cache flush, fp16 overflow in the feature normalisation); full-replay accuracy still misses the gate on far-range outliers (lane y max 0.28 m at 192 m, lead x 1.9 m at 100 m; HARDWARE.md), so the unit stays on ORT. Open: reduce the fp16 activation error (ncnn per-layer fp32 is broken on Vulkan) or relax the gate to the ranges that matter
- [x] newer openpilot vision/policy split models (the default road model since lane L27, 2026-11-19; section at the end): surveyed 2026-11-18 (lane L21, `docs/lanes/L21-budget-and-models.md`). Candidate: the v0.11.1 `driving_vision.onnx` alone (MIT; lanes, edges, leads, pose without recurrent state; converts to ncnn with every layer on Vulkan, largest activation 583, an estimated 28-38 ms of GPU per run against 47). Needs a parser, a five-frame ring per input, the phone accuracy gate and a check of the presence flicker (it has no memory: 2-69 ego-lane on/off changes a minute against 0-41). v0.9.8 / v0.9.9 and the merged graphs of v0.11.2 and master are not candidates

## Model choice note (2026-10-21)
supercombo v0.9.7 is used first: single model, 54 ms fp32 on the phone, outputs lane lines, road edges, leads, plan, pose directly (docs/SUPERCOMBO_SPEC.md). openpilot v0.9.8–v0.11.1 split it into `driving_vision.onnx` + `driving_policy.onnx` with a 5 Hz history, which would allow running every 4th frame (less CPU) at the cost of a new parser; current master merged them again. Revisit when CPU budget becomes the problem.

## Known limits after phase 1 (2026-10-21)
- Thermal: on the bench with no airflow the SoC reaches 90-95 C with modeld on the big cores; the unit runs modeld at `--max-hz 5 --throttle-temp 75000`, which gives about 2.5-5 road messages/s; signd (roadsense-signs) on the little cores gives 1.5-2 fps. In-car mounting needs airflow or a lower duty cycle. 20 Hz like openpilot is not reachable on this SoC on CPU.
- Camera orientation: no rotation is applied; decide at mounting time (camerad option to add).
- Exposure metadata from the soft ISP is unreliable (no sensor helper for IMX363); image quality itself is fine.
- Calibration is a static config file; auto pitch/yaw from the model's pose is phase 2.
- api and daemons run as root (phase 1); a system user with RemoveIPC=no comes later.

## One road geometry for a consumer that draws the road (lane L17, 2026-11-17; docs/lanes/L17-road-geometry.md)
- [x] mapd `ahead.geometry`: the most likely path, the side roads at its junctions and the first roundabout as polylines in the frame of the matched position; on a ring the path takes the announced exit
- [x] topic `roadgeom` / `/v1/roadgeom` (`api/roadsense_api/roadgeom.py`): the camera's lane centre near the car, the map's road beyond, side roads and a roundabout in the car's frame, the map part fitted to the camera every frame and withheld when the two disagree; provenance and confidence per part; `tools/roadgeom_replay.py`, `tools/roadgeom_drive.py`
- [x] optional driver intent: `turn_signal` in the position input, relayed in `ego`, selects the side road or roundabout exit
- [ ] first drives with a real position: agreement rate of camera and map, the fit on wide roads
- [ ] the turn-signal convention at roundabouts against real use

- int8 quantisation tried and rejected (2026-10-21, docs/QUANTIZATION.md): the FastViT backbone's heavy-tailed activations make every int8 variant fail the lane-accuracy gate, and on ARM the QDQ overhead eats the gain anyway. fp32 stays. (The 2.09 GHz big-core cap that came out of it was removed on 2026-10-26 under the speed-first ruling: modeld runs on the GPU and the fan holds the die near 60 C; docs/lanes/L3-speed.md.)

## GPU backend decision (2026-10-21)
The ncnn/Vulkan fp16-packed backend misses the original accuracy gate only at the far end of the lane lines (x = 192 m: max 0.28 m) and for leads beyond 50 m (up to 1.9 m, i.e. about 3 %). Within 100 m the lane error stays under 0.11 m and the error does not accumulate through the recurrent state. For data gathering that is acceptable, and the GPU path runs at 60-90 ms per run on about half a little core instead of four big cores at 90 C, so the systemd unit now uses `--backend ncnn --max-hz 20`. The ORT fp32 path stays as the exact reference (`--backend ort`, `tools/ncnn/compare_replay.py`). Open: the residual fp16 error sits in the temporal attention block; turnip needs `TU_DEBUG=flushall` (set by modeld) or results are randomly corrupted, which is a driver bug worth reporting upstream.
- Live GPU stack on the phone (2026-10-21 22:05 CEST, 2 min, root units, no heatsink, display disconnected): 12 road messages/s at 76 ms per run, averaged die 76 C (cpu7 72 C, GPU 73 C), modeld 8 % CPU, camerad 6 %, signd 29 % on the little cores, big cores idle. Compare the CPU path: 2.5-5/s at 87-94 C.

## Vehicle input API 1.1 (lane L18, 2026-11-17; docs/VEHICLE_API.md, docs/lanes/L18-vehicle-api.md)
- [x] one car-agnostic contract for everything a host knows about the car: `gnss`, `motion` (now also accelerations, partial messages), `controls` (turn signal, brake, accelerator, cruise), `body` (lamps, wipers, rain, light, temperature, doors), `vehicle` (static description with camera and sensor mounts), `objects` (radar / lidar / ultrasonic object lists); per-signal quality (`good` / `uncalibrated` / `unverified` / `invalid`) and source; `age_ms` as an alternative clock; hello handshake with `accepts` / `signals` / `provides`; v1 feeders unchanged (same output message for message on a recorded drive)
- [x] mapd: `vehicle.sock` with the topics `vehicle` (every signal with age, quality, source; description; feeders; sensors; calibration) and `objects` (scans in the camera frame); api `GET` / `POST /v1/vehicle`, `/ws` topics, recorder
- [x] calibration feedback: gyroscope bias at standstill (applied), yaw-rate scale against the GNSS course per turn and against the wheel yaw, wheel-yaw scale, speed scale against GNSS, GNSS lag against the vehicle speed; kept between runs; a converged scale is applied only to a signal declared `uncalibrated`
- [x] lead tracker: radar / lidar fusion behind `radar_fusion` (off), tested on synthetic radar; `lead.ego.brake`
- [x] fused speed limit: a map-limit change ends a camera sign only from a sure match and after 40 m (the first drive with a position flapped 30 <-> 50 km/h 24 times in six minutes)
- [x] recorder: `vehicle`, `objects`, `roadgeom` recorded; synced to disk every 5 s; a session opened on a stale clock is renamed
- [ ] nothing of 1.1 has run on the phone or the car yet; the cost figures are from a laptop
- [ ] the calibration estimators against GNSS have only run on synthetic drives: the recorded drive has no raw fixes (the host did not log them) and its receiver was too weak to be a reference (3-5 satellites, fixes for 31 % of the driving time). A drive with logged fixes and `course_deg` is the check; `gnss.lag_s` on that receiver is the first number to look at
- [ ] dead-reckoning drift with gyroscope yaw against wheel yaw could not be measured on that drive for the same reason (1-3 usable windows); what it did show: on straights the rear-wheel yaw of the reference car is biased by -0.6 deg/s (a tyre difference) with 1.0-1.5 deg/s noise, its gyroscope by 0.02-0.11 deg/s with 0.3 deg/s noise
- [ ] not used yet: turn signal for the ego traffic light (a left-turn signal group), wipers / rain / light sensors as environment hints in `fused`, accelerations, the accelerator, the cruise state; the lane tracker's ego-motion compensation with the vehicle yaw rate is untried
- [ ] mapd's fix handling with a lagging, smoothing receiver: fixes rejected more often the faster the car goes and re-anchoring jumps after dead reckoning (drive 2026-11-17); the lag estimate is the input for it, the weighting and the re-anchor logic are open
- [ ] the map matcher takes service roads too readily while driving (38 % of the matches on that drive)

## roadsense 2.0: display channel and a live lead at standstill (lane L19, 2026-11-18; docs/lanes/L19-view-channel.md)

- [x] `lead` stays live while both stand (filter with a standing model, Huber, CUSUM ahead of the gate); the frozen `hold` mode is gone (`standing` and `departed` stay); measured on the car's recorded drives
- [x] topic `view` / `/v1/view`: display channel (lead, traffic lights, signs, lane lines, edges, path) with ids, car-frame positions, velocities, fades; never feeds the measurement topics; `road: null` slot for the fused road geometry (lane L20)
- [x] `tools/view_replay.py` (a recorder session through the trackers and the view, jitter metrics, the api's stream for consumer-side replays); the replay tools read truncated sessions
- [x] car computer (branch `v2-l19`): the MFD draws lead, ego light, lanes from `view`; scene built per `view` message; no stale health-mirror light; road picture redrawn while standing; read-out hysteresis
- [ ] not seen on the car yet: deploy roadsense and the car computer from the `v2-l19` branches, then one drive with the recorder (`tools/view_replay.py`, the car repo's `tools/scene_feed_replay.py`)
- [x] `view.road`: the fused road geometry, the junction and the lanes merged into the display channel (lane L22, below)

## Ego lane, junctions on the display (lane L20, 2026-11-18; docs/lanes/L20-ego-lane-junctions.md)
- [x] where the junction chain broke on the recorded drives (44 labelled junctions, no roundabout in any recording): no position at all at 21 of them (car side: the head-unit GNSS started on 17 Nov afternoon; since 17 Nov 22:07 the feeder sends only `fix: none` and no CAN); a matcher that trusted an optimistic receiver; the MFD's junction schematic off by default and a strict trust gate. Drawn on the MFD: 5 -> 15 (replay of the same drives)
- [x] mapd: receiver accuracy from its own innovations (`input.gnss.hacc_scale`), service / parking roads lose with speed, `match.confidence` of the road (median while moving 0.45 -> 0.55; service road at speed 20.5 -> 15.0 %, on raw fixes 24.4 -> 7.0 %)
- [x] api: ego-lane filter (`egolane.py`) in `fused.lanes`: probabilities per lane, "unknown" instead of a guess; held out on the car's camera: wrong and confident 21.4 -> 11.1 %, with the real map 17.6 -> 1.2 %
- [x] tools: `tools/map_replay.py`, `tools/egolane_replay.py`; car repo `tools/mfd_chain_replay.py`
- [ ] the car's position input since 17 Nov 22:07 (feeder connected, no CAN, no fix) and the weak head-unit receiver: to check on the car
- [ ] a drive through a roundabout with a fix
- [x] `turn:lanes` and speed cameras in the pack (lane L22, below)

## roadsense 2.0: one line, one display contract (lane L22, 2026-11-18; docs/lanes/L22-v2-integration.md)
- [x] branches `v2` (roadsense, car computer): L19 and L20 merged; api version 2 (`/v2/...`, `/v1` aliases, `__version__` 2.0.0); migration notes in docs/ARCHITECTURE.md ("roadsense 2.0: what changed since 1.x")
- [x] `view.road` (`roadview.py`): geometry (camera near, map beyond, side roads, ring), the junction view with arms, lanes with per-lane probabilities and the lit / likely lane, turn arrows; `approximate` and provenance per part; trust hysteresis and distance carrying moved from the consumer into roadsense; the topic `roadgeom` is gone
- [x] ego lane: lane changes read as a pattern on the tracked lines, a changed lane count opens the estimate, turn arrows say the side; wrong and confident: night expressway 3.8 -> 1.5 %, city 7.7 -> 4.6 %, the car's camera 11.1 -> 5.3 %
- [x] lane lines on the display: smoothing that grows with the distance in `view`, lines moved with their own rate between messages on the MFD: drawn steps over 0.3 m at 30 m on the day drive 56 -> 6 (before L19: 15); ego-motion compensation measured and left off
- [x] map pack format 2: turn lanes (75,813 edges in Poland), speed cameras on the road (117); mapd reads formats 1 and 2; Poland pack rebuilt on the dev box (263.4 MB, +1.1 MB)
- [x] session 20261118-095209 (parked, no lane line or lead): the scene (a lawn and a hedge), not a fault
- [x] tools: `tools/replay_lib.py` (the api's derived topics offline, shared by the replay tools and the car's), `tools/view_replay.py --map`
- [ ] nothing of 2.0 has run on the phone or the car: deployment order and what a drive has to show are in the lane report
- [ ] `/v1/...` aliases: remove once no consumer uses them
- [ ] speed cameras mapped beside the road (526 of 643 in Poland) need a point layer in the pack

- [ ] `view.road`: merge the fused road geometry (lane L20)

## Budget and models (lane L21, 2026-11-18; docs/lanes/L21-budget-and-models.md)
- [x] budget of the running stack per daemon and stage (`tools/budget/`): 6.67 W at the rail, big cores 146 % and 70 C, GPU 91 % busy and 57 C; a busy big core is about 2 W, the GPU about 2.3 W and full in time
- [x] envd on the little cores: the same labels (60 of 60 replayed frames identical), 0.5 W and 8 C on the big cores less, road latency 120 -> 108 ms
- [x] traffic lights at 5 Hz (roadsense-signs unit): reported share of the light-time on the replays 95.5 -> 98.3 % by day, 93.6 -> 98.9 % at night; lights under 12 px 81 -> 93-96 %
- [x] light state model for the car's own camera (roadsense-signs `tools/lights/`, weights local): lit lights of the two recorded drives given their colour 70 -> 91 %, called dark 29 -> 9 % (cross-validated on 320 labelled detections)
- [ ] none of it is deployed; the ego light over a whole drive with the new state model is unmeasured (the recorder keeps one frame a second)
- [ ] signs under about 15 px and the sign detector's 5 Hz: the GPU has no time left; the v0.11.1 road model is the way to make room
- [ ] light detector on the CPU is the largest big-core load (90 %): int8 or a detector whose layers all run on Vulkan
- [ ] arrow signals, night / rain / yellow for the state model: need recorded drives and labels

## The new model line as the default road model (lane L27, 2026-11-19; docs/lanes/L27-new-model-default.md)
- [x] `modeld` default: openpilot v0.11.1 `driving_vision` (GPU) + `driving_policy` (CPU, 5 runs a second, the plan held in between and marked); supercombo v0.9.7 selectable and the fallback when the new files are not installed (flagged in `road` and in health); model files pinned by URL and sha256, conversions reproducible
- [x] lead tracker for this model (profile by `road.road_model`): relative speed of the 200 ms pair trusted, weak evidence through presence dropouts (marked), acquisition by accumulated evidence, ids kept for a re-found or re-located vehicle. On four rides it was not tuned on, after the tracker: on/off 1.75 a minute against supercombo's 1.65 (3.15 with supercombo's settings), identity changes 0.69 against 0.95, relative-speed noise 0.7 against 3.3 m/s
- [ ] the lead at close range in town is still less steady than supercombo's (per-frame step 0.44 against 0.28 m rms on warsaw-city; the drawn lead about twice as unsteady there); a gone lead stays 0.9 s (marked `coast`) against 0.2 s
- [x] ego lane: with this model's lines the filter commits at 0.9 instead of 0.8: wrong and confident 3.3 % of the labelled frames (supercombo 3.4 %, the switch alone 5.5 %), right 86.8 % (90.2 %), unknown 9.9 % (6.3 %); threshold chosen on the same three rides
- [x] seam for a separate lane source for the display (`display_lanes` on `<rundir>/laned.sock`, `view.lines_source`); no provider yet
- [x] laned: the display lane source (classical paint analysis on the little cores, optional unit, off by default; seam modes `agree` / `prefer` / `replace`; docs/lanes/L29-display-lanes.md). Measured: no candidate (classical, UFLD, YOLOP) beats the road model's own lines for the display; in `agree` it puts the drawn lines on the paint. Phone timing and on-car A/B open
- [ ] lane lines of this model are worse than supercombo's at far range and flicker more (L24-M); by the owner's ruling a separate cheap lane analysis for the display is the next step, not part of this lane
- [x] load governor: the policy's rate (`plan`, level 8) is shed before the road model's rate (now level 9)
- [x] phone, benchmarks only (nothing installed): the stack with the new default against the same stack with supercombo: 6.59 against 6.83 W, GPU busy 84 against 91 %, road 15.2 against 13.8 runs a second, road latency 85 / 97 against 111 / 131 ms (median / p90), frame prep on the little cores 11.8 against 20.7 % of a core; the policy 7.9 ms per run on a big core (1.1 % of a core at 5 Hz)
- [ ] fp16 on the Adreno: the lead distance of this model differs from fp32 by 0.37 m on average (0.86 m p95), twice supercombo's; the strict gate script rejects both models on the 120-frame bench clip (its maximum runs over absent lines to 192 m). A longer clip with leads through both arithmetic paths and the tracker is open
- [ ] the run rate is now set by the GPU turn-taking with the sign detector (tuned for supercombo's 54 ms run), not by the GPU: 7 points of GPU time are unused
- [ ] not deployed; the car's own camera has not seen this model
