# L7 traffic-lights: detection, state and ego-lane relevance next to the running stack

Lane opened 2026-10-27 ~06:35 CET (Opus, build + measurement). Goal: roadsense serves traffic lights (boxes,
colour state red / yellow / red+yellow / green / flashing / off, which light applies to the ego lane) live on
the phone next to modeld + signd + envd, without costing the road rate much (speed-first ruling: road >= ~12/s).
Times CET (Warsaw local, UTC+1). Laptop scratch: `~/l7/` (venv, scripts, exported models, eval outputs); downloads
in `~/datasets/l7-models/`; phone scratch `~/l7/` (sources, build `build-signs`, models, scripts, `live/`).

## 0. Starting point
Phone up 8 h 48 min, full stack from the L4 units on the Krakow replay (camerad-replay 20 fps): modeld 14.1 runs/s
(model 53 ms, 50 GPU turns per 10 s), signd 5.0/s, envd 1 Hz, api; `roadsense.target` disabled.

## 1. Candidate models and licences

| model | source | licence (weights) | classes | used for |
|---|---|---|---|---|
| **dronefreak/bdd100k-yolo11n** | HF, commit `2b55541`, `best.pt` sha256 `c120d075...` | card AGPL-3.0 (Ultralytics); trained on BDD100K (research / non-commercial terms) | 10 BDD classes incl. `traffic light` (no state) | **chosen detector** |
| Ultralytics yolo11n.pt (COCO) | local copy of the Ultralytics release asset | AGPL-3.0 | COCO, `traffic light` = 9 | baseline, rejected |
| dronefreak/bdd100k-yolov8n | HF, commit `2c24162` | AGPL-3.0 / BDD100K | as above | not evaluated (see nulls: loading the downloaded pickle was denied) |
| AutowareFoundation/traffic_light_classifier | HF, commit `527c490` | **Apache-2.0** (TIER IV, Japanese lights) | 11 lamp combos (green, red, red+arrows, yellow, unknown) | state baseline |
| AutowareFoundation/traffic_light_fine_detector (YOLOX-s) | HF | Apache-2.0 | traffic_light, pedestrian | not used: refines map ROIs, Japanese lights, ~4x the compute of YOLO11n |
| Megvii YOLOX-nano / tiny ONNX (COCO) | GitHub release 0.1.1rc0 | Apache-2.0 | COCO | downloaded, not evaluated (COCO-class detector; COCO YOLO11n already far behind) |
| own state CNN (48x24) | trained here on BDD100K val crops | weights: BDD100K terms (local only); code AGPL-3.0 | red, yellow, green, none | **chosen state** |

Datasets: BDD100K val (10 000 images 1280x720, FiftyOne mirror already on disk from L2, `~/datasets/dashcam/bdd100k_val_fo`,
26 884 traffic-light boxes with colour G 11 430 / NA 8 355 / R 6 605 / Y 494; BDD100K terms: research /
non-commercial) and the Krakow replay ride (CC BY 3.0, Relaxing Roads 4K, attribution in `L5-ride-replay/ATTRIBUTION.txt`).
No accounts created; Bosch STL, LISA, DTLD, S2TLD not used (registration or non-commercial, and BDD covers the need).
Repository: the detector is AGPL/Ultralytics and the CNN weights are research-licensed, so the provider lives in
roadsense-signs (AGPL) as `signd --lights`; roadsense core only gets the topic contract, the api annotation and the overlay.

## 2. Detector: which model, which input (BDD100K val, laptop, ORT fp32)

Method: `~/l7/scripts/eval_det.py MODEL CLS NAME x0,y0,x1,y1@scale ...` (region of the 1280x720 frame resized by
`scale`, padded to multiples of 32, Ultralytics decode, score > 0.01, NMS 0.5) on the 2000 images whose sha1 % 5 == 0
(5 533 lights), `~/l7/scripts/score_det.py` (greedy matching by score, AP all-point; recall by GT box height at
score > 0.25). The BDD100K val set is held out from the detector's training (the card's "test" split is BDD val).

| model | region @ scale | input | AP50 | AP30 | R / P @0.25 | R h<12 | 12-24 | 24-48 | >=48 px |
|---|---|---|---|---|---|---|---|---|---|
| COCO yolo11n | full @0.5 | 640x384 | 10.4 | 22.6 | 7.6 / 49.4 | 0.0 | 0.9 | 11.9 | 41.0 |
| COCO yolo11n | full @1.0 | 1280x736 | 25.3 | 41.0 | 19.0 / 55.1 | 1.2 | 8.9 | 31.8 | 56.3 |
| BDD yolo11n | full @0.5 | 640x384 | 49.6 | 71.6 | 44.7 / 72.6 | 7.1 | 35.3 | 65.6 | 79.8 |
| BDD yolo11n | full @0.75 | 960x544 | 63.8 | 81.2 | 58.6 / 75.5 | 23.2 | 52.4 | 76.5 | 83.8 |
| BDD yolo11n | full @1.0 | 1280x736 | 67.8 | 83.7 | 63.8 / 75.0 | 35.4 | 60.2 | 77.0 | 81.1 |
| BDD yolo11n | rows 0-448 @0.5 | 640x224 | 44.4 | 68.2 | 40.4 / 71.4 | 3.8 | 29.6 | 62.6 | 77.3 |
| BDD yolo11n | rows 0-448 @0.75 | 960x352 | 62.8 | 79.6 | 57.4 / 75.0 | 26.1 | 51.2 | 74.3 | 80.5 |
| **BDD yolo11n** | **rows 0-427 @0.75** | **960x320** | **61.1** | 78.2 | 56.0 / 75.6 | 20.8 | 49.4 | 74.5 | 80.9 |
| BDD yolo11n | rows 0-464 @0.69 | 896x320 | 60.9 | 78.9 | 54.9 / 76.0 | 18.3 | 48.1 | 73.6 | 82.4 |
| BDD yolo11n | centre 960x384 crop @1.0 | 960x384 | 55.2 | 71.5 | 53.4 / 72.7 | 35.9 | 53.1 | 60.6 | 57.6 |
| BDD yolo11n | centre 640x256 crop @1.0 | 640x256 | 34.3 | 48.1 | 35.0 / 68.0 | 29.0 | 37.0 | 38.2 | 22.5 |

Readings: (1) the BDD-trained detector is the only usable one (COCO: AP50 10-25). (2) Resolution matters for the
12-24 px lights (a 0.9 m housing at 30-60 m with f 690 px): 0.5x loses a third of them. (3) Lights sit above the
horizon (BDD: 82-98 % of the boxes have their centre above row 360, by size), so a full-width band of the upper
~60 % at 0.75x keeps almost all of the full-frame 0.75x accuracy (61.1 vs 63.8) for 61 % of the pixels; centre
crops at full resolution find small lights but lose the large near ones at the sides. Chosen: **960x320 on the
upper 62 % of the frame** (on the Krakow ride the horizon is at row ~420; rows 0-446 at 0.72x).

ncnn conversion (`tools/ncnn/make_ncnn.sh` path: onnxsim -> pnnx -> the sign detector's head split): CPU vs ORT max
|d| 0.002 px boxes / 0 scores, Vulkan fp16 on the laptop GPU boxes <= 4 px, scores <= 0.015 (`~/l7/scripts/cmp_ncnn.py`).
A lifetime trap in the comparison script cost 20 min: `ncnn.Mat(np.ascontiguousarray(x))` of a temporary array
reads freed memory (garbage outputs) - keep the array referenced.
C++ parity on the phone: `lights_replay --backend ncnn-cpu --in 960x320 --roi 0,0,1,0.59306 --thr 0.01 --no-track`
on 400 BDD images (JPEG -> NV12 like the ring) scores AP50 **61.9** vs **61.8** for the Python/ORT reference on the
same images (R@0.25 56.0 vs 57.2).

## 3. State: rule vs Autoware vs a small CNN

GT crops of BDD val (boxes >= 10 px tall: R 6 076, Y 476, G 10 611, NA 8 062). Recall per true class:

| method | red | yellow | green | none -> off/unknown | macro R/Y/G | cost |
|---|---|---|---|---|---|---|
| colour rule (hue band x lamp position, `tlstate.py` = `light_rule_scores`), all crops | 54.0 | 66.0 | 71.2 | 50.6 | 63.7 | ~0.1 ms |
| Autoware MobileNetV2 (Apache), 4000 random crops, rotated (red lamp right) | 86.2 | 53.4 | 85.1 | 74.2 | 74.9 | 224x224, ~0.3 GFLOP/crop |
| small CNN v1 (25 epochs), test half | 89.5 | 53.5 | 95.9 | 89.0 | 79.6 | ~1 ms/crop (phone) |
| small CNN v2 (40 epochs, class weights^0.85), test half | 87.3 | 57.3 | 96.0 | 88.4 | 80.2 | same |
| **small CNN, final** (`tools/lights/train_state.py`, defaults = v2 recipe), test half | **87.1** | **61.0** | **95.8** | **88.2** | **81.3** | same |

(CNN: `~/l7/scripts/train_cls.py`, committed as `roadsense-signs/tools/lights/train_state.py`; test half = images with
odd sha1, never trained on; v1/v2 were selected on that half, so it is not a clean holdout for the choice between them.)
Night vs day (v1): red 93.8 / 81.2, green 98.4 / 92.8, yellow 46 / 58, none 69.5 / 93.2. Yellow is the weak class
(240 training crops; confused with red).

On the Krakow ride (63 detected crops from 13 frames, `~/l7/ridesel_*.jsonl`, contact sheet checked by eye) the CNN
labels every lit Polish light that a human can read correctly and calls the back and side views `off`. Rule vs CNN on
those 63 crops: agree on 47 (green 12, red 18, off 17); the rule calls 8 back views against the sky `green` (sky hue
falls into the cyan-green LED band), 3 red lamps `yellow` (red LEDs render orange in this video), 2 red `green`, and
one clear red `red_yellow` (glare over the middle lamp; this override was then removed from the CNN path).
Red+yellow (not a BDD class): the ride has two red+yellow phases (frames 1060-1070 and 3200-3210); the CNN says
yellow for 4 and red for 8 of those 12 crops. Neither the hue rule nor a brightness-per-third rule separates them
from plain red (red lamps saturate to white with a halo over the middle third: 111 false vs 10 true at the best
threshold). So `red_yellow` is inferred in the tracker from the sequence (yellow observed straight after red ->
red+yellow, which in Poland is the only yellow that follows red); a red+yellow phase seen as red stays red (safe side).

## 4. Implementation

roadsense-signs (`046642a`):
- `signd/lights.{h,cc}`: `LightPipeline` (ROI resample with the sign letterbox's resampler, detector through the
  existing `ModelRunner`, class 8 decode + NMS, state per box: CNN on a 48x24 crop or the colour rule; fallback to
  the rule when the CNN file is missing), `LightTracker` (greedy IoU then centre-distance association, CV box
  centre, state switch after 2 agreeing observations or one at p >= 0.9, red+yellow from the sequence with a 2.5 s
  hold, flashing = yellow/off alternation in 4 s), `lights_json`.
- `signd/main.cc`: `--lights` starts a second thread with its own `RingReader`, rate cap `--lights-hz`, CPU affinity
  `--lights-cpus`, and its own `JsonLineServer` on `<rundir>/lightd.sock` (the server class is single-threaded, so
  the thread owns it). With `--lights-backend ncnn` and `--gpu-sync turn` it takes its own GPU turns from modeld;
  a process mutex keeps the sign and light detectors from asking at the same time (`gpu.sched` has one want/turn
  pair); light turns end with `end_turn(0)` so the shared `det_ms` (modeld's lead) stays the sign detector's.
- `signd/ncnn_runner.cc`: `ncnn-cpu` backend (ncnn's ARM kernels, fp16 arithmetic on the A75/A55).
- `tools/lights_replay.cc` (JPEG/NV12 -> `lights` JSON with raw detections and timing), `tools/lights/train_state.py`,
  `scripts/fetch-models.sh` / `tools/ncnn/make_ncnn.sh` / `scripts/install-phone.sh` handle the light models,
  `docs/LIGHT_MODELS.md` (sources, sha256, licences), unit `systemd/roadsense-signd.service`.

roadsense (this commit):
- `api/roadsense_api/lights.py`: ego-lane relevance (docs/ARCHITECTURE.md "light provider"): distance from the
  housing's long side (0.9 m), lateral offset from modeld's plan projected at that distance (same projection as the
  overlay), relevance = facing state x lateral window (+1.5 m centre, 4.5 m scale, -7..+10 m) x distance (60-120 m
  fade) x hits x sqrt(score); ego = best >= 0.15, state = relevance-weighted vote of the lights within 30 % of it.
- `state.py`: daemon `lightd`, topic `lights`, messages annotated before they are stored and forwarded; health gets
  `lights_ms` and the ego light. `server.py`: `GET /v1/lights`, HUD line, index-page section. `overlay.py`: boxes in
  the state colour, the ego light thick with "EGO #id state N m".
- Also fixed in `server.py` (one character each, L6's and mine): the index page's `.join("\n")` inside the
  non-raw Python string became a raw newline in the JS, a syntax error that blanked the whole status page.
- Tests: `api/tests/test_lights.py` (6 unit tests: distance/lateral, cross-street light not ego, back view ignored,
  group vote, curved path, no road), `test_api.py::test_lights` end to end with a fake `lightd` in
  `tools/fake_daemons.py`. `python -m unittest discover -s api/tests`: 41 tests OK (laptop, roadsense .venv-models).

## 5. Cost on the phone (live, Krakow replay, 200 s per row)

Method: L4's harness copied to `~/l7/` (`cfg.sh "SIGND_ARGS"` = runtime drop-in for signd only, restart, check that
modeld still prints stats; `live.sh NAME 200`: daemon stats = mean of the journal lines of the last 120 s, light
lines split from sign lines; ws latency frame -> client over the last 60 s incl. the `lights` topic), runner
`systemd-run --unit l7-suite1 sh ~/l7/suite1.sh`, 07:22-07:49 CET. All rows: modeld and envd as installed (L4),
signd's sign part as installed; light rows: 960x320 detector on rows 0-446, state CNN v2 (the final CNN, same recipe from the committed script, was installed for the 15-minute run; identical cost).

| run | lights placement | road runs/s | model mean / p90 ms | signs/s (detect ms) | lights/s | lights det / provider ms | ws latency med / p90: road, signs, **lights** | hottest zone last 2 min (max) |
|---|---|---|---|---|---|---|---|---|
| B0 | none (installed L4 signd) | **14.04** | 53.2 / 55.1 | 4.99 (96) | - | - | 107/125, 131/157, - | 57-62 (62) |
| LIT3 | ncnn-cpu, 4 thr, little cores 0-3, 3 Hz | 13.78 | 54.0 / 57.0 | 4.99 (95) | 3.00 | 142 / 180 | 102/122, 125/154, **213/245** | 60-62 (64) |
| BIG3 | ncnn-cpu, 2 thr, big cores 4-7, 3 Hz | 13.72 | 54.0 / 57.0 | 4.99 (99) | 3.00 | 90 / 100 | 107/126, 136/163, **139/158** | 64-82 (82) |
| GPU3 | ncnn Vulkan, own GPU turns, 3 Hz | **9.95** | 66.0 / 96.0 | 4.96 (113) | 2.98 | 108 / 171 (+53 wait) | 115/154, 146/184, 213/274 | 53-64 (64) |
| GPU3S | ncnn Vulkan 640x224, turns, 3 Hz | 11.99 | 58.2 / 73.3 | 4.99 (100) | 3.00 | 74 / 120 (+39 wait) | 96/122, 139/172, 137/231 | 59-70 (70) |
| LIT5 | as LIT3, 5 Hz | 13.68 | 54.3 / 57.3 | 4.99 (98) | 4.98 | 137 / 169 | 101/121, 126/158, 199/229 | 89-93 (93) |
| LIT3S | 640x224 on the little cores, 5 Hz | 13.75 | 53.7 / 56.3 | 4.96 (95) | 4.96 | 80 / 99 | 102/123, 128/156, 135/163 | 73-87 (87) |

Readings:
1. **The GPU is the wrong place**: the light detector needs ~90-110 ms per pass of the Adreno in a turn (the C2PSA
   MatMuls run on the CPU between submits, and modeld takes the turn back after 80 ms), so 3 lights/s cost 4.1 road
   runs/s (9.95) and even the 640x224 variant costs 2 (12.0) while being the weaker detector (AP50 44 vs 61).
2. **On the CPU the light detector is nearly free for the road**: -0.26 to -0.36 road/s (13.68-13.78 vs 14.04, the
   spread between repeats of one config in L4 was ~0.1), signs unchanged at 4.99/s, road latency unchanged.
   ncnn's fp16 ARM kernels run the 960x320 YOLO11n in ~90 ms on two A75s and ~140 ms on four A55s (ORT on four A75
   at 640x224: 75 ms vs ncnn 52 ms, `lights_replay` bench).
3. Big vs little cores: same road cost, lights latency 139 vs 213 ms median (the A55s are 1.5x slower and the
   resample `pre` step is 30 vs 9 ms); the big cores run warmer (82 C peak in the 200 s window).
4. 5 Hz on the little cores works (4.98/s) but the die ran at 89-93 C in that window (close to the 95 C stop; the
   window followed the GPU runs and the scene is not controlled, but it was the only run near the limit).
Choice (speed and responsiveness ruling): **BIG3** = `--lights --lights-backend ncnn-cpu --lights-threads 2
--lights-cpus 4-7 --lights-hz 3`, state CNN. Fallback documented in the unit: the little cores (LIT3) if heat or the
big cores become the constraint.

### 15-minute run of the installed configuration (07:51-08:06 CET, `systemd-run --unit l7-final sh ~/l7/live.sh FINAL 900`)
Installed units, no drop-ins: road **13.70 runs/s** (88 stats windows: 40 at 13.7, 32 at 13.6, 10 at 13.8, 5 at
13.5, one at 12.3), model 54.1 / p90 57.2 ms, 50 GPU turns per 10 s; signs **4.99/s**, detect 98 ms; lights
**3.00/s**, provider 103 ms (det 92 ms) over all 88 windows 106 ms; envd 1.0 Hz, 323 ms. ws, last 60 s: road
13.74 msg/s, latency 107 / 127 ms median / p90, newest road message 145 ms old; signs 137 / 166 ms; **lights 139 /
168 ms** (max 222); env 360 ms. Temperatures (hottest zone, 30 s samples): 62-72 C, one excursion to 81-84 C at
07:56-07:58 (cpu7 84), back to 62-66 C, 75 C at the end; never near 95 C.
Cost of lights vs B0: -0.34 road runs/s (-2.4 %), signs and road latency unchanged.

## 6. Live accuracy on the Krakow replay (qualitative)

`~/l7/scripts/grab_overlays.py http://192.168.0.86:8080 OUT 640 3` (laptop: reads `/preview` and `/ws?topics=lights`,
saves the overlay frame and the lights message at most every 3 s while lights are reported), 08:06-08:17 CET, one
ride loop: 43 frames. Four kept in `docs/lanes/L7-traffic-lights/` (CC BY 3.0, `ATTRIBUTION.txt`):

- `overlay-red.jpg`: three-lane approach on a tram crossing, overhead red (#371, ego, 24 m, lateral +0.1 m) and the
  right-hand pole red (#376, +4.0 m) agree (2/2); back views of the opposite direction are grey `off` and ignored.
- `overlay-red-yellow.jpg`: the same junction 10 s later shows **red+yellow** on the ego light (#433, 22 m; the CNN
  saw `yellow`, the tracker turned it into red+yellow because it followed red), one message later
  `overlay-green.jpg` green. The red overhead light for the left-turn lanes (#431, lateral -7.0 m) has relevance 0.
- `overlay-cross-traffic.jpg`: ego green 10 m ahead (#431) while three reds of the crossing/turning directions on the
  left are listed but not ego.

Sequence over the loop (43 messages): ego red at 24-40 m for ~6 s, then green (first junction; its real red+yellow
phase was seen as red, so the switch shows red -> green), later green 42 -> 10 m (second), red 31 -> 21 m for ~20 s
-> red_yellow -> green (third). Misses and false states seen: (a) the first junction's red+yellow phase (CNN said red,
reported red: the safe side); (b) one frame with ego green and a red light 22 m ahead at +1.2 m (#448, `agree 1/2`:
the lane split at that junction, the two heads control different lanes); (c) a red at 35 m reported ego with
relevance 0.24 at the end of the loop where the path curves (#473); (d) boxes trail the moving lights by up to a
box width (~140 ms latency + 3 Hz). No flashing yellow occurs in the ride (untested live; covered by the tracker
logic only). Night: not tested live (the ride is dusk); BDD night crops (CNN v1): red 94 %, green 98 %, yellow 46 %.

## 7. Example message (`/v1/lights`, phone, 08:13 CET, first 3 lights)
```
{"type":"lights","seq":639211,"ts_mono_ns":37146298461466,"ts_real_ns":1793085019461051444,"lights_ms":98.4,
 "frame":[1280,720],"roi":[0,0,1280,446],
 "lights":[{"track_id":431,"state":"red","state_conf":0.88,"state_s":34.7,"frame_state":"red","flashing":false,"score":0.649,
            "box":[404.7,284.7,420.1,313.9],"horizontal":false,"hits":105,"age":104,"dist_m":21.3,"lateral_m":-7.0,"relevance":0.0,"ego":false},
           {"track_id":432,"state":"off","state_conf":1.0,"state_s":34.1,"frame_state":"off","flashing":false,"score":0.411,
            "box":[677.5,304.9,690.8,327.6],"horizontal":false,"hits":99,"age":102,"dist_m":27.4,"lateral_m":1.8,"relevance":0.096,"ego":false},
           {"track_id":433,"state":"red_yellow","state_conf":0.68,"state_s":0.4,"frame_state":"yellow","flashing":false,"score":0.546,
            "box":[628.9,347.6,640.8,375.3],"horizontal":false,"hits":99,"age":99,"dist_m":22.4,"lateral_m":-0.2,"relevance":0.644,"ego":true}],
 "ego":{"track_id":433,"state":"red_yellow","flashing":false,"dist_m":22.4,"lateral_m":-0.2,"relevance":0.644,"agree":1,"n_relevant":1,"source":"path"}}
```

## 8. Nulls, deviations, open items
- GPU placement: null for speed (section 5). COCO detectors: null for accuracy (section 2).
- Colour rules for the state (hue x position; brightness per third): null (63.7 % macro on BDD, sky = green); kept only
  as the fallback when the CNN file is absent. Red+yellow: no per-frame method found; sequence inference only.
- Permission denial (verbatim): loading the downloaded `dronefreak/bdd100k-yolov8n` `.pt` for export was refused:
  "Permission for this action was denied by the Claude Code auto mode classifier. Reason: [Code from External]."
  The file was deleted unloaded; the YOLOv8n variant (pure convolutions, so fully on Vulkan) stays unevaluated.
  (The YOLO11n `.pt` had been loaded for export earlier in the lane without objection.)
- Not tested end to end: `scripts/fetch-models.sh` light steps (each step was run by hand: static export through
  Ultralytics, onnxsim + pnnx + head split, install). `lights_replay` is installed but not used by any unit.
- Thread budget: one laptop job briefly overlapped another (16 threads for ~1 min) before being stopped.
- Open: (1) flashing yellow and night unverified live; (2) Polish green conditional arrow (S-2) and arrow heads are
  just "green"/"red" boxes; (3) the dashcam itself (DVR camera) not tested, only the replay; (4) state CNN trained on
  US lights only (it reads Polish housings correctly in the ride, but yellow recall is 61 %); own recordings would
  let it learn red+yellow; (5) the ROI (rows 0-62 %) assumes the horizon near the middle; on a different mount use
  `--lights-roi` (or derive it from `calib` pitch); (6) relevance has no lane-arrow / turn-intent knowledge
  (`agree` < `n_relevant` flags such conflicts).

## 9. Phone state at hand-back
Installed: `/usr/local/bin/signd` and `lights_replay` from `~/l7/build-signs` (previous `signd.pre-l7`; previous unit
in `~/l7/roadsense-signd.service.pre-l7`), `/etc/systemd/system/roadsense-signd.service` = the repo unit (lights on),
`/var/lib/roadsense/models/bdd100k_yolo11n_320x960{.onnx,-ncnn.param,-ncnn.bin}` and `lights_state_bdd_48x24.onnx`
(local, not in any repo), api package in `/usr/local/lib/roadsense/api` = this commit (previous copy in
`api.pre-l7`). No runtime drop-ins. Running: camerad-replay (Krakow ride), modeld, signd (+ lights), envd, api;
`roadsense.target` disabled for boot.
