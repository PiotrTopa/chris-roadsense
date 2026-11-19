# L42: road-user classes end to end (non-critical)

Lane L42, 2026-11-19, laptop only (phone and car read-only; nothing deployed; the phone, `user@10.200.0.14`, did
not answer, so the 2026-11-19 recordings' frames could not be fetched: the evaluation uses the eleven earlier car
drives with frames on the laptop, `~/roadsense-rec/2026111[2-8]-*`). Owner's words: "we were adding on roadsense
a possibility to distinguish human, bike, bus, truck, car and visualise accordingly, as non-critical
classifications." Three branches: roadsense `v2-l42` (from `v2-l39`), roadsense-signs `l42-road-users` (from
`main`), car repo `l42-road-users` (from `main`). Lane L41 ran in parallel; none of its files were edited (section 8).

## 0. Result in one page

**Done and working (laptop):**

* **Perception, no extra inference.** The phone already runs a BDD100K YOLO11n for traffic lights (5 Hz, two
  views); it computes ten classes and used one. `signd --lights-road-users` (roadsense-signs) decodes the eight
  road classes from that output, pairs riders with their bike / motorbike, and writes `road_users` (class, score,
  box, clip edges, rider, all eight scores) into every `lights` message. Decode 0.04 ms median per light frame on
  the laptop; the light messages are otherwise byte-identical.
* **Integrator** (MIT, no model code): a `road_users` box places each box with the camera's calibration
  (bearing; ground distance from the bottom edge with a stated 1 sigma of ~15 % at 20 m, ~30 % at 40 m; nothing
  from the box size) -> public stream `road_users`; the `road_users` source annotates the track it points at;
  the class is a **vote over the track's life with a sticky label** (switch only at 2x the shown label's tally;
  confidence includes the detector's mean score; null below 0.35). Road users no track holds become
  `view.objects` of kind `road_user` (display only). Tracks, lead and ego state equal number for number with and
  without them.
* **Governor**: `road_users` is the first step shed (`road_users_hz` 0).
* **MFD**: lead and placed road users drawn by class (car, van, truck, bus/tram, motorbike, bicycle, person;
  unknown = today's car shape), fades from the view, carried by velocity; shots 88-94; shots 1-87 unchanged.

**Measured on the car's camera** (labels by eye before scoring: 216 detections, 150 lead frames):

| | result |
|---|---|
| per detection, precision | person 0.95, bus 0.90 (trams say bus), car 0.88, bike 0.73 (mostly parked bicycles; parked ones are never drawn), **truck 0.40** (half of the truck calls are vans: BDD's truck class), motorbike 0/2; no van class |
| lead class (125 car / van leads, after the vote) | 86 % car, 14 % unknown, **0 wrong** |
| lead class changes per minute, same track | **0** in 128 lead-minutes (car, ~2 votes/s) and 17 min of rides at 5 votes/s |
| display objects | 0 class hops in 277 objects (warsaw-city, 5 Hz); every one fades in; none lives under 1 s |
| placement vs the road model's lead (car's calibration) | distance ratio median 0.97, p10-p90 0.77-1.56; without the car's real calibration 0.61 |
| cost | no detector run added; integrator +0.05-0.07 ms per road frame (laptop); screen ~0.14 ms per redraw for 8 road users (laptop) |

**Not good enough, stated plainly:**

* **Other road users are placed only from ~19 m outwards** (the light views end at the horizon, a nearer box has
  no ground contact). Near pedestrians and the vehicles beside the car are not drawn with the default setup. An
  optional extra detector pass on a lower band (`--lights-ru-roi`) would place about five times as many; it is
  built and off, its cost is one detector run (~0.1 s of two big cores on the phone) each time.
* **truck vs van** cannot be told by this detector; the screen draws a vehicle class only from confidence 0.5,
  which hides most of the low-score truck calls, not all.
* **Only "car" is tested as a lead class on real footage**: the car's drives (and the rides) have no sustained
  truck, bus or two-wheeler lead.
* Nothing ran on the phone; its numbers are estimates.

## 1. What runs where

```
 phone: signd (roadsense-signs, AGPL)                         phone: chris-integrator (MIT)                      Pi: MFD
 ─────────────────────────────────────                        ──────────────────────────────                    ────────
 light thread, 5 Hz, views wide / zoom alternating            lights_raw ─┐
   BDD100K YOLO11n 320x960 (already running for lights)                   ├─> road_users box: place each box ──> road_users (stream, public)
   + decode of the 8 road classes from the same output                    │   with the camera's calib            │
     (--lights-road-users; riders paired; no extra run)       calib ──────┘                                      ├─> estimator: `road_users` source
   -> `lights` message + "road_users": [...]  ── lightd.sock ──>                                                 │   annotates the track it points at
 optional: road-user view (--lights-ru-roi, default off)                                                         │   -> tracks[].class, lead.class (vote)
   one EXTRA detector run on a lower band, <= --lights-ru-hz                                                     └─> view: road users no track holds
   -> {"type":"road_users","view":2,...}       ── lightd.sock ──> road_users_raw ─> road_users box                   -> view.objects kind "road_user"
 governor cap road_users_hz (step 1, shed first) pauses both                                                          │
                                                                                                    backend view_to_scene: vw.lead.k, vw.objs
                                                                                                    screen: lead + road users drawn by kind
```

Licence boundary: everything that touches the AGPL / research-licensed detector stays in roadsense-signs (the
decode in `signd/lights.cc`, the scan / labelling / scoring tools in `tools/road_users/`); chris-integrator gets
boxes, classes and scores over the socket and does geometry, voting and display on its own (MIT, no model code).
No model was added: the detector is the light detector already installed (`bdd100k_yolo11n_320x960`, source and
licence in roadsense-signs `docs/LIGHT_MODELS.md`); nothing to fetch, nothing committed.

## 2. Message fields

**`lights` (signd, `--lights-road-users`)**: one more key, absent without the flag (the rest of the message is
byte-identical with the flag on or off: section 3.7):

```
"road_users": [ {"cls": "car",              // person rider car truck bus train motor bike (BDD100K)
                 "score": 0.89,              // the class's score
                 "box": [x0, y0, x1, y1],    // frame pixels
                 "clip": "b",                // edges of the message's `roi` the box touches (l t r b); "b": no ground point
                 "rider": false,             // a bike / motor with its rider paired (union box; one road user)
                 "p": [0,0,0.89,0,0,0,0,0]} ]   // the eight class scores
"road_users": null                           // shed by the governor (`road_users_hz` 0)
```

Per-class thresholds (`--lights-ru-thr`, defaults set before any labelling and not tuned on the labels): person
0.40, rider 0.35, car 0.40, truck 0.40, bus 0.40, train 0.50, motor 0.35, bike 0.35; class-agnostic NMS 0.5, at
most 40 per frame. Rider pairing: a rider box over a bike / motor box (horizontal overlap >= half the narrower
box, the rider's centre above the vehicle's, the boxes touching) becomes one road user of the vehicle's class.

**`road_users` message (optional road-user view, `--lights-ru-roi x0,y0,x1,y1 --lights-ru-hz X`)**: `{"type":
"road_users", "seq", "ts_mono_ns", "ts_real_ns", "ms", "frame", "roi", "view": 2, "road_users": [...]}` on
`lightd.sock`, after the light message of the same frame (the lights never wait for it).

**Stream `road_users` (chris-integrator, public, kind measurement)**: per light frame with the key:

```
{"type": "road_users", "sensor": "cam", "seq", "ts_mono_ns", "view", "roi", "shed": false, "calib": "message",
 "objects": [{"class": "car" (human bike motorbike car truck bus tram), "class_conf": 0.89, "det_class": "car",
              "rider": false, "clip": "", "box": [...],
              "bearing_deg": 4.1, "bearing_std_deg": 1.2,          // left positive, car frame (calibration applied)
              "range_m": 24.3, "range_std_m": 3.9,                 // null when the box has no ground point
              "x_m": 24.2, "y_m": 1.7,                             // car frame, y LEFT; null without a range
              "method": "ground" | null, "weight": 1.0 (0.7 when clipped)}]}
```

Placement (`chris_integrator/core/roadusers.py`): bearing from the box's centre column through the calibration
(1 sigma = a quarter of the box's angular width + 0.3 deg); distance from the bottom edge on a flat road, only
when the box is not cut at the bottom of the detector's region and lies at least 4 px below the horizon (1 sigma
from 0.6 deg of pitch, 3 px + 4 % of the box height, 5 cm of camera height, through d = f h / dv); never from
the box size (a vehicle's width depends on whether it is seen from behind or from the side).

**`tracks.tracks[].class`, `lead.class`** (slot of L39, now fed): `{"label", "confidence", "source": "roadusers",
"age_s", "votes"}` or null. **`view.objects[]`**: the lead's `cls` as before, plus objects of kind `road_user`:
`{"id": "ru:<n>", "kind": "road_user", "role": "other", "x", "y" (y right), "vx", "vy", "sx", "sy", "age_s",
"meas_age_s", "phase", "alpha", "attrs": {"source": "roadusers", "approximate": true}, "cls", "box"}`.
**Car backend scene `vw`**: `lead.k` (class label from confidence 0.5; 0.35 for human / bike / motorbike) and
`objs: [{"id", "k", "d", "y", "vx", "vy", "f"}]` (nearest first, at most 8).

## 3. Measurements

Footage: the car's DVR camera (1280x720, f 975 px, wide lens), the drive recorder's one JPEG a second, eleven
drives of 12-18 November with frames (12 576 frames, day, dusk, night, low sun, rain); the 2026-11-19 drives
have no frames on the laptop and the phone did not answer. Full rate: two rides of the library
(`warsaw-city`, `krakow-lights`, other cameras) at 5 light frames a second.

### 3.1 Labelled sets (labelled by eye before anything was scored; one labeller)

* **Detections** (`~/datasets/l42/dets`): 216 boxes of the unit's two light views (141 wide, 75 second view), a
  stratified sample by the detector's class (45 per class where there were that many: bike 32, motorbike 4, no
  train at all) of the boxes with score >= 0.3 and height >= 14 px, from 199 frames of 9 drives. The sheets show
  the box on a crop and a number, never the class. Labels: person, bike (cyclist or bicycle), motorbike, car, van,
  truck, bus, tram, other. The same parked bus or a row of parked bicycles appears in several seconds: the
  sample is not 216 independent objects.
* **Leads** (`~/datasets/l42/lead`): 150 frames where the road model had a lead (prob >= 0.5, 3-70 m), at most
  one per 20 s of a drive, from 8 drives; the lead's projected rear drawn on a wide crop. Labels: 124 car, 1 van,
  25 none (no vehicle where the road model puts the lead, or unreadable). No truck, bus or two-wheeler lead in
  the sample: on these drives the car followed cars.
* The class thresholds were set before labelling and not tuned on the labels. One design change was made after
  seeing them: the vote's confidence includes the mean score of the label, and the screen draws a vehicle class
  only from 0.5 (section 3.2, `truck`).

### 3.2 Per-detection classes (before any vote)

`tools/road_users/score.py ~/datasets/l42/dets` (rows: label; columns: the detector's class after rider
pairing; `dropped` = below its class threshold):

| label \ detector | bike | bus | car | motorbike | person | truck | dropped | n |
|---|---|---|---|---|---|---|---|---|
| bike (cyclist or bicycle) | **19** | | | | | | 3 | 22 |
| bus | | **35** | | | | 1 | 2 | 38 |
| car | | | **35** | 1 | | 6 | 11 | 53 |
| van | | 1 | 1 | | | 11 | 7 | 20 |
| truck | | | | | | **12** | 4 | 16 |
| tram | | 3 | | | | | 1 | 4 |
| person | 5 | | | | **38** | | 5 | 48 |
| other | 2 | | 5 | 1 | 2 | | 5 | 15 |

Precision of the kept boxes: **person 0.95, bus 0.90** (trams are called bus), **car 0.88** (a van counted as car,
the detector having no van class), **bike 0.73** (19 of 22 bike labels are parked bicycles; persons next to
bicycles are called bike), **truck 0.40**: BDD100K's `truck` takes in the vans on this camera (11 of 30 truck
calls are vans, 6 are cars, one a bus), motorbike 0 of 2. What the screen draws from a single detection
(`--screen`: a vehicle class below 0.5 is the generic car shape): truck precision 0.57 (8 of 14), bus 0.92;
19 boxes become "generic" (12 cars / vans, 4 trucks, 3 buses: unknown, none wrong). Van as a class: not
possible with this detector. Not measured: recall over all road users in a frame (the sheets are built from
detections).

### 3.3 The lead's class on the labelled frames (after the vote)

`tools/road_users_eval.py` per drive (frames' boxes attached to the recorded light messages, the session through
the default stack with the vision model's profile, the car's calibration of section 3.5), at the 150 labelled
frames:

| label (by eye) | frames | voted class at the frame: car / none (unknown) / other | per-frame detector class at the lead's bearing: car / none / other |
|---|---|---|---|
| car | 124 | **107 / 17 / 0** | 82 / 39 / 3 (bus 2, person 1) |
| van | 1 | 1 car (right: no van class) / 0 / 0 | 1 car |
| none (no vehicle there, or unreadable) | 25 | 12 car / 12 / 1 human | 9 car / 14 / 2 person |

On the 125 car / van leads the voted class is never wrong (86 % car, 14 % unknown, 0 another class); the vote
turns 3 wrong per-frame calls and 40 frames without a matching box into car or unknown. Where the road model's
lead points at nothing (25 frames) the class is what the box on that bearing is: 12 times car (often the car the
road model's lead belongs to, a car width beside the projected point, section 6), once human (below).

### 3.4 Stability: the lead's class over time, the display objects

Changes of the lead's class while the track stays the same, after the vote (null -> label at a track's start
not counted; those are listed separately):

| footage | lead time | lead frames with a class | class changes, same track | per minute | labels voted |
|---|---|---|---|---|---|
| car, 11 drives, votes ~2/s (1 JPEG/s, two views) | 128.0 min | 97.9 % | **0** | **0** | car 102 947 frames, human 180, unknown 2 191 (null -> label at track starts: 47) |
| warsaw-city ride, 5 light frames/s | 11.5 min | 99.2 % | **0** | **0** | car 13 675, unknown 111 |
| krakow-lights ride, first 8 of 15 min with light frames | 5.8 min (whole ride) | 52.3 % (no light frames after 8 min) | **0** | **0** | car 3 633, unknown 3 310 |

The 180 `human` frames are one track on drive 20261117-145921 at a car park gate: the road model's lead was the
barrier arm, a cyclist crossed behind it, and the barrier "lead" got the cyclist's class. A wrong class on a
lead that is not a vehicle (0.17 % of the lead frames).

Display objects (`view.objects` kind `road_user`): car drives 0.00-0.15 per view message (mostly cars, persons on
20261117-145921), warsaw-city at 5 Hz 1.46 per message (63 % of the messages have one): 277 objects over 15 min,
median life 3.5 s, none shorter than 1 s, all 277 began with a fade-in (`appear`, alpha < 1), **0 class changes
within an object's life** (shape hops); by class car 20 289 / human 4 619 / bus 459 / bike 373 / truck 8 object-
messages, 519 with unknown class (drawn as the generic car).

### 3.5 Placement and its uncertainty

Coverage, from the scan of all 12 576 frames (per frame, boxes above threshold; distance from the ground point
with the car's calibration):

| view | vehicles with a distance (< 20 / 20-40 / >= 40 m) | vehicles bearing only | persons with a distance | persons bearing only |
|---|---|---|---|---|
| 0 (wide, top 62 %) | 0.01 / 0.29 / 0.15 | 1.99 | 0.03 | 0.12 |
| 1 (second view) | 0 / 0 / 0.23 | 1.07 | 0.003 | 0.08 |
| 2 (lower band 0.33-0.95, not run by the unit) | 1.61 / 0.56 / 0.15 | 0.53 | 0.17 | 0.02 |

So the light views place road users from about 19 m outwards (wide view) and 40 m (second view): the vehicle
ahead in a queue, the pedestrian at the kerb nearby, are bearing-only and can only annotate a track (the lead),
not be drawn. The optional road-user view (view 2) would place about five times as many, near ones included.

Distance against the road model's tracked lead (the nearest vehicle box on the lead's bearing, when it has a
ground point; drive 20261117-101013, n = 177): with the configured calibration (pitch / yaw 0 / 0) the boxes
are at 0.61 of the lead's distance (median); with pitch -1.3 deg, yaw -3.3 deg (the car's measured yaw error of
lane L32; the pitch sign found by this check, the opposite gave 0.38) **median 0.97, p10-p90 0.77-1.56, 99 %
within 3 sigma**; by distance 15-30 m 1.02, 30-60 m 0.95. Stated uncertainty (1 sigma): about 15 % of the
distance at 20 m, 30 % at 40 m; bearing a quarter of the box width + 0.3 deg (about 1 deg for a car at 30 m).
The other drives have too few matches for a check of their own (n 0-66, outliers where the lead is too near to
be detected and a farther car is matched).

### 3.6 Cost

| what | laptop (i7, measured) | phone (Poco F1, estimate) |
|---|---|---|
| detector | none added: the light detector already runs (5 Hz, two views) | none added |
| road-user decode in signd | 0.039 ms median, 0.064 p90, 0.44 max per light frame (4500 ride frames) | ~0.2 ms (A75, ~4x), 5 Hz: < 0.1 % of a big core |
| `lights` message | + up to ~40 boxes x ~150 B | the same |
| chris-integrator (placement, annotation, view objects) | warsaw-city: 1.457 against 1.383 ms per road frame (+0.07 ms, +5 %); 20261117-145921, first quarter (14 248 road frames), alternating runs in one process: 2.621 against 2.576 ms (+0.045 ms, +1.7 %); the road-user code does not appear among the 18 costliest calls of a profile (a whole-drive pair under heavy load from other jobs gave 10.4 / 5.8 ms: process time disturbed by the load, not repeated) | ~+0.3 ms per road frame (~0.4 % of a core at 13 frames/s) |
| screen: 8 road users drawn | 0.14 ms per scene redraw (one car 0.015 ms) | Pi Zero, ~30x: ~4 ms per redraw at <= 10 redraws/s; at most 8 objects are sent |
| optional road-user view (off) | one detector run each: 44-52 ms (4 threads, ncnn-cpu) | ~90-100 ms of two big cores each (the light frame's `lights_ms` on the car: 97 ms); at 1 Hz ~10 % of two big cores |

The phone was at 83-90 C under the current load; the default (no extra view) adds nothing measurable there. The
governor sheds road users first; without the optional view that frees only the numbers above.

### 3.7 Nothing measured changes

* Light messages: `lights_replay` on 101 car frames, unit configuration, main against this branch: byte-identical
  without the flag; with it the light part identical (only `road_users` added).
* Tracks, lead, ego state: equal number for number with and without road users (simulated town drive,
  `tests/test_road_users.py`; the L39 test with `object_classes` stays green); the view differs only by the
  lead's class and the `road_user` objects.
* chris-integrator suite 300 passed (288 before + 12 new); api 22 passed; car repo scene tests 111 passed (the
  car repo's full suite: 5 failures, all `test_bean_*` for a fixture file that is untracked in the main checkout,
  unrelated).

## 4. The load governor

New first step `road_users` (`road_users_hz` 0, `quick`): the light provider stops decoding road users and stops
the optional road-user view (`"road_users": null`), so the integrator gets no new class votes (a track keeps the
class it has) and no new display objects. Every later level moved up by one (`preview` is 2, ..., `road` is
10); a `governor.json` that sets `max_level` 8 / 7 means 9 / 8 now (none is in the repository). Without the
optional view the step saves only the decode and what the integrator does with the boxes (section 3.6); with it,
one detector run per road-user frame. `api/roadsense_api/governor.py`, tests in `api/tests/test_governor.py`,
the table in `docs/ARCHITECTURE.md` ("Load governor").

## 5. The screen

`cyberpunk_computer/backend/roadsense_scene.py` (`view_to_scene`: `lead.k`, `objs`), `ui/scene/inputs.py`
(`road_user_kind`, `RoadObject` with fade / id / velocity), `ui/scene/director.py` (road users carried by their
velocity between samples, corrections eased like the lead; part of the redraw signature), `ui/scene/render.py`
(`draw_road_user`: car, van, truck, bus / tram, motorbike, bicycle, person; persons and two-wheelers in the
warning colour; unknown = the car shape, as before). 1 px lines and flat fills, no new text. No single-frame
on / off: objects come and go with the view's fade (`appear` after two placed detections, `coast`, `leave`); no
shape hopping: the class is a vote with a sticky label in the integrator, and the backend's confidence rule.

Screenshots (headless, `SDL_VIDEODRIVER=dummy`, `python -m cyberpunk_computer.ui.scene.demo --shots DIR`), shots
88-94 in `~/datasets/l42/shots/` (`NN_name.png` native 480x240, `NN_name@4x.png`): 88 lead truck + car + bus in
the left lane, 89 lead bus, 90 lead van + truck, 91 lead motorbike + cyclist at the edge, 92 lead cyclist, 93
pedestrians at the edge and crossing + a cyclist, no lead, 94 lead of unknown class (generic car) + van, car,
pedestrian. Shots 1-87 and the sign legend are byte-identical to `main`'s (176 files compared).

## 6. What is not verified, and what is not done

* **Nothing ran on the phone.** Decode cost, integrator cost and the screen's drawing cost on the phone and the
  Pi are estimates from laptop numbers.
* **No truck, bus or two-wheeler lead on the car's drives**: the lead's class is shown right (car, 0 wrong labels)
  and stable (0 changes) on the car's footage, but that only tests "car". The rides do not fill the gap either:
  warsaw-city has 3 light frames with a large bus box straight ahead, no sustained bus or truck lead. A lead's
  class other than car is shown working only in the simulator (`tests/test_road_users.py`: a truck lead, one wrong
  call in four, never flips).
* **Trucks and vans**: the detector's `truck` is half vans on this camera; the screen draws a vehicle class only
  from confidence 0.5, which turns most low-score truck calls into the generic car, but a van seen with a high
  truck score is drawn as a truck. No van shape will appear with this detector.
* **Near road users are not drawn** with the default configuration (no ground point below the light views,
  nearer than ~19 m). The optional road-user view would place them; it is built and unit-tested (message,
  placement in the integrator) but its accuracy and cost were not measured on the car or the phone.
* **The calibration**: placement is right only with the car's real pitch / yaw (section 3.5). On the car this
  comes from the online calibration of L32, which is not proven on the car.
* The car's recordings give class votes at about 2 a second (one JPEG a second, two views), the phone at 5.
  The full-rate runs are rides of other cameras, with the light views moved to where the car's sit relative to
  the horizon (the rides' cameras are pitched up), and with supercombo's leads (L14 outputs), not the vision
  model's.
* Labels by one labeller from small crops; "none" leads (25) include unreadable night crops.
* `train` never appeared; `tram` is drawn as a bus. Riderless bikes and motorbikes are never drawn (parked).
* The equality of tracks / lead / ego state is shown on the simulator (and by construction: annotation only), not
  on a recorded drive with an integrator before and after.
* Recordings of 2026-11-19 (the vision model with `road_model` recorded) were not available with frames; three of
  the eleven drives with frames have no lead above 0.4 (night, empty roads), so they contribute nothing to the lead.

## 7. Deployment notes for the phone (nothing was deployed)

1. roadsense-signs `l42-road-users`: build and install as usual (`scripts/install-phone.sh`); the unit in the
   branch adds `--lights-road-users` (no extra inference; the optional road-user view is not enabled). Nothing
   else changes for the lights: their messages are byte-identical apart from the new key. No model to install.
2. roadsense `v2-l42` with chris-integrator: the `road_users` box and source are in the default stack; nothing
   to configure. The integrator reads `lightd.sock` already. `ego_state.sources` and `tracks.sensors` gain a
   `roadusers` entry (witness, never fused). An older integrator ignores the new key.
3. Governor: road users are level 1 now; if the phone has a `governor.json` with `max_level`, add one.
4. The camera calibration matters for where the road users are placed: on the recorded drives (configured
   pitch / yaw 0 / 0) the distances only agree with the road model's lead after the car's measured error is put
   in (section 3.5). The online calibration of lane L32 (`calib.source: auto`) supplies it on the car when it
   runs; with `source: config` the placed road users sit about 40 % too near.
5. The car computer's `l42-road-users` (backend + screen) with roadsense `v2-l42`: `vw.lead.k` and `vw.objs`
   arrive only from an integrator that has road users; the screen is unchanged without them.
6. Optional, to measure before using: the road-user view for near road users, e.g. `--lights-ru-roi 0,0.33,1,0.95
   --lights-ru-hz 1` (one more detector run a second, about 0.1 s of two big cores, section 3.6); first thing
   the governor sheds. Its accuracy is not measured on the car beyond the scan of section 3.5.
7. Nothing of this was run on the phone; the phone numbers in section 3.6 are estimates.

## 8. Lane L41's paths

Not edited: `chris-integrator/chris_integrator/core/ekf.py`, `core/ego.py`, `core/monitors.py`, the existing
files of `sources/`, `sim/`. Edited and also touched by L41 (`v2-l41`): `core/objects.py` (only the class block,
lines 44-80 of `v2-l39`: `classify` / `class_of` now call `core/classvote.py`; L41's hunks there are elsewhere),
`plugins/estimator.py` (one line: the default sources gain `road_users`), `stack.py`, `tests/test_api.py`,
`tests/test_mapper_box.py` (plug-in order: `road_users` before the estimator). A trial merge of `v2-l41` (2392517) into this branch: no conflict.

## 9. Reproduce

Laptop; heavy runs inside `systemd-run --user -p MemoryMax=...`. `SIGNS` = roadsense-signs checkout (branch
`l42-road-users`), `RS` = this checkout; the light detector `bdd100k_yolo11n_320x960(.onnx | -ncnn.param)` as
installed by roadsense-signs `scripts/fetch-models.sh` (here `~/datasets/l26/models/`); a venv with
onnxruntime + opencv for the scan (here `~/l7/venv`).

```sh
# signd, lights_replay and the tests (ncnn from scripts/build-ncnn.sh or the L26 tree)
cmake -S $SIGNS -B build -DROADSENSE_DIR=$RS -DROADSENSE_NCNN=ON -DNCNN_DIR=~/datasets/l26/ncnn && cmake --build build -j4
build/road_users_test && build/lights_tracker_test && build/signs_tracker_test
# 3.7: light messages unchanged with the flag off (old build = main) and the light part unchanged with it on
build/lights_replay --backend ncnn-cpu --model M-ncnn.param --roi2 0.125,0.125,0.875,0.569 --thr-keep 0.12 --coast 3 [--road-users] FRAMES...

# 3.1-3.2: the detector on the recorded frames (all three views), blind sheets, labels by eye, scores
LIGHTS_DET=M.onnx THREADS=6 python $SIGNS/tools/road_users/car_scan.py car_scan.npz ~/roadsense-rec/2026111[2-8]-*
python $SIGNS/tools/road_users/sheets.py dets car_scan.npz ~/datasets/l42/dets --per-class 45       # -> labels.txt by eye
python $SIGNS/tools/road_users/sheets.py lead SESSIONS... --out ~/datasets/l42/lead --n 150        # -> labels.txt by eye
python $SIGNS/tools/road_users/score.py ~/datasets/l42/dets [--screen]

# 3.3-3.5: through chris-integrator, per session (calibration of the car per section 3.5)
python $RS/tools/road_users_eval.py car_scan.npz ~/roadsense-rec/SESSION --calib -1.3,-3.3 --labels ~/datasets/l42/lead [--no-cost] --out SESSION.json

# 3.4 full rate: a ride split to 5 frames a second, the unit's two views with the ROIs moved to the car's horizon
python -I $SIGNS/tools/road_users/mjpeg_frames.py ~/datasets/rides/warsaw-city/warsaw-city.mjpeg ~/datasets/l42/rides/warsaw-city
build/lights_replay --backend ncnn-cpu --model M-ncnn.param --roi 0,0.10,1,0.72 --roi2 0.125,0.225,0.875,0.669 --thr-keep 0.12 \
    --coast 3 --threads 4 --road-users $(cat warsaw-city.list) > warsaw-city.lights.jsonl
python $RS/tools/road_users_ride.py ~/datasets/rides-road/warsaw-city.road.jsonl.gz warsaw-city.lights.jsonl warsaw-city.list

# tests
(cd $RS/chris-integrator && PYTHONPATH=$PP $PY -m pytest -q -p no:cacheprovider tests)   # 300 passed (288 + 12 new)
(cd $RS/api && PYTHONPATH=$PP $PY -m pytest -q -p no:cacheprovider tests)                # 22 passed
(cd CAR && SDL_VIDEODRIVER=dummy .venv-fe/bin/python -m pytest -q tests/test_mfd_scene.py tests/test_roadsense_scene.py)  # 111 passed
SDL_VIDEODRIVER=dummy .venv-fe/bin/python -m cyberpunk_computer.ui.scene.demo --shots DIR   # shots 88-94
```

Data (laptop, not in any repository: the owner's frames and labels): `~/datasets/l42/` (`car_scan.npz`,
`dets/` and `lead/` sheets, samples and labels, `rides/` frames and light messages, `shots/`).

## 10. Branch heads

* roadsense `v2-l42`: the commit that adds this report (on c01ef8e, from `v2-l39` 3c5c419)
* roadsense-signs `l42-road-users`: d4314fb (from `main` 86624a9)
* car repo `l42-road-users`: 8d35397 (from `main` 748546b)

No push, no merge.
