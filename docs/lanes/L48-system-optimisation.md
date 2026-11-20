# L48: the whole system's budget and the drastic candidates

Lane L48, 2026-11-20 from 07:40 CET, Fable. Owner's order: "run a Fable optimisation lane that will look at every
part of the system (integrator, roadsense, etc.) and look for drastic optimisations in GPU/CPU". Drastic = a
whole process, pass or model removed, or an order of magnitude; percent shaving is out of scope.

Branches (nothing pushed, nothing merged, nothing installed): roadsense `v2-l48`, roadsense-signs `l48-opt`, car
repo `l48-opt`. The phone was read only (lane L46 held it): process and thread accounting from `/proc`, unit
files, journals. Everything else ran on the laptop with the repos' replay and scoring tools. Sub-lanes on Opus:
L48-E (envd, [L48-envd.md](L48-envd.md)) and L48-B (car backend, car repo `docs/lanes/L48-backend-cost.md`).
The integrator's internals are lane L46's surface (`docs/lanes/L46-integrator-cost.md` on `v2-l46`); this report
only places the integrator in the pipeline.

## 0. Result in one page

**What the system spends** (section 1, measured on the phone this morning): the little cores are 80 % busy
and envd is 62 % of that; the big cores carry 1.4 cores of load, two thirds of it the light detector; the GPU is
84-90 % busy in time (road model 59 %, sign detector 25 %). About 7 W in all.

**Found and fixed on the laptop (phone proofs ordered, nothing installed):**

| | where the waste was | change | expected on the phone | quality |
|---|---|---|---|---|
| car backend | the CAN reader made 4,500 empty USB reads a second: 38 % of a little core, two thirds of the backend | reads paced at 5 ms (car repo `l48-opt`) | backend 59 -> about 25 % of its core; in the car the reader stops falling behind the bus | frames 0-5 ms later into a state that samples at 0.1 s; tests |
| debug face | parsed every 5 kB `road` line for nobody, on a big core | parse on demand, little cores (`v2-l48`) | 5 % of a big core -> under 5 % of a little one | identical answers (tests) |

**Found, built, and the owner's call (same quality within the noise, not strictly identical):**

| | change | expected on the phone | quality after the consumers |
|---|---|---|---|
| envd | one inference in 2 s, message re-sent every second | 80-100 points of a little core | inside the 1 Hz phase noise for place, light, tunnel and all map-backed decisions; weather and the env area prior slightly above it |
| light detector | while no light is tracked, one frame period in three without a pass (`--lights-idle ZW.Z.W`) | 7-30 points of a big core (0.15-0.6 W), most on roads without lights | light-time -0.1 / +0.5 points on the reference rides against a noise of 1.4; first report unchanged |

**Tested and rejected with numbers:** one detector instead of two (the light detector's sign class instead of
the sign detector, or with the sign detector at 1-2.5 Hz: 9 % of the momentary signs lost without it, loss
outside the noise already at 2.5 Hz); envd at 5-10 s, on the GPU, int8; a binary bus, zero-copy frames, a
shared GPU pre-processing pass (each under 5 % of one little core); dropping `road` from the backend's
subscription (no effect).

**Together**, if the four changes hold on the phone: about 120-140 points of little-core time and 12-35 points
of big-core time, 0.65-1.1 W of about 7 W. That is not an order of magnitude for the phone: the remaining large
items (road model on the GPU, the light detector's passes while lights exist, the sign detector) each do work
that the scoring shows is needed. The order-of-magnitude steps left are on the demand side (section 5): the
integrator telling perception what is needed (map-gated envd, the light idle veto, a standstill mode). The
channel for it exists after this lane (`<rundir>/demand`); the writer and its proof do not.

**First on the phone:** the backend's CAN reader (largest certain saving, no quality question, and it fixes a
fault seen while driving), then the P0 baseline, then envd at 2 s and the light idle pattern side by side with
the installed configuration.

## 1. The budget (phone, measured 2026-11-20 07:55 CET, replay playlist, bench, governor level 1)

Method: two read-only snapshots of `/proc/<pid>/task/*/stat` 40 s apart (CPU time per thread, the core it was
last seen on), `/proc/<pid>/task/*/status` 20 s apart (voluntary context switches = wake-ups), the daemons'
own stats lines in the journal. GPU time per process needs root (`fdinfo`), so the GPU column is L27's
measurement of the same model and schedule, cross-checked with today's stats lines (15.3 road runs x 44 ms +
5 sign-detector turns x 45 ms = 90 % of wall). Watts: L21's calibration (a busy A75 about 2 W, a busy A55 about
0.33 W, the GPU at 90 % about 2.3 W; idle phone with the clock pins and the wake-up latency hold 1.6 W).
Scripts: section 7.

| process, thread | what it does per second | little cores (% of one) | big cores (% of one) | GPU (% of wall) | about W | wake-ups /s | who uses the output |
|---|---|---|---|---|---|---|---|
| camerad-replay (bench) / camerad-dvr (car) | 20 JPEG decodes 1280x720 -> NV12 ring (1.38 MB a frame, 27.6 MB/s written) | car: 26 (13.6 ms a frame at 19 fps, the camera message of the 2026-11-19 drive) | bench: 8.4 | - | 0.1 | 60 | every reader of the ring |
| modeld `md-prep` | reads every frame (one 1.38 MB copy each), warp + pack for both model inputs, 7 ms | 13.2 | - | - | 0.04 | 711 (1 ms ring poll) | the road model |
| modeld `md-prefetch` | records the next run's GPU commands, 5.9 ms a run | - | 11.2 | - | 0.22 | 195 | the road model |
| modeld `md-main` | GPU dispatch, policy on ORT (5 runs/s), parse, 5 kB JSON a run | - | 8.6 | 59 (15.3 runs x 44 ms) | 0.17 + 1.5 | 74 | integrator (road, lead, lanes, ego motion), debug face |
| signd `sg-signs` | 5 frames/s: letterbox, sign detector on the GPU (55 ms), classifier + verifier on ORT, tracker | - | 12.6 | 25 (5 x 45-51 ms) | 0.25 + 0.64 | 96 | integrator (sign context, rules, speed limit) |
| signd `sg-lights` | 5 frames/s, two views alternating: YOLO11n 960x320 on 2 CPU threads (87 ms), state CNN, tracker; also the road-user classes | - | 92.0 | - | **1.84** | 72 | integrator (ego light, road users, view) |
| envd | CLIP image tower once a second, 3 threads; today 1.16 s a frame (0.86 frames/s): it cannot reach its 1 Hz | **196** | - | - | 0.65 | 108 | integrator: weak built-up prior (must hold 20 s), display labels |
| chris-integrator | every source's JSON, estimator, trackers, consolidated API | 46.5 | - | - | 0.15 | 148 | car backend, MFD, recorder |
| roadsense debug face (`roadsense_api`) | parses every daemon message, runs the load governor (1 Hz) | - | 5.1 (not pinned) | - | 0.10 | - | nobody unless a browser is open; the governor file |
| car backend (`cyberpunk_computer.backend`) | CAN (panda, gateway), store, MFD scene feed, recorder, API | 61.4 (of it `PandaCanReader` 37.2, main 9.3, `roadsense-scene` 5.3) | - | - | 0.20 | 4564 (`PandaCanReader`) + 300 | the car |
| **sum** | | **317 of 400** | **138 of 400** | **84-90** | about 7 | 6300 | |

Readings:

* The little cluster is 80 % busy and envd is 62 % of that. envd no longer meets its own rate, and everything
  else on the little cores (frame prep, the integrator, the car backend) queues behind it.
* On the big cores, where the heat is, one thread is two thirds of the load: the light detector (92 % of a core,
  about 1.8 W, a quarter of the phone's power). Its 5 passes a second run whether or not a light exists; in the
  journal of this replay (a rural ride) the tracker held no light at all (`live 0`).
* The GPU is full in time and is the road model's (59 %) and the sign detector's (25 %).
* JSON between the daemons is 70 kB/s in total; 63 kB/s of it is `road` (5 kB x 12.7/s). Parsing one `road`
  line costs 0.08 ms on the laptop, of the order of 1 ms on an A55: about 1.5 % of a little core per reader.
  There are three readers of `road` (integrator, debug face, car backend).
* Frames: one decode, one writer. Every reader copies a frame out of the ring (`RingReader::next`), 43 MB/s in
  total, about 1.5 % of a little core at memcpy speed. No frame is decoded or converted twice; each consumer
  resamples its own input (road model warp 7 ms, sign letterbox, light views, envd crop) from the same NV12.
* Wake-ups: 6,300 a second, 4,560 of them one thread of the car backend (`PandaCanReader`) spinning on a device
  that answers at once with nothing.

## 2. Candidates, ranked

Savings are for the bench stack above. "Shown" = measured after the trackers / consumers on the laptop;
"phone" = needs the order in section 6.

| # | candidate | mechanism | saving expected | quality risk and evidence | state | proof still needed | unlocks |
|---|---|---|---|---|---|---|---|
| 1 | envd: far fewer inferences | one inference every 2 s instead of every second, the last message re-sent each second (`--period 2 --republish 1`); the smoother window is in seconds already ([L48-envd.md](L48-envd.md)) | half of envd: 80-100 points of a little core (about 0.3 W); the little cluster 80 % -> about 55 % busy | 21 rides (246 min) + 6 drives of the car: place, light, tunnel and every map-backed decision inside the phase noise of 1 Hz; weather / `wet` 0.3-0.4 points above it; the env area prior 0.72 % of the time against a noise of 0.17-0.37 % (2 of 84 runs against 1 of 84 with a borderline 20 s vote). **Not a clean pass.** 5 s and 10 s periods and the scene-change trigger are clearly worse; int8 loses 1-2 points; the GPU path hangs; fp16 weights give identical labels, speed on the A55 unknown | done (`v2-l48` 891dfd1), defaults unchanged | phone order L48-envd; owner's call on the area prior | the order of magnitude is on the consumer side: with a usable map env decides only the tunnel OR and `wet`, so the integrator could cap `env_hz` to one frame in 10 s (not built) |
| 2 | car backend: the CAN reader stops spinning | the panda's old firmware answers a bulk read of an empty queue at once, so the reader made 4,500 empty USB transfers a second; reads are paced at 5 ms (a full transfer is read again at once) | bench: 38 % -> about 2 % of a little core (the backend 59 -> about 25 %); in the car it should also end the backlog seen on 2026-11-19 (1,690 frames/s, the reader up to 0.66 s behind, frames dropped) | a frame reaches the state 0-5 ms later (the state takes each id every 0.1 s at most); nothing is sent on CAN. Laptop: 196 reads/s against a fake panda that answers empty at once; 3 new tests | done (car repo `l48-opt` 520d816) | phone order L48-B | cpu3, the backend's only core, out of saturation while driving |
| 3 | light detector: idle pattern | 3.2: while no light is tracked, one frame period in three gets no detector pass | 8 % (city centre) to 33 % (rural, motorway) of the light thread: 7-30 points of a big core, 0.15-0.6 W, up to about 4 K on the big-core sensors | inside the method's noise on the two reference rides (light-time -0.1 / +0.5 points, first report unchanged); not strictly identical: a light appearing in a skipped period is seen 0.2 s later. Owner's call | done (signs `l48-opt` 5438959), off by default | phone order P2 | the map veto (section 4) makes it free at mapped signals |
| 4 | integrator: per-frame cost | L46's surface (46 % of a little core, 29 ms a frame) | L46's report | L46's equivalence tooling | running (L46) | L46 | room for a higher road rate |
| 5 | debug face: no parse of `road`, little cores | 3.3 | 5 % of a big core -> under 5 % of a little one, about 0.08 W | answers identical (tests) | done (`v2-l48` f777ee6) | phone order P1 | - |
| 6 | car backend: trip-log queue, CAN log in batches | one lock / notify per CAN frame replaced by a deque (done, 7a11246); one log entry per transfer instead of one JSON line per frame (not done: changes the trip file format) | 1-2 % of a core measured on the laptop for the queue; 10-20 % of cpu3 while driving estimated for the batches | lines reach the disk up to 1 s late; the format change needs the owner's view | queue done; batches not done | a drive | - |
| - | sign detector removed or slowed, light proposals instead | 3.1 | would free 12-25 % of the GPU (3-5 road runs a second) | **fails**: momentary signs 108.5 -> 99-107, matched intervals 58.5 -> 55-56.5 on human labels | rejected with numbers | - | - |
| - | binary bus, zero-copy frames, shared GPU pre-processing, event-driven ring | 3.4 | under 5 % of one little core each | - | rejected with numbers | - | - |
| - | car backend: `road` dropped from the scene subscription | L48-B | none measured (0.94 vs 0.93 % on the laptop) | - | null, reverted | - | - |

## 3. What was measured in this lane

### 3.1 One detector instead of two: the light detector's `traffic sign` class as the sign detector (null)

The light detector (BDD100K YOLO11n) already serves three outputs from one pass: lights, the road-user classes
(L42) and, optionally, sign proposals (L24-S: `--lights-signs`). The sign detector (MTSD YOLOv8n, 384x640 on the
GPU, 5 runs a second) is the only other detector: 25 % of the GPU's time and the reason the road model runs at
15 instead of 19-20 runs a second (each detector run costs about one road run, L4). Question: can it go, or run
rarely, with the light detector's sign boxes (through the same classifier and verifier) carrying the tracker?

Tooling of L24-S (roadsense-signs `tools/signs_eval/`, caches in `~/datasets/l24s/cache`): cached detections of
both detectors on six labelled rides -> signd's own tracker (`signs_track_replay`) -> the rules engine -> L11's
human labels; and the "detector at its best" reference on three rides. New here: `variant.py --no-det` (no
sign-detector frames at all). The rules engine and its 5 Hz hit thresholds were not changed.

Human labels, six rides, per pass (the two phases of today's pipeline differ by 2 false rules, 4 missed
intervals, 1 matched interval, 1 momentary sign, 1 warning: that is the noise):

| | today: sign detector 5 Hz | 2.5 Hz + light proposals | 2 Hz + light proposals | 1 Hz + light proposals | light proposals only | (L24-S) 5 Hz + light proposals |
|---|---|---|---|---|---|---|
| false rules | 24 | 28 | 26 | 26.5 | 25 | 27 |
| missed intervals of lasting rules | 38 | 38.5 | 40 | 38.5 | 37.5 | 37 |
| matched intervals | 58.5 | 56.5 | 55 | 55 | 55.5 | 58 |
| momentary signs found (of 209) | 108.5 | 107 | 105 | 102 | 99 | 114 |
| warnings found (of 61) | 36.5 | 37 | 36 | 35.5 | 34 | 38 |
| start of a lasting rule against the label, median s | -0.45 | -0.75 | -0.7 | 0.0 | -0.05 | -1.0 |
| GPU time of the sign detector | 25 % | 12.5 % | 10 % | 5 % | 0 | 25 % |

Reference (three rides, 942 reference signs; share listed at least once / share of sign-time listed for signs
of 24 px and more): today 0.747 / 0.67-0.86 by ride; light proposals only 0.722 / 0.58-0.71; 1 Hz + proposals
0.766; 2.5 Hz + proposals 0.790. The light views see small signs better (12-16 px sign-time 0.26-0.36 against
0.04-0.09) and lose large, near signs: each view is looked at 2.5 times a second and the zoomed one does not
cover the sides of the frame.

Verdict: **not the same quality.** Without the sign detector 9 % of the momentary signs (pedestrian crossings,
no-entry, turn bans) and 3 of 58 matched intervals are lost; the loss grows steadily as the detector's rate
falls and is already outside the noise at 2.5 Hz (matched -2, false +4). The sign detector at 5 Hz does work
nothing else does. Kept as it is. (The direction that did gain in L24-S, proposals in addition to the 5 Hz
detector, costs no detector run and is still an open A/B from that lane.)

Reproduce: `EVAL_DIR=~/datasets/l48/eval tools/signs_eval/eval_variant.sh lo "0 1" "--no-det --lights LW,LZ"`,
`... l2 "0 2" "--hz 2 --lights LW,LZ"` (roadsense-signs `l48-opt`), then `compare_rules.py`.

### 3.2 The light detector while no light is tracked

The light thread is the largest single consumer of the big cores (92 % of an A75, about 1.8 W). It makes 5
detector passes a second (wide view, zoomed view, alternating) at all times. Change (roadsense-signs `l48-opt`,
5438959): `signd --lights-idle P`. While the light tracker holds no track at all (not even an unconfirmed one,
so nothing ages, coasts or needs its state read), the frame periods follow a pattern instead of wide, zoom,
wide, ...: `W` = wide pass, `Z` = zoom pass, `.` = no pass. The first box of any score that may start a track
brings the full schedule back with the next frame. Off by default; `<rundir>/demand` `lights_idle: 0` forbids
it (section 4). `lights_replay --idle P` runs the same schedule offline.

How often nothing is tracked (first 5 minutes of each ride, share of the frame periods):

| ride | warsaw-city | krakow-lights | krakow-night-tunnel | a1-night | rural-zloczew |
|---|---|---|---|---|---|
| no track alive | 23 % | 50 % | 63 % | 90 % | 99 % |

Quality after the tracker (L15's method: `replay_score.py` against the detector at its best, every 4th frame of
the first 5 minutes, ORT on the laptop). The noise of the method: the same schedule started with the other view
(zoom, wide, ...) scores 96.2 % instead of 97.6 % on the night ride (1.4 points) and, by day, the same 97.1 %
with 93.8 % instead of 94.5 % at 24-48 px and a first report at 0.50 s instead of 0.40 s.

| schedule while idle | passes kept while idle | krakow-lights (day): light-time, 24-48 px, first report | passes saved | krakow-night-tunnel: light-time, 12-24 px | passes saved |
|---|---|---|---|---|---|
| today (wide, zoom, ...) | 5 /s | 97.1 %, 94.5 %, 0.40 s | 0 | 97.6 %, 96.6 % | 0 |
| `ZW.Z.W` | 3.3 /s | 97.0 %, 93.4 %, 0.40 s | 16.5 % | 98.1 %, 97.4 % | 20.9 % |
| `WZ.WZ.WZ` | 3.75 /s | 97.0 %, 94.0 %, 0.40 s | 12.6 % | 98.1 %, 97.4 % | 15.6 % |
| `Z.ZW` | 3.75 /s | 96.3 %, 90.5 %, 0.50 s | 12.5 % | 97.1 %, 95.7 % | 15.9 % |
| `W.Z.` | 2.5 /s | 96.8 %, 92.9 %, 0.50 s | 25.4 % | 95.7 %, 94.4 % | 31.3 % |
| `Z.W.` | 2.5 /s | 96.7 %, 91.9 %, 0.40 s | 24.7 % | 96.2 %, 95.3 % | 31.4 % |
| `Z.Z.Z.ZW` | 2.5 /s | 93.9 %, 75.9 %, 0.60 s | 19.6 % | 96.6 %, 95.7 % | 24.1 % |
| `Z.` (zoom only) | 2.5 /s | 89.5 %, 51.5 %, 1.90 s | 27.6 % | 96.2 %, 94.9 % | 32.5 % |
| one pass in 3 / in 5 periods | 1.7 / 1 /s | 95.4 % / 93.7 %, first 0.7 / 0.9 s | 33.7 / 41.6 % | 96.6 / 96.4 % | 44.1 / 53.3 % |

Readings:

* `ZW.Z.W` (one period in three skipped while idle) is inside the noise on both rides: light-time -0.1 points by
  day, +0.5 by night, first report unchanged, all 30 + 17 reference lights found. It keeps 3.3 messages a
  second, above the integrator's floor for `lights` (3 a second), so the topic is not marked slow.
* Skipping every other period costs 0.3-0.4 points by day and 1.4-1.9 by night (at the edge of the noise) and a
  first report 0.1 s later; it also falls under the 3-a-second floor. Not proposed.
* The wide view matters as much as the zoomed one for the first sighting (zoom-heavy patterns lose most), so
  there is no free pattern beyond a third: the detector is the sensor, and a pass not made is a first sighting
  up to one period later.
* Saving with `ZW.Z.W`: a third of the light passes while idle, i.e. 8 % (city centre) to 33 % (rural, motorway)
  of the light thread: 7 to 30 points of a big core, 0.15 to 0.6 W. By L21's envd move (61 % of a big core =
  8 K on the big-core sensors) up to about 4 K where no lights are. The road-user classes and sign proposals
  ride on the same passes and thin out the same way while no light is tracked (display only / not in use).
* Not "the same quality" in the strict sense: it is the same within the method's noise. Owner's call; phone
  proof in the order (P2). signd was run with the option against a replayed ring on the laptop (3.7 passes a
  second with `Z.ZW` on a ride without lights, as expected).

### 3.3 The debug face parsed every road line for nobody (done)

`roadsense_api` (the debug face on 8091, also the load governor's host) parsed every message of every daemon:
since the chris layout the integrator reads the same sockets itself, so the 5 kB `road` line was parsed twice
(three times with the car backend), and the face ran unpinned (seen on an A75, 5.1 % of it). Change on
`v2-l48` (f777ee6): lines starting with `{"type":"road",` are counted, timed and relayed as text and parsed on
the first GET (`Latest.get`); the unit is pinned to the little cores. Message handling over a recorded stream
(15,178 messages of the 2026-11-19 drive): 1.88 s -> 0.13 s on the laptop (14.7x); answers identical (23 tests
of `api/tests`, one new). Expected on the phone: 5 % of a big core -> under 5 % of a little core, about 0.08 W;
to be confirmed in the order (P1).

### 3.4 Formats, copies, shared pre-processing (measured, rejected)

| idea | what it could save at most | verdict |
|---|---|---|
| binary bus (msgpack / flatbuffers / shared memory) instead of JSON lines | parse + encode of 70 kB/s: about 1.5 % of a little core per `road` reader, 3 readers; laptop 0.08 ms parse and 0.17 ms encode per `road` message | **rejected**: under 5 % of one little core in total, and `nc -U` debugging is lost. The readers that should not parse at all were the waste (3.3, and the car backend's scene thread, L48-B) |
| zero-copy frame path | the readers' copies out of the ring: 43 MB/s, about 1.5 % of a little core | **rejected**: the copy is what makes a lapped slot detectable without locking |
| one shared GPU pre-processing pass for all consumers | modeld's prep 13 % of a little core, the detectors' letterbox a few ms on a big core 10 times a second | **rejected**: frames are born on the CPU (USB JPEG, software decode), so a GPU pass adds an upload; the GPU is the resource that is full in time; each consumer needs a different geometry (warp by calibration, letterbox, two light views) |
| event-driven ring instead of 1 ms polling | `md-prep`'s 711 wake-ups a second, about 1-2 % of a little core | not drastic; not done |

## 4. The integrator in the pipeline

* **Where it sits.** The integrator is the one consumer of the raw topics that matters. It runs its plug-ins once
  per `road` message: 29 ms of an A55 per frame at 15 frames a second = 46 % of a little core (L45; 45.8-46.5 %
  today). Its cost scales with the road rate, so any gain in road runs a second (a freed GPU) is paid here
  unless L46 lowers the per-frame cost. Its internals are L46's surface and were not profiled again.
* **Readers of `road`.** Three processes parsed the same 5 kB line: the integrator (needs it), the debug face
  (did not: fixed, 3.3) and the car backend's scene thread, which subscribes to `road` and nine more topics on
  the integrator's `/ws` (L48-B).
* **What it could command.** The integrator knows what perception cannot: the map ahead, the car's speed and
  standstill, which estimates are uncertain. Today perception runs flat out regardless. The lane adds the
  channel (`<rundir>/demand`, docs/ARCHITECTURE.md "Demand hints": same format and reader as the governor's
  file, absence = no hint) and its first key on the perception side (`lights_idle`, 3.2). The writer is not
  built: it belongs in the integrator next to the map box, and neither the laptop nor the bench has a recording
  with both a map position and lights to prove it on. Specification for the lane that builds it: a plug-in
  reading `map` (`ahead.features` of kind `traffic_signals` / `crossing_signals` with `dist_m`,
  `ahead.path_confidence`) and the ego speed; it writes `lights_idle: 0` (valid 3 s, rewritten every second)
  while a mapped signal lies within max(150 m, 8 s of travel) on the likely path, or while the path is uncertain
  at a junction; otherwise it writes nothing. Lights the map does not know (roadworks) are still found by the
  idle pattern itself, at its small delay.

## 5. Speculation and open decisions (not proven here)

Not proven in this lane; each needs the stated proof before it is believed.

| idea | mechanism | what it could give | why it is speculation | proof it needs |
|---|---|---|---|---|
| standstill mode (demand keys `signs_hz`, `road_hz`) | with the car's `standstill` signal: no new sign can appear (tracks hold), the road model's lanes do not change; the sign detector pauses, the road model drops to 5 runs a second, lights keep their rate (waiting at a red light is where they matter) | while standing: GPU 84 -> about 25 % (about -1.5 W), `sg-signs` off | the lead's distance would be 200 ms old instead of 67 ms while standing (pulling away is a lead event); the transition back must take one frame, the caps file is re-read only twice a second; how much of a drive is standstill is not measured (the 2026-11-19 recordings carry no vehicle feed) | owner's decision on freshness at standstill first; then a replay with `road` thinned while v = 0 and the lead tracker's departure delay compared |
| map veto of the light idle pattern | section 4 | makes 3.2 free at mapped signals | no recording with map and lights; OSM signal completeness on the owner's roads unknown | a drive with position, lights listed against the map's signal distances |
| the wide light view on the little cores | after envd and the CAN reader leave them (about 2 cores free): the wide view (2.5 passes a second) on 3 little threads, the zoom view stays on the big cores | big cores -46 % of a core, little +145 %; by L21's envd move (61 % of a big core = 8 K on the big-core sensors) about 6 K | wide-view latency about 200 ms instead of 90; two threads feeding one tracker in order; depends on the envd and backend results holding on the phone | phone only, after the three orders |
| one trained multi-task detector (signs, lights, road users) | a single network at one resolution replaces two detectors | either the sign detector's 25 % of the GPU or the light detector's 92 % of a big core | the existing BDD model's sign class is not enough (3.1); needs training on MTSD + BDD100K (research licences), a new scoring campaign for three outputs | weeks; not an optimisation lane |
| single-input student of the road model | the v0.11.1 vision model takes a narrow and a wide picture; with one camera both are warps of the same frame | part of the 44 ms a run | needs distillation; int8 of the road model was rejected for sensitivity, so a student is a research risk | training infrastructure; not now |
| turnip without `flushall` | the driver work-around costs about 5 ms a run (11 % of the road model's GPU time, 7 % of the GPU) | 1-1.5 road runs a second | the corruption it fixes was found on Mesa 26.2, which is still installed | re-test (`modeld_replay --ncnn-no-flushall`, 200 runs on one input, outputs bit-identical) at the next Mesa upgrade |

## 6. Orders for the phone

* `orders/L48-phone.md` (this lane): P0 baseline budget on three rides; P1 debug face; P2 light idle pattern
  (budget on three rides, light-time on the phone's own backend); P3 the wake-up latency hold with the new road
  model, measurement only, for the owner (L21: 0.34 W and 8 K on the big-core sensors for 0.5 road runs a second
  and 30 ms).
* `orders/L48-envd-phone.md` (L48-E): envd.
* car repo `docs/lanes/L48-B-order.md` (L48-B): the CAN reader on the bench (pass: `PandaCanReader` at most 4 % of
  a core at 150-260 wake-ups a second, the backend at most 25 %, the panda connected, its frames arriving) and in
  the car (frames a second as before, largest batch under 200, no backlog drops); `BACKEND_PANDA_POLL_MS=0` is
  the old loop and the rollback.

## 7. Reproduce

* Budget (phone, read only): `ssh HOST sh -s 40 < tools/budget/snap_ro.sh > snap.txt && python3
  tools/budget/snap_ro.py snap.txt`; daemon stats: `journalctl -u roadsense-modeld -u roadsense-signd -u
  roadsense-envd -n 6 --no-pager`.
* Message sizes and rates: the recorded drive `~/datasets/roadsense-rec/20261119-161325/messages.jsonl.gz`
  (count and bytes by `type`; `json.loads` / `json.dumps` over 2,000 `road` lines).
* Sign variants (3.1): roadsense-signs `l48-opt`, `tools/signs_eval/eval_variant.sh` with
  `EVAL_DIR=~/datasets/l48/eval`, variants `lo` (`--no-det --lights LW,LZ`), `l1`, `l2`, `l25` (`--hz N --lights
  LW,LZ`), baseline `r5` and `ls` of L24-S in `~/datasets/l24s/eval`; `compare_rules.py`; reference:
  `score_ref.py` on `~/datasets/l48/v/`.
* Light variants (3.2): `~/datasets/l48/lights/var.sh`, `var2.sh`, `var3.sh` (roadsense-signs `l48-opt`
  `lights_replay --idle P`, build `~/datasets/l48/build-signs`), `tools/lights/replay_score.py` against
  `~/datasets/l24s/lights/ref_*`.
* Debug face (3.3): `python -m unittest discover -s api/tests`; the timing loop is in the commit message of
  f777ee6.
