# L39: chris-integrator as a mixer; the consolidated side moved; the map as its own box

Lane L39, 2026-11-19, laptop only (phone and car read-only; nothing deployed, the phone was not reachable).
Branch `v2-l39` from `v2-l38` (75285f9) with `main` merged first (L36's turn-arrow fix in `roadview.py`; no
conflict). Lane L41 (estimation quality) runs in parallel on `v2-l41`; section 8 says what of its paths was
touched (nothing).

## 0. Result in one page

**Done and working** (tests: 288 in `chris-integrator/tests`, 22 in `chris-mapper/tests`, 21 in `api/tests`,
81 in `mapd/tests`; before: 27 / - / 262 / 94):

* **The mixer is code** (`chris-integrator/chris_integrator/mixer/`, about 600 lines that know no sensor,
  tracker or consumer): input channels, a bus of named streams, plug-in transformers with declared inputs and
  outputs, output channels. The stack is a configuration of it (`stack.py`).
* **The consolidated side lives in chris-integrator**: the trackers and decision logic, the HTTP / WebSocket
  API with the `/v1`, `/v2`, `/ws` shapes of api 2, the recorder, the replay pipeline, with their tests.
  chris-integrator imports nothing from roadsense and reads only its sockets. roadsense's api is the debug
  face of one camera.
* **The map is its own box**, `chris-mapper/` (top-level, own tests, README, licence, `pyproject`): in =
  the integrator's `ego_state`, out = `map`. It holds no position filter. In process or as its own process
  behind the same contract, with the same output message for message.
* **Object class slot**: `tracks[].class`, `lead.class`, `view.objects[].cls` with confidence and source;
  two inputs feed it; nothing reads it.
* **Equivalence**: 462 922 outgoing messages of six recordings through the old api state and through the
  mixer: byte-identical on five sessions; on the sixth one junction cue differs in `rules`, which is a bug of
  the old code found by this comparison (3.1) and fixed.
* **Cost**: 3.34 ms per road frame for the mixer against 3.33 ms for the hand-wired api state (same session,
  each alone). The out-of-process hop: 0.40 ms per message for the map box.
* **End to end** on the laptop: a session replayed on real sockets as a roadsense instance and the car's
  feed -> a real chris-integrator process -> the car computer's own `RoadsenseSceneFeed` (unmodified) on the
  WebSocket: scenes at 8.9 per second, `road` line in to `view` line out 7 ms (p50), section 3.5.

**Not working, or worse than before, stated plainly** (section 5):

* **"Position once" is not shown to be no regression on the bad-reception drive (113201)**: the estimator's
  position is 52 m (median) from held-out fixes there, the legacy mapd filter's 24 m; the box then matches
  worse (ambiguous 14 % against 7 %, the two agree on the road in 54 % of the steps). On the three other
  drives the two are about level in position and the box sits nearer its road with higher confidence, with
  3-5 points less availability. There is no ground truth for the road on real drives.
* A simulated city drive with a 2 s receiver lag: the box is on the driven route in 95.2 % of the steps
  against 99.5 % for the legacy engine, with a dead-reckoning excursion of 180 m (p90 of 42 steps).
* A receiver *without* delay on a motorway (simulation): the estimator's delay prior (2 s) is not unlearned at
  constant speed: 46 m against the legacy filter's 4 m. (L38's 5.1; the estimator is L41's.)

**Not done** (section 4): the vehicle input store (validation, per-signal quality, description, error replies,
`reset`) was not carried into the integrator's feed; nothing on the phone; the units' integration with
`roadsense.target`; a second roadsense instance's pose as a source.

## 1. The structure in a page

Full text: `chris-integrator/docs/DESIGN.md` section 14 (the mixer), 15 (the map box), 16 (classes); the
README has the public summary and "Adding a box".

```
 INPUT CHANNELS                     BUS                                              OUTPUT CHANNELS
 roadsense (one per camera):        named streams; an envelope = stream, message,    api      HTTP /v2 (+ /v1), WebSocket /ws
   road calib camera env            time of validity (ts_mono_ns), arrival time,     socket   integrator.sock (JSON lines,
   lights_raw signs_raw             producer, one serialised line                             {"subscribe": [...]} narrows)
   display_lanes governor                                                            recorder
 vehicle (feed.sock = position.sock,   PLUG-INS, order derived from the declarations
   POST /v2/vehicle):                   0 ride_watch    camera -> reset of every plug-in
   ego vehicle gnss objects             1 estimator     road, ego, vehicle, gnss, objects, calib, map(feedback) -> ego_state, tracks, lead
 map_in (only without the map box)      2 mapper        ego_state -> map                               [chris-mapper]
                                        3 lanes         road, ego_state, map -> lanes
                                        4 egolane       lanes, map -> egolane (wiring)
                                        5 roadgeom      lanes, map -> roadgeom (wiring)
                                        6 ego_light     lights_raw, road, lanes, map -> lights
                                        7 sign_context  signs_raw, lead, lights -> signs
                                        8 view          road + all of the above -> view
                                        9 rules         road, signs, env, lights, ego_state -> speed_limit, rules
                                       10 fused         map, env, speed_limit, lanes, egolane, lead -> fused
```

**Bus.** Streams are declared by their producer with a kind (`raw`, `measurement`, `estimate`, `decision`,
`context`, `display`, `status`), a time semantics (`frame`, `sample`, `periodic`, `event`) and `public` or
wiring. One producer per stream. The bus clock is the only "now" (`LiveClock` on the car, `MessageClock` in a
replay: the replay is deterministic and runs the car's code).

**Plug-in contract** (`mixer/plugin.py`). A plug-in declares `name`, `inputs` (`In(stream, trigger, optional,
feedback)`), `outputs` (`Out(stream, kind, time, public, doc)`), `DEFAULT_CFG`; implements
`on_input(stream, msg, ctx)`; reads with `ctx.get(stream, frame=True|False)` (frame = only if of the same
camera frame), writes with `ctx.emit`. Undeclared streams are refused. Health: `health()` plus what the
mixer measures (calls, errors, cost, age of the last output, unsatisfied inputs); an exception is caught,
counted, and does not stop the others. `reset`, `close`, `state_dump` / `state_load`. Order: producers before
consumers, stable; loops only through `feedback` inputs; never re-entered. Added / removed / replaced /
disabled at run time (`mixer.add`, `remove`, `replace`, `enable`) or in `mixer.json`. Alone:
`mixer.testing.Bench`. Out of process: `mixer/remote.py`, JSON lines over a pipe or a socket, sync (waits
for `done` up to a deadline; deterministic) or async (published when it answers); a silent plug-in is a
`fault`, is restarted, and its frame goes on without it.

**What L38 built sits in it unchanged.** The estimator plug-in is `core.Integrator` (source registry, EKF,
monitors, object layer); its inputs are the message types its registered sources read. A sensor is still
added as a source (file + configuration line); a box is added as a plug-in.

**The map box.** Decision: its own top-level directory `chris-mapper/` (package `chris_mapper`), not a
package inside the integrator, because it has its own data and tooling (pack format, pack builder, the map
data's licence), its own heavy state (tile cache: memory and I/O a host may want in another process), its
own test world, and is useful with any pose source. The integrator keeps a 50-line plug-in face
(`plugins/mapper.py`). In: `ego_state` (position with covariance and mode, heading with std, speed, odometer,
time). Out: `map` every 0.45 s: `match` (geometry of the map: the estimator's map source takes it as
cross-track = 0 and heading = the road's bearing), `position` (the pose that was used), `road`,
`speed_limit`, `ahead` (features, junctions, curves, limit changes, turn lanes, `junction_view`). The pack
format and `scripts/roadsense-maps` are unchanged (the builder imports `chris_mapper.rsmap`; `info` checked on
the Poland pack).

## 2. What moved where

| was | is | note |
|---|---|---|
| `api/roadsense_api/{lanes, egolane, roadgeom, roadview, view, speedlimit, rules, lights, signctx, fusion, lead}.py`, `rules/pl.json` | `chris-integrator/chris_integrator/trackers/` | `git mv`; imports only |
| the wiring in `api/roadsense_api/state.py` (`State.on_message`, `_on_frame`, `_on_speed`, `_fused_tick`, sign tracks, ride change, rate markers, carried faults) | `chris_integrator/plugins/{estimator, trackers, common, mapper}.py`, `stack.py` | each call became a plug-in with declared streams |
| `api/roadsense_api/server.py` (routes, `/ws`, status page), `State.health()` | `chris_integrator/io/http.py`, `io/api.py` | same paths and shapes; `/v2/mixer`, `/v2/calib`, `/v2/gnss` added |
| `api/roadsense_api/recorder.py` | `chris_integrator/io/recorder.py` | an output channel; config `<chris config dir>/recorder.json` |
| `api/roadsense_api/integrator/` (the shim), `CHRIS_INTEGRATOR_SOCK` daemon mode | gone | the direction of the dependency is reversed |
| `tools/replay_lib.py` | `chris_integrator/replay.py` (`tools/replay_lib.py` is a shim) | `Pipeline` is a mixer with the session as its input channel; outputs identical to the old one (3.2) |
| `overlay.Projector` (used by lights and sign context) | `chris_integrator/trackers/projection.py` | 30 lines of pinhole projection, duplicated: the drawing stays in roadsense |
| `chris_integrator/daemon.py`, `feed.py` | `daemon.py` on the mixer, `io/feed.py`, `io/sockets.py` | `feed.sock` also as `position.sock`; `POST /v2/vehicle` |
| `mapd/roadsense_mapd/{geo, rsmap, matcher, context, sim}.py`, `limits/`, three test files | `chris-mapper/chris_mapper/`, `chris-mapper/tests/` | plus `box.py`, `__main__.py` |
| `mapd/roadsense_mapd/{engine, posfilter, vehicle, __main__}.py` | stay: the **legacy** daemon | uses chris_mapper (aliased modules); not part of `roadsense.target` any more |
| 17 test files of `api/tests` | `chris-integrator/tests/` | `test_api.py` now runs fake daemons -> debug face -> chris-integrator |
| `api/tests/test_map_e2e.py` (mapd + api as processes) | removed | replaced by `tests/test_mapper_box.py` and the map part of `test_api.py`; its checks of `reset` over HTTP and of the `vehicle` topic's signal table are not carried over (section 4) |

roadsense keeps `api/roadsense_api/{state, server, sockets, overlay, governor}.py` (api 3: raw topics,
`/preview`, the frame ring, the governor; `api.sock` publishes the `governor` message; the consolidated topics
answer 410). Default port 8081, the unit binds loopback.

Changes of behaviour a consumer could notice (all else is byte-identical, 3.1):

* `/preview` no longer draws the HUD lines IN EFFECT / RECENT / AHEAD / MAP (they came from decided topics;
  the status page of the consolidated API shows them as text).
* The camera's intrinsics for the light / sign projection come from the `calib` message (focal, centre, size)
  instead of the frame ring's header. On the recordings `calib` carries 975 / 640 / 360 / 1280 x 720; that
  the ring on the phone says the same was **not checked** (phone unreachable).
* `health.daemons` lists the roadsense instance's five daemons and `mapd` (= the map box, present while its
  stream exists); `health.api.server = "chris-integrator"`; `health.mixer` added.
* `map.input`: `pose: "ego_state"`, `motion.source: "integrator"`; `gnss` carries `source`, `fix`, `hacc_m`,
  `age_s` (no counters); `map.position.dr_distance_m` is null; `map.proc` is gone.
* `tracks[].class`, `lead.class`, `view.objects[].cls` added (null without a detector).

## 3. Measurements

Commands in section 7. Laptop, heavy runs in `systemd-run --user -p MemoryMax=...`, `PYTHONHASHSEED=0`
(one message of the old code depends on set order otherwise).

### 3.1 The consolidated side before and after, message by message

`tools/mixer_equiv.py`: the session's inputs in recorded order through (a) the old api's `State` (tree =
`v2-l38` + `main`, i.e. the first commit of this branch) with its clock replaced by the messages' time, (b)
this branch's stack on a `MessageClock`; every outgoing message of `ego_state`, `tracks`, `lead`, `lanes`,
`view`, `speed_limit`, `rules`, `fused`, `lights`, `signs`, `map` compared as serialised lines, and the order
of the whole stream. `map` is the recorded one on both sides here (the box is 3.3). At the branch head the
comparison drops the keys `class` / `cls` (added by this lane, null on these recordings); at the commit
before the class slot (0b949c0) the same runs were compared without dropping anything, with the same result.

| session | messages compared | result |
|---|---|---|
| 065629 (camera only) | 19 872 | identical, same order |
| 113201 | 61 396 | identical, same order |
| 151146 (incident) | 110 359 | identical, same order |
| 153636 (incident) | 45 279 | identical, same order |
| 161325 (incident, camera only) | 49 434 | identical, same order |
| 104128 | 176 582 | ten topics identical (174 363 messages); `rules` differs from message 651 of 2220 |

**The `rules` difference is a bug of the old code.** The rules engine de-duplicated junction cues by
`id(dict)`; an address is reused after an old cue is dropped, and a new cue at a reused address was taken
for one already seen. Which cue is lost depends on the process' allocations: at commit 0b949c0 the *new*
process lost one on 151146 (junction count 5 against 6) and the *old* one lost one on 104128 (7 against 8:
the cue of sign track 174, D-1 / D-2, which `speed_limit` did register in both). Keyed by content since
commit 5b04fc8: 151146 became identical; on 104128 the mixer's stream has the cue the old api dropped.
Evidence that it is this and nothing structural: the ten other topics, including `speed_limit` (the engine
that feeds those cues), are identical on both sessions.

Against `v2-l38` itself (without `main`): only `view` differs (`view.road`: L36's turn arrows), as expected.

### 3.2 The replay pipeline

`chris_integrator.replay.Pipeline` (now a mixer) against the old `tools/replay_lib.Pipeline`, 3000 road frames
of 104128, a hash per topic over every returned message: identical for all 18 topics
(`scratch pipe_cmp.py` in section 7).

### 3.3 The map box against the legacy mapd engine

`tools/mapper_eval.py`. "legacy" = `mapd/roadsense_mapd/engine.py` with its own position filter, default
configuration; "box" = the default stack with chris-mapper in process.

**Recorded drives with the car's receiver**; every second fix held out of both; car moving (>= 3 m/s). A
fix of this receiver is about 2 s old at its stamp (L38 3.5), so the row "2 s before" is the fair one; "0 s"
rewards a filter that follows the fixes as they come.

| session | system | map available | confidence | ambiguous | on service / track | distance to the matched road p50 / p90 | held-out fixes: distance to the position 0 s / 2 s before the stamp, p50 (p90) |
|---|---|---|---|---|---|---|---|
| 104128 | legacy | 98.2 % | 0.62 | 8.9 % | 13.7 % | 8.8 / 24.8 m | 11.9 (48.8) / 28.8 (57.4) m |
| | box | 95.2 % | 0.84 | 5.1 % | 3.8 % | 2.2 / 13.4 m | 20.7 (66.9) / 18.5 (66.5) m |
| 113201 poor reception | legacy | 93.3 % | 0.56 | 6.7 % | 36.8 % | 12.4 / 22.1 m | 19.4 (66.7) / 24.1 (69.1) m |
| | box | 88.0 % | 0.68 | 14.0 % | 19.6 % | 4.0 / 32.3 m | **53.4 (134.2) / 51.9 (119.6) m** |
| 151146 | legacy | 98.6 % | 0.61 | 2.2 % | 5.2 % | 10.4 / 25.6 m | 5.9 (16.0) / 15.9 (35.4) m |
| | box | 95.4 % | 0.90 | 5.2 % | 6.9 % | 2.0 / 6.4 m | 17.7 (28.2) / 14.3 (26.3) m |
| 153636 | legacy | 100 % | 0.78 | 1.3 % | 0 % | 5.0 / 20.8 m | 4.4 (23.5) / 11.5 (28.5) m |
| | box | 96.6 % | 0.96 | 0 % | 0 % | 2.9 / 5.9 m | 14.7 (24.7) / 10.8 (17.5) m |

The two agree on the road (name, ref, class) in 73.9 % / 53.7 % / 87.7 % / 99.7 % of the steps where both
have one (same edge 59.1 / 20.4 / 76.2 / 88.7 %). Receiver delay state at the end: 1.58, 3.06, 1.54, 1.81 s.

Reading. (1) On three drives the estimator's position is as near the held-out fixes as the legacy
filter's once the receiver's age is allowed for (10.8-18.5 m against 11.5-28.8 m), with a better tail.
(2) On 113201 it is twice as far: a regression on that drive, the estimator's known weak case (L38 5.2:
3-5 satellites; position declared unknown much of the time), now visible in the map as well. (3) The box
sits nearer its matched road with higher confidence; part of that is the loop (the match is a measurement of
the estimator), so it is consistency, not independent evidence of the right road. (4) Availability is 3-5
points lower: the estimator says `unknown` / `no_source` where the legacy filter kept dead-reckoning.
(5) Which system is on the right road where they disagree is **not established** (no truth on real drives).

**Simulated drives on the Poland pack with truth** (`synth`; the routes of `tools/synth_drive.py`; on route
= the matched edge is on the driven route):

| drive | receiver lag | system | on route | position error with a fix p50 / p95 | dead reckoning p50 / p95 |
|---|---|---|---|---|---|
| krakow-city 4.2 km | 2 s | legacy / box | 100 % / 100 % | 25.3 / 38.5 m, 1.2 / 4.4 m | - , 1.6 / 3.2 m |
| opole-rural 30 km | 2 s | legacy / box | 100 % / 100 % | 43.4 / 52.9 m, 2.1 / 6.1 m | |
| warsaw-city 5.4 km | 2 s | legacy / box | 99.5 % / **95.2 %** | 24.7 / 31.2 m, 2.4 / 6.2 m | - , 5.8 / **179.5 m** (42 steps) |
| warsaw-s2-tunnel 9.5 km | 2 s | legacy / box | 99.3 % / 100 % (in the tunnel 100 % both) | 53.5 / 60.8 m, 2.8 / 5.9 m | 22.4 / 35.1 m, 12.4 / 21.9 m |
| krakow, opole, warsaw-city | 0 s | legacy / box | 100 % / 100 % each | 3.1-3.7 / 6.6-7.7 m, 1.6-2.0 / 6.0-7.0 m | |
| warsaw-s2-tunnel | 0 s | legacy / box | 99.3 % / 99.6 % | 3.9 / 7.6 m, **46.0 / 67.9 m** | 15.1 / 25.4 m, **52.8 / 61.7 m** |

Reading: with a receiver that lags as the car's does, the legacy filter is a lag behind (25-53 m), the
estimator is within 1-3 m; the map match hides that for the legacy engine on a straight road (on route 99-100 %).
The box loses 4 points on route in the simulated city and has one dead-reckoning excursion there (not looked
into). With a receiver without lag on a motorway the estimator's delay prior is its error.

In the test suite (`tests/test_mapper_box.py`, synthetic network, 2 s lag): map available and on the main
road throughout, position 5.5 m p50 / 9.0 m p95 against truth, the tunnel covered by dead reckoning with the
match used as a measurement 578 times.

### 3.4 A plug-in added and removed without touching the core

With a real one, the map box:

* **Added**: commit 0b949c0 adds `plugins/mapper.py` (52 lines), a test, and 12 lines in `stack.py` (the
  default wiring) and 9 in `daemon.py` (an option). Nothing in `mixer/`, `core/`, the other plug-ins or
  `io/` changes: the estimator already declared `map` as a feedback input and every consumer of `map` reads
  the stream whoever produces it. (`stack.py` is the stack's wiring list, not the core; from `mixer.json` no
  file changes at all: `{"plugins": [{"class": ...}]}`, test
  `test_a_plugin_from_the_configuration_and_one_left_out`.)
* **Replaced**: the box in process and as its own process (`--mapper box` / `--mapper remote`, 113201): 60 847
  outgoing messages identical, same order, 0 timeouts.
* **Removed**: `--mapper none`: the stack runs, `map` and `fused.map` are absent, the estimator has no map
  rows (its outputs differ, as they must); at run time `mixer.remove("mapper")` (test). The recorded `map`
  as an input channel instead (`external_map`) is what 3.1 ran with.

### 3.5 Cost and hops

| | value |
|---|---|
| the old api state, 113201 (9745 road frames, all inputs), alone | 3.329 ms per road frame |
| the mixer, same session, alone | 3.338 ms per road frame |
| per plug-in call: estimator 1.01 ms, lanes 0.36, roadgeom 0.09, view 0.10, rules 0.07, sign_context 0.05, ego_light 0.03, egolane 0.02, fused 0.02 | (estimator 60 % of the total) |
| + the map box in process | + 0.33 ms per pose (3.74 ms per road frame in all) |
| the map box out of process, sync | 0.74 ms per pose seen from the mixer (wait 0.66 ms mean, 15 ms max): **the hop costs 0.40 ms**; the main process then 3.56 ms per road frame |
| end to end, real sockets, real pace (below) | see the table |

End to end (`tools/stack_e2e.py`, 104128 from 900 s for 240 s, 2938 road frames; the recording's clocks moved
onto the laptop's by one offset):

| | map box in process | map box as its own process (sync) |
|---|---|---|
| scenes built by the car's `RoadsenseSceneFeed` | 2138 = 8.9 per second (its cap is 10) | 2148 = 8.9 per second |
| scenes with a lead / a lane line / a speed limit | 23.8 % / 36.9 % / 90.1 % | 23.7 % / 37.0 % / 90.1 % |
| scenes with the map layer in use (`map.ok`) / with the road picture (`geo`) | 68.8 % / 21.3 % | 68.9 % / 21.5 % |
| display channel in use (`vw`), lead source `tracked` | 99.8 %, 99.9 % | 99.8 %, 99.9 % |
| `road` line written on `modeld.sock` -> its `view` line read on the WebSocket, p50 / p90 / p99 | 7.1 / 16.9 / 60.8 ms | 5.6 / 12.6 / 34.2 ms |
| chris-integrator process: CPU, memory | 10.6 % of one core (9.4 ms per road frame, everything included), 70 MB | 8.0 % (6.8 ms; the box's own process not counted), 67 MB |
| map box per pose, seen from the mixer | 0.60 ms | 0.94 ms (wait 0.85 ms mean, 11.8 ms max, 0 timeouts) |
| `/v2/health`, `/v2/mixer` | five daemons and `mapd` connected; vehicle, wheels, vision, gnss `ok`; 11 plug-ins, 0 errors | the same |

(The car is slow or standing for most of this window: a lead in a quarter of the scenes. In the car's own
recording of the window the old stack's `fused.map` was available in 29 % of its messages; the two are
different position sources, not a like-for-like figure.) At real pace the laptop's core idles between frames
and the per-call wall times are 1.5-2.5 times the batch replay's and vary from run to run (a first pair of
runs gave 6.9 % of a core and p50 4.2 ms): take the batch figures above for cost and these for "it works and
the hop is a few milliseconds".

Phone: **not measured**. L38's estimate (4-6 times slower per operation) puts the whole stack at 15-22 ms per
road frame without a radar.

### 3.6 Object classes

`tests/test_classes.py` (simulated town drive): with a class detector that says "truck" (one "car" in seven)
the lead has `class.label = "truck"` in more than 90 % of its frames, confidence above 0.6, source named; the
view's lead object carries it; a detection that points at nothing starts nothing; **tracks, lead, view and
the ego state are equal, number for number, with and without the detector** (the class keys apart); with
the camera faulted a talking class detector does not make the lead valid; the `class` of a tracking camera's
objects feeds the same attribute.

## 4. What is not done

* **The vehicle input store.** mapd's `VehicleStore` (validation of every feeder message with error replies,
  per-signal quality and age, the car's description, the sensors table, the calibration report) was not
  ported. The integrator's feed is L38's converter: `motion`, `gnss`, `controls`, `body`, `objects` are taken
  as they come; `vehicle` (description) and `reset` are accepted and ignored; no error lines are sent back;
  the `vehicle` topic carries `present`, `controls`, `body` only; `motion` quality tags are dropped.
  Consequence of the ignored `reset`: a synthetic test drive's position is not forgotten when it ends (the
  box marks `map.input.synthetic` from the receiver's tag, so consumers still see it is synthetic).
* **The units as one stack.** `chris-integrator.service` is a unit of its own (`WantedBy=multi-user.target`);
  it is not part of `roadsense.target`, which a host starts and stops. Section 6 lists the drop-ins.
* **A second roadsense instance**: wired as a channel with suffixed streams (`road@rear`); a second pose as
  an estimator source and second-camera trackers are not built.
* **Reference of the consolidated topics**: the field tables are still in roadsense's `docs/ARCHITECTURE.md`
  (headed "consolidated topic (chris-integrator)"), not yet in chris-integrator's own docs.
* The estimator out of process, async plug-ins on a car, sign and light tracks in the object layer, feedback
  of the mount estimate: not built.

## 5. What does not work, or is unverified

5.1 **Position on the poor-reception drive** (3.3): worse than the legacy filter; the map follows.
5.2 **Simulated city, 2 s lag**: 95.2 % on route, one dead-reckoning excursion of 180 m. Not diagnosed.
5.3 **A receiver without delay**: the estimator's 2 s prior is slow to give way (26 m on the synthetic
    network at zero lag, 46 m on the simulated motorway). The car's receiver does lag; another car's may not.
5.4 **Unverified**: everything on the phone (CPU, memory, sockets against the real daemons, the unit file,
    `install.sh`, the ring's intrinsics against `calib`); the debug face's governor publishing under real
    shedding; `/preview` forwarding under a browser (tested with the fake daemons only); the car's proxy
    against the new status page.
5.5 `tools/` were repointed by import (`chris_integrator.trackers.*`); `turn_flicker.py`, `lanes_replay.py`,
    `rules_replay.py`, `integrator_eval.py sim`, `roadsense_maps.py info`, `mixer_equiv.py`, `mapper_eval.py`,
    `stack_e2e.py` were run; the others (`view_replay`, `egolane_replay`, `roadgeom_*`, `lights_*`,
    `speedlimit_replay`, `vision_eval/*`, `budget/api_profile.py`, `vision_eval/l32/chain_api.py`) were not, and
    the last two still import the old `State` and will fail.

## 6. What the car repo has to change (`~/projects/cyberpank-prius-gen2-computer`, read-only here)

Verified by reading its code at 748546b. Nothing was changed there.

**To stay connected (minimum):**

1. **Ports.** The integrator's defaults collide on the car: 8080 is the car backend, 8081 its ZMQ
   (`backend/service.py:778`). Set `/etc/chris/integrator.env`: `CHRIS_API_PORT=8090`,
   `CHRIS_API_HOST=127.0.0.1`, `CHRIS_ROADSENSE_HTTP=http://127.0.0.1:8091`; `/etc/roadsense/api.env`:
   `ROADSENSE_API_PORT=8091`. Then `BACKEND_ROADSENSE_API_PORT=8090` stays, and with it the scene feed
   (`backend/roadsense_scene.py:60`, topics `road, lights, speed_limit, env, rules, fused, map, lead, lanes,
   view`: all served, same shapes), the proxy (`api/roadsense_proxy.py:86`), the health check
   (`backend/roadsense_power.py:152-153`, `/v1/health`: every key it mirrors exists) and the camera preview
   fetch (`backend/camera_preview.py`, `/v1/frame.jpg`, forwarded).
2. **The feed socket.** `BACKEND_POSITION_RUNDIR=/run/chris` (`backend/__main__.py:157, 216`; default
   `/run/roadsense`). The client opens `<rundir>/position.sock` (`backend/position_client.py:41-42`): the
   integrator serves that name. The hello offers `controls`, `body`, `vehicle`, so the feeder keeps speaking
   vehicle input API 1.1; it relies on no error reply.
3. **Units.** Remove `roadsense-mapd.service` from `DEFAULT_UNITS` (`backend/roadsense_power.py:131-138`) /
   `BACKEND_ROADSENSE_UNITS`, disable and mask it on the phone (position must not be estimated twice). The
   manager starts and stops only `roadsense.target`: ship drop-ins `roadsense.target.d/` (`Wants=chris-integrator.service`)
   and `chris-integrator.service.d/` (`PartOf=roadsense.target`, optionally `Slice=roadsense.slice`), add
   `chris-integrator.service` to the unit list, and do not enable it under `multi-user.target` (a unit that
   is listed and runs permanently makes the manager's stop time out).

**For the new layout:**

4. Deployment on the phone (roadsense side): `chris-integrator/scripts/install.sh` (copies
   `chris_integrator` and `chris_mapper` to `/usr/local/lib/chris`, the unit, `/etc/chris`,
   `/var/lib/chris`); needs `py3-aiohttp`, `py3-numpy` (already there for the api). Move or link the map pack
   to `/var/lib/chris/maps/current.rsmap` (or `CHRIS_MAP_PACK=/var/lib/roadsense/maps/current.rsmap`);
   `recorder.json`, `integrator.json`, `lead.json`, `lanes.json`, `egolane.json`, `view.json`,
   `roadgeom.json`, `lights.json` from `/etc/roadsense` to `/etc/chris`; the carried `integrator.json` from
   `/var/lib/roadsense` to `/var/lib/chris`; recorder sessions go to the `dir` named in the moved
   `recorder.json`. `scripts/install-phone.sh` then installs the debug face (default 8081, loopback).
5. `api_ok` now means "the integrator answers", also when camerad or modeld are dead
   (`backend/roadsense_power.py:395-415`): for the auto-restart to mean what it meant, also require
   `health.daemons.camerad/modeld.connected`.
6. Synthetic drives (`backend/position_feeder.py:1259-1292` sends `reset`): ignored by the integrator
   (section 4). `SyntheticTaint` (`roadsense_scene.py:861-890`) reads `map.input.gnss.source` and `fix`: both
   are there (the receiver's tag passed through the estimator), `input.synthetic` is set from the tag; it
   compares the source with the literal `"synthetic"`: unchanged. Until `reset` is handled, the position of a
   finished synthetic session lingers until real fixes replace it.
7. `map.position.dr_distance_m` is null (dashboard `mapLayer.ts:190` shows a dash); `map.input` has no
   counters.
8. The browser page under `/roadsense/`: the proxy rewrites `/v1/`, `/preview`, `/ws` only
   (`api/roadsense_proxy.py:71-73`), the status page polls `/v2/...`: add `v2/` to the rewrite and routes.
   Optional: `lead`, `lanes`, `view`, `ego_state`, `tracks`, `mixer` in its whitelist and `WS_TOPICS`.
9. Units and frames: unchanged for every existing topic (`lead`, `lanes`, `view`: x forward, y right;
   `tracks.tracks[]`: x forward, y **left**). New fields `tracks[].class`, `lead.class`,
   `view.objects[].cls` for drawing road users by kind once a detector exists.
10. Tools: `tools/mfd_chain_replay.py` imports `replay_lib` from roadsense's `tools/` (the shim keeps
    `Pipeline`, `read_session`; outputs identical, 3.2). `tools/mfd_map_drive.py` and
    `tests/test_position_mapd_e2e.py` start `python -m roadsense_mapd` and `roadsense_api`: they exercise the
    legacy layout and need porting to `python -m chris_integrator`.
11. From L37 / L38, still open and unchanged: never fall back to raw `road.leads[0]` while `lead.valid` is
    false; "unknown" distinct from "no lead" in the alerts; the motion feed's silences; receiver timing
    fields; `lead.sources[]` instead of string-matching `lead.source`.

## 7. Reproducing

```sh
cd ~/projects/wt/roadsense-l39; D=<datasets>/roadsense-rec; P=$D/trip-pos/pos_20261119.txt; O=/tmp/l39; mkdir -p $O
PP=$(python3 -c "import pytest,os;print(os.path.dirname(os.path.dirname(pytest.__file__)))"); PY=~/projects/roadsense/.venv-models/bin/python
# tests (aiohttp lives in the models venv on this machine; pytest in the system python); one suite per call
(cd chris-integrator && PYTHONPATH=$PP $PY -m pytest -q -p no:cacheprovider tests)     # 288 passed, about 6 min
(cd chris-mapper && PYTHONPATH=$PP $PY -m pytest -q -p no:cacheprovider tests)         # 22 passed
PYTHONPATH=$PP $PY -m pytest -q -p no:cacheprovider api/tests                          # 21 passed
(cd mapd && PYTHONPATH=$PP $PY -m pytest -q -p no:cacheprovider tests)                 # 81 passed
# the reference tree: the first commit of this branch (v2-l38 + main), the old api state
git worktree add --detach ~/projects/wt/roadsense-l39-ref fc5545b
systemd-run --user --collect -p MemoryMax=10G --wait -- bash -c "cd $PWD && export PYTHONHASHSEED=0 PYTHON=$PY &&
  for s in 065629 113201 151146 153636 161325 104128; do
    python3 tools/mixer_equiv.py run $D/20261119-\$s --pos $P --tree ~/projects/wt/roadsense-l39-ref --out $O/\${s}_ref.jsonl.gz;
    python3 tools/mixer_equiv.py run $D/20261119-\$s --pos $P --out $O/\${s}_new.jsonl.gz;
    python3 tools/mixer_equiv.py diff $O/\${s}_ref.jsonl.gz $O/\${s}_new.jsonl.gz --drop class,cls; done > $O/equiv.txt;
  python3 tools/mixer_equiv.py run $D/20261119-113201 --pos $P --mapper box --out $O/box.jsonl.gz > $O/box.txt;
  python3 tools/mixer_equiv.py run $D/20261119-113201 --pos $P --mapper remote --out $O/remote.jsonl.gz > $O/remote.txt;
  python3 tools/mixer_equiv.py run $D/20261119-113201 --pos $P --mapper none --out $O/none.jsonl.gz > $O/none.txt;
  python3 tools/mixer_equiv.py diff $O/box.jsonl.gz $O/remote.jsonl.gz > $O/box_vs_remote.txt;
  python3 tools/mapper_eval.py real $D/20261119-104128 $D/20261119-113201 $D/20261119-151146 $D/20261119-153636 --pos $P > $O/mapper_real.txt;
  python3 tools/mapper_eval.py synth --lag 2.0 > $O/mapper_synth_lag2.txt;
  python3 tools/mapper_eval.py synth --lag 0.0 > $O/mapper_synth_lag0.txt;
  python3 tools/stack_e2e.py $D/20261119-104128 --pos $P --start 900 --seconds 240 > $O/e2e.txt;
  python3 tools/stack_e2e.py $D/20261119-104128 --pos $P --start 900 --seconds 240 --remote-mapper > $O/e2e_remote.txt"
```

Cost per road frame and the per-plug-in table are printed by every `mixer_equiv.py run` (take them from a run
with nothing else on the machine). The replay pipeline comparison of 3.2: feed
`chris_integrator.replay.read_session(...)` through `Pipeline()` in both trees and hash
`json.dumps(o, separators=(",", ":"))` per topic.

## 8. Shared ground with L41

Not touched (no line changed, moved or renamed): `chris_integrator/core/ekf.py`, `core/ego.py`,
`core/monitors.py`, `chris_integrator/sources/*`, `chris_integrator/sim/*`, `tools/integrator_eval.py`
(`git diff 75285f9 v2-l39 --stat` over those paths is empty; `integrator_eval.py sim` still runs through the
`tools/replay_lib.py` shim). The plug-in contract needed nothing inside them: the estimator plug-in wraps
`Integrator` from outside, and the new source kind `object_classes` lives in `core/classes.py` and registers
itself in `sources.KINDS` at import.

Where the two branches can meet: `core/objects.py` (this lane: `classify`, `class_of` and two call sites in
`_apply`, an early return for annotation-only scans; +45 lines), `core/lead.py` (`class` in the two messages,
`annotates_only` in `live`; 6 lines), `chris_integrator/__init__.py` (3 lines), `feed.py` (moved to
`io/feed.py`, a shim left), `daemon.py` (rewritten), `docs/DESIGN.md` (header, sections 8 and 9 edited, 14-16
appended), `README.md`.

## 9. Branch

`v2-l39`; commits after the merge of `main` (fc5545b), oldest first: dd563f6 chris-mapper as its own box ·
591f20c the mixer and the move of the consolidated side · 0b949c0 the map box as a plug-in, equivalence and
mapper tools · 5b04fc8 object classes, end-to-end tool, docs, the rules fix · a2d2752 the receiver's tag
through the map box, `roadsense.target` without mapd · the last: this report and the end-to-end clock fix.
Not pushed, not merged into `main`.
