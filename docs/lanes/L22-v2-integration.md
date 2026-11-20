# L22: roadsense 2.0 as one line: the merged branches, one display contract, the open quality points

Built 2026-11-18 on the dev box (laptop). Nothing was deployed; the phone and the car were not touched. Branches
`v2` in roadsense and in the car computer (both unpushed), made from `main` by merging `v2-l19` (display
channel, live lead) and `v2-l20` (ego-lane filter, weak-receiver mapd, junction schematic). Contract:
docs/ARCHITECTURE.md ("view.road: the road as one picture", "roadsense 2.0: what changed since 1.x"); car side:
docs/MFD_MAIN_SCREEN.md 7.5.

Owner's ruling during the lane (2026-11-18): milliseconds from facts to the display do not matter; latency
matters on the measurement topics. In `view` and on the MFD smoothness wins over lag, and nothing is spent to cut
display latency. It is written into the contract (ARCHITECTURE.md, "Measurement topics and the display channel").

## 1. The merge

Conflicts: api `state.py` (both trackers kept), `docs/ROADMAP.md`, the car's `tests/test_mfd_scene.py` (both
test groups kept). Suites on `v2`: roadsense api 211 passed, mapd 93 passed; car computer 1712 passed (9 skipped) with
`test_position_mapd_e2e` run against this worktree (`PRIUS_ROADSENSE_REPO`). The car suite needs
`tests/fixtures/bean_nhw20_frames.txt.gz`, which is untracked in the main clone (5 failures without it; copied
into the worktree untracked, not committed).

## 2. One display contract: `view.road`

Before, a consumer that draws combined three topics and judged the map by itself: `roadgeom` (geometry),
`map.ahead.junction_view` (junction), `fused.lanes` (lane boxes), with its own trust thresholds. Now
`view.road` (`api/roadsense_api/roadview.py`) carries all of it for one frame: the ego lane's centre from the
camera near the car and from the map beyond, side roads, a roundabout, the junction view with its arms, the
lane count with the probability per lane, the lane to light and the likely lane, painted turn arrows; each part
with provenance, a confidence and `approximate` when the position is dead-reckoned, weak or held.

Decisions:

* **`roadgeom` is no longer a topic.** It was display content (a low-passed fit of the camera onto the map, shown
  after a second of agreement), so it lives only in the display channel. `roadgeom.py` stays as the code that
  computes the geometry fields of `view.road`; `/v1/roadgeom` answers 410.
* **Display policy moved from the consumer into roadsense**: the trust hysteresis on the match confidence, the
  hold over a drop-out, the "near enough for this position error" rule for a junction, the `approximate` mark,
  the hysteresis of the lit lane, carrying distances between two `map` messages. The MFD adopts the decision
  (`map.tr`) instead of judging the same thing a second time.
* **Merged functions**: one offline pipeline, `tools/replay_lib.py` (the api's derived topics in the api's
  order), used by `tools/view_replay.py` and by the car's `tools/mfd_chain_replay.py`, which each had their own
  copy of the tracker wiring; one junction converter in the car backend for the one source.
* The measurement topics are untouched by all of this: `map`, `fused`, `lanes`, `lead`, `lights` say what was
  measured; the alerts of the car read those.

What `view.road` gives on the recorded drives (`tools/view_replay.py SESSION --map MAP.jsonl`, the `map`
messages replayed through mapd 2.0 on the format 2 pack; share of the frames while moving):

| session | map layer trusted | of it approximate | map geometry joined | junction given (approximate) | lane count | ego lane lit / likely only | turn arrows |
|---|---|---|---|---|---|---|---|
| 20261112-141112 | 20.9 % | 96.8 % | 1.3 % | 2.6 % (2.6 %) | 20.9 % | 20.9 / 0.0 % | 4.6 % |
| 20261117-145921 | 47.3 % | 54.2 % | 10.4 % | 30.6 % (17.7 %) | 47.3 % | 43.7 / 1.0 % | 9.3 % |
| 20261117-163836 | 19.5 % | 71.4 % | 1.7 % | 11.9 % (9.6 %) | 19.5 % | 14.5 / 1.0 % | 12.2 % |
| 20261117-190932 | 63.3 % | 37.1 % | 38.3 % | 45.6 % (18.1 %) | 63.3 % | 56.4 / 4.7 % | 14.8 % |
| 20261117-191703 | 55.9 % | 53.5 % | 4.5 % | 24.8 % (16.7 %) | 55.9 % | 50.8 / 0.0 % | 25.5 % |
| 20261117-101013 | 0 (no position source on that drive) | | | | | | |

Half or more of what the map layer shows on these drives is marked approximate: the head unit's receiver
(3-5 satellites) was dead-reckoning or off by tens of metres most of the time (L18, L20). That is the honest
picture; a better position turns the same parts exact.

End to end to the MFD (`tools/mfd_chain_replay.py` in the car repo, 28 labelled junctions with a recording that
has a position; before = L20's figures from its report):

| stage | L20 | L22 |
|---|---|---|
| in the scene event | 18 | 14 |
| drawn on the MFD | 15 | 14 |
| ... the schematic at the right distance | 9 | 8 |
| ... of the drawn ones marked approximate | (not counted) | 13 |
| share of the driving time with the junction drawn (5 sessions) | 2.8 / 33.9 / 15.0 / 52.4 / 27.5 % | 2.3 / 32.0 / 12.6 / 50.0 / 27.5 % |

One junction fewer is drawn (session 191703: 2 -> 1) and one fewer at the right distance. The scene stage is
lower by construction: L20's scene event carried every junction view of a usable map and the MFD's gates then
removed some; now roadsense applies those gates before sending. A first version of the trust rule that averaged
all map messages lost two junctions (13); judging only the usable messages and holding over doubtful ones, as
the MFD did, brought it to 14.

Headless frames from two recorded drives through the whole chain (mapd replay on the new pack, api, backend
scene, link model, UI at 20 Hz), not committed because they carry road numbers of the owner's routes:
`~/roadsense-l22-out/drive_191703_junction.png` (standing at a red light: junction schematic approximate,
lane boxes with arrows left / straight / straight + right, the own lane lit) and `drive_163836_arrows.png`.
Demo shots with made-up data are in the car repo: `docs/images/mfd_l22_turn_arrows.png`,
`mfd_l22_turn_arrows_likely_approx.png` (shots 84, 85).

Size and cost: `view.road` is 1.1-2.0 kB per message on these drives, a whole `view` message 3.1-3.5 kB with
it (1.6 kB before), 13 a second on `/ws`; the separate `roadgeom` message (1-4 kB per frame) is gone, so the
stream is not larger than before. The whole derived pipeline (lead, lanes, ego lane, geometry, view, fused)
costs 1.8 ms per road frame on the laptop (one core of an i7-8850H), `roadview.py` itself 0.02 ms. Not measured
on the phone (lane L21 has the phone).

## 3. The three regressions

### 3.1 Lane lines stepping in turns (L19)

L19 shortened the MFD's 0.3 s low-pass to a 0.1 s ease and the lines at 30 m stepped more in turns. Measured on
the same stream of the 2026-11-17 day drive (29 min), drawn frames at 20 Hz (`tools/scene_feed_replay.py` in the
car repo), left / right lane line at 30 m:

| | before L19 (`main`) | L19 | L22 |
|---|---|---|---|
| drawn steps over 0.3 m | 8 / 7 | 26 / 30 | 3 / 3 |
| drawn step, p95 | 0.077 / 0.073 m | 0.079 / 0.074 m | 0.072 / 0.068 m |
| drawn step at 10 m, rms | 0.010 / 0.009 m | 0.011 / 0.010 m | 0.010 / 0.010 m |
| session 20261112-141112, steps over 0.3 m | 0 / 1 | 6 / 9 | 0 / 0 |
| the other four driving sessions | 0 | 0 | 0 |

The six that remain on the day drive are one event: after a gap in the camera frames the lines are placed anew
(3.4 m and 3.8 m in one frame, while fading in). What caused the stepping was not the length of the smoothing:
the samples reach the screen unevenly. roadsense makes 13 frames a second, the backend passes on at most 10,
so their frame times are 75 or 150-225 ms apart while they arrive every 100 ms; easing towards the newest
sample turned a line that sweeps steadily in a turn into a stair (a large step at a late sample, small ones in
between). Two changes:

* `view` (roadsense): the lines' smoothing grows with the distance (two stages of 0.1 s at the car, 0.16 s at
  10 m, 0.28 s at 30 m). Per model frame at 30 m: steps over 0.3 m 119 (`lanes`) -> 102 (L19 view) -> 72.
* MFD (car): each line point moves with its own rate between two samples (from the last two samples and their
  frame times), drawn 0.1 s behind "now"; only the correction at a new sample is eased.

**Ego-motion compensation was built, measured and left off** (`line_motion` in `view.py`). The idea of the
brief: a painted line stands still in the world, so carry the drawn line by the car's speed and yaw rate and
smooth only the remainder. On the recorded drives the car's motion does not predict the next frame of a line
better than "it stays where it is in the car's frame" (`/tmp/l22/carry_test.py` on the day drive, line at 30 m,
one-step prediction error rms):

| | stays in place | carried, the model's yaw rate | carried, the vehicle's yaw rate |
|---|---|---|---|
| raw model line, turning (yaw > 0.05 rad/s) | 0.286 m | 0.280 m | 0.281 m |
| raw model line, elsewhere | 0.136 m | 0.137 m | 0.149 m |
| tracked line, turning | 0.145 m | 0.126 m | 0.148 m |
| tracked line, elsewhere | 0.071 m | 0.072 m | 0.093 m |

Rotation alone or sliding alone are each much worse than doing nothing (0.34 / 0.38 m turning): in a curve the
car follows the lane, the two cancel, and the line hardly moves in the car's frame. Sliding along the line also
carries a bias of +0.03 m per frame: with the uncalibrated camera the model's lanes are not metrically
consistent along the distance (the lane is wider far away), and the wheel-speed yaw rate is noisier than the
model's. As a display filter it left the lines 0.34 m off the camera's own line in turns against 0.25 m for
the plain filter. L15 found the same for the tracker. With a calibrated camera and a gyroscope it may pay; the
switch is there.

Display lag of the lines: about 0.4 s at 30 m and 0.2 s at 10 m behind `lanes` in `view`, plus 0.1 s on the
MFD. By the owner's ruling that is accepted for the display; the measurement topic `lanes` is unchanged.

### 3.2 Ego lane wrong-and-confident on the night expressway (L20)

Same labels, same scoring as L20 (`tools/egolane_replay.py`, the lane count as a tagged map would give it;
`/tmp/l22/ego_eval.py` runs all sets on cached tracker output and reproduces L20's figures exactly before the
change):

| wrong and confident (right, share of all frames) | rule before L20 | L20 | L22 |
|---|---|---|---|
| a1-night (motorway, 2 lanes), 8645 frames | 1.4 % (93.1 %) | 0.9 % (98.7 %) | **0.3 %** (98.9 %) |
| krakow-night-tunnel (expressway, 3 lanes), 5247 | 1.5 % (88.1 %) | 3.8 % (92.9 %) | **1.5 %** (93.5 %) |
| warsaw-city (arterials, 3 lanes), 4135 | 19.1 % (9.3 %) | 7.7 % (79.5 %) | **4.6 %** (87.3 %) |
| car camera, 6 sessions, 8263 | 21.4 % (72.6 %) | 11.1 % (78.2 %) | **5.3 %** (79.7 %) |
| ... 20261117-101013 (low sun), 3400 | | 20.6 % | 10.9 % |
| ... 20261117-163836, 3646 | | 4.4 % | 1.8 % |
| car, 5 sessions, the map as recorded, 4863 | 17.6 % (19.2 %) | 1.2 % (18.3 %) | 1.1 % (18.3 %) |

Why the expressway got worse in L20, found on the frames of the wrong stretches:

1. **Lane changes were never detected.** The filter looked for a one-frame jump of the raw ego-lane centre by
   half a lane width. Of the five labelled lane changes on the rides none produced one: the network relabels
   its lines over half a second, at night alternating between the two labellings, and the tracker slides
   between them. The direction test was inverted as well (it never fired, so nobody saw). A filter that was
   sure of "rightmost lane" then stayed sure for 2-3 s after the car had moved left, until a far line was seen;
   the old rule had no memory and no such error. Now the pattern is read on the tracked lines: the centre was
   at least 0.2 lane widths to one side and within 1.2 s moves by 0.38 to the other. It finds three of the five
   labelled lane changes, each in the right direction (a1-night: both; krakow: one; on krakow's second and on
   warsaw's the lines show no crossing: on krakow the right edge closes in, a lane ending) and reports six more
   on the nine sets, of which one is a change the labels name in a note (car 20261117-101013, left to right).
2. **A changed lane count was always taken as "a lane added on the left".** On an expressway lanes come and go
   on the right (exit and auxiliary lanes): the car in the rightmost through lane was reported in the new exit
   lane, confidently, for 2.5 s at each. Now the estimate opens up (half each way) and the camera settles it;
   with painted turn arrows on the new stretch the side is known when the new lanes at one edge only turn.
3. Two constants, chosen on all nine sets together (the car's sessions are therefore no longer a held-out
   test for these two): the weight of a far line that is NOT seen, 0.4 -> 0.2 (on the car's wide lens and in
   town a real lane beside the car often shows no far line), and the commit threshold 0.75 -> 0.8.

Contribution on the expressway: 3.8 % -> 2.9 % (lane changes) -> 1.3 % (count changes) -> 1.5 % with the two
constants (which trade 0.2 points there for the car: 9.7 % -> 5.3 %).

What is left on the car's camera (5.3 %) is nearly all one session, 20261117-101013: stretches where a closed
bus lane, roadworks or a queue make the lane count itself doubtful (the labeller marked several unsure), and
two junction approaches where a turn pocket appears. With the map as it really was on those drives the figure
is 1.1 %: without a trusted count the filter says "unknown".

### 3.3 Session 20261118-095209: no lane line, no lead

**The scene, not a fault.** The car was parked facing a lawn with a low fence and a hedge; no painted line and
nothing ahead of it. Evidence (scripts in `/tmp/l22c`): 30,981 `road` messages at 13.4 a second, model time
53.9 ms, `valid` in every one, no NaN; lane-line probabilities 0.001-0.016 that change from frame to frame
(never two identical line arrays in 30,980 pairs); lead probability median 0.05 at 6.1 m, never present.
Camera 19.46 fps, sequence monotonic, zero drops; 2,212 readable preview frames, no two alike,
mean grey-level change between consecutive frames 3.3-6.7; a passer-by and the camera's own clock advance in
them; luma 116-129. Calibration identical to the comparison sessions. Two parked sessions with a car 2.4 m
ahead and parking-bay lines (20261117-132015, 20261116-115104) show lead probability 0.98 and line
probabilities 0.19-0.22 at the same model time and frame rate.

Correction to the L19 note: there is a light in that session, one static false positive of the light provider
(state `off`, relevance 0.001, never the ego light), and a sign false positive on a house gable.

Found on the way, not perception faults: the last 30 preview frames (10:30:14-10:30:44 CET) are empty files and
the message file ends in a cut gzip block: an abrupt power loss (the phone has no battery; four camera
reconnects in the last two minutes point at a failing supply). And the vehicle feeder is connected for the
whole session but sends no motion (`vehicle.present` false, no `ego` messages), as L20 saw since 17 Nov 22:07.

## 4. Map pack format 2

`turn:lanes` per direction in the edge record (two string-table indices, 44 -> 48 bytes per edge) and
`highway=speed_camera` as a point feature. Poland, OSM data of 2026-11-14:

| | format 1 | format 2 |
|---|---|---|
| file | `~/roadsense-maps/packs/poland-20261114.rsmap` (kept) | `~/roadsense-maps/packs/poland-20261114-f2.rsmap` |
| size | 262,210,224 bytes | 263,357,088 bytes (+1.15 MB, +0.44 %) |
| edges with turn lanes | - | 75,813 of 5,518,936 |
| speed cameras on a road | - | 117 (of 643 in the extract; the others are mapped beside the road) |
| build | 27 min | 62 min (the dev box was busy with replays), 20 GB cap, `systemd-run --user -p MemoryMax=20G` |
| sha256 | | `838442af14db9ec45172f2f276a7cad0f9db4edeb0a9b2893cddae7209ccc1c5` |

`roadsense-maps verify` passes (41,894 tiles, 5,518,936 edges, 607,379 km, 0 broken references). mapd 2.0 reads
both formats (a format 1 pack has no turn lanes anywhere, `map.pack.format` says which is loaded) and refuses
any other with an error that names it; mapd 1.x refuses format 2 the same way. Matching is unchanged by the
pack: the mapd replay of 20261117-163836 gives identical figures on the old and the new pack and in L20's tree.

Exposed as `map.road.turn_lanes`, `map.ahead.turn_lanes` (the stretch, the junction it belongs to, the lanes
that lead where the most likely path goes), `junction_view.approach.turn_lanes`, in `fused`, and in
`view.road.lanes.turn`. The MFD draws the arrows in the lane boxes from 120 m before the stretch. In the
ego-lane filter they are used where it is sound: the arrows count the lanes where no count is tagged, and they
say on which side a lane was added. Not used: "the driver signals left, so he is in a lane with a left arrow"
(on the approach a left signal is as often a lane change towards that lane).

Not added from L20's list: `destination` / `destination:lanes` (0.7 % of the ways, text for a sign row the MFD
does not have); speed cameras beside the road need a point layer with its own spatial index, a larger format
change than their 526 points justify now.

## 5. Contract

docs/ARCHITECTURE.md: the endpoint table on `/v2`, the measurement / display split with the latency ruling, the
ego lane section, "Road geometry" (now inside `view.road`), the `view.road` object and its policy table, and
"roadsense 2.0: what changed since 1.x" (every changed, removed and new field and config key). docs/MAPS.md:
format 2. README, ROADMAP. Car repo: docs/MFD_MAIN_SCREEN.md 7.3-7.5.

Process layout and transport are unchanged for lane L21: the same daemons and sockets, `/ws` as before, the
same topics except `roadgeom` (gone; its work is done inside the `view` step of the same `road` frame, so the
per-frame cost moved, it did not grow). New: `/v2` HTTP paths beside `/v1`.

## Commands

```sh
# pack
PYTHON=~/roadsense-maps/.venv/bin/python scripts/roadsense-maps build --region europe/poland --out ~/roadsense-maps/packs/poland-20261114-f2.rsmap
scripts/roadsense-maps verify ~/roadsense-maps/packs/poland-20261114-f2.rsmap
# mapd replay on the new pack, then the api's derived topics and the display channel
tools/map_replay.py ~/roadsense-rec/SESSION --pack ~/roadsense-maps/packs/poland-20261114-f2.rsmap --out MAP.jsonl
tools/view_replay.py ~/roadsense-rec/SESSION/messages.jsonl.gz --map MAP.jsonl --ws WS.jsonl.gz
tools/view_replay.py ~/roadsense-rec/SESSION/messages.jsonl.gz --set line_motion=true      # the ego-motion variant
# ego lane against hand labels (labels are kept outside the repository)
tools/egolane_replay.py <datasets>/rides-road-l15/krakow-night-tunnel.road.jsonl.gz --labels LABELS/krakow-night-tunnel.lanes.csv
# car repo: what the glass shows, and the junction chain
.venv-fe/bin/python tools/scene_feed_replay.py WS.jsonl.gz [--strip T:N:PNG]
python3 tools/mfd_chain_replay.py ~/roadsense-rec/SESSION --labels LABELS/car-SESSION.junctions.csv --map MAP.jsonl --roadsense ~/projects/wt/roadsense-v2
.venv-fe/bin/python -m cyberpunk_computer.ui.scene.demo --shots OUT_DIR
```

## Open

* **Nothing of 2.0 has run on the phone or the car.** Deployment order: roadsense (api + mapd) first, then the
  pack, then the car backend, then the Pi frontend. roadsense 2.0 with the old car code shows the camera's road
  only (the old backend subscribes to `roadgeom`, which is gone); the new car code with roadsense 1.x shows no
  junction schematic and no map geometry. Update both in one session.
* A drive has to show: lane lines in a turn (no stair, lead still in its lane), the lit lane through a lane
  change and at an exit lane, turn arrows against the paint on the road, a junction marked approximate against
  where it really is, and a roundabout (still in no recording).
* The position input is the limit of everything map-based: the feeder has sent no motion since 17 Nov 22:07,
  and the receiver puts the car tens of metres off (in 20261117-163836 the junction 100 m "ahead" was at the
  stop line the car stood at, with a claimed accuracy of 1.5 m).
* One junction fewer than L20 is drawn in the chain replay (14 of 28 against 15).
* The MFD's far road (100-600 m when no `geo` is given) is still drawn from `map.ahead.shape`, the one road
  element not from `view`; to decide whether it goes.
* `/v1/...` aliases stay until the owner says they can go.
* Two estimator constants were chosen on all labelled sets; new labelled drives would be the independent test.
* The hand labels were only in `/tmp/l20`; a copy is in `~/roadsense-labels` (outside every repository).
