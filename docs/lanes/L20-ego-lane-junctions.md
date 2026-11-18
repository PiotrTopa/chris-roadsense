# L20: which lane am I in; why no junction or roundabout was ever shown

Built 2026-11-18 on the dev box (P52), on the drive-recorder sessions of 12-18 Nov and on ride videos. Nothing
was deployed; the phone and the car were not touched. Branches `v2-l20` in roadsense and in the car repo.

The owner's two complaints:

1. "It sometimes knows there are three lanes, from GPS or vision, but has trouble figuring out which one I am on."
2. "I still have not yet seen a junction (skrzyżowanie) or a roundabout" on the MFD, after several drives through both.

## Data

| session | what it is | driving (min, vision speed > 3 m/s) | with a position (min) | map usable (min) |
|---|---|---|---|---|
| 20261117-101013 | 29 min, day, city + forest | 15.3 | 0 | 0 |
| 20261112-141112 | 17 Nov ~16:00 (recorder clock stale) | 12.2 | 4.8 | 3.1 |
| 20261117-145921 | 70 min, mostly parked, short city drive | 4.6 | 4.5 | 3.9 |
| 20261117-163836 | evening, expressway exit, arterials | 7.4 | 3.4 | 2.9 |
| 20261117-190932, -191703 | evening, town | 2.3, 2.5 | 1.9, 2.2 | 1.8, 2.1 |
| 20261117-220714, -224454 | short, parking | 1.2, 0.4 | 0 | 0 |
| 20261116-115104, 20261117-132015, 20261118-095209 | parked (bench) | 0 | 0 | 0 |
| 20261118-143107/145015/150942 | api only (no camera, no `road`) | - | - | - |
| **total** | | **45.9** | **16.8** | **13.8** |

Ground truth was
labelled by hand from the frames (one per 2 s, contact sheets) by a separate labeller that saw no estimator
output: per stretch the number of own-direction lanes and the ego lane, and every junction and roundabout
driven through. The labels stay off the repository (they come from the owner's drives). Ride videos with 3+
lanes were labelled the same way (`a1-night`, `krakow-night-tunnel`, `warsaw-city`; `a1-s8-rain` was
labelled but its stored model output has no road edges, so it is not scored).

**No recorded drive passes a roundabout.** Neither the labels nor the tracks against the pack (no position
within 20 m of a `junction=roundabout` way) show one. The owner's roundabout drives are not among the
recordings, so the roundabout half of complaint 2 can only be judged by its mechanism: a roundabout goes the
same way through the chain as a junction (below), with the schematic's ring and exit number as its picture.
**44 major junctions** were driven in the recordings, 16 of them in 101013, which had no position at all.

## Complaint 2: where the chain breaks

`tools/mfd_chain_replay.py` in the car repo walks each session through the real code of every stage:
mapd's `map` (recorded, or a replay through mapd's engine with `tools/map_replay.py`), roadsense's fusion,
lane tracker and `roadgeom`, the backend's `SceneBuilder` (scene event) and the frontend's `MapDirector`
gates, and asks for the 12 s before each labelled junction which stage still knew it. "right" = the
schematic showed a distance within 30 m / 35 % of the distance actually driven to the labelled junction.

| stage (44 labelled major junctions) | as recorded (old code) | replay, old code | replay, old code + schematic on | **replay, L20** |
|---|---|---|---|---|
| mapd had a position (good / dead reckoning) | 23 | 23 | 23 | 23 |
| map layer usable (`fused.map.available`) | 18 | 20 | 20 | 21 |
| a junction announced (`ahead.junction_view` / `junctions` < 300 m) | 22 | 22 | 22 | 22 |
| in the scene event (`map.jv`, or `geo` with side roads / a ring) | 16 | 16 | 16 | 18 |
| **drawn on the MFD** (schematic, or side roads / a ring in the road picture) | **7** | **5** | 13 | **15** |
| ... the schematic at the right distance | 0 | 0 | 7 | **9** |

Attribution, stage by stage:

1. **Position input (car side): 21 of 44 junctions had no position at all.** The head-unit GNSS bridge only
   started on 17 Nov in the afternoon; 101013 (16 junctions) ran without any source. Where it ran, the
   receiver was weak (fixes 31 % of the driving time, 3-5 satellites, L18), and in 141112 mapd was `unknown`
   or off the road for most of the drive. Since 17 Nov 22:07 every session shows the feeder connected but
   sending only its 5-second `fix: none` heartbeat: no `motion`, no `controls`, no `body`, no fix (the
   `vehicle` topic: `present: false`, 458 messages in 38 min). 220714 is 1.2 min of driving with no CAN
   motion from the feeder; the later sessions are parked or api-only, so whether this persists on a drive
   needs the car (open, below).
2. **The receiver claims more than it has (roadsense).** While moving, the filtered position lay a median
   16 m from the matched road while it claimed 6 m; at the service-road matches checked against the frames
   the position was 20-40 m off a 3-lane arterial. mapd trusted the receiver's `hAcc` (1-4 m), so the
   matcher's distance term punished the right road and nearby service roads won (20-35 % of the moving
   time on a service road; the frames showed multi-lane roads). Fixed in mapd: the receiver's accuracy is
   estimated from its own innovations, service roads lose with speed, the confidence is the road's (below).
3. **roadgeom withholds the map part almost always on these drives (roadsense, by design).** In 145921 the
   fused geometry used the map in 3 % of the moving frames: `low_match_confidence` 43 %, `off_path` 18 %,
   `disagree` 18 %, `off_road` 10 %. With a position 15 m off, camera and map disagree; withholding is right.
   The side roads in the road picture therefore cannot carry junctions on a receiver like this one.
4. **The MFD (car side): the schematic was off by default** (`map_junction = False`, "until it has been
   seen with a real position"), and junction and roundabout pictograms were deliberately not drawn, so the
   only path was the road picture of point 3. With the schematic on, the old code would have drawn 13 of
   the 23 junctions with a position. The trust gate (a 3 s average match confidence of 0.7) kept the map
   layer off half of the time (median match confidence while moving 0.45).
5. The backend relay is not a loss: whatever reached `fused` / `map` while usable reached the scene event
   (the 16-18 above), and mapd announced 22 of the 23 junctions that had a position.

So for complaint 2 the cause is **position input first** (no source or a weak receiver for every junction
the owner drove before the evening of 17 Nov), **the MFD's defaults second** (the schematic off, a strict
trust gate), and **roadsense third** (a matcher that believed an optimistic receiver). The transport is fine.

### What changed

* **mapd** (`posfilter.py`, `engine.py`, `matcher.py`):
  * receiver accuracy: between consecutive fixes, a running mean (20 fixes) of the squared innovation against
    what the claimed `hacc_m` promises; `hacc_m` in use = claimed x sqrt(mean), 1..6 x (`gnss_adaptive`,
    `gnss_scale_n`, `gnss_scale_max`; `input.gnss.hacc_scale`);
  * class prior x (1 + 3 x min(1, (speed - 1.5) / 8.5)) for service, restricted and (new) parking edges
    (`class_speed`, `class_speed_mps`, `parking_prior`);
  * `match.confidence` of the road: connected candidate edges of the same class and ref / name are not rivals
    (`road_confidence`).
* **MFD** (car repo): the junction schematic on by default; trust gate 0.55 / 0.40 instead of 0.7 / 0.55; the
  schematic also while the position error is at most 40 % of the distance, then drawn approximate (amber frame,
  `~` distance; dead reckoning too) instead of hidden.

| matching while moving (5 sessions, replay with the recorded positions as fixes) | before | L20 |
|---|---|---|
| map usable | 84.3 % | 84.5 % |
| match confidence median; share >= 0.5; >= 0.7 | 0.45; 45 %; 21 % | 0.55; 58 %; 36 % |
| on a service road at > 25 km/h | 20.5 % | 15.0 % |
| junction view present | 58 % | 59 % |
| the same on 163836 with the head unit's raw fixes (its log): service at speed; confidence median | 24.4 %; 0.43 | 7.0 %; 0.57 |

The recorded positions are the old filter's smoothed output, which hides most of the receiver's scatter from
the accuracy estimator (hacc median 6.3 -> 6.8 m there); on raw fixes the effect is the larger one. There is
no ground-truth track: "service road at speed" is the error proxy (every case looked at in the frames was a
multi-lane road), the junction table above is the end-to-end check.

## Complaint 1: which lane am I in

Before: `fused.lanes.ego_index` was a rule: one lane -> 0; nothing seen to the right -> the rightmost; two
lanes and something to the right -> the left one; on a one-way road nothing to the left -> 0; else unknown.
It never named a middle lane, never used the road edges, lane changes, or time, and the MFD drew the boxes
only with a known ego lane.

Now: `api/roadsense_api/egolane.py`, a discrete Bayes filter over the own-direction lanes to the right of
the car (docs/ARCHITECTURE.md "Ego lane"): road-edge gaps (one-sided), outer lines, the map's tagged count,
lane changes from the raw inner lines, the turn signal; probabilities per lane; "unknown" below 0.75. Its
constants were calibrated on the three ride videos (measured: the far line is seen in 69-73 % of the frames
when a lane is there and 0-13 % when not; the model's edge sits 2.5 m + about 1.8 m per lane from the ego
line); the car's own sessions are the held-out test (another camera, another country road mix).

Scored on labelled frames with 2+ own lanes, labels sure / likely, moving; the map count is the labelled one
("as a tagged map would give it") unless stated:

| | frames | committed | right when committed | **wrong and confident** | right, share of all | time to correct, median |
|---|---|---|---|---|---|---|
| a1-night (motorway, 2 lanes): before / L20 | 8645 | 94.5 / 99.6 % | 98.5 / 99.1 % | 1.4 / **0.9 %** | 93.1 / 98.7 % | - / 0.3 s |
| krakow-night-tunnel (expressway, 3 lanes) | 5247 | 89.7 / 96.7 % | 98.3 / 96.1 % | 1.5 / **3.8 %** | 88.1 / 92.9 % | - / 1.9 s |
| warsaw-city (arterials, 3 lanes, middle) | 4135 | 28.3 / 87.2 % | 32.7 / 91.2 % | 19.1 / **7.7 %** | 9.3 / 79.5 % | - / 0.6 s |
| car, 6 sessions (held out) | 8263 | | | 21.4 / **11.1 %** | 72.6 / 78.2 % | |
| car, 5 sessions, with the map as recorded (no labelled count) | 4863 | | | 17.6 / **1.2 %** | 19.2 / 18.3 % | |

Readings: the middle lane, which the rule could not name, is where the gain is (city 9 % -> 80 % right). With
the map as it really was on those drives, the old rule was confidently wrong 17.6 % of the time (a tagged
count of a road the car was not on), the filter 1.2 %: it says "unknown" where the evidence is thin, which is
most of the time there (no usable map). One case got worse: the expressway's wrong-and-confident 1.5 -> 3.8 %
(night, faint dashes in the tunnels). The car's 101013 (low sun from the middle on) is the weak spot: 20.6 %
wrong and confident, mostly 1-vs-2-lane stretches where the labeller marked the lane count itself unsure.

MFD: the boxes come from `fused.lanes` as before; with the own lane unknown and the count known they now show
the count with no lane lit and the most likely lane outlined (>= 0.45). Demo shots 60 (known) and 83 (likely).

## Contract changes (consumers)

| where | change |
|---|---|
| `map.input.gnss.hacc_scale` | new: factor on the receiver's `hacc_m` in use |
| `map.match.confidence` | meaning: the confidence of the matched road, not of one edge (higher at nodes) |
| `map.position.hacc_m` | now honest for an optimistic receiver (can be up to 6 x the receiver's claim) |
| `fused.lanes` | new `ego_from_right`, `probs`, `probs_from` (`left` with a map count, else `right`), `estimator` (`confidence`, `provenance`, `last_change`, `evidence`); `ego_index` from the filter, `null` = unknown (more often than before, never a guess) |
| config | `mapd.json`: `gnss_adaptive`, `gnss_scale_n`, `gnss_scale_max`, `matcher.class_speed`, `class_speed_mps`, `parking_prior`, `road_confidence`; api: `<config dir>/egolane.json` |
| car scene | `map.lanes.p` (per-lane probabilities); tunables `map_junction` true, `map_min_match` 0.55, new `map_junction_hacc_share` |

`roadgeom` is unchanged (self-contained, for the merge into L19's `view`). No map pack change: the phone's pack
(`poland-20261114.rsmap`) stays valid.

## Map data

`lanes`, `lanes:forward` / `lanes:backward` and `oneway` are already in the pack. Not in it: `turn:lanes`,
speed cameras. Coverage in the Poland extract of 2026-11-14 (a pyosmium pass over the road ways and nodes of
`poland-261114.osm.pbf`, 40 min on the dev box): They were not added: a format change means a Poland rebuild
(27 min, 8 GB on the dev box) and a new pack on the phone, and the estimator's measured errors are not where
turn lanes would help first (they come from edges and lines that the camera does not see). Turn lanes earn
their place for the lane picture at a junction approach (which lane goes where), the obvious next step.

| | count | share | pack cost (estimate) | matcher cost |
|---|---|---|---|---|
| road ways of the pack's classes | 3,187,051 | | | |
| ... tertiary and above | 465,614 | | | |
| ways with `turn:lanes` (any of plain / `:forward` / `:backward`) | 67,779 | 2.1 % of all; 13.3 % of tertiary+ | 1.17 MB of tag text before deduplication and zlib, so well under 1 MB in a 262 MB pack (under 0.4 %) | none (an attribute read only at the look-ahead's junctions) |
| ways with `lanes` / `lanes:forward|backward` (already in the pack) | 452,102 / 34,937 | 14 % / 1.1 % | in the pack | - |
| ways with `destination` / `destination:lanes` | 23,164 | 0.7 % | not measured | - |
| `highway=speed_camera` nodes | 643 | | a few kB as a point feature | none |

Both would fit cheaply; turn lanes on 13 % of the major roads (concentrated in towns, at junction approaches)
are worth adding next to the junction view and the ego-lane filter's support. Speed cameras are 643 points under
ODbL like the rest of the pack.

## CPU (laptop, same replay)

| | before | L20 |
|---|---|---|
| mapd `step` (5 sessions, 1963 moving steps), mean / p95 | 1.13-1.22 / 2.10-2.36 ms | 1.14-1.21 / 2.13-2.31 ms (same within run-to-run noise) |
| api: ego-lane filter per `lanes` frame | - | 0.047 ms median (the lane tracker: 0.38 ms) |
| api: `fused.lanes` per `fused` message | 0.005 ms | 0.007 ms; 352 -> 649 bytes |

At 14 frames a second the filter is about 0.07 % of a laptop core; on the phone's little cores, 20-30 x
slower for this kind of Python (L13), about 2 %. Not measured on the phone.

## Commands

```sh
# mapd replay of recorded sessions (variants with --set; --fixes for raw fixes in JSON lines with ts_real_ns)
tools/map_replay.py ~/roadsense-rec/20261117-163836 [--set matcher.class_speed=0 --set gnss_adaptive=false --set matcher.road_confidence=false] [--out MAP.jsonl]
# ego lane against hand labels
tools/egolane_replay.py ~/datasets/rides-road-l15/warsaw-city.road.jsonl.gz --labels LABELS/warsaw-city.lanes.csv
tools/egolane_replay.py ~/roadsense-rec/20261117-163836 --clock session --labels LABELS/car-20261117-163836.lanes.csv --map-count recorded
# the whole chain to the MFD (car repo)
~/projects/roadsense/.venv-models/bin/python tools/mfd_chain_replay.py ~/roadsense-rec/SESSION --labels LABELS/car-SESSION.junctions.csv [--map MAP.jsonl] --roadsense ~/projects/roadsense
```

## Open

* **Position input on the car.** From 17 Nov 22:07 the feeder sends only `fix: none` and no CAN messages at
  all. Needs a look at the car (panda / CAN tap, head-unit bridge) on the next drive: `GET /api/v1/position`
  (`motion.can_age_s`, `gnss.has_fix`) and roadsense `/v1/vehicle` (`present`). Without it nothing of the map
  layer, the junction schematic or the map-based ego lane can show.
* **The receiver.** Even with the fixes flowing, 3-5 satellites put the position 15-40 m off the road; the
  accuracy estimate makes the map layer honest about it, it cannot make it right. The head unit's antenna
  placement and assistance data (car TODO 3a) are the lever; then "right distance" at junctions should follow
  the position (9 of 23 now).
* **No roundabout in any recording.** The roundabout path (schematic with ring and exit number, road-picture
  ring) is checked on synthetic networks and demo data only; the next drive through one with a fix is the check.
* **Ego lane:** the expressway at night (3.8 % wrong and confident) and low sun in town; lane changes are
  taken from line relabelling only (no vehicle lateral motion yet); the turn signal is not fed by the car
  (no decoder for the lamps); the lead and other vehicles are not used (the model reports the lead ahead only).
* **Map pack:** `turn:lanes` and speed cameras not added (above).
* The chain counts use a replay with the recorded (smoothed) positions as fixes; the real effect of the
  accuracy estimate on raw fixes was measured on one session only (163836).
