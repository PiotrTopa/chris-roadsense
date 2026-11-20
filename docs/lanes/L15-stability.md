# L15: stability of what a consumer draws (lane lines, road edges, traffic lights), drive recorder

Built 2026-11-16 on the dev box (laptop), deployed to the phone the same night (the car was parked). Times CET.
Contract: docs/ARCHITECTURE.md ("api-derived topic lanes", "light provider", "Drive recorder").

Observations that opened the lane (owner, after the evening drives of 2026-11-16: four sessions, 50 min with
the stack up, all of them at night, some in rain; the MFD scene of the car computer is the consumer):

1. a lot of traffic lights are missed;
2. lane lines jump on and off;
3. the trees beside the road wiggle left and right and scatter.

## Data

* **Nothing the stack saw on those drives is on disk.** The car backend's trip logs
  (`~/cyberpunk/logs/trips/trip_20261116_*.ndjson`, 185 MB) hold CAN / AVC / powerbox traffic and no
  roadsense message, the same finding as L14. What exists from the drives: the journal of the units
  (10 s statistics lines), `signs.jsonl` (ended sign tracks, no positions or times), the sign crops in
  `/run/roadsense/crops`, and the last preview frame.
* What the journal says: `road` 13.5-13.8 runs/s (model 54 ms), signs 5.0/s, lights 2.97/s at 107 ms per
  frame, hottest zone 83 C on average and up to 89.3 C (the unit throttles at 95 C). The light tracker had
  at least one live track in 35 of 280 ten-second samples (12.5 %); envd: night throughout, place
  city_street / residential / highway / forest_road, rain in a quarter of the minutes. mapd received GNSS
  messages and used none (no fix), so the map layer was `no_source` all evening.
* What the pictures say: the last preview frame (parked, night) is soft, with rain drops on the glass as
  bright discs around every lamp; the sign crops taken while driving are readable for signs lit by the
  headlamps and blown out for lamps (a red light is a white core in a red halo).
* So, as in L14, every number below comes from the ride library (docs/RIDES.md), other cameras, replayed on
  the laptop: `modeld_replay` (supercombo v0.9.7, ORT fp32, 20 fps) over the first 7.5 min (9000 frames) of
  `warsaw-city`, `krakow-night-tunnel`, `a1-night`, `forest-lubelskie`, `gdansk-night-snow` (the L14 dumps
  carry no road edges), kept in `<datasets>/rides-road-l15/` on the laptop, not in the repo; and roadsense-signs `lights_replay` (ORT) on the frames of
  `krakow-lights`, `krakow-night-tunnel`, `gdansk-night-snow`. "10 Hz" is every second road message, about
  what the phone delivers (13.6/s, ncnn fp16).
* The recorder of this lane (below) is what changes that from the next drive on.

## Lane lines and road edges

### Diagnosis (raw `road` output, `tools/lanes_replay.py`)

A consumer drew a lane line while its `prob` was at least 0.3 (the car backend drops it under 0.2) and low-
passed the geometry over 0.3 s.

| ride (10 Hz, moving) | lane lines on/off changes per min (four lines) | spans shorter than 1 s in 7.5 min | worst line | road edge step per frame at 10 m, median / p95 |
|---|---|---|---|---|
| warsaw-city (day) | 70.0 | 386 | far right 35.6/min | 0.11-0.14 / 0.51-0.53 m |
| krakow-night-tunnel | 60.4 | 344 | far right 23.5/min, right 20.8/min | 0.13-0.22 / 0.58-0.84 m |
| a1-night (motorway) | 17.6 | 100 | far right 12.0/min | 0.19-0.31 / 0.72-0.96 m |
| forest-lubelskie | 39.5 | 239 | far left 31.7/min | 0.07-0.08 / 0.26-0.27 m |
| gdansk-night-snow | 63.5 | 379 | left 31.2/min | 0.13-0.18 / 0.52-0.73 m |

At the full 20 fps the counts are 38-180 changes per minute. Nearly every change belongs to a span shorter
than a second: the probability of a weak line sits in 0.2-0.5 for 20-50 % of the time and crosses the
threshold back and forth. The lateral position of a lane line that is shown is not the problem (median step
0.02-0.08 m per frame at 10 m, and the consumer's low-pass on top); the road edges are: 0.07-0.31 m per frame
in the median with a `y_std` of 0.5-1.4 m, 0.3-1 m at the 95th percentile.

### Tracker (`api/roadsense_api/lanes.py`, topic `lanes`)

Design in docs/ARCHITECTURE.md. The points that were decided by measurement:

* **Presence**: a 0.5 s low-pass of the probability with a 0.45 / 0.2 hysteresis. A line needs about 0.35 s
  of prob 0.9 to appear and about 0.9 s of prob 0.05 to go.
* **Geometry**: a per-point Kalman filter whose measurement noise is the network's own `y_std` (0.3 x for
  lane lines: their frame-to-frame scatter is about a quarter of the `y_std`; 1.0 x for road edges).
  Process noise 0.2 + 0.006 x m per sqrt(s).
* **Ego-motion compensation: built, measured, switched off.** Sliding every line by the distance travelled
  and turning it by the frame's yaw is the textbook prediction for a static world. Open loop (predict frame
  k+1 from frame k) it is right in sign (the model's yaw rate is positive to the right) and barely better
  than no prediction (mean error at 30 m 0.038 m against 0.041 m, in turns 0.045 against 0.052), because a
  car that follows its lane sees lines that hardly move in its own frame. In the closed loop it was worse on
  every ride: the tracked inner lines trailed a zero-phase reference by 0.09-0.27 m rms at 10 m with it and
  0.04-0.14 m without (10 Hz, four rides, the same noise settings). Two reasons were found: a camera mounted
  with a little yaw sees every line at a slope, and sliding along x then drifts the line sideways unless the
  model's lateral velocity is subtracted, which carries the same bias (its mean while driving is -0.07,
  -0.16 and -1.44 m/s on warsaw-city, a1-night and forest-lubelskie: a mounting yaw, not a motion) and
  cancels only as well as both are known; and the model's speed scales with the camera calibration. `slide_gain`, `yaw_gain`, `vy_gain`
  stay in the configuration for a calibrated camera with a wheel speed.
* **Relabelled lines**: a frame more than 4 sigma and 0.8 m off over 5-45 m is not averaged in; two such
  frames that agree, 0.2 s apart, restart the track. In the five 7.5 min segments this happened once: lane
  changes on these rides are followed as a continuous motion.

### Result (`tools/lanes_replay.py`, 10 Hz, moving)

| ride | lane lines on/off changes per min: raw -> tracked | spans < 1 s: raw -> tracked | inner lines, tracked against a zero-phase reference at 10 m (rms; while the line moves faster than 0.5 m/s) | road edge step per frame at 10 m, median / p95: raw -> 0.3 s low-pass -> tracked |
|---|---|---|---|---|
| warsaw-city | 70.0 -> 9.2 | 386 -> 0 | 0.06-0.11 m (0.29-0.40 m) | 0.11-0.14 / 0.51-0.53 -> 0.04-0.05 / 0.18-0.19 -> 0.02-0.03 / 0.08-0.09 m |
| krakow-night-tunnel | 60.4 -> 4.5 | 344 -> 0 | 0.03-0.04 m (0.18 m) | 0.13-0.22 / 0.58-0.84 -> 0.05-0.07 / 0.20-0.26 -> 0.03 / 0.09-0.11 m |
| a1-night | 17.6 -> 1.1 | 100 -> 0 | 0.06-0.07 m (0.32-0.40 m) | 0.19-0.31 / 0.72-0.96 -> 0.05-0.09 / 0.19-0.26 -> 0.02-0.03 / 0.08 m |
| forest-lubelskie | 39.5 -> 0.5 | 239 -> 0 | 0.04-0.06 m (0.08-0.16 m) | 0.07-0.08 / 0.26-0.27 -> 0.02 / 0.07-0.08 -> 0.01-0.02 / 0.05 m |
| gdansk-night-snow | 63.5 -> 3.2 | 379 -> 0 | 0.04-0.10 m (0.15-0.19 m) | 0.13-0.18 / 0.52-0.73 -> 0.05-0.06 / 0.20-0.25 -> 0.02 / 0.07 m |

At 20 fps: 38-180 -> 0.3-8.9 changes per minute, 248-1247 -> 0 spans shorter than a second.

Readings:

* The flicker is gone on the replay: what remains (0.5-9 changes a minute) are lines that really come and
  go (junctions, a second lane beginning or ending), each shown or hidden for more than a second.
* The share of time a line is shown stays within 4 points of the raw one except where the raw line was
  mostly noise (gdansk right line: 9.0 % raw, 3.6 % tracked).
* The tracked lane-line geometry is steadier than the raw one and less smooth than the consumer's 0.3 s
  low-pass of it (median step at 10 m: raw 0.016-0.075, low-pass 0.008-0.026, tracked 0.013-0.033 m): the
  filter was tuned not to trail. A consumer that keeps its low-pass on top loses nothing.
* The road edges are where the geometry filter pays: the median step per frame falls by a factor of 4-11
  against the raw edge and 1.3-3 against the consumer's low-pass (1.5-3.6 at the 95th percentile), and an edge the network does not know (snow: `y_std`
  of several metres) is reported absent instead of jumping (gdansk: present 55-65 % of the time). The
  "lag" of an edge against the 1 s median of its raw value is 0.1-0.6 m rms at 10 m; that reference is
  itself noisy by 0.1-0.2 m, and an edge is a soft thing (parked cars, kerb or grass).
* **Map prior**, with a constant synthetic lane count (no ride has a position): `--lanes-total 2
  --lanes-forward 1` on forest-lubelskie (a two-lane country road, which is what the count says) takes the
  last far-right appearances away (0.5 -> 0.0 changes per minute); a wrong prior was not tried on a ride,
  the tests cover that strong camera evidence overrides it. On the car the prior has never been active.

Cost: 1 ms per road message on the laptop, 3 kB per message.

## Traffic lights

The consumer shows one light: the `ego` light of the `lights` topic. So "missed" is a chain: the provider's
detector (roadsense-signs `signd --lights`, about 3 frames a second) -> its tracker -> the api's ego
selection -> the screen. No picture of a traffic light from the car's camera exists (see Data), so the
chain was measured on three rides.

### Diagnosis

* **The message dropped a light whenever one frame missed it.** The provider listed confirmed tracks that
  were matched in the current frame only, and the api chose the ego light per message. At 3 Hz one missed
  frame is a third of a second without the light, and a missed frame is common for a small light.
* **Far lights are small for the detector.** The wide view (the upper 62 % of the frame into 960x320) scales
  the picture by 0.72. A 0.9 m housing is 15 px tall at 60 m with the car's camera (focal 975 px), 11 px
  after that downscale; the detector's recall falls quickly under about 20 px (L7: 21 % under 12 px, 49 % at
  12-24 px on BDD100K). On the two rides with enough of them the wide view reports 39-56 % of the light-time
  of lights under 12 px.
* **The ride library's cameras look lower than the unit's defaults assume**: their horizon is at rows
  440-525 of 720, and the default region ends at row 446, so far lights near the horizon were cut off in the
  earlier L7 replays. The car's horizon is near the middle of the frame as far as the one parked frame shows
  (the pitch is not calibrated); the replays below move both views to each ride's horizon the way they sit
  on the car.
* Not measurable without pictures from the car: what its camera makes of a light at night in rain (the
  parked frame shows every lamp as a disc of rain-drop reflections). This may well be the larger part of
  the complaint, and none of the changes here addresses it.

### Changes

roadsense-signs (`signd --lights`, all three optional, in the unit since this lane):

* `--lights-roi2 0.125,0.125,0.875,0.569`: every other light frame feeds the middle 960x320 pixels around
  the horizon to the same detector at scale 1.0 instead of the wide band at 0.72. Same number of detector
  runs, so the same cost (measured on the phone, same frames, old and new binary: 253 ms per run on the
  little cores while parked for both; in the car on the big cores it is about 92 ms).
* `--lights-thr-keep 0.12`: boxes scoring 0.12-0.25 continue an existing track and never start one.
* `--lights-coast 2`: a confirmed track stays in the message for two more light frames after its last match
  (`coast` 1, 2; predicted box, last smoothed state). The tracker does not count a frame against a track
  that lies outside the view of that frame.

api (`lights.py`, `EgoLightSelector`): the ego light keeps its role down to relevance 0.07 (it gets it at
0.15), a coasting track counts 0.85 per missed frame, the last ego light is served for 1.2 s after no light
qualifies (`held: true`, `age_s`), and a signal on the map's look-ahead within 150 m relaxes this to 0.10 /
2 s (`map_signal`; inert without a position source, tests only).

### Result

Reference ("what is there"): the same detector with everything it has, every 4th frame (5 Hz), four views
(the wide band, two full-resolution tiles and a 1.5 x tile around the horizon), boxes at 0.35 or more linked
into chains of at least 0.8 s that are lit most of the time (roadsense-signs `tools/lights/replay_score.py`).
It cannot see what the detector cannot see at its best, so the shares are upper bounds of the true rate.
Runs: every 7th frame (2.9 Hz) of the first 5 min of each ride.

| ride (lit reference lights, light-seconds) | configuration | light-time reported | lights under 12 px | 12-24 px | 24 px and more | drop-outs per min of light-time | reported light-frames without a reference box |
|---|---|---|---|---|---|---|---|
| krakow-lights, day (46, 844 s) | before | 84.6 % | 56.4 % | 74.6 % | 99.5 % | 4.2 | 0.04 % |
| | coast 2 | 87.5 % | 63.2 % | 79.7 % | 99.7 % | 1.7 | 3.6 % |
| | coast 2 + keep 0.12 | 90.9 % | 73.9 % | 85.1 % | 99.7 % | 1.0 | 4.0 % |
| | **+ second view at 1.0 (deployed)** | **95.8 %** | 83.2 % | 94.5 % | 99.5 % | 1.8 | 3.3 % |
| | + second view upscaled 1.5 x | 96.1 % | 91.7 % | 94.2 % | 98.8 % | 1.4 | 2.9 % |
| krakow-night-tunnel (25, 75 s) | before | 81.6 % | 39.2 % | 91.3 % | 94.9 % | 4.8 | 0.3 % |
| | coast 2 | 84.8 % | 45.9 % | 94.2 % | 94.9 % | 1.6 | 6.7 % |
| | coast 2 + keep 0.12 | 88.0 % | 56.8 % | 95.9 % | 94.9 % | 0.8 | 10.9 % |
| | **+ second view at 1.0 (deployed)** | **93.9 %** | 81.1 % | 97.9 % | 93.2 % | 1.6 | 7.0 % |
| | + second view upscaled 1.5 x | 92.8 % | 81.1 % | 98.8 % | 83.1 % | 3.2 | 8.2 % |
| gdansk-night-snow (8, 20 s) | before | 97.0 % | 87.5 % | 95.3 % | 100 % | 6.0 | 0 % |
| | **+ second view at 1.0 (deployed)** | **99.0 %** | 100 % | 97.7 % | 100 % | 3.0 | 11.6 % |

The ego light (`tools/lights_ego_replay.py`, the same runs with the road messages of the ride):

| ride | ego light shown, share of the time: before -> after | gaps in the ego light shorter than 1.5 s (blinks), 5 min: before -> after | ego on/off changes per min | of the shown time: held |
|---|---|---|---|---|
| krakow-lights | 45.7 % -> 52.0 % | 3 -> 0 | 3.6 -> 2.8 | 5.4 % |
| krakow-night-tunnel | 13.1 % -> 21.1 % | 8 -> 0 | 6.4 -> 2.8 | 16.6 % |
| gdansk-night-snow | 10.8 % -> 19.6 % | 12 -> 0 | 6.4 -> 0.8 | 7.7 % |

Readings:

* On the two rides with enough lights the reported share of the light-time goes from 82-85 % to 94-96 %,
  and for the lights under 12 px from 39-56 % to 81-83 %. Coasting with the keep threshold and the second
  view contribute about half each; scale 1.0 beats the 1.5 x upscale on the larger lights and at night.
* The price is reports with no reference box near (3-12 % of the reported light-frames, from none): the
  coasting tail after a light left the picture (two frames, 0.7 s), and weak boxes (0.12-0.2) that the
  reference, which starts at 0.2, does not count. Both are marked (`coast`, the box score) and the ego
  selection discounts the first.
* The ego light no longer blinks on the replays (23 short gaps -> 0) and is shown 6-9 points more of the
  time; part of that is the hold (5-17 % of the shown time is a state up to 1.2 s old, marked `held`).
* The gdansk sample is 20 light-seconds: a direction, not a number.

Not done, and the likely next step if lights are still missed on the car: a detector with a better recall on
small lights, or a full-resolution pass, both of which cost compute the phone does not have to spare at
83-89 C; and first of all pictures from the car (the recorder) to see whether the camera or the detector
is the limit.

## Drive recorder

`api/roadsense_api/recorder.py`, on the phone since this lane (`/etc/roadsense/recorder.json`: directory
`/var/lib/roadsense/rec`, 4 GB cap). Per api start one session with the outgoing topics (road reduced) and
the camera's preview JPEG once a second. Checked on the phone with the fake daemons (messages, frames,
`/v1/health`). After a drive:

```sh
tools/lanes_replay.py SESSION/messages.jsonl.gz                 # lane flicker, raw against tracked, of that drive
tools/lead_replay.py SESSION/messages.jsonl.gz                  # the lead tracker with the real wheel speed
zcat SESSION/messages.jsonl.gz | grep '"type":"lights"'          # what was reported, with `ego`, `coast`, `held`
lights_replay --backend ort ... SESSION/frames/*.jpg            # what a detector finds in the pictures (1 per second)
```

## Trees (roadside scenery of the MFD scene)

Not a perception output: the car computer draws conifers beside the road when envd's place is
`forest_road`, standing on the road edges. Two causes, both fixed:

* **Display bug (the main one), fixed in the car computer** (`ui/scene/render.py`, commit `d5efcb4`): every
  tree's distance from the edge and its height came from `(x * 13) % 5` and `(x * 7) % 4` with x the tree's
  *current* distance, so each tree swept through 0-1.25 m sideways every 0.38 m of travel and changed height
  by up to 30 %: at 50 km/h a new random offset on every frame. Now keyed by the tree's index along the
  road; a test follows every tree over 100 m of travel.
* **The edges they stand on** jump as measured above; the tracked edges of the `lanes` topic are steadier
  by the factors in the table. The car backend has to take them: branch `feat/roadsense-tracked-lanes`
  (one commit: the scene feed prefers the `lanes` topic, raw lines as fallback), not merged by this lane.

## Open

* Nothing here has been seen on the car: the next drive with the recorder on is the check (lane flicker:
  `tools/lanes_replay.py <session>/messages.jsonl.gz`).
* The phone runs ncnn fp16 at 13.6 Hz with skipped frames; the replay is fp32 at 20 and 10 Hz.
* `/preview` still draws the raw lane lines.
