# L17: one road geometry from camera, map and vehicle motion

Built 2026-11-17 on the dev box, deployed to the phone the same day (the car was parked).
Contract: docs/ARCHITECTURE.md (mapd `ahead.geometry`, `motion.turn_signal` / `ego.turn_signal`, api topic
`roadgeom`).

## Why

The consumer in the car draws the road from the camera's lanes (100 m) and, since L13, the map's road shape
beyond them; L16 added the next junction as a separate schematic. The request was one picture: the road
drawing itself should show a roundabout as a ring the lane runs into, a junction as roads leaving the road,
and the bend beyond the camera, joined with what the camera sees near the car. That needs the map's geometry
in the car's frame, lined up with the camera's, with a statement of how far each part can be trusted.

## What was built

* **mapd `ahead.geometry`** (`context.geometry`): the most likely path as a polyline (4 m steps to 200 m, 20 m
  to 600 m), the side roads at its junction nodes within 260 m as short polylines with class, direction of use
  and lane count, and the first roundabout as centre, radius, entry / exit distance and its arms in driving
  order. From the pack as it is. On a ring the path now takes the exit that was announced before the entry.
* **`turn_signal`** in the position input (`motion`) and in `ego`: an optional hook for the driver's intent.
  No host feeds it yet.
* **api `roadgeom`** (`roadgeom.py`): the fusion described in docs/ARCHITECTURE.md. The decisions that came
  out of the synthetic drives:
  * The car's heading in the map frame is the position filter's heading carried on by the yaw rate, not the
    direction of the path at the car's place: at the kinks of a junction (a ring's entry and exit are right
    angles in the map) a path-tangent heading swung the far geometry by up to 45 degrees (266 m at 350 m).
  * On a ring the position along it is taken from the heading. With 3 m of GNSS noise the matched position
    lagged the car by 5-8 m, which on a ring of 14 m radius is 20-40 degrees: the off-path check fired and the
    ring was lost half way round. Sliding the car along the path to where the ring runs in its direction keeps
    the ring for the whole passage.
  * The last fit of offset and angle is held by distance (150 m), not by time: a slow approach to a ring
    without lane lines outlasted a hold of a few seconds.
  * One unusable `map` message (a match-confidence dip in a junction) does not take the geometry away: the
    last usable one is used until it is 1.6 s old.
  * Side roads and the ring are shifted from the way's centre line to where they are for a car in its lane.

## Results

Synthetic drive over a made-up network (`tools/roadgeom_drive.py --network tour`: 1.9 km with a bend of 180 m
radius, a signal-controlled crossroads with a service road, a roundabout of 14 m radius with four arms, a bend
of 110 m radius, a side road; GNSS noise 3 m correlated, the synthetic camera sees 70 m, loses the lane lines on
the ring and 14 m before it):

| | |
|---|---|
| frames with the map part in use | 96.5 % (990 frames at 10 Hz) |
| withheld | 2.7 % `disagree` (the two seconds after leaving the ring, while the car straightens), 0.6 % before the first fix |
| ring in the message | from 215 m before the entry to the exit, without a gap |
| fused centre line against the true lane centre, open road | median 0.27 m at 0-50 m, 0.54 m at 100-150 m, 1.2 m at 350-400 m; p95 0.5 / 2.6 / 7.7 m |
| the same within 100 m of the ring and on it | median 0.4 m at 0-50 m, 0.6 m at 100-150 m; p95 4.0 / 7.3 m |
| turn signal right before the crossroads / left before the ring | the right-hand side road / the exit beyond the path's own is the way on in every frame of the signal |

The far errors are angular (about 1 degree of the fitted heading): at 300 m they are 2-3 pixels of a 480 x 240
screen. On a route through a city centre of the Poland pack (4.2 km, 332 s) the map part was in use in 82 %
of the frames (10 % `disagree`, 6 % `no_fit`, 1 % `off_path`): the synthetic car follows a route with turns,
the most likely path does not know it, and the map part is withheld while the two differ.

The first recorded drive from the car (L16; no position source): `tools/roadgeom_replay.py` gives the camera's
part alone in every message. While driving the centre line came from both inner lines in 74 %, one line in
20 %, the planned path in 5 % and nothing in 0.5 % of the frames; its reach was the full 100 m in the median
and 91 m at the 10th percentile.

Cost: laptop 0.09 ms per message without a map, 0.36 ms with one, 0.17 ms per `map` message; mapd's
`look_ahead` with the geometry: see the phone numbers below.

## Not done / open

* No real position has gone through this: there is no GNSS source on the car yet. Everything map-based is
  from synthetic drives. The first drives with a fix must show how often camera and map agree on real roads
  (`map.why` in the recorder), and what the fit looks like on wide roads, where the way's centre line is far
  from the lane.
* The pack has no lane-level geometry, turn lanes or turn restrictions: a roundabout is a circle of the mean
  radius of its way, a side road is a centre line with a width from its lane count.
* The turn-signal convention at roundabouts (left = beyond the path's exit) is a guess to be checked against
  how the driver signals, once a host decodes the lamps.
* `roadgeom` is not in the recorder's default topics (it can be recomputed from `road`, `ego` and `map` with
  `tools/roadgeom_replay.py`).
