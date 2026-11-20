# Labelling brief (lane L11, roadsense): what traffic rules apply to the car, read by eye from a recorded Polish drive

You label ONE ride. You never see any engine output; you write what a careful Polish driver would say from the frames.

## Material
* Contact sheets: `~/l11/frames/<slug>/sheet_NNN.jpg` — 12 frames per sheet, one frame every 2.0 s, the ride time (m:ss.s)
  printed on each tile. Look at EVERY sheet in order (Read tool). Signs are small on the sheets: whenever a tile shows a
  sign post, a junction, a village entry or anything doubtful, pull full-resolution frames with
  `~/projects/roadsense/.venv-models/bin/python ~/l11/scripts/frame.py <slug> T1 T2 ...` (ride seconds, 0.05 s
  steps are fine; prints the JPEG paths; ~2 s per call) and Read them. Be generous with full frames: a sign is readable
  only when it is close (one or two frames before the car passes it). Typical need: 30-80 full frames per ride.
* The video is 20 fps; frame.py time = ride time. The car drives on the right (Poland).

## What to label (ride seconds, one decimal)
A. `intervals` — lasting rules for THIS car, each as start/end ride time:
   * `priority_road`: D-1 (yellow diamond) start/continuation … until D-2 (diamond with a black bar) or a yield A-7 / stop B-20
     facing the car. D-1 repeats before junctions: ONE interval from the first D-1 to the end (don't split on repeats; note
     repeat times in `notes`).
   * `no_stopping`: B-36 (blue disc, red ring, red X) … until the NEXT INTERSECTION (PoRD: a junction with another public road
     at one level; NOT driveways, field/dirt tracks, car-park or petrol-station entries), or B-35, or a B-36/B-35 with a
     T-25c "end" plate (plate with an arrow pointing back / diagonal). A new B-36 after the junction = a new interval.
     `side`: which side of the road the sign stands (right = our side).
   * `no_parking`: B-35 (blue disc, red ring, one red slash) … same ending rules.
   * `no_overtaking`: B-25 (white disc, red ring, two cars) … until B-27 (grey end sign) or B-42 (grey disc, diagonal lines,
     "koniec zakazów") or the next intersection.
   * `height_limit` / `weight_limit` / `width_limit`: B-16 / B-18 / B-15 (red ring with a number + m / t) — note the number.
     Treat as in effect until the next intersection (engineering assumption).
   * `living_street`: D-40 (blue, house + people) … until D-41.
   * `built_up_area`: D-42 (white rectangle, black town silhouette) … until D-43 (same with a red bar). The green town-name
     sign E-17a is NOT D-42; note it separately in `notes`.
   * `motorway` D-9 / `expressway` D-7 (blue with a car) … until D-10 / D-8.
   * `one_way`: D-3 (blue rectangle, white up arrow, on OUR road) … until the next intersection.
   Each interval: {"rule": ..., "start_t": ..., "end_t": ..., "start_sign_t": <time the starting sign is passed>,
   "end_reason": "D-2|yield|stop|junction|B-42|B-35|B-36|end plate|D-43|D-41|ride end|unclear", "end_unclear": true/false,
   "side": "right|left" (B-35/B-36 only), "value": <number for dimension limits, else null>, "notes": ""}.
   `end_t` = when the car passes the ending sign or the centre of the ending intersection. If you cannot decide whether a
   crossing road is a public road (so whether the rule ends there), set `end_unclear: true` and give your best `end_t` plus
   `alt_end_t` for the other reading.
B. `momentary` — each relevant sign the car PASSES (time of passing, i.e. when the sign leaves the frame or is level with
   the car): no entry B-2 (red disc with a white bar; note whether it faces our carriageway or a side road), stop B-20,
   yield A-7 (triangle point down), no left/right turn B-21/B-22, no U-turn B-23, roundabout C-12 (blue, three arrows),
   mandatory direction C-2..C-8 (blue disc, white arrow), pedestrian crossing D-6 (blue square with a walker), one-way of a
   side road D-3 pointing sideways, no overtaking by trucks B-26, no trucks B-5, dead end D-4a, priority over oncoming D-5 /
   give way to oncoming B-31. {"sign": "B-2", "t": ..., "side": "right|left|overhead", "faces_us": true/false (B-2 only),
   "notes": ""}
C. `warnings` — each warning sign (yellow triangle, red border) passed: A-5 crossroads, A-6a/b/c side road, A-7 is NOT a
   warning here (it is in B), A-8 roundabout ahead, A-16 pedestrian crossing ahead, A-17 children, A-29 traffic signals,
   A-9/A-10 level crossing, A-14 roadworks, A-1..A-4 curves, A-11/A-11a bump, A-12 narrows, A-15 slippery, A-18 animals,
   A-20 two-way, A-21 trams, A-24 cyclists, A-30 other danger (use "A-?" with a description for others; note T-1/T-2 plates).
   {"sign": "A-8", "t": ..., "notes": ""}
D. `junctions` — every intersection the car passes or turns at (centre time), with "type": "signalised|side road right|side
   road left|crossroads|roundabout|T end", and "public": true/false/null (null = cannot tell: driveway or road?).
   This is the key to "until the next intersection", so be complete: a side street with asphalt, a name plate, markings or
   its own signs is public; a farm track, a gate, a shop entry, a petrol station is not.
E. `notes` — free text: anything odd (plates you could not read, signs for other vehicles, temporary signs, speed limits
   seen (B-33 with value and time) — speed limits are NOT required but welcome).

## Output
Write `~/l11/labels/<slug>.json` with keys ride, labeller ("opus-L11-labeller"), intervals, momentary, warnings,
junctions, notes, and `coverage`: {"sheets_read": N, "full_frames": N}. Valid JSON (check with python -m json.tool).
Then reply with a short summary: counts per category and the 5 most doubtful items. Do NOT read anything under
~/l11/rec or the roadsense repository's api code; do not edit anything else. Absolute paths only.
