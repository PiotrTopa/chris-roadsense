# L16: the car's own traffic light, junction view, start-up delay

Built 2026-11-17 on the dev box (laptop), deployed to the phone the same day (the car was parked). Times CET.
Contract: docs/ARCHITECTURE.md ("light provider", "mapd" junction view, "Running under a host controller").

## Data: the first recorded drive from the car

Recorder session `20261117-101013` (L15's recorder): 29 min, 10:10-10:39, day, low sun, city and a forest
road; 63,079 messages, 1,576 frames (one a second). The phone rebooted at 10:39 with the file open: the gzip
stream ends in a broken block, everything up to the last complete line is readable (a streaming inflate that
stops at the error). A second session, `20261116-115104`, is 3 min parked.

What it settles from L15's open list, on the car's own camera (ncnn fp16, 13.8 runs/s):

| | raw | tracked (`lanes` topic) |
|---|---|---|
| lane lines, on/off changes per minute (four lines) | 82.1 | 2.8 |
| spans shorter than 1 s in 29 min | 2120 | 0 |
| road edge step per frame at 10 m, median / p95 (left; right) | 0.18 / 0.73; 0.11 / 0.44 m | 0.02 / 0.06; 0.02 / 0.06 m |

(`tools/lanes_replay.py`.) The raw flicker on the car is at the upper end of what the ride library showed
(18-70 per minute), the tracker removes it as it did there.

Traffic lights: 4,786 `lights` messages, a light in 26 %, an ego light in 18.3 % of them (10.6 % of those
held), 3 gaps shorter than 1.5 s. 16 approaches to signals with two or more tracked heads, up to 12 at once.

## Which light is mine

### What the drive shows

* **Two signal groups at once** (facing heads showing stop and go together): 5 spans, 21 s, 67 messages,
  8 % of the messages with a facing light. Three are real: 10:12:52 and 10:13:34 (a gantry with red for the
  left-turn lane and green for straight on, a green repeater on the right post) and 10:30:32 (red overhead
  for straight on, green arrows for the left turn two lanes away). Two (10:30:02, 10:30:04; 0.4 s each) are
  a second box beside the near light while it changed.
* The selection of L15 (a lateral window around the planned path, centre +1.5 m, scale 4.5 m) showed the
  right group in all of them. It is close in places: the left-turn head at 10:12:52 reaches relevance 0.22
  and the left-turn arrows at 10:30:32 reach 0.14 against a threshold of 0.15 (0.10 with a mapped signal).
* **Arrows cannot be read.** The provider's states are red, yellow, red_yellow, green, off, unknown. A
  housing is 11-30 px tall at 60-20 m (focal 975 px), a lens 4-9 px.
* **`off` for lights that are lit** is the most frequent error in the pictures: against the sun and beyond
  about 40 m the state classifier says `off` (one green light: 95 of 107 observations; a gantry in the glare:
  all of them). Such a light has relevance 0.15 x, so it is no ego light until it comes near. This is why
  the ego light is often late on this drive, and it is not addressed here.

### Lane assignment (`api/roadsense_api/lights.py`)

Design in docs/ARCHITECTURE.md. Every facing head gets a slot (offset from the ego-lane centre in lane
widths at its distance, from the tracked lane lines; the planned path with a 3.3 m lane without them) and a
lane weight whose tolerance grows with the distance. Heads are split into stop and go; with both present the
group with the better head wins, the ego light is chosen inside it and only it votes on the state. The
message carries the confidence, the other group (`alt`) and `amb`.

Decided by the replay (`tools/lights_lane_replay.py SESSION/messages.jsonl.gz --events`):

* A first version treated every head beyond the outermost lane as a roadside post of that lane with a lane
  factor of 0.6. On the gantry at 10:12:52 the red head over the left lane (slot -1.56, one lane to the left
  known) then scored like the green one over the ego lane and won one message. A post now counts as a head
  of the outermost lane (`slot_eff` = lanes up to it + half of the rest), never better than a head over it.
* With an unknown lane count on the left, heads two lanes to the left were accepted as "a post" and a
  left-turn head in its yellow phase became the ego light for 4 messages after the ego head had left the
  picture (10:30:35). A post is now assumed on the right only.
* A version that dropped a lone light over another lane lost true ego lights for the first 0.7 s (a light
  first seen in a bend: the offset from the path is off by a lane at 30 m) and near posts on the right. The
  assignment now never removes a light; that case is marked (`other_lane`, `amb`).

### Result on the drive

| | lateral window (L15) | with the lane assignment |
|---|---|---|
| messages with an ego light | 877 | 878 |
| ego light, share of the time; gaps < 1.5 s | 18.3 %; 3 | 18.3 %; 3 |
| conflict spans: group shown (checked against the frames) | right in 5 of 5 | right in 5 of 5 |
| conflict messages by group | 67 of 67 | 67 of 67 |
| confidence in the conflicts, min / median per span | - | 0.67 / 0.80 (lines still warming up after the camera came back), 0.82 / 0.89, 0.93, 0.79, 0.87 / 1.00 |
| messages marked ambiguous (`amb`) | - | 18 of 878 |
| stop <-> go changes with a change of track within 3 s | 4 | 4 |
| wrong-group messages seen | 0 | 1 (10:30:35, then held: the left-turn head in yellow, 0.3 s) |
| stress: the chosen group's heads removed, an ego light from the other group is shown | 10 % (7 of 67) | 10 % (7 of 67) |

Readings:

* **On this drive the assignment changes no outcome.** The window was right wherever two groups were in
  view, and in the stress test (the detector misses the ego head) both show the other group's light in the
  same 7 messages: 5 at 34-42 m, where one lane of lateral offset cannot be told (slot -0.97 .. -1.29), and
  2 from the duplicate box.
* What is new is what the message says: that two groups are there, which one was taken and with what
  share, and when the choice is not safe. A consumer can show that instead of a bare state.
* One message is worse than before (the yellow left-turn head, above), from the conflict branch choosing
  between two groups that are both not the car's. It is 0.3 s; the fix (an absolute lane-weight gate in a
  conflict) was not tried against more data, because there is none.
* Cost: 0.12 ms per message with three or more lights on the laptop (0.04 ms before), three messages a
  second.

Not done: arrows; the state classifier in glare; a map prior from turn lanes (the pack has lane counts only;
the count caps the lanes to the right, never exercised with a real position).

## Junction view (`mapd/roadsense_mapd/context.py`)

`map.ahead.junction_view` (docs/ARCHITECTURE.md): the arms of the next junction with their direction,
class, whether they can be entered, the most likely way on, the control (signals / give way / stop), and
for a roundabout the ring, its exits in driving order, the exit of the most likely path, and the car's
place while on the ring. From the pack as it is.

Checked: a hand-made network (a four-arm roundabout followed by a crossroads with signals,
`mapd/tests/test_junction_view.py`, 4 tests) and the Poland pack along a route through Opole (660
positions 8 m apart: a view at 503 junction and 12 roundabout positions, none at 145; read by hand at the
roundabout of Plac Wolności, the crossroads Kołłątaja / 1 Maja with signals, the dual-carriageway nodes of
Reymonta). Seen there and left as they are:

* A dual carriageway that splits or joins is a junction of its own with one side arm.
* The entry and exit carriageways of one road meet a small ring far apart (Ozimska at +7 and -87 degrees
  on a ring of 8 m radius) and stay two arms.
* The most likely path, and so the highlighted arm, follows the road number, the name and the class; at
  Kołłątaja / 1 Maja it turns left with confidence 0.58. A consumer should not draw it as an instruction.

No position source exists on the car, so no real drive has produced a view.

## Start-up: why the road picture came minutes into the drive

Not a GNSS wait: nothing in the stack waits for a position. `map` says `no_source` from the first message
and every vision topic is served from the first camera frame. Journals of the phone (2026-11-16 / 17):

| | 17 Nov 10:09:57 ACC on |
|---|---|
| +5 s | the host powers the camera hub (after its ACC debounce) |
| +10 s | the camera enumerates |
| +15 s | `roadsense.target` started (8 s settle) |
| +17 s | first `road` message (model warm-up 143 ms); lights, signs, env within the same second |
| +43 s | MFD board up, scene feed connected |
| **+45 s** | **the camera leaves the USB bus** (`usb 1-1.4.4: USB disconnect`); camerad-dvr retries every second |
| **+2 min 48 s** | the camera is back, leaves again after 2 s, is back for good at +2 min 54 s |
| +2 min 55 s | `road` messages again (the car had been driving since about +0:50) |
| +7 min 17 s | the whole USB tree re-enumerates from the camera's hub port (`cannot reset (err = -71)`, "Maybe the USB cable is bad?"): 6 s without frames, the CAN adapter and the gateway drop with it |

The same camera drop-out on every power-up in the journal:

| power-up (16-17 Nov) | leaves the bus after | away for |
|---|---|---|
| 13:34 preview (key off) | 27 s | 5 s |
| 17:39 drive, first after 4 h | 28 s | **122 s** (+2 s, +4 s) |
| 20:11 preview | 25 s | 5 s |
| 20:38 drive | 39 s | 4 s |
| 21:14 drive | 37 s | 4 s |
| 21:32 drive | 31 s | 4 s |
| 23:37 preview | 24 s | 4 s |
| 01:28 preview | never in 164 s | - |
| 10:10 drive, first of the day | 35 s | **123 s** (+2 s, +4 s) |

So: the DVR camera re-enumerates once after power-up, and on two of five drives, both the first after hours
of standing, it stayed away for two minutes. During that time the device is absent from the bus; nothing
on the phone can read it. The cause is inside the camera or its supply and is not known (candidates: its
own boot or storage check when cold, the supply while the engine starts; the head unit on the same hub kept
its USB connection through both). Thermal and model warm-up play no part: 55.7 C at the start, first output
2 s after the units start.

What was done:

* the host (car backend, its branch `feat/mfd-junctions`) powers the camera hub at the early start signal
  it already has (a door was unlocked: 13 s before ACC on 17 Nov, 32 s on 16 Nov 17:39), without starting
  the stack, for at most 180 s. The whole sequence moves ahead of the drive by that time, and on a warm
  start the 4 s drop-out falls before the stack is up. It does not shorten the 122 s;
* documented that video-only is the path from the first frame and the map joins and leaves with the fix.

Open: what the camera does in those 122 s. Next to try at a cold start: `uhubctl -l 1-1.4 -p 4 -a cycle`
after 10 s of absence (whether a port power cycle brings it back sooner), and the camera's state flags
(`recording`, `formatting`, SD present) in the seconds before it leaves.

## Deployed

roadsense on the phone 2026-11-17 (api with the lane assignment, mapd with the junction view), rollback
`<phone-home>/roadsense-installed-20261117-pre-L16.tgz`. The camera unit's masked state and
`/etc/roadsense/recorder.json` are as they were.

## Open

* The lane assignment has not been needed yet; the next drives with multi-group junctions decide whether
  the one known wrong case (two groups in view, neither the car's) needs the absolute gate.
* Lights that are lit and classified `off` (glare, distance) are the main reason an ego light is late.
* Junction view and the lane-count cap wait for a position source.
