# Lane `detin`: richer detection inputs for the tracker

2026-11-24. Branches `detin` (roadsense-signs) and `v2-detin` (roadsense). Nothing was deployed; nothing in
chris-integrator was changed.

## Why
chris-integrator's tracker (the "radar") gets a box, a class and a score per road user. What is left of its
false warnings in simulation and its weak spots on real drives (range error at 25-60 m, row hops and phantom
velocities on parked cars, car / van box swaps, far trucks, pedestrian boxes 3 m long) comes from what that
input leaves out. This lane makes the detector (`signd --lights-road-users`) deliver five more inputs, each
switchable, each off by default, each with its cost measured, and states how the tracker should take them.

Documents:
* the contract: [ARCHITECTURE.md](../ARCHITECTURE.md), light provider (keys `road_users_low`, `emb_kind`; per
  road user `trunc`, `occ`, `occ_by`, `occ_edges`, `sym`, `lamps`, `emb`, `scale_src`);
* what was built and measured, in detail: roadsense-signs `docs/DETECTION_INPUTS.md`;
* how the tracker should consume each field, and what the simulator must emit:
  [detin-consumption-design.md](detin-consumption-design.md);
* appearance distance tables for the tracker's likelihood ratio: [detin-emb-distances.json](detin-emb-distances.json).

## The five inputs: state

| # | input | switch (signd) | state |
|---|---|---|---|
| 1 | appearance vector per box | `--lights-ru-emb yuv` / `FILE.onnx` | built: a model-free vector (`yuv3x3`), and a small head trained without labels that is published together with it (`reid32+yuv3x3`; the trained file stays local); a tap on the detector's feature maps was measured and rejected |
| 2 | low-score boxes for a second association stage | `--lights-ru-low X` | built (list `road_users_low`); the tracker's half is design |
| 3 | occlusion and truncation flags | `--lights-ru-flags` | built |
| 4 | orientation and ground contact | `--lights-ru-view` | partly: lamp pair (which end faces the camera, a ruler) in the dark, mirror symmetry (not validated); heading and wheel contact points need a trained head: design and data list only |
| 5 | scale rate at night | `--lights-ru-scale-lamps`, `--lights-ru-scale-multi N`, and `--lights-ru-low` | built: lamp-pair separation as a second measurement, several baselines, chains continued by low boxes |

With every switch off the `lights` messages are byte for byte the previous ones (checked with the final binary:
every message line of a day and a night ride, 3000 passes, against the outputs of roadsense-signs 12a960e, the
time fields set aside; and with everything on the first list is the same, box for box, on five rides). Tests:
`ctest` in roadsense-signs, 5 of 5 (the `scale_rate` test now renders night scenes too and takes minutes).

## Evidence, per input

### 1. Appearance vector
Question from the brief: a tap on the detector's own feature map, or a dedicated head? Measured on rides
(identities from dense 20 fps chains, no labels; query against every box of every other chain of the ride):

| vector | day (warsaw-city): rank-1 at 1 / 2 / 4 s | night (krakow-night-tunnel) |
|---|---|---|
| best tap on the detector's feature maps (backbone layer 6, 128 numbers; wide view, 5 min) | 0.66 / 0.72 / 0.80 (AUC 0.93-0.94) | 0.43 / 0.24 / 0.03 |
| 3 x 3 cells of mean Y, Cb, Cr of the crop, 27 numbers (same boxes) | 0.70 / 0.74 / 0.82 (AUC 0.95-0.98) | 0.48 / 0.23 / 0.09 |

No tap (backbone, neck, head branches; raw, standardised, PCA) beats nine coloured cells of the picture: the
detector's features are trained to give every car the same answer. So the cheapest source is not the feature
map but the pixels: **`yuv3x3`** (`--lights-ru-emb yuv`, no model, microseconds per box). A small dedicated head
was then trained without labels (identities = chains of 8 rides, 1470 chains, supervised contrastive loss; 214 k
parameters, 32 x 32 crop) and tested on two rides it never saw, full frame, 10 minutes each:

| held-out ride | score | `yuv3x3` at 1 / 2 / 4 s | learnt 32 | both (published as `reid32+yuv3x3`) |
|---|---|---|---|---|
| warsaw-city, day (21 022 vehicle boxes) | rank-1 against the other boxes of the frame (4.1 on average) | 0.948 / 0.922 / 0.883 | 0.968 / 0.945 / 0.920 | **0.975 / 0.954 / 0.931** |
| | rank-1 against every other chain of the ride | 0.605 / 0.469 / 0.353 | 0.616 / 0.491 / 0.395 | **0.719 / 0.598 / 0.463** |
| krakow-night-tunnel, night (1844) | in the frame (1.0 other box, chance 0.5) | 0.900 / 0.808 / 0.750 | 0.940 / 0.862 / 0.862 | 0.907 / 0.869 / 0.802 |
| | against the ride | 0.496 / 0.346 / 0.169 | 0.484 / 0.388 / 0.221 | **0.621 / 0.455 / 0.279** |

By day the head on top of the cells roughly halves the misses of the frame-level association (5.2 / 7.8 /
11.7 % -> 2.5 / 4.6 / 6.9 %); at night every vector is weak (vehicles are lamp pairs on dark bodies). Built:
both kinds in signd, the training and evaluation tools, distance tables for the tracker's likelihood ratio
([detin-emb-distances.json](detin-emb-distances.json): with log-normal fits and a ratio clamped to [1/8, 8], by
day 79 % (`yuv3x3`) / 86 % (with the head) of other-object pairs get a ratio of 1/4 or less at 1 s, 3-4 % of
same-object pairs do; at night 23-34 % against 5-6 %).

Caveats: identities are IoU chains, not labels; the tables come from full-frame crops at 10 fps, and the same
vector as signd publishes it in the unit's views (boxes cut at the view's bottom, 5 passes a second) shows
larger distances on both sides (same object about 1 s apart: median 0.54 by day against 0.54 in the table but a 90 % point of 2.1 against 1.3; at night medians 1.2-1.4 against 0.8): the tables must be re-derived from the unit's own messages before
a tracker's ratio is tuned (roadsense-signs `tools/road_users/emb_check.py`). The trained file stays on the
laptop (`<datasets>/detin-lane/out/reid32.onnx`), like the other locally trained weights.

### 2. Low-score list
`road_users_low`: the boxes between a floor X and their class threshold that no first-list box overlaps. A key
of its own, so a consumer that does not know it sees nothing new; the first list is unchanged by it.

BDD100K val, 3000 images, the whole frame (384x640 export), 37 488 labelled road users:

| | low boxes per image | of them on a label the first list missed | labels found | ... labelled occluded | ... under 30 px |
|---|---|---|---|---|---|
| first list alone | | | 46.6 % | 34.3 % | 15.3 % |
| + low, floor 0.20 | 3.4 | 56 % | 62.1 % | 52.3 % | 36.1 % |
| + low, floor 0.10 | 6.7 | 41 % | 68.7 % | 60.1 % | 45.6 % |

At a floor of 0.10 the low list holds 41 % of the road users the first list misses. A low box is sloppier: edge
error 10-13 % of the box height against 4 %, median IoU with the label 0.67 against 0.87, class right 95 %
against 98 %.

On the rides, the unit's views, floor 0.10 (no labels; a gap = a pass in which the first list misses an object it
has before and after):

| ride (5 min at the unit's schedule) | first-list boxes a pass | low boxes a pass | gap passes with a low box on the object | low boxes on no first-list object |
|---|---|---|---|---|
| warsaw-city (day) | 0.48 | 1.22 | 23 of 34 (68 %) | 93 % |
| a1-night-fog | 0.30 | 1.04 | 70 of 93 (75 %) | 87 % |
| krakow-night-tunnel | 0.16 | 0.55 | 6 of 7 | 95 % |
| gdansk-night-snow | 0.05 | 0.37 | 4 of 4 | 95 % |
| a1-night | 0.03 | 0.62 | no gap pass | 99 % |

Where the first list loses an object it has before and after, a low box lies on it in two passes of three or
more (at a floor of 0.20 still 55-86 %, with a third of the low boxes). Most low boxes are on nothing the first
list ever shows; how many of those are real objects is not known on the rides (no labels).

### 3. Flags
`trunc`: the frame edges that cut the box (`clip` stays the view's edges). `occ` / `occ_by` / `occ_edges`: how
much of the box nearer boxes cover, which one, which of its edges. Nearer = bottom edge lower; a cut bottom is
lower than it shows; ties count both ways (the flag errs towards "covered").

BDD100K val, 3000 images, against the labels' own attributes (roadsense-signs `tools/road_users/detin_bdd.py`):

| | of the flagged, labelled so | of the labelled, flagged |
|---|---|---|
| `trunc`, detector boxes on a label (17 481) | 97.9 % | 94.5 % |
| `occ` > 0, detector boxes | 93.0 % | 72.5 % |
| `trunc`, computed on the label boxes (37 488) | 98.3 % | 86.5 % |
| `occ` > 0, on the label boxes | 93.7 % | 74.1 % |

`occ` >= 0.3: 97 % labelled occluded; (0, 0.1): 80 %; no flag: 40 % (hidden by things that are no boxes). So a flag
takes trust away and no flag gives none. The depth order of two overlapping boxes has no label to be held against
(unit checks only), and on the rides at the unit's views the flags are rare (the views end near the horizon:
boxes seldom overlap there).

### 4. Orientation and ground contact
What the present model gives: a box. From the box's own pixels, without a model, two cues were built
(`--lights-ru-view`, vehicles only):

* **`lamps`**: in a dark frame, the vehicle's lamp pair: centroids, separation with its 1-sigma, colour. Red =
  the rear faces the camera, white = the front. Found on 142 of 195 vehicle boxes of the night ride at the unit's
  schedule (red 93, white 30, unknown 19), on 2 of 629 by day (the darkness gate). By eye on a sheet of 48 night
  boxes: 30 pairs, 28 on the vehicle's two lamps with the right colour, one on a lamp and a reflection, one on
  background lights. Separation noise (scatter about its local trend, 54 triples): 0.31 px at 12-20 px, about 2 %
  above 40 px, 0.8 px under 12 px (unusable there). No labelled truth for the colour beyond the sheet.
* **`sym`**: mirror symmetry of the box's image. **Not validated**: on the ride views the only clean day boxes
  were 16 side views of cars on a bridge, which scored 0.02-0.89: a car's side is itself nearly symmetric at this
  resolution. Without orientation labels it cannot be said what `sym` separates; it is published as a raw cue and
  the consumption design gives it no weight of its own.

**Ground contact.** The detector's contact row is the box's bottom edge; the flags say when it is not one
(`clip` / `trunc` b, `occ_edges` b). Range from it is the consumer's geometry with the calibrated camera and
the road model (consumption design, section 5: the vertical-profile term and the width ruler), which signd does
not have. Pixel rules for the contact row (the shadow edge under the vehicle) and for the view class were
considered and not built: there is nothing on this machine to hold their bias against.

**What needs training** (design only; roadsense-signs `docs/DETECTION_INPUTS.md` section 6): a small head on
the box crop (viewpoint as an angle, wheel contact points of the visible side, per-edge visibility), trained on
3D-box data sets (KITTI 3D object, nuScenes, Cityscapes 3D; ApolloCar3D for wheel points) and brought to the
car's camera with a few hundred own labels, as the light-state model was. None of that data is on this machine;
nothing was downloaded. This is the blocker for input 4 proper.

### 5. Scale rate at night
**Why it is worst at night** (the night ride at the unit's schedule, 195 vehicle boxes, patch alignment of
2026-11-23: 70 with a value, stated std 0.046-0.056 /s against 0.009 by day): (1) the detector misses in runs,
so chains restart (58 boxes were the first of their chain); (2) a vehicle in the dark is two lamps on a dark
body: little texture, and the lamps are clipped blobs whose size follows the exposure, so 35 boxes were measured
and refused for their std, 17 for a poor match or fit; (3) 5 passes a second at best, about half of them with
a box of a given object.

**What was built:** the lamp pair's separation as a second measurement (a robust line through ln(separation)
over the chain's passes; combined with the patch value, `scale_src`), several baselines per box, and chains that
the low list continues.

| ride (unit's schedule, 5 min) | first-list boxes | patch alone | + lamp pair | + 3 baselines | all (+ low list) | far boxes: agreement of successive values, alone -> all (1 = honest) |
|---|---|---|---|---|---|---|
| krakow-night-tunnel | 195 | 70 (36 %) | 93 (48 %) | 91 (47 %) | **109 (56 %)**, + 34 low boxes | 0.68 (48 pairs) -> 0.83 (79 pairs), none beyond 3 sigma |
| a1-night-fog (no part in the lamp noise model) | 334 | 162 (49 %) | 174 (52 %) | 193 (58 %) | **196 (59 %)**, + 100 low boxes | 0.95 (113 pairs) -> 0.78 (135 pairs), none beyond 3 sigma |
| gdansk-night-snow | 55 | 21 | 21 | 21 | 20, + 9 | too few |
| a1-night | 33 | 7 | 7 | 7 | 7, + 14 | too few |
| warsaw-city (day) | 667 | 259 (39 %) | (no dark frame) | 275 (41 %) | 275 (41 %), + 148 | (near objects: real dynamics) |

So at night 36 / 49 % -> 56 / 59 % of the boxes carry a scale rate, with the values still agreeing within their
stated std. **The stated std of a far box does not shrink** (0.03-0.05 /s): the gain is coverage, not precision.
Patch and lamp value of the same box agree without a bias on the tuning ride (17 boxes: median difference
-0.0003 /s) and differ by -0.023 /s on the fog ride (27 boxes; 0.45 of their joint sigma): glare in fog moves
the blobs. Lamp-only values (8-12 boxes a ride) are the weak part: successive lamp-only values of far objects
disagreed by 1.35-1.6 of the line fit's std, so they are published with that std times 1.5 (the factor comes
from these two rides: with it 1.05 and 0.90, 45 pairs, none beyond 3 sigma); a consumer should still take
`scale_src` "lamps" with a robust likelihood.

Synthetic night scene (blooming, exposure changes, motion blur; `build/scale_rate_test`): the patch alignment
loses half of the far boxes and understates its error (pull 1.4-1.6); the lamp pair covers 67-100 % with no
value beyond 3 sigma; together 86-100 %.

**The remaining limit is the sample rate**: 3-5 samples a second per object. Measuring the lamp pair on the
camera frames between two detector passes (the ring holds six) would give 8-16 points a second; designed, not
built.

## Cost

Laptop, one thread, taken while the laptop was loaded by other jobs (the scale figures are about twice a quiet
laptop's; the microsecond figures moved little with the load). Phone: estimates with the factors the project
has documented (big core 3-5 x a quiet laptop, little core about 10 x; roadsense-signs `docs/LIGHT_MODELS.md`).
Nothing was measured on the phone.

| feature | laptop (measured) | phone (estimated) |
|---|---|---|
| low list | inside the decode: decode + flags + view + emb per pass 0.09-0.12 ms median (0.06-0.09 with all off) | well under 1 ms a pass |
| flags | 0.3-0.7 us per box | microseconds |
| `sym` | 9-56 us per vehicle box (by size) | big 0.03-0.2 ms, little 0.1-0.5 ms per box |
| `lamps` | 9-140 us per vehicle box, dark frames only | 12 mid-sized vehicles in the dark: about 2-3 ms (big) / 6 ms (little) a pass |
| `emb` yuv3x3 | 1.4-8.4 us per box | little core under 0.1 ms per box |
| `emb` with the learnt part | 0.63 ms per box (one ORT run), cap 16 a pass | about 3 ms per box on a big core: up to 50 ms a pass at the cap (a quarter of a big core at 5 passes a second) |
| scale: lamp pair | 0.03-0.17 ms per box | big 0.05-0.4 ms per box |
| scale: 3 baselines | 2.5 x the patch alignment (0.70-1.5 ms per box against 0.28-0.63) | big cores 1-4.7 ms per box against 0.4-1.9; worst pass at the 12-box cap 32-55 ms against 13-22 |

Per pass on the rides with everything on (model-free vector): scale part p90 1.1-2.5 ms against 1.1-1.6 with
the patch alone, the worst pass 3-5 ms; the `lights` message grows by 20-45 % (per box: `emb` 62 bytes,
flags 11-56, `lamps` about 90; a low box about 150). No GPU work, no new thread, no new dependency; the governor's
`road_users_hz` 0 sheds all of it with the road users.

## Order in which to switch on

What helps the present tracker without any change on its side (it already reads `scale_rate`):
1. `--lights-ru-scale-multi 3`: more night boxes with a scale rate at an unchanged agreement of successive
   values; costs 2.5 x the scale part: measure on the phone first (estimated worst pass 32-55 ms on a big core at
   the 12-box cap against 13-22 today).
2. `--lights-ru-scale-lamps`: a few more boxes with a value in the dark for next to no cost; lamp-only values
   have a heavier tail, so switch it on once the tracker takes `scale_src` "lamps" with a robust likelihood
   (consumption design, section 6), not before.

What needs the tracker's side first (order = value for effort; consumption design section 8):
3. `--lights-ru-flags`: free; the tracker stops reading sizes and ground points off hidden edges.
4. `--lights-ru-low 0.10`: a fraction of a millisecond; the tracker's second kind of detection.
5. `--lights-ru-emb yuv`: microseconds, 56 bytes per box in the message; the appearance likelihood ratio.
6. `--lights-ru-view`: up to a few milliseconds a pass in the dark; lamp colour for heading and birth, then the
   separation as a size row.
7. `--lights-ru-emb FILE.onnx` (the learnt part): only after its phone cost is measured (estimated 3 ms per box
   on a big core) and its tables are re-derived on the car's camera.

## Not done, not verified

* **Nothing is consumed.** The tracker's half of every input is design
  ([detin-consumption-design.md](detin-consumption-design.md)); the brief forbade editing the integrator. No
  effect on the tracker's errors (range at 25-60 m, row hops, box swaps, `obstacle_ahead`) is measured.
* **Nothing ran on the phone or the car.** All costs are laptop figures (taken while the laptop was loaded by
  other jobs) and estimates for the phone from the project's documented factors. signd itself (the daemon with
  the ring) was built with the options but not run live; everything was exercised through `lights_replay`, which
  shares the pipeline code.
* **The car's own camera has not seen any of it.** The recorder's sessions hold one frame a second: no chains,
  no scale rate, no re-identification can be checked on them. All ride evidence is from the ride library (other
  cameras, YouTube rides at 20 fps); the night evidence rests on one ride with enough vehicles
  (krakow-night-tunnel: 195 first-list boxes at the unit's schedule) plus three thin ones.
* **Input 4 is the weakest.** Heading and wheel contact points are not delivered (no model, no data here).
  `sym` is published without a validation. The lamp colour was checked by eye on one sheet only.
* **Depth order of the occlusion flag** (which of two overlapping boxes is nearer) is a rule with unit checks,
  not a measured accuracy; BDD100K's attributes say only whether a box is occluded at all.
* **The low list's value on the unit's views** is shown by labels only on BDD100K (whole frame, another input
  size). On the rides there are no labels: what share of the low boxes "on nothing known" are real, never-boxed
  objects and what share clutter is not known.
* **The learnt embedding** is trained on chains of 8 rides from the ride library and tested on 2 held-out rides;
  identities are IoU chains, not labels, so another chain of the same car counts as a different object in the
  ride-wide score. Not trained or tested on the car's camera.
* **Lamp-only scale rates rest on a correction factor.** Their line-fit std understated their disagreement
  (pairs of blobs that change, glare in fog), so it is multiplied by 1.5, a factor set on the same two night
  rides it is checked on (45 pairs of far objects); on another camera it may be wrong either way. Measuring
  the lamp pair on the camera frames between two detector passes (8-16 points a second instead of 3-5) is the
  designed remedy and is not built.
* Not attempted: a feature-map tap in signd (measured in Python, rejected), ground-contact refinement from
  pixels, any dataset or model download.

## Reproduce
roadsense-signs `docs/DETECTION_INPUTS.md`, section "Reproduce". Scratch (not in git): the laptop's
`<datasets>/detin-lane/` (builds, ride outputs, crops, the trained `reid32` files).
