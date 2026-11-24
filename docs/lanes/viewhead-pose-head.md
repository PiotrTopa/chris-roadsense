# Lane `viewhead`: a trained heading and ground-contact head

2026-11-24. Branches `viewhead` (chris-signs, on top of `detin`) and `v2-detin` (this repository); a new
repository, **chris-pose**, for the model. Nothing was deployed; nothing in chris-integrator was changed; nothing
ran on the phone or the car.

## Why
Lane `detin` gave the tracker flags, a low-score list, an appearance vector, lamp pairs and a better scale rate,
and could not give the heading by day or the points where a vehicle meets the road: both need a trained model and
training data that were not on the machine ([detin-detection-inputs.md](detin-detection-inputs.md), input 4). The
owner approved a public 3D-box data set. This lane trained the head, wired it into the detector as an option,
measured it, and wrote down how the tracker should take it.

Documents:
* the contract: [ARCHITECTURE.md](../ARCHITECTURE.md), light provider, "Pose of a road user" (`pose`, `pose_kind`);
* the detector side (option, contract of the model file, measurements through its own code path, cost):
  chris-signs `docs/POSE_HEAD.md`;
* the model (code, model card with the full evaluation, licences): chris-pose;
* how the tracker should take it: [detin-consumption-design.md](detin-consumption-design.md), section 9.

## Three repositories, two licences
| | what it holds | licence |
|---|---|---|
| chris-signs | the machinery: `--lights-ru-pose FILE.onnx`, the crop and box numbers fed to a model file, the reading of its outputs, the message keys, tests against a stand-in model that is a written-out linear map | AGPL-3.0 |
| chris-pose | data preparation, training, calibration, evaluation, export; model card; no data, no weights in git | code MIT; **weights CC BY-NC-SA 3.0** |
| chris-roadsense | the contract and the consumption design | MIT |

The head is trained on KITTI, which is CC BY-NC-SA 3.0. The project treats weights trained on a data set as derived
from it, so the weights are non-commercial and share-alike. An AGPL work cannot carry a non-commercial
restriction, so the weights are not part of chris-signs: they are a separate file the daemon loads at run time, and
chris-signs builds, tests and runs without it. Consequences: fine for a private, non-commercial car; not to be
bundled into anything commercial; a commercially usable head needs retraining on permissively licensed or
synthetic data (the training code is MIT). The owner decides what ships; the trained files are on the laptop only
(`<datasets>/viewhead-lane/out/pose1-m64.onnx`), in no git. This is the project's reading, not legal advice.

## Data
| | |
|---|---|
| downloaded | KITTI 3D object benchmark: left colour images of the training and test split (12.6 GB zip), labels, calibration, development kit; KITTI tracking benchmark: left colour images (15.8 GB zip), labels, calibration. From the benchmark's public file server, no account, nothing about the owner sent |
| licence | CC BY-NC-SA 3.0 (Geiger, Lenz, Urtasun, CVPR 2012) |
| used | 7481 + 8008 pictures of the two training sets as JPEG (3.0 GB), read from the zips; the zips were deleted afterwards. 85 334 labelled objects at least 10 px tall (65 596 vehicles, 16 174 persons, 3 564 cyclists), 72 234 with a box of the detector |
| not used | nuScenes, Cityscapes 3D, ApolloCar3D, Waymo, KITTI-360: registration; A2D2: no-derivatives licence; Argoverse 2: openly downloadable, not needed for a first head (a candidate second camera for a later round) |
| disk | peak about 37 GB for a few minutes (both zips, the Python environment and the JPEGs at once: over the 30 GB aimed at), 15 GB now (environment 6.9 GB, records 4.2 GB, JPEGs 3.0 GB, ride frames and outputs) |
| split | by raw drive: 99 drives train (62 742 objects), 12 calibrate (5 372), 30 test (17 220); the tracking sequences tied to their raw drives by matching pictures, so a held-out drive does not come back through a sequence |

## The model
One network run per box: the box scaled by 1.3 stretched to 64 x 64 plus nine numbers about the box (aspect,
which edges are cut, optionally the box in units of the focal length); four small convolution stages, the 4 x 4
map flattened, two linear layers; **495 k parameters, 16.0 M multiply-accumulates** (the appearance head: 214 k,
12.1 M). Outputs: heading as 24 bins with residuals (uncertainty, opposite-direction probability and view class
come from the bins), the footprint's centre `gc`, near-face point `gn` and outer visible corners `gl`, `gr` with a
1-sigma and a "seen" probability each, dimensions, occlusion, truncation, kind. The exported file carries its
calibration (fitted on calibration drives) and gives finished numbers.

Trained on the detector's own boxes on KITTI (half of the crops) and jittered label boxes, with boxes cut as a
view's or the picture's edge cuts them, mirror images, exposure / gamma / blur / noise / loss of resolution, and
night-like crops with the vehicle's lamps drawn where the 3D box puts them (red on the rear, white on the
front): KITTI has no night.

## Results on held-out drives
12651 vehicles, the detector's boxes, camera numbers not given (all tables: the model card).

| heading | median error | over 30 deg | flipped |
|---|---|---|---|
| all | 2.5 deg | 8.8 % | 6.2 % |
| rear / front views | 1.7 / 2.0 deg | 6.3 / 3.5 % | 4.9 / 3.4 % |
| three-quarter | 4.3 deg | 13.1 % | 8.9 % |
| side | 14.2 deg | 36.9 % | 17.6 % |
| 0-15 / 15-30 / 30-50 / over 50 m | 2.3 / 2.1 / 3.0 / 3.2 deg | 9.9 / 7.3 / 10.6 / 7.4 % | 8.0 / 5.2 / 6.6 / 4.9 % |
| visible / partly / largely occluded | 2.0 / 3.2 / 3.5 deg | 6.1 / 10.1 / 13.5 % | 4.8 / 7.0 / 8.0 % |
| persons / two-wheelers | 23.5 / 47.9 deg | 43.6 / 57.1 % | 18.2 / 33.9 % |

Persons and two-wheelers: not usable; the detector publishes no pose for them (declared unknown).

Calibration of the heading: in the right lobe 68.6 % of the errors within one stated sigma, 91.7 % within
two, 2.8 % beyond three (heavy tails); the stated flip probability and lobe probability are honest over
their range (stated 0.004: flipped 0.003; stated 0.032: flipped 0.035; stated 0.072: flipped 0.090; stated 0.144: flipped 0.154; stated 0.269: flipped 0.272; stated 0.415: flipped 0.426). Over time: half of the error is the object's own (sigma 2.4 deg), errors
correlate 0.65 at 0.2 s and 0.33 at 1 s; flips come and go.

Ground contact, as the range it implies (relative error %, median / 90 % point (bias); the object's own ground
height, so only the pixel error counts; visible uncut vehicles):

| distance | n | centre: `gc` | centre: box bottom | near face: `gn` | near face: box bottom | near face: box height x class prior | ... x the head's height |
|---|---|---|---|---|---|---|---|
| 0-15 m | 1281 | 4.1 / 10.2 (-3.6) | 14.5 / 20.3 (-14.5) | 3.4 / 8.2 (+1.8) | 5.0 / 12.2 (+4.7) | 6.2 / 17.9 (+4.6) | 6.5 / 16.4 (+1.9) |
| 15-30 m | 1934 | 2.4 / 6.0 (-1.0) | 8.6 / 13.1 (-8.6) | 2.5 / 6.3 (-0.7) | 2.3 / 6.0 (+0.1) | 5.0 / 14.5 (-2.7) | 6.4 / 14.4 (-4.2) |
| 30-50 m | 1196 | 2.4 / 6.4 (+0.6) | 5.3 / 10.1 (-5.3) | 2.2 / 6.4 (-0.6) | 2.2 / 7.1 (+0.3) | 4.5 / 15.5 (-2.8) | 5.2 / 12.9 (-3.6) |
| over 50 m | 1010 | 2.6 / 7.3 (+0.5) | 2.9 / 8.1 (-1.8) | 2.8 / 7.3 (-1.2) | 2.5 / 7.5 (+1.4) | 6.9 / 19.0 (-5.3) | 5.8 / 15.3 (-4.5) |
| all | 5421 | 2.7 / 7.4 (-0.8) | 8.0 / 16.4 (-7.9) | 2.6 / 7.0 (-0.3) | 2.8 / 8.2 (+1.1) | 5.4 / 16.5 (-1.8) | 6.0 / 14.7 (-2.9) |

* On a whole box the near-face row `gn` is as good as the detector's bottom edge and not better (their errors
  correlate 0.72); nearer than 15 m it is better.
* The **centre** (the tracker's state) is not readable off the box bottom (8.0 %, biased -7.9 %);
  `gc` gives it to 2.7 %. The centre's bearing from `gc`: 1.51 px median against the box centre's
  2.97 px.
* The height-prior baseline (box height x class height) is at 5-6 % median with a 90 % point of 14-19 %; the
  head's own height does not beat the prior for cars.
* **A hidden bottom**: with the lower 25-55 % of the box cut away `gn` is found to 0.06-0.08 of the true box
  height in the median, unbiased, stated sigma on the safe side: about 9 % of the range for a car from a
  1.2 m camera, where such a box was a bearing only.
* With a flat road assumed instead of the object's own ground height the same rows give 9.5 % (`gn`) and
  9.4 % (box bottom): pitch and road profile, not the ground point, limit the range. That is the
  tracker's `dp` and `grade_k`.

Stated 1-sigma of the points: `gn` 70.2 % within one, 93.3 % within two on visible vehicles;
68.4 / 89.8 % on occluded ones.

Wheel contact points proper: not delivered (KITTI has no wheel labels).

## Checks on rides (other cameras, no labels)
Seven rides of the ride library (other cameras than KITTI's: dash cameras behind a windscreen, 1280 x 720, focal
lengths 690-1100 px, 1.05-1.7 m up; three by day: overcast city, rain, low sun; four at night: a lit tunnel and
expressway, a motorway, a motorway in fog, a snowy lit city), 5 minutes each, the whole frame through the
detector's 384x640 export at 5 passes a second, a pose on every vehicle box (`tools/pose/pose_check.py` of
chris-signs). None of these rides took part in training. **There are no labels**: each check holds the head against
another cue that has errors of its own.

| ride | vehicle boxes | red lamp pair: heading in the rear half (where the head is sure: `p` >= 0.8, `flip` < 0.1) | white lamp pair: heading in the front half (sure) | vehicle ahead (keeps its size 3 s in mid-picture): view rear | successive boxes 0.2 s apart: flipped | `gn` row - box bottom, box heights: median, spread |
|---|---|---|---|---|---|---|
| warsaw-city (day, overcast) | 6588 | 8: 100 % | 70: 24 % (19 %) *lamp rule wrong, below* | 13 chains, 983 poses: 97 %, median abs yaw 2.1 deg | 4.7 % | -0.010, 0.038 |
| katowice-rain (day, rain) | 4661 | 340: 91 % (97 %) | 567: 46 % (46 %) *lamp rule wrong, below* | 6 chains, 804: 92 %, 2.6 deg | 4.2 % | -0.023, 0.038 |
| s7-elblag-lowsun (day, low sun) | 1545 | none | 10: 0 % *lamp rule* | 4 chains, 460: 97 %, 2.3 deg | 1.4 % | -0.036, 0.044 |
| krakow-night-tunnel | 546 | 220: 97 % (99 %) | 138: 96 % (98 %) | none | 4.2 % | +0.012, 0.044 |
| a1-night-fog | 392 | 181: 93 % (100 %) | 8: 62 % | 2 chains, 103: 100 %, 0.8 deg | 7.6 % | +0.015, 0.051 |
| a1-night | 709 | 435: 95 % (100 %) | 19: 42 % | 3 chains, 187: 97 %, 2.9 deg | 7.3 % | +0.041, 0.054 |
| gdansk-night-snow (lit, snowy city) | 3296 | 434: 62 % (76 %) | 242: 45 % (34 %) | 2 chains, 247: 53 %, 11.5 deg | 11.2 % | -0.002, 0.039 |

What these say, and what they do not:

* **Heading by day**: the vehicle ahead is seen from behind in 92-99 % of its boxes (median |yaw| 2-3 degrees) on
  all three day rides: the common case works on cameras the head never saw. Crossing vehicles (chains that move
  sideways against the other boxes at a constant size: 11 chains, 115 poses on warsaw-city; 4 chains, 53 poses on
  gdansk-night-snow) point the way they move in 84 % and 94 % of the poses, but are called side-on (45-135 degrees)
  in only 42 % and 23 %: many are turning (three-quarter), and side views are also the head's weakest class on
  KITTI. Eleven chains prove little.
* **The lamp colour is a reference only in real darkness.** By day the lamp rule finds "white pairs" in dull
  frames (overcast, rain) on vehicles seen from behind; looked at by eye (sheets of 24 and 48 such boxes of
  warsaw-city and katowice-rain: all rear views, tail lamps visible) the head is right and the rule's colour wrong.
  A consumer must not take `lamps.c` white by day as "front"; with a pose it can tell.
* **Night, expressway and motorway**: red lamp pairs go with a rear-half heading in 93-97 % (99-100 % where the
  head is sure), white pairs with a front-half heading in 96 % on the one ride with enough of them (138); the two
  motorway rides have 8 and 19 white pairs (oncoming traffic across the median, small and glaring) and there the
  head is right in about half: not usable.
* **Night, snowy lit city: weak.** Against the lamp colour 62 % / 45 %, flips between successive boxes 11 %, the
  vehicle ahead called rear in 53 % of its boxes; and the head is surer than it should be (76 % right where it
  states `p` >= 0.8). By eye on a sheet of 48 lamp-pair boxes of that ride about 30 headings are clearly right
  and 5 clearly wrong, the rest cannot be judged, and several disagreements are the lamp rule's (parked vans
  facing the camera called red): both cues fail there.
* **What the lamp augmentation bought**: the same head trained without the drawn lamps called glowing tail lamps
  "front" with high confidence at night: on krakow-night-tunnel red pairs 71 % in the rear half, white 82 %,
  flips between successive boxes 14.2 %; with them 97 %, 96 % and 4.2 %. (On KITTI's day test the two heads are
  alike: 8.2 % and 8.8 % of the headings off by more than 30 degrees.) The check uses the lamp colour, the
  augmentation teaches lamp colour: it shows that the head now reads the lamps, not that it is right where the
  lamps mislead.
* **Ground contact**: on boxes with no cut and no covered edge the near-face row `gn` lies within 0.01-0.04 box
  heights of the detector's bottom edge in the median (spread 0.04-0.05), with a ride-dependent offset: in low sun
  `gn` is 3.6 % of the box height above the box bottom, on the dark motorway 4.1 % below. One of the two is biased
  there (a shadow thrown towards the camera, a bottom lost in the dark); without labels it cannot be said which.
  The centre `gc` is above `gn` in every box and inside the box in 84-99 %. That `gn` does not contradict the box
  where the box is good proves no accuracy: the gain claimed for `gn` is on boxes whose bottom is cut or hidden,
  and that is measured on KITTI only.
* **Over time**: the heading changes by 0.8-2.2 degrees in the median between boxes 0.2 s apart (KITTI, with
  labels: 1.3 degrees for a true change of 0.6), 0.13-0.38 of the joint stated sigma (KITTI: 0.45): the answers are
  smooth and, as on KITTI, far from independent. `gn` relative to the box moves by 0.010-0.022 box heights.
* The camera numbers of these rides (focal length, horizon) were used for range comparisons only, never given to
  the head.

## What was tried and what it showed
* **Night**: KITTI has no night. A first head (same recipe, no lamps) took glowing tail lamps for headlamps with high
  confidence. Drawing the lamps into night-like training crops where the 3D box puts them repaired it (table
  above) at no cost by day.
* **Camera numbers as an input**: no real gain on KITTI (heading 9.2 % against 8.8 % off by more than 30 degrees,
  `gn` row 0.042 against 0.044 box heights); the option exists (`--lights-ru-pose-cam`) and stays unset.
* **Size**: three sizes side by side (9.0 / 16.0 / 33.3 M multiply-accumulates): 9.5 / 8.8 / 7.5 % of the headings
  off by more than 30 degrees, the ground points alike. The middle one is shipped.
* **Dropout with class-balanced sampling**: worse (11.7 %); dropped.
* **One backbone with the appearance vector**: not built (different crops, different training data, and the
  joint weights would derive from both); with both options on a box costs two runs.

## Through the detector's own code path
`lights_replay --ru-pose` over KITTI's held-out pictures against chris-pose's evaluation path on the same 5792
boxes: heading differs by 0.17 degrees in the median, the `gn` row by 0.09 px; against the labels the two paths are
equal (heading median 2.47 / 2.48 degrees, 8.7 / 8.8 % over 30 degrees). With the option off the `lights` messages
are byte for byte those of chris-signs 5948fea (a day and a night ride at the unit's schedule, 1500 passes each,
detection inputs off and all on). `ctest`: 6 of 6, without any model file.

## Cost
Laptop, one thread, loaded by other jobs (a quiet laptop is up to twice as fast); phone: estimates from the
project's factors, nothing measured there.

| | laptop (measured) | phone (estimated) |
|---|---|---|
| per box (crop, run, decode) | 1.36-1.43 ms; 1.56 x the appearance head under the same load | 4-5 ms on a big core, 10-15 ms on a little one |
| per pass at the unit's schedule, cap 4 | passes with a pose: median 4.0 ms, 90 % point 6-11 ms; 2.0-2.1 ms averaged over all passes | at the cap 16-20 ms on a big core; about 7 ms a pass on average (3-4 % of a big core at 5 passes a second) |
| message | about 300 bytes per pose (+21 % on a busy day ride) | |

At most 4 runs a pass (`--lights-ru-pose-max`); with more vehicle boxes the passes take turns through them.

## The option and the fields
`signd --lights-ru-pose FILE.onnx` (with `--lights-road-users`; `--lights-ru-pose-max N`, `--lights-ru-pose-all`,
`--lights-ru-pose-cam F,CU,CV`; signd.json `lights_ru_pose...`; `lights_replay --ru-pose...`). Off by default; a
missing file leaves it off. Per road user of a vehicle class `pose` {`yaw`, `yaw_std`, `p`, `flip`, `vp`,
`vp_conf`, `gc`, `gn`, `gl`, `gr`, `dim`, `dim_std`, `occ`, `trunc`, `kind`}; per message `pose_kind` "pose1".
Persons and two-wheelers get none.

## For the tracker (design, not built)
[detin-consumption-design.md](detin-consumption-design.md) section 9. In short:
* `gn` is the tracker's own near-face point: its row goes into the existing ground-angle row. On a whole box it
  replaces nothing (one row, the better stated sigma); **on a box with a cut or covered bottom it is the ground row
  that box did not have**, combined with the one-sided bound (the true contact is at or below the visible bottom)
  as a truncated Gaussian. At the unit's schedule 97 % of a day ride's vehicle boxes are cut at the bottom by the
  view.
* `gc`'s column is the bearing of the centre (twice as good as the box centre, and right where a side is hidden);
  its row is not a second range.
* The heading enters as two things, like the lamp colour: the end by votes (`ln(p / flip)`, flips are transient),
  the axis as a row with `max(yaw_std, 3 deg)` and the errors' one-second correlation in its variance. It gives
  the birth heading and velocity direction, the side of the line of sight the box width cannot tell, and the
  angle the width and lamp rulers need.
* In real darkness the lamp colour outvotes the pose; by day a `white` lamp pair does not.
* Dimensions: not a ruler (no better than the prior for cars); a van vote at most.

## Not done, not verified
* **Nothing is consumed**: the tracker's half is design; no effect on its range errors or warnings is measured.
* **Nothing ran on the phone or the car**; all costs are loaded-laptop figures and estimates. The car's own
  camera has not been seen by the head. signd the daemon was built with the option and not run live (everything
  through `lights_replay`).
* **Accuracy is known on KITTI only.** The rides have no labels; their checks are consistency with other cues.
  Night accuracy is not measured, only agreement with lamp colours; in a snowy lit city the head is weak and too
  sure. No ride with a known camera calibration and ground truth exists to check `gn` as a range off KITTI.
* **Wheel contact points** proper: not delivered (no wheel labels in the data used).
* **Side views** (crossing traffic) are the weakest class; lorries and buses rest on few examples; the head's
  dimensions are not useful.
* **A second data set** was not added: every other 3D-box set considered needs a registration or forbids
  derivatives, except Argoverse 2 (not attempted).
* **A shared backbone** with the appearance vector: not built.
* **The cap's turn-taking** is checked on a ride, not by a unit test; the list's order changes between passes, so
  a box can wait longer than one round.
* **Disk**: the lane was meant to stay under about 30 GB and peaked at about 37 GB for a few minutes (both zips
  and the Python environment at once); it now holds 15 GB.

## Reproduce
chris-pose `README.md` (data preparation to export), chris-signs `docs/POSE_HEAD.md` (replays and checks). Scratch
on the laptop, in no git: `<datasets>/viewhead-lane/` (KITTI as JPEG, records, trained files in `out/`, ride
outputs, the Python environment with a CUDA build of PyTorch).

