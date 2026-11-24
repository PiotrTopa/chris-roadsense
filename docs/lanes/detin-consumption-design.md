# Detection inputs: how the tracker should take them (design, not built)

Lane `detin`, 2026-11-24. The detector side is built (roadsense-signs branch `detin`, `docs/DETECTION_INPUTS.md`;
the contract: [ARCHITECTURE.md](../ARCHITECTURE.md), light provider). This file is the consumption design for
chris-integrator's tracker (`chris_integrator/radar`, read at 63cdb71): which field enters where, with which
measurement model and which numbers, and what the simulator has to emit so that each can be exercised. Nothing in
the integrator was changed by this lane. Every number is marked: **M** measured in this lane (source named),
**A** assumed (a reasoned default to be tuned on the simulator and the car).

Names used: `from_lights` (core/roadusers.py: `lights` -> `road_users` stream), `on_road_users` (radar/box.py:
stream -> detections `{kind, z, R, attrs, w, wok}`), `Hyp.scan` (the hypotheses' scan), `r_visible`, `r_p_detect`,
`r_extent_update`, `lh` / `dp` (the native core's functions and states, docs/RADAR.md).

## 0. Passing the fields through

`from_lights` copies per object, when present: `trunc`, `occ`, `occ_by`, `occ_edges`, `sym`, `lamps`, `emb`,
`scale_src`; and the message's `emb_kind`. `road_users_low` becomes a second list `objects_low` of the same
`road_users` message (same projection, same keys, `"low": true` on each). Absent keys stay absent: with every
detector option off the stream is what it is today, and the tracker's behaviour must then be bit-identical
(regression test: a recorded session through both code paths).

The hidden-edge set of a box is used throughout:

    E = clip  ∪  trunc  ∪  occ_edges          (letters l t r b)

## 1. Flags: which rows a detection has, and what a missing box means

Today the rows follow `clip` only. With the flags they follow `E`:

| E contains | today (clip only) | with flags |
|---|---|---|
| `b` | kind `b` / `bt`: no ground angle, no box height | the same, now also when another box hides the bottom (`occ_edges` b): a ground angle read off an occluder's roof line is the error behind "the car, 6-8 m further" |
| `t` | kind `rb`: no box height | the same, also for a top hidden by a nearer box |
| `l` or `r` | `wok` false: the width is not the object's | the same from `occ_edges`; and the bearing (below) |
| none, `occ` > 0 | - | all rows; see "partly covered" |

* **Bearing of a box with a hidden side.** The box centre is the centre of the visible part. Keep the bearing
  row, with the variance `sb^2 + (0.5 * w_hidden)^2`, where `w_hidden = max(0, w_expected - w_box)` as an angle and
  `w_expected` the silhouette the object's extent state predicts at its range (new objects: the kind's width). (A)
* **Extent** (`r_extent_update`): only from boxes with `l`, `r` ∉ E and `occ` < 0.1 (A). This is the rule that
  stops a pedestrian's box merged with a neighbour, or a car half behind another, from teaching a length.
* **Box height / `lh`**: only with `t`, `b` ∉ E (as today for `clip`).
* **Partly covered, no edge hidden** (`occ` > 0, `occ_edges` empty: e.g. a person in front of a bus): rows as
  usual; the size rows' variance times `(1 + occ)^2` (A).
* **`occ_by` is an index into this message's `road_users`**, not a track id. After association the tracker knows
  which object took the occluder's box; that pairing ("object X was covered by object Y in this scan") is what
  `occ_by` adds to the message's `occ_by` output and to the group logic (a queue member hidden by the queue's own
  vehicles, RADAR.md section 7).
* **A missing box.** `r_visible` knows only settled objects as occluders (RADAR.md section 3, "not modelled").
  The scan's own boxes are occluders too, tracked or not: for a leaf that got no detection, compute the cover of
  its predicted box by the scan's boxes (both lists) that are nearer by the detector's rule (bottom edge lower;
  a cut bottom is lower than it shows) and use

      vis = min(vis_objects, 1 - cover_boxes)          pd = pd_sensor * ramp(vis)      (ramp: occl_from .. occl_full, as now)

  So a parked car behind a van that is boxed but not yet a settled object keeps its existence. The detector's
  rule and the tracker's must be the same code path in spirit: port `ru_nearer` / `flag_one` (roadsense-signs
  `signd/detin.cc`, 60 lines).
* **`trunc` against `clip`.** `clip` (view edge) says the other view may show the rest; `trunc` (frame edge)
  says no view will. For the "cut by the edge of the field" part of `pd` nothing changes; for extent, a
  `trunc` side means the object's length is at least what is seen (a one-sided bound; today such boxes are simply
  not used, which stays).

Numbers. **M** (BDD100K val, 3000 images, roadsense-signs `tools/road_users/detin_bdd.py`): of the detector's
boxes that match a label, `trunc` is set on 94.5 % of those labelled truncated and 97.9 % of the set ones are
labelled so; `occ` > 0 is set on 72.5 % of those labelled occluded (the rest are hidden by things that are no
boxes) and 93.0 % of the set ones are labelled so. On the label boxes themselves `occ` >= 0.3 is labelled
occluded in 97 %, `occ` in (0, 0.1) in 80 %, no flag in 40 %: an unflagged box is NOT known to be whole, so the
flags remove trust and never add any.

## 2. The low-score list: a second kind of detection, not a second pass

ByteTrack associates greedily in two passes. The tracker here weighs hypotheses, so the low list enters the
same scan as detections of a second kind with their own probabilities; no pass order is needed.

For a leaf with existence r, the detector now has three outcomes per visible object:

    in the first list   pd_hi           weight  r * pd_hi * l(z) * F / lam_hi(z)
    in the low list     pd_lo           weight  r * pd_lo * l_lo(z) * F / lam_lo(z)
    not reported        1 - pd_hi - pd_lo        weight  1 - r * (pd_hi + pd_lo)

and a low detection that no leaf takes has weight 1 and **starts nothing** (no birth term: `new_exist` = 0 for
the low list; the unexplained-detection map is not fed by it).

| quantity | value | basis |
|---|---|---|
| `pd_lo` | 0.4 x (1 - `pd_hi`) at floor 0.10 for an object taken at random; 0.75 x (1 - `pd_hi`) inside a miss run of an object the first list has before and after | **M** BDD100K val: the first list finds 46.6 % of the labelled road users, first + low 68.7 %: 41 % of what the first list misses is in the low list (floor 0.15: 35 %, 0.20: 29 %, 0.30: 15 %). Rides, the unit's views (`tools/road_users/low_check.py`): of the passes in which the first list misses an object it has before and after, a low box lies on the object in 75-100 % at floor 0.10 (55-86 % at 0.20); table in the lane report. Start with 0.5 x (1 - `pd_hi`) |
| `lam_lo` | (low boxes on no known object per pass) / the view's measurement volume | **M** rides at night, floor 0.10: 0.35-0.9 low boxes a pass on no first-list object (floor 0.20: 0.1-0.25), against 0.03-0.3 first-list boxes a pass in the same rides; day: lane report. BDD (whole frame, busy scenes): 3.7 low boxes per image on no label at floor 0.10 against 0.22 first-list boxes on none (17x) |
| edge noise of a low box | bottom / top / side sigma: 1.5 x the first list's px term, relative term 0.10 instead of 0.04 | **M** BDD: edge error 10-13 % of the box height against 3.8-4.0 % (robust sigma; in px 2.3-4.7 against 1.9-3.7 at like sizes), median IoU with the label 0.67 against 0.87 |
| class of a low box | vote weight 0.3 (A) | **M** class right 95 % against 98 %, on boxes that do match |
| existence evidence | the hit cancels the miss term and adds at most ln 1.5 (the present lower bound); no score term | A: a low box must not confirm anything |

Rules that are not numbers:
* a low detection never counts as a `hit` for confirmation, never starts an object, never feeds the camera's
  smallest-box memory (`h_min`) and never teaches the pitch bias (`h_common`);
* it may update the state (bearing, the rows E allows, with the noise above), carry a scale rate and an
  appearance vector like any detection;
* an object whose last N sightings are all low (N = 4, A) is reported `coast`-like to consumers: seen, weakly.
* the governor's `road_users_hz` 0 sheds both lists (`road_users_low` absent while `road_users` is null).

What it is for: the detector's misses come in runs (RADAR.md: `p_d` 0.55 at night, "the detector misses in
runs"); during a run the object is usually still in the output below the threshold. Taking those boxes shortens
the stretches in which an object lives on its motion model alone, which is where the row hops and the phantom
velocities start (RADAR.md section 5, "then not boxed for 0.8 s, and the next box is of the car two places further").

## 3. Appearance vector: a likelihood ratio beside the geometry

Per leaf: a mean vector `mu` (as int8 / scale: 32 for `yuv3x3`; for `reid32+yuv3x3` the first 32 values / 127
and the other 27 / 32 x 0.5), the time of its last update, the `emb_kind` it belongs to. Per (leaf, detection)
pair with both vectors of one kind, the distance `d = |emb - mu|` (Euclidean, in those units) multiplies the
pair's likelihood by

    F_app(d, gap) = clamp( p_same(d | gap) / p_other(d),  1 / B,  B )

where `gap` is the time since `mu` was last updated. It sits where the box feature F sits today. `mu` is
updated (`mu += 0.3 (emb - mu)`, A) only from first-list boxes with `occ` < 0.2 and no hidden edge; a leaf with no
`mu`, a detection without `emb`, or different kinds: F_app = 1.

`p_same`, `p_other`: log-normal fits to the measured distance distributions (`docs/lanes/detin-emb-distances.json`:
quantiles and fits of same-object distances 1, 2, 4 s apart and of other-object distances in the same frame, a
day ride and a night ride, both kinds). **M**, with one caveat: those tables come from full-frame crops at 10
frames a second; the vector as the unit publishes it (boxes of the light views, cut at the view's bottom, 5
passes a second) shows larger distances on both sides (lane report). Before the ratio is tuned the tables are
to be re-derived from the unit's own messages (roadsense-signs `tools/road_users/emb_check.py` on recorded
`lights` messages; the recorder has to keep them at the full rate for that). The bound B = 8 by day, 3 in a
dark frame (A): one detection must not outvote the geometry, several in the N-scan window may (two agreeing
scans: 64). With the day tables and B = 8: 79 % of other-object pairs get a ratio of 1/4 or less at a gap of
1 s (`reid32+yuv3x3`: 86 %), 68 % (77 %) at 4 s, against 3-5 % of the same-object pairs; at night 23-34 %
against 5-6 %.

What it buys, by case (the tracker's open items in docs/RADAR.md):
* **an object that comes back after an occlusion**: its gate has grown with its covariance; among several boxes
  in the gate the one that looks like it gets up to B times the weight. This is the id kept through
  `occlusion`, `crossing_pair`, a queue that moves on;
* **one track fed by neighbouring cars of a row** (section 5, open): "the same object, driving with us" beats
  "missed, and this is the next car" by 0.992 to 0.006 on geometry. The next car is another car: by day its
  vector is far from `mu` in most cases (about four in five other-object pairs get the ratio 1/4 or less, above),
  and that factor is applied in each of the N scans the story stays open;
* **a car and a van swapping boxes in a platoon** (section 5): the same, plus the class;
* at night the vector is weak (dark bodies, lamps): the bound 3 says so. The lamp pair (section 4) is the
  night-time cue.

`reid32+yuv3x3` (the learnt vector with the 27 numbers appended) is consumed identically with its own distance
table; the two kinds are never mixed.

## 4. View cues

### `lamps`: which end faces the camera, and a ruler
* `c` = `red`: the rear faces the camera. `white`: the front. A measurement of the extent's heading: along the
  line of sight, pointing away (red) or towards the camera (white), sigma 40 deg (A: a three-quarter view still
  shows both lamps), applied like the road-axis prior (to its sigma and no better, however often). It settles
  "which end is the front" before the object's motion has shown it, and it resolves the mirror ambiguity of the
  box width for vehicles seen at an angle.
* **Birth.** A new object's moving hypotheses start with 12-15 m/s along the car's heading. With `white` lamps
  the prior of the moving hypotheses is that speed *against* the car's heading (an oncoming vehicle), and the
  standing hypothesis keeps its share (a car parked facing the traffic). With `red`: as now. (A; the closing
  speed of an oncoming car is twice what the present prior expects, and the first scale rates are spent on
  unlearning it.)
* **The separation as a size row.** `ln sep = ln f + ls - ln depth + ln|cos a|` (a: the heading against the
  line of sight, from the extent), with `ls` the log of the object's lamp separation: an object constant like
  `lh`, with a prior of its own (ln 1.2 m, sigma 0.2: A, to be calibrated on the car from parked cars whose
  range the car's own passing pins) and the same treatment (learnt by the standing hypothesis, held as a
  consider parameter by the moving ones: RADAR.md section 2). Row noise: `sep_std / sep` (**M**: the detector
  states hypot(0.35 px, 2 % of the separation), from the scatter of ln(separation) about its local trend on a
  night ride: 0.31 px at separations of 12-20 px, about 2 % above 40 px; below 10 px the pair picked flips
  between lamps and reflections (0.8 px): ignore `lamps` with `sep` < 10 px).
  What it buys at night: the box height of a dark body is the noisiest row the camera has (the simulator's night
  figures: 4.5 px + 6 % on the bottom edge alone: 0.3 in the logarithm for a 20 px box); the lamp row has
  0.02-0.04. Even with a wide prior on `ls` the *change* of range is seen an order of magnitude better per
  scan, independent of the scale rate's own chain. Cost: one more per-object state (or, without a native
  change: use it only as a second scale-rate source, which the detector already does).
* A pair is found on 61-73 % of the night vehicle boxes of three rides (**M**), on 4 of 55 on a snowy, lit street
  (the frame is not dark there) and on 2 of 629 by day (the frame-darkness gate): absent `lamps` says nothing.

### `sym`
Not validated against labels in this lane (see the lane report: on the few clean boxes of the ride views,
side views scored as high as rear views, because a car's side is itself nearly mirror-symmetric at 32 x 16).
Recommended use until a labelled check exists: none on its own; with the box aspect it separates three-quarter
views (aspect 1.4-2.4 and `sym` < 0.4, A) from rear / front views (aspect < 1.6, `sym` > 0.7) for the birth
heading. A wrong reading costs only the birth prior.

### What is not delivered: heading, wheel contact points
The present detector gives a box; no pixel rule of this lane tells a rear view from a three-quarter view by
day, or finds where the tyres meet the road. That needs a trained head (lane report, "orientation and ground
contact"). Until then the ground contact a consumer has is the box's bottom edge, valid when `b` ∉ E, and the
range from it is geometry on the consumer's side (next section).

## 5. Range from the ground plane: the consumer's geometry

The contact row is `v = box[3]` at `u = (box[0] + box[2]) / 2`, valid when `b` ∉ E. The ground angle the
tracker already uses is `atan(h_cam / range)` with the camera's pitch as a learnt bias. Two additions:

* **The road's vertical profile.** The angle of a ground point at distance x is off by `dz(x) / x`, where dz is
  the road's height there above the plane under the car. Where the road model's plan carries a height profile
  (`road.plan.z`), predict the row with it. Where it does not, the row's variance gets a term for the unknown
  vertical curvature k_v (dz = k_v x^2 / 2):

      sigma_angle,grade(x) = sigma_k * x / 2,        sigma_k = 1 / 3000 m      (A: crest and sag radii of built roads are 2-10 km)

  i.e. 0.19 deg at 20 m, 0.57 deg at 60 m: at 60 m as large as the pitch term, and it grows where the pitch term
  does not. This is "a far ground angle that knows its error" (RADAR.md section 5, the platoon swap: the van's
  bottom edge at 67 m read 33.5 m).
* **Width as a ruler for a view known to be axial.** For a vehicle whose view is known to be rear or front
  (`lamps` present, or the extent's heading within 20 deg of the line of sight), the box width with `l`, `r` ∉ E
  measures `W / range` with the kind's width prior (a car 1.8 m, 10 %: half the height prior's 20 %). Today
  the width teaches only the extent. As a range row it is the better ruler for exactly the objects ahead
  in the lane. (A; the detector's side edges are as good as its top and bottom: **M** 3.8-4.0 % of the box height.)

## 6. Scale rate: sources and overlapping windows

* `scale_src` absent or `"both"`: as today. `"lamps"`: the value comes from a line through 3-6 lamp separations;
  its stated std already carries an empirical factor of 1.5 (**M**: successive lamp-only values of far objects on
  two night rides, 45 pairs: 1.05 and 0.90 of the stated std with the factor, none beyond 3 sigma; without it
  1.35-1.6 and 5-12 % beyond 3), and in fog the lamp value sat 0.023 /s above the patch value (27 boxes). Take
  lamp-only values with a Huber likelihood (k = 2) or drop innovations beyond 3.5 sigma (A).
* **Successive values share data** (also true of the present values). A value is the mean rate over
  `scale_dt_s` (0.8-1.0 s) and a new one arrives every 0.2-0.4 s: consecutive values of one object share most of
  their baseline. Taken as independent rows they are counted two to four times. Use

      R_eff = scale_rate_std^2 * max(1, scale_dt_s / dt_since_this_object's_previous_scale_row)

  (A, the standard treatment of overlapping averages; the simulator must produce the correlation, below.)
* The stated std's floor `scale_rate_floor` 0.01 stays.

## 7. What the simulator must emit (`sim/roaduser_sim.py`, `DetectorSim.generate`)

| input | what to add | numbers |
|---|---|---|
| flags | a partly covered actor's box is the bounding box of its VISIBLE part (today: the full truth box unless 0.6 is covered, then nothing); `trunc`, `occ`, `occ_by`, `occ_edges` computed from the emitted boxes by the detector's rule; occluders that are not actors (a post row, a wall) for the "no flag is not whole" case | the detector's rule: signd/detin.cc; unflagged-but-hidden share: 40 % of the labelled-occluded (**M**) |
| low list | per visible actor not in the first list (miss run, size ramp): with `p_low` in `road_users_low`, score uniform in (floor, class threshold), edges noisier; low clutter per message; a few low boxes beside a first-list box | `p_low` 0.4 (**M** BDD, floor 0.10); edge noise 1.5 x px term, 0.10 relative (**M**); clutter and beside-a-box rates per pass: lane report's ride table (**M**); class wrong 5 % (**M**) |
| appearance | a latent vector per actor (drawn so that other-object distances match), per observation the latent plus noise that grows with the time since the last observation (so that same-object distances match at 1, 2, 4 s); at night both distributions as measured there; `emb_kind` | `docs/lanes/detin-emb-distances.json` (**M**) |
| lamps | at night, for a vehicle whose rear or front is within 40 deg of the line of sight: `lamps` with `c` by the facing end, `sep = f S |cos a| / depth`, S per actor (1.2 m, 20 %: A), noise hypot(0.35 px, 2 %), found with probability 0.6-0.7, one pair in ten on the wrong blobs (sep x 0.6-0.9) | **M** except S |
| scale rate | per actor and pass a noisy log size (the pass's own error), the reported rate the slope over the window of those samples: successive values then share errors as the real ones do; `scale_src` | per-pass sigma from the stated std x sqrt(window / step) |
| ground | the road's vertical curvature in the scenarios (crests, sags) with a plan `z` that the consumer may or may not be given | k_v up to 1 / 2000 m (A) |

Scenarios that exercise them (existing names): `occlusion`, `crossing_pair` (appearance, flags), `parked_street`,
`curve_parked` (appearance against row hops, low list in miss runs), `platoon` (appearance + class against the
swap, grade term), `oncoming` at night (`white` lamps: birth prior), `queue` (occ_by within the group),
`ped_wait_cross` (extent from flagged boxes only).

## 8. Order of implementation on the consumer's side

1. pass-through and the bit-identity test (section 0);
2. flags -> rows and `pd` (section 1): no new state, removes wrong information;
3. low list (section 2): needs `pd_lo`, `lam_lo` per sensor; the simulator's low list first;
4. appearance LR (section 3): per-leaf vector, tables;
5. lamps: colour -> heading / birth prior, then the size row (section 4);
6. grade term and width ruler (section 5), scale-row correlation (section 6).
