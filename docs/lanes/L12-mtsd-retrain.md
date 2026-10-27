# L12 MTSD retrain: sign classifier with a "not a sign" class, trained on the Mapillary Traffic Sign Dataset

Lane opened 2026-10-27 19:44 CET (Warsaw local, UTC+1; Opus; data + training + evaluation + deploy if it pays).
Goal (coordinator brief): retrain the sign stack on MTSD so that it (1) has an explicit "not a sign" class trained on
hard negatives (stop/tail lamps, traffic lights, sign-like objects), (2) confuses fewer real signs (warning triangles
read as D-1, C-5 discs as D-3, blue keep-right as no-entry), (3) recalls small/far signs better (B-33 70 missed in 2
of 3 passes, far D-7 never tracked). Deploy only if it beats the current models on pre-registered criteria.

Laptop scratch `~/l12/` (venv, crops, models, evaluation streams; nothing of it committed). Scripts:
roadsense-signs `tools/mtsd/` (data, training, classifier scoring), this repo `docs/lanes/L12-mtsd-retrain/`
(ride evaluation).

## Journal

### 0. Data (19:44-20:20)
* Signed links (13, valid until 2026-11-26) in `~/datasets/mtsd/urls.txt` (mode 600, never printed). HEAD of every
  link, then the two `.txt` files read (md5 lists): 6 files `mtsd_v2_fully_annotated_*` (annotation 121 MB,
  images test 8.96 GB, train.0/1/2 10.38/10.36/10.39 GB, val 4.57 GB) and 5 `mtsd_v2_partially_annotated_*`
  (annotation 72 MB, train.0-3 11.8-11.9 GB each, ~47.5 GB).
* Free disk 79 GB, floor 30 GB: downloaded the fully annotated annotation, val and train.0-2 only (35.9 GB,
  19:45-20:03, resumable `tools/mtsd/fetch.sh`, md5 of every file matches the list). **Skipped**: the test images
  (the annotation zip has 41 910 json = train 36 589 + val 5 320 only: test labels are withheld, so the test
  images are useless here) and the whole partially annotated set (47.5 GB, does not fit; its signs are the same
  physical signs seen in other images).
* Licence (`LICENSE.txt` in the annotation zip): "provided under the Creative Commons Attribution NonCommercial Share
  Alike (CC BY-NC-SA) license" (the URL it gives points to the by-nc 4.0 legalcode). Consequence: crops and every
  model trained here are never committed or redistributed; the scripts are (AGPL repo).
* Annotations: 206 386 objects, 401 labels = the 400 classes of the current classifier (identical names) +
  `other-sign` (135 958 objects: real signs outside the 400 classes). Flags: ambiguous 68 560, occluded 18 897,
  dummy 6 855; 941 panoramas (skipped).

### 1. Setup
* venv `~/l12/.venv`: torch 2.14.1+cu126 (the cu126 wheels still carry sm_60 code for the laptop's Quadro P3200,
  Pascal sm_61, 6 GB), ultralytics 8.4.183 (official pip), onnx, onnxruntime 1.31. No downloaded checkpoint is
  loaded anywhere: the classifier is trained from scratch (YOLOv8n-cls architecture from Ultralytics' bundled yaml).
* `signd_replay` built on the laptop (`~/l12/build-x86`, Ubuntu `libonnxruntime-dev` 1.23 + `libjpeg-dev` from apt)
  so the real signd pipeline (detector, classifier, tracker, verifier, flags) runs offline on ride frames.
* Validation of the offline path: krakow-2024 frames the phone's signd processed in L11's recording (every 4th
  frame), re-run on the laptop with the installed models and flags, `signs` messages swapped into the recording and
  annotated with the api's signctx (`ride_signs.py`), then L11's `rules_replay.py` + `score.py`: same false rules
  and misses as the phone's own stream except one one_way miss (detected offline) and one priority_road interval
  (agreement 0.851 live vs 0.673 offline: the offline D-1 track was ended by a turn later). Offline = live up to
  ORT fp32 vs ncnn fp16 differences; variants are compared offline against offline only.

### 2. Crops (`tools/mtsd/extract_crops.py`, `extract_bdd.py`)
Streamed from the zips (nothing unpacked). Per non-panorama image: every annotated object (`gt`); the current
detector's boxes (3 views: whole image at 640x384, and two 1280x720 windows of the image scaled to 1280 px, i.e. the
phone's scale) matched to an annotation (`detpos`, the detector's own box noise) or far from all annotations
(`detneg`: the detector's false positives, label `background`); random sign-sized boxes away from signs (`rand`,
`background`). Each crop is a 96x96 square, box long side 64 px. BDD100K val (L8's split by md5(id)): L8's lamp and
red-light blobs, annotated traffic lights of any colour, and the detector's false positives at the phone view, all
`background`; the train split for training, the test split held out.

## Pre-registration (committed before any new model is scored on rides or on held-out sets)

Variants: **base** = installed models (detector `mtsd_signdet_yolov8n_384x640`, classifier `mtsd_signcls_yolov8n_64`,
L8 verifier, `--class-min-score regulatory--no-entry=0.71`); **cls1** = the new classifier (402 classes: the 400 in
the same order, then `other-sign`, `background`) with the same detector, verifier and flags; signd flags a track whose
class vote is `background` as `suppressed`, reason `not_a_sign`. The classifier's training run is chosen on MTSD val
only (the checkpoint at the end of the schedule; no ride is looked at with a new model before the ride scoring).
A detector variant, if one is trained, is scored the same way as an extra variant **det2** (+ its own phone cost).

E1 classifier on held-out crops (`tools/mtsd/eval_cls.py`, MTSD val + BDD test split; conditions full / phone size /
<= 24 px / <= 16 px): top-1 on val signs of the 400 classes (merged names), macro recall, `other-sign` claims,
not-a-sign claims (p >= 0.6, the signd vote gate) for MTSD detector false positives, random boxes, BDD lamps, red
lights, lights, BDD detector false positives; the three confusion families (warning -> priority-road, go-straight ->
one-way, keep-right -> no-entry).

E2 L8 Krakow lamp check (`krakow_l8.py`): krakow-2024, offline, all 4 phases of "every 4th frame" (phase 0 = the
phone's recorded frames), labels from L8's every-frame harvest (lamp/light/other hand labels; other round red =
real). Per phase: lamp/light/other tracks accepted as no-entry, accepted as any sign class; real round red tracks
accepted with a round red class; physical B-2 and B-33 signs caught.

E3 L11 six rides (krakow-2024, rural-zloczew, forest-vistula-spit, s1-roadworks, warsaw-city, katowice-rain), phases
0 and 2, deployed rules engine (`rules_replay.py` defaults: lights cue on, priority inference on), L11's committed
labels and `score.py`: false rules (all rules), matched / missed intervals, pooled time agreement per rule,
momentary + warning true positives / engine / human (+-6 s).

E4 named cases: krakow B-33 70 (human 87.9 s): phases (of 4) in which the speed-limit state serves 70 from a sign
starting within 87.9 +- 10 s; s1-roadworks D-7 (human 67 s): phases (of 2) with an accepted `expressway` interval
starting within 67 +- 10 s, and any track classed `information--limited-access-road` / `motorway` in 55-80 s.

**Deploy rule** (cls1 vs base; det2 likewise vs the best of base/cls1). Deploy only if ALL hold:
1. L8 (E2): lamp/light/other tracks accepted as no-entry = 0 in every phase; accepted as any sign class, summed over
   the 4 phases, <= base; real round red tracks accepted, summed, >= 0.95 x base; physical B-2 signs caught in each
   phase >= base, B-33 (any value) phase count >= base.
2. Fewer false rules (E3): total false rules over 6 rides x 2 phases < base.
3. Equal or better recall (E3/E4): total missed intervals <= base; momentary + warning true positives >= base;
   B-33 70 phases >= base; no rule's pooled agreement (mean of the two phases) below base by more than 0.02.
4. Phone cost: road >= 13/s and signs >= 4.5/s with the new models installed (15-min window), classifier ms/crop
   within 1.5x of base.
Everything is reported whatever the outcome; a variant that fails is a null with its numbers.

### Pre-registration addendum (20:20, before any tracker variant or new model is scored)
Diagnosis while checking the base stream (E4 case, base models only): the krakow B-33 70 is **detected** on every
processed frame of its pass (84.6-85.9 s, raw scores 0.64-0.87, classifier p = 1.0; same on the phone's recorded
stream and offline) but **never tracked**: between two processed frames (every 4th, 5 Hz) the small sign at the right
edge moves ~1.1 box widths, the new track has no velocity yet, so neither pass of the association matches (IoU 0,
centre distance 1.10 > `dist_match` 1.0) and every detection starts a new 1-hit track; none reaches `min_hits` 3.
The miss is the tracker's, not the detector's or the classifier's. Change (roadsense-signs): `--dist-match-new X`
(default 1.0 = unchanged) is the fallback distance for tracks with one hit only. Value **2.0**, from that one
geometry (in-sample for the B-33 70 case, disclosed). Extra variants: **trk** = base models + `--dist-match-new 2.0`;
**cls1trk** = cls1 + `--dist-match-new 2.0`. The deploy rule above applies to every variant against base; if more
than one passes, the one with the lowest false rules + missed intervals (E3 totals) is deployed, ties to the
smaller change.
Same check for the s1-roadworks D-7 (base models, raw detections, after the addendum was committed): the D-7 is
detected and classified correctly (`information--limited-access-road`, scores 0.53-0.81) on all 7 processed frames
66.0-67.2 s, together with a speed-limit sign (64.8-67.8 s), and neither is ever tracked (D-7 box moves ~1.6 box
widths per processed frame). Both named misses are tracker misses.

### 3. Classifier cls1 (21:10-21:37)
Crops: MTSD train 36 588 images -> 196 163 training samples after selection (gt + detpos of the 400 classes, not
ambiguous; other-sign 39 197; background 67 024 incl. BDD train-split lamps 21 057, red lights 2 307, lights 12 083,
detector false positives 380 + 50). `tools/mtsd/train_cls.py`: YOLOv8n-cls from scratch (Ultralytics yaml, 1.95 M
parameters, the original's architecture), 402 classes, 30 epochs x 200 000 samples (class weight 1/sqrt(count)),
AdamW 2e-3 one-cycle, label smoothing 0.1, phone-like rendering (`cls_aug.py`: box jitter, box downscaled to its
phone size or 9-56 px, JPEG 35-95, NV12 chroma, gain/gamma/saturation/noise/motion blur, +-4 deg, no flips).
GPU: 43-58 s per epoch, 30 min total; final train accuracy 0.941 (augmented). ONNX export opset 17, `images`
[1,3,64,64] -> `output0` [1,402] softmax, |onnx - torch| 6e-7. Memory notes: Python 3.14's DataLoader default
(forkserver) pickled the 5 GB array into every worker (three OOM kills at the 12 GB cap); fixed with fork workers
and 72-px stored regions.
Disk: after extraction the MTSD image zips (35.7 GB) were deleted to keep >= 30 GB free (the links stay valid until
2026-11-26; annotations kept).

**E1 held-out crops** (`eval_cls.py`; MTSD val 6 516 signs (gt) / 4 453 (detector boxes), other-sign 4 000, MTSD
detector false positives 923, random 4 000; BDD test split: lamps 3 860 day / 3 063 night, red lights 240 / 454,
lights 2 120 / 1 711, detector FPs 95 / 26). "claim" = given a sign class with p >= 0.6 (a signd vote):

| metric | full res base | full res cls1 | phone size base | phone size cls1 | <= 16 px base | <= 16 px cls1 |
|---|---|---|---|---|---|---|
| top-1, val signs (gt box) | 0.902 | **0.945** | 0.859 | **0.939** | 0.783 | **0.919** |
| macro recall | 0.864 | **0.922** | 0.818 | **0.915** | 0.717 | **0.899** |
| top-1, detector boxes | 0.893 | **0.957** | 0.874 | **0.956** | 0.794 | **0.940** |
| other-sign claimed as one of the 400 | 0.675 | **0.082** | 0.667 | **0.090** | 0.620 | **0.101** |
| MTSD detector false positives claimed | 0.619 | **0.260** | 0.626 | **0.249** | 0.619 | **0.209** |
| MTSD random boxes claimed | 0.651 | **0.010** | 0.670 | **0.010** | 0.677 | **0.009** |
| BDD lamps claimed day / night | 0.650 / 0.771 | **0.012 / 0.002** | 0.657 / 0.779 | **0.009 / 0.001** | 0.661 / 0.771 | **0.006 / 0.002** |
| BDD red lights claimed day / night | 0.771 / 0.859 | **0.004 / 0.000** | 0.775 / 0.846 | **0.004 / 0.000** | 0.742 / 0.846 | **0.004 / 0.000** |
| BDD lights (any colour) day / night | 0.560 / 0.632 | **0.013 / 0.001** | 0.566 / 0.639 | **0.010 / 0.000** | 0.575 / 0.635 | **0.015 / 0.001** |
| BDD detector false positives | 0.768 | **0.316** | 0.758 | **0.295** | 0.695 | **0.284** |
| warning triangle -> priority-road (gt / det boxes) | 6/1722, 4/1434 | **0, 0** | 19, 14 | **0, 0** | 39, 42 | **0, 0** |
| blue mandatory -> round red class | 5/474 | **0** | 4 | 1 | 5 | **0** |
| go-straight -> one-way; keep-right -> no-entry | 1/37; 0/199 | 0; 0 | 0; 0 | 0; 0 | 0; 0 | 0; 0 |

Caveats: the renderings are cls1's own training augmentation family (an advantage for cls1); base is the PoVa
model trained on MTSD train as well (its authors' val top-1 89.3 % matches the 0.902 here). The remaining MTSD
"detector false positives" claimed by cls1 include unannotated real signs (seen in the extraction check sheet).

### 4. Ride evaluation (21:00-22:53; `eval_variant.sh` per variant, `compare.py base trk cls1 cls1trk`)
Four variants on identical frames (6 rides x phases 0 and 2, krakow also 1 and 3; laptop signd_replay, deployed
rules engine, L11 labels). No threshold, rule or model was changed after these numbers existed.

| metric | base | trk | cls1 | **cls1trk** |
|---|---|---|---|---|
| E3 false rules (6 rides x 2 phases) | 51 | 53 | 44 | **47** |
| E3 missed intervals | 76 | 74 | 77 | **74** |
| E3 matched intervals | 112 | 119 | 112 | **118** |
| E3 momentary + warning true positives | 266 | 295 | 267 | **291** |
| E3 momentary + warning engine events | 555 | 656 | 497 | 587 |
| E2 lamp/light/other tracks accepted as no-entry (4 phases) | 0 | 0 | 0 | **0** |
| E2 lamp/light/other tracks accepted as any sign (of 33) | 0 | 0 | 0 | **0** |
| E2 real round red tracks accepted / all | 126/137 | 145/158 | 122/137 | **139/158** |
| E2 physical B-2 signs caught per phase | 3,3,3,3 | 4,4,4,4 | 3,3,3,3 | **4,4,4,4** |
| E2 physical B-33 signs caught per phase | 1,2,2,2 | 3,3,3,3 | 1,2,2,2 | **3,3,3,3** |
| E4 krakow B-33 70 served (phases of 4) | 3 | 4 | 3 | **4** |
| E4 s1 D-7: track classed limited-access-road (phases of 2) | 0 | 2 | 0 | **2** |
| E4 s1 D-7: expressway from the sign at 69.4 s (speed-limit timeline, see note) | 0 | 2 | 0 | **2** |
| agreement priority_road (false, missed) | 0.807 (F6 M6) | 0.839 (F6 M6) | 0.897 (F2 M6) | **0.913 (F2 M6)** |
| agreement one_way | 0.802 (F13 M21) | 0.808 (F12 M22) | 0.824 (F13 M22) | 0.819 (F12 M22) |
| agreement no_stopping | 0.785 (F21 M32) | 0.782 (F24 M30) | 0.786 (F20 M32) | 0.780 (F24 M30) |
| agreement no_overtaking | 0.932 (F5 M2) | 0.951 (F5 M1) | 0.900 (F4 M2) | 0.912 (F4 M1) |
| agreement height_limit | 0.992 (F2 M0) | 0.998 (F2 M0) | 0.992 (F2 M0) | 0.998 (F2 M0) |
| agreement weight_limit | 0.976 (F1 M2) | 0.976 (F1 M2) | 0.980 (F0 M2) | 0.980 (F0 M2) |
| agreement no_parking | 0.990 (F3 M1) | 0.990 (F3 M1) | 0.990 (F3 M1) | 0.990 (F3 M1) |
| agreement built_up_area / expressway | 0.436 / 0.147 | same | same | same |

Deploy rule (pre-registered) per variant vs base:
* **trk**: L8 pass; recall pass; **fails rule 2** (false rules 53 >= 51: more tracks, more no-stopping rules). Null.
* **cls1**: L8 pass (real round red 122 >= 0.95 x 126 = 119.7); false rules 44 < 51 pass; **fails rule 3** (missed
  77 > 76; no_overtaking agreement -0.032). Null.
* **cls1trk**: L8 pass (0 / 0; real round red 139 >= 119.7; B-2 4 >= 3 and B-33 3 >= 1-2 per phase); false rules
  47 < 51 pass; missed 74 <= 76, momentary + warning TP 291 >= 266, B-33 70 4 >= 3 pass; agreements: no rule more
  than 0.02 below base: no_overtaking 0.912 vs 0.932 = **exactly 0.020** (score.py rounds agreements to 3
  decimals; at the limit, not beyond it), no_stopping -0.005. **Passes** -> deploy candidate (rule 4 on the phone).

Notes. (1) The E4 "expressway interval" metric as pre-registered cannot see the result: `rules_replay.py` records
`active` sign rules only, while D-7 sets the speed engine's road class (in effect as `expressway`, speed 120
"default expressway" at 69.4 s); counted from the speed-limit timeline instead (disclosed). The L11 pooled
"expressway agreement 0.147" has the same blind spot. (2) What cls1 fixed on the rides: priority_road false rules
6 -> 2 (the deer triangles read as D-1 on s1 are gone; agreement 0.807 -> 0.897/0.913), weight_limit false 1 -> 0,
fewer engine events (555 -> 497) at equal true positives. (3) What cls1 lost: on the Polish rides it votes less
often on some signs (a B-42 end-of-prohibition and an A-7 junction warning in forest-vistula-spit came out
`traffic-sign`, no confident vote, in phase 2: a B-25 then ran to its 3 km cap = the no_overtaking drop). On MTSD
val its vote rate (p >= 0.6) is the same as base's (0.938 vs 0.941, correct votes 0.925 vs 0.855), so this is a
domain effect (Polish variants), not a calibration collapse; candidate fix in the open items. (4) The tracker change
alone (trk) adds recall and false rules together: it starts tracks that the old classifier then misreads.

### 5. Phone deploy (22:57) and live check
cls1trk passed rules 1-3, so it was installed (`~/l12/install.sh` on the phone, as root): signd + signd_replay built
on the phone from roadsense-signs 376f841 + this lane's unit (backups `/usr/local/bin/signd.pre-l12`,
`signd_replay.pre-l12`, `~/l12/roadsense-signd.service.pre-l12`), classifier `/var/lib/roadsense/models/
mtsd_signcls_l12_64.onnx` + `mtsd_signcls_l12_classes.txt` (= laptop `~/l12/cls1/`), unit adds `--cls ... --cls-names
... --dist-match-new 2.0`. One signd restart; modeld, api, camerad-replay (21-ride playlist, untouched), envd stayed
active; autostart still disabled. signd log: "classifier on ort (mtsd_signcls_l12_64.onnx), 402 classes (258 merged)".

Rule 4 (phone cost), 15-min windows from the journals:

| window | road /s | signs /s | detect_ms | thermal max |
|---|---|---|---|---|
| before (22:40-22:55) | 13.71 | 4.96 | 94.9 | 81.3 |
| after (22:58-23:13) | 13.56 | 4.96 | 94.9 | 85.2 |

Classifier cost on the phone (signd_replay, ORT, little cores, 40 krakow frames with 75 crops): 25.9 ms (base) vs
25.4 ms (L12) per frame. Rule 4 passes (road >= 13, signs >= 4.5, classifier within 1.5x). Live after the install
(rides 6-7 of the playlist): 44 ended tracks, 39 signs, 3 `traffic-sign` (no confident vote), 1 `other-sign`, 1
flagged `glow`; no `not_a_sign` yet on these rides (offline the krakow ride gives the most). The rides differ from
the evaluation rides only in position; the evaluation is the offline one above.

## Result
* Deployed: MTSD-retrained classifier (402 classes with `other-sign` and `background`) + tracker `--dist-match-new 2.0`
  (variant cls1trk), the only variant that passed every pre-registered rule. Six rides x 2 phases: false rules
  51 -> 47, missed intervals 76 -> 74, momentary + warning hits 266 -> 291; Krakow lamps/lights still never accepted
  (0 of 33 tracks), real round red tracks 126 -> 139; B-33 70 served in 4 of 4 phases (base 3; the phone's own pass
  was the missed one); the s1 D-7 is now tracked and sets the expressway road class in both phases (base never).
* The two named recall misses were **tracker** misses (detected and correctly classified on every processed frame,
  never associated across 5 Hz frames), not detector misses; so no detector was retrained (the MTSD image zips were
  deleted after crop extraction to keep the disk floor; the links stay valid until 2026-11-26).
* Held-out classifier gains are large (MTSD val top-1 at phone size 0.859 -> 0.939, lamps/lights claimed 66-85 % ->
  0-1 %, warning -> priority-road 19 -> 0), but on Polish rides cls1 alone did not pass (one more miss and a B-25
  running to its cap after a B-42 got no confident vote).

## Limitations / open items
* no_overtaking agreement sits exactly at the pre-registered limit (-0.020); cls1 votes `traffic-sign` (no class
  with p >= 0.6) on some Polish signs (B-42, an A-7 warning). Candidate (not tried, would need a new pre-registration):
  train without label smoothing or gate on the merged-class probability (variants --g1/--g2 split the mass), and add
  Polish-specific renderings.
* E4's expressway interval metric is blind to road-class rules (`rules_replay.py` records sign rules only); the L11
  "expressway 0.147" carries the same blind spot. Fix belongs to the scorer.
* Single evaluation recording per ride (phases 0/2 are the same drive); labels single labeller (L11).
* `trk` alone adds false rules: the tracker change needs the better classifier. Larger `dist_match` for established
  tracks was not tested.
* Detector unchanged: far signs below ~15 px are still missed by the detector itself; a higher input size costs GPU
  turns (modeld must stay >= 13/s). Fine-tuning needs a pretrained detector checkpoint (external load) or a long
  from-scratch run.
* Weights research-licensed (MTSD CC BY-NC-SA, BDD100K): laptop `~/l12/cls1/`, phone `/var/lib/roadsense/models/`;
  never committed.

## Deviations
* Tracker variant added after the base diagnosis (pre-registration addendum committed 20:20, before any variant was
  scored); its value 2.0 came from the B-33 70 geometry (in-sample for that case).
* E4 D-7 counted from the speed-limit timeline instead of the rules intervals (tool blind spot, above).
* Stored training regions shrunk 96 -> 72 px and random negatives 50 -> 30 % after three OOM kills (before any scoring).
* Disk floor: 26 GB free for a few minutes (~21:50) before the image zips were deleted.
* No permission denial occurred; no downloaded checkpoint was loaded.
