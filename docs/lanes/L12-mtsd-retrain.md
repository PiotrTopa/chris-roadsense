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

### Pre-registration addendum (20:5x, before any tracker variant or new model is scored)
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
