# L8 sign false positives: lamps and traffic lights read as round red signs

Lane opened 2026-10-27 ~10:35 CET (Warsaw local, UTC+1; Opus, build + data + measurement). Problem (user): signd
very often reports `regulatory--no-entry` (B-2) on the leading car's stop lamps. User ruling: do all of
(1) per-class acceptance threshold, (2) context suppression (traffic-light overlap, lead vehicle, sign height),
(3) a glow/appearance check on the crop, (4) negative training (verifier or hard-negative retraining) with
better open data; check the other round red classes too; keep B-2 and speed limits alive (L6 consumes them).

Laptop scratch `~/l8/` (scripts, recordings, harvests, models); phone scratch `~/l8/`. Downloads under
`~/datasets/<name>/` with a `SOURCE.txt` each.

## 0. First look (phone sign history, before any change)
`/var/lib/roadsense/signs.jsonl` on the phone (pulled 10:41 CET): 13 134 ended tracks over 68.4 replay loops
(seq 39..820 875; the replay never dropped a frame, so `file_frame = (seq - 1) mod 12000`).
908 `no-entry` tracks = 13.3 per loop. Clustered by position in the loop they recur every loop:

| loop time (s) | tracks / loop | median best score | median hits | what the crops show |
|---|---|---|---|---|
| 140-155 | 3 | 0.46-0.56 | 17-18 | red traffic light (round lamp in a housing) |
| 210, 260 | ~0.1 | 0.86 | 3-9 | rare (not inspected) |
| 240-247 | 1 | 0.88 | 24 | real B-2 (with "nie dotyczy" plate) |
| 270-272 | 1 | 0.88 | 8 | real B-2 |
| 360 | 1 | 0.86 | 9 | real B-2 |
| 420-438 | 4 | 0.39-0.55 | 4-127 | red traffic lights + bus/tram brake lamp bar (430-435 s, up to 127 hits) |
| 450-469 | 1.2 | 0.48-0.57 | 46 | red traffic light |
| 480-490 | 2 | 0.60-0.68 | 12-26 | red traffic lights (round, glowing) |

So per loop ~3 real B-2 tracks and ~10 false ones; the false ones are mostly red traffic lights, plus one
bus/tram lamp bar. (Crop files are named by track id and ids restart with signd, so only crops of the
current signd process were used; `~/l8/phone0/ne_sheet*.png`.)

## 1. Pre-registration (written before any fix was scored)
Classes in scope ("round red", RR): merged MTSD names matching
`^regulatory--(no-|maximum-speed-limit|road-closed|height-limit|weight-limit|width-limit|stop)`.
Key classes: `no-entry`, `maximum-speed-limit-*`, `no-stopping`, `no-parking`, `no-left-turn`,
`no-right-turn`, `no-overtaking`.

Ground truth. A track is labelled by a human (this lane) from its best crop and, where needed, the frame:
`real` (a real sign of the reported class, or of another RR class), `light` (traffic-light lamp), `lamp`
(vehicle lamp), `other` (anything else). Because the replay repeats, tracks are grouped into loop-position
clusters (file time +- 3 s and box overlap) and a cluster is labelled once after looking at >= 3 of its crops;
a cluster with mixed content is split by box.

Split (the Krakow loop repeats the same physical objects, so anything tuned on it is in-sample):
* tuning: Krakow loop time 0-300 s, BDD100K val images whose id hashes to train (md5(id) first hex digit < c),
  GTSRB `Train/`;
* scoring: Krakow loop time 300-601 s, BDD100K held-out images (hex digit >= c), GTSRB `Test/`.
  Whole-loop numbers are reported too, marked in-sample for 0-300 s.

Metrics, per RR class and pooled, per fix and combined:
* track precision = real / accepted tracks; track retention = accepted real tracks after / real tracks before
  (the "recall" available without annotating every physical sign); physical-sign recall for B-2 and B-33
  (passes with >= 1 accepted track);
* false `no-entry` tracks per loop (before / after, phone recordings, >= 2 loops each);
* L6: speed-limit timeline agreement with the L6 human reading over a recorded loop
  (`tools/speedlimit_replay.py`), before vs after;
* held-out negatives: share of BDD-mined lamp/light detections (classified RR) that a fix still accepts;
  share of GTSRB Test RR signs that the appearance check / verifier rejects;
* cost on the phone: detect_ms, signs/road/lights rates (road must stay >= ~13/s), die temperature.

Decision rule: a fix ships if on the scoring split it removes >= 50 % of the false RR tracks while keeping
>= 95 % of the real RR tracks (and every B-2 / B-33 physical sign that was caught before), and costs < 0.3
road/s. Fixes that fail are reported as nulls.

## 2. Data (downloads, licences, what each is used for)
| Set | Where | Licence | Use |
|---|---|---|---|
| Krakow ride (10-min loop, 1280x720, 20 fps) | `~/datasets/rides/krakow-2024/` (L5) | CC BY 3.0 (Relaxing Roads 4K, Wikimedia Commons) | harvest of every frame with a Python mirror of signd (ORT, `tools/verifier/harvest_ride.py`): 11 000 detections, 1 111 IoU tracks, 103 with >= 2 round red votes, hand-labelled (34 false: 16 light, 12 lamp, 6 other); phone recordings |
| BDD100K val (10 000 images, FiftyOne export) | `~/datasets/dashcam/bdd100k_val_fo/` (L2/L7) | BDD100K licence, non-commercial research | detector-mined round red detections vs GT boxes (582: 540 on traffic signs, 23 on vehicles/lights), 27 980 red-blob crops in car/truck/bus boxes (stop/tail lamps, day + night), 3 001 red-light blobs (GT colour `R`) |
| GTSRB (51 839 images) | `~/datasets/gtsrb-zenodo/` (new, 320 MB, md5 7faafd0d...) | official site: free to use, cite Stallkamp et al. IJCNN 2011 (site unreachable 2026-10-27); Zenodo record 13741936 copy lists CC BY 4.0 | positives: round red classes (speed limits, no overtaking, no entry, no vehicles, no trucks, stop): 6 000 train / 3 000 test crops |
| MTSD (for hard-negative retraining of the classifier) | - | CC BY-NC-SA, download needs a Mapillary account | not used (no accounts): the classifier stays as is, a separate verifier is trained instead |

No download or loading step was denied. Weights trained here (BDD/GTSRB/ride) are not committed.

## 3. Offline analysis (laptop)
### 3a. Appearance numbers per detection (32x32 box resample, `tools/verifier/glow32.py` = signd `glow_score`)
Glow >= 0.10 share by source (all splits): ride real signs 0.3 % (only one large white no-right-turn, 18 of
152 detections), ride lamps 100 %, ride lights 100 %; BDD lamp blobs 64-88 % (unlit tail lamps are not
glowing); but **GTSRB real round red signs 13-16 %** and BDD real signs 5-13 % (sunlit signs with a blown-out
white field and a bright ring). Per physical GTSRB track (Train, 640 tracks) the track rule (>= half of the
detections glow) would flag **14.8 %** of real signs; stricter settings do not fix it (px 0.3 / share 0.8:
8.7 % of real flagged while catching only 89 % of the ride's lamp/light tracks). => glow alone is a **null**
outside overcast light; it ships only gated by the verifier (suppresses when the verifier mean is < 0.9 as
well) and `--glow-alone` exists for users without a verifier who accept the risk.

A colour check for wrong-class tracks (a blue keep-right sign read as no-entry at 212 s, a blue cycle-path
sign read as height-limit at 177 s) was tried as "red share of the disc"; the ride's red rings are too dull
(R ~ 100-150) for a fixed threshold: null, left open.

### 3b. Verifier (`tools/verifier/train_verifier.py`, 12 epochs, 4 threads, ~5 min)
Train: all `train` splits (GTSRB Train, BDD id-hash train, ride loop time < 300 s), balanced real/not-real,
augmentation (box jitter, small-box blur 8-32 px, gain/gamma/saturation, flips). Held-out share predicted
real (p >= 0.5):

| held-out set | n | v1 (all train data) | ext (no ride data) |
|---|---|---|---|
| ride lamps (bus/tram bar, 430-460 s) | 384 | 0.000 | 0.000 |
| ride red lights (300-601 s) | 324 | 0.000 | 0.000 |
| ride real round red signs (300-601 s) | 949 | 0.997 | 0.945 |
| BDD lamp blobs day / night | 585 / 415 | 0.005 / 0.002 | 0.007 / 0.000 |
| BDD red-light blobs day / night | 172 / 328 | 0.012 / 0.000 | 0.012 / 0.000 |
| BDD real signs day / night (US designs) | 115 / 21 | 0.913 / 0.857 | 0.948 / 0.952 |
| GTSRB Test round red | 3 000 | 0.999 | 0.999 |
| BDD detector-mined on vehicles | 7 | 3 round bus lamps accepted, 2 real signs (STOP, DO NOT ENTER inside a vehicle box) accepted, 2 rejected | |

Commands: `build_sets.py ~/l8/harvest ~/datasets/gtsrb-zenodo/data ~/l8/harvest/sets.npz`, then
`train_verifier.py ~/l8/harvest/sets.npz ~/l8/models/verifier_v1.onnx [--exclude-src krakow]`.

### 3c. Fixes scored offline on the phone's before recording
Before recording: phone `/ws` 10:42-11:14 CET (`~/l8/scripts/rec_ws.py`, 81 289 messages, loops 68-71, 139 ended round red
tracks; every one labelled from its phone crop, 16 harvest labels corrected by eye, `~/l8/rec/labels_before.json`).
Glow/verifier per phone detection from the harvest's pixels of the same frame (IoU >= 0.3); context from the
recorded lights / road messages. `eval_fixes.py before_tracks.json labels_before.json ~/l8/harvest/krakow verifier_v1.onnx`.
F1 = no-entry best score >= 0.71 (midpoint of the tuning half: max false 0.558, min real 0.86); F2a = >= half of the
detections inside a light box (+25 %); F2b = inside the lead's projected rear; F3 = glow alone; F4 = verifier.
Scoring half (loop time >= 300 s): 61 round red tracks; loops [68, 69, 70, 71]
| fix | accepted real | accepted false (lamp/light/other) | accepted wrong-class | real kept | false removed | precision | false no-entry per loop (this half; 3.19 loops recorded) |
|---|---|---|---|---|---|---|---|
| none | 37 | 22 | 2 | 37/37 = 1.000 | 0/22 = 0.00 | 0.607 | 6.89 |
| F1 score | 37 | 0 | 2 | 37/37 = 1.000 | 22/22 = 1.00 | 0.949 | 0.00 |
| F2a light | 36 | 16 | 2 | 36/37 = 0.973 | 6/22 = 0.27 | 0.667 | 5.01 |
| F2b lead | 37 | 6 | 2 | 37/37 = 1.000 | 16/22 = 0.73 | 0.822 | 1.88 |
| F2 light|lead | 36 | 0 | 2 | 36/37 = 0.973 | 22/22 = 1.00 | 0.947 | 0.00 |
| F3 glow | 37 | 7 | 2 | 37/37 = 1.000 | 15/22 = 0.68 | 0.804 | 2.19 |
| F4 verifier | 37 | 7 | 2 | 37/37 = 1.000 | 15/22 = 0.68 | 0.804 | 2.19 |
| F1+F3+F4 | 37 | 0 | 2 | 37/37 = 1.000 | 22/22 = 1.00 | 0.949 | 0.00 |
| F3+F4 | 37 | 7 | 2 | 37/37 = 1.000 | 15/22 = 0.68 | 0.804 | 2.19 |
| all | 36 | 0 | 2 | 36/37 = 0.973 | 22/22 = 1.00 | 0.947 | 0.00 |

F3/F4 numbers are a lower bound offline: 7 small pedestrian-light tracks (421-472 s) had no matching red
detection in the laptop harvest, so no pixels to judge (on the phone every detection is judged). F2a kills one
real STOP (3 hits, next to a signal); F2b passes but a real no-left-turn sits 40 % of its life inside the lead box.
Wrong-class real signs (blue signs read as no-entry / height-limit / stop, 7 tracks) are untouched by every fix.

Phone history before any change (69 loops, `signs.jsonl`, clusters labelled by the recording): no-entry per loop
= 2.96 real, 7.14 red lights, 2.96 lamps, 0.10 wrong class, i.e. **10.2 false no-entry tracks per loop**.

## 4. What was built
* **signd** (roadsense-signs 7ba2247, c07af24): per round red detection a 32x32 resample of the box (same
  resampler as the classifier) gives `glow` and, with `--verifier PATH`, the verifier probability (ORT, CPU).
  The tracker flags a confirmed track `suppressed` + `reason`: `verifier` (mean < 0.3 over >= 2 detections),
  `glow` (>= half glow >= 0.10 and verifier mean < 0.9; without a verifier only with `--glow-alone`), `score`
  (best detector score below `--class-min-score`, prefix match; unit: `regulatory--no-entry=0.71`). Tracks
  are still reported (flag + `glow` + `verify` fields); a missing verifier file only disables the verifier.
  `signd_replay` has the same flags. Docs: roadsense-signs `docs/SIGN_MODELS.md` section 6, README.
* **api** (roadsense d6ebdc3): `signctx.py` adds `context` {traffic_light, lead_vehicle} shares and flags
  round red tracks (>= 2 detections, share >= 0.5) unless the provider vouches (verify >= 0.9, glow < 0.5);
  flagged tracks are kept apart (`suppressed_tracks` in `/v1/signs`, not live, grey with reason in `/preview`,
  `speed_limit` lists them in `ignored` as `suppressed (<reason>)`); history keeps the flags. Contract in
  `docs/ARCHITECTURE.md` ("Flagged tracks"). 47 api tests (6 new in `api/tests/test_signctx.py`).
* **tools/verifier/** (roadsense-signs): ride/BDD harvest, BDD blob miner, GTSRB fetch, set builder, trainer.

## 5. Phone results (Krakow replay, Warsaw local times, `~/l8/scripts/rec_ws.py` + `rec_tracks.py` + `eval_live.py`)
Labels of the later recordings are transferred from the labelled before recording (same class, loop time
within 2 s; loop time from camerad-replay's `file_frame`, since frame drops shift `(seq - 1) mod 12000`).

| run | config | loops | round red tracks | real kept | false (lamp/light/other) left | wrong-class left | precision raw -> accepted | **false no-entry per loop** |
|---|---|---|---|---|---|---|---|---|
| history 23:16-10:41 | before | 69 | - | - | - | - | - | **10.2** (7.1 lights, 3.0 lamps, 0.1 wrong class) |
| before 10:42-11:14 | before | 3.19 | 139 | 101/101 | 31/31 | 7 | 0.727 -> 0.727 | **10.65** |
| after 11:21-11:53 | test build, verifier thr 0.5, glow alone, score, api context | 3.16 | 144 | 108/109 | 0/30 | 5 | 0.757 -> 0.956 | **0.95** (all: a blue keep-right sign read as no-entry) |
| final 11:56-12:17 | installed: verifier thr 0.3, glow gated, score, api context | 2.10 | 89 | 65/65 | 0/20 | 2 | 0.730 -> 0.942 | **0.00** |

Scoring half only (loop time >= 300 s): after 29/30 real kept, 0/21 false left; final 21/21 real kept, 0/15
false left. The one real track lost in "after" was a small STOP at 356 s (3 hits, verifier mean 0.40); every
false track had a verifier mean <= 0.19 and every other real track >= 0.77, hence the installed threshold
0.3 (chosen on this loop, so in-sample; the held-out BDD/GTSRB rates at 0.3 are in section 3b's model:
lamp/light blobs accepted 0.3-1.9 %, real signs kept 90-100 %).

Per class, final run (whole loop): no-entry 26 tracks, precision 0.23 -> 1.00, real kept 6/6; no-stopping
25/25 real kept; no-left-turn 10/10 (2 wrong-class-free unlabelled tracks accepted, verifier 1.0); no-right-turn
6/6; speed limit 40 2/2 and 70 1/1; stop 2/2; no-parking 3/3; height-limit 2/2 (+1 blue sign read as
height-limit left); no-heavy-goods 2/2 (+1 blue sign left). Reasons: verifier 19, score 1. The api's context
flagged every false track as well (light or lead, share 0.8-1.0) and no real track in the after run.

Speed limit (phone api's own `speed_limit` topic, `~/l8/scripts/sl_compare.py`): the timeline is unchanged:
B-33 40 at loop time 193.3 s -> 50 at 210 s in every pass (before 3/3, after 4/4, final 2/2), B-33 70 at 87.9 s
in 1 of 3 / 1 of 4 / 1 of 2 passes (detector recall, fold 6). Time share 50/40/70: before 0.960/0.026/0.014,
final 0.952/0.027/0.021. No B-33 track was flagged.

## 6. Cost (same recordings)
| run | road /s | signs /s | lights /s | detect_ms mean / p90 | modeld thermal max (C) |
|---|---|---|---|---|---|
| before | 13.43 | 4.98 | 2.99 | 100.6 / 123.9 | 89 |
| after | 13.55 | 4.98 | 2.99 | 99.8 / 122.7 | 89 |
| final | 13.38 | 4.96 | 2.99 | 99.6 / 122.3 | 88 |
No measurable cost (the verifier runs only on round red boxes, < 0.1 ms each); road stays >= 13/s. The
final run's camera rate was 19.75/s (a few replay frame drops; also seen as the loop-time shift above).

## 7. Which fixes paid (pre-registered rule: >= 50 % of false removed, >= 95 % real kept, every caught B-2/B-33 kept)
| fix | scoring half offline | live | verdict |
|---|---|---|---|
| F1 per-class score (no-entry >= 0.71) | 22/22 removed, 37/37 kept | 1 flag (rest already flagged by the verifier) | **ships** (safety net without a verifier). Caveat: BDD single-image detections of real US "do not enter" signs have median score 0.72; on the ride all 204 real B-2 tracks of 69 loops had best >= 0.854 |
| F2a traffic-light overlap | 6/22 removed, 1 real STOP lost | flags all lights it overlaps | fails alone (pedestrian lights are often not in the lights topic) |
| F2b lead vehicle | 16/22 removed, 37/37 kept | flags the lamp bar and low lights | passes alone; a real no-left-turn sat 42 % of its life inside the lead box (margin is thin) |
| F2 light or lead (api) | 22/22 removed, 36/37 kept | flags every false track, no real one | **ships gated** (only when the verifier does not vouch) |
| F3 glow | 15/22 (coverage-limited), 37/37 kept; per detection perfect on the ride | 30/30 flagged (after run) | **null alone** (flags 14.8 % of real GTSRB signs); ships gated by the verifier |
| F4 verifier | 15/22 (coverage-limited), 37/37 kept | 30/30 + 19/19 flagged, 108/109 then 65/65 kept | **ships** (main fix) |
| hard-negative retraining of the classifier | - | - | not possible without an account (MTSD) |

## 8. Open items and nulls
* Wrong-class real signs (blue keep-right read as no-entry at 212 s in some passes, blue cycle-path sign as
  height-limit, priority-road/blue signs as stop or no-heavy-goods) are untouched: they are real signs, so the
  verifier accepts them (0.5-0.99). A colour-consistency check failed on this ride (dull reds). Candidate: a
  class-aware verifier or colour check per class family.
* Generalisation: the ride is overcast city daylight only; night, rain, sun glare and the DVR camera are
  untested live. Held-out BDD night: lamp/light blobs accepted <= 0.7 %, real signs kept 86-95 %.
* The verifier is research-licensed (BDD100K, GTSRB, ride): installed by hand on the phone
  (`/var/lib/roadsense/models/sign_verifier_32.onnx`, = `~/l8/models/verifier_v1.onnx`), never committed.
* `context` uses modeld's lead and the replay's guessed calibration; the lead box margin to real signs is thin.

## 9. Phone state at hand-back (12:2x)
Installed: `/usr/local/bin/signd` + `signd_replay` from `~/l8/build-signs` (previous `signd.pre-l8`), unit
`/etc/systemd/system/roadsense-signd.service` = repo (adds `--verifier ... --class-min-score
regulatory--no-entry=0.71`; previous `~/l8/roadsense-signd.service.pre-l8`), verifier model, api from this
repo (previous `/usr/local/lib/roadsense/api.pre-l8`). No drop-ins (L8's /run dirs removed; an empty
`roadsense-modeld.service.d` predates the lane). Full stack running on the replay; `roadsense.target` autostart
disabled. Verified 12:2x: road 13.3/s, signs 4.9/s, lights 2.9/s, 75 C; history shows `no-entry` lamps flagged
`verifier`.

## Deviations
* None of the permission-relevant kind: no download or load was denied; no account was created.
* The pre-registered "share of GTSRB Test signs the appearance check rejects" was answered per image (13.2 %) and
  per physical track on GTSRB Train (14.8 %) since GTSRB Test has no track ids.
* The verifier threshold (0.5 -> 0.3) and the glow gating were changed after the after-run, from its numbers
  (in-sample for the ride; marked so in section 5).
