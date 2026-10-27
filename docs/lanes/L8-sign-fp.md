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
