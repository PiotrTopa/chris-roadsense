# L29: a separate lane-line source for the display (laned), and the lead release time

Lane of the 2.0 campaign, 2026-11-19, Fable. Branch `v2-l29` from `v2` (5fd6c2f). Owner's rulings of
2026-11-19: the new model line is the base; lane lines for drawing come from a separate cheap analysis and are a
nice-to-have, display only; build the lane model for the MFD view; the lead is released after about 0.7 s.
Laptop only: the phone was in use by L28 and was not touched.

## Verdict

* **Lead release: done.** `coast_s` 0.8 -> 0.6 in the vision profile. Release 0.92 -> 0.70 s (median, 13.8
  runs/s, scoring rides), paid with 17 % more on/off changes (section 1).
* **Display lanes: no candidate beats the road model's own lines as a source of its own.** Four detectors were
  run through one back-end and one display chain (a classical paint analysis, UFLD, YOLOP, TwinLiteNet). Used
  alone (`replace`) each of them is worse than the new model's lines on flicker (2-5 times), on steps, on
  far-range consistency in town, and each invents lines on snow where the model draws none. The classical
  analysis is the best or equal best of the four and costs about a hundredth of the others.
* **What was built and what it is worth.** `laned` (the classical analysis as a daemon, optional unit, off by
  default) and a seam mode `agree` (default): a drawn line is laned's where laned and the road model agree on
  the marking, the model's everywhere else. In that mode the picture has exactly the model's lines (same
  presence, same flicker, nothing invented), and they lie on the paint more often: at 20-40 m 58 % of the drawn
  points are on a paint stripe against 43 % (held-out rides). Far-range consistency is better on the night
  motorway and the night expressway (0.06 against 0.13 m, 0.12 against 0.17 m at 20-30 m) and worse in the day
  city and on the day expressway (0.16 against 0.09 m, 0.05 against 0.03 m): on average no gain (0.10 against
  0.10 m). Supercombo's level (0.06 m) is not reached.
* **Recommendation:** keep the unit off on the car until the phone timing and an on-road A/B (section 7) are
  done; the gain on the top-down MFD is small and mixed. Nothing is forced in: with the unit off, `v2` behaves
  as before, except for the lead release.

## 1. Lead release (job 1)

`PROFILES["vision"]["coast_s"]` 0.8 -> 0.6 (`api/roadsense_api/lead.py`). Re-measured on the current profile
with L27's tool and split (tuned on krakow-2024, warsaw-storm, gdansk-night-snow, tunnels-s3; scored on
s7-elblag-lowsun, krakow-night-tunnel, a1-night, warsaw-city; 5 min each):
`python tools/vision_eval/lead_eval.py score LEAD_DIR --rides s7-elblag-lowsun,krakow-night-tunnel,a1-night,warsaw-city [--set coast_s=0.8]`

| after the tracker, mean of the 4 scoring rides | supercombo 13.8/s | vision, L27's profile (`coast_s` 0.8) 13.8/s | **vision, `coast_s` 0.6, 13.8/s** | 17/s: 0.8 -> 0.6 | 20/s (3 rides): 0.8 -> 0.6 |
|---|---|---|---|---|---|
| release after the lead is gone, median / longest, s | 0.22 / 0.22 | 0.92 / 1.45 | **0.70 / 1.27** | 1.01 / 1.76 -> 0.98 / 1.73 | 1.02 / 1.68 -> 0.98 / 1.43 |
| lead on/off changes / min | 1.65 | 1.75 | **2.05** | 1.95 -> 2.25 | 2.07 -> 2.47 |
| identity changes / min | 0.95 | 0.69 | **0.84** | 0.79 -> 0.94 | 0.80 -> 1.00 |
| distance jumps over max(1.5 m, 8 %) / min | 0.63 | 0.50 | **0.40** | 0.55 -> 0.50 | 0.53 -> 0.40 |
| per-frame step after the relative speed: rms / p95, m | 1.04 / 1.93 | 0.73 / 1.26 | **0.71 / 1.25** | 0.62 / 1.10 -> 0.63 / 1.10 | 0.56 / 0.93 -> 0.56 / 0.94 |
| per-frame steps over 0.5 m / min | 84 | 108 | **106** | 118 -> 116 | 130 -> 128 |
| relative-speed noise, m/s | 3.31 | 0.69 | **0.69** | 0.71 -> 0.69 | 0.71 -> 0.70 |
| follows the raw distance: median / p90 error, % | 5.8 / 11.9 | 1.9 / 7.2 | **1.9 / 7.1** | 1.9 / 6.9 -> 1.9 / 7.0 | 1.9 / 7.4 -> 2.0 / 7.3 |
| lead present, % of runs | 40.8 | 46.3 | **45.2** | 47.4 -> 45.9 | 54.5 -> 53.1 |
| of it not confirmed (`coast`), % | 16.9 | 17.7 | **16.3** | 21.1 -> 19.8 | 27.9 -> 26.8 |
| presence agrees with supercombo's, % (only here / only there) | - | 90.3 (7.6 / 2.1) | **91.1 (6.7 / 2.3)** | 89.1 -> 90.5 | 86.3 -> 87.6 |

The 0.8 column reproduces L27's own-profile column exactly. Tuning rides (in-sample): release 1.01 -> 0.75 s,
on/off 1.81 -> 1.94 (+7 %; L27 had estimated +5 % on an earlier profile).

Reading:

* At 13.8 runs/s the release is 0.70 s. The price on the scoring rides is larger than L27's estimate: on/off
  changes +17 % (1.75 -> 2.05 a minute, 35 -> 41 events in 20 minutes), identity changes +22 %; jumps go down.
  With this few events the difference is within the spread between rides, but the direction is the same on all
  three rates.
* **At 17-20 runs/s the median stays near 1 s.** There the measured events are mostly leads that fade through
  weak evidence (probability 0.1-0.3 with a distance and speed that agree with the track): those tracks end
  `weak_coast_s` (2.5 s) after their last confirming frame, not `coast_s`. A lead that is cleanly gone (no weak
  frame) is released after 0.1-0.5 s now (0.3-0.75 s before). On the tuning rides `weak_coast_s` 1.5 brought the
  17/s median to 0.51 s and the longest release under 1 s at every rate with no change of the on/off count at
  13.8/s (1.94) and 17/s (2.19): not adopted (not asked for), a decision for the owner. The phone runs about 15
  runs/s.

### Follow-up: `weak_coast_s` 2.5 -> 1.5 (owner's ruling, adopted)

Rule set by the coordinator: adopt if on the held-out scoring rides the release improves and on/off and identity
changes do not rise by more than about 5 % against `coast_s` 0.6 / `weak_coast_s` 2.5 at 13.8 runs/s.
`python tools/vision_eval/lead_eval.py score LEAD_DIR --rides s7-elblag-lowsun,krakow-night-tunnel,a1-night,warsaw-city --set weak_coast_s=W`
(mean of the four scoring rides; 20/s: three rides; each cell 2.5 -> 1.5):

| after the tracker, `coast_s` 0.6 | 13.8/s: weak 2.5 -> **1.5** | 17/s | 20/s |
|---|---|---|---|
| release after the lead is gone, median / longest, s | 0.70 / 1.27 -> **0.70 / 0.86** | 0.98 / 1.73 -> 0.72 / 0.97 | 0.98 / 1.43 -> 0.73 / 0.93 |
| lead on/off changes / min | 2.05 -> **2.05** | 2.25 -> 2.25 | 2.47 -> 2.60 |
| identity changes / min | 0.84 -> **0.84** | 0.94 -> 0.94 | 1.00 -> 1.07 |
| distance jumps over max(1.5 m, 8 %) / min | 0.40 -> **0.40** | 0.50 -> 0.50 | 0.40 -> 0.40 |
| per-frame step after the relative speed: rms / p95, m | 0.71 / 1.25 -> **0.72 / 1.25** | 0.63 / 1.10 -> 0.63 / 1.12 | 0.56 / 0.94 -> 0.52 / 0.95 |
| relative-speed noise, m/s | 0.69 -> **0.69** | 0.69 -> 0.70 | 0.70 -> 0.72 |
| lead present, % of runs | 45.2 -> **45.0** | 45.9 -> 45.5 | 53.1 -> 52.3 |

In between (release median / longest at 13.8, 17, 20 runs/s): 2.0: 0.70 / 1.10, 0.84 / 1.31, 0.89 / 1.25;
1.75: 0.70 / 0.96, 0.78 / 1.14, 0.82 / 1.10. On/off and identity at 13.8/s and 17/s are the same for every value
from 1.5 to 2.5.

The rule is met: at 13.8 runs/s on/off and identity changes do not move, the median release stays 0.70 s and the
longest falls from 1.27 to 0.86 s; at 17 and 20 runs/s the median falls from 0.98 to 0.72-0.73 s and the longest
under 1 s. The one cost: at 20 runs/s (three rides) on/off +5 % (2.47 -> 2.60) and identity +7 % (1.00 -> 1.07),
which is one or two events in 15 minutes. `PROFILES["vision"]["weak_coast_s"]` is 1.5 now; a release of about
0.7 s holds at every rate.

## 2. Candidates

Phone: Adreno 630 at 84 % with two GPU clients already taking turns (envd on ncnn/Vulkan hung the GPU once:
`roadsense-envd.service`), big cores at 137 %, little cores at 192 of 400 %. So the room is on the little cores;
a third GPU client was not considered further. Phone times below are **estimates**: laptop single-thread time
times 3 (L27 measured the policy at 6.0 ms on the laptop and 18.0 ms on a little core).

| candidate | code licence | weights: where from, size, what they were trained on | input | ships as | laptop, 1 thread, per frame | phone estimate, one little core | state |
|---|---|---|---|---|---|---|---|
| **classical** (this lane: marking filter on a road-plane grid, line following, quadratic fit; `laned/`) | MIT (ours) | none | luma plane, about 130 image rows read | C++ | 1.4 ms idle, 2.1-5.4 ms with the laptop loaded | **4-16 ms: 2-8 % of one little core at 5 Hz** | evaluated, built |
| UFLD (ResNet18, CULane) | MIT | `mlboydaisuke/Ultra-Fast-Lane-Detection-OpenVINO` `ufld.onnx` (sha256 860ec2b2..., 178 MB; a conversion of litert-community's of cfzd's CULane weights). CULane: non-commercial research licence | 800x288 RGB | ONNX | 199 ms | 0.6 s (0.25 s on three cores: 1.5-4 Hz with all the spare little cores) | evaluated |
| UFLDv2 | MIT | PINTO model zoo 324: one archive of 2.9 GB for all variants; CULane / TuSimple | 800x320 or 1600x320 | ONNX in the archive | not run | at least UFLD's | not downloaded: 2.9 GB for a heavier network than UFLD, which already fails on cost and quality |
| YOLOP (lane segmentation head) | MIT | `hustvl/YOLOP` `weights/yolop-320-320.onnx` (86d6e8b6..., 32 MB) and `-640-640` (cd66a3e0..., 36 MB); BDD100K (its licence: non-commercial) | 320x320 / 640x640 letterbox | ONNX in the repo | 117 ms / 541 ms | 0.35 s / 1.6 s | 320 evaluated; 640 timed only |
| TwinLiteNet | MIT | `chequanghuy/TwinLiteNet` `pretrained/best.pth` (98ffc6c8..., 1.8 MB, 0.44 M parameters; a plain state dict, loaded with `weights_only=True`, exported to ONNX here); BDD100K | 640x360 | PyTorch state dict | 250 ms (torch), 690 ms (ORT: dilated convolutions) | 0.75-2 s | evaluated on the tuning rides only (the run was stopped: too slow) |
| TwinLiteNet+ (nano ... large) | MIT | Google Drive only, PyTorch files | 640x384 | PyTorch | not run | unknown | not fetched: needs the owner's go if wanted (Drive download of pickles; `weights_only` would have to be tried) |
| CLRNet (ResNet18 / DLA34) | Apache-2.0 | `Turoad/CLRNet` release `models` (`*.pth.zip`, 44-160 MB); CULane, LLAMAS, TuSimple | 800x320 | PyTorch checkpoints (pickle), custom NMS op | not run | heavier than UFLD | **needs the owner's go** (pickle checkpoints; no ONNX shipped) |
| YOLOPv2, HybridNets | MIT | TorchScript / PyTorch; BDD100K | 640x640 and up | no ONNX in the repo | not run | heavier than YOLOP | not pursued |

Licences: all code is MIT or Apache-2.0 and could go into the MIT core. **Every learned candidate's weights were
trained on a data set with a non-commercial or research-only licence (CULane, TuSimple, LLAMAS, BDD100K).**
Whether weights inherit that is not settled; they would in any case be fetched by a script and not committed. If
one were adopted, the cautious place is roadsense-signs or a fetch-only model with the note in docs/LICENSES.md:
a decision for the owner. The classical analysis has no such question. Nothing was downloaded beyond the three
model files above (246 MB) into `~/datasets/l29/dl`, each in its own directory; no data set.

## 3. Method

One back-end, one display chain for every candidate, so that the detectors are compared and not their
post-processing. A learned detector's output (lane mask, or UFLD's points drawn as lines) is an "evidence
image"; `laned_replay --evidence` feeds it to laned's line follower, tracker and slot logic in place of the
marking filter. Every source then goes through the api's `view` (`tools/laned/display_eval.py views`: the
provider's message of the frame at t arrives at t + 50 ms, 5 messages a second, road stream `vis.13.8` of L24-M)
and is scored on what `view.lines` draws (`score`). Ideal conditions for the learned ones: their real latency
(0.3-2 s) would put them beyond the seam's freshness limit.

Rides: L24-M's nine, the same stretches (5 min, 5 frames a second extracted: `tools/laned/extract_frames.sh`).
**Tuned on** krakow-2024 (day city), warsaw-storm (rain on the screen), gdansk-night-snow (no markings visible),
tunnels-s3 (expressway, tunnels). **Scored on** s7-elblag-lowsun, krakow-night-tunnel, a1-night, warsaw-city,
winter-snow-rural (no markings). Every constant of laned and of the seam was chosen on the tuning rides.

Measures (definitions in the tool's head):

* presence of the ego lane's lines; appear + disappear events per minute, inner and outer lines;
* step of a drawn inner line between two frames at 10 m and 30 m, jumps over 0.3 m;
* **far range**: a drawn point at 20-30 m (40-50 m) carried by the car's motion until it is 5-15 m ahead,
  against the line drawn there then (L24-M's measure, on the drawn lines; motion = mean of both models' pose);
* against supercombo's lines at 10 m after one scale factor per ride: share over 0.5 m (another marking);
* outer lines against L20's lane labels: drawn where the label has no lane ("invented"), missing where it has;
* **paint**: the drawn inner lines against the image, one frame a second: share of the drawn points at 20-40 m
  within 0.25 m of a paint stripe (a marking-width ridge of the luma across the line). The lines are projected
  with the height that belongs to their metres (the model's own for model lines). This measure uses the same
  kind of filter as the classical candidate and therefore favours it; it is also the only one that looks at
  the picture itself;
* no geometric ground truth exists (the labels are lane counts). Hand check: contact sheets of laned's lines on
  the frames of the four tuning rides, by eye, one judge.

Found on the way, and fixed in the tool: the road model's metres are not `calib.json`'s. On tunnels-s3 the
model's lane is 2.78 m wide, the road-plane analysis measures 3.4-3.6 m with the calibrated camera height
(1.5 m); the model never uses the height. Consequences: the seam scales the provider's metres to the model's
(section 5); the comparison takes one scale factor per ride out.

## 4. Results

`a` = seam mode `agree` (default), `r` = `replace` (the source alone). Means over the rides.

Held-out rides with markings (s7-elblag-lowsun, krakow-night-tunnel, a1-night, warsaw-city):

| source | both inner lines, % | flicker inner / outer, per min | step 10 / 30 m, m | jumps at 30 m per min | far 20-30 / 40-50 m, m | other marking, % | outer invented / missing, % (3 labelled rides) | on paint at 6-15 m / 20-40 m, % |
|---|---|---|---|---|---|---|---|---|
| supercombo | 89 | 2.2 / 3.5 | 0.022 / 0.033 | 2.5 | 0.06 / 0.15 | - | 9 / 10 | 60 / 47 |
| new model's own lines (`vis`, today) | 89 | 5.5 / 3.2 | 0.015 / 0.027 | 0.5 | 0.10 / 0.26 | 1 | 8 / 24 | 58 / 43 |
| **classical, `agree` (built, default mode)** | 89 | 5.5 / 3.2 | 0.014 / 0.031 | 1.9 | 0.10 / 0.27 | 1 | 8 / 24 | **64 / 58** |
| classical, `replace` | 80 | 10.5 / 21.8 | 0.024 / 0.036 | 2.9 | 0.10 / 0.27 | 2 | 22 / 48 | 64 / 60 |
| UFLD, `agree` | 89 | 5.5 / 3.2 | 0.015 / 0.029 | 1.0 | 0.12 / 0.30 | 1 | 8 / 24 | 57 / 50 |
| UFLD, `replace` | 82 | 9.5 / 13.7 | 0.024 / 0.033 | 1.9 | 0.14 / 0.30 | 3 | 7 / 48 | 55 / 51 |
| YOLOP-320, `agree` | 89 | 5.5 / 3.2 | 0.016 / 0.035 | 1.6 | 0.15 / 0.38 | 1 | 8 / 24 | 50 / 38 |
| YOLOP-320, `replace` | 39 | 21.0 / 21.3 | 0.027 / 0.030 | 1.8 | 0.36 / 0.90 | 11 | 23 / 70 | 60 / 48 |

Per held-out ride, new model's lines against classical `agree` (far 20-30 / 40-50 m; on paint at 20-40 m; jumps
at 30 m per minute):

| ride | `vis` | classical `agree` | supercombo |
|---|---|---|---|
| a1-night (night motorway) | 0.13 / 0.30 m; 17 %; 0.0 | **0.06 / 0.14 m; 68 %; 0.0** | 0.03 / 0.08 m; 33 % |
| krakow-night-tunnel (night expressway) | 0.17 / 0.42 m; 37 %; 1.4 | **0.12 / 0.30 m; 50 %**; 6.1 | 0.08 / 0.21 m; 43 % |
| s7-elblag-lowsun (day expressway, low sun) | **0.03 / 0.06 m**; 67 %; 0.5 | 0.05 / 0.14 m; 66 %; 0.5 | 0.03 / 0.06 m; 64 % |
| warsaw-city (day city) | **0.09 / 0.25 m; 50 %; 0.0** | 0.16 / 0.51 m; 46 %; 1.1 | 0.08 / 0.24 m; 49 % |

No markings (the source alone must draw nothing; ego lane's left / right line drawn, % of the frames):

| ride | supercombo | `vis` | any source in `agree` | classical `replace` | UFLD `replace` | YOLOP-320 `replace` | TwinLiteNet `replace` |
|---|---|---|---|---|---|---|---|
| winter-snow-rural (held out) | 0 / 0 | 0 / 0 | 0 / 0 | 41 / 24 | 68 / 64 | 59 / 79 | - |
| gdansk-night-snow (tuning) | 41 / 0 | 27 / 0 | 27 / 0 | 34 / 22 | 59 / 61 | 30 / 49 | 2 % both |

Tuning rides (in-sample for the classical candidate and the seam; all four candidates):

| source | flicker inner / outer | step 10 / 30 m | far 20-30 / 40-50 m | other marking, % | on paint 6-15 / 20-40 m, % |
|---|---|---|---|---|---|
| supercombo | 3.3 / 1.2 | 0.016 / 0.026 | 0.14 / 0.32 | - | 43 / 25 |
| `vis` | 4.3 / 0.9 | 0.016 / 0.029 | 0.23 / 0.58 | 2 | 43 / 29 |
| classical `agree` / `replace` | 4.3 / 0.9; 18.3 / 17.6 | 0.018 / 0.037; 0.023 / 0.039 | 0.24 / 0.61; 0.40 / 0.85 | 1; 14 | 46 / 34; 52 / 40 |
| UFLD `agree` / `replace` | 4.3 / 0.9; 16.5 / 5.9 | 0.019 / 0.036; 0.025 / 0.039 | 0.24 / 0.66; 0.37 / 0.90 | 2; 12 | 44 / 30; 42 / 32 |
| YOLOP-320 `agree` / `replace` | 4.3 / 0.9; 23.4 / 24.9 | 0.018 / 0.035; 0.031 / 0.032 | 0.24 / 0.62; 0.44 / 1.42 | 2; 17 | 43 / 29; 44 / 33 |
| TwinLiteNet `agree` / `replace` | 4.3 / 0.9; 11.9 / 10.8 | 0.018 / 0.032; 0.019 / 0.037 | 0.24 / 0.60; 0.33 / 0.83 | 2; 7 | 48 / 32; 56 / 45 |

Reading, plainly:

* **Alone, every candidate is worse than the model's own lines.** Lines come and go two to five times as often
  (a detector that reports paint loses the line at every junction, behind every vehicle and on worn paint; the
  road model draws the lane through), they are on another marking in 2-17 % of the frames, the far range in
  town and rain is off by more, and on snow each of them draws lines on wheel tracks and snow edges where the
  road model draws none. "Absent rather than invented" is what the road model does better than any of them.
* **Among the candidates** the classical analysis is equal or better than UFLD and YOLOP-320 on the far range,
  on the paint measure and on snow, and worse than UFLD on outer-line flicker; TwinLiteNet is the best of the
  learned ones on the tuning rides (the only one that stays nearly silent on the night snow) and still worse
  than the model's lines; it takes 250-690 ms of a laptop core. None of the learned ones justifies 0.35-2 s of
  a little core per frame.
* **In `agree`** presence, flicker and the outer lines are the road model's by construction. What changes is
  the geometry of the lines both see: on paint at range 43 -> 58 %; far range better at night on the motorway
  and expressway, worse in the day city and in low sun, the same on average; jumps at 30 m 0.5 -> 1.9 a minute
  (a slot changes its source). The weakness L24-M found in the new model (far range 1.6-1.9 times supercombo's,
  more presence flicker) is **not** repaired by this lane: far 0.10 / 0.27 m against supercombo's 0.06 / 0.15 m.
* One more thing was tried and dropped: laned's own extra lines (a line the model does not have) while laned
  is anchored on an agreeing line. On krakow-night-tunnel the outer lines "invented" against the labels went
  from 24 to 61 % (kerb and tunnel wall), so `display_agree_extra` is off.
* A second design was prototyped offline and not built: search the paint next to the model's own lines
  (projected with the model's 3D) and correct the line by a + b x. It put the lines on the paint (tunnels-s3: 37
  -> 54 % at range) and made the far-range consistency worse (0.15 -> 0.20 m). The image measure and the
  carry-forward measure do not agree on what "right" is at range; without ground truth this lane cannot say
  which one to trust.

Hand check (by eye, tuning rides): on the expressway and in the tunnels the classical lines sit on the paint
(solid, dashed, and the double edge line as one). In town it finds the dashed lines and also kerbs and the edge
of a hatched area; behind stopped traffic it draws nothing. In the storm ride it drew lines on a kerb, on a
fence base and on the streak of a red light (the rule against lines through the camera's foot point was added
for that). On the night snow it follows wheel tracks. Sheets: `~/datasets/l29/out/*.jpg`.

## 5. What was built

**laned** (`laned/lanefind.{h,cc}`, `laned/main.cc`, `systemd/roadsense-laned.service`, `config/laned.json`;
contract and algorithm: docs/ARCHITECTURE.md "Display lane source"). The daemon follows envd: newest frame of
the ring, `--hz 5`, `--cpus 0-3`, nice 5, `<rundir>/laned.sock`, config file, `--set`, stats on stderr. No
model file, no GPU, no library. `roadsense.target` does not want the unit: it runs only after
`systemctl enable roadsense-laned.service`. Without it nothing changes.

**The seam** (`ViewBuilder._display_lanes`, `api/roadsense_api/view.py`), changed and documented:

| change | why |
|---|---|
| `display_lanes_mode`: `agree` (default), `prefer`, `replace` | section 4: alone, no source is good enough; `replace` is L27's original behaviour |
| the source's metres are scaled to the road model's (`view.lines_scale`, lane-width ratio, 8 s) | the model's metres are not the calibration's (section 3); without it a laned line and a model object do not belong to one picture |
| road edges stay the model's unless the source sends `edges` | laned finds lane lines only; L27's seam dropped the edges with a source |
| a source line continues as the model's beyond its own reach | found on the MFD replay: the ego lane ended at 40 m instead of reaching the lead at 98 m |
| `view.lines_source`: `"model"`, `"laned"`, `"laned+model"` | marked, as asked |
| `display_lanes` is passed on at `/ws` as it came | a consumer can see what the source said |

**Governor.** `laned_hz` 0 is part of the `scenery` step (level 3, before `view` at level 4): display only, shed
early. No renumbering. Paused, laned stops sending and after 0.5 s `view.lines_source` is `"model"` again.
`camerad/governor.h`, `tools/governor/emu.py` and the tables in ARCHITECTURE carry the key.

**The measurement topics do not see it**: `api/tests/test_display_lanes.py` runs the pipeline with and without
a provider that disagrees with the model on everything, and compares `road`, `lead`, `lanes`, `fused`, the
ego-lane filter, and `view.road` / `view.objects` message for message (identical), and the same through
`State.on_message`. The only reader of the message is `ViewBuilder.on_display_lanes`.

End to end on the laptop (`tools/ring_replay.py` as the camera at 5 frames a second, the daemon, a socket
client, a governor file): 5.0 messages a second, 3.5-4.9 ms per frame with the laptop loaded, no message while
`laned_hz` is 0, back at once when the cap goes (`~/datasets/l29/e2e/run.sh`).

## 6. MFD (car repo)

Chain replay as L27 did it: `tools/view_replay.py STREAM --ws WS`, then the car repo's
`tools/scene_feed_replay.py WS --strip 60:6:PNG` (frontend interpreter
`~/projects/cyberpank-prius-gen2-computer/.venv-fe`, system numpy through `PYTHONPATH=~/datasets/l27/mfd/np`).
"Before" = the new default's road stream (13.8 runs/s, 0-180 s); "after" = the same stream with laned's messages
(5 a second, 50 ms late), seam default.

| what the MFD draws | a1-night before | a1-night after | warsaw-city before | warsaw-city after |
|---|---|---|---|---|
| `view.lines_source` | model | laned or laned+model on 99.6 % of the frames | model | laned or laned+model on 74 % |
| left / right line at 10 m, step rms | 0.011 / 0.010 m | 0.006 / 0.006 m | 0.011 / 0.009 m | 0.012 / 0.009 m |
| left / right line at 30 m, step rms; steps over 0.3 m | 0.010 / 0.009 m; 0 | 0.009 / 0.009 m; 0 | 0.028 / 0.020 m; 2 | 0.034 / 0.025 m; 3 |
| scenes published | 9.1/s | 9.1/s | 9.5/s | 9.5/s |

The MFD draws the new lines; the picture is the same picture with slightly moved lines, steadier near the car
on the motorway and a little less steady at 30 m in town. Screenshots (six consecutive drawn frames from 60 s):
`/home/piotr/datasets/l29/mfd/mfd_a1-night_before.png`, `/home/piotr/datasets/l29/mfd/mfd_a1-night_after.png`,
`/home/piotr/datasets/l29/mfd/mfd_warsaw-city_before.png`, `/home/piotr/datasets/l29/mfd/mfd_warsaw-city_after.png`.

Car repo: the MFD code needed no change (the scene director already handles lines of changing length). One
edit, in a tool only, on branch `v2-l29` (worktree `~/projects/wt/prius-l29`, so that `wt/prius-v2` stayed
untouched): `tools/scene_feed_replay.py` indexed the 30 m point of every line and crashed on a line that ends
before 30 m (seen with the first version of the seam; with the final seam the lines are full length again).
`tools/mfd_chain_replay.py` on the recorded car drive 20261117-145921: output identical for `v2` and this branch
(no provider in a recording).

## 7. Cost, and the phone runbook (not run: the phone belonged to L28)

Laptop: 1.4 ms per frame idle, 2.1-5.4 ms per frame over the nine rides with the laptop loaded by other jobs
(one thread; `laned_replay` prints it). Estimate for the phone: 4-16 ms on a little core per frame, at 5 Hz 2-8 %
of one little core (192 of 400 % are in use), no GPU time, no big-core time; the frame copy out of the ring
(1.4 MB, 5 times a second) is part of it. The api's extra work is one line mix per road frame. Watts: not
estimated.
All of this is an estimate until step 2 below.

1. Files: this branch (`laned/`, `camerad/`, `api/`, `systemd/roadsense-laned.service`, `config/laned.json`,
   `tools/laned_replay.cc`) to a scratch directory on the phone, e.g. `~/l29` (`scripts/sync-phone.sh` or rsync).
   No model files.
2. Build laned alone (no dependencies, under a minute): `cmake -S ~/l29/laned -B ~/l29/build-laned && cmake --build ~/l29/build-laned -j4`.
3. Timing next to the running stack, nothing installed (a scratch run directory, so the installed api does not
   see it): `sudo systemd-run --unit l29-laned -p CPUAffinity=0-3 -E ROADSENSE_RUNDIR=/run/l29 ~/l29/build-laned/laned --hz 5 --stats 10 --frames 600`,
   then `journalctl -u l29-laned`: `ms per frame` and the fps; CPU share from `pidstat -p $(pidof laned) 10 6`.
   Repeat with `--hz 10`. Offline on a clip: `~/l29/build-laned/laned_replay ~/rs-replay/ljkr.nv12 --width 1920 --height 1080 --focal F --calib /etc/roadsense/calib.json`.
4. A/B of the budget (L21's method, `tools/budget/run.sh`, 240 s after 60 s, ride `krakow-2024`): profile
   `new-default` of L27 against the same with laned started in the stack's run directory (`ROADSENSE_RUNDIR` of
   `tools/budget/stack.sh`) and this branch's api (`L21_HOME=~/l29`). Compare the rail watts, little-core busy,
   `road` / `lead` / `lanes` rate and latency (must not move), and `view.lines_source` in the tap.
5. On the car, when wanted: install this branch (`scripts/install-phone.sh` installs the binary, the unit and
   `laned.json`; the unit stays disabled), then `sudo systemctl enable --now roadsense-laned.service`; off again:
   `sudo systemctl disable --now roadsense-laned.service`. `view.json` `{"display_lanes_mode": "replace"}` shows
   laned alone for a look at what it sees. The DVR camera's `calib.json` is `pitch 0, height 1.22` from config:
   laned trims pitch and yaw by up to 3.4 degrees itself and the view takes the scale from the model, but a
   pitch further off than that needs a calibration first.

## 8. Tests

| suite | command | result |
|---|---|---|
| roadsense api | `PYTHON=~/projects/roadsense/.venv-models/bin/python sh tools/smoke_api.sh` | 252 passed (242 on `v2`, one seam test replaced by 11 in `api/tests/test_display_lanes.py`) |
| C++ | whole tree, Release, `-DROADSENSE_NCNN=ON` (L27's configuration), build tree `~/datasets/l29/build` | builds, laned and laned_replay without warnings; laned alone: `cmake -S laned` |
| daemon end to end, laptop | `~/datasets/l29/e2e/run.sh` | section 5 |
| car suite (`v2-l29` of the car repo = `v2` + the tool fix) | `PRIUS_ROADSENSE_REPO=<this worktree> SDL_VIDEODRIVER=dummy .venv-fe/bin/python -m pytest -q tests` | 1697 passed, 19 skipped, 5 failed: the five BEAN tests of `tests/test_bean_rx.py` / `test_bean_tx.py` that read `tests/fixtures/bean_nhw20_frames.txt.gz`, a file that is untracked in `wt/prius-v2` and therefore absent in a fresh worktree (FileNotFoundError); no roadsense code |
| MFD chain replay, recorded car drive | section 6 | identical to `v2` |
| MFD scene feed replay with laned | section 6 | draws; screenshots there |

## 9. Reproduce

```
sh tools/laned/extract_frames.sh RIDE START_S DUR_S                       # 5 frames a second, NV12, to $L29_FRAMES
laned_replay $L29_FRAMES/RIDE.nv12 --width 1280 --height 720 --focal F --calib RIDE/calib.json --frame0 N --step 4 > DL.jsonl
python tools/laned/candidate_evidence.py ufld|yolop320|twinlite RIDE      # evidence maps of a learned candidate
laned_replay ... --evidence $L29_EVID/RIDE.CAND.u8 640 360 > DL.jsonl     # the same back-end on them
python tools/laned/display_eval.py views ROAD/RIDE.vis.13.8.jsonl.gz V.npz [--laned DL.jsonl] [--view-set display_lanes_mode='"replace"']
python tools/laned/display_eval.py score sc=SC.npz vis=VIS.npz cl=V.npz --ego SC.npz,VIS.npz --ref sc \
       [--labels LANES.csv --source RIDE] --paint $L29_FRAMES/RIDE.nv12 --frame0 N --focal F --calib RIDE/calib.json --trim-for cl=DL.jsonl
```

The drivers that ran the tables (`~/datasets/l29/work/*.sh`, results `~/datasets/l29/res/*.json`) are not in the
repo; they only loop these commands over the rides.

## 10. Limits, and what is open

* No geometric ground truth for lane lines; one judge for the hand check; 5-minute stretches; the provider ran
  at 5 frames a second from extracted frames with an assumed 50 ms latency.
* The rides' `calib.json` are estimates (+-10 %); laned's trims and the view's scale absorb most of it, the car's
  camera has not been seen by laned at all (its recordings hold one frame a second: too few for the tracker).
* The classical analysis is a quadratic per line: wrong in sharp bends (in `agree` such a line stops agreeing
  and the model's is drawn). It cannot tell paint from a snow rut or a tram rail; that is why it does not decide
  alone.
* Phone: nothing measured (section 7).
* Not done: 10 Hz evaluation; chroma (yellow lines, red streaks); UFLDv2, CLRNet, TwinLiteNet+ (table of
  section 2).
