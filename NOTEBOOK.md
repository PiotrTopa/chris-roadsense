# roadsense notebook

## Fold 0 (2026-10-23): baseline
Facts (measured on the phone, DVR camera behind the hub, see docs/PHONE_DVR_DEPLOY.md):
1. camerad-dvr 19.5 fps into the ring, ~3 % of a core.
2. modeld on the Adreno 630 (ncnn/Vulkan, fp16-packed, `TU_DEBUG=flushall`): 71-79 ms per run,
   9.3-11.5 runs/s (12/s at its best on 2026-10-21), die 63-70 C without signd.
3. signd on 4 big cores (ONNX Runtime, MTSD YOLOv8n 384x640 + 64x64 classifier): 170-200 ms,
   4.8-5.6 fps; drives the die to a ~95 C plateau without a fan, ~86 C with the user's
   improved cooling (23:13 CEST).
Open question that opens L1: can signs move to the GPU, or is the GPU already saturated by modeld?

## Lanes
| Lane | Model | State | Report |
|---|---|---|---|
| L1 signs-vulkan | Fable | done (2026-10-24 ~01:00 CEST) | docs/lanes/L1-signs-vulkan.md |
| L1-soak | Opus | superseded by L3 (speed ruling) | orders/L1-soak.md |
| L2 envd (environment/weather analyser, 1 fps) | Fable (user ruling 2026-10-26) | done (2026-10-26 ~18:20 CET) | docs/lanes/L2-envd.md |
| L2-envd-phone | Opus | folded into L3 | orders/L2-envd-phone.md |
| L3 speed-tune + envd on phone | Opus | done (2026-10-26 21:40 CET) | docs/lanes/L3-speed.md |
| L4 GPU turn-taking (signd/modeld) | Opus | done (2026-10-27 00:28 CET) | docs/lanes/L4-gpu-sched.md |
| L6 speed-limit state from sign tracks | Opus | done (2026-10-27 ~07:15 CET) | docs/lanes/L6-speed-limit.md |
| L7 traffic lights (detector + state) | Opus | done (2026-10-27 ~08:20 CET) | docs/lanes/L7-traffic-lights.md |
| L8 sign false positives (lamps as no-entry) + negative training | Opus | done (2026-10-27 ~12:30 CET) | docs/lanes/L8-sign-fp.md |
| L9 Polish ride library (diverse open recordings) | Opus | done (2026-10-27 12:55 CET) | docs/lanes/L9-ride-library.md |
| L10 ride playlist (all rides in a loop on the phone) | Opus | done (2026-10-27 14:12 CET) | docs/lanes/L10-playlist.md |
| L11 signs in effect (rules engine + panel) | Fable (user ruling 2026-10-27) | done (2026-10-27 18:45 CET) | docs/lanes/L11-signs-in-effect.md |
| L12 MTSD download + sign classifier retraining (negatives) | Opus | done (2026-10-27 23:20 CET) | docs/lanes/L12-mtsd-retrain.md |
| L5 ride replay (10-min dashcam ride as camera source) | Opus | done; replay live on the phone since 23:16 CET (2514b6c) | docs/lanes/L5-ride-replay.md |

## Fold 1 (2026-10-24): signs on the GPU (L1, Fable)
- [FAV] signd's detector runs on the Adreno 630 (ncnn/Vulkan) next to modeld: 5.0 signs/s at
  9.7-9.9 road/s with the die at 67 C (shipped unit, `--max-hz 5`), vs 5.6 signs/s at 94 C and
  rising on the big cores (ORT). Uncapped: 8.7 signs/s and 8.7 road/s at 72 C.
- [FAV] Accuracy: all 21 detections on 3 Polish photos + 60 Ljubljana frames identical to ORT fp32
  (boxes within 2 px, scores within 0.01); deterministic with `TU_DEBUG=flushall`.
- [UNFAV] The classifier is slower on the GPU (10-15 ms/crop vs 3 ms on CPU), so it stays on ORT;
  ncnn computes the YOLOv8 head's final Concat wrongly (worked around with separate outputs).
- [INSTR] GPU headroom measured from the devfreq tracepoint: modeld alone uses 56 % of the GPU
  (~48 ms GPU of each ~76 ms run); capping modeld to 10 Hz frees little (devfreq drops the clock).
- [INSTR] Side finding: modeld ran 14-15 runs/s when the little cores were busy, because devfreq held
  the GPU at 710 MHz; a GPU min-frequency pin may give modeld about +20 %.
Facts: 4. signd det-ncnn + cls-ORT, 2 threads, cpus 4-7: numbers above (lane report, 70 s runs).
5. modeld GPU busy 56 % uncapped, 51 % at --max-hz 10.
Changes: roadsense-signs c16d572 (ncnn runner, --max-hz, conversion tools, unit on the GPU split);
roadsense 753d2ae (lane report, soak order). Phone: signd binary, ncnn models and unit reinstalled,
signd left stopped.
Roadmap: next L1-soak (Opus, 15-min plateaus of the shipped and uncapped configs, does modeld's
80 C throttle ever fire); candidate L2 = GPU min-frequency pin for modeld; then mount/calibration.

## Fold 2 (2026-10-26): environment analyser envd (L2, Fable)
- [FAV] envd works with zero-shot CLIP ViT-B/32 (OpenAI weights, MIT) + a linear probe: held-out macro
  recall weather 78.9 % (BDD), light 86.2 %, place 74.0 % (BDD) / 73 % (Places365 proxy incl.
  underground parking 98 %, forest road 83 %). Smoothed labels: 0-2 extra flips at 20 % wrong frames,
  5-6 s to follow a real change.
- [UNFAV] Zero-shot alone is not enough (BDD weather macro 56.9 %, place 58.3 %; highway 35 %,
  overcast 35 %, snow 30 %); fog is barely testable on BDD (13 images; ~10 %). The probe is trained on
  research-licensed data (BDD100K, DAWN CC BY-NC, Places365), so it is not committed; the repo ships
  the zero-shot class file.
- [INSTR] Encoder choice by pre-registered zero-shot rule: ViT-B/32 openai 71.1 > SigLIP2 68.1 >
  TinyCLIP-40M 67.0 > ViT-B/32 laion2b 65.4; MobileCLIP excluded (Apple research-only). C++ path
  bit-exact to Python (cos 1.00000), smoother identical on 360/360 frames. Laptop cost 89 ms/frame at
  2 threads. ncnn conversion done but unverified (x86 ncnn wheel segfaults), so ORT on the little cores
  is the default.
Deviations: two disclosed in the lane report (anchored probe variant; extra datasets split A/B after a
BDD-only probe could not learn the Places-only classes). The pre-registration section was first
committed together with results (2c03711), so its precedence is the lane's statement, not a git fact.
Not measured: phone cost (the user's stress test ran the whole lane); order L2-envd-phone.
Changes: roadsense 2c03711..460b5ef (envd/, tools, api env topic + /v1/env, unit, docs; 11 api tests pass).
Roadmap: L2-envd-phone (Opus: build, install with the probe class file, cost rows, 30-min run) once
the stress test ends; L1-soak still open; candidates: GPU min-clock pin, boost auto-recovery, mount
and calibration, in-car recordings as own training data for envd.

## Fold 3 (2026-10-26): speed profile + envd live on the phone (L3, Opus)
- [FAV] Road rate 9.6 -> 13.2 runs/s (15-min run, 13.0-13.3 in every window), model 95 -> 74 ms,
  frame-to-client latency 185 -> 144 ms median; signs unchanged at 5.0/s (detect 107 -> 81 ms);
  envd live at 1 Hz, 323 ms per frame on the big cores. Die 61-64 C after the first 4 min (max 82 C).
  Verified live 21:39 CET: road 12.7/s, signs 4.6/s, env connected.
- [FAV] The lever was CPU clocks, not the GPU: modeld's main thread on the big cores, little and big
  cores pinned at max (big-core 2.09 GHz cap removed); GPU min-freq pin and throttle-temp had no
  measurable effect (kept as guard / raised to 95 C).
- [UNFAV] The phone hard-reset by itself twice under high load (20:14 and ~21:03 CET; also 16:38 in the
  user's stress test). No oops, no thermal event, temps < 66 C, boost healthy afterwards; suspected
  supply brown-out (battery eliminator + VBUS boost feeding hub and camera; peaks >= 2.8 A with the
  speed profile). Not proven. Fallback: drop the big-core pin (12.6 road/s).
- [UNFAV] envd on ncnn/Vulkan hung the GPU (kernel hangcheck), leaving modeld failing every run until
  restart; modeld now exits after 20 consecutive failures (built, not yet exercised by a real hang).
  envd at 2 Hz heats 60 -> 88 C for no gain.
- [INSTR] Signs cost ~3.8 road runs/s (16.3/s without signd): modeld runs take ~105 ms when they overlap
  a detector run on the GPU (bimodal). Per-stage road latency: camera ~38 ms, prep 11 ms, model 56 ms
  median, ~20 ms queue/JSON, api +4 ms.
- [INSTR] envd live on a dark bench: weather clear, light night; place flips between underground
  parking and gas station (known probe confusion, conf ~0.4-0.6).
Changes: roadsense 0e531ef (units, envd.json, modeld lost-device exit, report), roadsense-signs
cb9087e (comment). Phone: speed profile installed and running; autostart disabled; old modeld and api
kept as backups.
Roadmap / candidates: (a) supply check or soak for the resets (user hardware: eliminator rating,
wire gauge, powered hub); (b) code lane: interleave/serialise signd and modeld on the GPU to win back
part of the 3.8 road/s; (c) envd place smoothing or a gas-station/underground-parking disambiguation;
(d) ledlat glass-to-API; (e) boost auto-recovery; (f) mount + calibration; (g) own recordings.

### Fold 3 correction (2026-10-26, user)
The resets at 16:38, 20:14 and ~21:03 CET were caused by the user's own hardware testing, not by the
stack or a supply brown-out. The [UNFAV] reset item of fold 3 is withdrawn; roadmap item (a) is dropped;
the big-core pin stays.

## Fold 4 (2026-10-26): ride replay source, phase 1 (L5, Opus)
- [FAV] camerad-replay plays a 10-min Krakow city drive (Wikimedia Commons, CC BY 3.0, Relaxing Roads 4K)
  as a camera: 20.0 fps steady, 2 ms decode, loops with a 1 s seam; supercombo overlays plausible on
  the laptop (lines on markings, lead at 45 m on a bridge; weak lanes in dense junctions).
  One-command switch: scripts/use-replay.sh --enable FILE | --disable.
- [UNFAV] City only: no open Polish rural/forest/highway clip found (fallbacks: German mixed 57 min,
  Slovenian motorway 12 min). Intrinsics are estimates (focal 690 px, pitch -0.085, height 1.1 m).
- [INSTR] Bug fixed in camerad/ring.h: readers never noticed a replaced ring at their 200 ms timeout,
  so modeld/envd stayed on a dead ring after any camerad restart; now a 500 ms re-check timer. The
  phone's installed daemons predate the fix (use-replay.sh restarts them).
Deviation: the lane stopped a download with `pkill -x curl -u piotr` (broader than its PID).

### Fold 4 addendum (2026-10-26 23:20 CET): replay live on the phone
- [FAV] Phone runs the Krakow ride as its camera: 20.0 fps, 0 drops; road 12.6-13.1/s, signs 4.5-5.0/s
  with real tracks (yield, priority road), envd city_street/overcast/dusk; overlays on the phone match
  the markings (lead 11.8 m, 89 m). use-replay.sh worked on its first run.
- [INSTR] Sign boxes trail the moving scene by ~40 px (~100 ms of sign latency at ~10 m/s), visible in
  the preview; envd's place label wanders (city_street vs highway on wide arterials).

## Fold 5 (2026-10-27): GPU turn-taking + pipelined modeld (L4, Opus)
- [FAV] Road 13.0 -> 14.05 runs/s (+7.8 %, 15 min on the replay, 89/90 windows at 14.0-14.1) at 5.0
  signs/s; model mean/p90 74/107 -> 53/55 ms (no longer bimodal); road latency frame->ws median/p90
  121/168 -> 107/126 ms. Verified live 00:3x CET: road 14.0/s, model 54.8 ms, 62 C.
- [INSTR] Gains by step: turn-taking alone ~neutral (13.26); signd records its detector before taking the
  turn 13.57; two-stage (pipelined) modeld 13.68; CPU wake-latency request (--cpu-latency-us 0) 14.05.
  Free-running pipelined without turn-taking ties on rate (14.17) but has p90 103 ms; turn-taking kept
  for latency. GPU busy 92-93 %; estimated ceiling ~15.9 road/s at 5 signs/s.
- [INSTR] Contract: <rundir>/gpu.sched (camerad/gpusched.h, docs/ARCHITECTURE.md "GPU schedule"),
  every wait bounded; each daemon works alone; signd kill -9 x3 harmless. Outputs bit-identical
  (modeld 39/39 runs, signd 14/14 detections).
- [UNFAV] Sign latency +16 ms median (signd waits for its turn). --cpu-latency-us 0 costs idle power in
  the car. Remaining lever small: ~2 ms/run of CPU-only ncnn layers in the policy stage (<= +0.5 road/s).
Deviation/incident: road stream down 13 min (22:27-22:40 CET) from a bad test drop-in (pointed at the build
directory); script now verifies modeld after each switch. Empty drop-in dirs left in /run (harmless).
Changes: roadsense d9ace97, roadsense-signs 26183d1; phone runs the new binaries from installed units
(old ones kept as *.pre-l4); replay still the camera source; autostart disabled.
Roadmap: idle-power policy for --cpu-latency-us in the car; policy-stage op conversion (small);
mount + calibration; envd place stability; own recordings; boost auto-recovery.

## Fold 6 (2026-10-27): speed-limit state (L6, Opus)
- [FAV] api serves `speed_limit` (/v1/speed_limit, /ws, health, status page): limit, source (sign track or
  default rule), confidence, since (time + distance from v_ego), expiry rule, area, zone, pending/ignored.
  Polish rule table from primary texts (PoRD art. 20; sign regulation Dz.U. 2019 poz. 2310). 34/34 api
  tests; 42 us per message. Live on the phone (verified 07:1x CET: 50 km/h, built-up from env).
- [FAV] Krakow loop: ~90 % agreement with a human reading (98 % with the area known); B-33 40 on the
  bridge caught every pass; false D-9 / D-40 rejected; never served a value above the human reading.
- [UNFAV] Not observable by the camera: D-42 built-up entry and B-43 zones (not in the 400 classes; area from
  envd at confidence 0.45, zone guessed for <= 30 km/h), plates (served unconditional, flagged), junctions
  (proxies: junction signs, >= 50 deg turn, distance cap; ends limits ~5-7 s late). B-33 70 missed in 2 of 3
  passes (detector recall: signd sees every 4th frame, small sign). Untested off-city (no open clip).
  Road-type gaps: dual carriageway outside town served 90 (law 100: safe side); single-carriageway
  expressway served 120 (law 100: ABOVE the legal limit; the lane called both "low side", which is wrong for
  this one; open fix).
- [INSTR] Design: pure state module in the api (also runs offline: tools/speedlimit_replay.py); sign
  acceptance needs >= 5 hits, vote >= 0.6, mean score >= 0.45, lateral position checks.
Deviation: two api restarts on the phone instead of one (env prior fix). L7 later installed its api build
containing the same speedlimit.py.
Changes: roadsense 723d9f9..794162d.

## Fold 7 (2026-10-27): traffic lights (L7, Opus)
- [FAV] Lights live: `signd --lights` (2nd thread, lightd.sock, AGPL repo) with BDD100K YOLO11n on a 960x320
  band of the upper frame, ncnn CPU on the big cores at 3 Hz, state CNN per crop; api picks the ego light
  from modeld's path (/v1/lights, /ws, overlay, health). 15 min: road 14.04 -> 13.70/s (-2.4 %), signs 4.99,
  lights 3.0/s, frame->ws 139/168 ms; die 62-72 C (one 84 C). Verified live 08:2x CET.
- [FAV] Accuracy: detector AP50 61.1 on held-out BDD val (C++ on the phone 61.9 = Python); state CNN recall
  red 87 / yellow 61 / green 96 (macro 81.3 vs Autoware 74.9, colour rule 63.7). Krakow loop: ego light right
  at all three junctions, one red -> red+yellow -> green sequence captured.
- [UNFAV] Weights are research-licensed (BDD100K) and AGPL (Ultralytics): fetched locally, never committed.
  Yellow recall weak; night, flashing yellow, green arrows unverified; DVR camera untested; ROI assumes a
  centred horizon; ego choice ignores lane arrows and turn intent.
- [UNFAV] GPU placement is a null (detector needs ~100 ms per turn; modeld reclaims after 80 ms: road 9.95/s).
- [INSTR] Fixed a status-page JavaScript syntax error (raw newline from join("\n") in server.py).
Permission denial (surfaced to the user verbatim): loading the downloaded dronefreak/bdd100k-yolov8n .pt
was refused by the auto-mode classifier ("Code from External"); file deleted unloaded; that GPU-friendly
variant stays unevaluated.
Deviation: one laptop job briefly ran ~16 threads (cap 8) for ~1 min.
Changes: roadsense 60914fb, roadsense-signs 046642a; phone runs signd with lights, old binary *.pre-l7.
Roadmap: expressway default fix (fold 6), lights as junction cue for speed limits, own recordings
(yellow/red+yellow, night), mount + calibration (also sets the lights ROI from pitch).

### Note (2026-10-27, user report -> L8)
User: "no entry" often fires on the lead car's stop lamps. Coordinator check of the phone's sign history:
9/100 tracks were no-entry; of 6 inspected crops 3 real (score 0.86-0.88), 2 bus/tram brake lamps (0.50-0.55,
up to 123 hits), 1 red traffic light (0.59). User ruling: do all fixes (class threshold, overlap with lights /
lead vehicle, glow check) and look for better data for negative training.

## Fold 8 (2026-10-27): sign false positives (L8, Opus)
- [FAV] False no-entry tracks on the Krakow replay 10.2-10.65 per loop -> 0.00 (installed config, 2.1 loops);
  no-entry precision 0.23 -> 1.00 with 6/6 real kept; all 65/65 real round red signs kept; speed-limit
  timeline unchanged; phone cost nil (road 13.4, detect 100 ms). Verified live 12:3x CET.
- [FAV] Main fix: sign verifier CNN (~24k params, 32x32 crop) trained on BDD100K lamp/red-light blobs (neg) and
  GTSRB (pos), no ride data: rejects 100 % of ride lamps/lights; keeps 99.9 % GTSRB test signs.
  Plus `--class-min-score regulatory--no-entry=0.71` and api context flags gated on the verifier.
- [INSTR] Flagged tracks are kept with `suppressed`/`reason` (suppressed_tracks in /v1/signs, grey boxes in
  /preview, ignored reasons in speed_limit); documented in ARCHITECTURE "Flagged tracks".
- [UNFAV] Nulls: glow check alone (flags 14.8 % of sunlit real signs; now only with an unsure verifier); light
  overlap alone (27 %); colour check for wrong-class signs; hard-negative classifier retraining impossible
  (MTSD needs an account). Wrong-class real signs remain (blue keep-right as no-entry in some passes).
  Only overcast city daylight tested; 0.71 threshold may be tight on other footage (half of BDD US
  do-not-enter detections score < 0.71).
Deviations: verifier threshold 0.3 and glow gating tuned on the same ride they were scored on (in-sample,
disclosed); pre-registration committed 10:49, the "before" phone run started 10:42.
Data: Krakow (CC BY 3.0), BDD100K val (research), GTSRB via Zenodo 13741936 (CC BY 4.0 on the record). Verifier
weights research-licensed, not committed.
Changes: roadsense f64911b, d6ebdc3, 4b8fb31, 7ecfcf6; roadsense-signs 7ba2247, c07af24.
Next: validate on L9's diverse Polish rides (night/rain/glare/motorway) before trusting the thresholds.

## Fold 9 (2026-10-27): Polish ride library (L9, Opus)
- [FAV] 21 openly licensed Polish rides, 4 h 06 min (docs/RIDES.md): rural with area signs, forest, snow,
  fog, A1 at night (also in fog), A1/S8 rain, S7 low sun, S1 roadworks, five tunnels (S3, POW Warsaw, Gdansk,
  Swinoujscie, DTS), night snow, night expressway, rainy tram city, thunderstorm, arrow signals. All on the
  laptop (~28 GB) and the phone (~/rides, 19 GB, md5 verified, 75 GB free). Laptop overlays plausible.
- [UNFAV] Gaps: underground/multi-storey parking, flashing yellow, daylight falling snow, night rain, level
  crossings, zone signs. Intrinsics mostly guessed (one measured: S7 focal ~900 px). Some rides have overlays
  (speed bar, captions, dashboard in the bottom 30 %).
- [INSTR] Licences: 20 YouTube CC BY (verified via yt-dlp's licence field 2026-10-27, attribution in each
  SOURCE.txt); the Krakow YouTube original now shows a standard licence, the Commons copy (CC BY, reviewed
  2025) remains valid. Downloads via yt-dlp in a private venv (YouTube's terms of service restrict
  downloading even for CC content; private test use only, nothing redistributed).
Changes: roadsense ca7efd0, 00775ab.
Next: sweep the stack (signs verifier + thresholds, lights, speed limit, envd) across the library; MTSD from
the user would enable classifier retraining.

## Fold 10 (2026-10-27): ride playlist live (L10, Opus)
- [FAV] The phone loops all 21 rides (~4 h 08 min per loop) with per-ride intrinsics (new ring per ride) and
  per-ride calibration (camerad-replay writes calib.json; modeld already reloads it every 2 s; 2.5 s pause
  at the change). No modeld restart; first road message 0.13 s into a ride; road 13.3-13.8/s, signs 5.0,
  lights 3.0 across changes. Dashboard/preview show ride, author, licence. Verified live 14:1x CET:
  forest-lubelskie, forest_road/clear/day, limit 90, road 12.8/s.
- [INSTR] The api resets driving state (speed limit, odometer, live tracks) on a ride change; readers treat
  the frame-number restart as a camera restart. envd on the phone predated the ring fix and was rebuilt.
- [UNFAV] Single temperature readings 94.9-95.5 C around 13:44 (not sustained; 66-69 C later).
Correction to the lane report: the Wi-Fi outage began at 13:40:00 (router deauth GROUP_KEY_HANDSHAKE_TIMEOUT
after a TKIP group rekey, ath10k key install -110), not 14:01; recovered 14:07 (CMDB poco.md).
Changes: roadsense bfc7c5f, d68dc22. Phone rollback copies envd.pre-l10, api.pre-l10.
Open: Wi-Fi watchdog on the phone / router WPA2-AES only; stack sweep across the library.

## Fold 11 (2026-10-27): signs in effect (L11, Fable)
- [FAV] api serves `rules` (/v1/rules, /ws, health, status panel, /preview HUD): IN EFFECT / RECENT / AHEAD from a
  data-driven Polish table (62 entries with legal sources; guesses marked) wrapping the unchanged speed-limit
  engine. 69/69 api tests (verified). Live on the phone.
- [FAV] Six labelled rides (labels committed before scoring): time agreement no parking 0.99, weight limit 0.98,
  no overtaking 0.93, no stopping 0.82, one-way 0.81, priority road 0.79. Starts median -0.3 s from the sign,
  ends +1.6 s. Traffic-light junction cue: junctions found 82 -> 98 of 178, end delay 11.8 -> 3.6 s (+2 false).
- [UNFAV] Built-up area 0.44 (D-42 not a class; envd never voted built-up in villages); expressway 0.15 (far D-7
  never tracked); misses mostly detector (30 of 35); false rules mostly classifier confusions (warning triangles
  read as D-1, C-5 as D-3) and signs for the other carriageway; night lights give false junctions on motorways.
- [UNFAV] Shipped a status-page JS bug again (14:55-18:42, every box "..."); fixed + regression test scanning the
  page script. Legal correction: B-42 does not end B-35/B-36 (par. 32 ust. 1).
- [INSTR] api restart with a /ws client used to take 90 s; now 1 s.
Deviations: two D-1 inferences added after the first scoring of two evaluation rides (in-sample there; pre-registered
numbers kept, switchable); a >= 2-frame rule added after a non-evaluation ride; three api restarts on the phone.
Single labeller per ride.
Changes: roadsense 3ca7c62..6e70f22.
Next candidates: verifier for yellow/blue classes (D-1/D-3 confusions), road-width-gated lateral window for
B-36, envd built-up prior on own rides, reuse the six-ride labels for any detector change.

## Fold 12 (2026-10-27): MTSD-retrained classifier + tracker fix (L12, Opus)
- [FAV] Deployed cls1trk (only variant passing every pre-registered rule): YOLOv8n-cls trained from scratch on
  MTSD (196k crops) + other-sign + background classes (BDD lamps/lights, detector false positives), plus
  `--dist-match-new 2.0` in the tracker. Six rides: false rules 51 -> 47, misses 76 -> 74, momentary/warning
  hits 266 -> 291, priority-road agreement 0.807 -> 0.913; Krakow lamps/lights 0/33 accepted; B-33 70 served
  4/4 phases (was 3), s1 D-7 tracked 2/2 (was 0). Held-out: top-1 0.859 -> 0.939, lamps read as signs
  0.66/0.78 -> 0.009/0.001, triangles -> D-1 19 -> 0. Phone cost unchanged (road 13.6/s, signs 5.0/s).
- [INSTR] The named recall misses were tracker misses (small edge signs jump 1.1-1.6 box widths between 5 Hz
  frames); detector not retrained. Background-voted tracks flagged `suppressed: not_a_sign`.
- [UNFAV] no_overtaking agreement 0.932 -> 0.912 (exactly at the 0.020 limit); cls1 sometimes abstains
  (`traffic-sign`) on Polish signs (a missed B-42 let a B-25 run to its 3 km cap); detector still misses
  signs under ~15 px; single recording per ride, single labeller.
Deviations: tracker value 2.0 chosen from the B-33 70 geometry (in-sample for that case, addendum committed
before scoring); training data reduced after OOM (before scoring); disk floor breached briefly (26 GB) before
the image zips were deleted; D-7 scored from the speed-limit timeline (rules_replay cannot see road class).
Data: MTSD fully annotated train+val (CC BY-NC-SA; weights local only, never committed); zips deleted after
crop extraction (links valid to 2026-11-26). No checkpoint loaded; no permission denial.
Live check 23:2x CET: deployed unit verified; big cores 91-93 C, envd self-pauses above 92 C (32 pauses).
Changes: roadsense 492a673..327ede4; roadsense-signs 4e1938e, 376f841, c732a02.
