# L6 speed-limit: the limit that applies to the car, from sign tracks

Lane opened 2026-10-27 (Opus, build). Goal: the api serves the CURRENT speed limit, derived from the
documented `signs` contract only (MIT core; no signd internals), Polish rules as the default rule set.
Times CET (Warsaw local, UTC+1).

## Result

* Topic `speed_limit` (on `/ws`, on every change and at least 1/s), endpoint `/v1/speed_limit`
  (adds `recent_changes`, `ignored`), summary in `/v1/health`, a "speed limit" block on the status
  page. Schema: docs/ARCHITECTURE.md "api-derived topic".
* Code: `api/roadsense_api/speedlimit.py` (pure state machine, no I/O), rules as data in
  `api/roadsense_api/rules/pl.json` (`--rules` / `ROADSENSE_RULES` picks another country file),
  wiring in `state.py` / `server.py`, offline runner `tools/speedlimit_replay.py`.
* Tests: 21 unit tests on synthetic Polish sequences (`api/tests/test_speedlimit.py`) + one api
  end-to-end test; whole suite 34/34 (`PYTHON=.venv-models/bin/python tools/smoke_api.sh`).
* Krakow replay: agrees with a human reading of the preview for ~90 % (with env) to ~98 % (area known) of the loop; the misses come
  from the detector (one 70 sign found in 1 of 2 passes) and from the junction proxy being late
  (+5 s), see "Replay".
* Cost: 42 us per message incl. a snapshot (laptop, 13 212 recorded messages); on the phone road stayed
  at 14.0/s after the install.

### Why in the api and not a separate daemon
The api already receives road, signs and env and does the fan-out; a daemon would need its own socket
clients, a socket, a unit and an api client for one more topic. The state machine is a pure module, so
the same code runs offline (`tools/speedlimit_replay.py`) and in tests. If another consumer ever needs
it without the api, the module can be wrapped in a daemon unchanged.

## Polish rules (sources)
Primary texts, downloaded from the Sejm ELI API (PDF) and grepped:

* **PoRD** = Prawo o ruchu drogowym, consolidated text Dz.U. 2024 poz. 1251
  (https://api.sejm.gov.pl/eli/acts/DU/2024/1251/text/O/D20241251.pdf). Later amendments exist
  (ELI change date 2026-10-12); art. 20 was checked in this consolidated text only.
  * art. 20 ust. 1: built-up area **50 km/h**; ust. 1a (60 km/h at 23-5) **repealed** (by the 2021
    amendment, in force 1 June 2021), so 50 day and night.
  * art. 20 ust. 2: residential zone (strefa zamieszkania) **20 km/h**.
  * art. 20 ust. 3 pkt 1 (passenger car, motorcycle, truck <= 3.5 t) outside built-up area:
    motorway **140**, dual-carriageway expressway **120**, single-carriageway expressway and a dual
    carriageway with >= 2 lanes per direction **100**, other roads **90**.
  * art. 2 pkt 10: an intersection (skrzyżowanie) is a junction **at one level**, excluding a junction of
    a paved road with a dirt road or an internal road (so interchanges, driveways, car parks, fuel
    stations do not end a limit).
  * art. 2 pkt 15: the built-up area is the area marked by signs (D-42/D-43), not "looks like a town".
* **Sign regulation** = Rozporządzenie w sprawie znaków i sygnałów drogowych, consolidated text Dz.U. 2019
  poz. 2310 (https://api.sejm.gov.pl/eli/acts/DU/2019/2310/text/O/D20192310.pdf); amendments after it:
  2021/433, 2021/2065, 2022/2372, 2026/133 (not checked for these paragraphs).
  * § 15 ust. 1-2: a prohibition sign applies on the road it stands on, from where it stands;
    ust. 4: a prohibition under D-42 applies until D-43.
  * § 27 ust. 1: **B-33** applies until another B-33 or B-43, D-40, or the start/end of a built-up area;
    ust. 2: a plate with a vehicle symbol limits it to that vehicle; ust. 3: B-33 > 50 in a built-up area
    applies to cars/motorcycles/trucks <= 3.5 t; ust. 4: **B-34** ends a B-33.
  * § 31: **B-43** speed zone, **B-44** end of the zone.
  * § 32 ust. 1: **B-42** ends B-33 (among others); ust. 2: if not ended earlier, B-33 applies **until the
    next intersection**, except an intersection on a dual carriageway where the side road enters only from
    the left with no link to the right carriageway.
  * § 48: D-7/D-8 expressway, D-9/D-10 motorway; § 58: D-40/D-41 residential zone, D-42/D-43 built-up area.
* Secondary explainers that agree (not used as sources of rules): interia.pl "Skrzyżowanie nie zawsze
  odwołuje ograniczenie prędkości", auto-swiat.pl "Dokąd obowiązuje znak B-33".

### Rule table (`rules/pl.json`)
| Classifier class (MTSD, `--gN` stripped) | PL sign | Effect |
|---|---|---|
| `regulatory--maximum-speed-limit-N`, `...-led-N` | B-33 | limit N; ends at next intersection (proxy), B-34/B-42, another B-33, area/zone signs, D-9/D-7 |
| `regulatory--end-of-maximum-speed-limit-N` | B-34 | back to the default |
| `regulatory--end-of-prohibition` | B-42 | back to the default |
| `regulatory--end-of-speed-limit-zone` | B-44 | ends a (suspected) speed zone |
| `information--living-street` / `end-of-living-street` | D-40 / D-41 | zone 20 / back to built-up 50 |
| `information--end-of-built-up-area` | D-43 | area outside, default 90, ends B-33 |
| (`information--built-up-area`) | D-42 | not in the 400 classes; hook kept for other providers |
| `information--motorway` / `end-of-motorway` | D-9 / D-10 | 140 / ordinary road; D-9 needs score >= 0.6 and right side |
| `information--limited-access-road` / `end-of-...` | D-7 / D-8 | 120 / ordinary road; same stricter evidence |
| `complementary--maximum-speed-limit-N` | (advisory plate) | `advisory_kmh` for 300 m, not a limit |
| `regulatory--truck-speed-limit-N` | B-33 + truck symbol | ignored (passenger car) |
| `complementary--trucks/buses/...` under a B-33 | T-23 style plates | the B-33 is ignored (vehicle plate) |
| `complementary--distance/...` under a B-33 | other plates | limit applied, `source.plate` set, confidence x0.8 |
| yield, stop, priority-road, end-of-priority-road, roundabout, stop line | A-7/B-20, D-1/D-2, C-12 | junction at sign + 10-15 m |
| warning crossroads / T / Y / side road / roundabout / traffic signals | A-5..A-8, A-29 | junction at sign + 75 m (built-up) / 180 m (outside) |

Defaults (passenger car): built-up 50, residential zone 20, outside 90, expressway 120, motorway 140.
Two deliberate under-reads: outside a built-up area a 2x2 dual carriageway is 100, and a single-carriageway
expressway is 100 not 120; neither is observable (no lane-direction or carriageway model), so 90 and 120
are served and stated in `notes`.

## Design
Inputs used (contract only): `signs.detections[]` (`track_id`, `cls`, `score`, `box`, `hits`),
`signs.tracks_ended[]` (`track_id`), `road.v_ego` (fallback `pose.vx`), `road.pose.yaw_rate`,
`ts_mono_ns`, `env.place` (label, smoothed scores).

1. **Odometer**: trapezoid integral of `v_ego` over road message timestamps (gaps counted as at most
   0.5 s). All distances (since, expiry caps, junction positions) are odometer metres.
2. **Track confirmation** (per track, own bookkeeping): class = score-weighted vote over the track;
   a sign counts only with >= 5 hits, vote share >= 0.6, mean score >= 0.45 (D-9/D-7: 0.6). Decided when
   the track ends (or 2 s without detections), or early once it has 12 hits. Single detections and short
   or flickering tracks never change the limit; while undecided they are listed in `pending`.
3. **Geometry**: lateral offset of the sign from the camera, `(u - cx) * 0.8 m / h_box` (B-33 medium
   size; the focal length cancels), median over the track. Accepted between -9 m (left: dual carriageway
   or one-way duplicates) and +8 m; beyond +8 m it is treated as a separate roadway (exit lane / slip
   road). Median box aspect w/h < 0.55: the sign is turned towards a side road. Boxes < 12 px tall are not
   used; a track with no usable box is not trusted. Rejections go to `ignored` with the reason.
4. **Plates**: a `complementary--*` track directly under the sign's box in the same frame. The classifier
   cannot read text plates ("dotyczy", hours, "nie dotyczy ...", weekday windows, distances): every limit
   is served as if unconditional, with `source.plate` and lower confidence when some plate is seen. A
   vehicle-symbol plate (trucks/buses) makes the B-33 not apply to the car.
5. **Area** (built-up or not): D-42 is not one of the 400 MTSD classes, so entering a built-up area is
   not observable from signs. Prior: envd's smoothed place scores, city_street + residential mass vs
   rural_road + forest_road mass (>= 0.25 and 4x the other, held 20 s); "highway" (envd's label for wide
   arterials as well as motorways) abstains. A seen area sign (D-43, D-40/D-41, D-9/D-7) beats the prior
   for 3 km. Without either the area is `unknown` and the default limit is `null` (`--`
   `default_area` in the config can set a start assumption).
6. **B-33 until the next intersection** (§ 32 ust. 2): there is no junction detector, so the proxy is
   (a) a junction-type sign passed after the limit sign (more than 20 m after it, so signs on the same pole
   do not count), at the sign + offset; (b) a heading change >= 50 deg within 60 m of travel (turned at a
   junction or roundabout; may also fire on a hairpin); (c) a distance cap without a repeat: 700 m
   built-up, 3 km outside, 1.2 km unknown, 8 km on motorway/expressway. Confidence fades linearly to half
   towards the cap. On motorways/expressways junction proxies are off (no one-level intersections; the
   limit lasts to B-34/B-33/D-10/D-8 or the cap).
7. **Zones**: B-43 is not a class either; a zone sign is likely read as `maximum-speed-limit-30` (the
   inner disc). So a B-33 <= 30 km/h inside a built-up area is marked `zone_suspect`: junctions do not
   end it, B-44/B-34/B-42/another limit/area signs do, cap 2 km.
8. **Hysteresis**: the same value seen again refreshes the limit (`confirmations`, cap restarts, `since`
   kept); a different value on the same spot (< 30 m) only wins if its track quality is higher (a pair of
   signs for different lanes or a misread digit on a duplicate).
9. **Output**: `limit_kmh` (null = unknown), `source` (sign + track, or default rule + area source),
   `confidence` (heuristic: track quality, plate, fade to cap; defaults inherit the area confidence:
   env 0.45, sign 0.6-0.8), `since` (time and odometer distance since the served value changed),
   `expiry`, `area`, `road_class`, `zone`, `advisory_kmh`, `pending`, `v_ego_kmh`, `odometer_m`.

## Tests (synthetic, `api/tests/test_speedlimit.py`)
A synthetic drive moves a pinhole camera (f 690 px) past signs at a given lateral offset; road 10 Hz,
signs 5 Hz, env 1 Hz. Scenarios: unknown at start; env prior entering a built-up area then D-43 (90, and
the prior must not override the fresh sign); env prior needs 20 s and "highway" abstains; B-33 70 then a
yield sign (back to 50, change event names the junction); junction sign on the same pole does not
cancel; B-34; B-42 outside (60 -> 90); D-40/D-41 (20, survives a junction, back to 50); zone-suspect 30
survives junctions until B-44; motorway (140, gantry B-33 100 survives a yield sign and 2 km, B-34 -> 140,
D-10 -> 90); expressway 120; D-9 on the left or with score 0.5 rejected; single detection and 3-hit track
do not flip; pending while live; class flicker 80/30 rejected; exit-lane sign at +14 m and side-road sign
(aspect 0.4) rejected; left-side duplicate at -6 m accepted; truck plate -> ignored, distance plate ->
applied + flagged; distance cap (60 holds 650 m, gone at 750 m); turn proxy (90 deg turn ends a 40);
repeat refreshes, `since` kept; new value replaces. All pass; api end-to-end test checks the endpoint,
the health summary, the status page and the `/ws` topic with the fake daemons' B-33 50.

## Replay: phone's live Krakow ride (10-min loop, 601 s period)
Recorded from the phone's `/ws` (laptop, `aiohttp`), frames from `/v1/frame.jpg` every 2 s:
* rec 1: 06:38:32-06:50:32 CET, road + signs (13 695 messages, 720 s = 1.2 loops), no env (forgotten).
* rec 2: 06:46:56-06:57:32 CET, road + signs + env (13 212 messages, 660 s).
Run offline: `.venv-models/bin/python tools/speedlimit_replay.py <rec>.jsonl` (files kept in the lane's
scratchpad, not committed: they are derived from the CC BY 3.0 ride and are 60 MB).

Speed-relevant tracks the detector produced in a loop (all in rec 1; same in rec 2 where covered):
B-33 70 at 22.2-22.8 s (pass 1 only), B-33 40 at 127.2-128.2 s (both passes), `information--motorway`
at 59-61 s (a blue sign on the left, score 0.41), `information--living-street` 1 detection at 233 s (a
far blue sign, score 0.39), `information--motorway`/pedestrian crossing 2 detections at 299 s.

What a human says from the frames (spot-checked contact sheets around every event and 3 random 40-s
windows): the whole loop is inside Kraków's built-up area (50); a B-33 **70** on the right of a dual
carriageway at ~22 s, valid to the signalised junction at ~52 s; a B-33 **40** on the Vistula bridge at
~127 s, valid to the side-street junction at ~140 s (a D-1 follows at ~144 s); no other speed sign seen.

Timeline served by the state machine (seconds from the loop start):

| t (s) | rec 2 (with env, final code) | rec 1 (no env) | human |
|---|---|---|---|
| 0-20 | null (env vote not yet 20 s old) | null | 50 |
| 20-25 | 50 default (built_up from env, conf 0.45) | null | 50 |
| 25.0 | (70 sign not detected in this pass) 50 | **70** B-33 #19339, conf 0.71 | 70 from ~22 s |
| 51.7 | 50 | 70 ended "intersection after D-1 (track 19363)" -> null | 50 from ~52 s |
| 59-63 | D-9 candidate rejected (score 0.41 < 0.6, lateral -12.7 m) | same | no motorway |
| 130.6 | **40** B-33 #19959, conf 0.75 (at 253.6 s in rec 2 time) | 40 B-33 #19438, conf 0.76 | 40 from ~128 s |
| 147.3 | 40 ended "intersection after D-1 (track 19982)" -> 50 | -> null | 50 from ~140 s |
| 233-235 | D-40 rejected (3 hits, score 0.39) | same | no living street |
| rest | 50 | null | 50 |

Agreement with the human reading (time share of the 601-s loop): rec 2 ~90 % (the missed 70 = 30 s,
40 late by ~2.6 s at the start and ~7 s at the end, 20 s of null after start); rec 1 replayed with
`default_area=built_up` (stand-in for env, which rec 1 lacks) ~98 % (70 set ~3 s after the sign, both
ends ~0-7 s late). Wrong-side errors: none high; the served value was never above the human's.

Live check after the install (07:01 CET): phone `/v1/speed_limit` served; road 14.0/s unchanged.
Live loop on the phone (`/ws?topics=speed_limit`, 07:01:47-07:11:50 CET, 586 messages): null for 10 s,
then 50 (built_up from env, 0.45); B-33 **40** (track 164) 07:10:46 -> back to 50 at 07:11:03 ("intersection
after D-1"); the 70 at ~07:08:58 was again not detected (so the 70 sign was found in 1 of 3 passes).
signd was restarted by L7 at 07:07:18 and L7 reinstalled the api at 07:11:46 with this lane's
`speedlimit.py` (same md5) plus its own changes; `speed_limit` is still served.

## Limitations / nulls
* **No junction detector**: the § 32 rule is approximated (junction signs, turn, distance cap). In the
  replay both B-33 ends were found through a D-1 placed after the junction, i.e. 5-7 s late; a junction
  with no sign and no turn ends a limit only by the cap (700 m built-up). L7's traffic lights would be a
  direct junction cue (not wired: not in the contract yet).
* **D-42 and B-43 are not classifier classes**: built-up entry comes from envd (weak, 0.45, needs 20 s
  of city scores; envd calls wide arterials "highway"); zones are guessed from B-33 <= 30.
* **Plates cannot be read** (hours, days, "dotyczy", lanes): served as unconditional, flagged.
* **Detector recall bounds everything**: the same 70 sign was missed in one of two identical passes
  (signd processes every 4th frame; the small sign at ~30-60 px got no confirmed track). Signs passed at
  speed give 4-6 detections, so most commits happen at track end, ~2 s after passing the sign.
* Lateral offset assumes a 0.8 m sign; large motorway signs (1.2 m) read 1.5x farther out (still inside
  the window for normal placement). Focal/cx come from the ring header on the phone (replay estimates).
* v_ego is supercombo's on estimated intrinsics: distances (caps, `since.distance_m`) carry that error.
* Night/vehicle rules beyond a passenger car are not modelled (trailer, trucks: other defaults).
* Not tested on a real rural, expressway or motorway drive (the only Polish open clip is city).

## Phone
api installed twice (06:59 and 07:01 CET, one `roadsense-api` restart each; the second carried the env
prior fix after the first live check showed envd labelling the arterial "highway"); original api kept
as `/usr/local/lib/roadsense/api.pre-l6`. No other unit touched; replay still the camera source.
At 07:11:46 CET L7 installed its api over it (this lane's version kept by L7 as `api.pre-l7`).
