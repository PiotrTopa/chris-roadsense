# L11 signs in effect: the traffic rules that currently apply to the car, from sign tracks

Lane opened 2026-10-27 14:25 CEST (Warsaw local, UTC+1; Fable, build + offline evaluation + deploy).
Goal: the api serves the list of rules IN EFFECT for the car (priority road, no stopping / no parking,
no overtaking, dimension limits, living street, built-up area, motorway/expressway, speed limit), the
RECENT momentary signs (no entry, turn bans, stop, yield, pedestrian crossing, one-way, ...) and the
warnings AHEAD with a distance countdown, derived from the documented `signs` contract, with Polish rules
as data. The L6 speed-limit engine is generalised; `/v1/speed_limit` and its tests stay as they are.

Laptop scratch `~/l11/` (recordings, frames, labels; nothing committed). Phone: untouched until the deploy
step at the end; the ride playlist keeps running.

## Result
* Topic **`rules`** (`/ws`, on change and >= 1/s), **`GET /v1/rules`** (+ `pending`, `recent_changes`, `ignored`),
  `rules` summary in `/v1/health`, a "rules in effect" panel on the status page, IN EFFECT / RECENT / AHEAD HUD
  lines on `/preview`. Engine `api/roadsense_api/rules.py` wrapping the unchanged L6 `SpeedLimit` (one hook
  added); rules as data in `rules/pl.json` -> `effects` (62 entries, legal sources per entry, guesses marked).
  Traffic lights are a junction cue for every "until the next intersection" rule, the speed limit included.
  Schema: `docs/ARCHITECTURE.md` "api-derived topic: rules". Offline runner `tools/rules_replay.py`.
* Tests: 16 synthetic scenarios (`api/tests/test_rules.py`), api end-to-end (`test_api.py::test_rules`, `/ws`
  with `rules`), status-page script guard (`test_index_js.py`); whole suite 69/69.
* Offline evaluation on six labelled Polish rides (pre-registered, section 6): time agreement no stopping 0.82,
  one-way 0.81, priority road 0.79, no parking 0.99, no overtaking 0.93; starts within 1 s of the sign, ends
  within ~2-4 s where a junction cue exists; 30 of 35 misses are detector misses, 25 false rules mostly
  classifier confusions (warning triangles as D-1, C-5 discs as D-3, B-35/B-36). The traffic-light cue raises
  junction recall 82 -> 98 of 178 and cuts the median end delay 12 -> 4 s.
* Phone: installed (14:55, 18:40, 18:42), `api.pre-l11` backup; road 13.2-14.0/s, signs 4.5-5.0/s unchanged.
  Live example (offline from the katowice-rain recording at 423 s, abbreviated):

```
"in_effect": [
 {"id":"speed_limit","sign":"default","name":"speed limit (built_up)","value":50,"unit":"km/h","confidence":0.45,
  "source":{"kind":"default","rule":"built_up","area_source":"env"},"since":{"age_s":62.1,"distance_m":438.4}},
 {"id":"built_up_area","sign":"D-42","source":{"kind":"env"},"uncertain":["entry sign D-42 not observable; area from envd"],...},
 {"id":"priority_road","sign":"D-1","name":"priority road","category":"lasting","confidence":0.857,
  "source":{"kind":"sign","track_id":4157,"cls":"regulatory--priority-road","hits":12,"vote":0.92,"score":0.555,"lateral_m":6.3,"plate":null},
  "since":{"age_s":44.1,"distance_m":349.1},"ends":"D-2, a yield/stop facing the car, or turning off the road","remaining_m":2959.8},
 {"id":"no_stopping","sign":"B-36","name":"no stopping","side":"right","confidence":0.925,"since":{"age_s":2.6,"distance_m":23.5},
  "ends":"next intersection (par. 32 ust. 2), B-35 or an end plate","remaining_m":685.0,"uncertain":[],...},
 {"id":"no_left_turn","sign":"B-21","name":"no left turn (and no U-turn)","category":"momentary","confidence":0.854,
  "ends":"the next intersection (par. 22 ust. 2)","remaining_m":304.7,...} ],
"recent": [ {"id":"no_left_turn","sign":"B-21","passed":{"age_s":24.9,"distance_m":139.6},"ended":"intersection (sign D-1/D-2, track 4195)",...},
            {"id":"pedestrian_crossing","sign":"D-6","passed":{"age_s":8.6,"distance_m":79.7},"lateral_m":6.3,...} ],
"ahead": [], "junction": {"last":{"odo":2341.0,"source":"sign D-1/D-2","detail":4195,"age_m":39.2},"count":17},
"area": {"type":"built_up","source":"env","confidence":0.45}, "road_class":"ordinary", "zone":{"type":"none"}, "v_ego_kmh":31.0, "rules":"PL"
```
Health: `"rules": {"in_effect": ["default 50", "D-42 built-up area?", "D-1 priority road", "B-36 no stopping (R)", "B-21 no left turn"], "recent": [...], "ahead": [...]}`.

## Limitations / nulls
* **Not observable**: D-42 built-up entry (area only from envd's weak prior, which never voted built-up in the
  villages: 5 of 5 built-up intervals missed), B-43/B-44 zones, B-27/B-28 and B-24 end signs, T-25 and all text
  plates (rules served unconditionally and flagged), dimension-limit values (served without a number).
* **Detector recall bounds everything** (30 of 35 lasting-rule misses, D-6 recall 0.50, A-7 0.59): every 4th
  frame at 5/s, small or far signs, kerbside signs leaving the narrow-FOV warsaw camera early.
* **Classifier confusions become rules**: warning triangles as D-1 (10 km of false priority road on s1), C-5
  discs as D-3, a blue sign as B-16, B-35 vs B-36; no appearance check exists for these classes (L8's verifier
  covers round red signs only).
* **Junctions are still proxies**: junction signs, turns, lights, caps. Lights at mid-block pedestrian crossings
  end rules early (2 cases); unsignalised side streets without signs end nothing until the cap (the "merged"
  intervals: 9 of 57 B-36). False lights at night (motorway lamps) make false junctions.
* **Lateral window**: B-36 on wide arterials at +9 to +10 m is rejected (4 misses); widening it would admit
  service-road signs. Not tuned (pre-registration).
* **Labels**: single labeller per ride, 2-s sheets (+-1-2 s on some times), 11 unnamed warning signs, D-2 only in
  interval ends; "unclear" stretches excluded from agreement. The two D-1 inferences of the final engine are
  in-sample for rural-zloczew and krakow-2024 (table in section 6 shows pre and final).
* Vehicle classes other than a passenger car, time/day windows, E-17a town-wide prohibitions: not modelled.

## Journal

### 0. Reading (14:25-14:45)
* L6 engine (`speedlimit.py`): per-track vote/hits/score confirmation, lateral/aspect geometry, plates,
  env area prior, junction proxies (junction signs + offset, >= 50 deg turn, distance caps), zone suspicion,
  hysteresis. All of that is reusable for every "until the next intersection" rule; the engine only acts
  on its own classes and silently drops the others, and it has no extension point.
* Classifier classes (phone `/var/lib/roadsense/models/mtsd_signcls_classes.txt`, 400 entries, 14:30):
  verified PRESENT: `regulatory--priority-road`, `end-of-priority-road`, `yield`, `stop`, `no-stopping`,
  `no-parking`, `no-parking-or-no-stopping`, `end-of-no-parking`, `end-of-prohibition`, `no-overtaking`,
  `no-overtaking-by-heavy-goods-vehicles`, `height-limit`, `weight-limit`, `weight-limit-with-trucks`,
  `width-limit`, `information--living-street` / `end-of-living-street`, `end-of-built-up-area`,
  `end-of-motorway`, `end-of-limited-access-road`, `roundabout`, `one-way-left/right/straight`,
  `no-left-turn`, `no-right-turn`, `no-u-turn`, `no-entry`, `warning--roundabout`, `warning--school-zone`,
  `warning--children`, `warning--crossroads(-with-priority-to-the-right)`, `warning--pedestrians-crossing`,
  `information--pedestrians-crossing`, railway crossings, `warning--traffic-signals`.
  Verified ABSENT: `information--built-up-area` (D-42), speed-zone entry (B-43; only `end-of-speed-limit-zone`),
  `end-of-no-overtaking` (B-27) and B-28, B-24 (end of no-U-turn), B-37/B-38 (odd/even days), B-39/B-40
  (parking zone), D-37/D-38 (tunnel), T-25 plates. Any such sign in the field is read as some other class or not
  at all.
* Phone at 14:30: all units active (camerad-replay playlist, modeld, signd + lights, envd, api); api files
  identical to HEAD (md5). Ride 5/21 (winter-snow-rural) playing; one loop is ~4 h 08 min.
* Recording started 14:31 CEST: laptop `~/l11/rec/loop.jsonl` <- phone `/ws?topics=camera,road,signs,lights,env,speed_limit`
  for 15 900 s (one full playlist loop; `~/l11/scripts/rec_ws.py`, PID in `loop.pid`). The `signs` messages
  carry the api's L8 context flags, so the offline run sees exactly what the phone's engine would see.

### 1. Design
**Where.** New pure module `api/roadsense_api/rules.py` (`RulesInEffect`), wrapping the unchanged L6 engine:
it owns a `SpeedLimit`, feeds it every message, and reads its odometer, track objects, geometry helpers,
area / zone / road-class beliefs and junction list. `speedlimit.py` gets one extension point: a list of
track hooks called whenever a track is decided (live early-commit or at its end), so the new engine judges
the classes L6 ignores with the same confirmation, geometry and plate code. `/v1/speed_limit` keeps its
behaviour; the only new input that can touch it is the traffic-light junction cue (below), appended to the
same junction list the L6 proxies use (off by config if unwanted).

**Rules as data.** `rules/pl.json` gains an `effects` table (one entry per classifier class pattern):
`id`, Polish sign code, short name, `category` (`lasting` | `momentary` | `warning` | `end`), how it ends
(`end` signs, `junction` yes/no, `cap_m` per area, `replaced_by`), `until_junction` for momentary signs
that bind at the next intersection (B-20, A-7, B-21/B-22/B-23, C-2..C-8), `applies_to_car`, `side_specific`
(B-35/B-36 apply to the side of the road they stand on), acceptance (`min_hits`, `min_score`, lateral
window), the legal source and a `guess` note where the ending is an engineering assumption. The table
is below ("Polish rules").

**Confirmation.** Same as L6: score-weighted class vote over the track, decided when the track ends
(or at 12 hits live); lasting rules need >= 5 hits, vote >= 0.6, mean score >= 0.45 (per-class overrides),
usable boxes (>= 12 px), lateral offset inside the window, aspect >= 0.55 (turned signs belong to a side
road). Momentary signs and warnings need >= 3 hits (they pass quickly). Suppressed tracks (L8 flags) are
"not seen" and listed in `ignored` with the reason. Plates (`complementary--*` under the sign) cannot be
read: the rule applies unconditionally and is flagged `uncertain: ["plate"]`; a vehicle-symbol plate
(trucks/buses) makes the rule not apply to a passenger car.

**Junctions.** One junction stream for every "until the next intersection" rule (§ 32 ust. 2):
(a) L6 proxies: junction signs passed (A-7/B-20/D-1/D-2/C-12 + 10-15 m; A-5..A-8/A-29 + 75/180 m),
>= 50 deg heading change within 60 m, distance caps; (b) NEW: traffic lights — the ego light (L7 topic,
api's `ego` annotation) seen within 45 m and then gone for >= 1.5 s while the car moved >= 5 m registers a
junction at the last sighting's position + its distance (a light passed overhead or left the frame at
the stop line; red waits are covered because the light stays in view until the car moves on). Lights at
mid-block pedestrian crossings will produce false junctions; the offline evaluation runs with and
without the cue (`lights_junction` config) to see whether it ends rules more accurately.

**Output.** Topic `rules` (`/ws` on every change and >= 1/s; `GET /v1/rules` adds `recent_changes`,
`ignored`, `pending`), a summary in `/v1/health`, a "rules in effect" panel on the status page and up
to three HUD lines on `/preview` (IN EFFECT / RECENT / AHEAD). Schema in `docs/ARCHITECTURE.md`.

**Resets.** A ride change or camera restart (L10) replaces the whole engine (`State.reset_drive`).

### 2. Evaluation protocol (PRE-REGISTERED, committed before any scoring)
Data: the phone recording above (real pipeline: signd with the L8 verifier, lights, envd), split by ride
with the `camera` messages (`ride`, `ride_t_s`); frames for labelling are cut from the laptop's copies of the
rides (`<datasets>/rides/<slug>/<slug>.mjpeg`, CC BY 3.0 per `SOURCE.txt`, never committed) at the
ride times of the recording (contact sheets, one thumbnail every 2 s, plus single frames around events).

Rides (chosen before looking at any engine output on them): **krakow-2024, rural-zloczew, forest-vistula-spit,
s1-roadworks, warsaw-city, katowice-rain** (city, rural with area signs, forest road with no-stopping and
crossings, roadworks with temporary signs, tram city, rainy arterials).

Ground truth (human reading from frames, by this lane; written to `~/l11/labels/<ride>.json` BEFORE the
engine's timeline for that ride is looked at): for every lasting rule in scope (priority road, no stopping,
no parking, no overtaking, dimension limit, living street, built-up area, motorway/expressway) the
intervals [start ride time, end ride time] in which a human says the rule is in effect for the car,
with the ending reason seen (end sign / junction / unclear). For momentary signs and warnings: the ride
time at which each relevant sign is passed. Signs legible only to a human (plates, zone entries) are
labelled as the law reads them and the engine's deviation is counted as a known limitation, not an
error. Where the human cannot tell (e.g. whether a crossing side street is a public road), the interval
end is marked "unclear" and both readings are accepted.

Metrics (per ride and pooled):
1. **time agreement** per lasting rule: share of the ride's time in which engine and human agree
   (in effect / not), excluding "unclear" stretches;
2. **start delay** (engine start - human start, s; negative = early) and **end delay** per interval;
3. **misses**: human intervals with no engine interval overlapping >= 50 %; each attributed to
   `detector` (no confirmed track of that class in the recording within +-5 s of the sign) or `engine`
   (a track existed and was rejected or mis-ended);
4. **false rules**: engine intervals with no overlapping human interval, with the cause (false track,
   wrong class, side road, wrong end);
5. momentary / warning signs: precision and recall of "passed" events (match within +-6 s);
6. junction cue: for rules that end at a junction, |end delay| with and without the lights cue, and the
   number of ends attributed to lights, signs, turn, cap;
7. cost: us per message offline; on the phone, road / signs rate before and after the install.

No threshold in the rule table is tuned on these six rides after the labels exist; anything changed
after scoring is disclosed as in-sample.

### 3. Build (14:40-15:05)
* `speedlimit.py`: one extension point, `track_hooks` (called from a new `_decide(track, live)` wrapper around the
  unchanged `_consider`); nothing else changed, 21/21 L6 tests pass.
* `rules.py` (`RulesInEffect`, ~330 lines): wraps `SpeedLimit`; `effects` table compiled from `rules/pl.json`
  (62 entries; every one of the 400 classifier classes that matters maps to an entry, checked against the phone's
  class file); per-track judgement with the shared helpers (`quality`, `_geometry`, `_plate_for`,
  `_distance_ahead`); active rules / recent / ahead; junction cues (L6 sign proxies copied from the engine's list,
  own turn detector on the engine's heading window, traffic lights); expiry (caps per area, junction, hold, warning
  range + 100 m grace); snapshot and a `summary()` for health and the HUD.
* `state.py`: `rules_engine` owns the speed engine (`state.speed` stays as an alias, so L10's reset test and the
  e2e tests are untouched); `lights` messages now also reach the derived path; `rules` emitted on change and once
  a second; health carries `rules`. `server.py`: `GET /v1/rules`, status-page panel, HUD lines via `rules_hud()`
  (ASCII, 92 chars). `tools/fake_daemons.py`: a B-25 track and an A-16 track on a second 20 s schedule so the api
  e2e test can see a lasting rule and a warning. `docs/ARCHITECTURE.md`: endpoint row, `/ws` topic, topic schema.
* Tests: `api/tests/test_rules.py`, 15 synthetic scenarios (priority road until D-2 / until a facing yield, B-36 not
  ended by B-42 but by a junction sign, B-36 -> B-35 -> end plate, B-25 until B-42 / 3 km cap / truck plate ignored /
  distance plate flagged, B-21 until a traffic-light junction which also ends a B-33 40 (and not with the cue off),
  A-8 countdown and C-12, generic warning outside (225 m), B-2 on a side road flagged, B-26 not for this car,
  height limit without a value, suppressed / 2-frame / other-roadway tracks ignored, repeat refreshes + turn proxy,
  health strings). Whole suite 65/65 (`PYTHON=.venv-models/bin/python tools/smoke_api.sh`, 16 s).
* `tools/rules_replay.py`: offline runner over a recorded `/ws` stream (raw lines or the recorder's wrapper), split
  by ride with the camera messages, per-ride focal from `<datasets>/rides/<slug>/camera-replay.json`, prints
  START/END/PASS/AHEAD/JUNC/IGN events in ride seconds, `--out` writes the intervals for scoring, `--no-lights`.
* Cost: 13-20 us per message offline (laptop, all topics incl. camera skipped).
* First look (NOT evaluation rides; used only to catch bugs): `winter-snow-rural` 765-880 s: D-1 with a plate at
  765.0 (engine START D-1 765.0), yield + "STOP 50 m" plate at 861 (engine: A-7 at 861.0 ends the priority road;
  D-2 passed 863.6 on the next pole), D-6 at 866-868 (engine PASS D-6 868.8), STOP at 868 (engine START B-20 868.4,
  ended by the turn at the junction 879.1) - all checked on full frames. `a1-night` (motorway, night): three "A-24
  cyclists ahead" warnings; one is a real warning triangle of another class (23 s), two are lights in the dark
  (scores 0.46-0.67, 1-3 frames); two "traffic light" junction cues on a motorway (L7 false lights at night).
  Change after this look (before any evaluation ride was scored): a track must have been SEEN in >= 2 frames by
  the api (the provider's `hits` can be 3 on a track the api saw once).
* Commits: 4c199b5 (engine, topic, tests, tools, docs).

### 4. Phone deploy #1 (14:54-14:56)
`~/l11/deploy.sh` as root: `api.pre-l11` backup, 5 files installed (`rules.py`, `speedlimit.py`, `state.py`,
`server.py`, `rules/pl.json`), syntax check, `roadsense-api` restart. Finding: the api took **90 s to stop**
(`State 'stop-sigterm' timed out. Killing.`): with a `/ws` client attached (this lane's recorder) the aiohttp
shutdown waits for the websocket handler, which never returns; systemd killed it at its 90 s TimeoutStopSec.
Pre-existing (earlier lanes restarted without a ws client); the recorder reconnected by itself, ~95 s of
`a1-night` missing from the recording (not an evaluation ride). 14:56: all five units active, road 13.3/s, signs
4.9/s, lights 2.8/s, `rules` at 0.5/s (heartbeat), `/v1/rules` served; first live cue: a "traffic light" junction
at odo 1110 m on the A1 at night (false light, see above).

### 5. Labels (15:00-15:36)
Six Opus labellers (one ride each, two in parallel; brief `docs/lanes/L11-signs-in-effect/labels/BRIEF.md`), each
with the ride's contact sheets (one frame every 2 s, 12 per sheet, direct MJPEG indexing: `sheets.py`) and a
full-resolution frame tool (`frame.py`, any ride time, ~2 s per call); they never saw detector or engine output.
Labels committed 15:36 (55c44fe) before any engine run on these rides:

| ride | intervals | momentary | warnings | junctions | full frames viewed | labeller's main doubts |
|---|---|---|---|---|---|---|
| krakow-2024 | 17 (D-1 x3, B-36 x7, B-35, D-3 x5, B-18) | 40 | 6 | 24 | 122 | whether a turn at a signalised junction ends D-1; which side roads are public |
| rural-zloczew | 5 (D-1 13.8-854, D-42/43 x3, B-36) | 12 | 20 | 47 | 309 | one D-42 read from a blurry frame; B-36 end street |
| forest-vistula-spit | 17 (B-25 x2, D-42, D-1 x3, B-36 x11) | 30 | 9 | 30 | 182 | B-25 -> B-27 5 s apart; no D-43 seen |
| s1-roadworks | 3 (D-1, D-7 to ride end, D-3) | 3 | 17 | 9 | 70 | what an intersection is on an expressway |
| warsaw-city | 40 (D-1 x6, B-36 x22, D-3 x12) | 103 | 3 | 39 | 157 | median openings as intersections; B-35 vs B-36 at a distance |
| katowice-rain | 23 (B-36 x16, D-1 x2, D-3 x2, B-35, B-18, D-42 area) | 42 | 6 | 37 | 208 | built-up start never seen (only the D-43 at 771 s) |

The labellers put D-2 under the interval's `end_reason`, not in `momentary`, so the engine's D-2 passes score as
unmatched below (6 of them); `one_way_side` (D-3 of a cross street, 13 human) is likewise a class the labellers
used and the engine rarely accepts (lateral window).

### 6. Evaluation (18:00-18:45; rides recorded 15:43-18:38, one pass of the phone's real pipeline)
Three engine variants, all from the same recording (`~/l11/scripts/eval_all.sh`, scores in `~/l11/rec/eval/`):
**pre** = the pre-registered engine (commit ddc6b57 behaviour) with the traffic-light junction cue;
**pre, no lights** = the same without it (`--no-lights`); **final** = pre + two D-1 inferences added after the
first scoring of rural-zloczew and krakow-2024 (IN-SAMPLE for those two rides, out of sample for the other four,
switchable: `priority_inference`): a turn of >= 50 deg at a junction ends the priority road (the car left the
road; where the priority road itself bends a D-1 with a T-6 plate is repeated and restarts it), and an A-6
side-road warning (par. 5 ust. 2: the side road is subordinate) refreshes the D-1 distance cap. No other
threshold was touched after the labels existed.

Pooled over the six rides (time agreement = share of ride time, outside the labellers' "unclear" stretches, in
which engine and human agree whether the rule is in effect; matched = human intervals with a matching engine
interval; merged = human intervals that fall inside an engine interval already matched by an earlier one, i.e.
the engine kept one rule across a junction the human counted as an end + restart):

| rule | human n | pre: agreement | pre, no lights | **final** | matched / merged / missed / false (final) |
|---|---|---|---|---|---|
| no_stopping B-36 | 57 | 0.818 | 0.795 | **0.818** | 32 / 9 / 16 / 11 |
| one_way D-3 | 20 | 0.808 | 0.772 | **0.808** | 10 / 1 / 9 / 7 |
| priority_road D-1 | 16 | 0.729 | 0.729 | **0.789** | 13 / 0 / 3 / 3 |
| no_parking B-35 | 2 | 0.990 | 0.990 | **0.990** | 2 / 0 / 0 / 1 |
| no_overtaking B-25 | 2 | 0.931 | 0.930 | **0.931** | 1 / 1 / 0 / 2 |
| weight_limit B-18 | 2 | 0.980 | 0.980 | **0.980** | 1 / 0 / 1 / 0 |
| height_limit B-16 | 0 | 0.991 | 0.991 | **0.991** | 0 / 0 / 0 / 1 (a blue sign read as height-limit) |
| built_up_area D-42 | 5 | 0.436 | 0.436 | **0.436** | 0 / 0 / 5 / 0 (D-42 not a class; envd never voted built-up in the villages) |
| expressway D-7 | 1 | 0.147 | 0.147 | **0.147** | 0 / 0 / 1 / 0 (far small sign, no track) |

Per ride (final; agreement, human/engine interval counts):

| ride | no_stopping | one_way | priority_road | other | junctions: human / found by engine (lights cues) |
|---|---|---|---|---|---|
| krakow-2024 | 0.77 (7/7) | 0.81 (5/3) | 0.85 (3/4; pre 0.70) | B-35 0.99, B-18 0.96 (missed), B-16 0.99 (false) | 24 / 14 (7) |
| rural-zloczew | 0.98 (1/1) | - | 1.00 (1/1; pre 0.85) | D-42 0.71 (3 missed), B-25 0.97 (1 false) | 46 / 16 (1) |
| forest-vistula-spit | 0.76 (11/14) | - | 0.96 (3/4) | B-25 0.89 (2/2), D-42 0.46 (missed) | 27 / 19 (1) |
| s1-roadworks | - | 0.98 (1/1) | 0.25 (1/2) | D-7 0.15 (missed) | 7 / 4 (0) |
| warsaw-city | 0.79 (22/11) | 0.67 (12/8) | 0.82 (6/3) | - | 38 / 20 (14) |
| katowice-rain | 0.80 (16/10) | 0.78 (2/5) | 0.85 (2/2) | B-35 0.99, B-18 1.00, D-42 0.14 (missed) | 36 / 25 (13) |

Delays (final, 59 matched intervals): start median -0.3 s (the engine commits a lasting rule when the sign's
track ends, i.e. as the sign is passed); end median +1.6 s, |end| median 2.4 s on intervals whose end the
human was sure of. Ends by cause (n, median delay): junction cue 41 (+3.6 s), facing A-7 9 (-1.0 s), B-36 -> B-35
and back 3 (0 / +20 s), turned off the road 4 (-12 s), distance caps 2 (+8 s, +203 s).

**Lights as a junction cue** (pre vs pre-no-lights, out of sample): 36 of 184 junction cues came from the ego
traffic light; human junctions found by any cue 98 vs 82 of 178; median end delay of junction-ended rules
11.8 s -> 3.6 s (|delay| 11.8 -> 5.5 s); no_stopping agreement 0.795 -> 0.818, one_way 0.772 -> 0.808 (warsaw
0.71 -> 0.79, katowice 0.78 -> 0.80); false rules +2 (two B-36 ended by a light at a mid-block crossing after a
real one had ended). Ends by the cue were right at the stop line (city) and wrong where the light belonged to a
pedestrian crossing without a junction (expected; 2 cases found).

**Misses** (final, 35): 30 detector (no confirmed track of the class within +-5 s: far or small signs, the narrow
1100 px warsaw camera that loses kerbside signs early, D-42 absent from the classes, D-7 tiny), 4 engine (B-36 at
lateral +9.1 to +10.2 m on wide arterials, outside the +8 m window inherited from L6), 1 classifier (B-36 read as
B-35). **False rules** (25): 11 B-36, 7 D-3, 3 D-1, 2 B-25, 1 B-16, 1 B-35. Inspected causes: classifier
confusions (two deer-warning triangles read as `priority-road` at 0.81 -> 10 km of false D-1 on s1-roadworks;
blue C-5 "straight ahead" discs on medians read as `one-way-straight` D-3; a blue sign as `height-limit`; B-35
vs B-36), signs for the other carriageway or a service road inside the lateral window, and a few timing
disagreements with the labeller (engine B-36 at 94.8 s vs human 104.5 s in katowice). On a dual carriageway a
median D-3 is legally correct for that carriageway, so part of the D-3 "false" rules are label disagreements.

**Momentary signs** (pooled, +-6 s): D-6 pedestrian crossing 53 tp / 85 engine / 105 human (P 0.62, R 0.50);
A-7 yield 20/26/34 (P 0.77, R 0.59); B-21 no left turn 10/13/22 (P 0.77, R 0.45); C-12 roundabout 6/11/10;
B-2 no entry 2/4/7 (all human B-2 were for side roads; the engine flags them "side road?" when beyond +8 m);
C-2..C-8 mandatory direction 5/19/8 (median C-5 discs at every big junction fire on the engine's side too);
B-23 1/3/1. Recall is bounded by the detector seeing every 4th frame at 5/s; precision by the lateral window
(signs for cross streets at junctions). **Warnings** (ahead): A-17 children 5/6/6, A-29 traffic signals 3/3/3,
A-14 roadworks 2/3/2, A-30 other danger 5/11/7, A-1..A-4 curves 6/11/6, A-18 animals 2/5/5, A-16 2/6/3;
false classes: A-20 two-way traffic 11 false of 12 (tram / lane boards), A-5 crossroads 7/0, lane-layout 6/0;
11 human "A-?" signs the labellers could not name scored as misses.

**Cost**: 19-32 us per message offline (laptop; city rides highest); phone road 13.2-14.0/s, signs 4.5-5.0/s,
lights 2.5-2.8/s before and after the install (14:30 vs 18:13-18:42), die 85-91 C in the sunny forest ride with
one 93.6 C reading (18:13), 72 C at 18:42.

### 7. Phone deploy #2 and #3 (18:40-18:42)
Recorder stopped 18:38 (one full loop, 618 242 messages, 2.75 GB in `~/l11/rec/loop.jsonl`; per-ride files for the
six rides next to it). 18:40: final engine (rules.py with the D-1 inferences, pl.json, server.py with the `/ws`
close-on-shutdown) installed, restart took 1 s (was 90 s); all units active, road 13.2/s. 18:41: the
coordinator reported the status page stuck at "..." - a raw newline inside the rules panel's `join("\n")`
(the same bug class as fold 7; this lane's own check had only looked at one of the two places). Fixed, regression
test `api/tests/test_index_js.py` (scans the page's `<script>` for newlines inside quoted literals), installed
18:42 with one more restart, the served page verified clean. The 21-ride playlist was never touched.

## Polish rules (sources)
Primary text: **Rozporządzenie w sprawie znaków i sygnałów drogowych**, consolidated text Dz.U. 2019
poz. 2310 (Sejm ELI API PDF, downloaded 14:30, grepped), amendments after it not checked for these
paragraphs; **PoRD** Dz.U. 2024 poz. 1251 as in L6 (art. 2 pkt 10 intersection; art. 20 speeds).

| Rule (id) | Sign | Classifier class | Category | Ends at | Source | Guess / note |
|---|---|---|---|---|---|---|
| priority_road | D-1 | regulatory--priority-road | lasting | D-2; a yield (A-7) or stop (B-20) facing the car; soft cap 3 km built-up / 10 km outside (fade) | § 43 ust. 1, 3 (D-1 start/continuation, D-2 end) | A-7/B-20 ending and the cap are inferences (the law has no distance; D-1 is repeated before junctions) |
| no_stopping | B-36 | regulatory--no-stopping (+ no-parking-or-no-stopping, not a Polish sign, treated as B-36 and flagged) | lasting, side-specific | next intersection; B-35 (then no_parking starts); "end-of-no-parking" class; NOT B-42 | § 28 ust. 2-4, § 32 ust. 2 | T-25a/b/c plates unreadable (flag); B-42 does not cancel B-35/B-36 (§ 32 ust. 1 lists B-23, B-25, B-26, B-29, B-33 only): the brief's assumption corrected |
| no_parking | B-35 | regulatory--no-parking | lasting, side-specific | next intersection; "end-of-no-parking" class; replaced by B-36 | § 28 ust. 1, 3, § 32 ust. 2 | as above; B-37/B-38 not in the classes |
| no_overtaking | B-25 | regulatory--no-overtaking | lasting | B-27 (not a class), B-42, next intersection; caps as B-33 | § 23 ust. 1, 4; § 32 ust. 1-2 | T-23 vehicle plates: applies to that vehicle only (ignored for the car) |
| no_overtaking_hgv | B-26 | regulatory--no-overtaking-by-heavy-goods-vehicles | momentary info, applies_to_car false | - | § 23 ust. 3, 5 | shown in Recent as "not for this car" |
| height_limit / weight_limit / width_limit | B-16 / B-18 / B-15 | regulatory--height-limit, weight-limit(-with-trucks), width-limit | lasting (value unreadable) | soft cap 1 km (fade); next intersection | § 20 ust. 1, 2, 4; § 15 ust. 1-2 (on this road from the sign) | value not readable by the classifier: shown without a number; a passenger car is normally under it; cap is a guess |
| living_street | D-40 / D-41 | information--living-street / end-of-living-street | lasting zone (20 km/h, pedestrians have priority) | D-41 only | § 58 ust. 1; PoRD art. 20 ust. 2 | from the L6 zone belief |
| built_up_area | D-42 / D-43 | (D-42 absent) / information--end-of-built-up-area; entry from envd | lasting | D-43; env prior | § 58 ust. 3; PoRD art. 2 pkt 15 | entry only via envd (conf 0.45) |
| motorway / expressway | D-9 / D-10, D-7 / D-8 | information--motorway / end-of-motorway, limited-access-road / end-of-... | lasting | end sign | § 48 | from the L6 road-class belief |
| speed_limit | B-33 ... | (L6) | lasting | (L6) | § 27, § 32 | unchanged engine |
| one_way | D-3 | regulatory--one-way-straight | lasting | next intersection (guess) | § 44 ust. 1 | D-3 marks start or continuation; no end sign exists |
| no_left_turn / no_right_turn | B-21 / B-22 | regulatory--no-left-turn / no-right-turn | momentary, until the next intersection | that intersection; cap 300 m | § 22 ust. 1-2 | "nie dotyczy" plates unreadable |
| no_u_turn | B-23 | regulatory--no-u-turn | momentary, until the next intersection inclusive | that intersection; B-42; cap 300 m | § 22 ust. 5-6; § 32 ust. 1 | B-24 not a class |
| stop / yield | B-20 / A-7 | regulatory--stop / yield | momentary, until the intersection | sign + 15 m | § 21 ust. 1; § 5 ust. 5; § 3 ust. 3 (A-7 25-50 m before) | also junction proxies (L6) |
| roundabout | C-12 | regulatory--roundabout | momentary | passed | § 36 | junction proxy (L6) |
| mandatory direction | C-2..C-8 | regulatory--go-straight*, turn-left/right(-ahead), go-straight-or-turn-* | momentary, until the next intersection | that intersection; cap 200 m | § 35 | |
| no_entry | B-2 | regulatory--no-entry | momentary | passed (usually a side road or the opposite carriageway) | § 17 ust. 1 | lateral offset reported; "side road?" beyond +8 m |
| no_vehicles | B-1 / B-3 / B-3a / B-4 / B-5 ... | regulatory--road-closed(-to-vehicles), no-motor-vehicles(-except-motorcycles), no-heavy-goods-vehicles, no-buses, no-motorcycles, no-bicycles, no-pedestrians, no-motor-vehicle-trailers, no-vehicles-carrying-dangerous-goods | momentary (entry prohibitions of a road) | passed | § 16-19 | applies_to_car per class (B-5/B-5a/B-6 trucks, buses, bikes: false) |
| pedestrian_crossing | D-6 | information--pedestrians-crossing | momentary | passed | § 47 ust. 1, 4 | |
| priority_over_oncoming / give_way_oncoming | D-5 / B-31 | regulatory--priority-over-oncoming-vehicles / give-way-to-oncoming-traffic | momentary | passed | § 46; § 25 | |
| dead_end | D-4a | information--dead-end | momentary info | passed | § 45 | |
| end_of_prohibitions | B-42 | regulatory--end-of-prohibition | end | ends no_overtaking, no_u_turn (and B-33 via L6) | § 32 ust. 1 | |
| end_of_priority_road | D-2 | regulatory--end-of-priority-road | end | ends priority_road | § 43 ust. 3 | |
| end_of_no_parking | (B-35/B-36 + T-25c) | regulatory--end-of-no-parking | end | ends no_parking and no_stopping | § 28 ust. 7 pkt 3 | the MTSD class is a foreign end sign; mapping is a guess |
| warnings "ahead" | A-5..A-8, A-9/A-10, A-16, A-17, A-29, A-1..A-4, A-11/A-11a, A-12, A-14, A-15, A-18, A-20, A-21, A-24, A-30 ... | warning--* | warning | expected 0-100 m where the limit is <= 60 km/h (built-up), 150-300 m otherwise; dropped past the range + 100 m or at the junction it announced | § 3 ust. 1-2 (distances), § 5 (A-5..A-8), § 6 (A-9/A-10), § 8 (A-16, A-17) | T-1/T-2 plates (distance / section length) unreadable: flagged |

Not modelled (not observable or out of scope): B-43/B-44 speed zones (entry sign absent), B-39/B-40 parking
zones, B-37/B-38, D-44/D-45 paid parking, C-9/C-10/C-11 (pass on this side: trivially obeyed), D-18 parking,
D-15 bus stops, E-17a/E-18a town signs (prohibitions under E-17a apply to the whole town: § 15 ust. 3),
vehicle classes other than a passenger car, time/day windows on signs or plates (§ 15 ust. 6).
