# L11 signs in effect: the traffic rules that currently apply to the car, from sign tracks

Lane opened 2026-10-27 14:25 CEST (Warsaw local, UTC+1; Fable, build + offline evaluation + deploy).
Goal: the api serves the list of rules IN EFFECT for the car (priority road, no stopping / no parking,
no overtaking, dimension limits, living street, built-up area, motorway/expressway, speed limit), the
RECENT momentary signs (no entry, turn bans, stop, yield, pedestrian crossing, one-way, ...) and the
warnings AHEAD with a distance countdown, derived from the documented `signs` contract, with Polish rules
as data. The L6 speed-limit engine is generalised; `/v1/speed_limit` and its tests stay as they are.

Laptop scratch `~/l11/` (recordings, frames, labels; nothing committed). Phone: untouched until the deploy
step at the end; the ride playlist keeps running.

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
rides (`~/datasets/rides/<slug>/<slug>.mjpeg`, CC BY 3.0 per `SOURCE.txt`, never committed) at the
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

### 3. Build
(filled in as the work proceeds)

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
