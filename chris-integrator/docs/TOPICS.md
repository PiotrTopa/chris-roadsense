# The consolidated topics and API of chris-integrator

What chris-integrator serves to consumers: the HTTP / WebSocket API (`chris_integrator/io/http.py`, `io/api.py`)
and the messages of its public streams. The text and tables below were moved verbatim from chris-roadsense's
`docs/ARCHITECTURE.md` (where they were kept until this component had its own reference; lane L43); words like
"api", "mapd" and "roadsense 2.0" in them name the component that served a topic before lane L39. The `map`
topic (the map box) is described in chris-mapper's `docs/MAP.md`, the map pack format in its `docs/PACK.md`.
The raw per-camera topics this API passes through (`road`, `signs`, `lights` with its `road_users`, `env`,
`camera`, `calib`, `display_lanes`, `governor`) are chris-roadsense's (its `docs/ARCHITECTURE.md`). The object
class fields of lane L42 (`tracks[].class`, `lead.class`, `view.objects[].cls`) are in the tables below; the
`road_user` objects of `view` are described in `chris_integrator/trackers/roadusers_view.py` and in
`docs/DESIGN.md` section 16. Paths in the moved text are those of the chris-roadsense repository at the time
(`docs/VEHICLE_API.md`, `chris-integrator/chris_integrator/...`, "section Calibration" of its ARCHITECTURE.md).

Ports in a deployment: the consolidated API on 8090, the camera instance's debug face on 8091 (`docs/INSTALL.md`).

The **consolidated API** is chris-integrator's output channel (`chris-integrator/chris_integrator/io/http.py`;
default port 8080, the port roadsense's api had). The table below is that API: the paths, topic names and
message shapes are those of roadsense api 2, so a consumer of api 2 only points at the integrator. Additions:
`GET /v2/mixer` (the wiring: channels, streams, plug-ins in running order with health and cost), `GET
/v2/calib`, `GET /v2/gnss`, `health.mixer`, `tracks[].class` / `lead.class` / `view.objects[].cls` (object
class as non-critical metadata). `/preview`, `/v2/frame.jpg`, `/v2/camera` and `/v2/governor` are forwarded
to the debug face (`--roadsense-http`). `POST /v2/vehicle` goes to the integrator's feed channel.

| Endpoint | Content |
|---|---|
| `GET /v2/road` | latest `road` message |
| `GET /v2/signs` | latest `signs` message plus the currently live tracks (`live_tracks`) and the flagged ones (`suppressed_tracks`, with `reason`) |
| `GET /v2/signs/history?limit=100` | ended tracks, newest first |
| `GET /v2/env` | latest `env` message (place / weather / light with scores and confidence) |
| `GET /v2/speed_limit` | current speed limit derived by the api (topic `speed_limit`, below) plus `recent_changes` and `ignored` sign tracks |
| `GET /v2/lights` | latest `lights` message with the api's ego-lane annotation (`ego`, per light `dist_m`, `lateral_m`, `relevance`) |
| `GET /v2/rules` | rules in effect for the car derived by the api (topic `rules`, below: `in_effect`, `recent`, `ahead`) plus `pending`, `recent_changes` and `ignored` sign tracks |
| `GET /v2/map` | latest `map` message of mapd (position state, matched road, map speed limit, look-ahead); 503 while mapd is not running |
| `GET /v2/fused` | map and camera combined per fact with provenance (topic `fused`, below) |
| `GET /v2/lanes` | the tracked road geometry (topic `lanes`, below): four lane lines and two road edges with a presence that has hysteresis and a filtered geometry; 503 before the first `road` message |
| `GET /v2/lead` | the tracked lead vehicle (topic `lead`, below): distance, relative speed, time gap, standing flag, `departed` cue, with the raw vision value alongside; 503 before the first `road` message |
| `GET /v2/view` | the display channel (topic `view`, below): smoothed objects and lines with ids, velocities and fades, and the road picture `view.road`, for a consumer that draws; 503 before the first `road` message |
| `GET /v2/version` | `{"name":"roadsense-api","version":"2.0.0","api":2,"topics":[...]}`; `/v2/health` carries the same as `api: {"major": 2, "version": "2.0.0"}` |
| `GET /v2/vehicle` | the car as the host describes it: mapd's `vehicle` message (every signal with age, quality and source, description, calibration estimates, feeders) plus the last scan of every range sensor (`objects`); 503 while mapd is not running. `docs/VEHICLE_API.md` 8 |
| `POST /v2/vehicle`, `POST /v2/position` | vehicle input API over HTTP (body: one message, a JSON array, or JSON lines), forwarded to mapd's `position.sock`; answers `{"api":1,"accepted":n,"rejected":[...]}`, 503 without mapd |
| `GET /v2/camera` | latest `camera` message + ring header fields |
| `GET /v2/governor`, `POST /v2/governor` | the load governor (below, "Load governor"): state, and holding a shed level by hand |
| `GET /v2/health` | `governor` (shed level, what is shed, headroom, per-topic rate / latency / degraded marker); `road_model` (`model`: `vision` or `supercombo`, `fallback`, `plan`: whether `road` carries one, `plan_age_ms`, `policy_hz`, `policy_ms`); `calibration` (the camera calibration modeld warps with: `source` `config` or `auto`, `pitch`, `yaw`, `roll`, `height_m`, and `online`: the estimate's `state`, `applied`, `pitch`, `yaw`, `off_config_deg` = how far calib.json is from it, `spread_deg`, `error_deg`, `blocks`; section Calibration); per-daemon: connected, last message age, fps, latency (`model_ms`, `detect_ms`, `env_ms`), current `env` labels, `speed_limit` summary, `rules` summary (short strings: in effect / recent / ahead), `lights_ms` and the ego light, `map` (position state, availability, pack date, matched road, map limit, mapd CPU share), `fused` (limit and source, place, lanes, tunnel) and `vehicle` (feeders, fresh signals, yaw source, range sensors) summaries; CPU temp, battery |
| `GET /v2/frame.jpg` | current preview JPEG |
| `GET /preview` | MJPEG stream with lane lines, path, leads, sign boxes and traffic lights (box in the state colour, the ego light thick with its distance) drawn on top; HUD lines IN EFFECT / RECENT / AHEAD from the `rules` topic |
| `GET /ws` | WebSocket; every message from every daemon, as-is (`lights` with the api's annotation, `signs` with its context flags), plus the api's own `speed_limit`, `rules`, `fused`, `ego_state`, `lead`, `lanes` and `view`; mapd's `ego` (the car's motion, while a feeder sends it), `vehicle` and `objects` pass through like every daemon message; `?topics=road,signs,env,speed_limit,rules,lights,map,fused,lead,lanes,view,ego,vehicle,objects` filters. Per road frame the order is `road`, `lead`, `lanes`, `view`: a consumer that needs one frame complete waits for `view` |


Paths: `/v2/...` since roadsense 2.0. Every `/v1/...` path of api 1 still answers, from the same handler and
with the content of api 2 (an alias for the transition, to be removed); `/v1/roadgeom` and `/v2/roadgeom`
answer 410 with a pointer to `view`. `/ws`, `/preview` and `/` have no version in their path.

api never touches the camera or the models; it is a fan-out and overlay layer, plus the derived
topics below (`speed_limit`, `rules`, `fused`, `lead`, `lanes`, `view`) and an optional drive recorder.

**Measurement topics and the display channel (roadsense 2.0).** Every topic except `view` is measurement-grade:
it says what was measured for its frame, adds no display latency, invents no value, and marks a held or
predicted value as such (`coast`, `held`, `meas_age_s`) with a time-out. A consumer that decides anything
(an alert, a future driving function) reads those. `view` is derived from them for consumers that draw:
smoothed, identity-keeping, with fades and velocities; it never feeds back into the measurement topics.

The split is also one of latency (owner's ruling, 2026-11-18). On the measurement topics latency matters and
nothing is smoothed for looks: they stay as fresh as the model's frame. On the display channel milliseconds do
not matter: where smoothness and lag conflict, `view` chooses smoothness (a few hundred milliseconds of display
lag are accepted), and no CPU or bandwidth is spent to cut display latency. Display policy (trust hysteresis,
holding a picture over a drop-out, fades, the "approximate" mark, which lane is lit) lives in `view` only; a
consumer that draws needs no thresholds of its own, and a consumer that decides must not read `view`.

### consolidated topic (chris-integrator): `{"type":"speed_limit", ...}`
`chris-integrator/chris_integrator/trackers/speedlimit.py` turns the documented `signs` contract (only `detections[]`
track_id/cls/score/box/hits and `tracks_ended[]` track_id), `road` (`v_ego`, `pose.yaw_rate`,
`ts_mono_ns`) and `env` (`place` label) into the limit that applies to the car. Sent on `/ws` on
every change and at least once a second; `/v1/speed_limit` adds `recent_changes` and `ignored`.
Rules are data: `chris-integrator/chris_integrator/trackers/rules/<country>.json` (the `rules` plug-in's `rules` key, default
`pl`). Design, Polish sources and limits: `docs/lanes/L6-speed-limit.md`.

```
"limit_kmh": 70,                         // null = unknown (no sign, area unknown)
"source": {"kind":"sign","sign":"B-33","cls":"regulatory--maximum-speed-limit-70","track_id":19339,
           "hits":24,"vote":1.0,"score":0.83,"lateral_m":4.1, "plate":"complementary--distance"?},
       // or {"kind":"default","rule":"built_up|outside|residential_zone|expressway|motorway","area_source":"sign|env|config"}
       // or {"kind":"unknown","rule":"unknown_area"}
"confidence": 0.71,                      // 0..1, heuristic (track quality, fades towards the expiry cap)
"since": {"ts_real_ns":..,"age_s":12.3,"distance_m":180.0},   // since the served value last changed
"expiry": {"rule":"next intersection (par. 32 ust. 2)","distance_cap_m":700,"remaining_m":520.0},  // null for defaults
"area": {"type":"built_up|outside|unknown","source":"sign|env|config","confidence":0.4},
"road_class": "ordinary|expressway|motorway",
"zone": {"type":"none|residential|speed_zone","limit_kmh":20},
"advisory_kmh": null,                    // complementary speed plate, not a limit
"pending": [ {"track_id":..,"cls":..,"sign":"B-33","action":"limit","value_kmh":50,"hits":4,"vote":1.0,
              "quality":0.6,"lateral_m":3.2} ],   // live tracks not yet confirmed
"v_ego_kmh": 48.2, "odometer_m": 3837.0, "rules": "PL"
```
### consolidated topic (chris-integrator): `{"type":"rules", ...}`
`chris-integrator/chris_integrator/trackers/rules.py` (lane L11) generalises the speed-limit engine: it owns the `SpeedLimit` above,
feeds it every `road` / `signs` / `env` message and also takes `lights`, and judges every other sign class of
the rule table `rules/<country>.json` -> `effects` (category, how it ends, side, legal source, guesses marked)
with the same track confirmation, lateral-position and plate code. Sent on `/ws` on every change and at least
once a second; `/v1/rules` adds `pending`, `recent_changes` and `ignored`. Design, Polish sources, rule table
and the offline evaluation: `docs/lanes/L11-signs-in-effect.md`; offline runner `tools/rules_replay.py`.

```
"in_effect": [                           // lasting rules + momentary signs that bind at the next intersection
  {"id":"speed_limit","sign":"B-33","name":"speed limit","category":"lasting","value":40,"unit":"km/h",
   "source":{"kind":"sign","track_id":164,"cls":"regulatory--maximum-speed-limit-40",...},   // as in speed_limit
   "since":{"ts_real_ns":..,"age_s":12.3,"distance_m":180.0}, "confidence":0.71,
   "ends":"next intersection (par. 32 ust. 2)", "remaining_m":520.0, "uncertain":[], "applies_to_car":true, "side":null},
  {"id":"built_up_area","sign":"D-42","name":"built-up area",...,"source":{"kind":"env"},"uncertain":["entry sign D-42 not observable; area from envd"]},
  {"id":"no_stopping","sign":"B-36","name":"no stopping","category":"lasting","value":null,"unit":null,
   "source":{"kind":"sign","track_id":1780,"cls":"regulatory--no-stopping","hits":14,"vote":1.0,"score":0.81,"lateral_m":3.4,"plate":null},
   "since":{...}, "confidence":0.62, "ends":"next intersection (par. 32 ust. 2), B-35 or an end plate",
   "remaining_m":430.0, "uncertain":["plate complementary--distance"], "applies_to_car":true, "side":"right"},
  {"id":"no_left_turn","sign":"B-21","category":"momentary",...,"ends":"the next intersection (par. 22 ust. 2)"} ],
   // other ids: priority_road, no_parking, no_overtaking, height_limit, weight_limit, width_limit, one_way,
   // living_street, motorway, expressway, stop, yield, no_right_turn, no_u_turn, mandatory_direction
"recent": [                              // momentary signs just passed, newest first; kept 300 m / 120 s
  {"id":"no_entry","sign":"B-2","name":"no entry","category":"momentary","source":{...},
   "passed":{"age_s":8.1,"distance_m":95.0}, "confidence":0.7, "uncertain":["side road?"], "applies_to_car":true,
   "lateral_m":11.2, "ended":null} ],   // `ended`: why an until-intersection sign left in_effect
"ahead": [                               // warning signs, nearest first
  {"id":"roundabout_ahead","sign":"A-8","name":"roundabout ahead","category":"warning","source":{...},
   "expected_in_m":60.0, "range_m":[0.0,60.0], "since_sign_m":40.0, "confidence":0.66, "uncertain":[]} ],
"junction": {"last":{"odo":2318.0,"source":"sign D-1/D-2|traffic light|turn","detail":1780,"ts_real_ns":..,"age_m":12.0},"count":6},
"area": {...}, "road_class": "ordinary", "zone": {...}, "v_ego_kmh": 48.2, "odometer_m": 3837.0, "rules": "PL"
```
`/v1/health` carries `rules: {"in_effect": ["B-33 40", "D-42 built-up area?", "B-36 no stopping (R)"],
"recent": ["B-2 no entry"], "ahead": ["A-8 roundabout ahead ~60 m"]}` (a trailing `?` marks an uncertain entry).
A ride change or camera restart replaces the engine (L10), like the speed limit.

### consolidated topic (chris-integrator): `{"type":"fused", ...}`
`chris-integrator/chris_integrator/trackers/fusion.py` combines the map layer (`map`) with the camera perception (`speed_limit`,
`env`, `road`), one fact at a time; every fact names its `source` and carries the unfused inputs
(`vision`, `map`) next to the result. The vision topics are not modified. About 2 per second on `/ws`;
`/v1/fused`. The map is used only when the `map` message is younger than 3 s, `available`, and
`match.confidence` >= 0.5; otherwise `map.available` is false with a `reason` (`no_mapd`, `stale`,
`no_pack`, `no_position`, `position_unknown`, `off_road`, `low_match_confidence`) and every fact is its
vision value: without a position source the output is vision only.

Match-confidence hold (since 2026-11-15): the matcher's confidence dips under 0.5 for a message or two where a
side road branches off while the matched road stays the same, and every fact (the speed limit that consumers
show and act on above all) would fall back to vision for that moment. A `map` message with a low confidence is
still used when it is otherwise available and fresh, it is the same road as the last message matched with
confidence (the same `match.edge` and `dir`, or the same `road.class` with the same `name` / `ref`, which must
not both be empty), its map speed limit (value, source, rule) is unchanged, it comes within 2 s
(`match_hold_s`) of that message and is at most the third low one in a row (`match_hold_msgs`). The facts are
then computed with that message's last good confidence and `map.match_hold` =
`{"messages":1,"age_s":0.5,"confidence":0.9}` is added (`match_confidence` stays the true value). It never
holds across a change of the matched road or of its map limit, an unavailable map (no position, off road) or a
stale one; once a hold has run out or been refused, only a confident message starts a new one.

```
"map": {"available":true,"state":"good","reason":null,"age_s":0.2,"match_confidence":0.97},   // + "match_hold":{..} only while held
"speed_limit": {"limit_kmh":40,
                "source":"sign|sign_held|map|map_implicit|vision_default|unknown",
                "confidence":0.8,"rule":null,"sign":"B-33","track_id":164,
                "agreement":"agree|conflict|vision_only|map_only|none",   // vision value against map value
                "note":null,"variable":false,
                "vision":{"limit_kmh":40,"kind":"sign","sign":"B-33","rule":null,"confidence":0.71},
                "map":{"limit_kmh":70,"source":"maxspeed","rule":null,"detail":"tagged","confidence":0.9}},
"built_up": {"value":true,"source":"sign|map_tagged|map_class|map_landuse+env|map_landuse|env|none","confidence":0.7,
             "vision":{"value":true,"source":"env","confidence":0.4},"map":{"value":true,"source":"landuse"}},
"place": {"label":"tunnel","confidence":0.93,"source":"fused|fused(map)|fused(vision)|map|vision|none",
          "scores":{...},"vision":{"label":"city_street","confidence":0.5},"map":{"label":"tunnel","prior":{...}}},
"lanes": {"count":4,"forward":2,"backward":2,          // lanes of the road, own direction, oncoming
          "ego_index":0,                               // 0 = leftmost lane of the own direction; null = unknown
          "ego_from_right":1,                          // since L20: lanes of the own direction to the right; null = unknown
          "probs":[0.81,0.19],"probs_from":"left",     // since L20: probability per own lane (from the left with a map
                                                       //   count, else from the right: lanes to the right 0..4)
          "estimator":{"probs":..,"confidence":0.81,"provenance":["vision","map:tagged","lane_change"],
                       "last_change":{"dir":"left","age_s":3.2},"evidence":{"edge_right_m":4.1,"lane_right":true}},
          "oncoming_left":true,                        // the lane to the left is oncoming traffic; null = unknown
          "source":"map:tagged+vision","confidence":0.8,
          "vision":{"lane_to_left":true,"lane_to_right":true},"map":{"total":4,"forward":2,"backward":2,"source":"tagged"}},
"tunnel": {"in_tunnel":true,"exit_in_m":640.0,"entry_in_m":null,"length_m":null,"source":"map|vision|map+vision|none",
           "vision":{"in_tunnel":true,"confidence":0.9},"map":{...}},
"lead": {"present","mode","track_id","dist_m","dist_std_m","lateral_m","v_rel_mps","v_lead_mps","a_lead_mps2","time_gap_s","ttc_s",
         "standing","departed","source","vision":{...}},   // the `lead` topic's latest message, shortened (below); null when it is older than 1 s. Added 2026-11-15
"road": {"class","name","ref","oneway","bridge","lit","surface","roundabout","link","motorroad","expressway","dual_carriageway","toll"},   // null without the map
"ahead": {"horizon_m","path_confidence","features","junctions","curves","speed_limit_changes","built_up_change","road_change"},   // as in `map`; null without the map
"position": {"lat","lon","heading_deg","speed_mps","hacc_m","confidence","source"}                                               // null without the map
```

Rules:

* **Speed limit.** (1) A sign the camera reads (`speed_limit.source.kind` `sign`) overrides the map from the
  moment it is read. (2) With the map available that sign limit is then *held by the map*: it stays in
  effect (`sign`, or `sign_held` once the vision engine has dropped it by its distance cap or its junction
  proxies) until the car has passed a real junction of the road graph (a node with a side road that is not a
  service road; counted from 20 m past the sign, because a sign is read before it is reached and often stands
  just after a junction; not on motorways and expressways, where a limit lasts until it is cancelled), until
  the tagged map limit changes to a third value at least 100 m after the sign (only from a match that is not
  ambiguous and at least 0.6 confident, and once the new limit has held for 40 m: a match that flips between two
  roads is not a change of the limit; on the first drive with a position it ended a sign 24 times in six minutes), until a tagged or signed
  built-up boundary, or 5 km. (3) Otherwise the tagged map limit (`map`), else the implicit map limit
  (`map_implicit`, with the built-up state from an area sign when the camera saw one and the pack only
  inferred it), else the vision engine's default (`vision_default`), else `unknown`. Without the map the
  vision engine decides alone, as before.
* **Built-up area.** Area sign seen by the camera > tag on the way > road class > landuse (confidence 0.7
  when envd agrees, 0.45 when not) > envd.
* **Place.** envd's smoothed scores are multiplied by a prior from the matched road raised to the match
  confidence (tunnel x20 on a tunnel edge and x0.15 elsewhere; motorway / expressway -> highway; forest
  flag -> forest_road; built-up -> city_street or residential by class; parking aisle -> parking_lot or
  underground_parking; bridge x3) and renormalised. Without envd the map label alone is served (`map`).
* **Lanes** (since lane L20). The ego lane comes from the ego-lane filter (`egolane.py`, below) whenever it
  ran with the matched road's tagged count: its answer, or `ego_index: null` when no lane holds 0.8 of the
  probability ("unknown" is a valid answer, a wrong confident one is not). Otherwise, and before L20, the rule
  below.
* **Lanes, rule** (before L20 and as a fallback). Counts from the map (`road.lanes`, source marked: tagged per direction, tagged total, assumed).
  The ego lane comes from the model's four lane lines: a lane to the right exists when the tracker (topic
  `lanes`, below) reports the far-right and the right line present (`vision.tracked: true`; without a fresh
  `lanes` message the frame's probabilities decide as before: far-right >= 0.5 and right >= 0.3); likewise
  to the left. Nothing to the right = the rightmost
  lane; `oncoming_left` is false on one-way carriageways, true in the leftmost own lane of a two-way road.
* **Tunnel.** In a tunnel when the matched edge is one (then `exit_in_m` is the map distance to its end) or
  when envd's label is `tunnel` with confidence >= 0.5 (no distance).

### The data integrator (chris-integrator): topics `ego_state`, `tracks`, and the source of `lead`

The integrator is a component of its own, `chris-integrator/` (design: `chris-integrator/docs/DESIGN.md`; README
there for the contracts). Since lane L39 it is the mixer that hosts every box of the consolidated side; the
estimator box (source registry, EKF, monitors, object layer) puts out the three topics of this section per
road frame (per vehicle sample while no camera frames arrive). Configuration: `<chris config dir>/integrator.json`
(`chris_integrator.DEFAULT_CFG` keys; `sources`: extra sensors, e.g. `[{"kind": "radar", "id": "*"}]`, the
default); `lead.json` keys go to the camera-lead source. A frame the estimator cannot answer goes out with
`lead.valid: false`, reason `integrator_timeout`, never from raw inputs.

`ego_state` (api 3, a superset of the 2.1 message; `GET /v2/ego_state`), one per road frame, stamped with the
frame's time of validity:

| field | meaning |
|---|---|
| `speed`, `accel`, `yaw_rate`, `heading`, `grade` | each `{value, std, valid, reason, mode: fused / coast / none, age_s, sources}`. `speed` is in the scale of the car's speed signal (m/s, signed); `yaw_rate` rad/s positive left; `heading` degrees clockwise from north; `grade` rad |
| `position` | `{valid, reason, mode, lat, lon, alt_m, hacc_m (1 sigma, semi-major), cov_en, fix_age_s, sources, local: {frame, e_m, n_m, heading_rad}, origin}`; `local` is dead-reckoned odometry also without any receiver (`reason: "no_absolute_source"`); `frame` changes when the local frame is anchored again |
| `odometer_m` | path length of the published speed |
| `standstill` | `{value, for_s, source}` |
| `calibration` | per source every estimated calibration state `{value, std, unit, known}`: `odometry.speed_scale`; `vehicle` gyro bias / scale, wheel-difference yaw scale, accelerometer offsets; `vision` delay, speed scale, yaw scale, mount `dyaw` / `dpitch` against the calibration in use with `state` (`converging / ok / offset / moved / unknown`), `in_use`, `suggested`; `gnss` delay `tau`, the slowly varying error `ue` / `un` in units of its level, and in metres `bias_e_m`, `bias_n_m`, `bias_sigma_m`, with `level` (that error against the claimed `hacc`) and `level_measured` (against a matched road, or still the prior); `map` lane offset `off` and the road-hypothesis state `amb`; radars' mount yaw and delay |
| `sources` | per source group `state` (`absent / stale / unchecked / ok / suspect / fault`), `usable`, `reason`, `isolation` (`rank / vote / self / carried`), `since_s`, `age_s`, `rate_hz`, `nis` per row class, and what the source reports of itself. `gnss`: `bias_sigma_m`, `wander_mps` (how fast it moves differently from the car; 0 while steady), `jumps`, `held_back_m` (steps of the receiver held back from its fixes), `held` (not used at all), `delay_by_motion` (`[lag s, sigma s, steps]`: the delay as its own motion against the car's shows it), `few_satellites`, `noise_scale`. `map`: `used`, `held` (why the match gives no row), `spread_m` (the spread of the candidate roads' lines the row carries), `receiver_offset_m` and `receiver_right_p` (the receiver's recent fixes beside the offered roads, and the odds that the receiver is the one that is right) |
| `checks` | every monitor: raw pairs (`share` beyond 4 sigma, `ratio`), self checks, innovation consistency (`nis`) |
| `signals` | pass-through values with `age_s` (`turn_signal`, `brake`) |
| `epoch`, `reanchors`, `rejected`, `late`, `dropped` | clock epochs seen, position re-anchors, rows not applied / too old |

`tracks` (new; `GET /v2/tracks`), one per road frame: `frame: "car"` (x forward, y **left**, origin the
reference camera), `tracks[]` each `{id, state: tentative / confirmed / coast, exist, x_m, y_m, vx_mps, vy_mps
(over ground), ax_mps2, std[5], v_rel_mps, age_s, meas_age_s, sources[], seen{source: age_s}, hits, flags
(designated, standing, ghost, stationary, on_path, merged), path_offset_m, class, attrs}` (`class`: what kind of
road user, `{label, confidence, source, age_s, votes}` or null; non-critical metadata, never a gate), `sensors{}` (per object
source: scans, detections, assigned, started, late, dropped, `nis`, `age_s`, `kind`), `path` (`plan` or null),
`valid`, `reason`.

`lead` keeps its fields (next section) and gains `exist`, `sources[]`, `on_path`, `path_offset_m`,
`lateral_std_m`; `source` names sensor kinds (`radar+vision+ego`), `radar.sensor` the sensor.

### consolidated topic (chris-integrator): `{"type":"lead", ...}`
`chris-integrator/chris_integrator/trackers/lead.py` tracks the lead vehicle over time: one message per `road` message (so at the
model's rate, 10-14 per second on the reference phone), about 0.7 kB; `/v1/lead`. `road.leads[]` is not
modified and stays the network's frame-by-frame output; `lead` is what a consumer should draw or act on
(added 2026-11-15, docs/lanes/L14-lead.md).

Why: `road.leads[0].x[0]` jitters by about 1.5 % of the distance from frame to frame (0.08 m at 4 m, with
single frames 0.5-1 m off, e.g. under a wiper) and wanders by a few decimetres over seconds even behind a
standing car; shown rounded to metres it flips between two values several times a second whenever the true
distance is near a half metre.

Inputs: `road` (`leads[0]`: `prob`, `x[0]`, `y[0]`, `v[0]`, `x_std[0]`, `v_std[0]`; the relative speed is
`v[0]` minus the model's own speed of the same frame) and the integrator's `ego_state` of the frame (above):
the ego speed with its source, standstill, the brake, and whether the camera may be believed. With a vehicle
feed the ego speed is the vehicle's (`ego.source: "vehicle"`, `source: "vision+ego"`); without one, or in a
gap of the feed, the road model's as the integrator filters it (`"vision"`): the tracker runs the same way on
vision alone. While the integrator holds the vision source faulted, or the ego speed is unknown, nothing is
tracked: `valid: false`, `present: false`, `mode: "none"`, `vision: null`, event `invalid`, and `reason` says
why (since 2.1, lane L37; before, the tracker followed whatever the model reported: on 2026-11-19 a standing
"lead" at 4-8 m for minutes behind a camera knocked 8 degrees off).

```
"valid": true, "reason": null, "inputs": {"vision": "ok", "vehicle": "ok"},   // the integrator's verdict (above)
```
"present": true,                 // a lead is tracked; false: every numeric field below is null
"mode": "track|coast|none",      // see below (until roadsense 2.0 also "hold", which froze the value)
"track_id": 12,                  // changes when the tracked vehicle is replaced (cut-in, lead change, lost and found);
                                 //   vision profile: kept when the same vehicle is found again within 3 s where it
                                 //   would be by now (`reacquired`), or is seen at another distance at the same speed (`relocated`)
"age_s": 31.4,                   // of this track
"source": "vision+ego|vision",   // vision+ego: the ego speed / standstill came from vehicle motion
"dist_m": 3.61,                  // camera -> rear of the lead along x, the same quantity as road.leads[0].x[0]
"dist_std_m": 0.62,              // 68 %: the filter's uncertainty and the network's own x_std (its absolute accuracy) in quadrature
"dist_filter_std_m": 0.021,      // the filter's part alone: how steady the value is, not how true
"gap_m": 2.11,                   // only when `camera_to_front_m` is configured: dist_m minus the camera -> front bumper distance
"lateral_m": -0.07,              // y, right positive, smoothed
"v_rel_mps": 0.0, "v_rel_std_mps": 0.0,   // lead speed minus ego speed (negative = closing); d(dist)/dt of the track
"v_lead_mps": 0.0,               // the lead's own speed over the ground (along the ego heading)
"a_lead_mps2": 0.0,              // its acceleration: the derivative of v_lead within the filter, slow (about 1 s); a hint, not a measurement
"time_gap_s": null,              // (gap_m or dist_m) / ego speed; null below 1 m/s
"ttc_s": null,                   // (gap_m or dist_m) / closing speed; null unless closing faster than 0.5 m/s
"standing": true,                // both stand (standing regime of the filter; the value stays live)
"meas_age_s": 0.0,               // since the last confirming measurement went into the track (> 0 while coasting)
"evidence": "strong",            // this frame: strong = a usable frame went in; weak = a frame under the drop
                                 //   threshold that agrees with the track (mode coast, below); none = prediction only
"road_model": "vision",          // the road model of the frame; it selects the tracker's profile (below)
"pending_change": false,         // the last frame(s) did not fit the track; a replacement is being considered
"prob": 0.99,                    // road.leads[0].prob of this frame (null when the frame had no usable lead output)
"departed": false,               // the lead moved off while ego still stands (below)
"departure": null,               // or {"age_s":0.4,"dist_at_stop_m":3.61,"moved_m":0.8}
"event": {"kind":"acquired|reacquired|cut_in|changed|relocated|departed|gap_changed|lost|reset|invalid","ts_mono_ns":..,"track_id":12,"age_s":31.4},   // the last one; null before the first
"ego": {"speed_mps":0.0,"std_mps":0.0,"standstill":true,"source":"vehicle|vision","age_s":0.04,"tag":"car-can",   // the integrator's ego speed of the frame (null when unknown); age_s: distance in time to the nearest vehicle sample
        "brake":true},              // the brake pedal (the integrator's pass-through of `vehicle.controls.brake`); null when no host sends it
"radar": null,                   // or {"sensor":"front_radar","id":17,"dist_m":31.4,"lateral_m":0.2,"v_rel_mps":-0.8,"age_s":0.03}: the range-sensor object used in this frame
"vision_scale": null,            // radar distance / camera distance, learned while both see the lead (radar fusion only)
"vision": {"dist_m":3.58,"dist_std_m":0.61,"lateral_m":-0.07,"v_lead_mps":0.0,"v_rel_mps":0.0,"a_lead_mps2":0.0,"prob":0.99}   // road.leads[0] of this frame, unfused; null without a lead output
```
`seq`, `ts_mono_ns`, `ts_real_ns` are those of the `road` message: the estimate refers to the frame, no
prediction to "now" is applied.

Estimator. State: distance `d`, the lead's speed `v_lead` and acceleration `a_lead` over the ground; the ego
speed is a known input: `d' = v_lead - v_ego`, `v_lead' = a_lead`, `a_lead` a random walk decaying over 3 s
(Kalman filter, jerk noise 30 (m/s3)2 s). So "both stand" and "the gap closes" are different states of the
model, and the relative speed is by construction the rate of change of the distance. Measurements per frame:
`x[0]` (sigma = max(hypot(0.10 m, 2 % of d), 0.2 x_std)) and the relative speed `v[0] - road.v_ego`, the
model's own pair (sigma = max(0.15, 2 v_std) + 0.1 d m/s: trusted near, little beyond 30 m where it is biased).
The model's `a[0]` is not used (it made the track lag). Modes:

| mode | when | what |
|---|---|---|
| `track` | a lead with `prob` >= 0.5 for 2 frames | the filter. A frame whose distance is more than 4 sigma and more than max(0.6 m, 12 % of d) from the prediction is **not averaged in**: the track coasts and the frame starts a candidate. Frames that agree with the candidate replace the track after 0.15 s when it is nearer (`cut_in`) and after 0.5 s when it is farther (`changed`); a new `track_id` either way. A candidate that does not persist was an outlier or the model flickering between two vehicles: nothing between the two is ever reported |
| `track` with `standing: true` | ego stands (vehicle: `standstill` for 0.3 s; vision only: the model's `v_ego` under 0.15 m/s for 1 s) and the lead stands (estimate under 0.35 m/s, measured under 0.12 m/s) for 0.5 s | **the value stays live** (one update per frame; since roadsense 2.0, lane L19). The filter's model changes: the ego speed is exactly zero, the lead's speed is measured with the model's standstill speed output (sigma max(0.12, 0.5 v_std) m/s), the lead's jerk noise drops to 0.05, and the distance is a random walk scaled so the per-frame jitter is averaged over about 3 s (`stand_tau_s`); a frame more than 2.5 sigma off counts with a weight falling as 1 / residual (Huber: a wiper pass moves it little). A two-sided CUSUM on the model's lead speed (sigma as above, slack 1 sigma, 3.5 sigma-tenths-of-a-second: a lead rolling at 0.3 m/s is caught in about 0.25 s) runs before the distance gate and ends the regime at once (event `departed` when the lead moves away, `gap_changed` when it comes nearer); ego moving (a wheel turns) ends it on that frame. What a live value shows that the old hold hid: the network's slow wander of its close-range distance (a few decimetres over seconds behind a standing car); a display that rounds to metres should add a hysteresis (0.3 m is enough on the recorded drives) |
| `coast` | `prob` under 0.3 | prediction only, `meas_age_s` counting, for 0.6 s (2.5 s while standing); then `present: false`, event `lost` |

**Profile for the vision model** (`lead.PROFILES["vision"]`, applied when `road_model` is `"vision"`; a key in
`lead.json` wins; lane L27, docs/lanes/L27-new-model-default.md). The numbers above were measured on
supercombo. The vision model has no memory: its `prob` flickers from frame to frame and it finds and drops
distant leads; its relative speed, from an image pair 200 ms apart, scatters by 0.8-1.0 m/s per frame at 50 m
and beyond where supercombo's scatters by 2-4, and its own `v_std` overstates that. The profile uses the speed
on purpose:

| | supercombo (default settings) | vision profile |
|---|---|---|
| relative speed | sigma max(0.15, 2 v_std) + 0.1 d: little weight beyond 30 m | sigma max(0.15, 0.5 v_std) + 0.01 d: the filter's speed carries the distance through gaps; relative to `pose.vx` of the same run |
| distance | sigma max(hypot(0.10 m, 2 % of d), 0.2 x_std); kinematics only | sigma with 0.4 x_std (its per-frame scatter is 2-4 m at 50 m and beyond), plus a random walk of (3 % of d)^2 per second: the model's distance drifts against its own speed output, and a track that trusted the speed alone stopped following the distance |
| start of a track | 2 frames with `prob` >= 0.5 | accumulated evidence: the `prob` of successive frames (>= 0.3) whose distance moved as the reported relative speed says add up, a frame that does not agree starts again; a track starts at 2.5. Two agreeing frames at >= 0.9 start it at once (a clear cut-in is not delayed) |
| a frame under 0.3 | no measurement: `coast`, prediction only, 0.6 s | from 0.1 on, if its distance is within max(2 m, 8 %) and its speed within 2.5 m/s of the track: a measurement with 2.5 times the sigma, `mode: "coast"`, `evidence: "weak"`; the track ends 1.5 s after its last confirming frame (2.5 s until L29). Otherwise `evidence: "none"`, prediction only, 0.6 s (0.8 s until L29; a departed lead is released after about 0.7 s, docs/lanes/L29-display-lanes.md) |
| a replacing hypothesis at another distance | new `track_id` (`changed` / `cut_in`) | the same `track_id` when its speed is within 1.5 m/s and its lateral position within 1 m of the track's (`relocated`): the model re-estimated the distance of the same vehicle. The distance still jumps; nothing is held |
| found again after `lost` | new `track_id` | the same `track_id` within 3 s when it is where the lost track would be (max(3 m, 15 %)): `reacquired`. It was absent in between and the message said so |

Nothing is held in either profile: a track without evidence is predicted for a bounded time, marked `coast`
with `evidence` and `meas_age_s`, and then dropped.
| `none` | no track | `vision` still carries the raw value |

`departed`: the standing regime ended because the lead moved away while ego still stands; true from that frame
until ego moves, both stand again, or 8 s. `departure.moved_m` is how far it has gone. On the replayed
rides it comes 0.25-0.95 s (median 0.45 s) after the lead starts to roll, when the raw distance has grown by about 0.3-0.8 m
(docs/lanes/L14-lead.md). It is information for a "the car ahead is leaving" hint, not a go signal.

Latency against smoothing: at short range the distance measurement is weighted heavily and the model's
relative speed feeds the filter directly, so on the rides the tracked distance trails a zero-phase reference
by about 10 ms under 40 m (135 ms beyond, where the model's speed is weighted little) while the gap closes faster than 2 m/s, with a third to a half less noise than the raw value; the
standing regime costs nothing while moving. Standing, on the car's recorded drives (5,400 s behind a standing
lead, docs/lanes/L19-view-channel.md): frame-to-frame jitter 0.009-0.021 m rms (raw 0.10-0.23 m), never constant
for more than 1.2 s (the old hold: up to 32 min), within 0.02-0.10 m rms of the camera's 2 s mean (the old
hold: 0.17-0.37 m). Constants: `DEFAULT_CFG` in `lead.py`, overridable per key in
`<config dir>/lead.json` (e.g. `{"camera_to_front_m": 1.6}`).

Range sensors (`{"radar_fusion": true}` in `lead.json`; **off by default**, built 2026-11-17 and tested on
synthetic radar only: no car with a radar feeds it yet). A host with a radar or lidar sends its object lists
over the vehicle input API (`objects`); mapd relays each scan with positions in the camera's frame when the
`vehicle` description has the mounts of the sensor and of the camera. The camera decides which vehicle is the
lead (a radar does not see lanes); the tracker then looks for the object of a scan at most 0.3 s from the
frame within max(3 m, 25 % of the distance) and 1.8 m laterally of the track (of the camera's lead when there
is no track yet). While one is associated, its range (sigma hypot(0.3 m, 1 %)) and closing speed (0.2 m/s)
are the frame's measurement in place of the camera's, `source` is `radar+vision+ego`, and `vision` keeps the
camera's raw value. The ratio of the two distances is learned (`vision_scale`, 5 s) and applied to the
camera's distance for a minute after the radar lost the object, so handing back to the camera is not a step.
When the camera loses the lead while the radar still has the object the track lives on the radar alone for 5 s
(`source: "radar+ego"`); a radar alone never starts a track. Without fresh scans, and always with the flag
off, the tracker is the vision tracker message for message. On the synthetic scenes (camera distance 15 %
long, radar noise 0.15 m): error under 0.5 m at 30 m against 4.5 m for the camera alone; no track restart
and under 1.5 m of error through a radar drop-out. Cost: 0.13 ms per frame with 16 objects against 0.09 ms
without (laptop).

Replay and measurement on recorded `road` lines: `tools/lead_replay.py ROAD.jsonl[.gz] [--ego surrogate]
[--every N] [--csv OUT]`.

### consolidated topic (chris-integrator): `{"type":"lanes", ...}`
`chris-integrator/chris_integrator/trackers/lanes.py` tracks the road geometry over time: one message per `road` message (10-14 per
second on the reference phone), about 2.5 kB; `/v1/lanes`. `road.lane_lines[]` / `road.road_edges[]` are not
modified and stay the network's frame-by-frame output; `lanes` is what a consumer should draw (added
2026-11-16, docs/lanes/L15-stability.md).

Why: a lane line's `prob` hovers around any fixed drawing threshold (at 0.3: 18-70 on/off changes of the four
lines per minute on the replayed rides at 10 Hz, nearly all of them spans shorter than a second), and the road
edges jump by 0.1-0.4 m from frame to frame (median; p95 0.4-1 m at 10 m), which moved everything a consumer
stands on them.

```
"x": [33],                         // the model's distances (X_IDXS), shared by all six lines
"lines": [ {"present": true,       // far left, left, right, far right: presence with hysteresis
            "mode": "track|coast|none",   // coast: present, no usable frame right now, geometry held
            "conf": 0.83,          // the filtered probability the presence is decided on
            "prob": 0.91,          // this frame's road.lane_lines[i].prob (null without a line in the frame)
            "age_s": 12.3,         // since the presence last changed
            "y": [33], "y_std": [33]} x4 ],   // filtered lateral offsets (m, right positive) and the filter's sigma; null when not present
"edges": [ {...} x2 ],             // left, right road edge, the same fields; `prob` is made from the network's y_std
"lane_to_left": true, "lane_to_right": false,   // both lines of the neighbouring lane are present
"lane_width_m": 3.42,              // between the inner lines over 6-30 m; null unless both are present
"prior": null,                     // or {"total":2,"forward":1,"backward":1,"source":"tagged","outer_left":"expected","outer_right":"unlikely"}
"source": "vision|vision+map",
"valid": true, "reason": null, "inputs": {"vision": "ok", "vehicle": "ok"},   // the integrator's verdict; false: no line is present, the lines are unknown
"ego": {"speed_mps": 12.1, "yaw_rate": 0.004, "source": "vehicle|vision"}   // the integrator's ego motion of the frame; yaw rate positive RIGHT here (the device frame)
```
`seq`, `ts_mono_ns`, `ts_real_ns` are those of the `road` message.

* **Presence.** `conf` is a 0.5 s low-pass of the frame probability; a line appears when `conf` reaches 0.45
  and disappears when it falls under 0.2 (about 0.35 s to appear at prob 0.9, about 0.9 s to go). While a
  present line gets no usable frame (prob under 0.2) its geometry is held (`mode: "coast"`), for 1.5 s at
  most. A road edge has no probability in the model output; its pseudo probability is 1 when the network
  places the edge within 0.6 m (`y_std` over 10-40 m) and 0 at 2.5 m (snow, a wide junction).
* **Geometry.** One scalar Kalman filter per line and distance, a random walk in the car's frame, with the
  network's own `y_std` as the measurement noise (0.3 x for a lane line, 1.0 x for a road edge, floors 0.05 /
  0.25 m): far points are smoothed harder than near ones and a road edge much harder than a lane line. A
  frame that does not fit the track (more than 4 sigma and 0.8 m over 5-45 m: the network relabelled its
  lines at a lane change or a junction) is not averaged in; after 0.2 s and two such frames that agree the
  track restarts on them, so nothing between two lanes is reported. An ego-motion compensation (slide by the
  distance travelled, turn by the frame's yaw) is implemented and off: on the rides it made the tracked lines
  trail the raw ones more.
* **Map prior** (soft, optional). With a `map` message that is available, younger than 3 s, matched with
  confidence >= 0.5 and a lane count that is tagged (not assumed), the count shifts the thresholds of the two
  outer lines: `unlikely` (a single-lane road: both; one lane in the own direction of a two-way road: the
  right one) needs `conf` 0.70 to appear and goes at 0.28, `expected` (the oncoming lane's far line on that
  two-way road) appears at 0.325 and goes at 0.12. The inner lines and the edges are never touched by the map,
  strong camera evidence always wins, and without a position source `prior` is null and every threshold is
  the neutral one. On the car there is no live GNSS fix yet, so this path has run on tests and with a constant
  synthetic prior only.

Constants: `DEFAULT_CFG` in `lanes.py`, overridable per key in `<config dir>/lanes.json`. Replay and
measurement on recorded `road` lines: `tools/lanes_replay.py ROAD.jsonl[.gz] [--every N] [--lanes-total N
--lanes-forward N] [--out LANES.jsonl]`.

### Ego lane (`chris-integrator/chris_integrator/trackers/egolane.py`, lanes L20 and L22; inside `fused.lanes`)
A discrete Bayes filter (an HMM) over k, the number of own-direction lanes to the right of the car, run on
every `lanes` message (10-14 per second). Index from the left is `forward - 1 - k` once the map gives the own
lane count. Evidence per frame, read at 8-20 m:

* the right road edge's distance from the ego lane's right line (one-sided: an edge well inside the k-th lane
  rules k out, one beyond k lanes says little, a shoulder or verge lies there; the model places edges nearer
  than the lane geometry, 2.5 m + about 1.8 m per lane on the labelled rides);
* the far-right line one lane width out (seen in 70 % of the frames when a lane is there, 0-13 % when not).
  Its absence counts only weakly (`absence_trust` 0.2 since L22, 0.4 before: on the car's wide lens and in
  town an unseen far line is common beside a real lane);
* the same on the left, where the lanes are `forward - 1 - k` own plus the oncoming ones of a two-way road;
* the map's tagged lane count (`map.road.lanes`, match >= 0.5) as the support of k;
* **lane changes** (L22): the centre of the tracked ego lane moves to one side and comes back on the other
  within 1.2 s by at least 0.38 lane widths (the car nears a line, the network relabels its lines). The
  direction is the side the centre was on before: lines moving right = the car goes left. `ego.turn_signal` in
  that direction makes the change more likely. (Until L22 a one-frame jump of the raw centre was looked for; on
  the labelled rides no real lane change produced one, and its direction was inverted.)
* **a change of the map's lane count** (L22): the lane was added or removed on the right or on the left; the
  count does not say which, so the estimate is split half and half (k moves with a lane on the right, stays
  with one on the left) and the following seconds of camera evidence settle it. With painted turn arrows on
  the new stretch (`map.road.turn_lanes`) the side is known when the new lanes at one edge only turn (a turn
  pocket, an exit lane): 0.85 to that side. A count that is unknown for up to 10 s in between (a junction, a
  doubtful match) does not hide the change. Before L22 the lane was always taken to be added on the left, so
  an exit lane opening on the right of an expressway left the estimate confidently wrong for seconds.

Consecutive frames see the same road, so their evidence counts as one observation per 0.5 s; the distribution
diffuses by 1 % per second (a change nobody saw). `ego_index` only when one lane holds 0.8 (`commit_p`; 0.9 when the lines are the vision road model's, `egolane.PROFILES`, lane L27: at 0.8 the filter was wrong and confident on 5.5 % of the labelled frames against supercombo's 3.4 %, at 0.9 on 3.3 % with 9.9 % unknown against 6.6 %; 0.75
until L22): "unknown" is a valid answer, a wrong confident one is not. Constants: `DEFAULT_CFG` in
`egolane.py`, overridable per key in `<config dir>/egolane.json`. Cost 0.05 ms per frame on the laptop (the
`lanes` tracker: 0.38 ms). Replay and score against hand labels:
`tools/egolane_replay.py ROAD_OR_SESSION --labels LANES.csv [--clock ride|session] [--map-count label|recorded|none]`.
Wrong-and-confident share on the labelled frames (the map count as a tagged map would give it; before L20 /
L20 / L22): night motorway 1.4 / 0.9 / 0.3 %, night expressway with tunnels 1.5 / 3.8 / 1.5 %, city arterials
19.1 / 7.7 / 4.6 %, the car's camera (6 sessions) 21.4 / 11.1 / 5.3 %; right share of all frames on the car's
camera 72.6 / 78.2 / 79.7 % (docs/lanes/L20-ego-lane-junctions.md, docs/lanes/L22-v2-integration.md).

### Road geometry (`chris-integrator/chris_integrator/trackers/roadgeom.py`; the geometry fields of `view.road`)
Until roadsense 2.0 this was a topic of its own (`roadgeom`, `/v1/roadgeom`). It is display content (the
camera and the map joined by a low-passed fit, shown after a second of agreement), so since 2.0 it exists only
inside the display channel: the fields below are fields of `view.road` (next section), computed per road frame.

`roadgeom.py` joins everything that is known about the shape of the road ahead into one geometry in the car's
frame (x forward from the camera, y right, metres): the camera's lane as far as the camera knows it, the map's
road beyond, the side roads of the junctions ahead and a roundabout as a ring (lane L17,
docs/lanes/L17-road-geometry.md). `lanes` stays the camera's own result and `map` the map's.

```
"source": "vision" | "vision+map",
"lane_width_m": 3.4,
"vision": {"from": "lanes|line|plan|none", "reach_m": 99.2},     // what the camera's centre line is made of and how far it is used
"motion": {"source": "vehicle|vision", "speed_mps": 13.1, "yaw_rate_rps": 0.004},
"map": {"used": true, "why": null,           // why not: no_map, no_position, off_road, low_match_confidence, position_accuracy, no_geometry, stale, advance, no_fit, disagree, off_path
        "match_confidence": 0.93, "age_s": 0.31, "advance_m": 4.1, "hacc_m": 2.9, "state": "good",
        "synthetic": false,                  // the position is made up (map.input.synthetic): a consumer in a car must not draw the map part
        "fit": {"dy_m": 0.21, "dyaw_deg": -0.8, "rms_m": 0.12, "ok": true, "to_m": 50.0},   // this frame's fit, or {"held_s": 2.1}
        "align": {"dy_m": 0.18, "dyaw_deg": -0.7},     // the low-passed fit that is applied
        "way_offset_m": 1.7, "off_path_deg": 0.4, "ring_slide_m": 0.0},
"centre": {"s": [..], "x": [..], "y": [..],  // the ego lane's centre line as a polyline, s = distance along it
           "conf": [..],                     // camera part: from the tracker's sigma; map part: confidence of the path up to the point
           "src": "vvvvvvbbbbmmmm",          // per point: v camera, b camera blended into the map, m map
           "vision_to_m": 99.2,              // the camera's part ends here
           "end": "horizon|path_end|dead_end|no_way_on|intent"},
"branches": [ {"dist_m": 71.0, "pts": [[71.2,-1.7],..], "class": "tertiary", "minor": false, "exit": true, "entry": true,
               "lanes": 2, "ring": false, "side": "left|right", "way": false} ],     // way: chosen by the driver's turn signal
"ring": {"centre": [44.1,-1.6], "radius_m": 14.1, "lanes": null, "on_ring": false, "entry_m": 30.2, "exit_m": 74.3},
"way": {"source": "map", "confidence": 0.58} | {"source": "intent", "turn": "right", "branch": 1},
"intent": null | {"turn": "right", "source": "turn_signal", "at_m": 33.4}
```
Without a usable map the object carries the camera's part alone (`source: "vision"`, `map.used: false` and the
reason, no branches, no ring). (`junction` is no longer a four-field head here: `view.road.junction` is the
whole junction view.)

* **Camera part.** The centre of the ego lane: the middle of the two tracked inner lines (`lanes`), one line
  and the lane width, or the model's planned path (`road.plan`) when no line is present. It is used as far as
  the tracker's own sigma stays under 0.9 m, 100 m at most.
* **Map part.** `map.ahead.geometry` arrives twice a second in the frame of the matched position. Between two
  messages it is carried by the distance driven (vehicle speed when a host feeds it): the car is taken to be
  on the most likely path, that far along it, with the heading of the position filter carried on by the yaw
  rate (`ego.yaw_rate_rps`, else the model's). On a ring the car is slid along the path, by at most 14 m, to
  where the ring runs in the car's direction: 5 m of position error along a ring of 14 m radius are 20
  degrees, and the heading is known to 2.
* **Joining.** What the map cannot know (where in the carriageway the car is; a few metres and degrees of
  error) is taken from the camera: the offset and the angle between the camera's centre line and the map's path
  over 4-50 m are fitted every frame, low-passed (0.8 s) and applied to all map geometry. The fit must stay
  within 7 m, 9 degrees and a residual of 0.5 m + 1.2 % of its length; the map part appears after 1 s of
  agreement and goes after 0.5 s without (`why: "disagree"`), or when the car's heading leaves the path's
  direction by more than 35 degrees (`off_path`). Without a camera centre to compare with (lines gone in a
  junction, on a ring) the last fit is held for 150 m. Side roads and the ring are moved from the way's centre
  line to where they are for a car in its lane (`way_offset_m`, from the lane count and the lane seen to the
  right). The centre line follows the camera up to `vision_to_m` (blended into the map over the last 15 m)
  and the map's path beyond; before a ring the camera's part ends 3 m before the entry.
* **Honesty.** `centre.conf` of a map point is the confidence of the most likely path up to it: it steps down
  at every junction where the way on is a guess, and a consumer should stop drawing where it gets low (the
  reference consumer stops at 0.5). `way.source` says whether the way through the next junction is the map's
  guess or the driver's signal.
* **Driver intent** (optional). With `ego.turn_signal` `left` / `right` (a host that reads the turn-signal
  lamps feeds it with `motion`), the nearest side road on that side within 150 m that can be driven into is
  the way on (service roads last): `branches[].way`, `way.source: "intent"`, and the centre line ends at that
  junction. On and before a roundabout `right` is the next exit; `left` is the first exit beyond the one the
  map's path takes, reached on round the ring (a convention of how drivers signal, not a rule). Without the
  signal nothing changes.

Cost on the laptop: 0.09 ms per frame without a map, 0.36 ms with one, plus 0.17 ms per `map` message; on the phone: see
docs/lanes/L17-road-geometry.md. Constants: `DEFAULT_CFG` in `roadgeom.py`, overridable per key in
`<config dir>/roadgeom.json`. Tools: `tools/roadgeom_replay.py SESSION.jsonl.gz` (a recorded drive through the
tracker and the fusion), `tools/roadgeom_drive.py --network tour [--turn T0:T1:left|right] [--out REC]` (a
synthetic drive over a made-up network or a pack with a synthetic camera, compared with the truth; the
recording carries `view` messages).

### consolidated topic (chris-integrator): `{"type":"view", ...}` (display channel, roadsense 2.0)
`chris-integrator/chris_integrator/trackers/view.py` (objects, lines) and `roadview.py` (`road`): one message per road frame (10-14 per
second), after `lead` and `lanes` of that frame; 1.5 kB without a map, 3-4 kB with the road picture; `/v2/view`.
Added 2026-11-18 (docs/lanes/L19-view-channel.md; `road`: docs/lanes/L22-v2-integration.md). For any consumer
that draws (a car display, a HUD, a dashboard): objects that keep their identity, move smoothly between two
detections, fade in and out, and carry a velocity so the consumer can extrapolate to its own render time.
Derived from the measurement topics, never fed back into them; nothing in it belongs to one display.

```
"x": [0, 5, 10, 15, 20, 25, 30, 40, 50, 60, 70, 80, 100, 120],   // distances of every line, m
"ego": {"speed_mps": 12.1, "yaw_rate_rps": 0.01, "source": "vehicle|vision|none"},   // the integrator's; yaw positive left
"integrity": {"valid": true, "reason": null, "inputs": {"vision": "ok", "vehicle": "ok"}},   // false: the camera is not believable
                                 //   now (the reason says why): no lead, no lines, no path in this message; a display shows that state
"objects": [ {"id": "lead:12" | "light:7" | "sign:31",   // stable while the measurement track lives
              "kind": "vehicle|traffic_light|sign", "role": "lead|ego|other",   // ego: the ego traffic light
              "x": 34.2, "y": 1.9,                        // car frame (x forward from the camera, y right), m
              "vx": -12.1, "vy": 0.0,                     // m/s relative to the car: x(t) = x + vx (t - ts)
              "sx": 2.1, "sy": 0.5,                       // 1 sigma of the drawn position (null: not known)
              "age_s": 4.2,                               // since the object appeared
              "meas_age_s": 0.12,                         // since its last measurement (> 0: predicted)
              "phase": "appear|steady|coast|leave",       // coast: no measurement for 0.5 s; leave: fading out
              "alpha": 1.0,                               // suggested opacity of the fade, 0..1
              "attrs": {...},                             // light: state, flashing, relevance, ego_state, held, amb, alt,
                                                          //   conf, agree, dist_m (the measured one); sign: cls, score;
                                                          //   lead: dist_m, dist_std_m, v_rel_mps, a_lead_mps2, time_gap_s,
                                                          //   ttc_s, standing, departed, mode (the `lead` topic's values)
              "box": [x0, y0, x1, y1]} ],                 // last image box (frame pixels) for lights and signs, else null
"lines": [ {"id": "lane:0".."lane:3" | "edge:0" | "edge:1", "kind": "lane_line|road_edge",
            "y": [..],                                    // at x, right positive; null beyond reach_m
            "dy": [..],                                   // lateral rate m/s per point
            "reach_m": 80.0, "conf": 0.83, "age_s": 12.3,
            "phase": "appear|steady|coast|shift|leave",   // shift: blending to a restarted track (other marking)
            "alpha": 1.0} ],
"lines_source": "model",                                  // "model": the road model's tracked lines (`lanes`); "laned": the
                                                          //   display lane source's; "laned+model": both (below)
"lines_scale": null,                                      // factor applied to the display lane source's metres (null: no source)
"road_model": "vision",                                   // the road model of the frame
"path": {"y": [..]},                                      // the model's planned path at x, as measured
"road": {...}                                             // the road as one picture (below)
```
`seq`, `ts_mono_ns`, `ts_real_ns` are those of the `road` message; every position refers to that frame time.

| element | display filtering | price |
|---|---|---|
| traffic lights, signs | one Kalman filter per track in the car's frame, carried between detections by the car's own motion (`ego` speed and yaw rate when a host feeds them, else the model's) and corrected by each detection with its real uncertainty (light distance from the housing size: 25 % at 1 sigma, lateral 0.4 m + 3 %; sign distance from a 0.7 m sign: 30 %); a detection beyond 3 sigma is down-weighted; the drawn position eases to the filter's over 0.25 s at most 1 m/s + 10 % of the distance per second. `lights.lateral_m` (offset from the planned path) is turned back into the car's frame with the path. A light the provider only coasts is not a measurement. Leaves 1 s after its last detection (fade 0.3 s) or when it passes the camera | the drawn distance trusts odometry between detections; a biased housing-size distance is corrected over seconds, not frames |
| lead | the `lead` topic's track: carried by its relative speed, corrections eased over 0.1 s + 0.01 s per metre (0.15 s at 5 m, 0.5 s at 40 m); fade in 0.3 s, fade out 0.4 s at its extrapolated place after the track ends | drawn distance trails the tracker by up to the ease time at long range; `attrs.dist_m` is the tracker's value |
| lines | the `lanes` topic's geometry, two first-order stages whose time constant grows with the distance (`line_tau_s` + `line_tau_rel` x: 0.1 s at the car, 0.16 s at 10 m, 0.28 s at 30 m); a restart of the track on another marking (the near part moves by more than 0.8 m in one frame) is blended linearly over 0.4 s; fade 0.3 s when a line comes or goes; `reach_m` where the tracker's sigma passes 1 m; `dy` the rate of each point | two to three frames behind `lanes` at 10 m, about five (0.4 s) at 30 m: smoothness is chosen over lag on the display channel. Carrying the lines by the car's own motion (slide, turn) is implemented and off (`line_motion`): on the recorded drives it predicts the next frame no better than "the line stays where it is in the car's frame" (a line at 30 m: 0.280 m rms against 0.286 m in turns, 0.137 against 0.136 m elsewhere), and with the uncalibrated camera it adds a bias of 0.03 m per frame |

On the 2026-11-17 day drive (29 min, `tools/view_replay.py`): the ego light's drawn distance step per frame after
the car's own motion 1.96 -> 0.12 m rms, steps over 2 m 129 -> 0, lateral 0.32 -> 0.07 m; lane lines and edges
that pop on or off without a fade 106 -> 2; the lead's step per frame after its relative speed 0.27 -> 0.09 m rms;
a lane line's step per frame at 30 m 0.089 -> 0.068 m rms, steps over 0.3 m 119 -> 72 (the reference consumer,
which moves the lines with their own rate between two messages, draws 6 such steps on that drive).
Cost: 0.15-0.18 ms per frame on the laptop without the road picture.

**`view.road`: the road as one picture** (`chris-integrator/chris_integrator/trackers/roadview.py`). Everything a consumer needs to
draw the road and what lies ahead on it, from one message of one frame; each part says where it comes from
(`provenance`, `source`, `centre.src`), how sure it is, and whether it rests on a weak position
(`approximate`). Until 2.0 a consumer combined `roadgeom` (geometry), `map.ahead.junction_view` (junction) and
`fused.lanes` (lane boxes) and applied its own trust thresholds.

```
"road": {
  "source": "vision" | "vision+map",
  "position": {"state": "good|dead_reckoning|...", "available": true, "reason": null,   // null without mapd
               "hacc_m": 6.1, "match_confidence": 0.62, "match_confidence_smoothed": 0.58,
               "trusted": true,          // the map layer may be drawn (below); false: only the camera's part is given
               "held": false,            // the map dropped out for a moment: the last trusted message is shown
               "approximate": false, "why": [],     // why: dead_reckoning, position_accuracy, held
               "synthetic": false,       // the position is made up (map.input.synthetic): a consumer in a car must not draw the map parts
               "age_s": 0.21},
  // the geometry (previous section): the ego lane's centre from the camera near the car, the map beyond
  "lane_width_m": 3.4, "vision": {...}, "map": {"used": true, "why": null, ...},
  "centre": {"s","x","y","conf","src","vision_to_m","end"},
  "branches": [ {..., "approximate": false} ], "ring": {..., "approximate": false} | null,
  "way": {...}, "intent": null | {...},
  // the junction or roundabout ahead: mapd's junction view (kind, control, approach, arms, ring; "mapd" above)
  "junction": {"kind": "junction|roundabout", "dist_m": 171.0,       // carried to this frame by the distance driven
               "confidence": 0.58, "control": "signals", "approach": {...}, "arms": [...], "ring": null,
               "provenance": "map", "approximate": false, "why": []} | null,
  "lanes": {"count": 4, "forward": 3, "backward": 1, "count_source": "map:tagged",    // null without a trusted map
            "ego_index": 1,              // the lane to light, from the left; null = not known
            "likely_index": null,        // with ego_index null: the most likely lane when it holds 0.45
            "ego_from_right": 1, "probs": [0.05, 0.9, 0.05], "probs_from": "left|right", "confidence": 0.9,
            "oncoming_left": false, "provenance": ["vision", "map:tagged", "lane_change"], "approximate": false,
            "count_held": false,         // L36: the map counts other lanes now; the old count (and its lit lane) is shown until the new one held 2 s
            "turn": {"lanes": [["left"], ["through"], ["through","right"]],   // painted arrows (map.ahead.turn_lanes)
                     "arrows": ["l", "t", "tr"],     // the same as letters per lane: l left, t through, r right, u turn back, m merge
                     "count": 3,                     // always = forward above (L36): arrows for other lanes are not given
                     "dist_m": 0.0, "junction_m": 60.0,   // never grow while it is the same stretch (L36)
                     "path": [false, true, true],    // lanes that lead where the most likely path goes; null = not known
                     "path_source": "map",           // L36: the map's most likely path, not a route
                     "here": true,                   // the car is on the stretch
                     "held": false,                  // L36: mapd does not announce it this frame; kept (up to 2 s), carried by the distance driven
                     "provenance": "map:turn_lanes", "confidence": 0.62} | null},
  "route": null}                 // L36: a navigation route being followed; null: no route source exists (yet). A future
                                 // route feed gives {"active": true, ...}; consumers draw the turn arrows only then
```

| part | where it comes from | display policy applied here |
|---|---|---|
| `position.trusted` | `map.match.confidence`, `map.available` | a `map` message is usable when available with a match confidence of at least 0.5 (the rule of `fused`); the confidence of the usable ones is low-passed over about 2 s and the layer is trusted from 0.55 (`trust_on`). A message that is not usable (the confidence dips where a side road branches off; off the road for a moment) is skipped and the last trusted one is held for 2.5 s (`held`, distances carried on; `approximate` after 1 s); after 2.5 s without a usable message the layer is gone and trust is earned again. A gap in the camera frames (over 1 s) keeps a `map` message of the last second, re-based by the speed (L36; until then the layer was gone for up to 0.5 s after every frame gap) |
| `centre`, `branches`, `ring`, `way` | camera lines / planned path, `map.ahead.geometry` | the join of the previous section (fit, 1 s of agreement, withheld on disagreement); branches and ring carry `approximate` from the position |
| `junction` | `map.ahead.junction_view` | given when the path up to it is likely (0.5; always on a ring) and the position error allows it: `hacc_m` within 20 m, or within 40 % of the distance. `approximate` when dead-reckoned, held for more than 1 s, `hacc_m` over 20 m, or over 15 % of the distance: draw it as "about there", without an exact distance. The distance is carried by the distance driven between two `map` messages; a frame older than the newest `map` message (the camera frame lags it by the model's latency) uses that message as it is (until L36 such frames had no junction and no turn arrows: one or two blank frames per map message) |
| `lanes` count | `map.road.lanes` of the trusted match | a new count is shown after it held 2 s (`count_settle_s`; L36: short segments tagged with another count, a match hopping between parallel edges); meanwhile the old count, its lit lane and probabilities stay and `count_held` is true. `count_source` says tagged or assumed |
| `lanes.ego_index` | the ego-lane filter (`fused.lanes`, measurement) | the committed lane stays lit until another lane is committed or its own probability falls under 0.6 (`lane_release_p`): no flicker at the commit threshold. `likely_index` from 0.45 |
| `lanes.turn` | `map.ahead.turn_lanes` | carried by the distance driven; given only for as many lanes as `forward` (L36); a new stretch after 0.3 s of sightings (`turn_on_s`, timed also while the count still settles), kept through a drop-out of 2 s (`turn_hold_s`, `held: true`), its distances never growing while it is the same stretch (same arrows, junction within 25 m), the next stretch replacing it without a blank. `path` is the map's guess (`path_source: "map"`) |
| `route` | none yet | null: no navigation source. Reserved for a route feed (`{"active": true, ...}`); the reference consumer draws turn arrows only while it is active (owner's ruling 2026-11-19) |

Without mapd, without a pack or without a position `road` still carries the camera's part (`source: "vision"`,
`position` null or untrusted, `junction` null, lane counts null, the filter's `probs` from the right).
"Unknown" is a value here: a part that cannot be trusted is null, never a guess drawn as a fact. Constants:
`<config dir>/view.json` (keys of `view.DEFAULT_CFG`; the key `road` holds overrides of
`roadview.DEFAULT_CFG`). Replay: `tools/view_replay.py SESSION [--map MAP.jsonl]` prints how often each part is
given on a recorded drive.
