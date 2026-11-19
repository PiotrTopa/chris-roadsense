# L36 turn boxes: no turn arrows without navigation, no flicker of the lane boxes

Lane of 2026-11-19 (Opus). Branches: roadsense `v2-l36` (this file), car computer `l36-turn-boxes`. No deploy;
phone and car untouched (recordings copied read-only).

## Complaint and ruling
Drive of 2026-11-19 (roadsense 2.0, new MFD): "nonsense boxes with directions before crossovers with no
navigation enabled", "what's worse, they flicker". Ruling: "remove the boxes with direction with no nav
active; lanes arrows flicking pls fix".

## Route source: none
Nothing in either repository follows a navigation route. `view.road.way` is the map's most likely path or
the driver's turn signal (`way.source: "intent"`, roadgeom.py), `lanes.turn.path` is the map's guess, the
car's backend has only a synthetic position route (position_feeder.py, a test drive, not navigation).
So, since L36:

* roadsense: `view.road.route` is a new reserved key, `null` (no route source). A future route feed gives
  `{"active": true, ...}`. `lanes.turn.path_source: "map"` says what `path` is.
* MFD: one condition, `roadsense_scene.route_active(view.road)`: true only for `route.active == true`. It
  gates `tl` / `tp` in `road_lanes_to_scene`, so today the arrows are never sent, with this roadsense or an
  older one (no `route` key). The drawing code is unchanged and comes back when a route feed exists.
* Plain lane boxes (count, own lane lit, likely lane outlined, oncoming boxes) stay: they are the MFD's lane
  layout element (docs/MFD_MAIN_SCREEN.md 6.5, since L20), shown with or without a junction ahead; the arrows
  are an L22 addition inside them. The owner can overrule with the MFD setting `map_lanes` = false.

## Causes of the flicker (file:line at main f3ea71c / car main aa8e0aa)

| source | cause | effect |
|---|---|---|
| roadsense `api/roadsense_api/roadview.py:147` with `:100-102` | the junction and the turn arrows were given only for `0 <= adv`, adv = odometer now minus the odometer at the map message. `on_map` places a map message at `odo + v * lag`, lag = its time minus the last frame's time, which is positive: mapd stamps the message when it sends it, the camera frame is older by the model's latency. Every frame between a map message and the camera catching up got `adv < 0` and lost `junction` and `lanes.turn` | 381 of 391 turn drop-outs on 151146, 268 of 277 on 104128 (instrumented replay); 35 on/off per minute at the full view rate on 151146, every second recorded view (834 of 2545 had a turn) |
| roadsense `roadview.py:242-258` (`_turn`) | no presence hysteresis, no hold, distance re-read per message (a match stepping back grows it), and a stretch with another lane count than `forward` passed on (3 arrows, 1 lane) | each map drop-out = an on/off; a consumer that drops mismatched arrows (the MFD) switches them off and on when the count changes |
| roadsense `roadview.py:112-113` | a gap in the camera frames (> 1 s, frequent on 113201) dropped the map message; the layer came back only with the next one | boxes off for up to 0.5 s after every frame gap |
| roadsense `roadview.py:204-205` | the lane count was the map's of the current message | count hops on short segments tagged with another count (113201: 2 -> 1 -> 2 within 1.4-3 s) |
| car `backend/roadsense_scene.py:813-816` | arrows passed on only when `len(arrows) == fwd` and within 120 m, no route condition | arrows from 120 m before every tagged junction; on/off with every count or distance change |
| car `ui/scene/mapinfo.py:679-682` | `MapDirector` drew the boxes of the current scene only: a scene without lanes (roadsense's trust off for a moment) removed them at once | single-scene gaps of the boxes |

The lane count and trust flicker on 151146 is mostly the map's real tagging (segments of 4.6 s and longer) and
match losses of seconds, with the camera knocked about 8 degrees off its calibration on that drive.

## Changes

roadsense (`api/roadsense_api/roadview.py`, display policy only; measurement topics untouched):
* a frame older than the newest map message uses it as it is (`adv` clamped to 0 within +-60 m);
* a frame gap keeps a map message of the last second (`gap_keep_s`), re-based by the speed;
* lane count: a new count is shown after it held 2 s (`count_settle_s`); meanwhile the old count, its lit
  lane and probabilities stay, `lanes.count_held: true`;
* `lanes.turn`: only for as many lanes as `forward`; a new stretch after 0.3 s of sightings (`turn_on_s`;
  timed also while the count settles, so it comes with the new count); kept through a drop-out of 2 s
  (`turn_hold_s`, `held: true`, carried by the distance driven; while a successor waits, up to 2 s longer);
  the same stretch (same arrows, junction within 25 m, `turn_same_m`) never gets further away; it goes when
  the map stops announcing it and the odometer is 5 m past the junction (not on the odometer alone: it runs
  ahead of the match), the next stretch replacing it without a blank;
* `view.road.route: null`, `lanes.turn.path_source: "map"`;
* tests: 7 new in `api/tests/test_roadview.py`; `tools/turn_flicker.py` (the measurement below).

car computer: see the commit on `l36-turn-boxes` (route condition, `MapDirector` hold 1 s for the boxes,
0.3 s on / 1 s hold for arrows, demo shots 86/87, docs/MFD_MAIN_SCREEN.md 7.5, `tools/mfd_chain_replay.py`
counts on/off on the glass).

## Before / after
Drives of 2026-11-19 from the phone's recorder (healthy: 104128, 113201; 151146 = camera off calibration;
065629 has no map position and shows nothing). Per minute of session time.

`view.road` at the full view rate (replay through the api's derived topics):

```
python3 tools/turn_flicker.py REC/20261119-151146 --api ~/projects/roadsense/api   # before (main f3ea71c)
python3 tools/turn_flicker.py REC/20261119-151146                                  # after (this branch)
python3 tools/turn_flicker.py REC/20261119-151146 --recorded                       # the 2/s recorded view
```

| per minute | 104128 before | after | 113201 before | after | 151146 before | after |
|---|---|---|---|---|---|---|
| `lanes.turn` on/off | 15.74 | 0.28 | 6.21 | 0.34 | 34.79 | 0.67 |
| drawable arrows on/off (count = forward, within 120 m) | 7.29 | 0.23 | 0.75 | 0.34 | 18.29 | 0.62 |
| boxes (trusted, 2+ lanes) on/off | 0.28 | 0.28 | 0.59 | 0.25 | 0.98 | 0.94 |
| lane count changes (boxes on) | 0.20 | 0.20 | 0 | 0 | 0.22 | 0.22 |
| ego lane on/off | 0.68 | 0.57 | 0.59 | 0.42 | 0.13 | 0.13 |
| `position.trusted` on/off | 1.11 | 1.08 | 1.59 | 1.34 | 1.07 | 1.03 |

The remaining arrow on/off are real stretches coming and going (one per junction approach, each lasting
7 s or more; the 2 "distance up-jumps" are the next junction replacing a passed one). The recorded 2/s view
showed 4.11 turn on/off per minute on 151146 (the aliased picture of the 35 at full rate).

On the glass (car `tools/mfd_chain_replay.py`, the whole chain: api -> SceneBuilder -> MapDirector):

| per minute | 104128 | 113201 | 151146 |
|---|---|---|---|
| turn arrows on/off: car before + roadsense before | 4.93 | 0.54 | 13.96 |
| car before + roadsense L36 | 0.21 | 0.27 | 0.61 |
| car L36 (no route), either roadsense | 0 | 0 | 0 |
| lane boxes (2+ lanes) on/off, before -> after | 0.31 -> 0.31 | 0.54 -> 0.40 | 1.13 -> 1.04 |
| box segments shorter than 3 s (count), before -> after | 1 -> 1 | 4 -> 1 | 3 -> 2 |
| box count changes, before -> after | 0.18 -> 0.18 | 0 -> 0 | 0.22 -> 0.22 |

Cost: the api's derived topics 0.554 -> 0.574 ms per frame on the laptop (`tools/view_replay.py
REC/20261119-113201`, within run-to-run noise).

## Not verified
* Nothing ran on the phone or in the car; the numbers are replays of recorded drives.
* The replay feeds the map messages in the order they were recorded; on the phone the api sees them in
  arrival order too, so the `adv < 0` cause is expected to be the same, not measured live.
* The MFD's view of a route (`view.road.route`) is a contract stub: no route feed exists to test it with.
* The remaining box on/off (0.3-1.0 per minute) follow the map trust of roadsense (match lost for seconds);
  not touched here (trust has its own 2.5 s hold, and "unknown" is the honest answer then).
* The api's aiohttp-based tests (test_api, test_map_e2e, test_display_lanes, ...) did not run on this laptop
  (no aiohttp); the 214 others pass.
