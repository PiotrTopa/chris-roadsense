# L13: map layer (mapd, position input, map packs, fusion)

Built 2026-11-15 on the dev box (P52). Nothing was deployed to the phone and no real position source
exists yet: every number below comes from a synthetic position feed (`tools/synth_drive.py`) over the
real Poland pack, or from unit tests on a hand-made network. Contract: docs/ARCHITECTURE.md ("mapd",
"api-derived topic fused"); packs: docs/MAPS.md.

## What exists

| part | where |
|---|---|
| map daemon (filter, matcher, context, sockets) | `mapd/roadsense_mapd/` (`posfilter.py`, `matcher.py`, `context.py`, `engine.py`, `__main__.py`, `rsmap.py`, `limits/pl.json`) |
| pack format + reader/writer | `mapd/roadsense_mapd/rsmap.py`, docs/MAPS.md |
| pack tool | `scripts/roadsense-maps` -> `tools/roadsense_maps.py` (fetch, build, info, verify, install) |
| position input API v1 | `<rundir>/position.sock`, `POST /v1/position`; reference client `tools/position_client.py` |
| synthetic source | `mapd/roadsense_mapd/sim.py` (route, vehicle, noise, dropouts), `tools/synth_drive.py` (live / offline / JSON lines; demos `warsaw-s2-tunnel`, `warsaw-city`, `krakow-city`, `opole-rural`) |
| fusion | `api/roadsense_api/fusion.py`; topics `map`, `fused`; `/v1/map`, `/v1/fused`; status page and `/preview` HUD line |
| deployment | `systemd/roadsense-mapd.service` (in `roadsense.target`), `config/mapd.json`, `scripts/install-phone.sh` |

## Checks (commands and results)

```sh
python3 -m unittest discover -s mapd/tests -t mapd            # 50 tests: 37 unit, 6 daemon process, 7 on the Poland pack (skipped without one)
.venv-models/bin/python -m unittest discover -s api/tests -t .   # 88 tests: 69 existing + 16 fusion rules + 3 end to end (fake camera daemons + real mapd + api)
tools/synth_drive.py --pack ~/roadsense-maps/packs/poland-20261114.rsmap --demo warsaw-s2-tunnel --offline
```

| scenario (Poland pack; GNSS 1 Hz with 3 m noise correlated over 20 s; motion 20 Hz with 1 % speed scale error and 0.15 deg/s yaw bias; 3 seeds each) | result |
|---|---|
| S2 Warsaw, 9.5 km incl. the 2.3 km Ursynów tunnel, fix lost in the tunnel; motion as yaw rate, as steering angle, as wheel speeds | tunnel announced 1342 m ahead with its length (2335 m); dead reckoning for 108 s, on the route 100 % of it; position error in the tunnel max 16-26 m (the 1 % odometer scale error), reported `hacc_m` 42 m at the exit; exit distance counts down to the portal; `good` with the first fix after the exit; whole drive on the route 99.2-99.9 % |
| the same tunnel without any motion input | 5 s coasting, then `unknown`: map layer unavailable (`position_unknown`) until the fix returns |
| Warsaw centre, 5.4 km of arterials (98 edges) | on the route 99.6 %; with a 90 s fix dropout: dead reckoning on the route 100 %, max position error 4-12 m |
| Kraków, 4.2 km (70 edges) | on the route 99.8 % |
| Niemodlin - Nysa, 30 km rural (217 edges) | on the route 99.9-100 %; with a 300 s fix dropout (5.5 km): dead reckoning on the route 100 %, max position error 30 m; limits from `maxspeed` and from the implicit rules (built-up 50, outside 90), built-up changes and curves in the look-ahead |
| fixes pushed 15 m sideways for 12 s (multipath) | the match stays on the driven route (> 90 % asserted) |
| position error with a fix | mean 2.3-5.5 m, max 6-14 m (the simulated GNSS error is 3 m, correlated) |

mapd cost on the laptop (i7-8850H, one core), live process fed in real time with the Poland pack:
0.6 % CPU (`/proc/<pid>/stat` over 105 s; 1.2 % before the candidate pre-filter and the cached edge
bearings), 43 MB resident, 0.7 ms per step, 4 kB per `map` message, 7-22 tiles decoded per drive.
Not measured on the phone.

## Findings

* The recorded rides (`docs/RIDES.md`) carry no position: video plus yt-dlp metadata with, at most, a place
  name; `s7-elblag-lowsun` shows a GPS bar burnt into the pixels. No ride replays the map layer. The demo
  `warsaw-s2-tunnel` drives the road of the ride `tunnel-pow-warsaw`, not aligned with it in time.
* Polish expressways are mapped as `highway=motorway` with S refs (S2 has no `motorroad` tag): the implicit
  limit takes the expressway rule (120 / 100) when all refs are S-numbers, else 140.
* Map feedback while dead reckoning has to be gated (match within 10 m and 10 degrees of the filter):
  ungated, a wrong branch at a fork dragged the filtered position 800 m away in 60 s.
* A branch that leaves at a shallow angle (exit ramp) captured the match for 8 s on S2 with the fix good;
  a branch now has to be clearly nearer than the continuation of the same road, and the alternatives of the
  last fork stay reachable for 150 m.

## Open

* No real feeder: head-unit GNSS and Prius CAN motion are not connected; latency, accuracy fields and the
  sign conventions of the real sources are untested.
* Not deployed; phone CPU / memory / tile-decode hiccups (a dense city tile decodes in 8 ms on the laptop)
  unmeasured.
* Fusion rules are tested on fabricated messages and on fake camera daemons, not on a drive with a real
  camera and a real position together.
* Built-up state is inferred from landuse where untagged; D-42 / D-43 nodes unused. Turn restrictions,
  turn lanes, speed cameras, fuel stations are not in the pack.
* The most likely path does not use blinkers or a navigation route; `path_confidence` drops fast in dense
  street grids.
* Vehicle constants are nominal; no online calibration of the odometer scale (the along-track error in a
  tunnel is that scale error).
* The simulated vehicle follows the centre line of the way; lane offsets, GNSS outliers of real urban
  canyons and real tunnel entries (fix degrading before it is lost) are not modelled.
