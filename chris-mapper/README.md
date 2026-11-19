# chris-mapper

The map box of **chris**, an open perception stack for a car computer. A pose goes in; which road that is, and
what the map says about it, comes out. It estimates no position: where the car is, is the statement of the
component that fuses the sensors (chris-integrator), taken with its uncertainty.

```
 ego_state  (pose: lat, lon, east/north covariance,        map   match        matched point, the road's bearing,
             heading with std, speed, odometer,     ──>               confidence, ambiguity
             freshness of the last absolute fix;         ──>    position     the pose that was used, and its state
             time of validity)                                  road         class, name, ref, lanes, turn lanes, ...
                                                                speed_limit  tagged or implicit (country rules)
                                                                ahead        features, junctions, curves, limit
                                                                             changes, `junction_view`, tunnel
```

chris informs the driver. It does not control the car.

## The box

```python
from chris_mapper.box import MapBox

box = MapBox({"pack": "/var/lib/chris/maps/current.rsmap"})
m = box.on_ego_state(ego_state)        # a `map` message when one is due (every 0.45 s of input time), else None
```

* **In**: `ego_state` messages. Used fields: `ts_mono_ns`; `position` (`valid`, `mode`: `fused` while an absolute
  fix is fresh, `coast` in dead reckoning; `lat`, `lon`, `cov_en`, `fix_age_s`, `local.frame`); `heading`
  (`value` in degrees clockwise from north, `std`, `valid`); `speed.value`; `odometer_m`.
* **Out**: `map`. `match` is geometry of the map (the matched road's point and bearing): the estimator takes
  it as a measurement (cross-track distance to the road = 0, heading = the road's bearing), so the loop
  estimator -> map -> estimator does not feed a pose back to itself. `position` repeats the pose that was used
  with its state (`good`, `dead_reckoning`, `unknown`, `no_source`). `road`, `speed_limit`, `ahead` are the
  map context.
* No sensor input, no position filter, no wall clock: times are the input's. A changed pack file is loaded
  without a restart.

It runs in the integrator's process as a plug-in, or as its own process behind the same contract
(`python3 -m chris_mapper --plugin`: JSON lines on stdin / stdout; the first line declares its inputs and
outputs). It needs nothing of chris-integrator: the contract is the messages.

## Parts

| module | what |
|---|---|
| `box.py` | the box: pose in, `map` out |
| `rsmap.py` | the map pack format (`.rsmap`: tiled road graph with attributes, built from OpenStreetMap) |
| `matcher.py` | map matching: candidates by distance and heading, continuity along the graph, fork memory |
| `context.py` | road attributes, implicit limits (`limits/<country>.json`), look-ahead, junction and roundabout views |
| `sim.py` | routes and a simulated drive on a pack (truth for tests) |

## Tests

```sh
python3 -m pytest -q tests        # a hand-made network written as a real pack; no map data in the repository
```

## Map data

Packs are built from OpenStreetMap extracts by a separate tool; map data is (c) OpenStreetMap contributors,
ODbL. No map data is part of this repository.

## Licence

MIT, see [LICENSE](LICENSE).
