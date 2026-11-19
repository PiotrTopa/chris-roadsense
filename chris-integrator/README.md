# chris-integrator

The data integrator of **chris**, an open perception stack for a car computer (named after St Christopher).
Every sensor of the car goes *in*; one consistent picture comes *out*: where the car is and how it moves, what
is around it, how far each of those statements can be trusted, and why not when it cannot.

chris informs the driver. It does not control the car.

```
 sources (any process, any rate, any latency)              chris-integrator                      consumers
                                                 ┌───────────────────────────────────┐
 chris-roadsense (one per camera):               │ time: clock epochs, per-source    │   ego_state  position, heading,
   road model pose, lead, plan      ───────────> │   delays, out-of-sequence data    │ ─>   speed, acceleration, yaw rate,
 car: wheel speed, gyro, accelerometers,         │ ego estimator: one EKF, full      │      grade, calibration of every
   controls (vehicle input API)     ───────────> │   state + calibration states      │      sensor, per-source health
 GNSS receiver                      ───────────> │ fault detection and isolation     │   tracks     one multi-target list
 map matcher                        ───────────> │ object layer: association, track  │ ─>   in the car frame
 radars, further cameras            ───────────> │   life cycle, lead by the path    │   lead       the vehicle ahead
                                                 └───────────────────────────────────┘
```

The rule of the architecture: **inputs go to the integrator, and only then to any logic.** A consumer never
picks a raw sensor, never guesses whether a value is fresh, and never has to know which sensors a particular
car has.

## What is in it

* **One ego estimate.** An extended Kalman filter over position, heading, speed, acceleration, yaw rate,
  altitude, road grade and the scale of the car's speed signal, plus the calibration states each source
  declares: gyro bias and scale, accelerometer offsets, per-sensor time delays, the camera's mounting angles,
  a receiver's correlated error, a radar's mount misalignment. Measurements are kept in a time-ordered buffer;
  a late one is inserted where it belongs and the filter is re-run from there (the exact answer to the
  out-of-sequence problem); the state for a camera frame is the fixed-lag smoothed one.
* **Honest uncertainty.** Heavy-tailed sources get Student-t weighted updates, errors correlated in time get
  Gauss-Markov states instead of inflated noise, states a measurement cannot tell apart are "consider" states
  (Schmidt-Kalman). The test suite checks NEES against simulated truth.
* **Fault detection and isolation.** Raw measurements of two sources are compared pairwise, never against the
  fused state; with three independent sources of a quantity the culprit is found by vote, also when it is the
  highest-ranked one. Innovation statistics watch the row classes that have no partner. A faulted source
  leaves the estimate at once and everything built on it goes out as *unknown, with the reason*.
* **An object layer.** One track list in the car frame, state `[x, y, vx, vy, ax]` with the car's own motion as
  the input of the process model; sensors are models (measurement function, noise, field of view, clutter);
  global nearest neighbour association, existence probabilities, cross-sensor merging by covariance
  intersection, ghost and stationary marks, and the lead chosen as the nearest confirmed track on the path.
* **Sources as plug-ins.** A sensor is one file: an input description, a measurement model, its mount and
  calibration states. The core knows no sensor. Included: vehicle motion, road-model pose, GNSS, map match,
  the road model's lead, radar (any number, any mount), a further camera's object list.
* **A simulator with known truth** (`chris_integrator/sim`) that emits the same message types the real
  sources send, with noise, bias, latency, drop-outs and faults; and an injector that adds synthetic radars
  and cameras to a real recording, consistently with the car's recorded motion.

Design, with the alternatives that were considered: [docs/DESIGN.md](docs/DESIGN.md).

## Quick start

Requirements: Python 3.10 or newer and numpy. Nothing else.

```sh
git clone <this repository> && cd chris-integrator
python3 -m pytest -q tests                # about two minutes: filter properties, simulated drives, sensors
```

A simulated drive through the integrator:

```python
from chris_integrator import Integrator
from chris_integrator.sim import World, default_sensors
from chris_integrator.sim.radar import RadarSim

world = World(seed=1, scenario="town")
sensors = default_sensors() + [RadarSim()]
integ = Integrator({"sources": [{"kind": "radar", "id": "*"}]})
for arrival, msg in world.messages(sensors):          # in arrival order: late and out of order as in a car
    if msg["type"] == "road":                         # one camera frame through the whole box
        ego_state, tracks, lead = integ.frame(msg)
        truth = world.truth_at(msg["ts_mono_ns"] / 1e9)
        print(ego_state["speed"], truth["v"], lead["dist_m"])
    else:
        integ.on_message(msg)
```

As a daemon next to a running chris-roadsense:

```sh
CHRIS_RUNDIR=/run/chris python3 -m chris_integrator --roadsense /run/roadsense
# reads <roadsense run dir>/modeld.sock (and its vehicle relays), accepts the car's feed on /run/chris/feed.sock,
# serves ego_state / tracks / lead as JSON lines on /run/chris/integrator.sock
```

Replaying a recording: any file of the input messages (JSON lines, in arrival order) can be fed with
`Integrator.on_message` / `Integrator.frame`. Recordings are not part of this repository; the simulator
produces data with truth, and a chris-roadsense recorder session is a ready input.

## Input contract

JSON objects with a `type` and `ts_mono_ns`, the time of the *measurement* on the host's monotonic clock
(a clock that restarts is handled: a new epoch keeps the calibration and drops what was tied to the old time
axis). Sources shipped here read:

| type | from | fields used |
|---|---|---|
| `road` | chris-roadsense (road model) | `pose` (velocity in the device frame with stds, yaw rate), `leads[0]`, `plan`, `road_model`, `valid` |
| `calib` | chris-roadsense | the calibration in use (pitch, yaw) |
| `ego` / feeder `motion` | the car | `speed_mps`, `standstill`, `wheel_speeds_mps`, `yaw_rate_rps`, `accel_long_mps2`, `accel_lat_mps2`, `turn_signal` |
| `vehicle` / feeder `controls` | the car | `controls.brake` |
| `gnss` | a receiver | `fix`, `lat`, `lon`, `hacc_m`, `speed_mps`, `course_deg`, `alt_m` |
| `map` | a map matcher | `match`: matched point, bearing, confidence, ambiguity |
| `objects` | radars, cameras | `sensor`, `kind`, per object range / azimuth / range rate or x / y in the sensor's frame |

No source is required. A car with a camera only gets a camera-only estimate that says so.

## Output contract

Three messages per camera frame (per vehicle sample while no camera runs), stamped with the frame's time of
validity:

* `ego_state`: `speed`, `accel`, `yaw_rate`, `heading`, `grade`, each
  `{value, std, valid, reason, mode: fused | coast | none, age_s, sources}`; `position` (lat, lon, local east /
  north, covariance, `hacc_m`); `odometer_m`; `standstill`; `calibration` (every estimated bias, scale, delay
  and mount angle with its std); `sources` (per source `absent | stale | unchecked | ok | suspect | fault`,
  the reason, how a fault was isolated); `checks` (every monitor's verdict).
* `tracks`: every track with id, state, existence probability, position / velocity / acceleration with std,
  the sensors that feed it and when each last saw it, flags (`standing`, `ghost`, `stationary`, `on_path`).
* `lead`: the track on the path with distance, relative speed, time gap, time to collision, life-cycle events,
  the sensors behind it, and `valid` / `reason`.

Rules that hold for every field: `valid: false` always carries a reason; a value that is not a fresh fusion is
marked; nothing is held or smoothed for looks.

## Adding a sensor

```python
from chris_integrator import Source, Meas, AuxState
from chris_integrator.core import ego as E
import numpy as np

class Barometer(Source):
    id = "baro"
    types = ("baro",)                                   # the message type it reads
    aux = (AuxState("bias", 0.0, 20.0, 1e-4, unit="m"),)  # a calibration state the estimator carries for it

    def measure(self, msg, ctx):
        ib = self.ix["bias"]
        idx, coef = np.array([E.Z, ib]), np.array([1.0, 1.0])
        h = lambda x: (x[E.Z] + x[ib], idx, coef)       # prediction and sparse Jacobian
        return [Meas("alt", msg["ts_mono_ns"] / 1e9, msg["alt_m"], 0.5, h, "baro.alt")]

integ.register(Barometer())
```

An object sensor returns a `Scan` with a `SensorModel` instead (see `sources/radar.py`, 300 lines, and
`sources/camera_objects.py`, 100 lines). Neither the core nor any consumer changes: the health table grows a
row and tracks list the new sensor among their sources.

## Status and limits

Working and tested: everything listed above, on the simulator and on recordings of the reference deployment.
Not hidden:

* The reference car has one camera, wheel speeds, a gyro, two accelerometers and a poor GNSS receiver. Radar
  and second-camera sources are proven on the simulator and on synthetic sensors injected into real
  recordings, not on a real radar.
* No 6-axis IMU is modelled: roll and bank are not states, grade and accelerometer bias are separable only with
  an altitude source, sideslip is ignored.
* Monitor thresholds rest on a small amount of real driving (town, daylight).
* Pure Python and numpy: about 2-3 ms per camera frame on a laptop core, more with radars. The filter core is
  the candidate for a compiled implementation.
* The consolidated consumer API (HTTP / WebSocket) and the map matcher still live in chris-roadsense while
  they are being moved here.

## Reference deployment

A second-generation Toyota Prius with a phone (Poco F1, postmarketOS) as the car computer running
chris-roadsense and this integrator; a separate project feeds the car's CAN signals and draws the display.
That deployment is the test bed, not a requirement: the integrator is car-agnostic and has no dependency on
either.

## Licence

MIT, see [LICENSE](LICENSE). The integrator contains no code of other projects; it reads the *messages* of
chris-roadsense (MIT) and of separately licensed detectors.
