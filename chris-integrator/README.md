# chris-integrator

The data integrator of **chris**, an open perception stack for a car computer (named after St Christopher).
Every sensor of the car goes *in*; one consistent picture comes *out*: where the car is and how it moves, what
is around it, how far each of those statements can be trusted, and why not when it cannot.

chris informs the driver. It does not control the car.

```
 INPUT CHANNELS                       chris-integrator: a mixer                        OUTPUT CHANNELS

 chris-roadsense (one per camera):    ┌───────────────────── bus ─────────────────────┐   consolidated API
   road model, signs, lights,   ───>  │ named streams, time-stamped, with metadata    │ ─>  HTTP /v2, WebSocket /ws
   scenery, camera status             │      │ ▲      │ ▲      │ ▲      │ ▲           │ ─>  JSON-lines socket
 the car (vehicle input API):   ───>  │      ▼ │      ▼ │      ▼ │      ▼ │           │ ─>  drive recorder
   wheel speed, gyro, accel.,         │  estimator  map box  trackers  decisions,     │
   controls, GNSS                     │  (ego state, (chris-  (lanes,   display       │
 radars, further cameras,       ───>  │  objects,    mapper)  lights,   (rules, limit,│
   class detectors                    │  lead)                signs)    fused, view)  │
                                      └───── plug-in transformers: in -> out ─────────┘
```

The rule of the architecture: **inputs go to the integrator, and only then to any logic**, and the
integrator is where decisions are taken and what consumers talk to. A consumer never picks a raw sensor,
never guesses whether a value is fresh, and never has to know which sensors a particular car has. A new
camera, radar or detector adds an input; a new piece of logic adds a plug-in; neither touches the rest.

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
* **Sources as plug-ins of the estimator.** A sensor is one file: an input description, a measurement model,
  its mount and calibration states. The core knows no sensor. Included: vehicle motion, road-model pose, GNSS,
  map match, the road model's lead, radar (any number, any mount), a further camera's object list, a
  road-user class detector.
* **A mixer** (`chris_integrator/mixer`): input channels, a bus of named streams, plug-in transformers with
  declared inputs and outputs, output channels. The order of the plug-ins is derived from their declarations;
  a plug-in can be added, removed, replaced or switched off at run time or from a configuration file, tested
  alone on a bench, and run in another process behind the same contract (JSON lines over a pipe or a socket).
  `GET /v2/mixer` shows the wiring with each box's health and cost.
* **The boxes of the stack** (`chris_integrator/plugins`, logic in `chris_integrator/trackers`): the
  estimator above; the lane-line tracker; ego lane and fused road geometry; the ego traffic light; sign
  context; signs in effect and the speed limit; map + camera fusion; the display channel `view`; the map box
  (the separate component chris-mapper: a pose in, the map match and the map context out).
* **The consolidated API** (`chris_integrator/io`): HTTP and WebSocket over the bus, a JSON-lines socket for
  other processes, a drive recorder, the car's direct feed as an input.
* **A simulator with known truth** (`chris_integrator/sim`) that emits the same message types the real
  sources send, with noise, bias, latency, drop-outs and faults; and an injector that adds synthetic radars
  and cameras to a real recording, consistently with the car's recorded motion.

Design, with the alternatives that were considered: [docs/DESIGN.md](docs/DESIGN.md).

## Quick start

Requirements: Python 3.10 or newer and numpy. The HTTP / WebSocket API needs aiohttp; the map box needs
chris-mapper. Nothing else.

```sh
git clone <this repository> && cd chris-integrator
python3 -m pytest -q tests                # filter properties, simulated drives, sensors, the mixer, the API
```

A simulated drive through the estimator box alone:

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

The same drive through the whole stack (every box, on a clock that follows the messages):

```python
from chris_integrator.mixer import MessageClock
from chris_integrator.stack import build

stack = build(clock=MessageClock())
stack.mixer.add_output("print", lambda env: print(env.stream, env.ts), streams=("lead", "lanes", "view"))
for arrival, msg in world.messages(sensors):
    stack.deliver(msg)                                # routed to its stream by `type`
print(stack.mixer.describe()["plugins"])              # order, health, cost per box
```

As a daemon next to a running chris-roadsense:

```sh
CHRIS_RUNDIR=/run/chris python3 -m chris_integrator --roadsense /run/roadsense --http-port 8080
# reads the roadsense instance's sockets (camerad, modeld, signd, lightd, envd, laned, api),
# accepts the car's feed on /run/chris/feed.sock (and POST /v2/vehicle),
# serves the consolidated API on :8080 (GET /v2/lead, /v2/view, ..., /v2/health, /v2/mixer; WebSocket /ws?topics=...),
# and JSON lines for other processes on /run/chris/integrator.sock
```

Replaying a recording: `chris_integrator.replay.Pipeline` is a mixer with a recorder session as its input
channel. Recordings are not part of this repository; the simulator produces data with truth.

## Input contract

JSON objects with a `type` and `ts_mono_ns`, the time of the *measurement* on the host's monotonic clock
(a clock that restarts is handled: a new epoch keeps the calibration and drops what was tied to the old time
axis). Sources shipped here read:

| type | from | fields used |
|---|---|---|
| `road` | chris-roadsense (road model) | `pose` (velocity in the device frame with stds, yaw rate), `leads[0]`, `plan`, `road_model`, `valid`; lane lines and road edges (the lane tracker) |
| `calib` | chris-roadsense | the calibration in use (pitch, yaw) and the camera's intrinsics |
| `lights`, `signs`, `env`, `camera` | chris-roadsense detectors | light and sign detections, place / weather / light, camera status |
| `governor` | chris-roadsense | its load governor's level and caps (the `rate` marker of measurement topics) |
| `ego` / feeder `motion` | the car | `speed_mps`, `standstill`, `wheel_speeds_mps`, `yaw_rate_rps`, `accel_long_mps2`, `accel_lat_mps2`, `turn_signal` |
| `vehicle` / feeder `controls` | the car | `controls.brake` |
| `gnss` | a receiver | `fix`, `lat`, `lon`, `hacc_m`, `speed_mps`, `course_deg`, `alt_m` |
| `map` | the map box (chris-mapper), or an external matcher | `match`: matched point, bearing, confidence, ambiguity; the map context |
| `objects` | radars, cameras, class detectors | `sensor`, `kind`, per object range / azimuth / range rate, or x / y in the sensor's frame, or a `class` with where it points |
| `lights` + `road_users` | a camera's class detector (roadsense-signs `signd --lights-road-users`) | per box `cls`, `score`, `box`, `clip`, `rider`; placed by the `road_users` box with the camera's `calib` (bearing, ground distance with its uncertainty) -> stream `road_users` |

No source is required. A car with a camera only gets a camera-only estimate that says so.

## Output contract

The estimator box puts out three messages per camera frame (per vehicle sample while no camera runs), stamped
with the frame's time of validity:

* `ego_state`: `speed`, `accel`, `yaw_rate`, `heading`, `grade`, each
  `{value, std, valid, reason, mode: fused | coast | none, age_s, sources}`; `position` (lat, lon, local east /
  north, covariance, `hacc_m`); `odometer_m`; `standstill`; `calibration` (every estimated bias, scale, delay
  and mount angle with its std); `sources` (per source `absent | stale | unchecked | ok | suspect | fault`,
  the reason, how a fault was isolated); `checks` (every monitor's verdict).
* `tracks`: every track with id, state, existence probability, position / velocity / acceleration with std,
  the sensors that feed it and when each last saw it, flags (`standing`, `ghost`, `stationary`, `on_path`),
  and `class` (what kind of road user: label, confidence, source, age, votes; a vote over the track's life with
  a sticky label, `null` while no sensor says or the vote is not sure; never used to gate a measurement).
* `lead`: the track on the path with distance, relative speed, time gap, time to collision, life-cycle events,
  the sensors behind it, its `class`, and `valid` / `reason`.

The other boxes add the consolidated topics: `lanes` (tracked lane lines), `lights` (traffic lights with the
ego light selected), `signs` (with context flags), `rules` and `speed_limit`, `map` (the map box), `fused`
(map + camera), and `view` (the display channel: one picture per frame for a screen; smoothed, informational,
never fed back). All of them are served as `GET /v2/<topic>` (the newest message) and on `/ws?topics=...`.

Rules that hold for every field: `valid: false` always carries a reason; a value that is not a fresh fusion is
marked; nothing on a measurement topic is held or smoothed for looks.

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

## Adding a box

A plug-in declares what it reads and writes and handles its trigger messages:

```python
from chris_integrator.mixer import In, Out, Plugin

class Headway(Plugin):
    """Seconds to the vehicle ahead, with a warning level."""
    name = "headway"
    DEFAULT_CFG = {"warn_s": 1.2}
    inputs = (In("lead", trigger=True), In("ego_state"))
    outputs = (Out("headway", kind="decision", time="frame", doc="time gap to the lead and its level"),)

    def on_input(self, stream, lead, ctx):
        es = ctx.get("ego_state", frame=True)            # this frame's, or None
        gap = lead.get("time_gap_s") if lead.get("valid") else None
        ctx.emit("headway", {"type": "headway", "ts_mono_ns": lead["ts_mono_ns"], "gap_s": gap,
                             "valid": gap is not None and es is not None,
                             "level": None if gap is None else "warn" if gap < self.cfg["warn_s"] else "ok"})

stack.mixer.add(Headway())             # or in mixer.json: {"plugins": [{"class": "mypkg.headway:Headway"}]}
```

It runs after the estimator because it reads `lead`; `/v2/headway` and `/ws?topics=headway` serve it; nothing
else was edited. Alone on a bench: `Bench(Headway()).feed("lead", lead_message)`. In another process:
`RemotePlugin(cmd=[...])` with `remote.serve(Headway(), sys.stdin.buffer, sys.stdout.buffer)` at the other
end, or any program that speaks the lines (docs/DESIGN.md section 14.3).

## Fault detection: what acts, what reports

Every detector that can take a source out of the estimate has a mode in `integrator.json`
(`{"detectors": {"speed.cusum": "gate"}}`): `gate` (its verdict takes effect; what depends on that source alone
is published as unknown) or `report` (its verdict is published with the evidence in `ego_state.diagnostics`,
and nothing is set aside). By default a detector gates only if it was quiet, with a stated margin, on the
reference car's healthy drives; `docs/FAULTS.md` has that table and the fault table, `docs/DESIGN.md` section
5b the mechanism. `ego_state.detectors` lists the modes in use, `ego_state.checks[].peak` how near each test
came to firing.

## Documents

`docs/DESIGN.md` (estimator, fault detection, object layer, the mixer), `docs/FAULTS.md` (which detectors gate;
the fault table), `docs/TOPICS.md` (field tables of the consolidated topics), `docs/VEHICLE_INPUT.md` (the
vehicle input API as served here), `docs/INSTALL.md` (install, units, ports, rollback).

## Status and limits

Working and tested: everything listed above, on the simulator and on recordings of the reference deployment.
Not hidden:

* The reference car has one camera, wheel speeds, a gyro, two accelerometers and a poor GNSS receiver. Radar
  and second-camera sources are proven on the simulator and on synthetic sensors injected into real
  recordings, not on a real radar.
* No 6-axis IMU is modelled: roll and bank are not states, grade and accelerometer bias are separable only with
  an altitude source, sideslip is ignored.
* Position has no truth on the reference car. In simulation it is consistent (town and motorway). With the
  poor receiver it is often declared unknown, and a persistent offset of the only absolute source cannot be
  told from a movement of the car.
* Fault detection is scored by a campaign over a fault taxonomy (docs/DESIGN.md section 5a). Not detectable
  with one camera, one speed chain and one receiver: which of two disagreeing sources is wrong when no third
  exists, two sources wrong the same way, a wrong camera lead range, a slow drift of the receiver.
* Monitor thresholds rest on a small amount of real driving (town, daylight).
* Pure Python and numpy: about 4 ms per camera frame on a laptop core for the whole stack (the estimator about
  half of it), more with radars. The filter core is the candidate for a compiled implementation.
* The car's direct feed is taken as it comes: the validation, per-signal quality and the description of the
  car that a feeder may send (vehicle input API 1.1) are not yet carried into the `vehicle` topic.
* A second chris-roadsense instance is wired as a channel (its streams are suffixed with the instance name);
  a second road-model pose as an estimator source is not built.
* Out-of-process plug-ins are exercised by the tests and by replays, not yet on a car.

## Reference deployment

A second-generation Toyota Prius with a phone (Poco F1, postmarketOS) as the car computer running
chris-roadsense and this integrator; a separate project feeds the car's CAN signals and draws the display
from the consolidated API.
That deployment is the test bed, not a requirement: the integrator is car-agnostic and has no dependency on
either.

## Licence

MIT, see [LICENSE](LICENSE). The integrator contains no code of other projects; it reads the *messages* of
chris-roadsense (MIT) and of separately licensed detectors, and uses chris-mapper (MIT) as its map box. Map
data is not part of this repository.
