# The vehicle input API as served by chris-integrator

The integrator is the receiver of the vehicle input API 1.1 (the contract is roadsense's [`docs/VEHICLE_API.md`](https://github.com/PiotrTopa/roadsense/blob/main/docs/VEHICLE_API.md);
this page says how the integrator serves it and where it differs from the legacy mapd that served it before).
The store behind it is `chris_integrator/io/vehicle_input.py`, ported from mapd's `vehicle.py` / `engine.py`
(lane L43-V). The integrator informs the driver; nothing here is an output to the car.

## 1. Where

| channel | what |
|---|---|
| `<run dir>/position.sock`, `<run dir>/feed.sock` | the same server under two names (`position.sock` is the name a feeder written for mapd knows). Unix stream socket, mode 0666, one JSON object per line, any number of feeders at once |
| `POST /v2/vehicle` (alias `POST /v1/vehicle`, `POST /v2/position`) | the same messages over HTTP: one object, a JSON array, or JSON lines. Answer `{"api":1,"accepted":n,"rejected":[{"index":i,"reason":"..."}]}`; a body that is not JSON: 400 |
| `GET /v2/vehicle`, `/ws` topic `vehicle` | the `vehicle` topic (section 5); `GET` adds the last scan of every range sensor under `objects` |
| `/ws` topic `objects` | every accepted scan (section 6) |

## 2. Lines on the socket

Line for line as mapd's `position.sock`:

* The server speaks only when spoken to. A feeder's `{"type":"hello",...}` gets the hello reply:

  ```json
  {"type":"hello","api":1,"minor":1,"server":"chris-integrator",
   "accepts":["gnss","motion","reset","controls","body","vehicle","objects"],
   "signals":{"motion":[...],"controls":[...],"body":[...]},"clock":"CLOCK_MONOTONIC","now_mono_ns":8126624315000}
  ```

  `name` and `provides` of the feeder's hello are shown in the `vehicle` topic (`feeders`).
* An accepted message gets no reply.
* A rejected line gets `{"type":"error","line":n,"reason":"..."}` and is dropped; `n` counts the lines of this
  connection from 1 (the hello is line 1, empty lines count). The connection stays open.
* `bad JSON` for a line that is not JSON; otherwise the reason of the check that failed (section 3).

## 3. Validation

The checks and their reasons are mapd's. A rejected message changes nothing and produces nothing.

| check | reason (examples) |
|---|---|
| not an object, unknown `type` | `not a JSON object`, `unknown type 'wipers'` |
| `ts_mono_ns` not an integer, older than 5 s, more than 0.5 s ahead of this host's CLOCK_MONOTONIC | `ts_mono_ns must be an integer`, `ts_mono_ns is 6.0 s old (CLOCK_MONOTONIC of this host expected)`, `ts_mono_ns is in the future (...)` |
| `age_ms` negative or older than 5 s | `age_ms must be a number >= 0`, `age_ms: the sample is 60.0 s old` |
| a number outside its range, not finite | `speed_mps out of range`, `speed_mps must be a finite number` |
| a bool / enum of the wrong kind | `brake must be true or false`, `gear must be one of P R N D B`, `turn_signal must be one of off left right both` |
| `quality` | `quality must be an object {signal: quality}`, `quality of low_beam must be one of good uncalibrated unverified invalid` |
| `motion` without any signal, `controls` / `body` without a known one | `missing field speed_mps`, `no known signal in the controls message` |
| `gnss` | `fix must be one of none 2d 3d dgps rtk dr`, `missing field lon`, `lat out of range`, `course_deg out of range` |
| `vehicle` | `wheelbase_m out of range`, `name must be a string`, `every sensor needs an id (string)`, `sensor kind must be one of ...`, `camera.z_m out of range` |
| `objects` | `objects needs a sensor id (string)`, `objects must be a list (...)`, `objects[0] needs range_m or x_m`, `objects[0].range_m out of range`, `tracked must be true or false`, `more than 256 objects` |
| `reset` | `reason must be a string` |

Unknown fields are ignored; a signal that is `null` is "not sent". Wheel speeds that are not four numbers in
0..120 m/s are ignored, not rejected (as in v1). A fix with `hacc_m` above 100 m is accepted and not passed on.

Configuration: `<config dir>/vehicle_input.json` (all keys optional): `max_input_age_s` (5), `max_input_future_s`
(0.5), `max_fix_hacc_m` (100), `pair_s` (0.3), `vehicle` (the description's defaults for this car, keys of the
`vehicle` message).

## 4. What the estimator gets

| input | to the stack | notes |
|---|---|---|
| `motion` with a usable `speed_mps` | `ego` | signed speed (negative with gear `R` / `reverse`), standstill (speed and every wheel exactly 0), the yaw rate in use and its source: the gyro as read (`yaw_rate`; its bias and scale are the estimator's states), else the rear wheel pair over `track_m` (`wheel_speeds`), else the steering angle with `steering_ratio` and `wheelbase_m` (`steering`); a derived yaw rate is 0 at standstill. Steering angle and accelerations when usable. A signal of a message without a speed joins the next one with a speed within `pair_s` |
| `motion` without a speed | nothing | stored for the next one with a speed, and for the `vehicle` topic |
| `gnss` | `gnss` | the known fields as sent, `ts_mono_ns` (the fix's time), `arrival_mono_ns` |
| `controls`, `body`, `vehicle` | the `vehicle` topic | the estimator reads `controls.brake` from it (`lead.ego.brake`) |
| `objects` | `objects` | section 6 |
| `reset` | stream `position_reset` | section 7 |

Quality (`good`, `uncalibrated`, `unverified`, `invalid`): the estimator's sources do not read quality tags, so
the store applies their meaning before the estimator: only `good` and `uncalibrated` motion signals go into
`ego` (an `unverified` gyro is not used; the yaw rate falls back to the wheels), `invalid` ones are absent
everywhere. The tags themselves are carried in the `vehicle` topic (`signals`). The estimator learns the scale of
every speed and yaw sensor it uses, whatever its tag: `good` and `uncalibrated` are not told apart there.

## 5. The `vehicle` topic

Twice a second (also without a feeder: `present: false`, every signal null) and at once, at most 20 a second, when
a switch-like signal, the description, the sensors, the feeders or the reset count change. Stamped when it is
built (`ts_mono_ns` of this host).

```
{"type":"vehicle","seq":..,"ts_mono_ns":..,"ts_real_ns":..,"api":1,
 "present":true,                              // at least one signal is fresh
 "motion":{"speed_mps":13.42,"standstill":false,"gear":"D","reverse":false,     // the last `ego` handed to the estimator
           "yaw_rate_rps":0.031,"yaw_source":"wheel_speeds",
           "wheel_speeds_mps":[..],"steering_wheel_deg":12.0,"accel_long_mps2":0.31,"accel_lat_mps2":0.4},
 "controls":{"turn_signal":..,"brake":..,"brake_pressure":..,"accelerator":..,"cruise":..,"cruise_set_speed_mps":..,"parking_brake":..},
 "body":{"low_beam":..,"high_beam":..,"parking_lights":..,"fog_front":..,"fog_rear":..,"wipers":..,"rain":..,
         "ambient_light_lux":..,"outside_temp_c":..,"door_open":..},
 "signals":{"brake":{"age_s":0.5,"quality":"good","source":"my-can"}, ...},     // every signal ever sent
 "sensors":[{"id":"front_radar","kind":"radar","age_s":0.03,"objects":5,"hz":19.8,"scans":8123,"mounted":true}],
 "description":{"name":..,"wheelbase_m":..,"track_m":..,"steering_ratio":..,"camera":{..},"sensors":[..],
                "sources":{"wheelbase_m":"feeder","track_m":"config","steering_ratio":"default"}},
 "calibration":{...},                         // below
 "feeders":[{"name":"my-car","provides":{..},"connected_s":812.4,"messages":90211}],
 "input":{"counts":{"gnss":..,"motion":..,"reset":..,"controls":..,"body":..,"vehicle":..,"objects":..,"errors":..,"gnss_ignored":..},
          "last_error":"...","resets":1,"last_reset":{"ts_mono_ns":..,"reason":"...","auto":false}}}
```

A value is null when it was never sent, is older than its hold time (motion 1.5 s, controls 3 s, body 15 s) or was
declared `invalid`; an `unverified` value is shown. Every key of mapd's `vehicle` message is there; `input` is new
(mapd had its counters in `map.input`). `motion.speed_mps` / `yaw_rate_rps` are what was handed to the estimator
(the gyro as read), not the estimate: that is `ego_state`.

`calibration` keeps mapd's shape and reports what the estimator knows (from its newest `ego_state.calibration`);
nothing is estimated twice. `value` is the factor that makes a fed signal true:

| key | from the estimator | not estimated (null) |
|---|---|---|
| `yaw_rate.bias_rps`, `bias_std_rps` | `vehicle.b_g` (when known) | `bias_n` |
| `yaw_rate.scale` | 1 / (1 + `vehicle.s_g`) | `n` (`converged` = the state is known, `ref` = "estimator") |
| `yaw_rate.scale_vs_wheels` | | the whole entry: the estimator holds the wheels' scale against the yaw rate, not against the gyro (`wheel_yaw.scale`) |
| `wheel_yaw.scale` | 1 / (1 + `vehicle.s_wy`) | `n` |
| `speed.scale` | 1 / (1 + `odometry.speed_scale`) | `n` |
| `gnss.lag_s`, `lag_std_s`, `converged` | `gnss.tau` | `speed_ratio`, `n` |

## 6. The `objects` stream

One message per accepted scan: `{"type":"objects","seq","ts_mono_ns","api":1,"sensor","kind","frame":"sensor",
"tracked","source","mounted","objects":[...],"mount":{...}}`. The objects are the validated ones (known keys
only), **in the sensor's frame** (polar `range_m` / `azimuth_deg` positive left, or `x_m` along the boresight,
`y_m` to the left). `mount` is the sensor's place in the estimator's car frame (relative to the camera, x
forward, y left, `yaw_deg` of the boresight): present when the description places both the sensor and the
camera. The estimator's radar sources read exactly this; positions of objects in the car frame are in `tracks`.

## 7. `reset`

`{"type":"reset","reason":"..."}`: the input so far must not live on as the car's position (a synthetic drive
ended, the receiver was swapped). It goes through the mixer as the stream `position_reset` (input channel
`vehicle`, not public); the plug-ins that declare it act on it:

| who | what is dropped | what is kept |
|---|---|---|
| estimator (`Integrator.new_position_source`) | the local frame (origin, anchor), position, heading and altitude with their covariance; the states of the absolute sources (receiver delay `gnss.tau`, its correlated error `gnss.be` / `bn`, the map's lane offset `map.off`) back to their priors; those sources' memories (receiver tag, noise scale, track points), arrival history and position monitors | speed, yaw rate, accelerations, grade; every calibration state of the car's own sensors and of the cameras (speed scale, gyro bias and scale, wheel-yaw scale, accelerometer offsets, mounts); faults; the object layer; the odometer |
| map box (`plugins/mapper.py`) | match, matcher, position state: `map.state` `no_source`, `map.input.synthetic` false | the pack |
| the store | the record of which receiver fed the position | signals, description, sensors, feeders |

Without camera frames the estimator sends an `ego_state` without the position at once, and the map box answers
it with `no_source`, so a consumer sees the reset without waiting for the next sample. `map.input.resets` /
`last_reset` (present after the first reset) and `vehicle.input.resets` / `last_reset` say what happened. The next usable fix anchors the position
again (a new frame id).

As in mapd, a `gnss` message whose `source` changes between synthetic (`synthetic...`) and real (or back) resets
the position by itself first (`last_reset.auto: true`). The plug-in `reset(reason)` of the mixer (a new drive, a
clock restart: every plug-in starts afresh) is a different thing and is not used for this.

An out-of-process map box does not declare `position_reset`: it sees the estimator's position go and its new
frame id and drops its match (state `unknown` instead of `no_source`).

## 8. Differences from the legacy mapd, deliberately

| mapd | here | why |
|---|---|---|
| greeting: none | none | (L39's integrator sent an unsolicited hello on connect; dropped, so the line stream is mapd's) |
| `hello.server` `roadsense-mapd` | `chris-integrator` | |
| its own calibration estimators (`vehicle-calibration.json`, per source tag) | the estimator's states, reported (section 5) | position and motion calibration live in the estimator; no second filter |
| `use_calibration`, `gnss_lag_s` | not carried | the estimator applies its scales and estimates the receiver's delay itself |
| the gyro relayed bias-corrected, forced to 0 at standstill | relayed as read | the estimator learns the bias at standstill from the raw reading |
| `objects` relayed in the camera frame (y right) | in the sensor's frame with the mount | the estimator's radar source reads sensor-frame scans with a mount (a camera-frame scan with y to the right would be mirrored there) |
| the `map` topic stepped twice a second | the map box answers every `ego_state` (camera frames, or vehicle samples without a camera) | |
| vision odometry from `modeld.sock` | not here | the estimator fuses the camera itself |

## 9. Tests

`tests/test_vehicle_input.py` (the store, and `reset` through the stack with the map box), `tests/test_vehicle_e2e.py`
(the integrator as a process: socket lines, HTTP, the `vehicle` topic, `reset` over HTTP, the reference clients),
`tests/test_car_feeder.py` (a car backend's own client and feeder, synthetic session and its reset: set
`CHRIS_TEST_CAR_REPO` to its checkout, optionally `CHRIS_TEST_CAR_PYTHON`; skipped otherwise).
