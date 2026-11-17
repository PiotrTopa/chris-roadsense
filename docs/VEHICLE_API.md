# Vehicle input API (version 1.1)

How a host tells roadsense about the car: where it is, how it moves, what the driver does, what the body
reports and what the car's own range sensors see. One contract for any car; nothing in it is specific to
one make. Everything is optional: without a feeder roadsense runs on the camera alone, and every piece a
feeder adds is used where it helps and reported with where it came from.

roadsense informs the driver. It never steers, brakes or accelerates, and nothing in this API is an
output to the car.

```
                        ┌────────────── roadsense host ───────────────────────────────────────────┐
 car integration        │                                                                         │
 (CAN, OBD, GNSS,  ───> │ position.sock ──> mapd ──┬─> mapd.sock    map     (position, road)      │
  radar, a satellite    │  (this API)              ├─> ego.sock     ego     (motion, feeder rate) │──> api: /v1/…, /ws
  that reads lamps…)    │                          └─> vehicle.sock vehicle (all signals), objects│
                        └─────────────────────────────────────────────────────────────────────────┘
```

Contents: [transport](#1-transport-and-handshake), [clock](#2-clock), [frames and signs](#3-frames-units-signs),
[quality](#4-quality-and-source), [messages](#5-input-messages), [rates](#6-rates-and-latency),
[what roadsense does with it](#7-what-roadsense-does-with-the-input), [outputs](#8-output-vehicle-objects-ego),
[calibration feedback](#9-calibration-feedback), [a minimal feeder](#10-a-minimal-feeder),
[compatibility](#11-versions-and-compatibility), [not in the API](#12-left-out).

## 1. Transport and handshake

* Unix stream socket `<rundir>/position.sock` (default `/run/roadsense/position.sock`, mode 0666), UTF-8
  JSON, one object per line. Any number of feeders at the same time (one for GNSS, one for CAN, one for a
  radar is fine). The socket keeps the name of the API's first version, which carried position only.
* A valid message gets no reply. An invalid one gets `{"type":"error","line":<n>,"reason":"..."}` on the
  same connection and is dropped; the connection stays open and the rest of the stream is unaffected.
* Unknown fields are ignored. An unknown message `type` is an error reply.
* A feeder must never block on the socket: when roadsense is away or slow, drop the message.
* The same messages over HTTP: `POST /v1/vehicle` (one object, a JSON array, or JSON lines; answer
  `{"api":1,"accepted":n,"rejected":[{"index":i,"reason":"..."}]}`; `POST /v1/position` is the same
  route under its first name). The api listens on loopback by default. Use the socket for anything
  faster than a few messages a second.

Handshake (optional, recommended): the first line a feeder sends is

```json
{"type":"hello","api":1,"name":"my-car","provides":{"gnss":["lat","lon","hacc_m"],"motion":["speed_mps","yaw_rate_rps"],"controls":["brake","turn_signal"]}}
```

`provides` declares what the feeder intends to send (message type -> signals); it is shown in the
`vehicle` topic (`feeders`) and changes nothing else. roadsense answers

```json
{"type":"hello","api":1,"minor":1,"server":"roadsense-mapd",
 "accepts":["gnss","motion","reset","controls","body","vehicle","objects"],
 "signals":{"motion":["accel_lat_mps2","accel_long_mps2","gear","reverse","speed_mps","steering_wheel_deg","wheel_speeds_mps","yaw_rate_rps","turn_signal"],
            "controls":["accelerator","brake","brake_pressure","cruise","cruise_set_speed_mps","parking_brake","turn_signal"],
            "body":["ambient_light_lux","door_open","fog_front","fog_rear","high_beam","low_beam","outside_temp_c","parking_lights","rain","wipers"]},
 "clock":"CLOCK_MONOTONIC","now_mono_ns":8126624315000}
```

A feeder that wants to work with older and newer roadsense versions sends a message type only when
`accepts` lists it (a roadsense before 1.1 lists `gnss`, `motion`, `reset`, or `gnss`, `motion` only), and
a signal only when `signals` lists it or it does not mind the signal being ignored. `now_mono_ns` lets a
feeder check that it reads the same clock.

## 2. Clock

Every message may carry the time its content was **measured** (not sent):

| field | meaning |
|---|---|
| `ts_mono_ns` | integer, CLOCK_MONOTONIC of the roadsense host at the instant of the measurement: the clock of every roadsense message. For a feeder on the same machine: `time.monotonic_ns()` taken when the sample was read from the bus |
| `age_ms` | alternative for a feeder that relays from another device and shares no clock with the host: the age of the sample at the moment of sending, in ms. roadsense subtracts it from the arrival time |
| neither | the arrival time is used |

Older than 5 s or more than 0.5 s in the future: rejected. One timestamp covers all signals of a message,
so group signals that were measured together, and send signals with different times in different
messages (a `motion` message may carry any subset of its signals, section 5.2).

## 3. Frames, units, signs

Input uses the vehicle frame of **ISO 8855**, fixed since v1:

* x forward, y to the **left**, z up; origin on the ground under the centre of the rear axle;
* angles, yaw rate and steering angle positive to the **left** (counter-clockwise seen from above);
* SI units throughout: m, m/s, m/s², rad/s; degrees for angles a person reads (steering wheel, azimuth,
  course, mounts); pedals and intensities as 0..1;
* wheel order `[front left, front right, rear left, rear right]`;
* GNSS course in degrees clockwise from true north (the receiver's convention).

roadsense's **output** topics (`road`, `lead`, `lanes`, `roadgeom`, and the `objects` relay below) use
the camera frame of the model: x forward, y to the **right**, origin at the camera. The relay converts
range-sensor objects into that frame, so a consumer of the outputs never mixes the two. In the outputs
that repeat input signals (`ego`, `vehicle`) the input's signs are kept (`yaw_rate_rps` positive left).

## 4. Quality and source

Every `motion`, `controls` and `body` message may carry

```json
"quality": {"yaw_rate_rps": "uncalibrated", "accelerator": "unverified", "*": "good"},
"source": "my-can"
```

`quality` maps a signal of this message to one of four words; `"*"` sets the default for the message;
a signal without an entry is `good`.

| quality | what the feeder says | what roadsense does |
|---|---|---|
| `good` | measured, decoded and scaled correctly | uses it as it is |
| `uncalibrated` | right signal and sign, scale or offset may be off by a few per cent | uses it; applies its own scale estimate once that has converged (section 9) |
| `unverified` | the decoding itself is not confirmed (meaning, sign, scale) | does not act on it; shows it in `vehicle`; still estimates its calibration, which is how a host finds out whether to trust it |
| `invalid` | the sensor reports a fault | treats the signal as absent |

`source` is a free tag of the sender (`"my-can"`, `"obd2"`, `"headunit"`), reported back per signal. A
tag that is `synthetic` or starts with it marks made-up input: mapd keeps a synthetic and a real
position apart and flags the position (`map.input.synthetic`).

A signal that the feeder cannot read is left out (or `null`). Never send a default in its place: "turn
signal off" and "turn signal unknown" are different statements.

## 5. Input messages

| `type` | content | since |
|---|---|---|
| `gnss` | one raw fix of a receiver | 1.0 |
| `motion` | the car's own motion: speed, gear, yaw rate, wheel speeds, steering angle, accelerations | 1.0, extended in 1.1 |
| `reset` | forget the position | 1.0 |
| `controls` | what the driver does: turn signal, brake, accelerator, cruise control | 1.1 |
| `body` | lamps, wipers, rain and light sensors, outside temperature, doors | 1.1 |
| `vehicle` | static description of the car: dimensions, camera and sensor mounts | 1.1 |
| `objects` | one scan of a range sensor (radar, lidar, ultrasonic): an object list | 1.1 |

### 5.1 `gnss`

One raw fix, unfiltered; roadsense does the filtering. Send every fix as it comes.

| field | unit | required | meaning |
|---|---|---|---|
| `fix` | | no (default `"3d"`) | `"none"`, `"2d"`, `"3d"`, `"dgps"`, `"rtk"`, `"dr"` (the receiver's own dead reckoning). `"none"` = the receiver has no fix now: roadsense starts dead reckoning at once instead of after 3 s of silence; no other field is needed |
| `lat`, `lon` | degrees WGS84 | yes unless `fix` is `none` | position of the antenna |
| `hacc_m` | m | recommended (default 10) | horizontal accuracy as a 68 % radius; fixes above 100 m are ignored |
| `speed_mps` | m/s | no | speed over ground, >= 0 |
| `speed_acc_mps` | m/s | no | 68 % accuracy of the speed |
| `course_deg` | degrees clockwise from true north, 0..360 | no | direction of travel; used at `speed_mps` >= 2.5 and not in reverse |
| `course_acc_deg` | degrees | no | 68 % accuracy of the course |
| `sats`, `hdop`, `utc_ms` | | no | satellites used, HDOP, UTC of the fix in Unix ms (reported back, not used by the filter) |
| `alt_m`, `vacc_m` | m | no | accepted, not used |
| `source` | string | no | tag of the receiver |

The timestamp matters more here than anywhere else: a receiver that delivers a fix 0.7 s after it was
valid puts the car 15 m behind itself at 80 km/h. Give `ts_mono_ns` of the fix (Android:
`Location.getElapsedRealtimeNanos()` is the same clock when the feeder runs on the same device) or
`age_ms`. roadsense estimates the remaining delay itself and reports it (section 9, `calibration.gnss`).

### 5.2 `motion`

| field | unit, range | meaning |
|---|---|---|
| `speed_mps` | m/s, 0..120 | vehicle speed, magnitude, from the wheels |
| `gear` | `"P"`, `"R"`, `"N"`, `"D"`, `"B"` | `"R"` makes the speed negative along the heading |
| `reverse` | bool | the same as gear `"R"`, for cars without a gear signal |
| `yaw_rate_rps` | rad/s, positive = turning left | from a gyroscope / yaw-rate sensor |
| `wheel_speeds_mps` | m/s `[FL, FR, RL, RR]`, each >= 0 | a yaw rate is derived from the rear pair and `track_m` |
| `steering_wheel_deg` | degrees of the steering wheel, positive = left | a yaw rate is derived with `steering_ratio` and `wheelbase_m` (kinematic bicycle), as the last resort |
| `accel_long_mps2` | m/s², positive = speeding up | longitudinal acceleration sensor (includes the road grade) |
| `accel_lat_mps2` | m/s², positive = towards the left | lateral acceleration sensor |
| `turn_signal` | as in `controls` | accepted here since v1; its place is `controls` |
| `quality`, `source` | | section 4 |

Rules:

* A message **with** `speed_mps` moves the position filter and is relayed as one `ego` message. It stands
  for itself: signals of earlier complete messages are not reused, so leaving a signal out means "not
  available now".
* A message **without** `speed_mps` (a gyroscope on another bus, an IMU) is stored, and its signals are
  joined to the next message with a speed when they are at most 0.3 s apart.
* The yaw rate comes from the first usable of `yaw_rate_rps`, `wheel_speeds_mps`, `steering_wheel_deg`;
  with none of them the heading is held between fixes. `ego.yaw_source` says which one it was.
* While the car stands (`speed_mps` exactly 0 and, when sent, every wheel exactly 0) the yaw rate is zero
  whatever the sensor reads, and a gyroscope's reading is learned as its bias.
* `motion` without any `gnss` never makes a position; it still feeds `ego`, `vehicle` and the lead tracker.

### 5.3 `reset`

`{"type":"reset","reason":"..."}`: forget the position (filter, last fix, match). The map is `no_source`
until a new fix arrives. Motion keeps being accepted. For a feeder whose input so far must not live on
(a test drive ended, the receiver was swapped). Details in docs/ARCHITECTURE.md, "mapd".

### 5.4 `controls`

| field | unit, range | meaning |
|---|---|---|
| `turn_signal` | `"off"`, `"left"`, `"right"`, `"both"` | the lamps as the driver set them (`both` = hazard). Send the stalk / lamp state, not the blinking: a lamp reader holds `left` through the dark half of the cycle |
| `brake` | bool | brake pedal pressed (the brake-light switch) |
| `brake_pressure` | 0..1 | brake pressure relative to full braking |
| `accelerator` | 0..1 | accelerator pedal position |
| `cruise` | `"off"`, `"standby"`, `"active"` | cruise control: main switch off / armed, not controlling / holding a speed |
| `cruise_set_speed_mps` | m/s | the speed the cruise control holds or would resume |
| `parking_brake` | bool | parking brake applied |

### 5.5 `body`

| field | unit, range | meaning |
|---|---|---|
| `low_beam`, `high_beam`, `parking_lights`, `fog_front`, `fog_rear` | bool | lamps that are lit |
| `wipers` | `"off"`, `"intermittent"`, `"low"`, `"high"` | front wiper mode |
| `rain` | 0..1 | rain sensor intensity |
| `ambient_light_lux` | lux | light sensor |
| `outside_temp_c` | °C | outside air temperature |
| `door_open` | bool | any door open |

### 5.6 `vehicle`

The static description, sent once per connection (after a reconnect again). Every key is optional and
overrides, key by key, the `"vehicle"` object of `/etc/roadsense/mapd.json`, which overrides the nominal
defaults. The description stays in force after the feeder disconnects, until another one replaces it.

```json
{"type":"vehicle","name":"My car",
 "wheelbase_m":2.70,"track_m":1.48,"track_front_m":1.50,"steering_ratio":15.5,
 "length_m":4.45,"width_m":1.72,"height_m":1.49,"rear_axle_to_front_m":3.60,
 "wheel_speed_min_mps":0.3,
 "camera":{"x_m":1.85,"y_m":0.0,"z_m":1.25},
 "sensors":[{"id":"front_radar","kind":"radar","x_m":3.55,"y_m":0.0,"z_m":0.45,"yaw_deg":0.0,"fov_deg":90,"max_range_m":160}]}
```

| key | meaning |
|---|---|
| `wheelbase_m`, `steering_ratio` | for the yaw rate from the steering angle (steering wheel angle / road wheel angle) |
| `track_m` | rear track: for the yaw rate from the rear wheel speeds |
| `track_front_m`, `length_m`, `width_m`, `height_m`, `rear_axle_to_front_m` | reported back; for consumers (a gap to the front bumper needs the last one and the camera mount) |
| `wheel_speed_min_mps` | below this the wheel sensors read zero ("standstill" means "below this") |
| `camera` | where roadsense's camera sits in the vehicle frame (section 3). Its angles are camera calibration, not part of this message |
| `sensors[]` | range sensors: `id` (the `sensor` of its `objects` messages), `kind` (`radar`, `lidar`, `ultrasonic`, `camera`, `other`), position, `yaw_deg` of its boresight (0 = straight ahead, positive = left), optional `fov_deg`, `max_range_m` |

### 5.7 `objects`

One message is one complete scan of one sensor: every object it reports **now**. An empty list is a
statement ("nothing there"); silence is not.

```json
{"type":"objects","sensor":"front_radar","kind":"radar","tracked":true,"ts_mono_ns":8126624315000,
 "objects":[{"id":17,"range_m":31.2,"azimuth_deg":-1.5,"range_rate_mps":-0.8,"rcs_dbsm":12.0,"prob":0.97}]}
```

| field | unit | meaning |
|---|---|---|
| `sensor` | string, required | the sensor's `id` in the `vehicle` description |
| `kind` | | as in the description; needed only without one |
| `tracked` | bool | the objects are the sensor's own tracks (stable `id`s), not raw detections |
| `objects[].range_m`, `azimuth_deg` | m, degrees positive left | polar position in the sensor's frame (an ultrasonic sensor sends `range_m` only) |
| `objects[].x_m`, `y_m` | m | or Cartesian in the sensor's frame: x along the boresight, y to the left. One of the two forms is required |
| `objects[].range_rate_mps` | m/s, negative = approaching | radial speed (Doppler) |
| `objects[].vx_mps`, `vy_mps` | m/s | or the relative velocity in the sensor's frame |
| `objects[].id` | integer or string | track id of the sensor |
| `objects[].rcs_dbsm`, `prob`, `width_m`, `length_m` | dBsm, 0..1, m | radar cross-section, existence probability, extent |
| `objects[].class` | | `car`, `truck`, `motorcycle`, `bicycle`, `pedestrian`, `unknown` |

At most 256 objects per message.

## 6. Rates and latency

| message | send | a signal is "unknown" after |
|---|---|---|
| `gnss` | every fix (1..10 Hz); `fix: "none"` on the loss and every 5 s while it lasts | 3 s without a fix = lost |
| `motion` | 10..20 Hz (5..50 accepted) | 1.5 s |
| `controls` | on every change, and once a second | 3 s |
| `body` | on every change, and every 5 s | 15 s |
| `vehicle` | once per connection | never |
| `objects` | every scan, 10..20 Hz | 0.3 s for the lead tracker |

Measured on the reference car (Toyota Prius, broadcast CAN through a USB CAN interface on the same phone,
2026-11-17, 110 min of logs): the bus delivers the motion frames every 13-24 ms; the receipt times of
the USB transfers are 2.5 ms apart in the median, 12 ms at the 95th and 24 ms at the 99th percentile,
which is the accuracy of `ts_mono_ns` there. At `motion` 10 Hz, `controls` came to 1.1-2.3 messages a
second (the accelerator moves), `body` to 0.2. The car's GNSS (a head unit's receiver relayed over USB)
delivered 1 fix a second while it had one. Its positions under-read the car's motion (a receiver that
smooths and lags by 0.5-1 s): timestamp what you can and let roadsense report the rest.

Cost on the roadsense side, per message, measured on a laptop core (the reference phone's little cores
are an order of magnitude slower in Python; not yet measured there): `motion` with a speed 0.18 ms
(0.16 ms before 1.1; the position filter is most of it), `controls` / `body` 0.015 ms, the `vehicle`
output message 0.12 ms (2.4 kB, twice a second and on changes), one `objects` scan of 32 objects
0.35 ms. A radar at 20 Hz with dozens of objects is the one input that costs: prefer the sensor's
tracked object list over raw detections, and 10 Hz if the host is small.

## 7. What roadsense does with the input

| input | use | where |
|---|---|---|
| `gnss` | position filter, map matching, everything on the `map` and `fused` topics | mapd |
| `motion` speed, gear, yaw | dead reckoning between and without fixes; the ego speed and standstill of the lead tracker; the heading of the fused road geometry between frames | mapd, api `lead`, `roadgeom` |
| `motion` at standstill | yaw rate forced to zero, gyroscope bias learned | mapd |
| `controls.turn_signal` | the driver's intent at the next junction or roundabout: which side road or exit the fused road geometry follows | api `roadgeom` |
| `controls.brake` | passed on in `lead.ego.brake`, next to the `departed` cue ("the lead moved off and you still hold the brake") | api `lead` |
| `vehicle` | `track_m`, `wheelbase_m`, `steering_ratio` for the derived yaw rates; mounts for `objects` | mapd |
| `objects` (radar / lidar, mounts known) | range and closing speed of the lead vehicle, with a fallback to the camera; **off by default** (`lead.json`: `{"radar_fusion": true}`) | api `lead` |
| everything | the `vehicle` topic with age, quality and source per signal; the drive recorder | mapd, api |
| the rest (`accelerator`, `cruise`, `body`, accelerations, steering angle while another yaw source exists) | reported, recorded, not acted on yet | |

Everything degrades by itself: a signal that stops coming becomes unknown after its hold time and the
consumer falls back (wheel yaw instead of the gyroscope, the camera's distance instead of the radar's,
the model's own speed instead of the car's, no position instead of a dead-reckoned one).

## 8. Output: `vehicle`, `objects`, `ego`

On `<rundir>/vehicle.sock`, on the api as `GET /v1/vehicle` (the `vehicle` message plus the last scan of
every sensor under `objects`) and on `/ws` (topics `vehicle`, `objects`).

**`vehicle`**: the car as roadsense holds it now. Sent with every `map` message (2 a second) and at once
(at most 20 a second) when a switch-like signal, the description, the feeders or the sensors change.
Present without any feeder too (`present: false`, every signal null).

```
{"type":"vehicle","seq":4711,"ts_mono_ns":..,"ts_real_ns":..,"api":1,
 "present":true,                              // at least one signal is fresh
 "motion":{"speed_mps":13.42,                 // signed: negative in reverse; as used (after calibration)
           "standstill":false,"gear":"D","reverse":false,
           "yaw_rate_rps":0.0312,"yaw_source":"yaw_rate",       // "yaw_rate" | "wheel_speeds" | "steering" | null
           "wheel_speeds_mps":[13.4,13.5,13.3,13.5],"steering_wheel_deg":12.0,
           "accel_long_mps2":0.31,"accel_lat_mps2":0.4},
 "controls":{"turn_signal":"left","brake":false,"brake_pressure":null,"accelerator":0.12,
             "cruise":"off","cruise_set_speed_mps":null,"parking_brake":null},
 "body":{"low_beam":true,"high_beam":false,"parking_lights":true,"fog_front":false,"fog_rear":null,
         "wipers":null,"rain":null,"ambient_light_lux":null,"outside_temp_c":6.5,"door_open":false},
 "signals":{"yaw_rate_rps":{"age_s":0.04,"quality":"uncalibrated","source":"my-can"}, ...},   // every signal ever sent
 "sensors":[{"id":"front_radar","kind":"radar","age_s":0.03,"objects":5,"hz":19.8,"scans":8123,"mounted":true}],
 "description":{"name":"My car","wheelbase_m":2.7,"track_m":1.48,"steering_ratio":19.1,"camera":{..},"sensors":[..],
                "sources":{"wheelbase_m":"feeder","track_m":"config","steering_ratio":"default"}},
 "calibration":{...},                         // section 9
 "feeders":[{"name":"my-car","provides":{..},"connected_s":812.4,"messages":90211}]}
```

A value is `null` when it was never sent, is older than its hold time (section 6) or was declared
`invalid`; `signals` keeps its age, quality and source. A signal declared `unverified` shows its value
and is not acted on.

**`objects`**: every accepted scan, as it came, with positions in the camera's frame when the mounts of
the sensor and of the camera are known (`"frame":"camera"`: `x_m` forward, `y_m` to the **right**,
relative to the camera, like `lead.dist_m` / `lead.lateral_m`), else in the sensor's own frame with the
same axes (`"frame":"sensor"`). `range_m`, `azimuth_deg`, `range_rate_mps` and the other fields pass
through unchanged.

**`ego`** (`<rundir>/ego.sock`, `/ws`): one message per `motion` message with a speed, at most 20 a
second: the car's motion at the feeder's rate. Since 1.1 also `yaw_source`, and `steering_wheel_deg`,
`accel_long_mps2`, `accel_lat_mps2` when fed. docs/ARCHITECTURE.md, "mapd".

## 9. Calibration feedback

What roadsense can estimate and a feeder cannot, in `vehicle.calibration`:

```
{"yaw_rate":{"bias_rps":-0.00027,"bias_n":16355,                       // learned while standing; always applied
             "scale":{"value":1.031,"std":0.012,"n":14,"converged":true,"ref":"gnss_course"},
             "scale_vs_wheels":{"value":1.006,"std":0.009,"n":2290,"converged":true,"ref":"wheel_speeds"}},
 "wheel_yaw":{"scale":{"value":0.99,"std":0.02,"n":14,"converged":false,"ref":"gnss_course"}},
 "speed":{"scale":{"value":1.021,"std":0.002,"n":312,"converged":true,"ref":"gnss"}},
 "gnss":{"lag_s":0.75,"speed_ratio":0.93,"n":249,"converged":true}}
```

| estimate | how | read it as |
|---|---|---|
| `yaw_rate.bias_rps` | mean gyroscope reading while the car stands for 1 s or more | subtracted from every reading |
| `yaw_rate.scale` | between two fixes on straights at most 40 s apart, the integral of the yaw rate has to equal the change of the GNSS course; one sample per turn of 20 degrees or more | true yaw rate = `value` x the fed one. Needs a receiver that sends `course_deg` |
| `yaw_rate.scale_vs_wheels` | sample by sample against the yaw of the rear wheel pair | consistency of the two sensors; not an absolute reference (it contains `track_m`) |
| `wheel_yaw.scale` | as `yaw_rate.scale`, for the wheel-speed yaw | a value away from 1 = `track_m` or the wheel speeds are off |
| `speed.scale` | GNSS speed against the vehicle speed while cruising straight above 8 m/s; without a receiver speed, the straight-line distance between fixes 150 m apart against the distance driven | true speed = `value` x the fed one |
| `gnss.lag_s` | the speed implied by two successive fixes against the vehicle speed 0 .. 3 s earlier, in steps of 0.25 s; the delay with the smallest squared error | the receiver's positions are this late. `speed_ratio` well below 1 = the receiver smooths its track |

`converged` needs enough evidence (12 turns; 60 speed samples; 60 fix pairs with a clear minimum) and a
small standard error (1.5 % for a yaw scale, 0.4 % for the speed).

What is applied by itself (`"use_calibration": false` in `mapd.json` switches it off): the gyroscope bias,
always; a converged `yaw_rate.scale` and `speed.scale`, only to a signal the feeder declared
`uncalibrated`. A signal declared `good` is never rescaled, an `unverified` one is never used, and both
are still estimated, so the numbers tell a host what to declare. `gnss.lag_s` is reported only; a host
that trusts it sets `"gnss_lag_s"` in `mapd.json` or, better, fixes the timestamps it sends.

The sums behind the estimates are kept per source tag in `/var/lib/roadsense/vehicle-calibration.json`
(`calibration_file`; `""` = not kept), so a new drive starts where the last one ended. A new tag (another
car) starts from nothing. Old evidence fades (the last 400 turns / 5000 speed samples count), so new
tyres are followed.

## 10. A minimal feeder

Two message types are a complete feeder: `gnss` once a second and `motion` a few times a second.

```python
from vehicle_client import VehicleFeed          # tools/vehicle_client.py: one file, standard library only; copy it

feed = VehicleFeed("/run/roadsense", name="obd-gps",
                   provides={"gnss": ["lat", "lon", "hacc_m"], "motion": ["speed_mps"]})
while True:
    fix = receiver.read()                        # your GNSS
    feed.gnss(fix.lat, fix.lon, hacc_m=fix.hacc, speed_mps=fix.speed, course_deg=fix.course, source="gps")
    for _ in range(5):
        v = obd.speed_kmh() / 3.6                # OBD-II PID 0x0D: whole km/h
        feed.motion(speed_mps=v, quality={"speed_mps": "uncalibrated"}, source="obd2")
        time.sleep(0.2)
```

With this roadsense has a filtered position, the matched road, the map's speed limit and look-ahead,
dead reckoning through short drop-outs (speed from the car, heading held or from the map), and a lead
tracker that knows when the car really stands. Each further signal adds something: a yaw rate (or wheel
speeds) keeps the heading through tunnels and turns without a fix; the gear makes reversing right; the
turn signal picks the road at a junction; the brake annotates the lead's departure; a radar measures the
lead.

* `tools/vehicle_client.py`: the reference client (`VehicleFeed`): reconnects, never blocks, reads the
  hello reply, sends `controls` / `body` / `vehicle` / `objects` only to a roadsense that accepts them,
  repeats the description after a reconnect. Also a command line: `vehicle_client.py hello`,
  `... gnss LAT LON`, `... controls --brake 1`, `... stdin`.
* `tools/examples/obd_gps_feeder.py`: the loop above for an NMEA receiver and python-OBD; `--demo` runs
  without hardware.
* `tools/vehicle_replay.py`: replays a recorded input stream through mapd's engine offline (what was
  accepted, what it cost, dead-reckoning drift across withheld fixes, calibration at the end).
* `tools/position_client.py`: the v1 client (`gnss`, `motion`, `reset`), unchanged and still valid.
* A complete integration for one car: the Toyota Prius car computer
  (github.com/PiotrTopa/cyberpank-prius-gen2-computer, `backend/position_feeder.py`, docs/POSITION_FEEDER.md).

## 11. Versions and compatibility

* `api` (1) is the major version: it changes only with a change of meaning. `minor` (1) counts
  additions. This document is 1.1; 1.0 was the position input API v1 (`gnss`, `motion`; `reset` and
  `turn_signal` were added to it in November 2026 without a minor number).
* A 1.0 feeder works against 1.1 unchanged and gets the same result: checked message for message on a
  recorded drive (docs/lanes/L18-vehicle-api.md).
* A 1.1 feeder works against 1.0 when it reads `accepts` from the hello reply and holds back what is not
  listed; unknown fields in `motion` are ignored by 1.0.
* New in 1.1 on the output side: `vehicle.sock`, the topics `vehicle` and `objects`, `GET` / `POST
  /v1/vehicle`, three optional fields in `ego`, `lead.radar` / `lead.vision_scale` / `lead.ego.brake`.
  Nothing was removed or changed in meaning.
* Two behaviour changes for 1.0 feeders, both towards the documented intent: a standing car's yaw rate
  is zero (a gyroscope's reading at standstill was integrated before), and wheel speeds outside 0..120
  m/s are ignored instead of used.

## 12. Left out

Deliberately not in 1.1, with the reason:

* **Per-signal timestamps inside one message.** One time per message; send signals of different times
  in different messages. Keeps every message flat.
* **Raw IMU streams** (3-axis gyroscope and accelerometer at 100 Hz and more), **wheel ticks**, **raw
  radar detections / point clouds**: a small host cannot afford them in JSON, and roadsense has no use for
  them yet.
* **Outputs to the car.** roadsense informs; it does not control.
* **Fuel, battery, engine and climate state**: not perception inputs.
* **Navigation route** (the driver's intent beyond the next junction): a different input with a different
  life cycle; open.
* **GNSS raw measurements** (pseudo-ranges, per-satellite C/N0): the receiver's own fix is the input.
