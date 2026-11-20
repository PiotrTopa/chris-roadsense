# L14: lead-vehicle tracker (topic `lead`), ego relay, position-input reset

Built 2026-11-15 on the dev box (laptop). Nothing was deployed to the phone. Times CET. Contract:
docs/ARCHITECTURE.md ("api-derived topic lead", "mapd": `ego.sock`, `reset`, vision odometry default).

Observation that opened the lane: standing in a traffic jam behind a car that does not move, the distance to
the lead shown in the car flips between about 2 and 3 m.

## Data

* No recording of the car has the lead output: the 2026-11-13 trip log on the phone
  (`~/cyberpunk/logs/trips/trip_20261113_*.ndjson`) holds CAN / BEAN / powerbox traffic and no roadsense
  message; `metrics.db` has `speed` but no lead; the Prius fixture `roadsense_ws_2026-11-14.jsonl.gz` is 22
  fabricated messages. So there is no real jam with the lead output and the wheel speeds together.
* Real lead output therefore comes from the ride library (docs/RIDES.md): `modeld_replay` (supercombo v0.9.7,
  ORT fp32, every frame at 20 fps) over four whole rides with jams and red lights: `a1-s8-rain`,
  `warsaw-city`, `katowice-rain`, `krakow-lights`; 60 min, 71 997 `road` messages, kept (leads, v_ego, pose,
  plan; 15 MB each) in `<datasets>/rides-road/<slug>.road.jsonl.gz` on the laptop, not in the repo.
  How: `ffmpeg -f mjpeg -framerate 20 -i RIDE.mjpeg -vf scale=in_range=full:out_range=full,format=nv12 -f rawvideo - |
  modeld_replay /dev/stdin --width 1280 --height 720 --fps 20 --focal F --calib calib.json --model supercombo-fp32.onnx`
  (in a `systemd-run --user -p MemoryMax=4G` unit; about 12 fps on 2 threads).
* These rides have no ground truth and no wheel speeds. "Reference" below is a centred 1 s median of the raw
  distance (zero phase); "surrogate ego" is a stand-in wheel speed made from the model's own `v_ego` (0.3 s
  median, exactly 0 with the standstill flag after 0.5 s under 0.15 m/s). It exercises the ego path of the
  tracker; it is not a measurement of the car.

## Diagnosis

| question | finding (four rides) |
|---|---|
| quantisation in roadsense | none: `x[0]` is a float; the Prius scene feed rounds to 0.1 m and the MFD prints `int(d + 0.5)`: whole metres |
| existing smoothing | none in roadsense. The MFD director low-passes the lead geometry with 0.30 s, which does not remove a slow wander |
| selection / identity | modeld takes, per output, the better of the model's two lead hypotheses (`supercombo.cc`); at standstill behind one car no switching was seen. While driving the output does jump between vehicles (17 changes / cut-ins in 60 min) |
| noise against distance (frame to frame, about the reference) | robust sd 0.075 m at 0-6 m, 0.11 m at 6-15 m, 0.23 m at 15-30 m, 0.81 m at 30-60 m, 1.6 m at 60-120 m: 1.2-2 % of the distance; 5 % of frames are 0.26-0.47 m off under 6 m. The model's own `x_std` is 4-8 times larger (0.65 m at 4 m): it describes absolute accuracy, not jitter |
| standstill behind a standing car (21 spans, 401 s, gaps 3-8.5 m) | jitter sd 0.13 m (robust 0.08), 3.8 % of frames more than 0.3 m off, single frames up to 1 m (wipers). On top a **slow wander**: 1 s means move by 0.03-0.37 m sd within one stop, the raw value ranges over 0.4-2.7 m per stop |
| bimodal? | no: one peak (bimodality coefficient 0.09, kurtosis 13: a narrow core with heavy tails). Not two hypotheses |
| what a display shows | the value rounded to metres changes 883 times in 401 s (132 per minute); worst spans sit on a half-metre edge (4.44 m: 74 flips in 30 s; 3.45 m: 200 in 65 s) |
| the wander is the model, not the lead | `katowice-rain` 602-684 s: the raw distance drifts 4.3 -> 2.8 -> 3.8 m in ten seconds while a car passes in the next lane; the van ahead is the same size in the frames, the model's lead speed stays at 0.005 m/s and its `x_std` is 1.1 m. With the bonnet hiding where the lead's wheels meet the road the network has no ground-contact cue at 3-5 m and leans on context |
| a usable motion cue | the model's lead speed `v[0]`: p50 0.003 m/s, p99 0.04 m/s behind a standing car, and it rises within 0.1-0.2 s when the lead rolls off. Its relative speed `v[0] - v_ego` matches d(distance)/dt within 0.11 m/s under 6 m, 0.5 m/s at 6-15 m; beyond 30 m it is biased (-0.6 m/s) and noisy (2-4 m/s) |
| the model's `a[0]` | not the derivative of its own `v[0]`; used as a measurement it made the track lag 130-200 ms while closing. Not used |
| the model's `v_ego` at standstill | p50 0.002 m/s, 0.04 % of frames above 0.15 m/s (blips to 0.45 m/s): good enough to detect standing after 1 s, not exact |
| update rate / latency | 10-14 messages per second on the phone (ncnn), 76 ms per run; the tracker adds none (output per `road` message, stamped with the frame time) |

So the 2-3 m flip is jitter plus slow wander of the network's close-range distance, made visible by rounding
to metres; at standstill the true gap is constant and the right estimator is one that knows that.

## Design and placement

In the api (`api/roadsense_api/lead.py`, pure Python, 3x3 Kalman filter, about 0.1 ms per message on the
laptop), next to the other derived topics: modeld stays the network's faithful output (other consumers and the
ORT / ncnn comparison rely on that), and the api is where `road` and the map layer's inputs already meet. The
car's motion reaches it through mapd, which already validates the feeder's `motion` messages: mapd relays each
accepted one as `ego` on a second socket `ego.sock` (20 per second at most, standstill changes at once);
`map` twice a second would be too slow. Without a feeder the same tracker runs on the model's `v_ego`.

Estimator: see ARCHITECTURE.md. In short: state (distance, lead speed, lead acceleration) with the ego speed
as input; measurements `x[0]` and `v[0] - v_ego`; gating instead of averaging for frames that do not fit;
a hold while both stand, ended by a CUSUM on the lead speed or by the ego moving; `departed` when the lead
leaves first.

Trade-off, stated: the filter is fast where it matters (short range, where the distance and the speed are
good) and smooth where the measurements are poor (beyond 30 m). The price is at long range: 135 ms of lag
while closing beyond 40 m. A new nearer vehicle is taken after 0.15 s, a farther one after 0.5 s (in between
the track coasts). At standstill the hold trades truth for steadiness on purpose: the value is frozen, and a
real slow creep of the lead with no speed cue is followed only after it exceeds max(0.6 m, 1.3 x_std).

## Results

Tracker constants as committed. `tools/lead_replay.py` prints the per-ride figures.

Standstill behind a standing car, the four rides (21 spans, 401 s; first second after the stop excluded):

| | raw `x[0]` | tracked, vision only | tracked, surrogate ego |
|---|---|---|---|
| sd within a stop (rms over stops) | 0.219 m | 0.078 m | 0.078 m |
| range within a stop, mean / max | 0.4-2.7 m | 0.18 / 0.63 m | 0.18 / 0.63 m |
| changes of the value rounded to metres | 883 (132 / min) | 11 (1.6 / min) | 10 (1.5 / min) |
| the same at 10 Hz (every 2nd frame) | 481 (72 / min) | 9 (1.3 / min) | 9 (1.3 / min) |
| share of the time in `hold` | | 98 % | 98 % |

The remaining changes come from stops where the model's wander exceeds the hold's thresholds (`warsaw-city`
643-668 s with `x_std` 1.3-1.6 m; 782-794 s at 8.5 m, on a rounding edge while the hold settles).

Moving (reference = centred 1 s median of the raw value; frames in `track` mode):

| range | n | raw rms | tracked rms |
|---|---|---|---|
| 0-6 m | 3 591 | 0.157 m | 0.108 m |
| 6-15 m | 11 952 | 0.214 m | 0.129 m |
| 15-30 m | 5 309 | 0.413 m | 0.270 m |
| 30-60 m | 5 464 | 1.171 m | 0.751 m |
| 60-200 m | 5 452 | 2.492 m | 1.378 m |

Counting the frames in `hold` too, the 0-6 m row reads raw 0.127 m, tracked 0.149 m (n 10 258): the reference
is made of the raw value, so it follows the model's wander and a held value differs from it by design.

Lag while the gap closes faster than 2 m/s: +8 ms under 40 m (n 857), +135 ms beyond (n 1 524).
`departed`: 17 events in 60 min (15 with the surrogate ego), 0.25-0.95 s after the raw lead speed leaves zero
(median 0.45 s), raw distance grown by 0.3 m (median) by then. Each was looked up in the raw series; none was
checked against the video. Events in 60 min: 45 acquired, 43 lost, 11 cut-ins, 6 changes.

Synthetic scenes (noise as measured above; `api/tests/test_lead.py`, 20-40 seeds; here the truth is known):

| case | with vehicle motion | vision only |
|---|---|---|
| 60 s jam at 2.5 m: raw 73 flips / min, sd 0.15 m | 0 flips, sd 0.000 m, error 0.04 m (max 0.10) | 0 flips, sd 0.002 m, error 0.05 m (max 0.12) |
| lead pulls away at 1.2 m/s2, ego stands | `departed` after 0.40 s (max 0.58 s), lead moved 0.12 m (max 0.23 m); 0 false in 40 min | the same |
| lead brakes 4 m/s2 from 20 m/s at 30 m | distance trails the truth by 108 ms on average at its worst moment (max 320 ms over 20 seeds); closing speed past 1 m/s after 0.40 s (truth 0.25 s) | 105 ms (max 303 ms); 0.35 s |
| steady following at 30 m | rms 0.24 m (raw 0.50 m) | the same |
| cut-in 14 m in front of a 40 m lead | new track within 0.4 s, nothing between the two reported | |
| model alternating between two vehicles (2 frames of 5) | track stays on the lead | |

Vision only is steadier than the raw value on every measure above except the one just named; the difference to the ego path is at the edges:
standing is recognised after 1 s instead of 0.3 s, a creeping ego (wheels turning under 0.15 m/s) is not seen,
and a model that believes the car creeps while it stands (seen with a large vehicle filling the frame) prevents
the hold, in which case the output is the filtered value.

## Checks

```sh
.venv-models/bin/python -m unittest discover -s api/tests -t .      # 114: 88 existing + 24 lead + 2 end to end (lead with a feeder, reset over HTTP)
python3 -m unittest discover -s mapd/tests -t mapd                  # 55: 50 existing + ego relay (daemon), ego snapshot, default off, reset, synthetic
tools/lead_replay.py <datasets>/rides-road/warsaw-city.road.jsonl.gz --ego surrogate --events
```
`api/tests/data/lead_*.csv`: two real stops (numbers only: t, v_ego, prob, x, x_std, v, v_std), replayed at
20 and 10 Hz with and without the surrogate ego.

## Also in this lane

* **Vision odometry off by default** in mapd (`"vision_odometry": false`; `--vision-odometry` to switch on).
  An installed `/etc/roadsense/mapd.json` is not overwritten by `install-phone.sh`: the phone's copy still
  says `true` and has to be edited.
* **`reset`** in the position input API (socket and `POST /v1/position`), implicit reset when the GNSS source
  changes between synthetic and real, `map.input.synthetic` / `resets` / `last_reset`;
  `tools/position_client.py` `feed.reset(reason)` and `reset` sub-command.

## Open

* Not verified on the car. Needed: one drive recorded with `/ws?topics=road,ego,lead` (or `road` plus the CAN
  log) through a jam; `tools/lead_replay.py` reads such a capture directly (`ego` lines included).
* Noise figures are from the fp32 backend on every frame of other people's cameras; the phone runs ncnn fp16
  at 10-14 Hz on the DVR camera with an uncalibrated focal length / pitch / height (a scale error on every
  distance that no filter removes).
* The absolute distance at standstill is whatever the model said when the car stopped (its `x_std` there is
  0.5-1.1 m; `dist_std_m` says so). A better close-range distance needs another cue: the apparent width of the
  lead (a vehicle box detector, calibrated once against the model at 10-20 m where it is good) or the ground
  contact line; neither exists in the stack today.
* `departed` was not checked against video frame by frame; the wheel-speed threshold of the Prius sensors
  (below which they read zero) decides how early a creeping ego ends the hold.
* `/preview` and the status page's `road` box still use the raw `leads[0]`; the status page shows both.
* `camera_to_front_m` is unset: `dist_m` is from the camera, like `leads[0].x[0]`.
