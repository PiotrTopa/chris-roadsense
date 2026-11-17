# L18: vehicle input API 1.1 (2026-11-17)

Goal: one car-agnostic way for a host to hand roadsense what it knows about the car (CAN signals, GNSS,
later a radar and turn signals), make use of it where that is cheap, and keep the one existing integration
(the Prius car computer) working. Contract: `docs/VEHICLE_API.md`. This note holds the decisions, the
measurements and what was left open.

## Decisions

| Decision | Why |
|---|---|
| An extension of the position input API (same socket, `api` stays 1, `minor` 1), not a second API | v1's own rule: additions keep the major number. One socket, one client, one place for clock and frame rules. A v1 feeder needs no change and a 1.1 feeder can talk to an older roadsense by reading `accepts` |
| mapd stays the single entry | it already validated the input and relayed `ego`; a second daemon would cost a Python process on a phone that runs hot. The engine stays free of I/O |
| New output socket `vehicle.sock`; `ego.sock` untouched | `ego.sock` was documented as carrying `ego` messages; a consumer that reads it line by line must not meet a new type |
| Messages grouped by what is measured together (`motion`, `controls`, `body`), one timestamp per message, flat fields | simple to produce from a CAN decoder; per-signal timestamps would nest every value. Partial `motion` messages cover sensors on different buses |
| Four quality words per signal | the reference car has every case: verified (speed), plausible but not calibrated (yaw-rate sensor), decode unconfirmed (accelerator scaling). The words say what roadsense does: use / use and calibrate / show only / ignore |
| Calibration is estimated for every signal and applied only to `uncalibrated` ones | the feeder owns the claim about its signal; roadsense reports what it sees, so the host can change the claim |
| Input in ISO 8855 (y left), outputs in the camera frame (y right), converted in one place (the `objects` relay) | v1 fixed the input signs; every output topic already uses the model's frame |
| Radar fusion in the lead tracker behind a flag, camera decides which vehicle | no radar exists to test with; the camera knows lanes, the radar knows range. A radar alone never starts a track |
| No use yet of wipers, lamps, accelerations, accelerator, cruise | no measured benefit, and the reference car cannot send wipers; in Poland dipped beams are on all day, so `low_beam` says nothing about darkness |

## Tests

| Suite | Result |
|---|---|
| `mapd/tests` (62 before, unchanged, + `test_vehicle.py` 20) | 82 pass |
| `api/tests` (171 before + radar 8, vehicle e2e 1, recorder 2, fusion 1) | 183 pass |
| v1 feeder against 1.1, real drive (2026-11-17, 42 min of CAN as v1 `motion` + 2815 fixes, Poland pack): old engine against new, step by step | 9169 steps, position 0.000 m apart at every step, state, road and limit identical |
| the same drive as a 1.1 stream (all motion signals, controls, body; wheel yaw in use) against the v1 run | state identical; position median 0.000 m, p99 9.4 m, max 15.8 m; road differs in 19 of 6539 located steps. The difference: where the wheel frames were not paired with the speed frame, v1 held the heading and 1.1 uses the steering angle |

## The recorded drive as a test of dead reckoning

Input: the car computer's trip logs of 2026-11-17 (broadcast CAN, 110 min, of which 30 min moving) run
through its real feeder code into a 1.1 stream (36,004 `motion`, 9,705 `controls`, 1,229 `body` messages),
and roadsense's recorder sessions of the same drive. The host did not log its raw fixes, so the recorded
filter positions (while `good`) served as fixes and as the reference (`tools/vehicle_replay.py`).

It did not work as a benchmark, and the reason is the receiver: fixes covered 31 % of the driving time,
in pieces, from 3-5 satellites, and the positions lag and smooth (the other analysis of that drive:
fix-to-fix distance 0.74 x the car's, median). Windows of 10-40 s with a reference at both ends while the
car moved: 1, 3 and 2. In them the error at the end was the size of the distance driven (median 108 m and
111 m for 20 s and 40 s, wheels; 111 m and 142 m, gyroscope), which measures the reference, not the dead
reckoning. Standing windows show the receiver's own wander: 4-11 m median, up to 43 m. No conclusion about
gyroscope against wheel yaw can be drawn from position on this drive.

What the CAN data alone shows (no reference needed: on a straight the car does not turn; steering within
3 degrees, above 8 m/s, 3,743 samples):

| Yaw source | mean on straights | noise (sd) |
|---|---|---|
| rear wheel speeds / track | -0.62 and -0.67 deg/s (two sessions) | 0.98 and 1.49 deg/s |
| yaw-rate sensor (bias from standstill removed) | +0.02 and +0.11 deg/s | 0.29 and 0.35 deg/s |

The wheel yaw carries a bias of about 0.65 deg/s (a tyre circumference difference of about 0.1 %, so it
grows with speed, which the filter's constant yaw-bias state follows only partly): 39 degrees a minute
before the filter's correction, against 1-7 for the gyroscope. Over 24 min of driving the integrated
headings differ by 605 degrees; per minute the two differ by 30-38 degrees (median). The scale of the
gyroscope against the wheel yaw was 1.006 +- 0.009 over both sessions (1.044 on the shorter one alone), so
its scale against truth is still unknown to a few per cent: that is what `calibration.yaw_rate.scale`
will say on a drive with a receiver that sends `course_deg`.

## Cost (laptop core; the phone's little cores were not measured)

| Path | per call |
|---|---|
| `motion` with a speed, filter running | 0.18 ms (0.16 ms before the change) |
| `controls`, `body` | 0.013-0.017 ms |
| `vehicle` message built and serialised (2.4 kB) | 0.12 ms; twice a second plus changes |
| `objects`, 32 objects, JSON in and out | 0.35 ms |
| lead tracker per frame | 0.09 ms; 0.13 ms with a 16-object scan and `radar_fusion` |

At the reference car's rates (motion 10 Hz, controls 1-2 Hz, body 0.2 Hz, no radar) the additions are
about 0.5 ms of a laptop core per second. mapd on the phone ran 20-30 times slower than the laptop in
earlier measurements, which would make it about 1 % of a little core; to be measured there.

## Open

See docs/ROADMAP.md, "Vehicle input API 1.1".
