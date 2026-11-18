# L24: quality that survives heat (load governor), and what is still to be measured on the phone

Lane of the 2.0 campaign, 2026-11-18 21:20-23:50 CET, Fable. Branch `v2` in roadsense and in roadsense-signs.
Owner's ruling for the lane: quality first, not power saving; go to the maximum, but in a way in which quality
does not suffer when the phone throttles. Updated during the lane: (1) better quality, (2) the same quality for
less, which makes room; a regression in measurement quality is not acceptable for speed.

**State of this report.** The laptop was powered off at 22:28 CET (USB Wi-Fi card driver) and the phone has not
been reachable since. Done and committed: the throttling contract, the governor in every daemon, tests, a
platform emulation on the dev box with the real daemons. **Not done: any measurement on the phone.** The
on-phone proof is section 5, a runbook. The road-model decision (sub-lane L24-M, `v2-l24m`) and the sign / light
quality steps (sub-lane L24-S, `v2-l24s`) were taken over by the coordinator after the power cut and are not
part of this report; the budget table at the chosen configuration depends on them and on the phone, and is
given here only as the estimate of section 4.

## 1. The contract under heat

Before this lane the stack had two private thermal mechanisms: modeld halved its own rate above
`--throttle-temp` (95 C in the unit, 80 C by default) and envd paused itself above 92 C. Everything else ran
until the platform cut the clocks, and the first thing roadsense gave up by itself was the road model.

Now (docs/ARCHITECTURE.md "Load governor"):

* `road`, `lead`, `lanes` keep full rate and latency at any temperature the platform's limiter allows.
* What is shed, in order: 1 preview (one picture a second), 2 recorder pictures and preview, 3 scenery (one
  look in 5 s), 4 display channel `view` (5 a second), 5 sign detector at half rate, 6 light detector at half
  rate, 7 scenery paused and both detectors at a quarter, 8 the road model at half rate. Step 8 is a last
  resort: it is taken only when the limit is being hit (under 1.5 K of headroom now, or the platform limiter
  already cutting), never on a forecast, and `max_level: 7` removes it altogether.
* A rate cap, not a slow-down: a capped stage runs less often on the newest frame; nothing queues, latency per
  message does not grow.
* The consumer is told: `health.governor` (level, what is shed, headroom, the limiter's state, the last 32
  steps with time and reason), and in each measurement topic the measured rate and latency with `degraded` and
  its reasons (`rate` in `lead`, `lanes`, `lights`, `signs`; `rate_hz`, `effective_hz`, `rate_limited`,
  `governor_level`, `limited` in `road`).
* One policy: `--throttle-temp` and `--pause-temp` are accepted and ignored; `road.thermal_c` is gone.

Why this order. The first four steps cost no measurement: pictures for people, a 10 s-smoothed label, the
display. Steps 5-7 cost detector rate: the measured price is in L21 (lights 5 -> 3 runs a second: light-time
reported 98.3 -> 95.5 % by day, lights under 12 px 93 -> 81 %) and L12 (small edge signs are lost by the
tracker below 5 runs a second), so they come after everything that is free, and before the road model, which
every other measurement stands on (the ego light and the sign context use the road message).

Not car-specific, not phone-specific: the probe reads the generic Linux thermal sysfs (a zone's lowest
`passive` trip is its limit, any cooling device with a state above 0 is the limiter), or a reading file the
host writes, or a class the host names. The reference phone needs no configuration (trips at 90 C on the CPU
zones, 85 C on the GPU zones; cooling devices `cpufreq-cpu0`, `cpufreq-cpu4`, the GPU's devfreq).

## 2. What was built

| piece | where |
|---|---|
| policy, probes, caps file, topic marker | `api/roadsense_api/governor.py` |
| the governor's task, `GET/POST /v2/governor`, `health.governor`, `rate` in the topics, `view_hz`, recorder pictures | `api/roadsense_api/server.py`, `state.py`, `recorder.py` |
| reader for the daemons (header-only) | `camerad/governor.h` |
| `preview_hz` | `camerad`, `camerad-dvr`, `camerad-replay` |
| `env_hz` (replaces the 92 C pause) | `envd/main.cc` |
| `road_hz` (replaces `--throttle-temp`), `rate_hz` / `governor_level` / `limited` in `road` | `modeld/main.cc`, `supercombo.{h,cc}` |
| `signs_hz`, `lights_hz` | roadsense-signs `signd/main.cc` |
| units without the thermal flags | `systemd/roadsense-modeld.service`, `roadsense-envd.service`, `config/envd.json` |
| tests | `api/tests/test_governor.py` (14) |
| on-device proof tools | `tools/governor/trace.py`, `cond.sh`, `host.py`, `profile.sh`, `compare.py` |
| dev-box platform emulation | `tools/governor/emu.py`, `emu_stack.sh`, `emu_table.py` |

Without the api (or with `"enabled": false`) the caps file goes stale after 10 s and every daemon runs as
configured: the governor can only ever take load away, and its absence is the old behaviour minus the two
private guards.

Phone: the lane's binaries were built in `/home/user/l24` (build 21:41-21:43 CET, from the sources of commit
5cfc0b9; the later policy changes are Python only). Nothing was run with them: the phone lock was held by lane
L25 until the laptop went down.

## 3. Proof on an emulated platform (dev box, real daemons)

What can be shown without the phone: the real daemons obey the caps, the order is the stated one, the road
outputs hold while the rest is shed, and what a blind limiter does to them instead. `tools/governor/emu.py` runs
camerad-replay, modeld, signd (signs and lights), envd and the api (all on CPU backends) on the ride
`krakow-2024` from its first frame, inside one systemd unit with a CPU quota, and emulates the platform
around them:

* power = the phone's measured budget per unit of work (L21) times the rates the stack is running at:
  1.62 W + 2.61 W x road + 0.45 W x signs + 1.55 W x lights + 0.65 W x scenery, each rate relative to the
  rate this platform reaches at full clock (road 6.2, signs 3.6, lights 3.3, scenery 0.66 a second; the road
  rate is capped at 6.8 as the phone's is bounded by its GPU), so the full stack is 6.9 W as on the phone;
* temperature = first order towards ambient + 7.3 K/W x power, time constant 40 s (shortened bench);
* limiter = a passive trip at 90 C: above it the CPU quota of the whole stack is cut by 4 % a second (to 30 %
  at most), below 88 C given back by 2 % a second. Every stage slows alike, as with a clock cut.

The governor sees this platform through the file probe. Limits of the emulation: the dev box was loaded by
other jobs (the unit ran with a raised CPU weight; its quota, not the box, was the binding limit), the daemons
share CPU where the phone splits work between GPU, big and little cores, the thermal time constant is shorter
than the phone's, and the shed rates are scaled to this platform's full rates. It shows the mechanism, not the
phone's numbers.

Runs of 360 s; the tables use seconds 150-360. `ref` / `ref2`: ambient 22 C, twice (the difference between the
two is the run-to-run noise: the model is recurrent and runs on whichever frames it reaches). Errors are
against `ref` on the same frames (`tools/governor/compare.py`).

| ambient 45 C | `ref` (22 C) | `ref2` (22 C) | `hotN`: no governor | `hotG2`: governor |
|---|---|---|---|---|
| emulated temperature, mean / max over the whole run, C | 72.9 / 74.0 | 71.2 / 72.66 | 88.8 / 90.57 | 82.0 / 88.21 |
| emulated power, W | 6.99 | 6.71 | 6.01 | 5.14 |
| limiter: share of the time cutting / mean cut of the CPU quota | 0 % / 0.0 % | 0 % / 0.0 % | 100 % / 14.8 % | 0 % / 0.0 % |
| governor level, share of the time | 0: 100 % | 0: 100 % | 0: 100 % | 6: 93 %, 7: 7 % |
| road: messages a second | 6.36 | 6.1 | 5.38 | 6.78 |
| road: latency camera -> consumer, median / p90 / p99, ms | 195.2 / 250.8 / 308.1 | 206.4 / 273.3 / 331.3 | 238.4 / 316.8 / 395.7 | 133.5 / 166.1 / 203.9 |
| road: longest gap between two messages, ms | 318.0 | 332.0 | 378.0 | 256.0 |
| road: model ms | 136.5 | 148.4 | 177.2 | 92.7 |
| `lead` / `lanes` marked degraded, % of messages | 0.0 / 0.0 | 0.2 / 0.3 | 10.5 / 11.3 | 0.0 / 0.0 |
| signs: messages a second (marked degraded, %) | 4.26 (0.0) | 4.07 (0.0) | 3.48 (8.9) | 1.74 (100.0) |
| lights: messages a second (marked degraded, %) | 3.9 (0.0) | 3.7 (1.3) | 3.09 (21.3) | 1.59 (100.0) |
| env / view: messages a second | 1.0 / 6.36 | 1.0 / 6.1 | 1.0 / 5.38 | 0.13 / 2.14 |
| ego lines of `road` against the reference at 9 m, median / p95 of dy, m | ref | 0.019 / 0.129 | 0.024 / 0.174 | 0.018 / 0.162 |
| ... at 32 m | ref | 0.046 / 0.217 | 0.055 / 0.272 | 0.043 / 0.251 |
| ... at 61 m | ref | 0.103 / 0.603 | 0.114 / 0.58 | 0.109 / 0.529 |
| tracked right line of `lanes` at 32 m, median / p95, m | ref | 0.04 / 0.17 | 0.047 / 0.203 | 0.035 / 0.157 |
| tracked lead distance, median / p95, relative | ref | 0.01 / 0.038 | 0.014 / 0.066 | 0.01 / 0.042 |
| tracked lead relative speed, median / p95, m/s | ref | 0.407 / 1.96 | 0.463 / 2.52 | 0.413 / 2.38 |
| lead presence disagreeing with the reference, % of frames | ref | 1.18 | 0.89 | 1.27 |

At 45 C the full stack would sit at 95 C. Without the governor the limiter cut the whole stack by 15 % and held
90 C: road 6.4 -> 5.4 messages a second, latency median 195 -> 238 ms (p99 308 -> 396 ms), every tenth `lead` /
`lanes` message marked degraded (`limited` + `slow`), and all detectors slowed too. With the governor the
limiter never engaged (maximum 88.2 C): it shed in the stated order to level 7 during the heat-up (preview,
recorder pictures, scenery within 3 s; view, signs, lights, detector floor at 8 s intervals), gave the detector
floor back after 90 s and stayed at level 6. Road, lead and lanes: full rate (6.8 a second, the cap), latency
lower than the cool reference (133 against 195 ms median), outputs within the run-to-run noise of the two
reference runs at every distance, none marked degraded. On this emulated platform the road model even gains
(the daemons share CPU, so what the detectors give up it gets); on the phone the gain is the GPU time of the
sign detector only. Signs and lights ran at half rate and every message said so (`shed`, `slow`).

| ambient 55 C | `ref` (22 C) | `hotN55`: no governor | `hotG55`: governor |
|---|---|---|---|
| emulated temperature, mean / max over the whole run, C | 72.9 / 74.0 | 88.9 / 91.68 | 84.6 / 91.55 |
| emulated power, W | 6.99 | 4.69 | 4.04 |
| limiter: share of the time cutting / mean cut of the CPU quota | 0 % / 0.0 % | 100 % / 41.1 % | 0 % / 0.0 % |
| governor level, share of the time | 0: 100 % | 0: 100 % | 7: 62 %, 8: 38 % |
| road: messages a second | 6.36 | 3.82 | 5.39 |
| road: latency camera -> consumer, median / p90 / p99, ms | 195.2 / 250.8 / 308.1 | 333.5 / 427.1 / 519.4 | 119.8 / 153.8 / 192.9 |
| road: longest gap between two messages, ms | 318.0 | 509.0 | 434.0 |
| road: model ms | 136.5 | 255.1 | 79.7 |
| `lead` / `lanes` marked degraded, % of messages | 0.0 / 0.0 | 96.9 / 97.0 | 22.3 / 22.3 |
| signs: messages a second (marked degraded, %) | 4.26 (0.0) | 2.24 (99.2) | 0.9 (100.0) |
| lights: messages a second (marked degraded, %) | 3.9 (0.0) | 2.0 (100.0) | 0.82 (100.0) |
| env / view: messages a second | 1.0 / 6.36 | 1.0 / 3.82 | - / 1.91 |
| ego lines of `road` against the reference at 9 m, median / p95 of dy, m | ref | not comparable (1) | 0.02 / 0.171 |
| ... at 32 m | ref | not comparable (1) | 0.055 / 0.28 |
| ... at 61 m | ref | not comparable (1) | 0.12 / 0.697 |
| tracked right line of `lanes` at 32 m, median / p95, m | ref | not comparable (1) | 0.047 / 0.275 |
| tracked lead distance, median / p95, relative | ref | not comparable (1) | 0.012 / 0.092 |
| tracked lead relative speed, median / p95, m/s | ref | not comparable (1) | 0.41 / 2.74 |
| lead presence disagreeing with the reference, % of frames | ref | not comparable (1) | 1.24 |

At 55 C not even level 7 fits. Without the governor the limiter cut 41 %: road 3.8 a second, latency median
334 ms (p99 519 ms), 97 % of `lead` / `lanes` messages marked degraded. With the governor: level 7 for 62 % of
the time, and for 38 % the last resort, the road model at half rate; the limiter did not cut in the window. (In the
first 70 s of that run it did: the emulated heat-up, 25 K in 20 s from a cold start at 55 C, outran eight steps,
and the limiter cut for 48 s, up to 60 %, until the shedding had caught up; maximum 91.6 C.) Road 5.4 a second on
average with a latency of 120 ms median (154 ms p90): fewer messages, each one fresh, and within the noise of the
reference in content; 22 % of `lead` / `lanes` messages carried `degraded` (`shed`), which is the honest part.
The road step went in and out (two cycles of about 80 s on and 135 s off in the run): the hold doubles on each
regret, so it lengthens, but at an ambient that sits between two levels the governor will alternate. Open point
for the phone traces.

(1) In `hotN55` the replayed camera itself was starved by the quota cut and dropped 524 of 7,253 frames, so a
frame number no longer names the same picture as in the reference and the content comparison is void (the
tool printed 0.3-0.9 m line errors, which measure the misalignment). Rates and latencies are valid.

Reproduce (dev box; builds of `v2` with the system ONNX Runtime in `~/datasets/l24gov/build-rs`, `build-signs`):

```
PY=~/roadsense-maps/.venv/bin/python; N="--out OUT --nominal 6.2,3.6,3.3,0.66 --road-hz 6.8 --quota0 300 --seconds 360"
$PY tools/governor/emu.py ref   --ambient 22 --governor on  $N
$PY tools/governor/emu.py hotN  --ambient 45 --governor off $N
$PY tools/governor/emu.py hotG2 --ambient 45 --governor on  $N
python3 tools/governor/emu_table.py OUT ref ref2 hotN hotG2 --from 150 --to 360
```

### What the emulation changed in the policy

The first version (commit 5cfc0b9) failed on it, in two ways worth keeping:

1. It treated an active limiter as a reason to shed a step every 3 s. One overshoot of the trip was enough: the
   limiter's states linger while the temperature already falls, and the governor emptied the whole list down to
   the road model and stayed there.
2. With steps every 3 s on a forecast it ran through to the road step during the first heat-up although the
   headroom never fell under 2.8 K (run `hotG` in the output directory), then restored a step, had to shed it
   again within a minute, and oscillated.

Now: shed on the headroom expected 10 s ahead, but only while the temperature is not already falling; quick
steps only after the two that save nothing; 3 s steps only with under 1.5 K left now; the road model only when
the limit is really being hit; a restored step that had to be shed again needs 3 K more headroom and twice the
hold the next time.

## 4. Budget and ambient temperature (estimate until section 5 is run)

From the bench point given to the lane (10.0 W at 94-95 C in 22 C air, fan 100 %): 7.3 K/W to the air, taken
as linear and as valid for the hottest zone's average; the air at the phone is taken as the cabin air. The
governor starts to shed at 86 C (4 K under the 90 C passive trip), so with nothing shed the phone can spend
(86 - Ta) / 7.3 W. The per-step savings are from L21's subtraction table (scenery 0.65 W, signs 0.45 W at 5 runs
a second, lights 1.55 W at 5, road model 2.6 W at 13.8), not measured per level.

| governor level | stack W | runs with nothing more shed up to (air at the phone) | road / lead / lanes |
|---|---|---|---|
| 0: the stack proposed by L21 (envd little, lights 5 Hz) | 7.0 | 35 C | full |
| 3: scenery once in 5 s | 6.5 | 38 C | full |
| 6: signs and lights at half rate | 5.7 | 44 C | full |
| 7: scenery paused, detectors at a quarter | 5.1 | 49 C | full |
| 8: road model at 10 runs a second | 4.2 | 55 C | half rate, same latency |
| above | - | - | the platform limiter; topics marked `limited`, `slow`, `late` |

Readings:

* The full stack runs with nothing shed up to about 35 C of air at the phone with the fan at 100 %. That is
  below a summer cabin. Quality is the priority, so the answer is not a smaller stack but the order above: up to
  about 49 C the road outputs stay whole and the detectors pay.
* Every watt added for quality (task 3) moves these temperatures down by 7.3 K per watt; every watt freed by a
  cheaper road model or a light detector on the GPU moves them up by the same. The freed budget of the
  sub-lanes should be read through this table.
* What the fan policy must do (the car repo's `fan_policy.py`, not edited): reach 100 % before the governor
  sheds anything, i.e. full duty from 80 C on the hottest zone, or whenever `health.governor.level` > 0 or
  `health.governor.headroom_k` < 10; and not fall back while the level is above 0. With the present policy
  (about 31 % for roadsense on the bench) the path to the air is longer than 7.3 K/W and every temperature in
  the table is lower by an amount that only the runbook measures.

## 5. Pending: the proof on the phone (runbook)

Preconditions: phone reachable (`ssh user@10.42.0.1`), `/run/phone-lane.lock` absent (create it with `L24`,
remove it at the end), `prius-netwatch` masked for the duration (`systemctl mask --runtime --now`; whoever
finishes last unmasks and restarts it), clock right (`date`), nothing of L24 left running
(`systemctl list-units 'l21-*' 'l24-*'`; the low-priority build unit `l24-build` had finished at 21:43).
Root: `dotinfra vault exec poco_user_password -- ssh user@10.42.0.1 "sudo -S -p '' <cmd>"`. Safety stop:
`trace.py` brings the stack down when the hottest zone stays above 95 C for 3 s.

1. Sources and tools (the C++ binaries in `~/l24/build-rs`, `~/l24/build-signs` are current; only Python and
   scripts changed):
   `tar czf - -C /home/piotr/projects/wt/roadsense-v2 api tools | ssh user@10.42.0.1 'cd ~/l24/roadsense && tar xzf - && cp tools/governor/*.py tools/governor/*.sh tools/budget/stack.sh ~/l24/'`
2. Functional check, 2 min, fan 100 %: `sudo sh ~/l24/cond.sh smoke 6 100 90` then
   `python3 compare.py out/smoke.jsonl`: signs and lights at 2.5 a second and marked `shed`, `view` at 5, `env`
   once in 5 s, road at its full rate, `governor_level` 6 in `road`.
3. Watts per level (fills the table of section 4 with measurements), fan 100 %, 150 s each:
   `for l in 0 3 4 5 6 7 8; do sudo sh ~/l24/cond.sh lvl$l $l 100 150; done`; read `platform.rail_w`,
   `big_mean`, `road.hz` of `compare.py out/lvl$l.jsonl --from 60`.
4. The fan duty at which the ungoverned stack sits at the limit (90-94 C, cooling devices active) without
   reaching the 95 C stop: `sudo sh ~/l24/cond.sh pilot off F 300` for F = 60, 40, 25 (wait 10 s between fan
   changes; the script sets the fan once). Take the highest F whose last minute has `limiter_active_pct` > 50.
5. The proof, 480 s each, same ride from its first frame:
   `cond.sh ref on 100 480`; `cond.sh hotN off F 480`; `cond.sh hotG on F 480`; `cond.sh ref2 on 100 480`;
   then on the laptop `python3 tools/governor/compare.py ref.jsonl ref2.jsonl hotN.jsonl hotG.jsonl --from 180`.
   Also, for the record of what the installed binaries do at the same fan duty (their private guards):
   `sudo sh ~/l24/cond.sh hotOld off F 480 ~/l21/profiles/F-final.sh` (L21's builds).
6. Pass criteria for `hotG` against `ref`, with `ref2` as the noise: road messages a second not lower than
   `ref2`'s; road latency median and p90 within 10 ms; lane-line and lead errors within twice `ref2`'s; limiter
   active less than 5 % of the time; `lead` / `lanes` never marked degraded; the shed steps in the stated order
   in `health.governor.events`. For `hotN`: report the same numbers; no criterion.
7. If `hotG` still reaches the limiter at the chosen F, lower `shed_at_k` is not the fix: record which level it
   settles at and the rail W, and repeat with F one step higher. If the governor oscillates, record the events
   and raise `restore_at_k`.
8. Leave the phone as found: `python3 ~/l24/host.py fan_auto`, `python3 ~/l24/host.py roadsense_mode
   '{"mode":"auto"}'`, `sudo L21_HOME=/home/user/l24 sh ~/l24/stack.sh down`, remove the lock, unmask and start
   `prius-netwatch` if no other lane holds the phone.

Also still to do on the phone: thermal time constants (the dwell and hold times are sized for a response of
tens of seconds; the trace of step 5 gives them), and the car: the governor has never seen the DVR camera,
the cabin, or the car's fan policy.

## 6. Tests

| suite | command | result |
|---|---|---|
| roadsense api | `~/roadsense-maps/.venv/bin/python -m unittest discover -s api/tests` (in the repo root) | 225 passed |
| mapd | `~/roadsense-maps/.venv/bin/python -m unittest discover -s mapd/tests -t mapd` | 93 passed |
| signd | `ctest` in a build of roadsense-signs `v2` | 1 of 1 passed (`lights_tracker`) |
| car computer | in `wt/prius-v2`: `PRIUS_ROADSENSE_REPO=<roadsense v2> SDL_VIDEODRIVER=dummy ~/projects/cyberpank-prius-gen2-computer/.venv-fe/bin/python -m pytest -q tests` | 1712 passed, 9 skipped |

The interpreter question. The api tests are `unittest` tests and need `aiohttp`; no virtualenv on this box has
both `aiohttp` and `pytest` (`~/roadsense-maps/.venv` and `~/projects/roadsense/.venv-models` have aiohttp,
the car repo's `.venv` and `.venv-fe` have pytest), so the working command is `unittest discover` with
`~/roadsense-maps/.venv`. The hang the coordinator saw is reproduced with the system `python3` (no aiohttp):
`test_api` and `test_map_e2e` start the api and the fake daemons as subprocesses in `setUpClass`; the api
exits at once, the wait times out, and `unittest` does not call `tearDownClass` after a failed `setUpClass`,
so `fake_daemons.py` stayed behind holding the runner's stdout: anything that reads the output to its end
(`| tail`, a harness) waited for ever. Fixed: both set-ups clean up when they fail and say that the api exited;
the system `python3` now ends in 48 s with 4 errors (`No module named 'aiohttp'`) and leaves nothing behind.
The car suite needs `.venv-fe` (pygame) and the `tests` directory as the argument (the repository root holds
scripts that open a serial port on import).

## 7. Rejected, and open

* **Governor as a daemon of its own**: rejected for now. It lives in the api (one process fewer; the api is the
  one place every consumer already asks). The cost: without the api nothing is shed. The daemons then run as
  configured and the platform limiter guards, which is the behaviour before this lane.
* **Shedding by lowering clocks or thread counts**: rejected. That is what the limiter does; a rate cap keeps
  the latency of what still runs.
* **Fully pausing detectors**: rejected; the floor is a quarter rate (1.25 a second).
* **Tracker ages in frames**: at a capped rate the sign and light trackers' frame-counted ages (`--lights-coast`,
  `max_age`) last longer in time. Not changed; to be judged on the phone traces.
* **`env` carries no marker**: it is slow by design; `health.governor.shed` says when it is slowed or paused.
* **Per-level watts, the fan duty of the proof, time constants**: section 5.
* **Road model decision, sign / light quality steps, final budget table at the chosen configuration**: with the
  coordinator (sub-lanes L24-M and L24-S).
* **The BDD YOLOv8n checkpoint** (light detector with every layer on Vulkan, int8): not touched, needs the
  owner's go, as before.
