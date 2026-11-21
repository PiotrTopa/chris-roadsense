# L53: three owner decisions of 2026-11-21, installed and measured on the phone

Execution lane, 2026-11-21. The owner adopted three candidates of lane L48 (docs/lanes/L48-system-optimisation.md,
L48-envd.md, L48-phone-results.md):

1. **envd every 2 s**: one inference every 2 s, the last message re-sent every second (`--period 2 --republish 1`).
2. **No CPU wake-up latency hold**: the modeld unit no longer passes `--cpu-latency-us 0`.
3. **Light detector idle pattern**: `signd --lights-idle ZW.Z.W` (roadsense-signs). While no light is tracked,
   one frame period in three gets no detector pass. The owner allowed "even one in four"; quality decides, see
   section 3.

## What changed

| where | change |
|---|---|
| `systemd/roadsense-envd.service` | `--period 2 --republish 1`. The binary's default and `config/envd.json` stay at 1 Hz; the unit sets the rate because the command line beats `envd.json` |
| `systemd/roadsense-modeld.service` | `--cpu-latency-us 0` removed; the option stays in modeld (`--cpu-latency-us N`, `cpu_latency_us`) |
| roadsense-signs `systemd/roadsense-signd.service` | `--lights-idle ZW.Z.W` |
| roadsense-signs `signd` | a `lights` message sent while the pattern runs and nothing is tracked carries `"idle": {"pattern": "ZW.Z.W", "hz": 3.33}` (passes a second at the current `--lights-hz`, after the governor's cap) |
| `chris-integrator/chris_integrator/plugins/common.py` (`Rates`) | while the newest `lights` message announces `idle` (and for one 2 s rate window after the last one), the topic's floor is 0.8 x `idle.hz`. The rate is then counted from the first to the last arrival, so a short window does not pull it down; an arrival more than two announced periods late counts up to now, so a stall shows. With a light tracked the full floor (3 a second) applies. The marker gains `idle_hz` |
| tests | `chris-integrator/tests/test_env_rate.py` (env has no floor; the env area prior acts the same at 2 s with re-sent copies), `tests/test_lights_idle_rate.py` (idle at the announced rate is healthy, an idle stream behind its announcement is `slow`, idle -> tracked -> slower than the full floor), roadsense-signs `tools/road_users_test.cc` (the `idle` key) |

The consumers of `env` need no change: no rate floor or staleness rule exists for `env` in chris-integrator or
the debug face; the car backend drops `env` after 5 s and `lights` after 2 s (the longest idle gap is 0.4 s).

## Method on the phone

- Replay of the same rides (`scripts/use-replay.sh --enable` per ride, `--playlist` for the last row), full
  stack, little cores at their top clock.
- Each row: wait until the hottest zone is under 70 C, settle 60 s, then 240 s of `tools/budget/measure.py`
  (CPU per unit and thread, little / big; GPU; rail power; thermal zones; aborts above 95 C).
- During each row: `tools/budget/snap_ro.sh` 60 s, the `envd.sock` and `lightd.sock` arrivals for 60 s,
  `/v2/health` and `/v2/mixer` in the middle, the governor file every 5 s.
- Changes were added one at a time, so each row differs from the one before by a single change.
- Road latency: a 45 s WebSocket probe on the integrator.
- Light quality on the phone's own backend: `lights_replay` (ncnn CPU, fp16, 2 big threads, production options)
  on the first 5 minutes of `krakow-lights` and `krakow-night-tunnel`, every 4th frame, stack stopped. Scored on
  the laptop with roadsense-signs `tools/lights/replay_score.py` against the L15 reference, as L48's P2.
- One measured row of each kind; run-to-run noise on this bench is about 0.2 W and 2 K.

## 1. envd every 2 s (rural-zloczew; rows R0 -> R1)

| | 1 Hz (as installed before) | 2 s + re-send every 1 s |
|---|---|---|
| envd, % of one little core (measure.py / snap_ro) | 179.5 / 177.1 | **89.1 / 88.2** |
| little cores in total, % of 400 | 255 | 169 |
| rail W, last 60 s | 7.01 | **6.34** |
| big / little / hottest C, last 60 s | 72.6 / 67.9 / 73.2 | 71.1 / 65.9 / 72.0 |
| road runs / s | 14.6 | 15.6 |
| inferences / s (journal) | 1.00 | 0.50 |
| `env` messages in 60 s (distinct `seq`) | 62 (61) | 62 (31) |
| arrival interval median / max | 1.00 / 1.07 s | 1.00 / 1.08 s |
| governor level | 0 | 0 |

- `/v2/health` and `/v2/mixer` show `env` at 0.5-0.9 a second. That figure is a 2 s window's count and has no
  floor, so `env` is never marked `slow` or `degraded`; its age stays under 1 s.
- **Caveat accepted by the owner** (laptop study, L48-envd.md section 2): against 1 Hz the env area prior
  disagrees 0.72 % of the time, against a 1 Hz phase noise of 0.17-0.37 %. Weather / `wet` is 0.3-0.4 points
  above the noise. Place, light, the tunnel and every map-backed decision stay inside the noise. Tunnel entry
  p90 is 2.0 s, against 0.5 s at 1 Hz.

## 2. No wake-up latency hold (krakow-2024; rows H0 -> H1, both with envd at 2 s)

| | hold (`--cpu-latency-us 0`) | no hold |
|---|---|---|
| rail W, last 60 s (mean of 240 s) | 6.83 (6.54) | 6.70 (6.44) |
| big / little / hottest C, last 60 s | 70.0 / 65.7 / 70.9 (start 70.9) | 71.9 / 65.8 / 72.7 (start 68.9) |
| road runs / s (probe) | 15.24 | 15.18 |
| `road` latency median / p90 | 93.3 / 122.2 ms | 93.8 / 124.2 ms |
| `lead` latency median / p90 | 96.0 / 124.9 ms | 96.3 / 127.8 ms |

- Without the hold modeld makes no request: its journal has no `CPU wake-up latency request` line after the
  unit change, and its command line has no `--cpu-latency-us`.
- Rate and latency are the same within noise. Power is 0.1 W lower. The big-core temperature differs by less
  than this bench's noise. L48 P3 measured 0.1-0.3 W and 4.7 K with different start temperatures; this run
  confirms the power side, not the 4.7 K.

## 3. Light detector idle pattern `ZW.Z.W`

### On the phone (rows L0 -> L1; envd 2 s and no hold in both)

| ride | light thread, % of a big core | signd big | rail W, last 60 s | big C, last 60 s | lights fps | `lights` marker |
|---|---|---|---|---|---|---|
| rural-zloczew (no lights) | 87.9 -> **59.0** (-33 %) | 94.7 -> 66.0 | 6.08 -> **5.67** | 70.1 -> 68.2 | 4.99 -> 3.29 | idle: `hz` 3.8, `idle_hz` 3.33, not degraded |
| krakow-lights (lights) | 89.3 -> **75.9** (-15 %) | 103.7 -> 89.9 | 6.44 -> 5.58 (mean 6.52 -> 6.09) | 67.8 -> 65.9 | 4.99 -> 3.72 | tracked: `hz` 4.7, full floor, not degraded |

- On rural-zloczew 196 of 203 lights messages in 60 s announced `idle`; the longest gap was 0.42 s.
- In L48's P2 the same pattern was marked `slow` at 2.8 a second under the fixed 3-a-second floor. With the
  announced rate it is healthy, and the full floor still applies whenever a light is tracked.

Light quality on the phone's backend (first 5 minutes, every 4th frame):

| ride | without | `ZW.Z.W` | one pass in 2 (`W.Z.`) |
|---|---|---|---|
| krakow-lights: light-time, 24-48 px, first report | 96.9 %, 93.6 %, 0.40 s | 96.8 %, 92.4 %, 0.50 s | 96.6 %, 91.7 %, 0.50 s |
| krakow-night-tunnel: light-time, 12-24 px | 98.9 %, 99.0 % | 99.5 %, 100 % | 97.4 %, 96.1 % |
| frame periods without a pass | 0 | 16.5 % / 20.9 % | 24.9 % / 32.3 % |

`ZW.Z.W` repeats L48's P2 exactly (96.9 -> 96.8 %, 98.9 -> 99.5 %, first report +0.1 s).

### Sparser patterns (laptop)

Scored by L48's method (ORT, the same reference). Noise is the normal schedule started on the other view:
light-time 0.0 points by day and by night, 0.7 at 24-48 px, first report 0.40 / 0.50 s.

| idle pattern | idle passes saved | light-time day | 24-48 px day | first report day | light-time night | expected saving on idle roads |
|---|---|---|---|---|---|---|
| none | 0 | 97.1 % | 94.4 % | 0.40 s | 98.9 % | - |
| **`ZW.Z.W` (installed)** | 1/3 | 97.0 % | 93.3 % | 0.50 s | 99.5 % | 0.41 W measured (rural row above), 29 points of a big core |
| one pass in 2 (`W.Z.` / `Z.W.`) | 1/2 | 96.8 / 96.7 % | 92.7 / 91.7 % | 0.50 / 0.40 s | 97.9 / 98.9 % | about 0.6 W (scaled) |
| one pass in 3 (`W..Z..` / `Z..W..`) | 2/3 | 95.4 / 95.9 % | 86.0 / 87.4 % | 0.70 / 0.50 s | 97.9 / 96.3 % | about 0.8 W (scaled) |
| one pass in 4 (`W...Z...` / `Z...W...`) | 3/4 | 96.0 / 94.9 % | 88.4 / 82.5 % | 0.50 / 0.50 s | 94.4 / 97.1 % | about 0.9 W (scaled) |

- Every pattern still reports every reference light at least once (27/27 by day, 16/16 by night).
- **One pass in 2 is not installed.** It passes the coarse lines (light-time within 1.5 points, first report
  +0.1 s), but it loses 1.7-2.7 points of the 24-48 px (far) light-time against a noise of 0.7 on the laptop,
  and 1.9 points on the phone's backend. That is a measurable loss of far lights.
- One in 3 and one in 4 lose 1-4.5 points of light-time.
- The owner can overrule knowingly with these numbers. The savings column is the measured `ZW.Z.W` power
  saving scaled by the share of passes saved (inferred, not measured).

## The phone with all three on (playlist, 240 s)

| | |
|---|---|
| rail W, last 60 s (mean) | **5.87** (5.95). Row R0 with none of the three: 7.01; L48's whole-bench budget: 6.8-7.2 |
| little cores, % of 400 | 167: envd 88.5, the car backend about 28, chris-integrator 24.5, modeld frame prep 11.9 |
| big cores, % of 400 | 110: signd 87.1 (light thread 76.6), modeld 19.5, camerad-replay about 10 |
| GPU busy | 83.7 % |
| big / little / GPU / hottest C, last 60 s (max) | 69.1 / 64.0 / 62.9 / 70.8 (84.7; the row started at 83.1 after the cool-wait limit) |
| road runs / s | 14.8 |
| governor level | 0 throughout |
| `env` | 62 messages in 60 s, 31 inferences, never degraded |
| `lights` | 264 messages in 60 s, 72 announcing idle; marker healthy |
| journals | no errors in the stack's units over every row |

Against the L49 deploy's budget (L48-phone-results.md: little 270 %, big 118-130 %, 6.8-7.2 W) the phone now
uses about 100 points less of the little cores, about 15 points less of the big cores and about 1 W less on
the playlist.

## What happened on the bench

- **The phone reset itself once, at 00:58:58 (lane's first night).** The stack had just been switched off by the
  car backend: the camera hub relay (ch4) went off, then `uhubctl` disabled port 1-1/4, and the journal ends
  there. No pstore record exists.
  - Cause not established. It is either the hardware watchdog after a hang, or a supply dip when the relay
    switched (the phone runs without a battery).
  - The same key-off sequence runs at every key-off in the car, so this can happen there.
  - Side effects:
    - A `roadsense-signd.service` being written at that moment was left empty, so systemd treated it as masked.
    - The phone came back with its clock at the last saved timesync value (no RTC, no NTP because the modem
      reports `sim-missing` since the day before), so it ran about 11 h behind.
    - The replay restarted with the stack after boot and ran for 11 h. The recorder's 4 GB cap then deleted the
      198 older recorder sessions. 18 of them have no copy on the laptop: 20261118-185123, -190340, -194014,
      -195616, -202334, 20261119-011416, -015403, -033032, -050016, -071452, -091546, -114923, -124243, -131145,
      -132334, -164206, -223650, -232330 (bench restarts and possibly drives).
  - The second part of the run swapped units with a unit restart instead of switching the stack off, and ran
    under `systemd-inhibit`.
- No suspend was involved: the journal has no sleep entries, and `/proc/uptime` matches the boot.

## State left (2026-11-21 12:46 CET)

- Stack active on the dashcam source:
  - `roadsense-camerad-dvr` enabled and active; `roadsense-camerad-replay` disabled.
  - Backend mode `auto`. There is no camera on the bench, so the backend's 5-minute restart loop runs.
- envd runs `--period 2 --republish 1`. modeld runs without `--cpu-latency-us`. signd runs `--lights-idle ZW.Z.W`
  and is unmasked, enabled and active.
- The integrator has the idle-aware `common.py`.
- Learned state restored and verified by checksum.
- Bench recorder sessions moved to the rollback folder.
- Clock set by hand from the laptop.
- Rollback: `<phone-home>/rollback-20261121-l53/` (`RESTORE.txt`).
