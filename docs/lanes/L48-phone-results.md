# L48 phone orders: results (run by lane L49, 2026-11-20 12:07-14:08 CET)

Orders: `orders/L48-phone.md` (P0-P3), `orders/L48-envd-phone.md` (envd rows), car repo `docs/lanes/L48-B-order.md`
(bench part). Phone: Poco F1 on the bench, the stack of roadsense `v2-l49` fae9098 + roadsense-signs `l48-opt` c8998b5
+ car backend `l48-opt` c7a8b81 installed by L49 at 12:01 (new options at their defaults). Frames from the replay
playlist (one ride per run where the order says so). Fan on its automatic policy (41-46 % in every row). Every row:
`tools/budget/measure.py` 240 s (envd rows 600 s) after 60 s (120 s) of settling, the stack's units plus the car
backend by pid. Raw files on the phone in `<phone-home>/l48/out` and `<phone-home>/l48/lq`. CPU in % of one core, little / big.

**Deviation from the orders:** the candidates (P1, P2, envd B/C) ran against an installed stack that already
carries `v2-l48`'s debug face and the new signd / envd binaries (L49 deployed before measuring, as its brief says).
So P0 is the deployed stack with the options off; P1 measures the OLD face (9a9ba27 api through a drop-in) as the
comparison row; P2-0 (lane build without the option) is P0 itself (the same binary).

## P0: baseline (deployed stack, defaults)

| ride | little | big | GPU | W | big / little / GPU C, last 60 s | max C | road runs/s | lights fps, ms | envd fps, ms | governor |
|---|---|---|---|---|---|---|---|---|---|---|
| rural-zloczew | 269 | 121 | 84 | 6.90 | 72.8 / 68.3 / 62.8 | 90.3 | 15.1 | 4.99, 94.6 | 1.00, 856 | 0-1 |
| krakow-lights | 271 | 130 | 85 | 7.21 | 71.3 / 66.7 / 63.1 | 90.0 | 15.5 | 4.99, 94.8 | 1.00, 840 | 0-1 |
| a1-night | 271 | 118 | 85 | 6.75 | 73.4 / 68.1 / 62.3 | 86.3 | 15.0 | 4.99, 93.6 | 1.00, 870 | 0 |

Per unit (all three rides): envd 184-186 / 0; chris-integrator 34-37 / 0; car backend 28-29 / 0 (main 9, CAN reader
6.3-6.5, scene thread 5.5-6.8); modeld 12-14 / 20, GPU 58-59; signd 0 / 94-104 (lights 88-89, signs 7-14), GPU 25-26;
debug face 6 / 0; camerad-replay 1 / 4-7. Per core: cpu0-2 84-85 %, cpu3 35-36 %, cpu4-7 33-37 %. GPU at 710 MHz
all the time.

## P1: debug face

| | old face (9a9ba27, unpinned) | installed face (`v2-l48` f777ee6, `CPUAffinity=0-3`) |
|---|---|---|
| `roadsense-api` little / big | 1.0 / 4.4 | 6.2 / 0 (6.4, 6.3 on the other rides) |

Checks on the installed face: `/v1/health` fps.road 14.76 (P0 14.55) with `model_ms` 42.7; `/v1/road` seq 6515 -> 6535
in a second; `/run/roadsense/governor` rewritten (ts 4.05 s apart over 3 s + reads); the integrator's `/v2/health`
shows the governor (level 0). Effective CPUs of the face: 0-2 (the slice's AllowedCPUs 0-2,4-7 ∩ 0-3).

**Not a pass, not a fail.** Big-core time 0 and every check true, but the total is 6.2-6.4 % of a little core,
above the 6 % line; not above the fail line (3 x the old face's big share = 13 %). The work did not shrink by
much in CPU time (5.4 % mixed -> 6.2 % little); it moved off the big cores (about 0.09 W -> 0.02 W by L21's figures).

## P2: light detector's idle pattern `--lights-idle ZW.Z.W`

| ride | `sg-lights` big, P0 -> P2-A | periods skipped | needed fall (0.8 x share) | W | big C last 60 s | road runs/s | `lights` marker at the end |
|---|---|---|---|---|---|---|---|
| rural-zloczew | 87.7 -> 59.0 (-33 %) | 33.0 % | 26 % | 6.90 -> 6.31 | 72.8 -> 66.8 | 15.1 -> 15.7 | **2.8 Hz, degraded, why `slow`** |
| krakow-lights | 89.2 -> 76.7 (-14 %) | 20.5 % | (not required) | 7.21 -> 6.97 | 71.3 -> 67.0 | 15.5 -> 15.3 | 3.8 Hz, ok |
| a1-night | 87.7 -> 61.8 (-30 %) | 29.8 % | 24 % | 6.75 -> 6.29 | 73.4 -> 67.3 | 15.0 -> 15.8 | 3.7 Hz, ok |

signd published 3.29-3.50 lights messages a second on the idle rides (journal), but the integrator's `lights` topic
ran at 2.8 Hz on rural-zloczew and was marked `degraded` / `slow` (the API's floor is 3 a second).

Quality on the phone's own backend (`lights_replay`, ncnn CPU fp16, 2 big threads, production options, every 4th
frame of the first 5 min, stack stopped), scored with `tools/lights/replay_score.py` against L15's reference files:

| ride | light-time without -> with | first report | periods skipped | wall time of the run |
|---|---|---|---|---|
| krakow-lights | 96.9 -> 96.8 % (24-48 px 93.6 -> 92.4) | 0.40 -> 0.50 s | 16.5 % | 168 -> 140 s |
| krakow-night-tunnel | 98.9 -> 99.5 % | 0.00 -> 0.00 s | 20.9 % | 161 -> 130 s |

**FAIL** (as the order defines it): `lights` was degraded with why `slow` on rural-zloczew because of the pattern;
also road runs a second 15.3 against 15.5 on krakow-lights (within run-to-run noise, but the line says "not lower").
Everything else held: CPU fall above the required share on both idle rides, big-core temperature lower on every
ride (-4 to -6 K), 0.24-0.59 W less, light-time within 1.5 points on both rides, first report at most 0.1 s later.
The option was switched back off; nothing tuned.

**Adopted by the owner 2026-11-21 (lane L53):** `--lights-idle ZW.Z.W` is in the signd unit; signd now announces
the pattern in its messages (`idle {pattern, hz}`) and chris-integrator's `lights` floor follows it, so the idle
stream is no longer marked `slow`. Bench check: docs/lanes/L53-owner-decisions.md.

## P3: wake-up latency hold (`--cpu-latency-us 0`), krakow-2024, measurement only

| | with (installed) | without |
|---|---|---|
| rail W (last 60 s) | 7.00 (7.05) | 6.91 (6.78) |
| big-core C last 60 s / hottest max | 75.8 / 87.7 | 71.1 / 87.3 |
| road runs/s (modeld) | 15.2 | 15.1 |
| model ms p50 / p90 | 43.5 / 50.4 | 43.7 / 54.0 |
| `road` latency median / p90 (`wsprobe.py 45`) | 119.0 / 134.7 ms | 114.0 / 132.2 ms |

With the new road model the hold buys nothing measurable in rate or latency; without it 0.1-0.3 W less and the big
cores 4.7 K cooler in the last minute (one run each; the start temperatures differed: 73.9 / 82.1 C). For the owner.

**Owner 2026-11-21: hold off (lane L53).** The installed modeld unit no longer passes `--cpu-latency-us 0`; the
option stays in modeld. Bench check of the deploy: docs/lanes/L53-owner-decisions.md.

## Car backend's CAN reader (car repo `docs/lanes/L48-B-order.md`, bench part)

The panda was on the bus (`idProduct` ddcc, `Panda connected`, fw DEV-f120999e, 1 frame/s, `panda_can_rx_errs` 0).
Default of the branch: `BACKEND_PANDA_POLL_MS` unset = 5 ms (0 = the old loop). The order's ssh block (its `join`
is missing on the phone's busybox; the two snapshots were joined on the laptop).

| | before (l44-chris, 11:19, stack up, no frames) | after (12:05, same state) | after, replay stack at full rate (12:24) | pass / fail line |
|---|---|---|---|---|
| `PandaCanReader` CPU | 40.1 % | 2.4 % | 6.3 % | <= 4 % / > 8 % |
| its voluntary switches | 5215 /s | 406 /s | 413 /s | 150-260 / > 1000 |
| whole backend | 51.2 % | 14.5 % | 30.2 % | <= 25 % / > 35 % |
| frames | 1 /s | 1 /s | 1 /s | 1 /s kept |

**Pass with the stack's frames off; neither pass nor fail under the replay stack** (6.3 % and 30.2 % sit between the
lines). The switches stay near 410 /s in both states: two per 5 ms period (the USB transfer's completion and the
pacing wait), so the order's 150-260 line assumed one. The in-car part (frames/s, batches, backlog) needs a drive.

# envd (`orders/L48-envd-phone.md`)

| row | envd little | env_ms median | inferences /s | road runs/s | W | big / little C last 60 s | governor |
|---|---|---|---|---|---|---|---|
| A installed (1 Hz) | 184 | 866 | 0.93-1.00 | 15.0 | 6.89 | 71.7 / 67.2 | 0-2 |
| B `--period 2 --republish 1` | 95 (52 % of A) | 899 | 0.50 | 15.0 | 8.33 | 72.9 / 67.8 | 0-1 |
| C B + fp16 model | 115 | 19 500 | 0.05 | 15.5 | 8.28 | 73.1 / 68.1 | 0-2 |
| D | not run: C is not faster than A (19.5 s a frame: ORT on the A55 has no fp16 kernels) | | | | | | |

`trigger_test`: all checks passed. The rail power of B and C (8.3 W against 6.9 W) is not explained by the units'
CPU (lower) or the GPU (same); the playlist plays other rides in each 10-minute window.

Labels against a parallel installed envd: **not measurable on the phone without breaking the comparison.**
First as ordered (the second envd on the little cores): both were starved (B 0.28 inferences/s at 3.5 s each, the
reference 0.8/s at 1.2 s); over 582 s place disagreed 8.0 % (line 3.5 %), weather 1.1 % (2.3 %), light 0.2 %
(4.5 %), no tunnel in the window. Second with the reference on the big cores: the phone reached 94.6 C, the
governor went to level 7 and paused the unit's envd (36 inferences in 224 s): void.

**B: CPU line passes (52 % of A <= 55 %), road rate and temperatures equal within noise; the label line could not
be measured validly (the one complete measurement fails place, under starvation).** C fails (slower). Installed
configuration kept.

**B adopted by the owner 2026-11-21 (lane L53):** `--period 2 --republish 1` is now the installed configuration
(unit), with the laptop study's caveat accepted (env area prior 0.72 % against a noise of 0.17-0.37 %, weather
slightly above the noise; docs/lanes/L48-envd.md). Bench check of the deploy: docs/lanes/L53-owner-decisions.md.

## State at the end

All drop-ins removed (`/run/systemd/system/*.d` empty, removed), replay off (`roadsense-camerad-replay` disabled,
`roadsense-camerad-dvr` enabled), calib.json back, learned state restored from the copies taken at 12:06 and checked
by SHA-256, the two recorder sessions of the bench (619 MB) moved to `<phone-home>/rollback-20261120-l49/rec-bench-l49/`,
backend mode `auto`. Peak temperature in a measured row 90.3 C (P0 rural, at the start); 94.6 C during the second
label run (no abort; the 95 C guard did not trip).
