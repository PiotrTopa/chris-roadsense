# Order L48-phone: the phone-side proofs of lane L48

Placed by lane L48 (whole-system optimisation), 2026-11-20. Execution lane: Opus. Phone: the car computer
(Poco F1) on the bench, replay playlist, reached as in the CMDB entry of the phone; root through the vault
(`ssh HOST 'sudo -S -p "" CMD'`, password on stdin only, never
printed). Background: docs/lanes/L48-system-optimisation.md. Related orders of the sub-lanes, to be run in the
same session where it says so: `orders/L48-envd-phone.md` (envd), car repo `docs/lanes/L48-B-order.md` (backend).

Rules: nothing is installed by this order. Candidates run from `~/l48` through runtime drop-ins
(`/run/systemd/system/<unit>.d/99-l48.conf`, gone at the next boot) or by hand; the installed binaries, units
and configs stay as they are, and every step ends with the drop-in removed, `systemctl daemon-reload` and the
installed unit restarted. Never reboot, never touch USB / VBUS. Stop everything of this order the moment any
thermal zone passes 95 C. One configuration at a time; before each run the hottest zone must be back under
70 C. Fan as found (note its duty); the same duty for all runs of one table. Times in CET.

Measuring tool for every row (root): `python3 ~/l48/measure.py NAME 240 --units
roadsense-camerad-replay,roadsense-modeld,roadsense-signd,roadsense-envd,roadsense-api,chris-integrator,prius-backend
--out ~/l48/out` after 60 s of settling (`tools/budget/measure.py`: CPU per unit and thread by core class, GPU
time per unit, rail power, zones), plus the daemons' stats lines of the same window (`journalctl -u UNIT -o cat
--since ...`) and `GET http://127.0.0.1:8090/v2/health` once per run (the `rate` markers of `road`, `lights`,
`signs`). Without root, `tools/budget/snap_ro.sh` gives the CPU columns only.

Rides: the replay playlist switched to one ride per run with `sudo sh scripts/use-replay.sh --enable
<phone-home>/rides/<slug>/<slug>.mjpeg` (back to the playlist at the end: `--playlist <phone-home>/rides`):
`rural-zloczew` (no lights for minutes), `krakow-lights` (lights nearly always), `a1-night` (motorway, night).

## P0. Baseline (the installed stack, nothing changed)

One run per ride. Record: the budget table (per unit and thread: little, big, GPU; rail W; big-core, little-core
and GPU temperatures of the last 60 s; hottest zone maximum), road runs a second and model ms (modeld stats),
`signd: lights ... fps, lights_ms` and `live`, envd `env_ms` and fps, the governor level over the run
(`/run/roadsense/governor`). This is the reference of every table below and of the two sub-lane orders.

## P1. Debug face: lazy `road`, little cores (roadsense `v2-l48`, f777ee6)

```
# laptop: copy api/roadsense_api of v2-l48 to the phone as ~/l48/api/roadsense_api
# phone, root:
mkdir -p /run/systemd/system/roadsense-api.service.d
printf '[Service]\nEnvironment=PYTHONPATH=<phone-home>/l48/api\nCPUAffinity=0-3\n' > /run/systemd/system/roadsense-api.service.d/99-l48.conf
systemctl daemon-reload && systemctl restart roadsense-api
```
Run `rural-zloczew`, 240 s. Checks while it runs: `curl -s 127.0.0.1:8091/v1/health` answers with `fps.road`
within 0.5 of P0 and a `model_ms`; `curl -s 127.0.0.1:8091/v1/road` is the newest `road` message (its `seq` moves between two calls);
`/run/roadsense/governor` is rewritten every second; the integrator's `/v2/health` shows the governor.

| | P0 | P1 |
|---|---|---|
| `roadsense-api`: little / big, % of one core | | |

Pass: big-core time of the unit 0; its total under 6 % of a little core; every check above true. Fail: any check
false, or the total above P0's big-core share times 3 (the same work moved, nothing removed).

## P2. Light detector's idle pattern (roadsense-signs `l48-opt`)

Build signd of `l48-opt` on the phone into `~/l48/build-signs` (as `scripts/install-phone.sh` builds it:
`-DROADSENSE_NCNN=ON -DNCNN_DIR=<phone-home>/vulkan-work/ncnn`, roadsense headers of `v2-l48`), do not install.
Drop-in: the installed `ExecStart` line with the binary replaced by `<phone-home>/l48/build-signs/signd` and the
option appended (`ExecStart=` first, to clear the installed one).

| run | signd | rides |
|---|---|---|
| P2-0 | lane build, no new option (must equal P0: same `lights` fps and `sg-lights` CPU within 3 points) | rural-zloczew |
| P2-A | `--lights-idle ZW.Z.W` | all three |

Record per run: `sg-lights` big-core %, rail W, big-core temperature (last 60 s) and hottest zone maximum, road
runs a second, the `signd: lights idle: N frame periods without a pass, M passes` lines (share of periods
skipped), the `lights` marker of `/v2/health` (`hz`, `degraded`, `why`), governor level.

Quality on the phone's own backend (ncnn CPU, fp16), stack stopped (`roadsense_mode off` at the car backend, back
to `auto` afterwards): `lights_replay` of the lane build on the first 5 minutes of `krakow-lights` and
`krakow-night-tunnel`, every 4th frame, production options, once without and once with `--idle ZW.Z.W`
(the frames and the command are those of L21's `~/l21/lightseval.sh`; add `--idle P`); copy the outputs to the
laptop and score them with roadsense-signs `tools/lights/replay_score.py` against L15's reference files
(`<datasets>/l24s/lights/ref_*`), exactly as section 3.2 of the lane report.

Pass (all of): P2-0 equals P0; on `rural-zloczew` and `a1-night` `sg-lights` falls by at least the share of
periods skipped times 0.8; the big-core temperature does not rise on any ride; `lights` is never `degraded`
with `why` = `slow` because of the pattern (the API's floor is 3 messages a second: a pattern that drops below it
while idle fails here); road runs a second not lower than P0; light-time on the phone's backend within
1.5 points of its own baseline on both rides and the first-report delay not more than 0.1 s longer.
Fail: anything else; report the numbers, do not tune.

## P3. For the owner's decision, measurement only: the wake-up latency hold with the new road model

L21 measured (supercombo, one run each) that modeld's `--cpu-latency-us 0` costs 0.34 W at idle and that the
stack without it ran 8 K cooler on the big-core sensors (62 against 70 C) for 0.5 road runs a second and 30 ms of
road latency. The road model has changed since (L27). Two runs on `krakow-2024`, 240 s each, drop-in on
`roadsense-modeld` with the installed command line and `--cpu-latency-us 0` removed for the second: rail W,
big-core temperature, hottest zone maximum, road runs a second, `road` latency median / p90 from
`tools/budget/wsprobe.py 45`. No pass / fail: the table goes to the owner (freshness against degrees).

## Report

One table per step in `docs/lanes/L48-phone-results.md` (new file), the raw `~/l48/out` files kept on the phone,
pass / fail per step as defined above, anything unexpected (restarts by the car backend's watchdog, governor
levels, thermal aborts) with its time. Update the CMDB entry of the phone only if something on it changed
(nothing should).
