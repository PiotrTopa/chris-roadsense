# Order L48-envd-phone: envd every 2 s (and fp16) against the installed envd, on the phone

Placed by lane L48-E, 2026-11-20. Execution lane: Opus. Background: docs/lanes/L48-envd.md. Phone: Poco F1
(user ssh; root via `ssh user@phone 'sudo -S -p "" CMD'`,
password on stdin only). Preconditions: no other lane holds the phone; never reboot, never touch USB/VBUS;
stop this order's units the moment a thermal zone passes 95 C (measure.py aborts by itself).

## 1. Build (user), install side by side
```
scripts/sync-phone.sh                                    # laptop: branch v2-l48 -> ~/roadsense
cmake -S ~/roadsense/envd -B ~/l48e/build-envd -DCMAKE_BUILD_TYPE=Release && make -C ~/l48e/build-envd -j4 envd trigger_test
~/l48e/build-envd/trigger_test                           # must print "all checks passed"
scp <datasets>/envd-cache/l48/models/vitb32_fp16.onnx user@phone:~/l48e/   # laptop (row C only)
```
Do not install over /usr/local/bin/envd; rows run the new binary through a runtime drop-in of
`roadsense-envd.service` (`/run/systemd/system/roadsense-envd.service.d/l48e.conf`, ExecStart= cleared and
set to `<phone-home>/l48e/build-envd/envd ARGS --stats 60`), removed at the end.

## 2. Rows (stack as deployed, replay playlist `sudo sh scripts/use-replay.sh --playlist <phone-home>/rides`)
| row | envd ARGS |
|---|---|
| A (installed) | the installed unit, no drop-in |
| B | `--backend ort --threads 3 --cpus 0-3 --period 2 --republish 1` |
| C | B + `--model <phone-home>/l48e/vitb32_fp16.onnx` |
| D (only if C is faster than A per frame) | A + the fp16 model (1 Hz) |

Per row, 10 min after 2 min settling:
`sudo python3 ~/roadsense/tools/budget/measure.py L48E-<row> 600 --units roadsense-camerad,roadsense-modeld,roadsense-envd,roadsense-api --out <phone-home>/l48e/out`
(envd CPU split little / big, rail power, thermal zones), the envd stats lines (fps, env_ms, republished) from
the journal, road runs/s from modeld's journal.

## 3. Labels against the installed configuration (same frames)
During row B, also run the installed envd by hand in a second rundir on the same ring:
`ROADSENSE_RUNDIR=/tmp/l48e-a /usr/local/bin/envd --stats 60` (config as installed), and record both
streams for the 10 min (`python3 -c` socket reader on `/run/roadsense/envd.sock` and `/tmp/l48e-a/envd.sock`,
one JSON line per message with the arrival time). On the laptop: per group the time share of disagreement
of the smoothed labels and the delay of each change (`tools/envd/rate_sim.py` functions `held` / `compare`),
plus the tunnel label around the portals of tunnels-s3 / tunnel-* rides.

## Pass / fail
- B passes if: envd CPU <= 55 % of row A's (little cores), road runs/s and temperatures not worse than A within
  run-to-run noise, and the label disagreement with the parallel installed envd is within the laptop's noise
  floor (place <= 3.5 %, weather <= 2.3 %, light <= 4.5 %, tunnel entry delay p90 <= 3 s; docs/lanes/L48-envd.md
  section 2). Otherwise B fails and the installed config stays.
- C passes if its env_ms median is below B's and its labels agree with B's on >= 99 % of messages.
- Report: `docs/lanes/L48-envd-phone.md` (table per row, the agreement numbers, pass/fail); remove the drop-in,
  `use-replay.sh --disable`, phone state at hand-back.
