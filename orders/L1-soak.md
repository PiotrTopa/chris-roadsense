# Order L1-soak: thermal plateau of the sign stack on the GPU next to modeld

Placed by lane L1 (signs-vulkan), 2026-10-24. Execution lane: Opus. Phone: Poco F1 (`user@phone`)
(user ssh; root via `ssh user@phone 'sudo -S -p "" CMD'`,
password on stdin only, never printed). Background: docs/lanes/L1-signs-vulkan.md (short runs only;
this order measures what the short runs cannot: the plateau).

## What to run
The deployed stack as it is (camerad-dvr + modeld on the GPU + api; `systemctl start roadsense.target`
if not running; never reboot, never touch USB/VBUS). Then, one configuration at a time, 15 minutes
each with 5 minutes of nothing-but-the-stack between them (the die must be back under 65 C before
the next one starts), the signd of the roadsense-signs L1 build. If the roadsense-signd unit has
been updated to the L1 recommendation (check `systemctl cat roadsense-signd`), run it through the
unit (`systemctl start roadsense-signd`) for config A and by hand for the others; otherwise run all
by hand as root with `ROADSENSE_RUNDIR=/run/roadsense ROADSENSE_CONFIG_DIR=/etc/roadsense
/usr/local/bin/signd --stats 10 ARGS` (binary and models installed by `scripts/install-phone.sh`
with `NCNN_DIR=<phone-home>/vulkan-work/ncnn`; fallback: `<phone-home>/l1/build-signs/signd
--models-dir <phone-home>/roadsense-signs/models`).

| config | signd arguments | expectation from the 70 s runs |
|---|---|---|
| A (L1 recommendation) | `--det-backend ncnn --cls-backend ort --threads 2 --cpus 4-7` | 8.7 signs/s, modeld 8.7/s, die ~75 C |
| B (duty cycle, the shipped unit) | A + `--max-hz 5` | ~5 signs/s, modeld ~9.1/s, ~68 C (4 Hz: 4.0 / 9.6; 6 Hz: 6.0 / 8.7 measured) |
| C (both on GPU, little cores) | `--backend ncnn --threads 1 --cpus 0-3` | 8.1 signs/s, modeld 8.1/s |
| D (reference, the old unit) | `--backend ort --threads 4 --cpus 4-7` | 5.6 signs/s, modeld 9.7/s, 94 C after 70 s: **stop at 95 C** |

Run D last and only if the user's fan is on the phone; stop it the moment any thermal zone passes
95 C (`cat /sys/class/thermal/thermal_zone*/temp | sort -n | tail -1`).

## What to record (every 30 s, one CSV line per config, plus the signd/modeld stats lines)
- `date -u`, hottest zone, cpu7 (zone10), gpu-top (zone11), cluster0/1 (zones 5/6), GPU freq
  (`/sys/class/devfreq/5000000.gpu/cur_freq`), big-core freq (`cpu4/cpufreq/scaling_cur_freq`).
- signd: fps and detect_ms from its `--stats 10` stderr lines; modeld: runs/s and model_ms from
  `journalctl -u roadsense-modeld -o cat`; camerad-dvr fps if it logs one.
- GPU busy over each 60 s: `sudo sh <phone-home>/l1/phone/gpubusy.sh 60 LABEL` (valid only while
  the GPU stays active; ignore loads > 100 %).
- The plateau temperature (mean of the last 5 minutes) and whether modeld's `--throttle-temp 80000`
  guard kicked in (modeld logs `effective_hz` < 20 in its stats line).

## Stop conditions
- Any zone > 95 C: kill signd immediately, note the time and config, continue with the next after cooling.
- modeld rate < 7/s sustained for 2 minutes or camerad-dvr stalls: stop the config, note it.
- The phone must be left with camerad-dvr + modeld + api running and roadsense-signd in the state
  it was found (stopped unless the user has started it).

## Report
`docs/lanes/L1-soak.md`: one table (config, signs fps, modeld runs/s, plateau temps, GPU busy, time
to plateau), the raw CSVs attached or linked, and a one-line answer: which of A/B/C holds the die
under the user's target with modeld >= 9/s, if any.
