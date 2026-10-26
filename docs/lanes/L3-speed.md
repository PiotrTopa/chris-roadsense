# L3 speed-tune: envd on the phone + the live stack tuned for speed

Lane opened 2026-10-26 ~19:10 CET (Opus, execution). User ruling 2026-10-26 ~18:40: thermal headroom
is ample with the fan (plateau ~80 C); optimise for speed and responsiveness; lane safety stop at
95 C sustained (any zone). Times CET (Warsaw local, UTC+1). Root on the phone via
`dotinfra vault exec poco_user_password -- ssh user@192.168.0.86 'sudo -S -p "" sh SCRIPT'`.
Supersedes orders/L1-soak.md and folds orders/L2-envd-phone.md.

## 0. Starting point (19:14 CET)
Found running: camerad-dvr + modeld + api + **signd** (unit config, `--max-hz 5`); envd not installed;
rs-stress-* inactive; roadsense.target disabled; boost healthy (0x1109 = 0a, 0x1110 = 00).
GPU devfreq `simple_ondemand` 257-710 MHz (7 steps); big cores scaling_max 2092800 (cap from modeld's
ExecStartPre; cpuinfo_max 2649600), schedutil. Installed api predates envd (no `env` in /v1/health).
`/v1/health` at 19:14: camera 17.6 fps, road 9.88/s (model_ms 108), signs 4.92/s (detect_ms 105), 52 C.
The phone has no curl (busybox ash); python3 urllib is used for the API.

## 1. envd on the phone (orders/L2-envd-phone.md), 19:15-19:29 CET

Install: `scripts/sync-phone.sh`; scp of `models/envd-image.onnx`, `models/envd-classes.json`, the probe class
file (`~/datasets/envd-cache/envd-classes-probe.json` -> phone models dir, not committed), the ncnn
conversion (`~/datasets/envd-cache/ncnn/vitb32_openai.*` -> `envd-image-ncnn.*`) and the TinyCLIP-40M pair
(row F). Build as user: `cmake -S ~/roadsense/envd -B ~/l3/build-envd -DCMAKE_BUILD_TYPE=Release
-DROADSENSE_NCNN=ON -DNCNN_DIR=/home/user/vulkan-work/ncnn && make -j4` (clean). Root: envd + envd_replay to
/usr/local/bin, models to /var/lib/roadsense/models, `roadsense-envd.service` + `roadsense.target` to
/etc/systemd/system, the api tree replaced by the current one (`/v1/env`; the old copy kept as
/usr/local/lib/roadsense/api.old), `/etc/roadsense/envd.json` = config/envd.json plus
`"classes": "/var/lib/roadsense/models/envd-classes-probe.json"`.

Cost rows (signd stopped, camerad-dvr + modeld + api running; `~/l3/row.sh NAME FRAMES ARGS` =
`ROADSENSE_RUNDIR=/tmp/l2 envd --frames N --stats 10 --verbose ARGS` + `top -b -n 3 -d 5` + the modeld
journal lines of the window; bench scene = the dark room, labels underground_parking / clear / night).
envd CPU = busybox top %CPU of all 8 cores (25 % = 2 cores).

| row | ARGS | env_ms median / p90 | encode median | envd CPU | modeld runs/s (model_ms) in the window |
|---|---|---|---|---|---|
| A (unit default) | `--backend ort --threads 2 --cpus 0-3 --max-hz 1` | 812 / 816 (rerun A2: 810 / 814) | 795 | 19 % | 14.8 (63.5) / A2 14.3 (65) |
| B | `--threads 4 --cpus 0-3` | 569 / 572 | 536 | 21 % | 13.3 (69) |
| C | `--threads 2 --cpus 4-7` | 408 / 410 | 396 | 10 % | 12.0 (75) |
| G (extra) | `--threads 4 --cpus 4-7` | 260 / 267 | 248 | 10 % | 12.6 (73) |
| D (GPU) | `--backend ncnn --cpus 0-3` | **failed: GPU hang** | - | - | modeld dead (see below) |
| E (ceiling) | A with `--max-hz 100`, 30 frames (E2) | 810 / 812 at 1.23 Hz | 794 | 24 % | 14.3 (66) |
| F (TinyCLIP-40M) | A with the TinyCLIP model + zero-shot class file (F2) | 434 / 439 | 397 | 10 % | 13.3 (69) |

Reference without envd: modeld 11.5 runs/s, 78 ms (19:25, after the restart). The little-core
rows *raise* modeld's rate (11.5 -> 14.3-14.8) for the reason L1 found: CPU load keeps devfreq at 710 MHz.

Nulls / incidents:
- `--max-hz 0` is not "uncapped" in envd (period falls back to 1 s: `envd/main.cc:275`); the first E
  row ran at 1.00 Hz. E2 uses `--max-hz 100`: back-to-back frames give 1.23 Hz on A's cores.
- **Row D hung the GPU.** 19:23:04 CET: kernel `hangcheck detected gpu lockup ... offending task: envd
  (--backend ncnn)`, recovery, then a second hang blamed on modeld; envd exited (`vkWaitForFences
  failed -4`, `extract out0 failed`) and **modeld stayed `active` but failed every run**
  (`vkQueueSubmit failed -4`, VK_ERROR_DEVICE_LOST) until restarted by hand at 19:25:28 (rows E/F of the
  first pass ran with modeld dead and were redone as E2/F2). The ncnn ViT-B/32 graph is not usable on
  turnip here; ORT stays. Open item: modeld should exit on a lost device so systemd restarts it.

## 2. Speed tuning of the live stack (camerad-dvr + modeld + signd + envd + api)

Method: each candidate is applied as runtime drop-ins (`/run/systemd/system/roadsense-*.service.d/l3.conf`,
script `~/l3/cfg.sh` on the phone: ExecStart of modeld / signd / envd, and in modeld's ExecStartPre /
ExecStopPost the big-core `scaling_max_freq` (BIGMAX), the GPU devfreq `min_freq` (GPUMIN) and the
little-core `scaling_min_freq` (LITMIN)), the daemons restarted, then `~/l3/live.sh NAME 200`: 200 s,
a sample every 30 s (hottest zone, cpu7, gpu-top, cluster0/1, GPU MHz, cpu4 / cpu0 MHz; abort at > 95 C
twice in a row), daemon stats = mean of the journal lines of the last 120 s, ws latency over the last 60 s
(`~/l3/wslat.py`: subscribes to `/ws?topics=road,signs,env` and takes now - `ts_real_ns` of the frame,
which camerad-dvr stamps when it sees the new frame header on USB, before the 21 ms read and 17 ms decode;
"newest age" = age of the newest road message at uniformly random instants). Runs go through
`systemd-run --unit l3-NAME` on the phone because the Wi-Fi dropped the ssh session twice (a run
started from ssh died with it). Bench: dark room, fan on.

| run (CET) | change vs previous | road runs/s (model_ms) | signs/s (detect_ms) | env Hz (env_ms) | road latency med / p90 ms | newest road age med | hottest last 2 min (C) |
|---|---|---|---|---|---|---|---|
| S0 19:40 | deployed: modeld `--throttle-temp 80000`, big max 2092800, GPU auto, signd `--max-hz 5`, **no envd** | 9.59 (95.4) | 4.98 (107) | - | 185 / 220 | 237 | 49-52 |
| S1 19:44 | + envd row A (unit default, cpus 0-3) | **11.18** (84.8) | 4.99 (95) | 1.0 (810) | 155 / 195 | 201 | 57-58 |
| S3 19:48 | + modeld `--throttle-temp 95000`, GPU min 710 MHz | 11.12 (85.1) | 4.99 (95) | 1.0 (809) | 157 / 199 | 204 | 54-57 |
| S4 19:52 | + big cores uncapped (2649600) | 11.19 (84.6) | 4.98 (93) | 1.0 (810) | 152 / 195 | 199 | 54-59 |
| S5 19:55 | + signd uncapped (no `--max-hz`) | 8.98 (105.9) | **9.07** (110) | 1.0 (809) | 181 / 201 | 237 | 58-61 |
| S6 20:00 | S4 (signd capped again) + little cores pinned at max (`scaling_min_freq` 1766400) | 11.39 (83.5) | 4.99 (93) | 1.0 (809) | 150 / 188 | 192 | 56-60 |
| S9 20:07 | S6 without envd | 11.45 (83.1) | 4.99 (93) | - | 150 / 190 | 194 | 53-57 |
| S7 20:04 | S6 with modeld `--cpus 4-7 --prep-cpus 0-3` (model thread on the big cores, prep on the little) | **12.46** (77.6, prep 11.5) | 4.98 (96) | 1.0 (808) | 149 / 193 | 192 | 57-59 |

Readings so far:
1. Adding envd made modeld *faster* (9.6 -> 11.2 road/s, model_ms 95 -> 85). The GPU was already at
   710 MHz in every 30 s sample of S0, so it is not the GPU clock: the per-sample cpu0 frequency was
   1132-1612 MHz in S0 and 1766 MHz (the little cluster's max) in every sample of S1-S4. modeld's ncnn
   CPU layers and Vulkan submission run on the little cores (`--cpus 0-3`), and schedutil clocks them
   down between runs unless something else keeps them busy. Lever: pin the little cores' `scaling_min_freq`.
2. GPU min 710 MHz: no effect here (devfreq already sat at 710 with this load). Kept as a cheap guard.
3. Big-core cap removed: no measurable change in road (11.19 vs 11.12) or signs (4.98); detect_ms 95 -> 93.
4. signd uncapped: 9.1 signs/s but road drops to 9.0/s (the detector's GPU dispatches stall modeld runs),
   as L1 found. Road is the primary objective, so signd stays capped.
5. Little cores pinned (S6): 11.4 road/s, the same as envd gave by keeping them busy; without envd
   (S9) the pinned stack gives 11.45, so with the pin envd is free for the road rate (11.39 vs 11.45,
   noise) and for signs (4.99 both).
6. Swapping modeld's placement (S7: model thread + ncnn CPU layers on the A75s, prep on the A55s)
   gives 12.5 road/s (model_ms 83.5 -> 77.6); prep goes 6.9 -> 11.5 ms on the A55s but it runs in its
   own thread ahead of the model, so latency is unchanged (149 ms median).

**Event: unexplained phone reset, 20:14:12 CET.** During S10 (S7 + modeld `--threads 2`, i.e. two ncnn
CPU threads on the big cores; signd 5 Hz, envd A; hottest zone 55-65 C, die at 56 C by modeld's own
reading) the journal of the previous boot ends mid-run at 20:14:12 with normal stats lines and no
shutdown, oops or thermal message; pstore is empty. The phone was back on Wi-Fi with uptime 2 min at
20:26 (the next boot's early journal is stamped 20:14:31 from the saved clock before NTP), so it was
gone for ~10 min. Not a boost latch: 0x1109 = 0a, 0x1110 = 00 after the reset, camera 1902:3241
enumerated. Not thermal (< 66 C). Not initiated by this lane (no reboot command was sent). The
user's own boot list shows an earlier reset at 16:38 (before this lane). The runtime drop-ins and the
frequency pins were lost with /run; the stack came up stopped (target disabled). S10's last 3 min
of journal (20:11-20:14) read: modeld 12.3-12.5 runs/s, model_ms 77.3-78.2 (the same as S7 with
1 thread), signd 4.97-5.00, envd 1.00 Hz / 807 ms: `--threads 2` brings nothing.

Runs after the reset (stack restarted ~20:26 by `systemctl start roadsense.target`; same method):

| run (CET) | change vs S7 (modeld `--cpus 4-7 --prep-cpus 0-3`, GPU + little pins, big uncapped, signd 5 Hz, envd A) | road runs/s (model_ms, prep) | signs/s (detect_ms) | env Hz (env_ms) | road latency med / p90 | newest road age med | hottest last 2 min |
|---|---|---|---|---|---|---|---|
| S11 20:27 | signd `--cpus 0-3` | 12.47 (77.5, 11.6) | 4.98 (**109**) | 1.0 (811) | 148 / 192 | 190 | 52-58 |
| S12 20:31 | modeld `--prep-cpus 4-7` | 12.51 (77.3, 6.9) | 4.99 (95) | 1.0 (894) | 141 / 185 | 184 | 54-57 |
| S13 20:34 | signd off (road ceiling with envd) | 16.33 (58.8, 11.4) | - | 1.0 (807) | 131 / 149 | 161 | 52-55 |
| S7b 20:38 | S7 repeated | 12.48 (77.5, 11.6) | 4.99 (97) | 1.0 (808) | 146 / 189 | 188 | 54-57 |
| S14 20:42 | envd row B (4 threads, cpus 0-3) | 12.33 (78.1) | 4.99 (95) | 1.0 (532) | 147 / 191 | 191 | 54-58 |
| S15 20:46 | envd row C (2 threads, **cpus 4-7**) | 12.60 (77.1) | 4.99 (90) | 1.0 (**344**) | 147 / 192 | 191 | 56-60 |
| S16 20:50 | S15 at `--max-hz 2` | 12.97 (75.2) | 4.99 (88) | **2.0** (336) | 143 / 187 | 185 | **73-88** |
| S17 20:53 | envd 4 threads cpus 4-7 | 12.45 (77.9) | 4.98 (92) | 1.0 (244) | 149 / 193 | 192 | 62-63 |
| S18 20:57 | S15 + big cores pinned at max (`scaling_min_freq` 2649600) | **13.12** (74.5, 11.3) | 4.99 (**81**) | 1.0 (323) | **143** / 187 | **184** | 58-62 |
| S19 21:01 | S18 + modeld `--prep-cpus 4-7` | 13.3 (73.5, 3.9) (journal, 21:02-21:03) | 4.98 (79-84) | 1.0 (321) | - | - | 60-63 |

Readings:
7. signd on the little cores costs 13 ms of detect latency and gives nothing back (S11).
8. Without signd the road rate is 16.3/s (S13): signd's 5 detector runs/s cost modeld ~3.8 runs/s
   on the shared GPU. That is the price of the signs stream; the 5 Hz cap stays (road first, signs at the
   rate they had).
9. envd on the big cores (S15) is free for road and signs (12.60 vs 12.48, 4.99 signs/s, detect 90 ms) and
   2.4x fresher (344 vs 810 ms per frame; message age at publish 402 vs 862 ms median). 4 threads
   (S17) save another 100 ms but cost a little road (12.45); 2 Hz (S16) raised the die from ~60 to 88 C
   in 3 minutes for an output that a 10 s smoothing window averages anyway: not free, rejected.
10. Pinning the big cores at max (S18) takes model_ms 77 -> 74.5 and detect_ms 90 -> 81: 13.1 road/s.
11. Prep on the big cores too (S19) looked equal or slightly better (13.3/s, prep 3.9 ms), but the run
    ended in the second reset below, so it is not adopted on that evidence.

**Event: second unexplained reset, ~21:03:22 CET**, during S19 (all clocks pinned at max, every daemon's
heavy thread on the big cores; hottest zone 60-66 C). The journal ends mid-run with normal stats lines;
the next boot logged a kernel oops while loading the `bu64748` camera-actuator module, a 2 s boot
followed, and the phone was back on Wi-Fi at ~21:12 (uptime 1 min at 21:13:48). Boost healthy afterwards
(0x1109 = 0a, 0x1110 = 00), camera enumerated. Not thermal, not a boost latch, not a reboot command.
Power: the phone runs from the battery eliminator (fuel gauge pinned at 80 %, ~4.0 V, "Discharging"),
and the camera + hub hang off the phone's VBUS boost. Two resets in ~2 h of high-load runs, both in the
highest-power configurations (S10: two ncnn CPU threads; S19: everything on the big cores at max clock),
plus the user's own reset at 16:38 during the stress test, point at the supply (brownout under peak
current), but there is no direct evidence (no pstore, no PMIC log). Current draw from the fuel gauge
(`~/l3/powerlog.py`, 2 Hz, fsync per line):

| config | battery-side current median / max (mA) | voltage min / median (mV) |
|---|---|---|
| old units + envd A (P0, 21:16, 200 s) | 1269 / 1654 | 3917 / 3954 |
| chosen config (FINAL, 21:21, 900 s) | 1419 / 2783 | 3811 / 3939 |

The speed profile draws ~150 mA more on the median and its peaks reach 2.8 A (sampled at the gauge's
~1 Hz update, so true peaks are higher). No reset during the 15-minute final run.

## 3. Chosen configuration, deployed (installed 21:20 CET, 15-minute run 21:21:30-21:36:30)

- modeld unit: `--backend ncnn --cpus 4-7 --prep-cpus 0-3 --max-hz 20 --throttle-temp 95000 --stats 10`;
  ExecStartPre pins GPU devfreq `min_freq` = `max_freq` (710 MHz), cpu0-3 `scaling_min_freq` = max
  (1766400), cpu4-7 `scaling_max_freq` and `scaling_min_freq` = max (2649600; the 2092800 cap is gone);
  ExecStopPost restores the minima. Binary rebuilt with the lost-device exit (below).
- signd unit (roadsense-signs): unchanged arguments (`--det-backend ncnn --cls-backend ort --threads 2
  --cpus 4-7 --max-hz 5`), comment refreshed.
- envd unit: `--backend ort --threads 2 --cpus 4-7 --max-hz 1 --stats 60`; config/envd.json `cpus` 4-7;
  on the phone `/etc/roadsense/envd.json` additionally points `classes` at the probe file.
- `roadsense.target` stays disabled for boot.

15-minute run (`~/l3/live.sh FINAL 900` + `~/l3/powerlog.py`, units as installed, no drop-ins):
modeld 13.0-13.3 runs/s over all 90 stats windows (mean 13.23, model_ms 74, never throttled, 0 run
failures); signd 4.97-5.00 (mean 4.99, detect_ms 81); envd 1.00 Hz at 322-324 ms every minute; camera
19.1 fps (`/v1/health`). Hottest zone 75-82 C in the first 4 minutes (cpu7 up to 82), then a 60-65 C
plateau for the last 10 minutes (cpu7 54-61, gpu 60-65); GPU 710 MHz, cpu4 2649 MHz, cpu0 1766 MHz in
every sample. Latency (last 60 s): road 144 / 189 / 206 ms median / p90 / max from the frame stamp to a
ws client, newest road message 183 ms old at a random instant (p90 227); signs 136 / 164 ms; env 381 ms.

Per-stage road latency (`~/l3/socklat.py 30`, 21:38, the same frames read from `modeld.sock` and through
the api at once): frame stamp -> modeld socket 141 ms median (p90 184); api hop +4 ms (145 / 188).
Inside the 141 ms: camerad-dvr read 21 ms + JPEG decode/NV12 17 ms (2026-10-23 deploy numbers; the
stamp is taken when the frame header is seen, before both), prep 11.4 ms, model 56 ms median but bimodal
(p90 106 ms: runs that overlap a signd detector dispatch on the GPU take twice as long; mean 74), the rest
(~20 ms) is waiting for the model slot and serialising the 24 KB road JSON. Glass-to-API would add the
camera's own exposure-to-USB delay on top (LED test on the laptop: 30 / 56 / 84 ms min / median / max
glass-to-host, which overlaps the 38 ms read+decode counted above); tools/ledlat was not run (needs the
RP2040 on the phone's USB).

envd live labels (bench, dark room, fan on): weather clear (0.89, stable 1083 s), light night (0.60),
place alternates underground_parking <-> gas_station (3 changes in 17 min, place confidence ~0.55-0.61):
the probe's known underground-parking / gas-station confusion on an ambiguous indoor scene; weather and
light did not change.

### Speed objective, starting point vs chosen

| metric | start (S0: deployed units, no envd) | start + envd as shipped (S1) | chosen (FINAL, 15 min) |
|---|---|---|---|
| road runs/s (primary) | 9.59 | 11.18 | **13.23** (13.0-13.3) |
| model_ms mean | 95.4 | 84.8 | 74.0 |
| signs/s (detect_ms) | 4.98 (107) | 4.99 (95) | 4.99 (81) |
| env Hz / env_ms / message age at publish | - | 1.0 / 810 / 876 | 1.0 / 323 / 381 |
| road latency stamp -> ws, median / p90 | 185 / 220 | 155 / 195 | 144 / 189 |
| newest road message age, median | 237 | 201 | 183 |
| hottest zone, last 2 min / max (C) | 49-52 / 52 | 57-58 / 58 | 61-64 / 82 |
| battery-side current median / max (mA) | - | 1269 / 1654 (P0) | 1419 / 2783 |

## 4. Code change: modeld exits on a lost GPU device
`modeld/main.cc`: after 20 consecutive `model->run` failures modeld logs and `_Exit(1)`s so systemd's
`Restart=always` brings it back with a fresh Vulkan device (row D left it `active` and failing every run
for 2 minutes until a manual restart). Built on the phone (`make -C ~/l3/build-rs modeld`) and installed;
the previous binary is kept as /usr/local/bin/modeld.pre-l3. Not exercised by a real hang in this lane.

## 5. Phone state at hand-back (21:39 CET)
Running: camerad-dvr, modeld, envd, api, signd from the installed units (no drop-ins left in /run);
`roadsense.target` disabled for boot; boost 0x1109 = 0a; clocks pinned by modeld's unit while it runs.
Installed: envd + envd_replay, models incl. `envd-classes-probe.json` (research-licensed, not in git),
`envd-image-ncnn.*` and the TinyCLIP pair (unused), current api tree (old copy /usr/local/lib/roadsense/api.old),
modeld with the lost-device exit (old binary modeld.pre-l3). Scratch: `~/l3/` (scripts, logs, rows/, live/, out/).

## 6. Open items
- **Resets.** Two unexplained hard resets in ~2.5 h of high-load runs (20:14, 21:03), both in the most
  power-hungry configs, none in the 15-minute final run. The chosen profile draws more (median +150 mA,
  peaks >= 2.8 A). If the supply (battery eliminator + wiring, VBUS boost for the camera) is marginal,
  the speed profile makes resets likelier; worth a check of the eliminator's current rating / wire
  gauge, or a longer soak. Fallback without the big-core floor: drop the cpu4-7 `scaling_min_freq`
  line (S15: 12.6 road/s, detect 90 ms).
- The first boot after the second reset logged a kernel oops in `bu64748` (camera actuator module) and
  a 2 s boot; the phone recovered by itself.
- envd ncnn (row D) hangs the Adreno; ORT only. `envd --max-hz 0` means 1 Hz, not uncapped.
- modeld's model_ms is bimodal (56 ms alone, ~105 ms when a signd detector run overlaps); the signs stream
  costs ~3.8 road runs/s (16.3/s without signd). Serialising or interleaving the two GPU clients
  (e.g. signd triggered right after a modeld run) might recover part of it: a code lane, not tuning.
- Place label flips between underground_parking and gas_station on the dark bench scene.
- camerad-dvr per-stage numbers were not re-measured (stats need `--verbose`, i.e. a camera restart).
- Glass-to-API with tools/ledlat (needs the RP2040 on the phone's USB) not done.
