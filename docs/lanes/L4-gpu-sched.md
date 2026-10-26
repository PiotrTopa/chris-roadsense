# L4 gpu-sched: GPU turn-taking between signd and modeld

Lane opened 2026-10-26 22:15 CET (Opus, code + measurement). Goal: win back road-model throughput lost
when signd's detector and modeld collide on the Adreno 630, without lowering the sign rate (speed-first
ruling). Objective in order: road runs/s (signs >= 5/s), modeld latency p90, signs/s.
Baseline (L3, docs/lanes/L3-speed.md): 13.2 road/s, 5.0 signs/s, model 74 ms mean, 16.3 road/s without signd.
Times CET (Warsaw local, UTC+1). Root via `dotinfra vault exec poco_user_password -- ssh user@192.168.0.86 'sudo -S -p "" ...'`.
Scratch on the phone: `~/l4/` (sources `~/l4/roadsense{,-signs}`, builds `~/l4/build-rs`, `~/l4/build-signs`,
scripts, `live/` results).

## 0. Starting point (22:16 CET)
Phone up 24 min (a reset before the lane, the user's hardware testing); full stack active from the L3 units,
`roadsense.target` disabled. modeld journal: 13.3 runs/s, model 73.6 ms avg / 109 max; signd 4.97-4.99 fps,
detect 80-84 ms (det 72-76).

## 1. Where the GPU time goes

### devfreq busy (L1 method, `sudo sh ~/l1/phone/gpubusy.sh N LABEL`)
| state | road runs/s | GPU busy (devfreq, of wall) |
|---|---|---|
| live stack (modeld + signd 5 Hz + envd on CPU), 60 s | 13.3 | 79.1 % |
| signd stopped, 30 s | 17.7 (model 55.0 ms avg, 58 max) | 68.3 % |

Note: modeld alone now runs 17.7/s (L3's S13 measured 16.3 with envd; this one has envd too, the phone
had just rebooted and was cool).

### Per-submit timeline (msm tracepoints)
`drm_msm_gpu:msm_gpu_submit{,_flush,_retired}` give, per GPU submit, the pid and the GPU `elapsed` time
(`~/l4/gputrace.sh SECONDS OUT`, analysis `gpuan.py` in the lane scratch). The retired event's start/end
ticks come from a counter that stops while the GPU is power-collapsed, so in tick space the GPU looks
100 % busy even with modeld alone; per-submit `elapsed` is the GPU execution time, idle time is not
visible in ticks (it is visible in wall time: alone, 17.7 runs/s x 47 ms = 83 % vs 100 % wall).

One modeld run = 6 submits: **36.9 ms** (vision backbone) + 0.3 + 1.4 + 0.1 + **7.6** + 0.6 ms =
**47.0 ms of GPU per run**, identical alone and with signd. One signd detector run = **45.7 ms** + **5.0 ms**
(`out0` box branch with its download, then `out1` score branch) = 50.7 ms. modeld's wall run alone is
55-56 ms, so ~9 ms per run is CPU-side (input copies, ncnn's CPU layers between the small submits, output
parse), plus the JSON publish between runs.

Budget at 5 signs/s: 5 x 50.7 = 254 ms of GPU per second for signs, leaving 746 ms for modeld:
**ceiling (1000 - 254) / 47 = 15.9 road/s** if the GPU never idled. Measured 13.3 x 47 + 254 = 879 ms, i.e.
~12 % of the wall time nobody uses the GPU (both daemons on their CPU side at once). The detector
landing in the middle of a modeld run (after its 37 ms submit, before the policy submits) makes that run
wait ~46 ms: the bimodal 56 / 105 ms model_ms of L3.

modeld's own GPU idle (raw CPU timestamps of submit/retire, modeld alone): each 56.5 ms cycle has one
**5-6.7 ms gap at the run boundary** (last small retire -> next 36.9 ms backbone submit) plus ~0.8 ms
between the small policy submits. The between-runs part of modeld's loop (JSON + publish 0.9 ms, loop
0.7-0.9 ms, from the new stats below) is a small share; the rest is the run's own start and end (input
copies, ncnn upload staging, download, parse). The recurrent inputs (features buffer, previous
curvature) make run N+1 depend on run N, so modeld cannot overlap its own runs.

## 2. The contract: `<rundir>/gpu.sched` (header-only `camerad/gpusched.h`)
A 4096-byte shared file (layout in docs/ARCHITECTURE.md, "GPU schedule"). modeld creates it and ticks
`run_seq` (futex word) at the end of every model run. A sign provider can (a) `after`: wait for the tick
before its detector, or (b) `turn`: set `want`, and modeld then grants `turn` at its next run end and
waits (bounded by `--gpu-yield MS`) until the provider clears it after the detector. Every wait has a
timeout; the provider does not wait at all if the file, the magic, modeld's pid or a run in the last
second is missing; modeld takes a turn back after `--gpu-yield` ms, so a provider that dies holding it
costs one timeout. Both daemons work alone (B0 below = the contract present and unused).

Code: roadsense `camerad/gpusched.h`, `modeld/main.cc` (`--gpu-sched PATH|none`, `--gpu-yield MS`,
`--gpu-lead MS`; stats line gained `p50 p90` of the run time, `post` = JSON + publish, `gap` = run end to
next run start, `gpu turns` and mean `yield`); roadsense-signs `signd/main.cc` (`--gpu-sync none|after|turn`,
`--gpu-sync-timeout MS`; stats gained `gpu wait`, `ticks`, `turns`), `signd/yolo.{h,cc}` (before/after
detector hooks).

Method per config: runtime drop-ins (`~/l4/cfg.sh "MODELD_ARGS" "SIGND_ARGS"`: `ExecStart` of modeld and
signd pointed at the L4 builds, the units' clock pins unchanged), 15 s settle, then L3's `live.sh` (as
`~/l4/live.sh NAME 200`: 200 s, daemon stats = mean of the journal lines of the last 120 s, ws latency over
the last 60 s from `~/l3/wslat.py`, abort at > 95 C twice), then an 8 s GPU trace. Runner:
`systemd-run --unit l4-suiteN sh ~/l4/suiteN.sh`. Base arguments = the installed units
(modeld `--backend ncnn --cpus 4-7 --prep-cpus 0-3 --max-hz 20 --throttle-temp 95000`, signd
`--det-backend ncnn --cls-backend ort --threads 2 --cpus 4-7 --max-hz 5`), envd unchanged at 1 Hz.

**Incident (mine), 22:27-22:40 CET: road stream down for 13 minutes.** The first suite pointed modeld's
drop-in at the build directory instead of the binary (`build-rs/modeld` vs `build-rs/modeld/modeld`);
systemd restarted it 120+ times (status 203/EXEC) while signs and env kept running; the suite's numbers
were void. Fixed at 22:40 (road back at 13.7/s); the suite now checks for a modeld stats line 15 s after
each switch and falls back to the installed units if it is missing.

## 3. First pass (suite 1b, 22:40-22:52 CET, 200 s each)
| run | modeld / signd | road runs/s | model ms mean / p50 / p90 | signs/s | signd det / detect ms (gpu wait) | road latency med / p90 (newest age) | signs latency med |
|---|---|---|---|---|---|---|---|
| B0 | L4 build, no sync | 13.23 | 73.8 / 56.1 / 105.8 | 4.99 | 73.1 / 81.3 (0) | 129 / 172 (169) | 123 |
| A1 | signd `--gpu-sync after` | 13.22 | 73.8 / 56.5 / 103.9 | 4.99 | 92.1 / 129.0 (28.7) | 128 / 172 (169) | 170 |
| T1 | modeld `--gpu-yield 80`, signd `--gpu-sync turn` | **12.62** | 55.5 / 55.4 / **56.9** | 4.99 | 56.3 / 94.6 (30.1) | **112 / 134** (153) | 137 |

Readings:
1. B0 reproduces L3 (13.2 road/s, bimodal 56 / 106 ms runs). The latency figures are lower than L3's
   (129 vs 144 ms median) on the same configuration; L3's ws numbers came from another session/boot.
2. `after` alone is a null: the detector wakes on the tick, but by the time it has recorded its command
   buffer modeld's next backbone submit is usually already queued, so the detector still waits behind it
   (det 73 -> 92 ms) and modeld's runs stay bimodal.
3. `turn` removes the collisions (every run 55-57 ms; 5 turns/s, 55 ms yield each) and cuts road latency
   (median -17 ms, p90 -38 ms, newest age -16 ms), but costs 0.6 road/s: during a turn the GPU only runs the
   50.7 ms detector, and modeld's ~5 ms preamble and signd's own CPU around the detector are not overlapped.

Two changes follow: (a) signd's detector becomes **one** GPU submit (both outputs recorded into one
command buffer and downloaded together; ncnn's `extract()` submits once per output); (b) modeld can start
its next run `--gpu-lead` ms before the detector's expected end, using a new `det_ms` field that the
provider publishes (moving average of its detector wall time in a turn), so modeld's preamble overlaps the
detector's tail and its backbone submit queues right behind it. Also (c) modeld wraps the two 1.5 MB image
inputs as ncnn Mats instead of copying them (256x128 fp32 planes need no channel padding).

Checks (phone, 22:5x CET, against the installed binaries, ljkr 1920x1080):
- modeld zero-copy: `modeld_replay ljkr.nv12 --width 1920 --height 1080 --backend ncnn --max-frames 40`,
  installed vs L4 build: 39/39 road lines identical apart from timing fields.
- signd single submit: `signd_replay --models-dir /var/lib/roadsense/models --det-backend ncnn --cls-backend ort
  --threads 2 --raw --size 1920x1080 --every 2 --max-frames 60 ljkr.nv12`, then `tools/ncnn/compare_replay.py`:
  14/14 detections matched, box error 0.00 px, score and class probability differences 0.0000.

## 4. Second pass on the DVR camera (suite 2, 22:56-23:08 CET, 200 s each)
| run | modeld / signd | road runs/s | model mean / p50 / p90 | signs/s | det / detect (wait) | road latency med / p90 (age) | signs lat |
|---|---|---|---|---|---|---|---|
| B1 | zero-copy modeld, single-submit signd, no sync | 13.14 | 74.4 / 56.0 / 106.2 | 4.99 | 67.7 / 75.9 | 129 / 175 (171) | 117 |
| T2 | B1 + `--gpu-yield 80 --gpu-lead 6`, signd `turn` | 13.22 | 55.4 / 55.3 / **56.7** | 4.99 | 55.5 / 92.1 (28.4) | **112 / 133** (151) | 132 |
| T3 | as T2, `--gpu-lead 10` | 13.27 | 56.7 / 55.8 / 59.6 | 4.99 | 55.5 / 92.7 (29.1) | 113 / 135 (152) | 133 |

- Zero-copy and the single-submit detector are neutral on rates (B1 vs B0); the single submit takes
  signd's det from 73 to 68 ms with no sync.
- With the lead the turn no longer costs throughput (13.22-13.27 vs 13.14-13.23) and keeps the latency
  gain of T1. GPU trace of T2 (`gpuan3.py`): GPU powered 88 % of the trace, modeld 627 ms/s, signd
  254 ms/s (5.0 x 50.7 ms); modeld's backbone is submitted 0.4 ms after the detector retires (the lead
  lines up), but signd submits ~4 ms after modeld's tick (its wake-up + command recording), and every
  modeld run still has its own ~5 ms gap at the boundary.

### Where modeld's boundary gap goes
`trace_marker` stamps in a throwaway build (`~/l4/diag-rs`, not committed), 79 runs, median: last policy
retire -> download/parse 0.3 ms, JSON + publish 0.9 ms, loop to the next `take` 0.75 ms, Supercombo
image copies 0.5 ms, ncnn command recording + upload staging until the backbone submit **2.9 ms**:
~5.3 ms of GPU idle per run, ~70 ms/s at 13 runs/s; the policy part adds ~2.3 ms of CPU-layer gaps
(MatMul / ExpandDims / Squeeze have no Vulkan kernels).

## 5. Pipelined modeld: the next run's vision stage prepared during the current run
The vision backbone (343 of 499 ncnn layers) depends only on the two image inputs; the recurrent inputs
(desire, traffic convention, lateral params, previous curvature, features buffer) enter after it. So
modeld now runs each pair in two stages (`modeld/backend.h` `record_stage1 / submit_stage1 / run_stage2`,
`modeld/ncnn_backend.cc`, `modeld/supercombo.{h,cc}` `prefetch / submit / run(Prefetch)`): a prefetch
thread (`Prefetcher` in `modeld/main.cc`) takes the next pair and records stage 1 while the main thread
runs stage 2 of the current pair, and submits it the moment the main thread says go (right after the run,
before the JSON; after a granted GPU turn; never before the rate cap allows). Stage 1 is found from the
graph at load time: GPU layers whose inputs come only from in0/in1 or constants; its frontier (here blobs
425-429, 452, 453) is extracted into the job's extractor, which then runs stage 2. Each job owns its blob
and staging allocators (ncnn's VkMat extract path uses none by default). `--no-pipeline` gives the
one-stage loop; ORT has no split.

Correctness (phone, 23:2x CET): `modeld_replay ljkr.nv12 --width 1920 --height 1080 --backend ncnn
--max-frames 40 --pitch -0.085 --yaw 0.025 --roll 0 --height-m 1.22` with `--split` (the new sequential
two-stage path) and without, against the installed binary: 39/39 runs identical in every numeric field
(max abs diff 0). (A first comparison showed large differences in all variants including the unchanged
one-stage path: L5 had installed the ride's `calib.json` in between and modeld_replay reads it by default;
hence the explicit calibration.)

## 6. On the replay source (Krakow city ride, from 23:17 CET)
L5 switched the camera to camerad-replay (1280x720 @ 20 fps, loops) at 23:16-23:17; signd's classifier
sees more signs in the city (live tracks 3 instead of 0), GPU costs are scene-independent. Re-baselined
before comparing. Suite 3, 23:24-23:40 CET, 200 s each:

| run | modeld / signd | road runs/s | model mean / p50 / p90 | signs/s | det / detect (wait) | road latency med / p90 (age) | signs lat | hottest last 2 min |
|---|---|---|---|---|---|---|---|---|
| R0 | one-stage (`--no-pipeline`), no sync = L3 behaviour | 13.03 | 74.4 / 55.8 / 106.6 | 4.96 | 67.9 / 80.2 | 121 / 168 (164) | 118 | 58-62 |
| RT | one-stage, turn, lead 6 (installed since 23:16) | 13.26 | 55.3 / 55.1 / 56.5 | 4.99 | 55.6 / 102.2 (28.6) | **105 / 126** (145) | 142 | 70-75 |
| RP | pipelined, no sync | **14.01** | 71.3 / 54.0 / 103.7 | 4.99 | 72.3 / 85.8 | 153 / 183 (186) | 119 | 78-83 |
| RPT | pipelined, turn, lead 2 | 13.50 | 54.1 / 53.8 / 55.6 | 4.99 | 55.3 / 94.4 (28.7) | 130 / 181 (173) | 127 | 59-63 |

(`model_ms` of the pipelined runs is measured from the stage-1 submit, the recording having overlapped
the previous run.)
Readings: the pipeline wins throughput (+7.5 % over R0) but loses latency: the prefetch thread took its
pair when the previous run's stage 2 started and then waited for go, through the rest of that run (and a
~50 ms detector turn in RPT), so the frame aged before its backbone started. Fix: while waiting for go,
the prefetch thread re-records stage 1 whenever a newer pair arrives (CPU work while the GPU is busy), so
the submitted job always carries the newest frame (`rerec` in the stats line counts it). And signd now
records its detector before asking for the turn and only submits on the grant (`ModelRunner::run_hooked`,
the hook runs between recording and submit), so it reaches the GPU as soon as the turn is granted.

Suite 4 (replay, 23:42-23:54 CET) and suite 5 (23:55-00:07 CET), 200 s each; `rerec` = newer pairs
re-recorded per 10 s; signd from here on records before asking for the turn:

| run | modeld / signd | road runs/s | model mean / p50 / p90 | signs/s | det / detect (wait) | road latency med / p90 (age) | signs lat | GPU busy (trace) | hottest last 2 min |
|---|---|---|---|---|---|---|---|---|---|
| RP2 | pipelined + re-record, no sync | 13.82 | 71.9 / 54.6 / 104.0 | 4.99 | 71.5 / 85.4 | 120 / 167 (162) | 125 | 92.3 % | 78-81 |
| RPT2 | pipelined + re-record, turn, lead 2 | 13.68 | 54.2 / 54.0 / 55.9 | 4.99 | 55.5 / 94.4 (28.1) | **104 / 124** (142) | 130 | 91.0 % | 58-75 |
| RT2 | one-stage, turn, lead 6 | 13.57 | 55.2 / 55.1 / 56.4 | 4.99 | 55.7 / 98.2 (28.2) | 105 / 126 (143) | 135 | 90.1 % | 58-63 |
| RPT2L | RPT2 + CPU wake-up latency held at 0 us | **14.05** | 53.1 / 52.8 / 55.0 | 4.99 | 55.1 / 94.0 (27.3) | **104 / 123** (139) | 127 | 92.7 % | 58-62 |
| RP2L | RP2 + latency 0 us | 14.17 | 70.3 / 53.0 / 102.6 | 4.99 | 71.3 / 88.6 | 119 / 165 (161) | 124 | 92.9 % | 74-80 |
| RPT2b | RPT2 repeated (latency hold off) | 13.59 | 54.1 / 53.9 / 55.9 | 4.96 | 55.6 / 95.3 (28.1) | 106 / 126 (144) | 132 | 91.3 % | 61-74 |

Readings:
1. Re-recording removes the pipeline's latency penalty (RP 153 -> RP2 120 ms median, = R0's 121) and keeps
   most of its throughput (13.82).
2. signd recording before the turn lifts the one-stage turn from 13.26 (RT) to 13.57 (RT2); the pipeline adds
   ~0.1 on top in turn mode (RPT2 13.68, repeat 13.59): with the turns lined up, the remaining idle is
   inside stage 2 (GPU trace of RPT2: stage 1 = one 39.5 ms submit, submitted 0.2 ms after the previous
   run's stage 2; the detector 0.5 ms after modeld's tick; but ~2.2 ms from stage 1's fence to the first
   stage-2 submit and ~2 ms of CPU-layer gaps inside stage 2, ~4.2 ms per run).
3. Those are wake-ups: the big cores' deep idle state (`cpu-sleep-1-0`, 885 us exit latency, ~1 M entries
   on cpu4 since boot) is entered between submits. A process holding `/dev/cpu_dma_latency` at 0 (here a
   `systemd-run` python helper for RPT2L / RP2L) keeps them in WFI: +0.4 road/s in turn mode (14.05 vs
   13.59-13.68), +0.35 without (14.17 vs 13.82); temperatures unchanged within the noise of the scene.
   Built into modeld as `--cpu-latency-us N` (PM QoS request held by modeld, released when it exits).
4. With the latency hold, turn-taking and free-running are tied on road rate (14.05 vs 14.17; the spread
   between repeats of one config is ~0.1), and turn-taking halves the model p90 (55 vs 103 ms) and cuts
   road latency p90 by 42 ms (123 vs 165). Chosen: pipelined + turn (`--gpu-yield 80 --gpu-lead 2`) +
   `--cpu-latency-us 0`, signd `--gpu-sync turn`.

## 7. Chosen configuration, installed (00:08 CET), 15-minute run 00:09-00:24 CET
Installed on the phone: `/usr/local/bin/{modeld,modeld_replay,signd,signd_replay}` from the L4 builds (the
previous binaries kept as `*.pre-l4`), `/etc/systemd/system/roadsense-{modeld,signd}.service` from the repos
(previous units in `~/l4/*.service.pre-l4`), no drop-ins left. Units:
- modeld: `--backend ncnn --cpus 4-7 --prep-cpus 0-3 --max-hz 20 --throttle-temp 95000 --gpu-yield 80 --gpu-lead 2
  --cpu-latency-us 0 --stats 10` (pipelined runs on by default), clock pins unchanged.
- signd: `--det-backend ncnn --cls-backend ort --threads 2 --cpus 4-7 --max-hz 5 --gpu-sync turn --stats 10`.

`~/l4/live.sh FINAL 900` (`systemd-run --unit l4-final`), replay source, envd 1 Hz: modeld 14.05 runs/s mean over
all 90 stats windows (89 at 14.0-14.1, one at 12.5), model 53.1 ms mean, p90 54.9 (max window p90 56.6), 0 run
failures, never throttled; signd 4.99 fps (4.63-5.01), det 55.2 ms, 50 turns per 10 s, mean yield 48.6 ms;
GPU 710 MHz, cpu4 2649 MHz, cpu0 1766 MHz in every sample. ws, last 60 s: road 14.05 msg/s, latency 107 / 126 /
135 ms median / p90 / max, newest road message 143 ms old (p90 189); signs 134 / 162 ms; env 361 ms.
Temperatures: hottest zone 75-83 C in the first 5 minutes, then a 58-62 C plateau for the last 8 minutes
(max 83; the safety stop never came near). `/v1/health`: camera 19.6 fps, road 13.8, signs 5.04.

### Result vs baselines
| metric | L3 baseline (DVR camera, L3 report) | R0 = L3 behaviour on the replay (200 s) | **L4 chosen (replay, 15 min)** |
|---|---|---|---|
| road runs/s (primary) | 13.2 | 13.03 | **14.05** (+7.8 % vs R0) |
| model ms mean / p90 | 74 / ~106 | 74.4 / 106.6 | **53.1 / 54.9** |
| signs/s (detect ms) | 5.0 (81) | 4.96 (80) | 4.99 (97, of which 28 waiting for the turn) |
| road latency frame -> ws, median / p90 | 144 / 189 (L3 FINAL) | 121 / 168 | **107 / 126** |
| newest road message age, median | 183 | 164 | 143 |
| signs latency median | 136 | 118 | 134 |
| modeld alone (signd stopped) | 16.3-17.7 | - | 19.1 (camera at 20 fps) |
| GPU busy (msm trace) | 88 % | 88.6 % | 92-93 % |

Where the 1.0 road/s comes from (replay, 200 s rows): turn-taking alone is neutral to +0.2 (13.26 RT, the
detector no longer stalls a run but each turn leaves the GPU idle while both sides wake); signd recording
before the turn +0.3 (13.57 RT2); pipelined modeld +0.1 in turn mode (13.68 RPT2; +0.8 without turns, 13.82 RP2,
where the free-running detector fills less of the boundary); holding the CPU wake-up latency +0.4 (14.05).
The ceiling at 5 signs/s from the per-submit GPU times is ~15.9 road/s (section 1); the rest is in stage 2's
CPU layers and the hand-offs.

## 8. Robustness of the contract (installed units, 00:25-00:27 CET, `~/l4/robust.sh`)
- signd stopped 25 s: modeld 19.1 runs/s, 0 turns, p90 52.8 ms (alone, camera-bound).
- modeld stopped 12 s: signd 4.98 fps, `gpu wait 0.0`, 0 ticks (no modeld = no waiting); with modeld back,
  50 turns per 10 s again.
- signd killed with SIGKILL three times 4 s apart (systemd restarts it): modeld kept running (16.0-16.8 runs/s
  while signd was down, no failures), yield per turn <= 48.5 ms (no turn hit the 80 ms takeback);
  signd back at 4.6-5.0 fps with turns.
- Older binaries interoperate: an old signd never sets `want`, so modeld never yields (R0/RP rows ran the new
  modeld that way); an old modeld never creates the file, so signd `--gpu-sync turn` never waits.

## 9. Nulls and notes
- `--gpu-sync after` (tick only, no grant): null (13.22 vs 13.23); the detector still lands behind modeld's
  next backbone.
- Turn-taking without the lead: -0.6 road/s (12.62); lead 6 vs 10 ms (one-stage): no difference.
- Zero-copy image inputs: neutral on the rate (kept: one 3 MB copy fewer, identical outputs).
- Free-running pipelined modeld + latency hold (RP2L 14.17) ties the chosen config on road rate but keeps the
  bimodal runs (p90 103 ms) and road latency p90 165 ms; not chosen.
- The 13-minute road outage at 22:27-22:40 CET was this lane's mistake (section 2).
- Open: stage 2 still idles the GPU ~2 ms per run at its CPU layers (MatMul / ExpandDims / Squeeze have no ncnn
  Vulkan kernels); moving those to GPU-capable ops in the converted graph is the next lever (~+0.5 road/s at
  most). `--cpu-latency-us 0` keeps all cores out of deep idle: no measurable heat on the bench, but it adds
  idle power in the car.

## 10. Phone state at hand-back
Running from the installed units: camerad-replay (L5's replay source, Krakow ride), modeld, signd, envd, api;
`roadsense.target` disabled for boot. Scratch `~/l4/` (sources, builds, `live/` results and traces, `chk/`
replay comparisons, `diag-rs` + `build-diag` = the throwaway marker build).
