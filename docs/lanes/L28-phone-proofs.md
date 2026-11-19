# L28: the pending phone measurements on the merged `v2` branches

Execution lane of the 2.0 campaign, 2026-11-19 01:40-05:26 CET (Warsaw local), Opus. Phone on the bench (no camera, no
gateway), reached over its access point; rides replayed. Nothing installed: the merged branches (roadsense `v2`
27b9fcc, roadsense-signs `v2` 3749b60) were built in `/home/user/l28` (`l28-build`, 01:45-01:51) and run from
there as transient units; models in `/home/user/l28/models` (copied from `/home/user/l27/models`, sha256 as in
L27 section 1). Phone lock taken from `L24` as `L28`. Harness: `tools/governor/l28/` (batch scripts, profile) and
two small fixes (section 4).

## Summary

| job | result | verdict |
|---|---|---|
| A. governor proof, new default | smoke and the order pass; watts per level measured (6.55 W -> 3.39 W from level 0 to 9); **no CPU limiter exists on this kernel** (only the GPU zones have a cooling device), so without the governor the stack runs into the 95 C safety stop (fan 10 %: new stack at 420 s, the installed builds at 366 s); with it, 8 min at mean 74.5 C (max 91.3 C), 1.3 W less, road / lead / lanes whole and never marked degraded | 6 of 7 criteria pass (one vacuously); **road latency p90 +13.5 ms fails the 10 ms bound**; alternation seen at the lights step (5 <-> 6), one restore-and-regret in the first heat-up |
| B. fp16 lead after the tracker | three 4-5 min stretches with leads; fp16 adds about 0.5-0.6 % of the distance (0.02-0.07 m under 30 m, about 1 m p95 beyond 60 m); after the tracker the same on/off and identity counts on the two day rides, 0-2 extra jumps per 5 min; on a1-night (far leads, night) the two tracks follow different hypotheses in 7-12 % of the both-present frames, counts moving both ways and less than the fp32 stream's own rate-to-rate difference | fp16 can stay; not a measurable worsening in flicker; a far-range distance offset of about 0.5 % |
| C. sign proposals A/B | +3 % of a big core on `sg-lights`, +2 % on `sg-signs`, no GPU, road runs unchanged; `signs` 5 -> 10 a second; late light updates 13 % (`keep`) / 5 % (`all`) after a first minute in which nearly all were late; road latency p90 13-30 ms higher (45 s probes, base runs differ by 13 ms) | cheap in CPU; the latency effect needs a longer repeat before either option becomes a default |


## A. Governor proof with the new default road model (L24 runbook, section 5)

Stack: `tools/governor/l28/profile.sh` = this build, vision + policy at 5 Hz, signd of roadsense-signs `v2`
(governor-aware: `signs_hz`, `lights_hz`), L21/L27's proposed arguments (envd on the little cores, lights at 5 Hz,
the car light-state model). Ride `krakow-2024` from its first frame. Runbook levels translated (L27): 8 = `plan`,
9 = `road`.

### A1. Smoke (runbook step 2): pass

`cond.sh smoke 6 100 120` (forced level 6, fan 100 %), seconds 20-120: signs 2.5/s and lights 2.5/s, every message
`shed` + `slow`; `view` 4.98/s; `env` 0.2/s; road 17.6/s, not capped, `governor_level` 6 in `road`; `lead` /
`lanes` 17.6/s, never degraded. Everything the runbook asks.

### A2. Watts per level (runbook step 3), fan 100 %, 150 s each, from second 60

`for l in 0 3 4 5 6 7 8 9; do cond.sh lvl$l $l 100 150; done` (a cool start before each), `compare.py lvlN.jsonl --from 60`:

| level (step taken last) | rail W | big cores C mean / hottest max | road runs/s | road latency median / p90 ms | signs / lights / env / view per s |
|---|---|---|---|---|---|
| 0 (nothing shed) | 6.55 | 61.6 / 77.8 | 15.4 | 95.8 / 119.7 | 4.99 / 4.99 / 1.0 / 15.4 |
| 3 (preview, recorder pictures, scenery 1 in 5 s) | 5.95 | 57.0 / 73.5 | 15.3 | 99.8 / 123.3 | 4.99 / 4.99 / 0.2 / 15.3 |
| 4 (`view` 5/s) | 5.93 | 56.9 / 75.2 | 15.1 | 88.0 / 119.4 | 4.99 / 4.99 / 0.2 / 5.0 |
| 5 (signs half) | 5.72 | 55.8 / 76.5 | 17.5 | 97.6 / 121.0 | 2.5 / 4.99 / 0.2 / 5.0 |
| 6 (lights half) | 4.91 | 48.9 / 68.9 | 17.6 | 107.5 / 123.7 | 2.5 / 2.5 / 0.2 / 5.0 |
| 7 (scenery paused, detectors a quarter) | 4.34 | 44.1 / 65.0 | 18.7 | 100.9 / 119.0 | 1.25 / 1.25 / - / 5.0 |
| 8 (`plan`, policy 2/s) | 4.26 | 43.8 / 65.7 | 18.9 | 90.6 / 119.0 | 1.25 / 1.25 / - / 5.0 |
| 9 (`road` at 10/s) | 3.39 | 40.6 / 61.7 | 10.0 | 105.0 / 122.2 | 1.25 / 1.25 / - / 5.0 |

Readings:

* Measured savings per step: scenery 0.60 W, `view` 0.02 W, signs at half rate 0.21 W, lights at half rate 0.81 W,
  scenery paused + detector floor 0.57 W, plan 0.08 W, road at half rate 0.87 W. L24's estimate (section 4)
  had lights 1.55 W at 5 Hz (0.78 W for half: matches), scenery 0.65 W (matches), road 2.6 W at 13.8 (measured
  0.87 W for 15.4 -> 10: the new model is cheaper and the cut is smaller).
* Shedding the sign detector gives the road model GPU time: 15.4 -> 17.5 runs/s from level 5 on, up to 18.9.
  The full stack is 6.55 W; level 7 is 4.34 W, level 9 3.39 W.

### A3. The fan duty for the proof (runbook step 4): the criterion cannot be met on this phone

`cond.sh pilotF off F 300` (governor off), last minute:

| fan % | rail W | hottest mean / max, C | samples over 90 C | limiter active | clocks |
|---|---|---|---|---|---|
| 60 | 6.73 | 69.5 / 81.1 | 0 | 0 % | at the pins (2650 / 1766 MHz, GPU 710 MHz) |
| 40 | 6.79 | 71.5 / 82.1 | 0 | 0 % | same |
| 25 | 6.89 | 75.0 / 85.0 | 0 | 0 % | same |
| 10 | 7.02 | 80.8 / 91.9 | 5 % (from s 180) | 0 % | same |
| 0 | 7.19 | 86.3 / **96.2** | 27 % (from s 180) | 0 % | same |

**The platform limiter never engaged, at any duty, up to 96 C.** The reason is in the kernel's thermal binding: on
this phone's kernel only the two GPU zones (`gpu-top`, `gpu-bottom`, passive trip 85 C) have a cooling device
bound (`devfreq-5000000.gpu`); the CPU zones (`cpu0`-`cpu7`, passive trips 90 / 95 C, critical 110 C) and the
cluster zones have none (`/sys/class/thermal/thermal_zone*/cdev*` is empty for them). The cooling devices
`cpufreq-cpu0` / `cpufreq-cpu4` exist but nothing drives them; with modeld's clock pins the big cores stayed at
2650 MHz at 96 C. The hottest zone is always a big core, and the GPU zones stayed under their 85 C trip. So the
runbook's "fan duty where the ungoverned stack sits at 90-94 C with the limiter active > 50 %" does not exist: the
ungoverned stack heats until the lane's 95 C safety stop (or, without it, towards the 110 C critical trip).

Safety: in `pilot0` single one-second samples reached 95.5 C (s 229), 95.2 C (s 260) and 96.2 C (s 287) on a
big-core zone; the stop of `trace.py` at the time needed three samples in a row over 95 C and did not fire; the run
ended at 300 s as planned. From then on every run used a stop on the first sample over 95 C (`--stop-n 1`,
section 4). The big-core sensors swing by 10-20 C within seconds under this load (bursty threads), so a one-second
sample is a spike reading, not a plateau.

Chosen instead: **F = 10 %**, the lowest duty at which the ungoverned stack stayed under 95 C for 300 s (step 7
of the runbook: record and adapt), 480 s runs.

### A4. The proof (runbook step 5), 480 s each from a cool start, window from second 180

`batchA3.sh 10`: `ref` (governor on, fan 100), `hotN` (off, 10 %), `hotG` (on, 10 %), `ref2` (on, 100), `hotOld`
(L21's builds, supercombo, the installed api, `--throttle-temp 95000`: the private guards), then
`compare.py ref.jsonl ref2.jsonl hotN.jsonl hotG.jsonl --from 180`.

| | `ref` (fan 100) | `ref2` (fan 100) | `hotN` (no governor, fan 10) | **`hotG` (governor, fan 10)** | `hotOld` (installed builds, fan 10) |
|---|---|---|---|---|---|
| ran to the end | yes | yes | **no: safety stop at 420 s (95.9 C)** | yes | **no: safety stop at 366 s (95.2 C)** |
| hottest zone mean / max, C | 68.3 / 80.4 | 68.6 / 80.8 | 80.9 / 95.9 | **74.5 / 91.3** | 81.9 / 95.2 |
| samples over 90 C / over 85 C (from s 180) | 0 / 0 % | 0 / 0 % | 12.7 / 31.4 % | **1.7 / 10.0 %** | 19.0 / 37.5 % |
| rail W | 6.67 | 6.67 | 7.05 | **5.73** | 7.23 |
| limiter active | 0 % | 0 % | 0 % | 0 % | 0 % |
| governor level, share of time | 0: 100 % | 0: 100 % | 0 (off) | 3: 2 %, 4: 17 %, 5: 20 %, 6: 61 % | - |
| road runs/s | 15.05 | 15.10 | 15.19 | **17.09** | 13.63 (supercombo) |
| road latency median / p90 / p99 ms | 89.6 / 110.2 / 130.2 | 87.3 / 115.2 / 129.7 | 92.1 / 118.4 / 132.2 | **98.8 / 123.7 / 136.0** | 108.0 / 130.2 / 142.7 |
| longest road gap ms | 154 | 136 | 126 | 150 | 135 |
| `lead` / `lanes` degraded | 0 / 0 % | 0 / 0 % | 0 / 0 % | **0 / 0 %** | 0 / - |
| signs / lights per s (degraded %) | 4.99 / 4.99 (0) | 4.99 / 4.99 (0) | 4.99 / 4.99 (0) | 2.97 (68) / 3.47 (45) | 4.99 / 4.98 (0) |
| env / view per s | 1.0 / 15.1 | 1.0 / 15.1 | 1.0 / 15.2 | 0.2 / 5.2 | - |
| ego lines of `road` vs `ref`, p95 dy at 9 / 32 / 61 m (left; right) | ref | 0.044 / 0.085 / 0.257; 0.044 / 0.079 / 0.217 | 0.016 / 0.030 / 0.081; 0.018 / 0.035 / 0.084 | **0.036 / 0.071 / 0.212; 0.037 / 0.071 / 0.186** | not comparable (other model) |
| tracked lines of `lanes` vs `ref`, p95 at 9 / 32 m (left; right) | ref | 0.03 / 0.05; 0.035 / 0.05 | 0.02 / 0.03; 0.02 / 0.035 | **0.03 / 0.045; 0.03 / 0.045** | - |
| tracked lead distance vs `ref`, median / p95 relative | ref | 0.003 / 0.012 | 0.001 / 0.010 | **0.002 / 0.013** | - |
| tracked lead relative speed, median / p95 m/s | ref | 0.045 / 0.33 | 0.01 / 0.30 | **0.03 / 0.30** | - |
| lead presence disagreeing with `ref` | ref | 1.46 % | 0.0 % | **0.0 %** | - |

(`hotN`'s errors cover s 180-420 only. Its errors are lower than `ref2`'s because it hits the same frames as
`ref` more often: neither ran anything the other did not, the stack is deterministic up to scheduling.)

Pass criteria for `hotG` (runbook step 6):

| criterion | measured | verdict |
|---|---|---|
| road messages a second not lower than `ref2`'s | 17.09 against 15.10 | **pass** |
| road latency median within 10 ms of `ref` | 98.8 against 89.6 ms (+9.2) | **pass** (barely) |
| road latency p90 within 10 ms of `ref` | 123.7 against 110.2 ms (+13.5) | **fail** |
| lane-line and lead errors within twice `ref2`'s | all at or below `ref2`'s (table) | **pass** |
| limiter active < 5 % of the time | 0 % | pass, **but vacuous**: no CPU limiter is bound on this kernel (A3) |
| `lead` / `lanes` never marked degraded | 0 / 0 % | **pass** |
| shed steps in the stated order | preview -> recorder pictures -> scenery -> view -> signs -> lights, restores in reverse (trace of `health.governor.shed` once a second) | **pass** |

The latency: shedding the sign detector lets the road model run more often (17 instead of 15 a second), and each
run then waits a little longer for its GPU turn; the same shows in the forced levels (A2: level 5-6 median 98-108
ms against 96 at level 0) and in the smoke run. It is a latency cost of the detector steps, not of heat: `hotN`
at 15.2 runs/s has 92 / 118 ms. More road messages, each about 9 ms (median) / 13 ms (p90) later.

What the governor bought on this phone is not "no throttling" (there is none to avoid) but temperature: the
ungoverned new stack and the installed stack both ran into the 95 C stop at fan 10 % within 6-7 minutes; the
governed stack spent 1.3 W less, ran the whole 8 minutes at a mean of 74.5 C with a single 91.3 C sample, and
kept road, lead and lanes whole.

### The two blemishes of the emulation

1. **Limiter cutting during the first heat-up**: not reproducible here, there is no limiter. What did happen in
   the first heat-up of `hotG`: preview, recorder pictures and scenery were shed at 49-51 s (headroom 8.9 K, a
   forecast on a rising spike), then restored to level 2 at 83 s and level 1 at 114 s (headroom 18 / 14 K after the
   spike passed), then shed again at 127-131 s (headroom 5 K): one restore-and-regret cycle of about 45 s during
   heat-up. The step-up followed the sensor's spikes more than the plateau.
2. **Alternation at the road step**: the road step (now 9) and the plan step (8) were never reached at fan 10 %
   (deepest level 6). Alternation did occur one step earlier, at the lights step: 6 at 263 s, 5 at 295 s, 6 at 317
   s, 5 at 379 s, 6 at 391 s (on for 32, 62 s; off for 22, 12 s). The doubled hold lengthened the second "on"
   period (32 -> 62 s) as designed; it does not stop the alternation within 8 minutes. Each restore was triggered by
   headroom 16-18 K right after a spike had passed, each re-shed by headroom 6-7 K about 15-20 s later: the big-core
   sensor noise (A3) is larger than `restore_at_k` (9 K) minus `shed_at_k` (4 K). The runbook's fix ("raise
   `restore_at_k`") applies; a smoothed hottest reading (median of a few seconds) is the other candidate. Not changed
   here (execution lane).

Thermal time constants (runbook "also"): at fan 10 % the ungoverned stack went from a cool start (34 C) to the
first sample over 90 C in 250 s (`hotN`, `pilot10`) and 108 s at fan 0; at fan 100 it settles within about 60 s
(cooling from 70 to 50 C takes under 20 s). The dwell and hold times (seconds to tens of seconds) are of the right
order; the noise, not the time constant, is the issue.

## B. fp16 lead distance after the tracker

Both arithmetic paths on the phone's GPU, on the same decoded frames: ncnn Vulkan fp16 packed (as deployed) and
ncnn Vulkan fp32 (`modeld_replay --ncnn-fp32`, same graph and backend, only the arithmetic differs), vision model
without the policy (the tracker reads `leads`, `v_ego`, `pose`), streams at the deployed rate (15/s) and at L27's
13.8/s (`--rates 13.8,15`: one network run per frame, rate selection as on the phone). Frames: the rides' MJPEG
decoded on the phone (`mjpeg_nv12.py`, one decode feeding both paths). `batchB.sh`, 03:30-04:58.
Reference check: ORT fp32 on the CPU on the first 400 frames of warsaw-city gives the same lead as ncnn fp32 to
0.001 m (raw and tracked, 78 frames with a lead): ncnn fp32 is a valid fp32 reference. Scored by
`tools/vision_eval/lead_fp16.py REF RUN` (the api's `LeadTracker` with the vision profile on each stream).

Clips (the stretches of L24-M / L27, chosen for leads; all three are L27 scoring rides, none was a tuning ride):

* **warsaw-city 0-300 s**: day, city, three lanes, close leads (lead present 76 % of runs);
* **s7-elblag-lowsun 60-300 s**: low sun, expressway, far leads (30 %);
* **a1-night 0-300 s**: night motorway, far leads, several vehicles (66 %).

| ride, rate | min | frames both raw leads >= 0.5 | raw distance error mean / p95 / max m (mean relative) | raw presence disagree % | tracked distance error mean / p95 / max m | tracked v_rel error mean / p95 m/s | tracked presence disagree % | on/off per min fp32 / fp16 | identity changes per min fp32 / fp16 | jumps per min fp32 / fp16 | per-frame step rms m fp32 / fp16 | identity changes of fp16 not in fp32 / of fp32 not in fp16 | present % fp32 / fp16 |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| warsaw-city 15/s | 5.0 | 3094 | 0.20 / 0.79 / 1.25 (0.49 %) | 0.18 | 0.21 / 0.77 / 1.03 (median 0.06; 0.49 %) | 0.07 / 0.26 | 0.02 | 2.20 / 2.20 | 2.00 / 2.00 | 2.80 / 2.80 | 0.56 / 0.55 | 0 / 0 | 75.7 / 75.7 |
| warsaw-city 13.8/s | 5.0 | 2857 | 0.20 / 0.79 / 1.14 (0.49 %) | 0.17 | 0.23 / 0.78 / 5.98 (median 0.06; 0.53 %) | 0.10 / 0.31 | 0.0 | 2.20 / 2.20 | 2.20 / 2.20 | 3.00 / 3.40 | 0.61 / 0.60 | 0 / 0 | 74.7 / 74.7 |
| s7-elblag-lowsun 15/s | 4.0 | 952 | 0.54 / 1.13 / 1.80 (0.64 %) | 0.14 | 0.62 / 1.01 / 9.48 (median 0.55; 0.74 %) | 0.30 / 0.64 | 0.17 | 2.00 / 2.00 | 1.00 / 1.00 | 0.25 / 0.75 | 0.76 / 0.77 | 0 / 0 | 30.1 / 30.0 |
| s7-elblag-lowsun 13.8/s | 4.0 | 879 | 0.54 / 1.14 / 1.78 (0.63 %) | 0.12 | 0.54 / 1.00 / 2.45 (median 0.53; 0.61 %) | 0.25 / 0.67 | 0.21 | 2.00 / 2.00 | 1.25 / 1.25 | 0.50 / 0.50 | 0.76 / 0.76 | 0 / 0 | 29.6 / 29.4 |
| a1-night 15/s | 5.0 | 2421 | 0.43 / 1.43 / 3.46 (0.58 %) | 0.44 | 0.75 / 2.71 / 27.1 (median 0.33; 0.98 %) | 1.37 / 6.61 | 0.22 | 4.80 / 4.80 | 6.41 / 7.01 | 7.41 / 7.61 | 1.27 / 1.21 | 6 / 3 | 66.4 / 66.2 |
| a1-night 13.8/s | 5.0 | 2228 | 0.43 / 1.39 / 3.46 (0.57 %) | 0.34 | 0.97 / 3.74 / 21.5 (median 0.35; 1.21 %) | 1.62 / 8.04 | 0.29 | 4.40 / 4.40 | 7.81 / 6.61 | 9.41 / 8.01 | 1.31 / 1.32 | 0 / 6 | 63.5 / 63.2 |

Tracked distance error by the fp32 track's distance (mean / p95, m):

| ride, rate | 0-15 m | 15-30 m | 30-60 m | 60 m and beyond | frames over 2 m (share of both-present) |
|---|---|---|---|---|---|
| warsaw-city 15/s | 0.02 / 0.05 | 0.07 / 0.17 | 0.29 / 0.53 | 0.62 / 0.86 | 0 |
| warsaw-city 13.8/s | 0.02 / 0.05 | 0.07 / 0.17 | 0.39 / 0.74 | 0.61 / 0.88 | 14 (0.5 %), two episodes |
| s7-elblag-lowsun 15/s | - | - | 0.34 / 0.59 | 0.73 / 1.05 | 17 (1.6 %) |
| s7-elblag-lowsun 13.8/s | - | - | 0.24 / 0.61 | 0.64 / 1.02 | 2 (0.2 %) |
| a1-night 15/s | - | 0.06 / 0.16 | 0.31 / 0.94 | 0.93 / 3.52 | 205 (6.9 %) |
| a1-night 13.8/s | - | 0.05 / 0.12 | 0.28 / 0.99 | 1.24 / 4.78 | 307 (11.7 %) |

Readings:

* **The fp16 error is a far-range scale error, about 0.5-0.6 % of the distance on average** (raw, all three
  rides; L27's 120-frame clip gave 1.1 %, 0.37 / 0.86 m). Under 30 m it is 0.02-0.07 m mean, 0.05-0.17 m p95:
  irrelevant against the model's own scatter there (0.5 m at 15-30 m, L27 section 3). From 60 m on it is 0.6-0.9 m
  mean, about 1 m p95, below the model's own distance scatter at that range (2.2-2.4 m). The lead speed is barely
  touched (raw 0.03 m/s mean on warsaw-city).
* **After the tracker, on warsaw-city and s7 the tracker passes the error through unchanged** (tracked error =
  raw error), with the same on/off and identity counts in both paths, and 0-2 extra distance jumps in 4-5 minutes
  (warsaw-city 13.8/s: +2, s7 15/s: +2; elsewhere equal). Its short divergences (14-17 frames over 2 m) are moments
  where one path took a relocation and the other a few frames later.
* **On a1-night the tracker amplifies it**: in 7-12 % of the frames where both show a lead the two tracks follow
  different hypotheses (far leads at night, several vehicles), errors up to 21-27 m. The counts move in both
  directions (identity changes 6.4 -> 7.0 a minute at 15/s, 7.8 -> 6.6 at 13.8/s; jumps 7.4 -> 7.6, 9.4 -> 8.0), and
  the same fp32 stream at the two rates differs by more (identity 6.4 against 7.8, jumps 7.4 against 9.4): fp16 is
  one more small perturbation of decisions that are fragile on this ride anyway, not a systematic worsening.
* Verdict: fp16 does not make the lead worse after the tracker in flicker or presence on any of the three rides;
  it adds a distance error of about 0.5 % of the distance on average (beyond 60 m 0.6-0.9 m mean, about 1 m p95
  on the two day rides) and, where the
  tracker is already undecided between far vehicles, changes which one it follows at some moments. The deployed
  path (fp16) can stay; an fp32 vision run costs 1.4 times the GPU time in this tool (model 317-320 against
  228-234 ms per run here, both paths and the installed stack sharing the GPU), which the budget does not have.
* Limits: no ground truth (fp32 is the reference, not the truth); 14 minutes on three rides; frames decoded by
  Pillow on the phone (0.5 LSB from ffmpeg's decode used for the laptop streams), so the laptop's ORT streams were
  not used as a reference. The installed roadsense target was running during B (section 5); it shares the GPU and
  changes only the time per run, not the outputs.


## C. Sign proposals from the light detector, A/B on the phone (roadsense-signs L24-S section 7)

The L28 stack of A (new default road model, governor automatic, it stayed at level 0) on ride `krakow-2024`, fan
100 %, cool start; `tools/budget/run.sh` (240 s budget after 60 s, then 45 s `/ws` probe), in the order base,
`--lights-signs-keep --lights-signs-nohits`, `--lights-signs`, base again (`batchC.sh`, 05:00-05:25). One run each.

| | `c-base` | **`c-keep`** (`--lights-signs-keep --lights-signs-nohits`) | **`c-all`** (`--lights-signs`) | `c-base2` |
|---|---|---|---|---|
| rail W, mean / last 60 s | 6.68 / 6.66 | 6.79 / 6.77 | 6.79 / 6.84 | 6.54 / 6.58 |
| `sg-lights`, % of a big core | 89.0 | 92.4 | 92.0 | 89.9 |
| `sg-signs`, % of a big core | 10.9 | 12.7 | 12.8 | 10.9 |
| big cores busy (of 400 %) / their temperature last 60 s | 136.6 % / 67.0 C | 142.2 % / 67.8 C | 142.3 % / 68.7 C | 137.3 % / 66.5 C |
| api CPU little / big | 8.9 / 11.4 % | 10.0 / 11.8 % | 9.2 / 12.2 % | 9.1 / 11.4 % |
| GPU busy, % of wall (signd / modeld) | 84.6 (25.3 / 59.3) | 83.6 (25.4 / 58.2) | 84.3 (25.4 / 58.9) | 83.8 (25.4 / 58.4) |
| road runs/s (modeld's count) | 15.28 | 15.10 | 15.28 | 15.14 |
| `road` at `/ws`: rate, latency median / p90 ms | 15.02, 84.0 / 94.1 | 15.38, 104.7 / 124.0 | 15.42, 92.0 / 120.1 | 15.02, 88.1 / 107.5 |
| `signs` messages a second, latency median / p90 | 5.0, 130.0 / 144.6 | 9.98, 143.5 / 177.4 | 9.98, 144.0 / 172.6 | 5.0, 141.7 / 160.4 |
| `lights` messages a second | 5.0 | 5.0 | 5.0 | 4.98 |
| light updates older than the last message (cumulative counter of signd) | - | 295 at the first stats line (60 s), 478 at the end | 292, 364 | - |
| ... after the first minute | - | 0.65 a second = 13 % of the 4.99 light updates a second | 0.26 a second = 5 % | - |
| `rules` messages a second | 1.13 | 1.24 | 1.24 | 1.13 |

Readings:

* **Cost of either option: about +3 % of a big core on the light thread** (89-90 -> 92 %; the sign classifier on
  the proposals) **and +2 % on the sign thread** (10.9 -> 12.7 %, the extra tracker updates), +5 % big cores in all,
  0.1-0.2 W (within the 0.14 W spread of the two base runs, at the edge of it), +1 C on the big cores. No GPU cost
  (signd's GPU time unchanged, 25.4 %), road runs unchanged (15.1-15.3 a second).
* **`signs` doubles to 10 messages a second** (every published light update is a message, view 2), each later than
  the detector's messages (median 144 against 130-142 ms).
* **Late light updates: on the phone most are published**, unlike on the laptop (L24-S: 109 of ~120 late). In the
  first minute nearly all were late (about 290 of the first ~300, both options: the start-up while the models
  warm), after that 13 % (`keep`) and 5 % (`all`). So the phone runs the "published" case of L24-S's replay more
  than the "quiet" one its scores assumed.
* **Road latency**: 105 / 124 ms (`keep`) and 92 / 120 ms (`all`) against 84 / 94 and 88 / 108 ms for the two
  base runs. The p90 is 13-30 ms higher with either option. The probe is 45 s per run and the two base runs differ by
  13 ms at the p90 themselves, so this is suggestive, not established; the plausible cause is the extra big-core
  work next to modeld's `md-main` / `md-prefetch` threads. It matters under guideline 1 (latency counts on the
  measurement topics): a repeat with longer probes is the way to settle it.
* Quality was not measured here (no labels on the phone; L24-S's scores stand). What the phone adds to L24-S's
  verdict: the CPU price is small (~5 % of a big core), the published share is high (87-95 %), so the "published"
  rows of L24-S section 3 are the relevant ones for both options, and the road latency effect needs a second look
  before either option becomes a default.


## 4. Harness changes (committed on `v2-l28`)

* `tools/governor/trace.py`: `--stop-n N` (samples over `--stop-c` in a row before the stop; default 3 as before).
  L28 ran with 1 after `pilot0` (A3).
* `tools/governor/cond.sh`: passes `$TRACE_ARGS` to `trace.py`.
* `tools/governor/l28/`: `profile.sh` (stack of this lane), `cool.sh` (stack down, fan 100 %, wait for a
  temperature), `hot.sh`, `batchA1.sh` (smoke + levels), `batchA2.sh` (fan pilot; `FANS` overrides the duties),
  `batchA3.sh` (the proof), `batchB.sh` (fp16 / fp32 streams), `batchC.sh` (sign proposals A/B).
* `tools/vision_eval/mjpeg_nv12.py`: MJPEG -> NV12 without ffmpeg (the phone has none); several outputs so two
  arithmetic paths read the same decoded frames. Against ffmpeg on the laptop: same frame indices, 0.5 LSB mean
  difference (IDCT rounding).
* `tools/vision_eval/lead_fp16.py`: two road streams of the same frames before and after the lead tracker
  (distance / speed error, presence, per-stream flicker of `lead_eval.py`, unmatched identity changes).

## 5. Phone: what ran, side effects, end state

* Timeline (CET): lock taken 01:40; build 01:45-01:51; A1 01:51-02:18; A2 02:18-02:48 (fans 60/40/25, then 10/0);
  A3 02:49-03:29; B 03:30-04:58; C 05:00-05:25; lock removed 05:26. No reset, no reboot, no drop of the link.
* Hottest readings: 96.2 C (one 1 s sample, `pilot0`, fan 0 %, governor off, A3); the safety stop ended `hotN`
  at 95.9 C and `hotOld` at 95.2 C (first sample over 95 C) as designed.
* The host's `roadsense_mode on` starts the installed target when it is not running; `stack.sh up` stops it again
  (as in L21 / L27). During B (no candidate api on 8090) the installed target ran from 03:30:30 until mode `auto`
  at 04:58 stopped it; it shared the GPU with B's replays (outputs unaffected). Each start opened a bench recorder
  session in `/var/lib/roadsense/rec` (`20261119-015403`, `20261119-033032`, `20261119-050016`; left in place).
* End state (05:26): no `l28-*` or `l21-*` units; installed `roadsense.target` inactive (as found); fan automatic
  and roadsense mode `auto` (both commands accepted); clock pins released (scaling minimums at the hardware
  minimum, GPU minimum 257 MHz); hottest zone 32 C; `/run/phone-lane.lock` removed; the netwatch drop-in
  `/run/systemd/system/prius-netwatch.service.d/99-lane-off.conf` **left in place** (prius-netwatch inactive);
  nothing installed, `/etc` and the car backend untouched, no `apk`, no adb.
* Files: `/home/user/l28` (1.5 GB: sources, builds, models copy, `out/` traces and logs, `fp16/` streams 430 MB);
  other lanes' directories untouched. Laptop copies of the results: `/mnt/win/linux-scratch/l28/{out,fp16}`.

## 6. Open

* The governor's probe sees a limiter that does not exist for the CPU on this kernel. Either the 95 C passive trip
  (and the 110 C critical) is the real boundary, or a cooling binding for the CPU zones is missing in the device
  tree. Worth a decision: the governor's urgent / last-resort logic keys on the limiter state.
* Hottest-zone noise (median 16 C range within 5 s on the big cores under load) drives the governor's restores and
  re-sheds; a smoothed reading or a larger `restore_at_k` - `shed_at_k` gap is the candidate fix.
* The latency cost of the detector shed steps (road p90 +13 ms at levels 5-6) and of the sign proposals: both are
  GPU / big-core turn-taking effects; a longer measurement per configuration would separate them from noise.
* The road step (9) and plan step (8) were never reached automatically; a hotter condition (fan 0 % with the
  governor) would test them, at the cost of running close to the stop.

