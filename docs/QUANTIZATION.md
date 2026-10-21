# supercombo v0.9.7 on ORT 1.30 CPU, Poco F1 (SDM845): int8 / mixed-precision study

Date 2026-10-21 (20:05-20:45 CEST). Lane result: **no int8 or mixed variant is both faster and accurate enough.
Keep fp32.** Recommended file: `/tmp/claude-1000/-home-piotr/4097b1e0-437a-49af-9dad-9e35d3c588f6/scratchpad/fp32/supercombo-fp32.onnx`
(unchanged). `models/supercombo-opt_ext.onnx` is an optional drop-in: identical outputs on the laptop, same run time,
about 25 % faster session load on the phone (600 vs 800 ms). Its outputs were not checked on the phone.

## Files (all in this directory)

| file | what |
|---|---|
| `gen_inputs.py` | builds 119 real input sets from the Ljubljana 1920x1080 NV12 clip (spec sec. 2 warp+pack, same code as `scripts-modeld/crosscheck.py`), rolls fp32 closed-loop (features_buffer / prev_desired_curv feedback, v_ego from `ljkr_road.jsonl`), saves `inputs.npz`. Check vs the C++ modeld_replay dump runs 0-2: max abs diff 3.4e-4 |
| `quant.py` | **the quantisation script**, one variant per call: `python quant.py <variant>` writes `models/supercombo-<variant>.onnx` |
| `sweep.py`, `sweep.tsv` | per-node sensitivity: quantise one Conv/Gemm/MatMul at a time, lane-y error and hidden-state cosine |
| `evalq.py`, `eval_results.json` | accuracy vs fp32 + laptop speed; `NOISE<x>` pseudo-variants = fp32 with +-x grey-level uniform input noise |
| `ortbench2.cc` | phone bench (`~/quant-work/ortbench2`): `-c 4-7` affinity, `-nospin`, `-s 60` sustained with 10 s windows, `-prof` ORT profile, temperatures |
| `profagg.py` | per-op-type sum of per-node *minimum* times from ORT profiles (robust to contention) |

Calibration: MinMax over the 60 even-indexed runs. Accuracy is measured on the 59 odd runs (held out), with the recurrent
inputs taken from the fp32 rollout ("TF"). It is also measured as a closed loop over all 119 runs, where the variant feeds back its
own hidden state ("CL"). Weights are per-channel S8 and activations U8 unless stated otherwise. Desire is zero, traffic convention is RHT.
Note: `CalibMaxIntermediateOutputs` is broken in ORT 1.31 (it discards the data and raises "No data is collected").

## Accuracy (TF, held-out runs; worst case over runs unless "mean")

Acceptance criteria: lane y < 0.05 m mean and < 0.2 m max, lead x within 1 m, probabilities within 0.05.

| variant | size MB | laptop ms (i7-8850H, 4 thr) | lane y mean / max (m) | lane y x<=48 m mean / max | lane prob | lead x all / lead p>0.5 (m) | lead prob | plan y / plan x max (m) | hidden cos min | CL lane y mean / max |
|---|---|---|---|---|---|---|---|---|---|---|
| fp32 | 102.7 | 12.9 | 0 | 0 | 0 | 0 | 0 | 0 | 1.0000 | 0 |
| opt_ext (fp32, ORT EXTENDED offline) | 102.7 | 13.8 | 0.000 / 0.00 | | 0 | 0 | 0 | 0 | 1.0000 | 0 |
| dyn_mm (dynamic int8 MatMul, Gemm->MatMul) | 45.7 | 12.0 | 0.358 / 5.31 | 0.121 / 1.36 | 0.344 | 23.5 / 23.5 | 0.176 | 18.2 / 95 | 0.9994 | 0.352 / 5.14 |
| qdq_full (Conv+Gemm+MatMul) | 27.2 | 28.9 | 2.10 / 23.0 | | 0.963 | 63.8 | 0.843 | 41 / 259 | 0.03 | 1.91 / 24.7 |
| qdq_full_x (stem conv + heads fp32) | 31.8 | 32.8 | 2.31 / 25.1 | | 0.952 | 72.4 | 0.935 | 50 / 278 | -0.02 | 2.06 / 27.6 |
| qdq_conv (Conv only) | 83.8 | 29.8 | 2.94 / 36.6 | | 0.969 | 64.5 | 0.947 | 54 / 222 | 0.01 | 5.04 / 61.0 |
| qdq_conv_x (stem fp32) | 83.8 | 29.8 | 3.29 / 39.7 | | 0.972 | 58.7 | 0.967 | 58 / 217 | 0.00 | 5.23 / 53.2 |
| qdq_conv_s8 (S8 activations) | 83.8 | 38.0 | 3.03 / 41.0 | | 0.978 | 63.9 | 0.963 | 56 / 229 | -0.02 | 4.60 / 53.4 |
| qop_conv (QOperator format) | 83.5 | 30.9 | = qdq_conv | | | | | | | |
| qdq_s2 (int8 only on 12 stage-2 1x1 convs) | 95.8 | 13.7 | 0.043 / 0.87 | 0.018 / 0.23 | 0.074 | 5.7 / 4.4 | 0.063 | 1.65 / 12.6 | 0.995 | 0.049 / 1.27 |
| qdq_s23 (+ stage-3 fc1, SE) | 90.7 | 13.3 | 0.048 / 1.01 | | 0.080 | 5.0 | 0.061 | 1.52 / 11.2 | 0.994 | 0.055 / 1.25 |
| nb8 (8-bit block-32 weight-only MatMulNBits on all Gemm/MatMul) | 47.9 | 23.7 | 0.025 / 0.27 | 0.013 / 0.07 | 0.023 | 15.7 / 7.6 | 0.018 | 0.55 / 12.3 | 0.9999 | 0.026 / 0.40 |
| nb8acc4 (same, int8 compute) | 47.9 | 12.3 | 0.038 / 0.44 | | 0.073 | 14.4 | 0.034 | 1.19 / 13.0 | 0.9998 | 0.039 / 0.57 |
| nb8_s2 | 40.9 | 24.2 | 0.050 / 0.93 | | 0.082 | 16.0 | 0.071 | 1.68 / 11.4 | 0.995 | 0.055 / 1.34 |
| *fp32 + +-0.5 grey-level input noise* | | | 0.012 / 0.15 | 0.004 / 0.03 | 0.016 | 0.6 / 0.5 | 0.013 | 0.65 / 2.0 | 0.999 | 0.012 / 0.27 |
| *fp32 + +-1 grey level* | | | 0.031 / 0.41 | 0.012 / 0.09 | 0.064 | 3.1 / 3.1 | 0.056 | 1.16 / 10.1 | 0.996 | 0.032 / 0.46 |
| *fp32 + +-2 grey levels* | | | 0.073 / 0.80 | 0.035 / 0.43 | 0.136 | 8.0 / 4.8 | 0.080 | 3.6 / 22 | 0.921 | 0.086 / 2.09 |

Findings:
* **Full static int8 destroys the model.** The hidden-state cosine to fp32 is about 0, whether the format is QDQ or QOperator, whether activations are U8 or S8, and with or without the stem and heads excluded.
  The FastViT/MobileOne backbone has heavy-tailed activations. For example, the stem.2 output spans -235..129 with std 4.7, and the final_conv input spans
  -595..552 with std 30, so an 8-bit per-tensor activation step is about 1.4-4.5 units. The sweep (`sweep.tsv`) shows that even a *single*
  quantised node gives 0.004-1.7 m mean lane-y error. The worst are the depthwise token mixers of stages 0-1, stem.1/2, final_conv, stage-3 fc2 and
  the attention c_proj.
* **The model's own sensitivity is about as large as the acceptance criteria.** Plain fp32 with +-1 grey-level input noise (far below camera noise)
  already fails them: 0.41 m lane max and 3 m lead x. +-0.5 LSB passes only the mean. The 0.2 m / 1 m thresholds are therefore
  stricter than the model's own input-noise floor. The best quantised variants (nb8, qdq_s2) sit at about the +-1 to +-2 LSB level.
  nb8's lead x error (7.6 m when lead p>0.5) is worse than that floor.

## Speed on the phone (A75 cluster, cpus 4-7, 4 intra-op threads, zero inputs)

**Conditions: heavily contended and thermally saturated. These numbers are not comparable to the 54 ms cold baseline.** The phone was
rebooted at about 20:30 CEST, and roadsense.target then ran as root: modeld (fp32, `--cpus 4-7 --max-hz 5`, about 20 % CPU) on the *same* big
cluster, signd (about 27 %), and camerad. The ncnn/glslang build in ~/vulkan-work was compiling on all cores during the first runs and was later stopped (state T).
The maximum thermal zone stayed at **84-96 C throughout**. The <60 C cool-down was impossible without stopping services, which this lane did not do.
All comparisons are therefore back-to-back and interleaved under identical conditions.

| variant | interleaved bench, median of 3x15 runs (ms) | sum of per-node min time, ORT profile (ms) | 60 s sustained, 10 s window medians (ms) | -nospin median (ms) |
|---|---|---|---|---|
| fp32 | 254-271 (first rounds, ncnn build running: min 68, median 110) | 122-128 | 254, 266, 270, 315, 279, 281 (overall 272) | 242 |
| opt_ext | 240-259 | | | 231 |
| qdq_s2 | 212-248 | 125 | 269, 263, 262, 267, 317, 269 (overall 269) | 223 |
| qdq_conv / qop_conv / qdq_conv_s8 | 153-183 vs fp32 110 in the same round | 108 (vs 81 fp32 in the same session) | | |
| qdq_full | 134-157 vs fp32 110 | | | |
| dyn_mm | 122-125 vs fp32 110 | | | |
| nb8 / nb8acc4 | 367-421 vs fp32 254-271 | 233 (MatMulNBits alone 124 ms vs 15 ms fp32 Gemm) | | |

Why int8 does not pay on this ORT 1.30 ARM64 build (from the per-node profiles):
* Pointwise 1x1 convs do get faster in int8 (QLinearConv about 2x faster than the fp32 Conv/FusedConv for the same nodes). The
  12 stage-2 convs save about 8.5 ms, but the added QuantizeLinear/DequantizeLinear nodes cost about 10 ms, so the net is zero.
  qdq_s2 is within noise of fp32 (269 vs 272 ms sustained).
* Depthwise convs (3x3 and 7x7, about a quarter of fp32 conv time) are 2.4x **slower** in int8 (31.5 vs 13.1 ms).
* MatMulNBits 8-bit (weight-only) has no fast path here and is about 8x slower than the fp32 Gemm. Dynamic MatMulInteger is break-even.
* Elementwise GELU-tanh (Mul/Add/Tanh) is about 20 % of fp32 time and is untouched by any quantisation.
* `session.intra_op.allow_spinning=0` was within noise (it was slightly faster under contention, 242 vs 254-272 ms). Use it when sharing cores.

Untested routes that could still give a large gain: ORT fp16 kernels (A75 has FP16 arithmetic; needs an fp16 model plus an ORT build with
ARM fp16 MLAS); XNNPACK EP (good int8/fp16 depthwise); ncnn (the other lane); quantisation-aware fine-tuning.
A different graph is also an option: the v0.11 split models at 5 Hz, as recommended in the spec, sec. 6.

## Big-core clock cap

* Available gold-cluster frequencies (kHz): 825600 902400 979200 1056000 1209600 1286400 1363200 1459200 1536000 1612800
  1689600 1766400 1843200 1920000 1996800 2092800 2169600 2246400 2323200 2400000 2476800 2553600 2649600.
  The boost frequency is 2803200 (`boost`=0, disabled). cpuinfo_max is 2649600 and the governor is schedutil. The energy model (/sys/kernel/debug) is not readable.
  time_in_state shows the cluster requested 2.65 GHz most of the time. The 4-5x slowdown at about 95 C is therefore
  thermal (LMh hardware limiting below the requested OPP) plus core sharing with modeld, not governor choice.
* Estimate, using latency ~1/f for this compute-bound model and power ~f*V^2. Public SDM845 gold OPP voltages run from about 0.6-0.65 V at 0.83 GHz to about
  0.95-1.0 V at 2.65 GHz, and about 0.8 V near 2.0 GHz. Energy per inference therefore scales with V^2: about 0.65x at 2.0 GHz relative to 2.65 GHz,
  for about 1.3x the latency.
* **Recommendation: cap cpu4-7 `scaling_max_freq` at 2092800** (or 1996800 if it still sits at the thermal limit). The cold
  latency estimate is about 54*2.65/2.09 ≈ 68 ms, well inside the 200 ms budget of modeld's `--max-hz 5`. Per-inference
  heat drops by about 1/3, which should keep the SoC out of the LMh throttle region where the uncapped clock currently yields 110-150 ms.
  Setting the cap needs root, so it must come from the root-run roadsense service or its unit (ExecStartPre writing the sysfs file). Verify with
  `ortbench2 -s 60` after the change.
