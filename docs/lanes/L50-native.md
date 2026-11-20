# L50: native code where it pays

Lane L50, 2026-11-20 from 11:35 CET, Fable. Owner's ruling: "We can move to native compiled code wherever it
would make a tangible execution time difference. C / Rust, whatever would work best; you can install any
compiler required." Branch `v2-l50` from `v2-l49` fae9098 (roadsense), `l50-native` from `l48-opt` c7a8b81 (car
repository). Nothing pushed, nothing merged.

**State of this report: the laptop work is complete; the phone was held by lane L49 for the whole time, so
nothing here was measured or installed on the phone.** Every phone number below is L46's (the "before") or an
estimate marked as such. Section 6 is the order for the phone, ready to run.

## 0. Result

chris-integrator has a native core: one C file with the arithmetic and the text work of the per-message path.
The pure-Python implementation stays, is the reference, and is what runs wherever the compiled module is
absent. Both pass the same test suite; a switch (`CHRIS_NATIVE`) and `/v2/health` say which one runs.

| | before (`v2-l49`) | this branch, pure Python | this branch, native |
|---|---|---|---|
| laptop core, a recorded drive with vehicle feed and the map box (20261119-113201), ms per road frame | 3.17 | 3.00 | **1.59** (-50 %) |
| laptop core, the bench capture (raw `road` lines, no vehicle feed), ms per road frame | 2.29 | 2.24 | **1.36** (-41 %) |
| ... median / 99th percentile frame (drive) | 2.49 / 3.6-4.4 | 2.33 / 3.3-3.6 | 1.14 / 1.8-1.9 |
| outputs against `v2-l49` | | identical, line for line (4 inputs) | see below |
| outputs, native against pure Python: the exact parts (JSON, rounding, the display walk, the lane tracker) | | | identical, line for line |
| outputs, native against pure Python: all parts (the filter too) | | | every stream identical except `ego_state`: 69 of 55,786 messages differ, each in one field, by the sign of a zero |
| phone, little core, bench: per-message path | 16.6 ms (L46) | not measured | not measured; expected about 10 ms |
| phone, with a car feed replayed | 22.0 ms (L46) | not measured | not measured; expected about 11-12 ms |

**Bad news first.**

* **Nothing was measured on the phone.** The expectation above is the laptop's ratio applied to L46's phone
  numbers; the phone prices numpy calls higher than the laptop does, so the native parts should gain more
  there than here, but that is an argument, not a measurement.
* **It is a half, not an order of magnitude, for the whole daemon.** The functions made native are 4 to 26
  times cheaper than their Python on the laptop (section 4); the daemon as a whole is twice as cheap, because
  what is left is not arithmetic: it is the estimator's bookkeeping (monitors, fault detection, sources, the
  5 kB `ego_state`), the trackers' decisions and the mixer, spread over about 12,000 lines of Python with no
  function above 5 % (section 7). Making that native is a rewrite of the integrator in another language, kept
  in step with a reference of the same size. It was not done; section 7 says what each part would give.
* **The filter is not bit-identical between the two implementations**, and cannot be made so against numpy:
  numpy hands sums of products to BLAS, whose order of additions is its own. After the monitors and the
  trackers the difference is a rounded zero's sign in one calibration field (section 3).
* JSON reading is 2.4 times cheaper, not ten: what remains is CPython building the objects.

## 1. What is native, and why those parts

Measured first (`tools/cost_bench.py --profile` on the recorded drive, laptop; L46's unit prices for the
phone): the filter's loop and its linear algebra were 27 % of the daemon, JSON text 15 %, the lane tracker
8 %, `round()` 4 %; everything else was below 5 % a function.

| part | native function | what it replaces | why |
|---|---|---|---|
| `ekf` | `ekf_predict` | the process model: 25 numpy calls on a 27-state covariance | the filter's inner loop: 5 a frame with a car feed |
| `ekf` | `ekf_project`, `ekf_update`, `ekf_sym` | a scalar row: P h, the gain, the update (Joseph form for consider states), the symmetric part: 10 numpy calls | 8-17 rows a frame |
| `ekf` | `ekf_rts` | one smoother step (`np.linalg.solve` and three products) | 0.6 a frame; no faster than LAPACK on the laptop (section 4), kept for the call overhead on the phone |
| `json` | `dumps` | `json.dumps(msg, separators=(",", ":"))` for every line that leaves (WebSocket, sockets, recorder) | 1.5 us a float on the phone |
| `json` | `dumps` of `Rounded` | the recorder's reduced `road`: 700 numbers rounded and written; replaces L46's number tables (and their 8 MB) | the largest single line written |
| `json` | `loads` | `json.loads` of every input line | 1 us a number on the phone |
| `round` | `round`, `round_list` | `builtins.round` (correct in decimal: 2.1 us on the phone, 260 a frame) | spread over every message builder: 25 modules import it |
| `interp` | `interp_all` | the display box's table walk, 153 interpolations a frame | |
| `lanes` | `lanes_measure`, `lanes_predict`, `lanes_fit`, `lanes_apply`, `round_lists` | the lane tracker's arrays: twelve lists into arrays, noise, gate, innovation, fit, update, rounded lists: 70 numpy calls on 6 x 33 values | numpy calls on tiny arrays are what the phone prices highest |

The control flow stays in Python in every case: which row is gated, which line is present, what a message
contains. The native functions are arithmetic on arrays and text on objects, with no knowledge of the stack.

Also done in Python, with identical outputs (they showed up once the arithmetic was gone): the innovation
monitors keep their window sorted instead of sorting 80 values five times a second per row class; the filter
remembers per entry which rows were applied, and `near()` walks the window first (the reach of a position is
the whole ten-second buffer: every applied row of it was walked several times a frame); the rules' turn window
keeps its extremes as it slides (it walked a window that grows by one sample a frame **while the car stands**:
0.5 ms a frame on a phone after a minute at a light, rules 0.17 -> 0.05 ms a frame on the laptop drive); plain
lists are tested for finiteness without a call per element.

## 2. Language, binding, build chain

**C99, the CPython limited API (abi3), arrays through the buffer protocol.** One translation unit, 2,070
lines; the module is 75 kB.

| considered | verdict |
|---|---|
| C, CPython limited API, buffer protocol (chosen) | one binary for every CPython from 3.11 on (the phone's 3.14.8 loads a module built against 3.14.10 headers; a Python update on the phone does not break it); no numpy headers and no numpy ABI; builds with any C compiler and the Python headers; nothing vendored; plain enough to read next to the Python it mirrors |
| Rust + PyO3 | memory safety by construction, the real argument for the parser. Against: a Rust toolchain and maturin for a public pure-Python package, a cross-linker for aarch64 musl, the `numpy` crate for arrays; the numeric code would be `unsafe`-free but the gain is the parser alone. The parser's exposure is answered otherwise (below) |
| Cython, the Python sources compiled as they are | **measured: 10 % slower** than CPython 3.14 on this code (54 modules compiled, outputs identical, 2.55 against 2.32 ms a frame). The interpreter specialises untyped code better than Cython compiles it; typed Cython is a rewrite like C, with a non-stable ABI per Python version |
| numpy C API | saves the buffer protocol's few hundred nanoseconds a call; ties the binary to numpy's ABI |
| orjson for JSON | 1.6 times faster than this core at reading and 2.5 times at writing (measured on the captured messages, laptop). Not used: its text is not `json.dumps`'s (small floats without exponent, `1.5e-7` for `1.5e-07`, NaN as `null`, no `\u` escapes: identical on 1,269 of 1,837 captured messages), so the recorder's files and every consumer's lines would change; and it is not on the phone (`py3-orjson` is not installed, and the phone's packages are pinned) |

**Memory safety of the parser.** `loads` reads the daemons' lines and the vehicle feed. It is a
recursive-descent reader over a (pointer, end) pair with every read checked, nesting limited to 64, strings
validated by CPython's own UTF-8 decoder, and no guesses: whatever is not plain goes to `json.loads` (so
errors are CPython's). Evidence: 40,000 cut and corrupted lines must give `json.loads`'s result or an error
(`test_loads_survives_damaged_input`); the native tests pass under the address and undefined-behaviour
sanitizers (`SANITIZE=1 sh native/build.sh`; they found one zero-length copy into an unallocated buffer,
fixed). The writer runs no Python code while it walks a message.

**Build chain.** Nothing is compiled on the phone and nothing is installed on it.

    sh chris-integrator/native/cross-build.sh
    # -> chris-integrator/native/dist/linux-aarch64-musl/_chris_native.abi3.so + BUILD.txt

builds in an `alpine:edge` arm64 container on the laptop (docker; qemu-user through binfmt), gcc 15.2.0,
musl, Python 3.14.10 headers, and runs the self-test and `tests/test_native.py` there against the container's
numpy 2.5.5 (23 passed, emulated: the lane tracker's and the single-index filter rows' bit-for-bit tests
included, so the claim "numpy's bits" holds on aarch64 with OpenBLAS's NEON kernels too). The artefact of the
committed source: sha256 `5382b037...a190` (source `569d5083...ff54`). `native/dist/` is not tracked;
`scripts/sync-phone.sh` now carries it with the tree.

Flags: `-O3 -ffp-contract=off -fno-fast-math`, no `-march`. `-ffp-contract=off` matters: a fused
multiply-add would change bits that are otherwise numpy's.

Public packaging: `setup.py` adds the module as an **optional** extension (`pip install .` builds it when a
compiler is there, installs pure Python when not: tried with `CC=/bin/false`; `CHRIS_NATIVE_BUILD=0` forces
pure). Wheels are `cp311-abi3`. `docs/NATIVE.md` has the build, the contract and the wheel instructions.
Licences: original code under the package's MIT; nothing vendored; links libm and the CPython stable ABI.

## 3. Equivalence

"Identical" = every message of every stream a plug-in produces, the recorder's file, the WebSocket
subscriber's lines and the socket's greeting lines equal as text, in order (`tools/cost_bench.py --dump`, `diff`).

**Exact parts** (JSON, rounding, the display walk, the lane tracker): identical by construction and by test;
`docs/NATIVE.md` says why each is exact. The lane tracker's two means over the gate points are added in the
order numpy adds them there (in sequence; found by experiment: `a[:, mask]` comes out in column order, so the
reduction is not pairwise). That is numpy's implementation, not its contract; `tests/test_native.py` holds it.

**The filter.** State propagation, the transition matrix, and rows of one state index (most rows: speed, yaw
rate, standstill, altitude, heading) give numpy's bits. The covariance propagation, rows over several states
and the smoother agree to rounding (1e-13 relative on a step).

Laptop, CPython 3.14.4, numpy 2.5.4, the map box in process:

| input | road frames | `v2-l49` against this branch, pure | pure against the exact native parts | pure against all native |
|---|---|---|---|---|
| 20261119-113201 (15 min, vehicle-fed) | 9,745 | IDENTICAL | IDENTICAL | `ego_state` 9,748 of 9,760 identical; everything else identical |
| 20261119-104128 (38 min, vehicle-fed) | 28,306 | IDENTICAL | IDENTICAL | `ego_state` 28,294 of 28,306 |
| 20261119-151146 (the incident drive) | 17,720 | IDENTICAL | IDENTICAL | `ego_state` 17,675 of 17,720 |
| bench capture (raw `road` lines, 563 frames: the laptop's partial copy) | 563 | IDENTICAL | IDENTICAL | IDENTICAL |

Every one of the 69 differing `ego_state` messages differs in one field, `calibration.odometry.speed_scale.value`,
printed `-0.0` in one implementation and `0.0` in the other: the speed scale is unobservable without an
absolute source, its estimate is rounding noise around zero, and the noise has the other sign. `lead`,
`tracks`, `lanes`, `view`, `rules`, `fused`, `map`, `speed_limit`, `roadgeom`, `egolane`, `lights`, `signs`:
identical on all three drives.

**With the receiver and the map box in the loop** (the recordings carry no fixes; L47's tools inject the trip
log's positions, which exercises position and heading rows, the map's rows, re-anchoring: where rounding
could move a decision). Reference against native, both from this branch:

| tool (L43 / L47) | result |
|---|---|
| `tools/mapper_eval.py real`, four drives: held-out fixes, map availability, delay state, re-anchorings, normalised error | every reported number identical (the files differ in their CPU lines only) |
| `tools/integrator_eval.py sim`, town and motorway x three receivers: errors against truth, NEES | identical text, 6 runs |
| `tools/fault_campaign.py sim` (59 faults): detection, isolation, wrong-valid seconds | identical JSON |
| `tools/fault_campaign.py real` on the 104128 slice | identical JSON |
| `tools/mixer_equiv.py`, frame by frame, four drives | 113201 and 153636: the whole stream identical (same SHA-256); 104128: 6 of 28,306 `ego_state`; 151146: 2 of 17,720; the same field, the same sign of zero; every other topic identical |
| `tools/mapper_eval.py synth` (Krakow, three receivers, two seeds) | every reported number identical |

Tests (chris-integrator): 393 pass on the native core; 384 pass and 9 are skipped (the native comparisons)
with `CHRIS_NATIVE=0`; the same 384 without the compiled module. chris-mapper 30, api 23. New:
`tests/test_native.py` (23: every native function against its reference on random, tie, extreme and damaged
input; the lane tracker frame by frame; a simulated drive through both implementations in two processes),
`tests/test_exact_savings.py` (4: the Python savings against the plain forms they replaced).

Commands:

```sh
cd ~/projects/wt/roadsense-l50; PY=~/projects/roadsense/.venv-models/bin/python; D=~/datasets/roadsense-rec
export PYTHONHASHSEED=0; PACK=~/roadsense-maps/packs/poland-20261114-f2.rsmap; O=/tmp/l50; B=chris-integrator/tools/cost_bench.py
(cd chris-integrator && sh native/build.sh)                         # this machine's module, with its self-test
(cd chris-integrator && PYTHONPATH=/usr/lib/python3/dist-packages $PY -m pytest -q -p no:cacheprovider tests)
(cd chris-integrator && CHRIS_NATIVE=0 PYTHONPATH=/usr/lib/python3/dist-packages $PY -m pytest -q -p no:cacheprovider tests)
for s in 113201 104128 151146; do          # inside systemd-run --user -p MemoryMax=24G
  $PY $B run $D/20261119-$s --pack $PACK --tree ~/projects/wt/roadsense-l49 --dump $O/${s}_base.jsonl.gz
  CHRIS_NATIVE=0 $PY $B run $D/20261119-$s --pack $PACK --dump $O/${s}_py.jsonl.gz
  CHRIS_NATIVE=json,round,interp,lanes $PY $B run $D/20261119-$s --pack $PACK --dump $O/${s}_exact.jsonl.gz
  CHRIS_NATIVE=1 $PY $B run $D/20261119-$s --pack $PACK --dump $O/${s}_nat.jsonl.gz
  $PY $B diff $O/${s}_base.jsonl.gz $O/${s}_py.jsonl.gz; $PY $B diff $O/${s}_py.jsonl.gz $O/${s}_exact.jsonl.gz
  $PY $B diff $O/${s}_py.jsonl.gz $O/${s}_nat.jsonl.gz; done
# the regression tools, once with CHRIS_NATIVE=0 and once with 1 (P=$D/trip-pos/pos_20261119.txt):
$PY tools/mapper_eval.py real $D/20261119-113201 --pos $P;  $PY tools/integrator_eval.py sim --scenario town --rx car
$PY tools/fault_campaign.py sim -j 4 --out fc.json;  $PY tools/fault_campaign.py real $D/20261119-104128 --pos $P --from 260 --to 560 -j 4 --out fcr.json
$PY tools/mixer_equiv.py run $D/20261119-104128 --pos $P --mapper box --out eq.jsonl.gz;  $PY tools/mixer_equiv.py diff A B
# sanitizers:  SANITIZE=1 OUT=/tmp/asan sh native/build.sh; cp /tmp/asan/*.so chris_integrator/native/
#   LD_PRELOAD=$(cc -print-file-name=libasan.so):$(cc -print-file-name=libubsan.so) ASAN_OPTIONS=detect_leaks=0 $PY -m pytest -s tests/test_native.py
# cost: the runs above without --dump, with --json OUT;  unit prices: taskset -c N $PY chris-integrator/tools/native_bench.py CAPTURE
```

## 4. Cost (laptop; the phone is section 6)

One laptop core (`taskset`), `tools/cost_bench.py`, two passes each, ms per road frame:

| stage | drive with vehicle feed: `v2-l49` | pure | native | bench capture: `v2-l49` | pure | native |
|---|---|---|---|---|---|---|
| reading the inputs | 0.159 | 0.159 | 0.081 | 0.215 | 0.212 | 0.123 |
| estimator | 1.69 | 1.64 | 0.84 | 0.757 | 0.720 | 0.453 |
| lanes | 0.278 | 0.275 | 0.108 | 0.342 | 0.337 | 0.149 |
| view | 0.118 | 0.117 | 0.092 | 0.242 | 0.240 | 0.160 |
| rules | 0.172 | 0.052 | 0.045 | 0.042 | 0.041 | 0.038 |
| roadgeom, egolane, ego_light, the others | 0.15 | 0.15 | 0.14 | 0.23 | 0.22 | 0.22 |
| recorder | 0.417 | 0.415 | 0.185 | 0.156 | 0.154 | 0.081 |
| WebSocket output | 0.128 | 0.126 | 0.041 | 0.239 | 0.240 | 0.065 |
| mixer dispatch | 0.061 | 0.060 | 0.054 | 0.064 | 0.062 | 0.061 |
| **total** | **3.17** | **3.00** | **1.59** | **2.29** | **2.24** | **1.36** |

Unit prices on the same core (`tools/native_bench.py`, a captured `road` line and the stack's own messages
for it), microseconds per call:

| | pure Python | native | ratio |
|---|---|---|---|
| filter: one prediction, 27 states | 41.2 | 5.0 | 8.3 |
| filter: one scalar row (project, update, symmetrise) | 13.6 | 2.2 | 6.1 |
| filter: one smoother step | 40.4 | 39.3 | 1.0 |
| lanes: the frame's measurement arrays | 61.8 | 4.5 | 13.6 |
| lanes: noise, gate, innovation, fit | 38.0 | 3.5 | 10.9 |
| lanes: update of 4 lines | 16.1 | 0.6 | 25.8 |
| lanes: rounded lists | 15.0 | 7.2 | 2.1 |
| `round` of a list of 200 / 200 calls of `round` | 67.9 | 3.9 / 16.1 | 17.3 / 4.2 |
| interpolate one line at 14 distances | 3.5 | 0.7 | 5.0 |
| the recorder's reduced `road` line (against L46's table method) | 206 | 58 | 3.6 |
| write `lanes` (2.8 kB) / `road` (15.3 kB) | 88.6 / 481 | 16.7 / 107 | 5.3 / 4.5 |
| read `lanes` / `road` / `road` as the default stack does (trimmed) | 47.5 / 217 / 133 | 19.5 / 94 / 55 | 2.4 / 2.3 / 2.4 |

The order of magnitude is reached where the reference was numpy calls on small arrays (the lane tracker, the
prediction; the phone should show more, a numpy call there costs 5-18 us against 1-2 here) and for bulk
rounding; not for JSON (object building in CPython is the floor: 2.4 reading, 4.5-5.3 writing) and not for the
smoother (27-state products: LAPACK is as fast as plain C; the gain is only the call overhead).

**Memory**: the two number-text tables of L46 (8 MB) are no longer built when the native core writes the
recorder's line; the module adds 75 kB. Resident memory on the phone was not measured.

## 5. Selection, fallback, the plug-in contract

* `chris_integrator/native/__init__.py` decides at import: the compiled module loads and has the expected
  version -> native; `CHRIS_NATIVE=0` -> pure Python; `CHRIS_NATIVE=json,round,...` -> those parts; absent or
  not loadable -> pure Python with the reason. `GET /v2/health` -> `"native": {"active", "implementation",
  "parts", "version", "reason"}`; the daemon logs `hot path NATIVE (json, round, interp, ekf, lanes)` or
  `hot path pure Python: <why>` at start; `python -m chris_integrator.native` prints the same and runs the
  self-test.
* A native function that is not certain of the reference's result for an input raises `Fallback` (or calls
  the reference itself) and the reference runs for that call: unknown types, numbers outside the exact paths,
  damaged text, arrays that are not contiguous. No behaviour exists only in C.
* `scripts/install.sh` (section "2b"): a module at hand for the host's machine and C library
  (`native/dist/linux-<machine>-<libc>/`, or built in the tree, or `NATIVE=/path`) is copied in and must pass
  `python3 -m chris_integrator.native` on the host; then `native core: IN USE (...)`. Fails or absent: removed
  / nothing, and `native core: ... pure Python` with the reason. `NATIVE=0` forces pure. The script never
  compiles. Tried in scratch prefixes under dash and busybox sh: nothing at hand, a fitting artefact (twice:
  re-runnable), an artefact of another machine (removed, pure), `NATIVE=0` over an installed module, `NATIVE=`
  a path, a module built in the tree.
* **The mixer is untouched**: inputs, bus, plug-ins declared by `name` / `inputs` / `outputs` / `DEFAULT_CFG`,
  outputs. How compiled code fits the plug-in contract (`docs/NATIVE.md`, "A native plug-in"): (1) a Python
  box with a native kernel, as the estimator, the lane box and the display box now are: the box stays the
  declared, replaceable unit and its arithmetic exists twice; (2) a box written as an extension type (C, Rust
  with PyO3, Cython): any importable class with those attributes and `on_input` is a plug-in, named in
  `mixer.json` as before; (3) a box in another process in any language over `mixer.remote.RemotePlugin`'s JSON
  lines. This lane used (1) only.

## 6. The phone: not done, and the order for it

The phone was lane L49's until this report was written. Read only (13:12 CET): Python 3.14.8, numpy 2.5.4,
OpenBLAS 0.3.30, musl 1.2.6-r5, gcc 15.2.0-r9, no `python3-dev`, no `py3-orjson`; the installed package is
`v2-l49` fae9098. **Nothing of this lane is on the phone; nothing was installed there; the CMDB was not
touched.**

What is to be done there, in this order (the inputs are L46's, still on the phone: `~user/l46/bench.cap.gz`,
3,035 frames, and `~user/l46/car113201-slice`):

1. Trees: `git archive fae9098` as `~user/l50/base`, this branch's head as `~user/l50/final`, with
   `native/dist/linux-aarch64-musl/_chris_native.abi3.so` copied to `final/chris-integrator/chris_integrator/native/`.
   `python3 -m chris_integrator.native` from `final/chris-integrator`: the self-test on the phone's own
   Python, numpy and BLAS.
2. Stack off (`roadsense_mode off`), learned state saved with checksums (as L46 section 7).
   Quiet little core, `taskset -c 1`: `cost_bench.py run` on both inputs for `base`, `final` with
   `CHRIS_NATIVE=0` and `final` native, twice; `native_bench.py` for the unit prices; then the dumps and
   `diff` (base / pure / exact parts / all native). **Pass**: pure identical to base; exact parts identical
   to pure; all native differing from pure in nothing but `ego_state` zeros' signs (or state what else);
   native clearly cheaper. Otherwise the phone stays on pure Python.
3. Deployment (authorised for the bench): a rollback folder `~user/rollback-20261120-l50` with the installed
   package and `RESTORE.txt`; the package of this branch and the module into `/usr/local/lib/chris`; live
   runs on the replay playlist with `tools/cost_live.py`, the two implementations alternating by
   `CHRIS_NATIVE=0` / unset in `/etc/chris/integrator.env` (200 s warm-up, 240 s window, as L46 section 5):
   share of a little core, ms per road frame, latency, resident memory, temperatures.
4. Back to the state as found (replay off, DVR source, state by checksum, bench recordings out of the
   recordings folder, mode `auto`), `~/.infra/servers/poco.md` updated in place.

The order with its scripts is `orders/L50-phone.md`.

Expected, from the laptop's ratios applied to L46's phone stages (an estimate, not a measurement):

| stage, phone, ms per road frame | bench: L46 | expected | car feed: L46 | expected |
|---|---|---|---|---|
| reading the inputs | 1.4 | 0.7 | 1.0 | 0.5 |
| estimator | 5.2 | 2.8-3.2 | 11.5 | 5-6 |
| lanes | 2.1 | 0.6-0.9 | 1.8 | 0.5-0.8 |
| view | 1.7 | 1.1 | 0.8 | 0.6 |
| rules | (in "others") | | 2.2 | 0.6 |
| the other boxes | 1.9 | 1.8 | about 1.4 | 1.3 |
| recorder | 2.7 | 1.2 | 2.8 | 1.2 |
| WebSocket output | 1.3 | 0.35 | 0.7 | 0.25 |
| mixer dispatch | 0.6 | 0.55 | 0.6 | 0.55 |
| **total, quiet core** | **16.6** | **about 10** | **22.0** | **about 11-12** |
| live, share of one little core at 15 frames/s | 39 % (26 ms a frame) | about 28-30 % | 45-50 % (L46's estimate) | about 30 % |

The live figure keeps L46's 9 ms a frame of kernel, event loop and shared-cache cost, which this lane did not
touch.

## 7. What stayed in Python, with the numbers

Shares of the native build's per-message path on the laptop drive (1.59 ms a frame; the stages are
`cost_bench.py`'s, the estimator is split by a profile of the same run):

| what | share | why it stays |
|---|---|---|
| estimator: a source's message path (`on_message`, `_measure`, the monitors, fault detection), the camera lead, the `lead` and `tracks` messages | 22 % | bookkeeping per measurement: deques, dicts, thresholds, small dict messages. No arithmetic to speak of; a port is a rewrite of the fault logic |
| estimator: `ego_state` (the 5 kB message: quantities, per-source health, checks; tables rebuilt at 5 Hz; the smoother) | 11.5 % | building dicts of rounded numbers; the rounding and the smoother step are native |
| estimator: the filter loop (3.5 points of it are the native calls themselves) | 10.7 % | each row calls a source's Python model `h(x)` and the monitors' callbacks: moving the loop to C keeps those calls and saves perhaps a third of the rest |
| estimator: the object layer (tracks, association) | 8.5 % | scalar filters and association in Python, 640 lines |
| lanes, after the native arrays | 6.8 % | presence, candidates, the message |
| view | 5.8 % | the display channel: list arithmetic over 6 lines x 14 points; a kernel for it would take about half of it. Display only: not done |
| rules, roadgeom, egolane, lights, signs, speed limit, fusion | 11.5 % | decisions; a little numpy in roadgeom (0.11 ms a frame on the laptop bench capture) |
| recorder | 11.6 % | 4.5 points of it are gzip (zlib, native already); the rest is the reduction of `road` and file handling |
| reading the inputs | 5.1 % | native; the cut of unread members (L46) and object building |
| WebSocket and socket output | 3.0 % | native text |
| mixer dispatch | 3.4 % | 11 envelopes and 13 plug-in calls a frame |

About 80 % of what the daemon now costs is Python byte code; 13 % is the native core, 3.5 % zlib.

The whole per-message path native (parse -> estimator -> trackers -> serialise, Python left for wiring and
I/O) would remove most of the 80 % that is still Python: an estimated further 1.0-1.2 ms a frame on the laptop,
6-8 ms on the phone, i.e. the daemon at 3-5 ms a frame on a little core instead of about 10. The price is
about 12,000 lines (core 4,900, sources 2,500, trackers 5,000) written a second time and kept equal to the
reference line for line, fault logic included. It is the order of magnitude; it is a project of weeks with its
own equivalence campaign, and it changes what the public package is (a native program with a Python reference
instead of a Python program with native kernels). **Not done here; the owner's call.** The next steps short
of that, each with the same method as the lane tracker's: the object layer (8.5 %), the display box's lines
(half of 5.8 %), the filter loop with the row models of the fixed sources in C (a third of 7 %).

**Beyond the integrator.**

| process | hot loop | number | verdict |
|---|---|---|---|
| chris-mapper (in process: the `mapper` box) | the matcher, every 0.45 s of input time | 0.4-0.9 ms a match on the laptop (`tools/mapper_eval.py real`, "the box alone": 0.5-1.1 % of a laptop core at the tool's rate; in the daemon 2.2 matches a second) = about 1 % of a little core by estimate | not tangible; left |
| roadsense debug face | parsed every `road` line (5 % of a big core) | L48 made it parse on demand: under 5 % of a little core | nothing left to take |
| car backend | after L48's pacing fix: frame path, engine loop, scene feed | car repository `docs/lanes/L50-native.md`: unpacking a CAN transfer 0.62 us a frame on the laptop (0.7 % of a core at 1,690 frames/s); the trip log per changed frame 10-20 % of the core while driving, cut by a format change (L48-B candidate 2), not by a compiler; the scene feed's JSON 2 % | nothing taken; no code changed there |
| kernel and event loop of the integrator | 15 kB lines in and out, 10 messages in and 4.7 out a frame | 4-5 ms a frame live (L46). Tried on the laptop: a protocol-based socket reader instead of `StreamReader.readline`: no measurable difference (10-18 against 11-13 us a line, noise larger than the difference) | left |
| camerad, modeld, signd, envd | native already | L48's surface | not touched |

## 8. Files

| | |
|---|---|
| `chris-integrator/chris_integrator/native/` | `__init__.py` (selection, status), `__main__.py` (self-test), `src/chris_native.c` (the core) |
| `chris-integrator/native/` | `build.sh` (host), `cross-build.sh` (container); `dist/` (artefacts, not tracked) |
| `chris-integrator/setup.py`, `pyproject.toml` | the optional extension; the C source as package data |
| `chris-integrator/scripts/install.sh`, `scripts/sync-phone.sh` | the native step; the artefacts travel with the tree |
| `core/ekf.py`, `core/ego.py` | the three steps of a row and the prediction, each numpy (reference) and native; `near()`; `Entry.usedq` |
| `trackers/lanes.py` | `_fit`, `_update`, `_round2`, the measurement arrays: numpy and native |
| `trackers/view.py`, `io/recorder.py`, `mixer/bus.py`, `io/sockets.py`, `fastjson.py` | the walk, the reduced `road` line, `dumps` / `loads` |
| 25 modules | `from ..native import round` |
| `core/monitors.py`, `trackers/speedlimit.py`, `trackers/rules.py`, `core/registry.py`, `sources/camera_lead.py` | the Python savings of section 1 |
| `io/api.py`, `daemon.py` | `native` in `/v2/health`; the start-up log line |
| `docs/NATIVE.md`, `docs/INSTALL.md` | the contract, the build, the install step |
| `tests/test_native.py`, `tests/test_exact_savings.py`, `tools/native_bench.py` | |

## 9. Not verified

* Everything on the phone: that the module loads there and passes its self-test (it does in an emulated
  Alpine arm64 container with the same libc and a newer Python); the cost; the equivalence against the phone's
  numpy and OpenBLAS (the exact parts are tested bit for bit on emulated aarch64; the filter's differences on
  the phone may be other messages than on the laptop, of the same kind); memory; the live share.
* The install script's native step on the phone's busybox ash (tried under busybox sh on the laptop).
* A drive: the first one should read `cost_live.py`, as L46 asked.
* numpy versions other than 2.5.4 (x86-64) and 2.5.5 (aarch64, emulated) for the lane tracker's order of
  additions; another order shows as a failing test, not as wrong output beyond rounding.
* Free-threaded CPython and CPython 3.10: the module is not built for them (pure Python runs).

## 10. Branches

roadsense `v2-l50` and the car repository's `l50-native`: heads in the lane's hand-back. `v2-l49` had no new
commits when this was written (fae9098). Not pushed, not merged.
