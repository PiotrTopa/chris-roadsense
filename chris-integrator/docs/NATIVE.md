# The native core

chris-integrator is pure Python + numpy, and stays complete that way. On a slow core (the reference
deployment runs it on a phone's little core) the price of Python is the budget, so the arithmetic and the
text work of the per-message hot path also exist in C: one file, `chris_integrator/native/src/chris_native.c`,
compiled to one module, `chris_integrator/native/_chris_native.abi3.so`. Where that module is present and loads,
it is used; where it is not, the same functions run in Python. Nothing else in the package knows which.

    python -m chris_integrator.native      which implementation runs here + a self-test against the reference
    GET /v2/health  ->  "native": {"active": true, "implementation": "native", "parts": [...], "version": 2}
    the daemon's first log lines: "hot path NATIVE (json, round, interp, ekf, lanes)"  |  "hot path pure Python: <why>"

## What is native, and what "the same" means for each part

| part | native function | reference (pure Python) | results |
|---|---|---|---|
| `json` | `dumps(obj)` | `json.dumps(obj, separators=(",", ":"))` | the same text, byte for byte |
| `json` | `loads(bytes or str)` | `json.loads` | the same objects (types, values, sign of zero, key order) |
| `json` | `dumps` of `Rounded((list, n))` | `json.dumps([round(v, n) for v in list])` | the same text (the recorder's reduced `road`) |
| `round` | `round(x, n)`, `round_list(list, n)` | `builtins.round` | the same value and type |
| `interp` | `interp_all(xs, ys, qs)` | `trackers.view._interp_all_py` | the same values |
| `lanes` | `lanes_measure`, `lanes_predict`, `lanes_fit`, `lanes_apply`, `round_lists` (the lane tracker's arrays) | numpy expressions in `trackers/lanes.py` | the same bits |
| `ekf` | `ekf_project`, `ekf_update`, `ekf_sym` (one scalar row of the filter) | numpy expressions in `core/ekf.py` | the same bits for rows of one state index (most rows); to rounding for rows over several states |
| `ekf` | `ekf_predict` (the process model) | `EgoModel._predict_py` | state and transition: the same bits; covariance: to rounding |
| `ekf` | `ekf_rts` (one smoother step) | `np.linalg.solve` + products in `core/ekf.py` | to rounding |

The `lanes` part is exact because the tracker's arithmetic is elementwise (numpy computes those with plain
IEEE operations, one element at a time), and its two means over the gate points are added in the order numpy
adds them there (in sequence: the masked selection comes out in column order). That order is numpy's
implementation, not its contract: `tests/test_native.py` holds it against the installed numpy.

"To rounding": numpy hands sums of products (`@`, `solve`) to its BLAS, which adds them in an order of its own
(and with fused multiply-adds on ARM); the native code adds them in index order with plain IEEE arithmetic.
Both are correct to the last bit or two of a double; they are not the same bits. The filter's results
therefore differ between the two implementations at the 1e-13 level (and between two machines' numpy the same
way: BLAS kernels differ per CPU). After the monitors and trackers, on recorded drives, that shows as a handful
of messages per drive in which a number that rounds to zero carries the other sign (`-0.0` / `0.0`) and, rarely,
a last printed digit; lane report L50 has the counts. Everything else in the table is exact, by construction
and by test.

A native function never guesses. Where it is not certain to give the reference's result for an input (a type
it does not know, a number outside its exact path, damaged text) it raises `native.Fallback` (or calls the
reference itself) and the reference runs for that one call. So errors, corner cases and exotic inputs are
CPython's own: `loads` of bad JSON raises `json.JSONDecodeError` from `json.loads`.

### Why the exact parts are exact

* **Writing a float.** `repr(x)` is the shortest decimal string that reads back as `x`. If `x` equals
  `k / 10^n` for an integer `k` below 10^15 (tested by one exact division), the string of `k` with `n` decimals
  is that shortest string: decimals of 15 significant digits or fewer and doubles correspond one to one. Every
  number the stack rounds before it writes is of this kind. Everything else (17 digits, exponent forms) goes
  through CPython's own `PyOS_double_to_string`.
* **Reading a float.** A decimal with a mantissa up to 2^53 and at most 22 decimals is one exact division of
  two exact doubles, correctly rounded: what `strtod` returns. Everything else goes through
  `PyOS_string_to_double`.
* **`round(x, n)`.** `round` rounds correctly in decimal. `rint(x * 10^n) / 10^n` gives the same double except
  next to a decimal tie; within 1e-6 of a tie (and beyond 1e9, and for anything that is not a plain float)
  `builtins.round` itself is called.

### Memory safety of the parser

`loads` reads external input (the daemons' lines, the vehicle feed). It is a recursive-descent reader over a
(pointer, end) pair: every read is checked against `end`, nesting is limited to 64 levels (deeper input goes
to `json.loads`), strings are validated as UTF-8 by CPython's decoder, numbers longer than 63 characters or
19 digits go to the reference. `tests/test_native.py::test_loads_survives_damaged_input` feeds it 40000 cut
and corrupted lines and requires the reference's result or an error; build with `SANITIZE=1` to run the suite
under the address and undefined-behaviour sanitizers:

    SANITIZE=1 OUT=/tmp/asan sh native/build.sh
    cp /tmp/asan/_chris_native.abi3.so chris_integrator/native/
    LD_PRELOAD=$(cc -print-file-name=libasan.so) ASAN_OPTIONS=detect_leaks=0 python -m pytest tests/test_native.py

## Selection

`chris_integrator/native/__init__.py` decides at import:

* the module loads and has the expected version -> native, every part;
* `CHRIS_NATIVE=0` in the environment -> pure Python (the reference; the comparison baseline);
* `CHRIS_NATIVE=json,round,interp,lanes` -> only those parts native (the exact ones: a run with them must give
  the reference's output line for line; adding `ekf` adds the filter's rounding-level differences);
* the module is absent or does not load -> pure Python, and `status()["reason"]` says why.

The unit's environment can carry the switch (`/etc/chris/integrator.env`: `CHRIS_NATIVE=0`) for a rollback
without touching files.

## Building

The module uses the CPython limited API (`abi3`): one binary loads in every CPython from 3.11 on, for the
machine type and C library it was built for. It needs a C compiler and the Python headers to build, nothing
to run but libm. No numpy headers: arrays pass through the buffer protocol.

    sh native/build.sh                        this machine -> chris_integrator/native/_chris_native.abi3.so
    pip install .                             builds it when a compiler is there; installs pure Python when not
    CHRIS_NATIVE_BUILD=0 pip install .        pure Python on purpose
    sh native/cross-build.sh                  another platform, in a container (default: linux/arm64, Alpine: musl)

Flags that matter: `-ffp-contract=off` (no fused multiply-add: it would change bits that are otherwise
numpy's), no `-ffast-math`, no `-march`. The code is plain C99.

### A target that must not be touched (the reference phone)

The phone's operating system is pinned and has no Python headers. Nothing is installed on it:

    sh native/cross-build.sh
    # -> native/dist/linux-aarch64-musl/_chris_native.abi3.so  and BUILD.txt (compiler, image, checksums, test result)

builds in an `alpine:edge` arm64 container on the development machine (docker with qemu-user binfmt), runs
the self-test and `tests/test_native.py` there against the container's numpy, and leaves the artefact in
`native/dist/`. The directory is not tracked by git; it travels with the source tree that is copied to the
target. `scripts/install.sh` then:

1. copies the package;
2. if `native/dist/linux-<machine>-<libc>/_chris_native.abi3.so` exists for the target (or `NATIVE=/path` names
   a file, or the tree holds a module built in place), puts it into the installed package;
3. runs `python3 -m chris_integrator.native` with the target's interpreter: the module must load and give the
   reference's results on that machine (JSON text, rounding, parsing, the filter steps against numpy);
4. keeps the module if that passes; removes it and says so if not. The log line is one of
   `native core: IN USE (linux-aarch64-musl, sha256 ..., self-test ok)`,
   `native core: NOT used, the compiled module ... failed here and was removed: pure Python`,
   `native core: no compiled module for linux-aarch64-musl at hand ...: pure Python`,
   `native core: not installed (NATIVE=0): pure Python`.

The script never compiles. Undo on a running system: `CHRIS_NATIVE=0` in `integrator.env`, or delete the
`.so`, and restart the unit.

### Wheels

`pip wheel .` gives `chris_integrator-<v>-cp311-abi3-<platform>.whl` with the module, or a pure wheel with
`CHRIS_NATIVE_BUILD=0`. For a release: cibuildwheel with `CIBW_BUILD=cp311-*` (abi3 covers later versions),
manylinux and musllinux, x86_64 and aarch64; plus the sdist, from which `pip install` builds or falls back.

## Tests

    python -m pytest tests                          the whole suite on what is installed (native if built)
    CHRIS_NATIVE=0 python -m pytest tests           the whole suite on the reference
    python -m pytest tests/test_native.py           native against reference, function by function, and a
                                                    simulated drive through both (two processes)
    tools/cost_bench.py run REC --dump A.gz         (CHRIS_NATIVE=0 ... --dump B.gz;  cost_bench.py diff A.gz B.gz)
                                                    every message of every stream of a recorded drive, both ways

Without the compiled module the native comparisons are skipped and everything else runs: the package passes
its tests with no compiler.

## A native plug-in

The mixer's plug-in contract (`mixer/plugin.py`, DESIGN.md 14.3) is about streams, not about languages:
a box declares `name`, `inputs`, `outputs`, `DEFAULT_CFG` and answers `on_input(stream, msg, ctx)`. Three
ways to put compiled code behind it, in order of cost:

1. **A Python box with a native kernel** (what the estimator and the display box do): the box stays the
   declared, replaceable unit in Python; its arithmetic calls functions that exist twice, reference and
   native, selected in `chris_integrator.native`. The contract and the wiring do not change; a replay is
   deterministic; the reference stays testable.
2. **A box written as an extension type** (C, C++, Rust with PyO3, Cython): any importable class with the
   attributes above and `on_input` is a plug-in; `mixer.json` names it (`{"plugins": [{"class":
   "package.module:Class"}]}`, see `stack.py`). Messages are the bus' dicts, so such a box pays the
   conversion at its edges and nothing else.
3. **A box in another process**, in any language: `mixer.remote.RemotePlugin` speaks the same declarations
   as JSON lines over a pipe or a Unix socket (`{"type":"plugin","contract":1,...}` handshake). The hop costs
   a serialisation each way (`tools/hop_cost.py`); for boxes that are not tied to a camera frame
   (`sync=False`).

A box that keeps its own reference implementation follows the rule of this package: the reference decides
what is right, the compiled code is an accelerated copy with the same tests.

## Licences

The native core is original code under the package's MIT licence. Nothing is vendored; it links libm and
the CPython stable ABI only.
