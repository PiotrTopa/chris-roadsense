# L46: what chris-integrator costs on the phone, and what was taken out of it

Lane L46, 2026-11-20 07:35-11:00 CET, Poco F1 on the bench (no camera attached; frames from the replay
playlist), laptop for the equivalence runs. Branch `v2-l46` from `main` 6054373. Not pushed, not merged.
The owner's rule for this lane: the outputs stay the same; only work that changes nothing is removed.

## 0. Result

| | before (6054373) | after (`v2-l46`) |
|---|---|---|
| CPU per road frame, whole per-message path, quiet little core (replayed capture, 2.1) | 21.4 ms | **16.6 ms** (-22 %) |
| the same, median / 99th percentile / worst frame | 18.8 / 23.7 / 112 ms | 14.3 / 19.5 / 70 ms |
| a recorded drive with vehicle feed, receiver and map box, quiet little core (section 6) | 24.7 ms | 22.0 ms (-11 %) |
| live, full stack running, share of one little core (section 5) | 47.0 / 48.3 % | **38.7 / 39.3 %** |
| live, CPU per road frame | 31.1 / 32.1 ms | **25.5 / 26.3 ms** (-18 %) |
| latency added by the integrator (road -> lead / road -> lanes) | 15.5 / 19.2 ms | 9.5 / 12.7 ms |
| resident memory | 64-65 MB | 71-74 MB (two number tables, 3.3) |
| outputs | | identical line for line (section 4) |

Every stream a plug-in produces, the recorder's file, what the WebSocket subscriber receives and the socket's
greeting lines are byte-identical before and after, on three car recordings (one of them also with the map
box), and on the phone itself on a capture of its own input and on a recorded drive with the map box.

**Bad news first.** The reduction is a fifth, not a half. Nothing in the daemon was grossly wasteful except
the recorder (section 1); the rest is the unit price of Python on a little core: 1 us to read a JSON number
and 1.5 us to write one, 2.1 us for a `round()`, 5-18 us for a numpy call on a small array, and the stack
reads about 1500 numbers, writes about 2000, rounds about 1000 and makes several hundred numpy calls per
camera frame. **In the car it will cost more than on the bench**: with the vehicle's feed the estimator, which
this lane did not touch, is half of the cost; a recorded drive replayed on the phone gives 22 ms per frame
on a quiet core against 16.6 for the bench input, about 45-50 % of a little core live (section 6). System
time rose by 0.2-0.7 ms a frame and the cause is not pinned down (section 5). What is left, and what each
part would take, is section 6.

## 1. Where the 29 ms went (before)

Three measurements of the deployed 6054373, all on the phone:

**Live, 07:48-07:52 CET** (`tools/cost_live.py`, replay playlist, full stack, fan 0 %, governor level 0,
little cores at 1.77 GHz the whole time): 42.6 % of one core, 14.7 road frames/s, **28.9 ms CPU per road
frame** (26.4 user + 2.5 system), 64-65 MB, peak 87 C (cpu7). The mixer's own accounting gives the plug-ins
19.3 ms of that; it is wall time and includes what each plug-in's emits cost in the output channels.

**The per-message path off line** (`tools/cost_bench.py`, section 2.1): 200 s of the daemon's real input,
captured from the sockets, pushed through the same code without an event loop, on a quiet core: **21.4 ms per
road frame**. The stages:

| stage | ms / road frame | what |
|---|---|---|
| parse | 1.86 | `json.loads` of every input line; the 15.3 kB `road` line alone is 1.54 ms |
| estimator | 5.19 | 2 filter passes a frame (the frame's rows arrive behind newer entries), the smoother, the monitors, the 5 kB `ego_state` |
| lanes | 3.07 | 13 list -> array conversions with a finiteness test each, 3 `np.median`, per-line rounding |
| view | 2.07 | 153 linear interpolations a frame in Python, 200 `round()` |
| roadgeom, egolane, rules, ego_light, others | 1.95 | |
| **recorder** | **5.49** | `road`: the line was parsed again (1.5 ms), ~750 numbers rounded one by one (1.6 ms), written again as JSON (1.2 ms), gzip 0.3 ms; `view`, `ego_state`, `tracks` were serialised 15 times a second to be written twice a second |
| socket output | 0.73 | `ego_state`, `tracks`, `lead` serialised every frame for a greeting line no client was there to read |
| WebSocket output | 0.72 | serialising `lanes` (0.65) for the car backend; `view` and `lead` had been paid by the recorder and the socket |
| mixer dispatch | 0.77 | 11.5 envelopes and 13 plug-in calls a frame; freeing the previous `road` message |

**Live under a profiler** (cProfile on the unit for 3 min; times inflated by about 1.5): of the busy time,
81 % is the per-message path above, 4.5 % the WebSocket handler (one task and one `asyncio.wait` per message
were half of it), 3 % asyncio's readers, 1.3 % the 20-a-second vehicle tick, the rest the loop itself. No
health polling worth naming.

So the 29 ms were: 21.4 ms the per-message path as on a quiet core, about 2.5 ms kernel (15 kB lines in over
Unix sockets and out again over TCP to the backend, 4.7 WebSocket messages a frame), about 2 ms asyncio and
aiohttp, and the rest the price of sharing three little cores and their cache with envd and the camera
pipeline (the same code is slower while the stack runs).

The estimates made on the laptop were low for two reasons. The laptop replays fed recorder sessions, whose
`road` is the recorder's reduced 5 kB form, and they ran without the recorder, the socket and a WebSocket
subscriber: half of the phone's cost was never in them. And the ratio between this phone core and the laptop
is about 6.5 for this code, not the 3-4 assumed.

Facts about this core that decide what is worth doing (measured, `taskset -c 2`):

| operation | cost |
|---|---|
| `json.loads` of the 15.3 kB `road` line | 1.54 ms (1 us per number) |
| `json.dumps` of a list of 750 floats | 1.0-1.2 ms (1.5 us per float: the shortest-text search) |
| `round(x, 2)` | 2.1 us |
| numpy: a ufunc on 33 values 5 us, `np.asarray(list of 33)` 8.5 us, `np.round` 23 us, `isfinite().all()` 18 us, `np.median` 138 us |
| `np.array(list of 700)` 111-128 us, `.tolist()` of 700 100 us |
| zlib level 3: 20 MB/s (0.28 ms per recorded `road` line); level 1: 32 MB/s, +13 % size |
| garbage collector: no measurable share |

## 2. How it was measured

### 2.1 `chris-integrator/tools/cost_bench.py`

`cost_capture.py` connects to the roadsense daemons' sockets like the integrator does and writes every line
with its arrival time (and what the vehicle input store handed to the mixer, from the WebSocket).
`cost_bench.py run` builds the daemon object of a tree (stack, socket output, recorder into a scratch
directory, the API's output channel with one WebSocket subscriber asking for what the car backend asks for)
and pushes every captured line through what the daemon does with it, on the messages' clock, so two runs of
a tree give identical output. `--tree DIR` runs another checkout with the same harness; `--dump` writes every
produced message, the recorder's lines and digests of the socket's and the subscriber's lines; `diff`
compares two dumps.

The capture used throughout: 200 s from 07:58 CET, 9849 messages, 3035 road frames
(`<datasets>/roadsense-rec/l46/bench.cap.gz` on the laptop is incomplete: the phone's LTE uplink gave up at
2.8 of 15 MB; the whole file is on the phone, `<phone-home>/l46/bench.cap.gz`).

### 2.2 `chris-integrator/tools/cost_live.py`

CPU of the unit's process from `/proc`, memory, temperatures, the little cluster's frequency table, and the
mixer's and the health page's numbers at both ends of a window; no polling in between.

## 3. The changes, each with what it saves (phone, ms per road frame, 2.1)

| # | change | saves | where |
|---|---|---|---|
| 1 | A message is serialised only when something takes its line. The socket output keeps the envelope of its three greeting streams and makes the line when a client connects; the WebSocket output serialises a stream only if a subscriber asked for it (`?topics=`); the recorder applies its rate cap before it asks for the line | 0.7 (socket) + 0.5 (recorder: `view`, `ego_state`, `tracks`) | `daemon.py`, `io/sockets.py`, `io/api.py`, `io/http.py`, `io/recorder.py` |
| 2 | The recorder reduces `road` from the message the bus already holds instead of parsing the line again | 1.5 | `io/recorder.py` |
| 3 | The recorder's ~750 numbers are rounded in a few numpy calls with exactly `round()`'s results, and their JSON text comes from a table of the texts of k / 10^n instead of one float-to-text search each (3.1-3.3) | 0.8 (measured on first-touch data; 1.1 on warm data) | `fastround.py`, `io/recorder.py` |
| 4 | `road` is parsed without the members no box of the default stack reads: 16 of the plan's 19 arrays, the height of every lane line and road edge (6.3 of 15.3 kB). The line itself goes on whole to every consumer; `GET /v2/<topic>` serves the envelope's line. A wiring with boxes of its own gets the whole message (3.4) | 0.4 parse + 0.2 freeing | `fastjson.py`, `io/sockets.py`, `stack.py`, `io/http.py` |
| 5 | lanes: the twelve lists of a frame become one array with one finiteness test; `np.median` of a small array replaced by sort-and-pick (the same value); offsets and deviations of all lines rounded in one call | 1.0 | `trackers/lanes.py` |
| 6 | view: interpolation walks the table once per line instead of once per point (same comparisons, same arithmetic); the lines' 170 roundings in one exact batch | 0.4 | `trackers/view.py` |
| 7 | the mixer's rate window counts from the newest arrival and stops at the window's edge (it scanned 200 entries per call, 2.7 calls a frame); which output channels take a stream is worked out once | 0.1-0.2 | `mixer/mixer.py` |
| 8 | the WebSocket handler awaits its queue directly and drains what queued up meanwhile; no task and no `asyncio.wait` per message | live only: about half of the handler's 4.5 % | `io/http.py` |

Stage by stage after: parse 1.44, estimator 5.19, lanes 2.09, view 1.69, the other boxes 1.89, recorder 2.69
(`road` 2.19), socket 0.05, WebSocket output 1.30 (`lanes` 0.65, `view` 0.49, `lead` 0.14: it now pays for
what the recorder and the socket used to serialise first), mixer 0.55: **16.6 ms**.

### 3.1 Rounding with `round()`'s results (`fastround.round_flat`)

`round(x, n)` rounds correctly in decimal; `np.round` multiplies, rounds and divides in binary and differs
near ties. The batch computes k = rint(x * 10^n) for the whole list and repairs with `round()` itself every
element where the two could differ: within 1e-6 of a tie after scaling, beyond 1e9, not finite, and whole
numbers (an `int` stays an `int`: JSON `0` stays `0`). For every other element k is the integer the decimal
rounding picks and k / 10^n is the double nearest to it, which is what `round` returns. The daemons print
five significant digits, so real decimal ties are common (about 25 a frame) and the model's fixed distances
are integers (63 a frame): about 100 repairs a frame, each cheap or exact by construction.

### 3.2 The text of a rounded number from a table (`fastround.text_flat`)

A number rounded to n digits is k / 10^n and its JSON text depends on k alone. The texts for k = -32768 ..
32767 are built once per digit count (2 and 3: 4 MB each, at start-up) and fetched for the whole message in
one indexing step. Repaired one by one, with `repr` as `json.dumps` uses it: ties, whole numbers, values
outside the table, not finite, and negative numbers that round to zero (`-0.0`). The recorder's line is
`json.dumps` of the skeleton with a marker for each list, split at the markers and joined with the lists'
texts; a message in which a string equals the marker takes the plain path.

### 3.3 Memory

+7 to +9 MB resident: the two text tables (65536 strings each). 71-74 MB against 64-65 MB;
`MemoryMax=768M` is untouched.

### 3.4 Parsing `road` in part

Read by nothing in the default stack (checked by search and by the equivalence runs, which would differ
otherwise; `tests/test_road_trim.py` runs the default stack on modeld's own lines both ways): `plan.vx vy vz
ax ay az roll pitch yaw roll_rate pitch_rate yaw_rate x_std y_std z_std` (and `prob` when it is an array),
and `z` of every lane line and road edge. `meta` and the desire outputs are not read either, but finding
their end costs more than parsing them. `Stack.trim_road` is true only when the wiring adds or replaces no
box (`mixer.json` `plugins` / `replace`) and `"road_whole": true` is not set; the daemon then gives its socket
readers `parse_line_trimmed`. Replays and tools that call `stack.deliver` are not affected. What consumers
get: the WebSocket, `integrator.sock` and now also `GET /v2/road` pass the source's own line (before, the GET
re-serialised the parsed message: the same content with Python's number formatting); the recorder's reduced
`road` never contained these members.

## 4. Equivalence

"Identical" below means: for every stream produced by a plug-in (`ego_state`, `tracks`, `lead`, `lanes`,
`egolane`, `roadgeom`, `view`, `lights`, `signs`, `speed_limit`, `rules`, `fused`, `road_users`, `map` with the
map box) every message equal as serialised text, in order; every line of the recorder's file equal; the
SHA-256 of what the WebSocket subscriber received per topic equal; the socket's greeting lines equal.

| input | where | road frames | result |
|---|---|---|---|
| capture of the phone's own input (raw `road` lines: the only input that exercises 3.4) | phone, Python 3.14.8, numpy 2.5.4 | 3035 | IDENTICAL |
| 20261119-104128 (38 min, vehicle-fed, healthy) | laptop | 28306 | IDENTICAL |
| 20261119-113201 (15 min, vehicle-fed, healthy) | laptop | 9760 | IDENTICAL |
| 20261119-151146 (the incident drive) | laptop | 17720 | IDENTICAL |
| 20261119-113201 with the map box in process (`map` stream too) | laptop | 9760 | IDENTICAL |
| 20261119-113201 with the map box (vehicle feed, receiver, map: the phone's Python and numpy on those paths) | phone | 9745 | IDENTICAL |

Two runs of the baseline tree on the phone give the same dump (SHA-256 `8ba4a2e0...9e69`), so the comparison
is not passing by accident of a noisy reference.

Unit tests added (346 pass in chris-integrator; api 22, chris-mapper 22):

* `tests/test_fastround.py`: `round_flat` against `round()` on 300 random lists, 300000 values on and next to
  decimal ties, integers, zeros, both signs of zero, extremes, not-finite values; `text_parts` against
  `json.dumps` of the rounded lists on the same kinds of input and on every entry of both tables.
* `tests/test_recorder.py`: the table-made `road` line equals `json.dumps(slim_road(...))` on 200 messages,
  including one whose string equals the marker.
* `tests/test_interp_walk.py`: the single-walk interpolation equals the per-point one on 20000 random tables
  and query lists, sorted or not, with NaN.
* `tests/test_road_trim.py`: only the listed members go; other layouts are left alone; the default stack's
  outputs are the same from trimmed and whole messages; a stack with its own boxes gets the whole message.
* the small exact median against `np.median` on 20000 random arrays (run once, not kept as a test).

Commands:

```sh
cd ~/projects/wt/roadsense-l46; PY=~/projects/roadsense/.venv-models/bin/python; O=/tmp/l46; D=<datasets>/roadsense-rec
export PYTHONHASHSEED=0; PACK=~/roadsense-maps/packs/poland-20261114-f2.rsmap
(cd chris-integrator && CHRIS_TEST_CAR_REPO=~/projects/cyberpank-prius-gen2-computer PYTHONPATH=/usr/lib/python3/dist-packages $PY -m pytest -q -p no:cacheprovider tests)
for s in 104128 113201 151146; do      # inside systemd-run --user -p MemoryMax=12G
  $PY chris-integrator/tools/cost_bench.py run $D/20261119-$s --tree ~/projects/roadsense --dump $O/${s}_base.jsonl.gz
  $PY chris-integrator/tools/cost_bench.py run $D/20261119-$s --dump $O/${s}_new.jsonl.gz
  $PY chris-integrator/tools/cost_bench.py diff $O/${s}_base.jsonl.gz $O/${s}_new.jsonl.gz; done
# with the map box: add  --pack $PACK  to both runs
# on the phone (as user; base/ = git archive 6054373, final/ = git archive of the branch head):
cd ~/l46; taskset -c 1 python3 final/chris-integrator/tools/cost_bench.py run bench.cap.gz --tree base  --dump base-dump.jsonl.gz
          taskset -c 1 python3 final/chris-integrator/tools/cost_bench.py run bench.cap.gz --tree final --dump final-dump.jsonl.gz
          python3 final/chris-integrator/tools/cost_bench.py diff base-dump.jsonl.gz final-dump.jsonl.gz
# cost: the same without --dump, with --json OUT; a capture: sudo taskset -c 4-7 nice python3 cost_capture.py OUT.cap.gz --seconds 200 --url http://127.0.0.1:8090
# live: python3 cost_live.py --seconds 240
```

## 5. Cost after, on the phone

Live, the full stack on the replay playlist, each run started fresh so that every run sees the same footage
(`krakow-2024` from 198 s), 200 s of warm-up, a 240 s window, the two versions alternating (09:45-10:12 CET;
fan 0 %, little cores at 1.77 GHz throughout, governor level 0 except run 1 at 3):

| run | version | share of one core | road frames/s | CPU ms / road frame (user + system) | latency road / lead / lanes, ms | peak | memory |
|---|---|---|---|---|---|---|---|
| 1 | 6054373 | 47.0 % | 15.1 | 31.1 (28.5 + 2.6) | 76.9 / 91.5 / 94.9 | 87.7 C | 64-67 MB |
| 2 | v2-l46 | **38.7 %** | 15.2 | **25.5** (22.2 + 3.3) | 68.6 / 77.5 / 80.1 | 85.0 C | 71-74 MB |
| 3 | 6054373 | 48.3 % | 15.1 | 32.1 (29.2 + 2.9) | 91.5 / 107.8 / 111.9 | 86.7 C | 63-67 MB |
| 4 | v2-l46 | **39.3 %** | 14.9 | **26.3** (23.2 + 3.2) | 92.7 / 102.7 / 106.5 | 87.7 C | 71-74 MB |

* **CPU: 47.7 -> 39.0 % of one little core, 31.6 -> 25.9 ms per road frame (-5.7 ms, -18 %).** The 4.8 ms
  saved on a quiet core are worth more beside a running stack.
* **Latency added by the integrator** (the topic's latency minus `road`'s): `lead` 14.6 and 16.3 -> 8.9 and
  10.0 ms; `lanes` 18.0 and 20.4 -> 11.5 and 13.8 ms. About 6 ms less on both measurement topics. (`road`'s
  own latency is the camera pipeline's and moved between the pairs with the phone's temperature.)
* **Worst single call since start** (the mixer's `max_ms`): estimator 52-59 -> 62-63 ms (the first frames
  after start; unchanged), lanes 48 -> 15 ms, view 41 -> 41 ms. Off line the worst whole frame went from
  112 to 70 ms, the 99th percentile from 23.7 to 19.5 ms.
* **System time rose** by 0.3-0.7 ms a frame (2.6-2.9 -> 3.2-3.3), inside the totals above. It shows off line too (0.18 -> 0.42 ms a frame in the bench), so it
  is the new code and not the sockets: the likely cause is memory, the batches allocate arrays, lists and
  byte strings of a few kB where the old code made many small ones, and musl gives blocks of that size back
  to the kernel. Keeping the rounding helpers' work arrays between calls did not change it (0.39 ms; taken
  out again, commits 1c6cc16 and 9a9ba27), so the source is elsewhere (the trimmed line's pieces, the lanes
  batch) and was not chased further: 0.2 ms.
* **Memory** +7 MB (3.3). **Temperatures** 85-88 C peak (cpu7) in all four runs: the integrator's 9 % of a
  little core does not show against signd and envd on the big cores. 0 plug-in errors in every run.
* The first measurement of this lane (07:48 CET, 42.6 %, 28.9 ms) and the first after the deployment (09:30
  CET, 40.1 %, 26.6 ms) were taken at other places of the playlist and are not comparable with each other;
  the table above is.

**The budget reached: 39 % of one little core at 15 road frames a second, 26 ms per frame live, 16.6 ms on a
quiet core.** What limits it is section 6.

## 6. What remains expensive, and why

**First, what the bench does not show: the car.** The bench has no vehicle feed, no receiver and no map
position. A recorded drive with all three (20261119-113201, 15 min, 9745 road frames, 26544 input messages,
the map box in process) replayed through the same harness **on the phone**:

| | before | after |
|---|---|---|
| CPU per road frame, quiet little core | 24.7 ms | 22.0 ms (-11 %) |
| estimator | 11.7 ms (5.0 ms a call, 2.3 calls a frame) | 11.5 ms |
| lanes / view / rules | 2.6 / 0.9 / 2.2 ms | 1.8 / 0.8 / 2.2 ms |
| recorder / socket / WebSocket output | 3.8 / 1.1 / 0.3 ms | 2.8 / 0.05 / 0.7 ms |
| median / 99th percentile / worst frame | 21.1 / 30.1 / 53 ms | 18.3 / 27.6 / 52 ms |
| filter entries re-run because of late rows | 25074 (2.6 a frame) | the same |

(The input is a recorder session: its `road` is already the reduced form, so reading it costs 1.0 ms instead
of the 1.4-1.9 of a live line, and the recording holds `ego` at no more than 25 a second.) With the live
overhead measured in section 5 on top, the integrator in the car is to be expected at about **45-50 % of a
little core** after this lane (50-55 % before), not the 39 % of the bench: **with the car's feed the estimator
is half of the cost**, and this lane did not touch it. Every wheel and gyro sample is a filter insertion with a
prediction and its updates, and every camera frame, stamped 100 ms earlier than it arrives, makes the filter
run again over the two or three samples that arrived in between. This is an estimate from a replay, not a
measurement in the car: the first drive should read `cost_live.py`.

In order of size (bench input, ms per road frame of 16.6):

* **The estimator, 5.2 ms, untouched.** Two filter passes per camera frame (the frame's measurements are
  stamped earlier than entries that already arrived, so the filter is run again from there: the exact answer
  to late data, as designed), a prediction of a 26-state filter each (about 25 numpy calls), four scalar
  updates (about 10 numpy calls each), the fixed-lag smoother (a linear solve), the monitors, and the
  `ego_state` message. No part is more than a quarter of it and none is waste. With numpy call overhead of
  5-18 us on this core, a 26-state filter in Python cannot be made much cheaper without changing its
  arithmetic (results then differ at rounding level and would have to be shown after the trackers) or a
  native core. With the car feeding wheel and gyro samples it is 11.4 ms (above).
* **JSON, 4.2 ms** in four places: reading the inputs 1.4; the recorder's `road` line 1.9 and its other topics
  0.5; `lanes`, `view` and `lead` for the WebSocket consumer 1.3. The table technique of 3.2 would take about
  0.6 off `lanes` and `view` (their lists are already rounded); it was not done: it needs each tracker to hand
  the mixer its rounded arrays, a wider change than this lane's. Reading cannot be made cheaper in pure Python.
* **lanes 2.1, view 1.7, roadgeom 0.7, egolane 0.4**: numpy on arrays of 33 and Python loops over 14 points,
  already batched where a batch gives the same numbers.
* **The recorder's gzip, 0.3**: level 1 instead of 3 would save 0.1 for 13 % larger files (not done: it
  shortens what the 4 GB cap holds).
* **Kernel and event loop, about 4-5 ms live**: 15 kB in and 15 kB out per frame, 4.7 WebSocket messages a
  frame to the backend. A consumer that does not subscribe to `road` would save its 15 kB send; that is the
  backend's choice.

Three larger steps exist and none was taken, because each needs a decision of the owner:

1. **A native core for the filter or for JSON.** Not available as the phone is installed: there are no Python
   headers (`python3-dev` is not installed, and no `apk add`), so an extension module cannot be built there.
   A plain C library loaded with `ctypes` could be (gcc is there, and roadsense itself is compiled on the
   phone by `scripts/install-phone.sh`), but chris-integrator's install is "POSIX sh, no compiler" on purpose
   and its results would differ from numpy's at rounding level (BLAS sums in another order). Not relied on.
2. **Recording `road` as it arrives** (the source's own line, no rounding, no second serialisation) would save
   about 1.7 ms a frame and triple the size of the largest topic in the recording.
3. **The 20-a-second vehicle tick** costs about 1 % of the daemon; it could wake only when a held-back change
   is due.

## 7. On the phone now, and how to undo it

**Deployed** (the owner authorised bench deployments for this night): the `chris_integrator` package of this
branch in `/usr/local/lib/chris/chris_integrator`, put there at 09:27 CET with the stack stopped (the way
`scripts/install.sh` swaps a package; no unit, no configuration and no other package was touched, and the
install script was not run on the live system: it would re-create the unit's `WantedBy` link that the car's
provisioning removes).

* Content: the package of branch head (code last changed in d0885a8; 9a9ba27 has the same package). Check:
  `cd /usr/local/lib/chris && find chris_integrator -name '*.py' | LC_ALL=C sort | xargs sha256sum | sha256sum`
  = `be75eec7dc44075983a1679474d6ac7d629f02656b79bb21fd46a1020678e3f1` (the same command in
  `chris-integrator/` of this branch gives the same). Marker `/usr/local/lib/chris/.deployed-commit-chris_integrator`.
  The 6054373 package and `chris_mapper` together hashed `4bd790b7...0a00` before (plain `sort`, both packages).
* The phone's source tree `<phone-home>/d1/roadsense` got this branch's `chris-integrator/`, `scripts/install-phone.sh`,
  the two docs and the api unit file (comments); `.deployed-commit` says 9a9ba27.
* Checked live after the deployment: the car backend receives all its topics at the camera's rate; a
  WebSocket client gets exactly the topics it asks for (and everything without `?topics=`); `integrator.sock`
  greets with the last `ego_state`, `tracks`, `lead` and narrows on `{"subscribe": [...]}`; `GET /v2/road`
  returns modeld's whole line (22 plan members, the lines' `z`, `meta`); the recorder's session reads back
  with the reduced `road` (plan `x y z held age_ms`); the daemon stops cleanly on SIGTERM and saves its state.
* **Undo** (as root; also in `<phone-home>/rollback-20261120-l46/RESTORE.txt`): stack off through the backend
  (`roadsense_mode off`), `mv /usr/local/lib/chris/chris_integrator{,.l46}`, `tar xzf
  <phone-home>/rollback-20261120-l46/chris-lib-6054373.tar.gz -C / usr/local/lib/chris/chris_integrator`,
  `roadsense_mode auto`. The folder (root 0700) also holds the replaced source-tree files
  (`d1-roadsense-replaced-files-6054373.tgz`), the state files as found and the bench's recorder sessions.
* CMDB: the phone's entry updated in place (package, hashes, cost, rollback, install variables),
  `dotinfra lint` clean, synced; event 8.

**The state the phone was left in (10:59 CET):** as found. `roadsense_mode auto`, the stack running on the
DVR camera source with no camera attached (the backend's five-minute restart loop of the bench goes on as
before this lane, one small recorder session per start). Replay off (`roadsense-camerad-replay` disabled,
`roadsense-camerad-dvr` enabled, `calib.json` back). The learned state restored from the copies taken at
07:39 and verified by checksum: `/var/lib/chris/integrator.json`, `/var/lib/chris/signs.jsonl`,
`/var/lib/roadsense/camera-calibration.json`, `signs.jsonl`, `vehicle-calibration.json`, `/etc/roadsense/calib.json`.
The recordings folder holds the 82 sessions it held at 07:39; the seven sessions of this lane's replay runs
(333 MB) are in the rollback folder under `rec-bench-l46/`. No backend setting was changed (the replay
source keeps the camera daemon connected, so the bench restart setting was not needed); no reboot; no
package installed. Highest temperature of the lane 88.3 C (cpu7; a guard that stops the stack at 95 C ran
during the long comparison). Battery gauge 74 % at both ends. The lane's working files are in `<phone-home>/l46`
(42 MB: the capture, the car slice, both trees, result files, scripts) and can be deleted.

Seen and left alone: now and then the daemon logs `Task exception was never retrieved ... KeyboardInterrupt`
when it is stopped (both versions; the state file is written all the same). The 82 sessions in the
recordings folder include about 40 empty ones of the no-camera restart loop since 04:19.

## 8. The install scripts' variable (also in this lane)

`scripts/install-phone.sh` and `chris-integrator/scripts/install.sh` both read `API_PORT` / `API_HOST`, the
first for roadsense's debug face and the second for the consolidated API, and the first runs the second: the
documented `API_PORT=8091 scripts/install-phone.sh` made the second refuse with "the camera's debug face and
the consolidated API collide".

* `scripts/install-phone.sh`: the debug face is `DEBUG_API_PORT` / `DEBUG_API_HOST`. `API_PORT` / `API_HOST` are
  still read there with the same meaning (the commands in the car repo's notes keep working) and are taken
  out of the environment before chris-integrator's script runs.
* `chris-integrator/scripts/install.sh`: the consolidated API is `CHRIS_API_PORT` / `CHRIS_API_HOST` (the names
  of the keys it writes); `API_PORT` / `API_HOST` are read when those are not set, for a stand-alone run.
* Both stay re-runnable. Tested in scratch prefixes under busybox sh and dash, twice each (second run changes
  nothing): no variable -> 8091 / 8090; `API_PORT=8091` -> 8091 / 8090 (failed before);
  `DEBUG_API_PORT=8093 CHRIS_API_PORT=8094` -> 8093 / 8094 with the integrator's link following the debug
  face; `API_PORT=8090` -> refused before anything is written (a real collision with the default 8090);
  chris-integrator's script alone with the old names and with the new name taking precedence.
  The same on the phone under its busybox ash, from the updated source tree, into scratch prefixes that were
  removed afterwards: the same four results.
* The daemon's built-in default port stays 8080 on all interfaces: it is what a stand-alone start by hand on
  a development machine or a single-purpose box expects, and every installed unit sets 8090 on loopback
  through its environment, so the default is never used on the car. Documented in
  `chris-integrator/docs/INSTALL.md`, the README and the option's help text.

## 9. Shared ground with L47

Not touched: `chris_integrator/sources/*`, `plugins/mapper.py`, `plugins/estimator.py`, `core/*`,
`chris-mapper/`. Touched in chris-integrator: `daemon.py`, `fastjson.py` (appended), `fastround.py` (new),
`stack.py` (`trim_road`), `mixer/mixer.py` (`publish`, `fps`), `io/{api,http,recorder,sockets}.py`,
`trackers/lanes.py` (`_measure`, `_line`, `_message`, `_median`), `trackers/view.py` (`_interp_all`, the end
of `_lines`), `scripts/install.sh`, `docs/INSTALL.md`, `README.md`, tests and tools. If L47 changes the estimator's rows or the
map-match rows, the equivalence runs of section 4 are to be repeated against L47's tree as the baseline
(`--tree`): the comparison is between two trees, whatever their numbers are.

## 10. Branch

`v2-l46`, head: see the lane's hand-back. Not pushed, not merged.
