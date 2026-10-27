# Architecture and inter-process contract

Four core processes (camerad, modeld, envd, api) plus any number of external
consumers such as a sign provider, all talking through two simple mechanisms: a
shared-memory frame ring (camerad -> consumers) and newline-delimited JSON over
Unix stream sockets (every daemon -> api / tools). These two mechanisms are the
whole contract: a process that writes the ring as described below is a valid
camerad (any camera, any device), and a process that reads the ring and serves
`<rundir>/signd.sock` is a valid sign provider. Nothing else couples the
processes; they share no libraries beyond the header-only `camerad/ring.h` and
`camerad/jsonsock.h`, which exist for convenience. One optional addition, the GPU
schedule (`<rundir>/gpu.sched`, below), lets a sign provider that runs on the same GPU
as modeld take turns with it; every process still works alone without it.
No message broker, no protobuf, no capnp. `nc -U /run/roadsense/modeld.sock`
must always be a valid debugging tool.

```
camerad ──frame ring (shm)──┬──> modeld ─────────────────JSON──┐
                            ├──> envd (1 fps) ─────────────JSON──┤
                            └──> sign provider (external) ─JSON──┼──> api ──> HTTP / WebSocket / MJPEG
camerad ──JSON (frame meta)──────────────────────────────────────┘
```

## Frame ring (`/dev/shm/roadsense-frames`)

A single file created by camerad, mmapped by everyone else. Little-endian,
all offsets in bytes, header is 4096 bytes. camerad builds a fresh file under a
temporary name and `rename()`s it over the path at start-up (never truncates a
mapped file, so old readers cannot SIGBUS); readers detect the new inode and
remap (`RingReader` in `camerad/ring.h` does this). Reference implementation of
everything in this section: `camerad/ring.h` (header-only, `RingWriter` /
`RingReader`). Tests may override the path with env `ROADSENSE_RING`.

Pixel data: NV12, **BT.601 full range** (JFIF: Y 0..255, Cb/Cr 0..255 centred
on 128; `Y = 0.299R + 0.587G + 0.114B`, `Cb = -0.168736R - 0.331264G + 0.5B + 128`,
`Cr = 0.5R - 0.418688G - 0.081312B + 128`), sRGB-gamma values as the software
ISP delivers them, chroma = box average of each 2x2 block.

```
struct RingHeader {            // offset 0
  char     magic[8];           // "RSFRAME1"
  uint32_t header_size;        // 4096
  uint32_t slot_count;         // 6
  uint32_t slot_size;          // bytes per slot incl. SlotHeader, 4096-aligned
  uint32_t width, height;      // 1920, 1080 (native sensor size; camerad-dvr: 1280, 720)
  uint32_t y_stride;           // bytes per luma row (1920)
  uint32_t uv_stride;          // bytes per chroma row (1920, NV12 interleaved)
  uint32_t format;             // 1 = NV12 (Y plane then interleaved UV plane, 4:2:0)
  float    focal_px;           // horizontal focal length in pixels at width x height (camerad config)
  float    cx, cy;             // principal point (default width/2, height/2)
  uint32_t reserved0;          // offset 52: natural-alignment padding, 0
  uint64_t write_seq;          // offset 56: last fully written frame number (atomic store, release); 0 = none yet
  uint64_t camerad_pid;        // offset 64: liveness
}                              // 72 bytes used, rest of the 4096 zero
struct SlotHeader {            // at header_size + i*slot_size, i = seq % slot_count
  uint64_t seq;                // frame number, starts at 1
  int64_t  ts_mono_ns;         // CLOCK_MONOTONIC at sensor timestamp (libcamera SensorTimestamp)
  int64_t  ts_real_ns;         // CLOCK_REALTIME at the same instant
  uint32_t exposure_us;
  float    analogue_gain;
  float    colour_gain_r, colour_gain_b;
  uint32_t data_offset;        // from slot start, 4096
  uint32_t data_size;          // y_stride*height + uv_stride*height/2
}                              // 48 bytes; pixels at slot start + data_offset
```

With 1920x1080: `slot_size` = 3117056, file size 4096 + 6*3117056 = 18706432.

Writer protocol: pick slot `seq % slot_count`, store the new `seq` into the
SlotHeader **first** (followed by a release fence), then write the rest of the
SlotHeader and the pixel data, then store `write_seq = seq` with release
semantics. Writing the slot seq before the pixels is what lets a reader detect
an overwrite that started while it was copying.
Reader protocol: read `write_seq` (acquire), check the slot's `seq` equals it
(else it was already lapped), copy or process the slot, acquire fence, then
re-read that slot's `seq`; if it changed the slot was overwritten mid-read,
drop the frame. Readers poll `write_seq` with a 1-2 ms sleep; there is no
notification channel (30 fps makes polling cheap).

camerad also writes a JPEG of every Nth frame to `/run/roadsense/preview.jpg`
(atomic rename) for the MJPEG endpoint, N configurable (default 3): 960x540
(2x box downscale of the NV12 frame), baseline 4:2:0, quality 80, ~110 KB.
If the encoder is still busy with the previous preview the frame is skipped.

Run directory: `/run/roadsense`, overridden by env `ROADSENSE_RUNDIR` for
every daemon (`/tmp/roadsense` for development).

**/dev/shm and logind:** systemd-logind's default `RemoveIPC=yes` deletes a
regular (uid >= 1000) user's files in `/dev/shm` when that user's last session
ends (observed on the phone: the ring vanished after the ssh session closed).
The systemd units must therefore run camerad as a system user (uid < 1000) or
root, or logind needs `RemoveIPC=no`. During development, run camerad and its
readers inside the same login session.

### camerad configuration (`/etc/roadsense/camera.json`, env `ROADSENSE_CONFIG_DIR`)

Flat JSON object of scalars, every key optional:

| key | default | meaning |
|---|---|---|
| `focal_px` | 1470 | ring header focal length (4.3 mm lens / 5.64 mm sensor width * 1920 px) |
| `cx`, `cy` | 960, 540 | principal point |
| `fps` | 20 | sensor frame rate set via the IMX363 `vertical_blanking` control; 0 = leave the sensor alone (~30.3 fps). CLI `--fps N` overrides |
| `preview_every` | 3 | JPEG every Nth frame, 0 = off (CLI `--no-preview`) |
| `preview_quality` | 80 | JPEG quality |
| `little_cores` | true | pin the whole process (incl. libcamera threads) to cpu0-3 |
| `convert_threads` | 1 | threads for XRGB->NV12 (1 is enough: ~12 ms per frame on an A55) |
| `sensor_line_ns`, `sensor_base_lines` | 10468, 3045 | empirical frame-timing model `T = (base_lines + vblank) * line_ns`, see HARDWARE.md |

CLI: `camerad [--verbose] [--frames N] [--no-preview] [--fps N] [--vblank N]`
(`--verbose` prints fps and timings to stderr once per second; `--vblank` writes
a raw VBLANK for calibration). SIGINT/SIGTERM stop the camera, close clients and
unlink the socket.

## Alternative camera sources

Any process that writes the ring and serves `camerad.sock` per this document is
a camerad; modeld, the sign provider and the api read the geometry and
intrinsics from the ring header and do not care which one runs. Run exactly one
(the systemd units `Conflicts=` each other).

### camerad-dvr: USB "DVR" head-unit camera (Huiying/Buildwin, USB 1902:3241)

A cheap car head-unit camera that enumerates only as a USB mass-storage disk
(kernel name "Buildwin Media-Player"). Its firmware serves the live stream from
fixed sectors inside the file `VSFILE` of its FAT volume: an info sector (frame
size, frame sequence, check word) and the current frame, a 1280x720 baseline
JPEG (4:2:0, restart interval one MCU row, ~45-70 KB) at ~19.5 fps. No commands
are needed. Protocol: [HUIYING_DVR_PROTOCOL.md](HUIYING_DVR_PROTOCOL.md);
probe: `tools/dvr_probe.py`.

`camerad-dvr (--auto | --dev /dev/sdX) [--verbose] [--frames N] [--no-preview]`
(source `camerad-dvr/main.cc`, single thread, libjpeg-turbo):

* **Device:** `--auto` takes the first whole-disk block device whose USB parent
  is 1902:3241 (sysfs walk); `--dev` refuses anything that is not 1902:3241 or
  is a partition. Opened `O_RDONLY | O_DIRECT | O_EXCL`: **read-only, never
  writes** (the command sector of the protocol is not used), uncached, and
  exclusive (fails while mounted or held by another camerad-dvr). Needs root or
  group `disk`. The layout is validated like `dvr_probe.py` does: MBR, FAT
  boot sector, the 8.3 entry `VSFILE` in the root directory, and the info +
  audio + first video sector inside VSFILE's first cluster run (VSFILE is only
  192 KiB on the tested unit; frames extend past its end, served live by the
  firmware). Reads are confined to [VSFILE start, video + 0x4B000 + 0x1400].
* **Loop:** read the 512-byte info sector every `poll_us` (2 ms sleep plus a
  ~2.7 ms USB round trip: ~210 polls/s, timestamp resolution ~5 ms). On a new
  `video_seq`: `ts_mono_ns` = CLOCK_MONOTONIC right after the info read that
  showed it; one `O_DIRECT` pread of `(size & ~0x1FF) + 0x200` bytes (one
  READ(10) up to `max_sectors_kb`); check word verified (re-polled up to 3
  times, then counted as `rejected`); `jpeg_read_raw_data` decodes the Y plane
  straight into the ring slot and the Cb/Cr planes into 8-row buffers that are
  interleaved into the NV12 UV plane (no colour conversion, no upsampling, no
  RGB). JFIF YCbCr is exactly the ring's BT.601 full range, so the pixels are
  the camera's own. After 400 ms without a new sequence it does the app's
  dummy "kick" read.
* **Ring:** 1280x720, strides 1280, `exposure_us`/gains 0. Created (and
  `camerad.sock` bound) only once a camera has been opened, so a second
  instance never replaces a live ring.
* **Unplug:** an I/O error closes the device; it is re-resolved and reopened
  every second, the ring and socket stay up, the ring `seq` continues. modeld
  never pairs frames across a stall > 500 ms.
* **Preview:** every `preview_every`-th frame the camera's own JPEG (cut at its
  EOI; the stream carries stale bytes after it) is written to `preview.jpg`
  (temp + rename): full 1280x720, no re-encode.
* **Burned-in overlay:** the camera draws its date/time and speed (`2022/04/26
  19:53:14   0 km/h`, clock unset) into the bottom ~40 rows, left half. It
  stays in the ring frames (nothing blanks it); whether it falls inside
  supercombo's warped crop depends on `focal_px` and the mounting calibration
  and has not been evaluated. Sign or OCR consumers should ignore the bottom
  ~40 rows.
* **Message:** the `camera` message gets `"source":"dvr"`, `video_seq` (the
  DVR's counter), `jpeg_bytes`, `read_ms`, `conv_ms` (decode + NV12),
  `rejected`, `decode_fail`, `jpeg_warnings`, `recording` (DVR flag 0x4000)
  and `reconnects`; `sensor_drops` counts gaps in `video_seq` (frames the camera
  produced that camerad-dvr never saw); `dropped` stays 0.

Config `/etc/roadsense/camera-dvr.json` (`ROADSENSE_CONFIG_DIR`), flat JSON:

| key | default | meaning |
|---|---|---|
| `focal_px` | 448 | **uncalibrated guess**: wide dashcam lens, ~110 deg HFOV, 640 / tan(55 deg). Calibrate it (supercombo's warp depends on it) |
| `cx`, `cy` | 640, 360 | principal point |
| `preview_every` | 2 | camera JPEG to `preview.jpg` every Nth frame, 0 = off |
| `poll_us` | 2000 | sleep between info-sector polls |

Measured on a laptop (i7-8850H, USB 2.0 high speed, 2026-10-22, 3206 frames):
19.5 fps (the camera's own rate), 0 sequence gaps, 0 rejected; frame read
5.8 ms; decode + NV12 1.7 ms avg (2.9 ms max); camerad-dvr 4.4 % of one core
including polling. modeld pairs consecutive frames (pair dt 49-52 ms).
Estimate for a Cortex-A75: decode ~5-7 ms per frame. Units:
`systemd/roadsense-camerad-dvr.service` (`--auto`); `scripts/install-camerad-dvr.sh
--enable` installs it, masks `roadsense-camerad.service` (which `roadsense.target`
and modeld otherwise pull in) and starts it; `--disable` switches back.

### camerad-replay: a recorded ride, looping

`camerad-replay --file RIDE.mjpeg [--fps N] [--start S] [--once] [--gap-ms MS] [--verbose] [--frames N] [--no-preview]`
(source `camerad-replay/main.cc`, single thread, libjpeg-turbo) plays a recorded drive into the ring
so the whole stack (modeld, envd, sign provider, api, dashboard) runs on road footage:

* **File:** concatenated baseline JPEGs, YCbCr 4:2:0 full range, width and height multiples of 16
  (`ffmpeg ... -pix_fmt yuvj420p -f mjpeg`; `tools/make_ride.sh` makes one at 1280x720, 20 fps, about
  125 KB/frame, i.e. ~1.5 GB per 10 minutes). The file is mmapped and indexed (SOI..EOI) at start-up
  (~0.15 s for 12000 frames on the laptop). The ring takes the frame size of the file.
* **Pacing:** absolute CLOCK_MONOTONIC deadlines at the source rate; `ts_mono_ns` is the frame's
  deadline. A frame more than one period late is skipped (counted in `sensor_drops`) so the ride stays
  in real time. At the end it pauses `--gap-ms` (default 1000, longer than modeld's 500 ms pairing
  limit, so no model pair spans the seam) and starts again; `--once` stops instead.
* **Decode:** same raw libjpeg path as camerad-dvr (Y straight into the slot, CbCr interleaved, no
  colour conversion): 2 ms per 1280x720 frame on the laptop.
* **Preview:** every `preview_every`-th frame's own JPEG becomes `preview.jpg` (no re-encode).
* **Message:** `camera` with `"source":"replay"`, `file_frame`, `file_frames`, `file_t_s`, `loops`,
  `jpeg_bytes`, `decode_fail`, `jpeg_warnings`; exposure and gains 0.

Config `/etc/roadsense/camera-replay.json` (`ROADSENSE_CONFIG_DIR`): `focal_px` (default: 975 scaled to
the frame width, a placeholder), `cx`, `cy` (centre), `fps` (20), `preview_every` (2). The intrinsics
and the mounting (`calib.json`) belong to the ride, not to the phone: keep a `camera-replay.json` and a
`calib.json` next to the ride file. `scripts/use-replay.sh --enable FILE` (root) installs the binary
(building it directly with `c++` if needed) and `systemd/roadsense-camerad-replay.service`
(`Conflicts=` camerad and camerad-dvr), installs the ride's two config files (saving the current
`calib.json`), stops the current camera, starts the replay and restarts the running ring readers;
`--disable` restores `calib.json` and the camera that ran before. Ride preparation, licences and the
calibration guesses: `docs/lanes/L5-ride-replay.md`.

## GPU schedule (`<rundir>/gpu.sched`, optional)

modeld and a sign provider that runs its detector on the same GPU (the Adreno 630 has one ring and no
preemption) collide: a detector submit that lands in the middle of a model run makes that run wait for
the whole detector (model runs become bimodal, ~56 / ~105 ms on the Poco F1). This file lets the
provider ask modeld for the GPU between two runs. Reference implementation: header-only
`camerad/gpusched.h` (`GpuSched`); measurements and design: `docs/lanes/L4-gpu-sched.md`.

```
struct GpuSchedShm {     // offset 0 of <rundir>/gpu.sched (4096 bytes, MAP_SHARED), little-endian
  char     magic[8];     // "RSGPU001", written by modeld after the fields below
  uint32_t run_seq;      // futex word: modeld adds 1 at the end of every model run (FUTEX_WAKE all)
  uint32_t turn;         // futex word: 1 = modeld granted the GPU to the provider; the provider sets 0
                         //   (and wakes) when its detector is done
  uint32_t want;         // 1 = the provider wants the next turn; it sets 0 when it stops waiting
  uint32_t reserved;     // 0
  int64_t  pid;          // modeld's pid (liveness: kill(pid, 0))
  int64_t  run_end_ns;   // CLOCK_MONOTONIC at the end of modeld's last run
  float    run_ms;       // wall time of modeld's last run
  float    det_ms;       // provider: its detector's wall time in a turn (moving average), 0 = unknown
};                       // 48 bytes
```

Protocol. modeld creates the file (`--gpu-sched PATH|none`, default `<rundir>/gpu.sched`), publishes
its pid, and at the end of every run: if `want` is set and it runs with `--gpu-yield MS` > 0, it stores
`turn = 1`; then it increments `run_seq` and wakes waiters. After a grant it starts its next run only
when `turn` is back to 0, or `--gpu-lead MS` before the provider's `det_ms` runs out (so its own CPU
preamble overlaps the end of the detector), and at the latest `MS` after the grant (then it resets
`turn` itself). The provider (signd `--gpu-sync turn`) sets `want = 1`, waits on `run_seq` (timeout,
signd `--gpu-sync-timeout`, default 150 ms), clears `want`, and if `turn == 1` submits its detector,
then sets `turn = 0`, updates `det_ms` and wakes. `--gpu-sync after` only waits for the tick (no grant).

Robustness: every wait has a timeout; a provider that finds no file, no magic, a dead pid or no run in
the last second does not wait at all; a provider that dies holding a turn costs modeld one `--gpu-yield`
timeout. Without the file (or with `--gpu-sync none`, `--gpu-yield 0`) both sides behave as before.

## JSON sockets

Each daemon listens on `/run/roadsense/<name>.sock` (SOCK_STREAM). Clients
connect and receive one JSON object per line, newest-first never: messages are
emitted in order, slow clients are disconnected when their send buffer exceeds
1 MiB. Every message has:

```
{"type": "<topic>", "seq": <frame seq>, "ts_mono_ns": <int>, "ts_real_ns": <int>, ...}
```

### camerad: `{"type":"camera", ...}` per frame
`exposure_us`, `analogue_gain`, `colour_gains:[r,b]`, `fps` (rolling 1 s),
`dropped` (frames not written because a consumer held the slot too long; always 0 with the drop-on-overwrite reader protocol, reported for completeness).
Additional fields: `sensor_drops` (cumulative gaps in libcamera's frame
sequence, i.e. frames lost before camerad saw them) and `conv_ms` (XRGB->NV12
conversion + ring write time of this frame). `seq` is the ring seq; `ts_*` are
the frame's SensorTimestamp and its realtime equivalent (offset re-sampled once
per second).

```
{"type":"camera","seq":294,"ts_mono_ns":8126624315000,"ts_real_ns":1792603772832163915,"exposure_us":54262,"analogue_gain":480.000,"colour_gains":[1.4321,3.0155],"fps":20.01,"dropped":0,"sensor_drops":0,"conv_ms":12.39}
```

### modeld: `{"type":"road", ...}` per model run (every frame it keeps up with)
modeld runs the network through an inference backend (`modeld/backend.h`):
`--backend ort` (default; ONNX Runtime CPU, `supercombo-fp32.onnx`) or
`--backend ncnn` (ncnn on the GPU through Vulkan, `supercombo-ncnn.param/.bin`
from `tools/ncnn/make_ncnn.sh`; only when built with `-DROADSENSE_NCNN=ON`, see
`modeld/CMakeLists.txt` and HARDWARE.md for its precision). Frame prep, the
feedback FIFOs, parsing and this message are shared, so both backends publish
the same schema; with ncnn the default pinning is `--cpus 0-3`. Config key
`backend` in `modeld.json`.

All distances in metres, device frame exactly as openpilot defines it: x forward,
y RIGHT, z DOWN, origin at the camera (modeld passes supercombo outputs through
in this frame, no axis conversion). Times in seconds.

```
"model_ms": 298.1,
"frame_seq_prev": 1203,            // second frame of the pair
"plan": {"t": [33 T_IDXS], "x": [33], "y": [33], "z": [33],
         "vx": [33], "vy": [33], "vz": [33], "ax":[33],"ay":[33],"az":[33],
         "roll":[33],"pitch":[33],"yaw":[33],"roll_rate":[33],"pitch_rate":[33],"yaw_rate":[33],
         "x_std":[33],"y_std":[33]},
"lane_lines": [ {"x":[33 X_IDXS],"y":[33],"z":[33],"y_std":[33],"prob":0.97}, x4 (far left, left, right, far right) ],
"road_edges": [ {"x":[33],"y":[33],"z":[33],"y_std":[33]}, x2 (left, right) ],
"leads": [ {"prob":0.9,"t":[6],"x":[6],"y":[6],"v":[6],"a":[6],"x_std":[6],"y_std":[6],"v_std":[6],"a_std":[6]}, x3 (now, +2s, +4s hypotheses) ],
"pose": {"vx":..,"vy":..,"vz":..,"roll_rate":..,"pitch_rate":..,"yaw_rate":.., "*_std":..},
"desire_state": [8], "desire_pred": [[8] x 4],
"meta": {"engaged_prob":..,"disengage_probs":{"gas":[5],"brake":[5],"steer_override":[5],"brake_3ms2":[5],"brake_4ms2":[5],"brake_5ms2":[5]},
         "blinkers":{"left":[6],"right":[6]}, "hard_brake_pred":[..]}
```
Exact index tables come from openpilot v0.9.7 `selfdrive/modeld/constants.py`
and are mirrored in `modeld/supercombo_layout.h`.

### sign provider: `{"type":"signs", ...}` per processed frame
Sign detection is not part of roadsense. Any external process may provide it by
reading the frame ring and listening on `<rundir>/signd.sock` with the message
below; the api connects to that socket like to the others and treats a missing
socket as "not connected" (everything else keeps working). The reference
implementation is [roadsense-signs](https://github.com/PiotrTopa/roadsense-signs)
(AGPL-3.0, separate repository because of its model licences), whose `signd`
runs an MTSD detector + classifier + SORT tracker.

```
"detect_ms": 85.0,
"detections": [ {"track_id": 17, "cls": "regulatory--maximum-speed-limit-50", "cls_id": 23, "score": 0.81,
                 "box": [x0,y0,x1,y1]  // pixels in the 1920x1080 frame
                 , "age": 12, "hits": 11} ],
"tracks_ended": [ {"track_id": 15, "cls": "...", "first_seq": 1100, "last_seq": 1190, "hits": 40, "best_score": 0.93,
                   "best_crop": "/run/roadsense/crops/15.jpg"} ]
```
What the api relies on: `detections[]` entries with `track_id`, `cls`, `score`
and `box` (frame pixels) build the live-track list and the preview overlay;
`tracks_ended[]` entries (keyed by `track_id`) end a live track and are stored
as-is in the history and `signs.jsonl`; `detect_ms` is shown in `/v1/health`.
Everything else is passed through untouched.
`best_crop`, if present, is a JPEG path the provider wrote under the run directory.
The reference implementation's tracker is SORT-style (IoU association,
constant-velocity Kalman per box, `max_age` 10 frames, `min_hits` 3) and reports
a track in `detections` only once confirmed.

### envd: `{"type":"env", ...}` about once per second
`envd/` reads the ring at most `--max-hz` times per second (default 1, newest frame each time),
squashes the whole frame to the image encoder's input (224x224 or so, no crop), runs a zero-shot
CLIP-type image encoder (ONNX Runtime CPU on the little cores by default; `--backend ncnn` for the
GPU when built with `-DROADSENSE_NCNN=ON`) and scores the embedding against precomputed text
embeddings of three class groups (`/var/lib/roadsense/models/envd-classes.json`, from
`tools/envd/text_embeddings.py`; the phone never runs a text encoder):

* `place`: highway, city_street, residential, rural_road, forest_road, tunnel, parking_lot, underground_parking, gas_station, bridge
* `weather`: clear, overcast, rain, snow, fog
* `light`: day, dusk_dawn, night

Per group, the per-frame softmax is averaged over a `--window` (10 s) and the published `label`
only changes when another class's smoothed score exceeds the current label's by `--hysteresis`
(0.10) and the label is at least `--min-hold` (3 s) old; `confidence` is the smoothed score of the
label, `stable_s` the time since it last changed. Cheap frame statistics ride along (luma 0..255).

```
"env_ms": 180.0, "encode_ms": 160.0,
"place":   {"label":"highway","confidence":0.81,"stable_s":42.0,
            "scores":{"highway":0.81,"city_street":0.07, ...},       // smoothed, sums to 1 over the group
            "frame_label":"highway","frame_scores":{...}},            // this frame only (omit with --no-frame-scores)
"weather": {...}, "light": {...},
"stats":   {"luma":112.5,"luma_top":170.2,"luma_bottom":80.1,"dark_frac":0.02,"bright_frac":0.01,"chroma":11.3},
"model":   "ViT-B-32/openai ort"
```
The last message is sent to every client when it connects. Accuracy per class, the choice of
encoder and the smoothing check: `docs/lanes/L2-envd.md`. The committed class file holds the
zero-shot prompt embeddings; `tools/envd/train_probe.py` writes a drop-in replacement whose rows
are a linear probe on frozen embeddings (trained on research-licensed data, so not committed;
point `classes` in `envd.json` at it). Config `/etc/roadsense/envd.json`
(keys as the CLI options, see `envd --help`); offline: `envd_replay` on JPEGs or an NV12 sequence.

## api (Python, aiohttp, port 8080)

| Endpoint | Content |
|---|---|
| `GET /v1/road` | latest `road` message |
| `GET /v1/signs` | latest `signs` message plus the list of currently live tracks |
| `GET /v1/signs/history?limit=100` | ended tracks, newest first |
| `GET /v1/env` | latest `env` message (place / weather / light with scores and confidence) |
| `GET /v1/speed_limit` | current speed limit derived by the api (topic `speed_limit`, below) plus `recent_changes` and `ignored` sign tracks |
| `GET /v1/camera` | latest `camera` message + ring header fields |
| `GET /v1/health` | per-daemon: connected, last message age, fps, latency (`model_ms`, `detect_ms`, `env_ms`), current `env` labels, `speed_limit` summary; CPU temp, battery |
| `GET /v1/frame.jpg` | current preview JPEG |
| `GET /preview` | MJPEG stream with lane lines, path, leads, sign boxes drawn on top |
| `GET /ws` | WebSocket; every message from every daemon, as-is, plus the api's own `speed_limit`; `?topics=road,signs,env,speed_limit` filters |

api never touches the camera or the models; it is a fan-out and overlay layer, plus one derived
topic below.

### api-derived topic: `{"type":"speed_limit", ...}`
`api/roadsense_api/speedlimit.py` turns the documented `signs` contract (only `detections[]`
track_id/cls/score/box/hits and `tracks_ended[]` track_id), `road` (`v_ego`, `pose.yaw_rate`,
`ts_mono_ns`) and `env` (`place` label) into the limit that applies to the car. Sent on `/ws` on
every change and at least once a second; `/v1/speed_limit` adds `recent_changes` and `ignored`.
Rules are data: `api/roadsense_api/rules/<country>.json` (`--rules`, env `ROADSENSE_RULES`, default
`pl`). Design, Polish sources and limits: `docs/lanes/L6-speed-limit.md`.

```
"limit_kmh": 70,                         // null = unknown (no sign, area unknown)
"source": {"kind":"sign","sign":"B-33","cls":"regulatory--maximum-speed-limit-70","track_id":19339,
           "hits":24,"vote":1.0,"score":0.83,"lateral_m":4.1, "plate":"complementary--distance"?},
       // or {"kind":"default","rule":"built_up|outside|residential_zone|expressway|motorway","area_source":"sign|env|config"}
       // or {"kind":"unknown","rule":"unknown_area"}
"confidence": 0.71,                      // 0..1, heuristic (track quality, fades towards the expiry cap)
"since": {"ts_real_ns":..,"age_s":12.3,"distance_m":180.0},   // since the served value last changed
"expiry": {"rule":"next intersection (par. 32 ust. 2)","distance_cap_m":700,"remaining_m":520.0},  // null for defaults
"area": {"type":"built_up|outside|unknown","source":"sign|env|config","confidence":0.4},
"road_class": "ordinary|expressway|motorway",
"zone": {"type":"none|residential|speed_zone","limit_kmh":20},
"advisory_kmh": null,                    // complementary speed plate, not a limit
"pending": [ {"track_id":..,"cls":..,"sign":"B-33","action":"limit","value_kmh":50,"hits":4,"vote":1.0,
              "quality":0.6,"lateral_m":3.2} ],   // live tracks not yet confirmed
"v_ego_kmh": 48.2, "odometer_m": 3837.0, "rules": "PL"
```
Overlays project the 3D road outputs into the image with the pinhole model
from the ring header (focal_px, cx, cy) and the calibration used by modeld,
published by modeld in a `{"type":"calib", ...}` message to every client when it
connects, and to all clients on change.

## Calibration

modeld publishes `calib`: `{"pitch":..,"yaw":..,"roll":..,"height_m":..,"focal_px":..,"source":"config|auto"}`.
Phase 1 uses a config file (`/etc/roadsense/calib.json`); phase 2 estimates
pitch/yaw from the model's own pose output like openpilot's calibrationd.

## Build and deploy

C++17, CMake, one binary per daemon. In the reference deployment (Poco F1,
postmarketOS) everything is built on the phone (`g++`, `cmake`,
`onnxruntime-dev`, `libcamera-dev` from Alpine); `scripts/install-phone.sh`
builds, installs to `/usr/local/bin`, writes the systemd units from `systemd/`
and enables `roadsense.target`. Models under `/var/lib/roadsense/models`,
fetched with `scripts/fetch-models.sh`. A sign provider installs its own unit
with `PartOf=`/`WantedBy=roadsense.target`; the target does not require it.
