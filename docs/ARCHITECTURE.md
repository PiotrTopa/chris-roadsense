# Architecture and inter-process contract

Four processes, all started by systemd on the phone, all talking through two
simple mechanisms: a shared-memory frame ring (camerad -> consumers) and
newline-delimited JSON over Unix stream sockets (every daemon -> api / tools).
No message broker, no protobuf, no capnp. `nc -U /run/pocopilot/modeld.sock`
must always be a valid debugging tool.

```
camerad ──frame ring (shm)──┬──> modeld ──JSON──┐
                            └──> signd  ──JSON──┼──> api ──> HTTP / WebSocket / MJPEG
camerad ──JSON (frame meta)─────────────────────┘
```

## Frame ring (`/dev/shm/pocopilot-frames`)

A single file created by camerad, mmapped by everyone else. Little-endian,
all offsets in bytes, header is 4096 bytes. camerad builds a fresh file under a
temporary name and `rename()`s it over the path at start-up (never truncates a
mapped file, so old readers cannot SIGBUS); readers detect the new inode and
remap (`RingReader` in `camerad/ring.h` does this). Reference implementation of
everything in this section: `camerad/ring.h` (header-only, `RingWriter` /
`RingReader`). Tests may override the path with env `POCOPILOT_RING`.

Pixel data: NV12, **BT.601 full range** (JFIF: Y 0..255, Cb/Cr 0..255 centred
on 128; `Y = 0.299R + 0.587G + 0.114B`, `Cb = -0.168736R - 0.331264G + 0.5B + 128`,
`Cr = 0.5R - 0.418688G - 0.081312B + 128`), sRGB-gamma values as the software
ISP delivers them, chroma = box average of each 2x2 block.

```
struct RingHeader {            // offset 0
  char     magic[8];           // "PPFRAME1"
  uint32_t header_size;        // 4096
  uint32_t slot_count;         // 6
  uint32_t slot_size;          // bytes per slot incl. SlotHeader, 4096-aligned
  uint32_t width, height;      // 1920, 1080 (native sensor size)
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

camerad also writes a JPEG of every Nth frame to `/run/pocopilot/preview.jpg`
(atomic rename) for the MJPEG endpoint, N configurable (default 3): 960x540
(2x box downscale of the NV12 frame), baseline 4:2:0, quality 80, ~110 KB.
If the encoder is still busy with the previous preview the frame is skipped.

Run directory: `/run/pocopilot`, overridden by env `POCOPILOT_RUNDIR` for
every daemon (`/tmp/pocopilot` for development).

**/dev/shm and logind:** systemd-logind's default `RemoveIPC=yes` deletes a
regular (uid >= 1000) user's files in `/dev/shm` when that user's last session
ends (observed on the phone: the ring vanished after the ssh session closed).
The systemd units must therefore run camerad as a system user (uid < 1000) or
root, or logind needs `RemoveIPC=no`. During development, run camerad and its
readers inside the same login session.

### camerad configuration (`/etc/pocopilot/camera.json`, env `POCOPILOT_CONFIG_DIR`)

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

## JSON sockets

Each daemon listens on `/run/pocopilot/<name>.sock` (SOCK_STREAM). Clients
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

### signd: `{"type":"signs", ...}` per processed frame
```
"detect_ms": 85.0,
"detections": [ {"track_id": 17, "cls": "regulatory--maximum-speed-limit-50", "cls_id": 23, "score": 0.81,
                 "box": [x0,y0,x1,y1]  // pixels in the 1920x1080 frame
                 , "age": 12, "hits": 11} ],
"tracks_ended": [ {"track_id": 15, "cls": "...", "first_seq": 1100, "last_seq": 1190, "hits": 40, "best_score": 0.93,
                   "best_crop": "/run/pocopilot/crops/15.jpg"} ]
```
Tracker: SORT-style (IoU association, constant-velocity Kalman per box,
`max_age` 10 frames, `min_hits` 3). A track is reported in `detections` only
once confirmed.

## api (Python, aiohttp, port 8080)

| Endpoint | Content |
|---|---|
| `GET /v1/road` | latest `road` message |
| `GET /v1/signs` | latest `signs` message plus the list of currently live tracks |
| `GET /v1/signs/history?limit=100` | ended tracks, newest first |
| `GET /v1/camera` | latest `camera` message + ring header fields |
| `GET /v1/health` | per-daemon: connected, last message age, fps, latency; CPU temp, battery |
| `GET /v1/frame.jpg` | current preview JPEG |
| `GET /preview` | MJPEG stream with lane lines, path, leads, sign boxes drawn on top |
| `GET /ws` | WebSocket; every message from every daemon, as-is; `?topics=road,signs` filters |

api never touches the camera or the models; it is a fan-out and overlay layer.
Overlays project the 3D road outputs into the image with the pinhole model
from the ring header (focal_px, cx, cy) and the calibration used by modeld,
published by modeld in a `{"type":"calib", ...}` message to every client when it
connects, and to all clients on change.

## Calibration

modeld publishes `calib`: `{"pitch":..,"yaw":..,"roll":..,"height_m":..,"focal_px":..,"source":"config|auto"}`.
Phase 1 uses a config file (`/etc/pocopilot/calib.json`); phase 2 estimates
pitch/yaw from the model's own pose output like openpilot's calibrationd.

## Build and deploy

C++17, CMake, one binary per daemon, built on the phone
(`g++`, `cmake`, `onnxruntime-dev`, `libcamera-dev` from Alpine).
`scripts/install-phone.sh` builds, installs to `/usr/local/bin`, writes the
systemd units from `systemd/` and enables them. Models under
`/var/lib/pocopilot/models`, fetched with `scripts/fetch-models.sh`.
