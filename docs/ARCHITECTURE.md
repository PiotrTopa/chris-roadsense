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

A single file created and truncated by camerad, mmapped by everyone else.
Little-endian, all offsets in bytes, header is 4096 bytes.

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
  uint64_t write_seq;          // last fully written frame number (atomic store, release); 0 = none yet
  uint64_t camerad_pid;        // liveness
}
struct SlotHeader {            // at header_size + i*slot_size, i = seq % slot_count
  uint64_t seq;                // frame number, starts at 1
  int64_t  ts_mono_ns;         // CLOCK_MONOTONIC at sensor timestamp (libcamera SensorTimestamp)
  int64_t  ts_real_ns;         // CLOCK_REALTIME at the same instant
  uint32_t exposure_us;
  float    analogue_gain;
  float    colour_gain_r, colour_gain_b;
  uint32_t data_offset;        // from slot start, 4096
  uint32_t data_size;          // y_stride*height + uv_stride*height/2
}
```

Writer protocol: pick slot `seq % slot_count`, write SlotHeader with the new
seq and pixel data, then store `write_seq = seq` with release semantics.
Reader protocol: read `write_seq` (acquire), copy or process the slot, then
re-read that slot's `seq`; if it changed the slot was overwritten mid-read,
drop the frame. Readers poll `write_seq` with a 1-2 ms sleep; there is no
notification channel (30 fps makes polling cheap).

camerad also writes a JPEG of every Nth frame to `/run/pocopilot/preview.jpg`
(atomic rename) for the MJPEG endpoint, N configurable (default 3).

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

### modeld: `{"type":"road", ...}` per model run (every frame it keeps up with)
All distances in metres, device frame as openpilot defines it: x forward,
y left, z up, origin at the camera. Times in seconds.

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
published by modeld in a `{"type":"calib", ...}` message on start and on change.

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
