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
                            ├──> sign provider (external) ─JSON──┼──> api ──> HTTP / WebSocket / MJPEG
                            └──> light provider (external) ─JSON─┤
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

`camerad-replay --file RIDE.mjpeg | --playlist LIST|DIR [--fps N] [--start S] [--once] [--gap-ms MS] [--verbose] [--frames N] [--no-preview]`
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
  `jpeg_bytes`, `decode_fail`, `jpeg_warnings`; exposure and gains 0. Ride identity and attribution:
  `ride` (slug = the ride's directory name), `ride_index` (0-based position in the playlist), `ride_count`,
  `ride_t_s`, `ride_duration_s`, `playlist` (bool), `author` and `licence` (from the ride's `SOURCE.txt`:
  line 1 `<title> -- <author> (<url>)`, first `CC BY...` token). The dashboard shows
  "Ride: <slug> — <author>, <licence>" on the status page and on the `/preview` HUD (ASCII there).
* **Playlist** (`--playlist LIST|DIR [--shuffle] [--max-seconds-per-ride S] [--calib-out PATH | --no-calib]`):
  rides one after another, looping the list (`LIST`: one ride directory or `.mjpeg` per line, `#`
  comments, re-read at every wrap; `DIR`: its subdirectories by name; `--shuffle` reshuffles at every
  wrap). Per ride: `<dir>/*.mjpeg`, `camera-replay.json` (intrinsics, fps), `calib.json`, `SOURCE.txt`.
  **Ride change contract:** (1) the ride's `calib.json` is copied atomically (tmp + rename) to
  `--calib-out` (default `$ROADSENSE_CONFIG_DIR/calib.json`); (2) the next file is opened and indexed
  (the previous one is unmapped and dropped from the page cache); (3) a **new ring** is created (rename
  over the old one) with the ride's frame size and `focal_px`/`cx`/`cy`; **seq restarts at 1**; (4) a
  pause of `--gap-ms` (default 2500 in playlist mode) before the first frame. Readers follow the new
  ring within 500 ms (RingReader reopen clock); modeld re-reads `calib.json` on mtime change, checked at
  most every 2 s on frame arrival, so after a > 2 s pause its first frame of the new ride already uses
  the new calibration (no restart). The seq restart is the "camera restarted" signal every consumer
  already handles: modeld drops its frame pairing, envd resets its smoother, the sign provider flushes
  its sign and light tracks, and the api resets its speed-limit state (limit, area, odometer), live sign
  tracks and sign context on a change of `ride`/`ride_index` or a seq going backwards. A ride that
  cannot be opened is skipped; with none playable the player retries every 10 s.

Config `/etc/roadsense/camera-replay.json` (`ROADSENSE_CONFIG_DIR`): `focal_px` (default: 975 scaled to
the frame width, a placeholder), `cx`, `cy` (centre), `fps` (20), `preview_every` (2). The intrinsics
and the mounting (`calib.json`) belong to the ride, not to the phone: keep a `camera-replay.json` and a
`calib.json` next to the ride file. `scripts/use-replay.sh --enable FILE` (root) installs the binary
(building it directly with `c++` if needed) and `systemd/roadsense-camerad-replay.service`
(`Conflicts=` camerad and camerad-dvr), installs the ride's two config files (saving the current
`calib.json`), stops the current camera, starts the replay and restarts the running ring readers;
`--disable` restores `calib.json` and the camera that ran before. Ride preparation, licences and the
calibration guesses: `docs/lanes/L5-ride-replay.md`. `scripts/use-replay.sh --playlist [DIR] [--shuffle]` plays
the whole ride library (`docs/RIDES.md` catalogue order, written to `/etc/roadsense/replay-playlist.txt`;
camerad-replay installs each ride's `calib.json` itself); switching between `--enable` and `--playlist`
keeps the pre-replay `calib.json` backup and the camera to restore. Playlist: `docs/lanes/L10-playlist.md`.

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

**Flagged tracks (optional fields, lane L8).** A provider may judge a track as probably not a sign and
still report it, flagged, so the reason stays inspectable: `"suppressed": true, "reason": "<word>"` on a
`detections[]` entry (the decision for the track so far; it may clear later, e.g. `score`) and on a
`tracks_ended[]` entry (final). Absent means accepted. The reference provider flags round red classes
(prohibitions, speed limits, stop: stop lamps and red traffic lights get read as `no-entry`) with
`verifier` (mean probability of a small real-sign-vs-lamp CNN below 0.5, if one is installed), `glow`
(most of the track's detections look like a lamp: blown-out core fading into bright pink-red; only while
the verifier is unsure too, since sunlit real signs glow as well) and, for any class, `score` (best
detector score below a per-class minimum, e.g. `no-entry` 0.71). A classifier with a "not a sign" class (L12) flags tracks voted `background` with `not_a_sign`; its `other-sign` class (a real sign outside the known classes) is reported unflagged and matches no rule. It also reports `glow` (that share, 0..1) and `verify` (the verifier mean) on round red tracks.
The api adds its own context checks (`api/roadsense_api/signctx.py`): a round red track whose box centre
sits inside a traffic light box (`lights` topic) or inside the lead vehicle's rear (modeld `leads[0]`
projected, 2.6 m x 0-2 m) for >= half of its detections gets `"context": {"traffic_light": share,
"lead_vehicle": share}` and is flagged (`reason` `traffic_light` / `lead_vehicle`) unless the provider's
`verify` >= 0.9 with `glow` < 0.5 vouches for it. Flagged tracks are not live tracks, are not used by the
speed-limit state (listed in its `ignored` as `suppressed (<reason>)`), are drawn grey with the reason in
`/preview`, and are listed in `/v1/signs` as `suppressed_tracks`; history and `signs.jsonl` keep them with
the flag.

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

### light provider: `{"type":"lights", ...}` on `<rundir>/lightd.sock`
Traffic lights, like signs, come from an external provider: any process that reads the ring and serves
`<rundir>/lightd.sock`; the api connects to it like to the other daemons (daemon name `lightd`) and treats a
missing socket as "not connected". The reference implementation is `signd --lights` in roadsense-signs (a
second thread of signd with its own ring reader and socket; BDD100K YOLO11n detector on the upper part of the
frame, a 48x24 state CNN per light, a small tracker; models and licences: roadsense-signs
`docs/LIGHT_MODELS.md`, measurements: `docs/lanes/L7-traffic-lights.md`). One message per processed frame
(about 3/s):

```
"lights_ms": 160.0,                      // provider time for this frame (detector + state + tracker)
"frame": [1280, 720],                    // ring frame size; boxes are in these pixels
"roi": [0, 0, 1280, 446],                // part of the frame the detector looked at
"lights": [ {"track_id": 7, "state": "red",          // smoothed per track: red, yellow, red_yellow, green, off, unknown
             "state_conf": 0.97, "state_s": 4.2,      // confidence of the state, seconds since it last changed
             "frame_state": "red",                    // this frame's observation
             "flashing": false,                       // yellow <-> off alternating within 4 s
             "score": 0.66, "box": [x0,y0,x1,y1], "horizontal": false, "hits": 12, "age": 13} ]
```
Only confirmed tracks (>= 2 hits) matched in the current frame are listed. `red_yellow` is inferred from
the sequence (a yellow observation straight after red; the Vienna-convention phase before green), `off`
covers dark housings and side or back views. The api annotates every `lights` message before storing and
forwarding it (`api/roadsense_api/lights.py`): per light `dist_m` (from the housing size, rough),
`lateral_m` (offset from modeld's planned path at that distance), `relevance` 0..1 and `ego`, and a
message-level `ego` for the light that applies to the ego lane:

```
"ego": {"track_id": 7, "state": "red", "flashing": false, "dist_m": 34.0, "lateral_m": 2.1, "relevance": 0.71,
        "agree": 2, "n_relevant": 2, "source": "path"}     // null when no light is relevant
```

## api (Python, aiohttp, port 8080)

| Endpoint | Content |
|---|---|
| `GET /v1/road` | latest `road` message |
| `GET /v1/signs` | latest `signs` message plus the currently live tracks (`live_tracks`) and the flagged ones (`suppressed_tracks`, with `reason`) |
| `GET /v1/signs/history?limit=100` | ended tracks, newest first |
| `GET /v1/env` | latest `env` message (place / weather / light with scores and confidence) |
| `GET /v1/speed_limit` | current speed limit derived by the api (topic `speed_limit`, below) plus `recent_changes` and `ignored` sign tracks |
| `GET /v1/lights` | latest `lights` message with the api's ego-lane annotation (`ego`, per light `dist_m`, `lateral_m`, `relevance`) |
| `GET /v1/rules` | rules in effect for the car derived by the api (topic `rules`, below: `in_effect`, `recent`, `ahead`) plus `pending`, `recent_changes` and `ignored` sign tracks |
| `GET /v1/camera` | latest `camera` message + ring header fields |
| `GET /v1/health` | per-daemon: connected, last message age, fps, latency (`model_ms`, `detect_ms`, `env_ms`), current `env` labels, `speed_limit` summary, `rules` summary (short strings: in effect / recent / ahead), `lights_ms` and the ego light; CPU temp, battery |
| `GET /v1/frame.jpg` | current preview JPEG |
| `GET /preview` | MJPEG stream with lane lines, path, leads, sign boxes and traffic lights (box in the state colour, the ego light thick with its distance) drawn on top; HUD lines IN EFFECT / RECENT / AHEAD from the `rules` topic |
| `GET /ws` | WebSocket; every message from every daemon, as-is (`lights` with the api's annotation, `signs` with its context flags), plus the api's own `speed_limit` and `rules`; `?topics=road,signs,env,speed_limit,rules,lights` filters |

api never touches the camera or the models; it is a fan-out and overlay layer, plus the two derived
topics below.

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
### api-derived topic: `{"type":"rules", ...}`
`api/roadsense_api/rules.py` (lane L11) generalises the speed-limit engine: it owns the `SpeedLimit` above,
feeds it every `road` / `signs` / `env` message and also takes `lights`, and judges every other sign class of
the rule table `rules/<country>.json` -> `effects` (category, how it ends, side, legal source, guesses marked)
with the same track confirmation, lateral-position and plate code. Sent on `/ws` on every change and at least
once a second; `/v1/rules` adds `pending`, `recent_changes` and `ignored`. Design, Polish sources, rule table
and the offline evaluation: `docs/lanes/L11-signs-in-effect.md`; offline runner `tools/rules_replay.py`.

```
"in_effect": [                           // lasting rules + momentary signs that bind at the next intersection
  {"id":"speed_limit","sign":"B-33","name":"speed limit","category":"lasting","value":40,"unit":"km/h",
   "source":{"kind":"sign","track_id":164,"cls":"regulatory--maximum-speed-limit-40",...},   // as in speed_limit
   "since":{"ts_real_ns":..,"age_s":12.3,"distance_m":180.0}, "confidence":0.71,
   "ends":"next intersection (par. 32 ust. 2)", "remaining_m":520.0, "uncertain":[], "applies_to_car":true, "side":null},
  {"id":"built_up_area","sign":"D-42","name":"built-up area",...,"source":{"kind":"env"},"uncertain":["entry sign D-42 not observable; area from envd"]},
  {"id":"no_stopping","sign":"B-36","name":"no stopping","category":"lasting","value":null,"unit":null,
   "source":{"kind":"sign","track_id":1780,"cls":"regulatory--no-stopping","hits":14,"vote":1.0,"score":0.81,"lateral_m":3.4,"plate":null},
   "since":{...}, "confidence":0.62, "ends":"next intersection (par. 32 ust. 2), B-35 or an end plate",
   "remaining_m":430.0, "uncertain":["plate complementary--distance"], "applies_to_car":true, "side":"right"},
  {"id":"no_left_turn","sign":"B-21","category":"momentary",...,"ends":"the next intersection (par. 22 ust. 2)"} ],
   // other ids: priority_road, no_parking, no_overtaking, height_limit, weight_limit, width_limit, one_way,
   // living_street, motorway, expressway, stop, yield, no_right_turn, no_u_turn, mandatory_direction
"recent": [                              // momentary signs just passed, newest first; kept 300 m / 120 s
  {"id":"no_entry","sign":"B-2","name":"no entry","category":"momentary","source":{...},
   "passed":{"age_s":8.1,"distance_m":95.0}, "confidence":0.7, "uncertain":["side road?"], "applies_to_car":true,
   "lateral_m":11.2, "ended":null} ],   // `ended`: why an until-intersection sign left in_effect
"ahead": [                               // warning signs, nearest first
  {"id":"roundabout_ahead","sign":"A-8","name":"roundabout ahead","category":"warning","source":{...},
   "expected_in_m":60.0, "range_m":[0.0,60.0], "since_sign_m":40.0, "confidence":0.66, "uncertain":[]} ],
"junction": {"last":{"odo":2318.0,"source":"sign D-1/D-2|traffic light|turn","detail":1780,"ts_real_ns":..,"age_m":12.0},"count":6},
"area": {...}, "road_class": "ordinary", "zone": {...}, "v_ego_kmh": 48.2, "odometer_m": 3837.0, "rules": "PL"
```
`/v1/health` carries `rules: {"in_effect": ["B-33 40", "D-42 built-up area?", "B-36 no stopping (R)"],
"recent": ["B-2 no entry"], "ahead": ["A-8 roundabout ahead ~60 m"]}` (a trailing `?` marks an uncertain entry).
A ride change or camera restart replaces the engine (L10), like the speed limit.

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
Every unit runs in `roadsense.slice` (`systemd/roadsense.slice`: CPUWeight 100,
MemoryHigh 3G / MemoryMax 3584M) so whatever else the machine does stays
responsive while the stack saturates the GPU and the big cores; a sign provider
puts `Slice=roadsense.slice` in its unit too. The API port is
`ROADSENSE_API_PORT` (`/etc/roadsense/api.env`, written by
`API_PORT=N scripts/install-phone.sh`; default 8080).

## Running under a host controller (the car computer)

roadsense owns no power or ignition logic: it is a set of systemd units that a
host may start and stop. The reference deployment is the single Prius car
computer (the same Poco F1 runs the Cyberpunk Prius backend, repo
`cyberpank-prius-gen2-computer`, `backend/roadsense_power.py`), whose contract
with this stack is exactly:

* **process boundary only**: `systemctl start|stop|restart --no-block
  roadsense.target`, `systemctl is-active <unit>...`; nothing is imported or
  linked (the sign provider is AGPL, this repo is MIT, the car computer is its
  own project);
* **health**: `GET /v1/health` on `ROADSENSE_API_PORT` (8090 on that machine,
  because the car backend serves 8080); a silent API is restarted by the host
  after a timeout;
* **camera power**: the host switches the USB hub that carries the DVR camera
  (powerbox relay ch4) on before the start and off after the stop;
  `camerad-dvr --auto` tolerates the camera appearing late;
* **ignition**: the host starts the target when ACC comes on and stops it
  cleanly (grace, `systemctl stop`, wait for the units) when ACC goes off;
  `roadsense.target` is therefore **not enabled at boot** on that machine
  (`install-phone.sh` without `AUTOSTART=1`);
* **CPU profile**: the host keeps the Gold cluster online while any unit runs
  (modeld/signd/envd pin cpu4-7) and applies its idle profile only afterwards.

Nothing in this list is specific to that backend: any supervisor that can run
`systemctl` and read JSON can drive the stack the same way.
