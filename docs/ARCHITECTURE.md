# Architecture and inter-process contract

Five core processes (camerad, modeld, envd, mapd, api) plus any number of external
consumers such as a sign provider, all talking through two simple mechanisms: a
shared-memory frame ring (camerad -> consumers) and newline-delimited JSON over
Unix stream sockets (every daemon -> api / tools; and, in the other direction,
an external position feeder -> mapd). These two mechanisms are the
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
camerad ──JSON (frame meta)──────────────────────────────────────┤
position feeder (external) ──JSON (gnss, motion)──> mapd ──JSON──┘      (mapd: no camera; map pack on disk)
                                                     └──JSON (ego: the car's own motion, as fed)──> api (lead tracker)
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
"roi": [0, 0, 1280, 446],                // part of the frame the detector looked at in this frame
"view": 0,                               // optional: 0 = the wide view, 1 = the provider's second (far) view
"lights": [ {"track_id": 7, "state": "red",          // smoothed per track: red, yellow, red_yellow, green, off, unknown
             "state_conf": 0.97, "state_s": 4.2,      // confidence of the state, seconds since it last changed
             "frame_state": "red",                    // this frame's observation ("unknown" while coasting)
             "flashing": false,                       // yellow <-> off alternating within 4 s
             "score": 0.66, "box": [x0,y0,x1,y1], "horizontal": false, "hits": 12, "age": 13,
             "coast": 0} ]                            // optional: processed frames since the track was last detected
```
Confirmed tracks (>= 2 hits) are listed: those matched in the current frame and, since 2026-11-16 (lane L15),
those the reference provider is coasting (`coast` 1..`--lights-coast`, the box predicted, the state the last
smoothed one), so a light the detector misses in one frame, or one that only the other view can see, does not
drop out of the message. A provider that lists matched tracks only simply never sends `coast`. The reference
provider can alternate two views (`--lights-roi2`: every other frame looks at a smaller part of the picture
at a higher scale, for the far lights that are too small after the wide view's downscale; `roi` / `view` say
which one a message used) and can let boxes below the detection threshold continue an existing track
(`--lights-thr-keep`); both are measured in `docs/lanes/L15-stability.md`. The reference unit looks at the
lights five times a second since lane L21 (three before; `docs/lanes/L21-budget-and-models.md`). `red_yellow` is inferred from
the sequence (a yellow observation straight after red; the Vienna-convention phase before green), `off`
covers dark housings and side or back views. The api annotates every `lights` message before storing and
forwarding it (`api/roadsense_api/lights.py`): per light `dist_m` (from the housing size, rough),
`lateral_m` (offset from modeld's planned path at that distance), `relevance` 0..1 and `ego`, and a
message-level `ego` for the light that applies to the ego lane:

```
"ego": {"track_id": 7, "state": "red", "flashing": false, "dist_m": 34.0, "lateral_m": 2.1, "relevance": 0.71,
        "agree": 2, "n_relevant": 2, "source": "path",
        "held": false}                                     // null when no light is relevant
```
The ego light over time (`EgoLightSelector`, since 2026-11-16): the light that is the ego light stays it while
its relevance is at least 0.07 (it becomes it at 0.15: hysteresis), a coasting track counts with 0.85 per
missed frame, and when no light qualifies the last ego light is served for up to 1.2 s more with
`"held": true` and `"age_s"` (how old that state is). A consumer that must not show a state older than a
frame checks `held`.
Lane assignment (lane L16, 2026-11-17; `docs/lanes/L16-my-lights-junctions.md`): at a junction with separate
signal groups (a left-turn signal beside the one for straight on) the heads that face the car show different
states at the same moment. The lateral window alone then picks by the offset from the path; since L16 the
choice between the two groups is made by lane, and the message says how sure it is:

```
"lights": [ {..., "slot": -1.56} ],        // per light: offset from the centre of the ego lane in lane widths at its distance (0 = over the ego lane, -1 = one lane to the left)
"ego": {..., "amb": false,                 // the choice is ambiguous (below); a consumer that shows one light marks it
        "assign": {"conflict": true,       // facing heads show stop (red, red_yellow, yellow) and go (green) at once
                   "confidence": 0.89,     // conflict: the chosen group's share of the two groups' best scores (0.5 = a coin toss); else the chosen head's lane weight x 2, capped at 1
                   "basis": "lanes|path",  // lanes: the tracked lane lines of the `lanes` topic; path: no usable lines, the planned path with a 3.3 m lane
                   "slot": -0.66, "n_left": 1, "n_right": 0,     // the chosen head's slot; lanes between the ego lane and the road edge on each side (null = unknown)
                   "alt": {"track_id": 3, "state": "red", "side": "left", "slot": -1.61},   // the best head of the other group; null without a conflict
                   "other_lane": true}}    // only when set: no conflict, and the one facing light hangs over another lane
```
A head's lane weight is `exp(-(slot_eff / (0.7 * (1 + d / 60)))^2)`: the tolerance grows with the distance,
because the lateral offset of a far light is rough (the distance comes from the housing size, +-30 %). A head
beyond the outermost lane (a post at the roadside) counts as a head of the outermost lane it stands beside
(`slot_eff` = lanes up to it + half of the rest); with an unknown lane count this is assumed on the right only.
Its score is that weight times its relevance without the lateral window. The group whose best head scores
higher wins, the ego light is the best head inside it (the current ego track keeps the role at 60 % of the
best), and only that group votes on the state. `amb` is set when the winner's share stays under 0.7 for two
messages in a row, or when `other_lane` is set. The assignment never removes a light: when all facing heads
agree, the choice is the one of the lateral window, as before. A tagged lane count of the matched road
(`map.road.lanes`, source `tagged`) caps the lanes assumed to the right of the ego lane; inert without a
position source. Arrow lenses are not told apart: the provider has no arrow state (a lens is 4-9 px wide at
the distances that matter). `<config dir>/lights.json` `{"lane_assign": false}` switches the assignment off
(no `slot`, `assign`, `amb`). Replay on a recorder session: `tools/lights_lane_replay.py`.

Map prior (soft, optional): when the `map` message is usable (available, younger than 3 s, match confidence
>= 0.5) and its look-ahead lists `traffic_signals` or `crossing_signals` within 150 m on the most likely path,
the message carries `"map_signal": {"kind": "traffic_signals", "dist_m": 80.0, "confidence": 0.8}`, a light
becomes the ego light from relevance 0.10 instead of 0.15 and the hold is 2 s. Without a position source
`map_signal` is null and nothing changes; `map_signal` set with `ego` null is "the map expects a light here
and the camera has none". Not exercised on the car yet (no live GNSS fix).

### mapd: `{"type":"map", ...}` on `<rundir>/mapd.sock`, `{"type":"ego", ...}` on `<rundir>/ego.sock`, `{"type":"vehicle", ...}` and `{"type":"objects", ...}` on `<rundir>/vehicle.sock`, vehicle input on `<rundir>/position.sock`
`mapd/roadsense_mapd` (Python, numpy; `python3 -m roadsense_mapd`) is where everything a host knows about
the car comes in, and it owns everything that has to do with where the car is: it takes raw GNSS fixes,
vehicle motion, driver controls, body signals and range-sensor object lists from external feeders, filters
the position, matches it to the road graph of a map pack (OpenStreetMap, `docs/MAPS.md`) and publishes the
map context of that position, the car's motion (`ego`) and the car as the host describes it (`vehicle`,
`objects`). It never touches the camera. It runs, and says "unavailable", without a feeder and without a
pack; the vision stack does not depend on it.

**The contract of a feeder is `docs/VEHICLE_API.md` (vehicle input API 1.1)**: transport and handshake,
clock, frames and signs, per-signal quality, every message (`gnss`, `motion`, `reset`, `controls`, `body`,
`vehicle`, `objects`), rates, the `vehicle` / `objects` output and the calibration feedback. It is a
superset of the position input API v1 below, which stays valid word for word: a v1 feeder needs no change.

```
feeder ──gnss, motion──> position.sock ─┬──────────────── motion, as accepted ────────────────────────> ego.sock
modeld.sock (road: v_ego, yaw rate) ····┼─> filter (EKF) ─> map matcher ─> road, speed limit, look-ahead ─> mapd.sock
   (only with vision_odometry on)       │        ▲──── cross-track + bearing while dead reckoning ───┘
                       pack (.rsmap) ───┘
```

**Position input API v1** (the contract of a feeder; reference client `tools/position_client.py`, a single
file without dependencies that a feeder may copy; nothing is imported from roadsense):

* Transport: Unix stream socket `<rundir>/position.sock` (mode 0666, `--input-mode`), UTF-8 JSON, one
  object per line, any number of feeders. A valid message gets no reply. An invalid one gets
  `{"type":"error","line":<n>,"reason":"..."}` on the same connection and is dropped; the connection stays
  open. `{"type":"hello","api":1,"name":"..."}` is answered with
  `{"type":"hello","api":1,"minor":1,"server":"roadsense-mapd","accepts":["gnss","motion","reset","controls","body","vehicle","objects"],"signals":{...},"clock":"CLOCK_MONOTONIC","now_mono_ns":<int>}`
  (`reset` is listed since 2026-11-15, the vehicle API types and `minor` since 2026-11-17; a feeder that
  needs one checks `accepts`).
  The same messages can be sent over HTTP: `POST /v1/position` (one object, a JSON array, or JSON lines;
  answer `{"api":1,"accepted":n,"rejected":[{"index":i,"reason":"..."}]}`); the socket is the path for
  motion at its rate. A feeder must never block on the socket: when mapd is away, drop.
* Clock: `ts_mono_ns` (integer) is CLOCK_MONOTONIC **of the roadsense host** at the instant the sample was
  valid (the measurement, not the send time); the same clock as every other roadsense message. Instead of it
  `age_ms` (since 1.1): the sample's age when it was sent. Neither: mapd stamps the message on arrival. Older than 5 s or more than 0.5 s in the future: rejected. A feeder
  that relays samples from another device subtracts the known transport delay, or omits the field.
* Unknown fields are ignored. Additions keep `api` 1; a change of meaning raises it.

`{"type":"gnss", ...}`: one raw fix of the receiver, unfiltered (1..10 Hz; roadsense does the filtering).

| field | unit | required | meaning |
|---|---|---|---|
| `fix` | | no (default `"3d"`) | `"none"`, `"2d"`, `"3d"`, `"dgps"`, `"rtk"`, `"dr"` (receiver's own dead reckoning). `"none"` = the receiver has no fix now: mapd starts dead reckoning at once instead of after `fix_timeout_s`; no other field is needed |
| `lat`, `lon` | degrees WGS84 | yes unless `fix` is `none` | position of the antenna |
| `hacc_m` | m | recommended (default 10) | horizontal accuracy as a 68 % radius (Android `Location.getAccuracy()`); fixes above 100 m are ignored |
| `speed_mps` | m/s | no | speed over ground, >= 0 |
| `speed_acc_mps` | m/s | no | 68 % accuracy of the speed |
| `course_deg` | degrees, clockwise from true north, 0..360 | no | direction of travel over ground; used only at `speed_mps` >= 2.5 and not in reverse |
| `course_acc_deg` | degrees | no | 68 % accuracy of the course |
| `sats`, `hdop`, `utc_ms` | | no | satellites used, HDOP, UTC time of the fix in Unix ms (reported back in `input.gnss`, not used by the filter) |
| `alt_m`, `vacc_m` | m | no | accepted, not used |
| `source` | string | no | tag of the receiver, e.g. `"headunit"`, `"phone"` (reported back) |

`{"type":"motion", ...}`: the car's own motion (5..50 Hz, 10..20 Hz recommended).

| field | unit | required | meaning |
|---|---|---|---|
| `speed_mps` | m/s | yes (since 1.1: a message without it is stored and joined to the next one with a speed, VEHICLE_API.md 5.2) | vehicle speed, magnitude (>= 0), from the wheels |
| `gear` | `"P"`, `"R"`, `"N"`, `"D"`, `"B"` | no | `"R"` makes the speed negative along the heading |
| `reverse` | bool | no | the same as `gear` `"R"` for cars without a gear signal |
| `yaw_rate_rps` | rad/s, positive = turning left (counter-clockwise seen from above, ISO 8855) | no | from a gyroscope / yaw-rate sensor |
| `wheel_speeds_mps` | m/s, `[front left, front right, rear left, rear right]`, >= 0 | no | yaw rate is derived from the rear pair and `track_m` |
| `steering_wheel_deg` | degrees of the steering wheel, positive = left | no | yaw rate is derived with `steering_ratio` and `wheelbase_m` (kinematic bicycle) |
| `turn_signal` | `"off"`, `"left"`, `"right"`, `"both"` | no | the turn-signal lamps as the driver set them (since 2026-11-17). Relayed in `ego`; the api's road geometry uses it as the driver's intent at the next junction. A host that cannot read the signal leaves the field out |
| `source` | string | no | tag, e.g. `"prius-can"` (reported back) |

Yaw rate is taken from the first of `yaw_rate_rps`, `wheel_speeds_mps`, `steering_wheel_deg` that is
present (and, since 1.1, of a quality roadsense acts on); with none of them the heading is held between
fixes. While the car stands the yaw rate is zero. Since 1.1 also `accel_long_mps2`, `accel_lat_mps2` and a
`quality` object per message (VEHICLE_API.md 4, 5.2). `motion` without any `gnss` never makes a
position.

`{"type":"reset", "reason":"..."}` (`reason` optional, a string, reported back): **forget the position**. The
filter, the last fix and the match are dropped; the next `map` message is `state: "no_source"`,
`available: false`, `reason: "no_position"`, `position: null`, and stays so until a new fix arrives (which
starts the filter afresh, wherever it is). Motion input keeps being accepted and relayed; the odometer keeps
counting. Without it a position lives on by dead reckoning for as long as vehicle motion flows (up to 900 s /
25 km), which is right for a tunnel and wrong after a test session: a feeder sends `reset` when what it fed
so far must not be presented any longer (a synthetic drive ended, the receiver was swapped). A feeder that
never sends it gets the behaviour of before.

**Synthetic input.** A `source` tag that is `synthetic` or starts with it (any case; `tools/synth_drive.py`
uses it) marks made-up input. mapd keeps the two kinds apart on its own: a `gnss` message (a fix, or
`fix: "none"`) from a real receiver while the position was fed by a synthetic one, or the other way round, is
an implicit `reset` first (`last_reset.reason` "gnss source changed from synthetic to real"). And
`input.synthetic` in the `map` message is true when the current position was fed by a synthetic fix, or
carried on by synthetic motion, at any time since the last reset: a consumer that must not act on a made-up
place checks this one flag.

**`{"type":"ego", ...}` on `<rundir>/ego.sock`** (output; since 2026-11-15): every vehicle `motion` message
mapd accepted, passed on as it was measured, at most `ego_hz` (20) per second; a change of `standstill` is
never held back by that limit. It exists because the car's own speed is needed at the feeder's rate by
consumers other than the position filter (the api's lead tracker, below), and `map` comes twice a second.
Nothing is sent without a feeder; vision odometry is never relayed. `mapd.sock` carries `map` messages only,
as before.

```
{"type":"ego","seq":812,"ts_mono_ns":<the motion message's time>,"ts_real_ns":<int>,"api":1,
 "speed_mps":3.21,                 // signed along the heading: negative in reverse
 "standstill":false,               // speed_mps is exactly 0 and, when wheel speeds were sent, every wheel is exactly 0
 "gear":"D","reverse":false,       // gear: null when the feeder sends none
 "wheel_speeds_mps":[3.2,3.2,3.2,3.2],   // as fed, or null
 "yaw_rate_rps":0.004,             // as fed or derived (wheels, steering angle); positive = turning left; null = unknown
 "yaw_source":"wheel_speeds",      // since 1.1: "yaw_rate" | "wheel_speeds" | "steering" | null
 "turn_signal":null,               // as fed (in `controls` or `motion`): "off" | "left" | "right" | "both"; null when unknown
 "source":"vehicle","tag":"prius-can"}
```
Since 1.1 also `steering_wheel_deg`, `accel_long_mps2`, `accel_lat_mps2` when the feeder sends them, and the
yaw rate is the one in use: a gyroscope's reading minus its standstill bias, times roadsense's scale estimate
when the feeder declared it `uncalibrated` and the estimate has converged (VEHICLE_API.md 9).

**`{"type":"vehicle", ...}` and `{"type":"objects", ...}` on `<rundir>/vehicle.sock`** (output; since
2026-11-17): the car as the host describes it: every signal of the vehicle input API with its age, quality
and source, the vehicle description, the feeders, the range sensors and roadsense's calibration estimates,
with every `map` message and at once when a switch-like signal changes; and every scan of a range sensor,
converted to the camera's frame when the mounts are known. The last `vehicle` message is sent to a client
when it connects. Message reference: VEHICLE_API.md 8. `ego.sock` and `mapd.sock` carry what they carried
before.
`standstill` is as good as the car's sensors: wheel-speed sensors read zero below some creep speed (the feeder
knows which), so "exactly zero" means "below the sensor's threshold".

**Filter** (`posfilter.py`): an extended Kalman filter over east, north, heading, speed and yaw-rate bias in a
local tangent plane. Yaw rate is the control input, speed a measurement, GNSS position / speed / course are
measurements; a fix more than 99.9 % unlikely is dropped, five such fixes in a row restart the filter on the
fix. Without `course_deg` the first heading comes from the displacement between fixes. Motion sources, in
order: vehicle `motion` (fresh for `motion_timeout_s` 1.5 s), the normal source; else, **only when switched
on** (`"vision_odometry": true` in `mapd.json`, or `--vision-odometry`), **vision odometry**: modeld's `road`
message (`v_ego`, `pose.yaw_rate`; read from `modeld.sock`, at most 10 Hz, not parsed while vehicle motion
arrives); else constant velocity for `coast_s` 5 s. Vision odometry is **off by default** (since 2026-11-15):
its speed scales with the camera calibration (focal length, pitch, height), and an uncalibrated camera would
dead-reckon the position at a wrong speed. Switch it on once the camera is calibrated. With it off and no
vehicle motion, a lost fix means 5 s of coasting and then `unknown`.

**States** (`state`):

| state | meaning | leaves when |
|---|---|---|
| `no_source` | no usable fix since start or since the last `reset` | a fix arrives |
| `good` | a fix was used within `fix_timeout_s` (3 s) | no fix for 3 s, or `fix: "none"` |
| `dead_reckoning` | the position is carried by motion input and the map | a fix arrives; or it runs out: no motion input for more than `coast_s`, longer than `dr_max_s` (vehicle 900 s, vision 120 s), farther than `dr_max_m` (vehicle 25 km, vision 3 km), or `hacc_m` above `dr_max_hacc_m` (150 m) |
| `unknown` | the fix is lost and dead reckoning ran out; `position` is null | a fix arrives (the filter restarts on it) |

`available` is true only in `good` / `dead_reckoning` with a pack loaded and a matched road; otherwise
`reason` is `no_pack`, `no_position`, `position_unknown` or `off_road`, and `match` / `road` /
`speed_limit` / `ahead` are null. While dead reckoning with a confident match the matcher feeds the road
back to the filter (cross-track position, 4 m; road bearing, 6 degrees, not while turning; only a match
within 10 m and 10 degrees of the filter is fed back, so a wrong branch at a fork cannot drag the position
with it), which is what holds the position in a tunnel; along the road the uncertainty grows by 2 % of the distance travelled
(`filter.odo_scale_sigma`, the odometer scale is not calibrated) until the next fix, so `hacc_m` reaches
`dr_max_hacc_m` after about 7.5 km without a fix.

**Map matching** (`matcher.py`): every 0.45 s the edges within 35..120 m are scored by distance, by the
heading against the edge bearing (one-way edges only in their direction), a class prior (service and
restricted roads lose a tie) and a transition term: no penalty for the current edge and for edges reachable
from it along the graph within the distance travelled, a penalty for everything else. A road that is not
connected to the current one wins only when it fits far better at once or clearly better over 250 m, so a
parallel road does not capture the match during a multipath excursion and a wrong match corrects itself.
At a fork the continuation of the same road (same class and ref or name) is followed freely, a branch
(an exit ramp, a side street) has to be clearly nearer; the alternatives of the last fork stay reachable
for 150 m.
`match.confidence` is the winner's share among the candidate roads, scaled by how well the winner fits.

**Message** (2 per second, `--hz`; the last one is sent to a client when it connects):

```
"api": 1,
"state": "good", "available": true, "reason": null,
"pack": {"region":"poland","osm_timestamp":"2026-11-14T21:20:41Z","built":"...","format":1},      // null without a pack
"input": {"gnss": {"source":"headunit","fix":"3d","age_s":0.4,"hacc_m":4.0,"sats":14,"hdop":0.8,
                   "used":812,"ignored":0,"rejected":2,"resets":0},                                // null before the first gnss message
          "motion": {"source":"vehicle|vision|none","tag":"prius-can","age_s":0.03},
          "errors": 0, "last_error": null,
          "synthetic": false,                                   // the position was fed by input tagged synthetic since the last reset
          "resets": 0, "last_reset": null},                     // or {"reason":"...","age_s":12.3}: `reset` messages and source changes
"position": {"lat":52.1512345,"lon":21.0456789,"heading_deg":93.2,"heading_acc_deg":1.4,"speed_mps":22.31,
             "hacc_m":2.1,                 // RMS radial error of the filter, m
             "confidence":0.96,"source":"gnss|dead_reckoning","fix_age_s":0.4,"dr_distance_m":0.0,
             "motion_source":"vehicle","odometer_m":18234.5},                                      // null in no_source / unknown
"match": {"matched":true,"confidence":0.97,"ambiguous":false,"distance_m":1.8,"heading_diff_deg":0.6,
          "lat":..,"lon":..,"bearing_deg":92.6,"edge":"11bb1a2c:531","dir":1,"offset_m":212.4,"candidates":3},
"road": {"class":"primary",               // motorway, motorway_link, trunk, trunk_link, primary .. tertiary(_link), unclassified, residential, living_street, service
         "name":"Wrocławska","ref":"DK46","oneway":false,
         "lanes":{"total":2,"forward":1,"backward":1,"source":"tagged|tagged_total|assumed"},     // forward = the car's direction
         "tunnel":false,"bridge":false,"covered":false,"lit":true,                                 // lit: true / false / null (not tagged)
         "surface":"asphalt","roundabout":false,"link":false,"motorroad":false,"expressway":false,"dual_carriageway":false,
         "access_restricted":false,"toll":false,"parking":false,"forest":false,"layer":0,
         "built_up":{"value":true,"source":"tagged|landuse|class"}},
"speed_limit": {"limit_kmh":70,"source":"maxspeed|implicit",
                "rule":null,               // implicit: built_up, outside, outside_dual_2x2, residential_zone, expressway, expressway_single, motorway, link (null limit)
                "detail":"tagged|sign|zone" /* maxspeed */ or "built_up:tagged|landuse|class" /* implicit */,
                "confidence":0.9,"variable":false},
"ahead": {"horizon_m":1500.0,"path_confidence":0.62,                                               // along the most likely path
          "features":[{"kind":"traffic_signals","dist_m":230.0,"confidence":0.9}],                 // also crossing, crossing_signals, level_crossing, tram_crossing, stop, give_way, mini_roundabout, speed_bump, toll_booth, motorway_junction
          "junctions":[{"dist_m":412.0,"kind":"junction|roundabout|roundabout_exit|dead_end|no_way_on","branches":3,
                        "turn_deg":-2.0,           // turn of the most likely path there, positive = right
                        "minor":false,             // every side branch is a service road
                        "confidence":0.62}],
          "tunnel":{"in_tunnel":false,"entry_m":840.0,"length_m":2310.0,"exit_m":3150.0},          // in a tunnel: exit_m = distance to its end (the path is followed past the horizon for it)
          "built_up_change":{"dist_m":655.0,"to":false,"source":"landuse","confidence":0.4},
          "speed_limit_changes":[{"dist_m":655.0,"limit_kmh":90,"source":"implicit","rule":"outside","detail":"built_up:landuse","confidence":0.4}],
          "road_change":{"dist_m":1210.0,"class":"secondary","name":null,"ref":"414"},
          "shape":{"s":[0,20,..600],"x":[..],"y":[..],"curvature":[..]},   // road ahead beyond camera range; x forward, y right (modeld's frame), origin at the matched position; curvature 1/m, positive = bending right
          "curves":[{"start_m":300.0,"apex_m":380.0,"end_m":480.0,"min_radius_m":150.0,"direction":"left","angle_deg":-88.0}],   // radius below 500 m, at most 4
          "junction_view":{...},           // the first major junction or roundabout within 400 m on the path, or null (below)
          "geometry":{...}},               // the path, its side roads and the first roundabout as geometry (below)
"proc": {"step_ms":0.9,"cpu_pct":0.8,"feeders":1,"tile_loads":14}
```

**Junction view** (`ahead.junction_view`, lane L16; `context.junction_view`): what a consumer needs to draw
the next junction or roundabout as a picture, computed from the pack's graph (no lane-level data, no pack
rebuild). It is made for the first junction on the most likely path within 400 m that has at least one side
branch that is not a service road; junction nodes within 35 m of each other along the path are one junction
(the two carriageways of a crossing dual carriageway).

```
{"kind": "junction",
 "dist_m": 171.0, "confidence": 0.58,            // along the path to the (first) junction node; path confidence past it
 "control": "signals|give_way|stop" | null,      // from the point features on the path 60 m before .. 35 m behind the node
 "approach": {"class":"secondary","name":"..","ref":null,"lanes":2,"lanes_source":"tagged","oneway":true},
 "arms": [ {"angle_deg": -91.0,                  // direction of the arm, measured 25 m out, relative to the direction of travel into the junction: 0 = straight on, positive = right
            "class":"secondary","name":"1 Maja","ref":null,
            "exit": true,                        // can be driven into (false: a one-way street towards the junction)
            "entry": true,                       // traffic comes out of it
            "path": true,                        // the most likely path leaves here
            "minor": false,                      // a service road or restricted access
            "lanes": 1} ],                       // tagged lanes in the leaving direction, null when not tagged
 "ring": null}
{"kind": "roundabout", "dist_m": 52.0, ..., "control": "give_way",
 "arms": [ {"angle_deg": 124.0,                  // where the arm meets the ring, seen from its centre: 180 = where the car enters, positive = to the right
            ..., "approach": false,              // the arm the car comes from
            "exit_no": 1} ],                     // exits in driving order from the entry (service roads and the way back are not counted)
 "ring": {"radius_m": 8.2, "lanes": null, "exits": 3, "path_exit_no": 3,        // the exit the most likely path takes (same road number or name, else the direction of the approach)
          "exit_dist_m": 71.0,
          "on_ring": false, "car_angle_deg": null}}
```
The carriageways of one road that meet the ring within 45 degrees are one arm. While the car is on the ring
the view announced before the entry is repeated unchanged (`dist_m` 0, `on_ring` true, `car_angle_deg` = the
car's place on the ring in the same angles), so a picture does not turn under the driver; a position that is
first found on a ring gets a view with the car at 180 and no path exit. The view is a property of the most
likely path: with `confidence` near 0.5 the path, and therefore the highlighted arm, is a guess. Turn lanes
and turn restrictions are not in the pack. Cost: `look_ahead` with the view 0.53 ms on the laptop (Poland pack,
Opole centre, 660 positions); not yet measured on the phone.

**Road geometry** (`ahead.geometry`, lane L17; `context.geometry`): the road around the most likely path as
polylines, for a consumer that draws it. Frame of `shape`: x forward along the matched bearing, y right, metres,
origin at the matched position. The pack has no lane-level geometry: a road is its centre line and a lane
count. The api turns this into the car's frame and joins it with the camera (topic `roadgeom`, below); a
consumer should read that topic, not this object.

```
{"path": {"s":[0,4,..200,220,..600], "x":[..], "y":[..]},     // the most likely path: every 4 m up to 200 m, every 20 m up to 600 m
 "lanes": [{"dist_m":0.0,"forward":1,"backward":1,"source":"tagged"}],   // lane counts along the path, one entry per change
 "branches": [ {"dist_m": 171.0,            // path distance of the node it leaves from (null: an arm of the ring beyond the path's exit)
                "pts": [[171.0,0.4],[171.3,6.4],..],   // from the node outwards, at 0, 6, 12, 20, 32, 45 m of the side road (or to its end)
                "class":"tertiary","minor":false,      // minor: a service road or restricted access
                "exit":true,"entry":true,"oneway":false,   // exit: can be driven into; entry: traffic comes out of it
                "lanes":2,"lanes_source":"assumed",
                "ring":false,               // an arm of the roundabout: then also
                "ring_m": 203.0,            //   its place along the ring as a path distance (driving order)
                "path": false} ],           //   the path itself uses this road (entry or exit)
 "ring": {"centre":[214.0,0.3],"radius_m":14.1,"lanes":null,"on_ring":false,
          "entry_m":200.0,"exit_m":244.0}}  // where the path enters and leaves it; entry_m null on the ring
```
Side roads are listed for the junction nodes of the path within 260 m (at most 20), the ring is the first
roundabout within that distance. On a ring the path takes the exit that was announced before the entry (the
most likely path of a position first found on a ring has no reason to leave it); the junction view keeps its
picture the same way.

Speed limit: the `maxspeed` tag of the matched direction when present, else the Polish statutory limit
(`mapd/roadsense_mapd/limits/pl.json`: built-up 50, living street 20, outside 90, dual carriageway with two
lanes per direction 100, expressway 120 / single carriageway 100, motorway 140), `source` says which. An
expressway is a way with `motorroad=yes` or a `highway=motorway` whose refs are all S-numbers (`road.expressway`). The
built-up state comes from a tag on the way (`maxspeed`, `zone:traffic`, `source:maxspeed` = `PL:urban` /
`PL:rural`), else from landuse polygons (inferred, confidence 0.5); `built_up.source` says which. The most
likely path follows, at every junction, the continuation with the same ref, name and class and the smallest
turn; `path_confidence` is the product of its choices, and every `ahead` entry carries the confidence of the
path up to it. Turn restrictions are not in the pack.

Config `/etc/roadsense/mapd.json` (`ROADSENSE_CONFIG_DIR`), every key optional: `pack`
(`/var/lib/roadsense/maps/current.rsmap`; env `ROADSENSE_MAP_PACK`; the file is checked every 10 s and a
new one, e.g. a re-pointed symlink, is loaded without a restart), `hz`, `vision_odometry` (default `false`,
above), `ego_hz` (default 20; the `ego` relay below, 0 = no `ego.sock`), `wheelbase_m`,
`steering_ratio`, `track_m` (nominal defaults of a compact car: 2.70 m, 19.1, 1.48 m; the same keys in a
`"vehicle"` object, with the rest of the vehicle description, take precedence, and a feeder's `vehicle`
message over both: VEHICLE_API.md 5.6), `use_calibration` (default `true`), `gnss_lag_s` (default 0),
`calibration_file` (`/var/lib/roadsense/vehicle-calibration.json`; `""` = estimates are not kept between
runs), `horizon_m`, the timeouts above, `country`. CLI: `python3 -m roadsense_mapd --help`. Offline and live drivers:
`tools/synth_drive.py` (a synthetic drive over a pack: route, GNSS noise, tunnel and window dropouts, bias,
motion as yaw rate / steering angle / wheel speeds).

```sh
python3 -m unittest discover -s mapd/tests -t mapd      # filter, matcher, states, limits, look-ahead, input, daemon; drives on the Poland pack when one is built
PACK=~/roadsense-maps/packs/poland-YYYYMMDD.rsmap
tools/synth_drive.py --pack $PACK --demo warsaw-s2-tunnel --offline          # timeline + summary, no daemon
# whole stack without hardware: fake camera daemons, the real mapd, the api, a synthetic drive in real time
tools/fake_daemons.py --rundir /tmp/roadsense --no-mapd &
ROADSENSE_CONFIG_DIR=/tmp/roadsense PYTHONPATH=mapd python3 -m roadsense_mapd --rundir /tmp/roadsense --pack $PACK &
(cd api && python3 -m roadsense_api --rundir /tmp/roadsense --ring /tmp/roadsense/roadsense-frames --port 8080) &
tools/synth_drive.py --pack $PACK --demo warsaw-s2-tunnel --rundir /tmp/roadsense     # then open http://localhost:8080/
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
| `GET /v1/map` | latest `map` message of mapd (position state, matched road, map speed limit, look-ahead); 503 while mapd is not running |
| `GET /v1/fused` | map and camera combined per fact with provenance (topic `fused`, below) |
| `GET /v1/lanes` | the tracked road geometry (topic `lanes`, below): four lane lines and two road edges with a presence that has hysteresis and a filtered geometry; 503 before the first `road` message |
| `GET /v1/lead` | the tracked lead vehicle (topic `lead`, below): distance, relative speed, time gap, standing flag, `departed` cue, with the raw vision value alongside; 503 before the first `road` message |
| `GET /v1/view` | the display channel (topic `view`, below): smoothed objects and lines with ids, velocities and fades for a consumer that draws; 503 before the first `road` message |
| `GET /v1/vehicle` | the car as the host describes it: mapd's `vehicle` message (every signal with age, quality and source, description, calibration estimates, feeders) plus the last scan of every range sensor (`objects`); 503 while mapd is not running. `docs/VEHICLE_API.md` 8 |
| `POST /v1/vehicle`, `POST /v1/position` | vehicle input API over HTTP (body: one message, a JSON array, or JSON lines), forwarded to mapd's `position.sock`; answers `{"api":1,"accepted":n,"rejected":[...]}`, 503 without mapd |
| `GET /v1/camera` | latest `camera` message + ring header fields |
| `GET /v1/health` | per-daemon: connected, last message age, fps, latency (`model_ms`, `detect_ms`, `env_ms`), current `env` labels, `speed_limit` summary, `rules` summary (short strings: in effect / recent / ahead), `lights_ms` and the ego light, `map` (position state, availability, pack date, matched road, map limit, mapd CPU share), `fused` (limit and source, place, lanes, tunnel) and `vehicle` (feeders, fresh signals, yaw source, range sensors) summaries; CPU temp, battery |
| `GET /v1/frame.jpg` | current preview JPEG |
| `GET /preview` | MJPEG stream with lane lines, path, leads, sign boxes and traffic lights (box in the state colour, the ego light thick with its distance) drawn on top; HUD lines IN EFFECT / RECENT / AHEAD from the `rules` topic |
| `GET /ws` | WebSocket; every message from every daemon, as-is (`lights` with the api's annotation, `signs` with its context flags), plus the api's own `speed_limit`, `rules`, `fused`, `lead`, `lanes`, `roadgeom` and `view`; mapd's `ego` (the car's motion, while a feeder sends it), `vehicle` and `objects` pass through like every daemon message; `?topics=road,signs,env,speed_limit,rules,lights,map,fused,lead,lanes,roadgeom,view,ego,vehicle,objects` filters. Per road frame the order is `road`, `lead`, `lanes`, `roadgeom`, `view`: a consumer that needs one frame complete waits for `view` |

api never touches the camera or the models; it is a fan-out and overlay layer, plus the derived
topics below (`speed_limit`, `rules`, `fused`, `lead`, `lanes`, `roadgeom`, `view`) and an optional drive recorder.

**Measurement topics and the display channel (roadsense 2.0).** Every topic except `view` is measurement-grade:
it says what was measured for its frame, adds no display latency, invents no value, and marks a held or
predicted value as such (`coast`, `held`, `meas_age_s`) with a time-out. A consumer that decides anything
(an alert, a future driving function) reads those. `view` is derived from them for consumers that draw:
smoothed, identity-keeping, with fades and velocities; it never feeds back into the measurement topics.

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

### api-derived topic: `{"type":"fused", ...}`
`api/roadsense_api/fusion.py` combines the map layer (`map`) with the camera perception (`speed_limit`,
`env`, `road`), one fact at a time; every fact names its `source` and carries the unfused inputs
(`vision`, `map`) next to the result. The vision topics are not modified. About 2 per second on `/ws`;
`/v1/fused`. The map is used only when the `map` message is younger than 3 s, `available`, and
`match.confidence` >= 0.5; otherwise `map.available` is false with a `reason` (`no_mapd`, `stale`,
`no_pack`, `no_position`, `position_unknown`, `off_road`, `low_match_confidence`) and every fact is its
vision value: without a position source the output is vision only.

Match-confidence hold (since 2026-11-15): the matcher's confidence dips under 0.5 for a message or two where a
side road branches off while the matched road stays the same, and every fact (the speed limit that consumers
show and act on above all) would fall back to vision for that moment. A `map` message with a low confidence is
still used when it is otherwise available and fresh, it is the same road as the last message matched with
confidence (the same `match.edge` and `dir`, or the same `road.class` with the same `name` / `ref`, which must
not both be empty), its map speed limit (value, source, rule) is unchanged, it comes within 2 s
(`match_hold_s`) of that message and is at most the third low one in a row (`match_hold_msgs`). The facts are
then computed with that message's last good confidence and `map.match_hold` =
`{"messages":1,"age_s":0.5,"confidence":0.9}` is added (`match_confidence` stays the true value). It never
holds across a change of the matched road or of its map limit, an unavailable map (no position, off road) or a
stale one; once a hold has run out or been refused, only a confident message starts a new one.

```
"map": {"available":true,"state":"good","reason":null,"age_s":0.2,"match_confidence":0.97},   // + "match_hold":{..} only while held
"speed_limit": {"limit_kmh":40,
                "source":"sign|sign_held|map|map_implicit|vision_default|unknown",
                "confidence":0.8,"rule":null,"sign":"B-33","track_id":164,
                "agreement":"agree|conflict|vision_only|map_only|none",   // vision value against map value
                "note":null,"variable":false,
                "vision":{"limit_kmh":40,"kind":"sign","sign":"B-33","rule":null,"confidence":0.71},
                "map":{"limit_kmh":70,"source":"maxspeed","rule":null,"detail":"tagged","confidence":0.9}},
"built_up": {"value":true,"source":"sign|map_tagged|map_class|map_landuse+env|map_landuse|env|none","confidence":0.7,
             "vision":{"value":true,"source":"env","confidence":0.4},"map":{"value":true,"source":"landuse"}},
"place": {"label":"tunnel","confidence":0.93,"source":"fused|fused(map)|fused(vision)|map|vision|none",
          "scores":{...},"vision":{"label":"city_street","confidence":0.5},"map":{"label":"tunnel","prior":{...}}},
"lanes": {"count":4,"forward":2,"backward":2,          // lanes of the road, own direction, oncoming
          "ego_index":0,                               // 0 = leftmost lane of the own direction; null = unknown
          "oncoming_left":true,                        // the lane to the left is oncoming traffic; null = unknown
          "source":"map:tagged+vision","confidence":0.8,
          "vision":{"lane_to_left":true,"lane_to_right":true},"map":{"total":4,"forward":2,"backward":2,"source":"tagged"}},
"tunnel": {"in_tunnel":true,"exit_in_m":640.0,"entry_in_m":null,"length_m":null,"source":"map|vision|map+vision|none",
           "vision":{"in_tunnel":true,"confidence":0.9},"map":{...}},
"lead": {"present","mode","track_id","dist_m","dist_std_m","lateral_m","v_rel_mps","v_lead_mps","a_lead_mps2","time_gap_s","ttc_s",
         "standing","departed","source","vision":{...}},   // the `lead` topic's latest message, shortened (below); null when it is older than 1 s. Added 2026-11-15
"road": {"class","name","ref","oneway","bridge","lit","surface","roundabout","link","motorroad","expressway","dual_carriageway","toll"},   // null without the map
"ahead": {"horizon_m","path_confidence","features","junctions","curves","speed_limit_changes","built_up_change","road_change"},   // as in `map`; null without the map
"position": {"lat","lon","heading_deg","speed_mps","hacc_m","confidence","source"}                                               // null without the map
```

Rules:

* **Speed limit.** (1) A sign the camera reads (`speed_limit.source.kind` `sign`) overrides the map from the
  moment it is read. (2) With the map available that sign limit is then *held by the map*: it stays in
  effect (`sign`, or `sign_held` once the vision engine has dropped it by its distance cap or its junction
  proxies) until the car has passed a real junction of the road graph (a node with a side road that is not a
  service road; counted from 20 m past the sign, because a sign is read before it is reached and often stands
  just after a junction; not on motorways and expressways, where a limit lasts until it is cancelled), until
  the tagged map limit changes to a third value at least 100 m after the sign (only from a match that is not
  ambiguous and at least 0.6 confident, and once the new limit has held for 40 m: a match that flips between two
  roads is not a change of the limit; on the first drive with a position it ended a sign 24 times in six minutes), until a tagged or signed
  built-up boundary, or 5 km. (3) Otherwise the tagged map limit (`map`), else the implicit map limit
  (`map_implicit`, with the built-up state from an area sign when the camera saw one and the pack only
  inferred it), else the vision engine's default (`vision_default`), else `unknown`. Without the map the
  vision engine decides alone, as before.
* **Built-up area.** Area sign seen by the camera > tag on the way > road class > landuse (confidence 0.7
  when envd agrees, 0.45 when not) > envd.
* **Place.** envd's smoothed scores are multiplied by a prior from the matched road raised to the match
  confidence (tunnel x20 on a tunnel edge and x0.15 elsewhere; motorway / expressway -> highway; forest
  flag -> forest_road; built-up -> city_street or residential by class; parking aisle -> parking_lot or
  underground_parking; bridge x3) and renormalised. Without envd the map label alone is served (`map`).
* **Lanes.** Counts from the map (`road.lanes`, source marked: tagged per direction, tagged total, assumed).
  The ego lane comes from the model's four lane lines: a lane to the right exists when the tracker (topic
  `lanes`, below) reports the far-right and the right line present (`vision.tracked: true`; without a fresh
  `lanes` message the frame's probabilities decide as before: far-right >= 0.5 and right >= 0.3); likewise
  to the left. Nothing to the right = the rightmost
  lane; `oncoming_left` is false on one-way carriageways, true in the leftmost own lane of a two-way road.
* **Tunnel.** In a tunnel when the matched edge is one (then `exit_in_m` is the map distance to its end) or
  when envd's label is `tunnel` with confidence >= 0.5 (no distance).

### api-derived topic: `{"type":"lead", ...}`
`api/roadsense_api/lead.py` tracks the lead vehicle over time: one message per `road` message (so at the
model's rate, 10-14 per second on the reference phone), about 0.7 kB; `/v1/lead`. `road.leads[]` is not
modified and stays the network's frame-by-frame output; `lead` is what a consumer should draw or act on
(added 2026-11-15, docs/lanes/L14-lead.md).

Why: `road.leads[0].x[0]` jitters by about 1.5 % of the distance from frame to frame (0.08 m at 4 m, with
single frames 0.5-1 m off, e.g. under a wiper) and wanders by a few decimetres over seconds even behind a
standing car; shown rounded to metres it flips between two values several times a second whenever the true
distance is near a half metre.

Inputs: `road` (`leads[0]`: `prob`, `x[0]`, `y[0]`, `v[0]`, `x_std[0]`, `v_std[0]`; `v_ego`) and, when a
position feeder sends vehicle motion, mapd's `ego` messages (speed, standstill). Without `ego` (no feeder, no
mapd, or the last one more than 0.5 s from the frame) the model's own `v_ego` is the ego speed and the
message says `source: "vision"`: the tracker runs the same way on vision alone.

```
"present": true,                 // a lead is tracked; false: every numeric field below is null
"mode": "track|coast|none",      // see below (until roadsense 2.0 also "hold", which froze the value)
"track_id": 12,                  // changes when the tracked vehicle is replaced (cut-in, lead change, lost and found)
"age_s": 31.4,                   // of this track
"source": "vision+ego|vision",   // vision+ego: the ego speed / standstill came from vehicle motion
"dist_m": 3.61,                  // camera -> rear of the lead along x, the same quantity as road.leads[0].x[0]
"dist_std_m": 0.62,              // 68 %: the filter's uncertainty and the network's own x_std (its absolute accuracy) in quadrature
"dist_filter_std_m": 0.021,      // the filter's part alone: how steady the value is, not how true
"gap_m": 2.11,                   // only when `camera_to_front_m` is configured: dist_m minus the camera -> front bumper distance
"lateral_m": -0.07,              // y, right positive, smoothed
"v_rel_mps": 0.0, "v_rel_std_mps": 0.0,   // lead speed minus ego speed (negative = closing); d(dist)/dt of the track
"v_lead_mps": 0.0,               // the lead's own speed over the ground (along the ego heading)
"a_lead_mps2": 0.0,              // its acceleration: the derivative of v_lead within the filter, slow (about 1 s); a hint, not a measurement
"time_gap_s": null,              // (gap_m or dist_m) / ego speed; null below 1 m/s
"ttc_s": null,                   // (gap_m or dist_m) / closing speed; null unless closing faster than 0.5 m/s
"standing": true,                // both stand (standing regime of the filter; the value stays live)
"meas_age_s": 0.0,               // since the last measurement went into the track (> 0 while coasting)
"pending_change": false,         // the last frame(s) did not fit the track; a replacement is being considered
"prob": 0.99,                    // road.leads[0].prob of this frame (null when the frame had no usable lead output)
"departed": false,               // the lead moved off while ego still stands (below)
"departure": null,               // or {"age_s":0.4,"dist_at_stop_m":3.61,"moved_m":0.8}
"event": {"kind":"acquired|cut_in|changed|departed|gap_changed|lost|reset","ts_mono_ns":..,"track_id":12,"age_s":31.4},   // the last one; null before the first
"ego": {"speed_mps":0.0,"standstill":true,"source":"vehicle|vision","age_s":0.04,"tag":"prius-can",   // what the tracker used; age_s: distance in time between the frame and the nearest motion sample
        "brake":true},              // the brake pedal from the `vehicle` topic (controls.brake); null when no host sends it
"radar": null,                   // or {"sensor":"front_radar","id":17,"dist_m":31.4,"lateral_m":0.2,"v_rel_mps":-0.8,"age_s":0.03}: the range-sensor object used in this frame
"vision_scale": null,            // radar distance / camera distance, learned while both see the lead (radar fusion only)
"vision": {"dist_m":3.58,"dist_std_m":0.61,"lateral_m":-0.07,"v_lead_mps":0.0,"v_rel_mps":0.0,"a_lead_mps2":0.0,"prob":0.99}   // road.leads[0] of this frame, unfused; null without a lead output
```
`seq`, `ts_mono_ns`, `ts_real_ns` are those of the `road` message: the estimate refers to the frame, no
prediction to "now" is applied.

Estimator. State: distance `d`, the lead's speed `v_lead` and acceleration `a_lead` over the ground; the ego
speed is a known input: `d' = v_lead - v_ego`, `v_lead' = a_lead`, `a_lead` a random walk decaying over 3 s
(Kalman filter, jerk noise 30 (m/s3)2 s). So "both stand" and "the gap closes" are different states of the
model, and the relative speed is by construction the rate of change of the distance. Measurements per frame:
`x[0]` (sigma = max(hypot(0.10 m, 2 % of d), 0.2 x_std)) and the relative speed `v[0] - road.v_ego`, the
model's own pair (sigma = max(0.15, 2 v_std) + 0.1 d m/s: trusted near, little beyond 30 m where it is biased).
The model's `a[0]` is not used (it made the track lag). Modes:

| mode | when | what |
|---|---|---|
| `track` | a lead with `prob` >= 0.5 for 2 frames | the filter. A frame whose distance is more than 4 sigma and more than max(0.6 m, 12 % of d) from the prediction is **not averaged in**: the track coasts and the frame starts a candidate. Frames that agree with the candidate replace the track after 0.15 s when it is nearer (`cut_in`) and after 0.5 s when it is farther (`changed`); a new `track_id` either way. A candidate that does not persist was an outlier or the model flickering between two vehicles: nothing between the two is ever reported |
| `track` with `standing: true` | ego stands (vehicle: `standstill` for 0.3 s; vision only: the model's `v_ego` under 0.15 m/s for 1 s) and the lead stands (estimate under 0.35 m/s, measured under 0.12 m/s) for 0.5 s | **the value stays live** (one update per frame; since roadsense 2.0, lane L19). The filter's model changes: the ego speed is exactly zero, the lead's speed is measured with the model's standstill speed output (sigma max(0.12, 0.5 v_std) m/s), the lead's jerk noise drops to 0.05, and the distance is a random walk scaled so the per-frame jitter is averaged over about 3 s (`stand_tau_s`); a frame more than 2.5 sigma off counts with a weight falling as 1 / residual (Huber: a wiper pass moves it little). A two-sided CUSUM on the model's lead speed (sigma as above, slack 1 sigma, 3.5 sigma-tenths-of-a-second: a lead rolling at 0.3 m/s is caught in about 0.25 s) runs before the distance gate and ends the regime at once (event `departed` when the lead moves away, `gap_changed` when it comes nearer); ego moving (a wheel turns) ends it on that frame. What a live value shows that the old hold hid: the network's slow wander of its close-range distance (a few decimetres over seconds behind a standing car); a display that rounds to metres should add a hysteresis (0.3 m is enough on the recorded drives) |
| `coast` | `prob` under 0.3 | prediction only, `meas_age_s` counting, for 0.6 s (2.5 s while standing); then `present: false`, event `lost` |
| `none` | no track | `vision` still carries the raw value |

`departed`: the standing regime ended because the lead moved away while ego still stands; true from that frame
until ego moves, both stand again, or 8 s. `departure.moved_m` is how far it has gone. On the replayed
rides it comes 0.25-0.95 s (median 0.45 s) after the lead starts to roll, when the raw distance has grown by about 0.3-0.8 m
(docs/lanes/L14-lead.md). It is information for a "the car ahead is leaving" hint, not a go signal.

Latency against smoothing: at short range the distance measurement is weighted heavily and the model's
relative speed feeds the filter directly, so on the rides the tracked distance trails a zero-phase reference
by about 10 ms under 40 m (135 ms beyond, where the model's speed is weighted little) while the gap closes faster than 2 m/s, with a third to a half less noise than the raw value; the
standing regime costs nothing while moving. Standing, on the car's recorded drives (5,400 s behind a standing
lead, docs/lanes/L19-view-channel.md): frame-to-frame jitter 0.009-0.021 m rms (raw 0.10-0.23 m), never constant
for more than 1.2 s (the old hold: up to 32 min), within 0.02-0.10 m rms of the camera's 2 s mean (the old
hold: 0.17-0.37 m). Constants: `DEFAULT_CFG` in `lead.py`, overridable per key in
`<config dir>/lead.json` (e.g. `{"camera_to_front_m": 1.6}`).

Range sensors (`{"radar_fusion": true}` in `lead.json`; **off by default**, built 2026-11-17 and tested on
synthetic radar only: no car with a radar feeds it yet). A host with a radar or lidar sends its object lists
over the vehicle input API (`objects`); mapd relays each scan with positions in the camera's frame when the
`vehicle` description has the mounts of the sensor and of the camera. The camera decides which vehicle is the
lead (a radar does not see lanes); the tracker then looks for the object of a scan at most 0.3 s from the
frame within max(3 m, 25 % of the distance) and 1.8 m laterally of the track (of the camera's lead when there
is no track yet). While one is associated, its range (sigma hypot(0.3 m, 1 %)) and closing speed (0.2 m/s)
are the frame's measurement in place of the camera's, `source` is `radar+vision+ego`, and `vision` keeps the
camera's raw value. The ratio of the two distances is learned (`vision_scale`, 5 s) and applied to the
camera's distance for a minute after the radar lost the object, so handing back to the camera is not a step.
When the camera loses the lead while the radar still has the object the track lives on the radar alone for 5 s
(`source: "radar+ego"`); a radar alone never starts a track. Without fresh scans, and always with the flag
off, the tracker is the vision tracker message for message. On the synthetic scenes (camera distance 15 %
long, radar noise 0.15 m): error under 0.5 m at 30 m against 4.5 m for the camera alone; no track restart
and under 1.5 m of error through a radar drop-out. Cost: 0.13 ms per frame with 16 objects against 0.09 ms
without (laptop).

Replay and measurement on recorded `road` lines: `tools/lead_replay.py ROAD.jsonl[.gz] [--ego surrogate]
[--every N] [--csv OUT]`.

### api-derived topic: `{"type":"lanes", ...}`
`api/roadsense_api/lanes.py` tracks the road geometry over time: one message per `road` message (10-14 per
second on the reference phone), about 2.5 kB; `/v1/lanes`. `road.lane_lines[]` / `road.road_edges[]` are not
modified and stay the network's frame-by-frame output; `lanes` is what a consumer should draw (added
2026-11-16, docs/lanes/L15-stability.md).

Why: a lane line's `prob` hovers around any fixed drawing threshold (at 0.3: 18-70 on/off changes of the four
lines per minute on the replayed rides at 10 Hz, nearly all of them spans shorter than a second), and the road
edges jump by 0.1-0.4 m from frame to frame (median; p95 0.4-1 m at 10 m), which moved everything a consumer
stands on them.

```
"x": [33],                         // the model's distances (X_IDXS), shared by all six lines
"lines": [ {"present": true,       // far left, left, right, far right: presence with hysteresis
            "mode": "track|coast|none",   // coast: present, no usable frame right now, geometry held
            "conf": 0.83,          // the filtered probability the presence is decided on
            "prob": 0.91,          // this frame's road.lane_lines[i].prob (null without a line in the frame)
            "age_s": 12.3,         // since the presence last changed
            "y": [33], "y_std": [33]} x4 ],   // filtered lateral offsets (m, right positive) and the filter's sigma; null when not present
"edges": [ {...} x2 ],             // left, right road edge, the same fields; `prob` is made from the network's y_std
"lane_to_left": true, "lane_to_right": false,   // both lines of the neighbouring lane are present
"lane_width_m": 3.42,              // between the inner lines over 6-30 m; null unless both are present
"prior": null,                     // or {"total":2,"forward":1,"backward":1,"source":"tagged","outer_left":"expected","outer_right":"unlikely"}
"source": "vision|vision+map",
"ego": {"speed_mps": 12.1, "yaw_rate": 0.004, "source": "vehicle|vision"}
```
`seq`, `ts_mono_ns`, `ts_real_ns` are those of the `road` message.

* **Presence.** `conf` is a 0.5 s low-pass of the frame probability; a line appears when `conf` reaches 0.45
  and disappears when it falls under 0.2 (about 0.35 s to appear at prob 0.9, about 0.9 s to go). While a
  present line gets no usable frame (prob under 0.2) its geometry is held (`mode: "coast"`), for 1.5 s at
  most. A road edge has no probability in the model output; its pseudo probability is 1 when the network
  places the edge within 0.6 m (`y_std` over 10-40 m) and 0 at 2.5 m (snow, a wide junction).
* **Geometry.** One scalar Kalman filter per line and distance, a random walk in the car's frame, with the
  network's own `y_std` as the measurement noise (0.3 x for a lane line, 1.0 x for a road edge, floors 0.05 /
  0.25 m): far points are smoothed harder than near ones and a road edge much harder than a lane line. A
  frame that does not fit the track (more than 4 sigma and 0.8 m over 5-45 m: the network relabelled its
  lines at a lane change or a junction) is not averaged in; after 0.2 s and two such frames that agree the
  track restarts on them, so nothing between two lanes is reported. An ego-motion compensation (slide by the
  distance travelled, turn by the frame's yaw) is implemented and off: on the rides it made the tracked lines
  trail the raw ones more.
* **Map prior** (soft, optional). With a `map` message that is available, younger than 3 s, matched with
  confidence >= 0.5 and a lane count that is tagged (not assumed), the count shifts the thresholds of the two
  outer lines: `unlikely` (a single-lane road: both; one lane in the own direction of a two-way road: the
  right one) needs `conf` 0.70 to appear and goes at 0.28, `expected` (the oncoming lane's far line on that
  two-way road) appears at 0.325 and goes at 0.12. The inner lines and the edges are never touched by the map,
  strong camera evidence always wins, and without a position source `prior` is null and every threshold is
  the neutral one. On the car there is no live GNSS fix yet, so this path has run on tests and with a constant
  synthetic prior only.

Constants: `DEFAULT_CFG` in `lanes.py`, overridable per key in `<config dir>/lanes.json`. Replay and
measurement on recorded `road` lines: `tools/lanes_replay.py ROAD.jsonl[.gz] [--every N] [--lanes-total N
--lanes-forward N] [--out LANES.jsonl]`.

### api-derived topic: `{"type":"roadgeom", ...}`
`api/roadsense_api/roadgeom.py` joins everything that is known about the shape of the road ahead into one
geometry in the car's frame (x forward from the camera, y right, metres): the camera's lane as far as the
camera knows it, the map's road beyond, the side roads of the junctions ahead and a roundabout as a ring. One
message per `lanes` message (10-14 per second), 1-4 kB; `/v1/roadgeom`. Added 2026-11-17
(docs/lanes/L17-road-geometry.md). It is for a consumer that draws the road; `lanes` stays the camera's own
result and `map` the map's.

```
"source": "vision" | "vision+map",
"lane_width_m": 3.4,
"vision": {"from": "lanes|line|plan|none", "reach_m": 99.2},     // what the camera's centre line is made of and how far it is used
"motion": {"source": "vehicle|vision", "speed_mps": 13.1, "yaw_rate_rps": 0.004},
"map": {"used": true, "why": null,           // why not: no_map, no_position, off_road, low_match_confidence, position_accuracy, no_geometry, stale, advance, no_fit, disagree, off_path
        "match_confidence": 0.93, "age_s": 0.31, "advance_m": 4.1, "hacc_m": 2.9, "state": "good",
        "synthetic": false,                  // the position is made up (map.input.synthetic): a consumer in a car must not draw the map part
        "fit": {"dy_m": 0.21, "dyaw_deg": -0.8, "rms_m": 0.12, "ok": true, "to_m": 50.0},   // this frame's fit, or {"held_s": 2.1}
        "align": {"dy_m": 0.18, "dyaw_deg": -0.7},     // the low-passed fit that is applied
        "way_offset_m": 1.7, "off_path_deg": 0.4, "ring_slide_m": 0.0},
"centre": {"s": [..], "x": [..], "y": [..],  // the ego lane's centre line as a polyline, s = distance along it
           "conf": [..],                     // camera part: from the tracker's sigma; map part: confidence of the path up to the point
           "src": "vvvvvvbbbbmmmm",          // per point: v camera, b camera blended into the map, m map
           "vision_to_m": 99.2,              // the camera's part ends here
           "end": "horizon|path_end|dead_end|no_way_on|intent"},
"branches": [ {"dist_m": 71.0, "pts": [[71.2,-1.7],..], "class": "tertiary", "minor": false, "exit": true, "entry": true,
               "lanes": 2, "ring": false, "side": "left|right", "way": false} ],     // way: chosen by the driver's turn signal
"ring": {"centre": [44.1,-1.6], "radius_m": 14.1, "lanes": null, "on_ring": false, "entry_m": 30.2, "exit_m": 74.3},
"junction": {"kind": "roundabout", "dist_m": 30.2, "control": "give_way", "confidence": 1.0},   // head of map.ahead.junction_view, distance advanced
"way": {"source": "map", "confidence": 0.58} | {"source": "intent", "turn": "right", "branch": 1},
"intent": null | {"turn": "right", "source": "turn_signal", "at_m": 33.4}
```
`seq`, `ts_mono_ns`, `ts_real_ns` are those of the `road` message. Without a usable map the message carries
the camera's part alone (`source: "vision"`, `map.used: false` and the reason, no branches, no ring).

* **Camera part.** The centre of the ego lane: the middle of the two tracked inner lines (`lanes`), one line
  and the lane width, or the model's planned path (`road.plan`) when no line is present. It is used as far as
  the tracker's own sigma stays under 0.9 m, 100 m at most.
* **Map part.** `map.ahead.geometry` arrives twice a second in the frame of the matched position. Between two
  messages it is carried by the distance driven (vehicle speed when a host feeds it): the car is taken to be
  on the most likely path, that far along it, with the heading of the position filter carried on by the yaw
  rate (`ego.yaw_rate_rps`, else the model's). On a ring the car is slid along the path, by at most 14 m, to
  where the ring runs in the car's direction: 5 m of position error along a ring of 14 m radius are 20
  degrees, and the heading is known to 2.
* **Joining.** What the map cannot know (where in the carriageway the car is; a few metres and degrees of
  error) is taken from the camera: the offset and the angle between the camera's centre line and the map's path
  over 4-50 m are fitted every frame, low-passed (0.8 s) and applied to all map geometry. The fit must stay
  within 7 m, 9 degrees and a residual of 0.5 m + 1.2 % of its length; the map part appears after 1 s of
  agreement and goes after 0.5 s without (`why: "disagree"`), or when the car's heading leaves the path's
  direction by more than 35 degrees (`off_path`). Without a camera centre to compare with (lines gone in a
  junction, on a ring) the last fit is held for 150 m. Side roads and the ring are moved from the way's centre
  line to where they are for a car in its lane (`way_offset_m`, from the lane count and the lane seen to the
  right). The centre line follows the camera up to `vision_to_m` (blended into the map over the last 15 m)
  and the map's path beyond; before a ring the camera's part ends 3 m before the entry.
* **Honesty.** `centre.conf` of a map point is the confidence of the most likely path up to it: it steps down
  at every junction where the way on is a guess, and a consumer should stop drawing where it gets low (the
  reference consumer stops at 0.5). `way.source` says whether the way through the next junction is the map's
  guess or the driver's signal.
* **Driver intent** (optional). With `ego.turn_signal` `left` / `right` (a host that reads the turn-signal
  lamps feeds it with `motion`), the nearest side road on that side within 150 m that can be driven into is
  the way on (service roads last): `branches[].way`, `way.source: "intent"`, and the centre line ends at that
  junction. On and before a roundabout `right` is the next exit; `left` is the first exit beyond the one the
  map's path takes, reached on round the ring (a convention of how drivers signal, not a rule). Without the
  signal nothing changes.

Cost on the laptop: 0.09 ms per message without a map, 0.36 ms with one, plus 0.17 ms per `map` message; on the phone: see
docs/lanes/L17-road-geometry.md. Constants: `DEFAULT_CFG` in `roadgeom.py`, overridable per key in
`<config dir>/roadgeom.json`. Tools: `tools/roadgeom_replay.py SESSION.jsonl.gz` (a recorded drive through the
tracker and the fusion), `tools/roadgeom_drive.py --network tour [--turn T0:T1:left|right] [--out REC]` (a
synthetic drive over a made-up network or a pack with a synthetic camera, compared with the truth).

### api-derived topic: `{"type":"view", ...}` (display channel, roadsense 2.0)
`api/roadsense_api/view.py`: one message per road frame (10-14 per second), after `lead`, `lanes` and `roadgeom`
of that frame; about 1.5 kB; `/v1/view`. Added 2026-11-18 (docs/lanes/L19-view-channel.md). For any consumer
that draws (a car display, a HUD, a dashboard): objects that keep their identity, move smoothly between two
detections, fade in and out, and carry a velocity so the consumer can extrapolate to its own render time.
Derived from the measurement topics, never fed back into them; nothing in it belongs to one display.

```
"x": [0, 5, 10, 15, 20, 25, 30, 40, 50, 60, 70, 80, 100, 120],   // distances of every line, m
"ego": {"speed_mps": 12.1, "yaw_rate_rps": 0.01, "source": "vehicle|vision"},   // yaw positive left
"objects": [ {"id": "lead:12" | "light:7" | "sign:31",   // stable while the measurement track lives
              "kind": "vehicle|traffic_light|sign", "role": "lead|ego|other",   // ego: the ego traffic light
              "x": 34.2, "y": 1.9,                        // car frame (x forward from the camera, y right), m
              "vx": -12.1, "vy": 0.0,                     // m/s relative to the car: x(t) = x + vx (t - ts)
              "sx": 2.1, "sy": 0.5,                       // 1 sigma of the drawn position (null: not known)
              "age_s": 4.2,                               // since the object appeared
              "meas_age_s": 0.12,                         // since its last measurement (> 0: predicted)
              "phase": "appear|steady|coast|leave",       // coast: no measurement for 0.5 s; leave: fading out
              "alpha": 1.0,                               // suggested opacity of the fade, 0..1
              "attrs": {...},                             // light: state, flashing, relevance, ego_state, held, amb, alt,
                                                          //   conf, agree, dist_m (the measured one); sign: cls, score;
                                                          //   lead: dist_m, dist_std_m, v_rel_mps, a_lead_mps2, time_gap_s,
                                                          //   ttc_s, standing, departed, mode (the `lead` topic's values)
              "box": [x0, y0, x1, y1]} ],                 // last image box (frame pixels) for lights and signs, else null
"lines": [ {"id": "lane:0".."lane:3" | "edge:0" | "edge:1", "kind": "lane_line|road_edge",
            "y": [..],                                    // at x, right positive; null beyond reach_m
            "dy": [..],                                   // lateral rate m/s per point
            "reach_m": 80.0, "conf": 0.83, "age_s": 12.3,
            "phase": "appear|steady|coast|shift|leave",   // shift: blending to a restarted track (other marking)
            "alpha": 1.0} ],
"path": {"y": [..]},                                      // the model's planned path at x, as measured
"road": null                                              // reserved: the fused road geometry (`roadgeom`, lane L20)
```
`seq`, `ts_mono_ns`, `ts_real_ns` are those of the `road` message; every position refers to that frame time.

| element | display filtering | price |
|---|---|---|
| traffic lights, signs | one Kalman filter per track in the car's frame, carried between detections by the car's own motion (`ego` speed and yaw rate when a host feeds them, else the model's) and corrected by each detection with its real uncertainty (light distance from the housing size: 25 % at 1 sigma, lateral 0.4 m + 3 %; sign distance from a 0.7 m sign: 30 %); a detection beyond 3 sigma is down-weighted; the drawn position eases to the filter's over 0.25 s at most 1 m/s + 10 % of the distance per second. `lights.lateral_m` (offset from the planned path) is turned back into the car's frame with the path. A light the provider only coasts is not a measurement. Leaves 1 s after its last detection (fade 0.3 s) or when it passes the camera | the drawn distance trusts odometry between detections; a biased housing-size distance is corrected over seconds, not frames |
| lead | the `lead` topic's track: carried by its relative speed, corrections eased over 0.1 s + 0.01 s per metre (0.15 s at 5 m, 0.5 s at 40 m); fade in 0.3 s, fade out 0.4 s at its extrapolated place after the track ends | drawn distance trails the tracker by up to the ease time at long range; `attrs.dist_m` is the tracker's value |
| lines | the `lanes` topic's geometry, two first-order stages of 0.1 s; a restart of the track on another marking (the near part moves by more than 0.8 m in one frame) is blended linearly over 0.4 s; fade 0.3 s when a line comes or goes; `reach_m` where the tracker's sigma passes 1 m | about one frame (75 ms) behind `lanes` |

On the 2026-11-17 day drive (29 min, `tools/view_replay.py`): the ego light's drawn distance step per frame after
the car's own motion 1.96 -> 0.12 m rms, steps over 2 m 129 -> 0, lateral 0.32 -> 0.07 m; lane lines and edges
that pop on or off without a fade 106 -> 2; the lead's step per frame after its relative speed 0.27 -> 0.09 m rms.
Cost: 0.15-0.18 ms per frame on the laptop.

### Drive recorder (optional)
`api/roadsense_api/recorder.py`, off unless `<config dir>/recorder.json` names a directory
(`{"dir": "/var/lib/roadsense/rec", "max_mb": 4000}`). Every api start opens a session
`<dir>/<YYYYmmdd-HHMMSS>/` with `messages.jsonl.gz` (the topics as they went out: `road` reduced to lane
lines, road edges, `leads[0]`, plan x / y / z, speed and pose; `lights`, `signs`, `env`, `speed_limit`,
`rules`, `fused`, `map`, `ego` at 10 Hz, `vehicle`, `objects` at 10 Hz, `roadgeom` at 2 Hz, `calib`, one
`camera` message a second) and `frames/<seq>.jpg`,
camerad's preview JPEG once a second (copied, not re-encoded). About 30 kB/s of messages plus the JPEGs
(0.2-0.5 GB per hour); the oldest sessions are deleted above `max_mb`; a write error stops the recording, not
the api. The file is flushed and synced every 5 s (a host that loses power keeps all but the last seconds;
on the first recorded drive a hard reset cost the last half minute), and a session that was opened on a
stale wall clock is renamed to its true start time once the clock is set. `/v1/health` carries `recorder` (`dir`, counters, `failed`). The file replays through
`tools/lanes_replay.py` and `tools/lead_replay.py`, the frames through roadsense-signs `lights_replay` /
`signd_replay`. It exists because no drive of the car had its perception output on disk (the car backend's
trip log holds vehicle buses only), so nothing seen on the road could be measured afterwards.

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
fetched with `scripts/fetch-models.sh`. mapd is Python (`py3-numpy`), installed to
`/usr/local/lib/roadsense/mapd` with `roadsense-mapd.service`; its map pack is not part of
the tree: it is built on a dev box and copied with `scripts/roadsense-maps install` to
`/var/lib/roadsense/maps/current.rsmap` ([MAPS.md](MAPS.md)). A sign provider installs its own unit
with `PartOf=`/`WantedBy=roadsense.target`; the target does not require it.
Every unit runs in `roadsense.slice` (`systemd/roadsense.slice`: CPUWeight 100,
no memory throttling, MemoryMax 4608M as an OOM backstop only) so whatever else
the machine does stays responsive while the stack saturates the GPU and the big
cores; a sign provider puts `Slice=roadsense.slice` in its unit too. A host may
add a slice drop-in (e.g. `AllowedCPUs=`) to keep the stack off cores it
reserves. The API port and bind address are `ROADSENSE_API_PORT` /
`ROADSENSE_API_HOST` (`/etc/roadsense/api.env`, written by
`API_PORT=N API_HOST=ADDR scripts/install-phone.sh`; defaults 8080 / 0.0.0.0;
a loopback bind also removes the nftables opening).

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
* **API front**: on that machine the API listens on `127.0.0.1` only
  (`ROADSENSE_API_HOST`); the car backend re-publishes the HTTP/WS endpoints
  to its own clients (`/api/v1/roadsense/...`), over HTTP, without importing
  any roadsense code;
* **cores**: the host reserves cpu3 for its backend with a
  `roadsense.slice.d` drop-in (`AllowedCPUs=0-2,4-7`); `--prep-cpus 0-3` /
  `--cpus 4-7` masks are intersected with that set by the kernel. What runs
  where on the reference phone (lane L21, measured shares in
  `docs/lanes/L21-budget-and-models.md`): the big cores carry what must be
  fresh and cannot go to the GPU (modeld's GPU dispatch and CPU layers, the
  sign classifier, the light detector); the little cores carry frame decode,
  modeld's frame preparation, envd, mapd and the api; the GPU carries the road
  model and the sign detector and is full in time (91 %), so a new GPU job
  costs road runs. The daemons name their threads (`md-prep`, `md-prefetch`,
  `md-main`, `sg-signs`, `sg-lights`) for per-stage accounting
  (`tools/budget/measure.py`);
* **camera power**: the host switches the USB hub that carries the DVR camera
  (powerbox relay ch4) on before the start and off after the stop;
  `camerad-dvr --auto` tolerates the camera appearing late;
  the reference camera leaves the USB bus once, 23-39 s after it gets power,
  for 4-5 s or (first drive after a long stand) for about 122 s, and no road
  picture exists meanwhile (docs/lanes/L16-my-lights-junctions.md, "Start-up").
  A host that knows earlier that the car will be driven (a door was unlocked)
  should power the hub then;
* **ignition**: the host starts the target when ACC comes on and stops it
  cleanly (grace, `systemctl stop`, wait for the units) when ACC goes off;
  `roadsense.target` is therefore **not enabled at boot** on that machine
  (`install-phone.sh` without `AUTOSTART=1`);
* **CPU profile**: the host keeps the Gold cluster online while any unit runs
  (modeld/signd/envd pin cpu4-7) and applies its idle profile only afterwards.

* **position** (optional): the host may feed GNSS fixes and vehicle motion to
  `<rundir>/position.sock` (position input API v1, "mapd" above); roadsense does
  all filtering and map matching. Without a feeder the map layer reports
  `unavailable` and the stack runs on vision only: there is no waiting for a
  fix anywhere (every vision topic is served from the first camera frame, the
  map facts join when a fix arrives and leave when the position runs out). The same `motion` messages
  give the lead tracker (topic `lead`) the car's speed and standstill; a host
  that feeds test input sends `reset` when the test ends.

Nothing in this list is specific to that backend: any supervisor that can run
`systemctl` and read JSON can drive the stack the same way.
