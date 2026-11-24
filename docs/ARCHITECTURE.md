# Architecture and inter-process contract

**roadsense analyses one camera's video.** Its processes (camerad, modeld, envd, laned, external sign and
light providers) talk through two simple mechanisms: a shared-memory frame ring (camerad -> consumers) and
newline-delimited JSON over Unix stream sockets (every daemon -> whoever reads). These two mechanisms are the
whole contract: a process that writes the ring as described below is a valid camerad (any camera, any
device), and a process that reads the ring and serves `<rundir>/signd.sock` is a valid sign provider.
Nothing else couples the processes; they share no libraries beyond the header-only `camerad/ring.h` and
`camerad/jsonsock.h`, which exist for convenience. One optional addition, the GPU schedule
(`<rundir>/gpu.sched`, below), lets a sign provider that runs on the same GPU as modeld take turns with it;
every process still works alone without it. No message broker, no protobuf, no capnp.
`nc -U /run/roadsense/modeld.sock` must always be a valid debugging tool.

**roadsense's output is one of the sources of chris-integrator** (the stack is named chris; roadsense is to
become chris-roadsense). The integrator is a mixer: it takes each roadsense instance's sockets, the car's own
signals, GNSS, radars and further cameras as input channels, runs the estimator, the map box, the trackers
and the decision logic as plug-in transformers on one bus, and is what consumers talk to (the consolidated
HTTP / WebSocket API, the recorder). Design: `chris-integrator/docs/DESIGN.md` (section 9: the cut; section
14: the mixer). With a new camera a second roadsense instance adds inputs to the integrator; nothing here
changes.

```
 roadsense (per camera)                                                    chris-integrator (the mixer)
 camerad ──frame ring (shm)──┬──> modeld ───────── road, calib ──┐
                             ├──> envd (0.5 fps) ─ env ──────────┤         input channels ─> bus ─> plug-ins:
                             ├──> sign provider ── signs ────────┼──────>  estimator, map box (chris-mapper),
                             ├──> light provider ─ lights ───────┤         lanes, ego light, sign context, rules,
                             └──> laned (optional) display_lanes ┤         fused, view ─> output channels:
 camerad ─────────────────────────────────────── camera ─────────┤         HTTP /v2 + /ws, integrator.sock, recorder
 api (debug face): /preview, raw topics, governor ─ governor ────┘                ▲
                                                                                  │ feed.sock (vehicle input API)
 the car (CAN, head unit): motion, gnss, controls, body, objects ────────────────┘
```

What stays in roadsense: everything that needs one camera's pixels or one camera's model output and nothing
else, the load governor (it sheds this device's perception load by its heat), and a thin debug face (`api/`:
raw topics, `/preview` with overlays, the frame ring's header). What is in chris-integrator: everything that
combines sources, needs the car's motion or position, carries state across the gaps of a source, takes a
decision a consumer acts on, or is the consumer's interface. The messages chris-integrator serves are
described in its own reference, `chris-integrator/docs/TOPICS.md` (the `map` message in
`chris-mapper/docs/MAP.md`); their logic lives in `chris-integrator/chris_integrator/` (`trackers/`, `plugins/`,
`io/`).

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
**Road model (lane L27, 2026-11-19).** The default is the openpilot v0.11.1 model line: `driving_vision`
(stateless, an image pair 200 ms apart; lane lines, road edges, leads, pose) followed by `driving_policy` (ONNX
Runtime on the CPU, a 5 Hz history of the vision features) for the plan. openpilot v0.9.7 `supercombo` stays
selectable. `road_model` in every message says which one produced it.

| how modeld is started | what runs |
|---|---|
| no `--road-model`, no `road_model` in `modeld.json` (the unit in `systemd/`) | `vision` + policy when `driving_vision-*` and `driving_policy-fp32.onnx` are in the models directory; otherwise `supercombo` if its file is there, with `"road_model_fallback": true` in every message and `road_model.fallback` in health; with neither, exit 1 with the reason |
| `--road-model vision` / `"road_model": "vision"` | strict: a missing vision or policy file is an error (exit 1). `--policy none` runs without the policy: no `plan` key |
| `--road-model supercombo` / `"road_model": "supercombo"` | supercombo, as before L27 |
| `--model PATH` without a road model | the file's name decides (`driving_vision*` = vision, anything else = supercombo: what every command written before L27 meant) |

A modeld older than L27 knows none of this and runs supercombo; the unit passes no flag it would not know, so
an older binary under the new unit, and a new binary under an older unit, both start. The api's trackers take
their settings from `road_model` (below, `lead` and the ego lane), so an api and a modeld of different ages
work together, at worst with the other model's tracker settings.

The plan and `--policy-hz` (config `policy_hz`, default 5; 0 = after every vision run). The policy runs at
most that often; between two policy runs the last plan is published again and says so: `plan.held: true`,
`plan.age_ms` (camera time of this run minus that of the run the plan was made on; never above 1000: after a
second without a policy run there is no `plan` key). `policy_hz` and `policy_ms` (the CPU time of the policy
step of this run) are in the message. The load governor lowers the policy's rate (`policy_hz` cap, 2 a second)
before it touches the road model's own rate.

What differs between the two models in this message:

| field | vision + policy (default) | supercombo |
|---|---|---|
| `road_model` | `"vision"` | `"supercombo"` |
| `plan` | from the policy; `prob` null (one hypothesis); `held`, `age_ms` as above; absent without the policy | every run; `prob` = weight of the chosen hypothesis; `held` false, `age_ms` 0 |
| `desired_curvature`, `desired_curvature_std` | null (the model has none) | values |
| `desire_state` | from the policy (null without it) | values |
| `leads[]` | one hypothesis each; the speed comes from the 200 ms pair | best of the model's hypotheses |
| `pose`, `v_ego` | from the 200 ms pair; `v_ego` = `pose.vx` of this run. Steadier than supercombo's on the rides (speed 0.7 against 2.7 m/s, yaw rate 1.7 against 3.4 mrad/s about a 1 s mean) | `v_ego` = the speed fed into the run (the previous run's `pose.vx`) |
| `frame_seq_prev`, `pair_dt_ms` | the frame 200 ms earlier | the frame 50 ms earlier |
| `policy_hz`, `policy_ms` | numbers | null |

modeld runs the network through an inference backend (`modeld/backend.h`):
`--backend ort` (default; ONNX Runtime CPU, `driving_vision-fp32.onnx` / `supercombo-fp32.onnx`) or
`--backend ncnn` (ncnn on the GPU through Vulkan, `driving_vision-ncnn.param/.bin` from
`tools/ncnn/make_vision_ncnn.sh` / `supercombo-ncnn.param/.bin` from `tools/ncnn/make_ncnn.sh`; only when built
with `-DROADSENSE_NCNN=ON`, see `modeld/CMakeLists.txt` and HARDWARE.md for its precision). Frame prep, parsing
and this message are shared, so both backends and both models publish the same schema; with ncnn the default
pinning is `--cpus 0-3`. Config keys `backend`, `road_model`, `policy`, `policy_hz`, `models_dir` in
`modeld.json`. `scripts/fetch-models.sh` downloads both models (pinned URL and sha256 per file) and converts
them.

All distances in metres, device frame exactly as openpilot defines it: x forward,
y RIGHT, z DOWN, origin at the camera (modeld passes supercombo outputs through
in this frame, no axis conversion). Times in seconds.

```
"model_ms": 298.1,
"frame_seq_prev": 1203,            // second frame of the pair
"road_model": "vision",            // "vision" | "supercombo": which model made this message
"road_model_fallback": false,      // true: no model was named and the default one's files are missing (supercombo runs)
"policy_hz": 5, "policy_ms": 1.2,  // vision: the policy's rate cap now (0 = every run) and its CPU time in this run; null otherwise
"plan": {"t": [33 T_IDXS], "x": [33], "y": [33], "z": [33],
         "vx": [33], "vy": [33], "vz": [33], "ax":[33],"ay":[33],"az":[33],
         "roll":[33],"pitch":[33],"yaw":[33],"roll_rate":[33],"pitch_rate":[33],"yaw_rate":[33],
         "x_std":[33],"y_std":[33],
         "prob": null,             // supercombo: weight of the chosen hypothesis
         "held": false, "age_ms": 0},   // held: an earlier run's plan published again (policy_hz); age_ms: how much earlier
"lane_lines": [ {"x":[33 X_IDXS],"y":[33],"z":[33],"y_std":[33],"prob":0.97}, x4 (far left, left, right, far right) ],
"road_edges": [ {"x":[33],"y":[33],"z":[33],"y_std":[33]}, x2 (left, right) ],
"leads": [ {"prob":0.9,"t":[6],"x":[6],"y":[6],"v":[6],"a":[6],"x_std":[6],"y_std":[6],"v_std":[6],"a_std":[6]}, x3 (now, +2s, +4s hypotheses) ],
"pose": {"vx":..,"vy":..,"vz":..,"roll_rate":..,"pitch_rate":..,"yaw_rate":.., "*_std":..},
"desire_state": [8], "desire_pred": [[8] x 4],
"meta": {"engaged_prob":..,"disengage_probs":{"gas":[5],"brake":[5],"steer_override":[5],"brake_3ms2":[5],"brake_4ms2":[5],"brake_5ms2":[5]},
         "blinkers":{"left":[6],"right":[6]}, "hard_brake_pred":[..]}
```
Exact index tables come from openpilot v0.9.7 `selfdrive/modeld/constants.py`
and are mirrored in `modeld/supercombo_layout.h`; those of the v0.11.1 models (the ONNX metadata
`output_slices`) in `modeld/vision_layout.h`.

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
The api adds its own context checks (`chris-integrator/chris_integrator/trackers/signctx.py`): a round red track whose box centre
sits inside a traffic light box (`lights` topic) or inside the lead vehicle's rear (modeld `leads[0]`
projected, 2.6 m x 0-2 m) for >= half of its detections gets `"context": {"traffic_light": share,
"lead_vehicle": share}` and is flagged (`reason` `traffic_light` / `lead_vehicle`) unless the provider's
`verify` >= 0.9 with `glow` < 0.5 vouches for it. Flagged tracks are not live tracks, are not used by the
speed-limit state (listed in its `ignored` as `suppressed (<reason>)`), are drawn grey with the reason in
`/preview`, and are listed in `/v1/signs` as `suppressed_tracks`; history and `signs.jsonl` keep them with
the flag.

### envd: `{"type":"env", ...}` about once per second
`envd/` reads the ring at most `--max-hz` times per second (default 1, newest frame each time),
or once every `--period` s when that is set; `--republish S` re-sends the last message unchanged (same
`seq` and frame time) every S s between inferences. **Installed configuration (unit
`systemd/roadsense-envd.service`, lane L53, adopted by the owner 2026-11-21): `--period 2 --republish 1`**,
one inference every 2 s and the message every second, so consumers that judge `env` by its arrival (the
MFD's 5 s age rule, the speed engine) see a 1 Hz stream. Measured on the phone: envd 184 -> 95 % of a little
core (docs/lanes/L48-phone-results.md, docs/lanes/L53-owner-decisions.md). Accepted caveat (docs/lanes/L48-envd.md):
against 1 Hz the env area prior disagrees 0.72 % of the time (1 Hz phase noise 0.17-0.37 %) and weather /
`wet` 0.3-0.4 points above the noise; place, light, the tunnel and every map-backed decision stay inside the
noise. The binary's own default is still 1 Hz; the rate is set in the unit (CLI wins over `envd.json`).
No consumer has a rate floor for `env` (`rate` floors cover road / lead / lanes / lights / signs only).
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
             "coast": 0} ],                           // optional: processed frames since the track was last detected
"road_users": [ {"cls": "car",                       // optional (signd --lights-road-users, lane L42): person, rider,
                                                     //   car, truck, bus, train, motor, bike (BDD100K classes)
                 "score": 0.89, "box": [x0,y0,x1,y1], // frame pixels
                 "clip": "b",                         // edges of `roi` the box touches (l t r b): "b" = no ground point
                 "rider": false,                      // a bike / motor with its rider paired (union box)
                 "p": [0,0,0.89,0,0,0,0,0],           // the eight class scores
                 "scale_rate": 0.042,                 // optional (signd --lights-ru-scale): d ln(size)/dt of the object's
                 "scale_rate_std": 0.006,             //   image over the last scale_dt_s s, 1/s (> 0 growing), from
                 "scale_dt_s": 0.8,                   //   patch alignment; its 1-sigma. `"road_users": null` = shed
                 "scale_src": "both",                 // optional (--lights-ru-scale-lamps): "lamps" = from the lamp pair,
                                                     //   "both" = lamp pair and patch combined; absent = patch alignment
                 "trunc": "l",                        // optional (--lights-ru-flags): FRAME edges that cut the box (l t r b;
                                                     //   "" = none; present on every box when the flags are on)
                 "occ": 0.35, "occ_by": 2,            // optional (flags): share of the box covered by nearer boxes of this
                 "occ_edges": "rb",                   //   list, index of the one covering most, edges of this box they hide;
                                                     //   the three keys only when something covers the box
                 "sym": 0.83,                         // optional (--lights-ru-view, vehicles): mirror symmetry of the box's
                                                     //   image, -1..1 (near 1: seen from behind or in front)
                 "lamps": {"c": "red",                // optional (view cues, dark scenes): the vehicle's lamp pair: colour
                           "l": [612.4, 398.1],       //   red | white | unknown, left / right blob centroid (frame px),
                           "r": [633.9, 398.6],       //   their distance and its 1-sigma (px)
                           "sep": 21.5, "sep_std": 0.4},
                 "emb": "0a1f..."} ],                 // optional (--lights-ru-emb): appearance vector, hex of int8 values
"road_users_low": [ {...} ],                         // optional (--lights-ru-low X): boxes scoring above X but not above
                                                     //   their class threshold, same keys as road_users: to continue an
                                                     //   existing track only, never to start one; occ_by indexes road_users
"emb_kind": "yuv3x3",                                // optional (--lights-ru-emb): what `emb` is: "yuv3x3" (27 numbers,
                                                     //   model-free) | "reid32+yuv3x3" (32 learnt + the 27)
"idle": {"pattern": "ZW.Z.W", "hz": 3.33}            // optional (lane L53): nothing tracked, the provider runs its idle
                                                     //   pattern until a light shows up; hz = passes a second it keeps
```
Idle pattern (lanes L48 / L53; `signd --lights-idle P`, installed `ZW.Z.W` since 2026-11-21): while the tracker
holds no track at all, one frame period in three gets no detector pass; every message sent in that state says so
with `idle` (absent while a light is tracked or the pattern is off / forbidden by `<rundir>/demand`).
chris-integrator judges the `lights` stream against it: while the newest message announces `idle` (and for one
rate window after the last one) the topic's floor is 0.8 x `idle.hz` (2.7 a second for `ZW.Z.W`) and the rate
is measured over the arrivals without the short-window bias; with a light tracked the full floor (3 a second)
applies. So an idle stream that keeps its pattern is not `slow`, and one that falls behind it is.
Road users (lane L42, `docs/lanes/L42-road-users.md`): the light detector's own output carries eight road
classes; with `--lights-road-users` the provider decodes them from the same inference (no extra detector run;
the decode costs well under a millisecond) and lists them per message, absent without the flag, `null` while
the load governor sheds them (first step, `road_users_hz` 0). They are non-critical display metadata: the
api does nothing with them; chris-integrator places them with the camera's calibration (`road_users` stream),
folds their class into the vote of the track they point at (`tracks[].class`, `lead.class`) and draws those no
track holds as `view.objects` of kind `road_user`. Boxes cut at the bottom of the region (`clip` "b": nearer
than about 14 m in the wide view, 24 m in the second view) carry a bearing only.
Image scale change (`signd --lights-ru-scale`, roadsense-signs `docs/LIGHT_MODELS.md`, 2026-11-23): a road user
may carry `scale_rate` (1/s), the rate of change of the log size of its image, `scale_rate_std` (1/s, 1 sigma)
and `scale_dt_s` (s, the baseline). The provider aligns a grey patch of the box with the same object's patch
0.2-1.0 s earlier (normalised cross-correlation over scale and shift, then a least-squares fit of scale, shift,
gain and offset on the full-resolution frame), so the value does not depend on the box edges' jitter, the
object's true size, the camera pitch or the road slope: for an object straight ahead, closing speed / distance
= `scale_rate` (time to contact 1 / `scale_rate`), i.e. range rate = -range x `scale_rate`. The three keys come
together and are absent when not measured (no earlier patch of the object in the window, a tiny, textureless or
edge-clipped patch, a poor match, or a stated 1-sigma above 0.06 / `scale_dt_s`). Consumers weight it by
`scale_rate_std`; it is the mean rate over the baseline, so it lags a changing rate by about `scale_dt_s` / 2.
Detection inputs for a tracker (lane `detin`, 2026-11-24; roadsense-signs `docs/DETECTION_INPUTS.md`, how a tracker
should take them: `docs/lanes/detin-detection-inputs.md`). Five additions, each behind its own option, each off
by default, each adding only optional keys; with all of them off the message is byte for byte what it was:
* **Low-score list** (`--lights-ru-low X`): `road_users_low`, the boxes that score above `X` but not above their
  class threshold and that no listed road user (nor a better low box) overlaps by more than the NMS limit. Same
  element keys as `road_users`. The first list does not change with it. Meant for a second association stage
  (ByteTrack): a low box may continue an object a tracker already holds and must never start one; a consumer that
  does not know the key sees nothing new. Absent when off and while the road users are shed.
* **Flags** (`--lights-ru-flags`): `trunc` names the FRAME edges that cut the box (the object goes on outside the
  picture; `clip` names the edges of the view's region, which another view may see beyond). `occ` is the share
  of the box that nearer boxes of the same `road_users` list cover, `occ_by` the index of the one covering most,
  `occ_edges` the edges of this box they hide (an edge counts as hidden when half of it lies under nearer boxes).
  Nearer = the bottom edge lower in the picture; a bottom cut by the view or the frame is lower than it shows;
  two boxes whose bottoms are within 4 % of the taller one (or both cut) are not ordered and each counts the
  other. So `occ` errs towards "covered". It knows only boxes: a pole, a tree or an unboxed vehicle in front is
  not seen by it (on BDD100K it flags 72-74 % of the boxes labelled occluded, and 93 % of what it flags is
  labelled so). A consumer should not read the object's size from a hidden or cut edge, nor a ground point from
  a box whose `occ_edges` or `trunc` has `b`.
* **View cues** (`--lights-ru-view`, vehicles only, from the box's own pixels, no model): `sym` is the normalised
  correlation of the box's image with its mirror image (the best over small shifts of the axis; absent for boxes
  under 16 x 8 px): a vehicle seen from behind or in front is symmetric, one seen from the side or at an angle
  is not, nor is one half hidden. `lamps` is the vehicle's lamp pair where the frame is dark (median luma of the
  frame at most 90): two bright blobs at one height; `c` their colour (`red`: tail lamps, the rear faces the
  camera; `white`: headlamps, the front does; `unknown`: amber, mixed, dim), `l` / `r` the centroids, `sep` their
  distance with its 1-sigma. The centroids do not move with blooming, so `sep` is a ruler on the object: lamp
  centres of a car are 1.1-1.5 m apart, which gives a range that does not depend on the box.
* **Appearance vector** (`--lights-ru-emb yuv | FILE.onnx`): `emb`, a short vector per box (hex of its int8
  values, two characters each) such that two boxes of one object are near and boxes of different objects far
  (Euclidean distance); `emb_kind` at the message's top level says which vector it is, and vectors of different
  kinds must not be compared. `yuv3x3`: the central 90 % of the box in 3 x 3 cells, per cell the mean Y, Cb, Cr
  (nine Y, then nine Cb, then nine Cr; value = int8 / 32, in units of 50 grey levels about 110 for Y and of 20
  about 128 for chroma). `reid32+yuv3x3`: 32 numbers of a small network on the 32 x 32 crop, L2-normalised
  (value = int8 / 127), followed by the 27 `yuv3x3` numbers; distance = sqrt(|d learnt|^2 + (0.5 |d yuv|)^2).
  Absent for boxes under 8 px on a side (and, with the network, beyond the per-pass cap).
* **Scale rate in the dark** (`--lights-ru-scale-lamps`, `--lights-ru-scale-multi N`): the same three keys;
  `scale_src` says when the value came from the lamp pair's separation (`lamps`) or from both measurements
  (`both`). The stated std is then the combination's; where the two disagree by more than three of their joint
  sigma, half their difference is added to it.
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
forwarding it (`chris-integrator/chris_integrator/trackers/lights.py`): per light `dist_m` (from the housing size, rough),
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
**Legacy since lane L39.** In the chris stack the position is estimated once, by chris-integrator's
estimator, the car feeds the integrator directly (`<chris run dir>/feed.sock`, the same vehicle input API;
also served under the name `position.sock`), and the map is the box **chris-mapper** (top-level
`chris-mapper/`, package `chris_mapper`: pack format `rsmap.py`, `matcher.py`, `context.py`, `limits/`,
`box.py`): in = the integrator's `ego_state` (pose with uncertainty and time), out = the `map` message
described below, whose `match` goes back to the estimator as a measurement and whose `road`, `speed_limit`
and `ahead` are the map context. `map.position` then repeats the pose the box was given
(`motion_source: "integrator"`, `input.pose: "ego_state"`); `state` is `good` while the estimator has an
absolute fix within its freshness window, `dead_reckoning` while it coasts, `unknown` when its dead reckoning
ran out, `no_source` before any fix. The fields of `match`, `road`, `speed_limit`, `ahead` are unchanged.
The map pack format and `scripts/roadsense-maps` are unchanged (the builder imports `chris_mapper.rsmap`).
`mapd/roadsense_mapd` remains as the daemon described in this section (its own position filter, the vehicle
input store, the `ego` / `vehicle` / `objects` relays) for a host whose car backend still feeds roadsense's
`position.sock`; it uses chris_mapper for the map. Do not run it together with the map box: the position
would be estimated twice.

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

**`{"type":"gnss", ...}` on `<rundir>/ego.sock`** (output; since lane L37): every receiver fix mapd accepted, as
the feeder sent it (`fix`, `lat`, `lon`, `hacc_m`, `speed_mps`, `course_deg`, `alt_m`, `sats`, and `speed_acc_mps`,
`course_acc_deg`, `hdop`, `utc_ms`, `age_ms`, `source` when sent), with `ts_mono_ns` = the time mapd gave the fix
and `arrival_mono_ns`. Repeated fixes are relayed too. For the api's integrator (the receiver's speed as a
witness) and the drive recorder; mapd's own use of the fix is unchanged.

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
fix. **Receiver accuracy** (since lane L20): a receiver's own `hacc_m` can be far too optimistic (the
reference car's head unit claimed 1-4 m while its fixes scattered 10-15 m across the road with 3-5
satellites). Between consecutive fixes mapd keeps a running mean (20 fixes) of the fixes' squared innovations
against what the claimed accuracy promises and scales the `hacc_m` in use by its square root, never below
1 and at most 6 times (`gnss_adaptive`, `gnss_scale_n`, `gnss_scale_max`); the factor is reported as
`input.gnss.hacc_scale`. An honest `hacc_m` widens the matcher's distance term, so heading and continuity
weigh more than a position that is not as good as it says. Without `course_deg` the first heading comes from the displacement between fixes. Motion sources, in
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

The `map` message (map matching, the message, junction view, turn lanes, road geometry, speed limit and built-up
state) is described in **chris-mapper's `docs/MAP.md`** (`chris-mapper/docs/MAP.md`; moved there in lane L43).

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

## api: the debug face (Python, aiohttp, port 8091) and the consolidated API (chris-integrator, port 8090)

Since lane L39 `api/roadsense_api` is the **debug face of one camera** (api 3; default port 8091, loopback):

| Endpoint | Content |
|---|---|
| `GET /v2/road`, `/v2/signs`, `/v2/env`, `/v2/lights`, `/v2/calib` | the latest raw message of that daemon (`signs` with the detector's own live tracks); nothing is tracked, annotated or fused here |
| `GET /v2/camera` | latest `camera` message + ring header fields |
| `GET /v2/frame.jpg`, `GET /preview` | the current preview JPEG; the MJPEG stream with the raw model output drawn on top (lane lines, path, leads, sign boxes, lights) |
| `GET /v2/governor`, `POST /v2/governor` | the load governor ("Load governor" below) |
| `GET /v2/health` | this instance's daemons (connected, last message age), raw topic rates, `model_ms`, `road_model`, the raw `calib`, the governor, temperature, battery |
| `GET /ws?topics=...` | the raw messages as they arrive |
| `GET /v2/<consolidated topic>` (`lead`, `lanes`, `view`, `fused`, `rules`, `speed_limit`, `map`, `ego_state`, `tracks`, `vehicle`, `signs/history`) | 410 with a pointer: served by chris-integrator |
| `<rundir>/api.sock` | JSON lines: the `governor` message (`level`, `levels`, `shed`, `caps`, `limited`, `limiter`, `forced`, `headroom_k`) on every governor step; chris-integrator reads it as an input and marks the measurement topics (`rate.degraded`, `why`) with it |

The **consolidated API** (chris-integrator's HTTP / WebSocket output channel, port 8090 in a deployment) and the
consolidated topics it serves (`speed_limit`, `rules`, `fused`, `ego_state`, `tracks`, `lead`, `lanes`, the ego
lane, the road geometry, `view`) are described in **chris-integrator's `docs/TOPICS.md`**
(`chris-integrator/docs/TOPICS.md`; moved there in lane L43). This document keeps the raw per-camera topics.

### Display lane source (`display_lanes`, laned; lanes L27, L29)

The lane lines of the measurement topics (`road.lane_lines`, `lanes`, `fused.lanes`, the ego lane, the road
geometry in `view.road`) are the road model's, with the model's own confidence, whatever their quality. The
lines a consumer **draws** (`view.lines`) may be refined by another source: a separate, cheap lane-line analysis
whose only job is a good picture. There is one place where that is decided, `ViewBuilder._display_lanes()` in
`chris-integrator/chris_integrator/trackers/view.py`, and one message that feeds it:

```
{"type":"display_lanes","seq":..,"ts_mono_ns":..,"source":"laned",   // CLOCK_MONOTONIC of the frame the lines belong to
 "x":[..],                                                   // distances ahead, m (any grid; device frame)
 "lines":[ {"present":true,"y":[..],"y_std":[..]?,          // far left, left, right, far right; y right positive;
            "conf":0.9,"reach_m":48.0,"id":7}, x4 ],         //   y_std over 1 m = beyond the line's reach (not drawn)
 "edges":[ {"present":true,"y":[..]}, x2 ]?,                 // optional: left, right road edge
 "pitch_trim":0.004,"yaw_trim":-0.002,"analysis_ms":2.1}     // laned: its own trim of calib.json, its time per frame
```

sent as JSON lines on `<rundir>/laned.sock` by a provider (the api connects when the socket exists; it is not a
health failure when it does not). The message is also passed on to `/ws` consumers as topic `display_lanes`, as
it came. While the last message is at most `display_lanes_max_age_s` (0.5 s, `view.json`) from the frame,
`view.lines` is built with it; otherwise from `lanes` alone, as without a provider. **Nothing else reads the
message: `lead`, `lanes`, `fused`, the ego lane and `view.road` are the same, message for message, with and
without a provider** (`api/tests/test_display_lanes.py`).

How the source is used (`display_lanes_mode` in `view.json`), per line:

| mode | a line is drawn from | when to use |
|---|---|---|
| `agree` (default) | the source where source and road model agree on the line (within `display_agree_m` 0.5 m at 6 and 12 m for `display_agree_s` 0.6 s: the same marking, and the source measured the paint); the road model everywhere else. Beyond the source's reach the line continues as the model's, shifted by the difference at that point. Lines only the source has are not drawn (`display_agree_extra`: drawn while one source line agrees) | the source refines the model's lines and can never put a line on another marking or invent one |
| `prefer` | the source where it has the line, the model where it does not | a source that is trusted to pick the markings |
| `replace` | the source only: a line it does not see is not drawn | a source that is trusted alone ("absent rather than invented") |

Road edges stay the road model's unless the source sends `edges`. `view.lines_source` says what the lines of the
frame are: `"model"`, the source's name (`"laned"`: every drawn lane line is the source's), or `"laned+model"`.

**Scale.** A road-plane analysis measures in the metres of `calib.json`'s camera height; the road model in the
metres of its own idea of the camera (its lane is 2.8 m wide where a camera 1.5 m up sees 3.4 m). Where both see
the ego lane's two lines the ratio of the two lane widths is low-passed (`display_scale_tau_s` 8 s) and applied
to the source's `x` and `y` before anything else (`view.lines_scale`; null without a source;
`display_scale: false` turns it off), so the lines sit right next to the model's lead, path and objects.

**laned** (`laned/`, `roadsense-laned.service`; optional: `roadsense.target` does not want it, it runs after
`systemctl enable roadsense-laned.service`) is the provider that ships: a classical analysis of the frame's luma
plane, no model and no GPU.

1. A marking filter at the nodes of a road-plane grid (44 rows from 4 to 60 m, geometric; 0.08 m cells to 7 m on
   each side; three image rows per grid row): a stripe of the paint's width (0.15 m: `f * 0.15 / distance` pixels)
   that is brighter than the road on **both** sides, min(centre - left, centre - right) of three box means on one
   image row. A one-sided edge (a kerb in the sun, a vehicle's flank, a shadow) and a wide bright area (a zebra
   bar, a white car) give nothing.
2. Candidate lines of the frame: the tracks of the last frame are followed from where they were; new lines start
   at the peaks of the lateral profile of the family of parallel lines `y = a + b x + c x^2` that collects the most
   evidence up to 26 m. Each is followed row by row and fitted (weighted quadratic, curvature tied loosely to the
   frame's). A line must span 9 m, cover a tenth of its rows, fit to 0.14 m; lines 0.6-2 m apart (wheel tracks in
   snow, a rail next to a line) cancel each other; a new line under the car or pointing at the camera's foot
   (the streak of a lamp on a wet road) is not started.
3. Tracks with hysteresis (confidence low-passed over 0.6 s, on at 0.55, off at 0.2 or after 1 s without the
   line), then slots: the ego lane's lines are the nearest within 3.2 m of the car's axis on each side, an outer
   line lies one lane width (2.3-5.2 m) beyond. A slot is never filled by promoting a farther line.
4. Two slow trims of `calib.json`, reported in the message: pitch (lines of one road are parallel: their slope
   against their offset is the pitch error; 4 s) and yaw (on a straight road the car drives along the lines; 15 s).
   Bounded to 0.06 rad. The height is not trimmed: that is what the view's scale is for.

It reads the newest frame of the ring `--hz` times a second (default 5), on the little cores (`--cpus 0-3`),
nice 5; config `<config dir>/laned.json` (`hz`, `cpus`, `nice`, `source`, and any constant of
`laned/lanefind.h` by name); `--set KEY=VALUE`; re-reads `calib.json` when it changes. The load governor pauses
it with the `scenery` step (`laned_hz` 0), which returns the display to the model's lines. Replay:
`laned_replay FRAMES.nv12 ...` (one message per frame), evaluation: `tools/laned/display_eval.py`. What it is
worth, measured: docs/lanes/L29-display-lanes.md (in short: the default mode moves the drawn lines onto the paint
where source and model agree; it does not make the picture steadier, and alone, in `replace`, it is worse than
the road model's lines in town, rain and snow).

## Load governor (`<rundir>/governor`, roadsense 2.0)

One policy decides what roadsense gives up when the platform runs out of thermal headroom
(`api/roadsense_api/governor.py`, lane L24). Before it, modeld halved its own rate at a temperature
(`--throttle-temp`) and envd paused itself above 92 C: the road model, the most important output, was the
first thing cut. Both are gone (the flags are accepted and ignored).

**Contract under heat.**

* `road`, `lead` and `lanes` keep their full rate and latency at any temperature the platform's limiter
  allows. They are the last thing the governor touches, and it touches them only by a clean rate cap, never
  by letting them queue.
* Everything else is shed first, one step at a time, in this order of rising importance:

  | level | step (`shed[]` name) | cap written | what a consumer sees |
  |---|---|---|---|
  | 1 | `road_users` | `road_users_hz` 0 | the light provider stops decoding road users (`"road_users": null`); classes are voted no further (the last voted class stays on its track), no new `road_user` display objects (lane L42) |
  | 2 | `preview` | `preview_hz` 1 | `/preview` and `preview.jpg` one picture a second |
  | 3 | `recorder_frames` | `record_frames` 0, `preview_hz` 0 | the recorder keeps messages, no pictures; `/preview` stops |
  | 4 | `scenery` | `env_hz` 0.2, `laned_hz` 0 | `env` once in 5 s (its labels are smoothed over 10 s); the display lane source (laned, when it runs) is paused: `view.lines` are the road model's again, `view.lines_source` says so |
  | 5 | `view` | `view_hz` 5 | the display channel at 5 messages a second |
  | 6 | `signs` | `signs_hz` 2.5 | sign detector at half rate (the GPU time goes to the road model) |
  | 7 | `lights` | `lights_hz` 2.5 | light detector at half rate |
  | 8 | `detectors_floor` | `env_hz` 0, `signs_hz` 1.25, `lights_hz` 1.25 | scenery paused, detectors at a quarter |
  | 9 | `plan` | `policy_hz` 2 | the road model's policy at 2 runs a second: `road.plan` held in between (`plan.held`, `plan.age_ms` up to 500); road / lead / lanes untouched. Unused with supercombo (no policy) |
  | 10 | `road` | `road_hz` 10 | the road model at half rate (`road.rate_limited`) |

  (Until lane L27 the road model was level 8; `plan` was put in front of it. Lane L42 put `road_users` in
  front of everything: every level below moved up by one; a `governor.json` with `max_level` 8 / 7 means 9 / 8
  now.)

* Every step and every restore is reported, and every measurement topic says what it is delivering (below):
  a consumer never has to guess whether it is getting the full stream.

**Policy.** Once a second the governor reads the platform through a probe: every thermal zone with its limit
(the temperature at which the platform starts to cut clocks) and the state of the platform's limiter. Per zone
the temperature is averaged (4 s time constant: the zones swing with every inference burst); the headroom is
the smallest `limit - average` over the zones. The shed decision looks ahead: it uses the headroom expected
10 s on (the slope of the hottest average, itself averaged over 10 s, so one burst is not a trend); restoring
uses the headroom as it is.

* Shed: expected headroom under 4 K and the temperature not already falling -> one step, then 8 s for it to
  show before the next; with less than 1.5 K left now, 3 s and regardless of the trend; after a step that saves
  little (`quick`: preview, recorder pictures) 1 s.
* The road model (`last_resort`) is never shed on a forecast: only with less than 1.5 K left now, or while the
  platform limiter is cutting.
* An active limiter alone sheds nothing above the road step (its states linger while the temperature already
  falls); it blocks restoring.
* Restore: headroom above 9 K for 30 s and the limiter idle -> one step back. A step that had to be shed again
  within 90 s was worth more than assumed: the next time it needs 3 K more headroom (up to 20 K) and twice the
  hold (up to 10 min).

All of it is configuration (`<config dir>/governor.json`, keys of `governor.DEFAULT_CFG`: `shed_at_k`,
`urgent_k`, `lookahead_s`, `restore_at_k`, `shed_dwell_s`, `restore_hold_s`, `max_level` (9 = never touch the
road model's rate; 8 = nor the plan's), `steps` (the list above with its rates: a platform with other full rates sets its own),
`floors`, `enabled`).

**Probe.** roadsense knows no device. The default probe (`"probe": "sysfs"`) reads the generic Linux thermal
sysfs: `/sys/class/thermal/thermal_zone*` (a zone's limit is its lowest `passive` trip point, else its lowest
`hot` one; zones without either are ignored unless `limit_c` is set) and `cooling_device*` (any device with
`cur_state` > 0 means the limiter is cutting). A host with other sensors writes a reading file
(`"probe": "file:/path.json"`: `{"zones":[{"name","temp_c","limit_c"}],"limiter":[{"name","state","max"}]}`)
or names its own class (`"probe": "package.module:Class"`, one method `read()` returning `governor.Reading`).

**Caps file.** The governor runs inside the api and writes `<rundir>/governor`, one flat JSON object, on every
change and at least every 2 s:

```
{"v":1,"ts_mono_ns":...,"valid_s":10,"level":4,"limited":0,"headroom_k":3.4,"road_users_hz":0,
 "preview_hz":0,"record_frames":0,"env_hz":0.2,"laned_hz":0,"view_hz":-1,"signs_hz":-1,"lights_hz":-1,"policy_hz":-1,
 "road_hz":-1}
```

A cap of -1 leaves the stage at its configured rate, 0 pauses it, any other value is an upper bound. Every
daemon reads its key through `camerad/governor.h` (`GovernorReader`, header-only): the camera daemons
`preview_hz`, envd `env_hz`, laned `laned_hz`, a sign / light provider `signs_hz` / `lights_hz` (and
`road_users_hz` for the light provider's road users), modeld `road_hz` and `policy_hz`; the api itself
applies `view_hz` and `record_frames`. A file older than `valid_s` is ignored: without the api (or with
`"enabled": false`) every daemon runs as configured and the platform's limiter is the only guard. A provider
that does not read the file is simply never shed.

**What a consumer is told.**

* `GET /v2/health` -> `governor`: `level`, `shed` (names of the steps in force), `caps`, `headroom_k`, `headroom_ahead_k`,
  `hottest` (`zone`, `temp_c`, `limit_c`), `limited` and `limiter` (the active cooling devices), `events` (the
  last 32 steps: `action` shed / restore, `what`, `level`, `headroom_k`, `why`, `ts_real_ns`), `shed_count`,
  `restore_count`, and `topics`: for `road`, `lead`, `lanes`, `lights`, `signs` the marker below.
  `GET /v2/governor` returns the same object; `POST /v2/governor {"force": N}` holds level N by hand
  (measurements, tests), `{"force": null}` gives it back to the policy.
* The api-built and api-annotated measurement topics (`lead`, `lanes`, `lights`, `signs`) carry
  `"rate": {"hz": 13.8, "latency_ms": 109.2, "degraded": false, "why": [], "level": 0}`: the measured message
  rate over the last 2 s, the latency from the frame's camera time to the api, the governor's level, and
  `degraded` with its reasons: `shed` (the governor has capped this topic's stage), `slow` / `late` (under the
  floor of `governor.json` `floors`: by default 10 messages a second and 250 ms for `road` / `lead` / `lanes`,
  3 a second and 400 ms for `lights` / `signs`; for `lights` while the provider announces its idle pattern,
  0.8 x the announced rate, and the marker carries `idle_hz`), `limited` (the platform limiter is cutting clocks;
  on its own it does not make a topic degraded). `env` has no floor.
* `road` is forwarded as modeld wrote it and carries modeld's own fields: `rate_hz` (measured runs a second
  over 2 s), `effective_hz` (the cap in force), `rate_limited` (the governor holds the cap below `--max-hz`),
  `governor_level` (null without a governor), `limited`. The former `thermal_c` is gone (`health.governor.hottest`).

Measured on the reference phone (what each step saves, and the proof that the road outputs hold at the limit
while the lower steps are shed): docs/lanes/L24-quality-max.md.

## Demand hints (`<rundir>/demand`, optional, lane L48)

The governor says what the device can afford; a demand file says what the consumer currently needs. Same
format, reader (`camerad/governor.h`, `GovernorReader(path)`) and validity rule as the governor's caps file: one
flat JSON object with `ts_mono_ns` and `valid_s`, rewritten atomically; a missing or stale file means "no hint"
and every daemon then behaves as configured. A hint may only allow a daemon to do less while nothing is lost,
or forbid such a saving; it never raises a rate above the configured one and never overrides the governor.
Nothing in roadsense writes the file: it is the integrator's to write (it knows the map and the car), and a
host without an integrator runs unchanged.

| key | read by | meaning |
|---|---|---|
| `lights_idle` | the light provider (`signd --lights-idle P`) | 0 = never use the idle pattern (signals are expected ahead); 1 or absent = the idle pattern is allowed while no light is tracked |

Further keys are proposals of docs/lanes/L48-system-optimisation.md (section 5), not implemented.

## roadsense 2.0: what changed since 1.x

2.0 (2026-11-18, lanes L19-L22) is a breaking release of the consumer contract. A 1.x consumer has to change
where the first column says "breaking"; everything else is additive.

| | where | 1.x | 2.0 |
|---|---|---|---|
| breaking | topic `lead`, `fused.lead`: `mode` | `track`, `hold`, `coast`, `none`; in `hold` (both standing) `dist_m` was frozen | `hold` no longer exists: `track` with `standing: true`, the value stays live (it shows the network's slow wander of a few decimetres: a display that rounds to metres needs a hysteresis of about 0.3 m). `standing`, `departed`, `gap_changed`, `coast` unchanged |
| breaking | topic `roadgeom`, `GET /v1/roadgeom`, `?topics=roadgeom` | the fused road geometry, one message per frame | gone (the path answers 410). The same fields are in `view.road`; its four-field `junction` head is replaced by the whole junction view in `view.road.junction` |
| breaking | `fused.lanes.ego_index` | a rule; never a middle lane; a guess when unsure | the ego-lane filter's answer; `null` = unknown, and more often than before; commit threshold 0.8 (0.9 on the vision model's lines, L27) |
| breaking | `map.match.confidence` | the winner's share among all candidate edges (dipped at every side road) | the confidence of the matched **road**: pieces of the same road are not rivals (median while moving 0.45 -> 0.55). Thresholds tuned to the old meaning should be revisited |
| breaking | `map.position.hacc_m` | followed the receiver's claim | scaled by what the receiver's own fixes show (up to 6 x the claim; `map.input.gnss.hacc_scale`) |
| breaking | map pack | format 1 | format 2 (turn lanes, speed cameras). mapd 2.0 reads 1 and 2; mapd 1.x refuses 2: update roadsense first, then the pack |
| breaking | recorder | `roadgeom` at 2 Hz (`roadgeom_hz`) | `view` at 2 Hz (`view_hz`) |
| new | topic `view`, `GET /v2/view` | - | the display channel: objects (lead, lights, signs), lines, path, and the road picture `view.road` |
| new | `view.road` | - | geometry, junction, lanes with probabilities, turn arrows; `approximate` / `provenance` per part; trust hysteresis done by roadsense |
| new | `fused.lanes` | - | `ego_from_right`, `probs`, `probs_from`, `estimator` (`confidence`, `provenance`, `last_change`, `evidence`) |
| new | `map.road.turn_lanes`, `map.ahead.turn_lanes`, `junction_view.approach.turn_lanes`, `fused.road.turn_lanes`, `fused.ahead.turn_lanes` | - | painted turn arrows (pack format 2); null on a format 1 pack |
| new | `map.ahead.features[].kind` | - | `speed_camera` (format 2 packs; cameras that are a node of the road) |
| new | `map.road.lanes` | forward lanes only from `lanes` / both of `lanes:forward` + `lanes:backward` | also from `lanes:forward` alone and from the number of turn arrows (`source: "tagged"`) |
| new | `map.pack.format` | 1 | 1 or 2: the format of the loaded pack |
| new | HTTP paths | `/v1/...` | `/v2/...`; `/v1/...` kept as aliases with the content of 2.0; `/v2/version`; `health.api` |
| new | `/ws` order per road frame | `road`, `lead`, `lanes`, `roadgeom` | `road`, `lead`, `lanes`, `view`: a consumer that needs one frame complete waits for `view` |
| config | `<config dir>/view.json` | - | `view.DEFAULT_CFG` keys; `road`: `roadview.DEFAULT_CFG` keys |
| config | `<config dir>/egolane.json` | - | `egolane.DEFAULT_CFG` keys (`commit_p` 0.8, `absence_trust` 0.2, `change_*`, `count_change_right`, `turn_lanes_side_p`) |
| config | `<config dir>/lead.json` | `stand_*` keys of the hold | the standing model's keys (`stand_tau_s`, ...): `lead.DEFAULT_CFG` |
| config | `mapd.json` | - | `gnss_adaptive`, `gnss_scale_n`, `gnss_scale_max`, `matcher.class_speed`, `matcher.class_speed_mps`, `matcher.parking_prior`, `matcher.road_confidence` |
| config | `recorder.json` | `roadgeom_hz` | `view_hz` |
| breaking | `road.thermal_c`; modeld `--throttle-temp`, envd `--pause-temp` / `pause_temp` | modeld halved its rate and envd paused at a temperature of their own | gone: one load governor sheds in a fixed order with the road model last ("Load governor"); `road.rate_hz`, `road.governor_level`, `road.limited` are new, `effective_hz` / `rate_limited` now reflect the governor |
| new | `health.governor`, `GET/POST /v2/governor`, `rate` in `lead` / `lanes` / `lights` / `signs` | - | what is shed under heat and what each measurement topic is delivering (rate, latency, `degraded` + reasons) |
| config | `<config dir>/governor.json` | - | `governor.DEFAULT_CFG` keys |
| default changed (L27) | the road model behind `road` | openpilot v0.9.7 supercombo | openpilot v0.11.1 `driving_vision` + `driving_policy`; supercombo selectable (`road_model`). `road.desired_curvature` is null, `plan.prob` is null, `plan` may be `held`; the api version stays 2 (every field keeps its name, type and meaning; nulls were already allowed) |
| new (L27) | `road.road_model`, `road_model_fallback`, `policy_hz`, `policy_ms`, `plan.held`, `plan.age_ms`; `road_model` in `lead`, `lanes`, `view`; `health.road_model` | - | which road model runs, whether it is a fallback, how old the plan is |
| new (L27) | `lead.evidence`; `lead.event.kind` `reacquired`, `relocated` | - | weak / no evidence behind a coasting track; a `track_id` that is kept (vision profile) |
| new (L27) | `view.lines_source`, message `display_lanes` on `<rundir>/laned.sock` | - | the seam for a separate lane source for the display |
| new (L29) | `view.lines_scale`; `view.lines_source` may be `"laned+model"`; `display_lanes` passed on at `/ws`; governor cap `laned_hz`; daemon laned (optional) | - | the display lane source that ships; default mode `agree` |
| changed (L27) | governor levels | 8 = `road` | 8 = `plan` (`policy_hz`), 9 = `road`; levels 1-7 unchanged; `max_level` 8 keeps the road model's rate |
| config (L27) | `modeld.json` | - | `road_model`, `policy`, `policy_hz`, `models_dir` |
| config (L32) | `modeld.json` | - | `auto_calib` (`on` / `observe` / `off`), `calib_state` |
| unchanged | vehicle input API (`position.sock`, `POST /v2/vehicle`) | 1.1 | 1.1 (`"api": 1` in `map` / `ego` / `vehicle` messages is that API's version, not this one's) |
| new (2.1, L37) | topic `ego_state`, `GET /v2/ego_state`, `health.integrator`; mapd relays raw `gnss` (also on `/ws`) | - | the data integrator's output: ego motion with uncertainty and sources, per-source health, checks |
| new (2.1, L37) | `lead.valid` / `reason` / `inputs`, `lead.ego.std_mps`, event `invalid`; `lanes.valid` / `reason` / `inputs`; `view.integrity`; `fused.lead.valid` / `reason`; `rules.ego_source` | - | a product made from a faulted source goes out as unknown with the reason. A consumer that knows nothing of the new fields sees `present: false` / no present line (as when nothing is there), which keeps a 2.0 consumer correct; one that reads them can show "unknown" |
| changed (2.1, L37) | where the ego motion comes from | each tracker: vehicle within 0.5 s, else the model; the speed-limit / rules odometer always the model's | the integrator's estimate everywhere (`lead.ego`, `lanes.ego`, `view.ego`, the odometer of `rules` / `speed_limit`) |

Which topic for what: decisions (alerts, anything that acts) read the measurement topics (`lead`, `lanes`,
`lights`, `signs`, `speed_limit`, `rules`, `fused`, `map`); pictures read `view`. The reference consumer (the
car's display, `cyberpunk_computer`) draws its whole road scene from `view` and keeps its alerts on the
measurement topics.

### Drive recorder (optional)
`chris-integrator/chris_integrator/io/recorder.py`, an output channel of chris-integrator since lane L39; off unless `<chris config dir>/recorder.json` names a directory
(`{"dir": "/var/lib/chris/rec", "max_mb": 4000}`). Every start of the integrator opens a session
`<dir>/<YYYYmmdd-HHMMSS>/` with `messages.jsonl.gz` (the topics as they went out: `road` reduced to lane
lines, road edges, `leads[0]`, plan x / y / z, speed and pose; `lights`, `signs`, `env`, `speed_limit`,
`rules`, `fused`, `map`, `ego` as relayed (cap 25 Hz), raw `gnss`, `ego_state` at 2 Hz (the record of what was
said; replays recompute it), `vehicle`, `objects` at 10 Hz, `view` at 2 Hz, `calib`, one
`camera` message a second; since lane L37 `road` keeps `road_model`, without which a replay ran the trackers'
supercombo profile on a vision-model session: `tools/replay_lib.read_session(..., road_model="vision")` puts it
back for older sessions) and `frames/<seq>.jpg`,
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

modeld publishes `calib`: `{"pitch":..,"yaw":..,"roll":..,"height_m":..,"focal_px":..,"source":"config|auto",
"online":{..}}`: the calibration the frames of the following `road` messages were warped with. The configured
values come from `/etc/roadsense/calib.json`; pitch and yaw are also estimated online (lane L32,
`modeld/calibrator.h`, docs/lanes/L32-lead-lateral.md), like openpilot's calibrationd.

**Online pitch / yaw.** The model's `pose` is the camera's motion in the calibrated frame, so while the car drives
straight its direction is the calibration's remaining error: `yaw = yaw_used + atan2(vy, vx)`,
`pitch = pitch_used - atan2(vz, vx)` (measured for both road models: 0.9-1.0 deg per degree). A run counts when
`pose.vx` >= 6.7 m/s (24 km/h), |yaw rate| <= 1 deg/s and the model's own doubt about the lateral direction
(`atan2(vy_std, vx)`) is <= 0.5 deg. 100 such runs make a block (its median); the estimate is the median of the
last 50 blocks, valid from 5 blocks (about 40 s of straight driving) while the blocks agree (standard error of the
median, 1.25 sigma / sqrt(blocks), <= 0.3 deg to become valid and <= 0.45 deg to stay valid; robust sigma of the
blocks <= 1.5 deg) and the estimate is within 0.15 rad of the configured value. Roll and `height_m` are not observable this way
and stay as configured. The yaw estimate also absorbs an error of the principal point `cx` (the two are the same
shift of the image), the pitch estimate one of `cy`.

Why it is needed with the v0.11.1 vision model: its lane lines and road edges turn with the calibration's yaw
(geometry of the warped image), its lead and the policy's plan do not (the model refers them to its own motion).
A yaw error of e therefore draws the lead e to the side of its own lane, growing with distance (3.4 deg on the
car: one lane at 60 m). supercombo moved lines and lead together by about 0.4 e, so it hid the error.

| `auto_calib` (`modeld.json`, `--auto-calib`) | behaviour |
|---|---|
| `on` (default) | a valid estimate replaces the configured pitch / yaw in the warp; `source` becomes `"auto"`. The warp follows the estimate in steps of at least 0.25 deg; each step drops one frame pair (0.2 s without a `road` message for the vision model). An estimate that stops being valid is taken out again (back to the configured value) |
| `observe` | estimated and reported, never used |
| `off` | no estimate (`online.state` `"off"`) |

```jsonc
"online": {"state": "valid",        // "collecting" (< 5 blocks) | "valid" | "unstable" (blocks disagree) |
                                    // "out_of_range" (> 0.15 rad from the configured value) | "off"
           "applied": true,         // the warp uses the estimate
           "pitch": 0.0233, "yaw": -0.0594,              // the estimate, rad (null before the first block)
           "spread_pitch": 0.0017, "spread_yaw": 0.0030, // robust sigma of the block medians, rad
           "se_pitch": 0.0003, "se_yaw": 0.0006,         // standard error of the estimate, rad
           "blocks": 46, "min_blocks": 5, "samples": 12, "block": 100, "accepted": 4612, "seen": 67957,
           "restored": true,        // the blocks were read from the state file at start
           "config": {"pitch": 0, "yaw": 0}}             // calib.json's values
```

`calib` is sent again whenever a block ends or the warp's values change. The estimate is kept in
`calib_state` (`/var/lib/roadsense/camera-calibration.json`, `--calib-state PATH|none`) and belongs to one
`calib.json` and one camera (focal, principal point, frame size): after either changes (also a ride change of
camerad-replay's playlist) it is dropped and estimated anew. When the newest 5 blocks all leave the estimate by more
than 1.5 deg (the camera was moved) the older blocks are dropped. `/v2/health` shows it as `calibration`.
`modeld_replay` has the same estimator, off unless `--auto-calib on|observe`; `tools/vision_eval/
calib_from_pose.py` runs it on a recorded session and prints a `calib.json` line.

### Calibration tools (`tools/calib/`, lane L33)

How the numbers in the camera file and in `calib.json` are measured and installed: [CALIBRATION.md](CALIBRATION.md)
(the owner's procedure) and [lanes/L33-calibration-tools.md](lanes/L33-calibration-tools.md). `board.html` is the
target, `capture.py` takes stills from the frame ring on the camera host (read-only, standard library),
`lens.py` solves focal length, picture centre and distortion, `mount.py` estimates pitch and yaw from a recorded
straight drive (the road model's `pose`, and the vanishing point of the recorder's pictures), `apply.py` writes the
files, keeps the previous ones in `<config dir>/calibration-backups/<stamp>/`, and draws a verification view.
Keys they add, all flat scalars that today's daemons ignore: `k1`, `k2`, `p1`, `p2`, `k3` in the camera file (the
lens distortion, OpenCV's model; the input warp is still a pinhole), `camera_to_front_m` and `lateral_offset_m` in
`calib.json` (tape measurements; `camera_to_front_m` is also written to `lead.json`, where the lead tracker reads it).

## Build and deploy

C++17, CMake, one binary per daemon. In the reference deployment (Poco F1,
postmarketOS) everything is built on the phone (`g++`, `cmake`,
`onnxruntime-dev`, `libcamera-dev` from Alpine); `scripts/install-phone.sh`
builds, installs to `/usr/local/bin`, writes the systemd units from `systemd/`
and enables `roadsense.target`. Models under `/var/lib/roadsense/models`,
fetched and converted on a dev box with `scripts/fetch-models.sh` (the default road model: openpilot v0.11.1
`driving_vision` + `driving_policy`; supercombo v0.9.7 as the selectable one and modeld's fallback; URL and
sha256 pinned per file) and copied with `scripts/push-models.sh`. mapd is Python (`py3-numpy`), installed to
`/usr/local/lib/roadsense/mapd` with `roadsense-mapd.service`; its map pack is not part of
the tree: it is built on a dev box and copied with `scripts/roadsense-maps install` to
`/var/lib/roadsense/maps/current.rsmap` ([MAPS.md](MAPS.md)). A sign provider installs its own unit
with `PartOf=`/`WantedBy=roadsense.target`; the target does not require it.
Every unit runs in `roadsense.slice` (`systemd/roadsense.slice`: CPUWeight 100,
no memory throttling, MemoryMax 4608M as an OOM backstop only) so whatever else
the machine does stays responsive while the stack saturates the GPU and the big
cores; a sign provider puts `Slice=roadsense.slice` in its unit too. A host may
add a slice drop-in (e.g. `AllowedCPUs=`) to keep the stack off cores it
reserves. The debug face's port and bind address are `ROADSENSE_API_PORT` /
`ROADSENSE_API_HOST` (`/etc/roadsense/api.env`, written by
`DEBUG_API_PORT=N DEBUG_API_HOST=ADDR scripts/install-phone.sh`; defaults 8091 / 127.0.0.1;
a loopback bind also removes the nftables opening). `scripts/install-phone.sh` then runs
`chris-integrator/scripts/install.sh` (the consolidated API on 8090: `CHRIS_API_PORT` / `CHRIS_API_HOST`; `chris-integrator.service` in
`roadsense.target`, the legacy mapd disabled; `chris-integrator/docs/INSTALL.md`).

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
* **health**: `GET /v1/health` of the consolidated API (chris-integrator, `CHRIS_API_PORT`: 8090 on that
  machine, because the car backend serves 8080; the same fields as before, `daemons` lists this instance's
  daemons and the map box); a silent API is restarted by the host after a timeout. The debug face has its own
  `/v2/health` on `ROADSENSE_API_PORT` (8091);
* **API front**: on that machine the consolidated API listens on `127.0.0.1` only (`CHRIS_API_HOST`); the car
  backend re-publishes its HTTP/WS endpoints to its own clients (`/api/v1/roadsense/...`), over HTTP, without
  importing any code of the stack;
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

* **position** (optional): the host may feed GNSS fixes and vehicle motion to chris-integrator's
  `<chris run dir>/feed.sock` (the vehicle input API of `docs/VEHICLE_API.md`; also under the name
  `position.sock`); the integrator does all filtering, the map box the map matching. Without a feeder the map layer reports
  `unavailable` and the stack runs on vision only: there is no waiting for a
  fix anywhere (every vision topic is served from the first camera frame, the
  map facts join when a fix arrives and leave when the position runs out). The same `motion` messages
  give the lead tracker (topic `lead`) the car's speed and standstill; a host
  that feeds test input sends `reset` when the test ends.

Nothing in this list is specific to that backend: any supervisor that can run
`systemctl` and read JSON can drive the stack the same way.
