# L5 ride-replay: a recorded car ride through the live stack

Lane opened 2026-10-26 (Opus, execution). Goal: feed a real ~10-minute dashcam ride, looping, into the
frame ring on the phone instead of the bench camera, so the normal dashboard (`:8080/`, `/preview`)
shows the stack on road footage. Times CET (Warsaw local, UTC+1).
Phase 1 = laptop only (L4 measures the live stack on the phone; nothing on the phone is touched).

## Journal

### 1. Clip search (22:20-22:35 CET)
Openly licensed, continuous, >= 720p forward drives are rare. Wikimedia Commons search (API, rate-limited
after a few queries) gave few long European POV drives: `City Driving 4K- Kraków Poland 2024` (45 min,
2560x1440, CC BY 3.0), `Elten to Ruhr-Rhine confluence` (Germany, 57 min, 1080p), `Driving Slovenia
border to border` (motorway only, 3 h, 720p), `VW Passat B8 POV Highway Drive` (12 min, 1080p, Slovenia
motorway). comma2k19 (MIT) is US highway only. Chosen: **Kraków** (Polish signs and markings, the
target domain), although it is city-only: no rural/forest/highway part exists in an open Polish clip
found. Its author notes "technical issues due to inappropriate camera settings" in some frames; none
were visible in the chosen window.

| | |
|---|---|
| source | https://commons.wikimedia.org/wiki/File:City_Driving_4K-_Krak%C3%B3w_Poland_2024.webm (YouTube cpnOJaZlPaI, recorded 2024-01-10) |
| author | Relaxing Roads 4K (https://www.youtube.com/@RelaxingRoads4K) |
| licence | CC BY 3.0 (Commons licence review 2025-04-09); attribution required wherever frames are shown |
| downloaded | Commons 1080p VP9 transcode (2.04 GB, md5 = etag f0e15e81...), 8 parallel HTTP ranges (one stream ran at ~0.5 MB/s) |
| window | 25:00-35:00 of the original (10 min): wide arterials, a Vistula bridge, roundabout/large junctions, trams, buses, dense traffic, dusk/overcast light |
| camera | action camera on the bonnet line / low windscreen, rectilinear ("linear") wide mode, red bonnet visible in the bottom ~5 % of the frame, no overlays |
| laptop path | `~/datasets/rides/krakow-2024/` (`SOURCE.txt`, `download/`, `krakow-25-35.mjpeg`, `camera-replay.json`, `calib.json`, `sheet/`) — not in any repo |

### 2. Route: MJPEG file + camerad-replay (chosen) — 22:35-22:45 CET
Route (a) of the brief. The phone has no ffmpeg installed (it *is* in apk, `ffmpeg-8.1.2-r3`, not needed)
but has libjpeg-turbo dev. Raw NV12 would be ~16.6 GB per 10 min and needs no decode; MJPEG is 1.5 GB
and costs one ~2 ms (laptop; est. 5-7 ms on an A75, as camerad-dvr) libjpeg decode per frame, and gives
`preview.jpg` for free (the frame's own JPEG). New code:

* `camerad-replay/main.cc` (+ CMake option `ROADSENSE_CAMERAD_REPLAY`, on when libjpeg is found):
  mmap + SOI/EOI index, raw decode into the ring, absolute-deadline pacing, real-time skip, loop with a
  1 s seam pause, `camera` messages with `"source":"replay"`. Documented in ARCHITECTURE.md
  ("camerad-replay"). A standalone copy of camerad-dvr's decoder generalised to any W x H (multiples of 16)
  rather than a shared header, so camerad-dvr's code is untouched.
* `tools/make_ride.sh INPUT OUT.mjpeg [START [DURATION]]`: ffmpeg fps=20, area scale to 1280x720,
  full range yuvj420p, `-q:v 4`, optimal Huffman.
* `systemd/roadsense-camerad-replay.service` (`Conflicts=roadsense-camerad.service roadsense-camerad-dvr.service`,
  `EnvironmentFile=/etc/roadsense/camera-replay.env` with `REPLAY_FILE`, `REPLAY_ARGS`).
* `scripts/use-replay.sh --enable FILE | --disable | --status` (root). Remembers the previous source
  (`/var/lib/roadsense/replay-prev-source`, dvr or camerad), saves `/etc/roadsense/calib.json` as
  `calib.json.pre-replay`, installs the ride's `camera-replay.json` + `calib.json` (else
  `config/camera-replay.json` + `config/calib-replay.json`), keeps camerad masked, disables camerad-dvr,
  starts the replay and `try-restart`s modeld / envd / signd; `--disable` reverses all of it.
  If no build dir binary exists it compiles the single file with `c++ ... -ljpeg` (seconds).
* Transcode: 1 min 30 s on the laptop (8 threads, MemoryMax 8G scope), 12000 frames, **1 508 884 969 B**
  (123 KB/frame), md5 f2917ad4e104e7f84e7fbc6b4f4d3a00.

**Bug found and fixed: ring readers did not follow a replaced ring.** `RingReader::next()` checked for a
new ring file only every 500 idle polls of one call, but modeld and envd call it with a 200 ms timeout,
so the check never ran: after camerad (any variant) restarts, modeld keeps waiting on the old, dead ring
(seen on the laptop: road output stopped when the replay was restarted). Fix in `camerad/ring.h`: the
reopen check is a 500 ms clock kept in the reader. Verified: modeld (rebuilt) resumed by itself on a new
replay instance. The phone's installed modeld/envd/signd predate the fix, so `use-replay.sh` restarts
them after every switch (also harmless after they are rebuilt). roadsense-signs carries its own copy of
ring.h (not checked here; the restart covers it either way).

### 3. Intrinsics and mounting guess
* **focal_px 690** at 1280x720 (`camera-replay.json`). The lens draws straight lines straight (a
  rectilinear "linear" action-camera mode), and the view is wide: a linear-mode HFOV of ~86 deg gives
  640/tan(43 deg) = 686 px. Not measured; no camera model is stated in the source.
* **calib.json** (next to the ride): pitch **-0.085** rad (road vanishing point ~(650, 420) on straight
  sections, 60 px below the centre: camera tilted up by atan(60/690); openpilot sign: positive = down),
  yaw **+0.015** (vanishing point 10 px right of centre: positive = device yawed left), roll 0,
  **height_m 1.1** (lane width 3.25-3.5 m spans ~870 px 280 px below the horizon:
  h = W (v - v_h) / w_px = 1.05-1.13 m; independent of f).
* How wrong values show: a focal too small makes the model see a zoomed-out image: leads read too far,
  lane lines too narrow near the car and the projected lines fall inside the painted ones at distance;
  too large the opposite (leads too close, lines splay outwards). A pitch error moves the projected
  horizon (lines converge above or below the real vanishing point) and biases distances; a yaw error
  makes a straight road's path bend to one side. On these frames the projected lines converge at the
  real vanishing point and follow the painted lines over ~30 m (see below), so the guess is in the right
  range; refine with two straight-road frames if needed (focal from lane-line fit at distance).

### 4. Laptop verification (22:39-22:45 CET)
camerad-replay -> private ring (`ROADSENSE_RING=/dev/shm/roadsense-frames-l5`, rundir in the scratchpad)
-> modeld (ORT CPU, laptop build, `--threads 4 --cpus none`) -> api (port 18085). No signd, no envd.

* camerad-replay: 20.0 fps steady, decode+NV12 1.8-2.5 ms avg (max 8.4), 0 skipped, 0 decode
  failures over 5717 frames; index of the 1.5 GB file 150-160 ms.
* modeld: 19.6 runs/s at first, then the laptop's 80 C throttle halved it (the laptop was at 96-97 C,
  shared with other jobs): 9.9 runs/s, model 18-25 ms. `/v1/camera` shows `"source":"replay"`,
  focal 690, 1280x720; `/v1/health` camera 19.6 fps, road 19.5/s before the throttle.
* Loop seam (`--start 596`): `loop 1 (pause 1000 ms)`, seq continues, modeld keeps running across it.
* Overlay frames (from `/preview`; `/v1/frame.jpg` has no overlay option) in
  `docs/lanes/L5-ride-replay/` (frames: Relaxing Roads 4K, CC BY 3.0):
  - `overlay-2.jpg` (t = 2:22 of the clip, queued at a junction): lane lines low probability (0.1),
    path straight ahead along the lane, lead at 4.7 m p 0.98: plausible for a stop in a junction.
  - `overlay-4.jpg` (t = 3:12, wide multi-lane junction): path follows the lane, lead box on the car
    ahead at 5 m p 0.98, left/right lane lines p 0.79/0.55 roughly on the painted lines.
  - `overlay-5.jpg` (t = 3:37, bridge next to a tram): left lane line on the painted line edge, right
    line on the right marking, road edges on the kerbs, lead 45 m p 1.00: good.
  - `overlay-6.jpg` (t = 4:02, straight street): left line on the hatched-area edge, right line on the
    lane's dashed marking, road edges on the kerbs, lead 17 m p 1.00 on the car ahead: good. Polish
    signs in view (chevron board, no-stopping + P), useful for the sign provider on the phone.
  Overall plausible; lanes in the dense city junctions are weak, as expected for supercombo.

### 5. Phase 2 commands (not run; the phone belongs to L4 now)
See the hand-back report; copied here when phase 2 runs.
