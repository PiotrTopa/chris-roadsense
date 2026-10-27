# L10 playlist: every ride of the library, one after another, looping

Lane opened 2026-10-27 (Opus, build + deploy). Goal: the phone's live stack plays all 21 rides of
`docs/RIDES.md` in a loop through the normal dashboard (`:8080/`, `/preview`), each ride with its own
intrinsics and calibration, robust over days. Times CET (Warsaw local, UTC+1).

## Journal

### 1. Design (reading the code)
* modeld already re-reads `calib.json` when its mtime changes, checked every 2 s **on frame arrival**
  (`modeld/main.cc`, `prep_thread`, `next_calib_check += 2 s`). After a seam pause longer than 2 s the
  check is overdue, so it runs on the first frame of the next ride: a playlist that writes the next
  ride's `calib.json` (atomic rename) at the start of a >= 2 s pause gets the new calibration onto the
  very first frame, **with no modeld change** (the phone's modeld need not be rebuilt).
* Intrinsics live in the ring header and every reader copies them per frame (`FrameView.focal_px`),
  the api reads the header on every request. Ride change = **new ring** (rename over the old one, the
  same mechanism as a camerad restart): readers follow within 500 ms (ring.h reopen clock).
* **seq restarts at 1 on every ride.** All readers already treat a seq going backwards as "camera
  restarted": modeld resets its pairing/history (`seq <= last_seq`), envd resets its smoother
  (`pipe->reset()`), signd flushes its sign tracks and its light tracker (`lt.reset()`). So no stale
  temporal state crosses a ride change in any provider.
* api: its speed-limit engine keeps state (limit, area, odometer, tracks) and cached the focal once
  (`_speed_geom`). New: on a camera message whose `ride`/`ride_index` changed or whose seq went
  backwards, the api resets the speed-limit state (fresh `SpeedLimit`, geometry re-read from the new
  ring), the live/suppressed sign tracks and the sign-context cache. A jump between rides is not driving.

### 2. Build (12:55-13:20 CET)
* `camerad-replay/main.cc`: `--playlist LIST|DIR`, `--shuffle`, `--max-seconds-per-ride S` (testing),
  `--calib-out PATH` / `--no-calib`; the player became a class (`Player`). Ride change = write the next
  ride's `calib.json` (tmp + rename), unmap the old file (`POSIX_FADV_DONTNEED`: 20 GB of rides must not
  sit in the phone's page cache), open and index the next, new ring with its intrinsics, seq from 1,
  pause `--gap-ms` (2500 in playlist mode). The list file is re-read at every wrap (edit it live);
  a ride that fails to open is skipped. `--file` mode behaves as before (plus the ride fields).
* camera message: `ride`, `ride_index`, `ride_count`, `ride_t_s`, `ride_duration_s`, `playlist`,
  `author`, `licence` (from `SOURCE.txt`).
* api: `State._on_camera` resets the drive state on a ride change or a seq going backwards
  (`reset_drive`: new `SpeedLimit`, focal re-read from the new ring, live/suppressed sign tracks and the
  sign context cleared); `ride_line()` -> status page line (`<pre id="ride">`) and a `/preview` HUD line.
  The HUD font (Pillow's default Aileron) has no Polish letters and no em dash (seen as boxes), so the
  HUD line is ASCII-folded ("Inzynieria"); the status page keeps the real text. New
  `api/tests/test_ride_change.py`; all 49 api tests pass (venv with aiohttp).
* `scripts/use-replay.sh --playlist [DIR] [--shuffle]`: writes `/etc/roadsense/replay-playlist.txt` in
  the `docs/RIDES.md` catalogue order (rides present under DIR), then other ride dirs by name; env file
  gets `REPLAY_MODE=--playlist`. The unit's ExecStart is `camerad-replay $REPLAY_MODE ${REPLAY_FILE}
  $REPLAY_ARGS` with `Environment=REPLAY_MODE=--file` as the default (old env files keep working).
  The previous-camera state and `calib.json.pre-replay` are only written on the first switch, so
  `--enable` -> `--playlist` -> `--disable` restores the DVR camera and the original calibration.
* Contract written into `docs/ARCHITECTURE.md` (camerad-replay, "Playlist").

### 3. Laptop test (13:05-13:20 CET)
4-ride list (s1-roadworks f 800, a1-s8-rain f 1100, s7-elblag-lowsun f 900, krakow-2024 f 690),
`--max-seconds-per-ride 15`, private ring/rundir/config dir; modeld (ORT CPU, 2 threads), envd (rebuilt
from HEAD, 1 thread), api (port 18110), a socket monitor (`camerad.sock`, `modeld.sock`, `envd.sock`).
82 s, 4 ride changes incl. the wrap back to ride 1:

| ride | ring focal | modeld calib message (pitch / yaw / h / f) | first road msg after the first frame | road max gap | env after the change |
|---|---|---|---|---|---|
| s1-roadworks | 800 | 0 / 0.05 / 1.3 / 800 | 0.08 s | 0.11 s | rural_road, clear, day |
| a1-s8-rain | 1100 | -0.075 / 0.022 / 1.22 / 1100 | 0.07 s | 0.11 s | highway, overcast, day |
| s7-elblag-lowsun | 900 | -0.05 / 0.044 / 1.24 / 900 | 0.09 s | 0.11 s | highway, snow -> clear, day |
| krakow-2024 | 690 | -0.085 / 0.015 / 1.1 / 690 | 0.08 s | 0.11 s | city_street, clear, day |
| s1-roadworks (wrap) | 800 | 0 / 0.05 / 1.3 / 800 | 0.08 s | 0.06 s | rural_road, clear, day |

* modeld logged "calibration changed" at every change, and its `calib` message (new values) came with
  the first road message of the new ride: the calibration and intrinsics are switched from the first
  frame on. Ride change takes ~3 s of no frames (2.5 s gap + 0.4-0.8 s indexing).
* camera 20.0 fps in every ride, 0 skipped, 0 decode failures; road 9.4-13 runs/s (laptop at its 80 C
  throttle, which halves modeld's rate; not a playlist effect).
* api: speed-limit odometer restarted per ride (63 m into s1, 76 m 8 s into a1-s8-rain, 21 m 8 s into
  krakow), `/v1/camera` carries the ride fields, the ring focal follows; `/preview` HUD shows e.g.
  "Ride: krakow-2024 - Relaxing Roads 4K, CC BY 3.0  (4/4, 0/10 min)".
* **Finding: a reader built before the L5 RingReader fix does not follow a ring change.** The laptop's
  old envd build (2026-10-26 18:10) processed ride 1 and then saw no more frames. Rebuilt from HEAD it
  follows. On the phone `envd` was built 2026-10-26 19:18, i.e. before the fix (ring.h on the phone
  22:48): it must be rebuilt before the playlist runs there (modeld 2026-10-27 00:08 and signd 11:56
  were built after it).
* Regression checks: `--file` with `--start` at the end loops as before (loop 1, pause 1000 ms);
  `--playlist --once` with a missing entry skips it and stops after the last ride.
