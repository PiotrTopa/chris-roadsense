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

### 4. Phone deploy (13:20-13:34 CET)
* `git archive HEAD` of `camerad-replay/main.cc`, `camerad/ring.h`, `scripts/use-replay.sh`,
  `systemd/roadsense-camerad-replay.service`, `docs/RIDES.md` into `<phone-home>/roadsense` (the phone tree is
  otherwise older than HEAD in api/modeld files; those are installed elsewhere by their lanes and were
  not touched). The installed api (`/usr/local/lib/roadsense/api`) was identical to HEAD (md5) before.
* envd rebuilt as user in L3's build dir (`~/l3/build-envd`, `make envd`, ncnn on, sources = HEAD,
  ring.h with the reopen fix), installed to `/usr/local/bin/envd` (old binary kept as
  `envd.pre-l10`). The build heated the phone to 94.9 C for a minute (the stack was running); waited
  until < 88 C (75.8 C) before switching.
* Root `<phone-home>/l10/deploy.sh`: envd install, api backup `/usr/local/lib/roadsense/api.pre-l10` + the two
  new api files + api restart, then `sh scripts/use-replay.sh --playlist <phone-home>/rides`: compiled
  camerad-replay (`c++`), kept `replay-prev-source` = dvr and the existing `calib.json.pre-replay`
  (from L5, the DVR calibration), wrote the 21-ride `/etc/roadsense/replay-playlist.txt`
  (catalogue order), restarted camerad-replay, try-restarted modeld/envd/signd. All units active.
  First ride opened in 2.3 s (cold page cache for 1.5 GB).
* Quick check with test windows (`REPLAY_ARGS=--max-seconds-per-ride 45`, 13:23-13:33, rides 1-13):
  socket monitor as root (`<phone-home>/l10/mon.py`; the sockets are root-only) over 160 s, 4 changes:

| ride (index) | ring focal | modeld calib after the change (pitch / yaw / h / f) | camera | road | signs | lights | first road after 1st frame | road max gap | env | temp |
|---|---|---|---|---|---|---|---|---|---|---|
| a1-s8-rain (8) | 1100 | -0.075 / 0.022 / 1.22 / 1100 | 20.0 fps | 13.8/s | 5.2/s | 3.1/s | - | 0.12 s | highway, rain, day | 72 C |
| s7-elblag-lowsun (9) | 900 | -0.05 / 0.044 / 1.24 / 900 | 20.0 fps, 900 frames / 45 s | 619 / 45 s = 13.8/s | 5.0/s | 3.0/s | 0.13 s | 0.11 s | highway, snow, dusk_dawn | 72 C |
| s1-roadworks (10) | 800 | 0 / 0.05 / 1.3 / 800 | 20.0 fps | 13.8/s | 5.0/s (roadworks, 30 limit, curves ...) | 3.0/s | 0.13 s | 0.12 s | highway, snow, day | 74 C |
| tunnels-s3 (11) | 690 | -0.033 / -0.069 / 1.5 / 690 | 20.0 fps | 13.7/s | 5.0/s (priority road ...) | 3.0/s | 0.14 s | 0.12 s | highway, overcast, dusk_dawn | 74 C |
| tunnel-pow-warsaw (12) | 900 | (next) | 20.0 fps | 13.6/s | 4.7/s, 7 live tracks | 2.6/s | | | | 68 C |

  api snapshots of rides 0-3 (13:23-13:26): camera 19.7-19.9/s, road 13.2-13.5/s, model 53-61 ms,
  signs 4.9-5.0/s, detect 78-116 ms, lights 2.5-2.8/s, env ~1/s with labels per ride (krakow
  city_street/overcast/dusk_dawn, rural-zloczew rural_road), speed-limit odometer per ride (52 m 12 s
  into forest-vistula-spit). `/preview` HUD on the phone:
  `docs/lanes/L10-playlist/phone-hud-tunnel-pow-warsaw.jpg` (laptop checkout only, *.jpg is git-ignored) ("Ride:
  tunnel-pow-warsaw - Awizonosz, CC BY 3.0 (13/21, 0/5 min)"; frame by Awizonosz, CC BY 3.0); the
  status page shows the same line with the real characters.
* 13:33:54: back to whole rides (`REPLAY_ARGS=` and a camerad-replay restart): the playlist starts
  again at krakow-2024.

### 5. Whole rides on the phone: two natural ride changes (13:34-14:08 CET)
Socket monitor (root) for 27 min plus api snapshots from the laptop:

| ride (index) | span | camera | road | signs | lights | ride change | env | api speed limit |
|---|---|---|---|---|---|---|---|---|
| krakow-2024 (0) | 13:34-13:44 | 20.0 fps (11742 frames / 587 s) | 8068 / 587 s = 13.7/s, max gap 0.12 s | 2931 = 5.0/s | 1760 = 3.0/s | - | city_street / residential, overcast, day | odo 2300 m, limit 50 at 13:39 |
| rural-zloczew (1) | 13:44:06-13:59:05 | 20.0 fps (18000 frames, the whole ride) | 12200 / 900 s = 13.6/s, max gap 0.13 s | 4492 = 5.0/s (priority road, end of built-up area, 70 limit ...) | 2698 = 3.0/s | 13:44:00-13:44:06; modeld calib -0.072 / -0.094 / 1.5 with the first road message (0.13 s after the first frame) | rural_road, overcast, day | odo restarted: 439 m 35 s in, 8524 m at 13:51; limit None -> 90 -> 70 |
| forest-lubelskie (2) | from 13:59:13 | 19.8 fps | 13.3/s | 4.5/s | 2.6/s | calib -0.072 / -0.012 / 1.65 at the change | forest_road, clear, day | |

* The phone dropped off the network 14:01-14:07:53 (kernel log: wlan0 re-associated 14:07:36;
  uptime 16 h, no reboot). The stack kept running through it (forest-lubelskie at 9:19 when it came back).
* Temperature: the hottest zone touched 94.9-95.5 C in single samples (13:44, 13:45, and at the end of
  krakow-2024), 68 C at 13:51, 66-69 C at 14:08 (6 samples / 30 s), so no sustained 95 C. The spikes
  match L4/L5 (the stack at ~14 road runs/s plus signd and envd); not new with the playlist.
* Boot autostart unchanged: `roadsense.target` disabled (camerad-replay and signd are enabled, but
  only `roadsense.target` wants them, so nothing starts at boot).

### 6. State left (14:10 CET)
Playlist RUNNING on the phone: 21 rides in `docs/RIDES.md` order, 4 h 06 min of video plus ~3-5 s per
ride change (2.5 s gap + 0.4-2.3 s file indexing), i.e. a loop of ~4 h 08 min, forever. Commands
(root on the phone, from `<phone-home>/roadsense`):
* status: `sh scripts/use-replay.sh --status`
* playlist (again / other dir / shuffled): `sh scripts/use-replay.sh --playlist [<phone-home>/rides] [--shuffle]`
* one ride: `sh scripts/use-replay.sh --enable <phone-home>/rides/<slug>/<slug>.mjpeg`
* back to the DVR camera + the original calib.json: `sh scripts/use-replay.sh --disable`
* skip to the next ride: restart starts from ride 1; to change the order edit
  `/etc/roadsense/replay-playlist.txt` (re-read at every wrap) or restart after editing.
Rollback of this lane: `/usr/local/bin/envd.pre-l10`, `/usr/local/lib/roadsense/api.pre-l10`.
