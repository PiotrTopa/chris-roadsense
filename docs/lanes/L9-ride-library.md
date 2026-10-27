# L9 ride-library: openly licensed Polish driving recordings

Lane opened 2026-10-27 (Opus, data lane). Goal: a library of diverse, openly licensed forward-facing
driving recordings from Poland for laptop/phone replay (`tools/make_ride.sh` -> `camerad-replay`).
Catalogue: `docs/RIDES.md`. Times CET (Warsaw local, UTC+1). Videos never go into a repo.

## Journal

### 1. Search
(10:30-11:00 CET)
* **Wikimedia Commons** (API search incl. `deepcat:"Videos from Poland"`, categories Dashcam videos,
  Videos of roads): still only `City Driving 4K- Kraków Poland 2024` (the L5 ride). Everything else Polish
  is static/traffic-camera, drone, construction or event footage. Commons is exhausted for this purpose.
* **archive.org** advanced search (driving/dashcam/jazda + Poland/Polska, movies): nothing (news, newsreels).
* **YouTube with the Creative Commons filter** (`sp=EgIwAQ%3D%3D`) is the productive source. The licence
  is no longer printed in the watch-page HTML, so each candidate is verified with yt-dlp's `license` field
  (YouTube's own metadata; "Creative Commons Attribution license (reuse allowed)" = CC BY 3.0 unported).
  yt-dlp 2026.10.26 installed into a private venv `~/datasets/rides/.tools/venv` (pip, no permission issue).
  Note: the Kraków source video on YouTube (cpnOJaZlPaI) now reports a standard licence (`license` NA); the
  Commons copy was licence-reviewed as CC BY in 2025 and CC BY is irrevocable for copies made under it.
* Productive CC BY channels: **Road** (UC4jJHe_slMO4N_XCE8ODgJg; 2026 uploads, 4K/1440p 50 fps, Poland:
  rural, A1/A4 day and night, winter, rain, forest, S3 tunnels, fog), **Unwinding Rides**
  (UCRRmfOBOziJ8J2moFnmvwSw; 4K60, Silesia/Kraków/Warsaw; MIXED licences: several of its uploads are
  standard YouTube licence, checked per video), **Slow TV Travel Cruise** (UC3qZXeuhsKanjqe31PRSsTA; 1440p60,
  Tricity S6/S7/A1, forest DW214), **Red Mower / CzerwonaKosiarka** (UCN5PTiNKJXutX20bUUKGOdg; winter, night),
  plus single CC BY videos: Grzegorz Dymon / zabek1 (Gdańsk Martwa Wisła tunnel), FlyLokietek (Świnoujście
  tunnel), Awizonosz (POW tunnel, underground car park), Inżynieria Ruchu PL. (S1 construction),
  astrokacyper (Warsaw storm).
* Rejected on licence (standard YouTube licence per yt-dlp): Relaxing Roads 4K Warsaw (dwtmEnKCP7c,
  uJU_DEklaw4), Unwinding Rides fog (7CqGH52bBIw), misty-rain S1 (f1J20IOympw), Night City Warsaw
  (A_NNWccmOp4), Katowice rainy evening (c07yZv_qeoc), Warsaw nightfall (4nWDk-bXJJM), Warsaw morning
  (P76beg86C5U), S1 Bielsko-Żywiec (io7yT4rbFZE).
* Sped-up channels rejected for modeld: DSzumaher "Drive & Road Signs Chronicles" (CC BY, but 2x-4x
  time-lapse; useful only for sign tests, not added).
* **Mapillary**: CC BY-SA images, but API/download needs an account token (brief: no accounts) and the
  sequences are ~1-2 fps stills: not usable for modeld, skipped.
* **Pexels/Pixabay**: short stock clips (< 1 min, mostly drone/creative), no continuous Polish drives.
* **Research datasets**: comma2k19 (US), Zenseact ZOD (Europe, registration form), a Kraków object-detection
  image set (licence unstated): none gives continuous open Polish video without an account.
* Window choice: per candidate a contact sheet (one 360p frame every ~max(45 s, length/40) via the HLS
  stream, `~/datasets/rides/.tools/sheet.sh`), then `yt-dlp --download-sections` of only the chosen window
  (1080p or the best <= 1440p), then `tools/make_ride.sh`.
* Candidates rejected on content: Red Mower winter POV (iE2yK68Fy7Y; a mirror-mount housing blocks ~25 %
  of the frame), Awizonosz car-park video (handheld walk-through), A1 Gajewo -> Gdańsk (M2LjuuKaCPE;
  pillar-boxed), Slow TV DW214 forest (eDn50Sawvhc; burnt-in bar plus pictures of signs inserted by the
  editor; replaced by two Road forest rides), OMT S6 (overlay bar, duplicate of S7), Jaworzno night
  (y7RJ_838YcM; duplicate of krakow-night-tunnel), Rabka -> Wadowice "sunset" (overcast, no low sun),
  Martwa Wisła tunnel 4K (JZjKxh_E5pw; 2:15, duplicate of DYZJ2Pocpa0).

### 2. Download and preparation (10:57-11:50 CET)
* Tools (not in the repo; `~/datasets/rides/.tools/`): `sheet.sh` (contact sheet from the 360p HLS
  stream), `dl.sh` (yt-dlp `--download-sections`, best <= 1080p video only, `--write-info-json`),
  `prep.sh` (make_ride.sh with THREADS=3 in a `systemd-run --user --scope -p MemoryMax=8G -p CPUQuota=400%`
  scope + 3 gridded calibration frames), `meta.py` (camera-replay.json + calib.json from f and the vanishing
  point), `source.py` (SOURCE.txt from the info json), `verify.sh` (laptop replay check), `zoom3.sh`
  (frame triplets for dash tracking), `push_phone.sh`.
* Downloads ran 3 in parallel; YouTube serves section downloads at roughly real time per stream
  (0.3-1.4 MB/s each). 21 windows, 9.4 GB of source cuts incl. the L5 Kraków file.
* Transcodes: ~1:30 per 15 min window at <= 4 cores (CPUQuota 400 %).

### 3. Intrinsics method
* Pitch/yaw: road vanishing point from two painted lines (or kerbs/edges) on a straight section, read off a
  1280x720 frame with a 40 px grid; pitch = -atan((vp_y-360)/f), yaw = atan((vp_x-640)/f) (L5 sign
  convention). Height: a lane of width W spanning w px at dv px below the horizon gives h = W dv / w,
  independent of f (lane widths assumed 3.0-3.75 m by road class).
* Focal: a single frame cannot give f. Two ways used:
  - **measured** (s7-elblag-lowsun): the dashcam burns in the speed (115 km/h). A dash top tracked over
    0.2 s moves from dv 88.5 to 178.5 px (and a second one from 46 to 62.5): f h (1/dv1 - 1/dv2) =
    v dt gives f = 904 and 898 px with h = 1.24 m. The same frame then shows an 11.7 m dash period
    (P-1a, 4 m line + 8 m gap).
  - **dash rhythm**: with a known period (12 m) and h, f = 12 / (h (1/dv1 - 1/dv2)) for consecutive dash
    tops: a1-s8-rain 1100 px (h 1.22), krakow-night-tunnel ~1000 px. The same channel's warsaw-city at
    1100 px put the projected lane lines exactly on the painted dashes, so Unwinding Rides' phone camera
    is ~1000-1100 px at 1280x720 (HFOV ~60-65 deg). katowice-rain (same channel, dashboard mount) looked
    better at 760 than at 1100 (lane-line probabilities higher, lines on the double centre line), kept 760.
  - otherwise **guessed** from the camera type: Road channel action camera, rectilinear look -> 690 px
    (HFOV 86 deg, as L5); dashcams 600-800 px. The Road channel overlays (rural, forest, fog, night, S3)
    all put lane lines on the paint, so 690 is about right for that camera.
* A frame-time tracking try on tunnel-gdansk failed (the dashcam file has duplicated/dropped frames, the
  seeks were not monotonic); f stayed a guess there.

### 4. Laptop verification (11:10-11:52 CET)
`verify.sh SLUG MJPEG T1 T2`: camerad-replay (`--start T`) -> private ring `/dev/shm/roadsense-frames-l9`
-> modeld (L5's laptop build, ORT CPU 1.22, `--threads 2 --cpus none`) -> api (port 18091, venv from L5);
after 12 s (~240 frames) one `/preview` frame is saved as `overlay-T.jpg` in the ride folder; all
processes stopped by PID. Problem met: the scratchpad run dir path was too long for a unix socket
("socket path too long"), fixed with a shorter run dir. modeld ran at 10-20 runs/s (laptop also busy).
Results (two overlays per ride, judged by eye):
* Good (lane lines on the paint, path in lane, lead box on the car ahead with plausible distance):
  rural-zloczew, forest-lubelskie, forest-vistula-spit (curve followed), a1-night, a1-night-fog,
  winter-fog, a1-s8-rain (after f 1000 -> 1100 the right line sits on the paint), warsaw-city,
  krakow-night-tunnel, tunnels-s3, tunnel-gdansk (also on a curve), tunnel-pow-warsaw, s7-elblag-lowsun.
* Plausible but rough calibration: katowice-rain, warsaw-storm (heavy rain, low lane confidence),
  s1-roadworks (follows the yellow temporary line), tunnel-swinoujscie (curved tunnel), krakow-lights
  (junction scenes, road edges converge at the real horizon), winter-snow-rural (snow track edges found).
* Hard case: gdansk-night-snow city part (no markings under snow): the path wanders right; the
  expressway part later is fine.
* tunnel-pow-warsaw was re-cut after the check (first 5 s are drone footage); its overlays are from the
  first cut (times shifted by 5 s).

### 5. Phone copy
`push_phone.sh`: one plain `scp` per file, md5 compared with `md5sum` on the phone, free space checked
before each ride (stop below 20 GB). Only `<slug>.mjpeg`, `camera-replay.json`, `calib.json`,
`SOURCE.txt` go to the phone. The phone's camera source and services were not touched (L8 is measuring).
Result (12:00-12:55 CET): all 20 new rides copied, 80 files, every md5 equal to the laptop's
(`~/datasets/rides/.tools/push.log`); phone `~/rides` now 21 rides, 19.0 GB; phone root fs 75.3 GB free.

### 6. Hand-back
Catalogue and gaps: `docs/RIDES.md`. 21 rides (incl. L5's krakow-2024), 4 h 06 min, MJPEG 20.4 GB on
the laptop and on the phone; source cuts 9.4 GB on the laptop only. No permission denials.
