# L33: camera calibration tools

Lane L33 of the roadsense 2.0 campaign, 2026-11-19, laptop only (the phone was in the car and was not touched).
Branch `v2-l33`. The owner's procedure is [docs/CALIBRATION.md](../CALIBRATION.md); this is the record of what was
built, what was measured and how to reproduce every number.

## What exists now (`tools/calib/`)

| file | does | needs | where it runs |
|---|---|---|---|
| `board.html` | the target: ChArUco (or plain chessboard) on a screen, square size from the screen's lit width; prints at a set square size (A4 landscape) | a browser, no network | any screen |
| `capture.py` | stills from the running camera: luma of the frame ring as PGM (read-only map; keeps a picture when the scene holds still and has changed), or the preview JPEG | Python standard library | camera host (**not run on the phone**) |
| `lens.py solve` | focal length, picture centre, k1, k2 with uncertainties, reprojection error, coverage map; refuses unusable sets | OpenCV + aruco | laptop |
| `lens.py impact` | how far the distortion moves points inside the road models' input frames | numpy | laptop |
| `lens.py synth` | pictures of the board through a known camera (rehearsal, tests) | OpenCV | laptop |
| `mount.py` | pitch and yaw from recorded straight driving: the road model's motion, and (`--frames`) the vanishing point of the recorded pictures; picks the straight stretches itself; `--pictures DIR` for bare pictures | numpy; OpenCV for the pictures | laptop (the first method also on the phone) |
| `apply.py` | writes the camera file, `calib.json`, `lead.json`; previous files to `calibration-backups/<stamp>/`; plausibility checks; verification view; `--rollback`; `--dry-run` | numpy, Pillow | phone or laptop (**not run on the phone**) |
| `perturb.py` | turns the camera of a recording by a known angle, or bends it like a lens (validation) | OpenCV | laptop |
| `geom.py` | the shared geometry (roadsense's frames and signs, OpenCV's distortion model), plausibility checks | numpy | |
| `tests/test_calib.py` | 33 tests on synthetic data with known answers | 10 of them need OpenCV | |

OpenCV is not in `.venv-models`; the lane venv is `<datasets>/l33/venv`
(`pip install numpy pillow opencv-contrib-python-headless`: OpenCV 5.1.0, numpy 2.5.5). Below `PY` is that
venv's python and commands run from the checkout.

```sh
$PY -m unittest discover -s tools/calib/tests                 # 33 tests OK (about 40-50 s on the loaded laptop)
python3 -m unittest discover -s tools/calib/tests             # without OpenCV: 33 run, 10 skipped
PYTHON=~/projects/roadsense/.venv-models/bin/python sh tools/smoke_api.sh    # api suite: 252 OK, unchanged
```

## 1. Lens

**Board.** ChArUco, because a partly visible board still gives corners with known identities: that is what lets
the picture's corners be covered. `DICT_4X4_50`, marker 0.75 of a square, so the whole board is a grid of 8 cells
per square and draws crisply at any square size that is a multiple of 8 device pixels. The page was checked against
OpenCV's own drawing: a headless Chrome screenshot at 1920x1080 is identical pixel for pixel to
`CharucoBoard.generateImage` and the detector finds 28 of 28 corners and 20 of 20 markers; the page's print (PDF)
has squares of 30.00 mm for a requested 30 mm. A unit test compares the page's cell matrix (run under node) with
OpenCV.

```sh
google-chrome --headless=new --window-size=1920,1080 --screenshot=board.png "file://$PWD/tools/calib/board.html?w=344&hide=1"
google-chrome --headless=new --print-to-pdf=board.pdf --no-pdf-header-footer "file://$PWD/tools/calib/board.html?cols=8&rows=5&print=30"
```

The square size only sets the scale of the board's distance; focal length, centre and distortion do not depend on it.

**Capture.** The frame ring is world-readable (mode 0644) and is mapped read-only with the documented reader
protocol, so nothing in the stack is touched and no daemon is added. Luma is written as PGM: lossless, full size,
any camera source, no JPEG encoder needed (the phone has no OpenCV; Python 3 with the standard library is enough).
`--source preview` copies camerad-dvr's preview instead (the camera's own 1280x720 JPEG). The recorder's one
picture a second is a third source. Tested on the laptop against a real ring written by `camerad-replay` playing a
ride (4 PGM pictures and 2 preview copies, readable, right size) and in unit tests against a fake ring (still /
moved logic, a slot overwritten while read). The "holds still" test compares means over about 900 cells: on the
car's recorded pictures two pictures of a parked scene one second apart differ by 0.5-1.5 grey levels, two while
driving by 14-38; the thresholds are 2 (still) and 4 (moved).

```sh
B=<datasets>/l33/build; W=<scratch>/l33/ringtest
ROADSENSE_RING=$W/ring ROADSENSE_RUNDIR=$W/run ROADSENSE_CONFIG_DIR=$W/cfg $B/camerad-replay/camerad-replay --file <datasets>/rides/s7-elblag-lowsun/s7-elblag-lowsun.mjpeg --start 60 &
ROADSENSE_RING=$W/ring python3 -I tools/calib/capture.py --out $W/shots --count 4 --every 1.0 --all
ROADSENSE_RUNDIR=$W/run python3 -I tools/calib/capture.py --out $W/prev --source preview --count 2 --every 0.5
```

**Solver.** OpenCV `calibrateCameraExtended` with one focal length (roadsense has one `focal_px`), centre, k1, k2;
tangential terms and k3 fixed at 0. A freer fit (fx, fy, p1, p2) runs as a check and warns when pixels do not look
square or the lens looks tilted. Views that repeat another view (shared corners moved less than 12 px) count once;
views that do not fit (more than 3 times the median error) are dropped. The uncertainty of every value is the larger
of the fit's own (optimistic: it assumes independent corner noise) and a bootstrap over the views.

It refuses (exit 2, `"status": "refused"`, no `calibration` block, `apply.py` will not take it) when:

| condition | limit |
|---|---|
| distinct views with the board | fewer than 10 |
| views with the board tilted by 15 deg or more | fewer than 3 (focal length and distance cannot be told apart) |
| coverage: cells of an 8 x 6 grid over the picture holding a corner | below 60 % |
| reprojection error | above 1.0 px |
| uncertainty of the focal length | above 1.5 % |
| uncertainty of the centre | above 0.008 focal lengths (0.46 deg) in x or y |
| distortion not monotonic inside the picture, field of view or centre implausible | any |

It warns (and still answers) when a corner cell of the picture holds no board corner: the distortion out there is
extrapolated.

Rehearsal on synthetic pictures, rendered through a camera with focal 990, centre 655, 348, k1 -0.33, k2 +0.12
(blur 0.8 px, noise 1.5 grey levels), 26 views:

| value | truth | solved | stated uncertainty |
|---|---|---|---|
| focal length | 990 | 989.59 | 0.57 |
| centre x, y | 655, 348 | 655.06, 348.10 | 0.57, 0.45 |
| k1, k2 | -0.33, +0.12 | -0.3302, +0.1205 | 0.0006, 0.0009 |
| reprojection error | | 0.104 px | |

```sh
S=<scratch>/l33/rehearsal
$PY tools/calib/lens.py synth --out $S/synth --views 26 --focal 990 --cx 655 --cy 348 --k1 -0.33 --k2 0.12
$PY tools/calib/lens.py solve $S/synth --board charuco:8x5 --square-mm 30 --out $S/lens.json --debug-dir $S/dbg
```

**No real picture of the board through the car's camera exists on the laptop** (the five pictures of 2026-10-22 were
not kept), so the lens path is proven on synthetic pictures only: detection on a real LCD (moire, glare), through
the windscreen and at the camera's fixed focus is untested.

## 2. Distortion: what to do with it

Today nothing in roadsense uses a distortion: modeld's input warp, laned and the overlays treat the camera as a
pinhole with `focal_px`, `cx`, `cy`.

How far the lens moves points inside the road models' input frames, for the only data there is about the car's
lens (2026-10-22: focal 995 px, k1 -0.34 from a single off-centre view: indicative):

| what the warp is told | frame | moved, max / rms (model pixels of 512 x 256) | near the vanishing point (far vehicles) |
|---|---|---|---|
| the true focal 995 | medmodel (narrow input) | 14.7 / 4.4 | 0.05 |
| the true focal 995 | sbigmodel (wide input) | 43.3 / 14.0 | 0.22 |
| 975, as configured on the car today | medmodel | 8.2 / 1.8 | 1.0 (a 2 % scale error: far distances read 2 % long) |
| 975, as configured on the car today | sbigmodel | 38.1 / 11.2 | 0.8 |
| the best single focal for that frame (974 / 921) | medmodel / sbigmodel | 7.6 / 1.8 and 21.2 / 5.7 | |

```sh
$PY tools/calib/lens.py impact --focal 995 --k1 -0.34
$PY tools/calib/lens.py impact --focal 995 --k1 -0.34 --declared-focal 975
```

What that does to the road model's outputs, measured: a ride (s7-elblag-lowsun, 180 s, focal 900) bent with
k1 -0.34 by `perturb.py` and replayed, against the same ride unbent. `dist`: the warp is told the true focal
length; `distf`: the best single focal length for the narrow input (880.4):

| model | what the warp is told | speed (ratio to the unbent ride) | lane width at 10 / 20 / 40 m (ratio) | left, right line at 10 m (m, moved) | lead distance (ratio, frames) | lead sideways (m, moved) |
|---|---|---|---|---|---|---|
| vision | true focal | 0.978 | 1.000 / 1.001 / 1.001 | -0.012, -0.014 | 1.003 (1571) | -0.01 |
| vision | best single focal | 0.945 | 1.004 / 1.005 / 1.005 | -0.004, +0.009 | 0.993 (1552) | +0.05 |
| supercombo | true focal | 0.970 | 0.968 / 0.970 / 0.971 | +0.041, -0.058 | 0.991 (1695) | +0.00 |
| supercombo | best single focal | 0.957 | 0.960 / 0.963 / 0.965 | +0.065, -0.060 | 0.971 (1695) | +0.04 |

So an uncorrected distortion of that size costs a few per cent, not a lane: the model's own speed reads 2-3 % low
(4-5.5 % with a compromise focal: the true focal is the better thing to tell the warp), supercombo's lane comes out
3 % narrow (its lines 4-6 cm inwards at 10 m), the vision model's lane and the lead's distance and sideways position
stay within 1 %. The mount estimate on the bent ride moves by at most 0.12 deg (road model's motion) and 0.16 deg
(pictures) against the unbent one.

```sh
<datasets>/l33/scripts/dist_all.sh      # 4 replays (val.sh with K1=-0.34 [FDECL=880.4])
$PY <datasets>/l33/scripts/dist_eval.py
```

**Decision.**

1. The measured lens values go into the camera file as they are: `focal_px` = the **true** focal length (not a
   compromise), `cx`, `cy`, and `k1`, `k2` (`p1`, `p2`, `k3` = 0) as new flat keys that today's daemons ignore.
   The true focal is right where it matters most, around the vanishing point (far vehicles, the mount angles).
2. The distortion does **not** explain a lead drawn a lane to the side far away: near the vanishing point it moves
   points by 0.05-0.2 model pixels. The yaw error has the size of that symptom (section 3: 3.3-3.9 deg, which is
   3.5-4.1 m at 60 m); why it shows between the lead and the lane lines is lane L32's subject.
3. It does matter away from the centre: up to 15 pixels at the corners of the narrow input and up to 43 on the
   wide input, i.e. near lane lines and everything the wide input sees at the sides. So the warp should handle it.
   **Specification for a later lane** (not implemented: `modeld/warp.cc` belonged to lane L32 today):
   * `Intrinsics` gets `k1, k2, p1, p2, k3` (default 0). The camera daemons read them from the camera file and put
     them into the frame ring header, in the zeroed space after `camerad_pid` (offset 72: five `float`, then a
     `uint32` flag "distortion valid"); zeros mean pinhole, so old readers and old writers keep working.
   * `WarpTable::build` already computes, once per calibration change, the source point `(u, v)` of every model
     pixel from the matrix `M`. Insert the forward lens model there: `x = (u - cx) / f, y = (v - cy) / f`,
     `r2 = x^2 + y^2`, `x' = x (1 + k1 r2 + k2 r2^2 + k3 r2^3) + 2 p1 x y + p2 (r2 + 2 x^2)`, `y'` alike, then
     `u' = f x' + cx`. It is the forward model, closed form, no iteration, and it costs nothing per frame (the table
     is precomputed). Both warp kinds (bilinear v0.9.7, nearest v0.11.1) and the chroma tables take the same step.
     `tools/calib/geom.py` `distort()` / `Camera.project()` is the reference; `perturb.py --k1` makes test input.
   * Consumers that project or back-project pixels get the same model: the api overlay's `Projector`, laned's
     camera model, the sign and light providers' distance and bearing from a pixel.
   * Acceptance: a ride bent by `perturb.py --k1` and warped with the distortion declared gives the outputs of the
     unbent ride (the table above collapses to ratios of 1.00).

## 3. Mount from driving

**Method 1, the road model's motion** (openpilot `selfdrive/locationd/calibrationd.py`, `handle_cam_odom`, read at
tag v0.9.7: samples above 15 mph with a yaw rate below 2 deg/s and a sideways uncertainty below 0.25 deg; observed
pitch `-atan2(z, x)`, yaw `atan2(y, x)` of the translation, composed onto the calibration in use; blocks of 100, 5
needed; the mean of the blocks; a reset when the blocks span more than 2 deg in yaw or 4 deg in pitch). Differences
here, for an offline tool that must state an error: medians instead of means, an acceleration gate, whole stretches
instead of single samples, 0.5 deg for the model's uncertainty (0.25 passes little at the phone's rate, as lane L32
found), and much tighter limits on the spread. The model's
`pose` is the camera's velocity in the frame the calibration in use defines. Driving straight, its direction is the
road direction, so `R_true = R_used . R(0, -atan2(vz, |vx, vy|), atan2(vy, vx))`. The result is absolute: it does
not matter what calibration was in use. Samples are used when the speed is at least 6.7 m/s, the yaw rate at most
2 deg/s, the model's own sideways uncertainty at most 0.5 deg, the acceleration at most 0.7 m/s^2 (the mean speed of
the second after against the second before; the model's speed is too noisy for a plain difference), on stretches of
at least 2 s with 0.5 s cut from both ends. Blocks of 100 accepted samples; the estimate is the median of the block
medians. Reported: the spread of the blocks (1.4826 MAD), the difference between the first and the second half, and
an error = the median's standard error, not below half the difference of the halves, combined with 10 % of the
residual (the model shows less than a calibration error's full size, see below). Not enough: fewer than 5 blocks,
spread above 0.5 deg, halves more than 0.4 deg apart, or recordings that disagree by more than 0.4 deg (the camera
was moved between them).

**Method 2, the pictures' vanishing point** (independent of the road model). Line segments (OpenCV LSD) of the
pictures taken on the accepted stretches; nearly horizontal ones and those that sit at the same place in 30 % of
the pictures (bonnet, reflections) are dropped; the point that most segments below it aim at is searched coarse to
fine and refined by least squares; its spread comes from a bootstrap over the pictures plus a floor of 0.15 deg for
what a bootstrap cannot see (lane position, crossfall, uncorrected distortion). With `--lens` the segments' end
points are straightened first.

`recommended`: per angle, the estimate with the smaller error when the two agree (within 0.7 deg or twice their
combined error); the road model's motion when there are no usable pictures; nothing when they contradict each other
(`apply.py` then wants an explicit `--mount-source`).

Roll and height are not observable from driving and are not estimated.

### Validation on rides: a known change must come back

Two rides (s7-elblag-lowsun, day, expressway; a1-night, night, motorway), 180 s each, both road models, three
cases: `base` as declared; `decl` the calibration **declared** to modeld is wrong by pitch +1.5, yaw -2.0 deg (the
true mounting is unchanged: the tool must return the base estimate); `warp` the **camera is turned** by pitch -1.5,
yaw +2.5 deg (`perturb.py`: the frames are re-projected by that rotation; the tool must return base plus that).
Angles in degrees.

| ride | model | case | straight s / blocks | calibration in use off by (pitch, yaw) | road model's motion (+- stated) | its error | pictures | their error |
|---|---|---|---|---|---|---|---|---|
| s7-elblag-lowsun | vision | base | 95 / 19 | -0.17, -0.05 | -3.03, +2.48 (0.02, 0.02) | ride's calib.json: -2.86, +2.52 | -3.22, +2.40 | |
| s7-elblag-lowsun | vision | decl | 119 / 23 | -1.76, +1.80 | -3.11, +2.32 (0.18, 0.18) | -0.09, -0.16 | -3.24, +2.42 | -0.02, +0.03 |
| s7-elblag-lowsun | vision | warp | 112 / 22 | -1.75, +2.29 | -4.61, +4.82 (0.18, 0.23) | -0.08, -0.16 | -4.72, +4.93 | -0.00, +0.03 |
| s7-elblag-lowsun | supercombo | base | 116 / 23 | -0.34, -0.07 | -3.20, +2.45 (0.04, 0.02) | ride's calib.json: -2.86, +2.52 | -3.22, +2.47 | |
| s7-elblag-lowsun | supercombo | decl | 117 / 23 | -1.84, +1.87 | -3.20, +2.39 (0.18, 0.19) | +0.00, -0.06 | -3.22, +2.49 | +0.00, +0.02 |
| s7-elblag-lowsun | supercombo | warp | 120 / 24 | -1.84, +2.36 | -4.70, +4.87 (0.18, 0.24) | +0.00, -0.08 | -4.72, +4.98 | +0.01, +0.01 |
| a1-night | vision | base | 82 / 16 | +0.56, -0.47 | -5.81, -3.24 (0.06, 0.08) | ride's calib.json: -6.36, -2.75 | -5.96, -3.08 | |
| a1-night | vision | decl | 90 / 17 | -0.91, +1.29 | -5.77, -3.48 (0.09, 0.14) | +0.05, -0.24 | -5.90, -3.11 | +0.06, -0.03 |
| a1-night | vision | warp | 92 / 18 | -0.94, +1.80 | -7.30, -0.96 (0.10, 0.19) | +0.01, -0.22 | -7.41, -0.55 | +0.05, +0.04 |
| a1-night | supercombo | base | 158 / 31 | +0.57, -0.21 | -5.79, -3.01 (0.07, 0.04) | ride's calib.json: -6.36, -2.75 | -5.93, -3.05 | |
| a1-night | supercombo | decl | 158 / 31 | -0.91, +1.68 | -5.75, -3.12 (0.09, 0.17) | +0.04, -0.12 | -5.93, -3.04 | -0.00, +0.00 |
| a1-night | supercombo | warp | 153 / 30 | -0.91, +2.16 | -7.27, -0.62 (0.09, 0.22) | +0.02, -0.11 | -7.43, -0.54 | -0.00, +0.00 |

```sh
<datasets>/l33/scripts/val_all.sh        # 12 replays: val.sh RIDE START DUR MODEL TAG DECL_DP DECL_DY WARP_DP WARP_DY
$PY <datasets>/l33/scripts/summ.py       # runs mount.py on each, prints this table
```

Reading:

* **The pictures' vanishing point returned every known change to within 0.06 deg** (8 cases, day and night, both
  axes). Its floor of 0.15 deg is generous on these two rides.
* **The road model's motion returned the pitch to within 0.09 deg and the yaw 0.06-0.24 deg short**, i.e. the model
  shows 88-97 % of a yaw error (vision less than supercombo). Every error is inside the stated uncertainty. After
  the estimate is applied the residual is small and so is this term: a second pass converges.
* With no residual left the two methods differ by 0.02-0.19 deg (base rows).
* Against the rides' own `calib.json` (hand estimates, docs/RIDES.md): both methods agree with each other and
  differ from the files by 0.2-0.5 deg: s7 pitch is -3.0 to -3.2, not -2.86; a1-night is pitch -5.8 to -5.96,
  yaw -3.0 to -3.2, not -6.36 / -2.75. Lane L32's own replays of a1-night with the declared pitch changed by
  as it is and with the declared pitch lowered by 1 deg give the same absolute pitch (-5.78, -5.80 deg;
  `mount.py --min-blocks 3 <datasets>/l32/sweep/a1-night.vision.{base,pitch-1}.jsonl.gz`, one at a time).

Unselected pictures against the hand calibrations (one picture every 2 s of every whole ride, curves, junctions and
tunnels included, no straight selection because no road messages exist for them; `--pictures` mode):

| ride | pictures (agreeing) | support | pictures: pitch, yaw | ride's calib.json | difference |
|---|---|---|---|---|---|
| a1-night | 300 (298) | 74 % | -5.92, -3.18 | -6.36, -2.75 | +0.44, -0.43 |
| a1-night-fog | 360 (325) | 55 % | -2.61, -1.25 | -3.09, -0.40 | +0.49, -0.85 |
| a1-s8-rain | 400 (398) | 20 % | -4.20, +1.20 | -4.30, +1.26 | +0.09, -0.06 |
| forest-lubelskie | 400 (394) | 30 % | -4.86, -3.67 | -4.13, -0.69 | -0.73, -2.99 |
| forest-vistula-spit | 400 (330) | 11 % | -3.51, -5.09 | -2.92, -6.88 | -0.58, +1.78 |
| gdansk-night-snow | 361 (310) | 15 % | -8.04, -1.51 | -6.59, -1.66 | -1.46, +0.15 |
| katowice-rain | 400 (378) | 7 % | -0.41, -5.47 | -1.49, -3.78 | +1.08, -1.69 |
| krakow-2024 | 300 (284) | 8 % | -5.10, +1.59 | -4.87, +0.86 | -0.23, +0.73 |
| krakow-lights | 400 (340) | 10 % | -9.21, +0.36 | -9.40, +0.74 | +0.18, -0.39 |
| krakow-night-tunnel | 400 (386) | 46 % | -3.79, -1.88 | -4.01, -1.20 | +0.22, -0.67 |
| rural-zloczew | 400 (387) | 36 % | -4.41, -5.24 | -4.13, -5.39 | -0.29, +0.14 |
| s1-roadworks | 229 (206) | 11 % | +1.48, +0.83 | -0.00, +2.86 | +1.48, -2.04 |
| s7-elblag-lowsun | 400 (396) | 38 % | -3.23, +2.48 | -2.86, +2.52 | -0.36, -0.04 |
| tunnel-gdansk | 138 (138) | 26 % | -7.50, -0.64 | -6.07, +3.15 | -1.43, -3.80 |
| tunnel-pow-warsaw | 148 (99) | 6 % | -3.15, +15.81 | -5.39, +9.63 | +2.24, +6.18 |
| tunnel-swinoujscie | 97 (91) | 15 % | -5.85, +4.20 | -5.16, -2.64 | -0.69, +6.84 |
| tunnels-s3 | 311 (310) | 46 % | -1.13, -1.41 | -1.89, -3.95 | +0.76, +2.54 |
| warsaw-city | 400 (375) | 12 % | -5.07, +0.38 | -4.93, +0.86 | -0.14, -0.48 |
| warsaw-storm | 400 (328) | 5 % | -10.70, -6.84 | -10.20, -7.10 | -0.50, +0.26 |
| winter-fog | 193 (188) | 40 % | -5.68, -0.83 | -5.16, +0.00 | -0.52, -0.83 |
| winter-snow-rural | 400 (363) | 10 % | -4.30, -4.83 | -2.92, -5.39 | -1.38, +0.56 |

```sh
<datasets>/l33/scripts/ridepics.sh; $PY <datasets>/l33/scripts/ridepics_eval.py
$PY tools/calib/mount.py --pictures <scratch>/l33/ridepics/winter-fog --focal 690      # one ride
```

About half of the rides agree within 0.5 deg, the rest differ by 0.7-7 deg. To find out which side is wrong, three of the
disputed rides were replayed (supercombo, 150-180 s) and measured with straight selection, both methods:

| ride | road model's motion | pictures on the straight stretches | whole ride, unselected (above) | ride's calib.json | verdict |
|---|---|---|---|---|---|
| forest-lubelskie | -4.82, -3.58 | -4.83, -3.57 | -4.86, -3.67 | -4.13, -0.69 | the file's yaw is 2.9 deg off; both methods agree to 0.01 deg |
| tunnel-swinoujscie | -6.58, +1.66 (26 s) | -6.81, +1.96 | -5.85, +4.20 | -5.16, -2.64 | the file's yaw is 4.3-4.6 deg off; unselected pictures of the curved tunnel are 2.2 deg off as well |
| tunnels-s3 | -1.76, -4.18, **refused**: the two halves of the window differ by 2.05 deg in yaw | -1.55, -4.38 | -1.13, -1.41 | -1.89, -3.95 | within this window the file is right and the unselected whole-ride result is not; the recording is not one steady mounting |

```sh
for r in "tunnels-s3 60 180" "forest-lubelskie 60 180" "tunnel-swinoujscie 20 150"; do <datasets>/l33/scripts/val.sh $r supercombo base 0 0 0 0; done
$PY tools/calib/mount.py <datasets>/l33/val/forest-lubelskie.supercombo.base --frames --min-blocks 3
```

So: (a) **some rides' hand calibrations are wrong by degrees in yaw** (forest-lubelskie 2.9, tunnel-swinoujscie
4.3; docs/RIDES.md marks them G and R), which matters for every replay evaluation that reads positions off those
rides; (b) pictures that were not selected for straight driving are only as good as the road in them: on straight
motorway rides they are right, in curved tunnels and winding roads they can be 2-3 deg off while still calling
themselves "ok". `mount.py --pictures` says so in its output; the normal path (a recording) selects the straight
stretches itself.

### The car's recordings

The recorder's sessions are enough for a static estimate: they hold the `road` messages with `pose` at about
13.5 a second (method 1) and one full-size picture a second (method 2). Sessions with straight driving
(`~/roadsense-rec`, supercombo, calibration in use: pitch 0, yaw 0, focal 975):

| session | straight s / blocks | road model's motion: pitch, yaw | pictures: pitch, yaw | pictures minus motion |
|---|---|---|---|---|
| 20261112-141112 | 36 / 4 | +1.28 +- 0.15, -3.25 +- 0.32 | +1.22, -3.58 (40 pictures, 39 agree) | -0.06, -0.33 |
| 20261117-101013 | 487 / 60 | -1.07 +- 0.11, -3.34 +- 0.33 | -1.17, -3.58 (400, 390 agree) | -0.09, -0.24 |
| 20261117-145921 | 14 / 1 | not enough (rough +1.7, -3.3) | +1.45, -3.95 (15, 15 agree) | |
| 20261117-163836 | 156 / 21 | +1.30 +- 0.13, -3.49 +- 0.35 | +1.32, -3.96 (158, 158 agree) | +0.03, -0.46 |

The other sessions (20261116-115104, 20261117-132015, -190932, -191703, -220714, -224454, 20261118-*) hold no
straight driving above 24 km/h.

```sh
for s in 20261112-141112 20261117-101013 20261117-163836; do $PY tools/calib/mount.py ~/roadsense-rec/$s --frames --min-blocks 3; done
$PY tools/calib/mount.py ~/roadsense-rec/2026111[2678]-*          # all together: refused, "the camera was moved between them"
```

* **Yaw: the camera looks 3.3-3.9 deg to the right of the car's direction of travel** (yaw negative), with the
  calibration file saying 0. The two methods agree on it within the expected shortfall of method 1. At 60 m that is
  3.5-4.1 m sideways: the width of one lane, the size of the owner's symptom. This is the same finding as lane
  L32's (-3.4 deg), measured here a second, model-free way.
* **Pitch: the camera did not keep its pitch between drives.** +1.3 deg on 12 Nov, -1.1 deg on the morning of
  17 Nov, +1.3 to +1.5 deg that afternoon and evening; both methods say so to 0.1 deg, and the bonnet sits at a
  different height in the pictures (and at yet another on 18 Nov, where there is no driving to measure). The yaw of
  the last two sessions is 0.4 deg further right in the pictures as well. A mounting that moves by 2.4 deg cannot be
  calibrated once; the tool refuses to mix such recordings.
* There is **no straight drive with the camera as it is mounted now**, and none with the vision model.

## 4. Apply, verify, go back

`apply.py` (docstring and docs/CALIBRATION.md) writes atomically, after copying every file it touches to
`<config dir>/calibration-backups/<stamp>/` with a manifest; reads the files back the way the daemons' flat JSON
readers do; refuses a lens or mount report whose status is not ok and values that cannot be right (radians beyond
20 deg, a height in centimetres, a centre 15 % off the middle, a distortion that folds back); warns about unusual
ones. When the lens values change it keeps the pixel where the road vanishes and re-expresses the angles. The view
shows the horizon, the road's vanishing point, a 3.5 m lane, lines at 10 / 20 / 50 m and the outline of the narrow
model input on up to four pictures from the straight stretches of a session.

Run on a copy of the config with the car's session of 17 Nov 16:38 (yaw and pitch from that session, pinhole 975):
the red mark sits where the lane lines meet on all four pictures.

```sh
C=<scratch>/l33/cfg; mkdir -p $C; cp config/calib.json config/camera-dvr.json $C/
$PY tools/calib/mount.py ~/roadsense-rec/20261117-163836 --frames --out $C/mount.json
$PY tools/calib/apply.py --config-dir $C --camera-file camera-dvr.json --mount $C/mount.json \
    --height-m 1.25 --camera-to-front-m 1.6 --lateral-offset-m 0.02 --session ~/roadsense-rec/20261117-163836
$PY tools/calib/apply.py --config-dir $C --rollback
```

(The three tape numbers in this command are placeholders to exercise the tool, not measurements.)

## 5. Tape-measured values

| value | place | read by | check |
|---|---|---|---|
| lens height above the road | `calib.json` `height_m` (existing key) | laned, overlays, road-plane distances | 0.5-4 m, warning outside 0.9-2.0; below the roof and equal to `camera.z_m` if a vehicle description is given |
| lens to front bumper | `lead.json` `camera_to_front_m` (existing key of the lead tracker) and a copy in `calib.json` | api `lead.gap_m` | 0-6 m, warning outside 0.8-3.0; `camera.x_m` + it = `rear_axle_to_front_m` |
| lens off the centre line | `calib.json` `lateral_offset_m` (new, positive to the right as in the camera frame) | nothing yet | within 1.5 m and inside the car's width, warning beyond 0.6 |

The host's vehicle description (docs/VEHICLE_API.md 5.6, `camera: {x_m, y_m, z_m}`, y positive to the left) carries
the same position for the range-sensor relay; it is the host's to send, `apply.py --vehicle FILE` only checks
against it.

## 6. Proven, synthetic only, needs the car

| | |
|---|---|
| **proven on rides** | the mount estimate returns a known turn of the camera and a known wrong declaration (2 rides, 2 models, 8 cases: pictures within 0.06 deg, road model's motion within 0.24 deg and inside its stated error); straight-stretch selection; the recorder's data is enough; the view on real pictures; capture against a real frame ring fed from a recording |
| **proven on the car's recordings** | both mount methods agree to 0.1 deg in pitch and to the expected shortfall in yaw on three sessions; the yaw (3.3-3.9 deg) and the unstable pitch |
| **synthetic only** | the whole lens path (board detection on rendered pictures, the solver, its uncertainties and refusals), `apply.py` on files in a scratch directory, the capture's still / moved logic |
| **needs the car** | 30-40 board pictures through the windscreen and the lens solve on them; a straight drive with the camera as mounted now (and fixed so that it keeps its pitch); the three tape measurements; `capture.py` and `apply.py` on the phone (**not run on the phone**: Python 3 with numpy is assumed from mapd's needs, Pillow from the api's; `capture.py` needs neither) |

## 7. Notes for the fold

* **Overlap with lane L32.** L32's worktree holds an online estimator of pitch and yaw inside modeld
  (`modeld/calibrator.*`) and `tools/vision_eval/calib_from_pose.py`, which is method 1 of `mount.py` (same
  observation, same gates). This lane adds the model-free second method, the perturbation proof, the refusal on
  moved cameras, and the lens / apply / tape parts. If both merge, one of the two offline pose tools should go;
  `mount.py` carries the acceleration gate, the stretch selection and the error estimate, L32's mirrors modeld's
  arithmetic exactly.
* Shared files touched: `docs/ARCHITECTURE.md` (a "Calibration tools" subsection under "Calibration"),
  `docs/ROADMAP.md` (two sub-items under the calibration item). L32 may touch the same paragraphs.
* Nothing under `modeld/`, `api/` or `laned/` was changed.
* Scratch: `<datasets>/l33` (venv, build of `modeld_replay` / `camerad-replay` from this branch, `val/`
  replays, scripts), `<scratch>/l33` (ride pictures, ring test). Heavy jobs ran in
  `systemd-run --user -p MemoryMax=` units.

## 8. Hand-back (2026-11-19, 12:45 CET)

**FAV**

* The tools exist and are tested: board, capture, lens solver, mount estimator, apply / verify / rollback, the
  owner's procedure. 33 tool tests pass, the api suite is unchanged at 252.
* The mount estimate is proven on rides: a camera turned by a known angle and a calibration declared wrong by a known
  angle both come back, on a day and a night ride, with both road models: the pictures' vanishing point within
  0.06 deg, the road model's motion within 0.24 deg and inside the error it states.
* The recorder's sessions are enough (messages at 13.5 a second plus one picture a second). On the car's recordings
  two independent methods agree: the camera looks **3.3-3.9 deg to the right** of the direction of travel while
  `calib.json` says 0. At 60 m that angle is 3.5-4.1 m: one lane's width, the size of the owner's symptom.
* The lens distortion is quantified and is not that cause: 0.05-0.2 pixel near the vanishing point, a few per cent
  on speed and lane width. The change that removes it is specified and costs nothing per frame.
* The board page draws exactly OpenCV's board (pixel for pixel on a screen, 30.00 mm squares on paper).

**UNFAV**

* The lens path is proven on synthetic pictures only. No picture of a board through the car's camera exists on the
  laptop; a real screen, the windscreen and the camera's focus are untested.
* **The camera does not keep its pitch**: +1.3, -1.1, +1.3 to +1.5 deg over three days, and visibly different again
  on 18 Nov. A one-off calibration will not hold until the mounting is fixed (or pitch is estimated online, lane
  L32). There is no straight drive with the camera as it is mounted now, and none with the vision model.
* The road model's motion shows only 88-97 % of a yaw error on the first pass (hence the second drive, or the
  pictures).
* Pictures not selected for straight driving can be 2-3 deg off and still say "ok" (curved tunnels); only the
  recording path selects.
* Several rides' hand calibrations are wrong by degrees in yaw (forest-lubelskie 2.9, tunnel-swinoujscie 4.3):
  replay results that depend on sideways positions on those rides carry that error.
* Phone-side pieces (`capture.py`, `apply.py`) were **not run on the phone**. Roll and height cannot be seen from
  driving; roll is unchecked.
* Method 1 duplicates lane L32's `calib_from_pose.py`; two shared doc paragraphs may conflict at merge.

**INSTR**

* Owner, at the car (docs/CALIBRATION.md): fix the camera so that it keeps its aim; record 3-5 minutes of straight
  driving above 25 km/h with the recorder on; take 30-40 board pictures through the windscreen (corners, tilted
  views); measure lens height, lens to front bumper, lens off the centre line.
* Then on the laptop: `lens.py solve`, `mount.py --frames --lens`, `apply.py --dry-run` and look at the view, then
  `apply.py` on the phone; drive once more and check that `mount.py` reports a residual near zero.
* For a later lane: the distortion in the input warp (section 2); re-measure the rides' `calib.json` with
  `mount.py` (one replay per ride); decide which offline pose tool stays (this one or L32's).

**Decision** (owner): merge `v2-l33` and do the car steps; and whether to install the yaw from the existing
recordings now as an interim (about -3.9 deg from the latest two sessions' pictures, -3.5 from the road model's
motion; the pitch has to wait for a drive with the present mounting).
