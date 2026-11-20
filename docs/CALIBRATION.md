# Camera calibration: the owner's procedure

Two things are measured, in this order, plus three numbers from a tape measure:

1. **the lens** (focal length, picture centre, distortion): pictures of a board, once per camera;
2. **the mounting** (pitch, yaw): a straight drive, again after every time the camera was moved;
3. **tape**: lens height above the road, lens to front bumper, lens off the car's centre line.

Tools: `tools/calib/` (lane L33, [lanes/L33-calibration-tools.md](lanes/L33-calibration-tools.md)). The pictures are
taken on the camera host (the phone; standard-library Python, nothing to install), everything else runs on the
laptop. The laptop needs OpenCV once:

```sh
python3 -m venv <datasets>/l33/venv && <datasets>/l33/venv/bin/pip install numpy pillow opencv-contrib-python-headless
PY=<datasets>/l33/venv/bin/python          # used below; cd to the roadsense checkout
```

Order matters: the mounting angles are angles *for a given lens calibration*. The mounting may still be done
first, with the lens values as they are (leave out `--lens`): it is the yaw that puts a far vehicle a lane to the
side. When the lens values change later, `apply.py` re-expresses the angles (the road keeps vanishing at the same
pixel), and one more straight drive confirms them.

## 1. Lens: pictures of the board

**The board.** Open `tools/calib/board.html` in a browser on the largest flat screen at hand (a laptop works, a
monitor or TV is better; no network needed). Enter the lit width of the screen in mm, press F11, press H to hide
the bar. Brightness about half, no reflections, a lit garage or daylight rather than darkness (the camera would
overexpose a bright screen in the dark). Note the line the page shows, e.g. `--board charuco:8x5 --square-mm 34.50`.
Paper instead: set "printed square", Print at 100 %, glue the sheet to something flat, measure one square.

**Where to hold it.** In front of the camera, **through the windscreen as the camera is mounted** (the glass is part
of the optics). The board should fill **a third to a half of the picture width**: a 15.6" laptop at 0.6-1.0 m, a 24"
monitor at 1.0-1.5 m. If it has to be further away, choose 6x4 squares so that the markers stay readable.

**Taking the pictures** (camera stack running as usual; nothing is stopped). On the phone:

```sh
python3 ~/roadsense/tools/calib/capture.py --out ~/calib-shots --count 40
```

It keeps a picture whenever the scene holds still and differs from the last kept one, and prints a line each time:
hold the board still for a second, move it, hold again. **30-40 pictures:**

* the board in the middle, at each of the four edges and **in each of the four corners** of the picture (a ChArUco
  board may hang out of the picture: partly visible is fine, and is how the corners get covered);
* **at least a third of them with the board tilted 20-40 degrees** (left edge towards the camera, right edge, top,
  bottom). With the board always facing the camera the focal length cannot be measured, and the solver refuses;
* nothing moving while a picture is taken.

Alternatives that give the same folder of pictures: a recorder session made while showing the board (one picture a
second: `lens.py solve SESSION`), a video (`lens.py solve FILE.mp4 --every 10`), or the camera on the laptop.

**Solving** (laptop):

```sh
scp -r user@phone:calib-shots ~/calib/shots
$PY tools/calib/lens.py solve ~/calib/shots --board charuco:8x5 --square-mm 34.5 \
    --out ~/calib/lens.json --debug-dir ~/calib/lens-debug
```

It prints the focal length, the picture centre and the distortion, each with its uncertainty, the reprojection error
(good: below 0.5 px) and a map of where the board corners fell. Look at `lens-debug/coverage.png`: red cells are
empty. **If it says REFUSED, it says why** (too few distinct views, no tilted views, too little of the picture
covered, a poor fit, an uncertainty beyond 1.5 % of the focal length or 0.46 deg of the centre): take more pictures
of what is missing into a second folder and solve both folders together. A warning about empty picture corners
means the distortion out there is extrapolated; add corner pictures if you can.

## 2. Mounting: a straight drive

Record a drive with the recorder on (`recorder.json`, docs/ARCHITECTURE.md "Drive recorder"): a **straight, level
road, steady speed above 25 km/h (50-90 is best), 3-5 minutes of straight driving in total**, the car loaded as
usual, staying in the lane. Curves, junctions and stops in between do not hurt; the tool cuts them out. Clear lane
lines help the second method. Copy the session to the laptop (`scp -r user@phone:/var/lib/roadsense/rec/<session> ~/roadsense-rec/`), then:

```sh
$PY tools/calib/mount.py ~/roadsense-rec/<session> --frames --lens ~/calib/lens.json --out ~/calib/mount.json
```

It reports pitch and yaw twice: from the road model's own motion and from the vanishing point of the lines in the
recorded pictures, each with its spread over time and an error estimate, and then `recommended`: per angle the
estimate with the smaller error when the two agree, nothing when they contradict each other. It says
`NOT ENOUGH` when there was too little straight driving (it tells how many seconds it found and needs), when the
blocks disagree, or when recordings disagree with each other (**the camera was moved between them**: give only
recordings made after the last change). Roll and height cannot be seen from driving.

After a calibration that was far off (more than 1 deg), drive and run it once more: the road model shows 3-12 %
less than the full yaw error (measured on rides), so the second pass should report a residual near zero. The
pictures' estimate does not have that shortfall, which is why the recommendation takes the yaw from the pictures
when they are clear.

## 3. Tape

| value | how | goes to |
|---|---|---|
| `--height-m` | lens centre straight down to the road, car standing level, loaded as usual | `calib.json` `height_m` (every distance on the road plane scales with it) |
| `--camera-to-front-m` | lens to the front edge of the bumper, measured along the car (horizontally) | `calib.json` and `lead.json` `camera_to_front_m` (the lead's `gap_m`) |
| `--lateral-offset-m` | lens right (+) or left (-) of the car's centre line | `calib.json` `lateral_offset_m` (recorded; the host's vehicle description carries the same as `camera.y_m`, there positive to the left) |

`apply.py` refuses values that cannot be right (centimetres instead of metres, a lens above the roof) and warns about
unusual ones; `--vehicle FILE` checks them against a vehicle description (docs/VEHICLE_API.md 5.6).

## 4. Apply and check

First look, without changing anything (laptop; a copy of the phone's `/etc/roadsense` in `~/calib/etc`):

```sh
scp user@phone:/etc/roadsense/{calib.json,camera-dvr.json} ~/calib/etc/
$PY tools/calib/apply.py --config-dir ~/calib/etc --camera-file camera-dvr.json \
    --lens ~/calib/lens.json --mount ~/calib/mount.json \
    --height-m 1.25 --camera-to-front-m 1.60 --lateral-offset-m 0.00 \
    --session ~/roadsense-rec/<session> --view ~/calib/view.jpg --dry-run
```

**Open `view.jpg`.** On the straight-road pictures: the **red mark** sits where the lane lines meet, the **yellow
line** lies on the horizon, the **green lines** run along the lane (3.5 m wide; narrower lanes sit inside them), the
**blue lines** are 10, 20 and 50 m ahead (on a Polish expressway a dash is 4 m long and the next one starts 12 m after it: that gives the scale).
If the red mark is off to one side, the yaw is wrong; above or below the meeting point, the pitch; lane too wide or
too narrow at every distance, the height or the focal length.

Then for real, on the phone (copy `lens.json`, `mount.json` and one road picture over, or run with the same
numbers), as root:

```sh
sudo python3 ~/roadsense/tools/calib/apply.py --config-dir /etc/roadsense --camera-file camera-dvr.json \
    --lens lens.json --mount mount.json --height-m 1.25 --camera-to-front-m 1.60 --lateral-offset-m 0.00
sudo systemctl restart roadsense-camerad-dvr roadsense-api     # focal length / centre, lead.json
```

`calib.json` is picked up by modeld and laned within 2 s. The previous files are kept in
`/etc/roadsense/calibration-backups/<stamp>/` together with a note of what was changed.

## 5. Going back

```sh
sudo python3 ~/roadsense/tools/calib/apply.py --config-dir /etc/roadsense --rollback            # the last apply
sudo python3 ~/roadsense/tools/calib/apply.py --config-dir /etc/roadsense --rollback <stamp>    # a named one
sudo systemctl restart roadsense-camerad-dvr roadsense-api
```

A rollback first saves the files it replaces, so it can itself be undone.

## What roadsense does with the values today

| value | file | used by |
|---|---|---|
| `focal_px`, `cx`, `cy` | camera file (`camera-dvr.json`) -> frame ring header | modeld's input warp, every overlay and distance |
| `k1`, `k2` (`p1`, `p2`, `k3` = 0) | camera file | **recorded only.** The warp treats the camera as a pinhole; what that costs and the change that removes it: lane report L33, "Distortion" |
| `pitch`, `yaw`, `roll` | `calib.json` | modeld's input warp, laned, overlays |
| `height_m` | `calib.json` | laned, overlays, distances on the road plane |
| `camera_to_front_m` | `lead.json` | `lead.gap_m` |
| `lateral_offset_m` | `calib.json` | nothing yet |
