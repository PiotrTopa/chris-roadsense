# D1: the 2.0 deployment on the car computer (Poco F1), on the bench

Execution lane, 2026-11-19 06:36-07:03 CET, Opus. Phone on the bench (no camera, no gateway, MFD Pi not
connected), reached over its access point. Owner's approval: "deploy yes, whenever makes sense".

## Installed

| piece | version | how |
|---|---|---|
| roadsense | main c35dd53 (api 2.0.0) | git tree to `/home/user/d1/roadsense`, built there (`build/`, ncnn `/home/user/vulkan-work/ncnn`), `CMAKE_ARGS="-DROADSENSE_NCNN=ON -DNCNN_DIR=..." sh scripts/install-phone.sh` |
| roadsense-signs | main 86624a9 | `/home/user/d1/roadsense-signs`, `ROADSENSE_DIR=/home/user/d1/roadsense NCNN_DIR=... sh scripts/install-phone.sh` |
| road model files | v0.11.1, sha256 as in L27 section 1 | copied from `/home/user/l28/models` (identical to `/home/user/l27/models` and the laptop's `/mnt/win/linux-scratch/l27/models`) |
| light-state model | `lights_state_car_48x24.onnx` 97bfddf6... | from the laptop (`~/datasets/l26/models`, same as `wt/signs-L21`); was not installed before (signd used the BDD model) |
| map pack | `poland-20261114-f2.rsmap`, format 2, sha256 838442af14db9ec45172f2f276a7cad0f9db4edeb0a9b2893cddae7209ccc1c5 | `scripts/roadsense-maps install --host user@10.42.0.1`, then the symlink `current.rsmap` |
| car backend | car repo main ddee72a (working tree; `user_settings.json` change and the untracked fixture not part of the app) | the app step of `POCO/sync.sh --app` (rsync of `cyberpunk_computer/`, `requirements.txt`, powerbox files, `dashboard/dist/`; `prius-backend ensure-venv`; restart). The overlay/provision part was not run: its `--core` files were already identical on the phone (sha256 compared), the rest is the car network layer |

Unit states as found and left: `roadsense.target` disabled, `roadsense-laned.service` installed and disabled,
`roadsense-signd` and `roadsense-camerad-dvr` enabled (wanted by the target), `roadsense-camerad` masked.
Configs in `/etc/roadsense` kept (installer does not overwrite); new: `laned.json`. Sign proposals stay off.

## Verified (bench)

* Stack started through the backend (`POST /api/v1/commands/roadsense_mode {"mode":"on"}`): every unit
  starts; without a camera camerad-dvr retries every second and modeld / envd / signd wait for the ring.
  `/v1/health`: `api.major` 2, governor enabled at level 0; `/v2/governor` 200. Backend proxy
  `/api/v1/roadsense/health` 200, position feeder connected to mapd; mapd loaded the format-2 pack.
* Frames from `krakow-25-35.mjpeg` through the installed `camerad-replay` as a transient unit (no config
  touched; calib stays the DVR's), 2.5 min, fan automatic: camera 19.7-19.9 fps, road / lead / lanes / view
  14.6-15.4 a second, signs and lights 4.5-5.0, env 0.8-1.0; `road_model` = vision, no fallback, plan on,
  `policy_hz` 5; model step 40-58 ms; `view.lines_source` = model; hottest zone 82 C, governor stayed at 0.
* MFD scene path: the deployed backend's `RoadsenseSceneFeed` run against the live `/ws` for 30 s:
  298 scenes, 9.9 a second, lead and lanes `tracked`, path present. (The backend's own feed stays idle on the
  bench because it runs only while the MFD is powered.)

## Not done

* MFD frontend on the Pi Zero (`sudo prius-deploy-mfd`): Pi not connected (no `usb0`).
* No on-car run; no camera on the bench.

## Rollback

`/home/user/rollback-20261119/` on the phone (root 0700), made before any change: `roadsense-bin.tar.gz`,
`roadsense-lib-etc-units.tar.gz`, `roadsense-models.tar`, `backend-app.tar.gz`, `car-overlay.tar.gz`,
`state-before.txt`, `SHA256SUMS`, and `RESTORE.txt` with the exact commands.

## Follow-up: main ef0c2dd (same day, 07:12-07:17 CET)

The only code change since c35dd53 is `api/roadsense_api/lead.py` (vision lead profile `weak_coast_s`
2.5 -> 1.5). Rollback first: the installed api as `api-before-ef0c2dd.tar.gz` in
`/home/user/rollback-20261119/` (in `SHA256SUMS`, commands appended to `RESTORE.txt`). The git tree went to
`/home/user/d1/roadsense` and `scripts/install-phone.sh` ran as before: nothing was relinked and all 14
installed binaries kept their sha256; the installed api equals the source tree and `lead.py` carries 1.5.
Check as before (stack through the backend, about 1 min of `krakow-25-35.mjpeg`): api 2.0.0, road model
vision without fallback, plan at 5 Hz, road / lead / lanes / view 14.8-15.1 a second, governor level 0,
hottest 74 C, `/v2/governor` and the backend's proxy 200. Left with `roadsense.target` inactive, mode auto,
fan automatic, lock removed.

## Follow-up 2: main dcd043f (online camera calibration), in the car, 13:06-13:14 CET

Phone in the parked car (key off), reached over WireGuard, on the car's 12 V battery (12.75 V at the start,
lowest 12.34 V during the check, 12.59 V after). Rollback first: `roadsense-before-dcd043f.tar.gz` (binaries,
api + mapd, `/etc/roadsense`, units, nft rule) in `/home/user/rollback-20261119/`, in `SHA256SUMS`, commands in
`RESTORE.txt`. Git tree to `/home/user/d1/roadsense`, built there (13:09-13:10; `calibrator_test` all passed),
`scripts/install-phone.sh` as before. Changed binaries: only `modeld` (783672d6...a9b0 -> d0eed72b8e687798ec345e208f428dd4f748825302ff2b5cf7ddce8e3b9dfb8e)
and `modeld_replay` (-> 4ef2a636...a33a); the other 12 kept their sha256. Installed api and modeld unit equal
the tree. `/etc/roadsense/calib.json` (pitch 0.0561, yaw -0.0621, set by hand) unchanged (sha256 41d8a6a2...),
`calib.json.bak-20261119-nominal` kept, no `modeld.json` (so `auto_calib` is the default `on`).

Check without teaching the estimator another camera: a runtime drop-in
(`/run/systemd/system/roadsense-modeld.service.d/99-d1-verify.conf`, removed afterwards) ran modeld with
`--auto-calib observe --calib-state none`. Stack through the backend; first the car's DVR camera for 30 s
(camera 19.2 fps, road / lead / lanes / view 14.6 a second, vision, no fallback), then 1 min of
`krakow-25-35.mjpeg` (camerad-dvr stopped for it): camera 19.6 fps, road / lead / lanes / view 14.7-14.8,
signs / lights 4.5-5.0, vision without fallback, plan at 5 Hz, governor 0, hottest 76 C. `/v2/health`
`calibration`: source `config` with the hand values, `online.state` `collecting`, `applied` false, 0 blocks.
No `/var/lib/roadsense/camera-calibration.json` was written. Left: target inactive (the backend starts it at
key-on), mode auto, fan automatic, no transient units, lock removed. The first drive will start the estimator
from nothing (`collecting`, then valid after 5 blocks of straight driving above 24 km/h).
