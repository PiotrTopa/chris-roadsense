# Deploying the USB DVR camera path on the phone

Everything below was verified on the laptop on 2026-10-22; the phone side is a rebuild
plus a unit switch. Phone: Poco F1, postmarketOS, user `user`, sudo password in the vault
(`poco_user_password`), Wi-Fi 192.168.0.86. Dev checkouts live in `~user/roadsense` and
`~user/roadsense-signs`; models in `~user/roadsense/models`; ncnn built in
`~user/vulkan-work/ncnn`.

## What the laptop established

| item | result | where |
|---|---|---|
| camera protocol | plain sector reads of the mass-storage volume; no commands needed | docs/HUIYING_DVR_PROTOCOL.md, tools/dvr_probe.py |
| stream | 1280x720 baseline JPEG, 20 fps, 46-64 KB/frame | same |
| latency (LED reference) | min 30 / median 56 / max 84 ms glass-to-host | tools/ledlat |
| intrinsics | focal 975 px (953 pitch, 995 calibrateCamera), HFOV 68 deg, k1 -0.34 | config/camera-dvr.json |
| overlay | date/speed burned into the bottom-left ~40 rows | not cropped yet |
| camerad-dvr | 19.5 fps into the ring, 1.7 ms decode on x86, 4 % CPU, auto-reconnect | camerad-dvr/ |
| bug fixed | ring magic was still `PPFRAME1` on the phone build; api never read the header | camerad/ring.h |

## Phone prerequisites

- USB **host mode with the VBUS boost**: the hub and camera only enumerate when the phone
  sources VBUS (`sudo poco-vbus apply && sudo poco-usb host --reboot`). The phone then does
  not charge over USB; run it from the battery eliminator (4.0-4.2 V, NTC resistor) or an
  OTG Y-cable with a power input.
- Packages (already installed): `libjpeg-turbo-dev`, `onnxruntime-dev`, `g++`, `cmake`.
- `roadsense.target` autostart is disabled (thermal); start by hand once cooling is in place.

## Steps (from the laptop)

```sh
cd ~/projects/roadsense
scripts/sync-phone.sh                       # tree -> phone ~/roadsense
scripts/push-models.sh                      # supercombo-fp32.onnx + supercombo-ncnn.* (+ sign models live in roadsense-signs)
```

On the phone (`ssh user@192.168.0.86`), as root:

```sh
cd ~user/roadsense
CMAKE_ARGS="-DROADSENSE_NCNN=ON -DNCNN_DIR=/home/user/vulkan-work/ncnn" BUILD=/var/tmp/roadsense-build sh scripts/install-phone.sh
BUILD=/var/tmp/roadsense-build sh scripts/install-camerad-dvr.sh --enable   # masks camerad, starts camerad-dvr --auto
cd ~user/roadsense-signs && BUILD=/var/tmp/roadsense-signs-build ROADSENSE_DIR=/home/user/roadsense sh scripts/install-phone.sh
systemctl start roadsense.target
```

The rebuild also fixes the ring magic for the api. `install-phone.sh` reinstalls the
`roadsense-camerad` unit file, so re-run `install-camerad-dvr.sh --enable` after every
`install-phone.sh`. Back to the phone camera: `install-camerad-dvr.sh --disable`.

## Verify

```sh
journalctl -u roadsense-camerad-dvr -n 3        # "ring 1280x720 ... 19.x fps", no "reopen" loops
curl http://192.168.0.86:8080/v1/camera          # "source":"dvr", width 1280, focal_px 975
curl http://192.168.0.86:8080/v1/health          # camera ~19.5 fps, road > 9/s (GPU backend)
http://192.168.0.86:8080/                        # preview must show the camera, lanes drawn
```

Expected on the phone (not yet measured): decode 5-7 ms per frame on an A75, camerad-dvr
well under 20 % of one core; modeld unchanged (GPU); the DVR camera adds no SoC heat.

## Open items for the phone

- Measure latency on the phone with `tools/ledlat` (the firmware is for the RP2040-Zero; the
  test script needs `/dev/ttyACM0` and root).
- Full lens calibration (15-20 board views) and optional undistortion in camerad-dvr.
- Decide the model crop versus the burned-in overlay (`--crop-bottom` is not implemented).
- The camera clock: `tools/dvr_probe.py --cmd sync_time` writes one command sector.
