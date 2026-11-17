# Deploying the USB DVR camera path on the phone

**Deployed and verified on the phone 2026-10-23** (results under "Measured on the phone").

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
- `roadsense.target` autostart is disabled: on the Prius car computer the car backend starts it
  with ACC and stops it at key-off (docs/ARCHITECTURE.md "Running under a host controller"); on the
  bench, `systemctl start roadsense.target` by hand or the backend's `roadsense_mode on` command.
- The car computer's own backend serves port 8080, so the roadsense API runs on **8090** there:
  `API_PORT=8090 sh scripts/install-phone.sh` (writes `/etc/roadsense/api.env`).

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

`install-phone.sh` no longer enables boot autostart unless `AUTOSTART=1` is set.
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

## Measured on the phone (2026-10-23, camera behind the Terminus 1a40:0101 hub)

| item | result |
|---|---|
| camerad-dvr | 19.5 fps into the ring, 0 drops, read 21 ms + convert 17 ms per frame, ~3 % CPU |
| api | `/v1/camera` source dvr, 1280x720, focal 975; `/v1/frame.jpg` live |
| modeld (GPU) | 78.7 ms avg; 9.7 runs/s after the 80 C throttle halved the rate |
| signd (big cores) | detect 196 ms, 4.7 fps |
| heat | 89 C after 15 s, 95 C after ~1 min with signd on the big cores; stack stopped |

Without active cooling the full stack cannot run; modeld + camerad-dvr alone is the
realistic configuration until the fan is fitted (signd is the heat source).

**VBUS boost trip.** After a long uptime the port gave 0 V and nothing enumerated: the PMI8998
OTG block showed CMD_OTG (0x1140) = 1 but OTG_STATUS (0x1109) = 0x04 and the OTG fail bit in
0x1110 set, i.e. the boost had latched off (likely an inrush or overcurrent when the camera or
hub was plugged in). Unbinding the regulator driver does not reset an always-on regulator and
the regmap debugfs is read-only, so the fix is a reboot with the camera already attached;
healthy state is 0x1109 = 0x0a. Check with
`sudo grep -E "^(1109|1110|1140):" /sys/kernel/debug/regmap/0-02/registers`.

## USB drop-outs after power-up

**Fixed in camerad-dvr (2026-11-17): the drop-out is the camera waiting for the vendor app's first
command.** The app sends a time sync (opcode 11, `CAM_SET_TIME`) about 2 s after it connects; a
host that only reads never does, and the camera then re-enumerates once. Measured on the car, hub
power switched by relay, 20-30 s off between runs:

| | runs | camera left the bus |
|---|---|---|
| nothing sent (with or without reads of the info sector) | 4 | 4, at 28.0-28.7 s after power, back after 4.7-4.9 s (once not within 47 s) |
| time sync once, 3-4 s after the camera appeared | 5 | 0 in the 75-110 s watched |
| time sync every 5 s | 1 | 0 in 75 s |
| camerad-dvr with the fix, started 13 s after power | 3 | 0 in 110 s, frames flowing |

camerad-dvr now sends that one sector on the first valid info sector of every connection
(`time sync sent` in its log). It writes only when the FAT shows `Android/data/com.fvision.camera/HWC`
and the command sector (LBA 0x340 layout A, 0x2C0 layout B) lies inside that file; otherwise it logs
`time sync not sent` and stays read-only, as with `--no-set-time`. The camera's clock is set to the
host's local time as a side effect. The command must arrive before the ~28 s mark: a camera powered
long before the daemon starts (early camera power) still does its one drop-out, before the stream is
needed. The text below describes the behaviour without the command.


The DVR camera leaves the USB bus once after every power-up: it enumerates 2-4 s after its hub
gets power, disconnects 24-39 s later and is back after 4-5 s. On some cold starts (first power-up
after hours unpowered) it stayed away about 120 s, came back for 2 s, left again for 4 s. While it
is away no block device exists; `camerad-dvr --auto` retries every second and resumes on its own.
A host controller can move this sequence ahead of the drive by powering the camera earlier
(see docs/ARCHITECTURE.md "Running under a host controller").

Port power cycle tested and rejected (hub VIA `2109:2817`, advertises per-port power switching,
`wHubCharacteristic 0x00e9`; `uhubctl` 2.5.0; kernel 7.1):

| test | result |
|---|---|
| `uhubctl -l <hub> -p <port> -a off` | status `0000 off`, but the kernel re-powered the port within about 1 s |
| sysfs `<hub>:1.0/<hub>-port<N>/disable` = 1 for 4 s | port held `0000 off`; the camera was back 1 s after `disable` = 0 and did not leave the bus again in the next 54 s |

A real power-up takes 2-4 s to enumerate and is always followed by the drop-out, so the camera
kept running through the port switch (the hub does not cut VBUS, or the camera bridges a short
cut). A port cycle cannot restart the camera and so cannot end its long absence; no watchdog is
shipped. Restarting the camera means cutting the hub's own supply, which drops every other
device on the hub and restarts the camera's power-up sequence; that is a decision for the host
controller.

## Open items for the phone

- Measure latency on the phone with `tools/ledlat` (the firmware is for the RP2040-Zero; the
  test script needs `/dev/ttyACM0` and root).
- Full lens calibration (15-20 board views) and optional undistortion in camerad-dvr.
- Decide the model crop versus the burned-in overlay (`--crop-bottom` is not implemented).
- The camera clock (shows 2022 in the overlay): `tools/dvr_probe.py --cmd sync_time` writes one command sector.
- A boost watchdog that power-cycles OTG when 0x1109 leaves state 2 needs a register write path (small kernel module or driver patch).
