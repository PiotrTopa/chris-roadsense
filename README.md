# roadsense

Car-agnostic, device-agnostic road perception service, in the spirit of
[openpilot](https://github.com/commaai/openpilot) /
[flowpilot](https://github.com/flowdriveai/flowpilot) (flowpilot is GPL-3.0, so it
is a source of ideas only; nothing from it is copied): a forward-looking camera
watches the road and a few small daemons turn the stream into structured road
understanding, served over a local HTTP/WebSocket API.

**Scope (current phase): data gathering only.** No car control. The API is the
product; whatever later consumes it (a car gateway, a logger, an HMI) is a
separate project.

## What it serves

| Topic | Source | Content |
|---|---|---|
| `road` | modeld: openpilot supercombo model (v0.9.7, MIT) | planned path, lane lines (4) + probabilities, road edges (2), lead vehicles (x,y,v,a with probability), ego pose (velocity, rotation rates), desire state, meta (blinkers, disengage probabilities) |
| `camera` | camerad | frame metadata (exposure, gain, colour gains, fps), optional MJPEG preview with overlays |
| `signs` | external sign provider (optional), e.g. [roadsense-signs](https://github.com/PiotrTopa/roadsense-signs) | per-frame detections with class, box, score; persistent track ids, first/last seen, best crop |
| `env` | envd: zero-shot CLIP-type image encoder, 1 fps | place (highway, city street, residential, rural/forest road, tunnel, parking lot, underground parking, gas station, bridge), weather (clear, overcast, rain, snow, fog), light (day, dusk/dawn, night): smoothed scores, label, confidence |
| `map` | mapd: position filter + map matching on an OpenStreetMap pack (needs an external position feeder: GNSS fixes and vehicle motion) | position state (good / dead reckoning / unknown / no source), matched road (class, name, ref, lanes, tunnel, bridge, lit, surface), map speed limit (tagged or Polish statutory default, source marked), look-ahead along the most likely path (road shape and curves, junctions, roundabouts, crossings, signals, tunnel entry / exit distance, built-up and limit changes) |
| `fused` | api: map and camera combined per fact, with provenance | speed limit (sign > sign held to the next map junction > map > default), built-up state, place, lane count / ego lane / oncoming neighbour, tunnel with exit distance; vision-only values next to each result |
| `health` | all daemons | fps per stage, model latency, CPU temperature, battery |

## Architecture and contract

```
 camera ──> camerad ──> shared-memory frame ring (NV12 + intrinsics, 20-30 fps)
                               ┌──────────────┴───────────┬──────────────────────┐
                               ▼                          ▼                      ▼
                     modeld (C++, ONNX Runtime / ncnn)   envd (C++, 1 fps)   sign provider (optional, external)
                               │   newline-delimited JSON over Unix sockets  │                      │
                               └──────────────┬───────────┴──────────────────────┘
 position feeder (GNSS, vehicle motion) ──> mapd (Python; map pack) ──┤
                                              ▼
                                    api (Python, aiohttp) ──> HTTP /v1/*, WebSocket /ws, MJPEG /preview
```

The processes share nothing but two mechanisms, specified in
[docs/ARCHITECTURE.md](docs/ARCHITECTURE.md):
- [docs/PHONE_DVR_DEPLOY.md](docs/PHONE_DVR_DEPLOY.md): putting the USB DVR camera path on the phone (findings, steps, verification).


* the **frame ring** `/dev/shm/roadsense-frames`: a 4096-byte header (magic
  `RSFRAME1`, size, strides, focal length and principal point) followed by 6
  NV12 slots with sequence numbers and timestamps; lock-free, one writer, any
  number of readers (`camerad/ring.h` is a header-only reference implementation);
* **JSON sockets** `/run/roadsense/<daemon>.sock`: one JSON object per line
  (`camera`, `road`, `calib`, `signs`); `nc -U` is a valid debugging tool.

### Plugging in a different camera or device

* **Camera:** anything that writes the frame ring per the contract is a valid
  camerad. The bundled camerad is a libcamera client that needs an XRGB8888
  1920x1080 stream (its `--fps` control and default focal length are specific to
  the reference phone's IMX363 and are skipped or overridden by `camera.json`
  elsewhere); a V4L2, GStreamer, video-file or network source only has to fill
  the same header and slots, and publish `camera` messages if the api should
  show camera metadata. `camerad-dvr` is a second bundled source: a USB "DVR"
  head-unit camera (Huiying/Buildwin, USB 1902:3241) that streams 1280x720 MJPEG
  through sectors of a mass-storage volume
  ([docs/HUIYING_DVR_PROTOCOL.md](docs/HUIYING_DVR_PROTOCOL.md),
  docs/ARCHITECTURE.md "Alternative camera sources").
* **Model:** supercombo is device-independent given the intrinsics: modeld reads
  `focal_px`, `cx`, `cy` from the ring header and warps each frame to the
  model's own camera, so a different lens or resolution needs correct intrinsics
  (and the mounting calibration in `calib.json`), not a different model.
* **Compute:** modeld and the api are plain C++17 / Python with ONNX Runtime on
  the CPU; any Linux box that runs ONNX Runtime can host them.
* **Consumers:** further perception daemons (the sign provider is one) read the
  ring and serve their own socket; the api fans out whatever arrives.
* **Position and maps:** any process may feed raw GNSS fixes and the car's motion
  (speed, yaw rate or steering angle or wheel speeds) to `<rundir>/position.sock`
  (position input API v1, docs/ARCHITECTURE.md; reference client
  `tools/position_client.py`); mapd filters, dead-reckons through dropouts and
  matches to the map. Without a feeder, or without a pack, the map layer reports
  "unavailable" and everything else runs on vision alone. Map packs are built from
  OpenStreetMap with `scripts/roadsense-maps` and are never part of the repository
  ([docs/MAPS.md](docs/MAPS.md); map data © OpenStreetMap contributors, ODbL).

## Reference deployment: Poco F1 on postmarketOS

roadsense was developed and measured on a Xiaomi Poco F1 (Snapdragon 845,
Sony IMX363 rear camera through CAMSS and libcamera's software ISP,
postmarketOS edge, mainline kernel); see [docs/HARDWARE.md](docs/HARDWARE.md).
Inference runs on the Adreno 630 through the ncnn/Vulkan backend (~60-80 ms per
run, ~15 % of one little core, the shipped unit since 2026-10-21); the accuracy gate was
relaxed for data gathering because the fp16 path only deviates on far-range lane points
and leads beyond 50 m (docs/ROADMAP.md). The exact ONNX Runtime CPU path remains as the
reference (`--backend ort`, 54 ms cold but 90+ C sustained; no usable DSP/NPU path on
mainline). The USB-C port is configured as a host with its own VBUS (see
[poco-f1-usb-host](https://github.com/PiotrTopa/poco-f1-usb-host)) so the phone
can drive a car gateway later. `scripts/install-phone.sh`,
`scripts/sync-phone.sh`, `scripts/push-models.sh`, `systemd/` and `phone/` are
the scripts and units of this reference deployment.

Status (2026-10-21): camerad, modeld and the api run on the phone under systemd
and the API answers on port 8080. Verified: supercombo lanes/edges/path/lead on
a real motorway clip, live overlay from the bench. CPU-bound: ~5 road
messages/s with the phone at its thermal limit (supercombo 54 ms fp32 on the 4
big cores when cool). See [docs/ROADMAP.md](docs/ROADMAP.md).

```sh
scripts/fetch-models.sh            # dev box: downloads supercombo, converts to fp32 (+ ncnn model; NCNN=0 skips)
scripts/push-models.sh             # models -> phone ~/roadsense/models
scripts/sync-phone.sh              # tree -> phone ~/roadsense
ssh user@phone 'cd ~/roadsense && sudo sh scripts/install-phone.sh && sudo systemctl start roadsense.target'
# optional GPU backend (modeld --backend ncnn): build ncnn once, ~45 min, in a held-open ssh session
ssh -t user@phone 'sh ~/roadsense/scripts/build-ncnn.sh ~/ncnn'
ssh user@phone 'cd ~/roadsense && sudo CMAKE_ARGS="-DROADSENSE_NCNN=ON -DNCNN_DIR=/home/user/ncnn" BUILD=build-ncnn sh scripts/install-phone.sh'
curl http://phone:8080/v1/health ; open http://phone:8080/
# USB DVR camera (1902:3241) instead of the phone camera (needs libjpeg-turbo-dev at build time)
ssh user@phone 'cd ~/roadsense && sudo sh scripts/install-camerad-dvr.sh --enable'
```

Signs: install [roadsense-signs](https://github.com/PiotrTopa/roadsense-signs)
next to it; its unit joins `roadsense.target`.

## License

MIT, see [LICENSE](LICENSE). Third-party models and their licenses are listed
in [docs/LICENSES.md](docs/LICENSES.md); they are downloaded by
`scripts/fetch-models.sh` and never committed. Map packs are derived from
OpenStreetMap (© OpenStreetMap contributors, ODbL 1.0) and are never committed
either ([docs/MAPS.md](docs/MAPS.md)). The sign provider
roadsense-signs is a separate AGPL-3.0 project and is not part of this
repository.
