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
| `health` | all daemons | fps per stage, model latency, CPU temperature, battery |

## Architecture and contract

```
 camera ──> camerad ──> shared-memory frame ring (NV12 + intrinsics, 20-30 fps)
                               ┌──────────────┴──────────────────────┐
                               ▼                                     ▼
                     modeld (C++, ONNX Runtime CPU)      sign provider (optional, external)
                               │   newline-delimited JSON over Unix sockets  │
                               └──────────────┬──────────────────────┘
                                              ▼
                                    api (Python, aiohttp) ──> HTTP /v1/*, WebSocket /ws, MJPEG /preview
```

The processes share nothing but two mechanisms, specified in
[docs/ARCHITECTURE.md](docs/ARCHITECTURE.md):

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
  show camera metadata.
* **Model:** supercombo is device-independent given the intrinsics: modeld reads
  `focal_px`, `cx`, `cy` from the ring header and warps each frame to the
  model's own camera, so a different lens or resolution needs correct intrinsics
  (and the mounting calibration in `calib.json`), not a different model.
* **Compute:** modeld and the api are plain C++17 / Python with ONNX Runtime on
  the CPU; any Linux box that runs ONNX Runtime can host them.
* **Consumers:** further perception daemons (the sign provider is one) read the
  ring and serve their own socket; the api fans out whatever arrives.

## Reference deployment: Poco F1 on postmarketOS

roadsense was developed and measured on a Xiaomi Poco F1 (Snapdragon 845,
Sony IMX363 rear camera through CAMSS and libcamera's software ISP,
postmarketOS edge, mainline kernel); see [docs/HARDWARE.md](docs/HARDWARE.md).
Inference runs on the CPU through ONNX Runtime (no usable DSP/NPU path on
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
scripts/fetch-models.sh            # dev box: downloads supercombo, converts to fp32
scripts/push-models.sh             # models -> phone ~/roadsense/models
scripts/sync-phone.sh              # tree -> phone ~/roadsense
ssh user@phone 'cd ~/roadsense && sudo sh scripts/install-phone.sh && sudo systemctl start roadsense.target'
curl http://phone:8080/v1/health ; open http://phone:8080/
```

Signs: install [roadsense-signs](https://github.com/PiotrTopa/roadsense-signs)
next to it; its unit joins `roadsense.target`.

## License

MIT, see [LICENSE](LICENSE). Third-party models and their licenses are listed
in [docs/LICENSES.md](docs/LICENSES.md); they are downloaded by
`scripts/fetch-models.sh` and never committed. The sign provider
roadsense-signs is a separate AGPL-3.0 project and is not part of this
repository.
