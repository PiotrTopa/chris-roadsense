# pocopilot

Camera perception service for a **Xiaomi Poco F1 running postmarketOS**, in the
spirit of [openpilot](https://github.com/commaai/openpilot) /
[flowpilot](https://github.com/flowdriveai/flowpilot) (flowpilot is GPL-3.0, so it is a source of ideas only; nothing from it is copied): the phone's rear camera
looks at the road and a small set of daemons turn the stream into structured
road understanding, served over a local HTTP/WebSocket API.

**Scope (current phase): data gathering only.** No car control. The API is the
product; whatever later consumes it (a car gateway, a logger, an HMI) is a
separate project.

## What it serves

| Topic | Source | Content |
|---|---|---|
| `road` | openpilot supercombo model (v0.9.7, MIT) | planned path, lane lines (4) + probabilities, road edges (2), lead vehicles (x,y,v,a with probability), ego pose (velocity, rotation rates), desire state, meta (blinkers, disengage probabilities) |
| `signs` | traffic-sign detector + tracker | per-frame detections with class, box, score; persistent track ids, first/last seen, best crop |
| `camera` | libcamera | frame metadata (exposure, gain, colour gains), optional MJPEG preview with overlays |
| `health` | all daemons | fps per stage, model latency, CPU temperature, battery |

## Architecture

```
 IMX363 ──CSI──> camss ──> libcamera (soft ISP, GPU debayer, AE/AWB) ──> camerad
                                                                             │ shared-memory frame ring (YUV/RGB, 20-30 fps)
                               ┌──────────────────────────────┬──────────────┘
                               ▼                              ▼
                            modeld  (C++, ONNX Runtime CPU)  signd  (C++, ONNX Runtime CPU + tracker)
                               │ JSON messages over Unix sockets  │
                               └──────────────┬───────────────────┘
                                              ▼
                                       api  (Python, aiohttp)  ──> HTTP /v1/*, WebSocket /ws, MJPEG /preview
```

See [docs/ARCHITECTURE.md](docs/ARCHITECTURE.md) for the message formats and
[docs/HARDWARE.md](docs/HARDWARE.md) for what the phone can and cannot do.

## Status

Phase 1 complete (2026-10-21): the four services run on the phone under systemd and
the API answers on port 8080. Verified: supercombo lanes/edges/path/lead on a real
motorway clip, sign detection + classification on Polish street photos, live
overlay from the bench. CPU-bound: ~5 road messages/s and ~2 sign frames/s with
the phone at its thermal limit. See [docs/ROADMAP.md](docs/ROADMAP.md).

## Quick start

```sh
scripts/fetch-models.sh            # dev box: downloads supercombo, converts to fp32
# sign models: export locally per docs/SIGN_MODELS.md into models/
scripts/push-models.sh             # models -> phone ~/pocopilot/models
scripts/sync-phone.sh              # tree -> phone ~/pocopilot
ssh user@phone 'cd ~/pocopilot && sudo sh scripts/install-phone.sh && sudo systemctl start pocopilot.target'
curl http://phone:8080/v1/health ; open http://phone:8080/
```

## Hardware

Poco F1 (beryllium, Snapdragon 845), postmarketOS edge, mainline kernel 7.1,
rear Sony IMX363 through the Qualcomm CAMSS driver and libcamera's software ISP
(debayer on the Adreno 630 via GLES). Inference runs on the CPU through ONNX
Runtime; there is no usable DSP/NPU path on mainline. The USB-C port is
configured as a host with its own VBUS (see
[poco-f1-usb-host](https://github.com/PiotrTopa/poco-f1-usb-host)) so the
phone can drive a car gateway later.

## License

MIT, see [LICENSE](LICENSE). Third-party models and their licenses are listed
in [docs/LICENSES.md](docs/LICENSES.md); they are downloaded by
`scripts/fetch-models.sh` and never committed.
