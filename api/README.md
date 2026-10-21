# roadsense api

Python fan-out and overlay layer: connects to `camerad.sock`, `modeld.sock` and
`signd.sock` in the run directory, keeps the latest message per type and serves
the endpoints listed in [docs/ARCHITECTURE.md](../docs/ARCHITECTURE.md).
It never touches the camera or the models.

Dependencies: Python 3.14 + `py3-aiohttp`, `py3-numpy`, `py3-pillow` (Alpine packages, no pip).

```sh
cd api && python3 -m roadsense_api --rundir /run/roadsense --port 8080
```

| Option / env | Default | |
|---|---|---|
| `--rundir` / `ROADSENSE_RUNDIR` | `/run/roadsense` | daemon sockets and `preview.jpg` |
| `--datadir` / `ROADSENSE_DATADIR` | `/var/lib/roadsense` | `signs.jsonl` (ended sign tracks) is appended here if the directory exists |
| `--ring` / `ROADSENSE_RING` | `/dev/shm/roadsense-frames` | frame ring; only its header is read (intrinsics for `/v1/camera` and overlays) |
| `--port` / `ROADSENSE_API_PORT` | `8080` | |

Modules: `sockets.py` (one reconnecting client per daemon, backoff 0.2-5 s),
`state.py` (latest messages, rolling fps, live/ended sign tracks, ring header,
CPU temperature, battery), `overlay.py` (Pillow drawing + pinhole projection),
`server.py` (aiohttp routes), `__main__.py`.

`GET /` is a small status page: the `/preview` stream plus health, road and live
signs polled once per second. CORS allows all origins.

## Overlay projection

Model outputs are in openpilot's device frame, passed through unconverted by modeld
(x forward, y right, z down, origin at the camera). They are rotated by the `calib` message's roll/pitch/yaw (openpilot `rot_from_euler`), mapped
to the camera frame (x right, y down, z forward) and projected with focal_px/cx/cy
from the ring header (fallback: `calib.focal_px`, image centre), scaled to the
preview size (960x540 for a 1920x1080 sensor). The plan path is drawn on the
ground (`plan.z + calib.height_m`); lane lines and road edges use their own z.
Only lead hypothesis 0 (now) is drawn.

## Testing without hardware

```sh
tools/fake_daemons.py --rundir /tmp/roadsense-fake     # synthetic camerad/modeld/signd
tools/smoke_api.sh                                     # fake daemons + api, every endpoint
OVERLAY_OUT=/tmp/overlay.jpg tools/smoke_api.sh        # also save one overlay frame
```

`smoke_api.sh` runs `python3 -m unittest discover -s api/tests`, which starts both
processes on a temporary run directory and a free port (override with
`ROADSENSE_RUNDIR`, `ROADSENSE_TEST_PORT`).
