# roadsense api: the debug face

One camera's raw perception for a person or a tool: connects to `camerad.sock`, `modeld.sock`, `signd.sock`,
`envd.sock`, `lightd.sock` (and `laned.sock`) in the run directory, keeps the latest raw message per type and
serves them with the `/preview` overlay, the frame ring's header and the load governor
([docs/ARCHITECTURE.md](../docs/ARCHITECTURE.md), "api"). It never touches the camera or the models, and it
tracks, fuses and decides nothing: the consolidated side (lead, lanes, lights and signs in context, rules,
speed limit, map, `fused`, `view`, the consumer API, the recorder) is
[chris-integrator](../chris-integrator/README.md), which reads the same sockets. The topics that moved answer
410 here.

Dependencies: Python 3.14 + `py3-aiohttp`, `py3-numpy`, `py3-pillow` (Alpine packages, no pip).

```sh
cd api && python3 -m roadsense_api --rundir /run/roadsense --port 8081
```

| Option / env | Default | |
|---|---|---|
| `--rundir` / `ROADSENSE_RUNDIR` | `/run/roadsense` | daemon sockets, `preview.jpg`, the governor's caps file, `api.sock` (the `governor` message for chris-integrator) |
| `--ring` / `ROADSENSE_RING` | `/dev/shm/roadsense-frames` | frame ring; only its header is read (intrinsics for `/v2/camera` and overlays) |
| `--port` / `ROADSENSE_API_PORT` | `8081` | 8080 is the consolidated API's default |
| `--host` / `ROADSENSE_API_HOST` | `0.0.0.0` (the unit file: `127.0.0.1`) | |

Modules: `sockets.py` (one reconnecting client per daemon, backoff 0.2-5 s), `state.py` (latest raw messages,
rolling fps, ring header, CPU temperature, battery), `overlay.py` (Pillow drawing + pinhole projection),
`governor.py` (load shedding under heat), `server.py` (aiohttp routes, `api.sock`), `__main__.py`.

`GET /` is a small status page: the `/preview` stream plus health, the raw road message and the detectors'
tracks polled once per second. CORS allows all origins.

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
tools/fake_daemons.py --rundir /tmp/roadsense-fake --no-mapd   # synthetic camerad/modeld/signd/envd/lightd
python3 -m pytest -q api/tests                                 # the debug face over fake daemons, the governor
python3 -m pytest -q chris-integrator/tests/test_api.py        # fake daemons -> chris-integrator -> every endpoint
```
