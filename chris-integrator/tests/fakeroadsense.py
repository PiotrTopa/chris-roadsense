#!/usr/bin/env python3
"""A fake chris-roadsense instance for the end-to-end test (tests/test_api.py): no camera, no model, no
roadsense code.

It plays the part of one camera instance as chris-integrator sees it:

* JSON-line publishers on <rundir>/{camerad,modeld,signd,envd,lightd,api}.sock with contract-shaped messages
  (`camera` 30 Hz, `road` 5 Hz with a `calib` on connect, `signs` 10 Hz, `env` 1 Hz, `lights` 3 Hz, `governor`
  on connect and 1 Hz);
* optionally (--http-port) the instance's debug face: what the integrator forwards (`/preview`,
  `/v2/frame.jpg`, `/v2/camera` with the frame ring's geometry, `/v2/governor`). The "frames" are a few bytes
  with JPEG markers, not images.

The message generators follow chris-roadsense's own synthetic publishers (its tools/fake_daemons.py, MIT); the
sign schedule (a B-33 50 every 5.5 s, a B-25 and an A-16 in each 20 s) is what the tests assert on.

    python3 tests/fakeroadsense.py --rundir /tmp/fake-rs [--http-port 8091]
"""
import argparse
import asyncio
import json
import math
import os
import time

W, H = 1920, 1080
FOCAL, CX, CY = 1500.0, W / 2, H / 2
CAM_HEIGHT = 1.22
X_IDXS = [192.0 * (i / 32) ** 2 for i in range(33)]
T_IDXS = [10.0 * (i / 32) ** 2 for i in range(33)]
LEAD_T = [0.0, 2.0, 4.0, 6.0, 8.0, 10.0]
SIGN_LIFE, SIGN_GAP = 40, 15
# a second schedule (period 200 frames) with a lasting rule and a warning for the `rules` topic: (start, cls, cls_id)
EXTRA_SIGNS = ((60, "regulatory--no-overtaking", 71), (120, "warning--pedestrians-crossing", 300))
ENV_GROUPS = {"place": ["highway", "city_street", "residential", "rural_road", "forest_road", "tunnel",
                        "parking_lot", "underground_parking", "gas_station", "bridge"],
              "weather": ["clear", "overcast", "rain", "snow", "fog"],
              "light": ["day", "dusk_dawn", "night"]}


def jpeg_stub(n):
    """Bytes that start and end like a JPEG: enough for a forwarder, not an image."""
    return b"\xff\xd8\xff\xe0" + b"FAKE frame %d" % n + b"\xff\xd9"


class Publisher:
    def __init__(self, path, on_connect=None):
        self.path, self.on_connect, self.clients = path, on_connect, set()

    async def start(self):
        try:
            os.unlink(self.path)
        except FileNotFoundError:
            pass
        self.server = await asyncio.start_unix_server(self._client, path=self.path)

    async def _client(self, reader, writer):
        self.clients.add(writer)
        if self.on_connect:
            writer.write(self._line(self.on_connect()))
        try:
            await reader.read()          # until the reader goes away
        finally:
            self.clients.discard(writer)
            writer.close()

    @staticmethod
    def _line(msg):
        return (json.dumps(msg, separators=(",", ":")) + "\n").encode()

    def send(self, msg):
        data = self._line(msg)
        for w in list(self.clients):
            if w.transport.get_write_buffer_size() > 1 << 20:
                w.close()
                self.clients.discard(w)
            else:
                w.write(data)


class FakeRoadsense:
    def __init__(self, rundir):
        self.rundir, self.seq, self.t0 = rundir, 0, time.monotonic()
        p = lambda name, **kw: Publisher(os.path.join(rundir, name + ".sock"), **kw)   # noqa: E731
        self.cam, self.model = p("camerad"), p("modeld", on_connect=self.calib)
        self.sign, self.env_pub, self.light_pub = p("signd"), p("envd", on_connect=self.env), p("lightd")
        self.api_pub = p("api", on_connect=self.governor)
        self.light_n = self.env_n = self.sign_frame = self.track_id = 0
        self.last_camera = None

    def hdr(self, mtype):
        return {"type": mtype, "seq": self.seq, "ts_mono_ns": time.monotonic_ns(), "ts_real_ns": time.time_ns()}

    def calib(self):
        return {**self.hdr("calib"), "pitch": 0.0, "yaw": 0.0, "roll": 0.0, "height_m": CAM_HEIGHT,
                "focal_px": FOCAL, "source": "config"}

    def governor(self):
        return {**self.hdr("governor"), "enabled": True, "level": 0, "levels": 4, "shed": [],
                "caps": {"view_hz": 15.0, "signs_hz": 10.0}, "limited": False, "limiter": None, "forced": None,
                "headroom_k": 12.0, "hottest": None}

    def camera(self):
        return {**self.hdr("camera"), "exposure_us": 8000, "analogue_gain": 2.0, "colour_gains": [1.8, 1.6],
                "fps": 30.0, "dropped": 0}

    def road(self):
        t = time.monotonic() - self.t0
        curv = 0.0004 * math.sin(0.3 * t)
        lanes = []
        for off, prob in ((-5.4, 0.35), (-1.8, 0.97), (1.8, 0.93), (5.4, 0.2)):   # y right
            y = [round(off + 0.5 * curv * x * x + 0.1 * math.sin(t), 3) for x in X_IDXS]
            lanes.append({"x": X_IDXS, "y": y, "z": [CAM_HEIGHT] * 33, "y_std": [0.1 + 0.002 * x for x in X_IDXS],
                          "prob": prob})
        edges = [{"x": X_IDXS, "y": [round(off + 0.5 * curv * x * x, 3) for x in X_IDXS], "z": [CAM_HEIGHT] * 33,
                  "y_std": [0.3] * 33} for off in (-7.0, 3.5)]
        v = 20.0
        px = [v * ti for ti in T_IDXS]
        zeros = [0.0] * 33
        plan = {"t": T_IDXS, "x": [round(x, 3) for x in px], "y": [round(0.5 * curv * x * x, 3) for x in px],
                "z": zeros, "vx": [v] * 33, "vy": zeros, "vz": zeros, "ax": zeros, "ay": zeros, "az": zeros,
                "roll": zeros, "pitch": zeros, "yaw": zeros, "roll_rate": zeros, "pitch_rate": zeros,
                "yaw_rate": [curv * v] * 33, "x_std": [0.5] * 33, "y_std": [0.2] * 33}
        lx = 30.0 + 2.0 * math.sin(0.5 * t)
        leads = [{"prob": p, "t": LEAD_T, "x": [lx + k * 2.0 for k in range(6)], "y": [0.5 * curv * lx ** 2] * 6,
                  "v": [18.0] * 6, "a": [0.0] * 6, "x_std": [1.0] * 6, "y_std": [0.3] * 6,
                  "v_std": [0.5] * 6, "a_std": [0.2] * 6} for p in (0.9, 0.6, 0.4)]
        return {**self.hdr("road"), "model_ms": 290.0 + 10 * math.sin(t), "frame_seq_prev": self.seq - 1,
                "plan": plan, "lane_lines": lanes, "road_edges": edges, "leads": leads,
                "pose": {"vx": v, "vy": 0.0, "vz": 0.0, "roll_rate": 0.0, "pitch_rate": 0.0, "yaw_rate": curv * v,
                         "vx_std": 0.2, "vy_std": 0.1, "vz_std": 0.1, "roll_rate_std": 0.01,
                         "pitch_rate_std": 0.01, "yaw_rate_std": 0.01},
                "desire_state": [1.0] + [0.0] * 7, "desire_pred": [[1.0] + [0.0] * 7] * 4,
                "meta": {"engaged_prob": 0.95,
                         "disengage_probs": {k: [0.01] * 5 for k in ("gas", "brake", "steer_override",
                                                                       "brake_3ms2", "brake_4ms2", "brake_5ms2")},
                         "blinkers": {"left": [0.0] * 6, "right": [0.0] * 6}, "hard_brake_pred": [0.0] * 5}}

    def _sign(self, tid, cls, cls_id, j, x0, dx):
        s = 30 + 2 * j
        x, y = x0 + dx * j, 430 - 6 * j
        return {"track_id": tid, "cls": cls, "cls_id": cls_id, "score": round(0.6 + 0.3 * j / SIGN_LIFE, 3),
                "box": [x, y, x + s, y + s], "age": j, "hits": j + 1}

    def _ended(self, tid, cls):
        return {"track_id": tid, "cls": cls, "first_seq": self.seq - 3 * SIGN_LIFE, "last_seq": self.seq - 3,
                "hits": SIGN_LIFE, "best_score": 0.9, "best_crop": os.path.join(self.rundir, "crops", f"{tid}.jpg")}

    def signs(self):
        k = self.sign_frame % (SIGN_LIFE + SIGN_GAP)
        if k == 0:
            self.track_id += 1
        dets, ended = [], []
        if k < SIGN_LIFE:
            dets.append(self._sign(self.track_id, "regulatory--maximum-speed-limit-50", 23, k, 1250, 12))
        elif k == SIGN_LIFE:
            ended.append(self._ended(self.track_id, "regulatory--maximum-speed-limit-50"))
        k2 = self.sign_frame % 200
        for i, (start, cls, cls_id) in enumerate(EXTRA_SIGNS):
            j, tid = k2 - start, 900000 + 2 * (self.sign_frame // 200) + i
            if 0 <= j < SIGN_LIFE:
                dets.append(self._sign(tid, cls, cls_id, j, 1200, 10))
            elif j == SIGN_LIFE:
                ended.append(self._ended(tid, cls))
        self.sign_frame += 1
        return {**self.hdr("signs"), "detect_ms": 85.0, "detections": dets, "tracks_ended": ended}

    def env(self):
        n = self.env_n
        msg = {**self.hdr("env"), "env_ms": 180.0, "encode_ms": 160.0}
        for g, classes in ENV_GROUPS.items():
            k = (n // 30) % len(classes) if g == "place" else 0
            scores = {c: round(0.9 if i == k else 0.1 / (len(classes) - 1), 3) for i, c in enumerate(classes)}
            msg[g] = {"label": classes[k], "confidence": 0.9, "stable_s": float(n % 30), "scores": scores,
                      "frame_label": classes[k], "frame_scores": scores}
        msg["stats"] = {"luma": 120.0, "luma_top": 180.0, "luma_bottom": 80.0, "dark_frac": 0.02,
                        "bright_frac": 0.01, "chroma": 12.0}
        msg["model"] = "fake"
        return msg

    def lights(self):
        # one light over the lane ahead (red -> red_yellow -> green -> yellow) and one side view, off
        n = self.light_n
        state = ("red", "red_yellow", "green", "yellow")[(n // 9) % 4]
        s = 40 + n % 9
        x0, y0 = CX + 40, 260 - n % 9
        lights = [{"track_id": 1, "state": state, "state_conf": 0.9, "state_s": float(n % 9) / 3, "frame_state": state,
                   "flashing": False, "score": 0.7, "box": [x0, y0, x0 + s * 0.45, y0 + s], "horizontal": False,
                   "hits": n + 1, "age": n + 1},
                  {"track_id": 2, "state": "off", "state_conf": 0.8, "state_s": 5.0, "frame_state": "off",
                   "flashing": False, "score": 0.5, "box": [300, 300, 315, 335], "horizontal": False,
                   "hits": n + 1, "age": n + 1}]
        return {**self.hdr("lights"), "lights_ms": 120.0, "frame": [W, H], "roi": [0, 0, W, 670], "lights": lights}

    async def every(self, period, step):
        nxt = time.monotonic()
        while True:
            step()
            nxt += period
            await asyncio.sleep(max(0.0, nxt - time.monotonic()))

    def _camera_step(self):
        self.seq += 1
        self.last_camera = self.camera()
        self.cam.send(self.last_camera)

    def _light_step(self):
        self.light_n += 1
        self.light_pub.send(self.lights())

    def _env_step(self):
        self.env_n += 1
        self.env_pub.send(self.env())

    async def run(self, http_port=None):
        for p in (self.cam, self.model, self.sign, self.env_pub, self.light_pub, self.api_pub):
            await p.start()
        if http_port:
            await self.debug_face(http_port)
        await asyncio.gather(self.every(1 / 30, self._camera_step), self.every(0.2, lambda: self.model.send(self.road())),
                             self.every(0.1, lambda: self.sign.send(self.signs())), self.every(1.0, self._env_step),
                             self.every(1 / 3, self._light_step),
                             self.every(1.0, lambda: self.api_pub.send(self.governor())))

    async def debug_face(self, port):
        from aiohttp import web

        async def camera(req):
            ring = {"width": W, "height": H, "stride": W, "slots": 6, "focal_px": FOCAL, "cx": CX, "cy": CY,
                    "seq": self.seq}
            return web.json_response(dict(self.last_camera or {}, ring=ring))

        async def frame(req):
            return web.Response(body=jpeg_stub(self.seq), content_type="image/jpeg")

        async def governor(req):
            return web.json_response(self.governor())

        async def preview(req):
            resp = web.StreamResponse(headers={"Content-Type": "multipart/x-mixed-replace; boundary=frame"})
            await resp.prepare(req)
            while True:
                body = jpeg_stub(self.seq)
                await resp.write(b"--frame\r\nContent-Type: image/jpeg\r\nContent-Length: %d\r\n\r\n" % len(body)
                                 + body + b"\r\n")
                await asyncio.sleep(0.1)          # 10 Hz, as the real preview

        app = web.Application()
        app.router.add_get("/v2/camera", camera)
        app.router.add_get("/v2/frame.jpg", frame)
        app.router.add_route("*", "/v2/governor", governor)
        app.router.add_get("/preview", preview)
        runner = web.AppRunner(app)
        await runner.setup()
        await web.TCPSite(runner, "127.0.0.1", port).start()


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--rundir", required=True)
    ap.add_argument("--http-port", type=int, default=None, help="also serve a fake debug face on 127.0.0.1:PORT")
    a = ap.parse_args()
    os.makedirs(a.rundir, exist_ok=True)
    try:
        asyncio.run(FakeRoadsense(a.rundir).run(a.http_port))
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
