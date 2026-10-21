#!/usr/bin/env python3
"""Synthetic camerad/modeld/signd publishers for exercising the api without hardware.

Creates <rundir>/{camerad,modeld,signd}.sock, emits contract-shaped JSON at
30/5/10 Hz, writes <rundir>/preview.jpg (960x540, every 3rd frame) and a
header-only frame ring <rundir>/pocopilot-frames (pass --ring to the api).

    python3 tools/fake_daemons.py --rundir /tmp/pocopilot-fake
"""
import argparse
import asyncio
import io
import json
import math
import os
import struct
import time

import numpy as np
from PIL import Image, ImageDraw

W, H = 1920, 1080
FOCAL, CX, CY = 1500.0, W / 2, H / 2
CAM_HEIGHT = 1.22
X_IDXS = [192.0 * (i / 32) ** 2 for i in range(33)]
T_IDXS = [10.0 * (i / 32) ** 2 for i in range(33)]
LEAD_T = [0.0, 2.0, 4.0, 6.0, 8.0, 10.0]
RING_FMT = "<8s8I3f4xQQ"
SIGN_LIFE, SIGN_GAP = 40, 15


def now_ns():
    return time.monotonic_ns(), time.time_ns()


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
            await reader.read()          # wait for EOF
        finally:
            self.clients.discard(writer)
            writer.close()

    @staticmethod
    def _line(msg):
        return (json.dumps(msg, separators=(",", ":")) + "\n").encode()

    def send(self, msg):
        data = self._line(msg)
        for w in list(self.clients):
            if w.transport.get_write_buffer_size() > 1 << 20:   # slow client: drop it
                w.close()
                self.clients.discard(w)
            else:
                w.write(data)


class Fake:
    def __init__(self, rundir):
        self.rundir = rundir
        self.seq = 0
        self.t0 = time.monotonic()
        self.cam = Publisher(os.path.join(rundir, "camerad.sock"))
        self.model = Publisher(os.path.join(rundir, "modeld.sock"), on_connect=self.calib)
        self.sign = Publisher(os.path.join(rundir, "signd.sock"))
        self.sign_frame = 0
        self.track_id = 0
        self.preview_bg = self._background()

    def hdr(self, mtype):
        mono, real = now_ns()
        return {"type": mtype, "seq": self.seq, "ts_mono_ns": mono, "ts_real_ns": real}

    def calib(self):
        return {**self.hdr("calib"), "pitch": 0.0, "yaw": 0.0, "roll": 0.0, "height_m": CAM_HEIGHT,
                "focal_px": FOCAL, "source": "config"}

    def write_ring(self):
        slot = 4096 + ((W * H * 3 // 2 + 4095) // 4096) * 4096
        hdr = struct.pack(RING_FMT, b"PPFRAME1", 4096, 6, slot, W, H, W, W, 1, FOCAL, CX, CY,
                          self.seq, os.getpid())
        tmp = os.path.join(self.rundir, ".pocopilot-frames.tmp")
        with open(tmp, "wb") as f:
            f.write(hdr.ljust(4096, b"\0"))
        os.replace(tmp, os.path.join(self.rundir, "pocopilot-frames"))

    # ---- camerad ------------------------------------------------------------
    @staticmethod
    def _background():
        w, h = W // 2, H // 2
        hz = int(CY / 2)
        y = np.arange(h, dtype=np.float32)[:, None]
        sky = np.clip(np.stack([90 + 80 * y / hz, 140 + 60 * y / hz, 220 + 0 * y], -1), 0, 255)
        g = (y - hz) / (h - hz)
        ground = np.stack([70 + 40 * g, 70 + 40 * g, 75 + 40 * g], -1)
        img = np.where(y[..., None] < hz, sky, ground).repeat(w, axis=1).astype(np.uint8)
        im = Image.fromarray(img, "RGB")
        ImageDraw.Draw(im).line([(0, hz), (w, hz)], fill=(255, 255, 255), width=1)
        return im

    def write_preview(self):
        im = self.preview_bg.copy()
        ImageDraw.Draw(im).text((W // 2 - 110, H // 2 - 20), f"FAKE frame {self.seq}", fill=(255, 255, 255))
        buf = io.BytesIO()
        im.save(buf, "JPEG", quality=80)
        tmp = os.path.join(self.rundir, ".preview.jpg.tmp")
        with open(tmp, "wb") as f:
            f.write(buf.getvalue())
        os.replace(tmp, os.path.join(self.rundir, "preview.jpg"))

    async def camerad(self):
        period = 1 / 30
        nxt = time.monotonic()
        while True:
            self.seq += 1
            self.cam.send({**self.hdr("camera"), "exposure_us": 8000, "analogue_gain": 2.0,
                           "colour_gains": [1.8, 1.6], "fps": 30.0, "dropped": 0})
            if self.seq % 3 == 0:
                self.write_preview()
                self.write_ring()
            nxt += period
            await asyncio.sleep(max(0.0, nxt - time.monotonic()))

    # ---- modeld -------------------------------------------------------------
    def road(self):
        t = time.monotonic() - self.t0
        curv = 0.0004 * math.sin(0.3 * t)
        x = np.array(X_IDXS)
        lanes = []
        for off, prob in ((-5.4, 0.35), (-1.8, 0.97), (1.8, 0.93), (5.4, 0.2)):   # y right
            y = off + 0.5 * curv * x ** 2 + 0.1 * math.sin(t)
            lanes.append({"x": X_IDXS, "y": y.round(3).tolist(), "z": [CAM_HEIGHT] * 33,
                          "y_std": [0.1 + 0.002 * xi for xi in X_IDXS], "prob": prob})
        edges = [{"x": X_IDXS, "y": (off + 0.5 * curv * x ** 2).round(3).tolist(), "z": [CAM_HEIGHT] * 33,
                  "y_std": [0.3] * 33} for off in (-7.0, 3.5)]
        v = 20.0
        px = np.array([v * ti for ti in T_IDXS])
        zeros = [0.0] * 33
        plan = {"t": T_IDXS, "x": px.round(3).tolist(), "y": (0.5 * curv * px ** 2).round(3).tolist(), "z": zeros,
                "vx": [v] * 33, "vy": zeros, "vz": zeros, "ax": zeros, "ay": zeros, "az": zeros,
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

    async def modeld(self):
        while True:
            await asyncio.sleep(0.2)
            self.model.send(self.road())

    # ---- signd --------------------------------------------------------------
    def signs(self):
        k = self.sign_frame % (SIGN_LIFE + SIGN_GAP)
        if k == 0:
            self.track_id += 1
        dets, ended = [], []
        if k < SIGN_LIFE:
            s = 30 + 2 * k
            x0, y0 = 1250 + 12 * k, 430 - 6 * k
            dets.append({"track_id": self.track_id, "cls": "regulatory--maximum-speed-limit-50", "cls_id": 23,
                         "score": round(0.6 + 0.3 * k / SIGN_LIFE, 3), "box": [x0, y0, x0 + s, y0 + s],
                         "age": k, "hits": k + 1})
        elif k == SIGN_LIFE:
            first = self.seq - 3 * SIGN_LIFE
            ended.append({"track_id": self.track_id, "cls": "regulatory--maximum-speed-limit-50",
                          "first_seq": first, "last_seq": self.seq - 3, "hits": SIGN_LIFE, "best_score": 0.9,
                          "best_crop": os.path.join(self.rundir, "crops", f"{self.track_id}.jpg")})
        self.sign_frame += 1
        return {**self.hdr("signs"), "detect_ms": 85.0, "detections": dets, "tracks_ended": ended}

    async def signd(self):
        while True:
            await asyncio.sleep(0.1)
            self.sign.send(self.signs())

    async def run(self):
        for p in (self.cam, self.model, self.sign):
            await p.start()
        self.write_preview()
        self.write_ring()
        await asyncio.gather(self.camerad(), self.modeld(), self.signd())


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--rundir", default=os.environ.get("POCOPILOT_RUNDIR", "/tmp/pocopilot-fake"))
    a = ap.parse_args()
    os.makedirs(a.rundir, exist_ok=True)
    try:
        asyncio.run(Fake(a.rundir).run())
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
