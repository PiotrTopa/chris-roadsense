#!/usr/bin/env python3
"""Synthetic camerad/modeld/signd publishers for exercising the api without hardware.

Creates <rundir>/{camerad,modeld,signd,envd,lightd,mapd}.sock, emits contract-shaped JSON at
30/5/10/1/3/2 Hz (the fake mapd cycles: no position -> on a road -> tunnel ahead -> in the tunnel;
--no-mapd leaves mapd.sock and position.sock to a real mapd), writes <rundir>/preview.jpg (960x540, every 3rd frame) and a
header-only frame ring <rundir>/roadsense-frames (pass --ring to the api).

    python3 tools/fake_daemons.py --rundir /tmp/roadsense-fake
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
# a second schedule (period 200 frames) with a lasting rule and a warning for the `rules` topic:
EXTRA_SIGNS = ((60, "regulatory--no-overtaking", 71), (120, "warning--pedestrians-crossing", 300))   # (start, cls, cls_id)


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
        self.env_pub = Publisher(os.path.join(rundir, "envd.sock"), on_connect=self.env)
        self.light_pub = Publisher(os.path.join(rundir, "lightd.sock"))
        self.light_n = 0
        self.map_pub = Publisher(os.path.join(rundir, "mapd.sock"), on_connect=self.map)
        self.map_n = 0
        self.env_n = 0
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
        hdr = struct.pack(RING_FMT, b"RSFRAME1", 4096, 6, slot, W, H, W, W, 1, FOCAL, CX, CY,
                          self.seq, os.getpid())
        tmp = os.path.join(self.rundir, ".roadsense-frames.tmp")
        with open(tmp, "wb") as f:
            f.write(hdr.ljust(4096, b"\0"))
        os.replace(tmp, os.path.join(self.rundir, "roadsense-frames"))

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
        k2 = self.sign_frame % 200
        for i, (start, cls, cls_id) in enumerate(EXTRA_SIGNS):
            j, tid = k2 - start, 900000 + 2 * (self.sign_frame // 200) + i
            if 0 <= j < SIGN_LIFE:
                s = 30 + 2 * j
                x0, y0 = 1200 + 10 * j, 400 - 6 * j
                dets.append({"track_id": tid, "cls": cls, "cls_id": cls_id, "score": round(0.6 + 0.3 * j / SIGN_LIFE, 3),
                             "box": [x0, y0, x0 + s, y0 + s], "age": j, "hits": j + 1})
            elif j == SIGN_LIFE:
                ended.append({"track_id": tid, "cls": cls, "first_seq": self.seq - 3 * SIGN_LIFE, "last_seq": self.seq - 3,
                              "hits": SIGN_LIFE, "best_score": 0.9,
                              "best_crop": os.path.join(self.rundir, "crops", f"{tid}.jpg")})
        self.sign_frame += 1
        return {**self.hdr("signs"), "detect_ms": 85.0, "detections": dets, "tracks_ended": ended}

    async def signd(self):
        while True:
            await asyncio.sleep(0.1)
            self.sign.send(self.signs())

    # ---- envd ---------------------------------------------------------------
    ENV_GROUPS = {"place": ["highway", "city_street", "residential", "rural_road", "forest_road", "tunnel",
                            "parking_lot", "underground_parking", "gas_station", "bridge"],
                  "weather": ["clear", "overcast", "rain", "snow", "fog"],
                  "light": ["day", "dusk_dawn", "night"]}

    def env(self):
        n = self.env_n
        msg = {**self.hdr("env"), "env_ms": 180.0, "encode_ms": 160.0}
        for g, classes in self.ENV_GROUPS.items():
            k = (n // 30) % len(classes) if g == "place" else 0
            scores = {c: round(0.9 if i == k else 0.1 / (len(classes) - 1), 3) for i, c in enumerate(classes)}
            msg[g] = {"label": classes[k], "confidence": 0.9, "stable_s": float(n % 30), "scores": scores,
                      "frame_label": classes[k], "frame_scores": scores}
        msg["stats"] = {"luma": 120.0, "luma_top": 180.0, "luma_bottom": 80.0, "dark_frac": 0.02, "bright_frac": 0.01,
                        "chroma": 12.0}
        msg["model"] = "fake"
        return msg

    async def envd(self):
        while True:
            await asyncio.sleep(1.0)
            self.env_n += 1
            self.env_pub.send(self.env())

    # ---- lightd (signd --lights) ------------------------------------------------
    def lights(self):
        # one light over the lane ahead (cycles red -> red_yellow -> green -> yellow) and one side view
        n = self.light_n
        state = ("red", "red_yellow", "green", "yellow")[(n // 9) % 4]
        s = 40 + n % 9          # housing grows as it approaches
        x0, y0 = CX + 40, 260 - n % 9
        lights = [{"track_id": 1, "state": state, "state_conf": 0.9, "state_s": float(n % 9) / 3, "frame_state": state,
                   "flashing": False, "score": 0.7, "box": [x0, y0, x0 + s * 0.45, y0 + s], "horizontal": False,
                   "hits": n + 1, "age": n + 1},
                  {"track_id": 2, "state": "off", "state_conf": 0.8, "state_s": 5.0, "frame_state": "off",
                   "flashing": False, "score": 0.5, "box": [300, 300, 315, 335], "horizontal": False,
                   "hits": n + 1, "age": n + 1}]
        return {**self.hdr("lights"), "lights_ms": 120.0, "frame": [W, H], "roi": [0, 0, W, 670], "lights": lights}

    async def lightd(self):
        while True:
            await asyncio.sleep(1 / 3)
            self.light_n += 1
            self.light_pub.send(self.lights())

    # ---- mapd -----------------------------------------------------------------
    def map(self):
        n = self.map_n
        msg = {**self.hdr("map"), "seq": n, "api": 1, "pack": {"region": "fake", "osm_timestamp": None,
                                                                "built": None, "format": 1},
               "input": {"gnss": None, "motion": {"source": "none", "tag": None, "age_s": None}, "errors": 0,
                         "last_error": None},
               "proc": {"step_ms": 0.5, "cpu_pct": 0.5, "feeders": 0, "tile_loads": 1}}
        phase = n % 60
        if phase < 6:                                  # no position: the whole map layer is unavailable
            msg.update(state="no_source", available=False, reason="no_position", position=None, match=None,
                       road=None, speed_limit=None, ahead=None)
            return msg
        in_tunnel = phase >= 40
        to_tunnel = max(0.0, (40 - phase) * 10.0)
        left = 400.0 - (phase - 40) * 20.0 if in_tunnel else None
        msg.update(
            state="good", available=True, reason=None,
            position={"lat": 52.0 + n * 1e-4, "lon": 21.0, "heading_deg": 0.0, "heading_acc_deg": 2.0,
                      "speed_mps": 20.0, "hacc_m": 3.0, "confidence": 0.95, "source": "gnss", "fix_age_s": 0.3,
                      "dr_distance_m": 0.0, "motion_source": "none", "odometer_m": n * 10.0},
            match={"matched": True, "confidence": 0.93, "ambiguous": False, "distance_m": 1.5,
                   "heading_diff_deg": 0.5, "lat": 52.0 + n * 1e-4, "lon": 21.0, "bearing_deg": 0.0,
                   "edge": "00000000:0", "dir": 1, "offset_m": n * 10.0, "candidates": 2},
            road={"class": "secondary", "name": "Fake Street", "ref": "799", "oneway": False,
                  "lanes": {"total": 2, "forward": 1, "backward": 1, "source": "tagged_total"},
                  "tunnel": in_tunnel, "bridge": False, "covered": False, "lit": True, "surface": "asphalt",
                  "roundabout": False, "link": False, "motorroad": False, "expressway": False, "dual_carriageway": False,
                  "access_restricted": False, "toll": False, "parking": False, "forest": False, "layer": 0,
                  "built_up": {"value": True, "source": "tagged"}},
            speed_limit={"limit_kmh": 70, "source": "maxspeed", "rule": None, "detail": "tagged",
                         "confidence": 0.9, "variable": False},
            ahead={"horizon_m": 1500.0, "path_confidence": 0.8,
                   "features": [{"kind": "traffic_signals", "dist_m": 120.0, "confidence": 0.9},
                                {"kind": "crossing", "dist_m": 135.0, "confidence": 0.9}],
                   "junctions": [{"dist_m": 125.0, "kind": "junction", "branches": 3, "turn_deg": 0.0,
                                  "minor": False, "confidence": 0.8}],
                   "tunnel": {"in_tunnel": in_tunnel, "exit_m": left, "entry_m": None if in_tunnel else to_tunnel,
                              "length_m": None if in_tunnel else 400.0},
                   "built_up_change": None, "speed_limit_changes": [], "road_change": None,
                   "shape": {"s": [0.0, 20.0, 40.0], "x": [0.0, 20.0, 40.0], "y": [0.0, 0.1, 0.4],
                             "curvature": [0.0, 0.0005, 0.0]},
                   "curves": []})
        return msg

    async def mapd(self):
        async def feeder(reader, writer):            # position.sock: answer hello, swallow the rest
            try:
                while True:
                    raw = await reader.readline()
                    if not raw:
                        break
                    if b'"hello"' in raw:
                        writer.write(b'{"type":"hello","api":1,"server":"fake-mapd"}\n')
            finally:
                writer.close()
        path = os.path.join(self.rundir, "position.sock")
        try:
            os.unlink(path)
        except FileNotFoundError:
            pass
        await asyncio.start_unix_server(feeder, path=path)
        while True:
            await asyncio.sleep(0.5)
            self.map_n += 1
            self.map_pub.send(self.map())

    async def run(self, mapd=True):
        for p in (self.cam, self.model, self.sign, self.env_pub, self.light_pub) + ((self.map_pub,) if mapd else ()):
            await p.start()
        self.write_preview()
        self.write_ring()
        await asyncio.gather(self.camerad(), self.modeld(), self.signd(), self.envd(), self.lightd(),
                             *([self.mapd()] if mapd else []))


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--rundir", default=os.environ.get("ROADSENSE_RUNDIR", "/tmp/roadsense-fake"))
    ap.add_argument("--no-mapd", action="store_true", help="do not fake mapd (run the real one on this rundir)")
    a = ap.parse_args()
    os.makedirs(a.rundir, exist_ok=True)
    try:
        asyncio.run(Fake(a.rundir).run(mapd=not a.no_mapd))
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
