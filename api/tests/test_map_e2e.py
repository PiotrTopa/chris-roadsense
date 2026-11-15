"""End to end: fake camera daemons + the real mapd on a hand-made pack + the api.

Checks the VFR fallback (no position: the map layer is unavailable, the vision topics are served as before),
the position input over HTTP, and the `map` / `fused` topics on the REST endpoints and the WebSocket.
"""
import asyncio
import json
import os
import shutil
import socket
import subprocess
import sys
import tempfile
import time
import unittest
import urllib.error
import urllib.request

REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, os.path.join(REPO, "mapd"))
sys.path.insert(0, os.path.join(REPO, "mapd", "tests"))

import synthpack as SP      # noqa: E402


def free_port():
    with socket.socket() as s:
        s.bind(("127.0.0.1", 0))
        return s.getsockname()[1]


class MapE2E(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.tmp = tempfile.mkdtemp(prefix="roadsense-e2e-")
        cls.rundir = os.path.join(cls.tmp, "run")
        os.makedirs(cls.rundir)
        pack = os.path.join(cls.tmp, "synth.rsmap")
        SP.build(pack)
        cls.port = free_port()
        cls.base = f"http://127.0.0.1:{cls.port}"
        env = dict(os.environ, ROADSENSE_CONFIG_DIR=os.path.join(cls.tmp, "etc"), PYTHONPATH=os.path.join(REPO, "mapd"))
        cls.procs = [
            subprocess.Popen([sys.executable, os.path.join(REPO, "tools", "fake_daemons.py"), "--rundir", cls.rundir,
                              "--no-mapd"]),
            subprocess.Popen([sys.executable, "-m", "roadsense_mapd", "--rundir", cls.rundir, "--pack", pack, "--hz", "5",
                              "--no-vision-odometry", "--stats", "0"], env=env),
            subprocess.Popen([sys.executable, "-m", "roadsense_api", "--rundir", cls.rundir,
                              "--ring", os.path.join(cls.rundir, "roadsense-frames"), "--host", "127.0.0.1",
                              "--port", str(cls.port)], cwd=os.path.join(REPO, "api")),
        ]
        cls.wait(lambda: all(d["connected"] for d in cls.get("/v1/health")["daemons"].values()), 20)
        cls.wait(lambda: cls.get("/v1/speed_limit") is not None
                 and cls.get("/v1/fused")["speed_limit"]["vision"] is not None
                 and cls.get("/v1/fused")["place"]["vision"] is not None, 10)

    @classmethod
    def tearDownClass(cls):
        for p in cls.procs:
            p.terminate()
        for p in cls.procs:
            p.wait(10)
        shutil.rmtree(cls.tmp, ignore_errors=True)

    @classmethod
    def get(cls, path):
        try:
            with urllib.request.urlopen(cls.base + path, timeout=5) as r:
                return json.loads(r.read())
        except (urllib.error.URLError, OSError):
            return None

    @classmethod
    def post(cls, path, body):
        req = urllib.request.Request(cls.base + path, data=body.encode(), method="POST")
        try:
            with urllib.request.urlopen(req, timeout=5) as r:
                return r.status, json.loads(r.read())
        except urllib.error.HTTPError as e:
            with e:
                return e.code, json.loads(e.read())

    @classmethod
    def wait(cls, cond, timeout):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            try:
                if cond():
                    return
            except (TypeError, KeyError):
                pass
            time.sleep(0.2)
        raise AssertionError("timeout")

    def test_1_vfr_without_position(self):
        m = self.get("/v1/map")
        self.assertEqual((m["state"], m["available"], m["reason"]), ("no_source", False, "no_position"))
        self.wait(lambda: self.get("/v1/fused")["map"]["reason"] == "no_position", 5)   # the first map message is in
        fu = self.get("/v1/fused")
        self.assertEqual((fu["map"]["available"], fu["map"]["reason"]), (False, "no_position"))
        self.assertIsNone(fu["road"])
        self.assertIsNone(fu["ahead"])
        sl = self.get("/v1/speed_limit")
        self.assertIn(fu["speed_limit"]["source"], ("sign", "vision_default", "unknown"))
        self.assertEqual(fu["speed_limit"]["vision"]["kind"], sl["source"]["kind"])
        self.assertEqual(fu["place"]["source"], "vision")
        self.assertEqual(fu["lanes"]["count"], None)
        # the vision endpoints answer as before
        for path in ("/v1/road", "/v1/signs", "/v1/env", "/v1/speed_limit", "/v1/rules", "/v1/lights", "/v1/camera"):
            self.assertIsNotNone(self.get(path), path)
        h = self.get("/v1/health")
        self.assertEqual((h["map"]["state"], h["map"]["available"]), ("no_source", False))
        self.assertEqual(h["fused"]["map_available"], False)

    def test_2_position_over_http(self):
        t = time.monotonic_ns()
        msgs = []
        for i in range(8):
            lat, lon = SP.ll(100 + 3 * i, 0)
            ts = t - (8 - i) * 150_000_000
            msgs.append({"type": "gnss", "ts_mono_ns": ts, "lat": lat, "lon": lon, "hacc_m": 4.0, "speed_mps": 20.0,
                         "course_deg": 90.0, "source": "e2e"})
            msgs.append({"type": "motion", "ts_mono_ns": ts, "speed_mps": 20.0, "yaw_rate_rps": 0.0, "source": "e2e"})
        code, rep = self.post("/v1/position", "\n".join(json.dumps(m) for m in msgs))
        self.assertEqual((code, rep["accepted"], rep["rejected"]), (200, 16, []))
        code, rep = self.post("/v1/position", json.dumps([{"type": "gnss", "lat": 99.0, "lon": 21.0},
                                                          {"type": "motion", "speed_mps": 20.0}]))
        self.assertEqual((code, rep["accepted"]), (200, 1))
        self.assertEqual(rep["rejected"][0]["index"], 0)
        self.assertIn("lat out of range", rep["rejected"][0]["reason"])
        self.assertEqual(self.post("/v1/position", "{nope")[0], 400)
        self.wait(lambda: self.get("/v1/map")["available"], 5)
        m = self.get("/v1/map")
        self.assertEqual((m["state"], m["road"]["name"], m["road"]["ref"], m["speed_limit"]["limit_kmh"]),
                         ("good", "Main", "DK1", 70))
        self.assertEqual(m["input"]["gnss"]["source"], "e2e")
        self.wait(lambda: self.get("/v1/fused")["map"]["available"], 5)
        fu = self.get("/v1/fused")
        self.assertEqual((fu["road"]["name"], fu["lanes"]["count"], fu["built_up"]["source"]), ("Main", 2, "map_tagged"))
        self.assertIn(fu["speed_limit"]["source"], ("map", "sign", "sign_held"))
        self.assertEqual(fu["speed_limit"]["map"]["limit_kmh"], 70)
        self.assertEqual(fu["tunnel"]["in_tunnel"], fu["place"]["label"] == "tunnel" and fu["tunnel"]["source"] == "vision")
        kinds = [f["kind"] for f in fu["ahead"]["features"]]
        self.assertEqual(kinds[:2], ["traffic_signals", "crossing"])
        h = self.get("/v1/health")
        self.assertEqual((h["map"]["available"], h["map"]["road"], h["fused"]["road"]), (True, "DK1", "DK1"))
        sys.path.insert(0, os.path.join(REPO, "api"))
        from roadsense_api.server import map_hud
        line = map_hud(fu)
        self.assertTrue(line.startswith("MAP: DK1 Main  limit "), line)
        self.assertIn("lanes 1+1", line)
        self.assertTrue(line.isascii())

    def test_3_websocket_carries_map_and_fused(self):
        import aiohttp

        async def go():
            seen = set()
            async with aiohttp.ClientSession() as s:
                async with s.ws_connect(self.base + "/ws?topics=map,fused") as ws:
                    for _ in range(20):
                        m = json.loads((await asyncio.wait_for(ws.receive(), 5)).data)
                        seen.add(m["type"])
                        if seen == {"map", "fused"}:
                            break
            return seen
        self.assertEqual(asyncio.run(go()), {"map", "fused"})


if __name__ == "__main__":
    unittest.main()
