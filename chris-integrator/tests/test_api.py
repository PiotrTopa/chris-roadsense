"""End-to-end test: tools/fake_daemons.py + the api as subprocesses, every endpoint exercised.

    cd <repo> && python3 -m unittest discover -s api/tests -v
Env: ROADSENSE_RUNDIR (default: a temp dir), ROADSENSE_TEST_PORT (default: a free port), OVERLAY_OUT (save one /preview frame there).
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
import urllib.request

REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
HERE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
HAVE_ROADSENSE = os.path.exists(os.path.join(REPO, "tools", "fake_daemons.py")) and os.path.isdir(os.path.join(REPO, "api", "roadsense_api"))


def free_port():
    with socket.socket() as s:
        s.bind(("127.0.0.1", 0))
        return s.getsockname()[1]


@unittest.skipUnless(HAVE_ROADSENSE, "needs the chris-roadsense tree next to this one (its fake daemons and debug face)")
class ApiTest(unittest.TestCase):
    """The whole stack as processes: fake roadsense daemons -> their sockets -> chris-integrator (the consolidated
    API under test), with the roadsense debug face behind it for what belongs to the camera (/preview, the ring)."""

    @classmethod
    def setUpClass(cls):
        cls.tmp = tempfile.mkdtemp(prefix="chris-test-")
        cls.rundir = os.path.join(cls.tmp, "run")
        cls.datadir = os.path.join(cls.tmp, "data")
        os.makedirs(cls.rundir, exist_ok=True)
        os.makedirs(cls.datadir)
        cls.port, cls.dbg_port = free_port(), free_port()
        cls.base = f"http://127.0.0.1:{cls.port}"
        env = dict(os.environ, PYTHONPATH=os.pathsep.join([HERE, os.path.join(REPO, "chris-mapper")]),
                   CHRIS_MAP_PACK=os.path.join(cls.tmp, "no.rsmap"))
        cls.debug = subprocess.Popen(
            [sys.executable, "-m", "roadsense_api", "--rundir", cls.rundir,
             "--ring", os.path.join(cls.rundir, "roadsense-frames"), "--host", "127.0.0.1", "--port", str(cls.dbg_port)],
            cwd=os.path.join(REPO, "api"), env=dict(os.environ, ROADSENSE_CONFIG_DIR=os.path.join(cls.tmp, "etc")))
        cls.api = subprocess.Popen(
            [sys.executable, "-m", "chris_integrator", "--rundir", os.path.join(cls.tmp, "chris"), "--datadir", cls.datadir,
             "--config-dir", os.path.join(cls.tmp, "etc"), "--roadsense", cls.rundir,
             "--roadsense-http", f"http://127.0.0.1:{cls.dbg_port}", "--host", "127.0.0.1", "--http-port", str(cls.port)],
            env=env)
        cls.start_fake()
        try:
            cls.wait(lambda: all(d["connected"] for d in cls.get("/v1/health")["daemons"].values()), 15)
            # one sign track lives 40 x 0.1 s; wait until it has ended
            cls.wait(lambda: cls.get("/v1/signs/history")["count"] >= 1, 15)
        except BaseException:
            # unittest does not call tearDownClass when setUpClass fails: without this the fake daemons stay
            # behind (e.g. an interpreter without aiohttp: the api exits at once) and, holding the runner's
            # stdout, make `... | tail` or any harness that waits for end-of-file hang for ever
            cls.tearDownClass()
            raise

    @classmethod
    def start_fake(cls):
        cls.fake = subprocess.Popen([sys.executable, os.path.join(REPO, "tools", "fake_daemons.py"),
                                     "--rundir", cls.rundir, "--no-mapd"])

    @classmethod
    def tearDownClass(cls):
        for p in (cls.fake, cls.api, cls.debug):
            p.terminate()
            p.wait(10)
        shutil.rmtree(cls.tmp, ignore_errors=True)

    @classmethod
    def wait(cls, cond, timeout):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            try:
                if cond():
                    return
            except OSError:
                pass
            time.sleep(0.2)
        raise AssertionError("timed out waiting" + ("" if cls.api.poll() is None else
                                                    ": the api process exited with %s (needs aiohttp)" % cls.api.returncode))

    @classmethod
    def fetch(cls, path):
        with urllib.request.urlopen(cls.base + path, timeout=5) as r:
            return r.status, r.headers, r.read()

    @classmethod
    def get(cls, path):
        st, _, body = cls.fetch(path)
        assert st == 200, (path, st)
        return json.loads(body)

    def test_index(self):
        st, hdr, body = self.fetch("/")
        self.assertEqual(st, 200)
        self.assertIn(b"/preview", body)
        self.assertEqual(hdr["Access-Control-Allow-Origin"], "*")

    def test_road(self):
        r = self.get("/v1/road")
        for k in ("type", "seq", "ts_mono_ns", "ts_real_ns", "model_ms", "plan", "lane_lines", "road_edges",
                  "leads", "pose", "meta"):
            self.assertIn(k, r)
        self.assertEqual(r["type"], "road")
        self.assertEqual(len(r["lane_lines"]), 4)

    def test_signs(self):
        s = self.get("/v1/signs")
        for k in ("type", "seq", "detect_ms", "detections", "tracks_ended", "live_tracks"):
            self.assertIn(k, s)

    def test_signs_history(self):
        h = self.get("/v1/signs/history?limit=1")
        self.assertEqual(h["count"], 1)
        for k in ("track_id", "cls", "first_seq", "last_seq", "hits", "best_score", "best_crop"):
            self.assertIn(k, h["tracks"][0])
        with open(os.path.join(self.datadir, "signs.jsonl")) as f:
            self.assertIn("track_id", json.loads(f.readline()))

    def test_env(self):
        e = self.get("/v1/env")
        for k in ("type", "seq", "ts_mono_ns", "env_ms", "place", "weather", "light", "stats", "model"):
            self.assertIn(k, e)
        self.assertEqual(e["type"], "env")
        for g in ("place", "weather", "light"):
            for k in ("label", "confidence", "stable_s", "scores"):
                self.assertIn(k, e[g])
            self.assertIn(e[g]["label"], e[g]["scores"])
        self.assertIn(b"/v2/env", self.fetch("/")[2])

    def test_speed_limit(self):
        s = self.get("/v1/speed_limit")
        for k in ("type", "limit_kmh", "source", "confidence", "since", "expiry", "area", "road_class", "zone",
                  "pending", "v_ego_kmh", "odometer_m", "rules", "recent_changes", "ignored"):
            self.assertIn(k, s)
        self.assertEqual(s["type"], "speed_limit")
        self.assertEqual(s["rules"], "PL")
        # the fake signd shows a B-33 50 track every few seconds; the setup waited for one to end
        self.assertEqual(s["limit_kmh"], 50)
        self.assertEqual(s["source"]["kind"], "sign")
        self.assertEqual(s["source"]["sign"], "B-33")
        self.assertGreater(s["odometer_m"], 0)
        h = self.get("/v1/health")["speed_limit"]
        self.assertEqual(h["limit_kmh"], 50)
        self.assertIn(b"/v2/speed_limit", self.fetch("/")[2])

    def test_rules(self):
        # the fake signd also shows a B-25 (no overtaking) track at 6-10 s and an A-16 warning at 12-16 s of each 20 s
        self.wait(lambda: any(e["id"] == "no_overtaking" for e in self.get("/v1/rules")["in_effect"]), 25)
        r = self.get("/v1/rules")
        for k in ("type", "in_effect", "recent", "ahead", "junction", "area", "road_class", "zone", "v_ego_kmh",
                  "odometer_m", "rules", "pending", "recent_changes", "ignored"):
            self.assertIn(k, r)
        self.assertEqual(r["type"], "rules")
        by_id = {e["id"]: e for e in r["in_effect"]}
        self.assertEqual(by_id["no_overtaking"]["sign"], "B-25")
        self.assertIn("intersection", by_id["no_overtaking"]["ends"])
        for k in ("source", "since", "confidence", "ends", "remaining_m", "uncertain", "applies_to_car", "side"):
            self.assertIn(k, by_id["no_overtaking"])
        self.assertEqual(by_id["speed_limit"]["value"], 50)         # the L6 engine's B-33 50, as one of the rules
        h = self.get("/v1/health")["rules"]
        self.assertIn("B-25 no overtaking", h["in_effect"])
        self.assertIn(b"/v2/rules", self.fetch("/")[2])
        self.wait(lambda: any(w["id"] == "pedestrian_crossing_ahead" for w in self.get("/v1/rules")["ahead"]), 25)
        w = next(w for w in self.get("/v1/rules")["ahead"] if w["id"] == "pedestrian_crossing_ahead")
        self.assertEqual(w["sign"], "A-16")
        self.assertIn("expected_in_m", w)

    def test_lights(self):
        lt = self.get("/v1/lights")
        self.assertEqual(lt["type"], "lights")
        for k in ("lights_ms", "frame", "lights", "ego"):
            self.assertIn(k, lt)
        by_id = {li["track_id"]: li for li in lt["lights"]}
        for k in ("dist_m", "lateral_m", "relevance", "ego"):
            self.assertIn(k, by_id[1])
        # the fake light ahead is the ego light; the side view ('off', far left) is not
        self.assertIsNotNone(lt["ego"])
        self.assertEqual(lt["ego"]["track_id"], 1)
        self.assertTrue(by_id[1]["ego"])
        self.assertFalse(by_id[2]["ego"])
        self.assertIn(lt["ego"]["state"], ("red", "red_yellow", "green", "yellow"))
        self.assertIn("lights", self.get("/v1/health"))
        self.assertIn(b"/v2/lights", self.fetch("/")[2])

    def test_camera(self):
        c = self.get("/v1/camera")
        for k in ("type", "seq", "exposure_us", "analogue_gain", "colour_gains", "fps", "dropped", "ring"):
            self.assertIn(k, c)
        self.assertEqual(c["ring"]["width"], 1920)
        self.assertAlmostEqual(c["ring"]["focal_px"], 1500.0)

    def test_health(self):
        h = self.get("/v1/health")
        for k in ("daemons", "fps", "model_ms", "detect_ms", "env_ms", "env", "cpu_temp_c", "battery"):
            self.assertIn(k, h)
        self.assertIn("envd", h["daemons"])
        self.assertEqual(set(h["env"]), {"place", "weather", "light"})
        self.assertGreater(h["fps"]["camera"], 20)
        self.assertGreater(h["fps"]["road"], 3)
        self.assertGreater(h["fps"]["signs"], 7)
        for d in h["daemons"].values():
            self.assertLess(d["last_msg_age_s"], 1.0)

    def test_frame(self):
        st, hdr, body = self.fetch("/v1/frame.jpg")
        self.assertEqual(hdr["Content-Type"], "image/jpeg")
        self.assertTrue(body.startswith(b"\xff\xd8"))

    def test_preview(self):
        with urllib.request.urlopen(self.base + "/preview", timeout=10) as r:
            self.assertTrue(r.headers["Content-Type"].startswith("multipart/x-mixed-replace"))
            frames, t0, sign_frame = [], time.monotonic(), None
            while len(frames) < 3 or (sign_frame is None and len(frames) < 80):
                line = r.readline()
                if line.lower().startswith(b"content-length:"):
                    n = int(line.split(b":")[1])
                    r.readline()
                    frames.append(r.read(n))
                    if len(frames) == 3:
                        rate = 2 / (time.monotonic() - t0)
                    if len(frames) > 3 and self.get("/v1/signs")["live_tracks"]:
                        sign_frame = frames[-1]     # keep one frame showing a sign box for eyeballing
        for f in frames:
            self.assertTrue(f.startswith(b"\xff\xd8") and f.endswith(b"\xff\xd9"))
        self.assertGreater(rate, 3)        # preview is written at 10 Hz
        out = os.environ.get("OVERLAY_OUT")
        if out:
            os.makedirs(os.path.dirname(out), exist_ok=True)
            with open(out, "wb") as fh:
                fh.write(sign_frame or frames[-1])

    def test_ws(self):
        import aiohttp

        async def run():
            got = []
            async with aiohttp.ClientSession() as s:
                async with s.ws_connect(self.base + "/ws?topics=road,signs,env,speed_limit,rules") as ws:
                    while len(got) < 30 or "speed_limit" not in got or "rules" not in got:
                        msg = await asyncio.wait_for(ws.receive(), 5)
                        got.append(json.loads(msg.data)["type"])
            return got

        types = asyncio.run(run())
        self.assertTrue(set(types) <= {"road", "signs", "env", "speed_limit", "rules"})
        self.assertIn("signs", types)
        self.assertIn("env", types)

    def test_mixer_and_tracks(self):
        m = self.get("/v2/mixer")
        names = [p["name"] for p in sorted((p for p in m["plugins"] if p["enabled"]), key=lambda p: p["order"])]
        self.assertEqual(names[:3], ["ride_watch", "road_users", "estimator"])   # road users annotate the tracks (L42)
        self.assertLess(names.index("estimator"), names.index("mapper"))      # the map box reads the ego estimate
        self.assertLess(names.index("lanes"), names.index("view"))
        self.assertTrue(all(p["errors"] == 0 for p in m["plugins"]), [p["last_error"] for p in m["plugins"]])
        self.assertEqual(m["streams"]["lead"]["producer"], "estimator")
        self.assertIn("view", m["streams"]["lead"]["consumers"])
        self.assertEqual(self.get("/v2/tracks")["type"], "tracks")
        self.assertEqual(self.get("/v2/ego_state")["type"], "ego_state")
        self.assertEqual(self.get("/v2/map")["reason"], "no_pack")          # the map box runs and says why it has nothing
        g = self.get("/v2/health")["governor"]                             # the roadsense governor's state, an input
        self.assertTrue(g["enabled"])
        self.assertIn("view_hz", g["caps"])
        self.assertIn("lead", g["topics"])

    def test_vehicle_feed_over_http_and_socket(self):
        body = json.dumps([{"type": "motion", "speed_mps": 0.0, "wheel_speeds_mps": [0, 0, 0, 0], "yaw_rate_rps": 0.0},
                           {"type": "controls", "brake": True}]).encode()
        req = urllib.request.Request(self.base + "/v2/vehicle", data=body, method="POST")
        with urllib.request.urlopen(req, timeout=5) as r:
            self.assertEqual(json.loads(r.read())["accepted"], 2)
        self.assertTrue(self.get("/v2/vehicle")["controls"]["brake"])
        for name in ("feed.sock", "position.sock"):                           # the car backend's socket, under both names
            with socket.socket(socket.AF_UNIX) as s:
                s.connect(os.path.join(self.tmp, "chris", name))
                hello = json.loads(s.makefile().readline())
                self.assertIn("motion", hello["accepts"])
                self.assertIn("controls", hello["accepts"])

    def test_zz_reconnect(self):
        self.fake.terminate()
        self.fake.wait(5)
        self.wait(lambda: not self.get("/v1/health")["daemons"]["modeld"]["connected"], 5)
        self.start_fake()
        self.wait(lambda: all(d["connected"] for d in self.get("/v1/health")["daemons"].values()), 15)
        self.wait(lambda: self.get("/v1/health")["fps"]["road"] > 3, 10)
        self.assertEqual(self.get("/v1/road")["type"], "road")


if __name__ == "__main__":
    unittest.main()
