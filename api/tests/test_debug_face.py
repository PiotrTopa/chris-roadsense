"""The debug face of one roadsense instance over fake daemons: raw topics, the camera ring, /preview, the
governor message for chris-integrator; the consolidated topics answer 410 (they are chris-integrator's).

    cd <repo> && python3 -m unittest api.tests.test_debug_face -v
"""
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


def free_port():
    with socket.socket() as s:
        s.bind(("127.0.0.1", 0))
        return s.getsockname()[1]


class DebugFaceTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.tmp = tempfile.mkdtemp(prefix="roadsense-test-")
        cls.rundir = os.path.join(cls.tmp, "run")
        os.makedirs(cls.rundir)
        cls.port = free_port()
        cls.base = f"http://127.0.0.1:{cls.port}"
        cls.procs = [
            subprocess.Popen([sys.executable, "-m", "roadsense_api", "--rundir", cls.rundir,
                              "--ring", os.path.join(cls.rundir, "roadsense-frames"), "--host", "127.0.0.1",
                              "--port", str(cls.port)], cwd=os.path.join(REPO, "api"),
                             env=dict(os.environ, ROADSENSE_CONFIG_DIR=os.path.join(cls.tmp, "etc"))),
            subprocess.Popen([sys.executable, os.path.join(REPO, "tools", "fake_daemons.py"), "--rundir", cls.rundir,
                              "--no-mapd"])]
        try:
            cls.wait(lambda: all(d["connected"] for d in cls.get("/v2/health")["daemons"].values()), 15)
            cls.wait(lambda: cls.get("/v2/lights")["type"] == "lights" and cls.get("/v2/signs").get("type") == "signs"
                     and os.path.exists(os.path.join(cls.rundir, "preview.jpg")), 15)
        except BaseException:
            cls.tearDownClass()
            raise

    @classmethod
    def tearDownClass(cls):
        for p in cls.procs:
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
            except (OSError, KeyError):
                pass
            time.sleep(0.2)
        raise AssertionError("timed out waiting")

    @classmethod
    def get(cls, path):
        with urllib.request.urlopen(cls.base + path, timeout=5) as r:
            return json.loads(r.read())

    def test_raw_topics_and_ring(self):
        self.assertEqual(self.get("/v2/road")["type"], "road")
        self.assertEqual(len(self.get("/v1/road")["lane_lines"]), 4)
        self.assertIn("live_tracks", self.get("/v2/signs"))
        self.assertEqual(self.get("/v2/lights")["type"], "lights")
        self.assertNotIn("rate", self.get("/v2/lights"))               # raw: nothing is annotated here
        c = self.get("/v2/camera")
        self.assertEqual(c["ring"]["width"], 1920)
        h = self.get("/v2/health")
        self.assertEqual(set(h["daemons"]), {"camerad", "modeld", "signd", "envd", "lightd"})
        self.wait(lambda: self.get("/v2/health")["fps"]["road"] > 3, 10)
        self.assertEqual(h["api"]["major"], 3)

    def test_consolidated_topics_are_gone(self):
        for name in ("lead", "lanes", "view", "fused", "rules", "speed_limit", "map", "ego_state", "tracks"):
            with self.assertRaises(urllib.error.HTTPError) as e:
                urllib.request.urlopen(self.base + "/v2/" + name, timeout=5)
            self.assertEqual(e.exception.code, 410, name)

    def test_preview_and_frame(self):
        with urllib.request.urlopen(self.base + "/v2/frame.jpg", timeout=5) as r:
            self.assertTrue(r.read().startswith(b"\xff\xd8"))
        with urllib.request.urlopen(self.base + "/preview", timeout=10) as r:
            self.assertTrue(r.headers["Content-Type"].startswith("multipart/x-mixed-replace"))
            while True:
                line = r.readline()
                if line.lower().startswith(b"content-length:"):
                    n = int(line.split(b":")[1])
                    r.readline()
                    self.assertTrue(r.read(n).startswith(b"\xff\xd8"))
                    break

    def test_governor_message_for_the_integrator(self):
        with socket.socket(socket.AF_UNIX) as s:
            s.settimeout(5)
            s.connect(os.path.join(self.rundir, "api.sock"))
            g = json.loads(s.makefile().readline())
        self.assertEqual(g["type"], "governor")
        self.assertIn(g["level"], range(0, g["levels"] + 1))      # whatever this host's temperature makes it
        self.assertIn("view_hz", g["caps"])
        self.assertIn("limited", g)


if __name__ == "__main__":
    unittest.main()


class LazyRoadTest(unittest.TestCase):
    """`road` lines stay text until a reader asks (lane L48): same answers, no parse per frame."""

    def test_road_line_is_parsed_on_demand_only(self):
        from roadsense_api.sockets import LAZY_PREFIX
        from roadsense_api.state import State
        st = State("/nonexistent")
        got = []
        st.subscribers.add(lambda t, line: got.append((t, line)))
        line = '{"type":"road","seq":7,"ts_mono_ns":5,"ts_real_ns":%d,"model_ms":44.0}' % (time.time_ns() - 50_000_000)
        self.assertTrue(line.startswith(LAZY_PREFIX))
        st.on_line("road", "modeld", line)
        self.assertEqual(got, [("road", line)])                 # relayed as it came
        self.assertNotIn("road", st.latest)                     # nothing parsed yet
        self.assertAlmostEqual(st._road_lat_ms, 50.0, delta=20.0)
        self.assertEqual(st.latest.get("road")["seq"], 7)       # parsed when asked
        self.assertEqual(st.health()["model_ms"], 44.0)
        st.on_line("road", "modeld", line.replace('"seq":7', '"seq":8'))
        self.assertEqual(st.latest.get("road")["seq"], 8)       # the newest line wins over the parsed one
        st.on_message("modeld", {"type": "road", "seq": 9}, "{}")
        self.assertEqual(st.latest.get("road")["seq"], 9)       # a parsed message replaces a waiting line
