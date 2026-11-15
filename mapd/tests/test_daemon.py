"""mapd as a process: sockets, the position input API with the reference client, error replies, pack reload."""
import json
import os
import shutil
import socket
import subprocess
import sys
import tempfile
import time
import unittest

from . import synthpack as SP

REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, os.path.join(REPO, "tools"))

from position_client import PositionFeed      # noqa: E402


class Sub:
    """A reader of mapd.sock."""

    def __init__(self, path):
        self.s = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        self.s.connect(path)
        self.s.settimeout(5.0)
        self.buf = b""

    def next(self):
        while b"\n" not in self.buf:
            d = self.s.recv(65536)
            if not d:
                raise EOFError
            self.buf += d
        line, self.buf = self.buf.split(b"\n", 1)
        return json.loads(line)

    def until(self, cond, timeout=10.0):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            m = self.next()
            if cond(m):
                return m
        raise AssertionError("condition not met; last message: %s" % json.dumps(m)[:400])


class DaemonTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.tmp = tempfile.mkdtemp(prefix="roadsense-mapd-daemon-")
        cls.rundir = os.path.join(cls.tmp, "run")
        cls.cfgdir = os.path.join(cls.tmp, "etc")
        os.makedirs(cls.cfgdir)
        cls.packs = os.path.join(cls.tmp, "packs")
        os.makedirs(cls.packs)
        cls.current = os.path.join(cls.packs, "current.rsmap")       # absent at start: mapd must run without it
        env = dict(os.environ, ROADSENSE_CONFIG_DIR=cls.cfgdir, PYTHONPATH=os.path.join(REPO, "mapd"))
        cls.proc = subprocess.Popen([sys.executable, "-m", "roadsense_mapd", "--rundir", cls.rundir, "--pack", cls.current,
                                     "--hz", "10", "--pack-poll", "0.3", "--no-vision-odometry", "--stats", "0"], env=env)
        end = time.monotonic() + 10
        while not os.path.exists(os.path.join(cls.rundir, "position.sock")):
            assert time.monotonic() < end and cls.proc.poll() is None, "mapd did not start"
            time.sleep(0.05)
        cls.sub = Sub(os.path.join(cls.rundir, "mapd.sock"))

    @classmethod
    def tearDownClass(cls):
        cls.proc.terminate()
        cls.proc.wait(10)
        shutil.rmtree(cls.tmp, ignore_errors=True)

    def test_1_runs_without_pack_and_position(self):
        m = self.sub.next()
        self.assertEqual((m["type"], m["api"], m["state"], m["available"], m["reason"]),
                         ("map", 1, "no_source", False, "no_pack"))
        for k in ("position", "road", "speed_limit", "ahead"):
            self.assertIsNone(m[k])

    def test_2_pack_appears_and_is_loaded(self):
        real = os.path.join(self.packs, "synth-1.rsmap")
        SP.build(real)
        os.symlink(real, self.current)
        m = self.sub.until(lambda m: m["pack"] is not None)
        self.assertEqual((m["pack"]["region"], m["reason"]), ("synthetic", "no_position"))

    def test_3_feed_with_the_reference_client(self):
        feed = PositionFeed(self.rundir, name="test")
        for i in range(30):
            lat, lon = SP.ll(100 + 2 * i, 0)
            self.assertTrue(feed.gnss(lat, lon, hacc_m=4.0, speed_mps=20.0, course_deg=90.0, source="test"))
            self.assertTrue(feed.motion(20.0, yaw_rate_rps=0.0, gear="D", source="test"))
            time.sleep(0.02)
        m = self.sub.until(lambda m: m["available"])
        self.assertEqual((m["state"], m["road"]["name"], m["speed_limit"]["limit_kmh"]), ("good", "Main", 70))
        self.assertEqual((m["input"]["gnss"]["source"], m["input"]["motion"]["source"]), ("test", "vehicle"))
        self.assertEqual(feed.errors, [])
        # a bad message gets an error reply on the same connection and does not disturb the rest
        feed.send({"type": "gnss", "lat": 123.0, "lon": 21.0})
        feed.send({"type": "motion", "speed_mps": 20.0})
        time.sleep(0.3)
        feed._poll_replies()
        self.assertEqual(len(feed.errors), 1)
        self.assertIn("lat out of range", feed.errors[0]["reason"])
        feed.close()

    def test_4_hello_and_raw_lines(self):
        s = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        s.connect(os.path.join(self.rundir, "position.sock"))
        s.settimeout(5.0)
        s.sendall(b'{"type":"hello","api":1}\nnot json\n')
        data = b""
        while data.count(b"\n") < 2:
            data += s.recv(4096)
        hello, err = [json.loads(x) for x in data.splitlines()[:2]]
        self.assertEqual((hello["type"], hello["api"], hello["clock"]), ("hello", 1, "CLOCK_MONOTONIC"))
        self.assertLess(abs(hello["now_mono_ns"] - time.monotonic_ns()), 2e9)
        self.assertEqual((err["type"], err["reason"], err["line"]), ("error", "bad JSON", 2))
        s.close()

    def test_5_fix_lost_then_unknown(self):
        m = self.sub.until(lambda m: m["state"] == "dead_reckoning", 10)       # no input since test 3
        self.assertIsNotNone(m["position"])
        m = self.sub.until(lambda m: m["state"] == "unknown", 15)
        self.assertEqual((m["available"], m["reason"], m["position"]), (False, "position_unknown", None))

    def test_6_new_pack_is_picked_up_without_restart(self):
        real = os.path.join(self.packs, "synth-2.rsmap")
        from roadsense_mapd import rsmap as R
        R.build_pack(real, SP.edges(), {"region": "synthetic-2", "osm_timestamp": "", "built": "test"})
        tmp = self.current + ".new"
        os.symlink(real, tmp)
        os.replace(tmp, self.current)
        m = self.sub.until(lambda m: m["pack"] and m["pack"]["region"] == "synthetic-2")
        self.assertIsNone(self.proc.poll())
        self.assertEqual(m["type"], "map")


if __name__ == "__main__":
    unittest.main()
