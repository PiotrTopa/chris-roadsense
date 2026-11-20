"""End to end: chris-integrator as a process with the map box on a hand-made pack, fed over its sockets and
`POST /v2/vehicle` as the legacy mapd + api were (the checks of the removed `api/tests/test_map_e2e.py` that had
no equivalent: `reset` over HTTP, the `vehicle` topic's signal table, the input's error replies), and the
reference clients of the vehicle input API (`tools/vehicle_client.py`, `tools/position_client.py`) when the
roadsense tree is next to this one. No camera: the estimator's outputs go out on the vehicle's samples."""
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

HERE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
REPO = os.path.dirname(HERE)
MAPPER = os.path.join(REPO, "chris-mapper")
TOOLS = os.path.join(REPO, "tools")
NS = 10 ** 9

try:
    import importlib.util
    _spec = importlib.util.spec_from_file_location("synthpack_e2e", os.path.join(MAPPER, "tests", "synthpack.py"))
    SP = importlib.util.module_from_spec(_spec)
    sys.path.insert(0, MAPPER)
    _spec.loader.exec_module(SP)
except (ImportError, OSError, FileNotFoundError):
    SP = None
try:
    import aiohttp
    HAVE_AIOHTTP = True
except ImportError:
    HAVE_AIOHTTP = False
HAVE_CLIENTS = os.path.exists(os.path.join(TOOLS, "vehicle_client.py")) and os.path.exists(os.path.join(TOOLS, "position_client.py"))


def free_port():
    with socket.socket() as s:
        s.bind(("127.0.0.1", 0))
        return s.getsockname()[1]


@unittest.skipUnless(SP is not None and HAVE_AIOHTTP, "needs chris-mapper next to this tree and aiohttp")
class VehicleInputE2E(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.tmp = tempfile.mkdtemp(prefix="chris-veh-e2e-")
        cls.rundir = os.path.join(cls.tmp, "chris")
        rs = os.path.join(cls.tmp, "roadsense")
        datadir = os.path.join(cls.tmp, "data")
        etc = os.path.join(cls.tmp, "etc")
        for d in (rs, datadir, etc):
            os.makedirs(d)
        with open(os.path.join(etc, "vehicle_input.json"), "w") as f:
            json.dump({"vehicle": {"name": "Configured car", "wheelbase_m": 2.6}}, f)
        pack = os.path.join(cls.tmp, "synth.rsmap")
        SP.build(pack)
        cls.port = free_port()
        cls.base = f"http://127.0.0.1:{cls.port}"
        env = dict(os.environ, PYTHONPATH=os.pathsep.join([HERE, MAPPER]), CHRIS_MAP_PACK=pack)
        cls.proc = subprocess.Popen(
            [sys.executable, "-m", "chris_integrator", "--rundir", cls.rundir, "--datadir", datadir, "--config-dir", etc,
             "--roadsense", rs, "--host", "127.0.0.1", "--http-port", str(cls.port)], env=env)
        try:
            cls.wait(lambda: cls.get("/v2/vehicle") is not None and os.path.exists(os.path.join(cls.rundir, "position.sock")), 20)
        except BaseException:
            cls.tearDownClass()
            raise

    @classmethod
    def tearDownClass(cls):
        cls.proc.terminate()
        cls.proc.wait(10)
        shutil.rmtree(cls.tmp, ignore_errors=True)

    @classmethod
    def get(cls, path):
        try:
            with urllib.request.urlopen(cls.base + path, timeout=5) as r:
                return json.loads(r.read())
        except (urllib.error.URLError, OSError, ValueError):
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
            time.sleep(0.1)
        raise AssertionError("timed out" + ("" if cls.proc.poll() is None else ": the integrator exited with %s" % cls.proc.returncode))

    def drive(self, source, x0=100.0, n=45, v=20.0):
        """n motion samples at 10 Hz and a fix every 0.5 s along the synthetic main road, stamped over the last
        n / 10 s (within the 5 s the input accepts)."""
        t = time.monotonic_ns()
        msgs = []
        for i in range(n):
            ts = t - (n - i) * NS // 10
            msgs.append({"type": "motion", "ts_mono_ns": ts, "speed_mps": v, "yaw_rate_rps": 0.0, "gear": "D", "source": source})
            if i % 5 == 4:
                lat, lon = SP.ll(x0 + v * i / 10.0, 0)
                msgs.append({"type": "gnss", "ts_mono_ns": ts, "lat": lat, "lon": lon, "hacc_m": 4.0, "speed_mps": v,
                             "course_deg": 90.0, "source": source})
        return msgs

    def test_1_vehicle_topic_without_a_feeder(self):
        v = self.get("/v2/vehicle")
        self.assertEqual((v["type"], v["api"]), ("vehicle", 1))
        self.assertEqual((v["description"]["name"], v["description"]["wheelbase_m"], v["description"]["sources"]["wheelbase_m"]),
                         ("Configured car", 2.6, "config"))                  # <config dir>/vehicle_input.json
        self.assertEqual((v["calibration"]["yaw_rate"]["scale"]["ref"], v["input"]["resets"]), ("estimator", 0))
        m = self.get("/v2/map")
        self.assertIn(m["state"] if m else "no_source", ("no_source",))

    def test_2_raw_lines_and_error_replies(self):
        """position.sock line for line as mapd's: a hello reply, an error line per rejected line (numbered from 1
        on this connection), nothing for an accepted one; the connection stays open."""
        for name in ("position.sock", "feed.sock"):
            s = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
            s.connect(os.path.join(self.rundir, name))
            s.settimeout(5.0)
            s.sendall(b'{"type":"hello","api":1,"name":"raw"}\nnot json\n{"type":"wipers"}\n'
                      b'{"type":"motion","speed_mps":3.0}\n\n{"type":"motion","speed_mps":-1.0}\n'
                      b'{"type":"controls","brake":"yes"}\n{"type":"motion","speed_mps":3.0,"ts_mono_ns":5}\n')
            data = b""
            while data.count(b"\n") < 6:
                data += s.recv(65536)
            lines = [json.loads(x) for x in data.splitlines()]
            s.close()
            hello = lines[0]
            self.assertEqual((hello["type"], hello["api"], hello["minor"], hello["server"], hello["clock"]),
                             ("hello", 1, 1, "chris-integrator", "CLOCK_MONOTONIC"))
            self.assertLess(abs(hello["now_mono_ns"] - time.monotonic_ns()), 2 * NS)
            self.assertEqual(hello["accepts"], ["gnss", "motion", "reset", "controls", "body", "vehicle", "objects"])
            self.assertEqual([(e["type"], e["line"]) for e in lines[1:]],
                             [("error", 2), ("error", 3), ("error", 6), ("error", 7), ("error", 8)])
            self.assertEqual([e["reason"] for e in lines[1:5]],
                             ["bad JSON", "unknown type 'wipers'", "speed_mps out of range", "brake must be true or false"])
            self.assertTrue(lines[5]["reason"].startswith("ts_mono_ns is ") and "s old" in lines[5]["reason"])

    def test_3_vehicle_input_over_http_and_the_signal_table(self):
        code, rep = self.post("/v2/vehicle", json.dumps([
            {"type": "vehicle", "name": "E2E car", "camera": {"x_m": 1.9, "y_m": 0.0, "z_m": 1.2},
             "sensors": [{"id": "radar0", "kind": "radar", "x_m": 3.7, "y_m": 0.0}]},
            {"type": "controls", "brake": True, "turn_signal": "right", "accelerator": 0.3,
             "quality": {"accelerator": "unverified"}, "source": "e2e"},
            {"type": "body", "low_beam": True, "wipers": "low", "source": "e2e"},
            {"type": "objects", "sensor": "radar0", "objects": [{"id": 3, "range_m": 20.0, "azimuth_deg": 0.0}]},
            {"type": "controls", "brake": "yes"}]))
        self.assertEqual((code, rep["api"], rep["accepted"], [r["index"] for r in rep["rejected"]]), (200, 1, 4, [4]))
        self.assertEqual(rep["rejected"][0]["reason"], "brake must be true or false")
        code, rep = self.post("/v2/vehicle", json.dumps([{"type": "gnss", "lat": 99.0, "lon": 21.0},
                                                         {"type": "motion", "speed_mps": 2.0}]))
        self.assertEqual((code, rep["accepted"], rep["rejected"]), (200, 1, [{"index": 0, "reason": "lat out of range"}]))
        self.assertEqual(self.post("/v2/vehicle", "{nope")[0], 400)
        self.wait(lambda: self.get("/v2/vehicle")["controls"]["brake"] is True and self.get("/v2/vehicle")["objects"], 5)
        v = self.get("/v2/vehicle")
        self.assertEqual((v["present"], v["controls"]["turn_signal"], v["body"]["wipers"], v["description"]["name"]),
                         (True, "right", "low", "E2E car"))
        self.assertEqual(v["signals"]["brake"]["source"], "e2e")
        self.assertEqual((v["signals"]["accelerator"]["quality"], v["controls"]["accelerator"]), ("unverified", 0.3))
        self.assertEqual(set(v["signals"]) >= {"brake", "turn_signal", "accelerator", "low_beam", "wipers", "speed_mps"}, True)
        self.assertEqual((v["sensors"][0]["id"], v["sensors"][0]["mounted"], v["sensors"][0]["objects"]), ("radar0", True, 1))
        o = v["objects"][0]
        self.assertEqual((o["sensor"], o["frame"], o["mount"]["x_m"], o["objects"][0]["range_m"]), ("radar0", "sensor", 1.8, 20.0))
        h = self.get("/v2/health")["vehicle"]
        self.assertEqual((h["present"], "brake" in h["signals"], h["sensors"]), (True, True, ["radar0"]))

        async def go():
            seen = {}
            async with aiohttp.ClientSession() as s:
                async with s.ws_connect(self.base + "/ws?topics=vehicle,objects") as ws:
                    await asyncio.sleep(0.3)
                    await s.post(self.base + "/v2/vehicle", data=json.dumps({"type": "objects", "sensor": "radar0", "objects": []}))
                    for _ in range(20):
                        m = json.loads((await asyncio.wait_for(ws.receive(), 5)).data)
                        seen[m["type"]] = m
                        if len(seen) == 2:
                            break
            return seen
        seen = asyncio.run(go())
        self.assertEqual(set(seen), {"vehicle", "objects"})
        self.assertEqual(seen["objects"]["objects"], [])

    def test_4_reset_over_http_forgets_the_position(self):
        code, rep = self.post("/v2/vehicle", json.dumps(self.drive("synthetic")))
        self.assertEqual((code, rep["rejected"]), (200, []))
        self.wait(lambda: self.get("/v2/map")["state"] == "good", 10)
        m = self.get("/v2/map")
        self.assertEqual((m["input"]["synthetic"], m["input"]["gnss"]["source"]), (True, "synthetic"))
        self.assertTrue(self.get("/v2/ego_state")["position"]["valid"])
        code, rep = self.post("/v2/vehicle", json.dumps({"type": "reset", "reason": "e2e"}))
        self.assertEqual((code, rep["accepted"], rep["rejected"]), (200, 1, []))
        self.wait(lambda: self.get("/v2/map")["state"] == "no_source", 5)  # at once: no further motion is sent
        m = self.get("/v2/map")
        self.assertEqual((m["available"], m["reason"], m.get("position"), m["input"]["last_reset"]["reason"]),
                         (False, "no_position", None, "e2e"))
        self.assertEqual(m["input"]["synthetic"], False)
        es = self.get("/v2/ego_state")
        self.assertFalse(es["position"]["valid"])
        self.assertIn(es["position"]["reason"], ("no_absolute_source", "no_measurement"))   # (the motion is seconds old)
        self.wait(lambda: self.get("/v2/vehicle")["input"]["resets"] == 1, 2)   # (changes go out at most 20 a second)
        self.assertEqual(self.get("/v2/vehicle")["input"]["last_reset"]["reason"], "e2e")
        # motion goes on without a position; a real receiver brings it back
        code, rep = self.post("/v2/vehicle", json.dumps(self.drive("e2e", x0=1500.0)))
        self.assertEqual(rep["rejected"], [])
        self.wait(lambda: self.get("/v2/map")["state"] == "good", 10)
        self.assertEqual(self.get("/v2/map")["input"]["gnss"]["source"], "e2e")
        self.assertEqual(self.get("/v2/vehicle")["input"]["resets"], 1)

    @unittest.skipUnless(HAVE_CLIENTS, "needs roadsense's tools/ (the reference clients) next to this tree")
    def test_5_reference_clients(self):
        sys.path.insert(0, TOOLS)
        from position_client import PositionFeed
        from vehicle_client import VehicleFeed
        feed = VehicleFeed(self.rundir, name="test-car", provides={"motion": ["speed_mps"], "controls": ["brake"]})
        self.assertTrue(feed.motion(0.0, source="t"))
        end = time.monotonic() + 3
        while feed.accepts is None and time.monotonic() < end:
            time.sleep(0.02)
            feed.poll()
        self.assertEqual((feed.server["server"], feed.server["api"], feed.server["minor"]), ("chris-integrator", 1, 1))
        self.assertTrue(feed.supports("objects"))
        self.wait(lambda: any(f["name"] == "test-car" for f in self.get("/v2/vehicle")["feeders"]), 3)
        f = next(f for f in self.get("/v2/vehicle")["feeders"] if f["name"] == "test-car")
        self.assertEqual(f["provides"], {"motion": ["speed_mps"], "controls": ["brake"]})
        feed.close()
        self.wait(lambda: not any(f["name"] == "test-car" for f in self.get("/v2/vehicle")["feeders"]), 3)
        old = PositionFeed(self.rundir, name="old")                       # the v1 client: error replies as mapd's
        self.assertTrue(old.motion(3.0, source="t"))
        old.send({"type": "motion", "speed_mps": -1.0})
        end = time.monotonic() + 3
        while not old.errors and time.monotonic() < end:
            time.sleep(0.05)
            old._poll_replies()
        self.assertEqual([e["reason"] for e in old.errors], ["speed_mps out of range"])
        old.close()


if __name__ == "__main__":
    unittest.main()
