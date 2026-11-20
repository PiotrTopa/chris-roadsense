"""The real consumer of the vehicle input API, read-only: a car backend's own feeder code against a chris-integrator
process. Skipped unless CHRIS_TEST_CAR_REPO names a checkout of the reference car computer
(github.com/PiotrTopa/cyberpank-prius-gen2-computer); CHRIS_TEST_CAR_PYTHON may name its interpreter (default: the
checkout's `.venv/bin/python`, else this one). Needs chris-mapper installed.

The car's code runs in a process of its own (its interpreter, its imports), driven by the script below:

  1. its client (`backend/position_client.PositionFeed`): hello, the reply's server and `accepts`, accepted
     messages, an error reply with mapd's reason;
  2. its feeder (`backend/position_feeder.PositionFeeder`) with a run-time synthetic session: the `reset` it owes at
     the start, the synthetic drive (motion + fixes tagged "synthetic", the description of the car), the map and
     the car's own `SyntheticTaint` (`backend/roadsense_scene.py`) while it runs; then the session's end, the
     `reset` it sends, and the position forgotten: the map `no_source`, the taint cleared.

Nothing in the car's checkout is written (no file, no cache: the interpreter runs with -B)."""
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

HERE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
try:
    import chris_mapper
    MAPPER = os.path.dirname(os.path.dirname(os.path.abspath(chris_mapper.__file__)))   # where it was found
except ImportError:
    MAPPER = None
CAR = os.environ.get("CHRIS_TEST_CAR_REPO")
HAVE_CAR = bool(CAR) and MAPPER is not None and os.path.isfile(os.path.join(CAR or "", "cyberpunk_computer", "backend", "position_feeder.py"))


def car_python():
    py = os.environ.get("CHRIS_TEST_CAR_PYTHON")
    if py:
        return py
    venv = os.path.join(CAR, ".venv", "bin", "python")
    return venv if os.path.exists(venv) else sys.executable


DRIVER = r'''
import json, sys, time, urllib.request
car, rundir, base = sys.argv[1:4]
sys.path.insert(0, car)
from cyberpunk_computer.backend.position_client import PositionFeed
from cyberpunk_computer.backend import position_feeder as PF
from cyberpunk_computer.backend.roadsense_scene import SyntheticTaint

out = {}
def get(path):
    with urllib.request.urlopen(base + path, timeout=5) as r:
        return json.loads(r.read())

# 1. the client
feed = PositionFeed(rundir, name="car-test", provides=PF.PROVIDES)
ok = feed.motion(0.0, gear="P", wheel_speeds_mps=[0.0] * 4, source="prius-can")
end = time.monotonic() + 3
while feed.accepts is None and time.monotonic() < end:
    time.sleep(0.02)
    feed.poll()
feed.send({"type": "motion", "speed_mps": -1.0, "source": "prius-can"})
feed.send({"type": "controls", "brake": "pressed", "source": "prius-can"})
end = time.monotonic() + 3
while len(feed.errors) < 2 and time.monotonic() < end:
    time.sleep(0.02)
    feed.poll()
out["client"] = {"sent_ok": ok, "server": feed.server, "accepts": feed.accepts, "errors": feed.errors,
                 "errors_total": feed.errors_total, "sent": feed.sent, "dropped": feed.dropped}
feed.close()

# 2. the feeder and a run-time synthetic session
cfg = PF.PositionFeederConfig(enabled=True, source="headunit", rundir=rundir, retry_s=0.2)
fdr = PF.PositionFeeder(cfg, PF.make_source(cfg))
fdr.set_active(True)
for _ in range(5):                                   # connect; the hello reply comes in
    fdr.step()
    time.sleep(0.1)
started, why = fdr.start_synthetic("test")
taint = SyntheticTaint()
seen = []
t0 = time.monotonic()
good_at = None
while time.monotonic() - t0 < 60.0:
    fdr.step()
    time.sleep(0.1)
    m = get("/v2/map")
    tainted = taint.update(m)
    seen.append((round(time.monotonic() - t0, 1), m.get("state"), (m.get("input") or {}).get("synthetic"), tainted))
    if m.get("state") == "good" and good_at is None:
        good_at = time.monotonic()
    if good_at is not None and time.monotonic() - good_at > 3.0:
        break
during = {"started": started, "why": why, "map": get("/v2/map"), "ego_state_position": get("/v2/ego_state")["position"],
          "vehicle": get("/v2/vehicle"), "status": fdr.status(), "taint": taint.tainted, "first_good_s": None if good_at is None
          else round(good_at - t0, 1)}
fdr.stop_synthetic("test end")
after_seen = []
t1 = time.monotonic()
while time.monotonic() - t1 < 4.0:
    fdr.step()
    time.sleep(0.1)
    m = get("/v2/map")
    after_seen.append((round(time.monotonic() - t1, 1), m.get("state"), (m.get("input") or {}).get("synthetic"), taint.update(m)))
after = {"map": get("/v2/map"), "ego_state_position": get("/v2/ego_state")["position"], "vehicle": get("/v2/vehicle"),
         "status": fdr.status(), "taint": taint.tainted}
fdr.stop()
out.update(during=during, after=after, seen=seen[-5:], after_seen=after_seen[:12])
print("RESULT " + json.dumps(out))
'''


def free_port():
    with socket.socket() as s:
        s.bind(("127.0.0.1", 0))
        return s.getsockname()[1]


@unittest.skipUnless(HAVE_CAR, "set CHRIS_TEST_CAR_REPO to a checkout of the car computer (and install chris-mapper) to run its feeder against the integrator")
class CarFeederAgainstTheIntegrator(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.tmp = tempfile.mkdtemp(prefix="chris-car-")
        cls.rundir = os.path.join(cls.tmp, "chris")
        rs = os.path.join(cls.tmp, "roadsense")
        datadir = os.path.join(cls.tmp, "data")
        for d in (rs, datadir):
            os.makedirs(d)
        cls.port = free_port()
        cls.base = f"http://127.0.0.1:{cls.port}"
        env = dict(os.environ, PYTHONPATH=os.pathsep.join([HERE, MAPPER]), CHRIS_MAP_PACK=os.path.join(cls.tmp, "no.rsmap"))
        cls.proc = subprocess.Popen(
            [sys.executable, "-m", "chris_integrator", "--rundir", cls.rundir, "--datadir", datadir,
             "--config-dir", os.path.join(cls.tmp, "etc"), "--roadsense", rs, "--host", "127.0.0.1",
             "--http-port", str(cls.port)], env=env)
        end = time.monotonic() + 20
        while time.monotonic() < end and not os.path.exists(os.path.join(cls.rundir, "position.sock")):
            time.sleep(0.1)
        try:
            with urllib.request.urlopen(cls.base + "/v2/vehicle", timeout=5):
                pass
        except OSError:
            time.sleep(1.0)
        drv = os.path.join(cls.tmp, "car_driver.py")
        with open(drv, "w") as f:
            f.write(DRIVER)
        car_env = {k: v for k, v in os.environ.items() if k not in ("PYTHONPATH", "PYTHONHOME")}   # its own imports only
        p = subprocess.run([car_python(), "-B", drv, CAR, cls.rundir, cls.base], capture_output=True, text=True,
                           timeout=180, cwd=cls.tmp, env=dict(car_env, PYTHONDONTWRITEBYTECODE="1"))
        cls.log = p.stderr[-4000:]
        lines = [ln for ln in p.stdout.splitlines() if ln.startswith("RESULT ")]
        cls.r = json.loads(lines[-1][7:]) if lines else None

    @classmethod
    def tearDownClass(cls):
        cls.proc.terminate()
        cls.proc.wait(10)
        shutil.rmtree(cls.tmp, ignore_errors=True)

    def setUp(self):
        self.assertIsNotNone(self.r, "the car-side driver produced no result:\n" + self.log)

    def test_1_client_hello_acceptance_and_error_replies(self):
        c = self.r["client"]
        self.assertTrue(c["sent_ok"])
        self.assertEqual((c["server"]["server"], c["server"]["api"], c["server"]["minor"]), ("chris-integrator", 1, 1))
        self.assertEqual(c["accepts"], ["gnss", "motion", "reset", "controls", "body", "vehicle", "objects"])
        self.assertEqual([(e["line"], e["reason"]) for e in c["errors"]],
                         [(3, "speed_mps out of range"), (4, "brake must be true or false")])   # line 1: its hello
        self.assertEqual((c["errors_total"], c["dropped"]), (2, 0))

    def test_2_synthetic_drive_is_served(self):
        d = self.r["during"]
        self.assertTrue(d["started"], d["why"])
        st = d["status"]
        self.assertTrue(st["vehicle_api"])                                   # the feeder speaks vehicle input API 1.1
        self.assertEqual((st["errors"], st["reset_sent"], st["vehicle_sent"]), (0, 1, 1), st)   # the session's start reset
        self.assertIsNotNone(d["first_good_s"], self.r["seen"])
        m = d["map"]
        self.assertEqual((m["state"], m["input"]["synthetic"], m["input"]["gnss"]["source"]), ("good", True, "synthetic"))
        self.assertTrue(d["taint"])                                          # the car's own reading: synthetic
        self.assertTrue(d["ego_state_position"]["valid"])
        v = d["vehicle"]
        self.assertEqual((v["description"]["name"], v["description"]["sources"]["track_m"]), ("Toyota Prius NHW20", "feeder"))
        self.assertIn("prius-backend", [f["name"] for f in v["feeders"]])
        self.assertEqual((v["signals"]["speed_mps"]["source"], v["motion"]["yaw_source"]), ("synthetic", "wheel_speeds"))
        self.assertEqual((v["input"]["resets"], v["input"]["last_reset"]["reason"]), (1, "synthetic session started"))

    def test_3_reset_at_the_end_forgets_the_position(self):
        a = self.r["after"]
        st = a["status"]
        self.assertEqual((st["errors"], st["reset_sent"], st["last_reset"]["sent"]), (0, 2, True), st)
        self.assertEqual(st["last_reset"]["reason"], "synthetic session ended: test end")
        m = a["map"]
        self.assertEqual((m["state"], m["available"], m["input"]["synthetic"]), ("no_source", False, False), self.r["after_seen"])
        self.assertEqual(m["input"]["last_reset"]["reason"], "synthetic session ended: test end")
        self.assertFalse(a["taint"])                                         # the car's SyntheticTaint is clear
        self.assertFalse(a["ego_state_position"]["valid"])
        v = a["vehicle"]
        # the head unit's `fix: none` right after the session also tells the store the receiver changed (mapd's rule)
        self.assertIn(v["input"]["resets"], (2, 3))
        self.assertTrue(any(r for r in [v["input"]["last_reset"]] if r["reason"].startswith(("synthetic session ended",
                                                                                              "gnss source changed"))))


if __name__ == "__main__":
    unittest.main()
