"""Fault detection and isolation as a regression test: a few faults of different classes from the taxonomy
(sim/faults.py) through the integrator, with what it has to report and what its outputs have to be meanwhile.
The whole campaign with its table is tools/fault_campaign.py of the roadsense repository."""
import math
import os
import sys
import unittest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from chris_integrator import Integrator  # noqa: E402
from chris_integrator.sim import World, default_sensors  # noqa: E402
from chris_integrator.sim.faults import Fault, apply  # noqa: E402
from chris_integrator.sim.world import SCENARIOS  # noqa: E402

SHORT = dict(SCENARIOS["town"], T=140.0)          # the town drive up to 140 s: the car moves from 66 s on
T0 = 95.0
TRUE = dict(vehicle=dict(gyro_bias=0.004, gyro_scale=0.04, accel_bias=0.15), vision=dict(mount_yaw=-0.06, mount_pitch=0.05, speed_scale=0.03),
            gnss=dict(lag=2.2))
_STREAM = {}

from chris_integrator.core.integrator import GATE_ALL  # noqa: E402


def stream(seed=1):
    if seed not in _STREAM:
        w = World(seed=seed, scenario=SHORT, speed_scale=0.015)
        tk = {k: dict(v, seed=100 * seed + i) for i, (k, v) in enumerate(dict(TRUE, map={}).items())}
        _STREAM[seed] = (w, [m for _, m in w.messages(default_sensors(**tk))])
    return _STREAM[seed]


def run(faults, seed=1, gate=True):
    """[(t, ego_state, truth)] per road frame. gate: every detector acts (the simulated car's sensors support it);
    False: the defaults (detectors not shown quiet on a real car report only)."""
    w, msgs = stream(seed)
    I = Integrator(dict({"gnss_absolute": True}, **({"detectors": GATE_ALL} if gate else {})))
    out = []
    for m in apply(msgs, faults):
        if m["type"] != "road":
            I.on_message(m)
            continue
        es, tr, ld = I.frame(m)
        T = m["ts_mono_ns"] / 1e9
        out.append((T - w.t0, es, w.truth_at(T)))
    return out


def findings(es, level=("warn", "fault")):
    return {(d["source"], d["kind"]) for d in es["diagnostics"] if d["level"] in level}


class Faults(unittest.TestCase):
    def test_healthy_drive_raises_nothing(self):
        for t, es, tru in run([]):
            if t < 20.0:
                continue
            self.assertEqual({g for g, s in es["sources"].items() if s["state"] in ("fault", "suspect")}, set(), t)
            self.assertEqual(findings(es), set(), t)
            self.assertNotEqual(es["speed"]["mode"], "disputed")

    def test_stuck_vehicle_speed_is_self_evident(self):
        """A frozen speed needs no second opinion: the vehicle is faulted by its own signal, the camera (the lower
        rank, which disagrees with it) is never blamed, and the speed goes on from the camera with an honest std."""
        rows = run([Fault("vehicle.speed", "stuck", T0, 130.0)])
        self.assertTrue(all(es["sources"]["vision"]["state"] != "fault" for t, es, _ in rows))
        hit = [t for t, es, _ in rows if es["sources"]["vehicle"]["state"] == "fault"]
        self.assertLess(hit[0] - T0, 3.5)
        self.assertEqual(next(es for t, es, _ in rows if t > T0 + 5)["sources"]["vehicle"]["isolation"], "self")
        for t, es, tru in rows:
            if T0 + 4.0 <= t < 130.0 and es["speed"]["valid"]:
                self.assertLess(abs(es["speed"]["value"] - 1.015 * tru["v"]), max(3.0 * es["speed"]["std"], 0.5), t)

    def test_report_mode_names_the_fault_and_sets_nothing_aside(self):
        """The defaults: a frozen speed and a gyro bias are found and published with the detector's name and its
        evidence; no source and no quantity is set aside by a detector in report mode; no output goes invalid."""
        for fault, det, src in ((Fault("vehicle.speed", "stuck", T0, 130.0), "signal.stuck", "vehicle"),
                                (Fault("vehicle.gyro", "bias", T0, 130.0, b=0.04), "yaw_rate.cusum", None)):
            rows = run([fault], gate=False)
            late = [es for t, es, _ in rows if T0 + 8.0 <= t < 130.0]
            rep = [d for es in late for d in es["diagnostics"] if d.get("kind") == "reported" and d.get("detector") == det]
            self.assertTrue(rep, det)
            self.assertTrue(all(d["mode"] == "report" and d["text"] for d in rep))
            if src:
                self.assertTrue(any(det in es["sources"][src].get("reported", ()) for es in late))
            self.assertEqual(late[-1]["detectors"][det], "report")
            if det == "yaw_rate.cusum":
                self.assertTrue(all(not es["sources"]["vehicle"].get("set_aside") for t, es, _ in rows))
                self.assertTrue(all(es["yaw_rate"]["valid"] for es in late))

    def test_a_detector_mode_must_be_gate_or_report(self):
        with self.assertRaises(ValueError):
            Integrator({"detectors": {"speed.cusum": "maybe"}})
        self.assertTrue(Integrator({"detectors": {"speed.cusum": "gate"}}).gates("speed.cusum"))
        self.assertFalse(Integrator().gates("speed.cusum"))

    def test_gyro_bias_is_voted_out_and_the_speed_is_kept(self):
        """Gyro, wheel difference and camera are three yaw rates: a gyro bias is isolated by vote, only the gyro's
        rows are set aside, the vehicle's speed stays in use."""
        # (0.04 rad/s: 2.3 degrees a second. The limits that are quiet on the reference car's healthy drives find
        # half of that only after 12 s; the fault table has the smaller one)
        rows = run([Fault("vehicle.gyro", "bias", T0, 130.0, b=0.04)])
        late = [(t, es, tru) for t, es, tru in rows if T0 + 6.0 <= t < 130.0]
        self.assertTrue(all("yaw_rate" in es["sources"]["vehicle"].get("set_aside", ()) for t, es, _ in late))
        self.assertTrue(all(es["sources"]["vehicle"]["state"] != "fault" for t, es, _ in rows))
        self.assertTrue(all("vehicle.speed" in es["speed"]["sources"] for t, es, _ in late))
        err = [abs(es["yaw_rate"]["value"] - tru["w"]) for t, es, tru in late]
        self.assertLess(sorted(err)[int(0.95 * len(err))], 0.015)

    def test_camera_pitch_step_is_reported_with_its_size(self):
        """The mount's one degree of freedom: a pitch step of 1.7 degrees is found within a second, followed, and
        reported with its size; the yaw offset does not move."""
        rows = run([Fault("vision.mount", "mount", T0, 1e9, pitch=0.03)])
        first = next(t for t, es, _ in rows if ("vision", "pitch_step") in findings(es))
        self.assertLess(first - T0, 1.5)
        es = rows[-1][1]
        cv = es["calibration"]["vision"]
        self.assertAlmostEqual(cv["last_pitch_step"]["rad"], 0.03, delta=0.004)
        self.assertAlmostEqual(cv["dpitch"]["value"], 0.08, delta=0.004)
        self.assertAlmostEqual(cv["dyaw"]["value"], -0.06, delta=0.003)
        self.assertTrue(any(d["kind"] == "mount_step" and "camera pitch changed by" in d["text"] for d in es["diagnostics"]))
        self.assertTrue(all(e["sources"]["vision"]["state"] != "fault" for t, e, _ in rows))

    def test_camera_out_of_range_is_not_a_mount_measurement(self):
        """A large pitch change takes the road model out of its range: the camera is faulted, the mount is
        `unknown` (not `ok`: the pose's direction reads "aligned" then), and the finding says what it looks like."""
        rows = run([Fault("vision.mount", "mount", T0, 1e9, pitch=0.14, collapse=True)])
        late = [es for t, es, _ in rows if t > T0 + 6.0 and es["speed"]["valid"] and es["speed"]["value"] > 5.0]
        self.assertTrue(all(es["sources"]["vision"]["state"] == "fault" for es in late))
        self.assertTrue(all(es["calibration"]["vision"]["state"] == "unknown" for es in late))
        self.assertTrue(any(("vision", "model_out_of_range") in findings(es) for es in late))

    def test_noise_burst_is_weighted_not_believed(self):
        rows = run([Fault("vehicle.gyro", "noise", T0, 130.0, sigma=0.03)])
        self.assertTrue(any(("vehicle", "noise") in findings(es) for t, es, _ in rows if T0 < t < T0 + 6))
        err = [abs(es["yaw_rate"]["value"] - tru["w"]) for t, es, tru in rows if T0 + 6.0 <= t < 130.0]
        self.assertLess(math.sqrt(sum(e * e for e in err) / len(err)), 0.012)

    def test_clock_epoch_is_an_event_not_a_fault(self):
        rows = run([Fault("clock", "epoch", T0, 1e9, back=600.0)])
        es = rows[-1][1]
        self.assertGreaterEqual(es["epoch"], 1)       # (a message of the old clock still in flight starts a second one)
        self.assertTrue(any(d["kind"] == "epoch" for d in es["diagnostics"]))
        self.assertEqual({g for g, s in es["sources"].items() if s["state"] == "fault"}, set())


if __name__ == "__main__":
    unittest.main()
