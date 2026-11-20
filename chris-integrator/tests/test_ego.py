"""The ego estimator against simulated truth: accuracy, honest uncertainty, calibration states, time handling
(docs/DESIGN.md sections 2-5, 10)."""
import math
import random
import unittest

import numpy as np

from .simrun import Run
from chris_integrator import AuxState, Integrator, Meas, Source
from chris_integrator.core import ego as E
from chris_integrator.sim import GnssSim, MapSim, VehicleSim, VisionSim, World, default_sensors

TRUE = dict(vehicle=dict(gyro_bias=0.004, gyro_scale=0.04, accel_bias=0.15),
            vision=dict(mount_yaw=-0.06, mount_pitch=0.05, speed_scale=0.03), gnss=dict(lag=2.2))
_BASE = {}


def base():
    """One nominal run (town, every sensor with a bias or a scale to find), shared by the tests that read it."""
    if "r" not in _BASE:
        _BASE["r"] = Run(sensors=default_sensors(**TRUE), seed=1, speed_scale=0.015)
    return _BASE["r"]


class Nominal(unittest.TestCase):
    def test_accuracy_and_honest_uncertainty(self):
        r = base()
        t0 = 30.0                                       # after the start-up transient (heading, scales unknown)
        self.assertLess(np.sqrt(np.mean(r.col("dv", t0) ** 2)), 0.08)
        self.assertLess(np.sqrt(np.mean(r.col("dw", t0) ** 2)), 0.008)
        self.assertLess(math.degrees(np.sqrt(np.mean(r.col("dpsi", t0) ** 2))), 1.0)
        self.assertLess(np.sqrt(np.mean(r.col("along", t0) ** 2 + r.col("cross", t0) ** 2)), 4.0)
        # NEES per scalar: 1 when the stated std is right; never optimistic by more than 2x in variance
        for d, s in (("dv", "sv"), ("dw", "sw"), ("dpsi", "spsi"), ("dth", "sth")):
            n = r.nees(d, s, t0)
            self.assertLess(n, 2.0, (d, n))
        pos = float(np.mean(r.col("nees_pos", t0)))     # 2 degrees of freedom: expected 2
        self.assertLess(pos, 4.0)
        cov = float(np.mean(np.abs(r.col("dv", t0)) <= 2.0 * r.col("sv", t0)))
        self.assertGreater(cov, 0.9)

    def test_calibration_states_find_the_truth(self):
        r = base()
        for src, name, true, tol in (("odometry", "speed_scale", 0.015, 0.006), ("vehicle", "b_g", 0.004, 0.001),
                                     ("vehicle", "s_g", 0.04, 0.02), ("vehicle", "s_wy", -0.02, 0.03),
                                     ("vehicle", "b_ax", 0.15, 0.08), ("vision", "tau", 0.125, 0.05),
                                     ("vision", "s", 0.03 - 0.0, 0.02), ("vision", "dyaw", -0.06, 0.002),
                                     ("vision", "dpitch", 0.05, 0.002), ("gnss", "tau", 2.2, 0.25)):
            val, std = r.cal(src, name)
            self.assertAlmostEqual(val, true, delta=tol, msg="%s.%s" % (src, name))
            self.assertLess(abs(val - true), 4.0 * std + 1e-4, "%s.%s: %g +- %g against %g" % (src, name, val, std, true))
        self.assertEqual(r.es["calibration"]["vision"]["state"], "offset")       # 3.4 degrees off the calibration in use
        self.assertEqual({k: v["state"] for k, v in r.es["sources"].items() if k not in ("gnss_speed", "signrows")},
                         {"vehicle": "ok", "wheels": "ok", "vision": "ok", "gnss": "ok", "gnss_track": "ok", "map": "ok"})

    def test_message_shape(self):
        es = base().es
        for q in ("speed", "accel", "yaw_rate", "heading", "grade"):
            self.assertEqual(set(es[q]) >= {"value", "std", "valid", "reason", "mode", "age_s", "sources"}, True, q)
        self.assertTrue(es["position"]["valid"])
        self.assertIn(es["position"]["mode"], ("fused", "coast"))
        self.assertEqual(es["api"], 3)
        self.assertGreater(es["odometer_m"], 1500.0)
        import json
        json.dumps(es)


class Time(unittest.TestCase):
    def test_late_and_out_of_order_inputs_give_the_same_estimate(self):
        """Receiver and map messages delivered up to 0.4 s later than they were end at the same state: the buffer
        re-runs from where a late measurement belongs. (Exact for the filter: test_core. Here the measurement
        models read the estimate at arrival for their delay and gates, so the match is close, not bitwise.)"""
        w = World(seed=3, scenario="town")
        sens = default_sensors(**TRUE)
        a = Run(world=w, sensors=sens)
        rng = random.Random(5)

        def scramble(msgs):
            road = [(t, m) for t, m in msgs if m["type"] == "road"]
            other = [(t + rng.uniform(0.0, 0.4), m) for t, m in msgs if m["type"] in ("gnss", "map")]
            rest = [(t, m) for t, m in msgs if m["type"] not in ("road", "gnss", "map")]
            return sorted(road + other + rest, key=lambda x: x[0])
        b = Run(world=World(seed=3, scenario="town"), sensors=default_sensors(**TRUE), order=scramble)
        xa, xb = a.I.ekf.latest()[1], b.I.ekf.latest()[1]
        self.assertLess(abs(xa[E.V] - xb[E.V]), 0.02)
        self.assertLess(math.hypot(xa[E.PE] - xb[E.PE], xa[E.PN] - xb[E.PN]), 3.0)
        self.assertLess(abs(xa[E.PSI] - xb[E.PSI]), 0.01)
        self.assertGreater(b.I.ekf.reruns, a.I.ekf.reruns)

    def test_epoch_keeps_calibration_and_carries_faults(self):
        r = base()
        I = r.I
        before = I.calibration_snapshot()
        I.carry_fault("vision", "test")
        n_epoch = I.epoch
        # the clock restarts (a boot): the same sensors again from an earlier time
        w = World(seed=4, scenario="town")
        w.t = w.t - 900.0
        w.t0 -= 900.0
        states = []
        for arr, m in list(w.messages(default_sensors(**TRUE)))[:700]:
            if m["type"] == "road":
                es = I.on_road(m)
                states.append((es["sources"]["vision"]["state"], es["sources"]["vision"].get("isolation")))
            else:
                I.on_message(m)
        self.assertEqual(I.epoch, n_epoch + 1)
        after = I.calibration_snapshot()
        self.assertAlmostEqual(after["vision.dyaw"][0], before["vision.dyaw"][0], delta=0.003)
        self.assertAlmostEqual(after["sv"][0], before["sv"][0], delta=0.004)
        # the fault is carried into the new epoch, and ends only on evidence of agreement (the car moves again)
        self.assertEqual(states[0], ("fault", "carried"))
        self.assertEqual(states[30], ("fault", "carried"))
        self.assertEqual(states[-1][0], "ok")
        _BASE.clear()

    def test_vehicle_silence_and_stale_resume(self):
        """The feed is silent for 7 s and resumes with stale samples: the estimate rides the camera and the
        stale samples are not believed."""
        sens = default_sensors(vehicle=dict(silences=[(20.0, 27.0), (100.0, 107.0)]))
        r = Run(sensors=sens, seed=6)
        gap = [x for x in r.rows if 21.0 <= x["t"] < 27.0 and "dv" in x]
        self.assertTrue(gap)
        self.assertLess(np.sqrt(np.mean([x["dv"] ** 2 for x in gap])), 1.0)
        self.assertLess(np.mean([(x["dv"] / x["sv"]) ** 2 for x in gap]), 3.0)        # and it says how unsure it is
        after = [abs(x["dv"]) for x in r.rows if 27.0 <= x["t"] < 28.0 and "dv" in x]
        self.assertLess(max(after), 1.2)
        self.assertGreaterEqual(r.es["sources"]["vehicle"]["warmups"], 2)


class Sources(unittest.TestCase):
    def test_camera_only_car(self):
        """No vehicle feed, no receiver: the estimate stands on the camera, says so, and has no position."""
        r = Run(sensors=[VisionSim()], seed=7)
        es = r.es
        self.assertTrue(es["speed"]["valid"])
        self.assertEqual(es["sources"]["vision"]["state"], "unchecked")
        self.assertEqual(es["sources"]["vehicle"]["state"], "absent")
        self.assertEqual((es["position"]["valid"], es["position"]["reason"]), (False, "no_absolute_source"))
        self.assertIsNotNone(es["position"]["local"])
        # honest: the camera's error is correlated over seconds, and the stated std covers it
        t0 = 10.0
        self.assertLess(r.nees("dv", "sv", t0), 2.5)
        self.assertGreater(float(np.mean(r.col("sv", t0))), 0.2)

    def test_receiver_outage_and_jump(self):
        sens = default_sensors(gnss=dict(outage=[(90.0, 150.0)], jump=(170.0, 176.0, 40.0, -30.0)), map=dict(hz=0.001))
        r = Run(sensors=sens, seed=8)
        out = [x for x in r.rows if 100.0 <= x["t"] < 150.0 and "hacc" in x]
        self.assertTrue(all(x["pos_mode"] == "coast" for x in out[20:]))
        self.assertGreater(out[-1]["hacc"], out[0]["hacc"])                      # dead reckoning grows
        self.assertLess(np.mean([x["nees_pos"] for x in out]), 6.0)
        jump = [math.hypot(x["de"], x["dn"]) for x in r.rows if 170.0 <= x["t"] < 180.0 and "de" in x]
        self.assertLess(max(jump), 25.0)                                         # a 50 m excursion moves it by less than half

    def test_wrong_road_is_bounded_and_seen(self):
        """The matcher sits on a parallel road 25 m away for 20 s: the position is pulled by a bounded amount and
        the map source's innovations say so."""
        seen = []
        sens = default_sensors(map=dict(wrong=[(110.0, 130.0, 25.0)]))
        r = Run(sensors=sens, seed=9, on_frame=lambda run, t, es, tr, ld: seen.append((t, es["sources"]["map"]["state"])))
        cross = [abs(x["cross"]) for x in r.rows if 110.0 <= x["t"] < 132.0 and "cross" in x]
        self.assertLess(max(cross), 12.0)
        self.assertIn("suspect", {s for t, s in seen if 112.0 <= t < 132.0})

    def test_a_source_added_at_runtime(self):
        """Adding a source is a description, a model and its calibration states; the core is not edited."""
        class Baro(Source):
            id = "baro"
            types = ("baro",)
            rank = 0
            stale_s = 2.0
            aux = (AuxState("bias", 0.0, 20.0, 1e-4, unit="m"),)

            def measure(self, msg, ctx):
                ib = self.ix["bias"]
                idx = np.array([E.Z, ib], dtype=np.intp)
                co = np.array([1.0, 1.0])
                return [Meas("alt", msg["ts_mono_ns"] / 1e9, msg["alt_m"], 0.5, lambda x: (x[E.Z] + x[ib], idx, co), "baro.alt")]
        w = World(seed=10)
        I = Integrator({"gnss_absolute": True})
        msgs = list(w.messages(default_sensors()))
        n0 = I.model.n
        k = 0
        for arr, m in msgs:
            if k == 800:
                I.register(Baro())
            k += 1
            if k > 800 and k % 5 == 0:
                tr = w.truth_at(arr)
                I.on_message({"type": "baro", "ts_mono_ns": int(arr * 1e9), "alt_m": tr["z"] + 12.0})
            es = I.on_road(m) if m["type"] == "road" else (I.on_message(m) or None)
            if es is not None:
                last = es
        self.assertEqual(I.model.n, n0 + 1)
        self.assertEqual(last["sources"]["baro"]["state"], "unchecked")
        self.assertIn("bias", last["calibration"]["baro"])
        self.assertLess(abs(last["grade"]["value"] - w.truth_at(last["ts_mono_ns"] / 1e9)["th"]), 0.02)


if __name__ == "__main__":
    unittest.main()
