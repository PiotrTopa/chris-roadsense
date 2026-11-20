"""The vehicle input store (io/vehicle_input.py), ported from the legacy mapd (lane L43-V): validation with mapd's
error reasons, quality, the description, the sensors, the `vehicle` topic, and the feeder's `reset` through the
mixer (the estimator forgets the position and keeps the calibration of the car's sensors; the map box drops its
state)."""
import os
import sys
import tempfile
import unittest

HERE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, HERE)
MAPPER = os.path.join(os.path.dirname(HERE), "chris-mapper")
sys.path.insert(0, MAPPER)

from chris_integrator.io.feed import Feed  # noqa: E402
from chris_integrator.io.vehicle_input import TYPES, VehicleInput, calibration_report, hello  # noqa: E402
from chris_integrator.mixer import MessageClock  # noqa: E402
from chris_integrator.stack import build  # noqa: E402

try:
    import importlib.util
    from chris_mapper import sim
    from chris_mapper.box import MapBox
    from chris_mapper.rsmap import Pack
    from chris_integrator.plugins.mapper import Mapper
    _spec = importlib.util.spec_from_file_location("synthpack_vi", os.path.join(MAPPER, "tests", "synthpack.py"))
    S = importlib.util.module_from_spec(_spec)
    _spec.loader.exec_module(S)
except (ImportError, OSError, FileNotFoundError):
    S = None

NS = 10 ** 9
T0 = 1000 * NS

# every key of the legacy mapd's `vehicle` message (mapd/roadsense_mapd/vehicle.py VehicleStore.message)
MAPD_VEHICLE_KEYS = {"type", "seq", "ts_mono_ns", "ts_real_ns", "api", "present", "motion", "controls", "body", "signals",
                     "sensors", "description", "calibration", "feeders"}
MAPD_CAL_SHAPE = {"yaw_rate": {"bias_rps", "bias_n", "scale", "scale_vs_wheels"}, "wheel_yaw": {"scale"},
                  "speed": {"scale"}, "gnss": {"lag_s", "speed_ratio", "n", "converged"}}


def ok(tc, vi, msg, now=T0):
    outs, err = vi.on_message(msg, now)
    tc.assertIsNone(err, msg)
    return outs


class Validation(unittest.TestCase):
    def test_error_reasons_as_mapd(self):
        """The reasons of the error replies, as mapd gave them (mapd/tests/test_vehicle.py test_validation and the
        position input's own checks)."""
        vi = VehicleInput()
        bad = [({"type": "controls", "turn_signal": "up"}, "turn_signal must be one of off left right both"),
               ({"type": "controls", "brake": 1}, "brake must be true or false"),
               ({"type": "controls", "accelerator": 1.5}, "accelerator out of range"),
               ({"type": "controls", "foo": 1}, "no known signal in the controls message"),
               ({"type": "body", "wipers": "fast"}, "wipers must be one of off intermittent low high"),
               ({"type": "body", "low_beam": True, "quality": {"low_beam": "perfect"}}, "quality of low_beam must be one of"),
               ({"type": "body", "low_beam": True, "quality": "good"}, "quality must be an object {signal: quality}"),
               ({"type": "motion"}, "missing field speed_mps"),
               ({"type": "motion", "speed_mps": -1.0}, "speed_mps out of range"),
               ({"type": "motion", "speed_mps": "fast"}, "speed_mps must be a finite number"),
               ({"type": "motion", "speed_mps": 3.0, "accel_long_mps2": 99.0}, "accel_long_mps2 out of range"),
               ({"type": "motion", "speed_mps": 3.0, "gear": "X"}, "gear must be one of P R N D B"),
               ({"type": "motion", "speed_mps": 3.0, "turn_signal": "up"}, "turn_signal must be one of off left right both"),
               ({"type": "gnss", "lat": 99.0, "lon": 21.0}, "lat out of range"),
               ({"type": "gnss", "lat": 52.0}, "missing field lon"),
               ({"type": "gnss", "fix": "4d", "lat": 52.0, "lon": 21.0}, "fix must be one of none 2d 3d dgps rtk dr"),
               ({"type": "gnss", "lat": 52.0, "lon": 21.0, "course_deg": 400.0}, "course_deg out of range"),
               ({"type": "vehicle", "wheelbase_m": 0.2}, "wheelbase_m out of range"),
               ({"type": "vehicle", "name": 5}, "name must be a string"),
               ({"type": "vehicle", "sensors": [{"kind": "radar"}]}, "every sensor needs an id (string)"),
               ({"type": "vehicle", "sensors": [{"id": "r", "kind": "sonar"}]}, "sensor kind must be one of"),
               ({"type": "vehicle", "camera": {"z_m": 9.0}}, "camera.z_m out of range"),
               ({"type": "objects", "sensor": "r", "objects": [{"azimuth_deg": 1.0}]}, "objects[0] needs range_m or x_m"),
               ({"type": "objects", "objects": []}, "objects needs a sensor id (string)"),
               ({"type": "objects", "sensor": "r", "objects": {}}, "objects must be a list"),
               ({"type": "objects", "sensor": "r", "objects": [], "tracked": 1}, "tracked must be true or false"),
               ({"type": "objects", "sensor": "r", "objects": [{"range_m": 5.0, "class": "tank"}]}, "objects[0].class must be one of"),
               ({"type": "objects", "sensor": "r", "objects": [{"range_m": 5000.0}]}, "objects[0].range_m out of range"),
               ({"type": "reset", "reason": 5}, "reason must be a string"),
               ({"type": "controls", "brake": True, "age_ms": -5}, "age_ms must be a number >= 0"),
               ({"type": "controls", "brake": True, "age_ms": 60000}, "age_ms: the sample is 60.0 s old"),
               ({"type": "controls", "brake": True, "ts_mono_ns": 1.5}, "ts_mono_ns must be an integer"),
               ({"type": "controls", "brake": True, "ts_mono_ns": T0 - 6 * NS}, "ts_mono_ns is 6.0 s old (CLOCK_MONOTONIC of this host expected)"),
               ({"type": "controls", "brake": True, "ts_mono_ns": T0 + NS}, "ts_mono_ns is in the future"),
               ({"type": "wipers"}, "unknown type 'wipers'"),
               ([1, 2], "not a JSON object")]
        for msg, why in bad:
            outs, err = vi.on_message(msg, T0)
            self.assertIsNotNone(err, msg)
            self.assertTrue(err.startswith(why), (msg, err))
            self.assertEqual(outs, [], msg)                                   # a rejected message produces nothing
        self.assertEqual(vi.counts["errors"], len(bad))
        self.assertEqual(vi.store.sig, {})                                    # ... and stores nothing
        # unknown fields are ignored, a null signal is "not sent"
        ok(self, vi, {"type": "controls", "brake": False, "turn_signal": None, "seat_heater": 3})
        self.assertIsNone(vi.vehicle_message(T0)["controls"]["turn_signal"])
        self.assertEqual(ok(self, vi, {"type": "hello", "api": 1}), [])

    def test_hello(self):
        h = hello(5)
        self.assertEqual((h["type"], h["api"], h["minor"], h["server"], h["clock"], h["now_mono_ns"]),
                         ("hello", 1, 1, "chris-integrator", "CLOCK_MONOTONIC", 5))
        self.assertEqual(h["accepts"], ["gnss", "motion", "reset", "controls", "body", "vehicle", "objects"])
        self.assertEqual(list(TYPES), h["accepts"])
        self.assertIn("accel_long_mps2", h["signals"]["motion"])
        self.assertIn("turn_signal", h["signals"]["motion"])
        self.assertIn("cruise_set_speed_mps", h["signals"]["controls"])
        self.assertIn("outside_temp_c", h["signals"]["body"])


class Store(unittest.TestCase):
    def test_quality_reaches_the_estimator_only_as_its_meaning(self):
        """`unverified` / `invalid` signals are not handed to the estimator (its sources do not read quality tags);
        the tags are carried in the `vehicle` topic. The yaw rate in use falls back to the wheels, then the steering."""
        vi = VehicleInput({"vehicle": {"track_m": 1.6}})
        ego = ok(self, vi, {"type": "motion", "speed_mps": 6.0, "yaw_rate_rps": 0.5, "wheel_speeds_mps": [6.0, 6.0, 5.9, 6.1],
                            "accel_long_mps2": 0.4, "quality": {"yaw_rate_rps": "unverified", "accel_long_mps2": "invalid"},
                            "source": "can"})[0]
        self.assertEqual((ego["type"], ego["yaw_source"], ego["tag"], ego["source"]), ("ego", "wheel_speeds", "can", "vehicle"))
        self.assertAlmostEqual(ego["yaw_rate_rps"], 0.2 / 1.6, places=5)      # the configured track
        self.assertNotIn("accel_long_mps2", ego)
        v = vi.vehicle_message(T0)
        self.assertEqual(v["signals"]["yaw_rate_rps"], {"age_s": 0.0, "quality": "unverified", "source": "can"})
        self.assertEqual(v["motion"]["yaw_rate_rps"], ego["yaw_rate_rps"])     # the yaw rate in use ...
        self.assertEqual(v["motion"]["yaw_source"], "wheel_speeds")
        self.assertIsNone(v["motion"]["accel_long_mps2"])                      # an invalid signal is absent
        self.assertEqual(v["signals"]["accel_long_mps2"]["quality"], "invalid")
        # usable gyro: passed as read (the estimator learns its bias and scale)
        ego = ok(self, vi, {"type": "motion", "speed_mps": 0.0, "wheel_speeds_mps": [0, 0, 0, 0], "yaw_rate_rps": 0.003,
                            "quality": {"yaw_rate_rps": "uncalibrated"}}, T0 + NS // 10)[0]
        self.assertEqual((ego["yaw_source"], ego["yaw_rate_rps"], ego["standstill"]), ("yaw_rate", 0.003, True))
        # steering as the last resort; zero while standing; reverse makes the speed negative
        ego = ok(self, vi, {"type": "motion", "speed_mps": 2.0, "steering_wheel_deg": 90.0, "gear": "R"}, T0 + 2 * NS // 10)[0]
        self.assertEqual((ego["yaw_source"], ego["speed_mps"], ego["reverse"]), ("steering", -2.0, True))
        self.assertLess(ego["yaw_rate_rps"], 0.0)
        ego = ok(self, vi, {"type": "motion", "speed_mps": 0.0, "steering_wheel_deg": 90.0}, T0 + 3 * NS // 10)[0]
        self.assertEqual((ego["yaw_rate_rps"], ego["standstill"]), (0.0, True))
        # a message without a speed is stored and joins the next one with a speed within 0.3 s; never later
        self.assertEqual(ok(self, vi, {"type": "motion", "yaw_rate_rps": 0.1}, T0 + 4 * NS // 10), [])
        ego = ok(self, vi, {"type": "motion", "speed_mps": 5.0}, T0 + 5 * NS // 10)[0]
        self.assertEqual((ego["yaw_source"], ego["yaw_rate_rps"]), ("yaw_rate", 0.1))
        ego = ok(self, vi, {"type": "motion", "speed_mps": 5.0}, T0 + 9 * NS // 10)[0]
        self.assertIsNone(ego["yaw_source"])
        # an unverified speed moves nothing
        self.assertEqual([m["type"] for m in ok(self, vi, {"type": "motion", "speed_mps": 5.0, "quality": {"*": "unverified"}},
                                                T0 + 10 * NS // 10)], [])

    def test_signals_table_description_and_expiry(self):
        vi = VehicleInput({"vehicle": {"name": "Configured car", "wheelbase_m": 2.5}})
        v = vi.vehicle_message(T0)
        self.assertEqual(set(v) - {"input"}, MAPD_VEHICLE_KEYS)                # the legacy topic's keys, plus `input`
        self.assertEqual((v["present"], v["feeders"], v["sensors"], v["signals"]), (False, [], [], {}))
        self.assertEqual((v["description"]["name"], v["description"]["wheelbase_m"], v["description"]["track_m"]),
                         ("Configured car", 2.5, 1.48))
        self.assertEqual(v["description"]["sources"], {"name": "config", "wheelbase_m": "config", "track_m": "default",
                                                       "steering_ratio": "default"})
        ok(self, vi, {"type": "vehicle", "name": "Fed car", "track_m": 1.6})
        ok(self, vi, {"type": "controls", "brake": True, "accelerator": 0.2, "quality": {"accelerator": "unverified"},
                      "source": "can"}, T0)
        ok(self, vi, {"type": "body", "low_beam": True, "wipers": "low", "outside_temp_c": -3.5}, T0)
        v = vi.vehicle_message(T0 + NS // 2)
        self.assertEqual((v["present"], v["controls"]["brake"], v["controls"]["accelerator"], v["controls"]["cruise"]),
                         (True, True, 0.2, None))                             # unverified: shown
        self.assertEqual((v["body"]["low_beam"], v["body"]["wipers"], v["body"]["outside_temp_c"], v["body"]["rain"]),
                         (True, "low", -3.5, None))
        self.assertEqual(v["signals"]["brake"], {"age_s": 0.5, "quality": "good", "source": "can"})
        self.assertEqual(v["signals"]["accelerator"]["quality"], "unverified")
        self.assertNotIn("parking_brake", v["signals"])                       # never sent
        self.assertIsNone(v["motion"]["speed_mps"])                           # no motion message yet
        self.assertEqual((v["description"]["name"], v["description"]["sources"]["track_m"],
                          v["description"]["wheelbase_m"]), ("Fed car", "feeder", 2.5))
        # signals expire: controls after 3 s, body after 15 s
        v = vi.vehicle_message(T0 + 4 * NS)
        self.assertEqual((v["controls"]["brake"], v["body"]["low_beam"], v["present"]), (None, True, True))
        self.assertEqual(v["signals"]["brake"]["age_s"], 4.0)
        v = vi.vehicle_message(T0 + 16 * NS)
        self.assertEqual((v["body"]["low_beam"], v["present"]), (None, False))

    def test_topic_rate_and_feeders(self):
        vi = VehicleInput()
        self.assertEqual(vi.tick(T0)["type"], "vehicle")                       # present: false without a feeder
        self.assertIsNone(vi.tick(T0 + NS // 10))
        self.assertEqual(vi.tick(T0 + NS // 2)["seq"], 2)                     # twice a second
        outs = ok(self, vi, {"type": "controls", "brake": True}, T0 + 6 * NS // 10)
        self.assertEqual([m["type"] for m in outs], ["vehicle"])              # a switch change goes out at once
        self.assertIsNone(vi.tick(T0 + 61 * NS // 100))
        outs = ok(self, vi, {"type": "controls", "brake": True}, T0 + 7 * NS // 10)
        self.assertEqual(outs, [])                                            # no change, not due
        vi.connect("c", T0)
        vi.hello("c", {"name": "my-car", "provides": {"motion": ["speed_mps"]}}, T0)
        vi.on_message({"type": "motion", "speed_mps": 1.0}, T0 + 8 * NS // 10, key="c")
        v = vi.vehicle_message(T0 + NS)
        self.assertEqual(v["feeders"], [{"name": "my-car", "provides": {"motion": ["speed_mps"]}, "connected_s": 1.0,
                                         "messages": 1}])
        vi.disconnect("c")
        self.assertEqual(vi.vehicle_message(T0 + NS)["feeders"], [])

    def test_sensors_and_objects(self):
        vi = VehicleInput()
        ok(self, vi, {"type": "vehicle", "camera": {"x_m": 1.9, "y_m": 0.0, "z_m": 1.2},
                      "sensors": [{"id": "radar0", "kind": "radar", "x_m": 3.7, "y_m": 0.1, "yaw_deg": 2.0}]})
        o = ok(self, vi, {"type": "objects", "sensor": "radar0", "tracked": True, "source": "can",
                          "objects": [{"id": 3, "range_m": 20.0, "azimuth_deg": 0.0, "range_rate_mps": -1.0, "junk": 1}]})[0]
        self.assertEqual((o["type"], o["sensor"], o["kind"], o["frame"], o["tracked"], o["mounted"], o["source"]),
                         ("objects", "radar0", "radar", "sensor", True, True, "can"))
        self.assertEqual(o["objects"], [{"id": 3, "range_m": 20.0, "azimuth_deg": 0.0, "range_rate_mps": -1.0}])
        self.assertEqual(o["mount"], {"x_m": 1.8, "y_m": 0.1, "z_m": -1.2, "yaw_deg": 2.0})   # relative to the camera
        o2 = ok(self, vi, {"type": "objects", "sensor": "sonar", "kind": "ultrasonic", "objects": [{"range_m": 0.8}]},
                T0 + NS // 20)[0]
        self.assertEqual((o2["mounted"], "mount" in o2), (False, False))
        ok(self, vi, {"type": "objects", "sensor": "radar0", "objects": []}, T0 + NS // 10)
        v = vi.vehicle_message(T0 + NS // 10)
        self.assertEqual([(s["id"], s["kind"], s["objects"], s["scans"], s["mounted"]) for s in v["sensors"]],
                         [("radar0", "radar", 0, 2, True), ("sonar", "ultrasonic", 1, 1, False)])
        self.assertEqual(v["sensors"][0]["hz"], 10.0)

    def test_reset_and_synthetic_gnss(self):
        vi = VehicleInput()
        outs = ok(self, vi, {"type": "gnss", "lat": 52.0, "lon": 21.0, "hacc_m": 4.0, "source": "synthetic"})
        self.assertEqual([m["type"] for m in outs], ["gnss", "vehicle"])
        g = outs[0]
        self.assertEqual((g["fix"], g["ts_mono_ns"], g["arrival_mono_ns"], g["source"]), ("3d", T0, T0, "synthetic"))
        r = ok(self, vi, {"type": "reset", "reason": "drive ended"}, T0 + NS)[0]
        self.assertEqual((r["type"], r["reason"], r["auto"], r["ts_mono_ns"]), ("reset", "drive ended", False, T0 + NS))
        self.assertEqual(vi.vehicle_message(T0 + NS)["input"]["last_reset"]["reason"], "drive ended")
        # mapd's rule: a synthetic session's position never lives on as the real receiver's (and back)
        ok(self, vi, {"type": "gnss", "lat": 52.0, "lon": 21.0, "source": "synthetic"}, T0 + 2 * NS)
        outs = ok(self, vi, {"type": "gnss", "fix": "none", "source": "headunit"}, T0 + 3 * NS)
        self.assertEqual([m["type"] for m in outs][:2], ["reset", "gnss"])
        self.assertEqual((outs[0]["auto"], outs[0]["reason"]), (True, "gnss source changed from synthetic to real"))
        self.assertEqual(vi.resets, 2)
        # a fix worse than 100 m is accepted and not passed on
        self.assertEqual([m["type"] for m in ok(self, vi, {"type": "gnss", "lat": 52.0, "lon": 21.0, "hacc_m": 150.0,
                                                           "source": "headunit"}, T0 + 4 * NS)], ["vehicle"])
        self.assertEqual(vi.counts["gnss_ignored"], 1)

    def test_calibration_report_shape(self):
        rep = calibration_report(None)
        for k, keys in MAPD_CAL_SHAPE.items():
            self.assertTrue(keys <= set(rep[k]), k)
        rep = calibration_report({"odometry": {"speed_scale": {"value": 0.02, "std": 0.002, "known": True}},
                                  "vehicle": {"b_g": {"value": -0.0003, "std": 0.0001, "known": True},
                                              "s_g": {"value": -0.03, "std": 0.01, "known": False}},
                                  "gnss": {"tau": {"value": 0.8, "std": 0.1, "known": True}}})
        self.assertEqual(rep["speed"]["scale"]["value"], round(1 / 1.02, 4))   # true speed = value x the fed one
        self.assertTrue(rep["speed"]["scale"]["converged"])
        self.assertEqual((rep["yaw_rate"]["bias_rps"], rep["yaw_rate"]["scale"]["value"], rep["yaw_rate"]["scale"]["converged"]),
                         (-0.0003, round(1 / 0.97, 4), False))
        self.assertEqual((rep["gnss"]["lag_s"], rep["gnss"]["converged"], rep["source"]), (0.8, True, "estimator"))


@unittest.skipUnless(S is not None, "needs chris-mapper next to this tree")
class ResetInTheStack(unittest.TestCase):
    """A synthetic drive through the stack, then the feeder's `reset`: the estimator has no position and the map box
    no state (`no_source`), the calibration learned from the car's sensors is kept, motion goes on; a real fix
    brings the position back."""

    @classmethod
    def setUpClass(cls):
        cls.tmp = tempfile.mkdtemp(prefix="chris-vi-")
        cls.path = os.path.join(cls.tmp, "synth.rsmap")
        S.build(cls.path)

    def test_reset_forgets_the_position_and_keeps_the_calibration(self):
        pack = Pack(self.path)
        st = build(mapper=Mapper(MapBox({"pack": self.path})), clock=MessageClock())
        feed = Feed()
        last = {}
        st.mixer.add_output("t", lambda e: last.__setitem__(e.stream, e.msg), streams=("ego_state", "map", "vehicle"))
        drive = sim.Drive(pack, S.main_road_path(pack, 20.0, 3000.0))
        t_end = None
        motions = []
        for ts, m, truth in sim.feed(drive, t0_ns=10 ** 12, seed=3, source="synthetic"):
            if m is None:
                continue
            if t_end is None:
                t_end = ts + 90 * NS
            if ts > t_end:
                break
            if m["type"] == "motion":
                motions.append((ts, m))
            for im in feed.convert(m, now_ns=ts):
                st.deliver(im)
        es, mp = last["ego_state"], last["map"]
        self.assertTrue(es["position"]["valid"])
        self.assertEqual((mp["state"], mp["input"]["synthetic"], mp["input"]["gnss"]["source"]), ("good", True, "synthetic"))
        cal0 = es["calibration"]
        self.assertGreater(cal0["vehicle"]["b_g"]["value"], 0.001)           # the gyro bias learned (0.0026 rad/s fed)
        integ = st.mixer.plugin("estimator").integ
        frame0 = integ.frame_id
        snap0 = integ.calibration_snapshot()
        ts0 = ts
        for im in feed.convert({"type": "reset", "reason": "synthetic session ended"}, now_ns=ts0):
            st.deliver(im)
        self.assertFalse(integ.anchored)
        snap1 = integ.calibration_snapshot()
        absolute = ("gnss.", "map.")                                         # the receiver's and the map's states
        self.assertEqual({k: v for k, v in snap1.items() if not k.startswith(absolute)},
                         {k: v for k, v in snap0.items() if not k.startswith(absolute)})     # kept exactly
        self.assertNotEqual(snap1["gnss.tau"], snap0["gnss.tau"])
        self.assertGreater(integ.frame_id, frame0)
        # at once, without waiting for the next motion sample (no camera here)
        self.assertEqual((last["ego_state"]["position"]["valid"], last["map"]["state"], last["map"]["input"]["synthetic"]),
                         (False, "no_source", False))
        # motion goes on (the drive's own motion, without fixes)
        k = 0
        while k < 40:
            ts = ts0 + (k + 1) * NS // 20
            for im in feed.convert(dict(motions[-1][1], ts_mono_ns=ts), now_ns=ts):
                st.deliver(im)
            k += 1
        es, mp = last["ego_state"], last["map"]
        self.assertEqual((es["position"]["valid"], es["position"]["reason"]), (False, "no_absolute_source"))
        self.assertTrue(es["speed"]["valid"])                                  # motion is untouched
        self.assertEqual((mp["state"], mp["available"], mp["reason"]), ("no_source", False, "no_position"))
        self.assertEqual((mp["input"]["synthetic"], mp["input"]["gnss"]["source"]), (False, None))
        self.assertEqual((mp["input"]["resets"], mp["input"]["last_reset"]["reason"]), (1, "synthetic session ended"))
        cal1 = es["calibration"]
        for k_ in ("value", "std"):
            self.assertAlmostEqual(cal1["odometry"]["speed_scale"][k_], cal0["odometry"]["speed_scale"][k_], places=3)
            self.assertAlmostEqual(cal1["vehicle"]["b_g"][k_], cal0["vehicle"]["b_g"][k_], places=3)
        self.assertEqual(cal1["gnss"]["tau"]["value"], 2.0)                   # the receiver's delay: back to its prior
        v = last["vehicle"]
        self.assertEqual((v["input"]["resets"], v["input"]["last_reset"]["reason"]), (1, "synthetic session ended"))
        self.assertEqual(st.mixer.plugin("mapper").health()["position"], "no_source")
        self.assertEqual(sum(s.errors for s in st.mixer.slots.values()), 0)
        # another receiver: the position comes back (mapd's auto reset on a synthetic -> real change does not fire:
        # the explicit reset already forgot the synthetic session)
        for j in range(60):
            ts += NS // 10
            for im in feed.convert({"type": "motion", "ts_mono_ns": ts, "speed_mps": 13.0, "yaw_rate_rps": 0.0,
                                    "gear": "D", "source": "e2e"}, now_ns=ts):
                st.deliver(im)
            if j % 5 == 4:
                la, lo = S.ll(2700.0 + j * 1.3, 0.0)
                for im in feed.convert({"type": "gnss", "ts_mono_ns": ts, "lat": la, "lon": lo, "hacc_m": 4.0,
                                        "speed_mps": 13.0, "course_deg": 90.0, "source": "e2e"}, now_ns=ts):
                    st.deliver(im)
        self.assertEqual(feed.vi.resets, 1)
        es = last["ego_state"]
        self.assertTrue(es["position"]["valid"], es["position"])
        self.assertEqual(es["sources"]["gnss"]["receiver"], "e2e")

if __name__ == "__main__":
    unittest.main()
