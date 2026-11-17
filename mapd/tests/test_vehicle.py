"""Vehicle input API 1.1 (docs/VEHICLE_API.md): the messages beyond v1, quality, partial motion messages, the
`vehicle` and `objects` relay messages, calibration, and that a v1 feeder gets what it got before.

    python3 -m unittest discover -s mapd/tests -t mapd -v
"""
import json
import math
import os
import shutil
import subprocess
import sys
import tempfile
import time
import unittest

from roadsense_mapd import geo
from roadsense_mapd.engine import Engine
from roadsense_mapd.vehicle import Calibration, load_calibration, save_calibration

from . import synthpack as SP
from .test_daemon import Sub

REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, os.path.join(REPO, "tools"))

from vehicle_client import VehicleFeed      # noqa: E402

T0 = 10 ** 12
NS = 10 ** 9
MS = 10 ** 6


def ok(tc, eng, msg, now):
    tc.assertIsNone(eng.on_input(msg, now), msg)


class Messages(unittest.TestCase):
    def test_controls_and_body_are_stored_with_age_quality_and_source(self):
        eng = Engine()
        ok(self, eng, {"type": "controls", "ts_mono_ns": T0, "turn_signal": "left", "brake": True, "accelerator": 0.0,
                       "cruise": "standby", "cruise_set_speed_mps": 25.0, "source": "can",
                       "quality": {"accelerator": "unverified"}}, T0)
        ok(self, eng, {"type": "body", "ts_mono_ns": T0, "low_beam": True, "high_beam": False, "wipers": "low",
                       "outside_temp_c": -3.5, "source": "can"}, T0)
        v = eng.vehicle_message(T0 + 500 * MS, 0)
        self.assertEqual((v["type"], v["api"], v["present"]), ("vehicle", 1, True))
        self.assertEqual((v["controls"]["turn_signal"], v["controls"]["brake"], v["controls"]["cruise"],
                          v["controls"]["cruise_set_speed_mps"], v["controls"]["parking_brake"]),
                         ("left", True, "standby", 25.0, None))
        self.assertEqual((v["body"]["low_beam"], v["body"]["wipers"], v["body"]["outside_temp_c"], v["body"]["rain"]),
                         (True, "low", -3.5, None))
        self.assertEqual(v["signals"]["brake"], {"age_s": 0.5, "quality": "good", "source": "can"})
        self.assertEqual(v["signals"]["accelerator"]["quality"], "unverified")
        self.assertNotIn("parking_brake", v["signals"])               # never sent
        self.assertIsNone(v["motion"]["speed_mps"])                   # no motion message yet
        # signals expire: controls after 3 s, body after 15 s
        v = eng.vehicle_message(T0 + 4 * NS, 0)
        self.assertEqual((v["controls"]["brake"], v["body"]["low_beam"], v["present"]), (None, True, True))
        self.assertEqual(v["signals"]["brake"]["age_s"], 4.0)
        v = eng.vehicle_message(T0 + 16 * NS, 0)
        self.assertEqual((v["body"]["low_beam"], v["present"]), (None, False))

    def test_validation(self):
        eng = Engine()
        bad = [({"type": "controls", "turn_signal": "up"}, "turn_signal must be one of"),
               ({"type": "controls", "brake": 1}, "brake must be true or false"),
               ({"type": "controls", "accelerator": 1.5}, "accelerator out of range"),
               ({"type": "controls", "foo": 1}, "no known signal"),
               ({"type": "body", "wipers": "fast"}, "wipers must be one of"),
               ({"type": "body", "low_beam": True, "quality": {"low_beam": "perfect"}}, "quality of low_beam"),
               ({"type": "motion"}, "missing field speed_mps"),
               ({"type": "motion", "speed_mps": 3.0, "accel_long_mps2": 99.0}, "accel_long_mps2 out of range"),
               ({"type": "vehicle", "wheelbase_m": 0.2}, "wheelbase_m out of range"),
               ({"type": "vehicle", "sensors": [{"kind": "radar"}]}, "needs an id"),
               ({"type": "objects", "sensor": "r", "objects": [{"azimuth_deg": 1.0}]}, "needs range_m or x_m"),
               ({"type": "objects", "objects": []}, "needs a sensor id"),
               ({"type": "objects", "sensor": "r", "objects": {}}, "objects must be a list"),
               ({"type": "controls", "brake": True, "age_ms": -5}, "age_ms must be"),
               ({"type": "controls", "brake": True, "age_ms": 60000}, "6"),
               ({"type": "wipers"}, "unknown type")]
        for msg, why in bad:
            err = eng.on_input(msg, T0)
            self.assertIsNotNone(err, msg)
            self.assertIn(why, err)
        self.assertEqual(eng.counts["errors"], len(bad))
        # unknown fields are ignored, a null signal is "not sent"
        ok(self, eng, {"type": "controls", "brake": False, "turn_signal": None, "seat_heater": 3}, T0)
        self.assertEqual(eng.vehicle_message(T0, 0)["controls"]["turn_signal"], None)

    def test_age_ms_instead_of_a_timestamp(self):
        eng = Engine()
        ok(self, eng, {"type": "body", "outside_temp_c": 4.0, "age_ms": 800}, T0)
        self.assertEqual(eng.vehicle_message(T0, 0)["signals"]["outside_temp_c"]["age_s"], 0.8)
        ok(self, eng, {"type": "gnss", "lat": 52.0, "lon": 21.0, "hacc_m": 4.0, "age_ms": 300}, T0)
        self.assertEqual(eng.step(T0)["input"]["gnss"]["age_s"], 0.3)

    def test_turn_signal_from_controls_reaches_ego(self):
        eng = Engine()
        ok(self, eng, {"type": "controls", "ts_mono_ns": T0, "turn_signal": "right"}, T0)
        ok(self, eng, {"type": "motion", "ts_mono_ns": T0 + 100 * MS, "speed_mps": 8.0}, T0 + 100 * MS)
        self.assertEqual(eng.ego["turn_signal"], "right")
        # v1 place (in `motion`) still works and is the same signal
        ok(self, eng, {"type": "motion", "ts_mono_ns": T0 + 200 * MS, "speed_mps": 8.0, "turn_signal": "off"}, T0 + 200 * MS)
        self.assertEqual((eng.ego["turn_signal"], eng.vehicle_message(T0 + 200 * MS, 0)["controls"]["turn_signal"]),
                         ("off", "off"))
        # a stale turn signal is unknown, not "off"
        t = T0 + 5 * NS
        ok(self, eng, {"type": "motion", "ts_mono_ns": t, "speed_mps": 8.0}, t)
        self.assertIsNone(eng.ego["turn_signal"])


class Motion(unittest.TestCase):
    def test_yaw_source_order_and_quality(self):
        eng = Engine()
        base = {"type": "motion", "speed_mps": 10.0, "wheel_speeds_mps": [10.0, 10.0, 9.9, 10.1], "steering_wheel_deg": 30.0}
        ok(self, eng, dict(base, ts_mono_ns=T0, yaw_rate_rps=0.2), T0)
        self.assertEqual((eng.ego["yaw_source"], eng.ego["yaw_rate_rps"]), ("yaw_rate", 0.2))
        # an unverified gyroscope is reported but not used: the wheels decide
        ok(self, eng, dict(base, ts_mono_ns=T0 + 1, yaw_rate_rps=0.2, quality={"yaw_rate_rps": "unverified"}), T0 + 1)
        self.assertEqual(eng.ego["yaw_source"], "wheel_speeds")
        self.assertAlmostEqual(eng.ego["yaw_rate_rps"], 0.2 / 1.48, places=4)
        v = eng.vehicle_message(T0 + 1, 0)
        self.assertEqual((v["motion"]["yaw_rate_rps"], v["motion"]["yaw_source"]), (eng.ego["yaw_rate_rps"], "wheel_speeds"))
        self.assertEqual(v["signals"]["yaw_rate_rps"]["quality"], "unverified")
        # everything invalid but the steering angle
        ok(self, eng, dict(base, ts_mono_ns=T0 + 2, quality={"wheel_speeds_mps": "invalid"}), T0 + 2)
        self.assertEqual(eng.ego["yaw_source"], "steering")
        self.assertIsNone(eng.ego["wheel_speeds_mps"])
        ok(self, eng, {"type": "motion", "ts_mono_ns": T0 + 3, "speed_mps": 10.0}, T0 + 3)
        self.assertEqual((eng.ego["yaw_source"], eng.ego["yaw_rate_rps"]), (None, None))
        # a speed that is not usable does not move the filter and makes no ego message
        seq = eng.ego["seq"]
        ok(self, eng, {"type": "motion", "ts_mono_ns": T0 + 4, "speed_mps": 10.0, "quality": {"*": "unverified"}}, T0 + 4)
        self.assertEqual(eng.ego["seq"], seq)

    def test_partial_messages_merge_but_complete_ones_stand_alone(self):
        """A gyroscope on another bus sends `motion` without a speed; the next message with a speed uses it."""
        eng = Engine()
        ok(self, eng, {"type": "motion", "ts_mono_ns": T0, "yaw_rate_rps": 0.11, "accel_lat_mps2": 1.1, "source": "imu"}, T0)
        self.assertIsNone(eng.ego)
        ok(self, eng, {"type": "motion", "ts_mono_ns": T0 + 50 * MS, "speed_mps": 10.0, "source": "obd"}, T0 + 50 * MS)
        self.assertEqual((eng.ego["yaw_source"], eng.ego["yaw_rate_rps"], eng.ego["accel_lat_mps2"]), ("yaw_rate", 0.11, 1.1))
        # too old to pair (pair_s 0.3)
        ok(self, eng, {"type": "motion", "ts_mono_ns": T0 + 500 * MS, "speed_mps": 10.0, "source": "obd"}, T0 + 500 * MS)
        self.assertIsNone(eng.ego["yaw_rate_rps"])
        # what an earlier complete message carried is never reused (v1 behaviour)
        ok(self, eng, {"type": "motion", "ts_mono_ns": T0 + 600 * MS, "speed_mps": 10.0, "yaw_rate_rps": 0.3}, T0 + 600 * MS)
        ok(self, eng, {"type": "motion", "ts_mono_ns": T0 + 650 * MS, "speed_mps": 10.0}, T0 + 650 * MS)
        self.assertIsNone(eng.ego["yaw_rate_rps"])

    def test_standing_car_does_not_turn_and_the_gyro_bias_is_learned(self):
        eng = Engine()
        t = T0
        for i in range(60):                                    # 3 s at standstill with a biased gyroscope
            t += 50 * MS
            ok(self, eng, {"type": "motion", "ts_mono_ns": t, "speed_mps": 0.0, "wheel_speeds_mps": [0, 0, 0, 0],
                           "yaw_rate_rps": 0.013, "source": "can"}, t)
            self.assertEqual(eng.ego["yaw_rate_rps"], 0.0)
        cal = eng.vehicle_message(t, 0)["calibration"]
        self.assertAlmostEqual(cal["yaw_rate"]["bias_rps"], 0.013, places=4)
        t += 50 * MS
        ok(self, eng, {"type": "motion", "ts_mono_ns": t, "speed_mps": 5.0, "wheel_speeds_mps": [5, 5, 5, 5],
                       "yaw_rate_rps": 0.113, "source": "can"}, t)
        self.assertAlmostEqual(eng.ego["yaw_rate_rps"], 0.1, places=3)

    def test_description_from_the_feeder_replaces_config_and_defaults(self):
        eng = Engine(None, {"track_m": 1.60, "vehicle": {"steering_ratio": 15.0}})
        d = eng.vehicle_message(T0, 0)["description"]
        self.assertEqual((d["track_m"], d["steering_ratio"], d["wheelbase_m"]), (1.60, 15.0, 2.70))
        self.assertEqual(d["sources"], {"track_m": "config", "steering_ratio": "config", "wheelbase_m": "default"})
        ok(self, eng, {"type": "vehicle", "name": "Test car", "track_m": 2.0, "wheelbase_m": 3.0,
                       "camera": {"x_m": 1.8, "y_m": 0.0, "z_m": 1.2}}, T0)
        d = eng.vehicle_message(T0, 0)["description"]
        self.assertEqual((d["name"], d["track_m"], d["sources"]["track_m"], d["sources"]["steering_ratio"]),
                         ("Test car", 2.0, "feeder", "config"))
        ok(self, eng, {"type": "motion", "ts_mono_ns": T0, "speed_mps": 10.0, "wheel_speeds_mps": [10, 10, 9.9, 10.1]}, T0)
        self.assertAlmostEqual(eng.ego["yaw_rate_rps"], 0.2 / 2.0, places=4)


class Objects(unittest.TestCase):
    def test_scan_is_relayed_in_the_camera_frame_when_mounts_are_known(self):
        eng = Engine()
        scan = {"type": "objects", "ts_mono_ns": T0, "sensor": "front_radar", "kind": "radar", "tracked": True,
                "source": "radar-x", "objects": [
                    {"id": 7, "range_m": 40.0, "azimuth_deg": 0.0, "range_rate_mps": -2.0, "rcs_dbsm": 10.0},
                    {"id": 8, "range_m": 20.0, "azimuth_deg": 90.0},                # straight to the left
                    {"x_m": 10.0, "y_m": -1.0, "vx_mps": -1.0, "vy_mps": 0.5}]}       # Cartesian: 1 m to the right
        ok(self, eng, scan, T0)
        o = eng.objects_out.pop()
        self.assertEqual((o["type"], o["sensor"], o["kind"], o["frame"], o["tracked"], o["source"]),
                         ("objects", "front_radar", "radar", "sensor", True, "radar-x"))
        self.assertEqual([(a["x_m"], a["y_m"]) for a in o["objects"]], [(40.0, -0.0), (0.0, -20.0), (10.0, 1.0)])
        # mounts known: radar 3.6 m ahead of the rear axle, 0.3 m to the left, camera 1.9 m ahead on the centre line
        ok(self, eng, {"type": "vehicle", "camera": {"x_m": 1.9, "y_m": 0.0, "z_m": 1.2},
                       "sensors": [{"id": "front_radar", "kind": "radar", "x_m": 3.6, "y_m": 0.3, "z_m": 0.5, "yaw_deg": 0.0}]}, T0)
        ok(self, eng, dict(scan, ts_mono_ns=T0 + 50 * MS), T0 + 50 * MS)
        o = eng.objects_out.pop()
        self.assertEqual(o["frame"], "camera")
        a, b, c = o["objects"]
        self.assertEqual((a["x_m"], a["y_m"], a["id"], a["range_rate_mps"], a["range_m"]), (41.7, -0.3, 7, -2.0, 40.0))
        self.assertEqual((b["x_m"], b["y_m"]), (1.7, -20.3))        # output y is positive to the RIGHT
        self.assertEqual((c["x_m"], c["y_m"], c["vx_mps"], c["vy_mps"]), (11.7, 0.7, -1.0, -0.5))
        v = eng.vehicle_message(T0 + 60 * MS, 0)
        self.assertEqual([(s["id"], s["kind"], s["objects"], s["mounted"], s["hz"]) for s in v["sensors"]],
                         [("front_radar", "radar", 3, True, 20.0)])
        # a sensor turned 90 degrees to the left: its boresight is the car's left
        ok(self, eng, {"type": "vehicle", "sensors": [{"id": "side", "kind": "ultrasonic", "x_m": 1.9, "y_m": 0.9, "yaw_deg": 90.0}]}, T0)
        ok(self, eng, {"type": "objects", "sensor": "side", "objects": [{"range_m": 1.5}]}, T0)
        o = eng.objects_out.pop()
        self.assertEqual((o["kind"], o["objects"][0]["x_m"], o["objects"][0]["y_m"]), ("ultrasonic", 0.0, -2.4))
        ok(self, eng, {"type": "objects", "sensor": "side", "objects": []}, T0)       # an empty scan is a statement
        self.assertEqual(eng.objects_out.pop()["objects"], [])


class Calibrating(unittest.TestCase):
    def drive(self, gyro_scale=1.0, speed_scale=1.0, with_speed=True, turns=16, quality=None, cfg=None, straight_s=8):
        """A square-ish drive: straights with 90-degree turns, GNSS at 1 Hz, motion at 20 Hz; the gyroscope
        reads true / gyro_scale, the speedometer true / speed_scale."""
        eng = Engine(None, cfg)
        t, lat, lon, hdg, v = T0, 52.0, 21.0, 0.0, 12.0        # hdg: radians counter-clockwise from east
        step = 0
        for seg in range(turns * 2):
            turning = seg % 2 == 1
            n = 100 if turning else straight_s * 20            # 5 s turn, 8 s straight
            yaw = (math.pi / 2) / 5.0 * (1 if (seg // 2) % 3 else -1) if turning else 0.0
            for _ in range(n):
                t += 50 * MS
                step += 1
                hdg += yaw * 0.05
                lat, lon = geo.offset(lat, lon, v * 0.05 * math.sin(hdg), v * 0.05 * math.cos(hdg))
                m = {"type": "motion", "ts_mono_ns": t, "speed_mps": v / speed_scale, "yaw_rate_rps": yaw / gyro_scale,
                     "wheel_speeds_mps": [v, v, v - yaw * 0.74, v + yaw * 0.74], "source": "can"}
                if quality:
                    m["quality"] = quality
                ok(self, eng, m, t)
                if step % 20 == 0:
                    g = {"type": "gnss", "ts_mono_ns": t, "lat": lat, "lon": lon, "hacc_m": 3.0,
                         "course_deg": math.degrees(math.pi / 2 - hdg) % 360.0}
                    if with_speed:
                        g["speed_mps"] = v
                    ok(self, eng, g, t)
                    eng.step(t)
        return eng, t

    def test_scales_against_gnss(self):
        eng, t = self.drive(gyro_scale=1.06, speed_scale=0.97)
        cal = eng.vehicle_message(t, 0)["calibration"]
        ys, ws, ss = cal["yaw_rate"]["scale"], cal["wheel_yaw"]["scale"], cal["speed"]["scale"]
        self.assertEqual((ys["ref"], ys["converged"], ss["converged"]), ("gnss_course", True, True))
        self.assertAlmostEqual(ys["value"], 1.06, delta=0.01)
        self.assertAlmostEqual(ws["value"], 1.0, delta=0.01)
        self.assertAlmostEqual(ss["value"], 0.97, delta=0.003)
        self.assertAlmostEqual(cal["yaw_rate"]["scale_vs_wheels"]["value"], 1.06, delta=0.005)
        self.assertGreaterEqual(ys["n"], 12)
        # declared good: the feeder vouches, nothing is rescaled
        self.assertAlmostEqual(abs(eng.motion["v"]), 12.0 / 0.97, places=3)

    def test_converged_scale_is_applied_only_to_uncalibrated_signals(self):
        q = {"yaw_rate_rps": "uncalibrated", "speed_mps": "uncalibrated"}
        eng, t = self.drive(gyro_scale=1.06, speed_scale=0.97, quality=q)
        t += 50 * MS
        ok(self, eng, {"type": "motion", "ts_mono_ns": t, "speed_mps": 10.0, "yaw_rate_rps": 0.1, "quality": q,
                       "source": "can"}, t)
        self.assertAlmostEqual(eng.ego["yaw_rate_rps"], 0.106, delta=0.001)
        self.assertAlmostEqual(eng.ego["speed_mps"], 9.7, delta=0.03)
        eng2, t = self.drive(gyro_scale=1.06, speed_scale=0.97, quality=q, cfg={"use_calibration": False})
        t += 50 * MS
        ok(self, eng2, {"type": "motion", "ts_mono_ns": t, "speed_mps": 10.0, "yaw_rate_rps": 0.1, "quality": q,
                        "source": "can"}, t)
        self.assertEqual((eng2.ego["yaw_rate_rps"], eng2.ego["speed_mps"]), (0.1, 10.0))

    def test_too_little_evidence_is_not_converged_and_not_applied(self):
        q = {"yaw_rate_rps": "uncalibrated"}
        eng, t = self.drive(gyro_scale=1.06, turns=4, quality=q)
        ys = eng.vehicle_message(t, 0)["calibration"]["yaw_rate"]["scale"]
        self.assertFalse(ys["converged"])
        t += 50 * MS
        ok(self, eng, {"type": "motion", "ts_mono_ns": t, "speed_mps": 10.0, "yaw_rate_rps": 0.1, "quality": q, "source": "can"}, t)
        self.assertEqual(eng.ego["yaw_rate_rps"], 0.1)

    def test_speed_scale_without_a_receiver_speed(self):
        eng, t = self.drive(speed_scale=0.95, with_speed=False, turns=15, straight_s=30)
        ss = eng.vehicle_message(t, 0)["calibration"]["speed"]["scale"]
        self.assertGreater(ss["n"], 20)
        self.assertAlmostEqual(ss["value"], 0.95, delta=0.01)

    def test_unverified_gyro_is_calibrated_but_not_used(self):
        eng, t = self.drive(gyro_scale=1.06, quality={"yaw_rate_rps": "unverified"})
        cal = eng.vehicle_message(t, 0)["calibration"]
        self.assertTrue(cal["yaw_rate"]["scale"]["converged"])
        self.assertAlmostEqual(cal["yaw_rate"]["scale"]["value"], 1.06, delta=0.01)
        self.assertEqual(eng.ego["yaw_source"], "wheel_speeds")

    def test_gnss_lag_is_found(self):
        """A receiver whose positions are 0.75 s late, on a drive that speeds up and slows down."""
        for lag, want in ((0.75, 0.75), (0.0, 0.0)):
            eng = Engine()
            t, x = T0, 0.0
            hist = []
            for i in range(20 * 240):
                t += 50 * MS
                v = 12.0 + 6.0 * math.sin(i * 0.05 / 4.0)     # 6..18 m/s with a 25 s period
                x += v * 0.05
                hist.append((t, x))
                ok(self, eng, {"type": "motion", "ts_mono_ns": t, "speed_mps": v, "source": "can"}, t)
                if i % 20 == 0 and i > 100:
                    xl = hist[-1 - int(lag * 20)][1]
                    lat, lon = geo.offset(52.0, 21.0, xl, 0.0)
                    ok(self, eng, {"type": "gnss", "ts_mono_ns": t, "lat": lat, "lon": lon, "hacc_m": 3.0}, t)
            g = eng.vehicle_message(t, 0)["calibration"]["gnss"]
            self.assertEqual((g["lag_s"], g["converged"] or want == 0.0), (want, True), g)
            self.assertAlmostEqual(g["speed_ratio"], 1.0, delta=0.02)
        eng = Engine(None, {"gnss_lag_s": 0.75})               # applied by hand: the fix is dated back
        ok(self, eng, {"type": "gnss", "ts_mono_ns": T0, "lat": 52.0, "lon": 21.0, "hacc_m": 3.0}, T0)
        self.assertEqual(eng.step(T0)["input"]["gnss"]["age_s"], 0.75)

    def test_state_file_and_another_car(self):
        eng, t = self.drive(gyro_scale=1.06)
        tmp = tempfile.mkdtemp(prefix="roadsense-cal-")
        try:
            path = os.path.join(tmp, "cal.json")
            self.assertTrue(save_calibration(path, eng.cal))
            cal = Calibration()
            load_calibration(path, cal)
            self.assertEqual((cal.tag, cal.gyro_gnss.converged), ("can", True))
            self.assertAlmostEqual(cal.yaw_scale(), 1.06, delta=0.01)
            cal.set_tag("can")
            self.assertTrue(cal.gyro_gnss.converged)
            cal.set_tag("another-car")                         # other input: its estimates start from nothing
            self.assertIsNone(cal.yaw_scale())
            load_calibration(os.path.join(tmp, "missing.json"), cal)       # no file: no error
            self.assertFalse(save_calibration(os.path.join(tmp, "no", "dir", "cal.json"), cal))
        finally:
            shutil.rmtree(tmp, ignore_errors=True)


class Daemon(unittest.TestCase):
    """mapd as a process with the reference client: hello negotiation, vehicle.sock, calibration file."""

    @classmethod
    def setUpClass(cls):
        cls.tmp = tempfile.mkdtemp(prefix="roadsense-vehicle-daemon-")
        cls.rundir = os.path.join(cls.tmp, "run")
        cfgdir = os.path.join(cls.tmp, "etc")
        os.makedirs(cfgdir)
        with open(os.path.join(cfgdir, "mapd.json"), "w") as f:
            json.dump({"vehicle": {"name": "Configured car", "wheelbase_m": 2.5}, "track_m": 1.5}, f)
        pack = os.path.join(cls.tmp, "synth.rsmap")
        SP.build(pack)
        env = dict(os.environ, ROADSENSE_CONFIG_DIR=cfgdir, PYTHONPATH=os.path.join(REPO, "mapd"))
        cls.proc = subprocess.Popen([sys.executable, "-m", "roadsense_mapd", "--rundir", cls.rundir, "--pack", pack,
                                     "--hz", "5", "--stats", "0", "--calibration-file", os.path.join(cls.tmp, "cal.json")],
                                    env=env, stderr=subprocess.PIPE, text=True)
        end = time.monotonic() + 10
        while not os.path.exists(os.path.join(cls.rundir, "vehicle.sock")):
            assert time.monotonic() < end and cls.proc.poll() is None, "mapd did not start"
            time.sleep(0.05)

    @classmethod
    def tearDownClass(cls):
        cls.proc.terminate()
        cls.proc.wait(10)
        cls.stderr = cls.proc.stderr.read()
        cls.proc.stderr.close()
        shutil.rmtree(cls.tmp, ignore_errors=True)

    def test_1_without_a_feeder_the_vehicle_topic_says_so(self):
        veh = Sub(os.path.join(self.rundir, "vehicle.sock"))
        v = veh.until(lambda m: m["type"] == "vehicle")
        self.assertEqual((v["present"], v["feeders"], v["sensors"], v["signals"]), (False, [], [], {}))
        self.assertEqual((v["description"]["name"], v["description"]["wheelbase_m"], v["description"]["track_m"]),
                         ("Configured car", 2.5, 1.5))
        self.assertIsNone(v["motion"]["speed_mps"])
        veh.s.close()

    def test_2_hello_declares_both_sides(self):
        feed = VehicleFeed(self.rundir, name="test-car", provides={"motion": ["speed_mps"], "controls": ["brake"]})
        self.assertFalse(feed.supports("controls"))                   # nothing known before the hello reply
        self.assertTrue(feed.supports("motion"))
        self.assertTrue(feed.motion(0.0, source="t"))
        end = time.monotonic() + 3
        while feed.accepts is None and time.monotonic() < end:
            time.sleep(0.02)
            feed.poll()
        h = feed.server
        self.assertEqual((h["api"], h["minor"], h["server"], h["clock"]), (1, 1, "roadsense-mapd", "CLOCK_MONOTONIC"))
        self.assertEqual(h["accepts"], ["gnss", "motion", "reset", "controls", "body", "vehicle", "objects"])
        self.assertIn("accel_long_mps2", h["signals"]["motion"])
        self.assertIn("cruise_set_speed_mps", h["signals"]["controls"])
        self.assertTrue(feed.supports("objects"))
        veh = Sub(os.path.join(self.rundir, "vehicle.sock"))
        v = veh.until(lambda m: m["type"] == "vehicle" and m["feeders"])
        self.assertEqual((v["feeders"][0]["name"], v["feeders"][0]["provides"]),
                         ("test-car", {"motion": ["speed_mps"], "controls": ["brake"]}))
        feed.close()
        veh.s.close()

    def test_3_every_message_type_end_to_end(self):
        veh = Sub(os.path.join(self.rundir, "vehicle.sock"))
        ego = Sub(os.path.join(self.rundir, "ego.sock"))
        time.sleep(0.2)
        feed = VehicleFeed(self.rundir, name="test-car")
        feed.describe(name="Fed car", track_m=1.6, camera={"x_m": 1.9, "y_m": 0.0, "z_m": 1.2},
                      sensors=[{"id": "radar0", "kind": "radar", "x_m": 3.7, "y_m": 0.0}])
        # the description goes out once the hello reply is in (and again after every reconnect)
        end = time.monotonic() + 3
        e = None
        while time.monotonic() < end and (e is None or abs(e["yaw_rate_rps"] - 0.2 / 1.6) > 1e-3):
            self.assertTrue(feed.motion(6.0, wheel_speeds_mps=[6.0, 6.0, 5.9, 6.1], accel_long_mps2=-1.5, source="t"))
            e = ego.until(lambda m: m.get("accel_long_mps2") is not None)
            time.sleep(0.06)
        self.assertEqual((e["type"], e["yaw_source"], e["accel_long_mps2"]), ("ego", "wheel_speeds", -1.5))
        self.assertAlmostEqual(e["yaw_rate_rps"], 0.2 / 1.6, places=3)         # the fed track width
        t_send = time.monotonic()
        self.assertTrue(feed.controls(brake=True, turn_signal="left", source="t"))
        v = veh.until(lambda m: m["type"] == "vehicle" and m["controls"]["brake"] is True, 3)
        self.assertLess(time.monotonic() - t_send, 0.15)                       # a switch change is not held for the 5 Hz step
        self.assertEqual((v["controls"]["turn_signal"], v["description"]["name"], v["description"]["sources"]["track_m"]),
                         ("left", "Fed car", "feeder"))
        self.assertTrue(feed.body(low_beam=True, outside_temp_c=5.0))
        self.assertTrue(feed.objects("radar0", [{"id": 1, "range_m": 25.0, "azimuth_deg": 0.0, "range_rate_mps": -1.0}], tracked=True))
        o = veh.until(lambda m: m["type"] == "objects", 3)
        self.assertEqual((o["sensor"], o["kind"], o["frame"], o["objects"][0]["x_m"]), ("radar0", "radar", "camera", 26.8))
        self.assertLess(abs(o["ts_real_ns"] - time.time_ns()), 2e9)
        v = veh.until(lambda m: m["type"] == "vehicle" and m["sensors"] and m["body"]["low_beam"], 3)
        self.assertEqual((v["sensors"][0]["id"], v["sensors"][0]["mounted"], v["body"]["outside_temp_c"]), ("radar0", True, 5.0))
        self.assertEqual(feed.errors, [])
        feed.close()
        v = veh.until(lambda m: m["type"] == "vehicle" and not m["feeders"], 3)        # the feeder left
        veh.s.close()
        ego.s.close()

    def test_4_v1_feeder_is_served_as_before(self):
        """The v1 reference client, unchanged: hello with api 1, gnss + motion, error replies."""
        from position_client import PositionFeed
        out = Sub(os.path.join(self.rundir, "mapd.sock"))
        feed = PositionFeed(self.rundir, name="old")
        for i in range(30):
            lat, lon = SP.ll(100 + 2 * i, 0)
            self.assertTrue(feed.gnss(lat, lon, hacc_m=4.0, speed_mps=20.0, course_deg=90.0, source="test"))
            self.assertTrue(feed.motion(20.0, yaw_rate_rps=0.0, gear="D", source="test"))
            time.sleep(0.02)
        m = out.until(lambda m: m["available"])
        self.assertEqual((m["state"], m["road"]["name"]), ("good", "Main"))
        feed.send({"type": "motion", "speed_mps": -1.0})
        time.sleep(0.3)
        feed._poll_replies()
        self.assertEqual([e["reason"] for e in feed.errors], ["speed_mps out of range"])
        feed.close()
        out.s.close()


if __name__ == "__main__":
    unittest.main()
