"""The data integrator (api/roadsense_api/integrator, docs/INTEGRATOR.md) and the consumers behind it."""
import json
import math
import os
import random
import sys
import tempfile
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))

from roadsense_api.integrator import Integrator, Measurement, SourceSpec, validity  # noqa: E402
from roadsense_api.integrator.sources import default_sources  # noqa: E402
from roadsense_api.lanes import RoadGeometryTracker  # noqa: E402
from roadsense_api.lead import LeadTracker  # noqa: E402
from roadsense_api.recorder import slim_road  # noqa: E402
from roadsense_api.speedlimit import SpeedLimit  # noqa: E402
from roadsense_api.view import ViewBuilder  # noqa: E402

NS = 10 ** 9
X = [0.0, 2.0, 5.0, 9.0, 14.0, 20.0, 27.0, 35.0, 44.0, 54.0, 65.0, 77.0, 90.0]


def ns(t):
    return int(round(t * NS))


def ego(t, v, wheels=True, yaw=0.0, accel=None, **kw):
    m = {"type": "ego", "ts_mono_ns": ns(t), "speed_mps": v, "standstill": v == 0.0, "yaw_rate_rps": yaw,
         "yaw_source": "yaw_rate", "tag": "test-can"}
    if wheels:
        m["wheel_speeds_mps"] = [0.98 * v] * 4
    if accel is not None:
        m["accel_long_mps2"] = accel
    m.update(kw)
    return m


def road(t, vx, vx_std=None, yaw=0.0, lead=None, lines=True, seq=None):
    """A vision-model `road` message for the frame at t; `vx` is what the model says the speed is."""
    m = {"type": "road", "seq": seq if seq is not None else int(t * 20), "ts_mono_ns": ns(t), "ts_real_ns": ns(t) + 10 ** 18,
         "road_model": "vision", "v_ego": vx, "valid": True,
         "pose": {"vx": vx, "vx_std": vx_std if vx_std is not None else 0.3 + 0.02 * abs(vx), "yaw_rate": -yaw,
                  "yaw_rate_std": 0.004, "vy": 0.0}}
    if lead is not None:
        d, v = lead
        m["leads"] = [{"prob": 0.95, "x": [d], "y": [0.0], "v": [v], "a": [0.0], "x_std": [0.05 * d + 0.3], "y_std": [0.3],
                       "v_std": [0.5], "a_std": [0.5]}]
    if lines:
        m["lane_lines"] = [{"x": X, "y": [y0] * len(X), "y_std": [0.1] * len(X), "prob": 0.9} for y0 in (-5.2, -1.7, 1.7, 5.2)]
        m["road_edges"] = [{"x": X, "y": [y0] * len(X), "y_std": [0.2] * len(X)} for y0 in (-6.0, 6.0)]
        m["plan"] = {"x": X, "y": [0.0] * len(X), "z": [0.0] * len(X)}
    return m


def drive(integ, t0, t1, truth, vision=lambda t, v: v, vehicle=True, rng=None, vx_std=None, hz_road=12.5, late=0.1,
          vehicle_fn=None, extra=None, wheels=True):
    """Feed a drive in arrival order: vehicle samples at 10 Hz as they happen, `road` frames `late` seconds after
    their frame time. Returns the list of (t, ego_state) per frame."""
    rng = rng or random.Random(1)
    ev = []
    t = t0
    while t < t1:
        if vehicle:
            v = truth(t)
            ev.append((t + 0.004, 0, ego(t, vehicle_fn(t, v) if vehicle_fn else v, wheels=wheels)))
        if extra:
            ev.append((t + 0.004, 2, extra(t, truth(t))))
        t += 0.1
    t = t0
    while t < t1:
        v = truth(t - 0.125)                         # the image pair's centre
        vx = vision(t, v)
        if vx is not None:
            vx += rng.gauss(0.0, 0.25 + 0.035 * abs(v)) if v != 0.0 else 0.0
            ev.append((t + late, 1, road(t, vx, vx_std)))
        t += 1.0 / hz_road
    ev.sort(key=lambda e: (e[0], e[1]))
    out = []
    for _, k, m in ev:
        if k == 1:
            out.append((m["ts_mono_ns"] / NS, integ.on_road(m)))
        else:
            integ.on_message(m)
    return out


class Estimator(unittest.TestCase):
    def test_follows_the_vehicle_and_smooths_between_samples(self):
        """An accelerating car: the state at a frame time lies between the vehicle samples around it (the sample
        after the frame has arrived by the time the frame is processed), not at the last one before."""
        truth = lambda t: 5.0 + 2.5 * t
        integ = Integrator()
        out = drive(integ, 0.0, 6.0, truth)
        errs = [es["speed"]["value"] - truth(t) for t, es in out if t > 1.0]
        self.assertLess(max(abs(e) for e in errs), 0.06)             # a held sample would be off by up to 0.25 m/s
        es = out[-1][1]
        self.assertEqual(es["speed"]["mode"], "fused")
        self.assertIn("vehicle.speed", es["speed"]["sources"])
        self.assertLess(es["speed"]["std"], 0.05)
        self.assertAlmostEqual(es["accel"]["value"], 2.5, delta=0.3)
        self.assertEqual(es["sources"]["vehicle"]["state"], "ok")
        self.assertEqual(es["sources"]["vision"]["state"], "ok")
        self.assertEqual(es["sources"]["vehicle"]["tag"], "test-can")

    def test_arrival_order_does_not_matter(self):
        """The same measurements, the camera 0.1 s or 0.3 s late: the same state for the same time of validity."""
        truth = lambda t: 8.0 + 1.5 * math.sin(0.8 * t)
        a = Integrator()
        b = Integrator()
        drive(a, 0.0, 8.0, truth, late=0.1, rng=random.Random(3))
        drive(b, 0.0, 8.0, truth, late=0.3, rng=random.Random(3))
        for T in (5.0, 6.03, 7.2):
            sa, sb = a.state(ns(T)), b.state(ns(T))
            self.assertAlmostEqual(sa["speed"]["value"], sb["speed"]["value"], delta=1e-3)
            self.assertAlmostEqual(sa["yaw_rate"]["value"], sb["yaw_rate"]["value"], delta=1e-4)

    def test_camera_only_car(self):
        """No vehicle feed: the camera's speed, filtered; the source is alive and unchecked, nothing is invalid."""
        integ = Integrator()
        out = drive(integ, 0.0, 20.0, lambda t: 12.0, vehicle=False)
        es = out[-1][1]
        self.assertTrue(es["speed"]["valid"])
        self.assertEqual(es["speed"]["sources"], ["vision.pose"])
        self.assertAlmostEqual(es["speed"]["value"], 12.0, delta=0.5)
        self.assertEqual(es["sources"]["vision"]["state"], "unchecked")
        self.assertEqual(es["sources"]["vehicle"]["state"], "absent")
        self.assertEqual(validity(es, ("vision",), ("speed",))[:2], (True, None))
        self.assertEqual(validity(es)[2], {"vision": "unchecked", "vehicle": "absent"})

    def test_standstill(self):
        integ = Integrator()
        out = drive(integ, 0.0, 5.0, lambda t: 0.0)
        es = out[-1][1]
        self.assertEqual(es["speed"]["value"], 0.0)
        self.assertEqual((es["standstill"]["value"], es["standstill"]["source"]), (True, "vehicle"))
        self.assertGreater(es["standstill"]["for_s"], 4.0)
        self.assertEqual(es["yaw_rate"]["value"], 0.0)
        # camera only: the model's speed under the threshold for a second
        integ = Integrator()
        out = drive(integ, 0.0, 5.0, lambda t: 0.0, vehicle=False)
        self.assertEqual((out[-1][1]["standstill"]["value"], out[-1][1]["standstill"]["source"]), (True, "vision"))

    def test_vehicle_gap_is_bridged_by_the_camera_and_silence_is_unknown(self):
        integ = Integrator()
        truth = lambda t: 10.0 + 0.5 * t
        drive(integ, 0.0, 10.0, truth)
        out = drive(integ, 10.0, 14.0, truth, vehicle=False)          # the feeder stops, the camera goes on
        es = out[-1][1]
        self.assertTrue(es["speed"]["valid"])
        self.assertEqual(es["speed"]["sources"], ["vision.pose"])
        self.assertEqual(es["sources"]["vehicle"]["state"], "stale")
        self.assertAlmostEqual(es["speed"]["value"], truth(out[-1][0]), delta=1.0)
        self.assertGreater(es["speed"]["std"], 0.1)                   # and the uncertainty says so
        # everything silent: coast, then unknown
        T = out[-1][0]
        self.assertEqual(integ.state(ns(T + 0.6))["speed"]["mode"], "coast")
        late = integ.state(ns(T + 1.5))
        self.assertEqual((late["speed"]["valid"], late["speed"]["reason"]), (False, "no_measurement"))
        self.assertIsNone(late["speed"]["value"])

    def test_first_samples_after_a_feed_silence_are_not_believed(self):
        """Measured on the car: after a silence the feed's first samples repeat what it held before (here: 0 and
        standstill while the car does 8 m/s). They must not become the ego speed; the camera carries on."""
        integ = Integrator()
        drive(integ, 0.0, 10.0, lambda t: 8.0)
        drive(integ, 10.0, 17.0, lambda t: 8.0, vehicle=False)              # the feed is silent for 7 s
        integ.on_message(ego(17.00, 0.0))                                   # stale: what it held long ago
        integ.on_message(ego(17.08, 0.0))
        r = road(17.05, 8.1)
        es = integ.on_road(r)
        self.assertAlmostEqual(es["speed"]["value"], 8.0, delta=0.6)
        self.assertEqual(es["speed"]["sources"], ["vision.pose"])
        self.assertFalse(es["standstill"]["value"])
        self.assertEqual(es["sources"]["vehicle"]["state"], "stale")
        out = drive(integ, 17.4, 19.0, lambda t: 8.0)                       # 0.4 s later the feed is believed again
        self.assertIn("vehicle.speed", out[-1][1]["speed"]["sources"])
        self.assertEqual(out[-1][1]["sources"]["vehicle"]["warmups"], 1)

    def test_a_lone_camera_is_followed_not_locked_out(self):
        """A single-sample gate may only reject the camera while a trusted source anchors the state. Alone, the
        camera is the estimate: a quick true change must come through (a gate lock-out cost 9 m/s in a replay)."""
        integ = Integrator()
        truth = lambda t: 12.0 if t < 10.0 else max(2.0, 12.0 - 9.0 * (t - 10.0))     # an emergency stop
        out = drive(integ, 0.0, 14.0, truth, vehicle=False)
        self.assertEqual(out[-1][1]["rejected"], {})
        self.assertLess(max(abs(es["speed"]["value"] - truth(t)) for t, es in out if t > 11.5), 2.0)
        # and its std is not better than the camera's own (its error is correlated, filtering does not remove it)
        self.assertGreaterEqual(out[-1][1]["speed"]["std"], 0.25)

    def test_signals_pass_through_with_their_age(self):
        integ = Integrator()
        integ.on_message(ego(1.0, 5.0, turn_signal="left"))
        integ.on_message({"type": "vehicle", "ts_mono_ns": ns(1.0), "controls": {"brake": True}})
        es = integ.state(ns(1.5))
        self.assertEqual(es["signals"]["turn_signal"], {"value": "left", "age_s": 0.5})
        self.assertEqual(es["signals"]["brake"]["value"], True)
        # mapd's `vehicle` status line does not make the vehicle look alive
        integ = Integrator()
        integ.on_message({"type": "vehicle", "ts_mono_ns": ns(1.0), "controls": {"brake": None}})
        self.assertEqual(integ.state(ns(1.0))["sources"]["vehicle"]["state"], "absent")

    def test_epoch_on_a_clock_restart(self):
        integ = Integrator()
        drive(integ, 100.0, 110.0, lambda t: 12.0, vision=lambda t, v: 0.15 * v)
        self.assertEqual(integ.state(ns(110.0))["sources"]["vision"]["state"], "fault")
        out = drive(integ, 5.0, 6.0, lambda t: 0.0)                    # the device rebooted: the clock starts again
        es = out[-1][1]
        self.assertEqual(es["epoch"], 1)
        self.assertEqual(es["sources"]["vision"]["state"], "fault")    # the fault is carried, not forgotten
        self.assertEqual(es["sources"]["vision"]["isolation"], "carried")
        self.assertTrue(es["speed"]["valid"])


class Faults(unittest.TestCase):
    def test_camera_that_misreads_the_speed_is_faulted_and_recovers(self):
        """The incident of 2026-11-19: the road model reads a fraction of the true speed."""
        integ = Integrator()
        truth = lambda t: 0.0 if t < 5.0 else min(12.0, 3.0 * (t - 5.0))
        bad = lambda t, v: 0.15 * v if 5.0 <= t < 40.0 else v
        out = drive(integ, 0.0, 70.0, truth, vision=bad)
        state = lambda T: [es for t, es in out if t >= T][0]
        self.assertEqual(state(4.0)["sources"]["vision"]["state"], "unchecked")     # standing: nothing to check yet
        first = [t for t, es in out if es["sources"]["vision"]["state"] == "fault"][0]
        self.assertLess(first, 6.0 + 2.5)                                # 3 m/s is reached at 6 s: evidence starts there
        es = state(20.0)
        v = es["sources"]["vision"]
        # the wheels agree with the speed signal, but they are the same sensors: no vote, the rank decides
        self.assertEqual((v["state"], v["usable"], v["isolation"]), ("fault", False, "rank"))
        self.assertIn("speed disagrees with vehicle", v["reason"])
        self.assertAlmostEqual(es["speed"]["value"], 12.0, delta=0.05)   # the estimate is the vehicle's
        self.assertEqual(es["speed"]["sources"], ["vehicle.speed"])
        self.assertEqual(es["sources"]["vehicle"]["state"], "ok")
        ok, why, inputs = validity(es, ("vision",), ("speed",))
        self.assertFalse(ok)
        self.assertTrue(why.startswith("vision: speed disagrees with vehicle"))
        self.assertEqual(inputs["vision"], "fault")
        # recovery by evidence only: at most a quarter of the last 10 s of evidence may be bad, i.e. 7.5 s of
        # agreement after the camera reads right again (at 40 s)
        back = [t for t, es in out if t > 40.0 and es["sources"]["vision"]["state"] != "fault"][0]
        self.assertTrue(47.0 < back < 50.0, back)
        self.assertEqual(out[-1][1]["sources"]["vision"]["state"], "ok")

    def test_without_wheels_the_camera_is_isolated_by_rank(self):
        integ = Integrator()
        out = drive(integ, 0.0, 20.0, lambda t: 12.0, vision=lambda t, v: 0.4 * v, wheels=False)
        v = out[-1][1]["sources"]["vision"]
        self.assertEqual((v["state"], v["isolation"]), ("fault", "rank"))

    def test_fault_holds_while_standing(self):
        """Standing still is no evidence in either direction: a faulted camera stays faulted at the lights."""
        integ = Integrator()
        truth = lambda t: 12.0 if t < 15.0 else max(0.0, 12.0 - 3.0 * (t - 15.0))
        out = drive(integ, 0.0, 120.0, truth, vision=lambda t, v: 0.2 * v)
        self.assertEqual(out[-1][1]["sources"]["vision"]["state"], "fault")
        self.assertEqual(out[-1][1]["standstill"]["value"], True)

    def test_healthy_noise_and_a_few_wild_frames_are_no_fault(self):
        rng = random.Random(7)
        wild = lambda t, v: v + (6.0 if rng.random() < 0.15 else 0.0)     # 15 % of the frames 6 m/s off
        integ = Integrator()
        out = drive(integ, 0.0, 120.0, lambda t: 10.0 + 4.0 * math.sin(0.3 * t), vision=wild)
        self.assertFalse(any(es["sources"]["vision"]["state"] == "fault" for _, es in out))
        self.assertGreater(out[-1][1]["rejected"].get("vision.pose", 0), 50)      # single frames are gated out
        self.assertLess(max(abs(es["speed"]["value"] - (10.0 + 4.0 * math.sin(0.3 * t))) for t, es in out if t > 2), 0.1)

    def test_self_check_faults_a_camera_only_car(self):
        """No vehicle feed: the model's own stated uncertainty, far beyond its characterised one, is the check."""
        integ = Integrator()
        out = drive(integ, 0.0, 10.0, lambda t: 10.0, vehicle=False)
        out += drive(integ, 10.0, 30.0, lambda t: 10.0, vehicle=False, vision=lambda t, v: 0.3 * v, vx_std=5.0)
        es = out[-1][1]
        v = es["sources"]["vision"]
        self.assertEqual((v["state"], v["isolation"]), ("fault", "self"))
        self.assertIn("own speed uncertainty", v["reason"])
        self.assertEqual((es["speed"]["valid"], es["speed"]["reason"]), (False, "no_healthy_source"))
        first = [t for t, e in out if e["sources"]["vision"]["state"] == "fault"][0]
        self.assertLess(first, 14.0)
        out = drive(integ, 30.0, 50.0, lambda t: 10.0, vehicle=False)   # the model is itself again
        self.assertEqual(out[-1][1]["sources"]["vision"]["state"], "unchecked")
        self.assertTrue(out[-1][1]["speed"]["valid"])

    def test_carried_fault_clears_on_agreement(self):
        integ = Integrator()
        integ.carry_fault("vision", "carried over from the previous run: speed disagrees with vehicle")
        out = drive(integ, 0.0, 5.0, lambda t: 0.0)
        self.assertEqual(out[-1][1]["sources"]["vision"]["state"], "fault")       # parked: still faulted
        out = drive(integ, 5.0, 12.0, lambda t: 10.0)
        self.assertEqual(out[-1][1]["sources"]["vision"]["state"], "ok")


class Extension(unittest.TestCase):
    """A new sensor is an input description and a measurement model; nothing else changes."""

    @staticmethod
    def radar_spec():
        def model(msg, mem):
            # ego speed from the stationary targets of a scan: the median of -range_rate / cos(azimuth)
            v = sorted(-o["range_rate_mps"] / math.cos(o["azimuth_rad"]) for o in msg["objects"] if o.get("stationary"))
            if not v:
                return []
            return [Measurement("speed", msg["ts_mono_ns"] / NS, v[len(v) // 2], 0.15, "radar_front.ego_speed")]
        return SourceSpec("radar_front", "radar_front", 2, ("radar_scan",), model, stale_s=0.5)

    @staticmethod
    def scan(t, v):
        return {"type": "radar_scan", "ts_mono_ns": ns(t),
                "objects": [{"range_rate_mps": -v * math.cos(az), "azimuth_rad": az, "stationary": True} for az in (-0.2, 0.0, 0.3)]}

    def test_radar_joins_the_estimate_and_the_checks(self):
        integ = Integrator(sources=default_sources() + [self.radar_spec()])
        out = drive(integ, 0.0, 15.0, lambda t: 14.0, extra=self.scan)
        es = out[-1][1]
        self.assertEqual(es["sources"]["radar_front"]["state"], "ok")
        self.assertIn("radar_front.ego_speed", es["speed"]["sources"])
        pairs = {(c["a"], c.get("b")): c["verdict"] for c in es["checks"] if c["quantity"] == "speed"}
        self.assertEqual(pairs[("vehicle", "radar_front")], "agree")
        self.assertEqual(pairs[("radar_front", "vision")], "agree")

    def test_wrong_vehicle_speed_is_isolated_by_vote(self):
        """The vehicle speed reads 60 % (a wrong CAN scale); radar and camera agree with each other: the vote
        faults the higher-ranked vehicle, and the estimate is made from the others."""
        integ = Integrator(sources=default_sources() + [self.radar_spec()])
        out = drive(integ, 0.0, 20.0, lambda t: 14.0, vehicle_fn=lambda t, v: 0.6 * v, extra=self.scan)
        es = out[-1][1]
        v = es["sources"]["vehicle"]
        self.assertEqual((v["state"], v["isolation"]), ("fault", "vote"))
        self.assertEqual(es["sources"]["vision"]["state"], "ok")
        self.assertEqual(es["sources"]["radar_front"]["state"], "ok")
        self.assertAlmostEqual(es["speed"]["value"], 14.0, delta=0.3)
        self.assertNotIn("vehicle.speed", es["speed"]["sources"])

    def test_camera_checked_by_radar_without_any_vehicle_feed(self):
        integ = Integrator(sources=default_sources() + [self.radar_spec()])
        out = drive(integ, 0.0, 15.0, lambda t: 14.0, vehicle=False, vision=lambda t, v: 0.2 * v, extra=self.scan)
        es = out[-1][1]
        self.assertEqual((es["sources"]["vision"]["state"], es["sources"]["vision"]["isolation"]), ("fault", "rank"))
        self.assertAlmostEqual(es["speed"]["value"], 14.0, delta=0.2)


class Consumers(unittest.TestCase):
    """What lead / lanes / view / the odometer put out while the integrator holds the camera faulted."""

    def run_pipeline(self, bad_from=None, t1=30.0):
        integ, lead, lanes, view = Integrator(), LeadTracker(), RoadGeometryTracker(), ViewBuilder()
        rng = random.Random(5)
        outs = []
        t = 0.0
        while t < t1:
            v = 12.0
            integ.on_message(ego(t, v))
            bad = bad_from is not None and t >= bad_from
            vx = (0.15 * v if bad else v) + rng.gauss(0, 0.3)
            # the knocked camera reports a near-standing lead a few metres ahead with a high probability
            r = road(t + 0.05, vx, lead=(5.0, 0.3) if bad else (30.0, 12.0))
            es = integ.on_road(r)
            ld = lead.on_road(r, es)
            ln = lanes.on_road(r, es)
            vw = view.on_frame(r, ld, ln, ego=es)
            outs.append((t, es, ld, ln, vw))
            t += 0.1
        return outs

    def test_healthy(self):
        t, es, ld, ln, vw = self.run_pipeline()[-1]
        self.assertTrue(ld["present"] and ld["valid"])
        self.assertIsNone(ld["reason"])
        self.assertEqual(ld["inputs"], {"vision": "ok", "vehicle": "ok"})
        self.assertAlmostEqual(ld["dist_m"], 30.0, delta=1.0)
        self.assertEqual((ld["ego"]["source"], ld["source"]), ("vehicle", "vision+ego"))
        self.assertAlmostEqual(ld["ego"]["speed_mps"], 12.0, delta=0.05)
        self.assertTrue(ln["valid"] and ln["lines"][1]["present"] and ln["lines"][2]["present"])
        self.assertEqual(ln["ego"]["source"], "vehicle")
        self.assertTrue(vw["integrity"]["valid"])
        self.assertTrue(any(o.get("role") == "lead" for o in vw["objects"]))
        self.assertIsNotNone(vw["path"])

    def test_faulted_camera_gives_unknown_not_a_phantom_lead(self):
        outs = self.run_pipeline(bad_from=10.0)
        t, es, ld, ln, vw = outs[-1]
        self.assertEqual(es["sources"]["vision"]["state"], "fault")
        self.assertEqual((ld["present"], ld["valid"], ld["mode"], ld["dist_m"]), (False, False, "none", None))
        self.assertTrue(ld["reason"].startswith("vision: speed disagrees with vehicle"))
        self.assertIsNone(ld["vision"])                                  # no raw value to fall back on either
        self.assertEqual(ld["inputs"]["vision"], "fault")
        self.assertAlmostEqual(ld["ego"]["speed_mps"], 12.0, delta=0.05)  # the ego speed is still known: the vehicle's
        self.assertFalse(ln["valid"])
        self.assertFalse(any(x["present"] for x in ln["lines"] + ln["edges"]))
        self.assertFalse(ln["lane_to_left"] or ln["lane_to_right"])
        self.assertFalse(vw["integrity"]["valid"])
        self.assertIn("speed disagrees", vw["integrity"]["reason"])
        self.assertIsNone(vw["path"])
        late = [o for o in outs if o[0] > 16.0]
        self.assertFalse(any(any(ob.get("role") == "lead" for ob in o[4]["objects"]) for o in late))
        self.assertFalse(any(any(x.get("y") for x in o[4]["lines"]) for o in late))
        # how long the phantom was out: from the first bad frame to the first invalid lead
        first_invalid = [o[0] for o in outs if o[2]["valid"] is False][0]
        self.assertLess(first_invalid - 10.0, 2.5)
        ev = [o[2]["event"] for o in outs if o[2]["event"] and o[2]["event"]["kind"] == "invalid"]
        self.assertTrue(ev)

    def test_trackers_alone_run_the_same_integrator(self):
        """A tracker that is not handed an ego state builds its own from the messages it is fed."""
        lead = LeadTracker()
        rng = random.Random(2)
        o = None
        for i in range(300):
            t = 0.1 * i
            lead.on_ego(ego(t, 12.0))
            bad = t >= 10.0
            o = lead.on_road(road(t + 0.05, (0.15 if bad else 1.0) * 12.0 + rng.gauss(0, 0.3), lead=(5.0, 0.3) if bad else (30.0, 12.0)))
        self.assertEqual((o["present"], o["valid"]), (False, False))

    def test_odometer_runs_on_the_vehicle_speed(self):
        """Until L37 the speed-limit / rules odometer integrated the road model's speed, whatever the vehicle said."""
        sl = SpeedLimit()
        integ = Integrator()
        t = 0.0
        while t < 20.0:
            integ.on_message(ego(t, 12.0))
            r = road(t + 0.05, 2.0, lines=False)                         # the model reads 2 m/s at a true 12
            sl.on_message(r, integ.on_road(r))
            t += 0.1
        self.assertAlmostEqual(sl.odo, 12.0 * 20.0, delta=6.0)
        self.assertEqual(sl.ego_source, "vehicle")
        self.assertAlmostEqual(sl.v_ego, 12.0, delta=0.05)
        # camera only, and the camera is faulted by its own uncertainty: the odometer stands, it does not guess
        sl, integ = SpeedLimit(), Integrator()
        t = 0.0
        while t < 20.0:
            r = road(t, 2.0, vx_std=6.0, lines=False)
            sl.on_message(r, integ.on_road(r))
            t += 0.08
        self.assertIsNone(sl.v_ego)
        self.assertLess(sl.odo, 2.0 * 6.0)


class RecorderFidelity(unittest.TestCase):
    def test_slim_road_keeps_what_selects_the_profile(self):
        r = road(1.0, 10.0, lead=(20.0, 9.0))
        r["policy_hz"] = 5
        s = slim_road(r)
        self.assertEqual(s["road_model"], "vision")
        self.assertEqual(s["policy_hz"], 5)
        self.assertIn("pose", s)

    def test_replay_restores_road_model_and_merges_gnss(self):
        sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", "tools"))
        import replay_lib
        with tempfile.TemporaryDirectory() as d:
            path = os.path.join(d, "messages.jsonl")
            with open(path, "w") as f:
                for i in range(50):
                    r = road(100.0 + 0.1 * i, 10.0)
                    del r["road_model"]                                   # a session recorded before L37
                    r["ts_real_ns"] = ns(1_700_000_000.0 + 0.1 * i)
                    f.write(json.dumps(r) + "\n")
            pos = [{"type": "gnss", "ts_mono_ns": ns(102.0), "speed_mps": 9.5, "fix": "3d", "_wall": 1_700_000_002.0},
                   {"type": "gnss", "ts_mono_ns": ns(103.0), "speed_mps": 9.5, "fix": "3d", "_wall": 1_700_009_999.0}]   # another boot
            msgs = list(replay_lib.read_session(path, road_model="vision", pos=pos))
            self.assertTrue(all(m["road_model"] == "vision" for m in msgs if m["type"] == "road"))
            g = [m for m in msgs if m["type"] == "gnss"]
            self.assertEqual([m["ts_mono_ns"] for m in g], [ns(102.0)])
            i = msgs.index(g[0])
            self.assertLessEqual(msgs[i - 1]["ts_mono_ns"], ns(102.03))
            p = replay_lib.Pipeline()
            types = [o["type"] for m in msgs for o in p.feed(m)]
            self.assertIn("ego_state", types)
            self.assertEqual(p.lead.road_model, "vision")


if __name__ == "__main__":
    unittest.main()


class ApiState(unittest.TestCase):
    """state.py: the integrator in the fan-out, the fault kept across restarts."""

    def feed(self, st, t0, t1, bad):
        rng = random.Random(4)
        t = t0
        while t < t1:
            e = ego(t, 12.0)
            st.on_message("mapd", e, json.dumps(e))
            r = road(t + 0.05, (0.15 if bad else 1.0) * 12.0 + rng.gauss(0, 0.3), lead=(5.0, 0.3) if bad else (30.0, 12.0))
            st.on_message("modeld", r, json.dumps(r))
            t += 0.1

    def test_topics_and_carried_fault(self):
        from roadsense_api.state import State
        with tempfile.TemporaryDirectory() as d:
            old = os.environ.get("ROADSENSE_CONFIG_DIR")
            os.environ["ROADSENSE_CONFIG_DIR"] = d
            self.addCleanup(lambda: os.environ.pop("ROADSENSE_CONFIG_DIR", None) if old is None
                            else os.environ.__setitem__("ROADSENSE_CONFIG_DIR", old))
            st = State(d, datadir=d)
            seen = []
            st.subscribers.add(lambda topic, line: seen.append(topic))
            self.feed(st, 0.0, 6.0, bad=False)
            i = len(seen) - 1 - seen[::-1].index("road")
            self.assertEqual(seen[i:i + 5], ["road", "ego_state", "lead", "lanes", "view"])       # per frame, in this order
            self.assertTrue(st.latest["lead"]["present"] and st.latest["lead"]["valid"])
            self.assertEqual(st.health()["integrator"]["faults"], [])
            self.feed(st, 6.0, 12.0, bad=True)
            self.assertEqual(st.health()["integrator"]["faults"], ["vision"])
            self.assertEqual((st.latest["lead"]["present"], st.latest["lead"]["valid"]), (False, False))
            self.assertFalse(st.latest["view"]["integrity"]["valid"])
            saved = json.load(open(os.path.join(d, "integrator.json")))
            self.assertIn("vision", saved["faults"])
            # the next run (the car was parked): the camera starts faulted, and a standing phantom lead is not tracked
            st2 = State(d, datadir=d)
            e = ego(100.0, 0.0)
            st2.on_message("mapd", e, json.dumps(e))
            r = road(100.05, 0.0, lead=(5.0, 0.0))
            st2.on_message("modeld", r, json.dumps(r))
            v = st2.latest["ego_state"]["sources"]["vision"]
            self.assertEqual((v["state"], v["isolation"]), ("fault", "carried"))
            self.assertEqual((st2.latest["lead"]["present"], st2.latest["lead"]["valid"]), (False, False))
            # it drives and the camera agrees again: cleared, also on disk
            self.feed(st2, 101.0, 112.0, bad=False)
            self.assertEqual(st2.latest["ego_state"]["sources"]["vision"]["state"], "ok")
            self.assertEqual(json.load(open(os.path.join(d, "integrator.json")))["faults"], {})
