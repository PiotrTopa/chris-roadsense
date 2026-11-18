"""Display channel `view` (api/roadsense_api/view.py): smooth, identity-keeping, honest about prediction, and
never a source for the measurement topics.

    python3 -m unittest api.tests.test_view -v     (from the repository root)
"""
import copy
import json
import math
import os
import random
import sys
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))

from roadsense_api.state import State          # noqa: E402
from roadsense_api.view import ViewBuilder     # noqa: E402

NS = 10 ** 9
XS = [0.0, 3.0, 6.75, 12.0, 18.75, 27.0, 36.75, 48.0, 60.75, 75.0, 90.75, 108.0]


def road(t, seq, v=10.0, plan_y=0.0):
    return {"type": "road", "seq": seq, "ts_mono_ns": int(t * NS), "ts_real_ns": int(t * NS), "v_ego": v,
            "pose": {"vx": v, "yaw_rate": 0.0},
            "plan": {"x": [0.0, 50.0, 100.0, 150.0], "y": [plan_y] * 4, "z": [0.0] * 4}}


def lanes_msg(t, ys=None, present=(True, True, True, True)):
    ys = ys or [-5.4, -1.8, 1.8, 5.4]
    lines = [{"present": p, "mode": "track", "conf": 0.9, "y": [y] * len(XS) if p else None,
              "y_std": [0.1] * len(XS) if p else None} for p, y in zip(present, ys)]
    return {"type": "lanes", "ts_mono_ns": int(t * NS), "x": XS, "lines": lines,
            "edges": [{"present": False, "y": None}, {"present": False, "y": None}]}


def lights_msg(t, d, lat=0.5, tid=7, state="red", ego=True, coast=0):
    li = {"track_id": tid, "state": state, "dist_m": d, "lateral_m": lat, "box": [600, 100, 610, 130],
          "relevance": 0.8, "coast": coast}
    return {"type": "lights", "ts_mono_ns": int(t * NS), "lights": [li],
            "ego": {"track_id": tid, "state": state, "dist_m": d, "lateral_m": lat, "relevance": 0.8,
                    "held": False, "agree": 1} if ego else None}


def lead_msg(t, d, tid=3, present=True, v_rel=0.0, standing=False, mode="track"):
    if not present:
        return {"type": "lead", "present": False, "mode": "none", "track_id": None, "dist_m": None}
    return {"type": "lead", "present": True, "mode": mode, "track_id": tid, "dist_m": d, "lateral_m": 0.1,
            "v_rel_mps": v_rel, "dist_filter_std_m": 0.05, "dist_std_m": 0.5, "standing": standing,
            "departed": False, "meas_age_s": 0.0}


def obj(view, role):
    return next((o for o in view["objects"] if o["role"] == role), None)


class StaticObjects(unittest.TestCase):
    def test_a_light_approaches_smoothly_between_noisy_detections(self):
        """A light 80 m ahead, the car at 10 m/s; its distance is detected 3 times a second with the housing-size
        error (25 %). The drawn distance moves at the car's speed and does not step by the error."""
        rnd = random.Random(1)
        vb = ViewBuilder()
        t, truth, seq = 100.0, 80.0, 0
        steps, errs, raw_steps, last_raw = [], [], [], None
        prev = None
        next_light = t
        while truth > 15.0:
            t += 0.075
            truth -= 10.0 * 0.075
            vb.on_ego({"ts_mono_ns": int(t * NS), "speed_mps": 10.0, "yaw_rate_rps": 0.0})
            if t >= next_light:
                next_light += 0.33
                meas = truth * (1.0 + rnd.gauss(0, 0.15))
                vb.on_lights(lights_msg(t, meas))
                if last_raw is not None:
                    raw_steps.append(abs(meas - last_raw + 10.0 * 0.33))
                last_raw = meas
            seq += 1
            v = vb.on_frame(road(t, seq), None, None)
            o = obj(v, "ego")
            self.assertIsNotNone(o)
            if prev is not None and truth < 75.0:          # after the fade-in (the first detections)
                steps.append(abs(o["x"] - prev + 10.0 * 0.075))
            prev = o["x"]
            if truth < 50.0:
                errs.append(o["x"] - truth)
        rms = lambda v: math.sqrt(sum(x * x for x in v) / len(v))      # noqa: E731
        self.assertLess(rms(steps), 0.1 * rms(raw_steps))              # 10x steadier than the detections
        self.assertLess(max(steps), 1.5)
        self.assertLess(rms(errs), 4.0)                                # and still where the light is
        self.assertEqual(o["kind"], "traffic_light")
        self.assertEqual(o["attrs"]["state"], "red")
        self.assertLess(o["vx"], -9.0)                                 # approaching at the car's speed

    def test_a_light_that_is_no_longer_reported_leaves_marked_and_bounded(self):
        vb = ViewBuilder()
        vb.on_lights(lights_msg(100.0, 40.0))
        phases = []
        for k in range(1, 40):
            t = 100.0 + 0.075 * k
            v = vb.on_frame(road(t, k, v=0.0), None, None)
            o = next((o for o in v["objects"] if o["kind"] == "traffic_light"), None)
            phases.append(None if o is None else (o["phase"], o["meas_age_s"]))
        self.assertIn("coast", [p[0] for p in phases if p])            # predicted, and it says so
        self.assertIn("leave", [p[0] for p in phases if p])
        self.assertIsNone(phases[-1])                                  # gone after keep_s + fade
        self.assertTrue(all(p is None or p[1] <= 1.4 for p in phases))

    def test_lateral_is_in_the_car_frame(self):
        """lights.lateral_m is the offset from the planned path; the view puts it back into the car's frame."""
        vb = ViewBuilder()
        vb.on_road(road(100.0, 1, plan_y=2.0))
        vb.on_lights(lights_msg(100.0, 40.0, lat=0.5))
        o = obj(vb.on_frame(road(100.05, 2, plan_y=2.0), None, None), "ego")
        self.assertAlmostEqual(o["y"], 2.5, delta=0.05)

    def test_signs_are_objects_with_a_distance(self):
        vb = ViewBuilder()
        vb.on_signs({"type": "signs", "ts_mono_ns": 100 * NS, "detections": [
            {"track_id": 31, "cls": "regulatory--stop", "score": 0.8, "box": [900, 300, 920, 320]}]},
            (975.0, 640.0, 360.0, 1280, 720))
        o = next(o for o in vb.on_frame(road(100.05, 1, v=0.0), None, None)["objects"] if o["kind"] == "sign")
        self.assertEqual(o["id"], "sign:31")
        self.assertAlmostEqual(o["x"], 975.0 * 0.7 / 20.0, delta=0.5)
        self.assertGreater(o["y"], 0.0)


class LeadAndLines(unittest.TestCase):
    def test_lead_is_passed_on_without_lag_and_fades(self):
        vb = ViewBuilder()
        alphas = []
        for k in range(1, 12):
            v = vb.on_frame(road(100 + 0.075 * k, k, v=0.0), lead_msg(0, 3.0 + 0.01 * k, standing=True), None)
            o = obj(v, "lead")
            self.assertEqual(o["x"], round(3.0 + 0.01 * k, 2))            # the tracked value itself
            self.assertTrue(o["attrs"]["standing"])
            alphas.append(o["alpha"])
        self.assertEqual(alphas[0] < 1.0, True)
        self.assertEqual(alphas[-1], 1.0)
        out = []
        for k in range(12, 24):
            v = vb.on_frame(road(100 + 0.075 * k, k, v=0.0), lead_msg(0, 0, present=False), None)
            out.append(obj(v, "lead"))
        self.assertEqual(out[0]["phase"], "leave")
        self.assertIsNone(out[-1])

    def test_lines_fade_in_and_out_instead_of_popping(self):
        vb = ViewBuilder()
        seen = []
        for k in range(1, 30):
            present = (False, True, True, k < 15)
            v = vb.on_frame(road(100 + 0.075 * k, k), None, lanes_msg(100 + 0.075 * k, present=present))
            ln = next((l for l in v["lines"] if l["id"] == "lane:3"), None)
            seen.append(None if ln is None else ln["alpha"])
        self.assertLess(seen[0], 1.0)
        self.assertEqual(seen[10], 1.0)
        self.assertTrue(0.0 < seen[15] < 1.0)                          # fading
        self.assertIsNone(seen[-1])

    def test_a_restarted_line_is_blended_not_jumped(self):
        vb = ViewBuilder()
        for k in range(1, 10):
            vb.on_frame(road(100 + 0.075 * k, k), None, lanes_msg(100 + 0.075 * k))
        ys = []
        for k in range(10, 20):                                        # the left line restarts 3.6 m to the left
            v = vb.on_frame(road(100 + 0.075 * k, k), None, lanes_msg(100 + 0.075 * k, ys=[-9.0, -5.4, 1.8, 5.4]))
            ys.append(next(l for l in v["lines"] if l["id"] == "lane:1")["y"][2])
        steps = [abs(b - a) for a, b in zip([-1.8] + ys, ys)]
        self.assertLess(max(steps), 1.5)
        self.assertAlmostEqual(ys[-1], -5.4, delta=0.1)

    def test_inputs_are_not_modified(self):
        vb = ViewBuilder()
        ln, ld, li = lanes_msg(100.0), lead_msg(0, 5.0), lights_msg(100.0, 30.0)
        c_ln, c_ld, c_li = copy.deepcopy(ln), copy.deepcopy(ld), copy.deepcopy(li)
        vb.on_lights(li)
        vb.on_frame(road(100.05, 1), ld, ln)
        self.assertEqual((ln, ld, li), (c_ln, c_ld, c_li))


class InTheApi(unittest.TestCase):
    def test_view_follows_lead_and_lanes_and_never_feeds_back(self):
        st = State("/nonexistent")
        got = []
        st.subscribers.add(lambda t, line: got.append((t, json.loads(line))))
        for k in range(1, 6):
            m = road(100 + 0.075 * k, k)
            m.update(valid=True, leads=[{"prob": 0.9, "x": [20.0], "y": [0.0], "v": [10.0], "x_std": [1.0],
                                         "v_std": [0.5]}])
            st.on_message("modeld", m, json.dumps(m))
        types = [t for t, _ in got]
        self.assertIn("view", types)
        i = len(types) - 1 - types[::-1].index("view")
        j = len(types) - 1 - types[::-1].index("road")
        self.assertTrue(j < i and "lead" in types[j:i])               # after the frame's measurement topics
        lead_before = [m for t, m in got if t == "lead"]
        self.assertTrue(all("view" not in json.dumps(m) for m in lead_before))
        self.assertEqual(st.health()["view"]["objects"], 1)


if __name__ == "__main__":
    unittest.main()
