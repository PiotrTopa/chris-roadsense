"""The api with `road` messages of the v0.11.1 vision model (modeld --road-model vision, lane L24-M).

Without the policy the message has no `plan` key (absent, never null: lights.py reads road.get("plan", {})),
`desired_curvature` / `desire_state` are null and `road_model` is "vision". Every consumer must take that.

    cd <repo> && python3 -m unittest discover -s chris-integrator/tests -v
"""
import os
import sys
import unittest

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from chris_integrator.trackers import lights  # noqa: E402
from chris_integrator.replay import Pipeline  # noqa: E402

X = [192.0 * (i / 32) ** 2 for i in range(33)]


def line(y0, prob=0.9):
    return {"x": X, "y": [y0] * 33, "z": [1.2] * 33, "y_std": [0.1 + 0.01 * x for x in X], "prob": prob}


def road(seq, plan=False):
    m = {"type": "road", "seq": seq, "ts_mono_ns": seq * 50_000_000, "ts_real_ns": seq * 50_000_000,
         "model_ms": 30.0, "prep_ms": 4.0, "frame_seq_prev": seq - 4, "pair_dt_ms": 200, "frames_skipped": 0,
         "v_ego": 15.0, "thermal_c": None, "effective_hz": 20, "rate_limited": False, "steer_delay": 0.4,
         "valid": True, "road_model": "vision",
         "lane_lines": [line(-5.4, 0.3), line(-1.8), line(1.8), line(5.4, 0.3)],
         "road_edges": [{k: v for k, v in line(-7.0).items() if k != "prob"},
                        {k: v for k, v in line(3.5).items() if k != "prob"}],
         "leads": [{"prob": 0.9, "t_offset": t, "t": [0, 2, 4, 6, 8, 10], "x": [25.0] * 6, "y": [0.0] * 6,
                    "v": [14.0] * 6, "a": [0.0] * 6, "x_std": [1.0] * 6, "y_std": [0.3] * 6, "v_std": [0.5] * 6,
                    "a_std": [0.5] * 6} for t in (0, 2, 4)],
         "pose": {"vx": 15.0, "vy": 0.0, "vz": 0.0, "roll_rate": 0.0, "pitch_rate": 0.0, "yaw_rate": 0.0,
                  "vx_std": 0.3, "vy_std": 0.1, "vz_std": 0.1, "roll_rate_std": 0.01, "pitch_rate_std": 0.01,
                  "yaw_rate_std": 0.01},
         "road_transform": [0, 0, 1.2, 0, 0, 0], "road_transform_std": [0.1] * 6,
         "desired_curvature": None, "desired_curvature_std": None, "desire_state": None,
         "desire_pred": [[1.0] + [0.0] * 7] * 4,
         "meta": {"engaged_prob": 0.9, "disengage_probs": {}, "blinkers": {"left": [0] * 6, "right": [0] * 6},
                  "hard_brake_pred": [0] * 5, "fcw": False}}
    if plan:
        m["plan"] = {"t": [10.0 * (i / 32) ** 2 for i in range(33)], "x": X, "y": [0.0] * 33, "z": [0.0] * 33,
                     "prob": None}
    return m


class VisionRoadTest(unittest.TestCase):
    def test_pipeline_without_plan(self):
        p = Pipeline()
        types = set()
        for s in range(1, 60):
            for o in p.feed(road(s)):
                types.add(o["type"])
        self.assertTrue({"road", "lead", "lanes", "view", "fused"} <= types)
        self.assertTrue(p.latest["lead"]["present"])
        self.assertEqual(sum(1 for ln in p.latest["lanes"]["lines"] if ln["present"]), 2)
        self.assertIsNone(p.latest["view"].get("path"))           # no plan, no path object
        self.assertEqual(p.latest["view"]["road"]["vision"]["from"], "lanes")

    def test_pipeline_with_policy_plan(self):
        p = Pipeline()
        for s in range(1, 30):
            p.feed(road(s, plan=True))
        self.assertIsNotNone(p.latest["view"].get("path"))

    def test_lights_without_plan(self):
        m = {"type": "lights", "frame": [1280, 720], "lights": [
            {"track_id": 1, "state": "red", "score": 0.7, "hits": 5, "flashing": False, "box": [630, 200, 640, 225]}]}
        out = lights.annotate(m, road=road(10))   # must not need road["plan"]
        self.assertIn("lights", out)


if __name__ == "__main__":
    unittest.main()
