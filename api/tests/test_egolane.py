"""Ego-lane filter (egolane.py) and its use in fused.lanes (lane L20)."""
import os
import sys
import unittest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from roadsense_api.egolane import EgoLaneEstimator      # noqa: E402
from roadsense_api.fusion import Fusion                  # noqa: E402

NS = 10 ** 9
X = [float(x) for x in (0, 2, 4, 6, 8, 10, 12, 14, 16, 18, 20, 25, 30, 40, 50)]


def line(y, conf=0.9, present=True):
    return {"present": present, "conf": conf, "y": [y] * len(X)} if present else {"present": False, "conf": 0.0, "y": None}


def lanes_msg(ts, lanes_right, lanes_left, w=3.5, far_r=True, far_l=True, edges=True):
    """A tracked frame on a road with `lanes_right` lanes to the right and `lanes_left` to the left of the car."""
    yl, yr = -w / 2, w / 2
    gap = lambda k: 2.5 + 1.8 * k * (w / 3.4)          # noqa: E731 - the model's edge placement (egolane.py)
    return {"type": "lanes", "ts_mono_ns": ts, "x": X, "lane_width_m": w,
            "lines": [line(yl - w, present=far_l and lanes_left > 0), line(yl), line(yr),
                      line(yr + w, present=far_r and lanes_right > 0)],
            "edges": [line(yl - gap(lanes_left), present=edges), line(yr + gap(lanes_right), present=edges)],
            "ego": {"speed_mps": 15.0}}


def road_msg(ts, centre=0.0, w=3.5):
    ys = lambda y: [y] * len(X)                          # noqa: E731
    return {"type": "road", "ts_mono_ns": ts, "lane_lines": [
        {"x": X, "y": ys(centre - 1.5 * w), "prob": 0.5}, {"x": X, "y": ys(centre - w / 2), "prob": 0.9},
        {"x": X, "y": ys(centre + w / 2), "prob": 0.9}, {"x": X, "y": ys(centre + 1.5 * w), "prob": 0.5}]}


def map_msg(ts, fwd, bwd=0, source="tagged"):
    return {"type": "map", "ts_mono_ns": ts, "available": True, "match": {"confidence": 0.9},
            "road": {"oneway": bwd == 0, "lanes": {"total": fwd + bwd, "forward": fwd, "backward": bwd, "source": source}}}


def drive(est, n_lanes, ego_from_left, seconds, t0=0, hz=10, **kw):
    ts = t0
    out = None
    for i in range(int(seconds * hz)):
        ts = t0 + int(i * NS / hz)
        est.on_map(map_msg(ts, n_lanes) if n_lanes else None)
        out = est.on_lanes(lanes_msg(ts, n_lanes - 1 - ego_from_left if n_lanes else 1, ego_from_left, **kw),
                           road_msg(ts))
    return out, ts


class EgoLane(unittest.TestCase):
    def test_three_lanes_each_position(self):
        for ego in range(3):
            out, _ = drive(EgoLaneEstimator(), 3, ego, 3.0)
            self.assertEqual(out["ego_index"], ego, out)
            self.assertEqual(len(out["probs"]), 3)
            self.assertAlmostEqual(sum(out["probs"]), 1.0, places=2)
            self.assertIn("map:tagged", out["provenance"])

    def test_unknown_without_evidence(self):
        """No lines, no edges: the answer is 'unknown', never a guess."""
        est = EgoLaneEstimator()
        for i in range(30):
            ts = i * NS // 10
            est.on_map(map_msg(ts, 3))
            m = lanes_msg(ts, 1, 1, edges=False)
            m["lines"] = [line(0, present=False)] * 4
            out = est.on_lanes(m, None)
        self.assertIsNone(out["ego_index"])

    def test_missing_far_line_does_not_commit_to_the_rightmost_lane(self):
        """The middle of three lanes with the far-right line unseen (worn paint, a bus lane): absence alone must
        not make a confident 'rightmost lane'."""
        est = EgoLaneEstimator()
        out, ts = drive(est, 3, 1, 2.0)
        self.assertEqual(out["ego_index"], 1)
        for i in range(1, 60):
            t = ts + i * NS // 10
            est.on_map(map_msg(t, 3))
            out = est.on_lanes(lanes_msg(t, 1, 1, far_r=False, edges=False), road_msg(t))
        self.assertNotEqual(out["ego_index"], 2)

    def test_lane_change_moves_the_estimate(self):
        est = EgoLaneEstimator()
        out, ts = drive(est, 3, 2, 2.0)                 # rightmost
        self.assertEqual(out["ego_index"], 2)
        # a change to the left: the raw ego centre jumps by a lane width (lines move right in the car's frame)
        t = ts + NS // 10
        est.on_map(map_msg(t, 3))
        out = est.on_lanes(lanes_msg(t, 1, 1, far_r=False, far_l=False, edges=False), road_msg(t, centre=3.5))
        self.assertEqual(out["last_change"]["dir"], "left")
        self.assertEqual(out["ego_index"], 1)

    def test_turn_signal_strengthens_a_change(self):
        a, b = EgoLaneEstimator(), EgoLaneEstimator()
        b.on_ego({"turn_signal": "left"})
        for est in (a, b):
            _, ts = drive(est, 3, 2, 2.0)
            t = ts + NS // 10
            est.on_map(map_msg(t, 3))
            est.on_lanes(lanes_msg(t, 1, 1, far_r=False, far_l=False, edges=False), road_msg(t, centre=3.5))
        self.assertGreater(b.out["probs"][1], a.out["probs"][1])

    def test_without_a_map_count_index_from_the_right_only(self):
        est = EgoLaneEstimator()
        for i in range(30):
            ts = i * NS // 10
            out = est.on_lanes(lanes_msg(ts, 0, 2), road_msg(ts))
        self.assertIsNone(out["ego_index"])
        self.assertEqual(out["probs_from"], "right")
        self.assertEqual(out["ego_from_right"], 0)

    def test_motorway_shoulder_is_not_a_lane(self):
        """A 2.5-3 m hard shoulder beyond the right line: still the rightmost lane."""
        est = EgoLaneEstimator()
        for i in range(30):
            ts = i * NS // 10
            est.on_map(map_msg(ts, 2))
            m = lanes_msg(ts, 0, 1)
            m["edges"][1] = line(1.75 + 3.3)
            out = est.on_lanes(m, road_msg(ts))
        self.assertEqual(out["ego_index"], 1)


class FusedLanes(unittest.TestCase):
    def test_fused_takes_the_filter(self):
        est = EgoLaneEstimator()
        out, ts = drive(est, 3, 1, 3.0)
        m = map_msg(ts, 3)
        m.update(state="good", position={}, speed_limit=None, ahead={})
        m["road"].update({"class": "primary", "name": None, "ref": None})
        f = Fusion()._lanes(m, road_msg(ts), None, ts, out)
        self.assertEqual(f["ego_index"], 1)
        self.assertEqual(f["probs_from"], "left")
        self.assertEqual(len(f["probs"]), 3)
        self.assertGreater(f["confidence"], 0.5)

    def test_fused_unknown_stays_unknown(self):
        est = EgoLaneEstimator()
        ts = 0
        est.on_map(map_msg(ts, 3))
        m = lanes_msg(ts, 1, 1, edges=False)
        m["lines"] = [line(0, present=False)] * 4
        out = est.on_lanes(m, None)
        mm = map_msg(ts, 3)
        f = Fusion()._lanes(mm, None, None, ts, out)
        self.assertIsNone(f["ego_index"])
        self.assertEqual(f["confidence"], 0.0)


if __name__ == "__main__":
    unittest.main()
