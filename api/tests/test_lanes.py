"""Road-geometry tracker (api/roadsense_api/lanes.py): synthetic roads with the noise measured on replayed rides.

    python3 -m unittest api.tests.test_lanes -v     (from the repository root)

Noise model (docs/lanes/L15-stability.md): a lane line's lateral position jitters by a few centimetres per
frame near the car and by decimetres at 60 m, the network's own y_std is about four times that; a road edge
jumps by 0.2-0.5 m per frame with a y_std of 0.5-1.4 m; the probability of a weak line wanders around the
drawing threshold.
"""
import math
import os
import random
import sys
import unittest

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))

from roadsense_api.lanes import RoadGeometryTracker, map_prior      # noqa: E402

NS = 10 ** 9
X = [192.0 * (i / 32.0) ** 2 for i in range(33)]          # the model's X_IDXS


def road(ts, probs, offsets=(-5.25, -1.75, 1.75, 5.25), edges=(-6.5, 6.5), v=14.0, yaw=0.0, curv=0.0,
         rnd=None, edge_std=0.7, seq=None, slope=0.0):
    """One `road` message: straight (or constant-curvature, or oblique) lines at the given offsets."""
    def line(y0, std0, noise):
        ys = [y0 + slope * x + 0.5 * curv * x * x + (rnd.gauss(0, noise * (1 + x / 30.0)) if rnd else 0.0) for x in X]
        return {"x": X, "y": ys, "z": [0.0] * 33, "y_std": [std0 * (1 + x / 25.0) for x in X]}
    ll = []
    for y0, p in zip(offsets, probs):
        d = line(y0, 0.25, 0.04)
        d["prob"] = p
        ll.append(d)
    return {"type": "road", "seq": seq if seq is not None else int(ts * 20), "ts_mono_ns": int(ts * NS),
            "ts_real_ns": int(ts * NS), "v_ego": v, "pose": {"yaw_rate": yaw, "vy": 0.0},
            "lane_lines": ll, "road_edges": [line(e, edge_std, 0.3) for e in edges]}


def map_msg(ts, total, fwd, bwd, source="tagged", conf=0.9, available=True):
    return {"type": "map", "ts_mono_ns": int(ts * NS), "available": available, "match": {"confidence": conf},
            "road": {"lanes": {"total": total, "forward": fwd, "backward": bwd, "source": source}}}


def changes(seq):
    return sum(1 for a, b in zip(seq, seq[1:]) if a != b)


class Presence(unittest.TestCase):
    def test_a_flickering_probability_is_one_steady_line(self):
        rnd = random.Random(3)
        trk = RoadGeometryTracker()
        raw, out = [], []
        for k in range(600):                      # 30 s at 20 Hz, the far-left line's prob wanders around 0.3
            p = min(1.0, max(0.0, 0.33 + rnd.gauss(0, 0.12)))
            o = trk.on_road(road(100 + k / 20.0, (p, 0.95, 0.95, 0.02), rnd=rnd))
            raw.append(p >= 0.3)
            out.append(o["lines"][0]["present"])
        self.assertGreater(changes(raw), 100)     # what a fixed threshold shows
        self.assertLessEqual(changes(out[40:]), 1)
        self.assertEqual(o["lines"][3]["present"], False)
        self.assertTrue(o["lines"][1]["present"] and o["lines"][2]["present"])

    def test_appears_and_disappears_within_a_second(self):
        trk = RoadGeometryTracker()
        t_on = t_off = None
        for k in range(200):
            t = k / 20.0
            p = 0.9 if 2.0 <= t < 6.0 else 0.03
            o = trk.on_road(road(100 + t, (0.9, 0.9, 0.9, p)))
            on = o["lines"][3]["present"]
            if on and t_on is None:
                t_on = t
            if t_on is not None and not on and t_off is None:
                t_off = t
        self.assertLess(t_on - 2.0, 0.5)
        self.assertLess(t_off - 6.0, 1.2)
        self.assertGreater(t_off - 6.0, 0.4)      # a dip shorter than this never hides a line

    def test_short_dropout_coasts_on_the_prediction(self):
        trk = RoadGeometryTracker()
        modes = []
        for k in range(120):
            t = k / 20.0
            p = 0.05 if 3.0 <= t < 3.4 else 0.9   # 0.4 s without the right line (a wiper, a shadow)
            o = trk.on_road(road(100 + t, (0.9, 0.9, p, 0.9)))
            modes.append((o["lines"][2]["present"], o["lines"][2]["mode"]))
        self.assertTrue(all(pr for pr, _ in modes[20:]))
        self.assertIn("coast", [m for _, m in modes[60:70]])
        self.assertAlmostEqual(np.interp(20.0, o["x"], o["lines"][2]["y"]), 1.75, delta=0.1)

    def test_road_edges_follow_the_network_uncertainty(self):
        trk = RoadGeometryTracker()
        for k in range(60):
            o = trk.on_road(road(100 + k / 20.0, (0.9,) * 4, edge_std=0.6))
        self.assertTrue(o["edges"][0]["present"] and o["edges"][1]["present"])
        for k in range(60, 160):                  # snow: the network no longer knows where the road ends
            o = trk.on_road(road(100 + k / 20.0, (0.9,) * 4, edge_std=4.0))
        self.assertFalse(o["edges"][0]["present"] or o["edges"][1]["present"])


class Geometry(unittest.TestCase):
    def run_noise(self, hz):
        rnd = random.Random(7)
        trk = RoadGeometryTracker()
        raw, out, eraw, eout = [], [], [], []
        for k in range(int(30 * hz)):
            m = road(100 + k / hz, (0.9,) * 4, rnd=rnd)
            o = trk.on_road(m)
            raw.append(np.interp(30.0, X, m["lane_lines"][1]["y"]))
            out.append(np.interp(30.0, o["x"], o["lines"][1]["y"]))
            eraw.append(np.interp(30.0, X, m["road_edges"][1]["y"]))
            eout.append(np.interp(30.0, o["x"], o["edges"][1]["y"]))
        n0 = int(3 * hz)
        return [float(np.std(v[n0:])) for v in (raw, out, eraw, eout)] + [float(np.mean(out[n0:])), float(np.mean(eout[n0:]))]

    def test_jitter_is_reduced_and_the_mean_kept(self):
        for hz in (20.0, 12.0):
            raw, out, eraw, eout, mean, emean = self.run_noise(hz)
            self.assertLess(out, 0.7 * raw)
            self.assertLess(eout, 0.25 * eraw)
            self.assertAlmostEqual(mean, -1.75, delta=0.03)
            self.assertAlmostEqual(emean, 6.5, delta=0.1)

    def test_no_lag_in_a_steady_curve(self):
        # a constant curve: the lines are static in the car frame while the car yaws (y = c x^2 / 2, yaw rate =
        # v c to the right for c > 0). True without the ego-motion compensation and with it (slide and yaw cancel).
        c, v = 1.0 / 150.0, 14.0
        for cfg in (None, dict(slide_gain=1.0, yaw_gain=-1.0)):
            trk = RoadGeometryTracker(cfg)
            for k in range(100):
                o = trk.on_road(road(100 + k / 20.0, (0.9,) * 4, v=v, yaw=v * c, curv=c))
            for xq in (10.0, 30.0, 60.0):
                self.assertAlmostEqual(np.interp(xq, o["x"], o["lines"][2]["y"]), 1.75 + 0.5 * c * xq * xq, delta=0.08)

    def test_follows_a_lateral_move_of_the_car(self):
        # the car drifts 1 m to the right in 2 s: every line moves 1 m to the left in the car frame
        trk = RoadGeometryTracker()
        worst = 0.0
        for k in range(160):
            t = k / 20.0
            shift = -min(1.0, max(0.0, (t - 3.0) / 2.0))
            offs = tuple(o + shift for o in (-5.25, -1.75, 1.75, 5.25))
            o = trk.on_road(road(100 + t, (0.9,) * 4, offsets=offs))
            if t > 1:
                worst = max(worst, abs(np.interp(10.0, o["x"], o["lines"][1]["y"]) - offs[1]))
        self.assertLess(worst, 0.15)

    def test_relabelled_lines_replace_the_track_without_values_in_between(self):
        # a lane change: the network renames its lines, every line jumps by one lane width in one frame
        trk = RoadGeometryTracker()
        seen = []
        for k in range(120):
            t = k / 20.0
            offs = (-5.25, -1.75, 1.75, 5.25) if t < 3.0 else (-1.75, 1.75, 5.25, 8.75)
            o = trk.on_road(road(100 + t, (0.9,) * 4, offsets=offs))
            seen.append((t, np.interp(10.0, o["x"], o["lines"][1]["y"])))
        for t, y in seen:
            self.assertTrue(abs(y + 1.75) < 0.2 or abs(y - 1.75) < 0.2, (t, y))
        t_new = min(t for t, y in seen if abs(y - 1.75) < 0.2)
        self.assertLess(t_new - 3.0, 0.4)
        self.assertGreaterEqual(trk.snaps, 4)

    def test_single_outlier_frame_is_ignored(self):
        trk = RoadGeometryTracker()
        for k in range(100):
            offs = (-5.25, 0.2, 1.75, 5.25) if k == 60 else (-5.25, -1.75, 1.75, 5.25)
            o = trk.on_road(road(100 + k / 20.0, (0.9,) * 4, offsets=offs))
            self.assertAlmostEqual(np.interp(10.0, o["x"], o["lines"][1]["y"]), -1.75, delta=0.05)

    def test_restart_on_a_gap_or_a_seq_going_back(self):
        trk = RoadGeometryTracker()
        for k in range(40):
            o = trk.on_road(road(100 + k / 20.0, (0.9,) * 4))
        self.assertTrue(o["lines"][1]["present"])
        o = trk.on_road(road(105.0, (0.02,) * 4))                 # 3 s later: nothing carried over
        self.assertFalse(any(ln["present"] for ln in o["lines"]))
        for k in range(40):
            o = trk.on_road(road(105.05 + k / 20.0, (0.9,) * 4))
        o = trk.on_road(road(107.1, (0.02,) * 4, seq=3))          # a new ride: the seq restarts
        self.assertFalse(any(ln["present"] for ln in o["lines"]))

    def test_messages_without_lines_or_time_are_skipped(self):
        trk = RoadGeometryTracker()
        self.assertIsNone(trk.on_road({"type": "road", "seq": 1, "ts_mono_ns": NS}))
        self.assertIsNone(trk.on_road({"type": "road", "seq": 1, "lane_lines": []}))
        m = road(100.0, (0.9,) * 4)
        m["lane_lines"][0]["y"] = [float("nan")] * 33
        o = trk.on_road(m)
        self.assertFalse(o["lines"][0]["present"])
        self.assertTrue(o["lines"][1]["present"])


class MapPrior(unittest.TestCase):
    def run_outer(self, map_fn, p_right=0.55, p_left=0.4, seconds=6.0):
        trk = RoadGeometryTracker()
        o = None
        for k in range(int(seconds * 20)):
            t = 100 + k / 20.0
            if map_fn and k % 10 == 0:
                trk.on_map(map_fn(t))
            o = trk.on_road(road(t, (p_left, 0.9, 0.9, p_right)))
        return o

    def test_without_a_map_the_thresholds_are_neutral(self):
        o = self.run_outer(None)
        self.assertIsNone(o["prior"])
        self.assertEqual(o["source"], "vision")
        self.assertTrue(o["lines"][3]["present"])       # 0.55 >= on_conf
        self.assertFalse(o["lines"][0]["present"])      # 0.40 < on_conf

    def test_one_lane_per_direction(self):
        o = self.run_outer(lambda t: map_msg(t, 2, 1, 1))
        self.assertEqual(o["source"], "vision+map")
        self.assertEqual((o["prior"]["outer_left"], o["prior"]["outer_right"]), ("expected", "unlikely"))
        self.assertFalse(o["lines"][3]["present"])      # no lane to the right: 0.55 is not enough
        self.assertTrue(o["lines"][0]["present"])       # the oncoming lane's far line: 0.40 is
        self.assertFalse(o["lane_to_right"])
        self.assertTrue(o["lane_to_left"])

    def test_strong_evidence_beats_the_prior(self):
        o = self.run_outer(lambda t: map_msg(t, 2, 1, 1), p_right=0.92)
        self.assertTrue(o["lines"][3]["present"])       # the map may be wrong or old: the camera wins when sure

    def test_unusable_map_is_no_prior(self):
        for fn in (lambda t: map_msg(t, 2, 1, 1, source="assumed"),
                   lambda t: map_msg(t, 2, 1, 1, conf=0.3),
                   lambda t: map_msg(t, 2, 1, 1, available=False),
                   lambda t: map_msg(t - 10.0, 2, 1, 1)):          # stale
            o = self.run_outer(fn)
            self.assertIsNone(o["prior"])
            self.assertTrue(o["lines"][3]["present"])

    def test_prior_table(self):
        now = 100 * NS
        self.assertIsNone(map_prior(None, now))
        p = map_prior(map_msg(100, 1, 1, 0), now)
        self.assertEqual((p["outer_left"], p["outer_right"]), ("unlikely", "unlikely"))
        p = map_prior(map_msg(100, 4, 2, 2), now)
        self.assertEqual((p["outer_left"], p["outer_right"]), ("neutral", "neutral"))
        p = map_prior(map_msg(100, 2, None, None, source="tagged_total"), now)
        self.assertEqual((p["outer_left"], p["outer_right"]), ("neutral", "neutral"))

    def test_inner_lines_and_edges_ignore_the_map(self):
        o = self.run_outer(lambda t: map_msg(t, 1, 1, 0))
        self.assertTrue(o["lines"][1]["present"] and o["lines"][2]["present"])
        self.assertTrue(o["edges"][0]["present"] and o["edges"][1]["present"])
        self.assertAlmostEqual(o["lane_width_m"], 3.5, delta=0.05)


class Message(unittest.TestCase):
    def test_shape_and_size(self):
        import json
        trk = RoadGeometryTracker()
        for k in range(20):
            m = road(100 + k / 20.0, (0.9, 0.9, 0.9, 0.02))
            o = trk.on_road(m)
        self.assertEqual((o["type"], o["seq"], o["ts_mono_ns"]), ("lanes", m["seq"], m["ts_mono_ns"]))
        self.assertEqual((len(o["x"]), len(o["lines"]), len(o["edges"])), (33, 4, 2))
        self.assertIsNone(o["lines"][3]["y"])
        self.assertEqual(len(o["lines"][1]["y"]), 33)
        self.assertLess(len(json.dumps(o, separators=(",", ":"))), 3500)
        self.assertEqual(trk.summary(o)["lines"], "|||.")

    def test_vehicle_speed_is_used_when_fresh(self):
        trk = RoadGeometryTracker()
        trk.on_ego({"type": "ego", "ts_mono_ns": 100 * NS, "speed_mps": 9.0})
        self.assertEqual(trk.on_road(road(100.02, (0.9,) * 4, v=14.0))["ego"], {"speed_mps": 9.0, "yaw_rate": 0.0, "source": "vehicle"})
        self.assertEqual(trk.on_road(road(101.0, (0.9,) * 4, v=14.0))["ego"]["source"], "vision")


if __name__ == "__main__":
    unittest.main()
