"""Unit tests of the ego-light selection (roadsense_api/lights.py).

    cd <repo> && python3 -m unittest discover -s api/tests -v
"""
import os
import sys
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))
from roadsense_api import lights  # noqa: E402

F, CX, CY, W, H = 690.0, 640.0, 360.0, 1280, 720
X = [192.0 * (i / 32) ** 2 for i in range(33)]


def road(curve=0.0):
    return {"type": "road", "plan": {"x": X, "y": [curve * x * x for x in X], "z": [0.0] * 33}}


def light(tid, cx, top, h, state="red", score=0.7, hits=5):
    return {"track_id": tid, "state": state, "score": score, "hits": hits, "flashing": False,
            "box": [cx - 0.4 * h, top, cx + 0.4 * h, top + h]}


def msg(*ls):
    return {"type": "lights", "frame": [W, H], "lights": list(ls)}


class EgoTest(unittest.TestCase):
    def ann(self, m, r=None):
        return lights.annotate(m, r if r is not None else road(), {"pitch": 0, "yaw": 0, "roll": 0, "height_m": 1.2},
                               (F, CX, CY, W, H))

    def test_distance_and_lateral(self):
        m = self.ann(msg(light(1, CX + 69, 200, 31)))      # 0.9 m housing at 31 px -> 20 m; 69 px right -> 2 m
        li = m["lights"][0]
        self.assertAlmostEqual(li["dist_m"], 20.0, delta=0.2)
        self.assertAlmostEqual(li["lateral_m"], 2.0, delta=0.2)
        self.assertTrue(li["ego"])
        self.assertEqual(m["ego"]["state"], "red")

    def test_side_street_light_not_ego(self):
        # a green light 15 m to the left (cross street) vs a red one over the lane
        m = self.ann(msg(light(1, CX - 500, 250, 31, "green"), light(2, CX + 30, 220, 31, "red")))
        self.assertEqual(m["ego"]["track_id"], 2)
        self.assertEqual(m["ego"]["state"], "red")
        self.assertEqual(m["lights"][0]["relevance"], 0.0)

    def test_back_view_ignored(self):
        m = self.ann(msg(light(1, CX + 20, 220, 31, "off")))
        self.assertLess(m["lights"][0]["relevance"], lights.MIN_RELEVANCE)
        self.assertIsNone(m["ego"])

    def test_group_vote(self):
        # two heads of one signal group (pole right, overhead) agree on green; a weak far one says red
        m = self.ann(msg(light(1, CX + 120, 230, 40, "green"), light(2, CX + 10, 150, 36, "green"),
                         light(3, CX + 60, 300, 9, "red", score=0.3, hits=2)))
        self.assertEqual(m["ego"]["state"], "green")
        self.assertEqual(m["ego"]["agree"], 2)

    def test_follows_curved_path(self):
        # path bends right: a light right of the image centre at the bend is ahead of the car
        r = road(curve=0.004)
        m = self.ann(msg(light(1, CX + 150, 230, 31, "red"), light(2, CX - 150, 230, 31, "green")), r)
        self.assertEqual(m["ego"]["track_id"], 1)

    def test_no_road(self):
        m = lights.annotate(msg(light(1, CX + 30, 220, 31, "yellow")), None, None, (F, CX, CY, W, H))
        self.assertEqual(m["ego"]["source"], "image")
        self.assertEqual(m["ego"]["state"], "yellow")


class EgoOverTimeTest(unittest.TestCase):
    """EgoLightSelector: the ego light does not blink when a frame misses it or its relevance dips."""
    CAL = {"pitch": 0, "yaw": 0, "roll": 0, "height_m": 1.2}

    def step(self, sel, t, *ls):
        m = msg(*ls)
        m["ts_mono_ns"] = int(t * 1e9)
        return sel.annotate(m, road(), self.CAL, (F, CX, CY, W, H))["ego"]

    def test_hold_bridges_missed_frames_and_ends(self):
        sel = lights.EgoLightSelector()
        e = self.step(sel, 10.0, light(1, CX + 30, 220, 31, "red"))
        self.assertEqual((e["track_id"], e["state"], e["held"]), (1, "red", False))
        e = self.step(sel, 10.33)                                   # the detector missed it
        self.assertEqual((e["track_id"], e["state"], e["held"]), (1, "red", True))
        self.assertAlmostEqual(e["age_s"], 0.33, places=2)
        e = self.step(sel, 10.67, light(1, CX + 31, 220, 32, "green"))
        self.assertEqual((e["state"], e["held"]), ("green", False))
        self.assertTrue(self.step(sel, 11.67)["held"])              # 1.0 s after the last sight: still held
        self.assertIsNone(self.step(sel, 12.0))                     # 1.33 s: gone
        self.assertIsNone(self.step(sel, 12.33))                    # and it does not come back by itself

    def test_hysteresis_keeps_the_current_ego_light(self):
        sel = lights.EgoLightSelector(hold_s=0.0)
        weak = dict(light(1, CX + 30, 220, 31, "red", score=0.7, hits=5))
        self.assertIsNotNone(self.step(sel, 10.0, weak))
        # the same light, far to the side for a frame (the path swings in a turn): under MIN_RELEVANCE, above KEEP
        side = light(1, CX + 290, 220, 31, "red")
        m = lights.annotate(msg(dict(side)), road(), self.CAL, (F, CX, CY, W, H))
        self.assertIsNone(m["ego"])
        self.assertGreaterEqual(m["lights"][0]["relevance"], lights.KEEP_RELEVANCE)
        e = self.step(sel, 10.33, dict(side))
        self.assertEqual((e["track_id"], e["held"]), (1, False))
        # another light with the same low relevance never becomes the ego light that way
        sel2 = lights.EgoLightSelector(hold_s=0.0)
        self.assertIsNone(self.step(sel2, 10.0, dict(side)))

    def test_coasting_light_counts_less_and_restart_resets(self):
        a = lights.annotate(msg(light(1, CX + 30, 220, 31)), road(), self.CAL, (F, CX, CY, W, H))["lights"][0]["relevance"]
        c = dict(light(1, CX + 30, 220, 31), coast=2)
        b = lights.annotate(msg(c), road(), self.CAL, (F, CX, CY, W, H))["lights"][0]["relevance"]
        self.assertAlmostEqual(b, a * lights.COAST_FACTOR ** 2, places=2)
        sel = lights.EgoLightSelector()
        self.assertIsNotNone(self.step(sel, 100.0, light(1, CX + 30, 220, 31)))
        self.assertIsNone(self.step(sel, 5.0))                      # a new ride: the clock restarted, no hold

    def test_map_signal_ahead_is_a_soft_prior(self):
        def map_msg(t, dist=80.0, **kw):
            m = {"type": "map", "ts_mono_ns": int(t * 1e9), "available": True, "match": {"confidence": 0.9},
                 "ahead": {"features": [{"kind": "crossing", "dist_m": 20.0, "confidence": 0.9},
                                        {"kind": "traffic_signals", "dist_m": dist, "confidence": 0.8}]}}
            m.update(kw)
            return m
        # a far, weak light: relevance between MAP_MIN_RELEVANCE and MIN_RELEVANCE
        weak = light(1, CX + 40, 300, 9, "red", score=0.3, hits=1)
        rel = lights.annotate(msg(dict(weak)), road(), self.CAL, (F, CX, CY, W, H))["lights"][0]["relevance"]
        self.assertTrue(lights.MAP_MIN_RELEVANCE <= rel < lights.MIN_RELEVANCE, rel)
        sel = lights.EgoLightSelector()
        self.assertIsNone(self.step(sel, 10.0, dict(weak)))                 # no map: not an ego light
        sel.on_map(map_msg(10.2))
        m = msg(dict(weak))
        m["ts_mono_ns"] = int(10.33e9)
        sel.annotate(m, road(), self.CAL, (F, CX, CY, W, H))
        self.assertEqual(m["map_signal"], {"kind": "traffic_signals", "dist_m": 80.0, "confidence": 0.8})
        self.assertEqual(m["ego"]["track_id"], 1)                           # the map expects a light: accepted
        self.assertTrue(self.step(sel, 12.2)["held"])                       # and held 2 s (map refreshed below)
        sel.on_map(map_msg(12.3))
        self.assertIsNone(self.step(sel, 12.5))
        # an unusable map changes nothing
        for bad in (map_msg(20.0, available=False), map_msg(20.0, match={"confidence": 0.3}), map_msg(20.0, dist=400.0),
                    map_msg(10.0)):                                         # the last one: stale at t = 20.3
            sel = lights.EgoLightSelector()
            sel.on_map(bad)
            m = msg(dict(weak))
            m["ts_mono_ns"] = int(20.3e9)
            sel.annotate(m, road(), self.CAL, (F, CX, CY, W, H))
            self.assertIsNone(m["map_signal"])
            self.assertIsNone(m["ego"])


if __name__ == "__main__":
    unittest.main()


L = lights


class LaneAssignment(unittest.TestCase):
    """Lane L16: which of two signal groups applies to the ego lane."""
    F = 975.0
    INTR = (975.0, 640.0, 360.0, 1280, 720)

    def light(self, tid, state, lat, d=25.0, hits=6):
        h = self.F * L.HOUSING_M / d
        u = 640.0 + lat * self.F / d
        return {"track_id": tid, "state": state, "score": 0.7, "hits": hits,
                "box": [u - h / 6, 200.0, u + h / 6, 200.0 + h]}

    def lanes(self, yl=-1.7, yr=1.7, far_left=None, far_right=None, edge_l=None, edge_r=None, ts=0):
        xs = [0.0, 10.0, 30.0, 60.0]

        def ln(y):
            return {"present": y is not None, "y": [y] * 4 if y is not None else None}
        return {"type": "lanes", "ts_mono_ns": ts, "x": xs, "lane_width_m": (yr - yl) if None not in (yl, yr) else None,
                "lines": [ln(far_left), ln(yl), ln(yr), ln(far_right)], "edges": [ln(edge_l), ln(edge_r)]}

    def run_(self, lights, lanes=None, **kw):
        msg = {"type": "lights", "frame": [1280, 720], "lights": lights}
        return L.annotate(msg, None, None, self.INTR, lanes=lanes, **kw)["ego"]

    def test_left_turn_red_beside_straight_green(self):
        # ego in the right one of two lanes: green over it, red over the left (turn) lane
        ego = self.run_([self.light(1, "red", -3.4), self.light(2, "green", 0.2)],
                        self.lanes(far_left=-5.1, edge_l=-5.4, edge_r=2.0))
        self.assertEqual(ego["state"], "green")
        self.assertTrue(ego["assign"]["conflict"])
        self.assertGreaterEqual(ego["assign"]["confidence"], L.AMB_CONFIDENCE)
        self.assertEqual(ego["assign"]["alt"]["state"], "red")
        self.assertEqual(ego["assign"]["alt"]["side"], "left")
        self.assertFalse(ego["amb"])
        self.assertEqual(ego["agree"], 1)                 # the red head does not vote

    def test_ego_in_left_turn_lane(self):
        # the same junction from the left lane: the lane lines move, the red head is now over the ego lane
        ego = self.run_([self.light(1, "red", 0.0), self.light(2, "green", 3.6)],
                        self.lanes(far_right=5.1, edge_l=-2.0, edge_r=5.4))
        self.assertEqual(ego["state"], "red")
        self.assertEqual(ego["assign"]["alt"]["side"], "right")

    def test_between_two_heads_is_ambiguous(self):
        ego = self.run_([self.light(1, "red", -1.6), self.light(2, "green", 1.6)], self.lanes())
        self.assertTrue(ego["assign"]["conflict"])
        self.assertLess(ego["assign"]["confidence"], L.AMB_CONFIDENCE)
        self.assertTrue(ego["amb"])

    def test_right_post_counts_for_the_rightmost_lane(self):
        # ego in the rightmost lane: the post at the roadside (green) against a head two lanes to the left (red)
        ego = self.run_([self.light(1, "red", -6.5), self.light(2, "green", 4.5)],
                        self.lanes(far_left=-5.1, edge_l=-8.6, edge_r=2.0))
        self.assertEqual(ego["state"], "green")
        self.assertGreater(ego["assign"]["confidence"], 0.8)

    def test_no_lane_lines_uses_the_path(self):
        ego = self.run_([self.light(1, "red", -5.0), self.light(2, "green", 0.5)])
        self.assertEqual(ego["state"], "green")
        self.assertEqual(ego["assign"]["basis"], "path")

    def test_agreeing_heads_are_no_conflict(self):
        ego = self.run_([self.light(1, "green", -3.4), self.light(2, "green", 0.2)], self.lanes())
        self.assertFalse(ego["assign"]["conflict"])
        self.assertIsNone(ego["assign"]["alt"])
        self.assertEqual(ego["agree"], 2)

    def test_lone_light_over_another_lane_is_served_and_marked(self):
        # (with the map expecting a signal the window lets a light this far to the left through)
        ego = self.run_([self.light(1, "red", -4.5, d=20.0)],
                        self.lanes(yl=-1.4, yr=1.4, far_left=-4.2, edge_l=-7.2),
                        min_relevance=L.MAP_MIN_RELEVANCE)
        self.assertEqual(ego["state"], "red")             # never removed
        self.assertTrue(ego["assign"].get("other_lane"))
        self.assertTrue(ego["amb"])

    def test_switched_off_is_the_old_selection(self):
        ego = self.run_([self.light(1, "red", -3.4), self.light(2, "green", 0.2)], self.lanes(), lane_assign=False)
        self.assertNotIn("assign", ego)
        self.assertNotIn("slot", ego)

    def test_map_lane_count_caps_the_right_side(self):
        fr = L.lane_frame(self.lanes(far_right=5.1, edge_r=8.6), None, 20.0, lanes_forward=1)
        self.assertEqual(fr["n_right"], 0)
        self.assertEqual(L.lane_frame(self.lanes(far_right=5.1, edge_r=8.6), None, 20.0)["n_right"], 2)

    def test_selector_reports_ambiguity_once_it_lasts(self):
        sel = L.EgoLightSelector()
        out = []
        for k in range(3):
            sel.on_lanes(self.lanes(ts=int(k * 0.33e9)))
            msg = {"type": "lights", "ts_mono_ns": int(k * 0.33e9), "frame": [1280, 720],
                   "lights": [self.light(1, "red", -1.6), self.light(2, "green", 1.6)]}
            out.append(sel.annotate(msg, None, None, self.INTR)["ego"]["amb"])
        self.assertEqual(out, [False, True, True])
