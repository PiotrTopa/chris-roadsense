"""The road part of the display channel (`view.road`, roadview.py): trust with hysteresis, the junction with its
`approximate` mark, distances carried by odometry, the lane boxes' content, turn arrows."""
import os
import sys
import unittest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from roadsense_api.roadview import RoadView, arrows      # noqa: E402
from roadsense_api.view import ViewBuilder               # noqa: E402

NS = 10 ** 9


def map_msg(t, conf=0.9, hacc=4.0, state="good", jdist=180.0, fwd=3, available=True, turn=None, kind="junction",
            jconf=0.8, synthetic=False):
    ring = None if kind == "junction" else {"radius_m": 12.0, "exits": 3, "path_exit_no": 2, "exit_dist_m": jdist + 30.0,
                                           "on_ring": False, "car_angle_deg": None}
    return {"type": "map", "ts_mono_ns": int(t * NS), "state": state, "available": available,
            "reason": None if available else "off_road", "input": {"synthetic": synthetic},
            "position": {"hacc_m": hacc}, "match": {"confidence": conf},
            "road": {"oneway": True, "lanes": {"total": fwd, "forward": fwd, "backward": 0, "source": "tagged"}},
            "ahead": {"junction_view": None if jdist is None else {
                "kind": kind, "dist_m": jdist, "confidence": jconf, "control": "signals",
                "approach": {"lanes": fwd}, "arms": [{"angle_deg": 0.0, "path": True}, {"angle_deg": 90.0}],
                "ring": ring}, "turn_lanes": turn}}


def ego_lane(probs, ego=None):
    return {"probs": probs, "probs_from": "left", "ego_index": ego, "ego_from_right": None, "confidence": max(probs),
            "provenance": ["vision", "map:tagged"]}


def run(rv, t0, t1, speed=10.0, hz=10, lane=None, **mk):
    out = None
    for i in range(int(round((t1 - t0) * hz)) + 1):
        t = t0 + i / hz
        if i % (hz // 2) == 0:
            kw = dict(mk)
            if kw.get("jdist") is not None and "jdist" in kw:
                kw["jdist"] = kw["jdist"] - speed * (t - t0)
            rv.on_map(map_msg(t, **kw))
        out = rv.on_frame(int(t * NS), speed, None, lane)
    return out


class RoadViewTest(unittest.TestCase):
    def test_without_mapd_only_the_cameras_part(self):
        out = RoadView().on_frame(NS, 10.0, {"source": "vision", "lane_width_m": 3.4, "centre": {"x": [0, 10]},
                                             "map": {"used": False, "why": "no_map"}, "branches": [], "ring": None}, None)
        self.assertIsNone(out["position"])
        self.assertIsNone(out["junction"])
        self.assertEqual(out["lanes"]["forward"], None)
        self.assertEqual(out["centre"], {"x": [0, 10]})

    def test_junction_distance_is_carried_between_map_messages(self):
        rv = RoadView()
        out = run(rv, 0.0, 2.0, jdist=180.0)
        self.assertTrue(out["position"]["trusted"])
        d0 = out["junction"]["dist_m"]
        out = rv.on_frame(int(2.3 * NS), 10.0, None, None)       # 0.3 s after the last map message, 10 m/s
        self.assertAlmostEqual(d0 - out["junction"]["dist_m"], 3.0, delta=0.6)
        self.assertEqual(out["junction"]["provenance"], "map")
        self.assertFalse(out["junction"]["approximate"])
        self.assertEqual(out["junction"]["control"], "signals")
        self.assertEqual(len(out["junction"]["arms"]), 2)

    def test_trust_has_hysteresis_and_a_hold(self):
        rv = RoadView()
        out = run(rv, 0.0, 2.0, conf=0.45, jdist=None)             # never reached trust_on
        self.assertFalse(out["position"]["trusted"])
        self.assertIsNone(out["lanes"]["forward"])
        out = run(rv, 2.5, 5.0, conf=0.9, jdist=None)
        self.assertTrue(out["position"]["trusted"])
        out = run(rv, 5.5, 6.0, conf=0.45, jdist=None)             # one doubtful second: still trusted (0.40)
        self.assertTrue(out["position"]["trusted"])
        self.assertEqual(out["lanes"]["forward"], 3)
        rv.on_map(map_msg(6.5, available=False))                   # off the road for a moment: held
        out = rv.on_frame(int(6.6 * NS), 10.0, None, None)
        self.assertTrue(out["position"]["held"] and out["position"]["approximate"])
        self.assertEqual(out["lanes"]["forward"], 3)
        out = rv.on_frame(int(7.0 * NS), 10.0, None, None)
        for k in range(1, 8):
            rv.on_map(map_msg(6.5 + 0.5 * k, available=False))
            out = rv.on_frame(int((6.6 + 0.5 * k) * NS), 10.0, None, None)
        self.assertFalse(out["position"]["trusted"])               # ... and gone after hold_s
        self.assertIsNone(out["lanes"]["forward"])

    def test_weak_position_marks_the_junction_approximate_or_withholds_it(self):
        out = run(RoadView(), 0.0, 2.0, state="dead_reckoning", jdist=200.0)
        self.assertTrue(out["junction"]["approximate"])
        self.assertIn("dead_reckoning", out["junction"]["why"])
        out = run(RoadView(), 0.0, 2.0, hacc=30.0, jdist=220.0)     # 30 m of 200: shown, approximate
        self.assertTrue(out["junction"]["approximate"])
        self.assertIn("position_accuracy", out["junction"]["why"])
        out = run(RoadView(), 0.0, 2.0, hacc=30.0, jdist=70.0)      # 30 m of 50: the error is the distance
        self.assertIsNone(out["junction"])
        out = run(RoadView(), 0.0, 2.0, hacc=12.0, jdist=60.0)      # good receiver, but 12 m of 40 m
        self.assertTrue(out["junction"]["approximate"])
        out = run(RoadView(), 0.0, 2.0, hacc=4.0, jdist=200.0, jconf=0.3)   # the path there is a guess
        self.assertIsNone(out["junction"])

    def test_lit_lane_does_not_flicker_and_likely_lane(self):
        rv = RoadView()
        out = run(rv, 0.0, 2.0, lane=ego_lane([0.05, 0.9, 0.05], 1), jdist=None)
        self.assertEqual((out["lanes"]["ego_index"], out["lanes"]["likely_index"]), (1, None))
        self.assertFalse(out["lanes"]["oncoming_left"])
        out = rv.on_frame(int(2.1 * NS), 10.0, None, ego_lane([0.1, 0.7, 0.2], None))   # under the commit threshold
        self.assertEqual(out["lanes"]["ego_index"], 1)
        out = rv.on_frame(int(2.2 * NS), 10.0, None, ego_lane([0.1, 0.5, 0.4], None))
        self.assertEqual((out["lanes"]["ego_index"], out["lanes"]["likely_index"]), (None, 1))
        out = rv.on_frame(int(2.3 * NS), 10.0, None, ego_lane([0.34, 0.33, 0.33], None))
        self.assertEqual((out["lanes"]["ego_index"], out["lanes"]["likely_index"]), (None, None))
        self.assertEqual(out["lanes"]["probs"], [0.34, 0.33, 0.33])

    def test_turn_arrows(self):
        self.assertEqual(arrows(["slight_left", "through"]), "lt")
        self.assertEqual(arrows([]), "")
        self.assertEqual(arrows(["right", "through", "sharp_right"]), "tr")
        turn = {"dist_m": 120.0, "junction_m": 200.0, "lanes": [["left"], ["through"], ["through", "right"]],
                "count": 3, "path": [False, True, True], "confidence": 0.8}
        rv = RoadView()
        out = run(rv, 0.0, 2.0, turn=turn, jdist=None)
        t = out["lanes"]["turn"]
        self.assertEqual(t["arrows"], ["l", "t", "tr"])
        self.assertFalse(t["here"])                                  # announced, 120 m ahead
        self.assertEqual(t["path"], [False, True, True])
        rv = RoadView()
        out = run(rv, 0.0, 2.0, turn=dict(turn, dist_m=0.0, junction_m=60.0), jdist=None)
        self.assertTrue(out["lanes"]["turn"]["here"])               # on the stretch, the same number of lanes

    def test_view_message_carries_the_road(self):
        vb = ViewBuilder({"road": {"trust_on": 0.5}})
        self.assertEqual(vb.road.cfg["trust_on"], 0.5)
        for k in range(5):
            vb.on_map(map_msg(k * 0.5))
            out = vb.on_frame({"type": "road", "seq": k, "ts_mono_ns": int(k * 0.5 * NS), "v_ego": 10.0}, None, None)
        self.assertEqual(out["road"]["junction"]["kind"], "junction")
        self.assertEqual(vb.summary(out)["road"]["junction"], "junction")
        # frames a second apart restart the channel; the map's trust is kept
        out = vb.on_frame({"type": "road", "seq": 9, "ts_mono_ns": int(9.0 * NS), "v_ego": 10.0}, None, None)
        self.assertIsNone(out["road"]["junction"])
        self.assertTrue(vb.road.trusted)


if __name__ == "__main__":
    unittest.main()
