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


if __name__ == "__main__":
    unittest.main()
