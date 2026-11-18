"""Load governor (api/roadsense_api/governor.py): shed order, hysteresis, the caps file, the topic marker.

    cd <repo> && python3 -m unittest api.tests.test_governor -v
"""
import json
import os
import sys
import tempfile
import unittest

REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, os.path.join(REPO, "api"))

from roadsense_api import governor as G  # noqa: E402


class Clock:
    def __init__(self):
        self.t = 1000.0

    def __call__(self):
        return self.t


def reading(temp, limit=90.0, limiter=0):
    return G.Reading([("cpu", temp, limit), ("battery", 30.0, None)], [("cpufreq", limiter, 20)])


def run(g, clk, temp, seconds, limiter=0):
    for _ in range(int(seconds)):
        clk.t += 1.0
        g.step(reading(temp, limiter=limiter))


class GovernorTest(unittest.TestCase):
    def make(self, **cfg):
        clk = Clock()
        return G.Governor(dict({"tau_s": 0}, **cfg), clock=clk, wall=clk), clk

    def test_cool_stays_full(self):
        g, clk = self.make()
        run(g, clk, 70.0, 120)
        self.assertEqual(g.level, 0)
        self.assertEqual(set(g.caps().values()), {-1})
        self.assertEqual(g.state()["headroom_k"], 20.0)
        self.assertEqual(g.state()["hottest"]["zone"], "cpu")       # the zone without a limit is ignored

    def test_sheds_in_order_one_step_per_dwell(self):
        g, clk = self.make()
        run(g, clk, 87.0, 1)                                         # headroom 3 K < 4 K
        self.assertEqual(g.shed(), ["preview"])
        run(g, clk, 87.0, 7)
        self.assertEqual(g.level, 1)                                 # dwell 8 s not over
        run(g, clk, 87.0, 1)
        self.assertEqual(g.shed(), ["preview", "recorder_frames"])
        run(g, clk, 87.0, 200)
        self.assertEqual(g.shed(), [s["what"] for s in G.DEFAULT_STEPS])
        self.assertEqual(g.shed()[-1], "road")                       # the road model is the last step
        caps = g.caps()
        self.assertEqual((caps["road_hz"], caps["env_hz"], caps["preview_hz"], caps["record_frames"]), (10, 0, 0, 0))
        self.assertEqual([e["what"] for e in g.events if e["action"] == "shed"], g.shed())

    def test_urgent_and_limiter_step_faster(self):
        g, clk = self.make()
        run(g, clk, 89.5, 10)                                        # 0.5 K: a step every 3 s
        self.assertEqual(g.level, 4)
        g, clk = self.make()
        run(g, clk, 80.0, 10, limiter=3)                             # limiter active although the average is cool
        self.assertEqual(g.level, 4)
        self.assertTrue(g.state()["limited"])

    def test_max_level_keeps_the_road_model(self):
        g, clk = self.make(max_level=7)
        run(g, clk, 95.0, 300, limiter=5)
        self.assertEqual(g.level, 7)
        self.assertEqual(g.caps()["road_hz"], -1)

    def test_restore_with_hysteresis(self):
        g, clk = self.make()
        run(g, clk, 87.0, 20)
        self.assertEqual(g.level, 3)
        run(g, clk, 84.0, 300)                                       # 6 K: between the thresholds, nothing moves
        self.assertEqual(g.level, 3)
        run(g, clk, 79.0, 29)
        self.assertEqual(g.level, 3)
        run(g, clk, 79.0, 2)
        self.assertEqual(g.level, 2)                                 # one step after 30 s above the threshold
        run(g, clk, 79.0, 29)
        self.assertEqual(g.level, 2)
        run(g, clk, 79.0, 40)
        self.assertEqual(g.level, 0)

    def test_regretted_restore_is_held_longer(self):
        g, clk = self.make()
        run(g, clk, 87.0, 1)
        run(g, clk, 79.0, 31)
        self.assertEqual(g.level, 0)
        run(g, clk, 87.0, 1)                                         # shed again within 90 s
        self.assertEqual(g.level, 1)
        run(g, clk, 79.0, 45)
        self.assertEqual(g.level, 1)                                 # hold doubled to 60 s
        run(g, clk, 79.0, 20)
        self.assertEqual(g.level, 0)

    def test_average_ignores_a_burst(self):
        g, clk = self.make(tau_s=4.0)
        run(g, clk, 70.0, 10)
        run(g, clk, 92.0, 1)                                         # one hot sample
        run(g, clk, 70.0, 10)
        self.assertEqual(g.level, 0)

    def test_no_sensor_does_nothing(self):
        g, clk = self.make()
        for _ in range(30):
            clk.t += 1
            g.step(G.Reading())
        self.assertEqual(g.level, 0)
        self.assertIsNone(g.state()["headroom_k"])

    def test_force(self):
        g, clk = self.make()
        g.force(6)
        self.assertEqual(g.caps()["lights_hz"], 2.5)
        run(g, clk, 60.0, 300)
        self.assertEqual(g.level, 6)                                 # held
        g.force(None)
        run(g, clk, 60.0, 400)
        self.assertEqual(g.level, 0)

    def test_file_line_and_probe(self):
        g, clk = self.make()
        run(g, clk, 87.0, 20)
        d = json.loads(g.file_line())
        self.assertEqual(d["level"], 3)
        self.assertEqual(d["env_hz"], 0.2)
        self.assertEqual(d["road_hz"], -1)
        with tempfile.TemporaryDirectory() as tmp:
            p = os.path.join(tmp, "governor")
            self.assertTrue(G.write_caps(p, g.file_line()))
            self.assertEqual(json.load(open(p))["level"], 3)
            r = os.path.join(tmp, "reading.json")
            json.dump({"zones": [{"name": "soc", "temp_c": 88.0, "limit_c": 90}],
                       "limiter": [{"name": "x", "state": 2, "max": 5}]}, open(r, "w"))
            rd = G.make_probe({"probe": "file:" + r}).read()
            self.assertEqual(rd.zones, [("soc", 88.0, 90)])
            self.assertTrue(rd.limited)
            # sysfs layout
            z = os.path.join(tmp, "thermal_zone0")
            c = os.path.join(tmp, "cooling_device0")
            os.makedirs(z), os.makedirs(c)
            for name, val in (("type", "cpu4-thermal"), ("temp", "81500"), ("trip_point_0_type", "passive"),
                              ("trip_point_0_temp", "90000"), ("trip_point_1_type", "critical"),
                              ("trip_point_1_temp", "110000")):
                open(os.path.join(z, name), "w").write(val + "\n")
            for name, val in (("type", "cpufreq-cpu4"), ("cur_state", "0"), ("max_state", "23")):
                open(os.path.join(c, name), "w").write(val + "\n")
            rd = G.SysfsThermalProbe(root=tmp).read()
            self.assertEqual(rd.zones, [("cpu4-thermal", 81.5, 90.0)])
            self.assertFalse(rd.limited)

    def test_topic_status(self):
        g, clk = self.make()
        s = g.topic_status("road", 13.8, 110.0)
        self.assertEqual((s["degraded"], s["why"], s["level"]), (False, [], 0))
        self.assertTrue(g.topic_status("road", 8.0, 110.0)["degraded"])
        self.assertEqual(g.topic_status("lead", 13.0, 300.0)["why"], ["late"])
        g.force(6)
        self.assertEqual(g.topic_status("lights", 2.5, 150.0)["why"], ["shed", "slow"])
        self.assertFalse(g.topic_status("road", 14.0, 110.0)["degraded"])
        g.force(8)
        self.assertEqual(g.topic_status("road", 10.0, 110.0)["why"], ["shed"])
        g.force(None)
        g.level = 0
        g.limited = True
        s = g.topic_status("road", 14.0, 110.0)                      # limiter active, the topic still holds its floor
        self.assertEqual((s["degraded"], s["why"]), (False, ["limited"]))


if __name__ == "__main__":
    unittest.main()
