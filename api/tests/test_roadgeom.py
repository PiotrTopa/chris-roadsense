"""Fused road geometry (roadgeom.py) on hand-made lanes and map messages."""
import math
import os
import sys
import unittest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from roadsense_api.roadgeom import RoadGeometryFusion     # noqa: E402

X = [192.0 * (i / 32.0) ** 2 for i in range(33)]
NS = 10 ** 9


def lanes(t, centre=lambda x: 0.0, v=15.0, present=True, std=lambda x: 0.1 + 0.004 * x):
    def line(off):
        return {"present": present, "y": [centre(x) + off for x in X] if present else None,
                "y_std": [std(x) for x in X] if present else None}
    return {"type": "lanes", "seq": int(t * 10), "ts_mono_ns": int(t * NS), "ts_real_ns": 0, "x": X,
            "lines": [{"present": False}, line(-1.7), line(1.7), {"present": False}], "edges": [],
            "lane_width_m": 3.4, "ego": {"speed_mps": v, "yaw_rate": 0.0, "source": "vehicle"}}


def road_plan(t, centre=lambda x: 0.0):
    px = [i * 1.0 for i in range(33)]
    return {"type": "road", "ts_mono_ns": int(t * NS), "plan": {"x": px, "y": [centre(x) for x in px]}}


def map_msg(t, path, branches=(), ring=None, junctions=(), conf=0.9, heading=90.0, bearing=90.0, available=True,
            lane_runs=()):
    """path: f(s) -> (x, y) in the map frame."""
    s = [4.0 * i for i in range(51)] + [220.0 + 20.0 * i for i in range(20)]
    pts = [path(v) for v in s]
    return {"type": "map", "ts_mono_ns": int(t * NS), "available": available, "reason": None if available else "off_road",
            "state": "good", "input": {"synthetic": False},
            "position": {"heading_deg": heading, "hacc_m": 3.0}, "match": {"confidence": conf, "bearing_deg": bearing},
            "ahead": {"junctions": list(junctions), "junction_view": None,
                      "geometry": {"path": {"s": s, "x": [p[0] for p in pts], "y": [p[1] for p in pts]},
                                   "lanes": list(lane_runs), "branches": list(branches), "ring": ring}}}


def bend_at(s0, radius):
    """A path that is straight for s0 metres and then bends right with the given radius."""
    def f(s):
        if s <= s0:
            return (s, 0.0)
        a = (s - s0) / radius
        return (s0 + radius * math.sin(a), radius * (1.0 - math.cos(a)))
    return f


def run(f, t0, t1, centre=lambda x: 0.0, mapf=None, v=15.0, step=0.1, plan=False, **kw):
    """Feed frames from t0 to t1 (and a map message twice a second); returns the last roadgeom message."""
    g = None
    n = int(round((t1 - t0) / step))
    for i in range(n + 1):
        t = t0 + i * step
        if mapf is not None and i % 5 == 0:
            f.on_map(mapf(t))
        if plan:
            g = f.on_lanes(lanes(t, v=v, present=False), road_plan(t, centre))
        else:
            g = f.on_lanes(lanes(t, centre, v=v, **kw))
    return g


class RoadGeom(unittest.TestCase):
    def test_camera_only(self):
        g = run(RoadGeometryFusion(), 0.0, 1.0)
        self.assertEqual(g["source"], "vision")
        self.assertEqual(g["map"], {"used": False, "why": "no_map"})
        self.assertEqual(g["vision"]["from"], "lanes")
        self.assertAlmostEqual(g["vision"]["reach_m"], 99.2, delta=0.1)       # sigma 0.9 m is never reached before 100 m
        self.assertEqual(set(g["centre"]["src"]), {"v"})
        self.assertEqual((g["branches"], g["ring"]), ([], None))

    def test_camera_reach_follows_its_uncertainty_and_the_plan_is_the_fallback(self):
        g = run(RoadGeometryFusion(), 0.0, 0.5, std=lambda x: 0.1 + 0.02 * x)    # 0.64 m per line at 27 m
        self.assertAlmostEqual(g["vision"]["reach_m"], 54.2, delta=7.0)
        g = run(RoadGeometryFusion(), 0.0, 0.5, plan=True)
        self.assertEqual(g["vision"]["from"], "plan")
        self.assertLessEqual(g["vision"]["reach_m"], 32.0)

    def test_map_continues_the_camera_after_a_second_of_agreement(self):
        f = RoadGeometryFusion()
        path = bend_at(150.0, 200.0)
        mapf = lambda t: map_msg(t, lambda s: path(s + 0.0))          # noqa: E731  (the car stands for the test)
        g = run(f, 0.0, 0.6, mapf=mapf, v=0.0)
        self.assertFalse(g["map"]["used"])
        self.assertEqual(g["source"], "vision")
        g = run(f, 0.7, 1.6, mapf=mapf, v=0.0)
        self.assertTrue(g["map"]["used"])
        self.assertEqual(g["source"], "vision+map")
        c = g["centre"]
        self.assertEqual("".join(sorted(set(c["src"]))), "bmv")
        self.assertAlmostEqual(c["vision_to_m"], 99.2, delta=0.1)
        self.assertGreater(c["s"][-1], 390.0)
        i = min(range(len(c["s"])), key=lambda k: abs(c["s"][k] - 300.0))
        want = path(c["s"][i])
        self.assertAlmostEqual(c["x"][i], want[0], delta=1.5)
        self.assertAlmostEqual(c["y"][i], want[1], delta=1.5)

    def test_offset_and_heading_of_the_map_are_taken_from_the_camera(self):
        # the position's heading is 3 degrees off, the car drives 0.4 m left of its lane's centre, and the way is
        # the middle of a road with one lane each way
        f = RoadGeometryFusion()
        a = math.radians(3.0)
        br = [{"dist_m": 100.0, "pts": [[100.0 * math.cos(a), 100.0 * math.sin(a)], [100.0 * math.cos(a) - 40.0 * math.sin(a),
                                                                               100.0 * math.sin(a) + 40.0 * math.cos(a)]],
               "class": "tertiary", "minor": False, "exit": True, "entry": True, "lanes": 2, "ring": False}]
        mapf = lambda t: map_msg(t, lambda s: (s * math.cos(a), s * math.sin(a)), branches=br,     # noqa: E731
                                 lane_runs=[{"dist_m": 0.0, "forward": 1, "backward": 1, "source": "tagged"}])
        g = run(f, 0.0, 3.0, centre=lambda x: 0.4, mapf=mapf, v=0.0)
        self.assertTrue(g["map"]["used"])
        self.assertAlmostEqual(g["map"]["align"]["dy_m"], 0.4, delta=0.1)
        self.assertAlmostEqual(g["map"]["align"]["dyaw_deg"], -3.0, delta=0.3)
        self.assertLess(max(abs(y - 0.4) for y in g["centre"]["y"]), 0.6)    # the fused line is the lane centre, all 400 m
        self.assertEqual(g["map"]["way_offset_m"], 1.7)
        b = g["branches"][0]["pts"]
        self.assertAlmostEqual(b[0][0], 100.0, delta=0.5)
        self.assertAlmostEqual(b[0][1], 0.4 - 1.7, delta=0.3)               # the side road starts at the way's centre line
        self.assertAlmostEqual(b[1][0], 100.0, delta=1.0)                    # and leaves at a right angle

    def test_shapes_that_do_not_fit_withhold_the_map(self):
        f = RoadGeometryFusion()
        mapf = lambda t: map_msg(t, lambda s: (s, 0.0))               # noqa: E731
        g = run(f, 0.0, 2.0, mapf=mapf, v=0.0)
        self.assertTrue(g["map"]["used"])
        g = run(f, 2.1, 3.0, centre=lambda x: x * x / 300.0, mapf=mapf, v=0.0)     # the camera sees a bend of 150 m radius
        self.assertFalse(g["map"]["used"])
        self.assertEqual(g["map"]["why"], "disagree")
        self.assertEqual(set(g["centre"]["src"]), {"v"})
        g = run(f, 3.1, 3.6, mapf=mapf, v=0.0)
        self.assertFalse(g["map"]["used"])                             # back only after a second of agreement
        g = run(f, 3.7, 4.6, mapf=mapf, v=0.0)
        self.assertTrue(g["map"]["used"])

    def test_geometry_is_carried_by_the_distance_driven_between_map_messages(self):
        f = RoadGeometryFusion()
        br = [{"dist_m": 100.0, "pts": [[100.0, 0.0], [100.0, 40.0]], "class": "tertiary", "minor": False,
               "exit": True, "entry": True, "lanes": 2, "ring": False}]
        f.on_map(map_msg(0.0, lambda s: (s, 0.0), branches=br))
        g = None
        for i in range(0, 15):                                         # 1.4 s at 20 m/s on one map message
            g = f.on_lanes(lanes(i * 0.1, v=20.0))
        self.assertTrue(g["map"]["used"])
        self.assertAlmostEqual(g["map"]["advance_m"], 28.0, delta=0.5)
        b = g["branches"][0]
        self.assertAlmostEqual(b["dist_m"], 72.0, delta=0.5)
        self.assertAlmostEqual(b["pts"][0][0], 72.0, delta=0.5)
        self.assertEqual(b["side"], "right")
        g = f.on_lanes(lanes(1.8, v=20.0))
        self.assertFalse(g["map"]["used"])                             # 1.8 s without a map message: stale

    def test_unusable_maps_say_why(self):
        f = RoadGeometryFusion()
        f.on_map(map_msg(0.0, lambda s: (s, 0.0), available=False))
        self.assertEqual(f.on_lanes(lanes(0.0))["map"]["why"], "off_road")
        f.on_map(map_msg(0.1, lambda s: (s, 0.0), conf=0.3))
        self.assertEqual(f.on_lanes(lanes(0.1))["map"]["why"], "low_match_confidence")

    def test_roundabout_ends_the_camera_part_at_the_entry(self):
        f = RoadGeometryFusion()
        ring = {"centre": [54.0, 0.0], "radius_m": 14.0, "lanes": None, "on_ring": False, "entry_m": 40.0, "exit_m": 84.0}

        def path(s):                                                   # straight to the ring, half way round it, on
            if s <= 40.0:
                return (s, 0.0)
            if s <= 84.0:
                a = (s - 40.0) / 14.0
                return (54.0 - 14.0 * math.cos(a), 14.0 * math.sin(a))
            return (68.0 + (s - 84.0), 0.0)
        arms = [{"dist_m": 62.0, "ring_m": 62.0, "path": False, "pts": [[54.0, 14.0], [54.0, 59.0]], "class": "residential",
                 "minor": False, "exit": True, "entry": True, "lanes": 2, "ring": True},
                {"dist_m": None, "ring_m": 106.0, "path": False, "pts": [[54.0, -14.0], [54.0, -59.0]], "class": "tertiary",
                 "minor": False, "exit": True, "entry": True, "lanes": 2, "ring": True}]
        mapf = lambda t: map_msg(t, path, branches=arms, ring=ring)    # noqa: E731
        g = run(f, 0.0, 2.0, mapf=mapf, v=0.0)
        self.assertTrue(g["map"]["used"])
        self.assertAlmostEqual(g["centre"]["vision_to_m"], 37.0, delta=0.1)
        self.assertEqual(g["ring"]["centre"], [54.0, 0.0])                 # no lane count in this map message
        self.assertEqual((g["ring"]["on_ring"], g["ring"]["entry_m"], g["ring"]["exit_m"]), (False, 40.0, 84.0))
        self.assertEqual(len(g["branches"]), 2)
        self.assertEqual(g["way"]["source"], "map")
        on = [(x, y) for s, x, y in zip(g["centre"]["s"], g["centre"]["x"], g["centre"]["y"]) if 46.0 < s < 80.0]
        self.assertTrue(on and all(abs(math.hypot(x - 54.0, y) - 14.0) < 1.0 for x, y in on))
        # a right signal: the first exit; the centre line ends there
        for i in range(21, 26):
            t = i * 0.1
            f.on_ego({"type": "ego", "ts_mono_ns": int(t * NS), "speed_mps": 0.0, "yaw_rate_rps": 0.0, "turn_signal": "right"})
            f.on_map(mapf(t))
            g = f.on_lanes(lanes(t, v=0.0))
        self.assertEqual(g["way"], {"source": "intent", "turn": "right", "branch": 0})
        self.assertTrue(g["branches"][0]["way"])
        self.assertEqual(g["centre"]["end"], "intent")
        self.assertAlmostEqual(g["centre"]["s"][-1], 62.0, delta=2.5)
        # a left signal: the exit beyond the path's own, reached on round the ring
        for i in range(26, 31):
            t = i * 0.1
            f.on_ego({"type": "ego", "ts_mono_ns": int(t * NS), "speed_mps": 0.0, "yaw_rate_rps": 0.0, "turn_signal": "left"})
            f.on_map(mapf(t))
            g = f.on_lanes(lanes(t, v=0.0))
        self.assertEqual(g["way"]["branch"], 1)
        self.assertAlmostEqual(g["centre"]["x"][-1], 54.0, delta=1.5)
        self.assertAlmostEqual(g["centre"]["y"][-1], -14.0, delta=1.5)

    def test_a_car_that_turns_away_from_the_path_loses_the_map(self):
        f = RoadGeometryFusion()
        mapf = lambda t: map_msg(t, lambda s: (s, 0.0))               # noqa: E731
        g = run(f, 0.0, 2.0, mapf=mapf, v=0.0)
        self.assertTrue(g["map"]["used"])
        mapf = lambda t: map_msg(t, lambda s: (s, 0.0), heading=140.0)    # noqa: E731  50 degrees off the road
        g = run(f, 2.1, 3.0, mapf=mapf, v=0.0, present=False)
        self.assertFalse(g["map"]["used"])
        self.assertEqual(g["map"]["why"], "off_path")


if __name__ == "__main__":
    unittest.main()
