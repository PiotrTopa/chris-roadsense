"""Junction view (context.junction_view) on a hand-made network: a roundabout with four arms followed by a
signal-controlled crossroads.

            North (tertiary)                Cross (secondary)
               |                               |
    Main ---( ring r = 20 m )--- Main ---------+--------- Main      (primary DK1, the car drives east)
               |                               |  signals
            South (residential)             Cross
"""
import math
import os
import sys
import tempfile
import types
import unittest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from chris_mapper import context, rsmap as R       # noqa: E402
from chris_mapper import synthpack as S                     # noqa: E402

FT = R.FEATURE_ID


def _arc(a0, a1, r=20.0, step=15):
    """Points on the ring from angle a0 to a1 (degrees, counter-clockwise, 0 = east)."""
    n = max(1, int(abs(a1 - a0) / step))
    return [(r * math.cos(math.radians(a0 + (a1 - a0) * k / n)), r * math.sin(math.radians(a0 + (a1 - a0) * k / n)))
            for k in range(n + 1)]


def edges():
    main = dict(name="Main", ref="DK1", lanes=2)
    ring = dict(flags=R.F_ONEWAY | R.F_ROUNDABOUT, name="Rondo")
    return [
        S._edge("w0", "rw", [(-300, 0), (-150, 0), (-20, 0)], "primary", **main),
        S._edge("rw", "rs", _arc(180, 270), "primary", **ring),
        S._edge("rs", "re", _arc(270, 360), "primary", **ring),
        S._edge("re", "rn", _arc(0, 90), "primary", **ring),
        S._edge("rn", "rw", _arc(90, 180), "primary", **ring),
        S._edge("rs", "s1", [(0, -20), (0, -200)], "residential", name="South"),
        S._edge("rn", "n1", [(0, 20), (0, 200)], "tertiary", name="North"),
        S._edge("re", "c", [(20, 0), (150, 0), (300, 0)], "primary",
                feats=[(2, FT["traffic_signals"], R.DIR_BOTH)], **main),
        S._edge("c", "e1", [(300, 0), (600, 0)], "primary", **main),
        S._edge("cs", "c", [(300, -200), (300, 0)], "secondary", name="Cross"),
        S._edge("c", "cn", [(300, 0), (300, 200)], "secondary", name="Cross"),
        S._edge("c", "yard", [(300, 0), (340, 40)], "service", flags=R.F_ONEWAY, name="Out"),   # leaves only
    ]


class JunctionView(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.tmp = tempfile.mkdtemp(prefix="rs-jv-")
        path = os.path.join(cls.tmp, "jv.rsmap")
        R.build_pack(path, edges(), {"region": "synthetic", "osm_timestamp": "", "built": "test",
                                     "attribution": "hand-made test network"})
        cls.pack = R.Pack(path)
        cls.limits = context.load_limits("pl")

    def at(self, x, y, heading, memo=None):
        lat, lon = S.ll(x, y)
        best = None
        for e in self.pack.query(lat, lon, 30.0):
            d, s, brg = e.project(lat, lon)
            for direction in ((1,) if e.oneway else (1, -1)):
                b = brg if direction > 0 else (brg + 180.0) % 360.0
                sc = d + abs(context.geo.ang_diff(heading, b))
                if best is None or sc < best[0]:
                    best = (sc, e, direction, s, b)
        _, e, direction, s, b = best
        m = types.SimpleNamespace(edge=e, dir=direction, s=s, lat=lat, lon=lon, bearing=b)
        self.ahead = context.look_ahead(self.pack, m, self.limits, memo=memo)
        return self.ahead["junction_view"]

    def test_geometry_roundabout_and_side_roads(self):
        self.at(-200, 0, 90.0)
        g = self.ahead["geometry"]
        p = g["path"]
        self.assertEqual(p["s"][:3], [0.0, 4.0, 8.0])
        self.assertAlmostEqual(p["x"][10], 40.0, delta=0.2)             # straight towards the ring
        self.assertAlmostEqual(p["y"][10], 0.0, delta=0.2)
        ring = g["ring"]
        self.assertAlmostEqual(ring["centre"][0], 200.0, delta=1.5)
        self.assertAlmostEqual(ring["centre"][1], 0.0, delta=1.5)
        self.assertAlmostEqual(ring["radius_m"], 20.0, delta=1.5)
        self.assertEqual((ring["on_ring"], ring["entry_m"]), (False, 180.0))
        self.assertAlmostEqual(ring["exit_m"], 180.0 + math.pi * 20.0, delta=2.0)     # half way round
        arms = [b for b in g["branches"] if b["ring"]]
        self.assertEqual([b["class"] for b in arms], ["residential", "primary", "tertiary", "primary"])   # driving order
        south = arms[0]
        self.assertAlmostEqual(south["pts"][0][0], 200.0, delta=1.0)    # leaves the ring at its south point,
        self.assertAlmostEqual(south["pts"][0][1], 20.0, delta=1.0)     # y right: south is to the right of east
        self.assertGreater(south["pts"][-1][1], 60.0)
        self.assertEqual([b["path"] for b in arms], [False, True, False, True])
        self.assertIsNotNone(south["dist_m"])
        self.assertIsNone(arms[2]["dist_m"])                            # beyond the path's exit
        # the path goes round the ring: its points lie on the circle between entry and exit
        on = [(x, y) for s, x, y in zip(p["s"], p["x"], p["y"]) if ring["entry_m"] + 4 < s < ring["exit_m"] - 4]
        self.assertTrue(on and all(abs(math.hypot(x - 200.0, y) - 20.0) < 1.5 and y > 0 for x, y in on))
        # the crossroads behind it, 120 m further: beyond GEOM_BRANCH_M from here, listed from closer by
        self.at(100, 0, 90.0)
        g = self.ahead["geometry"]
        self.assertIsNone(g["ring"])
        side = sorted((b["class"], round(b["pts"][-1][0]), round(b["pts"][-1][1])) for b in g["branches"])
        self.assertEqual(side, [("secondary", 200, -45), ("secondary", 200, 45), ("service", 232, -32)])
        self.assertTrue(all(b["dist_m"] == 200.0 for b in g["branches"]))
        self.assertEqual(g["lanes"][0]["forward"], 1)

    def test_on_the_ring_the_path_takes_the_announced_exit(self):
        memo = {}
        self.at(-200, 0, 90.0, memo)
        self.at(20 * math.cos(math.radians(225)), 20 * math.sin(math.radians(225)), 135.0, memo)
        g = self.ahead["geometry"]
        self.assertGreater(g["path"]["s"][-1], 300.0)                   # out of the ring and on along Main
        self.assertAlmostEqual(g["ring"]["exit_m"], 20.0 * math.radians(135.0), delta=2.0)
        self.assertEqual([j["kind"] for j in self.ahead["junctions"]][:2], ["roundabout_exit", "junction"])

    def test_geometry_on_the_ring(self):
        self.at(20 * math.cos(math.radians(225)), 20 * math.sin(math.radians(225)), 135.0, {})
        g = self.ahead["geometry"]
        self.assertTrue(g["ring"]["on_ring"])
        self.assertIsNone(g["ring"]["entry_m"])
        self.assertAlmostEqual(math.hypot(*g["ring"]["centre"]), 20.0, delta=1.5)
        self.assertLess(g["ring"]["centre"][1], 0.0)                    # the island is on the left
        self.assertEqual(len([b for b in g["branches"] if b["ring"]]), 4)

    def test_roundabout_ahead(self):
        memo = {}
        jv = self.at(-200, 0, 90.0, memo)
        self.assertEqual(jv["kind"], "roundabout")
        self.assertAlmostEqual(jv["dist_m"], 180.0, delta=1.0)
        self.assertEqual(jv["control"], "give_way")
        self.assertEqual(jv["approach"]["name"], "Main")
        self.assertAlmostEqual(jv["ring"]["radius_m"], 20.0, delta=1.5)
        self.assertEqual((jv["ring"]["exits"], jv["ring"]["path_exit_no"], jv["ring"]["on_ring"]), (3, 2, False))
        arms = {a["name"]: a for a in jv["arms"]}
        self.assertAlmostEqual(arms["South"]["angle_deg"], 90.0, delta=3)       # first exit, to the right
        self.assertEqual(arms["South"]["exit_no"], 1)
        self.assertFalse(arms["South"]["path"])
        self.assertEqual(arms["North"]["exit_no"], 3)
        self.assertAlmostEqual(arms["North"]["angle_deg"], -90.0, delta=3)
        main = [a for a in jv["arms"] if a["name"] == "Main"]
        ahead = next(a for a in main if a["path"])
        self.assertAlmostEqual(ahead["angle_deg"], 0.0, delta=3)                # straight on: the same road number
        self.assertEqual(ahead["exit_no"], 2)
        back = next(a for a in main if not a["path"])
        self.assertTrue(back["approach"])
        self.assertAlmostEqual(abs(back["angle_deg"]), 180.0, delta=3)
        # on the ring: the same picture, the car's place on it, no distance
        on = self.at(20 * math.cos(math.radians(225)), 20 * math.sin(math.radians(225)), 135.0, memo)
        self.assertTrue(on["ring"]["on_ring"])
        self.assertEqual(on["dist_m"], 0.0)
        self.assertEqual([(a["name"], a["angle_deg"], a.get("exit_no"), a["path"]) for a in on["arms"]],
                         [(a["name"], a["angle_deg"], a.get("exit_no"), a["path"]) for a in jv["arms"]])
        self.assertAlmostEqual(on["ring"]["car_angle_deg"], 135.0, delta=4)     # between the entry (180) and South (90)
        self.assertEqual(on["ring"]["path_exit_no"], 2)

    def test_on_a_ring_without_an_announcement(self):
        on = self.at(20 * math.cos(math.radians(225)), 20 * math.sin(math.radians(225)), 135.0, {})
        self.assertEqual(on["kind"], "roundabout")
        self.assertTrue(on["ring"]["on_ring"])
        self.assertAlmostEqual(abs(on["ring"]["car_angle_deg"]), 180.0, delta=1)   # no entry known: the car is "below"
        self.assertEqual(on["ring"]["exits"], 4)

    def test_crossroads_with_signals(self):
        jv = self.at(100, 0, 90.0)
        self.assertEqual(jv["kind"], "junction")
        self.assertAlmostEqual(jv["dist_m"], 200.0, delta=1.0)
        self.assertEqual(jv["control"], "signals")
        self.assertEqual(jv["approach"]["lanes"], 1)
        self.assertEqual(jv["approach"]["lanes_source"], "tagged_total")
        self.assertIsNone(jv["ring"])
        by = {round(a["angle_deg"] / 45.0) * 45: a for a in jv["arms"]}
        self.assertEqual(sorted(by), [-90, -45, 0, 90])
        self.assertTrue(by[0]["path"] and by[0]["name"] == "Main")
        self.assertEqual((by[-90]["name"], by[-90]["exit"], by[-90]["entry"]), ("Cross", True, True))
        self.assertTrue(by[-45]["minor"] and by[-45]["exit"] and not by[-45]["entry"])   # one-way service road out, half left
        self.assertEqual([a["angle_deg"] for a in jv["arms"]], sorted(a["angle_deg"] for a in jv["arms"]))

    def test_none_beyond_the_view_distance_and_at_a_dead_end(self):
        self.assertIsNone(self.at(450, 0, 90.0))             # past the crossroads: the road just ends
        lat, lon = S.ll(-200, 0)
        e = min(self.pack.query(lat, lon, 30.0), key=lambda e: e.project(lat, lon)[0])
        d, s, brg = e.project(lat, lon)
        m = types.SimpleNamespace(edge=e, dir=1, s=s, lat=lat, lon=lon, bearing=brg)
        segs, _ = context.most_likely_path(self.pack, e, 1, s, 1500.0)
        self.assertIsNone(context.junction_view(self.pack, segs, [], m, max_m=100.0))


if __name__ == "__main__":
    unittest.main()
