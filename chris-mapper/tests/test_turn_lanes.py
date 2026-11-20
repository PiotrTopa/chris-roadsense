"""Pack format 2: turn lanes (`turn:lanes`) in the pack, in `road`, `ahead.turn_lanes` and the junction view;
a format 1 pack is still read (no turn lanes anywhere); an unknown format is refused.

    Main (primary, eastwards) ---- 3 lanes: left | through | through;right ----+---- Main
                                                                               |  Cross (north = left, south = right)
"""
import os
import struct
import sys
import tempfile
import types
import unittest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from chris_mapper import context, rsmap as R       # noqa: E402
from chris_mapper import synthpack as S                     # noqa: E402

MAN = {"region": "synthetic", "osm_timestamp": "", "built": "test", "attribution": "hand-made test network"}


def edges():
    main = dict(name="Main", ref="DK1")
    return [
        S._edge("w0", "w1", [(-600, 0), (-150, 0)], "primary", lanes=2, **main),
        S._edge("w1", "c", [(-150, 0), (0, 0)], "primary", lanes=4, lanes_fwd=3, lanes_bwd=1, flags2=R.F2_LANES_SPLIT,
                turn_fwd="left|through|through;right", **main),
        S._edge("c", "e1", [(0, 0), (400, 0)], "primary", lanes=2, **main),
        S._edge("c", "n1", [(0, 0), (0, 300)], "secondary", name="Cross"),
        S._edge("s1", "c", [(0, -300), (0, 0)], "secondary", name="Cross", turn_bwd="none|none"),
        S._edge("q0", "q1", [(0, 1000), (300, 1000)], "tertiary", lanes=3, lanes_fwd=2, lanes_bwd=1,
                flags2=R.F2_LANES_SPLIT, turn_fwd="left|through|right"),          # three arrows on two lanes
    ]


def ahead(pack, x, y, heading):
    lat, lon = S.ll(x, y)
    best = None
    for e in pack.query(lat, lon, 30.0):
        d, s, brg = e.project(lat, lon)
        for direction in ((1,) if e.oneway else (1, -1)):
            b = brg if direction > 0 else (brg + 180.0) % 360.0
            sc = d + abs(context.geo.ang_diff(heading, b))
            if best is None or sc < best[0]:
                best = (sc, e, direction, s, b)
    _, e, direction, s, b = best
    m = types.SimpleNamespace(edge=e, dir=direction, s=s, lat=lat, lon=lon, bearing=b)
    return context.road_info(e, direction), context.look_ahead(pack, m, context.load_limits("pl"))


class TurnLanes(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.tmp = tempfile.mkdtemp(prefix="rs-tl-")
        cls.p2, cls.p1 = os.path.join(cls.tmp, "v2.rsmap"), os.path.join(cls.tmp, "v1.rsmap")
        R.build_pack(cls.p2, edges(), MAN)
        R.build_pack(cls.p1, edges(), MAN, version=1)

    def test_announced_before_the_stretch_and_on_it(self):
        pack = R.Pack(self.p2)
        self.assertEqual(pack.manifest["format"], 2)
        road, a = ahead(pack, -400, 0, 90.0)
        self.assertIsNone(road["turn_lanes"])
        tl = a["turn_lanes"]
        self.assertEqual(tl["lanes"], [["left"], ["through"], ["through", "right"]])
        self.assertAlmostEqual(tl["dist_m"], 250.0, delta=2.0)
        self.assertAlmostEqual(tl["junction_m"], 400.0, delta=2.0)
        self.assertEqual(tl["path"], [False, True, True])          # the most likely path goes straight on
        self.assertEqual(a["junction_view"]["approach"]["turn_lanes"], tl["lanes"])
        road, a = ahead(pack, -60, 0, 90.0)
        self.assertEqual(road["turn_lanes"], tl["lanes"])
        self.assertEqual(road["lanes"]["forward"], 3)
        self.assertEqual(a["turn_lanes"]["dist_m"], 0.0)
        self.assertAlmostEqual(a["turn_lanes"]["junction_m"], 60.0, delta=2.0)
        road, a = ahead(pack, 200, 0, 90.0)
        self.assertIsNone(a["turn_lanes"])
        pack.close()

    def test_other_direction_and_empty_arrows(self):
        pack = R.Pack(self.p2)
        road, _ = ahead(pack, -60, 0, 270.0)                        # westwards on the same edge: nothing tagged
        self.assertIsNone(road["turn_lanes"])
        road, _ = ahead(pack, 0, -100, 180.0)                       # "none|none": no arrow painted = not tagged
        self.assertIsNone(road["turn_lanes"])
        road, _ = ahead(pack, 100, 1000, 90.0)                      # arrows that contradict the lane count
        self.assertIsNone(road["turn_lanes"])
        pack.close()

    def test_lanes_for_turn(self):
        tl = [["left"], ["through"], ["through", "right"]]
        self.assertEqual(context.lanes_for_turn(tl, -88.0), [True, False, False])
        self.assertEqual(context.lanes_for_turn(tl, 92.0), [False, False, True])
        self.assertEqual(context.lanes_for_turn([[], ["right"]], 3.0), [True, False])
        self.assertIsNone(context.lanes_for_turn([["through"], ["through"]], 90.0))

    def test_format_1_is_read_without_turn_lanes(self):
        pack = R.Pack(self.p1)
        self.assertEqual((pack.version, pack.manifest["format"]), (1, 1))
        road, a = ahead(pack, -60, 0, 90.0)
        self.assertIsNone(road["turn_lanes"])
        self.assertIsNone(a["turn_lanes"])
        self.assertEqual(road["lanes"]["forward"], 3)
        self.assertEqual(a["junction_view"]["kind"], "junction")
        pack.close()

    def test_unknown_format_is_refused(self):
        bad = os.path.join(self.tmp, "v9.rsmap")
        raw = bytearray(open(self.p2, "rb").read())
        struct.pack_into("<I", raw, 8, 9)
        open(bad, "wb").write(raw)
        with self.assertRaisesRegex(ValueError, "pack format 9"):
            R.Pack(bad)


if __name__ == "__main__":
    unittest.main()
