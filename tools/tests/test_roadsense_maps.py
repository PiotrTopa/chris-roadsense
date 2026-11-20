"""The pack builder (tools/roadsense_maps.py): turn-lane and feature tags into pack attributes of chris-mapper's
format 2. Moved here from chris-mapper's tests when the map box became its own repository (the builder is this
repository's tool).

    python3 -m pytest -q tools/tests
"""
import os
import sys
import unittest

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, os.path.join(ROOT, "tools"))
sys.path.insert(0, os.path.join(ROOT, "chris-mapper"))

import roadsense_maps as B                # noqa: E402
from chris_mapper import rsmap as R       # noqa: E402


class TestBuilderTags(unittest.TestCase):
    def test_builder_tags(self):
        at = B.way_attrs({"highway": "primary", "oneway": "yes", "turn:lanes": "left|through|through; right"})
        self.assertEqual((at[5], at[6], at[-2], at[-1]), (3, 3, "left|through|through;right", ""))
        at = B.way_attrs({"highway": "secondary", "turn:lanes:forward": "left|", "turn:lanes:backward": "|right"})
        self.assertEqual((at[6], at[7], at[-2], at[-1]), (2, 2, "left|none", "none|right"))
        self.assertTrue(at[2] & R.F2_LANES_SPLIT)
        at = B.way_attrs({"highway": "secondary", "oneway": "-1", "turn:lanes": "left|through"})
        self.assertEqual((at[-2], at[-1]), ("", "left|through"))                    # geometry order is reversed
        self.assertEqual(B.way_attrs({"highway": "primary", "turn:lanes": "left|right"})[-2:], ("", ""))   # two-way: ambiguous
        self.assertEqual(R.normalise_turn_lanes("none|none|"), "")
        self.assertEqual(B.node_feature({"highway": "speed_camera"})[0], R.FEATURE_ID["speed_camera"])


if __name__ == "__main__":
    unittest.main()
