"""Lane L20: a weak receiver that claims more accuracy than it has, service roads at speed, the confidence of the
matched road (not of one edge of it)."""
import os
import random
import shutil
import sys
import tempfile
import unittest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from roadsense_mapd import geo, rsmap as R                  # noqa: E402
from roadsense_mapd.engine import Engine                    # noqa: E402
from roadsense_mapd.matcher import MapMatcher               # noqa: E402

from tests import synthpack as SP                           # noqa: E402

NS = 10 ** 9
T0 = 10 ** 12


class Weak(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.tmp = tempfile.mkdtemp(prefix="roadsense-l20-")
        cls.path = os.path.join(cls.tmp, "synth.rsmap")
        SP.build(cls.path)
        cls.pack = R.Pack(cls.path)

    @classmethod
    def tearDownClass(cls):
        cls.pack.close()
        shutil.rmtree(cls.tmp, ignore_errors=True)

    def drive(self, eng, scatter_m, claimed_m, seconds=120, seed=3):
        """East along the main road at 12 m/s; fixes scattered by scatter_m (sigma) but reporting claimed_m."""
        rnd = random.Random(seed)
        outs = []
        for i in range(int(seconds * 10)):
            ts = T0 + i * NS // 10
            eng.on_input({"type": "motion", "ts_mono_ns": ts, "speed_mps": 12.0, "yaw_rate_rps": 0.0}, ts)
            if i % 10 == 0:
                x = 50.0 + 1.2 * i
                lat, lon = SP.ll(x + rnd.gauss(0, scatter_m), rnd.gauss(0, scatter_m))
                eng.on_input({"type": "gnss", "ts_mono_ns": ts, "lat": lat, "lon": lon, "hacc_m": claimed_m,
                              "course_deg": 90.0, "speed_mps": 12.0}, ts)
            if i % 5 == 0:
                outs.append(eng.step(ts))
        return outs

    def test_optimistic_receiver_is_scaled(self):
        """Fixes scattered 10 m that claim 2 m: the hacc in use grows to about the true scatter."""
        eng = Engine(self.pack)
        outs = self.drive(eng, 10.0, 2.0)
        scale = outs[-1]["input"]["gnss"]["hacc_scale"]
        self.assertGreater(scale, 3.0)
        self.assertLessEqual(scale, eng.cfg["gnss_scale_max"])
        self.assertGreater(outs[-1]["position"]["hacc_m"], 3.0)

    def test_honest_receiver_is_left_alone(self):
        eng = Engine(self.pack)
        outs = self.drive(eng, 2.0, 3.0)
        self.assertLess(outs[-1]["input"]["gnss"]["hacc_scale"], 1.5)

    def test_switch_off(self):
        eng = Engine(self.pack, {"gnss_adaptive": False})
        outs = self.drive(eng, 10.0, 2.0)
        self.assertEqual(outs[-1]["input"]["gnss"]["hacc_scale"], 1.0)

    def test_service_road_loses_at_speed(self):
        """Fixes between the main road and the parallel service road (15 m south), slightly nearer the service
        road: at walking pace the service road may win, at 50 km/h the main road does."""
        lat, lon = SP.ll(500, -11)
        fast = MapMatcher(self.pack).update(lat, lon, 90.0, 14.0, 6.0, 0.0)
        self.assertEqual(fast.edge.name, "Main")
        base = MapMatcher(self.pack, {"class_speed": 0.0}).update(lat, lon, 90.0, 14.0, 6.0, 0.0)
        self.assertEqual(base.edge.name, "Access")

    def test_road_confidence_does_not_dip_at_a_node(self):
        """Right at a graph node of the main road (x = 1000, where the road is split into two edges) the
        confidence is that of the road, not shared between its two edges."""
        lat, lon = SP.ll(1000.5, 0.3)
        on = MapMatcher(self.pack).update(lat, lon, 90.0, 14.0, 4.0, 0.0)
        off = MapMatcher(self.pack, {"road_confidence": False}).update(lat, lon, 90.0, 14.0, 4.0, 0.0)
        self.assertEqual(on.edge.name, "Main")
        self.assertGreater(on.confidence, off.confidence + 0.2)
        self.assertGreater(on.confidence, 0.8)


if __name__ == "__main__":
    unittest.main()
