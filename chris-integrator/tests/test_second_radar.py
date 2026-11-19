"""A second radar: configuration only (the same source class with another mount); no code of the core or of any
consumer changes."""
import math
import unittest

import numpy as np

from .simrun import Run
from .tracks_eval import per_vehicle
from chris_integrator import Integrator
from chris_integrator.sim import default_sensors
from chris_integrator.sim.radar import RadarSim

FL = (2.0, 0.8, math.radians(40.0))                  # a corner radar at the front left, looking 40 degrees left
CFG2 = {"sources": [{"kind": "radar", "id": "radar_front", "mount": {"x_m": 2.2, "y_m": 0.0, "yaw_deg": 0.0}},
                    {"kind": "radar", "id": "radar_fl", "mount": {"x_m": FL[0], "y_m": FL[1], "yaw_deg": 40.0}}]}


class SecondRadar(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        sens = default_sensors() + [RadarSim(), RadarSim("radar_fl", FL, seed=52, misalign_deg=-0.8, with_mount=False)]
        cls.r = Run(sensors=sens, seed=1, integ=Integrator(CFG2), keep_tracks=True)

    def test_both_radars_are_sources_with_their_own_calibration(self):
        es = self.r.es
        self.assertEqual((es["sources"]["radar_front"]["state"], es["sources"]["radar_fl"]["state"]), ("ok", "ok"))
        self.assertAlmostEqual(math.degrees(es["calibration"]["radar_fl"]["dyaw"]["value"]), -0.8, delta=0.25)
        self.assertEqual(es["calibration"]["radar_fl"]["mount"]["yaw_deg"], 40.0)
        tr = self.r.tracks[-1][1]
        self.assertEqual(set(tr["sensors"]) >= {"radar_front", "radar_fl", "camera_lead"}, True)

    def test_one_track_per_vehicle_across_both_fields_of_view(self):
        # the left-lane vehicle is seen by the corner radar, by the front radar, or by both
        left = per_vehicle(self.r, lambda o: o["id"] == 2 and 3.0 < o["x"] < 100.0 and o["d"] > 2.0)
        self.assertGreater(left["tracked"], 0.95)
        self.assertLess(left["dup"], 0.06)              # two tracks for a moment at the hand-over, then merged
        self.assertLessEqual(left["switches"], 8)
        self.assertLess(left["rms_x"], 0.6)
        both = sum(1 for t, tr in self.r.tracks for k in tr["tracks"]
                   if {"radar_front", "radar_fl"} <= set(k["sources"]) and k["state"] != "tentative")
        self.assertGreater(both, 500)                   # tracks fed by both radars exist


if __name__ == "__main__":
    unittest.main()
