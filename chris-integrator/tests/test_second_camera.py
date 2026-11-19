"""A second camera with its own object list as a source (sources/camera_objects.py); no change to the core."""
import math
import unittest

import numpy as np

from .simrun import Run
from .tracks_eval import per_vehicle
from chris_integrator import Integrator
from chris_integrator.sim import default_sensors
from chris_integrator.sim.camera2 import CameraObjectsSim
from chris_integrator.sim.radar import RadarSim


class SecondCamera(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cfg = {"sources": [{"kind": "camera_objects", "id": "cam_wide", "mount": {"x_m": 0.0, "y_m": 0.0, "yaw_deg": 0.0}}]}
        cls.cam = Run(sensors=default_sensors() + [CameraObjectsSim()], seed=1, integ=Integrator(cfg), keep_tracks=True)
        cfg = {"sources": cfg["sources"] + [{"kind": "radar", "id": "*"}]}
        cls.all = Run(sensors=default_sensors() + [CameraObjectsSim(), RadarSim()], seed=1, integ=Integrator(cfg), keep_tracks=True)

    def test_second_camera_tracks_what_the_lead_output_never_reports(self):
        # vehicles in the next lanes within its range: the road model's lead output never shows them
        side = per_vehicle(self.cam, lambda o: abs(o["d"]) > 2.0 and 4.0 < o["x"] < 55.0, gate=(8.0, 1.6))
        self.assertGreater(side["n"], 200)
        self.assertGreater(side["tracked"], 0.9)
        self.assertLess(side["rms_y"], 0.5)             # a camera places a vehicle across the road well ...
        self.assertGreater(side["rms_x"], 0.5)          # ... and along it poorly, which the track says:
        stds = [k["std"][0] for t, tr in self.cam.tracks[500::20] for k in tr["tracks"] if k["sources"] == ["cam_wide"]]
        self.assertGreater(np.median(stds), 0.5)
        self.assertEqual(self.cam.es["sources"]["cam_wide"]["state"], "unchecked")
        # the lead stays the road model's (one track, both cameras feed it)
        ld = self.cam.leads[-1][1]
        self.assertIn("cam_wide", ld["sources"])

    def test_camera_and_radar_tracks_are_one_list(self):
        side = per_vehicle(self.all, lambda o: abs(o["d"]) > 2.0 and 4.0 < o["x"] < 55.0)
        self.assertGreater(side["tracked"], 0.95)
        self.assertLess(side["dup"], 0.05)
        self.assertLess(side["rms_x"], 0.6)             # with the radar the range is right
        sens = self.all.tracks[-1][1]["sensors"]
        self.assertGreater(sens["cam_wide"]["scans"], 2000)         # both object sensors are heard, each in its own order
        self.assertGreater(sens["radar_front"]["scans"], 3500)
        three = [k for t, tr in self.all.tracks[500:] for k in tr["tracks"] if len(k["sources"]) >= 3]
        self.assertGreater(len(three), 300)             # the lead: road model + second camera + radar


if __name__ == "__main__":
    unittest.main()
