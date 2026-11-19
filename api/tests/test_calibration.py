"""The camera calibration in /health (lane L32): the configured calib.json or modeld's online pitch / yaw estimate."""
import json
import math
import os
import sys
import unittest

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, os.path.join(ROOT, "api"))
from roadsense_api.state import State  # noqa: E402


def calib(source="config", pitch=0.0, yaw=0.0, online=None):
    m = {"type": "calib", "seq": 1, "ts_mono_ns": 1, "ts_real_ns": 1, "pitch": pitch, "yaw": yaw, "roll": 0, "height_m": 1.22,
         "focal_px": 975, "cx": 640, "cy": 360, "width": 1280, "height": 720, "source": source}
    if online is not None:
        m["online"] = online
    return m


class CalibrationHealthTest(unittest.TestCase):
    def feed(self, st, m):
        st.on_message("modeld", m, json.dumps(m))

    def test_absent_and_old_modeld(self):
        st = State("/nonexistent")
        self.assertIsNone(st.health()["calibration"])
        self.feed(st, calib())                      # a modeld from before L32: no `online`
        c = st.health()["calibration"]
        self.assertEqual((c["source"], c["yaw"], c["online"]), ("config", 0.0, None))

    def test_collecting(self):
        st = State("/nonexistent")
        self.feed(st, calib(online={"state": "collecting", "applied": False, "pitch": None, "yaw": None,
                                    "spread_pitch": None, "spread_yaw": None, "blocks": 0, "min_blocks": 5, "samples": 40,
                                    "block": 100, "accepted": 40, "seen": 90, "restored": False,
                                    "config": {"pitch": 0, "yaw": 0}}))
        o = st.health()["calibration"]["online"]
        self.assertEqual((o["state"], o["applied"], o["off_config_deg"], o["spread_deg"]), ("collecting", False, None, None))

    def test_applied_estimate(self):
        st = State("/nonexistent")
        p, y = math.radians(1.33), math.radians(-3.41)
        self.feed(st, calib("auto", p, y, {"state": "valid", "applied": True, "pitch": p, "yaw": y,
                                           "spread_pitch": math.radians(0.1), "spread_yaw": math.radians(0.17),
                                           "blocks": 46, "min_blocks": 5, "samples": 3, "block": 100, "accepted": 4603,
                                           "seen": 60000, "restored": True, "config": {"pitch": 0, "yaw": 0}}))
        c = st.health()["calibration"]
        self.assertEqual(c["source"], "auto")
        self.assertAlmostEqual(c["yaw"], y)
        o = c["online"]
        self.assertEqual((o["state"], o["applied"], o["blocks"], o["restored"]), ("valid", True, 46, True))
        self.assertEqual(o["off_config_deg"], {"pitch": 1.33, "yaw": -3.41})
        self.assertEqual(o["spread_deg"], {"pitch": 0.1, "yaw": 0.17})


if __name__ == "__main__":
    unittest.main()
