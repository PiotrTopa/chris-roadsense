"""Sign context checks (signctx.py) and suppressed tracks in state / speed limit.

    cd <repo> && python3 -m unittest discover -s api/tests -v
"""
import json
import os
import sys
import unittest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from roadsense_api import signctx  # noqa: E402
from roadsense_api.state import State  # noqa: E402

INTR = (690.0, 640.0, 360.0, 1280, 720)
CALIB = {"pitch": 0.0, "yaw": 0.0, "roll": 0.0, "height_m": 1.2}


def signs(seq, dets, ended=None):
    return {"type": "signs", "seq": seq, "ts_mono_ns": seq * 200_000_000, "detect_ms": 50.0,
            "detections": dets, "tracks_ended": ended or []}


def det(tid, cls, box, **kw):
    return dict({"track_id": tid, "cls": cls, "cls_id": 1, "score": 0.6, "box": box, "age": 3, "hits": 3}, **kw)


def lights(seq, boxes):
    return {"type": "lights", "seq": seq, "ts_mono_ns": seq * 200_000_000, "frame": [1280, 720],
            "lights": [{"track_id": i, "box": b, "state": "red", "score": 0.8, "hits": 5} for i, b in enumerate(boxes)]}


def road(seq, lead_x=10.0, prob=0.9):
    return {"type": "road", "seq": seq, "ts_mono_ns": seq * 200_000_000,
            "leads": [{"prob": prob, "x": [lead_x], "y": [0.0], "v": [0.0]}]}


class SignContextTest(unittest.TestCase):
    def test_round_red_classes(self):
        self.assertTrue(signctx.is_round_red("regulatory--no-entry"))
        self.assertTrue(signctx.is_round_red("regulatory--maximum-speed-limit-50"))
        self.assertTrue(signctx.is_round_red("regulatory--stop"))
        self.assertFalse(signctx.is_round_red("regulatory--stop-here-on-red-or-flashing-light"))
        self.assertFalse(signctx.is_round_red("information--parking"))

    def test_light_overlap_suppresses(self):
        c = signctx.SignContext()
        lt = lights(1, [[600, 100, 620, 150]])
        for seq in (1, 2, 3):
            m = signs(seq, [det(7, "regulatory--no-entry", [602, 105, 618, 121])])
            lt["seq"], lt["ts_mono_ns"] = seq, seq * 200_000_000
            c.annotate(m, None, lt, CALIB, INTR)
        d = m["detections"][0]
        self.assertTrue(d["suppressed"])
        self.assertEqual(d["reason"], "traffic_light")
        self.assertEqual(d["context"]["traffic_light"], 1.0)
        e = signs(4, [], [{"track_id": 7, "cls": "regulatory--no-entry", "hits": 3, "best_score": 0.6}])
        c.annotate(e, None, None, CALIB, INTR)
        self.assertTrue(e["tracks_ended"][0]["suppressed"])
        self.assertNotIn(7, c.tracks)

    def test_stale_lights_ignored_and_non_red_untouched(self):
        c = signctx.SignContext()
        lt = lights(1, [[600, 100, 620, 150]])          # 1 s older than the signs below
        for seq in (6, 7, 8):
            m = signs(seq, [det(7, "regulatory--no-entry", [602, 105, 618, 121]),
                            det(8, "information--parking", [602, 105, 618, 121])])
            c.annotate(m, None, lt, CALIB, INTR)
        self.assertNotIn("suppressed", m["detections"][0])
        self.assertNotIn("context", m["detections"][1])

    def test_verifier_vouches(self):
        c = signctx.SignContext()
        for seq in (1, 2, 3):
            lt = lights(seq, [[600, 100, 620, 150]])
            m = signs(seq, [det(7, "regulatory--no-entry", [602, 105, 618, 121], verify=0.97, glow=0.0)])
            c.annotate(m, None, lt, CALIB, INTR)
        d = m["detections"][0]
        self.assertIn("context", d)                    # flagged for inspection ...
        self.assertNotIn("suppressed", d)              # ... but the appearance checks vouch for a real sign

    def test_lead_vehicle(self):
        c = signctx.SignContext()
        rect = signctx.lead_rect(road(1), CALIB, INTR, (1280, 720))
        # 10 m ahead, camera 1.2 m up, f 690: rear from v=360+690*1.2/10=443 (ground) to 360-690*0.8/10=305
        self.assertAlmostEqual(rect[3], 442.8, delta=0.5)
        self.assertAlmostEqual(rect[1], 304.8, delta=0.5)
        for seq in (1, 2):
            m = signs(seq, [det(9, "regulatory--no-entry", [630, 400, 650, 412]),       # stop-lamp height
                            det(10, "regulatory--no-entry", [630, 200, 650, 220])])     # 3+ m up: a sign
            c.annotate(m, road(seq), None, CALIB, INTR)
        self.assertEqual(m["detections"][0]["reason"], "lead_vehicle")
        self.assertNotIn("suppressed", m["detections"][1])
        m = signs(3, [det(11, "regulatory--no-entry", [630, 400, 650, 412])])
        c.annotate(m, road(3, prob=0.2), None, CALIB, INTR)
        self.assertNotIn("context", m["detections"][0])


class StateSuppressedTest(unittest.TestCase):
    def test_suppressed_not_live_and_not_a_speed_limit(self):
        st = State("/nonexistent")
        sup = det(5, "regulatory--maximum-speed-limit-30", [900, 200, 940, 240], suppressed=True, reason="glow",
                  glow=1.0)
        ok = det(6, "regulatory--no-stopping", [100, 200, 140, 240])
        for seq in range(1, 4):
            m = signs(seq, [dict(sup), dict(ok)])
            st.on_message("signd", m, json.dumps(m))
        self.assertEqual([t["track_id"] for t in st.tracks()], [6])
        s = st.tracks(suppressed=True)
        self.assertEqual([t["track_id"] for t in s], [5])
        self.assertEqual(s[0]["reason"], "glow")
        end = signs(4, [], [{"track_id": 5, "cls": "regulatory--maximum-speed-limit-30", "hits": 20, "best_score": 0.9,
                             "suppressed": True, "reason": "glow"}])
        st.on_message("signd", end, json.dumps(end))
        self.assertEqual(st.tracks(suppressed=True), [])
        self.assertTrue(any("suppressed (glow)" in i["reason"] for i in st.speed.ignored))
        self.assertNotEqual(st.speed.snapshot().get("limit_kmh"), 30)


if __name__ == "__main__":
    unittest.main()
