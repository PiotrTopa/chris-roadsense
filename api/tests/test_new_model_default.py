"""The api around the default road model (openpilot v0.11.1 vision + policy, lane L27).

* lead tracker, `vision` profile: weak evidence, acquisition by accumulated consistent evidence, a track id kept
  through a short absence and through a distance step of the same vehicle; supercombo's settings untouched;
* ego lane: the higher commit threshold for the vision model's lines;
* `road_model` in `lead`, `lanes`, `view`; a held plan (`plan.held`, `plan.age_ms`) drawn like a fresh one;
* the lane-source seam is tested in test_display_lanes.py (lane L29).

    python3 -m unittest api.tests.test_new_model_default -v     (from the repository root)
"""
import os
import sys
import unittest

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, os.path.join(ROOT, "api"))
sys.path.insert(0, os.path.join(ROOT, "tools"))
from roadsense_api import egolane, lead as lead_mod  # noqa: E402
from roadsense_api.lead import LeadTracker  # noqa: E402
from replay_lib import Pipeline  # noqa: E402
from api.tests.test_vision_road import X, line, road  # noqa: E402

DT = 0.07          # s between runs (about 14 runs a second)


def lead_road(i, prob, x, v_rel=0.0, model="vision", v_ego=20.0, y=0.0):
    ts = int(i * DT * 1e9)
    ld = {"prob": prob, "x": [x], "y": [y], "v": [v_ego + v_rel], "a": [0.0], "x_std": [0.1 * x], "y_std": [0.3],
          "v_std": [1.5], "a_std": [1.0]}
    return {"type": "road", "seq": i, "ts_mono_ns": ts, "ts_real_ns": ts, "valid": True, "road_model": model,
            "v_ego": v_ego, "pose": {"vx": v_ego, "yaw_rate": 0.0}, "leads": [ld, ld, ld]}


def run(tr, frames, i0=0, **kw):
    """frames: [(prob, x)] or [(prob, x, v_rel)]; returns the lead messages."""
    out = []
    for k, f in enumerate(frames):
        out.append(tr.on_road(lead_road(i0 + k, f[0], f[1], f[2] if len(f) > 2 else 0.0, **kw)))
    return out


class VisionLeadProfileTest(unittest.TestCase):
    def test_profile_follows_the_road_model(self):
        tr = LeadTracker()
        tr.on_road(lead_road(0, 0.0, 50.0, model="supercombo"))
        self.assertEqual(tr.cfg["acquire_score"], None)
        self.assertEqual(tr.cfg["rv_scale"], lead_mod.DEFAULT_CFG["rv_scale"])
        tr.on_road(lead_road(1, 0.0, 50.0, model="vision"))
        self.assertEqual(tr.cfg["acquire_score"], lead_mod.PROFILES["vision"]["acquire_score"])
        self.assertEqual(LeadTracker().cfg, lead_mod.DEFAULT_CFG)         # no road model named: supercombo's settings
        tr = LeadTracker({"coast_s": 0.3})                                  # a user's key wins over the profile
        out = tr.on_road(lead_road(0, 0.0, 50.0))
        self.assertEqual((tr.cfg["coast_s"], out["road_model"]), (0.3, "vision"))

    def test_supercombo_unchanged(self):
        tr = LeadTracker()
        out = run(tr, [(0.9, 40.0)] * 2, model="supercombo")
        self.assertTrue(out[1]["present"])                                  # two frames, as before
        out = run(tr, [(0.2, 40.0)] * 12, i0=2, model="supercombo")        # under drop_prob: prediction only
        self.assertEqual({o["evidence"] for o in out if o["present"]}, {"none"})
        self.assertFalse(out[-1]["present"])                                # gone after coast_s = 0.6 s

    def test_weak_evidence_carries_a_dropout_and_is_marked(self):
        tr = LeadTracker()
        out = run(tr, [(0.95, 60.0)] * 10)
        self.assertEqual((out[-1]["present"], out[-1]["mode"], out[-1]["evidence"]), (True, "track", "strong"))
        # the probability dips for 1.4 s while distance and relative speed stay where the track is
        out = run(tr, [(0.2, 60.3)] * 20, i0=10)
        self.assertTrue(all(o["present"] for o in out))
        self.assertEqual({(o["mode"], o["evidence"]) for o in out}, {("coast", "weak")})   # shown, never as confirmed
        out = run(tr, [(0.9, 60.5)] * 3, i0=30)
        self.assertEqual((out[-1]["mode"], out[-1]["evidence"], out[-1]["track_id"]), ("track", "strong", 1))

    def test_weak_evidence_is_bounded(self):
        tr = LeadTracker()
        run(tr, [(0.95, 60.0)] * 10)
        out = run(tr, [(0.2, 60.0)] * 60, i0=10)                           # 4.2 s without one confirming frame
        gone = next(k for k, o in enumerate(out) if not o["present"])
        self.assertAlmostEqual(gone * DT, tr.cfg["weak_coast_s"], delta=0.2)

    def test_inconsistent_low_frames_are_no_evidence(self):
        tr = LeadTracker()
        run(tr, [(0.95, 60.0)] * 10)
        out = run(tr, [(0.2, 35.0), (0.2, 80.0), (0.15, 44.0)] * 6, i0=10)  # low probability, distance anywhere
        self.assertEqual({o["evidence"] for o in out if o["present"]}, {"none"})
        gone = next(k for k, o in enumerate(out) if not o["present"])
        self.assertAlmostEqual(gone * DT, tr.cfg["coast_s"], delta=0.15)    # prediction only, then dropped
        tr = LeadTracker()
        run(tr, [(0.95, 60.0)] * 10)
        out = run(tr, [(0.2, 60.0, -6.0)] * 6, i0=10)                      # right distance, another speed: not it
        self.assertEqual({o["evidence"] for o in out if o["present"]}, {"none"})

    def test_acquisition_needs_consistent_frames(self):
        tr = LeadTracker()
        out = run(tr, [(0.6, 30.0), (0.6, 48.0), (0.6, 33.0), (0.6, 55.0), (0.6, 28.0), (0.6, 47.0)])
        self.assertFalse(any(o["present"] for o in out))                    # six frames that agree with nothing
        tr = LeadTracker()
        out = run(tr, [(0.6, 50.0)] * 6)
        first = next(k for k, o in enumerate(out) if o["present"])
        self.assertEqual(first, 4)                                          # 5 x 0.6 = 3.0 >= 2.5
        tr = LeadTracker()
        out = run(tr, [(0.95, 12.0)] * 3)
        self.assertTrue(out[1]["present"])                                  # a clear lead: two frames, as supercombo

    def test_reacquired_track_keeps_its_id(self):
        tr = LeadTracker()
        run(tr, [(0.95, 70.0)] * 10)
        out = run(tr, [(0.02, 30.0)] * 20, i0=10)                          # nothing for 1.4 s: the track is dropped
        self.assertFalse(out[-1]["present"])
        self.assertIsNone(out[-1]["track_id"])
        out = run(tr, [(0.9, 70.5)] * 4, i0=30)
        self.assertEqual((out[-1]["present"], out[-1]["track_id"], out[-1]["event"]["kind"]), (True, 1, "reacquired"))
        out = run(tr, [(0.02, 30.0)] * 20, i0=34)
        out = run(tr, [(0.9, 25.0)] * 4, i0=54)                            # another vehicle, elsewhere: a new id
        self.assertEqual((out[-1]["track_id"], out[-1]["event"]["kind"]), (2, "acquired"))

    def test_distance_step_of_the_same_vehicle_keeps_the_id(self):
        tr = LeadTracker()
        run(tr, [(0.95, 30.0)] * 12)
        out = run(tr, [(0.95, 44.0)] * 14, i0=12)                          # the model re-estimates the distance
        self.assertAlmostEqual(out[-1]["dist_m"], 44.0, delta=1.0)         # the track follows: nothing is held
        self.assertEqual((out[-1]["track_id"], out[-1]["event"]["kind"]), (1, "relocated"))
        out = run(tr, [(0.95, 20.0, -7.0)] * 8, i0=26)                     # nearer AND another speed: another vehicle
        self.assertEqual((out[-1]["track_id"], out[-1]["event"]["kind"]), (2, "cut_in"))

    def test_relative_speed_from_the_same_pair(self):
        tr = LeadTracker()
        m = lead_road(0, 0.9, 40.0, v_rel=-2.0)
        m["v_ego"] = 3.0                                                    # a stale value must not be used
        out = tr.on_road(m)
        self.assertAlmostEqual(out["vision"]["v_rel_mps"], -2.0, places=2)


class VisionEgoLaneProfileTest(unittest.TestCase):
    def test_commit_threshold_by_road_model(self):
        ln = {"type": "lanes", "ts_mono_ns": 0, "x": X, "lines": [], "edges": []}
        e = egolane.EgoLaneEstimator()
        self.assertEqual(e.cfg["commit_p"], egolane.DEFAULT_CFG["commit_p"])
        e.on_lanes(ln, {"road_model": "vision"})
        self.assertEqual(e.cfg["commit_p"], 0.9)
        e.on_lanes(dict(ln, ts_mono_ns=1), {"road_model": "supercombo"})
        self.assertEqual(e.cfg["commit_p"], egolane.DEFAULT_CFG["commit_p"])
        e = egolane.EgoLaneEstimator({"commit_p": 0.7})
        e.on_lanes(ln, {"road_model": "vision"})
        self.assertEqual(e.cfg["commit_p"], 0.7)


class NewDefaultContractTest(unittest.TestCase):
    def feed(self, p, n, **kw):
        for s in range(1, n):
            m = road(s, plan=True)
            m.update(kw)
            p.feed(m)

    def test_road_model_in_the_derived_topics(self):
        p = Pipeline()
        self.feed(p, 40)
        for t in ("lead", "lanes", "view"):
            self.assertEqual(p.latest[t]["road_model"], "vision", t)
        self.assertEqual(p.latest["view"]["lines_source"], "model")

    def test_held_plan_is_used(self):
        p = Pipeline()
        for s in range(1, 40):
            m = road(s, plan=True)
            m["plan"].update(held=s % 4 != 0, age_ms=50.0 * (s % 4))
            m["policy_hz"], m["policy_ms"] = 5, 1.2
            p.feed(m)
        self.assertIsNotNone(p.latest["view"].get("path"))


if __name__ == "__main__":
    unittest.main()
