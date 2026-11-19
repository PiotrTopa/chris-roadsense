"""Lead tracker (api/roadsense_api/lead.py): synthetic scenes with the noise measured on replayed rides, and
two real standstill spans (model output of recorded rides, api/tests/data/lead_*.csv).

    python3 -m unittest api.tests.test_lead -v     (from the repository root)

The synthetic lead output follows docs/lanes/L14-lead.md: distance jitter hypot(0.07 m, 1.6 % of the distance)
per frame, a slow wander of 0.1 m (5 s), 2 % outlier frames of 0.4-1.1 m, a lead speed output that sits within
millimetres per second of zero behind a standing car, the model's own v_ego with rare blips, x_std = 0.1 +
0.13 d, v_std = 0.05 + 0.3 min(|v|, 1) + 0.012 d.
"""
import csv
import math
import os
import random
import sys
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))

from roadsense_api.lead import LeadTracker      # noqa: E402

NS = 10 ** 9
DATA = os.path.join(os.path.dirname(os.path.abspath(__file__)), "data")


def flips(vals):
    """How often the value rounded to whole metres (what a display shows) changes."""
    r = [math.floor(v + 0.5) for v in vals]
    return sum(1 for a, b in zip(r, r[1:]) if a != b)


def flips_hyst(vals, h=0.3):
    """The same with a display hysteresis of h m (the number changes when the value is 0.5 + h from it)."""
    shown, n = math.floor(vals[0] + 0.5), 0
    for v in vals[1:]:
        if abs(v - shown) > 0.5 + h:
            shown, n = math.floor(v + 0.5), n + 1
    return n


def longest_constant(rows):
    """Longest time the published distance stays exactly the same (a frozen read-out), s."""
    best, t0, last = 0.0, None, None
    for t, _, _, o in rows:
        d = o["dist_m"]
        if d != last:
            t0, last = t, d
        best = max(best, t - t0)
    return best


class Scene:
    """Truth (ego and lead speeds over time) -> noisy `road` messages and vehicle `ego` messages."""

    def __init__(self, gap, v_ego=0.0, v_lead=0.0, hz=12.0, seed=1, ego_feed=True, ego_hz=10.0, blips=True):
        self.rnd = random.Random(seed)
        self.t = 100.0
        self.gap, self.ve, self.vl = gap, v_ego, v_lead
        self.ae = self.al = 0.0
        self.hz, self.ego_feed, self.ego_hz, self.blips = hz, ego_feed, ego_hz, blips
        self.wander = 0.0
        self.other = None          # a second vehicle {"gap", "v"} the model may pick instead (cut-in / flicker)
        self.pick_other = lambda t: False
        self.prob = 0.98
        self.seq = 0
        self.ego_seq = 0
        self._ego_next = self.t
        self.tr = None
        self.rows = []

    def attach(self, tracker):
        self.tr = tracker
        return self

    def _advance(self, dt):
        self.ve = max(0.0, self.ve + self.ae * dt)
        self.vl = max(0.0, self.vl + self.al * dt)
        self.gap += (self.vl - self.ve) * dt
        if self.other:
            self.other["gap"] += (self.other["v"] - self.ve) * dt
        self.t += dt

    def _ego_msgs(self, until):
        while self.ego_feed and self._ego_next <= until:
            self.ego_seq += 1
            still = self.ve == 0.0
            self.tr.on_ego({"type": "ego", "seq": self.ego_seq, "ts_mono_ns": int(self._ego_next * NS),
                            "speed_mps": round(self.ve, 3), "standstill": still, "gear": "D",
                            "wheel_speeds_mps": [round(self.ve, 3)] * 4, "source": "vehicle", "tag": "test"})
            self._ego_next += 1.0 / self.ego_hz

    def road(self):
        rnd = self.rnd
        use_other = self.other is not None and self.pick_other(self.t)
        d, vl = (self.other["gap"], self.other["v"]) if use_other else (self.gap, self.vl)
        self.wander += (1.0 / self.hz / 5.0) * (-self.wander) + math.sqrt(2.0 / self.hz / 5.0) * 0.1 * rnd.gauss(0, 1)
        x = d + rnd.gauss(0, math.hypot(0.07, 0.016 * d)) + self.wander * min(1.0, d / 4.0)
        if rnd.random() < 0.02:
            x += rnd.choice((-1, 1)) * rnd.uniform(0.4, 1.1)
        v_model = self.ve + rnd.gauss(0, 0.002 + 0.03 * self.ve)
        if self.blips and rnd.random() < 0.01:
            v_model += rnd.uniform(0.1, 0.45)
        rel_sigma = 0.004 if abs(vl - self.ve) < 0.02 and self.ve < 0.02 else 0.1 + 0.03 * d
        v = v_model + (vl - self.ve) + rnd.gauss(0, rel_sigma)
        self.seq += 1
        return {"type": "road", "seq": self.seq, "ts_mono_ns": int(self.t * NS), "ts_real_ns": int(self.t * NS),
                "v_ego": max(0.0, v_model), "valid": True,
                "leads": [{"prob": self.prob, "x": [max(0.5, x)] + [0.0] * 5, "y": [0.1] * 6, "v": [v] * 6, "a": [0.0] * 6,
                           "x_std": [0.1 + 0.13 * d] * 6, "y_std": [0.3] * 6, "v_std": [0.05 + 0.3 * min(abs(vl), 1.0) + 0.012 * d] * 6,
                           "a_std": [0.5] * 6}]}

    def run(self, seconds):
        """Advance the scene; returns the rows produced: (t, true gap, raw x, lead message)."""
        out = []
        n = int(round(seconds * self.hz))
        for _ in range(n):
            self._advance(1.0 / self.hz)
            self._ego_msgs(self.t + 0.02)                      # vehicle motion arrives ahead of the (late) frame
            m = self.road()
            o = self.tr.on_road(m)
            out.append((self.t, self.gap, m["leads"][0]["x"][0], o))
        self.rows += out
        return out


def tracked(rows):
    return [o["dist_m"] for _, _, _, o in rows if o["present"]]


class Standstill(unittest.TestCase):
    """The owner's case: stopped in a jam behind a standing car, the distance read-out flips between two metres."""

    def jam(self, ego_feed, gap=2.5, seed=3):
        s = Scene(gap + 6.0, v_ego=4.0, v_lead=0.0, seed=seed, ego_feed=ego_feed).attach(LeadTracker())
        s.ae = -4.0 / 3.0
        s.run(3.0)                                             # brake to a stop `gap` behind the lead
        s.ae, s.ve = 0.0, 0.0
        s.run(3.0)                                             # settle
        return s, s.run(60.0)

    def test_with_vehicle_motion_the_distance_is_steady_and_live(self):
        """Since roadsense 2.0 (L19) the value is never frozen: it is filtered, and a display with a small
        hysteresis on its whole metres shows one number."""
        for seed in (3, 4, 5):
            s, rows = self.jam(True, seed=seed)
            raw, trk = [r[2] for r in rows], tracked(rows)
            self.assertEqual(len(trk), len(rows))
            self.assertGreater(flips(raw), 30)                 # the raw value at x.5 m flips all the time
            self.assertLessEqual(flips(trk), max(2, flips(raw) // 10))
            self.assertEqual(flips_hyst(trk), 0)
            self.assertLess(longest_constant(rows), 1.0)       # live: it moves with every frame
            self.assertLess(max(abs(d - s.gap) for d in trk), 0.3)
            last = rows[-1][3]
            self.assertEqual((last["mode"], last["standing"], last["source"]), ("track", True, "vision+ego"))
            self.assertLess(abs(last["v_rel_mps"]), 0.05)
            self.assertEqual((last["ego"]["standstill"], last["departed"], last["meas_age_s"]), (True, False, 0.0))
            self.assertIsNone(last["time_gap_s"])
            self.assertFalse(any(o["departed"] for _, _, _, o in rows))

    def test_vision_only_is_steady_too(self):
        for seed in (3, 4, 5):
            s, rows = self.jam(False, seed=seed)
            raw, trk = [r[2] for r in rows], tracked(rows)
            self.assertGreater(flips(raw), 30)
            self.assertLessEqual(flips(trk), max(2, flips(raw) // 10))
            self.assertEqual(flips_hyst(trk), 0)
            self.assertLess(longest_constant(rows), 1.0)
            self.assertLess(abs(trk[-1] - s.gap), 0.3)
            self.assertEqual((rows[-1][3]["mode"], rows[-1][3]["source"]), ("track", "vision"))
            self.assertGreater(sum(1 for r in rows if r[3]["standing"]) / len(rows), 0.95)
            self.assertFalse(any(o["departed"] for _, _, _, o in rows))

    def test_never_worse_than_the_raw_value_at_standstill(self):
        for feed in (True, False):
            s, rows = self.jam(feed, gap=4.0, seed=7)
            raw, trk = [r[2] for r in rows], tracked(rows)
            sd = lambda v: math.sqrt(sum((x - sum(v) / len(v)) ** 2 for x in v) / len(v))      # noqa: E731
            self.assertLess(sd(trk), 0.5 * sd(raw))

    def test_lead_departure_is_detected_fast_and_reported(self):
        for feed in (True, False):
            delays = []
            for seed in range(1, 9):
                s, _ = self.jam(feed, gap=3.0, seed=seed)
                t0, gap0 = s.t, s.gap
                s.al = 1.2                                     # the lead pulls away, ego keeps standing
                rows = s.run(4.0)
                dep = [(t, g, o) for t, g, _, o in rows if o["departed"]]
                self.assertTrue(dep, "departure not reported (feed %s, seed %d)" % (feed, seed))
                t, g, o = dep[0]
                delays.append(t - t0)
                self.assertLess(g - gap0, 0.8)                 # reported before the lead has gone 0.8 m
                self.assertEqual(o["event"]["kind"], "departed")
                self.assertAlmostEqual(o["departure"]["dist_at_stop_m"], gap0, delta=0.35)
                end = rows[-1][3]
                self.assertEqual(end["mode"], "track")
                self.assertLess(abs(end["dist_m"] - s.gap), 0.8)             # the track follows the lead away
                self.assertGreater(end["v_lead_mps"], 3.0)
            self.assertLess(sum(delays) / len(delays), 0.8)
            self.assertLess(max(delays), 1.1)

    def test_departed_clears_when_ego_follows(self):
        s, _ = self.jam(True, gap=3.0)
        s.al = 1.2
        s.run(2.0)
        self.assertTrue(s.rows[-1][3]["departed"])
        s.ae = 1.5
        rows = s.run(1.0)
        self.assertFalse(rows[-1][3]["departed"])
        self.assertIsNone(rows[-1][3]["departure"])

    def test_ego_creeping_forward_leaves_standing_at_once(self):
        s, _ = self.jam(True, gap=4.0)
        s.ae = 0.8
        rows = s.run(1.0)                                      # ego rolls 0.4 m towards the standing lead
        self.assertFalse(rows[0][3]["standing"])               # first frame with a wheel turning
        s.ae, s.ve = 0.0, 0.0                                  # (a jam shuffle: stop again)
        rows += s.run(3.0)
        for t, g, _, o in rows:
            self.assertLess(abs(o["dist_m"] - g), 0.35)
        self.assertTrue(rows[-1][3]["standing"])
        self.assertLess(abs(rows[-1][3]["dist_m"] - s.gap), 0.3)
        self.assertFalse(any(o["departed"] for _, _, _, o in rows))

    def test_slow_creep_of_the_lead_is_followed(self):
        """A lead rolling at 0.08 m/s is under the speed detector's slack; the distance check catches it."""
        s, _ = self.jam(True, gap=3.0)
        s.vl = 0.08
        s.run(20.0)                                            # 1.6 m
        s.vl = 0.0
        rows = s.run(5.0)
        self.assertLess(abs(rows[-1][3]["dist_m"] - s.gap), 0.4)

    def test_lead_lost_while_standing(self):
        s, _ = self.jam(True, gap=3.0)
        s.prob = 0.05
        rows = s.run(1.0)
        self.assertTrue(rows[-1][3]["present"])                # e.g. a pedestrian crossing in front: kept, marked
        self.assertEqual(rows[-1][3]["mode"], "coast")
        self.assertGreater(rows[-1][3]["meas_age_s"], 0.9)
        rows = s.run(3.0)
        self.assertFalse(rows[-1][3]["present"])
        self.assertEqual(rows[-1][3]["event"]["kind"], "lost")


class Moving(unittest.TestCase):
    def follow(self, feed=True, seed=2, gap=30.0, v=20.0, hz=12.0):
        s = Scene(gap, v_ego=v, v_lead=v, seed=seed, ego_feed=feed, hz=hz).attach(LeadTracker())
        s.run(5.0)
        return s

    def test_steady_following_is_smoother_and_unbiased(self):
        for feed in (True, False):
            s = self.follow(feed)
            rows = s.run(30.0)
            e_raw = [x - g for _, g, x, _ in rows]
            e_trk = [o["dist_m"] - g for _, g, _, o in rows]
            rms = lambda v: math.sqrt(sum(x * x for x in v) / len(v))      # noqa: E731
            self.assertLess(rms(e_trk), 0.6 * rms(e_raw))
            self.assertLess(abs(sum(e_trk) / len(e_trk)), 0.15)
            o = rows[-1][3]
            self.assertAlmostEqual(o["time_gap_s"], 1.5, delta=0.1)
            self.assertAlmostEqual(o["v_rel_mps"], 0.0, delta=1.0)
            self.assertTrue(o["ttc_s"] is None or o["ttc_s"] > 15.0)
            self.assertEqual(o["mode"], "track")

    def test_lead_braking_is_not_smoothed_away(self):
        """Lead brakes 4 m/s2 at 20 m/s, 30 m ahead: the tracked distance may trail the truth by at most 0.15 s
        worth of closing speed, and the closing speed shows within half a second."""
        for feed in (True, False):
            worst, seen = 0.0, []
            for seed in range(1, 7):
                s = self.follow(feed, seed=seed)
                s.al = -4.0
                t0 = s.t
                rows = s.run(2.5)
                for t, g, _, o in rows:
                    closing = 4.0 * (t - t0)
                    if closing > 2.0:
                        worst = max(worst, (o["dist_m"] - g) / closing)
                seen.append(next(t - t0 for t, _, _, o in rows if o["v_rel_mps"] < -1.0))
                self.assertLess(abs(rows[-1][3]["v_rel_mps"] + 10.0), 2.5)
                self.assertIsNotNone(rows[-1][3]["ttc_s"])
                self.assertLess(rows[-1][3]["a_lead_mps2"], -1.0)
            self.assertLess(worst, 0.20, "tracked distance trails the truth by %.2f s" % worst)
            self.assertLess(sum(seen) / len(seen), 0.55)      # v_rel under -1 m/s 0.25 s after the truth at the earliest

    def test_cut_in_replaces_the_track_without_averaging(self):
        s = self.follow(gap=40.0)
        old = s.rows[-1][3]["track_id"]
        s.other = {"gap": 14.0, "v": 19.0}
        s.pick_other = lambda t: True
        t0 = s.t
        rows = s.run(2.0)
        sw = next(t for t, _, _, o in rows if o["track_id"] != old)
        self.assertLess(sw - t0, 0.4)
        for t, _, _, o in rows:
            self.assertFalse(18.0 < o["dist_m"] < 36.0, "an average of the two vehicles at %.2f s: %.1f m" % (t - t0, o["dist_m"]))
        end = rows[-1][3]
        self.assertEqual(end["event"]["kind"], "cut_in")
        self.assertAlmostEqual(end["dist_m"], s.other["gap"], delta=1.0)
        self.assertLess(end["v_rel_mps"], -0.3)

    def test_lead_change_to_a_farther_vehicle(self):
        s = self.follow(gap=15.0)
        old = s.rows[-1][3]["track_id"]
        s.other = {"gap": 45.0, "v": 20.0}
        s.pick_other = lambda t: True                          # the lead turned off; the next car is 45 m ahead
        t0 = s.t
        rows = s.run(2.0)
        sw = next(t for t, _, _, o in rows if o["track_id"] != old)
        self.assertLess(sw - t0, 0.8)
        self.assertGreater(sw - t0, 0.3)                       # not on the first frames: it could be a flicker
        for _, _, _, o in rows:
            self.assertFalse(19.0 < o["dist_m"] < 40.0)
        self.assertEqual(rows[-1][3]["event"]["kind"], "changed")

    def test_two_hypotheses_flickering_are_not_averaged(self):
        """The model alternates between the lead (20 m) and the vehicle in front of it (38 m): the track stays
        on the lead; nothing in between is ever reported."""
        s = self.follow(gap=20.0)
        tid = s.rows[-1][3]["track_id"]
        s.other = {"gap": 38.0, "v": 20.0}
        s.pick_other = lambda t: int(t * s.hz) % 5 < 2         # 2 frames of 5 on the far one
        rows = s.run(20.0)
        for _, g, _, o in rows:
            self.assertAlmostEqual(o["dist_m"], g, delta=1.5)
            self.assertEqual(o["track_id"], tid)
        self.assertTrue(any(o["pending_change"] for _, _, _, o in rows))

    def test_single_outlier_frames_are_ignored(self):
        s = self.follow(gap=8.0, v=8.0)
        s.other = {"gap": 12.0, "v": 8.0}
        s.pick_other = lambda t: int(t * s.hz) % 30 == 0
        rows = s.run(20.0)
        self.assertLess(max(abs(o["dist_m"] - g) for _, g, _, o in rows), 0.8)

    def test_lost_and_reacquired(self):
        s = self.follow()
        s.prob = 0.1
        rows = s.run(0.4)
        self.assertEqual((rows[-1][3]["present"], rows[-1][3]["mode"]), (True, "coast"))
        rows = s.run(0.6)
        o = rows[-1][3]
        self.assertEqual((o["present"], o["mode"], o["dist_m"], o["track_id"]), (False, "none", None, None))
        self.assertEqual(o["event"]["kind"], "lost")
        self.assertEqual(o["vision"]["prob"], 0.1)             # the raw value is always there
        s.prob = 0.9
        rows = s.run(0.5)
        self.assertTrue(rows[-1][3]["present"])
        self.assertFalse(rows[0][3]["present"])                # one frame is not enough

    def test_at_the_phone_rate_and_at_20_hz(self):
        for hz in (8.0, 12.0, 20.0):
            s = self.follow(hz=hz)
            rows = s.run(10.0)
            self.assertLess(max(abs(o["dist_m"] - g) for _, g, _, o in rows), 1.6)


class Inputs(unittest.TestCase):
    def test_without_ego_everything_is_vision(self):
        s = Scene(20.0, 10.0, 10.0, ego_feed=False).attach(LeadTracker())
        o = s.run(3.0)[-1][3]
        self.assertEqual((o["source"], o["ego"]["source"], o["ego"]["age_s"]), ("vision", "vision", None))
        self.assertAlmostEqual(o["ego"]["speed_mps"], 10.0, delta=1.0)

    def test_stale_vehicle_motion_falls_back_to_vision(self):
        s = Scene(20.0, 10.0, 10.0).attach(LeadTracker())
        o = s.run(3.0)[-1][3]
        self.assertEqual((o["source"], o["ego"]["source"], o["ego"]["tag"]), ("vision+ego", "vehicle", "test"))
        self.assertLess(o["ego"]["age_s"], 0.12)
        s.ego_feed = False                                     # the feeder goes away
        rows = s.run(2.0)
        self.assertEqual(rows[-1][3]["source"], "vision")
        self.assertTrue(rows[-1][3]["present"])
        self.assertLess(abs(rows[-1][3]["dist_m"] - s.gap), 1.5)
        self.assertEqual(rows[-1][3]["track_id"], o["track_id"])

    def test_vehicle_speed_decides_standstill_not_the_camera(self):
        """Stopped by the wheels while the model's own speed estimate drifts (a truck filling the frame)."""
        s = Scene(3.0, 0.0, 0.0).attach(LeadTracker())
        def creeping(road):                                    # the model thinks the car (and so the lead) creeps
            def f():
                m = road()
                m["v_ego"] += 0.6
                m["leads"][0]["v"] = [v + 0.6 for v in m["leads"][0]["v"]]
                return m
            return f
        s.road = creeping(s.road)
        rows = s.run(10.0)
        self.assertTrue(rows[-1][3]["standing"])
        self.assertEqual(rows[-1][3]["ego"]["speed_mps"], 0.0)
        s2 = Scene(3.0, 0.0, 0.0, ego_feed=False).attach(LeadTracker())
        s2.road = creeping(s2.road)
        self.assertFalse(s2.run(10.0)[-1][3]["standing"])     # vision only cannot know: not standing, still filtered

    def test_malformed_input_is_survived(self):
        tr = LeadTracker()
        tr.on_ego({"type": "ego"})
        tr.on_ego({"type": "ego", "ts_mono_ns": "x", "speed_mps": 1})
        self.assertIsNone(tr.on_road({"type": "road"}))
        for i, leads in enumerate((None, [], [{}], [{"prob": 0.9}], [{"prob": 0.9, "x": []}], [{"prob": "a", "x": [3.0]}],
                                   [{"prob": 0.9, "x": [float("nan")]}], [{"prob": 0.9, "x": [-2.0]}])):
            o = tr.on_road({"type": "road", "seq": i, "ts_mono_ns": (10 + i) * NS // 10, "leads": leads})
            self.assertEqual((o["type"], o["present"], o["dist_m"]), ("lead", False, None))
        # a lead without speed outputs still tracks on distance alone
        for i in range(20):
            o = tr.on_road({"type": "road", "seq": 100 + i, "ts_mono_ns": (30 + i) * NS // 10, "v_ego": 10.0,
                            "leads": [{"prob": 0.9, "x": [25.0 - 0.1 * i]}]})
        self.assertTrue(o["present"])
        self.assertAlmostEqual(o["dist_m"], 23.1, delta=0.5)

    def test_time_going_back_or_a_long_gap_restarts(self):
        s = Scene(20.0, 10.0, 10.0).attach(LeadTracker())
        s.run(3.0)
        tid = s.rows[-1][3]["track_id"]
        s.t += 5.0                                             # modeld restarted / a replay looped
        s._ego_next = s.t
        rows = s.run(1.0)
        self.assertFalse(rows[0][3]["present"])
        self.assertEqual(rows[-1][3]["track_id"], tid + 1)
        s.t -= 50.0
        s._ego_next = s.t
        rows = s.run(1.0)
        self.assertFalse(rows[0][3]["present"])
        self.assertTrue(rows[-1][3]["present"])

    def test_message_fields(self):
        s = Scene(20.0, 10.0, 9.0).attach(LeadTracker({"camera_to_front_m": 1.5}))
        o = s.run(2.0)[-1][3]
        for k in ("type", "seq", "ts_mono_ns", "ts_real_ns", "present", "mode", "track_id", "source", "age_s", "dist_m",
                  "dist_std_m", "dist_filter_std_m", "lateral_m", "v_rel_mps", "v_rel_std_mps", "v_lead_mps",
                  "a_lead_mps2", "time_gap_s", "ttc_s", "standing", "pending_change", "prob", "departed", "departure",
                  "event", "ego", "vision", "gap_m", "radar", "vision_scale"):
            self.assertIn(k, o)
        self.assertAlmostEqual(o["gap_m"], o["dist_m"] - 1.5, places=2)
        self.assertAlmostEqual(o["time_gap_s"], o["gap_m"] / o["ego"]["speed_mps"], delta=0.02)
        self.assertGreaterEqual(o["dist_std_m"], o["dist_filter_std_m"])
        self.assertEqual(set(o["vision"]), {"dist_m", "dist_std_m", "lateral_m", "v_lead_mps", "v_rel_mps", "a_lead_mps2", "prob"})
        self.assertEqual(o["ts_mono_ns"], int(s.t * NS))
        none = LeadTracker().on_road({"type": "road", "seq": 1, "ts_mono_ns": NS, "leads": [{"prob": 0.01, "x": [80.0]}]})
        self.assertEqual(set(none) - {"gap_m"}, set(o) - {"gap_m"})


@unittest.skip("L38: range sensors are sources of the integrator; the radar source is added by the next commits")
class Radar(unittest.TestCase):
    """Range-sensor fusion (`radar_fusion`): synthetic radar scans next to the synthetic camera output. The
    camera's distance is scaled by 1.15 (an uncalibrated focal length), the radar is right to 0.15 m."""

    CAM_SCALE = 1.15

    def scene(self, gap=30.0, v=20.0, fusion=True, seed=5, **kw):
        s = Scene(gap, v, v, seed=seed, **kw)
        s.attach(LeadTracker({"radar_fusion": fusion}))
        s.radar_on = True
        s.clutter = []                 # other objects: (x offset from the lead, y)
        rnd = random.Random(seed + 100)
        road0 = s.road

        def road():
            if s.radar_on:
                objs = [{"id": 1, "x_m": round(s.gap + rnd.gauss(0, 0.15), 2), "y_m": 0.2,
                         "range_rate_mps": round(s.vl - s.ve + rnd.gauss(0, 0.1), 2)}]
                objs += [{"id": 10 + i, "x_m": s.gap + dx, "y_m": y, "range_rate_mps": -s.ve} for i, (dx, y) in enumerate(s.clutter)]
                s.tr.on_objects({"type": "objects", "ts_mono_ns": int((s.t - 0.03) * NS), "sensor": "radar0",
                                 "kind": "radar", "frame": "camera", "objects": objs})
            m = road0()
            ld = m["leads"][0]
            ld["x"] = [x * self.CAM_SCALE for x in ld["x"]]
            return m
        s.road = road
        return s

    def test_radar_range_and_closing_speed_replace_the_cameras(self):
        s = self.scene()
        rows = s.run(8.0)[40:]
        o = rows[-1][3]
        self.assertEqual((o["source"], o["radar"]["sensor"], o["radar"]["id"]), ("radar+vision+ego", "radar0", 1))
        err = [abs(o["dist_m"] - g) for _, g, _, o in rows]
        self.assertLess(max(err), 0.5)                              # the camera alone is 4.5 m off at 30 m
        self.assertAlmostEqual(o["vision"]["dist_m"], 30.0 * self.CAM_SCALE, delta=2.0)      # the raw camera value stays visible
        self.assertAlmostEqual(o["vision_scale"], 1.0 / self.CAM_SCALE, delta=0.02)
        self.assertLess(o["dist_std_m"], 0.6)
        vis = self.scene(fusion=False).run(8.0)[-1][3]              # the flag is off: scans are ignored
        self.assertEqual((vis["source"], vis["radar"], vis["vision_scale"]), ("vision+ego", None, None))
        self.assertGreater(abs(vis["dist_m"] - 30.0), 3.0)

    def test_closing_is_seen_sooner_and_without_lag(self):
        s = self.scene(gap=40.0)
        s.run(4.0)
        s.al = -3.0                                                 # the lead brakes
        rows = s.run(1.5)
        o = rows[-1][3]
        self.assertAlmostEqual(o["v_rel_mps"], s.vl - s.ve, delta=0.5)
        self.assertAlmostEqual(o["dist_m"], s.gap, delta=0.5)
        self.assertIsNotNone(o["ttc_s"])

    def test_fallback_to_vision_is_clean(self):
        s = self.scene()
        s.run(6.0)
        tid = s.rows[-1][3]["track_id"]
        s.radar_on = False                                          # the radar goes away (sensor fault, feeder stopped)
        rows = s.run(6.0)
        srcs = [o["source"] for _, _, _, o in rows]
        self.assertEqual(srcs[-1], "vision+ego")
        self.assertTrue(all(o["present"] and o["track_id"] == tid for _, _, _, o in rows))       # the same track, no restart
        self.assertTrue(all(o["radar"] is None for _, _, _, o in rows[6:]))
        # the learned scale keeps the camera's distance where the radar had it: no 4.5 m step
        self.assertLess(max(abs(o["dist_m"] - g) for _, g, _, o in rows), 1.5)
        events = {o["event"]["kind"] for _, _, _, o in rows if o["event"]}
        self.assertEqual(events, {"acquired"})

    def test_no_feeder_no_change(self):
        """With the flag on and no scans the tracker is the vision tracker, message for message."""
        a, b = Scene(25.0, 15.0, 15.0, seed=7), Scene(25.0, 15.0, 15.0, seed=7)
        a.attach(LeadTracker({"radar_fusion": True}))
        b.attach(LeadTracker())
        self.assertEqual([o for _, _, _, o in a.run(10.0)], [o for _, _, _, o in b.run(10.0)])

    def test_other_objects_are_not_taken_for_the_lead(self):
        s = self.scene()
        s.clutter = [(-12.0, 3.4), (2.0, -3.5), (25.0, 0.0), (-20.0, 0.3)]     # next lanes, a far car, a near one out of the gate
        rows = s.run(8.0)[40:]
        self.assertTrue(all(o["radar"]["id"] == 1 for _, _, _, o in rows))
        self.assertLess(max(abs(o["dist_m"] - g) for _, g, _, o in rows), 0.5)

    def test_camera_loses_the_lead_radar_keeps_it_for_a_while(self):
        s = self.scene()
        s.run(5.0)
        s.prob = 0.05                                               # glare: the camera sees no lead
        rows = s.run(4.0)
        self.assertTrue(all(o["present"] for _, _, _, o in rows))
        self.assertEqual(rows[-1][3]["source"], "radar+ego")
        self.assertAlmostEqual(rows[-1][3]["dist_m"], s.gap, delta=0.5)
        rows = s.run(3.0)                                           # ... but not for ever: the camera says which vehicle is the lead
        self.assertFalse(rows[-1][3]["present"])
        # and a radar alone never starts a track
        s2 = self.scene()
        s2.prob = 0.05
        self.assertFalse(any(o["present"] for _, _, _, o in s2.run(5.0)))

    def test_stale_or_unplaced_scans_are_ignored(self):
        tr = LeadTracker({"radar_fusion": True})
        tr.on_objects({"type": "objects", "ts_mono_ns": NS, "sensor": "r", "kind": "radar", "frame": "sensor", "objects": []})
        tr.on_objects({"type": "objects", "ts_mono_ns": NS, "sensor": "u", "kind": "ultrasonic", "frame": "camera", "objects": []})
        tr.on_objects({"type": "objects", "sensor": "r", "kind": "radar", "frame": "camera", "objects": None})
        tr.on_objects(None)
        self.assertEqual(tr.scans, {})
        s = self.scene()
        s.run(3.0)
        s.radar_on = False
        s.tr.scans["radar0"] = (int((s.t - 5.0) * NS), [{"x_m": 30.0, "y_m": 0.0}])       # an old scan stays around
        self.assertIsNone(s.run(1.0)[-1][3]["radar"])

    def test_brake_is_passed_on(self):
        s = Scene(20.0, 0.0, 0.0).attach(LeadTracker())
        self.assertIsNone(s.run(1.0)[-1][3]["ego"]["brake"])
        s.tr.on_vehicle({"type": "vehicle", "ts_mono_ns": int(s.t * NS), "controls": {"brake": True}})
        self.assertIs(s.run(1.0)[-1][3]["ego"]["brake"], True)
        self.assertIsNone(s.run(4.0)[-1][3]["ego"]["brake"])        # stale: unknown, not "released"
        s.tr.on_vehicle({"type": "vehicle", "ts_mono_ns": int(s.t * NS), "controls": {"brake": None}})
        self.assertIsNone(s.run(0.5)[-1][3]["ego"]["brake"])


class RealSpans(unittest.TestCase):
    """Model output (supercombo, ORT fp32, 20 fps replay) of two recorded rides around a stop behind a car.
    Columns: t, v_ego, prob, x, x_std, v, v_std (docs/lanes/L14-lead.md says how they were made)."""

    def replay(self, name, every=1, surrogate=False):
        tr = LeadTracker()
        rows, low, hist = [], 0.0, []
        with open(os.path.join(DATA, name)) as f:
            data = [r for r in csv.reader(l for l in f if not l.startswith("#"))][1:]
        last = None
        for i, r in enumerate(data):
            if i % every:
                continue
            t, ve, p, x, xs, v, vs = (float(c) for c in r)
            ts = int(round(t * NS))
            if surrogate:                                      # stand-in wheel speed: exactly 0 while the car stands
                dt = 0.0 if last is None else t - last         # (median of 7 frames: wheels do not blip like the model)
                hist.append(ve)
                ve = sorted(hist[-7:])[len(hist[-7:]) // 2]
                low = low + dt if ve < 0.15 else 0.0
                tr.on_ego({"type": "ego", "ts_mono_ns": ts, "speed_mps": 0.0 if low >= 0.5 else ve,
                           "standstill": low >= 0.5, "tag": "surrogate"})
            last = t
            o = tr.on_road({"type": "road", "seq": i, "ts_mono_ns": ts, "v_ego": ve,
                            "leads": [{"prob": p, "x": [x], "x_std": [xs], "v": [v], "v_std": [vs], "y": [0.0]}]})
            rows.append((t, x, o))
        return rows

    def check(self, name, t_stop, t_go, t_dep, min_raw_flips):
        """t_stop: 3.5 s after the car stopped (standing has begun, tracker constants hold_* / stand_*)."""
        for every in (1, 2):                                   # 20 Hz as replayed, and every second frame (10 Hz)
            for sur in (False, True):
                rows = self.replay(name, every, sur)
                span = [(t, x, o) for t, x, o in rows if t_stop <= t <= t_go]
                raw, trk = [x for _, x, _ in span], [o["dist_m"] for _, _, o in span]
                self.assertGreaterEqual(flips(raw), min_raw_flips // every)
                self.assertEqual(flips_hyst(trk), 0, "%s every %d surrogate %s" % (name, every, sur))
                self.assertLessEqual(flips(trk), max(3, flips(raw) // 8))
                rms = lambda v: math.sqrt(sum(x * x for x in v) / len(v))      # noqa: E731
                steps = [abs(b - a) for a, b in zip(trk, trk[1:])]
                # jitter (frame-to-frame step) far below the raw value's; the slow wander of the network's distance
                # (a few decimetres over seconds) is what a live value shows, by design
                self.assertLess(rms(steps), 0.25 * rms([b - a for a, b in zip(raw, raw[1:])]))
                self.assertLess(max(steps), 0.1)               # no visible step from one frame to the next
                self.assertLess(longest_constant([(t, None, None, o) for t, _, o in span]), 1.5)   # live (the hold froze)
                self.assertFalse(any(o["departed"] for _, _, o in span))
                dep = [t for t, _, o in rows if o["departed"]]
                self.assertTrue(dep, "departure missed: %s every %d surrogate %s" % (name, every, sur))
                self.assertGreater(dep[0], t_dep - 0.2)
                self.assertLess(dep[0], t_dep + 0.9)

    def test_motorway_jam_in_rain(self):
        # a1-s8-rain: stop 3.5 m behind a car at 131.3 s, wipers across the view; it rolls off at 136.9 s
        self.check("lead_a1-s8-rain_jam.csv", 134.8, 136.8, 136.9, 8)

    def test_city_red_light(self):
        # warsaw-city: 30 s at a red light 4.4 m behind a car, the raw value sits on the 4 / 5 m rounding edge
        self.check("lead_warsaw-city_light.csv", 163.5, 190.0, 190.5, 50)


if __name__ == "__main__":
    unittest.main()
