"""Unit tests for the speed-limit state machine with synthetic Polish sign sequences.

    cd <repo> && python3 -m unittest api.tests.test_speedlimit -v
"""
import os
import sys
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))
from roadsense_api.speedlimit import SpeedLimit  # noqa: E402

F, CX, CY, D = 690.0, 960.0, 540.0, 0.8


class Drive:
    """Synthetic ride: road at 10 Hz, signs at 5 Hz, env at 1 Hz; signs approach on a pinhole camera."""

    def __init__(self, sl=None, v=12.0, place=None):
        self.sl = sl or SpeedLimit()
        self.v, self.t, self.seq, self.tid = v, 0.0, 0, 100
        self.place = place
        self.yaw_rate = 0.0
        self.out = []

    def _hdr(self, typ):
        self.seq += 1
        ns = int(self.t * 1e9)
        return {"type": typ, "seq": self.seq, "ts_mono_ns": ns, "ts_real_ns": 1_700_000_000_000_000_000 + ns}

    def step(self, dets=(), ended=()):
        """Advance 0.2 s: two road messages, one signs message, env every 1 s."""
        for _ in range(2):
            self.t += 0.1
            self.sl.on_message({**self._hdr("road"), "v_ego": self.v, "pose": {"vx": self.v, "yaw_rate": self.yaw_rate}})
        self.sl.on_message({**self._hdr("signs"), "detections": list(dets), "tracks_ended": list(ended)})
        if self.place and abs(self.t - round(self.t)) < 0.05:
            self.sl.on_message({**self._hdr("env"), "place": {"label": self.place, "confidence": 0.7,
                                                                "stable_s": 60.0}})
        self.out.append(self.sl.snapshot())

    def drive(self, metres):
        for _ in range(int(metres / (self.v * 0.2))):
            self.step()

    def sign(self, cls, lateral=3.0, z0=60.0, z1=9.0, score=0.8, aspect=1.0, cls_alt=None, alt_every=0,
             extra=None, frames=None):
        """One sign track from distance z0 to z1, then ended. extra: another (cls, dy_px) plate under it."""
        self.tid += 1
        tid, ptid = self.tid, self.tid + 1000
        z, k = z0, 0
        n = frames if frames is not None else 10 ** 9
        while z > z1 and k < n:
            h = F * D / z
            w = h * aspect
            u = CX + F * lateral / z
            v = CY - F * 1.5 / z
            c = cls_alt if alt_every and k % alt_every == 0 else cls
            dets = [{"track_id": tid, "cls": c, "score": score, "box": [u - w / 2, v - h / 2, u + w / 2, v + h / 2],
                     "hits": k + 1, "age": k}]
            if extra:
                dets.append({"track_id": ptid, "cls": extra, "score": 0.7,
                             "box": [u - w / 2, v + h / 2 + 2, u + w / 2, v + h / 2 + 2 + 0.5 * h], "hits": k + 1})
            self.step(dets)
            z -= self.v * 0.2
            k += 1
        ended = [{"track_id": tid, "cls": cls}] + ([{"track_id": ptid, "cls": extra}] if extra else [])
        self.step(ended=ended)
        return tid

    @property
    def last(self):
        return self.out[-1]


class SpeedLimitTest(unittest.TestCase):
    def test_unknown_at_start(self):
        d = Drive()
        d.drive(50)
        self.assertIsNone(d.last["limit_kmh"])
        self.assertEqual(d.last["source"]["kind"], "unknown")

    def test_enter_built_up_by_env_prior_then_end_sign(self):
        d = Drive(place="city_street")
        d.drive(300)
        self.assertEqual(d.last["limit_kmh"], 50)
        self.assertEqual(d.last["area"]["source"], "env")
        self.assertLessEqual(d.last["confidence"], 0.5)
        d.sign("information--end-of-built-up-area")
        d.drive(30)
        self.assertEqual(d.last["limit_kmh"], 90)
        self.assertEqual(d.last["area"], {"type": "outside", "source": "sign", "confidence": 0.8})
        d.drive(500)                       # the env prior must not override a fresh area sign
        self.assertEqual(d.last["limit_kmh"], 90)

    def test_b33_then_intersection(self):
        d = Drive(place="city_street")
        d.drive(200)
        tid = d.sign("regulatory--maximum-speed-limit-70")
        self.assertEqual(d.last["limit_kmh"], 70)
        self.assertEqual(d.last["source"]["track_id"], tid)
        self.assertEqual(d.last["source"]["sign"], "B-33")
        self.assertIn("next intersection", d.last["expiry"]["rule"])
        d.drive(150)
        self.assertEqual(d.last["limit_kmh"], 70)
        d.sign("regulatory--yield", lateral=4.0)
        d.drive(40)
        self.assertEqual(d.last["limit_kmh"], 50)
        self.assertEqual(d.last["source"]["kind"], "default")
        self.assertIn("intersection after A-7/B-20", d.sl.snapshot(full=True)["recent_changes"][0]["event"])

    def test_junction_sign_on_same_pole_does_not_cancel(self):
        d = Drive(place="city_street")
        d.drive(200)
        d.sign("regulatory--maximum-speed-limit-40")
        d.sign("regulatory--priority-road", z0=12, z1=9)  # same pole, passed right after
        d.drive(100)
        self.assertEqual(d.last["limit_kmh"], 40)

    def test_b34_end(self):
        d = Drive(place="city_street")
        d.drive(200)
        d.sign("regulatory--maximum-speed-limit-30")
        d.drive(50)
        d.sign("regulatory--end-of-maximum-speed-limit-30")
        d.drive(10)
        self.assertEqual(d.last["limit_kmh"], 50)

    def test_b42_end_of_prohibitions(self):
        d = Drive(place="rural_road")
        d.drive(200)
        self.assertEqual(d.last["limit_kmh"], 90)
        d.sign("regulatory--maximum-speed-limit-60")
        self.assertEqual(d.last["limit_kmh"], 60)
        d.sign("regulatory--end-of-prohibition")
        self.assertEqual(d.last["limit_kmh"], 90)

    def test_living_street_zone(self):
        d = Drive(place="city_street", v=6.0)
        d.drive(200)
        d.sign("information--living-street")
        self.assertEqual(d.last["limit_kmh"], 20)
        self.assertEqual(d.last["zone"]["type"], "residential")
        d.sign("regulatory--yield")         # junctions do not end a zone
        d.drive(100)
        self.assertEqual(d.last["limit_kmh"], 20)
        d.sign("information--end-of-living-street")
        self.assertEqual(d.last["limit_kmh"], 50)

    def test_speed_zone_suspect_survives_junctions_until_b44(self):
        d = Drive(place="city_street", v=8.0)
        d.drive(200)
        d.sign("regulatory--maximum-speed-limit-30")
        self.assertTrue(d.sl.limit["zone_suspect"])
        d.sign("regulatory--yield")
        d.drive(300)
        self.assertEqual(d.last["limit_kmh"], 30)
        d.sign("regulatory--end-of-speed-limit-zone")
        self.assertEqual(d.last["limit_kmh"], 50)

    def test_motorway(self):
        d = Drive(v=30.0)
        d.sign("information--motorway", lateral=5)
        self.assertEqual(d.last["limit_kmh"], 140)
        self.assertEqual(d.last["road_class"], "motorway")
        d.sign("regulatory--maximum-speed-limit-100", lateral=0.0)    # gantry above the lane
        d.sign("regulatory--yield")                                   # no one-level junctions here
        d.drive(2000)
        self.assertEqual(d.last["limit_kmh"], 100)
        self.assertIn("B-34", d.last["expiry"]["rule"])
        d.sign("regulatory--end-of-maximum-speed-limit-70")
        self.assertEqual(d.last["limit_kmh"], 140)
        d.sign("information--end-of-motorway")
        self.assertEqual(d.last["limit_kmh"], 90)

    def test_expressway(self):
        d = Drive(v=30.0)
        d.sign("information--limited-access-road")
        self.assertEqual(d.last["limit_kmh"], 120)

    def test_single_detection_and_short_track_do_not_flip(self):
        d = Drive(place="city_street")
        d.drive(200)
        d.sign("regulatory--maximum-speed-limit-30", frames=1)
        d.sign("regulatory--maximum-speed-limit-30", frames=3)
        self.assertEqual(d.last["limit_kmh"], 50)
        self.assertEqual(len(d.sl.snapshot(full=True)["ignored"]), 2)

    def test_pending_while_live(self):
        d = Drive(place="city_street")
        d.drive(200)
        d.sl.on_message({"type": "signs", "ts_mono_ns": int(d.t * 1e9), "detections": [
            {"track_id": 9, "cls": "regulatory--maximum-speed-limit-70", "score": 0.8,
             "box": [1000, 500, 1020, 520], "hits": 3}], "tracks_ended": []})
        p = d.sl.snapshot()["pending"]
        self.assertEqual(p[0]["value_kmh"], 70)
        self.assertEqual(d.sl.snapshot()["limit_kmh"], 50)

    def test_class_flicker_rejected(self):
        d = Drive(place="city_street")
        d.drive(200)
        d.sign("regulatory--maximum-speed-limit-80", cls_alt="regulatory--maximum-speed-limit-30", alt_every=2)
        self.assertEqual(d.last["limit_kmh"], 50)

    def test_exit_lane_and_side_road_rejected(self):
        d = Drive(place="city_street")
        d.drive(200)
        d.sign("regulatory--maximum-speed-limit-30", lateral=14.0, z0=90, z1=20)
        d.sign("regulatory--maximum-speed-limit-30", aspect=0.4)
        self.assertEqual(d.last["limit_kmh"], 50)
        reasons = [i["reason"] for i in d.sl.snapshot(full=True)["ignored"]]
        self.assertTrue(any("lateral" in r for r in reasons))
        self.assertTrue(any("aspect" in r for r in reasons))

    def test_left_side_duplicate_accepted(self):
        d = Drive(place="rural_road", v=20)
        d.drive(200)
        d.sign("regulatory--maximum-speed-limit-70", lateral=-6.0)
        self.assertEqual(d.last["limit_kmh"], 70)

    def test_truck_plate_ignored_and_other_plate_flagged(self):
        d = Drive(place="rural_road", v=20)
        d.drive(200)
        d.sign("regulatory--maximum-speed-limit-60", extra="complementary--trucks")
        self.assertEqual(d.last["limit_kmh"], 90)
        d.sign("regulatory--maximum-speed-limit-70", extra="complementary--distance")
        self.assertEqual(d.last["limit_kmh"], 70)
        self.assertEqual(d.last["source"]["plate"], "complementary--distance")

    def test_distance_cap(self):
        d = Drive(place="city_street")
        d.drive(200)
        d.sign("regulatory--maximum-speed-limit-60")
        d.drive(650)
        self.assertEqual(d.last["limit_kmh"], 60)
        self.assertLess(d.last["confidence"], 0.8)
        d.drive(100)
        self.assertEqual(d.last["limit_kmh"], 50)

    def test_turn_proxy(self):
        d = Drive(place="city_street", v=6.0)
        d.drive(200)
        d.sign("regulatory--maximum-speed-limit-40")
        d.drive(40)
        d.yaw_rate = 0.5                    # rad/s, ~90 deg in 3 s / 18 m
        d.drive(20)
        d.yaw_rate = 0.0
        self.assertEqual(d.last["limit_kmh"], 50)

    def test_repeat_refreshes_and_since(self):
        d = Drive(place="city_street")
        d.drive(200)
        d.sign("regulatory--maximum-speed-limit-70")
        s0 = d.last["since"]["ts_real_ns"]
        d.drive(500)
        d.sign("regulatory--maximum-speed-limit-70")
        d.drive(400)                        # 900 m after the first sign, 400 m after the repeat
        self.assertEqual(d.last["limit_kmh"], 70)
        self.assertEqual(d.last["since"]["ts_real_ns"], s0)
        self.assertGreater(d.last["since"]["distance_m"], 890)

    def test_new_sign_replaces(self):
        d = Drive(place="rural_road", v=20)
        d.drive(200)
        d.sign("regulatory--maximum-speed-limit-70")
        d.drive(300)
        d.sign("regulatory--maximum-speed-limit-50")
        self.assertEqual(d.last["limit_kmh"], 50)
        self.assertLess(d.last["since"]["distance_m"], 30)


if __name__ == "__main__":
    unittest.main()
