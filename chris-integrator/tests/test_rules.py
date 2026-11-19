"""Unit tests for the rules-in-effect engine with synthetic Polish sign sequences (lane L11).

    cd <repo> && python3 -m unittest api.tests.test_rules -v
"""
import os
import sys
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))
from chris_integrator.trackers.rules import RulesInEffect  # noqa: E402

F, CX, CY, D = 690.0, 960.0, 540.0, 0.8


class Drive:
    """Synthetic ride: road at 10 Hz, signs at 5 Hz, env at 1 Hz, lights on demand; a pinhole camera (f 690)."""

    def __init__(self, cfg=None, v=12.0, place="city_street"):
        self.r = RulesInEffect(cfg=dict({"env_min_stable_s": 5.0}, **(cfg or {})))
        self.v, self.t, self.seq, self.tid = v, 0.0, 0, 100
        self.place = place
        self.yaw_rate = 0.0
        self.ego = None          # (dist_m, track_id) of a synthetic ego traffic light, or None
        self.out = []

    def _hdr(self, typ):
        self.seq += 1
        ns = int(self.t * 1e9)
        return {"type": typ, "seq": self.seq, "ts_mono_ns": ns, "ts_real_ns": 1_700_000_000_000_000_000 + ns}

    def step(self, dets=(), ended=()):
        for _ in range(2):
            self.t += 0.1
            self.r.on_message({**self._hdr("road"), "v_ego": self.v, "pose": {"vx": self.v, "yaw_rate": self.yaw_rate}})
        self.r.on_message({**self._hdr("signs"), "detections": list(dets), "tracks_ended": list(ended)})
        if self.place and abs(self.t - round(self.t)) < 0.05:
            scores = {k: 0.02 for k in ("highway", "city_street", "residential", "rural_road", "forest_road")}
            scores[self.place] = 0.7
            self.r.on_message({**self._hdr("env"), "place": {"label": self.place, "confidence": 0.7,
                                                               "stable_s": 60.0, "scores": scores}})
        if self.ego is not None or self.lights_on:
            ego = {"track_id": self.ego[1], "state": "green", "dist_m": self.ego[0], "lateral_m": 1.0,
                   "relevance": 0.6, "agree": 1, "n_relevant": 1, "source": "path"} if self.ego else None
            self.r.on_message({**self._hdr("lights"), "lights_ms": 100.0, "frame": [1280, 720], "lights": [], "ego": ego})
        self.out.append(self.r.snapshot(full=True))

    lights_on = False

    def drive(self, metres):
        for _ in range(int(metres / (self.v * 0.2))):
            self.step()

    def sign(self, cls, lateral=3.0, z0=60.0, z1=9.0, score=0.8, aspect=1.0, extra=None, frames=None, suppressed=None):
        self.tid += 1
        tid, ptid = self.tid, self.tid + 1000
        z, k = z0, 0
        n = frames if frames is not None else 10 ** 9
        while z > z1 and k < n:
            h = F * D / z
            w = h * aspect
            u = CX + F * lateral / z
            v = CY - F * 1.5 / z
            d = {"track_id": tid, "cls": cls, "score": score, "box": [u - w / 2, v - h / 2, u + w / 2, v + h / 2],
                 "hits": k + 1, "age": k}
            if suppressed:
                d.update(suppressed=True, reason=suppressed)
            dets = [d]
            if extra:
                dets.append({"track_id": ptid, "cls": extra, "score": 0.7,
                             "box": [u - w / 2, v + h / 2 + 2, u + w / 2, v + h / 2 + 2 + 0.5 * h], "hits": k + 1})
            self.step(dets)
            z -= self.v * 0.2
            k += 1
        ended = [{"track_id": tid, "cls": cls}] + ([{"track_id": ptid, "cls": extra}] if extra else [])
        if suppressed:
            ended[0].update(suppressed=True, reason=suppressed)
        self.step(ended=ended)
        return tid

    def pass_light(self, start_m=40.0, track_id=9):
        """An ego traffic light approached from start_m and then gone (passed overhead)."""
        self.lights_on = True
        d = start_m
        while d > 6:
            self.ego = (d, track_id)
            self.step()
            d -= self.v * 0.2
        self.ego = None
        self.drive(30)
        self.lights_on = False

    @property
    def last(self):
        return self.out[-1]

    def ids(self):
        return [e["id"] for e in self.last["in_effect"]]

    def entry(self, rid):
        return next((e for e in self.last["in_effect"] if e["id"] == rid), None)

    def recent_ids(self):
        return [e["id"] for e in self.last["recent"]]


class RulesTest(unittest.TestCase):
    def test_snapshot_schema_and_env_area(self):
        d = Drive()
        d.drive(300)
        s = d.last
        for k in ("type", "in_effect", "recent", "ahead", "junction", "area", "road_class", "zone", "v_ego_kmh",
                  "odometer_m", "rules", "pending", "recent_changes", "ignored"):
            self.assertIn(k, s)
        self.assertEqual(s["type"], "rules")
        self.assertEqual(d.ids(), ["speed_limit", "built_up_area"])      # 50 km/h default + built-up from envd
        self.assertEqual(d.entry("speed_limit")["value"], 50)
        self.assertIn("envd", d.entry("built_up_area")["uncertain"][0])

    def test_priority_road_until_d2(self):
        d = Drive()
        d.drive(100)
        tid = d.sign("regulatory--priority-road")
        e = d.entry("priority_road")
        self.assertIsNotNone(e)
        self.assertEqual(e["source"]["track_id"], tid)
        self.assertEqual(e["sign"], "D-1")
        d.sign("regulatory--yield", aspect=0.4)        # a yield turned towards the side road: not for us
        d.sign("warning--crossroads")                  # a junction warning does not end D-1
        d.drive(400)
        self.assertIn("priority_road", d.ids())
        d.sign("regulatory--end-of-priority-road")
        self.assertNotIn("priority_road", d.ids())
        self.assertIn("end_of_priority_road", d.recent_ids())
        self.assertIn("D-2", d.last["recent_changes"][0]["event"])

    def test_priority_road_ended_by_facing_yield(self):
        d = Drive()
        d.drive(100)
        d.sign("regulatory--priority-road")
        d.drive(200)
        d.sign("regulatory--yield")
        self.assertNotIn("priority_road", d.ids())
        self.assertIn("yield", d.ids())                # yield binds until the intersection (sign + 15 m)
        d.drive(40)
        self.assertNotIn("yield", d.ids())
        self.assertIn("yield", d.recent_ids())

    def test_no_stopping_until_junction_not_b42(self):
        d = Drive()
        d.drive(100)
        d.sign("regulatory--no-stopping", lateral=3.0)
        e = d.entry("no_stopping")
        self.assertEqual(e["side"], "right")
        self.assertIn("intersection", e["ends"])
        d.sign("regulatory--end-of-prohibition")       # B-42 does not cancel B-36 (par. 32 ust. 1)
        self.assertIn("no_stopping", d.ids())
        d.drive(100)
        d.sign("regulatory--yield", lateral=4.0)       # junction proxy: sign + 15 m
        d.drive(40)
        self.assertNotIn("no_stopping", d.ids())
        self.assertTrue(any("ended: intersection (sign A-7/B-20" in c["event"] for c in d.last["recent_changes"]))

    def test_no_stopping_then_no_parking_then_end(self):
        d = Drive()
        d.drive(100)
        d.sign("regulatory--no-stopping")
        d.drive(50)
        d.sign("regulatory--no-parking")
        self.assertNotIn("no_stopping", d.ids())       # B-36 lasts to the B-35 (par. 28 ust. 4)
        self.assertIn("no_parking", d.ids())
        d.drive(50)
        d.sign("regulatory--end-of-no-parking")
        self.assertNotIn("no_parking", d.ids())
        d.drive(100)
        d.sign("regulatory--no-parking")
        d.sign("regulatory--no-stopping")              # B-36 subsumes B-35
        self.assertEqual([i for i in d.ids() if i in ("no_parking", "no_stopping")], ["no_stopping"])

    def test_no_overtaking_b42_cap_and_truck_plate(self):
        d = Drive(place="rural_road", v=20.0)
        d.drive(300)
        d.sign("regulatory--no-overtaking")
        e = d.entry("no_overtaking")
        self.assertIsNotNone(e)
        self.assertAlmostEqual(e["remaining_m"], 3000, delta=60)    # outside: 3 km cap
        d.drive(500)
        d.sign("regulatory--end-of-prohibition")
        self.assertNotIn("no_overtaking", d.ids())
        d.sign("regulatory--no-overtaking")
        d.drive(2800)
        self.assertIn("no_overtaking", d.ids())
        d.drive(400)
        self.assertNotIn("no_overtaking", d.ids())      # cap without a repeat
        d.sign("regulatory--no-overtaking", extra="complementary--trucks")
        self.assertNotIn("no_overtaking", d.ids())
        self.assertIn("vehicle plate", d.last["ignored"][0]["reason"])
        d.sign("regulatory--no-overtaking", extra="complementary--distance")
        self.assertIn("plate complementary--distance", d.entry("no_overtaking")["uncertain"][0])

    def test_no_left_turn_until_traffic_light_junction_also_ends_b33(self):
        d = Drive()
        d.drive(100)
        d.sign("regulatory--maximum-speed-limit-40")
        d.sign("regulatory--no-left-turn")
        self.assertIn("no_left_turn", d.ids())
        self.assertEqual(d.entry("speed_limit")["value"], 40)
        d.drive(60)
        d.pass_light()
        self.assertEqual(d.last["junction"]["last"]["source"], "traffic light")
        self.assertNotIn("no_left_turn", d.ids())
        self.assertIn("no_left_turn", d.recent_ids())
        self.assertEqual(d.entry("speed_limit")["value"], 50)      # the light also ended the B-33 40

    def test_lights_cue_can_be_switched_off(self):
        d = Drive(cfg={"lights_junction": False})
        d.drive(100)
        d.sign("regulatory--no-left-turn")
        d.drive(60)
        d.pass_light()
        self.assertIsNone(d.last["junction"]["last"])
        self.assertIn("no_left_turn", d.ids())
        d.drive(200)
        self.assertNotIn("no_left_turn", d.ids())                  # 300 m cap

    def test_warning_countdown_and_junction(self):
        d = Drive()
        d.drive(100)
        d.sign("warning--roundabout")
        a = d.last["ahead"]
        self.assertEqual(len(a), 1)
        self.assertEqual(a[0]["sign"], "A-8")
        self.assertAlmostEqual(a[0]["expected_in_m"], 100, delta=15)   # built-up: up to 100 m
        d.drive(60)
        self.assertAlmostEqual(d.last["ahead"][0]["expected_in_m"], 40, delta=15)
        d.drive(200)
        self.assertEqual(d.last["ahead"], [])                       # past the range + grace
        d.sign("warning--roundabout")
        d.sign("regulatory--roundabout")                            # C-12 at the junction ends the warning
        d.drive(30)
        self.assertEqual(d.last["ahead"], [])
        self.assertIn("roundabout", d.recent_ids())

    def test_generic_warning_outside(self):
        d = Drive(place="rural_road", v=20.0)
        d.drive(300)
        d.sign("warning--wild-animals")
        names = {w["id"]: w for w in d.last["ahead"]}
        self.assertIn("animals_ahead", names)
        self.assertAlmostEqual(names["animals_ahead"]["expected_in_m"], 225, delta=30)
        d.sign("warning--texts")
        names = {w["id"]: w for w in d.last["ahead"]}
        self.assertEqual(names["warning_other"]["name"], "texts ahead")
        self.assertGreater(names["animals_ahead"]["range_m"][1], 250)                 # 150-300 m outside

    def test_momentary_no_entry_side_road_and_truck_sign(self):
        d = Drive()
        d.drive(100)
        d.sign("regulatory--no-entry", lateral=12.0)
        self.assertNotIn("no_entry", d.ids())
        r = d.last["recent"][0]
        self.assertEqual(r["id"], "no_entry")
        self.assertIn("side road?", r["uncertain"])
        d.sign("regulatory--no-overtaking-by-heavy-goods-vehicles")
        r = d.last["recent"][0]
        self.assertEqual(r["sign"], "B-26")
        self.assertFalse(r["applies_to_car"])
        d.drive(350)
        self.assertEqual(d.last["recent"], [])                      # 300 m hold

    def test_height_limit_value_unreadable(self):
        d = Drive()
        d.drive(100)
        d.sign("regulatory--height-limit")
        e = d.entry("height_limit")
        self.assertIsNone(e["value"])
        self.assertIn("value not readable", e["uncertain"])

    def test_suppressed_and_short_tracks_are_not_seen(self):
        d = Drive()
        d.drive(100)
        d.sign("regulatory--priority-road", suppressed="traffic_light")
        self.assertNotIn("priority_road", d.ids())
        self.assertIn("suppressed (traffic_light)", d.last["ignored"][0]["reason"])
        d.sign("regulatory--priority-road", frames=2)
        self.assertNotIn("priority_road", d.ids())
        d.sign("regulatory--no-stopping", lateral=14.0)             # other roadway
        self.assertNotIn("no_stopping", d.ids())
        self.assertIn("lateral", d.last["ignored"][0]["reason"])

    def test_repeat_refreshes_and_turn_proxy(self):
        d = Drive()
        d.drive(100)
        d.sign("regulatory--no-stopping")
        since0 = d.entry("no_stopping")["since"]["distance_m"]
        d.drive(200)
        d.sign("regulatory--no-stopping")
        e = d.entry("no_stopping")
        self.assertGreater(e["since"]["distance_m"], since0 + 150)  # since kept
        self.assertGreater(e["remaining_m"], 600)                    # cap restarted
        d.drive(100)
        d.yaw_rate = 1.2                                             # ~70 deg/s: a turn at an intersection
        d.drive(20)
        d.yaw_rate = 0.0
        d.drive(30)
        self.assertNotIn("no_stopping", d.ids())
        self.assertEqual(d.last["junction"]["last"]["source"], "turn")

    def test_priority_road_turn_and_side_road_refresh(self):
        d = Drive(place="rural_road", v=20.0)
        d.drive(300)
        d.sign("regulatory--priority-road")
        d.drive(9000)
        d.sign("warning--junction-with-a-side-road-perpendicular-right")   # A-6b: our road has priority here
        d.drive(2000)
        self.assertIn("priority_road", d.ids())                             # refreshed: the 10 km cap restarted
        d.yaw_rate = 1.2
        d.drive(30)                                                         # turned off at a junction
        d.yaw_rate = 0.0
        d.drive(30)
        self.assertNotIn("priority_road", d.ids())
        self.assertIn("turned off", d.last["recent_changes"][0]["event"])
        d = Drive(cfg={"priority_inference": False}, place="rural_road", v=20.0)
        d.drive(300)
        d.sign("regulatory--priority-road")
        d.yaw_rate = 1.2
        d.drive(30)
        d.yaw_rate = 0.0
        d.drive(30)
        self.assertIn("priority_road", d.ids())                             # pre-registered behaviour

    def test_health_summary_strings(self):
        d = Drive()
        d.drive(100)
        d.sign("regulatory--no-stopping")
        d.sign("warning--pedestrians-crossing")
        s = d.r.summary()
        self.assertIn("B-36 no stopping (R)", s["in_effect"])
        self.assertTrue(s["ahead"][0].startswith("A-16 pedestrian crossing ahead ~"))


if __name__ == "__main__":
    unittest.main()
