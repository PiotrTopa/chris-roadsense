"""Fusion rules (api/roadsense_api/fusion.py) on fabricated `map`, `speed_limit`, `env` and `road` messages.

    python3 -m unittest api.tests.test_fusion -v     (from the repository root)
"""
import copy
import os
import sys
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))

from roadsense_api.fusion import Fusion      # noqa: E402

NS = 10 ** 9
PLACES = ("highway", "city_street", "residential", "rural_road", "forest_road", "tunnel", "parking_lot",
          "underground_parking", "gas_station", "bridge")


def map_msg(seq=1, t=0.0, odo=0.0, limit=None, source="implicit", rule="built_up", conf=0.9, junction_m=None,
            available=True, **road):
    r = {"class": "secondary", "name": "Street", "ref": "799", "oneway": False,
         "lanes": {"total": 2, "forward": 1, "backward": 1, "source": "tagged_total"},
         "tunnel": False, "bridge": False, "covered": False, "lit": True, "surface": "asphalt", "roundabout": False,
         "link": False, "motorroad": False, "dual_carriageway": False, "access_restricted": False, "toll": False,
         "parking": False, "forest": False, "layer": 0, "built_up": {"value": True, "source": "landuse"}}
    r.update(road)
    m = {"type": "map", "seq": seq, "ts_mono_ns": int(t * NS), "api": 1, "state": "good", "available": available,
         "reason": None if available else "no_position",
         "position": {"lat": 52.0, "lon": 21.0, "heading_deg": 0.0, "speed_mps": 14.0, "hacc_m": 3.0,
                      "confidence": 0.95, "source": "gnss", "odometer_m": odo} if available else None,
         "match": {"matched": True, "confidence": conf} if available else None,
         "road": r if available else None,
         "speed_limit": {"limit_kmh": limit if limit is not None else {"built_up": 50, "outside": 90}.get(rule),
                         "source": source if limit is None else "maxspeed", "rule": None if limit is not None else rule,
                         "detail": "tagged" if limit is not None else "built_up:landuse",
                         "confidence": 0.9 if limit is not None else 0.5, "variable": False} if available else None,
         "ahead": {"horizon_m": 1500.0, "path_confidence": 0.8, "features": [], "curves": [],
                   "junctions": [] if junction_m is None else [{"dist_m": junction_m, "kind": "junction", "branches": 3,
                                                                "turn_deg": 0.0, "minor": False, "confidence": 0.8}],
                   "tunnel": {"in_tunnel": r["tunnel"], "exit_m": 640.0 if r["tunnel"] else None, "entry_m": None,
                              "length_m": None},
                   "built_up_change": None, "speed_limit_changes": [], "road_change": None} if available else None}
    return m


def vis_msg(limit=None, kind="default", rule="built_up", track=7, odo=0.0, past_sign_m=None, area="built_up",
            area_src="env", conf=0.7):
    src = {"kind": kind, "rule": rule, "area_source": area_src} if kind != "sign" else \
        {"kind": "sign", "sign": "B-33", "cls": "regulatory--maximum-speed-limit-%d" % limit, "track_id": track}
    return {"type": "speed_limit", "limit_kmh": limit, "source": src, "confidence": conf,
            "expiry": None if kind != "sign" else {"rule": "next intersection", "distance_cap_m": 700,
                                                   "remaining_m": 700 - (past_sign_m or 0.0)},
            "area": {"type": area, "source": area_src, "confidence": 0.4}, "road_class": "ordinary",
            "zone": {"type": "none", "limit_kmh": None}, "odometer_m": odo}


def env_msg(label, conf=0.8):
    rest = (1.0 - conf) / (len(PLACES) - 1)
    return {"type": "env", "place": {"label": label, "confidence": conf, "stable_s": 20.0,
                                     "scores": {p: (conf if p == label else rest) for p in PLACES}}}


def road_msg(probs):
    return {"type": "road", "lane_lines": [{"prob": p} for p in probs]}


class FusionTest(unittest.TestCase):
    def fuse(self, f, t=0.0, **latest):
        return f.update({k: v for k, v in latest.items() if v is not None}, int(t * NS))

    # ---- no map: vision only --------------------------------------------------------------------
    def test_without_mapd_everything_is_the_vision_value(self):
        f = Fusion()
        vis = vis_msg(50, "default", "built_up")
        out = self.fuse(f, speed_limit=vis, env=env_msg("city_street"), road=road_msg([0.9, 0.9, 0.9, 0.1]))
        self.assertEqual(out["map"], {"available": False, "state": None, "reason": "no_mapd", "age_s": None,
                                      "match_confidence": None})
        sl = out["speed_limit"]
        self.assertEqual((sl["limit_kmh"], sl["source"], sl["agreement"]), (50, "vision_default", "vision_only"))
        self.assertEqual(sl["vision"]["limit_kmh"], vis["limit_kmh"])
        self.assertEqual((out["place"]["label"], out["place"]["source"]), ("city_street", "vision"))
        self.assertEqual((out["lanes"]["count"], out["lanes"]["source"]), (None, "none"))
        self.assertEqual(out["lanes"]["vision"], {"lane_to_left": True, "lane_to_right": False, "tracked": False})
        self.assertEqual((out["tunnel"]["in_tunnel"], out["tunnel"]["source"]), (False, "vision"))
        self.assertIsNone(out["road"])
        self.assertIsNone(out["ahead"])

    def test_unavailable_stale_or_poorly_matched_map_is_not_used(self):
        for m, why in ((map_msg(available=False, t=9.0), "no_position"), (map_msg(limit=70, t=0.0), "stale"),
                       (map_msg(limit=70, conf=0.2, t=9.0), "low_match_confidence")):
            f = Fusion()
            out = self.fuse(f, t=10.0, map=m, speed_limit=vis_msg(50))
            self.assertEqual((out["map"]["available"], out["map"]["reason"]), (False, why))
            self.assertEqual((out["speed_limit"]["limit_kmh"], out["speed_limit"]["source"]), (50, "vision_default"))
            self.assertIsNone(out["road"])

    def test_sign_without_map_follows_the_vision_engine(self):
        f = Fusion()
        out = self.fuse(f, speed_limit=vis_msg(40, "sign"))
        self.assertEqual((out["speed_limit"]["limit_kmh"], out["speed_limit"]["source"]), (40, "sign"))
        out = self.fuse(f, t=1.0, speed_limit=vis_msg(50, "default"))          # the vision engine ended it
        self.assertEqual((out["speed_limit"]["limit_kmh"], out["speed_limit"]["source"]), (50, "vision_default"))

    # ---- speed limit ------------------------------------------------------------------------------
    def test_map_limit_when_no_sign(self):
        f = Fusion()
        out = self.fuse(f, map=map_msg(limit=70), speed_limit=vis_msg(50))
        sl = out["speed_limit"]
        self.assertEqual((sl["limit_kmh"], sl["source"], sl["agreement"]), (70, "map", "conflict"))
        self.assertEqual((sl["vision"]["limit_kmh"], sl["map"]["limit_kmh"]), (50, 70))
        out = self.fuse(Fusion(), map=map_msg(rule="outside", built_up={"value": False, "source": "landuse"}),
                        speed_limit=vis_msg(None, "unknown", "unknown_area", area="unknown"))
        sl = out["speed_limit"]
        self.assertEqual((sl["limit_kmh"], sl["source"], sl["rule"], sl["agreement"]), (90, "map_implicit", "outside", "map_only"))

    def test_fresh_sign_overrides_the_map_and_is_held_until_a_map_junction(self):
        f = Fusion()
        # 70 on the map; the camera reads a 40 sign 30 m before reaching it
        out = self.fuse(f, t=0, map=map_msg(1, 0, odo=1000, limit=70, junction_m=400), speed_limit=vis_msg(40, "sign", past_sign_m=-30))
        sl = out["speed_limit"]
        self.assertEqual((sl["limit_kmh"], sl["source"], sl["agreement"]), (40, "sign", "conflict"))
        # past the sign; then the vision engine drops it (its distance cap / a false junction): the sign is held
        out = self.fuse(f, t=5, map=map_msg(2, 5, odo=1070, limit=70, junction_m=330), speed_limit=vis_msg(40, "sign", past_sign_m=40))
        self.assertEqual(out["speed_limit"]["source"], "sign")
        out = self.fuse(f, t=20, map=map_msg(3, 20, odo=1300, limit=70, junction_m=100), speed_limit=vis_msg(50, "default"))
        sl = out["speed_limit"]
        self.assertEqual((sl["limit_kmh"], sl["source"], sl["sign"]), (40, "sign_held", "B-33"))
        out = self.fuse(f, t=26, map=map_msg(4, 26, odo=1395, limit=70, junction_m=5), speed_limit=vis_msg(50, "default"))
        self.assertEqual(out["speed_limit"]["source"], "sign_held")
        # the junction is behind: the map limit is back
        out = self.fuse(f, t=28, map=map_msg(5, 28, odo=1420, limit=70, junction_m=600), speed_limit=vis_msg(50, "default"))
        sl = out["speed_limit"]
        self.assertEqual((sl["limit_kmh"], sl["source"]), (70, "map"))

    def test_junction_between_reading_and_reaching_the_sign_does_not_end_it(self):
        """A sign that stands just after a junction is read before the junction."""
        f = Fusion()
        self.fuse(f, t=0, map=map_msg(1, 0, odo=0, junction_m=20), speed_limit=vis_msg(40, "sign", past_sign_m=-40))
        self.fuse(f, t=2, map=map_msg(2, 2, odo=22, junction_m=500), speed_limit=vis_msg(40, "sign", past_sign_m=-18))
        out = self.fuse(f, t=6, map=map_msg(3, 6, odo=80, junction_m=440), speed_limit=vis_msg(40, "sign", past_sign_m=40))
        self.assertEqual((out["speed_limit"]["limit_kmh"], out["speed_limit"]["source"]), (40, "sign"))

    def test_sign_still_claimed_by_vision_ends_at_a_map_junction(self):
        f = Fusion()
        self.fuse(f, t=0, map=map_msg(1, 0, odo=0, junction_m=300), speed_limit=vis_msg(40, "sign", past_sign_m=30))
        self.fuse(f, t=10, map=map_msg(2, 10, odo=295, junction_m=5), speed_limit=vis_msg(40, "sign", past_sign_m=325))
        out = self.fuse(f, t=12, map=map_msg(3, 12, odo=330, junction_m=None), speed_limit=vis_msg(40, "sign", past_sign_m=360))
        sl = out["speed_limit"]
        self.assertEqual((sl["limit_kmh"], sl["source"], sl["rule"]), (50, "map_implicit", "built_up"))
        self.assertIn("junction passed", sl["note"])
        self.assertEqual(sl["vision"]["limit_kmh"], 40)                       # the unfused value stays visible

    def test_motorway_sign_is_not_ended_by_junctions(self):
        f = Fusion()
        mw = dict(**{"class": "motorway"}, oneway=True, built_up={"value": False, "source": "class"})
        self.fuse(f, t=0, map=map_msg(1, 0, odo=0, junction_m=300, rule="outside", **mw), speed_limit=vis_msg(100, "sign", past_sign_m=30))
        self.fuse(f, t=10, map=map_msg(2, 10, odo=295, junction_m=5, rule="outside", **mw), speed_limit=vis_msg(100, "sign", past_sign_m=325))
        out = self.fuse(f, t=12, map=map_msg(3, 12, odo=330, rule="outside", **mw), speed_limit=vis_msg(100, "sign", past_sign_m=360))
        self.assertEqual((out["speed_limit"]["limit_kmh"], out["speed_limit"]["source"]), (100, "sign"))

    def test_tagged_map_change_after_the_sign_overrides_it(self):
        f = Fusion()
        self.fuse(f, t=0, map=map_msg(1, 0, odo=0, limit=70), speed_limit=vis_msg(40, "sign", past_sign_m=30))
        out = self.fuse(f, t=5, map=map_msg(2, 5, odo=60, limit=50), speed_limit=vis_msg(40, "sign", past_sign_m=90))
        self.assertEqual(out["speed_limit"]["limit_kmh"], 40)                 # too close to the sign: could be map offset
        out = self.fuse(f, t=20, map=map_msg(3, 20, odo=400, limit=50), speed_limit=vis_msg(40, "sign", past_sign_m=430))
        sl = out["speed_limit"]
        self.assertEqual((sl["limit_kmh"], sl["source"]), (50, "map"))
        self.assertIn("map limit changed", sl["note"])

    def test_a_flipping_match_does_not_end_the_sign(self):
        """A map limit that differs for a moment (the match jumps to a side road and back), comes from a doubtful
        or ambiguous match, or has only just appeared, is not a change of the limit."""
        f = Fusion()
        self.fuse(f, t=0, map=map_msg(1, 0, odo=0, limit=50), speed_limit=vis_msg(30, "sign", past_sign_m=30))
        seq = 1
        for i in range(12):                                   # 50 <-> 20 every 15 m, far past the sign
            seq += 1
            odo = 200 + 15 * i
            out = self.fuse(f, t=20 + i, map=map_msg(seq, 20 + i, odo=odo, limit=20 if i % 2 else 50),
                            speed_limit=vis_msg(30, "sign", past_sign_m=odo + 30))
            self.assertEqual((out["speed_limit"]["limit_kmh"], out["speed_limit"]["source"]), (30, "sign"), i)
        for conf, amb in ((0.55, False), (0.9, True)):        # a steady other limit from a match that is not sure
            for i in range(6):
                seq += 1
                odo = 400 + 20 * i
                m = map_msg(seq, 40 + i, odo=odo, limit=20, conf=conf)
                m["match"]["ambiguous"] = amb
                out = self.fuse(f, t=40 + i, map=m, speed_limit=vis_msg(30, "sign", past_sign_m=odo + 30))
                self.assertEqual(out["speed_limit"]["limit_kmh"], 30)
        out = None
        for i in range(4):                                    # sure and lasting: now it is a change
            seq += 1
            odo = 600 + 20 * i
            out = self.fuse(f, t=60 + i, map=map_msg(seq, 60 + i, odo=odo, limit=20),
                            speed_limit=vis_msg(30, "sign", past_sign_m=odo + 30))
        self.assertEqual((out["speed_limit"]["limit_kmh"], out["speed_limit"]["source"]), (20, "map"))

    def test_sign_agreeing_with_the_map_gains_confidence(self):
        a = self.fuse(Fusion(), map=map_msg(limit=40), speed_limit=vis_msg(40, "sign", conf=0.6))["speed_limit"]
        b = self.fuse(Fusion(), map=map_msg(limit=70), speed_limit=vis_msg(40, "sign", conf=0.6))["speed_limit"]
        self.assertEqual((a["agreement"], b["agreement"]), ("agree", "conflict"))
        self.assertGreater(a["confidence"], b["confidence"])

    def test_area_sign_beats_landuse_for_the_implicit_limit(self):
        m = map_msg(rule="built_up")                                           # pack: built-up, inferred from landuse
        out = self.fuse(Fusion(), map=m, speed_limit=vis_msg(90, "default", "outside", area="outside", area_src="sign"))
        sl = out["speed_limit"]
        self.assertEqual((sl["limit_kmh"], sl["source"], sl["rule"]), (90, "map_implicit", "outside"))
        self.assertEqual(out["built_up"]["source"], "sign")
        m = map_msg(rule="built_up", built_up={"value": True, "source": "tagged"})
        out = self.fuse(Fusion(), map=m, speed_limit=vis_msg(90, "default", "outside", area="outside", area_src="env"))
        self.assertEqual((out["built_up"]["value"], out["built_up"]["source"]), (True, "map_tagged"))
        self.assertEqual(out["speed_limit"]["limit_kmh"], 50)

    # ---- place ------------------------------------------------------------------------------------
    def test_map_tunnel_prior_overrides_a_weak_vision_label(self):
        out = self.fuse(Fusion(), map=map_msg(tunnel=True), env=env_msg("city_street", 0.5))
        self.assertEqual(out["place"]["label"], "tunnel")
        self.assertEqual(out["place"]["source"], "fused(map)")
        self.assertEqual(out["place"]["vision"]["label"], "city_street")
        self.assertEqual((out["tunnel"]["in_tunnel"], out["tunnel"]["exit_in_m"], out["tunnel"]["source"]), (True, 640.0, "map"))

    def test_place_prior_biases_but_strong_vision_wins(self):
        m = map_msg(forest=True, built_up={"value": False, "source": "landuse"})
        env = env_msg("rural_road", 0.45)
        env["place"]["scores"].update(forest_road=0.30, city_street=0.05)     # vision torn between rural and forest
        out = self.fuse(Fusion(), map=m, env=env)
        self.assertEqual((out["place"]["label"], out["place"]["source"]), ("forest_road", "fused(map)"))
        out = self.fuse(Fusion(), map=map_msg(), env=env_msg("gas_station", 0.97))
        self.assertEqual((out["place"]["label"], out["place"]["source"]), ("gas_station", "fused(vision)"))
        out = self.fuse(Fusion(), map=map_msg(**{"class": "motorway"}))       # no envd at all
        self.assertEqual((out["place"]["label"], out["place"]["source"]), ("highway", "map"))

    def test_vision_tunnel_without_map_tunnel(self):
        out = self.fuse(Fusion(), map=map_msg(), env=env_msg("tunnel", 0.9))
        self.assertEqual((out["tunnel"]["in_tunnel"], out["tunnel"]["source"], out["tunnel"]["exit_in_m"]), (True, "vision", None))

    # ---- lanes ------------------------------------------------------------------------------------
    def test_lanes_use_the_tracked_presence_when_fresh(self):
        # the frame says "a lane to the right" (far-right 0.9), the tracker (hysteresis over time) says no
        dual = map_msg(oneway=True, lanes={"total": 2, "forward": 2, "backward": 0, "source": "tagged"})
        raw = road_msg([0.1, 0.9, 0.9, 0.9])
        trk = {"type": "lanes", "ts_mono_ns": 0, "lane_to_left": True, "lane_to_right": False}
        out = self.fuse(Fusion(), map=dual, road=raw, lanes=trk)["lanes"]
        self.assertEqual(out["vision"], {"lane_to_left": True, "lane_to_right": False, "tracked": True})
        self.assertEqual(out["ego_index"], 1)                                   # nothing to the right: the right lane
        stale = dict(trk, ts_mono_ns=-5 * NS)
        out = self.fuse(Fusion(), map=dual, road=raw, lanes=stale)["lanes"]     # an old tracker message: the frame decides
        self.assertEqual(out["vision"], {"lane_to_left": False, "lane_to_right": True, "tracked": False})
        self.assertEqual(out["ego_index"], 0)

    def test_lanes(self):
        two_way = map_msg()
        out = self.fuse(Fusion(), map=two_way, road=road_msg([0.8, 0.9, 0.9, 0.1]))["lanes"]
        self.assertEqual((out["count"], out["forward"], out["ego_index"], out["oncoming_left"]), (2, 1, 0, True))
        dual = map_msg(oneway=True, lanes={"total": 2, "forward": 2, "backward": 0, "source": "tagged"})
        out = self.fuse(Fusion(), map=dual, road=road_msg([0.9, 0.9, 0.9, 0.1]))["lanes"]      # a lane to the left only
        self.assertEqual((out["count"], out["ego_index"], out["oncoming_left"]), (2, 1, False))
        out = self.fuse(Fusion(), map=dual, road=road_msg([0.1, 0.9, 0.9, 0.9]))["lanes"]      # a lane to the right
        self.assertEqual((out["ego_index"], out["oncoming_left"]), (0, False))
        four = map_msg(lanes={"total": 4, "forward": 2, "backward": 2, "source": "tagged"})
        out = self.fuse(Fusion(), map=four, road=road_msg([0.9, 0.9, 0.9, 0.9]))["lanes"]      # left lane of two: oncoming beside
        self.assertEqual((out["ego_index"], out["oncoming_left"]), (0, True))
        out = self.fuse(Fusion(), map=four, road=road_msg([0.9, 0.9, 0.9, 0.1]))["lanes"]
        self.assertEqual((out["ego_index"], out["oncoming_left"]), (1, False))
        out = self.fuse(Fusion(), map=four)["lanes"]                                             # no model output
        self.assertEqual((out["count"], out["ego_index"], out["oncoming_left"]), (4, None, None))

    # ---- match-confidence hold ----------------------------------------------------------------------
    def drive(self, f, msgs, vis=None):
        """Feed map messages 0.5 s apart (mapd's 2 per second); the fused limit (value, source) after each."""
        out = []
        for i, kw in enumerate(msgs):
            t = 0.5 * i
            m = map_msg(seq=i + 1, t=t, odo=7.0 * i, **kw)
            r = self.fuse(f, t=t + 0.05, map=m, speed_limit=vis or vis_msg(50))
            out.append(((r["speed_limit"]["limit_kmh"], r["speed_limit"]["source"]), r["map"]))
        return out

    def test_a_short_dip_at_a_side_road_keeps_the_map_limit(self):
        f = Fusion()
        out = self.drive(f, [dict(limit=70, conf=0.9), dict(limit=70, conf=0.9), dict(limit=70, conf=0.35),
                             dict(limit=70, conf=0.42), dict(limit=70, conf=0.9)])
        self.assertEqual([o[0] for o in out], [(70, "map")] * 5)
        held = out[2][1]
        self.assertTrue(held["available"])
        self.assertIsNone(held["reason"])
        self.assertEqual(held["match_confidence"], 0.35)                 # the true value is reported
        self.assertEqual(held["match_hold"], {"messages": 1, "age_s": 0.5, "confidence": 0.9})
        self.assertEqual(out[3][1]["match_hold"]["messages"], 2)
        self.assertNotIn("match_hold", out[4][1])
        # the facts use the last good confidence, so the limit's confidence does not dip either
        r = self.fuse(Fusion(), t=0.05, map=map_msg(limit=70, conf=0.9), speed_limit=vis_msg(50))
        self.assertEqual(r["speed_limit"]["confidence"], round(0.9 * 0.9, 2))

    def test_implicit_limit_is_held_too(self):
        out = self.drive(Fusion(), [dict(conf=0.8), dict(conf=0.3), dict(conf=0.8)])
        self.assertEqual([o[0] for o in out], [(50, "map_implicit")] * 3)

    def test_no_hold_across_a_change_of_road(self):
        for change in (dict(name="Side", ref=None), dict(**{"class": "residential"}), dict(name=None, ref=None)):
            out = self.drive(Fusion(), [dict(limit=70, conf=0.9), dict(limit=70, conf=0.3, **change)])
            self.assertEqual(out[1][0], (50, "vision_default"), change)
            self.assertEqual(out[1][1]["reason"], "low_match_confidence")

    def test_no_hold_when_the_map_limit_changes(self):
        out = self.drive(Fusion(), [dict(limit=70, conf=0.9), dict(limit=50, conf=0.3)])
        self.assertEqual(out[1][0], (50, "vision_default"))
        self.assertEqual(out[1][1]["reason"], "low_match_confidence")

    def test_same_edge_holds_an_unnamed_road(self):
        f = Fusion()
        ms = [map_msg(seq=1, t=0.0, limit=70, conf=0.9, name=None, ref=None),
              map_msg(seq=2, t=0.5, limit=70, conf=0.3, name=None, ref=None)]
        for m in ms:
            m["match"].update(edge="ab:1", dir=1)
        self.fuse(f, t=0.05, map=ms[0], speed_limit=vis_msg(50))
        r = self.fuse(f, t=0.55, map=ms[1], speed_limit=vis_msg(50))
        self.assertEqual((r["speed_limit"]["limit_kmh"], r["speed_limit"]["source"]), (70, "map"))
        ms[1]["match"]["edge"] = "ab:2"                                   # another unnamed edge: not the same road
        f = Fusion()
        self.fuse(f, t=0.05, map=ms[0], speed_limit=vis_msg(50))
        r = self.fuse(f, t=0.55, map=ms[1], speed_limit=vis_msg(50))
        self.assertEqual(r["speed_limit"]["source"], "vision_default")

    def test_no_hold_after_a_loss_of_position(self):
        out = self.drive(Fusion(), [dict(limit=70, conf=0.9), dict(available=False), dict(limit=70, conf=0.3)])
        self.assertEqual([o[0] for o in out], [(70, "map"), (50, "vision_default"), (50, "vision_default")])
        self.assertEqual(out[2][1]["reason"], "low_match_confidence")
        # a stale map in between ends it the same way
        f = Fusion()
        self.fuse(f, t=0.05, map=map_msg(seq=1, t=0.0, limit=70, conf=0.9), speed_limit=vis_msg(50))
        r = self.fuse(f, t=3.5, map=map_msg(seq=1, t=0.0, limit=70, conf=0.9), speed_limit=vis_msg(50))
        self.assertEqual(r["map"]["reason"], "stale")
        r = self.fuse(f, t=1.05, map=map_msg(seq=2, t=1.0, limit=70, conf=0.3), speed_limit=vis_msg(50))
        self.assertEqual(r["speed_limit"]["source"], "vision_default")

    def test_hold_is_short(self):
        # at most match_hold_msgs low messages in a row, and within match_hold_s of the last good one
        out = self.drive(Fusion(), [dict(limit=70, conf=0.9)] + [dict(limit=70, conf=0.3)] * 5)
        self.assertEqual([o[0][1] for o in out], ["map"] * 4 + ["vision_default"] * 2)
        f = Fusion(cfg={"match_hold_msgs": 10})
        out = self.drive(f, [dict(limit=70, conf=0.9)] + [dict(limit=70, conf=0.3)] * 6)
        self.assertEqual([o[0][1] for o in out], ["map"] * 5 + ["vision_default"] * 2)     # 2.0 s
        # once it has run out it does not come back without a confident message
        out = self.drive(Fusion(), [dict(limit=70, conf=0.9)] + [dict(limit=70, conf=0.3)] * 4 +
                         [dict(limit=70, conf=0.9), dict(limit=70, conf=0.3)])
        self.assertEqual([o[0][1] for o in out], ["map"] * 4 + ["vision_default", "map", "map"])

    def test_a_repeated_low_message_counts_once(self):
        f = Fusion()
        self.fuse(f, t=0.05, map=map_msg(seq=1, t=0.0, limit=70, conf=0.9), speed_limit=vis_msg(50))
        low = map_msg(seq=2, t=0.5, limit=70, conf=0.3)
        for i in range(6):                                              # fusion runs on other topics too
            r = self.fuse(f, t=0.55 + 0.1 * i, map=low, speed_limit=vis_msg(50))
            self.assertEqual(r["map"]["match_hold"]["messages"], 1)

    def test_inputs_are_not_modified(self):
        latest = {"map": map_msg(limit=70), "speed_limit": vis_msg(40, "sign"), "env": env_msg("city_street"),
                  "road": road_msg([0.9, 0.9, 0.9, 0.9])}
        before = copy.deepcopy(latest)
        Fusion().update(latest, 0)
        self.assertEqual(latest, before)


if __name__ == "__main__":
    unittest.main()
