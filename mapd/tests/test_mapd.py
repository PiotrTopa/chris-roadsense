"""mapd unit tests on a hand-made network (synthpack.py): pack format, filter, matcher, states, limits,
look-ahead, input validation.

    python3 -m unittest discover -s mapd/tests -t mapd -v
"""
import math
import os
import random
import shutil
import tempfile
import unittest

from roadsense_mapd import context, geo, rsmap as R, sim
from roadsense_mapd.engine import Engine
from roadsense_mapd.matcher import MapMatcher
from roadsense_mapd.posfilter import PositionFilter

from . import synthpack as SP

T0 = 10 ** 12
NS = 10 ** 9


class PackCase(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.tmp = tempfile.mkdtemp(prefix="roadsense-mapd-test-")
        cls.path = os.path.join(cls.tmp, "synth.rsmap")
        SP.build(cls.path)
        cls.pack = R.Pack(cls.path)
        cls.limits = context.load_limits("pl")

    @classmethod
    def tearDownClass(cls):
        cls.pack.close()
        shutil.rmtree(cls.tmp, ignore_errors=True)

    def edge_at(self, x, y, name=None):
        lat, lon = SP.ll(x, y)
        best = None
        for e in self.pack.query(lat, lon, 30):
            d, s, _ = e.project(lat, lon)
            if (name is None or e.name == name) and (best is None or d < best[0]):
                best = (d, e, s)
        self.assertIsNotNone(best, "no edge at %s,%s" % (x, y))
        return best[1], best[2]

    def run_drive(self, eng, path, hz=2.0, on_step=None, **feed):
        drive = sim.Drive(self.pack, path)
        last = None
        outs = []
        for ts, msg, o in sim.feed(drive, t0_ns=T0, tick_hz=hz, **feed):
            if msg is not None:
                self.assertIsNone(eng.on_input(msg, ts))
            elif last is None or ts - last >= 0.99 * NS / hz:
                last = ts
                out = eng.step(ts)
                outs.append((o, out))
                if on_step:
                    on_step(o, out)
        return outs


class TestPack(PackCase):
    def test_manifest_and_counts(self):
        m = self.pack.manifest
        self.assertEqual(m["format"], 1)
        self.assertEqual(m["edges"], len(SP.edges()))
        self.assertGreater(m["tiles"], 1)

    def test_roundtrip_attributes(self):
        e, s = self.edge_at(100, 0)
        self.assertEqual((e.cls_name, e.name, e.road_ref, e.ms_fwd, e.lanes), ("primary", "Main", "DK1", 70, 2))
        self.assertAlmostEqual(e.length, 200.0, delta=0.5)
        self.assertAlmostEqual(s, 100.0, delta=0.5)
        self.assertTrue(e.flags & R.F_BUILTUP_TAGGED)

    def test_query_across_tile_border(self):
        # the main road crosses the lon 21.0 tile border ~685 m east of the origin: the edge m500-m900 lives in
        # the western tile and must be found from the eastern one
        lat, lon = SP.ll(800, 0)
        self.assertNotEqual(R.tile_key(lat, lon), R.tile_key(*SP.ll(500, 0)))
        e, _ = self.edge_at(800, 0, "Main")
        self.assertEqual(e.ref[0], R.tile_key(*SP.ll(500, 0)))

    def test_graph_is_consistent(self):
        for k in self.pack.index["key"]:
            t = self.pack.tile(int(k))
            for i in range(len(t.edges)):
                e = t.edge(i)
                self.assertIn((e.ref, 0), self.pack.node_edges(e.a))
                self.assertIn((e.ref, 1), self.pack.node_edges(e.b))
        e, _ = self.edge_at(350, 0)
        self.assertEqual(len(self.pack.node_edges(e.b)), 3)       # m500: the main road twice and the side street

    def test_projection(self):
        e, _ = self.edge_at(350, 0)
        d, s, brg = e.project(*SP.ll(300, 7))
        self.assertAlmostEqual(d, 7.0, delta=0.3)
        self.assertAlmostEqual(s, 100.0, delta=0.5)
        self.assertAlmostEqual(brg, 90.0, delta=0.5)

    def test_rejects_foreign_file(self):
        p = os.path.join(self.tmp, "junk.rsmap")
        with open(p, "wb") as f:
            f.write(b"x" * 200)
        with self.assertRaises(ValueError):
            R.Pack(p)


class TestFilter(unittest.TestCase):
    def test_converges_and_reports_uncertainty(self):
        rnd = random.Random(3)
        f = PositionFilter()
        lat, lon = 52.0, 21.0
        v, hd = 15.0, 60.0
        f.init(lat, lon, 8.0, speed=v, course_deg=hd)
        for i in range(60):
            f.predict(1.0, 0.0, speed_measured=True)
            f.update_speed(v)
            lat, lon = geo.offset(lat, lon, v * math.cos(math.radians(hd)), v * math.sin(math.radians(hd)))
            zl = geo.offset(lat, lon, rnd.gauss(0, 3), rnd.gauss(0, 3))
            self.assertTrue(f.update_position(zl[0], zl[1], 5.0))
            f.update_course(hd + rnd.gauss(0, 2), 3.0)
        self.assertLess(geo.dist_m(*f.latlon, lat, lon), 5.0)
        self.assertLess(f.hacc, 4.0)
        self.assertLess(abs(geo.ang_diff(f.heading_deg, hd)), 3.0)

    def test_dead_reckoning_through_a_turn(self):
        f = PositionFilter()
        f.init(52.0, 21.0, 3.0, speed=10.0, course_deg=0.0)
        # 100 m north, then a 90 degree right turn over 10 s (yaw rate 9 deg/s), then 100 m east
        for _ in range(100):
            f.predict(0.1, 0.0, speed_measured=True)
            f.update_speed(10.0)
        for _ in range(100):
            f.predict(0.1, math.radians(9.0), speed_measured=True)
            f.update_speed(10.0)
        for _ in range(100):
            f.predict(0.1, 0.0, speed_measured=True)
            f.update_speed(10.0)
        r = 100.0 / (math.pi / 2)
        want = geo.offset(52.0, 21.0, 100.0 + r, r + 100.0)
        self.assertLess(geo.dist_m(*f.latlon, *want), 2.0)
        self.assertAlmostEqual(f.heading_deg, 90.0, delta=0.5)
        self.assertGreater(f.hacc, 3.0)                      # uncertainty grew without fixes

    def test_outlier_is_gated_and_a_persistent_offset_resets(self):
        f = PositionFilter()
        f.init(52.0, 21.0, 3.0, speed=0.0, course_deg=0.0)
        for _ in range(5):
            f.predict(1.0, 0.0, speed_measured=True)
            f.update_speed(0.0)
            self.assertTrue(f.update_position(52.0, 21.0, 4.0))
        far = geo.offset(52.0, 21.0, 300.0, 0.0)
        self.assertFalse(f.update_position(far[0], far[1], 4.0))          # one jump: ignored
        self.assertLess(geo.dist_m(*f.latlon, 52.0, 21.0), 3.0)
        self.assertTrue(f.update_position(52.0, 21.0, 4.0))
        used = [f.update_position(far[0], far[1], 4.0) for _ in range(6)]
        self.assertTrue(used[-1])                                         # the fixes insist: the filter follows
        self.assertEqual(f.resets, 1)
        self.assertLess(geo.dist_m(*f.latlon, *far), 6.0)


class TestMatcher(PackCase):
    def test_follows_the_main_road(self):
        eng = Engine(self.pack)
        path = SP.main_road_path(self.pack)
        on_path = {e.ref for e, _, _, _ in path}
        outs = self.run_drive(eng, path, tunnel_blocks_gnss=False)
        avail = [out for _, out in outs if out["available"]]
        self.assertGreater(len(avail), 0.97 * len(outs))
        wrong = [o.t for o, out in outs if out["available"] and eng.pack.edge(self._ref(out)).ref not in on_path]
        self.assertEqual(wrong, [])

    @staticmethod
    def _ref(out):
        t, i = out["match"]["edge"].split(":")
        return int(t, 16), int(i)

    def test_parallel_road_does_not_capture_the_match(self):
        """A multipath excursion: for 12 s the fixes sit 12 m south of the main road, 3 m from the service road
        that runs beside it. Continuity keeps the match on the main road."""
        eng = Engine(self.pack)
        path = SP.main_road_path(self.pack, 20, 1500)
        names = []
        self.run_drive(eng, path, bias_m=lambda t: (0.0, -12.0) if 18.0 <= t < 30.0 else (0.0, 0.0), gnss_sigma_m=1.0,
                       on_step=lambda o, out: names.append((round(o.t), out["road"]["name"])) if out["available"] else None)
        self.assertGreater(len(names), 50)
        self.assertEqual({n for _, n in names}, {"Main"}, [x for x in names if x[1] != "Main"])

    def test_wrong_match_corrects_itself_over_distance(self):
        m = MapMatcher(self.pack)
        got = m.update(*SP.ll(400, -15), 90.0, 15.0, 4.0, 0.0)           # starts on the access road
        self.assertEqual(got.edge.name, "Access")
        names = []
        for i in range(1, 60):                                           # but the car drives the main road
            got = m.update(*SP.ll(400 + 7 * i, 0), 90.0, 15.0, 4.0, 7.0)
            names.append(got.edge.name)
        self.assertEqual(names[10], "Access")                            # not at once (70 m)
        self.assertEqual(names[45], "Main")                              # but after 250 m of evidence

    def test_turning_onto_the_parallel_road_is_followed(self):
        eng = Engine(self.pack)
        path = sim.route(self.pack, SP.ll(20, 0), SP.ll(700, -15))        # main road, then the access road
        self.assertIn("Access", [e.name for e, _, _, _ in path])
        names = []
        self.run_drive(eng, path, gnss_sigma_m=1.5,
                       on_step=lambda o, out: names.append((o.edge.name, out["road"]["name"])) if out["available"] else None)
        tail = names[-10:]
        self.assertTrue(all(m == "Access" for _, m in tail), tail)

    def test_far_jump_rematches(self):
        m = MapMatcher(self.pack)
        self.assertEqual(m.update(*SP.ll(1500, 0), 90.0, 15.0, 4.0, 0.0).edge.name, "Main")
        got = m.update(*SP.ll(1500, 2000), 90.0, 15.0, 4.0, 10.0)        # 2 km away: nothing of the old road in reach
        self.assertEqual(got.edge.road_ref, "A9")

    def test_off_road(self):
        m = MapMatcher(self.pack)
        lat, lon = SP.ll(1500, 800)
        self.assertIsNone(m.update(lat, lon, 90.0, 10.0, 4.0, 0.0))

    def test_one_way_is_not_matched_against_traffic(self):
        m = MapMatcher(self.pack)
        lat, lon = SP.ll(1500, 2000)
        got = m.update(lat, lon, 90.0, 20.0, 4.0, 0.0)
        self.assertEqual((got.edge.road_ref, got.dir), ("A9", 1))
        m.reset()
        got = m.update(lat, lon, 270.0, 20.0, 4.0, 0.0)                  # heading against the one-way
        self.assertLess(got.confidence, 0.05)


class TestStates(PackCase):
    def test_no_source_means_unavailable(self):
        eng = Engine(self.pack)
        out = eng.step(T0)
        self.assertEqual((out["state"], out["available"], out["reason"]), ("no_source", False, "no_position"))
        for k in ("position", "road", "speed_limit", "ahead", "match"):
            self.assertIsNone(out[k])
        # motion alone (no fix ever) does not make a position
        eng.on_input({"type": "motion", "ts_mono_ns": T0, "speed_mps": 10.0, "yaw_rate_rps": 0.0}, T0)
        self.assertEqual(eng.step(T0 + NS)["state"], "no_source")

    def test_no_pack(self):
        eng = Engine(None)
        lat, lon = SP.ll(100, 0)
        eng.on_input({"type": "gnss", "ts_mono_ns": T0, "lat": lat, "lon": lon, "hacc_m": 4}, T0)
        out = eng.step(T0)
        self.assertEqual((out["state"], out["available"], out["reason"]), ("good", False, "no_pack"))
        self.assertIsNotNone(out["position"])

    def test_fix_lost_without_motion_decays_to_unknown(self):
        eng = Engine(self.pack)
        lat, lon = SP.ll(100, 0)
        for i in range(5):
            t = T0 + i * NS
            la, lo = SP.ll(100 + 15 * i, 0)
            self.assertIsNone(eng.on_input({"type": "gnss", "ts_mono_ns": t, "lat": la, "lon": lo, "hacc_m": 4,
                                            "speed_mps": 15.0, "course_deg": 90.0}, t))
            eng.step(t)
        seen = []
        for i in range(5, 30):
            out = eng.step(T0 + i * NS)
            seen.append(out["state"])
        self.assertEqual(seen[0], "good")
        self.assertIn("dead_reckoning", seen)                 # coasting for a few seconds
        self.assertEqual(seen[-1], "unknown")
        self.assertLess(seen.index("unknown"), 12)            # fix_timeout 3 s + coast 5 s
        out = eng.step(T0 + 31 * NS)
        self.assertEqual((out["available"], out["reason"], out["position"]), (False, "position_unknown", None))
        # a new fix brings it back
        la, lo = SP.ll(700, 0)
        eng.on_input({"type": "gnss", "ts_mono_ns": T0 + 32 * NS, "lat": la, "lon": lo, "hacc_m": 4,
                      "speed_mps": 15.0, "course_deg": 90.0}, T0 + 32 * NS)
        out = eng.step(T0 + 32 * NS)
        self.assertEqual((out["state"], out["available"]), ("good", True))

    def test_explicit_no_fix(self):
        eng = Engine(self.pack)
        la, lo = SP.ll(100, 0)
        eng.on_input({"type": "gnss", "ts_mono_ns": T0, "lat": la, "lon": lo, "hacc_m": 4, "speed_mps": 0.0}, T0)
        self.assertEqual(eng.step(T0)["state"], "good")
        eng.on_input({"type": "gnss", "ts_mono_ns": T0 + NS // 2, "fix": "none"}, T0 + NS // 2)
        self.assertEqual(eng.step(T0 + NS)["state"], "dead_reckoning")

    def test_tunnel_dead_reckoning_with_vehicle_motion(self):
        """The fix is lost for the whole 800 m tunnel; speed + yaw rate and the map carry the position."""
        eng = Engine(self.pack)
        path = SP.main_road_path(self.pack, 2000, 5600)
        rows = []
        self.run_drive(eng, path, on_step=lambda o, out: rows.append((o, out)))
        in_tunnel = [(o, out) for o, out in rows if o.tunnel]
        self.assertGreater(len(in_tunnel), 20)
        self.assertTrue(all(out["state"] == "dead_reckoning" for o, out in in_tunnel[8:]))
        self.assertTrue(all(out["available"] and out["road"]["tunnel"] for o, out in in_tunnel[8:-2]))
        errs = [geo.dist_m(out["position"]["lat"], out["position"]["lon"], o.lat, o.lon) for o, out in in_tunnel]
        self.assertLess(max(errs), 25.0)                       # 1 % speed scale error over 800 m + start error
        exits = [out["ahead"]["tunnel"]["exit_m"] for o, out in in_tunnel[8:-2]]
        self.assertTrue(all(a > b for a, b in zip(exits, exits[1:])), "exit distance must count down")
        truth_left = SP.ORIGIN and geo.dist_m(in_tunnel[10][0].lat, in_tunnel[10][0].lon, *SP.ll(3800, 0))
        self.assertAlmostEqual(in_tunnel[10][1]["ahead"]["tunnel"]["exit_m"], truth_left, delta=25.0)
        self.assertEqual(rows[-1][1]["state"], "good")         # the fix is back after the tunnel

    def test_tunnel_with_steering_angle_only(self):
        eng = Engine(self.pack)
        rows = []
        self.run_drive(eng, SP.main_road_path(self.pack, 2000, 5600), motion="steer",
                       on_step=lambda o, out: rows.append((o, out)))
        tun = [(o, out) for o, out in rows if o.tunnel]
        self.assertTrue(all(out["state"] == "dead_reckoning" and out["available"] for o, out in tun[8:-2]))

    def test_delayed_fix_is_carried_forward(self):
        """A fix that reaches mapd 0.4 s after its measurement time (ts_mono_ns says so) must not drag the
        position 8 m back at 20 m/s."""
        eng = Engine(self.pack)
        pending = []
        for i in range(400):                                  # 20 s at 20 Hz, 20 m/s eastwards from x = 100
            t = T0 + i * NS // 20
            x = 100 + 20.0 * i / 20
            eng.on_input({"type": "motion", "ts_mono_ns": t, "speed_mps": 20.0, "yaw_rate_rps": 0.0}, t)
            if i % 20 == 0:
                la, lo = SP.ll(x, 0)
                pending.append((t + 4 * NS // 10, {"type": "gnss", "ts_mono_ns": t, "lat": la, "lon": lo, "hacc_m": 3.0,
                                                   "speed_mps": 20.0, "course_deg": 90.0}))
            while pending and pending[0][0] <= t:
                self.assertIsNone(eng.on_input(pending.pop(0)[1], t))
        out = eng.step(t)
        self.assertLess(geo.dist_m(out["position"]["lat"], out["position"]["lon"], *SP.ll(x, 0)), 2.0)

    def test_long_dead_reckoning_runs_out(self):
        eng = Engine(self.pack, {"dr_max_s": {"vehicle": 20.0, "vision": 5.0}})
        states = []
        self.run_drive(eng, SP.main_road_path(self.pack, 20, 3000), dropouts=[(20.0, 1e9)],
                       on_step=lambda o, out: states.append(out["state"]))
        self.assertEqual(states[-1], "unknown")
        self.assertIn("dead_reckoning", states)

    def test_vision_odometry_is_the_fallback_motion_source(self):
        eng = Engine(self.pack)
        t = T0
        for i in range(5):
            la, lo = SP.ll(100 + 15 * i, 0)
            eng.on_input({"type": "gnss", "ts_mono_ns": t, "lat": la, "lon": lo, "hacc_m": 4, "speed_mps": 15.0,
                          "course_deg": 90.0}, t)
            eng.step(t)
            t += NS
        out = None
        for i in range(200):                                  # 20 s of modeld `road` messages, no fix, no car data
            t += NS // 10
            eng.on_vision({"type": "road", "ts_mono_ns": t, "v_ego": 15.0, "pose": {"yaw_rate": 0.0}}, t)
            if i % 5 == 0:
                out = eng.step(t)
        self.assertEqual((out["state"], out["position"]["motion_source"]), ("dead_reckoning", "vision"))
        want = SP.ll(100 + 15 * 4 + 15.0 * 21.0, 0)
        self.assertLess(geo.dist_m(out["position"]["lat"], out["position"]["lon"], *want), 30.0)
        # vehicle motion takes over as soon as it arrives
        eng.on_input({"type": "motion", "ts_mono_ns": t, "speed_mps": 15.0, "yaw_rate_rps": 0.0}, t)
        eng.on_vision({"type": "road", "ts_mono_ns": t, "v_ego": 99.0, "pose": {"yaw_rate": 0.0}}, t)
        self.assertEqual(eng.step(t)["position"]["motion_source"], "vehicle")


class TestLimits(PackCase):
    def lim(self, x, y, direction=1):
        e, _ = self.edge_at(x, y)
        return context.speed_limit(e, direction, self.limits)

    def test_tagged_maxspeed(self):
        got = self.lim(100, 0)
        self.assertEqual((got["limit_kmh"], got["source"], got["detail"]), (70, "maxspeed", "tagged"))
        got = self.lim(4400, 0)
        self.assertEqual((got["limit_kmh"], got["source"], got["detail"]), (90, "maxspeed", "sign"))

    def test_implicit_polish_defaults(self):
        cases = {(1500, 0): (50, "built_up"), (2500, 0): (90, "outside"), (1500, 2000): (140, "motorway"),
                 (100, 3000): (20, "residential_zone"), (500, 4000): (120, "expressway"),
                 (500, 4100): (100, "expressway_single"), (500, 4200): (100, "outside_dual_2x2"),
                 (100, 4300): (None, "link"), (500, 4500): (120, "expressway")}
        for (x, y), (kmh, rule) in cases.items():
            got = self.lim(x, y)
            self.assertEqual((got["limit_kmh"], got["source"], got["rule"]), (kmh, "implicit", rule), (x, y))

    def test_built_up_source_is_marked(self):
        e, _ = self.edge_at(100, 0)
        self.assertEqual(context.built_up(e), {"value": True, "source": "tagged"})
        e, _ = self.edge_at(1500, 0)
        self.assertEqual(context.built_up(e), {"value": True, "source": "landuse"})
        self.assertEqual(self.lim(1500, 0)["detail"], "built_up:landuse")
        self.assertLess(self.lim(1500, 0)["confidence"], self.lim(100, 0)["confidence"])

    def test_lanes(self):
        e, _ = self.edge_at(100, 0)
        self.assertEqual(context.lanes(e, 1), {"total": 2, "forward": 1, "backward": 1, "source": "tagged_total"})
        e, _ = self.edge_at(1500, 2000)
        self.assertEqual(context.lanes(e, 1), {"total": 2, "forward": 2, "backward": 0, "source": "tagged"})
        e, _ = self.edge_at(250, 4400)
        self.assertEqual(context.lanes(e, 1)["forward"], 2)
        self.assertEqual(context.lanes(e, -1)["forward"], 1)
        e, _ = self.edge_at(500, 200)
        self.assertEqual(context.lanes(e, 1)["source"], "assumed")


class TestLookAhead(PackCase):
    def ahead(self, x, heading=90.0, horizon=6000.0):
        m = MapMatcher(self.pack)
        got = m.update(*SP.ll(x, 0), heading, 15.0, 3.0, 0.0)
        self.assertIsNotNone(got)
        return context.look_ahead(self.pack, got, self.limits, horizon_m=horizon)

    def test_features_and_junctions(self):
        a = self.ahead(100)
        feats = [(f["kind"], round(f["dist_m"])) for f in a["features"]]
        self.assertEqual(feats, [("traffic_signals", 900), ("crossing", 1100), ("level_crossing", 2400)])
        j = [(x["kind"], round(x["dist_m"]), x["minor"]) for x in a["junctions"]]
        self.assertEqual(j[:3], [("junction", 100, True), ("junction", 400, False), ("junction", 800, True)])
        self.assertEqual(a["junctions"][-1]["kind"], "dead_end")
        self.assertGreater(a["path_confidence"], 0.5)

    def test_tunnel_distances(self):
        t = self.ahead(100)["tunnel"]
        self.assertEqual((t["in_tunnel"], round(t["entry_m"]), round(t["length_m"])), (False, 2900, 800))
        t = self.ahead(3300)["tunnel"]
        self.assertEqual((t["in_tunnel"], round(t["exit_m"]), t["entry_m"]), (True, 500, None))

    def test_limit_and_built_up_changes(self):
        a = self.ahead(100)
        ch = [(c["limit_kmh"], round(c["dist_m"]), c["source"]) for c in a["speed_limit_changes"]]
        self.assertEqual(ch, [(50, 900, "implicit"), (90, 1900, "implicit"), (80, 2900, "maxspeed"), (90, 3700, "maxspeed")])
        self.assertEqual((a["built_up_change"]["to"], round(a["built_up_change"]["dist_m"])), (False, 1900))

    def test_curve_beyond_camera_range(self):
        a = self.ahead(4700)
        self.assertEqual(len(a["curves"]), 1)
        c = a["curves"][0]
        self.assertEqual(c["direction"], "left")
        self.assertAlmostEqual(c["min_radius_m"], 150.0, delta=25.0)
        self.assertAlmostEqual(c["start_m"], 300.0, delta=30.0)
        self.assertAlmostEqual(c["angle_deg"], -90.0, delta=12.0)
        sh = a["shape"]
        self.assertEqual(len(sh["s"]), len(sh["x"]))
        self.assertAlmostEqual(sh["x"][5], 100.0, delta=1.0)          # straight ahead first
        self.assertLess(sh["y"][-1], -50.0)                           # then to the left (y is right)

    def test_direction_of_travel(self):
        a = self.ahead(1500, heading=270.0)                           # westwards
        feats = [(f["kind"], round(f["dist_m"])) for f in a["features"]]
        self.assertEqual(feats, [("crossing", 300), ("traffic_signals", 500)])


class TestInput(PackCase):
    def test_validation(self):
        eng = Engine(self.pack)
        bad = [
            ({"type": "gnss", "lon": 21.0}, "missing field lat"),
            ({"type": "gnss", "lat": 91.0, "lon": 21.0}, "lat out of range"),
            ({"type": "gnss", "lat": 52.0, "lon": 21.0, "fix": "great"}, "fix must be"),
            ({"type": "gnss", "lat": "52", "lon": 21.0}, "lat must be a finite number"),
            ({"type": "motion"}, "missing field speed_mps"),
            ({"type": "motion", "speed_mps": -1.0}, "speed_mps out of range"),
            ({"type": "motion", "speed_mps": 3.0, "gear": "X"}, "gear must be"),
            ({"type": "imu"}, "unknown type"),
            ({"type": "gnss", "lat": 52.0, "lon": 21.0, "ts_mono_ns": T0 - 60 * NS}, "old"),
            ({"type": "gnss", "lat": 52.0, "lon": 21.0, "ts_mono_ns": T0 + 60 * NS}, "future"),
        ]
        for msg, why in bad:
            err = eng.on_input(msg, T0)
            self.assertIsNotNone(err, msg)
            self.assertIn(why, err)
        self.assertEqual(eng.counts["errors"], len(bad))
        self.assertEqual(eng.step(T0)["state"], "no_source")
        self.assertIsNone(eng.on_input({"type": "hello", "api": 1}, T0))
        self.assertIsNone(eng.on_input({"type": "gnss", "lat": 52.0, "lon": 21.0}, T0))     # ts defaults to now

    def test_poor_fix_is_ignored(self):
        eng = Engine(self.pack)
        la, lo = SP.ll(100, 0)
        eng.on_input({"type": "gnss", "ts_mono_ns": T0, "lat": la, "lon": lo, "hacc_m": 500.0}, T0)
        out = eng.step(T0)
        self.assertEqual(out["state"], "no_source")
        self.assertEqual(out["input"]["gnss"]["ignored"], 1)

    def test_heading_without_course(self):
        eng = Engine(self.pack)
        out = None
        for i in range(12):
            t = T0 + i * NS
            la, lo = SP.ll(100 + 12 * i, 0)
            eng.on_input({"type": "gnss", "ts_mono_ns": t, "lat": la, "lon": lo, "hacc_m": 3.0, "speed_mps": 12.0}, t)
            out = eng.step(t)
        self.assertAlmostEqual(out["position"]["heading_deg"], 90.0, delta=10.0)
        self.assertEqual(out["road"]["name"], "Main")


if __name__ == "__main__":
    unittest.main()
