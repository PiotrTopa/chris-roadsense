"""Synthetic drives over the real Poland pack. Skipped when no pack is found.

The pack is looked up in ROADSENSE_MAP_PACK, else the newest ~/roadsense-maps/packs/poland-*.rsmap
(scripts/roadsense-maps build). Nothing of the pack is stored in the repository: the scenarios are
pairs of coordinates, the roads come from the pack.

    python3 -m unittest discover -s mapd/tests -t mapd -p 'test_realpack.py' -v
"""
import glob
import os
import unittest

from roadsense_mapd import geo, rsmap as R, sim
from roadsense_mapd.engine import Engine

T0 = 10 ** 12
NS = 10 ** 9

# (from, to): start and destination of a route (lat, lon)
SCENARIOS = {
    "tunnel": ((52.13889, 20.98064), (52.15258, 21.11298)),   # S2 Warsaw southern bypass, eastbound, 2.3 km tunnel
    "city": ((52.2297, 21.0122), (52.1930, 21.0340)),         # Warsaw centre southwards, 5.4 km of arterials
    "rural": ((50.6420, 17.6190), (50.4760, 17.3340)),        # Niemodlin - Nysa, 30 km of national and regional roads
}


def find_pack():
    p = os.environ.get("ROADSENSE_MAP_PACK")
    if p:
        return p if os.path.exists(p) else None
    packs = sorted(glob.glob(os.path.expanduser("~/roadsense-maps/packs/poland-*.rsmap")))
    return packs[-1] if packs else None


@unittest.skipUnless(find_pack(), "no Poland pack (scripts/roadsense-maps build)")
class RealPack(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.pack = R.Pack(find_pack(), cache_tiles=800)
        cls.paths = {}

    @classmethod
    def tearDownClass(cls):
        cls.pack.close()

    def path(self, name):
        if name not in self.paths:
            a, b = SCENARIOS[name]
            self.paths[name] = sim.route(self.pack, a, b)
        return self.paths[name]

    def drive(self, name, hz=2.0, cfg=None, **feed):
        path = self.path(name)
        on_path = {e.ref for e, _, _, _ in path}
        eng = Engine(R.Pack(self.pack.path), cfg)        # its own small tile cache, like the daemon
        rows = []
        last = None
        for ts, msg, o in sim.feed(sim.Drive(self.pack, path), t0_ns=T0, tick_hz=hz, **feed):
            if msg is not None:
                self.assertIsNone(eng.on_input(msg, ts))
            elif last is None or ts - last >= 0.99 * NS / hz:
                last = ts
                out = eng.step(ts)
                rows.append((o, out, out["available"] and eng.match.edge.ref in on_path))
        eng.pack.close()
        return rows

    def test_pack_manifest(self):
        m = self.pack.manifest
        self.assertEqual(m["region"], "poland")
        self.assertIn("OpenStreetMap contributors", m["attribution"])
        self.assertEqual(m["licence"], "ODbL-1.0")
        self.assertGreater(m["edges"], 3_000_000)
        self.assertTrue(m["osm_timestamp"])

    def test_tunnel_with_vehicle_motion(self):
        """Fix lost for the whole tunnel: dead reckoning on speed + yaw rate, held on the road by the map."""
        path = self.path("tunnel")
        tunnel_m = sum(abs(s1 - s0) for e, _, s0, s1 in path if e.flags & R.F_TUNNEL)
        self.assertGreater(tunnel_m, 1500.0)
        rows = self.drive("tunnel")
        tun = [(o, out, ok) for o, out, ok in rows if o.tunnel]
        inner = tun[8:-6]            # the odometer runs 1 % long: the filter leaves the 2.3 km tunnel ~1 s early
        self.assertTrue(all(out["state"] == "dead_reckoning" for o, out, ok in inner))
        self.assertTrue(all(out["available"] and ok and out["road"]["tunnel"] for o, out, ok in inner),
                        [round(o.t) for o, out, ok in inner if not (out["available"] and ok)][:10])
        err = [geo.dist_m(out["position"]["lat"], out["position"]["lon"], o.lat, o.lon) for o, out, ok in tun]
        self.assertLess(max(err), 0.02 * tunnel_m + 15.0)             # 1 % odometer scale error + the error at entry
        exits = [out["ahead"]["tunnel"]["exit_m"] for o, out, ok in inner if out["ahead"]["tunnel"]["exit_m"] is not None]
        self.assertGreater(len(exits), 0.8 * len(inner))
        self.assertTrue(all(a >= b for a, b in zip(exits, exits[1:])))
        before = [out for o, out, ok in rows if not o.tunnel and out["available"] and out["ahead"]["tunnel"]["entry_m"]]
        self.assertTrue(before, "the tunnel must be announced before it is entered")
        self.assertAlmostEqual(before[0]["ahead"]["tunnel"]["length_m"], tunnel_m, delta=0.1 * tunnel_m)
        after = [out for o, out, ok in rows if o.t > tun[-1][0].t + 8.0]
        self.assertTrue(all(out["state"] == "good" for out in after))
        avail = [ok for o, out, ok in rows[4:]]
        self.assertGreater(sum(avail) / len(avail), 0.99)

    def test_tunnel_without_any_motion_input_falls_back_to_vision_only(self):
        """No car data and no vision odometry: a few seconds of coasting, then the position is unknown and
        the map layer is unavailable until the fix returns after the tunnel."""
        rows = self.drive("tunnel", motion="none")
        tun = [(o, out) for o, out, ok in rows if o.tunnel]
        states = [out["state"] for o, out in tun]
        self.assertIn("unknown", states)
        late = [out for o, out in tun if o.t > tun[0][0].t + 12.0]
        self.assertTrue(all(out["state"] == "unknown" and not out["available"] and out["reason"] == "position_unknown"
                            and out["road"] is None for out in late))
        after = [(out, ok) for o, out, ok in rows if o.t > tun[-1][0].t + 8.0]
        self.assertTrue(all(out["state"] == "good" and ok for out, ok in after))

    def test_city_drive(self):
        rows = self.drive("city", gnss_sigma_m=5.0, hacc_m=8.0)
        avail = [ok for o, out, ok in rows[6:]]
        self.assertGreater(sum(avail) / len(avail), 0.97)
        lim = [out["speed_limit"]["limit_kmh"] for o, out, ok in rows if out["available"]]
        self.assertGreater(sum(x is not None for x in lim) / len(lim), 0.95)
        kinds = {f["kind"] for o, out, ok in rows if out["available"] for f in out["ahead"]["features"]}
        self.assertTrue({"traffic_signals", "crossing"} <= kinds, kinds)
        self.assertTrue(any(j["kind"] == "junction" for o, out, ok in rows if out["available"] for j in out["ahead"]["junctions"]))

    def test_city_drive_with_a_gnss_dropout(self):
        rows = self.drive("city", dropouts=[(60.0, 150.0)])
        dr = [(o, out, ok) for o, out, ok in rows if 66.0 < o.t < 150.0]
        self.assertTrue(all(out["state"] == "dead_reckoning" for o, out, ok in dr))
        self.assertGreater(sum(ok for o, out, ok in dr) / len(dr), 0.9)       # a wrong branch at a square for ~5 s
        err = [geo.dist_m(out["position"]["lat"], out["position"]["lon"], o.lat, o.lon) for o, out, ok in dr]
        self.assertLess(max(err), 40.0)

    def test_city_drive_multipath_towards_the_other_carriageway(self):
        """Fixes pushed 15 m sideways for 12 s: the match stays on the driven route."""
        rows = self.drive("city", bias_m=lambda t: (11.0, 11.0) if 40.0 <= t < 52.0 else (0.0, 0.0))
        seg = [ok for o, out, ok in rows if 38.0 <= o.t < 60.0]
        self.assertGreater(sum(seg) / len(seg), 0.9)

    def test_rural_drive_limits_and_built_up(self):
        rows = self.drive("rural")
        av = [(o, out) for o, out, ok in rows if out["available"]]
        self.assertGreater(len(av) / len(rows), 0.98)
        sources = {out["speed_limit"]["source"] for o, out in av}
        self.assertEqual(sources, {"maxspeed", "implicit"})
        rules = {out["speed_limit"]["rule"] for o, out in av if out["speed_limit"]["source"] == "implicit"}
        self.assertTrue({"built_up", "outside"} <= rules, rules)
        self.assertTrue(any(out["ahead"]["built_up_change"] for o, out in av))
        self.assertTrue(any(out["ahead"]["curves"] for o, out in av))
        for o, out in av:
            if out["speed_limit"]["rule"] == "built_up":
                self.assertEqual(out["speed_limit"]["limit_kmh"], 50)
            if out["speed_limit"]["rule"] == "outside":
                self.assertEqual(out["speed_limit"]["limit_kmh"], 90)


if __name__ == "__main__":
    unittest.main()
