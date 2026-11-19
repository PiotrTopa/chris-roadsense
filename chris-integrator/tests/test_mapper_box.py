"""The map box in the stack: the car's feed -> the estimator's pose -> chris-mapper -> `map`, and the match back
into the estimator as a measurement. Position is estimated once. Needs chris-mapper next to this tree."""
import os
import sys
import tempfile
import unittest

HERE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, HERE)
MAPPER = os.path.join(os.path.dirname(HERE), "chris-mapper")
sys.path.insert(0, MAPPER)

try:
    from chris_mapper import geo, sim
    from chris_mapper.box import MapBox
    from chris_mapper.rsmap import Pack
    from tests_mapper import synthpack as S      # noqa: F401
except ImportError:
    try:
        import importlib.util
        spec = importlib.util.spec_from_file_location("synthpack_mapper", os.path.join(MAPPER, "tests", "synthpack.py"))
        S = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(S)
        from chris_mapper import geo, sim
        from chris_mapper.box import MapBox
        from chris_mapper.rsmap import Pack
    except (ImportError, OSError):
        S = None

from chris_integrator.io.feed import Feed  # noqa: E402
from chris_integrator.mixer import MessageClock  # noqa: E402
from chris_integrator.mixer.remote import RemotePlugin  # noqa: E402
from chris_integrator.plugins.mapper import Mapper  # noqa: E402
from chris_integrator.stack import build  # noqa: E402


def run(pack_path, mapper, x1=5600.0, seed=1, gnss_lag_s=2.0, **feed_kw):
    """A simulated drive along the synthetic network's main road through the stack (no camera: the outputs go
    out on the vehicle's samples). The receiver reports as the reference car's does: a fix is `gnss_lag_s` old
    when it arrives and is stamped at its arrival. Returns rows of (truth, ego_state, newest map message)."""
    pack = Pack(pack_path)
    st = build(mapper=mapper, clock=MessageClock())
    feed = Feed()
    rows, last = [], {}
    st.mixer.add_output("t", lambda e: last.__setitem__(e.stream, e.msg), streams=("ego_state", "map"))
    drive = sim.Drive(pack, S.main_road_path(pack, 20.0, x1))
    late = []
    for ts, m, truth in sim.feed(drive, t0_ns=10 ** 12, seed=seed, **feed_kw):
        if m is None:
            continue
        if m["type"] == "gnss":
            late.append(dict(m, ts_mono_ns=ts + int(gnss_lag_s * 1e9)))
            continue
        while late and late[0]["ts_mono_ns"] <= ts:
            for im in feed.convert(late.pop(0), now_ns=ts):
                st.deliver(im)
        for im in feed.convert(m, now_ns=ts):
            st.deliver(im)
        if m["type"] == "motion" and "ego_state" in last:
            rows.append((truth, last["ego_state"], last.get("map")))
    return st, rows


@unittest.skipUnless(S is not None, "needs chris-mapper next to this tree")
class MapperInTheStack(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.tmp = tempfile.mkdtemp(prefix="chris-mapbox-")
        cls.path = os.path.join(cls.tmp, "synth.rsmap")
        S.build(cls.path)

    def check(self, st, rows):
        maps = [(t, es, m) for t, es, m in rows if m is not None and t.t > 30.0]
        avail = [m for _, _, m in maps if m["available"]]
        self.assertGreater(len(avail) / len(maps), 0.97)
        self.assertGreater(sum(1 for m in avail if m["road"]["name"] == "Main") / len(avail), 0.99)
        # the pose the map shows is the estimator's own, and it stays near the truth
        err = []
        for t, es, m in maps:
            p = es["position"]
            if p["valid"]:
                err.append(geo.dist_m(t.lat, t.lon, p["lat"], p["lon"]))
        err.sort()
        self.assertLess(err[len(err) // 2], 6.0)
        self.assertLess(err[int(0.95 * len(err))], 15.0)
        # the tunnel (x 3000 .. 3800, no fixes): dead reckoning with the map match as a measurement
        tun = [(t, es, m) for t, es, m in maps if t.tunnel]
        self.assertGreater(len(tun), 100)
        self.assertGreater(sum(1 for _, _, m in tun if m["available"] and m["road"]["tunnel"]) / len(tun), 0.9)
        self.assertTrue(any(m["state"] == "dead_reckoning" for _, _, m in tun))
        # limits along the way come from the map
        self.assertEqual({m["speed_limit"]["limit_kmh"] for m in avail} >= {70, 50, 90, 80}, True)
        es = rows[-1][1]
        self.assertGreater(es["sources"]["map"]["used"], 50)                  # the match was used as a measurement
        self.assertEqual(st.mixer.plugin("mapper").health()["state"], "ok")
        self.assertEqual(sum(s.errors for s in st.mixer.slots.values()), 0)

    def test_in_process(self):
        st, rows = run(self.path, Mapper(MapBox({"pack": self.path})))
        self.check(st, rows)
        self.assertEqual([s.name for s in st.mixer.order][1:3], ["estimator", "mapper"])

    def test_out_of_process_gives_the_same_map(self):
        env = dict(os.environ, PYTHONPATH=MAPPER)
        remote = RemotePlugin(name="mapper", cmd=[sys.executable, "-m", "chris_mapper", "--plugin", "--pack", self.path],
                              env=env, timeout_s=5.0)
        st_r, rows_r = run(self.path, remote, x1=2600.0)
        st_l, rows_l = run(self.path, Mapper(MapBox({"pack": self.path})), x1=2600.0)
        st_r.mixer.close()
        self.assertEqual(len(rows_r), len(rows_l))
        self.assertEqual([m for _, _, m in rows_r], [m for _, _, m in rows_l])       # message for message
        self.assertEqual([es["position"] for _, es, _ in rows_r], [es["position"] for _, es, _ in rows_l])
        h = st_r.mixer.plugin("mapper").health()
        self.assertEqual((h["state"], h["transport"]["timeouts"]), ("ok", 0))

    def test_removed_the_rest_goes_on(self):
        st, rows = run(self.path, None, x1=1500.0)
        self.assertNotIn("map", st.mixer.bus.streams)
        self.assertTrue(rows[-1][1]["position"]["valid"])                     # the position does not need the map
        st2, rows2 = run(self.path, Mapper(MapBox({"pack": self.path})), x1=1500.0)
        st2.mixer.remove("mapper")                                            # at run time
        self.assertNotIn("map", st2.mixer.bus.streams)


if __name__ == "__main__":
    unittest.main()
