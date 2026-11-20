"""The loop estimator <-> map box against truth (lane L47; docs/DESIGN.md section 15a): simulated drives on the
hand-made network through the whole stack. Is the right road matched, is the position honest, what does a parallel
road do to the stated uncertainty, what does a turn tell about the position along the road, what is left with a
receiver on few satellites and a host that stalls. Needs chris-mapper next to this tree."""
import importlib.util
import math
import os
import sys
import tempfile
import unittest

HERE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, HERE)
MAPPER = os.path.join(os.path.dirname(HERE), "chris-mapper")
sys.path.insert(0, MAPPER)

try:
    spec = importlib.util.spec_from_file_location("synthpack_mapper", os.path.join(MAPPER, "tests", "synthpack.py"))
    S = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(S)
    from chris_mapper import geo, sim
    from chris_mapper.box import MapBox
    from chris_mapper.rsmap import Pack
except (ImportError, OSError, AttributeError):
    S = None

from chris_integrator.io.feed import Feed  # noqa: E402
from chris_integrator.mixer import MessageClock  # noqa: E402
from chris_integrator.plugins.mapper import Mapper  # noqa: E402
from chris_integrator.stack import build  # noqa: E402

# a receiver like the reference car's on a good day: 10 m slowly varying, smooth from fix to fix, claimed 5 m
CAR = dict(gnss_sigma_m=10.0, gnss_tau_s=15.0, gnss_smooth_s=5.0, gnss_white_m=0.7, hacc_m=5.0)


def drive(pack_path, a, b, seed=1, lag=2.0, **kw):
    """A simulated drive from a to b (metres on the synthetic network) through the stack with the map box in the
    loop; the receiver's fixes are `lag` s old when they arrive. Returns rows of (truth, ego_state, map)."""
    pack = Pack(pack_path)
    st = build(mapper=Mapper(MapBox({"pack": pack_path})), clock=MessageClock())
    feed = Feed()
    rows, last, late = [], {}, []
    st.mixer.add_output("t", lambda e: last.__setitem__(e.stream, e.msg), streams=("ego_state", "map"))
    for ts, m, truth in sim.feed(sim.Drive(pack, sim.route(pack, S.ll(*a), S.ll(*b))), t0_ns=10 ** 12, seed=seed, source="receiver", **kw):
        if m is None:
            continue
        if m["type"] == "gnss":
            late.append(dict(m, ts_mono_ns=ts + int(lag * 1e9)))
            continue
        while late and late[0]["ts_mono_ns"] <= ts:
            for im in feed.convert(late.pop(0), now_ns=ts):
                st.deliver(im)
        for im in feed.convert(m, now_ns=ts):
            st.deliver(im)
        if m["type"] == "motion" and "ego_state" in last and truth.t > 30.0 and last.get("map") is not None:
            rows.append((truth, last["ego_state"], last["map"]))
    return rows


def pct(v, p):
    v = sorted(v)
    return v[min(len(v) - 1, int(p / 100.0 * len(v)))]


@unittest.skipUnless(S is not None, "needs chris-mapper next to this tree")
class Loop(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.tmp = tempfile.mkdtemp(prefix="chris-maploop-")
        cls.path = os.path.join(cls.tmp, "synth.rsmap")
        S.build(cls.path)
        cls.local = geo.Local(*S.ORIGIN)

    def xy(self, lat, lon):
        return self.local.to_xy(lat, lon)

    def errors(self, rows):
        return [(geo.dist_m(t.lat, t.lon, es["position"]["lat"], es["position"]["lon"]), es["position"]["hacc_m"])
                for t, es, m in rows if es["position"]["valid"]]

    def test_beside_a_parallel_road(self):
        """A road with a twin 8 m beside it for 1.2 km, a receiver that is 10 m off for 15 s at a time. The match
        stays on the road that is driven, the twin is listed as a candidate with a share all along, and the
        position across the road is stated less certain there than on the stretch without a twin: the loop does
        not turn "probably this one" into "this one, to 2 m"."""
        for seed in (1, 2):
            rows = drive(self.path, (20, 6000), (2300, 6000), seed=seed, **CAR)
            avail = [m for _, _, m in rows if m["available"]]
            self.assertGreater(len(avail) / len(rows), 0.99)
            self.assertGreater(sum(1 for m in avail if m["road"]["name"] == "Twin") / len(avail), 0.99)
            twin = [(t, es, m) for t, es, m in rows if 500.0 < self.xy(t.lat, t.lon)[0] < 1400.0]
            single = [(t, es, m) for t, es, m in rows if self.xy(t.lat, t.lon)[0] > 1700.0]
            self.assertGreater(sum(1 for _, _, m in twin if m["match"]["alternatives"]) / len(twin), 0.95)
            # the road runs east: the north variance is the position across it
            n_twin = pct([math.sqrt(es["position"]["cov_en"][2]) for _, es, _ in twin], 50)
            n_single = pct([math.sqrt(es["position"]["cov_en"][2]) for _, es, _ in single], 50)
            self.assertGreater(n_twin, 1.2 * n_single, (seed, n_twin, n_single))
            self.assertGreater(n_twin, 2.9)
            cross = [abs(self.xy(t.lat, t.lon)[1] - self.xy(es["position"]["lat"], es["position"]["lon"])[1]) for t, es, _ in twin]
            self.assertLess(pct(cross, 95), 3.0)
            e = self.errors(rows)
            self.assertGreater(sum(1 for d, h in e if d <= 2.5 * h) / len(e), 0.97)

    def test_a_turn_tells_where_the_car_is_along_the_road(self):
        """Along a straight road the map says nothing about the position (the receiver and its delay do); after a
        90 degree turn the new road's line constrains what was along-track before."""
        rows = drive(self.path, (20, 7000), (800, 7600), seed=1, **CAR)
        self.assertGreater(sum(1 for _, _, m in rows if m["available"] and m["road"]["name"] == "Corner") / len(rows), 0.97)
        before = [(t, es) for t, es, _ in rows if 450.0 < self.xy(t.lat, t.lon)[0] < 760.0 and self.xy(t.lat, t.lon)[1] < 7005.0]
        after = [(t, es) for t, es, _ in rows if self.xy(t.lat, t.lon)[1] > 7100.0]
        sig_b = pct([math.sqrt(es["position"]["cov_en"][0]) for _, es in before], 50)
        sig_a = pct([math.sqrt(es["position"]["cov_en"][0]) for _, es in after], 50)
        err_a = pct([abs(self.xy(t.lat, t.lon)[0] - self.xy(es["position"]["lat"], es["position"]["lon"])[0]) for t, es in after], 50)
        self.assertLess(sig_a, 0.8 * sig_b, (sig_a, sig_b))
        self.assertLess(sig_a, 3.5)
        self.assertLess(err_a, 2.5)

    def test_few_satellites_and_a_host_that_stalls(self):
        """The receiver trails by 6 s of smoothing on top of its delay, jumps, repeats itself and drops out; every
        45 s nothing at all arrives for 6.5 s. The road stays the right one, and where the position is not known
        it is mostly said so (wide, or not offered)."""
        rx = sim.PoorReceiver(seed=1, gap_p=0.03, jump_p=0.01, repeat_p=0.03, smooth_s=6.0, hacc_m=7.0, sats=4)
        rows = drive(self.path, (20, 0), (5150, 600), seed=1, rx=rx, blackouts=[(60.0 + 45.0 * k, 66.5 + 45.0 * k) for k in range(8)],
                     **dict(CAR, gnss_sigma_m=15.0))
        avail = [m for _, _, m in rows if m["available"]]
        self.assertGreater(len(avail) / len(rows), 0.6)
        self.assertGreater(sum(1 for m in avail if m["road"]["name"] == "Main") / len(avail), 0.98)
        e = self.errors(rows)
        self.assertGreater(len(e) / len(rows), 0.6)
        self.assertGreater(sum(1 for d, h in e if d <= 2.5 * h) / len(e), 0.75)
        # confidently wrong (beyond 15 m and 3 of its own sigmas): not gone. The receiver trails by up to 120 m
        # here and how far is not found at this steady speed (measured: 16 % of the valid steps)
        self.assertLess(sum(1 for d, h in e if d > 15.0 and d > 3.0 * h) / len(e), 0.25)
        es = rows[-1][1]
        self.assertGreaterEqual(es["reanchors"], 4)                           # the gaps were met as gaps
        self.assertTrue(es["sources"]["gnss"]["few_satellites"])


if __name__ == "__main__":
    unittest.main()
