"""Savings that must not change a number (lane L50): each replaces a walk or a sort by bookkeeping, and is
held here against the plain form it replaced."""
import bisect
import collections
import math
import os
import random
import statistics
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from chris_integrator.core import ekf as K  # noqa: E402
from chris_integrator.core.monitors import NisMonitor  # noqa: E402
from chris_integrator.core.registry import all_fin, fin  # noqa: E402
from chris_integrator.trackers.speedlimit import SpeedLimit  # noqa: E402


def _near_walk(f, T, window_s, kinds=None, reach_s=1.0):
    """Filter.near as it was: every applied row of every entry in reach."""
    reach = max(window_s, reach_s)
    lo, hi = bisect.bisect_left(f.times, T - reach), bisect.bisect_right(f.times, T + reach)
    age, srcs = None, set()
    for e in f.entries[lo:hi]:
        d = abs(e.t - T)
        for r in e.rows:
            if not r.used:
                continue
            if kinds is None or r.q in kinds:
                if d <= window_s:
                    srcs.add(r.src)
                if age is None or d < age:
                    age = d
    for tt, q, src in f.carry:
        d = abs(tt - T)
        if d <= reach and (kinds is None or q in kinds) and q != "pos":
            if d <= window_s:
                srcs.add(src)
            if age is None or d < age:
                age = d
    return age, sorted(srcs)


def test_near_is_the_walk_over_every_row():
    rng = random.Random(5)
    checked = 0
    for _ in range(1500):
        f = K.Filter.__new__(K.Filter)
        f.entries, f.times, f.carry = [], [], []
        t = 100.0
        dense = rng.choice([0.05, 0.9])
        for _ in range(rng.randrange(0, 300)):
            t += rng.choice([0.0101, 0.02, 0.05, 0.3, 1.0, 0.6, 2.5]) if rng.random() < 0.3 else rng.uniform(0.005, 0.03)
            e = K.Entry(t)
            for _ in range(rng.choice([0, 1, 2, 3])):
                r = K.Row(rng.choice(["speed", "yaw_rate", "pos", "heading", "accel"]), 0.0, 1.0, None, rng.choice("abc"), "g")
                r.used = rng.random() < dense
                e.rows.append(r)
            e.usedq = tuple((r.q, r.src) for r in e.rows if r.used)      # what `_run` leaves
            f.entries.append(e)
            f.times.append(t)
        if rng.random() < 0.3:
            f.carry = [(100.0 + rng.uniform(0, 5), rng.choice(["speed", "pos"]), "z") for _ in range(3)]
        for _ in range(10):
            T = rng.choice(f.times) if f.times and rng.random() < 0.5 else 100.0 + rng.uniform(-2, t - 98.0)
            if f.times and rng.random() < 0.3:
                T = rng.choice(f.times) + rng.choice([0.3, -0.3, 1.0, -1.0, 1.6, -1.6])       # on the window's edge
            w = rng.choice([0.3, 0.3, 1.6, 3.9])
            kinds = rng.choice([None, ("speed",), ("pos",), ("pos", "heading"), ("yaw_rate", "accel")])
            reach = rng.choice([1.01, 1e9, 0.1])
            assert f.near(T, w, kinds, reach) == _near_walk(f, T, w, kinds, reach)
            checked += 1
    assert checked == 15000


def test_innovation_monitor_median_is_the_sorted_median():
    cfg = dict(window_s=8.0, max_n=80, min_n=8, fire=6.0, recover=3.0)
    rng = random.Random(1)
    evaluated = 0
    for _ in range(200):
        m = NisMonitor("g", "q", cfg)
        ref = collections.deque(maxlen=80)
        t = 0.0
        for _ in range(1500):
            t += rng.choice([0.01, 0.05, 0.2, 1.0, 3.0, 9.0]) if rng.random() < 0.9 else 0.0
            d = rng.choice([rng.random() * 5, 0.0, float("nan") if rng.random() < 0.02 else 1.0, rng.random()])
            m.offer(t, d)
            ref.append((t, d))
            while ref and ref[0][0] < t - 8.0:
                ref.popleft()
            assert len(m.q) == len(ref)
            if len(ref) >= 8:
                a, b = m._median(), statistics.median(d for _, d in ref)
                assert a == b or (a != a and b != b)
                evaluated += 1
            if rng.random() < 0.001:
                m.reset_data()
                ref.clear()
    assert evaluated > 50000


def test_turn_window_extremes_are_max_and_min_of_the_window():
    rng = random.Random(2)
    for trial in range(40):
        s = SpeedLimit()
        ts = 10 ** 12
        v = 10.0
        for i in range(1500):
            ts += int(rng.uniform(0.03, 0.2) * 1e9)
            if rng.random() < 0.02:
                v = rng.choice([0.0, 0.0, 3.0, 15.0, 30.0])              # a standing car: the window grows
            es = {"speed": {"value": v, "valid": rng.random() < 0.97, "sources": ["vehicle.speed"]},
                  "yaw_rate": {"value": rng.gauss(0.0, 0.2), "valid": rng.random() < 0.95}}
            s.on_message({"type": "road", "ts_mono_ns": ts, "seq": i}, es)
            span = s.heading_span()
            if len(s.heading) < 2:
                assert span is None
            else:
                yaws = [y for _, y in s.heading]
                assert span == (max(yaws), min(yaws)), (trial, i)
    s = SpeedLimit()
    s.heading.extend([(0.0, 1.0), (1.0, -2.0), (2.0, 5.0)])              # a window not filled through on_message
    assert s.heading_span() == (5.0, -2.0)


def test_all_fin_is_fin_of_every_element():
    rng = random.Random(3)
    pool = [1.5, 0, -3, 2 ** 70, True, None, "1", float("nan"), float("inf"), -0.0, 1e308, [], 3.25]
    for _ in range(20000):
        seq = [rng.choice(pool) if rng.random() < 0.15 else rng.uniform(-9, 9) for _ in range(rng.randrange(0, 12))]
        assert all_fin(seq) == all(fin(v) for v in seq), seq
    assert all_fin([]) and all_fin((1.0, 2)) and not all_fin([math.inf])
