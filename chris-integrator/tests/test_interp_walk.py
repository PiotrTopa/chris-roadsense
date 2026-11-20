"""view._interp_all is `_interp` for every query, whatever the order of table and queries."""
import random
import unittest

from chris_integrator.trackers.view import _interp, _interp_all


class TestInterpWalk(unittest.TestCase):
    def test_same_as_per_query(self):
        rnd = random.Random(5)
        nan = float("nan")
        for _ in range(20000):
            n, m = rnd.randint(0, 12), rnd.randint(0, 12)
            xs = sorted(rnd.uniform(0, 100) for _ in range(n)) if rnd.random() < 0.7 else \
                [rnd.choice([rnd.uniform(0, 100), nan, 5.0]) for _ in range(n)]
            ys = [rnd.choice([rnd.uniform(-5, 5)] * 6 + [None]) for _ in range(m)]
            qs = [rnd.uniform(-10, 120) for _ in range(rnd.randint(0, 15))]
            if rnd.random() < 0.7:
                qs.sort()
            if qs and rnd.random() < 0.1:
                qs[rnd.randrange(len(qs))] = nan
            self.assertEqual(repr([_interp(xs, ys, q) for q in qs]), repr(_interp_all(xs, ys, qs)))


if __name__ == "__main__":
    unittest.main()
