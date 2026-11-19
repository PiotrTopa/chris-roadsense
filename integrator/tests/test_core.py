"""The core's building blocks on their own: the time-ordered filter, the assignment, the monitors."""
import random
import unittest

import numpy as np

from . import simrun  # noqa: F401  (path)
from roadsense_integrator.core.ekf import Filter, Row
from roadsense_integrator.core.objects import assign


class _CV:
    """Constant velocity [p, v] with white acceleration: linear, so every property is exact."""

    def predict(self, x, P, dt):
        F = np.array([[1.0, dt], [0.0, 1.0]])
        q = 0.5
        Q = q * np.array([[dt ** 3 / 3, dt ** 2 / 2], [dt ** 2 / 2, dt]])
        return F @ x, F @ P @ F.T + Q, F


def _h(i):
    idx, one = np.array([i], dtype=np.intp), np.array([1.0])
    return lambda x: (float(x[i]), idx, one)


class FilterTest(unittest.TestCase):
    def rows(self, n=200, seed=1):
        rng = random.Random(seed)
        out, t, p, v = [], 0.0, 0.0, 1.0
        for k in range(n):
            dt = rng.uniform(0.02, 0.15)
            t += dt
            v += rng.gauss(0, 0.3) * dt ** 0.5
            p += v * dt
            out.append((t, 0 if k % 3 else 1, (p if k % 3 else v) + rng.gauss(0, 0.2)))
        return out

    def run_order(self, rows, order):
        f = Filter(_CV(), [0.0, 0.0], np.diag([100.0, 100.0]), horizon_s=1e9)
        f.insert(rows[0][0], Row("q%d" % rows[0][1], rows[0][2], 0.04, _h(rows[0][1]), "s", "g"))
        for k in order:
            t, i, z = rows[k]
            f.insert(t, Row("q%d" % i, z, 0.04, _h(i), "s", "g"))
            if k % 7 == 0:
                f.flush()
        return f

    def test_out_of_sequence_rows_give_the_in_order_result(self):
        rows = self.rows()
        a = self.run_order(rows, range(1, len(rows)))
        order = list(range(1, len(rows)))
        random.Random(3).shuffle(order)
        b = self.run_order(rows, order)
        (ta, xa, Pa), (tb, xb, Pb) = a.latest(), b.latest()
        self.assertEqual(ta, tb)
        np.testing.assert_allclose(xa, xb, atol=1e-9)
        np.testing.assert_allclose(Pa, Pb, atol=1e-9)
        self.assertGreater(b.reruns, 0)
        np.testing.assert_allclose(a.at(rows[100][0] + 0.01)[0], b.at(rows[100][0] + 0.01)[0], atol=1e-9)

    def test_smoothed_state_beats_filtered_and_matches_a_batch_solution(self):
        """`at(T)` with later rows in the buffer is the RTS-smoothed state: equal to the filter run backwards and
        combined, here checked against the forward filter's lower accuracy claim."""
        rows = self.rows(300, seed=2)
        f = Filter(_CV(), [0.0, 0.0], np.diag([100.0, 100.0]), horizon_s=1e9, lookahead_s=1e9)
        for t, i, z in rows:
            f.insert(t, Row("q%d" % i, z, 0.04, _h(i), "s", "g"))
        T = rows[150][0]
        xs, Ps = f.at(T)
        xf, Pf = f.filtered(T)
        self.assertLess(Ps[0, 0], Pf[0, 0])
        self.assertLess(Ps[1, 1], 0.8 * Pf[1, 1])
        # a late row far in the past changes the smoothed state at T
        f.insert(rows[149][0] + 1e-3, Row("q0", xs[0] + 5.0, 0.01, _h(0), "late", "g"))
        self.assertGreater(f.at(T)[0][0], xs[0] + 1.0)

    def test_robust_rows_and_the_anchor_rule(self):
        f = Filter(_CV(), [0.0, 0.0], np.diag([1.0, 1.0]), horizon_s=1e9)
        t = 0.0
        for k in range(50):
            t += 0.1
            f.insert(t, Row("p", 0.0, 0.01, _h(0), "exact", "a"))                    # an exact source holds p at 0
            f.insert(t, Row("p", 50.0, 0.01, _h(0), "wild", "b", robust=True))       # a wild one says 50
        x = f.latest()[1]
        self.assertLess(abs(x[0]), 0.05)
        self.assertEqual(f.rejected["wild"], 50)
        # alone, the same source is followed (never locked out by its own filter)
        g = Filter(_CV(), [0.0, 0.0], np.diag([1.0, 1.0]), horizon_s=1e9)
        t = 0.0
        for k in range(100):
            t += 0.1
            g.insert(t, Row("p", 50.0, 0.01, _h(0), "wild", "b", robust=True))
        self.assertGreater(g.latest()[1][0], 45.0)
        self.assertEqual(g.rejected, {})
        # a disabled group leaves the estimate at once
        f.set_disabled(["a"])
        self.assertGreater(f.latest()[1][0], 20.0)

    def test_consider_states_are_not_corrected(self):
        f = Filter(_CV(), [0.0, 0.0], np.diag([4.0, 4.0]), horizon_s=1e9)
        idx, co = np.array([0, 1], dtype=np.intp), np.array([1.0, 1.0])
        f.insert(0.0, Row("s", 3.0, 0.01, lambda x: (float(x[0] + x[1]), idx, co), "s", "g", freeze=np.array([1])))
        _, x, P = f.latest()
        self.assertEqual(x[1], 0.0)                         # the frozen state did not move ...
        self.assertAlmostEqual(P[1, 1], 4.0)                # ... and kept its uncertainty
        self.assertGreater(x[0], 1.0)
        self.assertGreater(np.linalg.eigvalsh(P).min(), 0.0)


class AssignTest(unittest.TestCase):
    def test_optimal_assignment(self):
        c = np.array([[1.0, 2.0, np.inf], [1.1, 1.0, 5.0], [np.inf, np.inf, np.inf]])
        self.assertEqual(sorted(assign(c)), [(0, 0), (1, 1)])
        c = np.array([[1.0, 1.1], [1.0, 9.0]])              # greedy would take (0,0) and pay 9
        self.assertEqual(sorted(assign(c)), [(0, 1), (1, 0)])
        self.assertEqual(assign(np.zeros((0, 3))), [])
        rng = np.random.default_rng(1)
        for _ in range(20):
            c = rng.uniform(0, 10, (5, 7))
            got = assign(c)
            self.assertEqual(len(got), 5)
            best = min(sum(c[i, p[i]] for i in range(5)) for p in __import__("itertools").permutations(range(7), 5))
            self.assertAlmostEqual(sum(c[i, j] for i, j in got), best, places=9)


if __name__ == "__main__":
    unittest.main()
