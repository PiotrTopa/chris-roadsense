"""fastround.round_flat gives exactly round(x, n): same value, same type, same JSON text."""
import json
import math
import random
import unittest

from chris_integrator.fastround import Layout, round_flat, scales


def same(a, b):
    return type(a) is type(b) and (a == b or (a != a and b != b)) and \
        (not isinstance(a, float) or math.copysign(1.0, a) == math.copysign(1.0, b))


class TestFastRound(unittest.TestCase):
    def check(self, values, digits):
        got = round_flat(values, scales(digits), digits)
        want = [v if v is None else round(v, d) for v, d in zip(values, digits)]
        for v, d, g, w in zip(values, digits, got, want):
            self.assertTrue(same(g, w), "round(%r, %d): %r, got %r" % (v, d, w, g))
        self.assertEqual(json.dumps(got), json.dumps(want))

    def test_random(self):
        rnd = random.Random(7)
        for _ in range(300):
            n = rnd.randint(1, 400)
            mag = 10.0 ** rnd.randint(-8, 8)
            self.check([rnd.uniform(-mag, mag) for _ in range(n)], [rnd.randint(0, 7) for _ in range(n)])

    def test_ties_and_neighbours(self):
        rnd = random.Random(11)
        vals, digs = [], []
        for _ in range(60000):
            d = rnd.randint(1, 5)
            k = rnd.randint(-10 ** 6, 10 ** 6)
            x = (k + 0.5) / 10.0 ** d                     # a decimal tie, as a double it sits just beside it
            for v in (x, math.nextafter(x, math.inf), math.nextafter(x, -math.inf), x * (1 + 1e-9), x * (1 - 1e-9)):
                vals.append(v)
                digs.append(d)
        self.check(vals, digs)

    def test_integers_zeros_and_types(self):
        vals = [0, -0, 0.0, -0.0, 1, 2.0, -3.0, 5, 1e-12, -1e-12, 0.004, -0.004, 0.005, -0.005, 123456789.125,
                1e15 + 0.3, -7e17, 1e300, 5e-324, True, False, 0.1 + 0.2, 2.675, 1.005, 0.285, 0.145]
        for d in (0, 1, 2, 3, 4, 7):
            self.check(vals, [d] * len(vals))

    def test_not_finite_and_not_numbers(self):
        self.check([1.234, float("nan"), float("inf"), -float("inf"), 2.5], [2] * 5)
        self.assertEqual(round_flat([1.234, None, 2.345], scales([2, 2, 2]), [2, 2, 2]), [1.23, None, round(2.345, 2)])
        self.assertEqual(round_flat([], scales([]), []), [])

    def test_layout(self):
        lay = Layout()
        rnd = random.Random(3)
        for _ in range(50):
            parts = [[rnd.uniform(-200, 200) for _ in range(33)] for _ in range(9)] + [[0, 1.5, -0, 2.25]]
            digs = [2, 3, 3] * 3 + [2]
            got = lay.round_parts(parts, digs)
            want = [[round(v, d) for v in p] for p, d in zip(parts, digs)]
            self.assertEqual(json.dumps(got), json.dumps(want))


class TestRoundedText(unittest.TestCase):
    """Layout.text_parts writes what json.dumps writes for the rounded lists."""

    def check(self, parts, digs):
        got = Layout().text_parts(parts, digs)
        want = [json.dumps([v if v is None else round(v, d) for v in p], separators=(",", ":")) for p, d in zip(parts, digs)]
        self.assertEqual(got, want)

    def test_random(self):
        rnd = random.Random(21)
        for _ in range(300):
            mag = 10.0 ** rnd.randint(-6, 4)
            parts = [[rnd.uniform(-mag, mag) for _ in range(rnd.randint(0, 40))] for _ in range(rnd.randint(1, 12))]
            self.check(parts, [rnd.choice((2, 3)) for _ in parts])

    def test_zeros_ints_ties_range(self):
        vals = [0, -0, 0.0, -0.0, -0.001, 0.001, -0.004, -0.0049999, -0.005, 0.005, 1, 3, 12, 192, 3.0, 0.5, 0.125, 2.675,
                1.005, 327.66, 327.67, 327.68, -327.68, -327.69, 32.767, 32.768, -32.768, -32.769, 1e6, -1e6, 1e-300,
                float("nan"), float("inf"), -float("inf"), None, True, 0.07, 0.285, 62.5e-3, 1.2345, 0.1 + 0.2]
        self.check([vals, vals, list(reversed(vals))], [2, 3, 2])

    def test_ties_dense(self):
        rnd = random.Random(23)
        parts, digs = [], []
        for d in (2, 3):
            p = []
            for _ in range(40000):
                x = (rnd.randint(-30000, 30000) + 0.5) / 10.0 ** d
                p += [x, math.nextafter(x, math.inf), math.nextafter(x, -math.inf)]
            parts.append(p)
            digs.append(d)
        self.check(parts, digs)

    def test_every_table_entry(self):
        for d in (2, 3):
            vals = [k / 10.0 ** d for k in range(-32768, 32768)]
            self.check([vals], [d])


if __name__ == "__main__":
    unittest.main()
