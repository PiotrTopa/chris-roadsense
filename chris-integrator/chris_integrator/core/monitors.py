"""Consistency monitors: raw measurements of one source against raw measurements of another (never against the
fused state), a source's stated uncertainty against its characterised one, and a slow witness test for sources
whose timing is poor. Each monitor keeps a window of *evidence time* and a verdict with hysteresis.
"""
import bisect
import collections
import math
import statistics

from ..native import round                  # builtins.round, native where built


class Series:
    """Raw measurements of one group for one quantity: (t, value, sigma), newest last."""

    def __init__(self, keep_s=12.0):
        self.t, self.v, self.s = [], [], []
        self.keep = keep_s

    def add(self, t, v, s):
        if self.t and t <= self.t[-1]:
            return
        self.t.append(t), self.v.append(v), self.s.append(s)
        if self.t[0] < t - 2.0 * self.keep:
            n = bisect.bisect_left(self.t, t - self.keep)
            del self.t[:n], self.v[:n], self.s[:n]

    def clear(self):
        del self.t[:], self.v[:], self.s[:]

    def at(self, t, max_gap_s):
        """(value, sigma) interpolated at t; None when t is not bracketed by samples at most max_gap_s apart;
        "later" when the series has not reached t yet."""
        if not self.t or t > self.t[-1]:
            return "later"
        i = bisect.bisect_left(self.t, t)
        if self.t[i] == t:
            return self.v[i], self.s[i]
        if i == 0 or self.t[i] - self.t[i - 1] > max_gap_s:
            return None
        w = (t - self.t[i - 1]) / (self.t[i] - self.t[i - 1])
        return self.v[i - 1] + w * (self.v[i] - self.v[i - 1]), max(self.s[i - 1], self.s[i])


class _Window:
    """Samples over evidence time: the clock advances by the time between evidence samples, capped."""

    def __init__(self, keep_s, max_step_s):
        self.q = collections.deque()
        self.keep, self.max_step = keep_s, max_step_s
        self.ev = 0.0
        self.last_t = None

    def add(self, t, item):
        if self.last_t is not None:
            self.ev += min(max(t - self.last_t, 0.0), self.max_step)
        self.last_t = t
        self.q.append((self.ev, item))
        while self.q and self.q[0][0] < self.ev - self.keep:
            self.q.popleft()

    def last(self, span_s):
        lim = self.ev - span_s
        return [it for ev, it in self.q if ev >= lim]

    def span(self):
        return self.ev - self.q[0][0] if self.q else 0.0

    def clear(self):
        self.q.clear()
        self.last_t = None


class Pair:
    """`tested` against `ref` (the higher-ranked group) for one quantity."""

    kind = "pair"

    def __init__(self, ref, tested, quantity, cfg):
        self.ref, self.tested, self.quantity = ref, tested, quantity
        self.c = cfg
        self.win = _Window(cfg["recover_window_s"], cfg["max_step_s"])
        self.pending = collections.deque(maxlen=64)
        self.verdict = "unknown"          # unknown | agree | disagree
        self.suspect = False
        self.stats = {"n": 0, "share": None, "ratio": None}
        self.changed_t = None
        self.last_delta = None
        self.times = collections.deque(maxlen=64)
        self.gp = self.gn = self.var = 0.0     # mean mode: CUSUM sums, learned variance of the difference
        self.n_ok = self.n_mean = 0
        self.recent = collections.deque(maxlen=8 if cfg["mode"] == "mean" else 24)
        self.cusum_on = self.by_cusum = False
        self.sample_t = None                   # stamp of the newest paired sample
        # the largest value each test statistic reached, as a fraction of its firing level (1 = fired): how near
        # a healthy pair came to an alarm, and what a limit's margin is on a drive
        self.peak = {}

    def _peak(self, name, value):
        if value > self.peak.get(name, 0.0):
            self.peak[name] = value

    def offer(self, t, v, s, ref_series, now):
        """A measurement of the tested group; paired at once when the reference brackets its time, else later."""
        self.pending.append((t, v, s))
        return self.flush(ref_series, now)

    def flush(self, ref_series, now):
        changed = False
        while self.pending:
            t, v, s = self.pending[0]
            r = ref_series.at(t - self.c["lag_s"], self.c["max_gap_s"])
            if r == "later":
                if now - t > self.c["pending_s"]:
                    self.pending.popleft()
                    continue
                break
            self.pending.popleft()
            if r is not None:
                changed |= self._sample(t, v, s, r[0], r[1])
        return changed

    def _sample(self, t, v, s, rv, rs):
        c = self.c
        if c["mode"] == "share" and c.get("cusum_h"):
            # beside the share-of-bad-samples test: a CUSUM of the normalised difference, for what never puts a
            # single sample beyond z_bad (a bias, a 10 % scale error, a slow drift). It sees every sample, also
            # those too small to be evidence for the share test (a gyro bias shows on a straight road).
            z = max(-6.0, min(6.0, (v - rv) / math.hypot(s, rs)))
            self.gp = min(max(0.0, self.gp + z - c["cusum_k"]), 2.0 * c["cusum_h"])
            self.gn = min(max(0.0, self.gn - z - c["cusum_k"]), 2.0 * c["cusum_h"])
            self.recent.append((v - rv, abs(rv)))
            g = max(self.gp, self.gn)
            if g >= c["cusum_h"]:
                md = abs(statistics.fmean(d for d, _ in self.recent))
                lim = max(c["abs_delta"], c["min_delta"] * statistics.median(r for _, r in self.recent))
                self.cusum_on = md > lim
                self.cusum_delta = md
                self._peak("cusum", md / lim)          # (the sum is over its level: what decides is the size)
            elif g < 0.5 * c["cusum_h"]:
                self.cusum_on = False
            self.sample_t = t
        if max(abs(v), abs(rv)) < c["min_evidence"]:
            if c["mode"] == "share" and c.get("cusum_h") and self.cusum_on != (self.verdict == "disagree") and self.verdict != "unknown":
                return self._evaluate(t)
            return False
        z = (v - rv) / math.hypot(s, rs)
        self.win.add(t, (abs(z) > c["z_bad"], v, rv))
        self.last_delta = v - rv
        self.times.append(t)
        self.sample_t = t
        if c["mode"] == "mean":
            return self._evaluate_mean(t, math.hypot(s, rs))
        return self._evaluate(t)

    def _evaluate_mean(self, t, sigma):
        """A noisy, slow source (a receiver's track against the odometer): no single sample says anything, their
        accumulation does. A two-sided CUSUM of the normalised difference about zero (both measure the same
        quantity), normalised by the spread the pair itself showed while it agreed (learned; the stated sigma
        until then): a large fault is decided in two or three samples, a small one when its evidence has added
        up, and a persistent fault cannot become the new normal (the spread is not learned while disagreeing)."""
        c = self.c
        d = self.last_delta
        self.n_mean += 1
        sd = math.sqrt(self.var) if self.n_ok >= c["min_n"] else sigma
        sd = max(sd, 0.3 * sigma)
        z = max(-6.0, min(6.0, d / sd))
        self.gp = max(0.0, self.gp + z - c["cusum_k"])
        self.gn = max(0.0, self.gn - z - c["cusum_k"])
        self.recent.append((d, z))
        items = self.win.last(c["fire_window_s"])
        mr = statistics.median(r for _, _, r in items)
        ratio = statistics.median(v for _, v, _ in items) / mr if abs(mr) > 1e-3 else None
        g = max(self.gp, self.gn)
        self.stats = {"n": len(items), "share": min(1.0, g / c["cusum_h"]), "ratio": ratio}
        self._peak("track", g / c["cusum_h"])
        old = self.verdict
        if old == "disagree":
            last = list(self.recent)[-c["recover_n"]:]
            if len(last) >= c["recover_n"] and all(abs(zz) < c["recover_z"] for _, zz in last):
                self.verdict = "agree"
                self.gp = self.gn = 0.0
        else:
            last = list(self.recent)[-2:]
            big = abs(sum(x for x, _ in last) / len(last)) > c["min_delta"] * max(abs(mr), 1.0)
            if g >= c["cusum_h"] and big:
                self.verdict = "disagree"
            elif self.n_mean >= c["min_n"]:
                self.verdict = "agree"
            if abs(z) < 3.0 and g < 0.5 * c["cusum_h"]:           # the healthy spread of this pair
                self.n_ok += 1
                self.var += (d * d - self.var) / min(self.n_ok, 20)
        self.suspect = False
        if self.verdict != old:
            self.changed_t = t
            return True
        return False

    def _evaluate(self, t):
        c = self.c
        fire = self.win.last(c["fire_window_s"])
        n = len(fire)
        share = sum(1 for b, _, _ in fire if b) / n if n else None
        ratio = None
        if n >= 3:
            mr = statistics.median(r for _, _, r in fire)
            ratio = statistics.median(v for _, v, _ in fire) / mr if abs(mr) > 1e-3 else None
        self.stats = {"n": n, "share": share, "ratio": ratio}
        if n >= c["min_n"] and c["mode"] == "share":
            self._peak("share", share / c["fire_share"])
        old = self.verdict
        if c["mode"] == "ratio":
            bad = n >= c["min_n"] and ratio is not None and not (c["ratio_lo"] <= ratio <= c["ratio_hi"])
            if n >= c["min_n"]:
                self.verdict = "disagree" if bad else "agree"
            self.suspect = False
        elif old == "disagree":
            rec = self.win.last(c["recover_window_s"])
            if self.by_cusum and n >= c["min_n"] and share >= c["fire_share"]:
                # the share test has reached its own level: the verdict is its verdict from here on (it is the
                # stronger statement, it may gate where the CUSUM only reports, and it ends by its own rule)
                self.by_cusum = False
                self.changed_t = t
                return True
            if self.by_cusum:
                if not self.cusum_on and (share is None or share <= c["recover_share"]):
                    self.verdict, self.by_cusum = "agree", False
            elif len(rec) >= c["recover_min_n"] and self.win.span() >= 0.9 * c["recover_window_s"] \
                    and sum(1 for b, _, _ in rec if b) / len(rec) <= c["recover_share"] \
                    and n >= c["min_n"] and share <= c["recover_share"] and not self.cusum_on:
                self.verdict = "agree"
        elif n >= c["min_n"] or self.cusum_on:
            fire = n >= c["min_n"] and share >= c["fire_share"]
            self.verdict = "disagree" if fire or self.cusum_on else "agree"
            self.by_cusum = self.cusum_on and not fire
            self.suspect = self.verdict == "agree" and share is not None and share >= c["suspect_share"]
            if self.by_cusum:
                self.stats["strength"] = 1.0           # the strength of the evidence, for the vote logic
        if self.verdict != old:
            self.changed_t = t
            return True
        return False

    def report(self):
        s = self.stats
        return {"kind": self.kind if self.c["mode"] != "ratio" else "witness", "a": self.ref, "b": self.tested,
                "quantity": self.quantity, "verdict": self.verdict, "n": s["n"],
                "by": self.decided_by() if self.verdict == "disagree" else None,
                "peak": {k: round(v, 2) for k, v in self.peak.items()},
                "share": None if s["share"] is None else round(s["share"], 2),
                "ratio": None if s["ratio"] is None else round(s["ratio"], 2),
                "evidence_s": round(min(self.win.span(), self.c["fire_window_s"]), 1)}

    def describe(self):
        s = self.stats
        txt = "%s disagrees with %s" % (self.quantity, self.ref)
        bits = []
        if s["ratio"] is not None:
            bits.append("ratio %.2f" % s["ratio"])
        if self.by_cusum:
            bits.append("accumulated difference: mean offset %.3g over the last %d samples" % (getattr(self, "cusum_delta", 0.0), len(self.recent)))
        elif s["share"] is not None and self.c["mode"] == "share":
            bits.append("%d %% of %d samples beyond %g sigma" % (round(100 * s["share"]), s["n"], self.c["z_bad"]))
        kind, what = self.kind_of()
        if kind is not None:
            bits.append("looks like %s: %s" % (kind, what))
        return txt + (" (" + ", ".join(bits) + ")" if bits else "")

    def kind_of(self):
        """The kind of the disagreement (sign, unit, scale, bias) from the samples of the window."""
        from .fdi import describe_pair
        return describe_pair([(v, r) for _, v, r in self.win.last(self.c["fire_window_s"])])

    def decided_by(self):
        """Which test holds the present verdict: "track" (a slow source compared by its mean), "cusum", "share"."""
        return "track" if self.c["mode"] == "mean" else "cusum" if self.by_cusum else "share"

    def strength(self):
        s = self.stats
        return s.get("strength", s.get("share")) if self.by_cusum else s.get("share")

    def expire(self, now, after_s=30.0):
        """A verdict held by the CUSUM alone is evidence of a difference that was accumulating; without a sample
        for `after_s` (the car stands, the source is silent) nothing accumulates and nothing clears it either: it
        lapses to "unknown". (A verdict of the share test stays until agreement is seen, as before.)"""
        if self.verdict == "disagree" and self.by_cusum and self.sample_t is not None and now - self.sample_t > after_s:
            self.verdict, self.by_cusum, self.cusum_on = "unknown", False, False
            self.gp = self.gn = 0.0
            self.changed_t = now
            return True
        return False

    def fresh(self, since_t):
        """Samples valid after a time (evidence gathered since a disagreement began)."""
        return sum(1 for t in self.times if t >= since_t)

    def reset_data(self):
        self.win.clear()
        self.pending.clear()
        self.gp = self.gn = 0.0
        self.recent.clear()
        self.cusum_on = False


class SelfCheck:
    """A source's stated uncertainty over its characterised one: the median over a window."""

    kind = "self"

    def __init__(self, group, quantity, cfg):
        self.group, self.quantity, self.c = group, quantity, cfg
        self.win = _Window(cfg["recover_window_s"], cfg["max_step_s"])
        self.fault = False
        self.median = None
        self.n = 0
        self.changed_t = None
        self.peak = 0.0

    def offer(self, t, ratio):
        c = self.c
        self.win.add(t, ratio)
        fire = self.win.last(c["fire_window_s"])
        self.n = len(fire)
        self.median = statistics.median(fire) if fire else None
        if self.n >= c["min_n"]:
            self.peak = max(self.peak, self.median / c["fire_ratio"])
        old = self.fault
        if not old:
            if self.n >= c["min_n"] and self.median > c["fire_ratio"]:
                self.fault = True
        else:
            rec = self.win.last(c["recover_window_s"])
            if len(rec) >= c["recover_min_n"] and self.win.span() >= 0.9 * c["recover_window_s"] \
                    and statistics.median(rec) < c["recover_ratio"] and self.median < c["recover_ratio"]:
                self.fault = False
        if self.fault != old:
            self.changed_t = t
            return True
        return False

    def report(self):
        return {"kind": "self", "a": self.group, "quantity": self.quantity,
                "verdict": "fault" if self.fault else ("ok" if self.n >= self.c["min_n"] else "unknown"),
                "n": self.n, "ratio": None if self.median is None else round(self.median, 2), "peak": round(self.peak, 2),
                "evidence_s": round(min(self.win.span(), self.c["fire_window_s"]), 1)}

    def describe(self):
        return "own %s uncertainty %.1fx its characterised level" % (self.quantity, self.median or 0.0)

    def reset_data(self):
        self.win.clear()


class NisMonitor:
    """Innovation consistency of one (group, quantity): the scaled median of the normalised innovations squared
    over a window of time (1 when the filter's stated uncertainty is right; the median, not the mean, so a
    heavy-tailed source is judged by its bulk)."""

    kind = "nis"
    MED = 0.4549                      # median of chi-square with 1 degree of freedom

    def __init__(self, group, quantity, cfg):
        self.group, self.quantity, self.c = group, quantity, cfg
        self.q = collections.deque(maxlen=cfg["max_n"])
        self._sorted = []                 # the window's values in order, kept as samples come and go (the median
        self._plain = True                # of 80 values five times a second per row class was a sort each time)
        self.value = None
        self.high = False
        self.n_total = 0
        self.t_eval = -1e18
        self.peak = 0.0

    def _median(self):
        """statistics.median of the window's values: from the ordered list (the same two middle values, the same
        arithmetic); from a fresh sort when a value that does not order (NaN) is or was in the window."""
        s = self._sorted
        n = len(s)
        if not self._plain or n != len(self.q):
            vals = [d for _, d in self.q]
            if all(d == d for d in vals):          # the value that did not order has left the window
                self._sorted, self._plain = sorted(vals), True
            return statistics.median(vals)
        return s[n // 2] if n % 2 else (s[n // 2 - 1] + s[n // 2]) / 2

    def _drop(self, d):
        s = self._sorted
        i = bisect.bisect_left(s, d)
        if i < len(s) and s[i] == d:
            del s[i]
        else:                              # not found: the list is not in order (a NaN went through it)
            self._plain = False

    def offer(self, t, d2):
        q = self.q
        if self._plain:
            if len(q) == q.maxlen:         # the deque drops its oldest sample by itself when it is full
                self._drop(q[0][1])
            if d2 == d2:
                bisect.insort(self._sorted, d2)
            else:
                self._plain = False
        q.append((t, d2))
        while q and q[0][0] < t - self.c["window_s"]:
            if self._plain:
                self._drop(q[0][1])
            q.popleft()
        self.n_total += 1
        if len(q) >= self.c["min_n"] and (t - self.t_eval >= 0.2 or t < self.t_eval):    # the median is not free: 5 Hz
            self.t_eval = t
            self.value = self._median() / self.MED
            self.peak = max(self.peak, self.value / self.c["fire"])
            old = self.high
            self.high = self.value > (self.c["recover"] if old else self.c["fire"])
            return self.high != old
        return False

    def report(self):
        return {"kind": "nis", "a": self.group, "quantity": self.quantity,
                "verdict": "unknown" if self.value is None else "high" if self.high else "ok",
                "n": len(self.q), "ratio": None if self.value is None else round(self.value, 2), "peak": round(self.peak, 2)}

    def describe(self):
        return "%s innovations %.1fx the stated uncertainty" % (self.quantity, self.value or 0.0)

    def reset_data(self):
        self.q.clear()
        self._sorted, self._plain = [], True
        self.value = None
        self.high = False
