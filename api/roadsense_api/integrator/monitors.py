"""Consistency monitors: raw measurements of one source against raw measurements of another (never against the
fused state), a source's stated uncertainty against its characterised one, and a slow witness test for sources
whose timing is poor. Each monitor keeps a window of *evidence time* and a verdict with hysteresis.
"""
import bisect
import collections
import math
import statistics


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
        if max(abs(v), abs(rv)) < c["min_evidence"]:
            return False
        z = (v - rv) / math.hypot(s, rs)
        self.win.add(t, (abs(z) > c["z_bad"], v, rv))
        return self._evaluate(t)

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
        old = self.verdict
        if c["mode"] == "ratio":
            bad = n >= c["min_n"] and ratio is not None and not (c["ratio_lo"] <= ratio <= c["ratio_hi"])
            if n >= c["min_n"]:
                self.verdict = "disagree" if bad else "agree"
            self.suspect = False
        elif old == "disagree":
            rec = self.win.last(c["recover_window_s"])
            if len(rec) >= c["recover_min_n"] and self.win.span() >= 0.9 * c["recover_window_s"] \
                    and sum(1 for b, _, _ in rec if b) / len(rec) <= c["recover_share"] \
                    and n >= c["min_n"] and share <= c["recover_share"]:
                self.verdict = "agree"
        elif n >= c["min_n"]:
            self.verdict = "disagree" if share >= c["fire_share"] else "agree"
            self.suspect = self.verdict == "agree" and share >= c["suspect_share"]
        if self.verdict != old:
            self.changed_t = t
            return True
        return False

    def report(self):
        s = self.stats
        return {"kind": self.kind if self.c["mode"] != "ratio" else "witness", "a": self.ref, "b": self.tested,
                "quantity": self.quantity, "verdict": self.verdict, "n": s["n"],
                "share": None if s["share"] is None else round(s["share"], 2),
                "ratio": None if s["ratio"] is None else round(s["ratio"], 2),
                "evidence_s": round(min(self.win.span(), self.c["fire_window_s"]), 1)}

    def describe(self):
        s = self.stats
        txt = "%s disagrees with %s" % (self.quantity, self.ref)
        bits = []
        if s["ratio"] is not None:
            bits.append("ratio %.2f" % s["ratio"])
        if s["share"] is not None and self.c["mode"] != "ratio":
            bits.append("%d %% of %d samples beyond %g sigma" % (round(100 * s["share"]), s["n"], self.c["z_bad"]))
        return txt + (" (" + ", ".join(bits) + ")" if bits else "")

    def reset_data(self):
        self.win.clear()
        self.pending.clear()


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

    def offer(self, t, ratio):
        c = self.c
        self.win.add(t, ratio)
        fire = self.win.last(c["fire_window_s"])
        self.n = len(fire)
        self.median = statistics.median(fire) if fire else None
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
                "n": self.n, "ratio": None if self.median is None else round(self.median, 2),
                "evidence_s": round(min(self.win.span(), self.c["fire_window_s"]), 1)}

    def describe(self):
        return "own %s uncertainty %.1fx its characterised level" % (self.quantity, self.median or 0.0)

    def reset_data(self):
        self.win.clear()
