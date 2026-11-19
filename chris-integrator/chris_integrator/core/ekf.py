"""Time-ordered extended Kalman filter: insert-and-re-run for late measurements, fixed-lag smoothing.

The buffer holds entries sorted by time of validity; each entry holds scalar measurement rows. A row that
arrives late is inserted where it belongs and the filter is run again from there (the exact answer to the
out-of-sequence problem: the result is what it would have been had the row arrived in order). `at(T)` returns
the Rauch-Tung-Striebel smoothed state at T over the entries newer than T (bounded look-ahead).

Rows are scalar: value, variance, and `h(x) -> (prediction, indices, coefficients)` (a sparse Jacobian row).
Noise classes: "exact" (applied as is; it anchors its quantity) and "robust" (Student-t re-weighting; beyond
the hard gate it is not applied, but only while an exact row anchors the same quantity: a source that is
alone is followed, never locked out). No knowledge of what the state means lives here.
"""
import bisect
import math

import numpy as np

T_NU = 4.0                      # degrees of freedom of the robust rows' Student-t


class Row:
    __slots__ = ("q", "z", "var", "h", "src", "group", "robust", "wrap", "used", "rejected", "counted",
                 "nis", "weight", "cb", "freeze")

    def __init__(self, q, z, var, h, src, group, robust=False, wrap=False, cb=None, freeze=None):
        self.q, self.z, self.var, self.h, self.src, self.group = q, z, var, h, src, group
        self.robust, self.wrap, self.cb = robust, wrap, cb
        self.freeze = freeze               # state indices this row must not correct (Schmidt "consider" states)
        self.used = self.rejected = self.counted = False
        self.nis = None
        self.weight = 1.0


class Entry:
    __slots__ = ("t", "rows", "x", "P", "xp", "Pp", "F", "anchor")

    def __init__(self, t):
        self.t = t
        self.rows = []
        self.x = self.P = self.xp = self.Pp = self.F = None
        self.anchor = None


def wrap_pi(a):
    return (a + math.pi) % (2.0 * math.pi) - math.pi


class Filter:
    def __init__(self, model, x0, P0, horizon_s=10.0, gate_chi2=23.9, lookahead_s=0.6, anchor_s=0.5,
                 wrap_idx=()):
        """model.predict(x, P, dt) -> (x, P, F). gate_chi2: hard gate of robust rows (chi-square, 1 dof;
        23.9 = 1e-6 tail). wrap_idx: indices of angle states (differences are wrapped)."""
        self.model = model
        self.x0, self.P0 = np.array(x0, float), np.array(P0, float)
        self.n = len(self.x0)
        self.horizon, self.gate, self.lookahead, self.anchor_s = horizon_s, gate_chi2, lookahead_s, anchor_s
        self.wrap_idx = tuple(wrap_idx)
        self.disabled = frozenset()
        self.quarantine = frozenset()      # (group, q): rows still evaluated (their innovations are watched), not applied
        self.rejected = {}                 # src -> rows not applied (hard gate)
        self.stat = None                   # callable(row, d2, t): first-pass normalised innovation of a row
        self.reruns = 0                    # entries re-processed because of late rows (cost accounting)
        self.reset()

    def reset(self, x=None, P=None):
        if x is not None:
            self.x0, self.P0 = np.array(x, float), np.array(P, float)
        self.entries, self.times = [], []
        self.base = None                   # (t, x, P): the state the first entry is predicted from
        self.dirty = None

    def rebase(self, t, x, P):
        """Restart the buffer from a given state (a re-anchor, an epoch): earlier entries are dropped."""
        self.entries, self.times = [], []
        self.base = (t, np.array(x, float), np.array(P, float))
        self.dirty = None

    # ---- input ---------------------------------------------------------------------------------------
    def insert(self, t, row):
        """False when the row is older than the buffer (not applied)."""
        if self.base is None:
            self.base = (t, self.x0.copy(), self.P0.copy())
        elif t < self.base[0]:
            return False
        i = bisect.bisect_left(self.times, t)
        if i < len(self.times) and self.times[i] - t < 1e-6:
            e = self.entries[i]
        elif i > 0 and t - self.times[i - 1] < 1e-6:
            i -= 1
            e = self.entries[i]
        else:
            e = Entry(t)
            self.entries.insert(i, e)
            self.times.insert(i, t)
        e.rows.append(row)
        if self.dirty is None or e.t < self.dirty:
            self.dirty = e.t
        return True

    def flush(self):
        if self.dirty is not None:
            i = bisect.bisect_left(self.times, self.dirty - 1e-6)
            self.dirty = None
            self.reruns += max(0, len(self.entries) - i - 1)
            self._run(i)

    def set_disabled(self, groups):
        groups = frozenset(groups)
        if groups != self.disabled:
            self.disabled = groups
            self.dirty = None
            self._run(0)

    def set_quarantine(self, keys):
        keys = frozenset(keys)
        if keys != self.quarantine:
            self.quarantine = keys         # from now on: rows already applied stay applied (no re-run)

    def prune(self, now):
        self.flush()
        lim = now - self.horizon
        n = 0
        while n < len(self.times) - 1 and self.times[n] < lim:
            n += 1
        if n:
            e = self.entries[n - 1]
            self.base = (e.t, e.x, e.P)
            del self.entries[:n], self.times[:n]

    # ---- filter --------------------------------------------------------------------------------------
    def _run(self, i):
        if self.base is None:
            return
        if i == 0:
            t, x, P = self.base
            anchor = {}
        else:
            e = self.entries[i - 1]
            t, x, P = e.t, e.x, e.P
            anchor = dict(e.anchor)
        predict = self.model.predict
        disabled, gate, stat = self.disabled, self.gate, self.stat
        for e in self.entries[i:]:
            dt = e.t - t
            if dt > 0.0:
                x, P, F = predict(x, P, dt)
            else:
                F = None
                x, P = x.copy(), P
            e.xp, e.Pp, e.F = x, P, F
            for r in e.rows:
                was = r.rejected
                r.used = r.rejected = False
                if r.group in disabled:
                    continue
                pred, idx, coef = r.h(x)
                ph = P[:, idx] @ coef
                s0 = float(coef @ ph[idx])
                nu = r.z - pred
                if r.wrap:
                    nu = wrap_pi(nu)
                var = r.var
                d2 = nu * nu / (s0 + var)
                r.nis = d2
                if not r.counted:
                    r.counted = True
                    if stat is not None:
                        stat(r, d2, e.t)
                    if r.cb is not None:
                        r.cb(nu, s0, e.t)
                if self.quarantine and (r.group, r.q) in self.quarantine:
                    continue
                if r.robust:
                    if d2 > gate and e.t - anchor.get(r.q, -1e18) <= self.anchor_s:
                        r.rejected = True
                        if not was:
                            self.rejected[r.src] = self.rejected.get(r.src, 0) + 1
                        continue
                    # Student-t weight on the innovation: the whole innovation variance is inflated, so the
                    # state moves by weight x the Kalman step however small the row's own noise is
                    w = (T_NU + 1.0) / (T_NU + d2)
                    if w < 1.0:
                        var = (s0 + var) / w - s0
                    r.weight = min(w, 1.0)
                else:
                    anchor[r.q] = e.t
                s = s0 + var
                k = ph / s
                x = x.copy()
                if r.freeze is None:
                    x += k * nu
                    P = P - np.outer(k, ph)
                else:
                    # Schmidt-Kalman: the frozen states' uncertainty is in S, but they are not corrected by this
                    # row (the gain is no longer optimal: the Joseph form of the covariance update)
                    k[r.freeze] = 0.0
                    x += k * nu
                    kp = np.outer(k, ph)
                    P = P - kp - kp.T + s * np.outer(k, k)
                P = 0.5 * (P + P.T)
                r.used = True
            e.x, e.P, t, e.anchor = x, P, e.t, dict(anchor)

    # ---- output --------------------------------------------------------------------------------------
    def latest(self):
        """(t, x, P) filtered at the newest entry, or None."""
        if self.base is None:
            return None
        self.flush()
        if self.entries:
            e = self.entries[-1]
            return e.t, e.x, e.P
        return self.base

    def filtered(self, T):
        """(x, P) predicted from the last entry at or before T (no smoothing)."""
        if self.base is None:
            return None
        self.flush()
        k = bisect.bisect_right(self.times, T) - 1
        t, x, P = self.base if k < 0 else (self.entries[k].t, self.entries[k].x, self.entries[k].P)
        dt = T - t
        if dt > 0.0:
            x, P, _ = self.model.predict(x, P, dt)
        return x, P

    def _diff(self, a, b):
        d = a - b
        for i in self.wrap_idx:
            d[i] = wrap_pi(d[i])
        return d

    def at(self, T):
        """Smoothed (x, P) at time T, or None before the first measurement."""
        if self.base is None:
            return None
        self.flush()
        k = bisect.bisect_right(self.times, T) - 1
        t, x, P = self.base if k < 0 else (self.entries[k].t, self.entries[k].x, self.entries[k].P)
        dt = T - t
        if dt > 0.0:
            x, P, _ = self.model.predict(x, P, dt)
        j0 = k + 1
        if j0 >= len(self.entries) or self.entries[j0].t - T > self.lookahead:
            return x, P
        j1 = j0
        while j1 + 1 < len(self.entries) and self.entries[j1 + 1].t - T <= self.lookahead:
            j1 += 1
        xs, Ps = self.entries[j1].x, self.entries[j1].P
        try:
            for j in range(j1, j0, -1):                # smooth back to entry j0
                e, a = self.entries[j], self.entries[j - 1]
                if e.F is None:
                    continue
                C = np.linalg.solve(e.Pp, e.F @ a.P).T
                xs = a.x + C @ self._diff(xs, e.xp)
                Ps = a.P + C @ (Ps - e.Pp) @ C.T
            e = self.entries[j0]
            dt = e.t - T
            if dt > 0.0:                               # and from entry j0 back to the node at T
                xp, Pp, F = self.model.predict(x, P, dt)
                C = np.linalg.solve(Pp, F @ P).T
                xs = x + C @ self._diff(xs, xp)
                Ps = P + C @ (Ps - Pp) @ C.T
            Ps = 0.5 * (Ps + Ps.T)
        except np.linalg.LinAlgError:
            return x, P
        return xs, Ps

    def near(self, T, window_s, kinds=None, reach_s=1.0):
        """(age_s of the nearest applied row of `kinds` within reach_s, sorted source ids applied within the
        window)."""
        reach = max(window_s, reach_s)
        lo, hi = bisect.bisect_left(self.times, T - reach), bisect.bisect_right(self.times, T + reach)
        age, srcs = None, set()
        for e in self.entries[lo:hi]:
            d = abs(e.t - T)
            for r in e.rows:
                if not r.used:
                    continue
                if kinds is None or r.q in kinds:
                    if d <= window_s:
                        srcs.add(r.src)
                    if age is None or d < age:
                        age = d
        return age, sorted(srcs)
