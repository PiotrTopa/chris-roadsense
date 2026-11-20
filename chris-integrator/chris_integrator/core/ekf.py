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

from ..native import ekf as _N

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
    __slots__ = ("t", "rows", "x", "P", "xp", "Pp", "F", "anchor", "usedq")

    def __init__(self, t):
        self.t = t
        self.rows = []
        self.usedq = ()                    # (q, src) of the rows applied in the last filter pass (for `near`)
        self.x = self.P = self.xp = self.Pp = self.F = None
        self.anchor = None


def wrap_pi(a):
    return (a + math.pi) % (2.0 * math.pi) - math.pi


# ---- the arithmetic of one scalar row -------------------------------------------------------------------
# Two implementations of the same three steps: numpy (the reference) and the native core (when it is in use:
# `chris_integrator.native`). `_run` below is the same code for both. The native steps work in place on arrays
# the loop owns (`own`: this pass made the array, nothing else holds it); the numpy steps make new arrays.
def _project_py(P, idx, coef, buf=None):
    """(ph, s0): P h^T and h P h^T of a sparse row h. (`buf`: a work array of the pass, used by the native step.)"""
    ph = P[:, idx] @ coef
    return ph, float(coef @ ph[idx])


def _apply_py(x, P, ph, s, nu, freeze, own):
    """The row's correction: x in place (the caller's own copy), the new P returned."""
    k = ph / s
    if freeze is None:
        x += k * nu
        return P - k[:, None] * ph[None, :]
    # Schmidt-Kalman: the frozen states' uncertainty is in S, but they are not corrected by this
    # row (the gain is no longer optimal: the Joseph form of the covariance update)
    k[freeze] = 0.0
    x += k * nu
    kp = k[:, None] * ph[None, :]
    P = P - kp - kp.T + s * (k[:, None] * k[None, :])
    return 0.5 * (P + P.T)


def _sym_py(P, own):
    return 0.5 * (P + P.T)


def _project_native(P, idx, coef, buf=None):
    ph = np.empty(P.shape[0]) if buf is None else buf
    try:
        return ph, _N.ekf_project(P, idx, coef, ph)
    except (TypeError, ValueError, BufferError):       # a row model that returns lists, or a view
        return _project_py(P, np.asarray(idx, dtype=np.intp), np.asarray(coef, dtype=float))


def _apply_native(x, P, ph, s, nu, freeze, own):
    if not own:
        P = P.copy()
    try:
        _N.ekf_update(x, P, ph, s, nu, freeze)
    except (TypeError, ValueError, BufferError):
        return _apply_py(x, P, ph, s, nu, freeze, own)
    return P


def _sym_native(P, own):
    if not own:
        return _sym_py(P, own)
    _N.ekf_sym(P)
    return P


if _N is None:
    _project, _apply, _sym = _project_py, _apply_py, _sym_py
else:
    _project, _apply, _sym = _project_native, _apply_native, _sym_native


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
        self.stat_nu = None                # callable(row, nu, s, t): ... its signed innovation and variance
        self.reruns = 0                    # entries re-processed because of late rows (cost accounting)
        self.reset()

    def reset(self, x=None, P=None):
        if x is not None:
            self.x0, self.P0 = np.array(x, float), np.array(P, float)
        self.entries, self.times = [], []
        self.carry = []
        self.base = None                   # (t, x, P): the state the first entry is predicted from
        self.dirty = None

    def rebase(self, t, x, P):
        """Restart the buffer from a given state (a re-anchor, an epoch): earlier entries are dropped. What they
        say about *when each quantity was last measured* is kept (`carry`): re-placing the position does not make
        the speed that was measured 50 ms ago unmeasured (before L43 a frame right after a re-anchoring was
        published with speed, yaw rate and lead unknown, "no_measurement")."""
        # (what an earlier restart carried and is still recent stays carried: two restarts in a row, a position
        # declared lost and placed again within one message, must not forget it: L47)
        carry = {(q, src): tt for tt, q, src in self.carry if tt >= t - 2.0}
        for e in self.entries[-64:]:
            if e.t >= t - 2.0:
                for r in e.rows:
                    if r.used:
                        carry[(r.q, r.src)] = e.t
        self.carry = [(tt, q, src) for (q, src), tt in carry.items()]
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
        buf = np.empty(x.shape[0])         # P h^T of the row in hand: used up before the next row is projected
        for e in self.entries[i:]:
            dt = e.t - t
            if dt > 0.0:
                x, P, F = predict(x, P, dt)
            else:
                F = None
            e.xp, e.Pp, e.F = x, P, F
            own = pown = dirty = False     # x / P are arrays of this entry's own (not the predicted ones it keeps)
            for r in e.rows:
                was = r.rejected
                r.used = r.rejected = False
                if r.group in disabled or (r.group, r.q) in disabled:
                    continue
                pred, idx, coef = r.h(x)
                ph, s0 = _project(P, idx, coef, buf)
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
                    if self.stat_nu is not None:
                        self.stat_nu(r, nu, s0 + var, e.t)
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
                if not own:
                    x, own = x.copy(), True                # (the predicted state is kept in the entry)
                P = _apply(x, P, ph, s, nu, r.freeze, pown)
                pown = True
                r.used = True
                dirty = True
            if dirty:
                P = _sym(P, pown)
                e.usedq = tuple([(r.q, r.src) for r in e.rows if r.used])
            else:
                e.usedq = ()
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
        # a model may return its transition in a structured form and multiply by it itself (`apply_F`)
        fmul = getattr(self.model, "apply_F", None) or (lambda F, M: F @ M)
        rts = getattr(self.model, "rts", None)         # a model may do the whole step itself (the native core)

        def back(Pp, F, Pa, xa, xp, xs, Ps):
            """One step back over a prediction: the smoothed (x, P) at its start from the smoothed at its end."""
            if rts is not None:
                r = rts(Pp, F, Pa, xa, xp, xs, Ps, self.wrap_idx)
                if r is False:
                    raise np.linalg.LinAlgError("singular")
                if r is not None:
                    return r
            C = np.linalg.solve(Pp, fmul(F, Pa)).T
            return xa + C @ self._diff(xs, xp), Pa + C @ (Ps - Pp) @ C.T
        try:
            for j in range(j1, j0, -1):                # smooth back to entry j0
                e, a = self.entries[j], self.entries[j - 1]
                if e.F is None:
                    continue
                xs, Ps = back(e.Pp, e.F, a.P, a.x, e.xp, xs, Ps)
            e = self.entries[j0]
            dt = e.t - T
            if dt > 0.0:                               # and from entry j0 back to the node at T
                xp, Pp, F = self.model.predict(x, P, dt)
                xs, Ps = back(Pp, F, P, x, xp, xs, Ps)
            Ps = 0.5 * (Ps + Ps.T)
        except np.linalg.LinAlgError:
            return x, P
        return xs, Ps

    def near(self, T, window_s, kinds=None, reach_s=1.0):
        """(age_s of the nearest applied row of `kinds` within reach_s, sorted source ids applied within the
        window)."""
        reach = max(window_s, reach_s)
        times, entries = self.times, self.entries
        lo, hi = bisect.bisect_left(times, T - reach), bisect.bisect_right(times, T + reach)
        # The entries about the window first: they give the sources, and the nearest row if there is one (every
        # entry outside is further away). Only when nothing was applied there is the rest of the reach walked,
        # outwards, to the first applied row on either side. (The reach of a position is the whole buffer: this
        # was a walk over every row of ten seconds, several times a frame.) Membership of the window is decided
        # by `d <= window_s` as before; the slice is a hair wider than it.
        wl = max(lo, bisect.bisect_left(times, T - window_s - 1e-6))
        wh = min(hi, bisect.bisect_right(times, T + window_s + 1e-6))
        age, srcs = None, set()
        for e in entries[wl:wh]:               # (`usedq`: the rows the last pass applied, kept per entry by `_run`)
            u = e.usedq
            if not u:
                continue
            d = abs(e.t - T)
            for q, src in u:
                if kinds is None or q in kinds:
                    if d <= window_s:
                        srcs.add(src)
                    if age is None or d < age:
                        age = d
        if age is None:
            for k in range(wl - 1, lo - 1, -1):
                u = entries[k].usedq
                if u and (kinds is None or any(q in kinds for q, _ in u)):
                    age = abs(entries[k].t - T)
                    break
            for k in range(wh, hi):
                u = entries[k].usedq
                if u and (kinds is None or any(q in kinds for q, _ in u)):
                    d = abs(entries[k].t - T)
                    if age is None or d < age:
                        age = d
                    break
        for tt, q, src in getattr(self, "carry", ()):      # measured before the buffer was restarted
            d = abs(tt - T)
            if d <= reach and (kinds is None or q in kinds) and q != "pos":
                if d <= window_s:
                    srcs.add(src)
                if age is None or d < age:
                    age = d
        return age, sorted(srcs)
