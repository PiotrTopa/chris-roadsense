"""Time-ordered linear Kalman filter with insert-and-re-run and fixed-lag (RTS) smoothing.

A `Channel` holds a short buffer of measurement entries sorted by their time of validity. A measurement that
arrives late is inserted where it belongs and the filter is run again from there; a request for the state at a
time T returns the smoothed state (everything in the buffer, also what is newer than T, counts). The process
models used here are exact discretisations (F(a + b) = F(b) F(a), Q composes), so stored posteriors stay valid
when a query puts a node between two entries.
"""
import bisect

import numpy as np


class Row:
    __slots__ = ("q", "h", "z", "var", "src", "group", "force", "used", "rejected", "obs")

    def __init__(self, q, h, z, var, src, group, force=False):
        self.q, self.h, self.z, self.var, self.src, self.group, self.force = q, h, z, var, src, group, force
        self.obs = int(np.argmax(h != 0.0))        # the state it observes (the first one in its row)
        self.used = False
        self.rejected = False


class Entry:
    __slots__ = ("t", "rows", "x", "P", "anchor")

    def __init__(self, t):
        self.t = t
        self.rows = []
        self.x = self.P = None
        self.anchor = None                 # state index -> time of the last applied ungated measurement of it


class Channel:
    def __init__(self, name, fq, x0, p0, horizon_s=2.0, gate_sigma=5.0, lookahead_s=0.6, anchor_s=0.5):
        """fq(dt) -> (F, Q); x0, p0: the state and its variances before the first measurement. A gated
        (heavy-tailed) measurement is only ever rejected while an ungated measurement of the same state was
        applied within anchor_s before it: a source that is alone is followed, never locked out by its own
        filter."""
        self.anchor_s = anchor_s
        self.name, self.fq = name, fq
        self.x0, self.P0 = np.array(x0, float), np.diag(np.array(p0, float))
        self.n = len(x0)
        self.horizon, self.gate2, self.lookahead = horizon_s, gate_sigma * gate_sigma, lookahead_s
        self.disabled = frozenset()
        self.rejected = {}                 # src -> count of measurements not applied (innovation gate)
        self.reset()

    def reset(self):
        self.entries, self.times = [], []
        self.base = None                   # (t, x, P): the state the first entry is predicted from
        self.dirty = None                  # the filter has to be run again from this time on (flush)

    # ---- input ---------------------------------------------------------------------------------------
    def insert(self, t, row):
        """Put a measurement at its time of validity; the filter is run again from there at the next flush().
        False when it is older than the buffer (not applied)."""
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
            self._run(i)

    def set_disabled(self, groups):
        groups = frozenset(groups)
        if groups != self.disabled:
            self.disabled = groups
            self.dirty = None
            self._run(0)

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
        for e in self.entries[i:]:
            dt = e.t - t
            if dt > 0.0:
                F, Q = self.fq(dt)
                x = F @ x
                P = F @ P @ F.T + Q
            for r in e.rows:
                was = r.rejected
                r.used = r.rejected = False
                if r.group in self.disabled:
                    continue
                ph = P @ r.h
                s = float(r.h @ ph) + r.var
                nu = r.z - float(r.h @ x)
                if not r.force and nu * nu > self.gate2 * s and e.t - anchor.get(r.obs, -1e18) <= self.anchor_s:
                    r.rejected = True
                    if not was:
                        self.rejected[r.src] = self.rejected.get(r.src, 0) + 1
                    continue
                k = ph / s
                x = x + k * nu
                P = P - k[:, None] * ph
                P = 0.5 * (P + P.T)
                r.used = True
                if r.force:
                    anchor[r.obs] = e.t
            e.x, e.P, t, e.anchor = x, P, e.t, dict(anchor)

    # ---- output --------------------------------------------------------------------------------------
    def at(self, T):
        """Smoothed (x, P) at time T, or None before the first measurement."""
        if self.base is None:
            return None
        self.flush()
        k = bisect.bisect_right(self.times, T) - 1
        if k < 0:
            t, x, P = self.base
        else:
            e = self.entries[k]
            t, x, P = e.t, e.x, e.P
        dt = T - t
        if dt > 0.0:
            F, Q = self.fq(dt)
            x = F @ x
            P = F @ P @ F.T + Q
        nodes = [(x, P)]
        preds = []
        tp = T
        for e in self.entries[k + 1:]:
            if e.t - T > self.lookahead:
                break
            dt = e.t - tp
            if dt <= 0.0:
                continue
            F, Q = self.fq(dt)
            xa, Pa = nodes[-1]
            preds.append((F, F @ xa, F @ Pa @ F.T + Q))
            nodes.append((e.x, e.P))
            tp = e.t
        xs, Ps = nodes[-1]
        try:
            for j in range(len(preds) - 1, -1, -1):
                F, xp, Pp = preds[j]
                xa, Pa = nodes[j]
                C = np.linalg.solve(Pp.T, (Pa @ F.T).T).T
                xs = xa + C @ (xs - xp)
                Ps = Pa + C @ (Ps - Pp) @ C.T
        except np.linalg.LinAlgError:
            xs, Ps = nodes[0]
        return xs, Ps

    def near(self, T, window_s, kinds=None, reach_s=1.0):
        """(age_s of the nearest applied measurement of `kinds` within reach_s, sorted source ids applied within
        the window, floor): floor is the smallest sigma of the gated measurements of `kinds` in the window when no
        ungated one is there, else None (the error of a heavy-tailed source alone is correlated in time: filtering
        does not average it down, and the reported std must not pretend it does)."""
        reach = max(window_s, reach_s)
        lo, hi = bisect.bisect_left(self.times, T - reach), bisect.bisect_right(self.times, T + reach)
        age, srcs = None, set()
        floor, anchored = None, False
        for e in self.entries[lo:hi]:
            d = abs(e.t - T)
            for r in e.rows:
                if not r.used:
                    continue
                if d <= window_s:
                    srcs.add(r.src)
                if kinds is None or r.q in kinds:
                    if age is None or d < age:
                        age = d
                    if d <= window_s:
                        if r.force:
                            anchored = True
                        elif floor is None or r.var < floor:
                            floor = r.var
        return age, sorted(srcs), (None if anchored or floor is None else floor ** 0.5)


def fq_speed(q_jerk, q_bias):
    """State [v, a, b]: v' = a, a' = white noise (PSD q_jerk), b' = white noise (PSD q_bias)."""
    def fq(dt):
        F = np.array([[1.0, dt, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]])
        Q = np.array([[q_jerk * dt ** 3 / 3.0, q_jerk * dt * dt / 2.0, 0.0],
                      [q_jerk * dt * dt / 2.0, q_jerk * dt, 0.0],
                      [0.0, 0.0, q_bias * dt]])
        return F, Q
    return fq


def fq_yaw(q):
    """State [r, r']: r'' = white noise (PSD q)."""
    def fq(dt):
        F = np.array([[1.0, dt], [0.0, 1.0]])
        Q = np.array([[q * dt ** 3 / 3.0, q * dt * dt / 2.0], [q * dt * dt / 2.0, q * dt]])
        return F, Q
    return fq
