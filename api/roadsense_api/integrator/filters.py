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
    __slots__ = ("q", "h", "z", "var", "src", "group", "force", "used", "rejected")

    def __init__(self, q, h, z, var, src, group, force=False):
        self.q, self.h, self.z, self.var, self.src, self.group, self.force = q, h, z, var, src, group, force
        self.used = False
        self.rejected = False


class Entry:
    __slots__ = ("t", "rows", "x", "P")

    def __init__(self, t):
        self.t = t
        self.rows = []
        self.x = self.P = None


class Channel:
    def __init__(self, name, fq, x0, p0, horizon_s=2.0, gate_sigma=5.0, lookahead_s=0.6):
        """fq(dt) -> (F, Q); x0, p0: the state and its variances before the first measurement."""
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

    # ---- input ---------------------------------------------------------------------------------------
    def insert(self, t, row):
        """Put a measurement at its time of validity and re-run the filter from there. False when it is older
        than the buffer (not applied)."""
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
        self._run(i)
        return True

    def set_disabled(self, groups):
        groups = frozenset(groups)
        if groups != self.disabled:
            self.disabled = groups
            self._run(0)

    def prune(self, now):
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
        if i == 0:
            t, x, P = self.base
        else:
            e = self.entries[i - 1]
            t, x, P = e.t, e.x, e.P
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
                if not r.force and nu * nu > self.gate2 * s:
                    r.rejected = True
                    if not was:
                        self.rejected[r.src] = self.rejected.get(r.src, 0) + 1
                    continue
                k = ph / s
                x = x + k * nu
                P = P - np.outer(k, ph)
                P = 0.5 * (P + P.T)
                r.used = True
            e.x, e.P, t = x, P, e.t

    # ---- output --------------------------------------------------------------------------------------
    def at(self, T):
        """Smoothed (x, P) at time T, or None before the first measurement."""
        if self.base is None:
            return None
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

    def near(self, T, window_s, kinds=None):
        """(age_s of the nearest applied measurement of `kinds`, sorted source ids applied within the window)."""
        lo = bisect.bisect_left(self.times, T - self.horizon)
        age, srcs = None, set()
        for e in self.entries[lo:]:
            d = abs(e.t - T)
            for r in e.rows:
                if not r.used:
                    continue
                if d <= window_s:
                    srcs.add(r.src)
                if (kinds is None or r.q in kinds) and (age is None or d < age):
                    age = d
        return age, sorted(srcs)


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
