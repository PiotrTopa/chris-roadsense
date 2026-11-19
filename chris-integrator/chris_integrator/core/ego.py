"""The ego state and its process model (docs/INTEGRATOR.md section 2).

Core (kinematic) states, in this order:
    PE, PN   position east / north of the local origin, m
    PSI      heading, rad counter-clockwise from east
    V, A     speed along the heading (signed) and its rate
    W, AL    yaw rate (positive left) and its rate
    Z, TH    altitude and road grade (rad)
    SV       scale of the speed state against the ground: V is the speed as the car's own speed signal scales it
             (what odometry-minded consumers want, known to centimetres per second); the distance over ground is
             V / (1 + SV). SV is observable only against absolute sources (positions over distance, a radar's
             stationary world); without one it keeps its prior.
Sources append their own calibration states (`AuxState`): random walk or first-order Gauss-Markov.
"""
import math

import numpy as np

PE, PN, PSI, V, A, W, AL, Z, TH, SV = range(10)
NCORE = 10
CORE_NAMES = ("pe", "pn", "psi", "v", "a", "w", "al", "z", "th", "sv")
G = 9.80665

DEFAULT_PROCESS = dict(
    jerk_psd=2.0,                # (m/s3)^2 s: how fast the longitudinal acceleration changes
    yaw_jerk_psd=1.0,            # (rad/s3)^2 s: how fast the yaw acceleration changes
    pos_psd=0.01,                # m^2 / s: position noise not explained by speed and heading (slip)
    alt_psd=0.01,                # m^2 / s
    grade_psd=4e-5,              # rad^2 / s: grade changes by about 0.6 % in 1 s (1 sigma)
    scale_psd=1e-9,              # 1 / s: the speed scale is a tyre property
    scale_sigma0=0.02,           # prior of the speed scale
)
# before any measurement: nothing is known but that it is a car
X0 = np.zeros(NCORE)
P0_DIAG = np.array([1e-4, 1e-4, 1e-6, 1e4, 9.0, 1.0, 1.0, 1e4, 0.01, 0.02 ** 2])


_CAPS = ((V, 60.0 ** 2), (A, 5.0 ** 2), (W, 0.6 ** 2), (AL, 2.0 ** 2), (PSI, math.pi ** 2))


class AuxState:
    """A calibration state a source asks the estimator to carry.

    name    unique within the source ("tau", "s_v", ...); the state's full name is "<source id>.<name>"
    x0, sigma0   prior
    psd     random walk: variance added per second          (tau_s None)
    tau_s   Gauss-Markov: correlation time; `sigma0` is then also its stationary sigma
    lo, hi  optional clamp of the estimate
    """
    __slots__ = ("name", "x0", "sigma0", "psd", "tau_s", "lo", "hi", "unit", "doc")

    def __init__(self, name, x0=0.0, sigma0=1.0, psd=0.0, tau_s=None, lo=None, hi=None, unit="", doc=""):
        self.name, self.x0, self.sigma0, self.psd, self.tau_s = name, x0, sigma0, psd, tau_s
        self.lo, self.hi, self.unit, self.doc = lo, hi, unit, doc


class EgoModel:
    """Process model over [core | aux...]."""

    def __init__(self, process=None):
        self.c = dict(DEFAULT_PROCESS, **(process or {}))
        self.aux = []                      # (full name, AuxState)
        self.index = {n: i for i, n in enumerate(CORE_NAMES)}
        self.n = NCORE
        self._psd = np.zeros(0)
        self._gm = []                      # (index, tau_s, sigma)
        self._clamp = []
        self.heading_free = True           # False: the heading does not move the position yet (not initialised)
        self._rho_cache = {}
        self._diag = None

    @staticmethod
    def apply_F(F, M):
        """F @ M for the transition `predict` returned (kinematic block and a diagonal)."""
        Fc, rho = F
        out = M.copy()
        out[:NCORE, :] = Fc @ M[:NCORE, :]
        if rho is not None:
            out *= rho[:, None]
        return out

    def add_aux(self, owner, aux):
        name = "%s.%s" % (owner, aux.name)
        if name in self.index:
            return self.index[name]
        i = self.n
        self.index[name] = i
        self.aux.append((name, aux))
        self.n += 1
        self._psd = np.append(self._psd, 0.0 if aux.tau_s else aux.psd)
        self._rho_cache.clear()            # (the cached vectors have the old length)
        if aux.tau_s:
            self._gm.append((i, aux.tau_s, aux.sigma0))
        if aux.lo is not None or aux.hi is not None:
            self._clamp.append((i, aux.lo, aux.hi))
        return i

    def initial(self):
        x = np.concatenate([X0, [a.x0 for _, a in self.aux]])
        p0 = P0_DIAG.copy()
        p0[SV] = self.c["scale_sigma0"] ** 2
        P = np.diag(np.concatenate([p0, [a.sigma0 ** 2 for _, a in self.aux]]))
        return x, P

    def rate(self, x):
        """dx/dt of the core states (for time-delay Jacobians)."""
        c, s = math.cos(x[PSI]), math.sin(x[PSI])
        vg = x[V] / (1.0 + x[SV])
        return (vg * c, vg * s, x[W], x[A], 0.0, x[AL], 0.0, vg * math.sin(x[TH]), 0.0, 0.0)

    def predict(self, x, P, dt):
        c = self.c
        n = self.n
        x = x.copy()
        v, a, w, al, psi, th = x[V], x[A], x[W], x[AL], x[PSI], x[TH]
        dt2 = dt * dt
        g = 1.0 / (1.0 + x[SV])            # odometric -> ground
        ds = (v * dt + 0.5 * a * dt2) * g
        pm = psi + 0.5 * w * dt + al * dt2 / 6.0
        cm, sm = math.cos(pm), math.sin(pm)
        sth, cth = math.sin(th), math.cos(th)
        x[PE] += ds * cm
        x[PN] += ds * sm
        x[PSI] = psi + w * dt + 0.5 * al * dt2
        x[V] = v + a * dt
        x[W] = w + al * dt
        x[Z] += ds * sth
        # the transition matrix is the identity but for the kinematic block (the calibration states are random
        # walks or Gauss-Markov: a diagonal). Built as a list, the covariance propagated block-wise: this is
        # the filter's inner loop.
        dsm, dcm, h = -ds * sm, ds * cm, 0.5 * dt
        Fc = np.array((
            (1.0, 0.0, dsm, dt * cm * g, 0.5 * dt2 * cm * g, dsm * h, dsm * dt2 / 6.0, 0.0, 0.0, -ds * g * cm),
            (0.0, 1.0, dcm, dt * sm * g, 0.5 * dt2 * sm * g, dcm * h, dcm * dt2 / 6.0, 0.0, 0.0, -ds * g * sm),
            (0.0, 0.0, 1.0, 0.0, 0.0, dt, 0.5 * dt2, 0.0, 0.0, 0.0),
            (0.0, 0.0, 0.0, 1.0, dt, 0.0, 0.0, 0.0, 0.0, 0.0),
            (0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0),
            (0.0, 0.0, 0.0, 0.0, 0.0, 1.0, dt, 0.0, 0.0, 0.0),
            (0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0),
            (0.0, 0.0, 0.0, dt * sth * g, 0.5 * dt2 * sth * g, 0.0, 0.0, 1.0, ds * cth, -ds * g * sth),
            (0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0),
            (0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0)))
        rho = None
        if self._gm:
            rho = self._rho_cache.get(dt)
            if rho is None:
                if len(self._rho_cache) > 256:
                    self._rho_cache.clear()
                r = np.ones(n)
                q = np.zeros(n)
                for i, tau, sig in self._gm:
                    r[i] = math.exp(-dt / tau)
                    q[i] = sig * sig * (1.0 - math.exp(-2.0 * dt / tau))
                rho = self._rho_cache[dt] = (r, q)
            x *= rho[0]
        P = P.copy()
        P[:NCORE, :] = Fc @ P[:NCORE, :]
        P[:, :NCORE] = P[:, :NCORE] @ Fc.T
        if rho is not None:
            P *= rho[0][:, None]
            P *= rho[0][None, :]
        F = (Fc, None if rho is None else rho[0])
        # process noise (white jerk and yaw jerk integrated over dt; position noise along the heading)
        dt3, dt4, dt5 = dt2 * dt, dt2 * dt2, dt2 * dt2 * dt
        qj, qy = c["jerk_psd"], c["yaw_jerk_psd"]
        q55, q44, q33 = qj * dt5 / 20.0, qj * dt4 / 8.0, qj * dt3 / 6.0
        pp = c["pos_psd"] * dt
        Qk = np.array((
            (q55 * cm * cm + pp, q55 * cm * sm, 0.0, q44 * cm, q33 * cm, 0.0, 0.0),
            (q55 * cm * sm, q55 * sm * sm + pp, 0.0, q44 * sm, q33 * sm, 0.0, 0.0),
            (0.0, 0.0, qy * dt5 / 20.0, 0.0, 0.0, qy * dt4 / 8.0, qy * dt3 / 6.0),
            (q44 * cm, q44 * sm, 0.0, qj * dt3 / 3.0, qj * dt2 / 2.0, 0.0, 0.0),
            (q33 * cm, q33 * sm, 0.0, qj * dt2 / 2.0, qj * dt, 0.0, 0.0),
            (0.0, 0.0, qy * dt4 / 8.0, 0.0, 0.0, qy * dt3 / 3.0, qy * dt2 / 2.0),
            (0.0, 0.0, qy * dt3 / 6.0, 0.0, 0.0, qy * dt2 / 2.0, qy * dt)))
        P[:7, :7] += Qk
        P[Z, Z] += c["alt_psd"] * dt
        P[TH, TH] += c["grade_psd"] * dt
        P[SV, SV] += c["scale_psd"] * dt
        if n > NCORE:
            d = self._diag
            if d is None or len(d) != n - NCORE:
                d = self._diag = np.arange(NCORE, n)
            P[d, d] += self._psd * dt
            if rho is not None:
                P[d, d] += rho[1][NCORE:]
        if dt > 0.5:
            # a gap without data: the white-noise model would let the uncertainty of bounded quantities grow
            # without limit; a car's speed, acceleration and yaw rate are bounded, a heading is at most unknown
            for i, lim in _CAPS:
                if P[i, i] > lim:
                    k = math.sqrt(lim / P[i, i])
                    P[i, :] *= k
                    P[:, i] *= k
        for i, lo, hi in self._clamp:
            if lo is not None and x[i] < lo:
                x[i] = lo
            elif hi is not None and x[i] > hi:
                x[i] = hi
        return x, P, F


# ---- geodesy (local tangent plane; fine over a drive) -------------------------------------------------
R_EARTH = 6378137.0


def to_local(origin, lat, lon):
    lat0, lon0 = origin
    return (math.radians(lon - lon0) * R_EARTH * math.cos(math.radians(lat0)), math.radians(lat - lat0) * R_EARTH)


def to_geo(origin, e, n):
    lat0, lon0 = origin
    return (lat0 + math.degrees(n / R_EARTH), lon0 + math.degrees(e / (R_EARTH * math.cos(math.radians(lat0)))))


def course_to_psi(course_deg):
    return math.radians(90.0 - course_deg)


def psi_to_course(psi):
    return (90.0 - math.degrees(psi)) % 360.0
