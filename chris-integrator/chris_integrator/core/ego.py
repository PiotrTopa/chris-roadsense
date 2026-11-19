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

    def add_aux(self, owner, aux):
        name = "%s.%s" % (owner, aux.name)
        if name in self.index:
            return self.index[name]
        i = self.n
        self.index[name] = i
        self.aux.append((name, aux))
        self.n += 1
        self._psd = np.append(self._psd, 0.0 if aux.tau_s else aux.psd)
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
        F = np.eye(n)
        F[PE, PSI] = -ds * sm
        F[PE, V] = dt * cm * g
        F[PE, A] = 0.5 * dt2 * cm * g
        F[PE, SV] = -ds * g * cm
        F[PN, SV] = -ds * g * sm
        F[Z, SV] = -ds * g * sth
        F[PE, W] = -ds * sm * 0.5 * dt
        F[PE, AL] = -ds * sm * dt2 / 6.0
        F[PN, PSI] = ds * cm
        F[PN, V] = dt * sm * g
        F[PN, A] = 0.5 * dt2 * sm * g
        F[PN, W] = ds * cm * 0.5 * dt
        F[PN, AL] = ds * cm * dt2 / 6.0
        F[PSI, W] = dt
        F[PSI, AL] = 0.5 * dt2
        F[V, A] = dt
        F[W, AL] = dt
        F[Z, V] = dt * sth * g
        F[Z, A] = 0.5 * dt2 * sth * g
        F[Z, TH] = ds * cth
        for i, tau, sig in self._gm:
            rho = math.exp(-dt / tau)
            x[i] *= rho
            F[i, i] = rho
        P = F @ P @ F.T
        # process noise
        dt3, dt4, dt5 = dt2 * dt, dt2 * dt2, dt2 * dt2 * dt
        qj, qy = c["jerk_psd"], c["yaw_jerk_psd"]
        P[V, V] += qj * dt3 / 3.0
        P[V, A] += qj * dt2 / 2.0
        P[A, V] += qj * dt2 / 2.0
        P[A, A] += qj * dt
        u = (cm, sm)
        for i, ui in ((PE, cm), (PN, sm)):
            for j, uj in ((PE, cm), (PN, sm)):
                P[i, j] += qj * dt5 / 20.0 * ui * uj
            P[i, V] += qj * dt4 / 8.0 * ui
            P[V, i] += qj * dt4 / 8.0 * ui
            P[i, A] += qj * dt3 / 6.0 * ui
            P[A, i] += qj * dt3 / 6.0 * ui
            P[i, i] += c["pos_psd"] * dt
        P[PSI, PSI] += qy * dt5 / 20.0
        P[PSI, W] += qy * dt4 / 8.0
        P[W, PSI] += qy * dt4 / 8.0
        P[PSI, AL] += qy * dt3 / 6.0
        P[AL, PSI] += qy * dt3 / 6.0
        P[W, W] += qy * dt3 / 3.0
        P[W, AL] += qy * dt2 / 2.0
        P[AL, W] += qy * dt2 / 2.0
        P[AL, AL] += qy * dt
        P[Z, Z] += c["alt_psd"] * dt
        P[TH, TH] += c["grade_psd"] * dt
        P[SV, SV] += c["scale_psd"] * dt
        if n > NCORE:
            d = np.arange(NCORE, n)
            P[d, d] += self._psd * dt
            for i, tau, sig in self._gm:
                P[i, i] += sig * sig * (1.0 - math.exp(-2.0 * dt / tau))
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
