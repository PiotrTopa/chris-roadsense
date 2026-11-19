"""What a source is (docs/INTEGRATOR.md section 7): an input description, a measurement model, and its mount
and calibration states. The core knows nothing else about any sensor."""
import math

import numpy as np

NS = 1e9


def fin(v):
    return isinstance(v, (int, float)) and not isinstance(v, bool) and math.isfinite(v)


class Mount:
    """Where a sensor sits, in the car frame (x forward, y left, origin the reference camera), and where its
    boresight points (yaw, rad, positive left)."""
    __slots__ = ("x", "y", "z", "yaw")

    def __init__(self, x=0.0, y=0.0, z=0.0, yaw=0.0):
        self.x, self.y, self.z, self.yaw = float(x), float(y), float(z), float(yaw)

    def as_dict(self):
        return {"x_m": self.x, "y_m": self.y, "z_m": self.z, "yaw_deg": round(math.degrees(self.yaw), 3)}


class Meas:
    """One scalar ego measurement, as a measurement model returns it.

    q       quantity tag. Rows of one tag share anchoring and NIS accounting; "speed" and "yaw_rate" (and any tag
            with `mon`) are also compared raw against the other sources' by the pair monitors.
    t       time of validity, s on the device's monotonic clock (the source subtracts its estimated delay)
    z, sigma    value and 1-sigma noise
    h       h(x) -> (prediction, indices, coefficients): the row's model and sparse Jacobian (role "fuse")
    role    "fuse" | "monitor" (not fused; monitors only) | "self" (z = stated / characterised sigma) |
            "flag" (q "standstill", z bool) | "signal" (q "signal:<name>", pass-through)
    robust  heavy-tailed source: Student-t weighting and a hard gate while an exact source anchors `q`
    wrap    the innovation is an angle
    mon     (value, characterised sigma) the monitors judge the source by; None: z and sigma
    cb      cb(nu, s0, t): called once with the row's first innovation (adaptive noise)
    local   the row corrects the kinematic state and its own source's calibration states only; other sources'
            calibration states are "consider" states for it (their uncertainty counts, they are not corrected).
            For a source without an absolute scale of its own (a monocular camera's speed must not teach the
            wheel-speed scale).
    consider  indices of further states this row must not correct (their uncertainty still counts): a state
            that this row alone cannot tell from another one (a camera's scale without a speed reference)
    """
    __slots__ = ("q", "t", "z", "sigma", "h", "src", "role", "robust", "wrap", "mon", "cb", "local", "consider")

    def __init__(self, q, t, z, sigma=None, h=None, src=None, role="fuse", robust=False, wrap=False, mon=None,
                 cb=None, local=False, consider=None):
        self.q, self.t, self.z, self.sigma, self.h, self.src, self.role = q, t, z, sigma, h, src, role
        self.robust, self.wrap, self.mon, self.cb, self.local, self.consider = robust, wrap, mon, cb, local, consider


def lin(idx, coef, const=0.0):
    """A linear row: prediction = const + sum(coef * x[idx])."""
    idx = np.array(idx, dtype=np.intp)
    coef = np.array(coef, dtype=float)

    def h(x):
        return const + float(coef @ x[idx]), idx, coef
    return h


class Source:
    """Base class of a source. Subclass, set the description, override `measure` and / or `scan`."""
    id = None                # unique name; the owner of its calibration states
    group = None             # health / isolation unit (several sources may form one group)
    rank = 0                 # isolation by rank: the higher rank is believed when no vote decides
    family = None            # groups of one family share physical sensors: one vote between them
    types = ()               # message types it reads
    stale_s = 1.0
    witness = False          # never fused, never starts a fault
    counts = True            # False: a side channel (its messages do not show the group alive)
    info = None
    aux = ()                 # AuxState list: calibration states the estimator carries for it
    mount = None

    def __init__(self, **kw):
        for k, v in kw.items():
            setattr(self, k, v)
        if self.group is None:
            self.group = self.id
        self.ix = {}             # aux name -> state index (set at registration)

    def measure(self, msg, ctx):
        """Ego measurement rows of a message (list of Meas)."""
        return []

    def scan(self, msg, ctx):
        """Object detections of a message (objects.Scan) or None."""
        return None

    def report(self, ctx):
        """Extra fields for the group's health entry."""
        return {}

    def calib_state(self, ctx):
        """Extra fields for `calibration.<id>` (e.g. a state word)."""
        return {}

    def reset(self):
        """A new epoch: forget what is tied to the old time axis."""


class Ctx:
    """What a measurement model may read: the current estimate (never raw data of other sources)."""

    def __init__(self, integ, src):
        self.integ, self.src = integ, src
        self.mem = {}

    @property
    def x(self):
        return self.integ.x_now

    @property
    def P(self):
        return self.integ.P_now

    @property
    def now(self):
        return self.integ.now

    def ix(self, name):
        """Index of a core state ("v") or of one of this source's calibration states ("tau")."""
        i = self.src.ix.get(name)
        return self.integ.model.index[name] if i is None else i

    def val(self, name):
        return float(self.integ.x_now[self.ix(name)])

    def std(self, name):
        i = self.ix(name)
        return math.sqrt(max(float(self.integ.P_now[i, i]), 0.0))

    def rate(self):
        return self.integ.model.rate(self.integ.x_now)

    def faulted(self, group=None):
        g = self.integ.groups.get(group or self.src.group)
        return g is not None and g.fault is not None

    def speed_known(self):
        """(speed, std) of the integrator's own estimate, or None while it has no speed measurement."""
        return self.integ.speed_now()

    def ego_at(self, t):
        return self.integ.ego_at(t)

    @property
    def cfg(self):
        return self.integ.cfg
