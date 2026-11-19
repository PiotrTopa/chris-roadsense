"""Receiver fixes (`gnss`, mapd's relay of the feeder's messages).

The reference car's head unit (docs/INTEGRATOR.md section 1): about one fix a second, stamped at arrival,
positions about 2 s old and consistent in scale with the odometer; a claimed `hacc` that is too optimistic;
a speed field that is a slower, smoothed output (1.5-9 s behind, low) and is therefore not fused: it is a
separate witness source. Positions are fused with an estimated delay and a noise scale adapted from the
innovations; nothing is fused while the car stands (a standing receiver wanders).
"""
import math

import numpy as np

from ..core import ego as E
from ..core.ego import AuxState
from ..core.registry import NS, Meas, Source, fin

FIXES = ("2d", "3d", "dgps", "rtk")


def _repeat(msg, mem):
    key = (msg.get("lat"), msg.get("lon"), msg.get("speed_mps"), msg.get("course_deg"))
    rep = key == mem.get("key")
    mem["key"] = key
    mem["n"] = mem.get("n", 0) + 1
    mem["rep"] = mem.get("rep", 0) + (1 if rep else 0)
    return rep


class GnssSource(Source):
    id = "gnss"
    group = "gnss"
    rank = 0
    types = ("gnss",)
    stale_s = 5.0
    info = "receiver positions (delay estimated, noise adapted)"
    aux = (AuxState("tau", 2.0, 1.5, 1e-5, lo=0.0, hi=9.0, unit="s", doc="a fix is this old at its stamp"),
           AuxState("be", 0.0, 2.5, tau_s=20.0, unit="m", doc="position error correlated over tens of seconds, east"),
           AuxState("bn", 0.0, 2.5, tau_s=20.0, unit="m", doc="... north"))
    straight_s = 8.0             # the course is a slow output too: used only this long after the last turn
    min_sigma_m = 3.0
    scale_max = 8.0
    scale_n = 20.0               # fixes the noise scale averages over
    course_sigma = math.radians(15.0)
    alt_sigma_m = 10.0

    def measure(self, msg, ctx):
        mem = ctx.mem
        rep = _repeat(msg, mem)
        mem["fix"], mem["hacc"], mem["receiver"] = msg.get("fix"), msg.get("hacc_m"), msg.get("source")
        lat, lon = msg.get("lat"), msg.get("lon")
        if rep or msg.get("fix") not in FIXES or not fin(lat) or not fin(lon):
            return []
        ts = msg["ts_mono_ns"] / NS
        sp = ctx.speed_known()
        if sp is None or abs(sp[0]) < 0.3:
            mem["skipped_standing"] = mem.get("skipped_standing", 0) + 1
            return self._alive(ts)
        hacc = msg.get("hacc_m")
        hacc = max(float(hacc), self.min_sigma_m) if fin(hacc) else 10.0
        scale = math.sqrt(min(max(mem.get("scale2", 1.0), 1.0), self.scale_max ** 2))
        itau = self.ix["tau"]

        def adapt(nu, s0, t, hacc=hacc):
            # the innovation of this fix against what `hacc` alone promises (innovation-based adaptation)
            r = nu * nu / (s0 + hacc * hacc)
            k = 1.0 / min(mem.get("scale_k", 0) + 1, self.scale_n)
            mem["scale_k"] = mem.get("scale_k", 0) + 1
            mem["scale2"] = mem.get("scale2", 1.0) + k * (min(r, 100.0) - mem.get("scale2", 1.0))
        course, gs = msg.get("course_deg"), msg.get("speed_mps")
        c = ctx.cfg
        if abs(float(ctx.x[E.W])) > 0.05 or sp[0] < 5.0:
            mem["turn_t"] = ts
        straight = ts - mem.get("turn_t", ts) >= self.straight_s
        course_ok = fin(course) and sp[0] >= c["anchor_min_speed"] and fin(gs) and gs >= 2.0
        x = ctx.x
        out = ctx.integ.abs_position(self.id, ts, float(lat), float(lon), hacc * scale, itau,
                                     float(course) if fin(course) else None, course_ok, cb=adapt,
                                     bias=(self.ix["be"], self.ix["bn"]))
        if not ctx.integ.anchored:
            return out or self._alive(ts)
        tau = float(x[itau])
        if course_ok and straight:
            i_h = np.array([E.PSI, itau], dtype=np.intp)

            def h_c(x, tau0=tau):
                return x[E.PSI] - x[E.W] * (x[itau] - tau0), i_h, np.array([1.0, -x[E.W]])
            # the innovation is wrapped: compare against the heading unwrapped to the state's branch
            zc = E.course_to_psi(float(course))
            zc = float(x[E.PSI]) + ((zc - float(x[E.PSI]) + math.pi) % (2.0 * math.pi) - math.pi)
            out.append(Meas("heading", ts - tau, zc, self.course_sigma, h_c, "gnss.course", robust=True,
                            local=not c["gnss_absolute"]))
        alt = msg.get("alt_m")
        if fin(alt):
            i_z = np.array([E.Z], dtype=np.intp)
            one = np.array([1.0])
            out.append(Meas("alt", ts - tau, float(alt), self.alt_sigma_m, lambda x: (x[E.Z], i_z, one),
                            "gnss.alt", robust=True))
        return out

    @staticmethod
    def _alive(ts):
        return [Meas("signal:gnss_fix", ts, True, role="signal")]

    def report(self, ctx):
        m = ctx.mem
        n = m.get("n", 0)
        return {"fix": m.get("fix"), "hacc_m": m.get("hacc"), "receiver": m.get("receiver"),
                "repeat_share": round(m.get("rep", 0) / n, 2) if n else None,
                "noise_scale": round(math.sqrt(min(max(m.get("scale2", 1.0), 1.0), self.scale_max ** 2)), 2),
                "anchored": ctx.integ.anchored}


class GnssSpeedSource(Source):
    """The receiver's speed field: a witness (reported and compared, never fused, no vote)."""
    id = "gnss_speed"
    group = "gnss_speed"
    rank = 0
    family = "gnss"
    types = ("gnss",)
    stale_s = 5.0
    witness = True
    info = "receiver speed field (a delayed, smoothed output: witness only)"

    def measure(self, msg, ctx):
        if _repeat(msg, ctx.mem) or msg.get("fix") not in FIXES:
            return []
        v = msg.get("speed_mps")
        if not fin(v):
            return []
        acc = msg.get("speed_acc_mps")
        return [Meas("speed", msg["ts_mono_ns"] / NS, float(v), max(float(acc), 0.5) if fin(acc) else 1.5,
                     src="gnss.speed", role="monitor")]
