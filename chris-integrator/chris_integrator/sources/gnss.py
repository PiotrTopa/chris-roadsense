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

    def on_reanchor(self, ctx):
        """The position was declared lost and is anchored again on this receiver: whether the old position or
        the new fixes were wrong is not known, so the fixes are believed at their widest for a while."""
        ctx.mem["scale2"] = self.scale_max ** 2
        ctx.mem["scale_k"] = 0

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


class GnssTrackSource(Source):
    """The receiver's track as a speed: path length over a few fixes divided by their time. Slow (one value
    every few seconds, seconds old) and noisy, but independent of every speed sensor of the car and with no
    scale of its own: the third opinion that decides between the vehicle's speed and the camera's when they
    disagree (a vote), and a check on both when they agree. Never fused."""
    id = "gnss_track"
    group = "gnss_track"
    rank = 0
    family = "gnss"
    types = ("gnss",)
    stale_s = 1e9                # it speaks only while the car moves: its silence says nothing
    info = "receiver track speed (path over a few fixes): the independent third opinion on the speed"
    chord_n = 3                  # fixes per value
    min_speed, max_accel = 3.0, 1.5
    max_gap_s = 2.5
    # compared by its mean, over a long window (monitors.Pair mode "mean")
    pair = dict(mode="mean", max_step_s=8.0, pending_s=8.0, fire_window_s=60.0, recover_window_s=60.0, min_n=4,
                cusum_k=1.0, cusum_h=5.0, min_delta=0.06, recover_n=4, recover_z=2.5)

    def reset(self):
        pass

    def measure(self, msg, ctx):
        mem = ctx.mem
        lat, lon = msg.get("lat"), msg.get("lon")
        if msg.get("fix") not in FIXES or not fin(lat) or not fin(lon) or ctx.integ.origin is None:
            return []
        ts = msg["ts_mono_ns"] / NS
        e, n = E.to_local(ctx.integ.origin, float(lat), float(lon))
        pts = mem.setdefault("pts", [])
        if pts and (ts - pts[-1][0] > self.max_gap_s or ts <= pts[-1][0]):
            del pts[:]
        if pts and e == pts[-1][1] and n == pts[-1][2]:
            del pts[:]                                   # a repeated position: the receiver holds (standing, or no fix)
            return []
        pts.append((ts, e, n))
        if len(pts) <= self.chord_n:
            return []
        d = sum(math.hypot(pts[i + 1][1] - pts[i][1], pts[i + 1][2] - pts[i][2]) for i in range(len(pts) - 1))
        T = pts[-1][0] - pts[0][0]
        mid = 0.5 * (pts[-1][0] + pts[0][0])
        del pts[:-1]
        i = ctx.integ.model.index.get("gnss.tau")
        tau = float(ctx.x[i]) if i is not None else 0.0
        # the value belongs to the middle of the chord, minus the receiver's delay. While that delay is poorly
        # known and the car accelerates, the time the value belongs to is not known well enough to compare it
        # with anything: no evidence then, rather than evidence with a timing error in it
        s_tau = max(math.sqrt(max(float(ctx.P[i, i]), 0.0)) if i is not None else 0.5, 0.15)
        a0, a1 = ctx.ego_at(mid - tau - 0.5 * T), ctx.ego_at(mid - tau + 0.5 * T)
        if not (a0[3] and a1[3]) or abs(a1[0] - a0[0]) / T * s_tau > 0.35:
            return []
        # ... and a chord that contains a start, a stop or hard braking is not the speed at its middle
        if min(abs(a0[0]), abs(a1[0]), d / T) < self.min_speed or abs(a1[0] - a0[0]) / T > self.max_accel:
            return []
        hacc = msg.get("hacc_m")
        sig = 1.4 * min(max(float(hacc), 1.0) if fin(hacc) else 5.0, 8.0) / T
        return [Meas("speed", mid - tau, d / T, sig, src="gnss_track.speed", role="monitor")]
