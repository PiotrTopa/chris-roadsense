"""The road model's pose (`road.pose`): speed and yaw rate of the 200 ms image pair, the direction of travel
in the device frame (camera mounting), its own stated uncertainty, a standstill detector for cars without a
vehicle feed.

Characterised on the reference car: speed sigma 0.25 + 0.035 v with heavy tails and an error correlated over
seconds; valid 0.10-0.15 s before the frame stamp; direction of travel robust spread 0.003-0.005 per frame at
speed on straights (docs/INTEGRATOR.md section 1).
"""
import math

import numpy as np

from ..core import ego as E
from ..core.ego import AuxState
from ..core.registry import NS, Meas, Source, fin
from .vehicle import SCALE_MIN_YAW, scale_reg

PAIR_LAG_S = 0.125               # prior of the delay: the pose describes the image pair
STOP_SPEED, GO_SPEED, STOP_CONFIRM_S = 0.15, 0.8, 1.0
MOUNT_MIN_SPEED, MOUNT_MAX_YAW = 5.0, 0.10
GM_SIGMA, GM_TAU_S = 0.30, 3.0   # the speed error's correlated part
WHITE_SHARE = 0.75               # ... and the share of the characterised sigma that is white per frame


def speed_sigma(v):
    """Characterised 1-sigma of the road model's speed against the vehicle's (two healthy drives)."""
    return 0.25 + 0.035 * abs(v)


class VisionSource(Source):
    id = "vision"
    group = "vision"
    rank = 1
    types = ("road", "calib")
    stale_s = 0.6
    info = "the road model's pose (200 ms image pair)"
    aux = (
        AuxState("tau", PAIR_LAG_S, 0.04, 1e-9, lo=0.0, hi=0.4, unit="s", doc="the pose is this old at the frame stamp"),
        AuxState("s", 0.0, 0.05, 1e-8, unit="1", doc="speed = (1 + s) x true speed + m (a monocular scale)"),
        AuxState("m", 0.0, GM_SIGMA, tau_s=GM_TAU_S, unit="m/s", doc="speed error correlated over seconds"),
        AuxState("s_w", 0.0, 0.10, 1e-8, unit="1", doc="yaw rate = (1 + s_w) x true yaw rate (measured 0.91-0.96 in turns)"),
        AuxState("dyaw", 0.0, 0.15, 1e-9, unit="rad", doc="direction of travel in the device frame, yaw (vy / vx): the mount against the calibration in use"),
        AuxState("dpitch", 0.0, 0.15, 1e-9, unit="rad", doc="... pitch (-vz / vx)"),
    )
    lever_m = 1.5                # camera ahead of the rear axle

    def reset(self):
        pass

    def measure(self, msg, ctx):
        mem = ctx.mem
        if msg.get("type") == "calib":
            key = (msg.get("pitch"), msg.get("yaw"))
            if mem.get("calib") is not None and key != mem["calib"]:
                # another calibration is in use: the residual angles start again from their prior
                for a in self.aux[4:]:
                    ctx.integ.reset_aux(self.ix[a.name], a.x0, a.sigma0)
                mem["mount_n"] = 0
            mem["calib"] = key
            return []
        out = []
        ts = msg["ts_mono_ns"] / NS
        pose = msg.get("pose") or {}
        v = pose.get("vx")
        if not fin(v):
            v = msg.get("v_ego")
        mem["road_model"] = msg.get("road_model")
        if msg.get("valid") is False or not fin(v):
            return out
        v = float(v)
        itau, isc, im, idy, idp, isw = (self.ix[k] for k in ("tau", "s", "m", "dyaw", "dpitch", "s_w"))
        x = ctx.x
        tau = float(x[itau])
        t = ts - tau
        char = speed_sigma(v)
        vs = pose.get("vx_std")
        white = WHITE_SHARE * char
        if fin(vs):
            white = max(white, float(vs))
        i_sp = np.array([E.V, isc, im, itau], dtype=np.intp)

        def h_speed(x, tau0=tau):
            k = 1.0 + x[isc]
            d = x[itau] - tau0
            return k * (x[E.V] - x[E.A] * d) + x[im], i_sp, np.array([k, scale_reg(x[E.V], 3.0), 1.0, -k * x[E.A]])
        # without a speed reference the scale cannot be told from the speed: it then stays at its prior and its
        # uncertainty is the speed's (a "consider" state), instead of drifting on its own linearisation
        sp0 = ctx.speed_known()
        ref = sp0 is not None and sp0[1] < 0.1
        out.append(Meas("speed", t, v, white, h_speed, "vision.pose", robust=True, mon=(v, char), local=True,
                        consider=None if ref else (isc,)))
        if fin(vs) and abs(v) >= 1.0:
            out.append(Meas("self:speed", ts, float(vs) / char, role="self"))
        r = pose.get("yaw_rate")
        if fin(r):
            r = -float(r)                                # the device frame is z down: positive right
            rs = pose.get("yaw_rate_std")
            sig = 0.007 + 0.25 * abs(r)
            sig_f = 0.007 + 0.05 * abs(r)                # as fused: the gain error is a state, not noise
            # (the delay is estimated from the speed rows only: the yaw acceleration the delay's Jacobian would
            # need here is itself too poorly known, and a noisy regressor biases what it teaches)
            i_w = np.array([E.W, isw], dtype=np.intp)

            def h_w(x):
                k = 1.0 + x[isw]
                return k * x[E.W], i_w, np.array([k, scale_reg(x[E.W], SCALE_MIN_YAW)])
            ref_w = ctx.P[E.W, E.W] < 0.01 ** 2          # a gyro (or wheels) holds the yaw rate
            out.append(Meas("yaw_rate", t, r, max(sig_f, float(rs)) if fin(rs) else sig_f, h_w, "vision.pose",
                            robust=True, mon=(r, sig), local=True, consider=None if ref_w else (isw,)))
        # camera mounting: the direction of travel in the device frame, judged at the integrator's speed
        sp = ctx.speed_known()
        vy, vz = pose.get("vy"), pose.get("vz")
        if sp is not None and sp[0] >= MOUNT_MIN_SPEED and abs(float(x[E.W])) < MOUNT_MAX_YAW and fin(vy) and fin(vz) \
                and not ctx.faulted() and v > 0.5 * sp[0]:
            L = self.lever_m
            i_y = np.array([E.V, idy, E.W], dtype=np.intp)
            i_z = np.array([E.V, idp], dtype=np.intp)

            def h_y(x):
                return x[E.V] * x[idy] - x[E.W] * L, i_y, np.array([x[idy], x[E.V], -L])

            def h_z(x):
                return -x[E.V] * x[idp], i_z, np.array([-x[idp], -x[E.V]])
            sy, sz = pose.get("vy_std"), pose.get("vz_std")
            fl = 0.02 + 0.006 * sp[0]
            out.append(Meas("mount", t, float(vy), max(fl, float(sy)) if fin(sy) else fl, h_y, "vision.mount", robust=True, local=True))
            out.append(Meas("mount", t, float(vz), max(fl, float(sz)) if fin(sz) else fl, h_z, "vision.mount", robust=True, local=True))
            mem["mount_n"] = mem.get("mount_n", 0) + 1
        # standstill for cars without a vehicle feed: the model's speed under the threshold for the confirmation
        # time. Leaky, so a single-frame blip (a wiper pass) does not end it; a real start does within 0.3 s.
        last = mem.get("t")
        dt = min(max(ts - last, 0.0), 1.0) if last is not None else 0.0
        mem["t"] = ts
        acc = mem.get("stop_s", 0.0)
        if v >= GO_SPEED:
            acc = 0.0
        elif v < STOP_SPEED:
            acc = min(acc + dt, STOP_CONFIRM_S + 0.3)
        else:
            acc = max(0.0, acc - 3.0 * dt)
        mem["stop_s"] = acc
        # soft: the camera infers the standstill (speed within 0.05 m/s, yaw rate within 0.005 rad/s)
        # (only while no better source holds the speed: a creeping car's wheels know it moves)
        alone = sp is None or sp[1] > 0.04 or abs(sp[0]) < 0.02
        out.append(Meas("standstill", ts, acc >= STOP_CONFIRM_S, sigma=0.05, src="vision.standstill" if alone else None,
                        role="flag"))
        return out

    def report(self, ctx):
        return {"road_model": ctx.mem.get("road_model")}

    def calib_state(self, ctx):
        """The mount as the estimator sees it: residual angles against the calibration in use, their sum with it
        (the calibration this camera should run with), and a state word."""
        cal = ctx.mem.get("calib")
        dy, dp = ctx.val("dyaw"), ctx.val("dpitch")
        sy, sp = ctx.std("dyaw"), ctx.std("dpitch")
        nm = ctx.integ.nis.get((self.group, "mount"))
        if ctx.faulted():
            state = "unknown"
        elif nm is not None and nm.high:
            state = "moved"
        elif max(sy, sp) > 0.005 or ctx.mem.get("mount_n", 0) < 100:
            state = "converging"
        else:
            state = "ok" if max(abs(dy), abs(dp)) < 0.02 else "offset"
        out = {"state": state, "samples": ctx.mem.get("mount_n", 0)}
        if cal is not None and fin(cal[0]) and fin(cal[1]):
            out["in_use"] = {"pitch": cal[0], "yaw": cal[1]}
            out["suggested"] = {"pitch": round(cal[0] + dp, 4), "yaw": round(cal[1] + dy, 4)}
        return out
