"""The car's own motion (mapd's `ego` relay of the feeder's `motion`, CAN) and its signals (`vehicle`).

Noise measured on the reference car (docs/lanes/L35-ego-fusion-audit.md section 4, L37 section 3.5):
speed 0x0B4 robust sigma 0.012-0.020 m/s; gyro zeroed at standstill; rear wheel difference sigma 0.010-0.019
rad/s; longitudinal accelerometer sigma 0.20-0.25 m/s2 about a slow offset.
"""
import numpy as np

from ..core import ego as E
from ..core.ego import AuxState
from ..core.registry import NS, Meas, Source, fin

FEED_GAP_S, FEED_WARMUP_S = 1.0, 0.35  # a feed silent this long is not believed for this long after it resumes
YAW_SIGMA = {"yaw_rate": (0.004, 0.03), "wheel_speeds": (0.02, 0.05), "steering": (0.03, 0.10)}
TRACK_M = 1.48
_I_V, _ONE = np.array([E.V], dtype=np.intp), np.array([1.0])


def _H_V(x):
    return x[E.V], _I_V, _ONE


SCALE_MIN_YAW = 0.05             # rad/s: a scale factor is learned only where its regressor stands clear of the noise


def scale_reg(value, floor):
    """The Jacobian of a scale state is the quantity it multiplies. Near zero that quantity is mostly its own
    estimation noise, and a noisy regressor teaches the scale nonsense (the errors-in-variables bias of an EKF);
    below `floor` the row therefore says nothing about the scale."""
    return value if abs(value) >= floor else 0.0


def warming(t, mem):
    """After a silence of the feed its first samples repeat what it held before the silence (measured: 19-25
    silences of about 7 s per drive, the first 2-3 samples after each, up to 0.28 s, stale by up to 7 m/s):
    they are not measurements. True while the feed warms up."""
    last = mem.get("last_t")
    mem["last_t"] = t
    if last is not None and t - last > FEED_GAP_S:
        mem["warm_until"] = t + FEED_WARMUP_S
        mem["warmups"] = mem.get("warmups", 0) + 1
    return t < mem.get("warm_until", -1e18)


class VehicleSource(Source):
    id = "vehicle"
    group = "vehicle"
    rank = 3
    types = ("ego",)
    stale_s = 0.6
    info = "vehicle motion from the host's feeder (CAN), relayed by mapd"
    aux = (
        AuxState("b_g", 0.0, 0.01, 1e-9, unit="rad/s", doc="gyro bias"),
        AuxState("s_g", 0.0, 0.08, 1e-9, unit="1", doc="gyro = (1 + s_g) x yaw rate + b_g"),
        AuxState("s_wy", 0.0, 0.08, 1e-9, unit="1", doc="rear wheel difference / track = (1 + s_wy) x yaw rate"),
        AuxState("b_ax", 0.0, 0.5, 1e-5, unit="m/s2", doc="longitudinal accelerometer offset (with grade when no altitude source)"),
        AuxState("b_ay", 0.0, 1.0, 1e-2, unit="m/s2", doc="lateral accelerometer offset (bank and bias)"),
    )
    track_m = TRACK_M
    fuse_accel_lat = True

    def measure(self, msg, ctx):
        t = msg["ts_mono_ns"] / NS
        mem = ctx.mem
        out = []
        warm = warming(t, mem)
        mem["tag"] = msg.get("tag")
        if warm:
            return out
        sig = msg.get("turn_signal")
        if sig is not None:
            out.append(Meas("signal:turn_signal", t, sig, role="signal"))
        v = msg.get("speed_mps")
        if not fin(v):
            return out
        v = float(v)
        still = bool(msg.get("standstill")) and v == 0.0
        out.append(Meas("standstill", t, still, src="vehicle.standstill", role="flag"))
        ibg, isg, iswy, ibax, ibay = (self.ix[k] for k in ("b_g", "s_g", "s_wy", "b_ax", "b_ay"))
        if not still:
            # the speed state is in this signal's scale by definition (core/ego.py, SV)
            sg = 0.03 + 0.002 * abs(v)
            out.append(Meas("speed", t, v, sg, _H_V, "vehicle.speed", mon=(v, sg)))
            a = msg.get("accel_long_mps2")
            if fin(a):
                i_ax = np.array([E.A, E.TH, ibax], dtype=np.intp)

                def h_ax(x):
                    th = x[E.TH]
                    return x[E.A] + E.G * np.sin(th) + x[ibax], i_ax, np.array([1.0, E.G * np.cos(th), 1.0])
                out.append(Meas("accel", t, float(a), 0.25, h_ax, "vehicle.accel"))
            ay = msg.get("accel_lat_mps2")
            if fin(ay) and self.fuse_accel_lat:
                i_ay = np.array([E.V, E.W, ibay], dtype=np.intp)

                def h_ay(x):
                    return x[E.V] * x[E.W] + x[ibay], i_ay, np.array([x[E.W], x[E.V], 1.0])
                out.append(Meas("accel_lat", t, float(ay), 0.5, h_ay, "vehicle.accel_lat"))
        else:
            out.append(Meas("speed", t, 0.0, 0.03, src="vehicle.speed", role="monitor"))
        r = msg.get("yaw_rate_rps")
        ysrc = msg.get("yaw_source")
        if fin(r):
            r = float(r)
            s0, s1 = YAW_SIGMA.get(ysrc, YAW_SIGMA["steering"])
            sg = s0 + s1 * abs(r)
            if ysrc == "yaw_rate":
                i_g = np.array([E.W, isg, ibg], dtype=np.intp)

                def h_g(x):
                    k = 1.0 + x[isg]
                    return k * x[E.W] + x[ibg], i_g, np.array([k, scale_reg(x[E.W], SCALE_MIN_YAW), 1.0])
            else:
                i_w = np.array([E.W], dtype=np.intp)

                def h_g(x):
                    return x[E.W], i_w, np.array([1.0])
            out.append(Meas("yaw_rate", t, r, sg, h_g, "vehicle.yaw", mon=(r, sg)))
        ws = msg.get("wheel_speeds_mps")
        if ysrc == "yaw_rate" and not still and isinstance(ws, (list, tuple)) and len(ws) == 4 \
                and all(fin(w) for w in ws):
            rw = (ws[3] - ws[2]) / self.track_m
            if msg.get("reverse"):
                rw = -rw
            i_wy = np.array([E.W, iswy], dtype=np.intp)

            def h_wy(x):
                k = 1.0 + x[iswy]
                return k * x[E.W], i_wy, np.array([k, scale_reg(x[E.W], SCALE_MIN_YAW)])
            out.append(Meas("yaw_rate", t, rw, 0.02 + 0.05 * abs(rw), h_wy, "vehicle.wheel_yaw", robust=True))
        return out

    def report(self, ctx):
        return {"tag": ctx.mem.get("tag"), "warmups": ctx.mem.get("warmups", 0)}


class VehicleSignals(Source):
    id = "vehicle.signals"
    group = "vehicle"
    rank = 3
    types = ("vehicle",)
    stale_s = 0.6
    counts = False

    def measure(self, msg, ctx):
        t = msg["ts_mono_ns"] / NS
        b = (msg.get("controls") or {}).get("brake")
        return [Meas("signal:brake", t, b, role="signal")] if isinstance(b, bool) else []


class WheelsSource(Source):
    """Mean of the wheel speeds: a second reading of the speed with its own decoding. Family `vehicle`: it
    checks the speed signal's decoding, it is no independent vote (same sensors), and it is not fused."""
    id = "wheels"
    group = "wheels"
    rank = 2
    family = "vehicle"
    types = ("ego",)
    stale_s = 0.6
    info = "mean of the wheel speeds: a second reading of the speed"

    def measure(self, msg, ctx):
        t = msg["ts_mono_ns"] / NS
        if warming(t, ctx.mem):
            return []
        ws = msg.get("wheel_speeds_mps")
        if not isinstance(ws, (list, tuple)) or len(ws) < 2 or not all(fin(w) for w in ws):
            return []
        m = sum(ws) / len(ws)
        if msg.get("reverse"):
            m = -m
        # the scale of wheel speeds against the speed signal is a car's own (0.98 here): the tolerance covers it
        return [Meas("speed", t, m, 0.10 + 0.04 * abs(m), src="wheels.speed", role="monitor")]
