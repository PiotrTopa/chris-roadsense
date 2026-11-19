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
from .vehicle import SCALE_MIN_SPEED, SCALE_MIN_YAW, clear_reg

PAIR_LAG_S = 0.125               # prior of the delay: the pose describes the image pair
STOP_SPEED, GO_SPEED, STOP_CONFIRM_S = 0.15, 0.8, 1.0
MOUNT_MIN_SPEED, MOUNT_MAX_YAW = 5.0, 0.10
PITCH_CUSUM_K, PITCH_CUSUM_H = 1.5, 14.0   # in sigma of the pitch row; a step of 1 degree at 10 m/s fires in about half a second
# the pose's motion at a standstill another source knows (m/s, rad/s, frames within 2 s). Set on ONE parked session of the
# reference car (54 min: rates below 0.0011 rad/s in 99.9 % of frames; a re-aiming by hand reached 0.038)
HANDLED_SPEED, HANDLED_RATE, HANDLED_FRAMES = 0.5, 0.01, 4
PITCH_STEP_MIN = 0.006           # rad: smaller "steps" are the estimator's ordinary business
# The speed error's structure, fitted to its autocorrelation on the reference car's two healthy drives (12 000
# frames; correlation 0.80 at one frame, 0.54 at 0.5 s, 0.42 at 1 s, 0.21 at 3 s, 0.13 at 5 s): hardly any of it
# is white. Shares of the characterised variance: a slow part 0.52 (3.5 s), a fast part 0.37 (0.35 s), white
# 0.11. Both coloured parts are Gauss-Markov states in m/s with the sigma they have at town speeds; what the
# characterised sigma has beyond them at higher speed is white. (States scaled by the speed-dependent sigma were
# tried: such a state is a second scale factor, and the filter then split the camera's scale between the two
# and got it wrong by 8 %.)
SLOW_SIGMA, SLOW_TAU_S = 0.45, 3.5
FAST_SIGMA, FAST_TAU_S = 0.38, 0.35
WHITE_SHARE = 0.33               # sqrt(0.11): at least this share of the sigma is white per frame
# the error is heavy-tailed (rms 1.3-1.5 x its robust sigma, p95 at 2.5 robust sigmas): the monitors judge by
# the robust sigma (`speed_sigma`), the estimate's uncertainty has to cover 95 % of the errors within 2 sigma
TAIL = 1.35


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
        AuxState("m", 0.0, SLOW_SIGMA, tau_s=SLOW_TAU_S, unit="m/s", doc="speed error correlated over seconds"),
        AuxState("mf", 0.0, FAST_SIGMA, tau_s=FAST_TAU_S, unit="m/s", doc="speed error correlated over a few frames"),
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
            old = mem.get("calib")
            if old is not None and key != old:
                # another calibration is in use. The mount itself has not moved: the residual against the new
                # calibration is the old residual plus what the calibration changed by (a little less certain:
                # the model's response to its warp is not exactly linear). Without numbers: back to the prior.
                for name, k in (("dpitch", 0), ("dyaw", 1)):
                    i = self.ix[name]
                    a = next(a for a in self.aux if a.name == name)
                    if fin(old[k]) and fin(key[k]):
                        ctx.integ.reset_aux(i, ctx.val(name) + old[k] - key[k], math.hypot(ctx.std(name), 0.005))
                    else:
                        ctx.integ.reset_aux(i, a.x0, a.sigma0)
                mem["cusum"] = None
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
        if fin(vs) and float(vs) > 2.0 * char:
            white = float(vs)                            # the model itself says it is lost: believed
        imf = self.ix["mf"]
        fz = TAIL * char                                 # the sigma that covers the error's tails (as fused)
        white = max(white, WHITE_SHARE * fz, math.sqrt(max(fz * fz - SLOW_SIGMA ** 2 - FAST_SIGMA ** 2, 0.0)))
        cs = cf = 1.0
        i_sp = np.array([E.V, isc, im, imf, itau], dtype=np.intp)

        vv, vw = float(ctx.P[E.V, E.V]), float(ctx.P[E.W, E.W])

        def h_speed(x, tau0=tau):
            k = 1.0 + x[isc]
            d = x[itau] - tau0
            return k * (x[E.V] - x[E.A] * d) + cs * x[im] + cf * x[imf], i_sp, \
                np.array([k, clear_reg(x[E.V], vv, SCALE_MIN_SPEED), cs, cf, -k * x[E.A]])
        # without a speed reference the scale cannot be told from the speed: it then stays at its prior and its
        # uncertainty is the speed's (a "consider" state), instead of drifting on its own linearisation
        sp0 = ctx.speed_known()
        # (a reference is another source holding the speed of a moving car: the camera's own inferred standstill
        # makes the speed certain too, and taught the scale nonsense at every pull-away before L41)
        ref = sp0 is not None and sp0[1] < 0.1 and abs(sp0[0]) > 1.0
        out.append(Meas("speed", t, v, white, h_speed, "vision.pose", robust=True, mon=(v, char), local=True,
                        consider=None if ref else (isc,)))
        if fin(vs) and abs(v) >= 1.0:
            out.append(Meas("self:speed", ts, float(vs) / char, role="self"))
        r = pose.get("yaw_rate")
        if fin(r):
            r = -float(r)                                # the device frame is z down: positive right
            rs = pose.get("yaw_rate_std")
            sig = 0.007 + 0.25 * abs(r)
            sig_f = 0.009 + 0.08 * abs(r)                # as fused: the gain error is a state, not noise
            # (the delay is estimated from the speed rows only: the yaw acceleration the delay's Jacobian would
            # need here is itself too poorly known, and a noisy regressor biases what it teaches)
            i_w = np.array([E.W, isw], dtype=np.intp)

            def h_w(x):
                k = 1.0 + x[isw]
                return k * x[E.W], i_w, np.array([k, clear_reg(x[E.W], vw, SCALE_MIN_YAW)])
            ref_w = ctx.P[E.W, E.W] < 0.01 ** 2          # a gyro (or wheels) holds the yaw rate
            out.append(Meas("yaw_rate", t, r, max(sig_f, float(rs)) if fin(rs) else sig_f, h_w, "vision.pose",
                            robust=True, mon=(r, sig), local=True, consider=None if ref_w else (isw,)))
        # camera mounting: the direction of travel in the device frame, judged at the integrator's speed
        sp = ctx.speed_known()
        vy, vz = pose.get("vy"), pose.get("vz")
        if sp is not None and sp[0] >= MOUNT_MIN_SPEED and abs(float(x[E.W])) < MOUNT_MAX_YAW and fin(vy) and fin(vz) \
                and not ctx.faulted() and v > 0.5 * sp[0]:
            L = self.lever_m
            # The rows are about the *direction* of the pose: its lateral and vertical velocity against its own
            # forward velocity (the same pose, the same monocular scale). The regressor is therefore the measured
            # vx, not the speed state: with the state there, these rows were a speed measurement in disguise
            # (vy = v x yaw offset), which let a camera without any reference "learn" its own scale.
            i_y = np.array([idy, E.W], dtype=np.intp)
            i_z = np.array([idp], dtype=np.intp)
            c_y, c_z = np.array([v, -L]), np.array([-v])

            def h_y(x):
                return v * x[idy] - x[E.W] * L, i_y, c_y

            def h_z(x):
                return -v * x[idp], i_z, c_z
            sy, sz = pose.get("vy_std"), pose.get("vz_std")
            fl = 0.02 + 0.006 * sp[0]
            sgz = max(fl, float(sz)) if fin(sz) else fl
            # the mount has one degree of freedom (pitch); yaw is fixed: two row classes, judged separately
            out.append(Meas("mount_yaw", t, float(vy), max(fl, float(sy)) if fin(sy) else fl, h_y, "vision.mount", robust=True, local=True))
            out.append(Meas("mount_pitch", t, float(vz), sgz, h_z, "vision.mount", robust=True, local=True))
            mem["mount_n"] = mem.get("mount_n", 0) + 1
            self._pitch_step(ctx, mem, ts, float(vz), sgz, v, idp)
        if ctx.faulted():
            # the road model far outside its range (measured: a camera tilted by 16 degrees) returns a collapsed
            # speed with an exploded stated std, a pose direction that reads "aligned", a phantom near lead. The
            # direction is then no measurement of the mount: the mount is unknown, and a large pitch change is
            # one of the things this looks like
            vsr = float(vs) / char if fin(vs) else None
            if sp is not None and sp[0] >= MOUNT_MIN_SPEED and (v < 0.5 * sp[0] or (vsr is not None and vsr > 4.0)):
                mem["out_of_range_t"] = ts
                ctx.integ.fdi._set("vision", "mount", "model_out_of_range", "warn", ts,
                                   "road model out of its range (speed %.2f of the estimate%s): a large change of the camera's "
                                   "pitch or a blocked view; the pose direction is not a mount measurement now, the mount is unknown"
                                   % (v / sp[0], ", stated std %.0fx normal" % vsr if vsr is not None else ""))
        elif ts - mem.get("out_of_range_t", -1e18) > 5.0:
            ctx.integ.fdi.findings.pop(("vision", "mount", "model_out_of_range"), None)
        # the camera moves while the car stands (another source knows the standstill exactly): somebody is handling
        # it. Whatever the mount was, it is not known any more; its pitch is free again until the next drive.
        st = ctx.integ.still.get("vehicle")
        if st and st[-1][1] is not None and ts - st[-1][0] < 0.5 and ts - st[-1][1] > 2.0:
            pr = pose.get("pitch_rate")
            moving = abs(v) > HANDLED_SPEED or (fin(r) and abs(r) > HANDLED_RATE) or (fin(pr) and abs(pr) > HANDLED_RATE)
            hq = mem.setdefault("handled", [])
            hq.append((ts, moving))
            while hq and hq[0][0] < ts - 2.0:
                hq.pop(0)
            if sum(1 for _, b in hq if b) >= HANDLED_FRAMES and ts - mem.get("handled_t", -1e18) > 10.0:
                mem["handled_t"] = ts
                mem["handled_n"] = mem.get("handled_n", 0) + 1
                a = next(a for a in self.aux if a.name == "dpitch")
                ctx.integ.reset_aux(idp, ctx.val("dpitch"), a.sigma0)
                mem["mount_n"] = 0
                ctx.integ.fdi.event("vision", "camera_handled", None, "the camera moved while the car stood (handled or re-aimed): its "
                                    "pitch is unknown until the next drive", quantity="mount")
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

    def _pitch_step(self, ctx, mem, ts, vz, sig, v, idp):
        """The pitch is the one thing about this mount that changes, and it changes in steps (it is re-aimed by
        hand). A two-sided CUSUM of the pitch row's own residual finds a step within a few frames; the state is
        then put at the step's estimate with an uncertainty that says so, and converges again within seconds.
        The step is reported with its size (docs/DESIGN.md section 2)."""
        dp = ctx.val("dpitch")
        r = vz + v * dp                                   # residual of the row; in angle: -r / v
        z = max(-6.0, min(6.0, r / sig))
        cs = mem.get("cusum") or {"p": 0.0, "n": 0.0, "d": []}
        mem["cusum"] = cs
        cs["p"] = max(0.0, cs["p"] + z - PITCH_CUSUM_K)
        cs["n"] = max(0.0, cs["n"] - z - PITCH_CUSUM_K)
        if cs["p"] == 0.0 and cs["n"] == 0.0:
            cs["d"] = []
        else:
            cs["d"].append(-r / v)
            del cs["d"][:-40]
        pend = mem.get("pitch_step")
        if max(cs["p"], cs["n"]) >= PITCH_CUSUM_H and len(cs["d"]) >= 6 and ctx.std("dpitch") < 0.01:
            d = sorted(cs["d"])
            step = d[len(d) // 2]
            if abs(step) >= PITCH_STEP_MIN:
                ctx.integ.reset_aux(idp, dp + step, max(0.01, 0.5 * abs(step)))
                mem["pitch_step"] = {"t": ts, "from": (pend or {}).get("from", dp), "n": mem.get("mount_n", 0)}
                mem["steps"] = mem.get("steps", 0) + 1
                ctx.integ.fdi._set("vision", "mount", "pitch_step", "warn", ts,
                                   "camera pitch changed by about %+.1f deg; re-estimating" % math.degrees(step))
            mem["cusum"] = None
        elif pend is not None and ctx.std("dpitch") < 0.003 and mem.get("mount_n", 0) - pend["n"] >= 40:
            step = dp - pend["from"]
            mem["pitch_step"] = None
            ctx.integ.fdi.findings.pop(("vision", "mount", "pitch_step"), None)
            mem["last_step"] = {"rad": round(step, 4), "deg": round(math.degrees(step), 2)}
            ctx.integ.fdi.event("vision", "mount_step", None, "camera pitch changed by %+.2f deg (%+.4f rad, +- %.2f deg); residual "
                                "against the calibration in use now %+.4f rad" % (math.degrees(step), step, math.degrees(2 * ctx.std("dpitch")), dp))

    def report(self, ctx):
        return {"road_model": ctx.mem.get("road_model")}

    def calib_state(self, ctx):
        """The mount as the estimator sees it: residual angles against the calibration in use, their sum with it
        (the calibration this camera should run with), and a state word."""
        cal = ctx.mem.get("calib")
        dy, dp = ctx.val("dyaw"), ctx.val("dpitch")
        sy, sp = ctx.std("dyaw"), ctx.std("dpitch")
        nm = [ctx.integ.nis.get((self.group, q)) for q in ("mount_yaw", "mount_pitch")]
        if ctx.faulted():
            state = "unknown"
        elif ctx.mem.get("pitch_step") is not None:
            state = "pitch_changed"
        elif any(m is not None and m.high for m in nm):
            state = "moved"
        elif max(sy, sp) > 0.005 or ctx.mem.get("mount_n", 0) < 100:
            state = "converging"
        else:
            state = "ok" if max(abs(dy), abs(dp)) < 0.02 else "offset"
        out = {"state": state, "samples": ctx.mem.get("mount_n", 0), "pitch_steps": ctx.mem.get("steps", 0),
               "handled": ctx.mem.get("handled_n", 0),
               "freedom": "pitch (yaw fixed)"}
        if ctx.mem.get("last_step"):
            out["last_pitch_step"] = ctx.mem["last_step"]
        if cal is not None and fin(cal[0]) and fin(cal[1]):
            out["in_use"] = {"pitch": cal[0], "yaw": cal[1]}
            out["suggested"] = {"pitch": round(cal[0] + dp, 4), "yaw": round(cal[1] + dy, 4)}
        return out
