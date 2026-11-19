"""This stack's sources: input descriptions and measurement models, with the noise measured on the car
(docs/lanes/L35-ego-fusion-audit.md section 4, docs/lanes/L37-integrator.md). These numbers belong to the
sources (CAN 0x0B4 speed, the road model's 200 ms image pair); a new source brings its own.

    vehicle   `ego` (mapd's relay of the feeder's motion): speed, standstill, accelerometer, yaw rate, turn signal;
              `vehicle`: the brake.                                          rank 3, fused
    wheels    `ego.wheel_speeds_mps`: mean wheel speed, a second reading of the speed with its own decoding.
              Family `vehicle`: it checks the speed signal's decoding, it is no independent vote (same sensors).
                                                                             rank 2, monitor only
    vision    `road.pose` (the road model): speed and yaw rate of the image pair, its own stated uncertainty,
              a standstill detector for cars without a vehicle feed.         rank 1, fused
    gnss      `gnss` (mapd's relay of the receiver's fixes): speed over ground.   witness
"""
import math

from .core import Measurement, SourceSpec

NS = 1e9
VISION_PAIR_LAG_S = 0.125        # the pose describes the 200 ms image pair: this long before the frame's stamp
VISION_STOP_SPEED, VISION_GO_SPEED, VISION_STOP_CONFIRM_S = 0.15, 0.8, 1.0
FEED_GAP_S, FEED_WARMUP_S = 1.0, 0.35  # a vehicle feed silent this long is not believed for this long after it resumes
YAW_SIGMA = {"yaw_rate": (0.004, 0.03), "wheel_speeds": (0.02, 0.05), "steering": (0.03, 0.10)}


def _fin(v):
    return isinstance(v, (int, float)) and not isinstance(v, bool) and math.isfinite(v)


def vision_speed_sigma(v):
    """Characterised 1-sigma of the road model's speed against the vehicle's (two healthy drives)."""
    return 0.25 + 0.035 * abs(v)


def _warming(t, mem):
    """After a silence of the feed its first samples repeat what it held before the silence (measured on the
    car: 19-25 silences of about 7 s per drive, the first 2-3 samples after each, up to 0.28 s, stale by up to
    5 m/s): they are not measurements. True while the feed warms up."""
    last = mem.get("last_t")
    mem["last_t"] = t
    if last is not None and t - last > FEED_GAP_S:
        mem["warm_until"] = t + FEED_WARMUP_S
        mem["warmups"] = mem.get("warmups", 0) + 1
    return t < mem.get("warm_until", -1e18)


def vehicle_model(msg, mem):
    t = msg["ts_mono_ns"] / NS
    out = []
    warm = _warming(t, mem)
    mem["info"] = {"tag": msg.get("tag"), "warmups": mem.get("warmups", 0)}
    if warm:
        return out
    sig = msg.get("turn_signal")
    if sig is not None:
        out.append(Measurement("signal:turn_signal", t, sig, role="signal"))
    v = msg.get("speed_mps")
    if not _fin(v):
        return out
    v = float(v)
    still = bool(msg.get("standstill")) and v == 0.0
    out.append(Measurement("standstill", t, still, src="vehicle.standstill", role="flag"))
    if not still:
        out.append(Measurement("speed", t, v, 0.03 + 0.002 * abs(v), "vehicle.speed"))
        a = msg.get("accel_long_mps2")
        if _fin(a):
            out.append(Measurement("accel", t, float(a), 0.25, "vehicle.accel"))
    else:
        out.append(Measurement("speed", t, 0.0, 0.03, "vehicle.speed", role="monitor"))   # for the monitors only
    r = msg.get("yaw_rate_rps")
    if _fin(r):
        s0, s1 = YAW_SIGMA.get(msg.get("yaw_source"), YAW_SIGMA["steering"])
        out.append(Measurement("yaw_rate", t, float(r), s0 + s1 * abs(r), "vehicle.yaw",
                               role="monitor" if still else "fuse"))
    return out


def vehicle_signals_model(msg, mem):
    t = msg["ts_mono_ns"] / NS
    b = (msg.get("controls") or {}).get("brake")
    return [Measurement("signal:brake", t, b, role="signal")] if isinstance(b, bool) else []


def wheels_model(msg, mem):
    if _warming(msg["ts_mono_ns"] / NS, mem):
        return []
    ws = msg.get("wheel_speeds_mps")
    if not isinstance(ws, (list, tuple)) or len(ws) < 2 or not all(_fin(w) for w in ws):
        return []
    m = sum(ws) / len(ws)
    if msg.get("reverse"):
        m = -m
    # the scale of wheel speeds against the speed signal is a car's own (0.98 here): the tolerance covers it
    return [Measurement("speed", msg["ts_mono_ns"] / NS, m, 0.10 + 0.04 * abs(m), "wheels.speed", role="monitor")]


def vision_model(msg, mem):
    """The road model's pose. `road.valid` false (non-finite outputs) yields nothing."""
    out = []
    ts = msg["ts_mono_ns"] / NS
    pose = msg.get("pose") or {}
    v = pose.get("vx")
    if not _fin(v):
        v = msg.get("v_ego")
    mem["info"] = {"road_model": msg.get("road_model")}
    if msg.get("valid") is False or not _fin(v):
        return out
    v = float(v)
    t = ts - VISION_PAIR_LAG_S
    char = vision_speed_sigma(v)
    vs = pose.get("vx_std")
    out.append(Measurement("speed", t, v, max(char, float(vs)) if _fin(vs) else char, "vision.pose", gate=True,
                           msigma=char))
    if _fin(vs) and abs(v) >= 1.0:
        out.append(Measurement("self:speed", ts, float(vs) / char, role="self"))
    r = pose.get("yaw_rate")
    if _fin(r):
        r = -float(r)                                # the device frame is z down: positive right
        rs = pose.get("yaw_rate_std")
        sig = 0.007 + 0.25 * abs(r)
        out.append(Measurement("yaw_rate", t, r, max(sig, float(rs)) if _fin(rs) else sig, "vision.pose", gate=True,
                               msigma=sig))
    # standstill for cars without a vehicle feed: the model's speed under the threshold for the confirmation
    # time. Leaky, so a single-frame blip (a wiper pass) does not end it; a real start does within 0.3 s.
    last = mem.get("t")
    dt = min(max(ts - last, 0.0), 1.0) if last is not None else 0.0
    mem["t"] = ts
    acc = mem.get("stop_s", 0.0)
    if v >= VISION_GO_SPEED:
        acc = 0.0
    elif v < VISION_STOP_SPEED:
        acc = min(acc + dt, VISION_STOP_CONFIRM_S + 0.3)
    else:
        acc = max(0.0, acc - 3.0 * dt)
    mem["stop_s"] = acc
    out.append(Measurement("standstill", ts, acc >= VISION_STOP_CONFIRM_S, role="flag"))
    return out


def gnss_model(msg, mem):
    """Receiver speed over ground. A message that repeats the previous one (the head unit re-posts its last
    Location) is not a measurement; the share of repeats is reported."""
    n, rep = mem.get("n", 0) + 1, mem.get("rep", 0)
    key = (msg.get("lat"), msg.get("lon"), msg.get("speed_mps"), msg.get("course_deg"))
    repeat = key == mem.get("key")
    mem["key"] = key
    if repeat:
        rep += 1
    mem["n"], mem["rep"] = n, rep
    mem["info"] = {"fix": msg.get("fix"), "hacc_m": msg.get("hacc_m"), "repeat_share": round(rep / n, 2),
                   "receiver": msg.get("source")}
    v = msg.get("speed_mps")
    if repeat or msg.get("fix") not in ("2d", "3d", "dgps", "rtk") or not _fin(v):
        return []
    acc = msg.get("speed_acc_mps")
    return [Measurement("speed", msg["ts_mono_ns"] / NS, float(v), max(float(acc), 0.5) if _fin(acc) else 1.5,
                        "gnss.speed", role="monitor")]


def default_sources():
    return [
        SourceSpec("vehicle", "vehicle", 3, ("ego",), vehicle_model, stale_s=0.6,
                   info="vehicle motion from the host's feeder (CAN), relayed by mapd"),
        SourceSpec("vehicle.signals", "vehicle", 3, ("vehicle",), vehicle_signals_model, stale_s=0.6, counts=False),
        SourceSpec("wheels", "wheels", 2, ("ego",), wheels_model, stale_s=0.6, family="vehicle",
                   info="mean of the wheel speeds: a second reading of the speed"),
        SourceSpec("vision", "vision", 1, ("road",), vision_model, stale_s=0.6,
                   info="the road model's pose (200 ms image pair)"),
        SourceSpec("gnss", "gnss", 0, ("gnss",), gnss_model, stale_s=5.0, witness=True,
                   info="receiver speed over ground (timing poor: witness only)"),
    ]
