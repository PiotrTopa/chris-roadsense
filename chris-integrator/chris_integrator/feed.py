"""The direct feed: the car backend speaks the vehicle input API (docs/VEHICLE_API.md of roadsense) to the
integrator's `feed.sock`, as it did to mapd's `position.sock`. Messages are turned into the shapes the sources
read (`ego`, `gnss`, `vehicle`, `objects`), stamped on this host's monotonic clock when they carry no stamp."""
import math
import time

ACCEPTS = ["gnss", "motion", "reset", "controls", "body", "vehicle", "objects"]
NS = 1_000_000_000


def hello():
    return {"type": "hello", "api": "1.1", "server": "chris-integrator", "accepts": ACCEPTS, "clock": "CLOCK_MONOTONIC",
            "now_mono_ns": time.monotonic_ns()}


def _stamp(m, now_ns):
    ts = m.get("ts_mono_ns")
    if isinstance(ts, int):
        return ts
    age = m.get("age_ms")
    if isinstance(age, (int, float)) and math.isfinite(age) and age >= 0:
        return now_ns - int(age * 1e6)
    return now_ns


class Feed:
    """Feeder messages -> integrator input messages. Keeps the last controls / body so that the `vehicle`
    message carries every signal."""

    def __init__(self):
        self.controls, self.body, self.seq = {}, {}, 0

    def convert(self, m, now_ns=None):
        """List of input messages for the integrator (possibly empty)."""
        if not isinstance(m, dict):
            return []
        now_ns = time.monotonic_ns() if now_ns is None else now_ns
        t = m.get("type")
        ts = _stamp(m, now_ns)
        self.seq += 1
        if t == "motion":
            v = m.get("speed_mps")
            ws = m.get("wheel_speeds_mps")
            still = m.get("standstill")
            if still is None and isinstance(v, (int, float)):
                still = v == 0.0 and (not isinstance(ws, list) or all(w == 0.0 for w in ws))
            out = {"type": "ego", "seq": self.seq, "ts_mono_ns": ts, "api": 1, "speed_mps": v, "gear": m.get("gear"),
                   "reverse": bool(m.get("reverse") or m.get("gear") == "R"), "standstill": bool(still),
                   "wheel_speeds_mps": ws, "yaw_rate_rps": m.get("yaw_rate_rps"),
                   "yaw_source": "yaw_rate" if m.get("yaw_rate_rps") is not None else None,
                   "turn_signal": m.get("turn_signal", self.controls.get("turn_signal")), "source": "vehicle",
                   "tag": m.get("source") or m.get("tag"), "steering_wheel_deg": m.get("steering_wheel_deg"),
                   "accel_long_mps2": m.get("accel_long_mps2"), "accel_lat_mps2": m.get("accel_lat_mps2")}
            if out["reverse"] and isinstance(v, (int, float)) and v > 0:
                out["speed_mps"] = -v
            return [out]
        if t == "gnss":
            return [dict(m, ts_mono_ns=ts)]
        if t in ("controls", "body"):
            (self.controls if t == "controls" else self.body).update({k: v for k, v in m.items() if k not in ("type", "ts_mono_ns", "age_ms")})
            return [{"type": "vehicle", "seq": self.seq, "ts_mono_ns": ts, "present": True, "controls": dict(self.controls),
                     "body": dict(self.body)}]
        if t == "objects":
            return [dict(m, ts_mono_ns=ts)]
        return []
