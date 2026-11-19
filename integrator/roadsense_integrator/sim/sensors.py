"""Simulated sensors of the stack's own inputs: each `generate(world)` yields (arrival time, message) with the
message's `ts_mono_ns` as the real producer would stamp it. Noise, bias, scale, latency, drop-outs and faults
are parameters, so a test states the truth it expects the integrator to recover."""
import math
import random

import numpy as np

from .world import CAM_TO_AXLE, G, NS, geo

X_IDXS = [192.0 * (i / 32.0) ** 2 for i in range(33)]


def ns(t):
    return int(round(t * NS))


class _GM:
    """First-order Gauss-Markov noise sampled at irregular times."""

    def __init__(self, rng, sigma, tau):
        self.rng, self.sigma, self.tau = rng, sigma, tau
        self.x, self.t = rng.gauss(0.0, sigma), None

    def at(self, t):
        if self.t is not None:
            rho = math.exp(-abs(t - self.t) / self.tau)
            self.x = rho * self.x + math.sqrt(max(1.0 - rho * rho, 0.0)) * self.rng.gauss(0.0, self.sigma)
        self.t = t
        return self.x


class VehicleSim:
    """The feeder's motion as mapd relays it (`ego`), 10 Hz. Faults: `silences` [(t0, t1)] with stale first
    samples on resume; `speed_fault` (t0, t1, factor): the speed signal and the wheels read factor x true."""

    def __init__(self, seed=11, hz=10.0, gyro_bias=0.0, gyro_scale=0.0, accel_bias=0.0, accel_lat_bias=0.0,
                 speed_sigma=0.015, gyro_sigma=0.003, accel_sigma=0.2, silences=(), stale_n=3, speed_fault=None,
                 latency=0.004, wheel_scale=0.98, track=1.48, off=()):
        self.__dict__.update(locals())

    def generate(self, w):
        rng = random.Random(self.seed)
        t = w.t0 + 0.013
        seq = 0
        held = None
        stale = 0
        was_silent = False
        while t < w.t0 + w.T:
            rel = t - w.t0
            silent = any(a <= rel < b for a, b in self.silences) or any(a <= rel < b for a, b in self.off)
            tr = w.truth_at(t)
            vg = tr["v"]
            v = (1.0 + w.speed_scale) * vg
            if self.speed_fault and self.speed_fault[0] <= rel < self.speed_fault[1]:
                v *= self.speed_fault[2]
            still = vg == 0.0
            vm = 0.0 if still else max(0.0, v + rng.gauss(0.0, self.speed_sigma))
            wz = tr["w"]
            half = 0.5 * wz * self.track
            ws = [0.0] * 4 if still else [round(max(0.0, self.wheel_scale * (vm + sgn * half) + rng.gauss(0, 0.01)), 3)
                                          for sgn in (-1, 1, -1, 1)]
            msg = {"type": "ego", "seq": seq, "ts_mono_ns": ns(t), "api": 1, "speed_mps": round(vm, 3), "gear": "D",
                   "reverse": False, "standstill": still, "wheel_speeds_mps": ws,
                   "yaw_rate_rps": round(0.0 if still else (1.0 + self.gyro_scale) * wz + self.gyro_bias + rng.gauss(0, self.gyro_sigma), 5),
                   "yaw_source": "yaw_rate", "turn_signal": None, "source": "vehicle", "tag": "sim-can",
                   "accel_long_mps2": round(tr["a"] + G * math.sin(tr["th"]) + self.accel_bias + rng.gauss(0, self.accel_sigma), 3),
                   "accel_lat_mps2": round(vg * wz + self.accel_lat_bias + rng.gauss(0, 0.3), 3)}
            if not silent:
                if was_silent and held is not None:
                    stale = self.stale_n
                if stale > 0 and held is not None:           # the feed resumes with what it held before the silence
                    stale -= 1
                    msg = dict(held, seq=seq, ts_mono_ns=ns(t))
                else:
                    held = msg
                yield t + self.latency, msg
                seq += 1
            was_silent = silent
            t += 1.0 / self.hz + rng.uniform(-0.004, 0.004)


class VisionSim:
    """The road model (`road`): pose of the 200 ms image pair, the lead hypothesis, the plan. Faults:
    `knock` (t0, t1, speed factor): the incident (speed collapses, stated std explodes, a fake near lead);
    `off` [(t0, t1)]: no frames."""

    def __init__(self, seed=21, hz=13.0, latency=0.10, pair_lag=0.125, speed_scale=0.0, gm_sigma=0.25, gm_tau=3.0,
                 white=(0.12, 0.018), yaw_sigma=0.006, yaw_gain=1.0, mount_yaw=0.0, mount_pitch=0.0, range_scale=1.0,
                 knock=None, off=(), lead_prob=0.93, flicker=0.04, road_model="vision", leads=True, lever=CAM_TO_AXLE):
        self.__dict__.update(locals())

    def generate(self, w):
        rng = random.Random(self.seed)
        gm = _GM(rng, self.gm_sigma, self.gm_tau)
        t = w.t0 + 0.05
        seq = 0
        low = 0
        while t < w.t0 + w.T:
            rel = t - w.t0
            if any(a <= rel < b for a, b in self.off):
                t += 1.0 / self.hz
                continue
            tp = w.truth_at(t - self.pair_lag)
            v = (1.0 + w.speed_scale) * tp["v"]
            sig = self.white[0] + self.white[1] * v
            knocked = self.knock is not None and self.knock[0] <= rel < self.knock[1]
            still = tp["v"] == 0.0
            vx = (1.0 + self.speed_scale) * v + (0.0 if still else gm.at(t) * (0.4 + 0.06 * v) / self.gm_sigma * self.gm_sigma + rng.gauss(0, sig))
            vx_std = 0.05 if still else 0.25 + 0.03 * v
            vy = v * self.mount_yaw - tp["w"] * self.lever + rng.gauss(0, 0.02 + 0.004 * v)
            vz = -v * self.mount_pitch + rng.gauss(0, 0.02 + 0.004 * v)
            if knocked:
                vx = self.knock[2] * v + rng.gauss(0, 0.5 + 0.2 * v)
                vx_std = 2.0 + 0.5 * v
            yaw = -(self.yaw_gain * tp["w"]) + rng.gauss(0, self.yaw_sigma)
            msg = {"type": "road", "seq": seq, "ts_mono_ns": ns(t), "ts_real_ns": ns(t) + 1_700_000_000 * NS,
                   "road_model": self.road_model, "v_ego": round(vx, 4), "valid": True, "pair_dt_ms": 200.0,
                   "pose": {"vx": round(vx, 4), "vy": round(vy, 4), "vz": round(vz, 4), "yaw_rate": round(yaw, 5),
                            "vx_std": round(vx_std, 3), "vy_std": 0.03, "vz_std": 0.03, "yaw_rate_std": 0.004}}
            # the plan: the road ahead over the next seconds, in the device frame (y right)
            reach = max(8.0, min(150.0, 6.0 * max(v, 1.0)))
            xs = [x for x in X_IDXS if x <= reach]
            msg["plan"] = {"x": [round(x, 2) for x in xs], "y": [round(-y, 3) for y in w.path_ahead(t, xs)]}
            if self.leads:
                ld = w.lead_at(t)
                p = self.lead_prob
                if low > 0:
                    low -= 1
                    p = rng.uniform(0.12, 0.28)
                elif rng.random() < self.flicker:
                    low = rng.randint(1, 4)
                lead = {"prob": 0.01, "x": [80.0], "y": [0.0], "v": [0.0], "a": [0.0], "x_std": [8.0], "y_std": [1.0],
                        "v_std": [3.0], "a_std": [1.0]}
                if ld is not None and ld["x"] < 120.0:
                    d = ld["x"] * self.range_scale
                    x = d * (1.0 + rng.gauss(0, 0.012)) + rng.gauss(0, 0.1)
                    vl = vx + (ld["vx"] - tp["v"]) * (1.0 + w.speed_scale) + rng.gauss(0, 0.15 + 0.004 * d)
                    lead = {"prob": round(p if ld["x"] < 90 else 0.4 * p, 3), "x": [round(x, 2)], "y": [round(-ld["y"] + rng.gauss(0, 0.15), 2)],
                            "v": [round(vl, 3)], "a": [0.0], "x_std": [round(0.5 + 0.06 * d, 2)], "y_std": [0.3],
                            "v_std": [round(0.3 + 0.01 * d, 2)], "a_std": [1.0]}
                if knocked:                                    # the incident's fake lead a few metres ahead
                    lead = {"prob": 0.8, "x": [round(5.5 + rng.gauss(0, 0.8), 2)], "y": [0.0], "v": [round(vx, 2)], "a": [0.0],
                            "x_std": [1.0], "y_std": [0.3], "v_std": [0.5], "a_std": [1.0]}
                msg["leads"] = [lead]
            yield t + self.latency + rng.uniform(0.0, 0.03), msg
            seq += 1
            t += 1.0 / self.hz + rng.uniform(-0.006, 0.02)


class GnssSim:
    """A receiver like the reference car's head unit: one fix a second, stamped when it arrives, describing
    where the car was `lag` s earlier; correlated position noise; a claimed hacc that is too small; a speed
    field that is slower still. `outage` [(t0, t1)]; `jump` (t0, t1, east m, north m): a multipath excursion."""

    def __init__(self, seed=31, hz=1.0, lag=2.0, sigma_gm=2.0, tau_gm=20.0, sigma_white=1.0, hacc=3.0, speed_lag=5.0,
                 speed_gain=0.9, outage=(), jump=None, alt_sigma=4.0):
        self.__dict__.update(locals())

    def generate(self, w):
        rng = random.Random(self.seed)
        ge, gn = _GM(rng, self.sigma_gm, self.tau_gm), _GM(rng, self.sigma_gm, self.tau_gm)
        t = w.t0 + 0.4
        while t < w.t0 + w.T:
            rel = t - w.t0
            if not any(a <= rel < b for a, b in self.outage):
                tr = w.truth_at(t - self.lag)
                # the antenna sits about over the rear axle: the estimator's position reference
                e = tr["pe"] + ge.at(t) + rng.gauss(0, self.sigma_white)
                n = tr["pn"] + gn.at(t) + rng.gauss(0, self.sigma_white)
                if self.jump and self.jump[0] <= rel < self.jump[1]:
                    e, n = e + self.jump[2], n + self.jump[3]
                if tr["v"] == 0.0:                         # a standing receiver repeats itself
                    e, n = round(e), round(n)
                lat, lon = geo(e, n)
                ts_ = w.truth_at(t - self.speed_lag)
                yield t + 0.02, {"type": "gnss", "fix": "3d", "lat": round(lat, 7), "lon": round(lon, 7), "hacc_m": self.hacc,
                                 "speed_mps": round(self.speed_gain * ts_["v"] + abs(rng.gauss(0, 0.3)), 3),
                                 "course_deg": round((90.0 - math.degrees(ts_["psi"]) + rng.gauss(0, 3.0)) % 360.0, 2),
                                 "alt_m": round(tr["z"] + rng.gauss(0, self.alt_sigma), 1), "sats": 7,
                                 "ts_mono_ns": ns(t), "source": "sim"}
            t += 1.0 / self.hz + rng.uniform(-0.01, 0.01)


class MapSim:
    """mapd's `map` message as far as the integrator reads it: the matched road's nearest point and bearing.
    The road's centre line lies `centre_offset` m left of the ego's lane centre. `wrong` [(t0, t1, offset m)]:
    the matcher sits on a parallel road."""

    def __init__(self, seed=41, hz=2.0, centre_offset=1.75, confidence=0.8, wrong=(), latency=0.05):
        self.__dict__.update(locals())

    def generate(self, w):
        t = w.t0 + 0.3
        seq = 0
        while t < w.t0 + w.T:
            rel = t - w.t0
            tr = w.truth_at(t)
            off = self.centre_offset
            for a, b, o in self.wrong:
                if a <= rel < b:
                    off = o
            x, y, hd = w._road_xy(tr["s"], off)
            lat, lon = geo(x, y)
            yield t + self.latency, {"type": "map", "seq": seq, "ts_mono_ns": ns(t), "state": "good",
                                     "match": {"matched": True, "confidence": self.confidence, "ambiguous": False,
                                               "lat": round(lat, 7), "lon": round(lon, 7),
                                               "bearing_deg": round((90.0 - math.degrees(hd)) % 360.0, 2)}}
            seq += 1
            t += 1.0 / self.hz


def default_sensors(**kw):
    """The reference car's sensor set (no radar): vehicle, road model, receiver, map."""
    return [VehicleSim(**kw.get("vehicle", {})), VisionSim(**kw.get("vision", {})), GnssSim(**kw.get("gnss", {})),
            MapSim(**kw.get("map", {}))]
