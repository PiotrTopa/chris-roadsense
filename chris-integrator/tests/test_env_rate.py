"""envd at one inference every 2 s with the last message re-sent every second (`--period 2 --republish 1`, the
installed configuration since lane L53): the consumers must take it as they took the 1 Hz stream.

    cd chris-integrator && python3 -m unittest tests.test_env_rate -v
"""
import os
import sys
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))
from chris_integrator.plugins.common import Rates  # noqa: E402
from chris_integrator.trackers.speedlimit import SpeedLimit  # noqa: E402

NS = 1_000_000_000


class _Bus:
    def __init__(self, governor=None):
        self.governor = governor

    def get(self, name):
        return self.governor if name == "governor" else None


class _Mixer:
    def __init__(self, governor=None):
        self.bus = _Bus(governor)


def ride(sl, seconds, period, republish):
    """Road at 10 Hz; env inference every `period` s, the same message (same seq and frame time) re-sent every
    `republish` s in between (0 = never). Returns the limit after `seconds`."""
    seq, last_env, t_inf = 0, None, None
    for k in range(int(seconds * 10) + 1):
        t = k / 10
        seq += 1
        sl.on_message({"type": "road", "seq": seq, "ts_mono_ns": int(t * NS), "v_ego": 12.0,
                       "pose": {"vx": 12.0, "yaw_rate": 0.0}})
        if k % int(period * 10) == 0:
            seq += 1
            scores = {c: 0.02 for c in ("highway", "residential", "rural_road", "forest_road")}
            scores["city_street"] = 0.7
            last_env = {"type": "env", "seq": seq, "ts_mono_ns": int(t * NS),
                        "place": {"label": "city_street", "confidence": 0.7, "stable_s": 60.0, "scores": scores}}
            t_inf = t
            sl.on_message(dict(last_env))
        elif republish and last_env is not None and k % int(republish * 10) == 0 and t > t_inf:
            sl.on_message(dict(last_env))         # the copy: unchanged, the frame time of its inference
    return sl.snapshot()["limit_kmh"]


class EnvRateTest(unittest.TestCase):
    def test_env_has_no_rate_floor(self):
        """Nothing marks `env` slow: no floor by default, so 1 message a second (and 0.5 without republish) is
        not degraded; only the governor's cap (`shed`) is."""
        r = Rates(_Mixer({"level": 0, "caps": {}}))
        self.assertNotIn("env", r.floors)
        for hz in (1.0, 0.5):
            s = r.status("env", hz=hz, latency_ms=1500.0)
            self.assertFalse(s["degraded"], s)
            self.assertEqual(s["why"], [])
        s = Rates(_Mixer({"level": 1, "caps": {"env_hz": 0.2}})).status("env", hz=0.2)
        self.assertEqual(s["why"], ["shed"])

    def test_env_area_prior_same_with_period_2_and_republish(self):
        """The env area prior must hold 20 s of frame time: the re-sent copies neither shorten nor break it."""
        for period, republish in ((1, 0), (2, 1), (2, 0)):
            with self.subTest(period=period, republish=republish):
                self.assertIsNone(ride(SpeedLimit(), 18, period, republish))
                self.assertEqual(ride(SpeedLimit(), 24, period, republish), 50)


if __name__ == "__main__":
    unittest.main()
