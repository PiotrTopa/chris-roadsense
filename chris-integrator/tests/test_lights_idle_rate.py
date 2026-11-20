"""The `lights` rate marker follows the light detector's state: the full floor while it tracks a light, a floor
that follows the announced idle pattern while it tracks none (`idle: {"hz": H}` in its message).

    cd chris-integrator && python3 -m unittest tests.test_lights_idle_rate -v
"""
import os
import sys
import unittest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from chris_integrator.mixer import MessageClock  # noqa: E402
from chris_integrator.stack import build  # noqa: E402

PERIOD_NS = 200_000_000                    # the detector's frame period at 5 Hz


def light_msg(seq, ts, idle_hz=None, pattern=None, tracked=False):
    m = {"type": "lights", "seq": seq, "ts_mono_ns": ts, "ts_real_ns": 1_700_000_000_000_000_000 + ts,
         "lights_ms": 120.0, "frame": [1920, 1080], "roi": [0, 0, 1920, 670], "view": 0, "lights": []}
    if tracked:
        m["lights"] = [{"track_id": 1, "state": "red", "state_conf": 0.9, "state_s": 1.0, "frame_state": "red",
                        "flashing": False, "score": 0.8, "box": [950, 300, 965, 340], "horizontal": False,
                        "hits": 8, "age": 8, "coast": 0}]
    if idle_hz is not None:
        m["idle"] = {"hz": idle_hz, "pattern": pattern}
    return m


class Ride:
    """Delivers light messages on the 5 Hz period grid; `pattern` letters: W / Z = a message, . = none."""

    def __init__(self):
        self.st = build(clock=MessageClock(), mapper=None)
        self.out = []
        self.st.mixer.add_output("t", lambda e: self.out.append(e.msg) if e.stream == "lights" else None)
        self.k, self.seq = 0, 0

    def run(self, seconds, pattern="WZ", idle_hz=None, tracked=False, announce=True):
        n = int(round(seconds * 5))
        for i in range(n):
            self.k += 1
            if pattern[i % len(pattern)] == ".":
                continue
            self.seq += 1
            self.st.deliver(light_msg(self.seq, self.k * PERIOD_NS, idle_hz if announce else None,
                                      pattern if idle_hz else None, tracked))
        return self.out[-1]["rate"]


def hz_of(pattern):
    return 5.0 * sum(c != "." for c in pattern) / len(pattern)


class LightsIdleRateTest(unittest.TestCase):
    def test_tracked_full_rate_is_healthy(self):
        r = Ride().run(6, "WZ", tracked=True)
        self.assertFalse(r["degraded"], r)
        self.assertIsNone(r["idle_hz"])

    def test_idle_patterns_that_keep_their_announced_rate_are_healthy(self):
        for pat in ("ZW.Z.W", "W.Z.", "W..Z..", "W...Z..."):
            with self.subTest(pattern=pat):
                r = Ride().run(8, pat, idle_hz=hz_of(pat))
                self.assertFalse(r["degraded"], r)
                self.assertNotIn("slow", r["why"])
                self.assertAlmostEqual(r["idle_hz"], round(hz_of(pat), 2))
                self.assertGreaterEqual(r["hz"], 0.8 * hz_of(pat))

    def test_unannounced_sparse_stream_is_slow(self):
        """One pass in two periods (2.5 a second) without the announcement: the full floor (3 a second) calls it
        slow; announced, it is healthy (previous test)."""
        r = Ride().run(8, "W.Z.", idle_hz=hz_of("W.Z."), announce=False)
        self.assertIn("slow", r["why"])
        self.assertTrue(r["degraded"])

    def test_idle_stream_slower_than_its_announcement_is_slow(self):
        """Announcing ZW.Z.W (3.3 a second) but delivering one message in four periods (1.25 a second)."""
        ride = Ride()
        ride.run(8, "W...Z...", idle_hz=hz_of("ZW.Z.W"))
        r = ride.out[-1]["rate"]
        self.assertIn("slow", r["why"])
        self.assertTrue(r["degraded"])

    def test_tracked_after_idle_and_back(self):
        ride = Ride()
        ride.run(6, "ZW.Z.W", idle_hz=hz_of("ZW.Z.W"))
        r = ride.run(0.4, "WZ", tracked=True)        # a light appears: inside the window of the idle arrivals
        self.assertNotIn("slow", r["why"], r)
        r = ride.run(4, "WZ", tracked=True)          # the window is all full-rate now: the full floor again
        self.assertFalse(r["degraded"], r)
        self.assertIsNone(r["idle_hz"])
        r = ride.run(4, "W.Z.", tracked=True)        # tracking a light at 2.5 a second: under the full floor
        self.assertIn("slow", r["why"])

    def test_health_marker_reads_the_providers_newest_message(self):
        ride = Ride()
        ride.run(8, "W..Z..", idle_hz=hz_of("W..Z.."))
        r = ride.st.mixer.services["rates"].status("lights")
        self.assertEqual(r["idle_hz"], round(hz_of("W..Z.."), 2))
        self.assertFalse(r["degraded"], r)


if __name__ == "__main__":
    unittest.main()
