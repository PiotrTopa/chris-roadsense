"""replay.load_pos: the generic line format and the example trip-log adapter (synthetic fixes, no recording)."""
import json
import os
import sys
import tempfile
import unittest
from datetime import datetime, timezone

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from chris_integrator.replay import load_pos  # noqa: E402


def fix(mono_s, real_s=None):
    m = {"type": "gnss", "ts_mono_ns": int(mono_s * 1e9), "fix": 3, "lat": 52.0, "lon": 20.99, "hacc_m": 4.0}
    if real_s is not None:
        m["ts_real_ns"] = int(real_s * 1e9)
    return m


class TestLoadPos(unittest.TestCase):
    def write(self, lines):
        fd, path = tempfile.mkstemp(suffix=".txt")
        with os.fdopen(fd, "w") as f:
            f.write("\n".join(lines) + "\n")
        self.addCleanup(os.unlink, path)
        return path

    def test_generic_lines(self):
        path = self.write([json.dumps(fix(20.0, 1_700_000_020.0)), json.dumps(fix(10.0, 1_700_000_010.0)),
                           json.dumps({"type": "motion", "ts_mono_ns": 1}), "not json", json.dumps(fix(30.0))])
        got = load_pos(path)
        self.assertEqual([d["ts_mono_ns"] for d in got], [10_000_000_000, 20_000_000_000])   # sorted; no wall time: skipped
        self.assertAlmostEqual(got[0]["_wall"], 1_700_000_010.0)

    def test_trip_log_adapter(self):
        rec = {"ts": 1500, "dir": "POS", "d": fix(5.0)}
        path = self.write(["logs/trip_20260101_120000.jsonl: " + json.dumps(rec),
                           "logs/trip_20260101_120000.jsonl: " + json.dumps({"ts": 1, "d": {"type": "motion"}})])
        got = load_pos(path, tz="UTC")
        self.assertEqual(len(got), 1)
        start = datetime(2026, 1, 1, 12, 0, 0, tzinfo=timezone.utc).timestamp()
        self.assertAlmostEqual(got[0]["_wall"], start + 1.5)
        try:
            got = load_pos(path, tz="Europe/Warsaw")                 # UTC+1 in January
        except Exception:                                              # no time zone database on this machine
            self.skipTest("no IANA time zone data")
        self.assertAlmostEqual(got[0]["_wall"], start - 3600 + 1.5)


if __name__ == "__main__":
    unittest.main()
