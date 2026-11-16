"""Drive recorder (api/roadsense_api/recorder.py).

    python3 -m unittest api.tests.test_recorder -v     (from the repository root)
"""
import gzip
import json
import os
import sys
import tempfile
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))

from roadsense_api.lanes import RoadGeometryTracker      # noqa: E402
from roadsense_api.recorder import Recorder, load_config, slim_road      # noqa: E402

X = [192.0 * (i / 32.0) ** 2 for i in range(33)]


def road(seq):
    line = {"x": X, "y": [1.23456789 + 0.001 * i for i in range(33)], "z": [0.0] * 33, "y_std": [0.3] * 33}
    return {"type": "road", "seq": seq, "ts_mono_ns": seq * 50_000_000, "ts_real_ns": 1, "model_ms": 55.1, "v_ego": 12.345678,
            "pose": {"yaw_rate": 0.0123456, "vy": 0.1}, "lane_lines": [dict(line, prob=0.912345) for _ in range(4)],
            "road_edges": [dict(line) for _ in range(2)],
            "leads": [{"prob": 0.9, "x": [30.123456] * 6, "y": [0.1] * 6, "v": [10.0] * 6, "x_std": [1.0] * 6, "v_std": [1.0] * 6}] * 3,
            "plan": {"x": X, "y": [0.0] * 33, "z": [0.0] * 33, "vx": [1.0] * 33, "t": [0.0] * 33},
            "meta": {"big": list(range(200))}, "desire_state": [0.0] * 8}


class Clock:
    def __init__(self):
        self.t = 1000.0

    def __call__(self):
        return self.t


class RecorderTest(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.preview = os.path.join(self.tmp.name, "preview.jpg")
        with open(self.preview, "wb") as f:
            f.write(b"\xff\xd8jpeg\xff\xd9")
        self.clock = Clock()

    def rec(self, **cfg):
        cfg.setdefault("dir", os.path.join(self.tmp.name, "rec"))
        return Recorder(cfg, self.preview, clock=self.clock, wall=lambda: 1794860000.0 + self.clock.t)

    def lines(self, r):
        r.close()
        with gzip.open(os.path.join(r.dir, "messages.jsonl.gz"), "rt") as f:
            return [json.loads(ln) for ln in f]

    def test_records_topics_slims_road_and_copies_frames(self):
        r = self.rec()
        for k in range(40):                                   # 2 s at 20 Hz
            self.clock.t += 0.05
            r.on_message("camera", json.dumps({"type": "camera", "seq": k + 1}))
            r.on_message("road", json.dumps(road(k + 1)))
            r.on_message("lanes", "{\"type\":\"lanes\"}")      # derived: not written
            if k % 7 == 0:
                r.on_message("lights", json.dumps({"type": "lights", "seq": k + 1, "lights": []}))
        out = self.lines(r)
        kinds = [m["type"] for m in out]
        self.assertEqual(kinds.count("road"), 40)
        self.assertEqual(kinds.count("lights"), 6)
        self.assertEqual(kinds.count("camera"), 2)            # one a second
        self.assertNotIn("lanes", kinds)
        rd = next(m for m in out if m["type"] == "road")
        self.assertNotIn("meta", rd)
        self.assertEqual(len(rd["leads"]), 1)
        self.assertEqual(rd["lane_lines"][0]["y"][0], 1.235)
        self.assertLess(len(json.dumps(rd)), 0.6 * len(json.dumps(road(1))))
        self.assertEqual(sorted(os.listdir(os.path.join(r.dir, "frames"))), ["1.jpg", "21.jpg"])
        self.assertEqual(r.status()["frames"], 2)

    def test_recorded_road_replays_through_the_tracker(self):
        trk = RoadGeometryTracker()
        o = None
        for k in range(30):
            o = trk.on_road(slim_road(road(k + 1)))
        self.assertTrue(o["lines"][1]["present"] and o["edges"][0]["present"])

    def test_rate_caps(self):
        r = self.rec(road_hz=5.0, ego_hz=2.0, frame_every_s=0)
        for k in range(40):
            self.clock.t += 0.05
            r.on_message("road", json.dumps(road(k + 1)))
            r.on_message("ego", json.dumps({"type": "ego", "seq": k}))
        kinds = [m["type"] for m in self.lines(r)]
        self.assertEqual((kinds.count("road"), kinds.count("ego")), (10, 4))
        self.assertEqual(os.listdir(os.path.join(r.dir, "frames")), [])

    def test_old_sessions_are_pruned_and_the_current_one_kept(self):
        base = os.path.join(self.tmp.name, "rec")
        for name in ("20261101-080000", "20261102-080000"):
            os.makedirs(os.path.join(base, name))
            with open(os.path.join(base, name, "messages.jsonl.gz"), "wb") as f:
                f.write(b"x" * 600_000)
        r = self.rec(max_mb=1)
        self.clock.t += 1
        r.on_message("env", "{\"type\":\"env\"}")
        self.assertEqual(sorted(os.listdir(base)), ["20261102-080000", os.path.basename(r.dir)])
        r.close()

    def test_a_failing_disk_stops_the_recording_not_the_caller(self):
        blocker = os.path.join(self.tmp.name, "file")
        with open(blocker, "w") as f:
            f.write("x")
        r = self.rec(dir=os.path.join(blocker, "rec"))        # cannot be created
        r.on_message("env", "{\"type\":\"env\"}")
        self.assertTrue(r.failed)
        r.on_message("env", "{\"type\":\"env\"}")             # and stays quiet

    def test_config_file_switches_it_on(self):
        self.assertIsNone(load_config(self.tmp.name))
        with open(os.path.join(self.tmp.name, "recorder.json"), "w") as f:
            json.dump({"dir": "/x", "max_mb": 10, "nonsense": 1}, f)
        self.assertEqual(load_config(self.tmp.name), {"dir": "/x", "max_mb": 10})
        with open(os.path.join(self.tmp.name, "recorder.json"), "w") as f:
            json.dump({"max_mb": 10}, f)
        self.assertIsNone(load_config(self.tmp.name))


if __name__ == "__main__":
    unittest.main()
