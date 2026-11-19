"""The map box alone: poses in (as `ego_state` messages), `map` out. No estimator, no sockets."""
import json
import os
import subprocess
import sys
import tempfile
import unittest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from chris_mapper.box import MapBox      # noqa: E402
from chris_mapper.rsmap import Pack      # noqa: E402
from tests import synthpack as S         # noqa: E402

NS = 10 ** 9


def ego_state(t, x, y, heading=90.0, speed=14.0, sigma=4.0, mode="fused", valid=True, reason=None, frame=1, odo=None):
    lat, lon = S.ll(x, y)
    pos = {"valid": valid, "reason": reason, "mode": mode if valid else "none", "lat": lat if valid else None,
           "lon": lon if valid else None, "cov_en": [sigma * sigma, 0.0, sigma * sigma] if valid else None,
           "fix_age_s": 0.5 if mode == "fused" else 40.0, "sources": ["gnss.pos"], "local": {"frame": frame}}
    return {"type": "ego_state", "ts_mono_ns": int(t * NS), "ts_real_ns": None, "position": pos,
            "heading": {"value": heading, "std": 3.0, "valid": True},
            "speed": {"value": speed, "valid": True}, "odometer_m": x if odo is None else odo}


class BoxTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.tmp = tempfile.mkdtemp(prefix="chris-mapper-")
        cls.path = os.path.join(cls.tmp, "synth.rsmap")
        S.build(cls.path)

    def box(self, **cfg):
        return MapBox(dict(cfg, pack=self.path))

    def drive(self, box, x0, x1, y=2.0, v=14.0, dt=0.5, **kw):
        out, t, x = [], 100.0, x0
        while x < x1:
            m = box.on_ego_state(ego_state(t, x, y, speed=v, **kw))
            if m is not None:
                out.append(m)
            t += dt
            x += v * dt
        return out

    def test_follows_the_road_and_gives_the_context(self):
        out = self.drive(self.box(), 50, 2400)
        m = out[-1]
        self.assertTrue(m["available"])
        self.assertEqual((m["state"], m["road"]["name"], m["road"]["ref"]), ("good", "Main", "DK1"))
        self.assertEqual(m["speed_limit"]["limit_kmh"], 90)                       # not built-up, no tag: implicit
        self.assertTrue(all(o["road"]["name"] == "Main" for o in out[2:]), "left the main road")
        first = next(o for o in out if o["available"])
        self.assertEqual(first["speed_limit"]["limit_kmh"], 70)
        kinds = [f["kind"] for f in first["ahead"]["features"]]
        self.assertIn("traffic_signals", kinds)
        mt = m["match"]
        self.assertTrue(mt["matched"] and not mt["ambiguous"] and mt["confidence"] > 0.5)
        self.assertAlmostEqual(mt["bearing_deg"], 90.0, delta=1.0)               # the road's geometry, not the pose's
        self.assertLess(mt["distance_m"], 4.0)
        self.assertEqual(m["input"]["pose"], "ego_state")

    def test_it_estimates_no_position(self):
        """What comes out as `position` is the pose that went in; a pose 30 m off the road stays 30 m off."""
        box = self.box()
        out = self.drive(box, 50, 400, y=30.0, sigma=25.0)
        m = out[-1]
        lat, lon = S.ll(m["position"]["odometer_m"], 30.0)
        self.assertAlmostEqual(m["position"]["lat"], lat, places=6)
        self.assertGreater(m["match"]["distance_m"], 25.0)
        self.assertFalse(hasattr(box, "filter"))

    def test_states_follow_the_estimators_statement(self):
        box = self.box()
        m = box.on_ego_state(ego_state(1.0, 100, 0, valid=False, reason="no_absolute_source"))
        self.assertEqual((m["state"], m["available"], m["reason"]), ("no_source", False, "no_position"))
        m = box.on_ego_state(ego_state(2.0, 100, 0))
        self.assertEqual((m["state"], m["available"]), ("good", True))
        m = box.on_ego_state(ego_state(3.0, 114, 0, mode="coast"))
        self.assertEqual((m["state"], m["position"]["source"]), ("dead_reckoning", "dead_reckoning"))
        m = box.on_ego_state(ego_state(4.0, 128, 0, valid=False, reason="dead_reckoning_ran_out"))
        self.assertEqual((m["state"], m["reason"], m["match"]), ("unknown", "position_unknown", None))
        m = box.on_ego_state(ego_state(5.0, 142, 0, sigma=200.0))                 # a pose too uncertain to match
        self.assertEqual(m["state"], "unknown")

    def test_rate_and_standing(self):
        box = self.box(period_s=0.45)
        n = sum(1 for k in range(200) if box.on_ego_state(ego_state(10.0 + 0.05 * k, 100 + 0.7 * k, 0)) is not None)
        self.assertEqual(n, 20)                           # 10 s of poses at 20 Hz, a message at least 0.45 s apart
        c = box.counts["matches"]
        for k in range(40):
            box.on_ego_state(ego_state(30.0 + 0.5 * k, 240, 0, speed=0.0))
        self.assertEqual(box.counts["matches"], c)                                # standing with a match: kept

    def test_a_reanchored_frame_is_a_new_position(self):
        box = self.box()
        self.drive(box, 50, 300)
        self.assertEqual(box.match.edge.cls_name, "primary")
        m = box.on_ego_state(ego_state(500.0, 1500, 2000, frame=2))               # the estimator anchored anew, far away
        self.assertEqual(m["road"]["ref"], "A9")

    def test_a_synthetic_receiver_is_marked(self):
        box = self.box()
        es = ego_state(1.0, 100, 0)
        es["sources"] = {"gnss": {"state": "ok", "receiver": "synthetic-drive", "fix": "3d", "hacc_m": 5.0}}
        m = box.on_ego_state(es)
        self.assertEqual((m["input"]["synthetic"], m["input"]["gnss"]["source"], m["input"]["gnss"]["fix"]), (True, "synthetic-drive", "3d"))
        es = ego_state(2.0, 110, 0)
        es["sources"] = {"gnss": {"state": "ok", "receiver": "headunit", "fix": "3d"}}
        self.assertFalse(box.on_ego_state(es)["input"]["synthetic"])

    def test_no_pack(self):
        box = MapBox({"pack": os.path.join(self.tmp, "missing.rsmap")})
        m = box.on_ego_state(ego_state(1.0, 100, 0))
        self.assertEqual((m["available"], m["reason"], m["road"]), (False, "no_pack", None))
        self.assertEqual(box.health()["state"], "idle")

    def test_pack_given_as_an_object(self):
        box = MapBox(pack=Pack(self.path))
        self.assertTrue(self.drive(box, 50, 300)[-1]["available"])

    def test_as_a_process_behind_the_plugin_wire(self):
        p = subprocess.Popen([sys.executable, "-m", "chris_mapper", "--plugin", "--pack", self.path], stdin=subprocess.PIPE,
                             stdout=subprocess.PIPE, cwd=os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
        try:
            hello = json.loads(p.stdout.readline())
            self.assertEqual((hello["type"], hello["name"], hello["inputs"][0]["name"], hello["outputs"][0]["name"]),
                             ("plugin", "mapper", "ego_state", "map"))
            got = []
            for k in range(1, 8):
                p.stdin.write((json.dumps({"in": "ego_state", "msg": ego_state(10.0 + k, 100 + 14 * k, 0), "n": k}) + "\n").encode())
                p.stdin.flush()
                while True:
                    o = json.loads(p.stdout.readline())
                    got.append(o)
                    if o.get("done") == k:
                        break
            maps = [o["msg"] for o in got if o.get("out") == "map"]
            self.assertEqual(len(maps), 7)
            self.assertEqual(maps[-1]["road"]["name"], "Main")
            self.assertEqual(got[-1]["health"]["state"], "ok")
        finally:
            p.stdin.close()
            p.wait(5)


if __name__ == "__main__":
    unittest.main()
