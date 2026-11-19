"""A front radar added as a source (sources/radar.py; no change to the core): fused tracks against simulated
truth, the radar's own calibration, and isolation by vote with three independent speed sources."""
import math
import unittest

import numpy as np

from .simrun import Run
from roadsense_integrator import Integrator
from roadsense_integrator.sim import default_sensors
from roadsense_integrator.sim.radar import RadarSim

RADAR = {"sources": [{"kind": "radar", "id": "*"}]}


def lead_stats(run, t0=5.0, t1=1e9):
    rows = [(t, ld, tl) for t, ld, tl in run.leads if t0 <= t < t1]
    both = [(ld["dist_m"] - tl["x"], ld["v_rel_mps"] - (tl["vx"] - run.w.truth_at(t + run.w.t0)["v"])) for t, ld, tl in rows
            if ld["present"] and tl is not None]
    return {"n": len(rows), "truth": sum(1 for _, _, tl in rows if tl is not None),
            "tracked": len(both) / max(sum(1 for _, _, tl in rows if tl is not None), 1),
            "false": sum(1 for _, ld, tl in rows if ld["present"] and tl is None) / max(len(rows), 1),
            "rms": float(np.sqrt(np.mean([d * d for d, _ in both]))) if both else None,
            "rms_v": float(np.sqrt(np.mean([v * v for _, v in both]))) if both else None,
            "ids": len({ld["track_id"] for _, ld, _ in rows if ld["present"]})}


class Fusion(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        vis = dict(range_scale=1.10)                     # the camera's distance is 10 % long (focal length, pitch)
        cls.cam = Run(sensors=default_sensors(vision=vis), seed=1)
        cls.fus = Run(sensors=default_sensors(vision=vis) + [RadarSim(misalign_deg=1.0)], seed=1,
                      integ=Integrator(RADAR), keep_tracks=True)

    def test_fused_lead_is_right_where_the_camera_alone_is_10_percent_off(self):
        c, f = lead_stats(self.cam), lead_stats(self.fus)
        self.assertGreater(c["rms"], 2.0)
        self.assertLess(f["rms"], 0.5)
        self.assertLess(f["rms_v"], 0.5)
        self.assertGreater(f["tracked"], 0.99)
        self.assertLess(f["false"], 0.02)
        ld = self.fus.leads[-1][1]
        self.assertEqual((ld["source"], "radar_front" in ld["sources"]), ("radar+vision+ego", True))
        self.assertAlmostEqual(ld["vision_scale"], 1.0 / 1.10, delta=0.02)      # the camera's scale, learned
        self.assertEqual(ld["radar"]["sensor"], "radar_front")

    def test_radar_calibrates_itself_and_the_speed_scale(self):
        cal = self.fus.es["calibration"]
        self.assertAlmostEqual(math.degrees(abs(cal["radar_front"]["dyaw"]["value"])), 1.0, delta=0.2)
        self.assertTrue(cal["odometry"]["speed_scale"]["known"])
        self.assertLess(cal["odometry"]["speed_scale"]["std"], 0.002)
        self.assertEqual(self.fus.es["sources"]["radar_front"]["state"], "ok")
        self.assertGreater(self.fus.es["sources"]["radar_front"]["ego_fit_share"], 0.2)       # this world has few roadside objects

    def test_tracks_other_vehicles_and_marks_ghosts(self):
        """Every vehicle in the radar's view has a confirmed track near it most of the time; double-bounce
        ghosts are marked and never the lead."""
        w = self.fus.w
        seen = total = ghost_lead = ghosts = 0
        for t, tr in self.fus.tracks[100::5]:
            conf = [k for k in tr["tracks"] if k["state"] != "tentative"]
            ghosts += sum(1 for k in conf if "ghost" in k["flags"])
            for o in w.targets_at(t + w.t0):
                if 3.0 < o["x"] < 120.0 and abs(math.atan2(o["y"], o["x"] - 2.2)) < math.radians(45):
                    total += 1
                    seen += any(abs(k["x_m"] - o["x"]) < 3.0 and abs(k["y_m"] - o["y"]) < 1.5 for k in conf)
        self.assertGreater(seen / total, 0.9)
        self.assertGreater(ghosts, 20)
        for t, ld, tl in self.fus.leads:
            if ld["present"] and tl is not None and ld["dist_m"] > 1.7 * tl["x"]:
                ghost_lead += 1
        self.assertEqual(ghost_lead, 0)


class Vote(unittest.TestCase):
    def test_knocked_camera_is_isolated_by_vote(self):
        sens = default_sensors(vision=dict(knock=(60.0, 1e9, 0.2))) + [RadarSim()]
        states = []
        r = Run(sensors=sens, seed=2, integ=Integrator(RADAR),
                on_frame=lambda run, t, es, tr, ld: states.append((t, es["sources"]["vision"], ld)))
        first = next(t for t, v, _ in states if v["state"] == "fault")
        self.assertLess(first, 71.0)                                 # the car stands until 64 s; moving at 3 m/s by 66 s
        v = states[-1][1]
        self.assertEqual((v["state"], v["isolation"]), ("fault", "vote"))
        # the radar keeps the lead while the camera is out; nothing the camera says is used
        late = [ld for t, _, ld in states if t > 80.0]
        self.assertTrue(all(ld["valid"] for ld in late))
        self.assertTrue(all("vision" not in ld["source"] for ld in late if ld["present"]))
        st = lead_stats(r, 135.0)
        self.assertGreater(st["tracked"], 0.9)
        self.assertLess(st["rms"], 0.6)

    def test_wrong_vehicle_speed_is_isolated_by_vote(self):
        """The speed signal reads 30 % low for 40 s. Rank alone would fault the camera (the lower rank); the radar
        and the camera together outvote the vehicle."""
        sens = default_sensors(vehicle=dict(speed_fault=(100.0, 140.0, 0.7))) + [RadarSim()]
        seen = []
        r = Run(sensors=sens, seed=3, integ=Integrator(RADAR),
                on_frame=lambda run, t, es, tr, ld: seen.append((t, {k: (s["state"], s.get("isolation")) for k, s in es["sources"].items()}, es["speed"])))
        mid = [s for t, s, _ in seen if 114.0 <= t < 138.0]
        self.assertTrue(all(s["vision"][0] != "fault" for t, s, _ in seen if 100.0 <= t < 114.0))   # nobody blamed by rank meanwhile
        self.assertTrue(all(s["vehicle"] == ("fault", "vote") for s in mid))
        self.assertTrue(all(s["vision"][0] != "fault" and s["radar_front"][0] != "fault" for s in mid))
        err = [abs(x["dv"]) for x in r.rows if 114.0 <= x["t"] < 138.0 and "dv" in x]
        self.assertLess(np.percentile(err, 95), 1.0)                 # the estimate stays with the true speed
        self.assertEqual(seen[-1][1]["vehicle"][0], "ok")           # and the vehicle comes back when it agrees again
        # without the radar the same fault costs the camera (rank), as documented
        r2 = Run(sensors=default_sensors(vehicle=dict(speed_fault=(100.0, 140.0, 0.7))), seed=3,
                 on_frame=lambda run, t, es, tr, ld: seen.append(("b", t, es["sources"]["vision"].get("isolation"))))
        self.assertIn("rank", {x[2] for x in seen if x[0] == "b"})

    def test_radar_blackout_and_camera_only_fallback(self):
        sens = default_sensors() + [RadarSim(blackout=[(80.0, 110.0)])]
        src = []
        r = Run(sensors=sens, seed=4, integ=Integrator(RADAR),
                on_frame=lambda run, t, es, tr, ld: src.append((t, ld["source"], ld["present"], ld["track_id"], es["sources"]["radar_front"]["state"])))
        dark = [x for x in src if 82.0 <= x[0] < 109.0]
        self.assertTrue(all(x[4] == "stale" for x in dark))
        self.assertTrue(all(x[2] and x[1] == "vision+ego" for x in dark))
        self.assertEqual(len({x[3] for x in src if 70.0 <= x[0] < 118.0}), 1)        # the same track through the blackout
        self.assertEqual(src[-1][1], "radar+vision+ego")


if __name__ == "__main__":
    unittest.main()
