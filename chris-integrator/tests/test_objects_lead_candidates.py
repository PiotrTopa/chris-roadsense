"""Which tracks may be the lead without a camera's designation (core/objects.py): a range sensor's track is a
lead candidate only when it has been seen moving (a significant speed held for a while, not one excursion of a
weakly observed velocity) or a camera's road model called it a vehicle. Stationary-never-moved returns, clutter
and ghosts are not, on the camera's plan or on the ego arc."""
import math
import unittest

import numpy as np

from .simrun import Run
from chris_integrator import Integrator
from chris_integrator.core.objects import ObjectLayer, Track
from chris_integrator.sim import default_sensors
from chris_integrator.sim.radar import RadarSim

RADAR = {"sources": [{"kind": "radar", "id": "*"}]}


def _layer():
    L = ObjectLayer()
    L._ve = 10.0
    return L


def _track(L, x, P, src="radar_front", t=0.0):
    tr = Track(L.new_id(), x, P, t, src, 0.999)
    tr.state, tr.hits = "confirmed", 50
    L.tracks.append(tr)
    return tr


class MovedFlag(unittest.TestCase):
    def test_one_excursion_of_a_weak_velocity_is_not_motion(self):
        """A stationary return whose lateral velocity is barely observed (radar: only through the azimuth's rate)
        reads 2.6 m/s for a moment: it has not moved."""
        L = _layer()
        tr = _track(L, [40.0, 5.0, 0.2, 2.6, 0.0], np.diag([1.0, 1.0, 0.09, 4.0, 1.0]))
        for k in range(20):
            T = 0.05 * k
            tr.last_seen["radar_front"] = T
            L._manage(T)
        self.assertFalse(tr.moved)

    def test_a_held_significant_speed_is_motion(self):
        L = _layer()
        tr = _track(L, [40.0, 0.0, 8.0, 0.0, 0.0], np.diag([1.0, 1.0, 0.04, 0.04, 1.0]))
        T = 0.0
        for k in range(7):
            T = 0.05 * k
            tr.last_seen["radar_front"] = T
            L._manage(T)
        self.assertFalse(tr.moved)                     # 0.3 s: not yet
        for k in range(7, 14):
            T = 0.05 * k
            tr.last_seen["radar_front"] = T
            L._manage(T)
        self.assertTrue(tr.moved)


class Candidates(unittest.TestCase):
    def test_stationary_never_moved_is_not_the_lead_unless_a_camera_called_it_a_vehicle(self):
        L = _layer()
        P = np.diag([0.1, 0.1, 0.05, 0.05, 0.1])
        st = _track(L, [30.0, 0.2, 1.0, 0.0, 0.0], P)          # on the path, 1 m/s (between "stationary" and "moved")
        self.assertIsNone(L._select_lead((10.0, 0.0, 0.1, False)))
        st.mem["vehicle"] = True                                 # a camera designated it once
        self.assertIs(L._select_lead((10.0, 0.0, 0.1, False)), st)
        st.mem.pop("vehicle")
        st.moved = True
        self.assertIs(L._select_lead((10.0, 0.0, 0.1, False)), st)


def _no_camera_lead(msgs):
    """The camera's road model without its lead and its plan: the path is the ego arc, every lead is a range
    sensor's."""
    out = []
    for t, m in msgs:
        if m.get("type") == "road":
            m = {k: v for k, v in m.items() if k not in ("leads", "plan")}
        out.append((t, m))
    return out


class RadarOnlyOnTheArc(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.r = Run(sensors=default_sensors() + [RadarSim(misalign_deg=1.0, seed=107)], seed=1, integ=Integrator(RADAR),
                    keep_tracks=True, order=_no_camera_lead)

    def test_no_lead_on_a_roadside_object_and_the_vehicle_is_the_lead(self):
        w = self.r.w
        on_static = on_true = n_true = 0
        for t, ld, tl in self.r.leads:
            if t < 10.0:
                continue
            if tl is not None and tl["x"] < 60.0:
                n_true += 1
                on_true += ld["present"] and abs(ld["dist_m"] - tl["x"]) < 3.0
            if ld["present"]:
                x, y = ld["dist_m"], -ld["lateral_m"]
                if any(abs(x - o["x"]) < 3.0 and abs(y - o["y"]) < 2.0 for o in w.statics_at(t + w.t0)) and \
                        not any(abs(x - o["x"]) < 4.0 and abs(y - o["y"]) < 2.5 for o in w.targets_at(t + w.t0)):
                    on_static += 1
        self.assertEqual(on_static, 0)
        self.assertGreater(on_true / max(n_true, 1), 0.8)


if __name__ == "__main__":
    unittest.main()
