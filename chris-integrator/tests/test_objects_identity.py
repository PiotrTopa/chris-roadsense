"""The id of a physical object survives the loss of one of its sensors (core/objects.py, sources/camera_lead.py):
the camera loses its lead for longer than it may hold a designation on a radar's evidence alone, the radar
keeps the vehicle, the camera re-acquires it, and the lead keeps one track id throughout."""
import unittest

from .simrun import Run
from chris_integrator import Integrator
from chris_integrator.sim import default_sensors
from chris_integrator.sim.radar import RadarSim

RADAR = {"sources": [{"kind": "radar", "id": "*"}]}
# the lead's own vehicle is ahead in all three, on straight road (moving, standing at a queue, moving); the sharp
# bends of the simulated town (25-33 s, 74-84 s) are left out of the spans: there the simulator's plan is placed by
# arc length and a radar-only lead may leave the corridor, which is not what this test is about
BLANK = [(36.0, 43.0), (54.0, 61.0), (88.0, 95.0)]
SPANS = [(34.0, 73.0), (86.0, 117.0)]


def _blank_leads(msgs, t0):
    out = []
    for t, m in msgs:
        if m.get("type") == "road" and any(a <= m["ts_mono_ns"] / 1e9 - t0 < b for a, b in BLANK):
            m = dict(m, leads=[{"prob": 0.01, "x": [80.0], "y": [0.0], "v": [0.0], "a": [0.0], "x_std": [8.0],
                                "y_std": [1.0], "v_std": [3.0], "a_std": [1.0]}])
        out.append((t, m))
    return out


class IdentityThroughCameraLoss(unittest.TestCase):
    def _run(self, range_scale):
        from chris_integrator.sim import World
        w = World(seed=3)
        src = []
        Run(world=w, sensors=default_sensors(vision=dict(range_scale=range_scale)) + [RadarSim(seed=307)],
            integ=Integrator(RADAR), order=lambda msgs: _blank_leads(msgs, w.t0),
            on_frame=lambda run, t, es, tr, ld: src.append((t, ld["present"], ld["track_id"], ld["source"])))
        return src

    def test_same_id_when_the_camera_comes_back(self):
        for scale in (1.0, 1.10):                       # the camera's distance right, and 10 % long
            src = self._run(scale)
            for a, b in SPANS:
                span = [x for x in src if a <= x[0] < b]
                self.assertTrue(all(x[1] for x in span), (scale, a))
                self.assertEqual(len({x[2] for x in span}), 1, (scale, a, sorted({x[2] for x in span})))
            dark = [x for x in src if any(a + 6.0 <= x[0] < b for a, b in BLANK)]
            self.assertTrue(dark and all("vision" not in x[3].split("+") for x in dark))    # the radar alone, said so


if __name__ == "__main__":
    unittest.main()
