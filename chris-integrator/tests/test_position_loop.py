"""The position loop (lane L47; docs/DESIGN.md sections 3a, 4a, 15a), against simulated truth: the receiver's
error model and its own motion checked against the car's, the delay measured from that motion, the heading across
a gap of all input, a position placed while the car stands, the map match as a hypothesis with a share, and what
is left of the position when the receiver lies."""
import math
import unittest

import numpy as np

from .simrun import Run
from chris_integrator import Integrator
from chris_integrator.core import ego as E
from chris_integrator.sim import GnssSim, MapSim, VehicleSim, VisionSim, World, default_sensors
from chris_integrator.sim.faults import Fault, apply
from chris_integrator.sim.world import SCENARIOS, geo

# a receiver like the reference car's on a good day: 10 m of slowly varying error, smooth from fix to fix, behind a
# claimed accuracy of 4 m; fixes 2.2 s old
CAR = dict(lag=2.2, sigma_gm=10.0, tau_gm=15.0, noise_smooth_s=5.0, sigma_white=0.7, hacc=4.0)
_RUNS = {}


def run(key, **kw):
    if key not in _RUNS:
        _RUNS[key] = Run(integ=Integrator(), **kw)
    return _RUNS[key]


def err(x):
    return math.hypot(x["de"], x["dn"])


def rows(r, a=0.0, b=1e9):
    return [x for x in r.rows if a <= x["t"] < b and "de" in x]


class ReceiverModel(unittest.TestCase):
    def test_stated_uncertainty_covers_a_receiver_like_the_cars(self):
        """10 m of slowly varying error behind a claimed 4 m. Before L47 that error was fused as white noise and the
        stated uncertainty was a third of the error; now the level is measured against the matched road and the
        error is a state."""
        r = run("car", sensors=default_sensors(gnss=CAR), seed=3)
        x = rows(r, 40.0)
        nees = float(np.mean([v["nees_pos"] for v in x]))
        self.assertLess(nees, 3.0)                       # 2 degrees of freedom: expected 2; never optimistic
        self.assertGreater(nees, 0.15)                   # ... and not a statement of "anywhere"
        self.assertGreater(float(np.mean([v["nees_pos"] <= 5.99 for v in x])), 0.93)
        self.assertLess(float(np.sqrt(np.mean([err(v) ** 2 for v in x]))), 8.0)
        cal = r.es["calibration"]["gnss"]
        self.assertTrue(cal["level_measured"])
        self.assertGreater(cal["level"], 1.5)            # the receiver claims 4 m and is 10 m off: about 2.5
        self.assertLess(cal["level"], 4.5)

    def test_without_a_map_the_position_is_as_uncertain_as_the_receiver(self):
        r = run("car_nomap", sensors=default_sensors(gnss=CAR, map=dict(hz=0.001)), seed=3)
        x = rows(r, 40.0)
        self.assertGreater(float(np.median([v["hacc"] for v in x])), 5.0)
        self.assertLess(float(np.mean([v["nees_pos"] for v in x])), 4.5)
        self.assertFalse(r.es["calibration"]["gnss"]["level_measured"])       # nothing to measure it against: the prior

    def test_the_delay_is_measured_from_the_receivers_own_motion(self):
        r = run("car", sensors=default_sensors(gnss=CAR), seed=3)
        lag = r.es["sources"]["gnss"]["delay_by_motion"]         # (the last such measurement: lag, its sigma, steps)
        self.assertIsNotNone(lag)
        self.assertLess(abs(lag[0] - 2.2), 3.0 * max(lag[1], 0.4))
        tau, std = r.cal("gnss", "tau")
        self.assertAlmostEqual(tau, 2.2, delta=0.6)
        self.assertLess(std, 0.5)

    def test_a_smoothed_track_is_found_out(self):
        """Few satellites: the receiver reports a track that trails the car by another 6 s of smoothing (78 m at
        town speed). Its delay as its own motion shows it is found, the position does not trail with it, and the
        receiver is named as not moving as the car does when it crawls and catches up."""
        seen = set()
        r = Run(sensors=default_sensors(gnss=dict(CAR, lag=2.0, sigma_gm=8.0, smooth_s=6.0, hacc=6.0, sats=4)), seed=4, integ=Integrator(),
                on_frame=lambda run_, t, es, tr, ld: seen.add(es["sources"]["gnss"].get("delay_by_motion") is not None))
        tau, std = r.cal("gnss", "tau")
        self.assertGreater(tau, 4.5)                     # (a first-order lag is not a pure delay: about 6 of the 8 s show)
        x = rows(r, 60.0)
        self.assertLess(float(np.sqrt(np.mean([err(v) ** 2 for v in x]))), 12.0)
        self.assertLess(float(np.mean([v["nees_pos"] for v in x])), 4.0)
        self.assertIn(True, seen)

    def test_a_jump_is_held_back(self):
        """A multipath excursion of 29 m for 30 s. The step is seen in the receiver's own motion (no map is asked),
        held back from its fixes, and the position stays on dead reckoning and the map."""
        r = Run(sensors=default_sensors(gnss=dict(CAR, sigma_gm=3.0, jump=(100.0, 130.0, 25.0, 15.0))), seed=5, integ=Integrator())
        during = rows(r, 101.0, 130.0)
        self.assertLess(max(err(v) for v in during), 14.0)
        self.assertLess(float(np.median([err(v) for v in during])), 6.0)
        self.assertGreaterEqual(r.es["sources"]["gnss"]["jumps"], 2)         # out and back
        after = rows(r, 150.0)
        self.assertLess(float(np.median([err(v) for v in after])), 6.0)

    def test_poor_reception_all_at_once(self):
        r = Run(sensors=default_sensors(gnss=dict(CAR, lag=2.0, sigma_gm=12.0, sigma_white=1.0, hacc=7.0, sats=4, gap_p=0.03, jump_p=0.01,
                                                  repeat_p=0.03, smooth_s=5.0)), seed=6, integ=Integrator())
        x = rows(r, 60.0)
        self.assertGreater(len(x), 1500)
        # (the receiver itself trails by up to 65 m here; what the position owes is to say how well it is known)
        self.assertLess(float(np.sqrt(np.mean([err(v) ** 2 for v in x]))), 45.0)
        self.assertLess(float(np.mean([v["nees_pos"] for v in x])), 4.0)
        self.assertGreater(float(np.mean([v["pos_valid"] for v in x])), 0.95)


class Gaps(unittest.TestCase):
    def test_a_gap_of_all_input_with_a_turn_inside(self):
        """Nothing arrives for 7 s (the host stalled) while the road turns 86 degrees. Afterwards the heading is
        not known and is not guessed: the position is stated wide, put on the next fix, and the heading comes
        from the receiver's own track once that was driven after the gap. Before L47 the first row after the
        gap was the road's bearing, which is a line: it locked the heading the other way round."""
        w = World(seed=2, scenario="town")
        bend = [i * w.dt for i in range(len(w.s)) if 300.0 <= w.s[i] <= 390.0]
        g0, g1 = bend[0] + 0.5, bend[0] + 7.5
        r = Run(world=w, sensors=default_sensors(gnss=CAR), integ=Integrator(),
                order=lambda msgs: [(a, m) for a, m in msgs if not (w.t0 + g0 <= a < w.t0 + g1)])
        self.assertEqual(r.es["reanchors"], 1)
        first = rows(r, g1, g1 + 2.0)
        # honest while it is not known: the error is inside 3 of its own stated sigmas, or the position is not offered
        self.assertTrue(all(not v["pos_valid"] or err(v) <= 3.0 * v["hacc"] for v in first))
        self.assertTrue(all(abs(v["dpsi"]) < math.radians(15.0) or v["spsi"] > math.radians(45.0) for v in first))
        soon = rows(r, g1 + 5.0, g1 + 10.0)
        self.assertLess(max(abs(v["dpsi"]) for v in soon), math.radians(8.0))
        self.assertLess(max(err(v) for v in soon), 20.0)
        later = rows(r, g1 + 20.0, g1 + 60.0)
        self.assertLess(float(np.median([err(v) for v in later])), 6.0)

    def test_the_position_is_placed_while_the_car_stands(self):
        """A cold start in a car park: 40 s standing, then driving off. The position is there before the car moves
        (heading unknown, said so), and the heading follows within a few seconds of driving."""
        sc = dict(T=100.0, speed=[(0, 0.0), (40, 0.0), (45, 12.0), (100, 12.0)], curv=SCENARIOS["town"]["curv"])
        out = []
        r = Run(world=World(seed=4, scenario=sc), sensors=default_sensors(gnss=dict(CAR, speed_lag=2.0)), integ=Integrator(),
                on_frame=lambda run_, t, es, tr, ld: out.append((t, es["position"]["valid"], es["heading"]["valid"], es["position"]["hacc_m"])))
        t_pos = next(t for t, p, h, _ in out if p)
        t_head = next(t for t, p, h, _ in out if h)
        self.assertLess(t_pos, 25.0)
        self.assertGreater(t_head, 40.0)
        self.assertLess(t_head, 50.0)
        standing = rows(r, 30.0, 40.0)
        self.assertTrue(all(err(v) <= 3.5 * v["hacc"] for v in standing))
        self.assertGreaterEqual(r.es["sources"]["gnss"].get("anchored"), True)
        moving = rows(r, 60.0)
        self.assertLess(float(np.median([err(v) for v in moving])), 9.0)


class MapAlt(MapSim):
    """The map box's match with shares: a parallel candidate `alt_m` beside the winner holding `share`, and / or a
    candidate of another direction holding `other`."""

    def __init__(self, alt_m=8.0, share=0.4, other=0.0, **kw):
        super().__init__(**kw)
        self.alt_m, self.share, self.other = alt_m, share, other

    def generate(self, w):
        for t, m in super().generate(w):
            mt = m["match"]
            tr = w.truth_at(m["ts_mono_ns"] / 1e9)
            x, y, _ = w._road_xy(tr["s"], self.centre_offset + self.alt_m)
            lat, lon = geo(x, y)
            alts = [{"lat": lat, "lon": lon, "bearing_deg": mt["bearing_deg"], "weight": self.share}] if self.share else []
            if self.other:
                alts.append({"lat": mt["lat"], "lon": mt["lon"], "bearing_deg": (mt["bearing_deg"] + 90.0) % 360.0, "weight": self.other})
            mt.update(weight=1.0 - self.share - self.other, fit=1.0, bearing_span_deg=0.0, alternatives=alts)
            yield t, m


def cross_sigma(I):
    x, P = I.x_now, I.P_now
    nx, ny = -math.sin(x[E.PSI]), math.cos(x[E.PSI])
    return math.sqrt(nx * nx * P[E.PE, E.PE] + 2.0 * nx * ny * P[E.PE, E.PN] + ny * ny * P[E.PN, E.PN])


class MapHypotheses(unittest.TestCase):
    def loop(self, **kw):
        return Run(sensors=[VehicleSim(), VisionSim(), GnssSim(**CAR), MapAlt(**kw)], seed=5, integ=Integrator())

    def test_one_road_pins_the_position_across_it(self):
        r = self.loop(share=0.0)
        self.assertLess(cross_sigma(r.I), 3.0)                               # the lane offset state: 2.5 m
        self.assertIn(r.es["sources"]["map"]["spread_m"], (None, 0.0))

    def test_a_parallel_candidate_keeps_the_position_across_the_road_uncertain(self):
        """A twin road 8 m beside the winner with 40 % of the share: hundreds of matches later the position across
        the road is still as uncertain as the choice between the two (their spread, 3.9 m), and it sits between
        them by their shares. Before L47 such a match was either fused with 1 m or not at all."""
        r = self.loop(share=0.4)
        self.assertGreater(r.es["sources"]["map"]["used"], 200)
        self.assertAlmostEqual(r.es["sources"]["map"]["spread_m"], 3.9, delta=0.2)
        self.assertGreater(cross_sigma(r.I), 3.7)
        self.assertLess(cross_sigma(r.I), 6.0)
        x = rows(r, 60.0)
        self.assertLess(max(abs(v["cross"]) for v in x), 3.0 * 4.0)
        less = self.loop(share=0.1)
        self.assertLess(cross_sigma(less.I), cross_sigma(r.I))
        self.assertGreater(cross_sigma(less.I), cross_sigma(self.loop(share=0.0).I))

    def test_candidates_of_another_direction_take_the_row_away(self):
        r = self.loop(share=0.0, other=0.6)
        self.assertEqual(r.es["sources"]["map"]["used"], 0)
        self.assertGreater(cross_sigma(r.I), 4.0)                            # the receiver's, not the road's

    def test_a_match_without_shares_is_used_by_the_rule_of_before(self):
        r = Run(sensors=[VehicleSim(), VisionSim(), GnssSim(**CAR), MapSim()], seed=5, integ=Integrator())
        self.assertGreater(r.es["sources"]["map"]["used"], 200)
        self.assertLess(cross_sigma(r.I), 3.0)


class ReceiverFaults(unittest.TestCase):
    """What is left of the position when the one absolute source lies (docs/FAULTS.md, "The receiver distrusted")."""

    def faulted(self, *faults, seed=6):
        w = World(seed=seed, scenario="town")
        I = Integrator()
        out = []
        for m in apply([m for _, m in w.messages(default_sensors(gnss=CAR))], list(faults)):
            if m["type"] != "road":
                I.on_message(m)
                continue
            es, tr, ld = I.frame(m)
            T = m["ts_mono_ns"] / 1e9
            sm = I.ekf.at(T)
            if I.anchored and sm is not None:
                tru = w.truth_at(T)
                oe, on = E.to_local(I.origin, *geo(0.0, 0.0))
                out.append((T - w.t0, math.hypot(sm[0][E.PE] - tru["pe"] - oe, sm[0][E.PN] - tru["pn"] - on), es["position"]["hacc_m"],
                            es["position"]["valid"], es["sources"]["gnss"]))
        return out

    def test_mirrored_coordinates_are_not_followed(self):
        """Latitude and longitude swapped for 55 s: the receiver's steps turn the wrong way against the car's yaw,
        each is held back, and the position goes on by dead reckoning and the map."""
        out = self.faulted(Fault("gnss.pos", "frame", 95.0, 150.0))
        during = [o for o in out if 96.0 <= o[0] < 150.0]
        self.assertLess(max(o[1] for o in during), 25.0)
        self.assertGreater(max(o[4]["held_back_m"] for o in during), 100.0)
        self.assertIn("suspect", {o[4]["state"] for o in during})
        after = [o for o in out if 200.0 <= o[0]]
        self.assertLess(float(np.median([o[1] for o in after])), 15.0)

    def test_a_position_jump_of_30_m(self):
        out = self.faulted(Fault("gnss.pos", "bias", 95.0, 150.0, e=30.0))
        during = [o for o in out if 96.0 <= o[0] < 150.0]
        self.assertLess(float(np.median([o[1] for o in during])), 8.0)
        self.assertLess(max(o[1] for o in during), 20.0)

    def test_a_slow_drift_is_followed_and_said_so(self):
        """1 m/s for 55 s cannot be told from the car's own motion on this class of receiver: along the road the
        position follows it (across it the map holds). What the design owes is the stated uncertainty."""
        out = self.faulted(Fault("gnss.pos", "drift", 95.0, 150.0, e=1.0))
        during = [o for o in out if 110.0 <= o[0] < 150.0]
        self.assertLess(max(o[1] for o in during), 45.0)
        self.assertLess(float(np.mean([o[1] > 15.0 and o[1] > 3.0 * o[2] for o in during if o[3]])), 0.35)
        after = [o for o in out if 175.0 <= o[0]]
        self.assertLess(float(np.median([o[1] for o in after])), 15.0)


class MotionBetweenTimes(unittest.TestCase):
    def test_yaw_and_distance_between_two_times_are_the_cars_own(self):
        r = run("car", sensors=default_sensors(gnss=CAR), seed=3)
        w, I = r.w, r.I
        t1 = w.t0 + w.T - 1.0
        for back in (5.0, 20.0, 40.0):
            a, b = w.truth_at(t1 - back), w.truth_at(t1)
            self.assertAlmostEqual(I.yaw_between(t1 - back, t1), b["psi"] - a["psi"], delta=0.03)
            self.assertAlmostEqual(I.dist_between(t1 - back, t1), b["s"] - a["s"], delta=0.02 * (b["s"] - a["s"]) + 0.5)
        self.assertIsNone(I.yaw_between(w.t0 - 500.0, t1))                   # outside what is remembered

    def test_a_gap_ends_the_stretch(self):
        w = World(seed=2, scenario="town")
        r = Run(world=w, sensors=default_sensors(gnss=CAR), integ=Integrator(),
                order=lambda msgs: [(a, m) for a, m in msgs if not (w.t0 + 100.0 <= a < w.t0 + 104.0)])
        self.assertIsNone(r.I.yaw_between(w.t0 + 98.0, w.t0 + 106.0))
        self.assertIsNone(r.I.dist_between(w.t0 + 98.0, w.t0 + 106.0))
        self.assertIsNotNone(r.I.dist_between(w.t0 + 106.0, w.t0 + 110.0))


if __name__ == "__main__":
    unittest.main()
