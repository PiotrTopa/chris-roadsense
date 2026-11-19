"""The map as a measurement (`map.match` of mapd's matcher): the car is on a road. What is used is the map's
geometry (the matched road's line and bearing), never the position that was matched: cross-track distance to
the road = 0 and, on straights, heading = the road's bearing. Only an unambiguous, confident match while
moving; both rows robust, so a wrong road pulls with bounded force and shows in the innovations."""
import math

import numpy as np

from ..core import ego as E
from ..core.ego import AuxState
from ..core.registry import NS, Meas, Source, fin


class MapMatchSource(Source):
    id = "map"
    group = "map"
    rank = 0
    types = ("map",)
    stale_s = 3.0
    info = "map matcher: cross-track and bearing of the matched road"
    # the matched line is the road's centre line; the car drives in a lane beside it. That offset is not noise
    # (it is the same from one match to the next): a slowly changing state, so that a thousand matches do not
    # pretend to know the position across the road to centimetres
    aux = (AuxState("off", 0.0, 2.5, tau_s=120.0, unit="m", doc="the car's offset from the matched road's centre line (left positive)"),)
    min_conf = 0.5
    min_speed = 3.0
    cross_sigma_m = 1.0          # per match, about the lane offset state
    heading_sigma = math.radians(8.0)
    max_yaw_rate = 0.05

    def measure(self, msg, ctx):
        mt = msg.get("match") or {}
        mem = ctx.mem
        mem["matched"], mem["conf"] = mt.get("matched"), mt.get("confidence")
        ts = msg["ts_mono_ns"] / NS
        alive = [Meas("signal:map_matched", ts, bool(mt.get("matched")), role="signal")]
        integ = ctx.integ
        sp = ctx.speed_known()
        if not integ.anchored or sp is None or sp[0] < self.min_speed or not mt.get("matched") or mt.get("ambiguous") \
                or not fin(mt.get("confidence")) or mt["confidence"] < self.min_conf \
                or not fin(mt.get("lat")) or not fin(mt.get("lon")) or not fin(mt.get("bearing_deg")):
            return alive
        em, nm = E.to_local(integ.origin, mt["lat"], mt["lon"])
        beta = E.course_to_psi(mt["bearing_deg"])
        x = ctx.x
        # the road is a line: its direction is known modulo pi; take the branch nearer the heading
        d = (beta - float(x[E.PSI]) + math.pi) % (2.0 * math.pi) - math.pi
        if abs(d) > 0.5 * math.pi:
            d = d - math.copysign(math.pi, d)
        nx, ny = -math.sin(beta), math.cos(beta)
        ioff = self.ix["off"]
        i_c = np.array([E.PE, E.PN, ioff], dtype=np.intp)
        co = np.array([nx, ny, -1.0])
        out = alive + [Meas("map_cross", ts, nx * em + ny * nm, self.cross_sigma_m,
                            lambda x: (nx * x[E.PE] + ny * x[E.PN] - x[ioff], i_c, co), "map.cross", robust=True)]
        if abs(float(x[E.W])) < self.max_yaw_rate and abs(d) < math.radians(30.0):
            i_h = np.array([E.PSI], dtype=np.intp)
            one = np.array([1.0])
            out.append(Meas("map_heading", ts, float(x[E.PSI]) + d, self.heading_sigma,
                            lambda x: (x[E.PSI], i_h, one), "map.heading", robust=True))
        mem["used"] = mem.get("used", 0) + 1
        return out

    def report(self, ctx):
        m = ctx.mem
        return {"matched": m.get("matched"), "match_confidence": m.get("conf"), "used": m.get("used", 0)}
