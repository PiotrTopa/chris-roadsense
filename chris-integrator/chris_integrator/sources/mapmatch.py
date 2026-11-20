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
    # Cross-check against the absolute source, raw against raw (detector `map.conflict`): the signed distance of
    # each new receiver fix from the matched road's line, median over the last fixes. A receiver that scatters
    # 10-15 m gives a median within a few metres of the road it is on; a matcher on a parallel road gives the
    # distance between the roads. The match is made from the position, the receiver is the absolute source: in a
    # conflict the map's rows are the ones set aside. (A fix is seconds old: along a straight road that does not
    # change its distance from the road's line; in bends the check pauses.)
    conflict_n, conflict_min_n = 8, 6
    conflict_m = 12.0            # ... beyond this (and beyond 2.5 x the fixes' own stated scatter of the median)
    conflict_clear = 0.6

    def measure(self, msg, ctx):
        mt = msg.get("match") or {}
        mem = ctx.mem
        mem["matched"], mem["conf"] = mt.get("matched"), mt.get("confidence")
        ts = msg["ts_mono_ns"] / NS
        alive = [Meas("signal:map_matched", ts, bool(mt.get("matched")), role="signal")]
        integ = ctx.integ
        sp = ctx.speed_known()
        settled = integ.position_settled(ts)
        mem["held"] = None if settled else "position not settled"
        if not settled or sp is None or sp[0] < self.min_speed or not mt.get("matched") or mt.get("ambiguous") \
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
        if self._conflict(ctx, mem, ts, em, nm, nx, ny, abs(float(x[E.W])) < self.max_yaw_rate) and integ.gates("map.conflict"):
            mem["held"] = "conflicts with the receiver"
            return alive
        out = alive + [Meas("map_cross", ts, nx * em + ny * nm, self.cross_sigma_m,
                            lambda x: (nx * x[E.PE] + ny * x[E.PN] - x[ioff], i_c, co), "map.cross", robust=True, local=True)]
        if abs(float(x[E.W])) < self.max_yaw_rate and abs(d) < math.radians(30.0):
            i_h = np.array([E.PSI], dtype=np.intp)
            one = np.array([1.0])
            out.append(Meas("map_heading", ts, float(x[E.PSI]) + d, self.heading_sigma,
                            lambda x: (x[E.PSI], i_h, one), "map.heading", robust=True, local=not ctx.cfg["gnss_absolute"]))
        mem["used"] = mem.get("used", 0) + 1
        return out

    def _conflict(self, ctx, mem, ts, em, nm, nx, ny, straight):
        fix = getattr(ctx.integ, "abs_latest", None)
        q = mem.setdefault("cq", [])
        if fix is not None and straight and fix[0] != mem.get("cfix") and ts - fix[0] < 3.0:
            mem["cfix"] = fix[0]
            q.append((nx * (fix[1] - em) + ny * (fix[2] - nm), fix[3]))
            del q[:-self.conflict_n]
        if fix is None or ts - fix[0] > 15.0:
            del q[:]                                 # no receiver: nothing to check against
        key = ("map", "cross", "conflict")
        if len(q) >= self.conflict_min_n:
            d = sorted(v for v, _ in q)
            med = 0.5 * (d[(len(d) - 1) // 2] + d[len(d) // 2])
            sg = sorted(s for _, s in q)[len(q) // 2]
            lim = max(self.conflict_m, 2.5 * 1.25 * sg / math.sqrt(len(q)))
            mem["conflict_peak"] = max(mem.get("conflict_peak", 0.0), abs(med) / lim)
            mem["conflict_m"] = round(med, 1)
            if abs(med) > (self.conflict_clear * lim if mem.get("conflict") else lim):
                mem["conflict"] = True
                ctx.integ.fdi._set(*key, "warn", ts, "the matched road lies %.0f m %s of the receiver's last %d fixes (median): a "
                                   "matcher on a parallel road, or a receiver offset; the map's rows are %s"
                                   % (abs(med), "right" if med > 0 else "left", len(q),
                                      "set aside" if ctx.integ.gates("map.conflict") else "still used (detector map.conflict in report mode)"))
                return True
        if mem.get("conflict") and len(q) >= self.conflict_min_n or not q:
            mem["conflict"] = False
            ctx.integ.fdi.findings.pop(key, None)
        return bool(mem.get("conflict"))

    def report(self, ctx):
        m = ctx.mem
        return {"matched": m.get("matched"), "match_confidence": m.get("conf"), "used": m.get("used", 0), "held": m.get("held"),
                "receiver_offset_m": m.get("conflict_m"), "conflict_peak": round(m.get("conflict_peak", 0.0), 2),
                "suspect_reason": ("the matched road lies %.0f m beside the receiver's recent fixes (map rows set aside)"
                                   % abs(m.get("conflict_m") or 0.0)) if m.get("held") == "conflicts with the receiver" else None}
