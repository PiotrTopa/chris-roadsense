"""The map as a measurement (`map.match` of the map box's matcher): the car is on a road. What is used is the map's
geometry (the matched road's line and bearing), never the position that was matched: cross-track distance to
the road = 0 and, on straights, heading = the road's bearing. Both rows robust, so a wrong road pulls with
bounded force and shows in the innovations.

The loop (L47; docs/DESIGN.md section 15a). The match is made from the estimate these rows then correct. What
keeps that from being a position confirming itself:

* Only the road's line comes back, and only across it. Along the road the rows say nothing, except where the
  road turns: then the new piece's line constrains what was along-track before the turn, which is real
  information (the turn is where the gyro says it is). In a bend the bearing at the matched point is only as
  good as the position along the road: the heading row's sigma grows with the road's change of direction
  around the match, and it is not used where the candidates disagree about the direction.
* Which road it is, is a hypothesis, and it stays one. The matcher reports the other candidates' lines with
  their shares (`match.alternatives`, `match.weight`). The row's target is the share-weighted line of the
  winner and of the candidates parallel to it, and the spread of those lines is the uncertainty of a state
  (`amb`, unit variance, slow) that these rows carry and never correct (a consider state): a thousand matches
  beside a parallel road 8 m away cannot make the position across the road more certain than the choice
  between the two roads is. Candidates that are not parallel (a crossing road, the next piece after a corner)
  take weight from the row instead (their share is the probability that this line does not constrain the car
  at all). Before L47 an unambiguous match (share >= 0.7) was fused with 1 m and an ambiguous one not at all;
  the pose then sat on the winner's line, which made the winner fit better.
* The car's place on the road (lane offset, `off`) is a slow state the rows carry: a thousand matches must not
  pretend to know the position across one road to centimetres either.
* Raw against raw (detector `map.conflict`): the receiver's recent fixes against the matched line, and against
  the parallel candidates' lines. Fixes that lie beside every line the matcher offers mean one of two things:
  the receiver is off (its slowly varying error is 10-17 m on the reference car with few satellites, 5-9 m
  with many), or the matcher is on a wrong road. Which, is a matter of odds: how likely such an offset is for
  this receiver's present error against how likely a wrong match is at all. Below even odds the map's rows
  stay, and "the receiver may be the one that is right" is one more candidate line in the spread above (so
  the position across the road is stated as uncertain as that); above them the map's rows are set aside.
  Before L47 the limit was 12 m whatever the receiver: on the reference car that set the map aside for 16-21 %
  of healthy driving, and in simulation it took a correct map away from a receiver 30 m off.

The same raw distances, on straights with one confident road, are how the receiver's slowly varying error is
measured (`GnssSource.offer_cross`)."""
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
    info = "map matcher: cross-track and bearing of the matched road; which road is a hypothesis with a share"
    # the matched line is the road's centre line; the car drives in a lane beside it. That offset is not noise
    # (it is the same from one match to the next): a slowly changing state, so that a thousand matches do not
    # pretend to know the position across the road to centimetres
    aux = (AuxState("off", 0.0, 2.5, tau_s=120.0, unit="m", doc="the car's offset from the matched road's centre line (left positive)"),
           AuxState("amb", 0.0, 1.0, tau_s=120.0, unit="x spread", doc="which of the candidate roads the car is on, in units of "
                    "the spread of their lines (never corrected by the map's own rows)"))
    min_conf = 0.5               # (a match without shares, from a matcher before L47: the rule of then)
    min_fit = 0.6                # the winner itself has to fit the pose
    max_turn_rate = 0.2          # rad/s: no row while the car turns faster than this
    max_heading_off = math.radians(35.0)     # ... or while its heading is further than this from the road's direction
    min_weight = 0.5             # ... and the lines parallel to it have to hold at least this share
    max_spread_m = 20.0          # candidates further apart than this constrain nothing
    parallel_deg = 20.0
    min_speed = 3.0
    cross_sigma_m = 1.0          # per match, about the lane offset state
    heading_sigma = math.radians(8.0)
    heading_alt_share = 0.15     # no heading row while candidates of another direction hold this share
    heading_known = 0.35         # ... nor while the heading is less certain than this (rad): a road's bearing is a line, and
                                 # which way along it the car drives has to be known already
    max_yaw_rate = 0.05
    # Cross-check against the absolute source, raw against raw (detector `map.conflict`): the signed distance of
    # each new receiver fix from the matched road's line, median over the last fixes. A receiver that scatters
    # 10-15 m gives a median within a few metres of the road it is on; a matcher on a parallel road gives the
    # distance between the roads. The match is made from the position, the receiver is the absolute source: in a
    # conflict the map's rows are the ones set aside. (A fix is seconds old: along a straight road that does not
    # change its distance from the road's line; in bends the check pauses.)
    conflict_n, conflict_min_n = 8, 6
    conflict_m = 12.0            # an offset below this is never a conflict
    wrong_prior = 0.03           # how often a confident match is on a wrong road at all
    wrong_span_m = 120.0         # ... and such a road lies anywhere within +- half of this
    conflict_clear = 0.35        # a conflict ends when the odds are back below this probability

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
        mem["spread"] = None
        if not settled or sp is None or sp[0] < self.min_speed or not mt.get("matched") \
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
        c0 = nx * em + ny * nm
        shares = fin(mt.get("weight")) and isinstance(mt.get("alternatives"), list)
        lines = [(0.0, 1.0)]             # (offset of a parallel candidate's line from the winner's, share)
        other = 0.0                      # share of the candidates that are not parallel to the winner
        if shares:
            lines = [(0.0, float(mt["weight"]))]
            for a in mt["alternatives"]:
                if not (isinstance(a, dict) and fin(a.get("lat")) and fin(a.get("lon")) and fin(a.get("bearing_deg")) and fin(a.get("weight"))):
                    continue
                db = (a["bearing_deg"] - mt["bearing_deg"] + 90.0) % 180.0 - 90.0
                if abs(db) <= self.parallel_deg:
                    ea, na = E.to_local(integ.origin, a["lat"], a["lon"])
                    lines.append((nx * ea + ny * na - c0, float(a["weight"])))
                else:
                    other += float(a["weight"])
        elif mt.get("ambiguous") or not fin(mt.get("confidence")) or mt["confidence"] < self.min_conf:
            return alive
        straight = abs(float(x[E.W])) < self.max_yaw_rate
        # one confident road and nothing beside it: there the raw fixes measure the receiver (`_conflict`)
        clean = (other < 0.05 and len(lines) == 1 and mt.get("fit", 0.0) >= 0.8) if shares else mt["confidence"] >= 0.75
        hard, p_rx, r_rx = self._conflict(ctx, mem, ts, em, nm, nx, ny, straight, lines, clean)
        if hard and integ.gates("map.conflict"):
            mem["held"] = "conflicts with the receiver"
            return alive
        if p_rx > 0.02 and integ.gates("map.conflict"):
            # the receiver's fixes lie beside the road by more than its error readily explains: "the receiver is
            # right and this is the wrong road" is a candidate too, with its probability as its share. It widens
            # the spread; it does not move the target (the receiver's rows do that themselves, given the spread)
            lines = [(o, w * (1.0 - p_rx)) for o, w in lines]
            rx_line = (r_rx, p_rx)
        else:
            rx_line = None
        wp = sum(w for _, w in lines)
        if shares and (mt.get("fit", 1.0) < self.min_fit or wp < self.min_weight):
            mem["held"] = "the match is weak"
            return alive
        # The car is driving along the road, or the road's line says nothing about it: not while it turns hard (a
        # junction, a roundabout, a car park: which piece it is on is the matcher's guess then), and not while its
        # heading, where known, is far from the road's direction. (A loop in a car park at walking pace pulled
        # the position onto the streets around it, 60 m off with 2 m stated, before these two conditions.)
        if abs(float(x[E.W])) > self.max_turn_rate or (ctx.P[E.PSI, E.PSI] < self.heading_known ** 2 and abs(d) > self.max_heading_off):
            mem["held"] = "not driving along the road"
            return alive
        mean = sum(o * w for o, w in lines) / wp
        var = sum(w * (o - mean) ** 2 for o, w in lines)
        if rx_line is not None:
            var += rx_line[1] * (rx_line[0] - mean) ** 2
            wp_all = wp + rx_line[1]
        else:
            wp_all = wp
        spread = math.sqrt(max(var / wp_all, 0.0))
        mem["spread"] = round(spread, 1)
        if spread > self.max_spread_m:
            mem["held"] = "the candidate roads are too far apart"
            return alive
        ioff, iamb = self.ix["off"], self.ix["amb"]
        i_c = np.array([E.PE, E.PN, ioff, iamb], dtype=np.intp)
        co = np.array([nx, ny, -1.0, -spread])
        # the candidates that are not parallel hold `1 - wp` of the belief: with that probability this line says
        # nothing about the car, and the row weighs that much less
        sig = self.cross_sigma_m / math.sqrt(max(wp, 1e-3))
        out = alive + [Meas("map_cross", ts, c0 + mean, sig,
                            lambda x: (nx * x[E.PE] + ny * x[E.PN] - x[ioff] - spread * x[iamb], i_c, co), "map.cross",
                            robust=True, local=True, consider=(iamb,))]
        span = math.radians(mt["bearing_span_deg"]) if fin(mt.get("bearing_span_deg")) else 0.0
        if straight and abs(d) < math.radians(30.0) and other < self.heading_alt_share and ctx.P[E.PSI, E.PSI] < self.heading_known ** 2:
            i_h = np.array([E.PSI], dtype=np.intp)
            one = np.array([1.0])
            out.append(Meas("map_heading", ts, float(x[E.PSI]) + d, math.hypot(self.heading_sigma, span),
                            lambda x: (x[E.PSI], i_h, one), "map.heading", robust=True, local=not ctx.cfg["gnss_absolute"]))
        mem["used"] = mem.get("used", 0) + 1
        return out

    def _conflict(self, ctx, mem, ts, em, nm, nx, ny, straight, lines=((0.0, 1.0),), clean=False):
        """The receiver's recent fixes against the lines the matcher offers. Returns (conflict: the map's rows are
        to be set aside, probability that the receiver is right and the match wrong, the fixes' median offset
        from the nearest offered line)."""
        fix = getattr(ctx.integ, "abs_latest", None)
        q = mem.setdefault("cq", [])
        if fix is not None and straight and fix[0] != mem.get("cfix") and ts - fix[0] < 3.0:
            mem["cfix"] = fix[0]
            dist = nx * (fix[1] - em) + ny * (fix[2] - nm)
            # the distance to the nearest line the matcher offers (the winner's, or a parallel candidate's)
            near = min((dist - o for o, w in lines if w >= 0.1 or o == 0.0), key=abs)
            q.append((dist, fix[3], near))
            del q[:-self.conflict_n]
            turned = ctx.integ.yaw_between(fix[0] - 5.0, ts)
            if clean and turned is not None and abs(turned) < 0.05:      # (the fix is seconds old: straight since then too)
                # one confident road on a straight: how far beside it the raw fix lies is the receiver's own
                # slowly varying error (and the car's place on the road): the receiver's source measures its
                # level from it
                g = ctx.integ.ctx.get(fix[4]) if len(fix) > 4 else None
                if g is not None and hasattr(g.src, "offer_cross"):
                    g.src.offer_cross(g, dist, g.mem.get("hacc"))
        if fix is None or ts - fix[0] > 15.0:
            del q[:]                                 # no receiver: nothing to check against
        key = ("map", "cross", "conflict")
        p_rx, near = 0.0, 0.0
        if len(q) >= self.conflict_min_n:
            d = sorted(v for v, _, _ in q)
            med = 0.5 * (d[(len(d) - 1) // 2] + d[len(d) // 2])
            dn = sorted(v for _, _, v in q)
            near = 0.5 * (dn[(len(dn) - 1) // 2] + dn[len(dn) // 2])
            # the fixes' error is slowly varying: the median of a few seconds of them is as far off as one of them
            sg = math.hypot(sorted(s for _, s, _ in q)[len(q) // 2], self.aux[0].sigma0)
            mem["conflict_m"] = round(med, 1)
            if abs(near) > self.conflict_m:
                like = math.exp(-0.5 * (near / sg) ** 2) / (sg * math.sqrt(2.0 * math.pi))
                odds = self.wrong_prior / (1.0 - self.wrong_prior) / self.wrong_span_m / max(like, 1e-300)
                p_rx = odds / (1.0 + odds)
            mem["conflict_peak"] = max(mem.get("conflict_peak", 0.0), p_rx / 0.5)
            mem["conflict_p"] = round(p_rx, 2)
            if p_rx > (self.conflict_clear if mem.get("conflict") else 0.5):
                mem["conflict"] = True
                ctx.integ.fdi._set(*key, "warn", ts, "the matched road lies %.0f m %s of the receiver's last %d fixes (median; the "
                                   "receiver's own error is about %.0f m): a matcher on a parallel road, or a receiver offset; the "
                                   "map's rows are %s" % (abs(med), "right" if med > 0 else "left", len(q), sg,
                                                          "set aside" if ctx.integ.gates("map.conflict") else "still used (detector map.conflict in report mode)"))
                return True, p_rx, near
        if mem.get("conflict") and len(q) >= self.conflict_min_n or not q:
            mem["conflict"] = False
            ctx.integ.fdi.findings.pop(key, None)
        return bool(mem.get("conflict")), p_rx, near

    def report(self, ctx):
        m = ctx.mem
        return {"matched": m.get("matched"), "match_confidence": m.get("conf"), "used": m.get("used", 0), "held": m.get("held"),
                "spread_m": m.get("spread"),
                "receiver_offset_m": m.get("conflict_m"), "receiver_right_p": m.get("conflict_p"),
                "conflict_peak": round(m.get("conflict_peak", 0.0), 2),
                "suspect_reason": ("the matched road lies %.0f m beside the receiver's recent fixes (map rows set aside)"
                                   % abs(m.get("conflict_m") or 0.0)) if m.get("held") == "conflicts with the receiver" else None}
