"""Online map matching: which edge of the road graph the car is on, in which direction, how far along.

Every update scores the edges near the filtered position:

    score = -(d / sigma_d)^2 / 2                     distance to the edge
            -(heading difference / sigma_h)^2 / 2    travel direction against the edge bearing (when moving)
            + class prior                            service / restricted roads lose a tie against public roads
            + transition                             0 for the current edge and for edges reachable from it along
                                                     the graph within the distance travelled; a penalty otherwise

The transition term is what keeps the match from jumping to a parallel road: a road that cannot be reached
from the current one has to fit far better at once, or clearly better over some distance (250 m), before it
wins. Confidence is the
winner's share of exp(score) over all candidate roads, scaled down when the winner itself fits poorly.

The match as a measurement (L47). The winner is one road, chosen with a strong preference for continuity; a
pose source that takes "the car is on this line" from it must also be told what else the car may be on, or
the choice confirms itself (the pose is pulled onto the line, the line then fits better). `weight` is the
winner's share and `alternatives` are the other candidates' lines with their shares, computed from the fit
alone (distance, heading, road class) with only a mild preference for the road the match was on
(`alt_penalty`): an unconnected parallel road is not a jump of the car, it is the possibility that the match
has been on the wrong one of the two. `fit` says how well the winner itself fits the pose, and
`bearing_span_deg` how much the road's direction changes around the matched point (in a bend the bearing at
the matched point is only as good as the position along the road).
"""
import math

from . import geo
from . import rsmap

CLASS_PRIOR = {"service": -1.0, "track": -1.5}


class Match:
    __slots__ = ("edge", "dir", "s", "dist", "lat", "lon", "bearing", "heading_diff", "confidence", "ambiguous",
                 "candidates", "share", "fit", "weight", "alternatives", "bearing_span")

    def as_dict(self):
        return {"matched": True, "confidence": round(self.confidence, 3), "ambiguous": self.ambiguous,
                "distance_m": round(self.dist, 1),
                "heading_diff_deg": None if self.heading_diff is None else round(self.heading_diff, 1),
                "lat": round(self.lat, 7), "lon": round(self.lon, 7), "bearing_deg": round(self.bearing, 1),
                "edge": "%08x:%d" % self.edge.ref, "dir": self.dir, "offset_m": round(self.s, 1),
                "candidates": self.candidates, "fit": round(self.fit, 3), "weight": round(self.weight, 3),
                "bearing_span_deg": round(self.bearing_span, 1),
                "alternatives": [{"lat": round(a[0], 7), "lon": round(a[1], 7), "bearing_deg": round(a[2], 1),
                                  "weight": round(a[3], 3)} for a in self.alternatives]}


class MapMatcher:
    def __init__(self, pack, cfg=None):
        c = dict(radius_min=35.0, radius_max=120.0, sigma_d_min=5.0, sigma_h=22.0, jump_penalty=7.0,
                 uturn_penalty=5.0, switch_margin=2.0, off_road_m=35.0, lost_updates=6, min_speed_heading=1.5,
                 persist=8, persist_m=250.0, persist_margin=1.5, branch_margin=1.2, fork_memory_m=150.0,
                 # lane L20: service / restricted / parking roads lose more the faster the car goes (a car at
                 # 40 km/h is rarely on a parking aisle); the class prior is scaled from 1 at walking pace up to
                 # 1 + class_speed at class_speed_mps and above
                 class_speed=3.0, class_speed_mps=10.0, parking_prior=-1.0,
                 # lane L20: confidence of the ROAD: candidates that are pieces of the matched road (the same
                 # edge in the other direction does not count; the next edge of the same named road does) are
                 # not rivals, so the confidence does not dip where the road is split at a side road
                 road_confidence=True,
                 # lane L47: the match as a measurement: the preference for the road the match was on when the
                 # candidates' shares are computed for the pose source, alternatives below `alt_min` are not
                 # listed (at most `alt_n`), the road's direction is looked at `span_m` either side
                 alt_penalty=1.0, alt_min=0.02, alt_n=4, span_m=15.0)
        c.update(cfg or {})
        self.cfg = c
        self.pack = pack
        self.cur = None             # Match
        self.lost = 0
        self.switches = 0
        self._alt = (None, 0, 0.0)
        self._fork = None           # [the match before the last change of edge, metres travelled since]

    def reset(self):
        self.cur = None
        self.lost = 0
        self._alt = (None, 0, 0.0)
        self._fork = None

    def _reachable(self, m, budget):
        """{(edge ref, dir): distance} of edges reachable forwards from the current match within budget metres."""
        out = {(m.edge.ref, m.dir): 0.0}
        remaining = (m.edge.length - m.s) if m.dir > 0 else m.s
        frontier = [(m.edge, m.dir, remaining)]
        steps = 0
        while frontier and steps < 40:
            steps += 1
            e, d, dist = frontier.pop()
            if dist > budget:
                continue
            node = e.b if d > 0 else e.a
            for ref, end in self.pack.node_edges(node):
                if ref == e.ref and e.a != e.b:
                    continue
                e2 = self.pack.edge(ref)
                if e2 is None:
                    continue
                d2 = 1 if end == 0 else -1
                if d2 < 0 and e2.oneway:
                    continue
                k = (ref, d2)
                if k in out and out[k] <= dist:
                    continue
                out[k] = dist
                frontier.append((e2, d2, dist + e2.length))
        return out

    def update(self, lat, lon, heading_deg, speed, hacc, travelled_m, reverse=False, cross_sigma=None):
        """One matching step. heading_deg may be None (unknown). hacc: RMS radial position error;
        cross_sigma: its part across the heading when known (dead reckoning is uncertain along the road, not
        across it). Returns a Match or None (off the network)."""
        c = self.cfg
        radius = min(c["radius_max"], max(c["radius_min"], 2.5 * hacc + 15.0))
        lateral = hacc if cross_sigma is None or heading_deg is None else min(hacc, 1.42 * cross_sigma)
        sigma_d = max(c["sigma_d_min"], 0.8 * lateral)
        use_heading = heading_deg is not None and abs(speed) >= c["min_speed_heading"]
        if use_heading and reverse:
            heading_deg = (heading_deg + 180.0) % 360.0
        cur = self.cur
        reach = self._reachable(cur, 1.5 * travelled_m + 2.0 * sigma_d + 40.0) if cur else None
        if cur and self._fork is not None:
            # shortly after a change of edge the alternatives of that fork stay reachable: a branch taken by
            # mistake (an exit ramp that leaves at a shallow angle) is left again without a penalty
            self._fork[1] += travelled_m
            if self._fork[1] > c["fork_memory_m"]:
                self._fork = None
            else:
                for k, v in self._reachable(self._fork[0], self._fork[1] + 2.0 * sigma_d + 40.0).items():
                    reach.setdefault(k, v)
        best_per_edge = {}
        cur_cand = None
        kspeed = 1.0 + c["class_speed"] * min(1.0, max(0.0, (abs(speed) - 1.5) / max(0.1, c["class_speed_mps"] - 1.5)))
        for e in self.pack.query(lat, lon, radius):
            d, s, brg = e.project(lat, lon)
            if d > radius:
                continue
            prior = CLASS_PRIOR.get(e.cls_name, 0.0)
            if e.flags & rsmap.F_ACCESS_RESTRICTED:
                prior -= 1.0
            if e.flags & rsmap.F_PARKING:
                prior += c["parking_prior"]
            base = -0.5 * (d / sigma_d) ** 2 + prior * kspeed
            for direction in ((1,) if e.oneway else (1, -1)):
                b = brg if direction > 0 else (brg + 180.0) % 360.0
                hd = geo.ang_diff(heading_deg, b) if use_heading else None
                score = base
                if hd is not None:
                    score -= 0.5 * (hd / c["sigma_h"]) ** 2
                connected = True
                raw = score
                if cur:
                    k = (e.ref, direction)
                    if k in reach:
                        pass
                    elif e.ref == cur.edge.ref:
                        score -= c["uturn_penalty"]
                        connected = False
                    else:
                        score -= c["jump_penalty"]
                        connected = False
                cand = (score, e, direction, s, d, b, hd, connected, raw)
                if cur and e.ref == cur.edge.ref and direction == cur.dir:
                    cur_cand = cand
                if e.ref not in best_per_edge or score > best_per_edge[e.ref][0]:
                    best_per_edge[e.ref] = cand
        if not best_per_edge:
            return self._lose()
        ranked = sorted(best_per_edge.values(), key=lambda x: -x[0])
        best = ranked[0]
        if cur_cand is not None and best is not cur_cand:
            if best[7]:
                # connected: the continuation of the same road switches freely, a branch has to be clearly nearer
                b_e, c_e = best[1], cur.edge
                same_road = b_e.cls == c_e.cls and ((b_e.road_ref and b_e.road_ref == c_e.road_ref)
                                                    or (b_e.name and b_e.name == c_e.name)
                                                    or not (c_e.road_ref or c_e.name))
                margin = 0.3 if same_road else c["branch_margin"]
            else:
                margin = c["switch_margin"]
            if best[0] - cur_cand[0] < margin:
                best = cur_cand
        # evidence over time: an unconnected road that keeps fitting clearly better wins after `persist` updates
        # and `persist_m` metres (a multipath excursion of a few seconds does not move the match; a wrong
        # match corrects itself)
        if cur_cand is not None and best is cur_cand:
            alt = max(best_per_edge.values(), key=lambda x: x[8])
            if alt is not cur_cand and alt[8] - cur_cand[8] > c["persist_margin"]:
                # counted while the current road keeps losing, whichever edge of the other road wins
                self._alt = (True, self._alt[1] + 1, self._alt[2] + travelled_m)
                if self._alt[1] >= c["persist"] and self._alt[2] >= c["persist_m"]:
                    best = alt
                    self._alt = (None, 0, 0.0)
            else:
                self._alt = (None, 0, 0.0)
        else:
            self._alt = (None, 0, 0.0)
        score, e, direction, s, d, b, hd = best[:7]
        if d > max(c["off_road_m"], 2.5 * lateral):
            return self._lose()
        tot = sum(math.exp(max(-60.0, x[0] - ranked[0][0])) for x in ranked)
        share = math.exp(max(-60.0, score - ranked[0][0])) / tot
        if c["road_confidence"]:
            same = sum(math.exp(max(-60.0, x[0] - ranked[0][0])) for x in ranked
                       if x[1].ref != e.ref and x[7] and self._same_road(x[1], e))
            share = min(1.0, share + same / tot)
        fit = math.exp(-0.5 * (d / (2.5 * sigma_d)) ** 2)
        if hd is not None:
            fit *= math.exp(-0.5 * (hd / (2.5 * c["sigma_h"])) ** 2)
        elif heading_deg is None:
            fit *= 0.6                               # direction of travel unknown
        m = Match()
        m.edge, m.dir, m.s, m.dist, m.bearing, m.heading_diff = e, direction, s, d, b, hd
        m.lat, m.lon, _ = e.at(s)
        m.share = share
        m.fit = fit
        # the candidates as a pose source needs them: shares from the fit alone, a mild preference for continuity
        # (one candidate per edge: the winner's edge is the winner itself, in the direction that was chosen)
        ws = [(math.exp(max(-60.0, x[8] - (0.0 if x[7] else c["alt_penalty"]) - best[8])), x)
              for x in [best] + [r for r in ranked if r[1].ref != e.ref]]
        tot_w = sum(w for w, _ in ws) or 1.0
        m.weight = next((w for w, x in ws if x is best), 0.0) / tot_w
        alts = sorted(((w / tot_w, x) for w, x in ws if x is not best and w / tot_w >= c["alt_min"]), key=lambda a: -a[0])
        m.alternatives = [x[1].at(x[3])[:2] + (x[5], w) for w, x in alts[:c["alt_n"]]]
        b0, b1 = e.at(s - c["span_m"])[2], e.at(s + c["span_m"])[2]
        m.bearing_span = abs(geo.ang_diff(b0, b1))
        m.confidence = max(0.0, min(1.0, share * fit))
        m.ambiguous = share < 0.7
        m.candidates = len(ranked)
        if cur is not None and (cur.edge.ref != e.ref or cur.dir != direction):
            self.switches += 1
            self._fork = [cur, 0.0]
        self.cur = m
        self.lost = 0
        return m

    @staticmethod
    def _same_road(a, b):
        """Two edges that are pieces of one road: same class and the same road number or name."""
        return a.cls == b.cls and ((a.road_ref and a.road_ref == b.road_ref) or (a.name and a.name == b.name))

    def _lose(self):
        self.lost += 1
        if self.lost >= self.cfg["lost_updates"]:
            self.cur = None
        return None
