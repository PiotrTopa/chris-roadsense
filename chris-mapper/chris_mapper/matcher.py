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
"""
import math

from . import geo
from . import rsmap

CLASS_PRIOR = {"service": -1.0, "track": -1.5}


class Match:
    __slots__ = ("edge", "dir", "s", "dist", "lat", "lon", "bearing", "heading_diff", "confidence", "ambiguous",
                 "candidates", "share")

    def as_dict(self):
        return {"matched": True, "confidence": round(self.confidence, 3), "ambiguous": self.ambiguous,
                "distance_m": round(self.dist, 1),
                "heading_diff_deg": None if self.heading_diff is None else round(self.heading_diff, 1),
                "lat": round(self.lat, 7), "lon": round(self.lon, 7), "bearing_deg": round(self.bearing, 1),
                "edge": "%08x:%d" % self.edge.ref, "dir": self.dir, "offset_m": round(self.s, 1),
                "candidates": self.candidates}


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
                 road_confidence=True)
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
