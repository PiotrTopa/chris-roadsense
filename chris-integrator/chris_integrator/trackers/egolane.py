"""Ego lane: which lane of its carriageway the car is in, as a filter over time (lane L20).

The question "three lanes, which one am I on" is answered by a discrete Bayes filter (an HMM) over k, the
number of own-direction lanes to the RIGHT of the car (k = 0: the rightmost lane). Index from the left, the
form a consumer draws, is `forward - 1 - k` once the number of own lanes is known.

Evidence per camera frame (the tracked lines and edges of the `lanes` topic, at 8-20 m):

* the right road edge: its distance from the ego lane's right line is about k lane widths plus a shoulder;
  the left road edge likewise counts the lanes on the left, own and (on a two-way road) oncoming;
* the outer lane lines: a far-right line one lane width beyond the right line says "a lane to the right"
  (and so k >= 1), its absence weakly says k = 0; the same on the left;
* the map's lane count of the matched road (`map.road.lanes`, tagged values only): it bounds k to
  0..forward-1 and ties the two sides together (lanes on the left = forward - 1 - k + backward);
* lane changes: the tracked ego-lane centre moves to one side and comes back on the other within about a
  second (the network relabels its lines when the car crosses one; measured on the labelled rides it is never
  one clean frame) shifts the distribution by one lane; the turn signal (vehicle input, `ego.turn_signal`)
  makes a change in its direction more likely, never forces one;
* a change of the map's lane count: the lane was added or removed on the right or on the left, which the count
  does not say, so a confident estimate opens up (half each way) until the camera settles it; painted turn
  arrows of the new stretch (`map.road.turn_lanes`, pack format 2) say the side when the new lanes at one edge
  only turn (a turn pocket, an exit lane).

Between frames the distribution diffuses a little (a lane change the camera did not see). `probs` is the
distribution over lanes from the left (or from the right while the count is unknown); `ego_index` is given only
when one lane holds at least `commit_p` of it: "unknown" is a valid answer, a wrong confident one is not.

No I/O and no wall clock in here; times come from the messages (CLOCK_MONOTONIC ns).
"""
import math

from ..native import round                  # builtins.round, native where built
from ..contract import LocalEgo, signal

NS = 1e9

DEFAULT_CFG = dict(
    k_max=5,                    # lanes to the right that are considered without a map count
    x_window=(8.0, 20.0),       # distances (m) at which the edges and outer lines are read
    # The model places road edges nearer than the lane geometry would: on the labelled rides (L20) the gap from
    # the ego lane's line to the edge was 2.5-3.3 m with no lane beyond and grew by only 1.6-2.4 m per lane.
    edge_gap0_m=2.5,            # expected gap with no lane on that side ...
    edge_lane_m=1.8,            # ... plus this per lane
    edge_near_m=1.0,            # an edge nearer than the expectation minus this rules out that many lanes
    edge_near_sigma_m=0.5,
    shoulder_max_m=2.5,         # ... one farther than expected says little (a shoulder, a verge, a parking strip)
    edge_far_sigma_m=1.6,
    edge_outlier=0.08,          # a road edge that is simply wrong (a kerb of a lay-by, a parked car)
    edge_min_conf=0.5,          # the tracker's edge confidence needed to use it
    line_hit=0.7, line_false=0.05,      # P(outer line seen | a lane there), P(... | no lane there): measured on
                                        # the labelled rides 0.69-0.73 and 0.00-0.13 (L20)
    absence_trust=0.2,          # an outer line NOT seen is weak evidence (worn paint, a bus lane's wide line, a
                                # queue beside the car, a wide lens): P(not seen | lane) is taken as
                                # 1 - absence_trust * line_hit. 0.4 until L22; 0.2 halved the confident errors on
                                # the car's camera and in town
    line_min_conf=0.45,
    width_default_m=3.4,
    diffuse_per_s=0.01,         # probability per second of a lane change the camera did not see
    # A lane change as the camera shows it: the ego-lane centre drifts to one side (the car nears a line), then
    # the network relabels its lines and the centre comes back on the other side. The relabelling is rarely one
    # clean frame (at night it alternates between the two labellings for half a second, and the tracker slides
    # between them), so the pattern is read on the tracked lines over a window: a move of change_drop lane
    # widths within change_window_s, from a centre at least change_from widths to one side to one at least
    # change_to widths to the other.
    change_drop=0.38, change_window_s=1.2, change_from=0.2, change_to=0.1,
    change_refractory_s=2.0,    # one change per this long
    change_p=0.85,              # probability that such a pattern is a real change (else a relabel glitch)
    count_memory_s=10.0,
    turn_lanes_side_p=0.85,     # ... unless the new lanes carry turn-only arrows on one side (map turn lanes)
    count_change_right=0.5,     # the map's lane count changed: probability that the lane was added / removed on
                                # the right (unknown without turn lanes: the estimate becomes "unknown", not wrong)
    signal_boost=1.5,           # the turn signal multiplies the weight of a change in its direction
    map_min_match=0.5,          # the map count is used from this match confidence on
    map_max_age_s=3.0,
    commit_p=0.8,               # ego_index only when one lane holds this much (0.75 until L22)
    evidence_tau_s=0.5,         # consecutive frames see the same road: their evidence counts as one
                                # independent observation per this many seconds (likelihood ** (dt / tau))
    min_speed_mps=2.0,          # standing: the evidence is not used (queues hide the edges)
)


# Per road model (`road.road_model`), on top of DEFAULT_CFG; a key in the user's egolane.json wins. The v0.11.1 vision
# model's lane lines are less steady than supercombo's (L24-M), and at the same threshold the filter was wrong and
# confident more often on the labelled rides (5.5 % of labelled frames against 3.4 %; night expressway 9.3 against
# 2.6 %). At 0.9 it is wrong as rarely as with supercombo (3.3 %) and says "unknown" more often (9.9 against
# 6.6 %): unknown beats confident wrong (docs/lanes/L27-new-model-default.md).
PROFILES = {"vision": dict(commit_p=0.9)}


def _norm(p):
    s = sum(p)
    if s <= 0:
        return [1.0 / len(p)] * len(p)
    return [v / s for v in p]


def _at(xs, ys, lo, hi):
    """Mean of ys over lo <= x <= hi (None when nothing there)."""
    if not xs or not ys:
        return None
    v = [y for x, y in zip(xs, ys) if lo <= x <= hi and y is not None]
    return sum(v) / len(v) if v else None


class EgoLaneEstimator:
    def __init__(self, cfg=None):
        self.user_cfg = dict(cfg or {})
        self.road_model = None
        self.cfg = dict(DEFAULT_CFG, **self.user_cfg)
        self._local = LocalEgo()
        self.reset()

    def reset(self):
        self.p = None               # distribution over k (lanes to the right), length n
        self.n = None               # number of own lanes the distribution is over (None: unknown, k_max)
        self.n_known = None         # (the map's last count, when)
        self.t = None
        self.map = None
        self.turn = None
        self.c_hist = []            # (t, centre of the ego lane from the tracked inner lines) of the last seconds
        self.c_block = 0            # no lane change is read before this time (refractory)
        self.last_change = None     # {"dir": "left"|"right", "ts": ns}
        self.evidence = {}
        self.evidence_side = None
        self.out = None

    # ---- inputs ------------------------------------------------------------------------------------
    def on_ego(self, msg):
        """A mapd `ego` message, for an instance that is not handed the integrator's state (docs/DESIGN.md)."""
        self._local.feed(msg)

    def on_map(self, msg):
        self.map = msg

    def _map_lanes(self, now):
        m = self.map
        c = self.cfg
        if not m or not m.get("available") or now is None or m.get("ts_mono_ns") is None:
            return None
        if (now - m["ts_mono_ns"]) / NS > c["map_max_age_s"]:
            return None
        if ((m.get("match") or {}).get("confidence") or 0.0) < c["map_min_match"]:
            return None
        ln = (m.get("road") or {}).get("lanes") or {}
        if ln.get("source") not in ("tagged", "tagged_total") or not ln.get("forward"):
            return None
        tl = (m.get("road") or {}).get("turn_lanes")
        if not (isinstance(tl, list) and len(tl) == int(ln["forward"]) and all(isinstance(x, list) for x in tl)):
            tl = None
        return {"forward": int(ln["forward"]), "backward": int(ln.get("backward") or 0),
                "oneway": bool((m.get("road") or {}).get("oneway")), "source": ln["source"], "turn_lanes": tl}

    def _side_of_new_lanes(self, ml):
        """The map's count grew and the road has painted turn arrows (pack format 2): a lane that only turns,
        at the edge of the carriageway, is the one that was added (a turn pocket, an exit lane). Probability
        that the new lanes are on the right, or None when the arrows do not say."""
        tl = ml.get("turn_lanes")
        if not tl or not self.n_known or ml["forward"] <= self.n_known[0]:
            return None
        d = ml["forward"] - self.n_known[0]
        only = lambda a, side: bool(a) and all(side in w for w in a)      # noqa: E731
        right = all(only(a, "right") for a in tl[-d:])
        left = all(only(a, "left") for a in tl[:d])
        if right != left:
            self.evidence_side = "right" if right else "left"
            return self.cfg["turn_lanes_side_p"] if right else 1.0 - self.cfg["turn_lanes_side_p"]
        return None

    def _resize(self, n, p_right=None):
        """Change the support to n own lanes (None: unknown). When the map's count differs from the last one it
        gave (within count_memory_s: a junction or a doubtful match in between does not hide the change), the
        lane was added or removed on the right with probability p_right (k, the lanes to the right, moves with
        it) and on the left otherwise (k stays): without knowledge of the side a confident estimate becomes an
        open one, which the next seconds of evidence settle. Probability beyond the count piles up on the
        outermost lane."""
        c = self.cfg
        size = n if n else c["k_max"]
        if self.p is None:
            self.p = [1.0 / size] * size
        else:
            old = self.p
            d = 0
            if n and self.n_known and n != self.n_known[0] and (self.t - self.n_known[1]) / NS <= c["count_memory_s"]:
                d = n - self.n_known[0]
            if d or len(old) != size:
                pr = (c["count_change_right"] if p_right is None else p_right) if d else 0.0
                new = [0.0] * size
                for k, v in enumerate(old):
                    new[min(max(k + d, 0), size - 1)] += v * pr          # on the right: k changes by d
                    new[min(k, size - 1)] += v * (1.0 - pr)              # on the left: k stays
                self.p = _norm([v + (0.15 / size if len(old) != size else 0.0) for v in new])
        self.n = n
        if n:
            self.n_known = (n, self.t)

    # ---- the filter --------------------------------------------------------------------------------
    def on_lanes(self, lanes, road=None, speed_mps=None, ego=None):
        """One tracked frame (`lanes` topic message); `road` is the raw model frame of it (lane changes are read
        on the tracked lines; of `road` only `road_model` is read: it selects the profile). Returns the estimate
        (also kept as `self.out`)."""
        rm = road.get("road_model") if isinstance(road, dict) else None
        if rm != self.road_model and (rm is not None or road is not None):
            self.road_model = rm
            self.cfg = dict(DEFAULT_CFG, **PROFILES.get(rm, {}))
            self.cfg.update(self.user_cfg)
        c = self.cfg
        ts = lanes.get("ts_mono_ns")
        if not isinstance(ts, int):
            return self.out
        es = ego if ego is not None else self._local.state(ts)     # the integrator's state: the turn signal
        sig = signal(es, "turn_signal", 2.0) if es is not None else None
        self.turn = sig if sig in ("left", "right") else None
        dt = 0.0 if self.t is None else max(0.0, (ts - self.t) / NS)
        self.t = ts
        ml = self._map_lanes(ts)
        self.evidence_side = None
        self._resize(ml["forward"] if ml else None, self._side_of_new_lanes(ml) if ml else None)
        size = len(self.p)
        # diffusion: a change the camera did not see
        if dt > 0 and size > 1:
            e = min(0.3, c["diffuse_per_s"] * dt)
            q = [0.0] * size
            for k, v in enumerate(self.p):
                q[k] += v * (1.0 - e)
                nb = [j for j in (k - 1, k + 1) if 0 <= j < size]
                for j in nb:
                    q[j] += v * e / len(nb)
            self.p = q
        xs = lanes.get("x") or []
        lines = lanes.get("lines") or [{}] * 4
        edges = lanes.get("edges") or [{}] * 2
        w = lanes.get("lane_width_m") or c["width_default_m"]
        w = min(4.5, max(2.6, w))
        lo, hi = c["x_window"]
        ev = {}
        spd = speed_mps if speed_mps is not None else ((lanes.get("ego") or {}).get("speed_mps") or 0.0)
        # ---- lane change from the tracked inner lines
        chg = self._lane_change(lanes, ts, w)
        if chg and size > 1:
            sh = 1 if chg == "left" else -1                 # moving left: one more own lane to the right
            pc = c["change_p"]
            if self.turn == chg:
                pc = min(0.97, 1 - (1 - pc) / c["signal_boost"])    # the driver signalled this way
            q = [0.0] * size
            for k, v in enumerate(self.p):
                j = k + sh
                if 0 <= j < size:
                    q[j] += v * pc
                    q[k] += v * (1 - pc)
                else:
                    q[k] += v                               # cannot leave the carriageway that way
            self.p = q
            self.last_change = {"dir": chg, "ts": ts}
            ev["change"] = chg
        if self.evidence_side:
            ev["new_lanes"] = self.evidence_side
        if spd >= c["min_speed_mps"]:
            yl = _at(xs, (lines[1] or {}).get("y"), lo, hi) if (lines[1] or {}).get("present") else None
            yr = _at(xs, (lines[2] or {}).get("y"), lo, hi) if (lines[2] or {}).get("present") else None
            like = [1.0] * size
            # right side: k lanes to the right
            er = edges[1] or {}
            if er.get("present") and (er.get("conf") or 0.0) >= c["edge_min_conf"] and yr is not None:
                yer = _at(xs, er.get("y"), lo, hi)
                if yer is not None:
                    g = yer - yr
                    ev["edge_right_m"] = round(g, 2)
                    for k in range(size):
                        like[k] *= self._edge_like(g, k, w)
            fr = lines[3] or {}
            seen_r = bool(fr.get("present")) and (fr.get("conf") or 0.0) >= c["line_min_conf"]
            if yr is not None:
                if seen_r:
                    yfr = _at(xs, fr.get("y"), lo, hi)
                    seen_r = yfr is not None and 0.6 * w <= yfr - yr <= 1.6 * w
                ev["lane_right"] = seen_r
                for k in range(size):
                    like[k] *= self._line_like(seen_r, k >= 1)
            # left side: the lanes to the left are own lanes (forward - 1 - k) and, on a two-way road, oncoming
            n_onc = ml["backward"] if ml else None
            el = edges[0] or {}
            fl = lines[0] or {}
            seen_l = bool(fl.get("present")) and (fl.get("conf") or 0.0) >= c["line_min_conf"]
            if yl is not None and seen_l:
                yfl = _at(xs, fl.get("y"), lo, hi)
                seen_l = yfl is not None and 0.6 * w <= yl - yfl <= 1.6 * w
            if ml:
                n = ml["forward"]
                if el.get("present") and (el.get("conf") or 0.0) >= c["edge_min_conf"] and yl is not None:
                    yel = _at(xs, el.get("y"), lo, hi)
                    if yel is not None:
                        g = yl - yel
                        ev["edge_left_m"] = round(g, 2)
                        for k in range(size):
                            like[k] *= self._edge_like(g, (n - 1 - k) + (n_onc or 0), w)
                if yl is not None:
                    ev["lane_left"] = seen_l
                    for k in range(size):
                        like[k] *= self._line_like(seen_l, (n - 1 - k) + (n_onc or 0) >= 1)
            ex = min(1.0, max(0.02, dt / c["evidence_tau_s"])) if dt > 0 else 1.0
            self.p = _norm([a * b ** ex for a, b in zip(self.p, like)])
        self.evidence = ev
        self.out = self._result(ml)
        return self.out

    def _line_like(self, seen, lane):
        c = self.cfg
        if lane:
            return c["line_hit"] if seen else 1.0 - c["absence_trust"] * c["line_hit"]
        return c["line_false"] if seen else 1.0 - c["line_false"]

    def _edge_like(self, g, k, w):
        """Likelihood of an edge gap g (m from the ego lane's line) with k lanes on that side: one-sided. An edge
        well inside the k-th lane is (nearly) impossible; one beyond k lanes plus a shoulder is unlikely only
        gently."""
        c = self.cfg
        mu = c["edge_gap0_m"] + k * c["edge_lane_m"] * (w / 3.4)
        lo = mu - c["edge_near_m"]
        hi = mu + c["shoulder_max_m"]
        v = 1.0
        if g < lo:
            v *= math.exp(-0.5 * ((g - lo) / c["edge_near_sigma_m"]) ** 2)
        if g > hi:
            v *= math.exp(-0.5 * ((g - hi) / c["edge_far_sigma_m"]) ** 2)
        return c["edge_outlier"] + v

    def _lane_change(self, lanes, ts, w):
        """'left' / 'right' when the ego-lane centre of the tracked lines shows the pattern of a lane change
        (DEFAULT_CFG): it was to one side, and within the window it moved by change_drop lane widths to the
        other side. A car that goes left sees its lines move right (the centre grows), then the lines of the
        lane on the left become the ego lane's (the centre falls by a lane width)."""
        c = self.cfg
        ln = (lanes or {}).get("lines") or []
        h = self.c_hist
        while h and (ts - h[0][0]) / NS > 3.0:
            h.pop(0)
        if len(ln) != 4 or not (ln[1] or {}).get("present") or not (ln[2] or {}).get("present"):
            return None
        xs = lanes.get("x") or []
        ya, yb = _at(xs, ln[1].get("y"), 3.0, 10.0), _at(xs, ln[2].get("y"), 3.0, 10.0)
        if ya is None or yb is None or not 0.6 * w <= yb - ya <= 1.5 * w:
            return None
        cur = 0.5 * (ya + yb)
        h.append((ts, cur))
        if ts < self.c_block:
            return None
        win = [v for t, v in h[:-1] if (ts - t) / NS <= c["change_window_s"]]
        if len(win) < 3:
            return None
        hi, lo = max(win), min(win)
        out = None
        if hi >= c["change_from"] * w and cur <= -c["change_to"] * w and hi - cur >= c["change_drop"] * w:
            out = "left"
        elif lo <= -c["change_from"] * w and cur >= c["change_to"] * w and cur - lo >= c["change_drop"] * w:
            out = "right"
        if out:
            self.c_block = ts + int(c["change_refractory_s"] * NS)
            del h[:]
        return out

    def _result(self, ml):
        c = self.cfg
        p = self.p
        size = len(p)
        best = max(range(size), key=lambda k: p[k])
        conf = p[best]
        if ml:
            n = ml["forward"]
            probs = [round(p[n - 1 - i], 3) for i in range(n)]          # from the left
            ego = (n - 1 - best) if conf >= c["commit_p"] else None
        else:
            probs = [round(v, 3) for v in p]                             # from the right
            ego = None
        prov = ["vision"]
        if ml:
            prov.append("map:" + ml["source"])
        if self.last_change and self.t - self.last_change["ts"] < 10 * NS:
            prov.append("lane_change")
        if self.turn:
            prov.append("turn_signal")
        if ml and ml.get("turn_lanes"):
            prov.append("map:turn_lanes")
        return {"ego_index": ego,
                "ego_from_right": best if conf >= c["commit_p"] else None,
                "probs": probs, "probs_from": "left" if ml else "right",
                "confidence": round(conf, 3), "forward": ml["forward"] if ml else None,
                "provenance": prov, "evidence": dict(self.evidence),
                "last_change": None if not self.last_change else {
                    "dir": self.last_change["dir"], "age_s": round((self.t - self.last_change["ts"]) / NS, 1)}}
