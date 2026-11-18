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
* lane changes: a jump of the camera's ego-lane centre by about one lane width between two frames (the
  network relabels its lines when the car crosses one) shifts the distribution by one lane; the turn signal
  (vehicle input, `ego.turn_signal`) makes a change in its direction more likely, never forces one.

Between frames the distribution diffuses a little (a lane change the camera did not see). `probs` is the
distribution over lanes from the left (or from the right while the count is unknown); `ego_index` is given only
when one lane holds at least `commit_p` of it: "unknown" is a valid answer, a wrong confident one is not.

No I/O and no wall clock in here; times come from the messages (CLOCK_MONOTONIC ns).
"""
import math

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
    absence_trust=0.4,          # an outer line NOT seen is weak evidence (worn paint, a bus lane's wide line, a
                                # queue beside the car): P(not seen | lane) is taken as 1 - absence_trust * line_hit
    line_min_conf=0.45,
    width_default_m=3.4,
    diffuse_per_s=0.01,         # probability per second of a lane change the camera did not see
    change_jump=0.55,           # a centre jump of this many lane widths between frames is a lane change
    change_max_dt_s=0.35,       # ... between frames at most this far apart
    change_p=0.85,              # probability that such a jump is a real change (else a relabel glitch)
    signal_boost=1.5,           # the turn signal multiplies the weight of a change in its direction
    map_min_match=0.5,          # the map count is used from this match confidence on
    map_max_age_s=3.0,
    commit_p=0.75,              # ego_index only when one lane holds this much
    evidence_tau_s=0.5,         # consecutive frames see the same road: their evidence counts as one
                                # independent observation per this many seconds (likelihood ** (dt / tau))
    min_speed_mps=2.0,          # standing: the evidence is not used (queues hide the edges)
)


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
        self.cfg = dict(DEFAULT_CFG, **(cfg or {}))
        self.reset()

    def reset(self):
        self.p = None               # distribution over k (lanes to the right), length n
        self.n = None               # number of own lanes the distribution is over (None: unknown, k_max)
        self.t = None
        self.map = None
        self.turn = None
        self.c_prev = None          # (t, centre, width) of the previous frame
        self.last_change = None     # {"dir": "left"|"right", "ts": ns}
        self.evidence = {}
        self.out = None

    # ---- inputs ------------------------------------------------------------------------------------
    def on_ego(self, msg):
        ts = msg.get("turn_signal")
        self.turn = ts if ts in ("left", "right") else None

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
        return {"forward": int(ln["forward"]), "backward": int(ln.get("backward") or 0),
                "oneway": bool((m.get("road") or {}).get("oneway")), "source": ln["source"]}

    def _resize(self, n):
        """Change the support to n own lanes (None: unknown). Probability beyond the new count piles up on the
        outermost lane; a grown support gets a little mass everywhere (a lane was added somewhere)."""
        c = self.cfg
        size = n if n else c["k_max"]
        if self.p is None:
            self.p = [1.0 / size] * size
        elif len(self.p) != size:
            old = self.p
            new = [0.0] * size
            for k, v in enumerate(old):
                new[min(k, size - 1)] += v
            self.p = _norm([v + 0.15 / size for v in new])
        self.n = n

    # ---- the filter --------------------------------------------------------------------------------
    def on_lanes(self, lanes, road=None, speed_mps=None):
        """One tracked frame (`lanes` topic message); `road` is the raw model frame of it (lane changes are read
        from its unfiltered inner lines). Returns the estimate (also kept as `self.out`)."""
        c = self.cfg
        ts = lanes.get("ts_mono_ns")
        if not isinstance(ts, int):
            return self.out
        dt = 0.0 if self.t is None else max(0.0, (ts - self.t) / NS)
        self.t = ts
        ml = self._map_lanes(ts)
        self._resize(ml["forward"] if ml else None)
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
        # ---- lane change from the raw inner lines
        chg = self._lane_change(road, ts, w)
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

    def _lane_change(self, road, ts, w):
        """'left' / 'right' when the camera's ego-lane centre jumped by about a lane width since the last frame."""
        c = self.cfg
        ll = (road or {}).get("lane_lines")
        if not ll or len(ll) != 4:
            return None
        a, b = ll[1], ll[2]
        if (a.get("prob") or 0) < 0.5 or (b.get("prob") or 0) < 0.5 or not a.get("y") or not b.get("y"):
            self.c_prev = None
            return None
        xs = a.get("x") or []
        ya, yb = _at(xs, a["y"], 3.0, 10.0), _at(b.get("x") or xs, b["y"], 3.0, 10.0)
        if ya is None or yb is None:
            return None
        cen = 0.5 * (ya + yb)
        prev, self.c_prev = self.c_prev, (ts, cen)
        if prev is None or (ts - prev[0]) / NS > c["change_max_dt_s"]:
            return None
        jump = cen - prev[1]
        if abs(jump) < c["change_jump"] * w:
            return None
        return "left" if jump > 0 else "right"           # lines moved right in the car's frame: the car went left

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
        return {"ego_index": ego,
                "ego_from_right": best if conf >= c["commit_p"] else None,
                "probs": probs, "probs_from": "left" if ml else "right",
                "confidence": round(conf, 3), "forward": ml["forward"] if ml else None,
                "provenance": prov, "evidence": dict(self.evidence),
                "last_change": None if not self.last_change else {
                    "dir": self.last_change["dir"], "age_s": round((self.t - self.last_change["ts"]) / NS, 1)}}
