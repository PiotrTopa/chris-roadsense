"""Road-geometry tracker: the api-derived topic `lanes`.

modeld's `road.lane_lines[]` / `road.road_edges[]` are the network's instantaneous guess, frame by frame: a
lane line's `prob` hovers around any fixed drawing threshold (tens to hundreds of on/off changes a minute on
the replayed rides) and the lateral positions jump from frame to frame (road edges by 0.2-0.5 m, median).
This module turns the six lines (four lane lines, two road edges) into tracks: one message per `road`
message with a presence that has hysteresis, a geometry that is filtered over time, and the raw probability
next to it. `road` is not modified.

    road (lane_lines, road_edges, v_ego, pose) ──┐
    ego (vehicle speed, optional) ───────────────┼─> RoadGeometryTracker ─> {"type":"lanes", ...}
    map (lane count of the matched road, opt.) ──┘

Geometry. Per line the state is the lateral offset y at the model's 33 fixed distances x (device frame: x
forward, y right), one scalar Kalman filter per point, a random walk in the car's frame. The measurement noise
is the network's own `y_std` per point (scaled, with a floor), so far points, where the network is unsure, are
smoothed much harder than the near ones, and a road edge (y_std 0.5-1.4 m) much harder than a lane line. A
frame that does not fit the track at all (the network relabelled its lines: a lane change, a junction) is not
averaged in; when the new picture persists the track is re-initialised on it, so nothing between two lanes is
ever reported. An ego-motion compensation of the prediction (slide every line by the distance travelled, turn
it by the yaw of the frame) is implemented and switched off (`slide_gain`, `yaw_gain`, `vy_gain` = 0): on the
replayed rides it made the tracked lines trail the raw ones more, not less (docs/lanes/L15-stability.md: while
the car follows its lane the lines hardly move in its frame, and the slide needs a speed and a camera yaw that
an uncalibrated camera does not give).

Presence. `conf` is a low-pass of the frame probability (`conf_tau_s`); a line appears when `conf` rises
above `on_conf` and disappears when it falls below `off_conf`. While a present line gets no usable frame its
geometry coasts on the prediction (`mode: "coast"`). Road edges carry no probability in the model output;
theirs is made from the near-range `y_std` (an edge the network places within `edge_std_good` m counts as 1,
one it only knows to `edge_std_bad` m as 0).

Map prior (optional, soft). When mapd has matched the road with confidence and the lane count is tagged, the
count says which of the two outer lines (the far boundaries of the neighbouring lanes) can exist: on a
single-lane road neither, on a two-way road with one lane per direction not the right one. An `unlikely`
outer line needs stronger and longer evidence to appear (`on_conf` + `prior_shift`), an `expected` one
appears sooner and is held longer. The inner lines and the edges are never changed by the map, and without a
position source (no mapd, no fix, an assumed lane count) every threshold is the neutral one: the tracker runs
the same way on vision alone.

No I/O and no wall clock in here; times come from the messages (CLOCK_MONOTONIC ns).
"""
import math

import numpy as np

from ..contract import LocalEgo, validity

NS = 1e9
N_LANE, N_EDGE = 4, 2
N_LINES = N_LANE + N_EDGE
LINE_NAMES = ("far_left", "left", "right", "far_right", "edge_left", "edge_right")

DEFAULT_CFG = dict(
    # presence
    conf_tau_s=0.5,              # low-pass of the frame probability
    on_conf=0.45, off_conf=0.2,  # hysteresis on the filtered probability (the raw threshold in use was 0.3)
    update_prob=0.2,             # a frame with a lower probability does not update the geometry (coast)
    coast_max_s=1.5,             # a geometry without a usable frame for this long is dropped (the line is then
                                 # not reported until a frame brings it back)
    low_prob_inflate=2.0,        # sigma factor of a frame between update_prob and on_conf
    edge_std_good=0.6, edge_std_bad=2.5, edge_std_x=(10.0, 40.0),   # road-edge pseudo probability from y_std (m)
    # geometry: measurement noise max(r_floor, r_scale * the model's y_std), process noise per sqrt(s)
    r_scale=0.3, r_floor=0.05,
    edge_r_scale=1.0, edge_r_floor=0.25,
    q0=0.2, q1=0.006,           # m/sqrt(s): q0 + q1 * x
    p0_scale=1.5,                # initial sigma = p0_scale * measurement sigma
    max_std=6.0,                 # cap of the filter sigma while coasting, m
    # consistency gate / re-initialisation
    gate_x=(5.0, 45.0),          # the points that decide whether a frame fits the track
    gate_sigma=4.0, gate_min_m=0.8,
    snap_s=0.2, snap_frames=2,   # a frame that does not fit, repeated this long, replaces the track
    cand_keep_s=0.6,             # ... counted across unusable frames for this long
    snap_tol_m=0.6,              # ... when the misfitting frames agree with each other within this
    # ego-motion compensation, off (see the module docstring): y(x) <- y(x + slide_gain * v dt)
    #   + yaw_gain * yaw_rate * dt * x + vy_gain * vy * dt; the physical values are 1, -1, -1 (the model's yaw
    #   rate is positive to the right)
    yaw_gain=0.0, vy_gain=0.0, slide_gain=0.0,
    # timing
    max_gap_s=1.0,               # road messages further apart than this restart the tracker
    ego_max_age_s=0.5,           # vehicle speed older than this (against the frame time) is not used
    # map prior
    map_max_age_s=3.0, map_min_confidence=0.5,
    prior_shift=0.25,            # added to on_conf (unlikely) / taken from it (expected, half of it)
    prior_off_shift=0.08,        # off_conf moves by this (down for expected, up for unlikely)
    # derived
    width_x=(6.0, 30.0),         # lane width is measured between the inner lines over these distances
)


def _fin(v):
    return isinstance(v, (int, float)) and math.isfinite(v)


def _arr(v, n):
    try:
        a = np.asarray(v, float)
    except (TypeError, ValueError):
        return None
    if a.shape != (n,) or not np.all(np.isfinite(a)):
        return None
    return a


def map_prior(map_msg, now_ns, cfg=DEFAULT_CFG):
    """Lane-count prior for the two outer lines from a mapd `map` message, or None when the map says nothing
    usable (no message, stale, not available, a doubtful match, an assumed count).

    Returns {"total", "forward", "backward", "source", "outer_left", "outer_right"} with the outer entries one
    of "expected", "neutral", "unlikely"."""
    if not isinstance(map_msg, dict) or not map_msg.get("available"):
        return None
    ts = map_msg.get("ts_mono_ns")
    if _fin(ts) and now_ns is not None and (now_ns - ts) / NS > cfg["map_max_age_s"]:
        return None
    conf = (map_msg.get("match") or {}).get("confidence")
    if not _fin(conf) or conf < cfg["map_min_confidence"]:
        return None
    ln = (map_msg.get("road") or {}).get("lanes") or {}
    src = ln.get("source")
    if src not in ("tagged", "tagged_total"):
        return None
    total, fwd, bwd = ln.get("total"), ln.get("forward"), ln.get("backward")
    if not (_fin(total) and total >= 1):
        return None
    left = right = "neutral"
    if total == 1:
        left = right = "unlikely"                 # one lane: nothing beside it
    elif _fin(fwd) and _fin(bwd):
        if fwd == 1 and bwd >= 1:
            right = "unlikely"                    # the own direction has one lane: no lane to the right
            left = "expected"                     # the oncoming lane's far boundary
        elif fwd == 1 and bwd == 0:
            left = right = "unlikely"
        elif fwd >= 3:
            left = right = "neutral"              # which lane the car is in is not known from the count
    return {"total": int(total), "forward": int(fwd) if _fin(fwd) else None,
            "backward": int(bwd) if _fin(bwd) else None, "source": src,
            "outer_left": left, "outer_right": right}


class RoadGeometryTracker:
    def __init__(self, cfg=None):
        self.cfg = dict(DEFAULT_CFG)
        if cfg:
            self.cfg.update(cfg)
        self._local = LocalEgo()    # the integrator of a tracker that is not handed an ego state (docs/DESIGN.md)
        self._map = None            # the last `map` message
        self.reset()

    # ---- life cycle ------------------------------------------------------
    def reset(self):
        self.x = None               # the model's 33 distances
        self.y = None               # [6, n] filtered lateral offsets
        self.p = None               # [6, n] variances
        self.have = np.zeros(N_LINES, bool)       # a geometry track exists
        self.present = np.zeros(N_LINES, bool)    # reported as present (hysteresis)
        self.conf = np.zeros(N_LINES)
        self.since = np.zeros(N_LINES)            # ts (s) of the last presence change
        self.unsupported = np.zeros(N_LINES)      # seconds without a usable frame
        self.cand_t = [None] * N_LINES            # ts (s) of the first misfitting frame
        self.cand_n = np.zeros(N_LINES, int)
        self.cand_y = [None] * N_LINES
        self.snaps = 0
        self.ts = None
        self.seq = None

    def on_ego(self, msg):
        """mapd's relay of the vehicle motion, for a tracker without an integrator of the api's."""
        self._local.feed(msg)

    def on_map(self, msg):
        self._map = msg if isinstance(msg, dict) else None

    # ---- inputs ----------------------------------------------------------
    @staticmethod
    def _motion(es):
        """(speed, source, yaw rate positive right) of the frame from the integrator's `ego_state`."""
        sp, yr = es.get("speed") or {}, es.get("yaw_rate") or {}
        v = abs(sp["value"]) if sp.get("valid") else 0.0
        src = "vehicle" if any(x.startswith("vehicle.") for x in sp.get("sources") or ()) else "vision"
        return v, src, (-yr["value"] if yr.get("valid") else 0.0)

    def _measure(self, road):
        """-> (x[n], y[6,n], std[6,n], prob[6], ok[6]) from a road message, or None when it carries no lines."""
        c = self.cfg
        ll = road.get("lane_lines") or []
        re = road.get("road_edges") or []
        if not ll and not re:
            return None
        x = None
        for ln in list(ll) + list(re):
            if isinstance(ln, dict) and isinstance(ln.get("x"), list) and len(ln["x"]) >= 4:
                x = _arr(ln["x"], len(ln["x"]))
                if x is not None:
                    break
        if x is None:
            return None
        n = len(x)
        y = np.zeros((N_LINES, n))
        s = np.ones((N_LINES, n))
        prob = np.zeros(N_LINES)
        ok = np.zeros(N_LINES, bool)
        near = (x >= c["edge_std_x"][0]) & (x <= c["edge_std_x"][1])
        for i in range(N_LINES):
            src = ll[i] if i < N_LANE and i < len(ll) else re[i - N_LANE] if i >= N_LANE and i - N_LANE < len(re) else None
            if not isinstance(src, dict):
                continue
            yi, si = _arr(src.get("y"), n), _arr(src.get("y_std"), n)
            if yi is None:
                continue
            if si is None:
                si = np.full(n, 0.3 if i < N_LANE else 1.0)
            y[i], s[i] = yi, np.maximum(si, 1e-3)
            if i < N_LANE:
                p = src.get("prob")
                prob[i] = min(1.0, max(0.0, float(p))) if _fin(p) else 0.0
            else:
                sd = float(np.median(si[near])) if near.any() else float(np.median(si))
                prob[i] = min(1.0, max(0.0, (c["edge_std_bad"] - sd) / (c["edge_std_bad"] - c["edge_std_good"])))
            ok[i] = True
        return x, y, s, prob, ok

    # ---- one frame -------------------------------------------------------
    def on_road(self, road, ego=None):
        """Feed one `road` message with the integrator's `ego_state` for its frame; returns the `lanes` message
        (dict) or None when the message has no lines. While the integrator holds the vision source faulted the
        lines are unknown: no line is present and the message says `valid: false` with the reason."""
        c = self.cfg
        ts = road.get("ts_mono_ns")
        seq = road.get("seq")
        if not _fin(ts):
            return None
        es = ego if ego is not None else self._local.at(road)
        z = self._measure(road)
        if z is None:
            return None
        x, zy, zs, prob, ok = z
        self._validity = validity(es, ("vision",), ())
        if not self._validity[0]:
            self.reset()
            self.x = x
            self.y = np.zeros((N_LINES, len(x)))
            self.p = np.ones((N_LINES, len(x)))
            self.ts, self.seq = ts / NS, seq
            self.since[:] = self.ts
            v, vsrc, yaw = self._motion(es)
            return self._message(road, np.zeros(N_LINES), np.zeros(N_LINES, bool), v, vsrc, yaw, None)
        t = ts / NS
        dt = None if self.ts is None else t - self.ts
        if (dt is None or dt <= 0 or dt > c["max_gap_s"] or self.x is None or len(self.x) != len(x)
                or (_fin(seq) and _fin(self.seq) and seq < self.seq)):
            self.reset()
            self.x = x
            self.y = np.zeros((N_LINES, len(x)))
            self.p = np.ones((N_LINES, len(x)))
            self.since[:] = t
            dt = 0.0
        self.ts, self.seq = t, seq
        v, vsrc, yaw = self._motion(es)
        pose = road.get("pose") or {}
        vy = float(pose["vy"]) if _fin(pose.get("vy")) else 0.0     # only the camera measures the lateral speed
        prior = map_prior(self._map, ts, c)

        if dt > 0:
            self._predict(dt, v, yaw, vy)

        # measurement noise
        r_lane = np.maximum(c["r_floor"], c["r_scale"] * zs[:N_LANE])
        r_edge = np.maximum(c["edge_r_floor"], c["edge_r_scale"] * zs[N_LANE:])
        r = np.vstack((r_lane, r_edge))
        gate = (x >= c["gate_x"][0]) & (x <= c["gate_x"][1])
        if not gate.any():
            gate[:] = True
        a = 1.0 - math.exp(-dt / c["conf_tau_s"]) if dt > 0 else 1.0
        # how well this frame fits every track, for all lines at once (cheap on a phone core)
        r = r * np.where(prob < c["on_conf"], c["low_prob_inflate"], 1.0)[:, None]   # a weak frame weighs less
        nu_all = zy - self.y
        an = np.abs(nu_all[:, gate])
        jumps = an.mean(axis=1)
        nsigs = (an / np.sqrt(self.p[:, gate] + r[:, gate] ** 2)).mean(axis=1)
        upd = []

        for i in range(N_LINES):
            # presence (hysteresis on the filtered probability, thresholds moved by the map prior)
            p_i = prob[i] if ok[i] else 0.0
            self.conf[i] += (p_i - self.conf[i]) * a
            on, off = c["on_conf"], c["off_conf"]
            if prior is not None and i in (0, 3):
                tag = prior["outer_left"] if i == 0 else prior["outer_right"]
                if tag == "unlikely":
                    on, off = on + c["prior_shift"], off + c["prior_off_shift"]
                elif tag == "expected":
                    on, off = on - 0.5 * c["prior_shift"], off - c["prior_off_shift"]
            was = bool(self.present[i])
            if not was and self.conf[i] >= on:
                self.present[i] = True
            elif was and self.conf[i] < off:
                self.present[i] = False
            if bool(self.present[i]) != was:
                self.since[i] = t

            # geometry
            usable = ok[i] and p_i >= c["update_prob"]
            if not usable:
                self.unsupported[i] += dt
                if self.cand_t[i] is not None and t - self.cand_t[i] > c["cand_keep_s"] + c["snap_s"]:
                    self.cand_t[i], self.cand_n[i], self.cand_y[i] = None, 0, None
                if self.have[i] and self.unsupported[i] > c["coast_max_s"]:
                    self.have[i] = False               # forgotten: the next usable frame starts afresh
                continue
            ri = r[i]
            if not self.have[i]:
                self._init(i, zy[i], ri)
                continue
            if jumps[i] > c["gate_min_m"] and nsigs[i] > c["gate_sigma"]:
                # the frame does not fit the track: collect it, replace the track when it persists
                if self.cand_y[i] is not None and \
                        float(np.abs(zy[i][gate] - self.cand_y[i][gate]).mean()) <= c["snap_tol_m"]:
                    self.cand_n[i] += 1
                else:
                    self.cand_t[i], self.cand_n[i] = t, 1
                self.cand_y[i] = zy[i].copy()
                self.unsupported[i] += dt
                if self.cand_n[i] >= c["snap_frames"] and t - self.cand_t[i] >= c["snap_s"]:
                    self._init(i, zy[i], ri)
                    self.snaps += 1
                continue
            self.cand_t[i], self.cand_n[i], self.cand_y[i] = None, 0, None
            upd.append(i)
            self.unsupported[i] = 0.0
        if upd:
            pu, ru = self.p[upd], r[upd]
            k = pu / (pu + ru * ru)
            self.y[upd] += k * nu_all[upd]
            self.p[upd] = pu * (1.0 - k)
        return self._message(road, prob, ok, v, vsrc, yaw, prior)

    def _init(self, i, y, r):
        self.y[i] = y
        self.p[i] = (self.cfg["p0_scale"] * r) ** 2
        self.have[i] = True
        self.unsupported[i] = 0.0
        self.cand_t[i], self.cand_n[i], self.cand_y[i] = None, 0, None

    def _predict(self, dt, v, yaw, vy):
        """Process noise (and the optional ego-motion compensation) for every tracked line."""
        c = self.cfg
        x = self.x
        shift = c["slide_gain"] * max(0.0, v) * dt
        q = (c["q0"] + c["q1"] * x) ** 2 * dt
        cap = c["max_std"] ** 2
        if shift == 0.0 and c["yaw_gain"] == 0.0 and c["vy_gain"] == 0.0:    # the default: a random walk
            np.minimum(self.p + q, cap, out=self.p)
            return
        dy = c["yaw_gain"] * yaw * dt * x + c["vy_gain"] * vy * dt
        for i in range(N_LINES):
            if not self.have[i]:
                continue
            if shift > 0:
                xs = x + shift
                yi = np.interp(xs, x, self.y[i])
                pi = np.interp(xs, x, self.p[i])
                # beyond the last point: continue with the last slope, and say so in the variance
                over = xs > x[-1]
                if over.any():
                    slope = (self.y[i][-1] - self.y[i][-2]) / (x[-1] - x[-2])
                    yi[over] = self.y[i][-1] + slope * (xs[over] - x[-1])
                    pi[over] = self.p[i][-1] + (0.05 * (xs[over] - x[-1])) ** 2
                self.y[i], self.p[i] = yi, pi
            self.y[i] += dy
            self.p[i] = np.minimum(self.p[i] + q, cap)

    # ---- output ----------------------------------------------------------
    def _line(self, i, prob, ok):
        if not (self.present[i] and self.have[i]):
            return {"present": False, "mode": "none", "conf": round(float(self.conf[i]), 3),
                    "prob": round(float(prob[i]), 3) if ok[i] else None, "y": None, "y_std": None}
        return {"present": True, "mode": "track" if self.unsupported[i] == 0.0 else "coast",
                "conf": round(float(self.conf[i]), 3),
                "prob": round(float(prob[i]), 3) if ok[i] else None,
                "age_s": round(self.ts - float(self.since[i]), 2),
                "y": np.round(self.y[i], 2).tolist(),
                "y_std": np.round(np.sqrt(self.p[i]), 2).tolist()}

    def _message(self, road, prob, ok, v, vsrc, yaw, prior):
        c = self.cfg
        valid, reason, inputs = getattr(self, "_validity", (True, None, {}))
        lines = [self._line(i, prob, ok) for i in range(N_LINES)]
        width = None
        if lines[1]["present"] and lines[2]["present"]:
            m = (self.x >= c["width_x"][0]) & (self.x <= c["width_x"][1])
            if m.any():
                width = round(float(np.median(self.y[2][m] - self.y[1][m])), 2)
        return {
            "type": "lanes", "seq": road.get("seq"), "ts_mono_ns": road.get("ts_mono_ns"),
            "ts_real_ns": road.get("ts_real_ns"),
            "road_model": road.get("road_model"),      # which model's lines these are (lane L27)
            "x": np.round(self.x, 2).tolist(),
            "lines": lines[:N_LANE],                  # far left, left, right, far right
            "edges": lines[N_LANE:],                  # left, right
            "lane_to_left": bool(lines[0]["present"] and lines[1]["present"]),
            "lane_to_right": bool(lines[3]["present"] and lines[2]["present"]),
            "lane_width_m": width,
            "prior": prior,
            "source": "vision+map" if prior is not None else "vision",
            # valid false: the lines are unknown (a source they are made from is faulted), not "no lines"
            "valid": valid, "reason": reason, "inputs": inputs,
            # the integrator's ego motion of the frame (yaw rate positive right, the device frame's convention)
            "ego": {"speed_mps": round(v, 2), "yaw_rate": round(yaw, 4), "source": vsrc},
        }

    def summary(self, msg):
        """Short form for /v1/health."""
        if not msg:
            return None
        return {"lines": "".join("|" if ln["present"] else "." for ln in msg["lines"]),
                "edges": "".join("|" if ln["present"] else "." for ln in msg["edges"]),
                "lane_width_m": msg.get("lane_width_m"), "source": msg.get("source")}
