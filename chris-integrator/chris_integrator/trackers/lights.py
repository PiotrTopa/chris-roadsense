"""Traffic lights: which light applies to the ego lane (adds `ego` and per-light geometry to `lights` messages).

A light provider (roadsense-signs `signd --lights`, docs/ARCHITECTURE.md "lights") publishes tracked light boxes
with a smoothed state. The provider does not know the road; the api does (modeld's `road` plan, the calibration
and the intrinsics), so it adds per light:

  dist_m     distance from the box's long side, assuming a 0.9 m housing (three 200-300 mm lamps):
             f * 0.9 / long_side_px (a 30 px housing at f 690 px = 21 m). Rough (+-30 %), only used for ranking.
  lateral_m  offset of the light from the planned path at that distance, metres (+ = right of the path), from
             the path point projected into the image: (u_light - u_path(dist)) * dist / f.
  relevance  0..1: facing state (red/yellow/red_yellow/green; off/unknown are mostly side or back views) x
             lateral window (lights for the ego lane hang over it or stand at its right edge, a little to the
             left on a median: centre +1.5 m, scale 4.5 m, cut outside -7..+10 m) x distance (full up to 60 m,
             zero at 120 m) x track maturity (hits / 3) x sqrt(detector score)
  ego        true for the selected light

and a message-level `ego`: {track_id, state, flashing, dist_m, lateral_m, relevance, agree, n_relevant, source}
for the most relevant light, whose state is the relevance-weighted vote of the lights within 30 % of the best
(two heads of the same signal group agree), or null when nothing is relevant. Without a road message the
path is the image column of the principal point (source "image").

A light the provider reports while coasting (`coast` > 0: a confirmed track it did not detect in this frame,
roadsense-signs >= 2026-11-16) counts with COAST_FACTOR per missed frame.

Over time (`EgoLightSelector`, used by the api): the choice above is made per message, so it used to drop out
whenever the ego light's relevance dipped under the threshold or its track missed a frame, and consumers saw
the light blink. The selector keeps the light that was the ego light while its relevance stays above
KEEP_RELEVANCE (hysteresis), and when no light qualifies it serves the last ego light for up to HOLD_S more
with `held: true` and its `age_s` (the state shown is then that old).

Map prior (optional, soft): when mapd has matched the road and its look-ahead lists a signal-controlled
junction or crossing within MAP_NEAR_M on the most likely path (`ahead.features` kind `traffic_signals` /
`crossing_signals`), a light is expected: the selector accepts an ego light from MAP_MIN_RELEVANCE instead
of MIN_RELEVANCE, holds it MAP_HOLD_S, and the message carries `map_signal: {kind, dist_m, confidence}` so a
consumer can tell "the map expects a light here and the camera has none" from "no light". Without a usable
map message (no position source, stale, a doubtful match) nothing changes and `map_signal` is null.

Lane assignment ("which of several lights is mine", lane L16): at a junction with separate signal groups (a
left-turn signal beside the one for straight on) the heads that face the car show different states at the
same moment. Then, and only then, the lateral window above is not what decides: every facing head gets a
`slot` (its offset from the centre of the ego lane in lane widths at its distance, from the tracked lane
lines of the `lanes` topic; 0 = over the ego lane, -1 = over the lane to the left) and a lane weight (a
Gaussian in the slot whose width grows with the distance, because the lateral offset of a far light is rough;
a head beyond the outermost lane, i.e. on a post at the roadside, counts as a head of the outermost lane it stands
beside, a little weaker). The heads are split into "stop" (red, red_yellow, yellow) and "go" (green); the group with the best
head wins and the ego light is chosen inside it, `assign.confidence` is that group's share of the two best
scores, and `amb` is set when the share stays under AMB_CONFIDENCE (the selector reports it from the second
message on). The other group is reported as `assign.alt` {state, side}. Without lane lines the slot is
measured from the planned path with a nominal lane width (`basis: "path"`). When all facing heads agree there
is no conflict and the choice is the one above; a lone light that is clearly over another lane (lane weight
under LANE_GATE with lane lines present, nearer than LANE_GATE_D) is still served, since its state is all
there is, with `assign.other_lane` and `amb` set. The assignment never removes a light: it decides between
groups and says how sure it is. A tagged lane count
of the matched road (map, optional) caps the number of lanes assumed beside the ego lane. Arrow lenses are
not told apart: the provider has no arrow state and a lens is 4-9 px wide at the distances that matter.
"""
import math

import numpy as np

from .projection import Projector

HOUSING_M = 0.9
FACING = {"red": 1.0, "yellow": 1.0, "red_yellow": 1.0, "green": 1.0, "off": 0.15, "unknown": 0.15}
LAT_CENTRE, LAT_SCALE, LAT_MIN, LAT_MAX = 1.5, 4.5, -7.0, 10.0
D_FULL, D_ZERO = 60.0, 120.0
MIN_RELEVANCE = 0.15
GROUP_FRAC = 0.3
COAST_FACTOR = 0.85          # relevance factor per frame a reported track has been coasting
KEEP_RELEVANCE = 0.07        # the current ego light stays the ego light down to this relevance
HOLD_S = 1.2                 # the last ego light is served this long after no light qualifies
MAP_NEAR_M = 150.0           # a mapped signal this near on the path = a light is expected
MAP_MIN_RELEVANCE = 0.10     # ... then a light qualifies from this relevance
MAP_HOLD_S = 2.0             # ... and the hold is this long
MAP_MAX_AGE_S = 3.0          # a map message older than this is not used
MAP_MIN_CONFIDENCE = 0.5     # match confidence, and confidence of the look-ahead feature
SIGNAL_KINDS = ("traffic_signals", "crossing_signals")
STOP_STATES, GO_STATES = ("red", "red_yellow", "yellow"), ("green",)
LANE_EVAL_M = (6.0, 30.0)    # the lane lines are read at the light's distance, clamped to this range
LANE_WIDTH_M = 3.3           # nominal lane width when it cannot be measured
LANE_WIDTH_RANGE = (2.5, 4.5)
SLOT_SIGMA = 0.7             # lane weight = exp(-(slot_eff / (SLOT_SIGMA * (1 + d / SLOT_SIGMA_D))) ** 2)
SLOT_SIGMA_D = 60.0
POST_EXCESS = 0.5            # a head beyond the outermost lane (a post): its distance past that lane's centre counts this much
LANE_MIN = 0.12              # in a conflict: lane score (relevance without the lateral window x lane weight) to qualify
LANE_KEEP = 0.06             # ... for the current ego track
LANE_GATE = 0.08             # no conflict: a lone head with a lane weight under this is marked `other_lane` (ambiguous)
LANE_GATE_D = 45.0           # ... when it is nearer than this (the offset of a far light is too rough to say)
LAT_FACING = 14.0            # heads farther than this from the path take no part in the lane assignment
AMB_CONFIDENCE = 0.7         # conflict won with a smaller share than this = ambiguous
AMB_MSGS = 2                 # ... reported (`amb`) from this many messages in a row


def _path_u(road, proj, d):
    """Image column of the planned path at distance d (device x), or None."""
    plan = (road or {}).get("plan") or {}
    xs = plan.get("x")
    if not xs:
        return None
    x = np.asarray(xs, float)
    y = np.asarray(plan.get("y") or np.zeros_like(x), float)
    z = np.asarray(plan.get("z") or np.zeros_like(x), float)
    if d <= x[0] or d > x[-1]:
        d = min(max(d, x[0]), x[-1])
    yi, zi = float(np.interp(d, x, y)), float(np.interp(d, x, z)) + proj.height
    pts = proj.project([d], [yi], [zi])
    return pts[0][0] if pts else None


def map_signal(map_msg, now_s=None):
    """The nearest signal on the path from a mapd `map` message: {"kind", "dist_m", "confidence"}, or None
    when the map says nothing usable (no message, not available, stale, a doubtful match, no signal near)."""
    if not isinstance(map_msg, dict) or not map_msg.get("available"):
        return None
    ts = map_msg.get("ts_mono_ns")
    if now_s is not None and isinstance(ts, (int, float)) and now_s - ts / 1e9 > MAP_MAX_AGE_S:
        return None
    conf = (map_msg.get("match") or {}).get("confidence")
    if not isinstance(conf, (int, float)) or conf < MAP_MIN_CONFIDENCE:
        return None
    best = None
    for ft in (map_msg.get("ahead") or {}).get("features") or []:
        if not isinstance(ft, dict) or ft.get("kind") not in SIGNAL_KINDS:
            continue
        d, fc = ft.get("dist_m"), ft.get("confidence")
        if not isinstance(d, (int, float)) or d < 0 or d > MAP_NEAR_M:
            continue
        if isinstance(fc, (int, float)) and fc < MAP_MIN_CONFIDENCE:
            continue
        if best is None or d < best["dist_m"]:
            best = {"kind": ft["kind"], "dist_m": round(float(d), 1),
                    "confidence": round(float(fc), 2) if isinstance(fc, (int, float)) else None}
    return best


def _at(xs, ys, d):
    """Linear interpolation of a line (x increasing) at d, or None when a value is missing."""
    try:
        v = float(np.interp(d, np.asarray(xs, float), np.asarray(ys, float)))
    except (TypeError, ValueError):
        return None
    return v if math.isfinite(v) else None


def lane_frame(lanes, road, d, lanes_forward=None):
    """The ego lane at a light's distance: {"c": lane centre minus path (m, + right), "w": lane width,
    "n_left" / "n_right": lanes between the ego lane and the road edge on that side (None = unknown),
    "basis": "lanes" | "path"}. `lanes` is the api's `lanes` message (tracked lines); without usable inner
    lines the frame is the planned path with a nominal width. `lanes_forward`: a tagged lane count of the
    matched road in the car's direction (map), which caps the lanes beside the ego lane."""
    out = {"c": 0.0, "w": LANE_WIDTH_M, "n_left": None, "n_right": None, "basis": "path"}
    if isinstance(lanes, dict) and isinstance(lanes.get("x"), list):
        dq = min(max(d, LANE_EVAL_M[0]), LANE_EVAL_M[1])
        xs = lanes["x"]
        plan = (road or {}).get("plan") or {}
        yp = _at(plan.get("x"), plan.get("y"), dq) if plan.get("x") else 0.0
        if yp is None:
            yp = 0.0

        def y_of(ln):
            if not isinstance(ln, dict) or ln.get("present") is not True or not ln.get("y"):
                return None
            return _at(xs, ln["y"], dq)
        lines = [y_of(ln) for ln in (lanes.get("lines") or [])[:4]] + [None] * 4
        edges = [y_of(ln) for ln in (lanes.get("edges") or [])[:2]] + [None] * 2
        yl, yr = lines[1], lines[2]
        w0 = lanes.get("lane_width_m")
        w0 = float(w0) if isinstance(w0, (int, float)) and LANE_WIDTH_RANGE[0] <= w0 <= LANE_WIDTH_RANGE[1] \
            else LANE_WIDTH_M
        if yl is not None and yr is not None and yr > yl:
            w = min(max(yr - yl, LANE_WIDTH_RANGE[0]), LANE_WIDTH_RANGE[1])
            out.update(c=0.5 * (yl + yr) - yp, w=w, basis="lanes")
        elif yl is not None:
            out.update(c=yl + 0.5 * w0 - yp, w=w0, basis="lanes")
            yr = yl + w0
        elif yr is not None:
            out.update(c=yr - 0.5 * w0 - yp, w=w0, basis="lanes")
            yl = yr - w0
        if out["basis"] == "lanes":
            w = out["w"]
            if edges[0] is not None:
                out["n_left"] = int(min(4, max(0, round((yl - edges[0] - 0.3) / w))))
            if lines[0] is not None:
                out["n_left"] = max(out["n_left"] or 0, 1)
            if edges[1] is not None:
                out["n_right"] = int(min(4, max(0, round((edges[1] - yr - 0.3) / w))))
            if lines[3] is not None:
                out["n_right"] = max(out["n_right"] or 0, 1)
    if isinstance(lanes_forward, int) and lanes_forward >= 1:
        cap = lanes_forward - 1                  # lanes of the own direction beside the ego lane
        if out["n_right"] is None or out["n_right"] > cap:
            out["n_right"] = cap
    return out


def lane_weight(lat, d, fr):
    """(slot, weight 0..1): the head's offset from the ego-lane centre in lane widths, and how much it counts
    as a light for the ego lane (module docstring, "Lane assignment")."""
    s = (lat - fr["c"]) / fr["w"]
    n = fr["n_right"] if s > 0 else fr["n_left"]
    a = abs(s)
    if n is None:                                # unknown: a post is expected on the right, not on the left
        eff = a if a <= 0.5 or s < 0 else 0.5 + POST_EXCESS * (a - 0.5)
    elif a > n + 0.5:                            # beyond the outermost lane: a post at the roadside
        eff = n + POST_EXCESS * (a - n)
    else:
        eff = a
    sig = SLOT_SIGMA * (1.0 + d / SLOT_SIGMA_D)
    return s, math.exp(-(eff / sig) ** 2)


def _colour(state):
    return "stop" if state in STOP_STATES else "go" if state in GO_STATES else None


def annotate(msg, road=None, calib=None, intrinsics=(690.0, 640.0, 360.0, 1280, 720), keep_track=None,
             min_relevance=MIN_RELEVANCE, lanes=None, lanes_forward=None, lane_assign=True):
    """Adds dist_m / lateral_m / relevance / ego to every light of `msg` (in place) and msg['ego']; returns msg.
    `keep_track`: the track that was the ego light so far; it qualifies down to KEEP_RELEVANCE.
    `lanes`: the api's `lanes` message for the lane assignment; `lane_assign=False` is the selection as it
    was before lane L16 (lateral window only)."""
    f, cx, cy, w, h = intrinsics
    fw, fh = (msg.get("frame") or [w, h])[:2]
    proj = Projector((f * fw / w, cx * fw / w, cy * fw / w, fw, fh), calib, (fw, fh))   # frame pixels
    fpx = proj.f
    best, cands = None, []
    source = "path" if (road or {}).get("plan", {}).get("x") else "image"
    for li in msg.get("lights") or []:
        box = li.get("box")
        if not box or len(box) != 4:
            continue
        bw, bh = max(1.0, box[2] - box[0]), max(1.0, box[3] - box[1])
        d = fpx * HOUSING_M / max(bw, bh)
        u = 0.5 * (box[0] + box[2])
        up = _path_u(road, proj, d) if source == "path" else None
        if up is None:
            up = proj.cx
        lat = (u - up) * d / fpx
        state = li.get("state") or "unknown"
        rel = FACING.get(state, 0.15)
        rel *= math.exp(-((lat - LAT_CENTRE) / LAT_SCALE) ** 2) if LAT_MIN <= lat <= LAT_MAX else 0.0
        rel *= max(0.0, min(1.0, (D_ZERO - d) / (D_ZERO - D_FULL)))
        rel *= min(1.0, (li.get("hits") or 1) / 3.0)
        rel *= math.sqrt(max(0.0, float(li.get("score") or 0.0)))
        rel *= COAST_FACTOR ** max(0, int(li.get("coast") or 0))
        base = FACING.get(state, 0.15) * max(0.0, min(1.0, (D_ZERO - d) / (D_ZERO - D_FULL)))
        base *= min(1.0, (li.get("hits") or 1) / 3.0) * math.sqrt(max(0.0, float(li.get("score") or 0.0)))
        base *= COAST_FACTOR ** max(0, int(li.get("coast") or 0))
        li.update(dist_m=round(d, 1), lateral_m=round(lat, 1), relevance=round(rel, 3), ego=False)
        li.pop("slot", None)
        if lane_assign:
            fr = lane_frame(lanes, road, d, lanes_forward)
            slot, wl = lane_weight(lat, d, fr)
            li["slot"] = round(slot, 2)
            li["_lane"] = (fr, wl, base * wl if abs(lat) <= LAT_FACING else 0.0)
        cands.append(li)
        if rel >= min_relevance and (best is None or rel > best["relevance"]):
            best = li
    assign = None
    if lane_assign:
        best, assign = _assign(cands, best, keep_track)
    if best is None and keep_track is not None and not (assign and assign["conflict"]):
        # hysteresis: the light that was the ego light so far
        kept = [li for li in cands if li.get("track_id") == keep_track and li["relevance"] >= KEEP_RELEVANCE]
        best = kept[0] if kept else None
        if best is not None and lane_assign:
            assign = _assign_single(best)
    for li in cands:
        li.pop("_lane", None) if li is not best else None
    if best is None:
        msg["ego"] = None
        return msg
    best["ego"] = True
    best.pop("_lane", None)
    group = [li for li in cands if li["relevance"] >= GROUP_FRAC * best["relevance"]]
    if assign and assign["conflict"]:                # two signal groups: the vote stays inside the chosen one
        col = _colour(best.get("state"))
        group = [li for li in group if _colour(li.get("state")) == col] or [best]
    votes = {}
    for li in group:
        votes[li.get("state")] = votes.get(li.get("state"), 0.0) + li["relevance"]
    state = max(votes, key=votes.get)
    msg["ego"] = {"track_id": best.get("track_id"), "state": state,
                  "flashing": bool(best.get("flashing")) and state == "yellow",
                  "dist_m": best["dist_m"], "lateral_m": best["lateral_m"], "relevance": best["relevance"],
                  "agree": sum(1 for li in group if li.get("state") == state), "n_relevant": len(group),
                  "source": source}
    if assign is not None:
        msg["ego"]["assign"] = assign
        msg["ego"]["amb"] = bool(assign["conflict"] and assign["confidence"] < AMB_CONFIDENCE
                                 or assign.get("other_lane"))
    return msg


def _assign_single(best):
    fr, wl, _ = best["_lane"]
    out = {"conflict": False, "confidence": round(min(1.0, wl / 0.5), 2), "basis": fr["basis"],
           "slot": best["slot"], "n_left": fr["n_left"], "n_right": fr["n_right"], "alt": None}
    if fr["basis"] == "lanes" and wl < LANE_GATE and best["dist_m"] <= LANE_GATE_D:
        out["other_lane"] = True                 # the only facing light, and it hangs over another lane
    return out


def _assign(cands, best, keep_track):
    """Lane assignment over the annotated candidates: (the ego light or None, the `assign` object or None)."""
    groups = {"stop": [], "go": []}
    for li in cands:
        col = _colour(li.get("state"))
        if col and li["_lane"][2] > 0.0 and (li.get("hits") or 1) >= 2:
            groups[col].append(li)
    if not (groups["stop"] and groups["go"]):
        if best is None:
            return None, None
        return best, _assign_single(best)
    top = {c: max(g, key=lambda li: li["_lane"][2]) for c, g in groups.items()}
    sc = {c: top[c]["_lane"][2] for c in top}
    win = "stop" if sc["stop"] >= sc["go"] else "go"
    lose = "go" if win == "stop" else "stop"
    share = sc[win] / (sc[win] + sc[lose])
    kept = [li for li in groups[win] if li.get("track_id") == keep_track and li["_lane"][2] >= LANE_KEEP]
    pick = kept[0] if kept and kept[0]["_lane"][2] >= 0.6 * sc[win] else top[win]
    alt = top[lose]
    fr = pick["_lane"][0]
    assign = {"conflict": True, "confidence": round(share, 2), "basis": fr["basis"], "slot": pick["slot"],
              "n_left": fr["n_left"], "n_right": fr["n_right"],
              "alt": {"track_id": alt.get("track_id"), "state": alt.get("state"),
                      "side": "left" if alt["lateral_m"] < pick["lateral_m"] else "right",
                      "slot": alt["slot"]}}
    if pick["_lane"][2] < (LANE_KEEP if kept and pick is kept[0] else LANE_MIN):
        return None, assign
    return pick, assign


class EgoLightSelector:
    """The ego light over time: `annotate` with hysteresis on the current ego track and a short hold when no
    light qualifies (module docstring). One instance per camera stream; `reset` on a ride change."""

    def __init__(self, hold_s=HOLD_S):
        self.hold_s = hold_s
        self.map = None
        self.lanes = None
        self.lane_assign = True
        self.reset()

    def reset(self):
        self.track = None           # track_id of the current ego light
        self.last = None            # the last ego dict that came from a frame
        self.last_ts = None
        self.amb_run = 0

    def on_lanes(self, msg):
        """The api's `lanes` message (tracked lane lines): the frame of the lane assignment."""
        self.lanes = msg if isinstance(msg, dict) else None

    def _lanes_forward(self, ts):
        """A tagged lane count in the car's direction from a usable `map` message, else None."""
        mp = self.map
        if not isinstance(mp, dict) or not mp.get("available"):
            return None
        t = mp.get("ts_mono_ns")
        if ts is not None and isinstance(t, (int, float)) and ts - t / 1e9 > MAP_MAX_AGE_S:
            return None
        conf = (mp.get("match") or {}).get("confidence")
        ln = (mp.get("road") or {}).get("lanes") or {}
        if not isinstance(conf, (int, float)) or conf < MAP_MIN_CONFIDENCE or ln.get("source") != "tagged":
            return None
        fwd = ln.get("forward")
        return int(fwd) if isinstance(fwd, (int, float)) and fwd >= 1 else None

    def on_map(self, msg):
        """mapd's `map` message: the signals on the path ahead (the soft prior of the module docstring)."""
        self.map = msg if isinstance(msg, dict) else None

    def annotate(self, msg, road=None, calib=None, intrinsics=(690.0, 640.0, 360.0, 1280, 720)):
        ts = msg.get("ts_mono_ns")
        ts = ts / 1e9 if isinstance(ts, (int, float)) else None
        if ts is not None and self.last_ts is not None and (ts < self.last_ts or ts - self.last_ts > 10.0):
            self.reset()            # the stream restarted
        sig = map_signal(self.map, ts)
        lanes = self.lanes
        lt = lanes.get("ts_mono_ns") if isinstance(lanes, dict) else None
        if ts is not None and isinstance(lt, (int, float)) and abs(ts - lt / 1e9) > 1.0:
            lanes = None            # stale lane lines: the planned path is the frame
        annotate(msg, road, calib, intrinsics, keep_track=self.track,
                 min_relevance=MAP_MIN_RELEVANCE if sig else MIN_RELEVANCE,
                 lanes=lanes, lanes_forward=self._lanes_forward(ts), lane_assign=self.lane_assign)
        msg["map_signal"] = sig
        ego = msg.get("ego")
        if ego is not None and "amb" in ego:        # ambiguity is reported once it lasts
            self.amb_run = self.amb_run + 1 if ego["amb"] else 0
            ego["amb"] = self.amb_run >= AMB_MSGS
        elif ego is None:
            self.amb_run = 0
        hold_s = max(self.hold_s, MAP_HOLD_S) if sig and self.hold_s > 0 else self.hold_s
        ego = msg.get("ego")
        if ego is not None:
            ego["held"] = False
            self.track, self.last, self.last_ts = ego.get("track_id"), dict(ego), ts
            return msg
        self.track = None
        if self.last is not None and ts is not None and self.last_ts is not None \
                and ts - self.last_ts <= hold_s:
            msg["ego"] = dict(self.last, held=True, age_s=round(ts - self.last_ts, 2))
        else:
            self.last = None
        return msg
