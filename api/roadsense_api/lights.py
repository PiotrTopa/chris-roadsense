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
"""
import math

import numpy as np

from .overlay import Projector

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


def annotate(msg, road=None, calib=None, intrinsics=(690.0, 640.0, 360.0, 1280, 720), keep_track=None,
             min_relevance=MIN_RELEVANCE):
    """Adds dist_m / lateral_m / relevance / ego to every light of `msg` (in place) and msg['ego']; returns msg.
    `keep_track`: the track that was the ego light so far; it qualifies down to KEEP_RELEVANCE."""
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
        li.update(dist_m=round(d, 1), lateral_m=round(lat, 1), relevance=round(rel, 3), ego=False)
        cands.append(li)
        if rel >= min_relevance and (best is None or rel > best["relevance"]):
            best = li
    if best is None and keep_track is not None:       # hysteresis: the light that was the ego light so far
        kept = [li for li in cands if li.get("track_id") == keep_track and li["relevance"] >= KEEP_RELEVANCE]
        best = kept[0] if kept else None
    if best is None:
        msg["ego"] = None
        return msg
    best["ego"] = True
    group = [li for li in cands if li["relevance"] >= GROUP_FRAC * best["relevance"]]
    votes = {}
    for li in group:
        votes[li.get("state")] = votes.get(li.get("state"), 0.0) + li["relevance"]
    state = max(votes, key=votes.get)
    msg["ego"] = {"track_id": best.get("track_id"), "state": state,
                  "flashing": bool(best.get("flashing")) and state == "yellow",
                  "dist_m": best["dist_m"], "lateral_m": best["lateral_m"], "relevance": best["relevance"],
                  "agree": sum(1 for li in group if li.get("state") == state), "n_relevant": len(group),
                  "source": source}
    return msg


class EgoLightSelector:
    """The ego light over time: `annotate` with hysteresis on the current ego track and a short hold when no
    light qualifies (module docstring). One instance per camera stream; `reset` on a ride change."""

    def __init__(self, hold_s=HOLD_S):
        self.hold_s = hold_s
        self.map = None
        self.reset()

    def reset(self):
        self.track = None           # track_id of the current ego light
        self.last = None            # the last ego dict that came from a frame
        self.last_ts = None

    def on_map(self, msg):
        """mapd's `map` message: the signals on the path ahead (the soft prior of the module docstring)."""
        self.map = msg if isinstance(msg, dict) else None

    def annotate(self, msg, road=None, calib=None, intrinsics=(690.0, 640.0, 360.0, 1280, 720)):
        ts = msg.get("ts_mono_ns")
        ts = ts / 1e9 if isinstance(ts, (int, float)) else None
        if ts is not None and self.last_ts is not None and (ts < self.last_ts or ts - self.last_ts > 10.0):
            self.reset()            # the stream restarted
        sig = map_signal(self.map, ts)
        annotate(msg, road, calib, intrinsics, keep_track=self.track,
                 min_relevance=MAP_MIN_RELEVANCE if sig else MIN_RELEVANCE)
        msg["map_signal"] = sig
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
