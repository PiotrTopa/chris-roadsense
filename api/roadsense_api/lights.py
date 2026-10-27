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


def annotate(msg, road=None, calib=None, intrinsics=(690.0, 640.0, 360.0, 1280, 720)):
    """Adds dist_m / lateral_m / relevance / ego to every light of `msg` (in place) and msg['ego']; returns msg."""
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
        li.update(dist_m=round(d, 1), lateral_m=round(lat, 1), relevance=round(rel, 3), ego=False)
        cands.append(li)
        if rel >= MIN_RELEVANCE and (best is None or rel > best["relevance"]):
            best = li
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
