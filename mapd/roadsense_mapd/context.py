"""Map context of a matched position: the current road, its speed limit, and the look-ahead along the
most likely path (shape, curvature, junctions, crossings, signals, tunnels, built-up and limit changes)."""
import json
import math
import os

import numpy as np

from . import geo
from . import rsmap as R

CLASS_RANK = {"motorway": 0, "motorway_link": 1, "trunk": 1, "trunk_link": 2, "primary": 2, "primary_link": 3,
              "secondary": 3, "secondary_link": 4, "tertiary": 4, "tertiary_link": 5, "unclassified": 5,
              "residential": 6, "living_street": 7, "service": 8, "track": 9, "unknown": 9}
MINOR = ("service", "track")


def load_limits(country="pl"):
    with open(os.path.join(os.path.dirname(__file__), "limits", country.lower() + ".json")) as f:
        return json.load(f)


# ---- current road --------------------------------------------------------------------------------
def lanes(e, direction):
    """Lane counts seen from the car: forward = own direction, backward = oncoming."""
    fwd, bwd = (e.lanes_fwd, e.lanes_bwd) if direction > 0 else (e.lanes_bwd, e.lanes_fwd)
    if e.oneway:
        if e.lanes:
            return {"total": e.lanes, "forward": e.lanes, "backward": 0, "source": "tagged"}
        n = 2 if e.cls_name in ("motorway", "trunk") and e.flags & R.F_DUAL else 1
        return {"total": n, "forward": n, "backward": 0, "source": "assumed"}
    if e.flags2 & R.F2_LANES_SPLIT and fwd and bwd:
        return {"total": e.lanes or fwd + bwd, "forward": fwd, "backward": bwd, "source": "tagged"}
    if e.lanes >= 2 and e.lanes % 2 == 0:
        return {"total": e.lanes, "forward": e.lanes // 2, "backward": e.lanes // 2, "source": "tagged_total"}
    if e.lanes == 1:
        return {"total": 1, "forward": 1, "backward": 1, "source": "tagged_total"}     # one shared lane
    if e.lanes >= 3:
        f = fwd or (e.lanes - bwd if bwd else e.lanes // 2)
        return {"total": e.lanes, "forward": f, "backward": e.lanes - f, "source": "tagged" if fwd or bwd else "assumed"}
    return {"total": 2, "forward": 1, "backward": 1, "source": "assumed"}


def is_expressway(e, limits=None):
    """An expressway (droga ekspresowa): motorroad=yes, or a road mapped as a motorway whose refs are all
    S-numbers (Polish expressways are commonly tagged highway=motorway; only the ref tells them apart)."""
    if e.flags & R.F_MOTORROAD:
        return True
    if e.cls_name not in ("motorway", "motorway_link") or not e.road_ref:
        return False
    prefix = (limits or {}).get("expressway_ref_prefix", "S")
    refs = [r.strip() for r in e.road_ref.split(";") if r.strip()]
    return bool(refs) and all(r.startswith(prefix) and r[len(prefix):len(prefix) + 1].isdigit() for r in refs)


def built_up(e):
    if e.flags & R.F_BUILTUP_TAGGED:
        return {"value": bool(e.flags & R.F_BUILTUP), "source": "tagged"}
    if e.cls_name in ("motorway", "motorway_link") or e.flags & R.F_MOTORROAD:
        return {"value": False, "source": "class"}
    if e.cls_name == "living_street":
        return {"value": True, "source": "class"}
    return {"value": bool(e.flags & R.F_BUILTUP), "source": "landuse"}


def road_info(e, direction):
    return {
        "class": e.cls_name, "name": e.name or None, "ref": e.road_ref or None,
        "oneway": e.oneway, "lanes": lanes(e, direction),
        "tunnel": bool(e.flags & R.F_TUNNEL), "bridge": bool(e.flags & R.F_BRIDGE),
        "covered": bool(e.flags & R.F_COVERED),
        "lit": True if e.flags & R.F_LIT_YES else False if e.flags & R.F_LIT_NO else None,
        "surface": R.SURFACES[e.surface] if e.surface < len(R.SURFACES) else "unknown",
        "roundabout": bool(e.flags & R.F_ROUNDABOUT), "link": bool(e.flags & R.F_LINK),
        "motorroad": bool(e.flags & R.F_MOTORROAD), "expressway": is_expressway(e),
        "dual_carriageway": bool(e.flags & R.F_DUAL),
        "access_restricted": bool(e.flags & R.F_ACCESS_RESTRICTED), "toll": bool(e.flags & R.F_TOLL),
        "parking": bool(e.flags & R.F_PARKING), "forest": bool(e.flags & R.F_FOREST),
        "layer": e.layer, "built_up": built_up(e),
    }


def speed_limit(e, direction, limits):
    """The limit the map gives for this edge: the tagged maxspeed, else the implicit national limit from the
    road class and the built-up state. `source` says which, `detail` how the tagged value is qualified."""
    ms = e.ms_fwd if direction > 0 else e.ms_bwd
    variable = bool(e.flags2 & R.F2_MS_VARIABLE)
    if 0 < ms < R.MS_NONE:
        detail = "sign" if e.flags2 & R.F2_MS_SIGN else "zone" if e.flags2 & R.F2_MS_ZONE else "tagged"
        return {"limit_kmh": ms, "source": "maxspeed", "rule": None, "detail": detail,
                "confidence": 0.75 if variable else 0.9 if detail != "zone" else 0.8, "variable": variable}
    d = limits["implicit_kmh"]
    bu = built_up(e)
    cls = e.cls_name
    express = is_expressway(e, limits)
    if cls == "living_street":
        rule, conf = "residential_zone", 0.8
    elif e.flags & R.F_LINK and cls in ("motorway_link", "trunk_link"):
        return {"limit_kmh": None, "source": "implicit", "rule": "link", "detail": None, "confidence": 0.0,
                "variable": variable}
    elif express:
        rule, conf = ("expressway" if e.flags & R.F_DUAL else "expressway_single"), 0.8
    elif cls == "motorway":
        rule, conf = "motorway", 0.85
    elif bu["value"]:
        rule, conf = "built_up", 0.8 if bu["source"] == "tagged" else 0.5
    else:
        dual = e.flags & R.F_DUAL and lanes(e, direction)["forward"] >= 2
        rule, conf = ("outside_dual_2x2" if dual else "outside"), 0.75 if bu["source"] == "tagged" else 0.5
    if variable:
        conf *= 0.7
    return {"limit_kmh": d[rule], "source": "implicit", "rule": rule, "detail": "built_up:" + bu["source"],
            "confidence": round(conf, 2), "variable": variable}


# ---- most likely path ----------------------------------------------------------------------------
def _end_bearing(e, direction):
    """Bearing of travel when leaving the edge."""
    return e.brg1 if direction > 0 else (e.brg0 + 180.0) % 360.0


def _start_bearing(e, direction):
    return e.brg0 if direction > 0 else (e.brg1 + 180.0) % 360.0


def _outs(pack, e, direction):
    """Edges that can be driven after this one: [(edge, dir)], and the count of all other edges at the node."""
    node = e.b if direction > 0 else e.a
    outs, others = [], 0
    for ref, end in pack.node_edges(node):
        if ref == e.ref and e.a != e.b:
            continue
        if ref == e.ref and e.a == e.b and end == (1 if direction > 0 else 0):
            continue                                     # the same end of a loop edge
        e2 = pack.edge(ref)
        if e2 is None:
            continue
        others += 1
        d2 = 1 if end == 0 else -1
        if d2 < 0 and e2.oneway:
            continue
        outs.append((e2, d2))
    return outs, others


def _choose(e, direction, outs, entry):
    """Pick the continuation; returns (index, share of the choice, turn in degrees, positive right)."""
    b_in = _end_bearing(e, direction)
    scores = []
    for e2, d2 in outs:
        turn = geo.ang_diff(_start_bearing(e2, d2), b_in)
        sc = -abs(turn) / 30.0
        on_rb, to_rb = e.flags & R.F_ROUNDABOUT, e2.flags & R.F_ROUNDABOUT
        if on_rb and not to_rb:                          # an exit of the roundabout we are on
            if entry and ((entry["ref"] and entry["ref"] == e2.road_ref) or (entry["name"] and entry["name"] == e2.name)):
                sc = 3.0
            elif entry and abs(geo.ang_diff(_start_bearing(e2, d2), entry["bearing"])) < 35.0:
                sc = 1.5
            else:
                sc = -1.5
        elif on_rb and to_rb:
            sc = 0.6
        else:
            if e.road_ref and e.road_ref == e2.road_ref:
                sc += 2.0
            if e.name and e.name == e2.name:
                sc += 1.5
            if e.cls == e2.cls:
                sc += 0.7
            else:
                sc -= 0.4 * max(0, CLASS_RANK[e2.cls_name] - CLASS_RANK[e.cls_name])
            if e2.flags & R.F_LINK and not e.flags & R.F_LINK:
                sc -= 1.0
            if e2.cls_name in MINOR and e.cls_name not in MINOR:
                sc -= 1.5
            if e2.flags & R.F_ACCESS_RESTRICTED and not e.flags & R.F_ACCESS_RESTRICTED:
                sc -= 2.0
        scores.append((sc, turn))
    i = max(range(len(outs)), key=lambda k: scores[k][0])
    tot = sum(math.exp(s - scores[i][0]) for s, _ in scores)
    return i, 1.0 / tot, scores[i][1]


def most_likely_path(pack, e, direction, s, horizon_m, max_edges=120, tunnel_extra_m=15000.0):
    """[(edge, dir, start distance from the car, s_from, s_to, confidence)] along the most likely path, and the
    junctions met on the way. The path ends at horizon_m, except inside a tunnel: it is followed to the
    tunnel's end (at most tunnel_extra_m further) so that the exit distance is always known."""
    segs, junctions = [], []
    dist = 0.0
    conf = 1.0
    entry = None
    seen = set()
    s_from = s
    while len(segs) < max_edges:
        s_to = e.length if direction > 0 else 0.0
        segs.append((e, direction, dist, s_from, s_to, conf))
        dist += abs(s_to - s_from)
        if dist >= horizon_m and not (e.flags & R.F_TUNNEL and dist < horizon_m + tunnel_extra_m):
            break
        seen.add((e.ref, direction))
        outs, others = _outs(pack, e, direction)
        if not outs:
            junctions.append({"dist_m": round(dist, 1), "kind": "dead_end" if others == 0 else "no_way_on",
                              "branches": others, "turn_deg": 0.0, "minor": False, "confidence": round(conf, 2)})
            break
        if not e.flags & R.F_ROUNDABOUT:
            entry = {"ref": e.road_ref, "name": e.name, "bearing": _end_bearing(e, direction)}
        i, share, turn = _choose(e, direction, outs, entry)
        e2, d2 = outs[i]
        if others >= 2:
            side = [o for k, o in enumerate(outs) if k != i]
            entering_rb = e2.flags & R.F_ROUNDABOUT and not e.flags & R.F_ROUNDABOUT
            leaving_rb = e.flags & R.F_ROUNDABOUT and not e2.flags & R.F_ROUNDABOUT
            if not (e.flags & R.F_ROUNDABOUT and e2.flags & R.F_ROUNDABOUT):
                junctions.append({
                    "dist_m": round(dist, 1),
                    "kind": "roundabout" if entering_rb else "roundabout_exit" if leaving_rb else "junction",
                    "branches": others, "turn_deg": round(turn, 0),
                    "minor": bool(side) and all(o[0].cls_name in MINOR for o in side) and not entering_rb,
                    "confidence": round(conf * share, 2)})
            conf *= share
        if (e2.ref, d2) in seen:
            break
        e, direction = e2, d2
        s_from = 0.0 if direction > 0 else e.length
    return segs, junctions


def _polyline(segs, limit_m):
    """Path geometry as (lat, lon) arrays from the car's position up to limit_m."""
    lat, lon = [], []
    for e, d, dist, s_from, s_to, _ in segs:
        if dist > limit_m:
            break
        lo, hi = (s_from, s_to) if d > 0 else (s_to, s_from)
        i0 = int(np.searchsorted(e.s, lo, side="right"))
        i1 = int(np.searchsorted(e.s, hi, side="left"))
        a = e.at(lo)
        b = e.at(hi)
        la = [a[0]] + e.lat[i0:i1].tolist() + [b[0]]
        lo_ = [a[1]] + e.lon[i0:i1].tolist() + [b[1]]
        if d < 0:
            la.reverse()
            lo_.reverse()
        lat += la
        lon += lo_
    return np.array(lat), np.array(lon)


def look_ahead(pack, m, limits, horizon_m=1500.0, shape_m=600.0, shape_step=20.0):
    """The `ahead` object of the map message for a Match."""
    e, direction = m.edge, m.dir
    segs, junctions = most_likely_path(pack, e, direction, m.s, horizon_m)
    feats, seen = [], set()
    tunnel = {"in_tunnel": bool(e.flags & R.F_TUNNEL), "exit_m": None, "entry_m": None, "length_m": None}
    changes = {"built_up": None, "speed_limit": [], "road": None}
    cur_bu = built_up(e)["value"]
    cur_lim = speed_limit(e, direction, limits)
    cur_road = (e.road_ref, e.name)
    prev_tunnel = tunnel["in_tunnel"]
    tunnel_start = 0.0 if prev_tunnel else None
    last_lim = cur_lim["limit_kmh"]
    end_dist = 0.0
    for e2, d2, dist, s_from, s_to, conf in segs:
        end_dist = dist + abs(s_to - s_from)
        # attribute changes at the start of this segment
        t = bool(e2.flags & R.F_TUNNEL)
        if t != prev_tunnel:
            if t and tunnel["entry_m"] is None and not tunnel["in_tunnel"]:
                tunnel["entry_m"] = round(dist, 1)
                tunnel_start = dist
            elif not t and tunnel["exit_m"] is None and tunnel_start is not None:
                tunnel["exit_m"] = round(dist, 1)
                if not tunnel["in_tunnel"]:
                    tunnel["length_m"] = round(dist - tunnel_start, 1)
            prev_tunnel = t
        bu = built_up(e2)
        if changes["built_up"] is None and bu["value"] != cur_bu:
            changes["built_up"] = {"dist_m": round(dist, 1), "to": bu["value"], "source": bu["source"],
                                   "confidence": round(conf * (0.8 if bu["source"] == "tagged" else 0.5), 2)}
        lim = speed_limit(e2, d2, limits)
        if lim["limit_kmh"] != last_lim and len(changes["speed_limit"]) < 4:
            changes["speed_limit"].append({"dist_m": round(dist, 1), "limit_kmh": lim["limit_kmh"],
                                           "source": lim["source"], "rule": lim["rule"], "detail": lim["detail"],
                                           "confidence": round(conf * lim["confidence"], 2)})
        last_lim = lim["limit_kmh"]
        if changes["road"] is None and (e2.road_ref, e2.name) != cur_road and not e2.flags & R.F_ROUNDABOUT:
            changes["road"] = {"dist_m": round(dist, 1), "class": e2.cls_name, "name": e2.name or None,
                               "ref": e2.road_ref or None}
        for pt, kind, fdir in e2.feats:
            if fdir == R.DIR_FORWARD and d2 < 0 or fdir == R.DIR_BACKWARD and d2 > 0:
                continue
            sf = float(e2.s[pt])
            ahead = (sf - s_from) if d2 > 0 else (s_from - sf)
            if ahead < -0.5 or ahead > abs(s_to - s_from) + 0.5:
                continue
            key = (kind, round(float(e2.lat[pt]), 6), round(float(e2.lon[pt]), 6))
            if key in seen:
                continue
            seen.add(key)
            feats.append({"kind": R.FEATURES[kind], "dist_m": round(max(0.0, dist + ahead), 1),
                          "confidence": round(conf, 2)})
    if tunnel["in_tunnel"] and tunnel["exit_m"] is None and prev_tunnel:
        tunnel["exit_beyond_m"] = round(end_dist, 1)        # the path ended (dead end, loop) inside the tunnel
    feats.sort(key=lambda f: f["dist_m"])
    out = {"horizon_m": round(end_dist, 1), "path_confidence": round(segs[-1][5], 2),
           "features": feats[:40], "junctions": junctions[:20], "tunnel": tunnel,
           "built_up_change": changes["built_up"], "speed_limit_changes": changes["speed_limit"],
           "road_change": changes["road"]}
    out.update(_shape(segs, m, shape_m, shape_step))
    return out


def _shape(segs, m, shape_m, step):
    """Road shape ahead in the car's frame (x forward along the matched bearing, y right, metres; the same
    frame as modeld's outputs) and its curvature (1/m, positive = bending right)."""
    lat, lon = _polyline(segs, shape_m)
    loc = geo.Local(m.lat, m.lon)
    ex, ny = (lon - loc.lon0) * loc.mlon, (lat - loc.lat0) * loc.mlat
    b = math.radians(m.bearing)
    fx = ex * math.sin(b) + ny * math.cos(b)
    ry = ex * math.cos(b) - ny * math.sin(b)
    d = np.hypot(np.diff(fx), np.diff(ry))
    keep = np.concatenate([[True], d > 0.01])
    fx, ry = fx[keep], ry[keep]
    sl = np.concatenate([[0.0], np.cumsum(np.hypot(np.diff(fx), np.diff(ry)))])
    total = float(sl[-1]) if len(sl) else 0.0
    if total < step:
        return {"shape": None, "curves": []}
    ss = np.arange(0.0, min(total, shape_m) + 1e-6, step)
    x, y = np.interp(ss, sl, fx), np.interp(ss, sl, ry)
    hd = np.arctan2(np.diff(y), np.diff(x))                  # chord headings, positive = to the right
    dh = (np.diff(hd) + math.pi) % (2 * math.pi) - math.pi
    k = np.concatenate([[0.0], dh / step, [0.0]])
    curves = []
    thr = 1.0 / 500.0
    i = 0
    n = len(k)
    while i < n and len(curves) < 4:
        if abs(k[i]) < thr:
            i += 1
            continue
        j = i
        sign = 1 if k[i] > 0 else -1
        while j + 1 < n and k[j + 1] * sign >= thr:
            j += 1
        kk = np.abs(k[i:j + 1])
        apex = i + int(np.argmax(kk))
        curves.append({"start_m": round(float(ss[max(0, i - 1)]), 0), "apex_m": round(float(ss[apex]), 0),
                       "end_m": round(float(ss[min(n - 1, j + 1)]), 0),
                       "min_radius_m": round(1.0 / float(kk.max()), 0),
                       "direction": "right" if sign > 0 else "left",
                       "angle_deg": round(math.degrees(float(k[i:j + 1].sum() * step)), 0)})
        i = j + 1
    return {"shape": {"s": [round(v, 1) for v in ss.tolist()], "x": [round(v, 1) for v in x.tolist()],
                      "y": [round(v, 1) for v in y.tolist()], "curvature": [round(v, 5) for v in k.tolist()]},
            "curves": curves}
