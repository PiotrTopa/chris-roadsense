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


def look_ahead(pack, m, limits, horizon_m=1500.0, shape_m=600.0, shape_step=20.0, memo=None):
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
    try:
        out["junction_view"] = junction_view(pack, segs, feats, m, memo)
    except Exception:                       # a view is an extra: odd geometry must not cost the look-ahead
        out["junction_view"] = None
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


# ---- junction view -------------------------------------------------------------------------------
VIEW_M = 400.0              # a junction view is made for the first major junction within this distance on the path
VIEW_MERGE_M = 35.0         # junction nodes this close along the path are one junction (dual carriageways)
VIEW_ARM_M = 25.0           # an arm's direction is taken this far from the junction node
VIEW_RING_MAX = 80          # edges followed around a roundabout


def _arm_bearing(e, d):
    """Bearing of travel away from the node along edge e in direction d, measured VIEW_ARM_M out (the first
    metres of an edge at a junction are often a short bent piece)."""
    s0 = 0.0 if d > 0 else e.length
    s1 = min(VIEW_ARM_M, e.length) if d > 0 else max(e.length - VIEW_ARM_M, 0.0)
    a, b = e.at(s0), e.at(s1)
    if abs(s1 - s0) < 1.0:
        return _start_bearing(e, d)
    return geo.bearing_deg(a[0], a[1], b[0], b[1])


def _arm(e, d, b_ref, on_path):
    """One arm of a junction: edge e leaving the node in direction d."""
    ln = lanes(e, d)
    return {"angle_deg": round(geo.ang_diff(_arm_bearing(e, d), b_ref), 0),
            "class": e.cls_name, "name": e.name or None, "ref": e.road_ref or None,
            "exit": not (d < 0 and e.oneway),           # can be driven away from the junction
            "entry": not (d > 0 and e.oneway),          # traffic comes in from it
            "path": bool(on_path), "minor": e.cls_name in MINOR or bool(e.flags & R.F_ACCESS_RESTRICTED),
            "lanes": ln["forward"] if ln["source"] != "assumed" else None}


def _node_arms(pack, node, skip):
    """[(edge, dir away from the node)] of every edge at the node except the refs in `skip`."""
    out = []
    for ref, end in pack.node_edges(node):
        if ref in skip:
            continue
        e = pack.edge(ref)
        if e is not None:
            out.append((e, 1 if end == 0 else -1))
    return out


def _control(feats, dist):
    """What controls the junction at path distance `dist`, from the point features on the path before it."""
    best = None
    for f in feats:
        if dist - 60.0 <= f["dist_m"] <= dist + 35.0:
            k = f["kind"]
            if k == "traffic_signals":
                return "signals"
            if k in ("stop", "give_way") and best is None:
                best = k
    return best


def _approach(e, d):
    ln = lanes(e, d)
    return {"class": e.cls_name, "name": e.name or None, "ref": e.road_ref or None,
            "lanes": ln["forward"], "lanes_source": ln["source"], "oneway": e.oneway}


def _plain_view(pack, segs, i, feats):
    """Junction view of the junction at the end of segs[i] (and of the junction nodes right behind it)."""
    e, d, dist, s_from, s_to, conf = segs[i]
    jd = dist + abs(s_to - s_from)
    b_ref = (_arm_bearing(e, -d) + 180.0) % 360.0          # direction of travel into the junction
    arms, skip = [], {e.ref}
    k = i
    while True:
        e1, d1 = segs[k][0], segs[k][1]
        node = e1.b if d1 > 0 else e1.a
        nxt = segs[k + 1] if k + 1 < len(segs) else None
        if nxt is not None:
            skip.add(nxt[0].ref)
        for e3, d3 in _node_arms(pack, node, skip):
            if e3.flags & R.F_ROUNDABOUT:
                continue
            arms.append(_arm(e3, d3, b_ref, False))
        if nxt is None:
            break
        end2 = nxt[2] + abs(nxt[4] - nxt[3])
        more = k + 2 < len(segs) and end2 - jd <= VIEW_MERGE_M and not nxt[0].flags & R.F_ROUNDABOUT \
            and _outs(pack, nxt[0], nxt[1])[1] >= 2
        if not more:
            arms.append(_arm(nxt[0], nxt[1], b_ref, True))
            break
        skip.add(nxt[0].ref)
        k += 1
    # the opposite carriageway of the approach road is not an arm of its own
    arms = [a for a in arms if not (abs(a["angle_deg"]) > 150.0 and a["name"] == (e.name or None)
                                    and a["ref"] == (e.road_ref or None) and not a["path"])]
    arms.sort(key=lambda a: a["angle_deg"])
    return {"kind": "junction", "dist_m": round(jd, 1), "confidence": round(segs[min(i + 1, len(segs) - 1)][5], 2),
            "control": _control(feats, jd), "approach": _approach(e, d), "arms": arms[:8], "ring": None}


def _ring(pack, e, d):
    """Walk a roundabout from edge e (direction d): (edge refs, lat list, lon list, [(node, s along the ring)])."""
    refs, lat, lon, nodes = [], [], [], []
    s = 0.0
    start = e.ref
    for _ in range(VIEW_RING_MAX):
        refs.append(e.ref)
        la, lo = (e.lat, e.lon) if d > 0 else (e.lat[::-1], e.lon[::-1])
        lat += la.tolist()
        lon += lo.tolist()
        s += e.length
        node = e.b if d > 0 else e.a
        nodes.append((node, s, float(la[-1]), float(lo[-1])))
        nxt = [(e2, d2) for e2, d2 in _outs(pack, e, d)[0] if e2.flags & R.F_ROUNDABOUT]
        if not nxt or nxt[0][0].ref == start:
            break
        e, d = nxt[0]
    return refs, lat, lon, nodes


def _roundabout_view(pack, segs, i, feats, memo, m):
    """Junction view of the roundabout entered after segs[i] (i = -1: the car is on the ring)."""
    on_ring = i < 0
    first = segs[0] if on_ring else segs[i + 1]
    old = (memo or {}).get("ring")
    if on_ring and old and first[0].ref in old["refs"]:
        # on the ring of the roundabout that was announced: the same picture (arms, exit numbers, the exit of
        # the path chosen before the entry), with the car's place on the ring
        view = dict(old["view"], dist_m=0.0, control=None)
        pos = geo.ang_diff(geo.bearing_deg(old["centre"][0], old["centre"][1], m.lat, m.lon), old["b_ref"])
        view["ring"] = dict(view["ring"], on_ring=True, car_angle_deg=round(pos, 0), exit_dist_m=None)
        return view
    refs, lat, lon, nodes = _ring(pack, first[0], first[1])
    ring_id = min(refs)
    loc = geo.Local(lat[0], lon[0])
    x, y = loc.to_xy(np.array(lat), np.array(lon))
    cx, cy = float(x.mean()), float(y.mean())
    radius = float(np.hypot(x - cx, y - cy).mean())
    clat, clon = loc.to_ll(cx, cy)
    if on_ring:
        jd, conf, appr = 0.0, segs[0][5], None
        b_ref = geo.bearing_deg(m.lat, m.lon, clat, clon)     # no announcement to go by: the car is "below"
    else:
        e, d, dist, s_from, s_to, conf = segs[i]
        jd = dist + abs(s_to - s_from)
        n = e.at(e.length if d > 0 else 0.0)
        b_ref = geo.bearing_deg(n[0], n[1], clat, clon)       # from the entry point to the centre
        appr = _approach(e, d)
    # the edge on which the most likely path leaves the ring
    exit_ref, exit_dist = None, None
    for sg in segs[(0 if on_ring else i + 1):]:
        if not sg[0].flags & R.F_ROUNDABOUT:
            exit_ref, exit_dist = sg[0].ref, sg[2]
            break
    ring_set = set(refs)
    raw = []
    for node, s, nla, nlo in nodes:
        ang = geo.ang_diff(geo.bearing_deg(clat, clon, nla, nlo), b_ref)
        for e3, d3 in _node_arms(pack, node, ring_set):
            a = _arm(e3, d3, b_ref, e3.ref == exit_ref)
            a["angle_deg"] = round(ang, 0)                    # where the arm meets the ring, seen from the centre
            a["_s"] = s
            raw.append(a)
    # the entry and the exit carriageway of one road are one arm
    arms = []
    for a in raw:
        for b in arms:
            if b["name"] == a["name"] and b["ref"] == a["ref"] \
                    and abs(geo.ang_diff(a["angle_deg"], b["angle_deg"])) < 45.0 \
                    and (b["exit"] != a["exit"] or b["entry"] != a["entry"]):
                mid = b["angle_deg"] + 0.5 * geo.ang_diff(a["angle_deg"], b["angle_deg"])
                b.update(angle_deg=round(geo.ang_diff(mid, 0.0), 0), exit=b["exit"] or a["exit"],
                         entry=b["entry"] or a["entry"], path=b["path"] or a["path"],
                         minor=b["minor"] and a["minor"])
                b["_s"] = min(b["_s"], a["_s"]) if a["exit"] else b["_s"]
                break
        else:
            arms.append(a)
    # exits are numbered in driving order from the entry; the arm the car came from is the one behind (180)
    arms.sort(key=lambda a: a["_s"])
    no = 0
    path_no = None
    for a in arms:
        a["approach"] = bool(not on_ring or appr) and abs(geo.ang_diff(a["angle_deg"], 180.0)) < 30.0
        if a["exit"] and not a["minor"] and not a["approach"]:    # back where the car came from is not counted
            no += 1
            a["exit_no"] = no
            if a["path"]:
                path_no = no
        a.pop("_s")
    pos = None
    if on_ring:
        pos = round(geo.ang_diff(geo.bearing_deg(clat, clon, m.lat, m.lon), b_ref), 0)
    view = {"kind": "roundabout", "dist_m": round(jd, 1), "confidence": round(conf, 2),
            "control": None if on_ring else (_control(feats, jd) or "give_way"), "approach": appr, "arms": arms[:10],
            "ring": {"radius_m": round(radius, 1), "lanes": first[0].lanes or None, "exits": no,
                     "path_exit_no": path_no,
                     "exit_dist_m": None if exit_dist is None else round(exit_dist, 1),
                     "on_ring": on_ring, "car_angle_deg": pos}}
    if memo is not None and not on_ring:
        memo["ring"] = {"refs": ring_set, "b_ref": b_ref, "centre": (clat, clon), "view": view}
    return view


def junction_view(pack, segs, feats, m, memo=None, max_m=VIEW_M):
    """The first junction on the most likely path that is worth a picture: its arms (direction relative to
    the approach, class, name, whether it can be driven into, which one the most likely path takes), what
    controls it, and for a roundabout the ring and its exits in driving order. None when there is none
    within max_m. `memo` (a dict the caller keeps) holds the orientation of the last roundabout so that the
    picture does not turn while the car drives around the ring."""
    if segs[0][0].flags & R.F_ROUNDABOUT:
        return _roundabout_view(pack, segs, -1, feats, memo, m)
    for i in range(len(segs) - 1):
        e, d, dist, s_from, s_to, conf = segs[i]
        if dist + abs(s_to - s_from) > max_m:
            break
        e2 = segs[i + 1][0]
        if e2.flags & R.F_ROUNDABOUT:
            return _roundabout_view(pack, segs, i, feats, memo, m)
        outs, others = _outs(pack, e, d)
        if others < 2:
            continue
        every = _node_arms(pack, e.b if d > 0 else e.a, {e.ref, e2.ref})
        if every and all(o[0].cls_name in MINOR for o in every):
            continue                                          # only service roads branch off
        if not every:
            continue
        view = _plain_view(pack, segs, i, feats)
        if len(view["arms"]) >= 2:                            # the way on and at least one other arm
            return view
    return None
