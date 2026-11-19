#!/usr/bin/env python3
"""roadsense-maps: fetch an OpenStreetMap extract, build a roadsense map pack (.rsmap), install it.

    roadsense-maps fetch   [--region europe/poland]            # Geofabrik extract -> <dir>/src/geofabrik/
    roadsense-maps build   [--region europe/poland] [--pbf F]  # -> <dir>/packs/<region>-<osm date>.rsmap
    roadsense-maps info    PACK                                # manifest, size
    roadsense-maps verify  PACK                                # decode every tile, check the graph
    roadsense-maps install PACK [--host user@phone]            # copy to the device, print the activation step

<dir> is ~/roadsense-maps (env ROADSENSE_MAPS_DIR, option --dir): outside every repository. Map data and
packs are never committed. `build` needs pyosmium, numpy and Pillow (dev box only; the daemon needs numpy).
OpenStreetMap data is (c) OpenStreetMap contributors, ODbL 1.0; a pack is a derived database under the
same licence (docs/MAPS.md).
"""
import argparse
import array
import hashlib
import json
import math
import os
import subprocess
import sys
import time
import urllib.request

sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "chris-mapper"))

import numpy as np                                   # noqa: E402

from chris_mapper import rsmap                       # noqa: E402

GEOFABRIK = "https://download.geofabrik.de"
USER_AGENT = "roadsense-maps/1"
ATTRIBUTION = "Map data (c) OpenStreetMap contributors, ODbL 1.0, https://www.openstreetmap.org/copyright"
DRIVABLE = ("motorway", "motorway_link", "trunk", "trunk_link", "primary", "primary_link", "secondary",
            "secondary_link", "tertiary", "tertiary_link", "unclassified", "residential", "living_street",
            "service")
MAJOR = ("motorway", "trunk", "primary", "secondary", "tertiary")
BUILTUP_LANDUSE = ("residential", "commercial", "retail", "industrial")
RESTRICTED = ("no", "private", "agricultural", "forestry", "delivery", "customers", "permit", "military")
MASK_PER_DEG = 2048                                  # landuse raster: ~54 m x ~33 m cells in Poland
SAMPLE_M = 40.0                                      # landuse sampling step along a road
MIN_RUN_SAMPLES = 5                                  # shorter built-up / forest runs are merged (200 m)


def log(*a):
    print(time.strftime("%H:%M:%S"), *a, file=sys.stderr, flush=True)


def maps_dir(a):
    return os.path.expanduser(a.dir or os.environ.get("ROADSENSE_MAPS_DIR") or "~/roadsense-maps")


def region_name(region):
    return region.rstrip("/").split("/")[-1]


# ---- fetch ---------------------------------------------------------------------------------------
def _get(url, dest=None):
    req = urllib.request.Request(url, headers={"User-Agent": USER_AGENT})
    with urllib.request.urlopen(req, timeout=60) as r:
        if dest is None:
            return r.read(), r.geturl()
        total, done, t0 = int(r.headers.get("Content-Length") or 0), 0, time.time()
        md5 = hashlib.md5()
        with open(dest + ".part", "wb") as f:
            while True:
                b = r.read(1 << 20)
                if not b:
                    break
                f.write(b)
                md5.update(b)
                done += len(b)
                if time.time() - t0 > 15:
                    t0 = time.time()
                    log("  %d / %d MB" % (done >> 20, total >> 20))
        os.replace(dest + ".part", dest)
        return md5.hexdigest(), r.geturl()


def cmd_fetch(a):
    d = os.path.join(maps_dir(a), "src", "geofabrik")
    os.makedirs(d, exist_ok=True)
    url = "%s/%s-latest.osm.pbf" % (GEOFABRIK, a.region)
    want, _ = _get(url + ".md5")
    want = want.decode().split()[0]
    dest = os.path.join(d, region_name(a.region) + "-latest.osm.pbf")
    if os.path.exists(dest) and not a.force:
        h = hashlib.md5()
        with open(dest, "rb") as f:
            for b in iter(lambda: f.read(1 << 20), b""):
                h.update(b)
        if h.hexdigest() == want:
            log("up to date:", dest)
            print(dest)
            return
    log("downloading", url)
    got, final = _get(url, dest)
    if got != want:
        os.unlink(dest)
        sys.exit("md5 mismatch for %s (got %s, expected %s)" % (final, got, want))
    log("fetched %s (%d MB) from %s" % (dest, os.path.getsize(dest) >> 20, final))
    print(dest)


# ---- OSM tag parsing -----------------------------------------------------------------------------
def parse_int(v, lo=0, hi=255):
    if not v:
        return 0
    try:
        return min(hi, max(lo, int(float(v.split(";")[0].strip()))))
    except ValueError:
        return 0


ZONE_SPEED = {"urban": 50, "rural": 90, "living_street": 20, "motorway": 140, "zone30": 30, "zone20": 20,
              "zone40": 40}


def parse_maxspeed(v):
    """-> (km/h or 0 or MS_NONE, is_zone, built_up or None, variable)."""
    if not v:
        return 0, False, None, False
    v = v.strip()
    if v == "none":
        return rsmap.MS_NONE, False, None, False
    if v in ("signals", "variable"):
        return 0, False, None, True
    if v == "walk":
        return 5, False, None, False
    if ":" in v:
        z = v.split(":", 1)[1].lower()
        return ZONE_SPEED.get(z, 0), True, {"urban": True, "living_street": True, "rural": False,
                                            "motorway": False, "trunk": False}.get(z), False
    try:
        if v.endswith("mph"):
            return min(254, int(round(float(v[:-3]) * 1.609))), False, None, False
        return min(254, max(0, int(float(v.replace("km/h", "").replace("kmh", "").split(";")[0])))), False, None, False
    except ValueError:
        return 0, False, None, False


SURFACE_MAP = {"asphalt": "asphalt", "concrete": "concrete", "concrete:plates": "concrete",
               "concrete:lanes": "concrete", "paved": "paved", "paving_stones": "paving_stones",
               "sett": "sett", "cobblestone": "sett", "unhewn_cobblestone": "sett", "compacted": "compacted",
               "fine_gravel": "gravel", "gravel": "gravel", "pebblestone": "gravel", "unpaved": "unpaved",
               "ground": "ground", "dirt": "ground", "earth": "ground", "sand": "ground", "grass": "ground",
               "mud": "ground"}


def way_attrs(t, with_tracks=False):
    """Tags of a highway way -> attribute tuple, or None when the way is not part of the pack."""
    hw = t.get("highway")
    if hw not in DRIVABLE and not (with_tracks and hw == "track"):
        return None
    if t.get("area") == "yes":
        return None
    F = rsmap
    flags = flags2 = 0
    ow = t.get("oneway")
    rb = t.get("junction") in ("roundabout", "circular")
    reverse = ow in ("-1", "reverse")
    oneway = ow in ("yes", "true", "1") or reverse or ((hw == "motorway" or rb) and ow != "no")
    if oneway:
        flags |= F.F_ONEWAY
    if rb:
        flags |= F.F_ROUNDABOUT
    tun = t.get("tunnel")
    if tun and tun not in ("no", "building_passage"):
        flags |= F.F_TUNNEL
    if tun == "building_passage" or t.get("covered") == "yes":
        flags |= F.F_COVERED
    if t.get("bridge") not in (None, "no"):
        flags |= F.F_BRIDGE
    lit = t.get("lit")
    if lit in ("yes", "24/7", "automatic", "limited"):
        flags |= F.F_LIT_YES
    elif lit == "no":
        flags |= F.F_LIT_NO
    if t.get("motorroad") == "yes":
        flags |= F.F_MOTORROAD
    if hw.endswith("_link"):
        flags |= F.F_LINK
    acc = t.get("motor_vehicle") or t.get("vehicle") or t.get("access")
    if acc in RESTRICTED:
        flags |= F.F_ACCESS_RESTRICTED
    if t.get("dual_carriageway") == "yes" or (oneway and hw in MAJOR and not rb):
        flags |= F.F_DUAL
    if t.get("toll") == "yes":
        flags |= F.F_TOLL
    if t.get("service") == "parking_aisle":
        flags |= F.F_PARKING
    ms, zone, bu, var = parse_maxspeed(t.get("maxspeed"))
    msf, zf, buf, vf = parse_maxspeed(t.get("maxspeed:forward"))
    msb, zb, bub, vb = parse_maxspeed(t.get("maxspeed:backward"))
    ms_fwd, ms_bwd = msf or ms, msb or ms
    if zone or zf or zb:
        flags2 |= F.F2_MS_ZONE
    if var or vf or vb or t.get("maxspeed:variable") not in (None, "no"):
        flags2 |= F.F2_MS_VARIABLE
    builtup = bu if bu is not None else buf if buf is not None else bub
    for k in ("zone:traffic", "zone:maxspeed", "maxspeed:type", "source:maxspeed"):
        v = t.get(k)
        if not v:
            continue
        if v == "sign":
            flags2 |= F.F2_MS_SIGN
        elif v.endswith(":urban"):
            builtup = True
        elif v.endswith(":rural") or v.endswith(":motorway") or v.endswith(":trunk"):
            builtup = False if builtup is None else builtup
    lanes, lf, lb = parse_int(t.get("lanes")), parse_int(t.get("lanes:forward")), parse_int(t.get("lanes:backward"))
    if lf or lb:
        flags2 |= F.F2_LANES_SPLIT
    if oneway and lanes and not lf:
        lf = lanes
    # turn lanes (format 2): `turn:lanes` is the driving direction of a one-way way, else the per-direction keys
    tf = F.normalise_turn_lanes(t.get("turn:lanes:forward") or (t.get("turn:lanes") if oneway else None))
    tb = "" if oneway else F.normalise_turn_lanes(t.get("turn:lanes:backward"))
    if tf and not lf:                                  # painted arrows count the lanes as well as a lanes tag
        lf = tf.count("|") + 1
        if not oneway:
            flags2 |= F.F2_LANES_SPLIT
    if tb and not lb:
        lb = tb.count("|") + 1
        flags2 |= F.F2_LANES_SPLIT
    if oneway and tf and not lanes:
        lanes = lf
    if reverse:
        ms_fwd, ms_bwd, lf, lb, tf, tb = ms_bwd, ms_fwd, lb, lf, tb, tf
    surface = F.SURFACE_ID[SURFACE_MAP.get(t.get("surface") or "", "other" if t.get("surface") else "unknown")]
    return (F.CLASS_ID[hw], flags, flags2, ms_fwd, ms_bwd, lanes, lf, lb, parse_int(t.get("layer"), -5, 5),
            surface, (t.get("name") or "")[:120], (t.get("ref") or "")[:60], builtup, reverse, tf, tb)


def node_feature(t):
    """Tags of a node -> (feature id, dir) or None."""
    F = rsmap.FEATURE_ID
    hw, kind = t.get("highway"), None
    if hw == "traffic_signals":
        kind = F["traffic_signals"]
    elif hw == "crossing":
        sig = t.get("crossing") == "traffic_signals" or t.get("crossing:signals") == "yes"
        kind = F["crossing_signals"] if sig else F["crossing"]
    elif hw == "stop":
        kind = F["stop"]
    elif hw == "give_way":
        kind = F["give_way"]
    elif hw == "mini_roundabout":
        kind = F["mini_roundabout"]
    elif hw == "motorway_junction":
        kind = F["motorway_junction"]
    elif t.get("railway") == "level_crossing":
        kind = F["level_crossing"]
    elif t.get("railway") == "tram_level_crossing":
        kind = F["tram_crossing"]
    elif t.get("traffic_calming") in ("bump", "hump", "table", "cushion", "rumble_strip", "mini_bumps", "yes"):
        kind = F["speed_bump"]
    elif t.get("barrier") == "toll_booth":
        kind = F["toll_booth"]
    elif hw == "speed_camera":
        kind = F["speed_camera"]
    if kind is None:
        return None
    d = t.get("direction") or t.get("traffic_signals:direction")
    return kind, rsmap.DIR_FORWARD if d == "forward" else rsmap.DIR_BACKWARD if d == "backward" else rsmap.DIR_BOTH


# ---- build ---------------------------------------------------------------------------------------
class Masks:
    """Rasters of built-up landuse (bit 0) and forest (bit 1) over the extract's bounding box."""

    def __init__(self, lon0, lat0, lon1, lat1):
        from PIL import Image, ImageDraw
        self.lon0, self.lat0 = math.floor(lon0 * 32) / 32 - 0.05, math.floor(lat0 * 32) / 32 - 0.05
        self.w = int((lon1 + 0.05 - self.lon0) * MASK_PER_DEG) + 2
        self.h = int((lat1 + 0.05 - self.lat0) * MASK_PER_DEG) + 2
        Image.MAX_IMAGE_PIXELS = None
        self.img = [Image.new("L", (self.w, self.h), 0) for _ in range(2)]
        self.draw = [ImageDraw.Draw(i) for i in self.img]
        self.n = [0, 0]
        self.state = None

    def add(self, which, outer, inners):
        def px(ring):
            return [((lon - self.lon0) * MASK_PER_DEG, (lat - self.lat0) * MASK_PER_DEG) for lon, lat in ring]
        if len(outer) < 3:
            return
        self.draw[which].polygon(px(outer), fill=1)
        for r in inners:
            if len(r) >= 3:
                self.draw[which].polygon(px(r), fill=0)
        self.n[which] += 1

    def finish(self):
        """Dilate (a road usually runs in the gap between two polygons; built-up landuse by two cells, which
        also closes small gaps inside a town, forest by one) and combine into one state raster."""
        out = np.zeros((self.h, self.w), dtype=np.uint8)
        for bit, im in enumerate(self.img):
            e = np.asarray(im, dtype=np.uint8)
            for _ in range(2 if bit == 0 else 1):
                d = e.copy()
                d[1:] |= e[:-1]
                d[:-1] |= e[1:]
                e = d.copy()
                e[:, 1:] |= d[:, :-1]
                e[:, :-1] |= d[:, 1:]
            out |= (e << bit)
        self.state = out
        self.img = self.draw = None

    def sample(self, lon, lat):
        ix = np.clip(((lon - self.lon0) * MASK_PER_DEG).astype(np.int64), 0, self.w - 1)
        iy = np.clip(((lat - self.lat0) * MASK_PER_DEG).astype(np.int64), 0, self.h - 1)
        return self.state[iy, ix]


def read_roads(pbf, with_tracks):
    import osmium
    feats = {}
    ways = []                                 # attribute tuples
    ids, lon, lat = array.array("q"), array.array("i"), array.array("i")
    starts = array.array("q", [0])
    fp = osmium.FileProcessor(pbf).with_locations() \
        .with_filter(osmium.filter.KeyFilter("highway", "railway", "traffic_calming", "barrier"))
    bad = 0
    for o in fp:
        if o.is_node():
            f = node_feature({t.k: t.v for t in o.tags})
            if f:
                feats[o.id] = f
        elif o.is_way():
            tags = {t.k: t.v for t in o.tags}
            at = way_attrs(tags, with_tracks)
            if at is None:
                continue
            n0 = len(ids)
            try:
                for n in o.nodes:
                    loc = n.location
                    lon.append(int(round(loc.lon * 1e6)))
                    lat.append(int(round(loc.lat * 1e6)))
                    ids.append(n.ref)
            except Exception:                 # a node outside the extract
                bad += 1
                del ids[n0:]
                del lon[n0:]
                del lat[n0:]
                continue
            if len(ids) - n0 < 2:
                del ids[n0:]
                del lon[n0:]
                del lat[n0:]
                continue
            ways.append(at)
            starts.append(len(ids))
            if len(ways) % 500000 == 0:
                log("  %d ways" % len(ways))
    log("roads: %d ways, %d way nodes, %d feature nodes, %d ways skipped (incomplete)"
        % (len(ways), len(ids), len(feats), bad))
    return (ways, np.frombuffer(ids, dtype=np.int64), np.frombuffer(lon, dtype=np.int32),
            np.frombuffer(lat, dtype=np.int32), np.frombuffer(starts, dtype=np.int64), feats)


def read_landuse(pbf, masks):
    import osmium
    flt = osmium.filter.KeyFilter("landuse", "natural")
    fp = osmium.FileProcessor(pbf).with_areas(flt).with_filter(flt)
    for o in fp:
        if not o.is_area():
            continue
        lu, nat = o.tags.get("landuse"), o.tags.get("natural")
        which = 0 if lu in BUILTUP_LANDUSE else 1 if (lu == "forest" or nat == "wood") else None
        if which is None:
            continue
        try:
            for outer in o.outer_rings():
                masks.add(which, [(n.lon, n.lat) for n in outer],
                          [[(n.lon, n.lat) for n in inner] for inner in o.inner_rings(outer)])
        except Exception:
            continue
    log("landuse: %d built-up polygons, %d forest polygons" % tuple(masks.n))


def _runs(states):
    """Run-length encode and merge runs shorter than MIN_RUN_SAMPLES into the previous run."""
    cut = np.flatnonzero(np.diff(states)) + 1
    bounds = [0] + cut.tolist() + [len(states)]
    runs = [[bounds[i], bounds[i + 1], int(states[bounds[i]])] for i in range(len(bounds) - 1)]
    out = []
    for r in runs:
        if out and (r[1] - r[0] < MIN_RUN_SAMPLES or out[-1][2] == r[2]):
            out[-1][1] = r[1]
        else:
            out.append(r)
    if len(out) > 1 and out[0][1] - out[0][0] < MIN_RUN_SAMPLES:
        out[1][0] = 0
        out.pop(0)
    return out


def split_polyline(pts, sl, cuts, fl):
    """Split a polyline (int32 (n, 2) points, cumulative length sl) at the distances `cuts` (ascending,
    inside the line) by inserting a point at each cut. -> [(points, features)], features re-indexed."""
    out = []
    start_i, start_pt = 0, None
    for cs in list(cuts) + [None]:
        if cs is None:
            end_i, end_pt = len(pts) - 1, None
        else:
            end_i = min(len(pts) - 2, int(np.searchsorted(sl, cs, side="right")) - 1)   # last vertex before the cut
            t = (cs - sl[end_i]) / max(1e-9, sl[end_i + 1] - sl[end_i])
            end_pt = np.round(pts[end_i] + t * (pts[end_i + 1] - pts[end_i]).astype(np.float64)).astype(np.int32)
        parts = ([start_pt[None, :]] if start_pt is not None else []) + [pts[start_i:end_i + 1]] + \
            ([end_pt[None, :]] if end_pt is not None else [])
        p = np.concatenate(parts)
        shift = (1 if start_pt is not None else 0) - start_i
        pf = [(j + shift, k, d) for j, k, d in fl if start_i <= j <= end_i]
        keep = np.concatenate([[True], np.any(p[1:] != p[:-1], axis=1)])
        if not keep.all():                                # a cut that fell on a vertex
            idx = np.cumsum(keep) - 1
            pf = [(int(idx[j]), k, d) for j, k, d in pf]
            p = p[keep]
        if len(p) < 2:                                    # cannot happen with cuts >= 100 m apart; keep the graph whole
            p = np.concatenate([p, p + 1])
        out.append((p, pf))
        start_i, start_pt = end_i + 1, end_pt
    return out


def make_edges(ways, ids, lon, lat, starts, feats, masks):
    """Split the ways at shared nodes and at built-up / forest changes; yield RawEdge objects."""
    F = rsmap
    log("graph: finding junction nodes")
    uniq, inv, counts = np.unique(ids, return_inverse=True, return_counts=True)
    is_break = counts[inv] >= 2
    del uniq, counts, inv
    feat_ids = np.fromiter(feats.keys(), dtype=np.int64, count=len(feats))
    has_feat = np.isin(ids, feat_ids)
    synth = -1
    n_out = 0
    for w, at in enumerate(ways):
        (cls, flags, flags2, ms_fwd, ms_bwd, lanes, lf, lb, layer, surface, name, ref, builtup, reverse, tf, tb) = at
        s, e = int(starts[w]), int(starts[w + 1])
        order = np.arange(e - 1, s - 1, -1) if reverse else np.arange(s, e)
        brk = is_break[order]
        brk[0] = brk[-1] = True
        cuts = np.flatnonzero(brk)
        infer = builtup is None and cls not in (F.CLASS_ID["motorway"], F.CLASS_ID["motorway_link"]) \
            and not flags & F.F_MOTORROAD
        base = flags
        if builtup or cls == F.CLASS_ID["living_street"]:
            base |= F.F_BUILTUP | (F.F_BUILTUP_TAGGED if builtup else 0)
        elif builtup is False:
            base |= F.F_BUILTUP_TAGGED                 # tagged, and the value is "not built-up"
        for c in range(len(cuts) - 1):
            pos = order[cuts[c]:cuts[c + 1] + 1]
            pts = np.stack([lon[pos], lat[pos]], axis=1)
            if len(pts) > 1 and np.any(np.all(pts[1:] == pts[:-1], axis=1)):     # duplicate points
                keep = np.concatenate([[True], np.any(pts[1:] != pts[:-1], axis=1)])
                keep[-1] = True
                pts, pos = pts[keep], pos[keep]
            if len(pts) < 2:
                continue
            fl = []
            if has_feat[pos].any():
                for j in np.flatnonzero(has_feat[pos]).tolist():
                    kind, d = feats[int(ids[pos[j]])]
                    if reverse and d:
                        d = 3 - d
                    fl.append((j, kind, d))
            a, b = int(ids[pos[0]]), int(ids[pos[-1]])
            # landuse along the piece
            x = pts[:, 0] * 1e-6
            y = pts[:, 1] * 1e-6
            mlon = 111194.9 * math.cos(math.radians(float(y[0])))
            sl = np.concatenate([[0.0], np.cumsum(np.hypot(np.diff(x) * mlon, np.diff(y) * 111194.9))])
            total = float(sl[-1])
            ns = max(2, int(total / SAMPLE_M) + 1)
            ss = np.linspace(0.0, total, ns)
            st = masks.sample(np.interp(ss, sl, x), np.interp(ss, sl, y)) if masks is not None else np.zeros(ns, np.uint8)
            if not infer:
                st = st & 2
            if base & F.F_BUILTUP:
                st = st & 1                              # no forest label inside a built-up area
            runs = _runs(st) if st.min() != st.max() else [[0, ns, int(st[0])]]
            if len(runs) == 1:
                pieces = [(a, b, pts, fl, runs[0][2])]
            else:
                parts = split_polyline(pts, sl, [0.5 * (ss[r[0] - 1] + ss[r[0]]) for r in runs[1:]], fl)
                nodes = [a]
                for _ in range(len(parts) - 1):
                    nodes.append(synth)
                    synth -= 1
                nodes.append(b)
                pieces = [(nodes[k], nodes[k + 1], pp, pf, runs[k][2]) for k, (pp, pf) in enumerate(parts)]
            for (pa, pb, pp, pf, state) in pieces:
                fg = base
                if infer and state & 1:
                    fg |= F.F_BUILTUP
                if state & 2 and not fg & F.F_BUILTUP:
                    fg |= F.F_FOREST
                for q0 in range(0, len(pp) - 1, 60000):          # one edge holds at most 65535 points
                    q1 = min(len(pp), q0 + 60001)
                    na = pa if q0 == 0 else ("s", w, c, q0)
                    nb = pb if q1 == len(pp) else ("s", w, c, q1 - 1)
                    n_out += 1
                    yield F.RawEdge(na, nb, pp[q0:q1], cls, fg, flags2, ms_fwd, ms_bwd, lanes, lf, lb, layer,
                                    surface, name, ref, [(j - q0, k, d) for j, k, d in pf if q0 <= j < q1], tf, tb)
        if w % 500000 == 0 and w:
            log("  %d / %d ways -> %d edges" % (w, len(ways), n_out))


def cmd_build(a):
    t0 = time.time()
    d = maps_dir(a)
    name = region_name(a.region)
    pbf = a.pbf or os.path.join(d, "src", "geofabrik", name + "-latest.osm.pbf")
    if not os.path.exists(pbf):
        sys.exit("no extract at %s (run `roadsense-maps fetch` or pass --pbf)" % pbf)
    import osmium
    hdr = osmium.io.Reader(pbf).header()
    osm_ts = hdr.get("osmosis_replication_timestamp") or hdr.get("timestamp") or ""
    log("extract %s, OSM data of %s" % (pbf, osm_ts or "unknown date"))
    ways, ids, lon, lat, starts, feats = read_roads(pbf, a.tracks)
    t1 = time.time()
    masks = None
    if not a.no_landuse:
        masks = Masks(lon.min() * 1e-6, lat.min() * 1e-6, lon.max() * 1e-6, lat.max() * 1e-6)
        read_landuse(pbf, masks)
        masks.finish()
    t2 = time.time()
    edges = list(make_edges(ways, ids, lon, lat, starts, feats, masks))
    del ways, ids, lon, lat, starts, masks
    log("graph: %d edges, %d with turn lanes, %d speed cameras on a road" % (
        len(edges), sum(1 for e in edges if e.turn_fwd or e.turn_bwd),
        sum(1 for e in edges for f in e.feats if f[1] == rsmap.FEATURE_ID["speed_camera"])))
    t3 = time.time()
    out = a.out or os.path.join(d, "packs", "%s-%s.rsmap" % (name, (osm_ts[:10] or time.strftime("%Y-%m-%d")).replace("-", "")))
    os.makedirs(os.path.dirname(out), exist_ok=True)
    manifest = {
        "region": name, "source": "%s/%s-latest.osm.pbf" % (GEOFABRIK, a.region) if not a.pbf else os.path.basename(pbf),
        "osm_timestamp": osm_ts, "built": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
        "attribution": ATTRIBUTION, "licence": "ODbL-1.0", "tracks": bool(a.tracks), "landuse": not a.no_landuse,
        "classes": list(rsmap.CLASSES), "features": list(rsmap.FEATURES), "surfaces": list(rsmap.SURFACES),
    }
    m = rsmap.build_pack(out, edges, manifest, progress=lambda n, tot: log("  tile %d / %d" % (n, tot)))
    t4 = time.time()
    log("pack %s: %.1f MB, %d tiles, %d edges, %d nodes, %d points" %
        (out, os.path.getsize(out) / 1e6, m["tiles"], m["edges"], m["nodes"], m["points"]))
    log("time: roads %.0f s, landuse %.0f s, graph %.0f s, write %.0f s, total %.0f s" %
        (t1 - t0, t2 - t1, t3 - t2, t4 - t3, t4 - t0))
    print(out)


# ---- info / verify -------------------------------------------------------------------------------
def cmd_info(a):
    p = rsmap.Pack(a.pack)
    m = dict(p.manifest, file=os.path.abspath(a.pack), size_bytes=os.path.getsize(a.pack))
    print(json.dumps(m, indent=1, ensure_ascii=False, sort_keys=True))


def cmd_verify(a):
    p = rsmap.Pack(a.pack, cache_tiles=64)
    ne = nn = 0
    km = 0.0
    cls_km = {}
    bad = 0
    keys = [int(k) for k in p.index["key"]]
    for n, k in enumerate(keys):
        t = p.tile(k)
        nn += len(t.nodes)
        for i in range(len(t.edges)):
            e = t.edge(i)
            ne += 1
            km += e.length / 1000
            cls_km[e.cls_name] = cls_km.get(e.cls_name, 0.0) + e.length / 1000
            if a.deep:
                for end, nref in ((0, e.a), (1, e.b)):
                    if (e.ref, end) not in p.node_edges(nref):
                        bad += 1
        t._cache.clear()
    print(json.dumps({"tiles": len(keys), "edges": ne, "nodes": nn, "road_km": round(km), "broken_refs": bad,
                      "km_by_class": {k: round(v) for k, v in sorted(cls_km.items(), key=lambda x: -x[1])}}, indent=1))
    if ne != p.manifest.get("edges") or bad:
        sys.exit("verify FAILED")


# ---- install -------------------------------------------------------------------------------------
def sha256(path):
    h = hashlib.sha256()
    with open(path, "rb") as f:
        for b in iter(lambda: f.read(1 << 20), b""):
            h.update(b)
    return h.hexdigest()


def cmd_install(a):
    """Copy the pack to <host>:<remote-dir>/ (rsync, resumable), check its sha256 there, and print the
    root step that makes it current. Nothing is restarted: mapd picks up a changed
    /var/lib/roadsense/maps/current.rsmap by itself."""
    rsmap.Pack(a.pack).close()                         # refuse to ship something that is not a pack
    name = os.path.basename(a.pack)
    want = sha256(a.pack)
    remote = "%s/%s" % (a.remote_dir.rstrip("/"), name)
    cmds = [["ssh", a.host, "mkdir -p %s" % a.remote_dir],
            ["rsync", "--partial", "--inplace", "--info=progress2", a.pack, "%s:%s.part" % (a.host, remote)],
            ["ssh", a.host, "sha256sum %s.part" % remote]]
    activate = ("sudo install -d /var/lib/roadsense/maps && sudo ln -sfn \"$(readlink -f %s)\" "
                "/var/lib/roadsense/maps/current.rsmap" % remote)
    if a.dry_run:
        for c in cmds:
            print(" ".join(c))
        print("ssh %s 'mv %s.part %s'" % (a.host, remote, remote))
        print("# then, as root on the device:\nssh -t %s '%s'" % (a.host, activate))
        return
    subprocess.run(cmds[0], check=True)
    subprocess.run(cmds[1], check=True)
    got = subprocess.run(cmds[2], check=True, capture_output=True, text=True).stdout.split()[0]
    if got != want:
        sys.exit("sha256 mismatch on the device (%s, expected %s); the .part file was left there" % (got, want))
    subprocess.run(["ssh", a.host, "mv %s.part %s" % (remote, remote)], check=True)
    log("installed %s:%s (sha256 ok)" % (a.host, remote))
    print("# make it the current pack (root on the device; mapd reloads it without a restart):")
    print("ssh -t %s '%s'" % (a.host, activate))


def main():
    ap = argparse.ArgumentParser(prog="roadsense-maps", description=__doc__.split("\n\n")[0])
    ap.add_argument("--dir", help="data directory (default ~/roadsense-maps, env ROADSENSE_MAPS_DIR)")
    sub = ap.add_subparsers(dest="cmd", required=True)
    s = sub.add_parser("fetch", help="download a Geofabrik extract (md5-checked)")
    s.add_argument("--region", default="europe/poland", help="Geofabrik path (default europe/poland)")
    s.add_argument("--force", action="store_true")
    s.set_defaults(fn=cmd_fetch)
    s = sub.add_parser("build", help="build a pack from an extract")
    s.add_argument("--region", default="europe/poland")
    s.add_argument("--pbf", help="extract file (default: the fetched one)")
    s.add_argument("--out", help="pack file (default <dir>/packs/<region>-<osm date>.rsmap)")
    s.add_argument("--tracks", action="store_true", help="include highway=track")
    s.add_argument("--no-landuse", action="store_true", help="skip the built-up / forest inference")
    s.set_defaults(fn=cmd_build)
    s = sub.add_parser("info", help="print a pack's manifest")
    s.add_argument("pack")
    s.set_defaults(fn=cmd_info)
    s = sub.add_parser("verify", help="decode every tile of a pack")
    s.add_argument("pack")
    s.add_argument("--deep", action="store_true", help="also check every edge/node cross reference")
    s.set_defaults(fn=cmd_verify)
    s = sub.add_parser("install", help="copy a pack to a device")
    s.add_argument("pack")
    s.add_argument("--host", default="user@192.168.0.86")
    s.add_argument("--remote-dir", default="roadsense-maps", help="directory on the device (relative to the home directory)")
    s.add_argument("--dry-run", action="store_true", help="print the commands only")
    s.set_defaults(fn=cmd_install)
    a = ap.parse_args()
    a.fn(a)


if __name__ == "__main__":
    main()
