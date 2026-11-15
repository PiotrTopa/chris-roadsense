"""The roadsense map pack (`.rsmap`, format version 1): one file, tiled road graph, read through mmap.

Layout (little-endian; docs/MAPS.md has the full description):

    header   64 bytes   "RSMAP001", u32 version, u32 manifest_len, u64 manifest_off,
                        u64 index_off, u64 index_count, zero padding
    manifest            UTF-8 JSON (region, OSM timestamp, build time, counts, attribution)
    tiles               zlib-compressed tile blobs, one per non-empty tile
    index               index_count x {u32 key, u64 off, u32 clen, u32 rlen}, sorted by key

A tile is a 1/32 degree cell (about 3.5 km x 2.1 km in Poland); key = ty << 16 | tx with
tx = floor((lon + 180) * 32), ty = floor((lat + 90) * 32). Coordinates inside a tile are int32
micro-degrees relative to the tile's south-west corner. The road network is a graph: edges (a road
piece between two graph nodes, with its geometry and attributes) and nodes (junctions, ends and
attribute changes) with their incident edges. An edge lives in the tile of its first point; tiles it
passes through list it as a foreign edge so that a spatial query finds it.

This module has no dependency beyond numpy; the builder (tools/roadsense_maps.py), the daemon and the
tests all use it.
"""
import collections
import json
import math
import mmap
import os
import struct
import zlib

import numpy as np

MAGIC = b"RSMAP001"
VERSION = 1
HEADER = struct.Struct("<8sIIQQQ")
HEADER_SIZE = 64
TILES_PER_DEG = 32
GRID_N = 16                               # spatial cells per tile side
CELLS_PER_DEG = TILES_PER_DEG * GRID_N    # 512: a cell is ~217 m x ~133 m in Poland
UNIT = 1e-6                               # degrees per coordinate unit
TILE_UNITS = int(round(1e6 / TILES_PER_DEG))

INDEX_DT = np.dtype([("key", "<u4"), ("off", "<u8"), ("clen", "<u4"), ("rlen", "<u4")])
NODE_DT = np.dtype([("x", "<i4"), ("y", "<i4"), ("adj_start", "<u4"), ("adj_count", "<u2"), ("flags", "<u2")])
ADJ_DT = np.dtype([("tile", "<u4"), ("edge", "<u2"), ("end", "<u2")])
EDGE_DT = np.dtype([
    ("a_tile", "<u4"), ("b_tile", "<u4"), ("a_idx", "<u2"), ("b_idx", "<u2"),
    ("pt_start", "<u4"), ("pt_count", "<u2"), ("feat_count", "<u2"), ("feat_start", "<u4"),
    ("length_dm", "<u4"), ("flags", "<u2"), ("name", "<u2"), ("ref", "<u2"),
    ("cls", "u1"), ("flags2", "u1"), ("ms_fwd", "u1"), ("ms_bwd", "u1"),
    ("lanes", "u1"), ("lanes_fwd", "u1"), ("lanes_bwd", "u1"), ("layer", "i1"),
    ("surface", "u1"), ("pad", "u1")])
FEAT_DT = np.dtype([("pt", "<u2"), ("kind", "u1"), ("dir", "u1")])
FOREIGN_DT = np.dtype([("tile", "<u4"), ("edge", "<u2"), ("pad", "<u2")])
TILE_HEADER = struct.Struct("<10I")       # strings_bytes, nodes, adj, edges, points, feats, foreign, grid_n, grid_items, 0

# ---- enumerations (stable: part of the format) ---------------------------------------------------
CLASSES = ("unknown", "motorway", "motorway_link", "trunk", "trunk_link", "primary", "primary_link",
           "secondary", "secondary_link", "tertiary", "tertiary_link", "unclassified", "residential",
           "living_street", "service", "track")
CLASS_ID = {c: i for i, c in enumerate(CLASSES)}
SURFACES = ("unknown", "asphalt", "concrete", "paved", "paving_stones", "sett", "compacted", "gravel",
            "unpaved", "ground", "other")
SURFACE_ID = {s: i for i, s in enumerate(SURFACES)}
FEATURES = ("none", "traffic_signals", "crossing", "crossing_signals", "level_crossing", "tram_crossing",
            "stop", "give_way", "mini_roundabout", "speed_bump", "toll_booth", "motorway_junction")
FEATURE_ID = {f: i for i, f in enumerate(FEATURES)}
DIR_BOTH, DIR_FORWARD, DIR_BACKWARD = 0, 1, 2

# edge flags (u16)
F_ONEWAY = 1 << 0          # traffic only from node a to node b (geometry order)
F_ROUNDABOUT = 1 << 1
F_TUNNEL = 1 << 2
F_BRIDGE = 1 << 3
F_LIT_YES = 1 << 4
F_LIT_NO = 1 << 5
F_MOTORROAD = 1 << 6       # expressway (motorroad=yes)
F_LINK = 1 << 7
F_BUILTUP = 1 << 8         # inside a built-up area
F_BUILTUP_TAGGED = 1 << 9  # ... and that is stated by a tag on the way (else inferred from landuse / class)
F_ACCESS_RESTRICTED = 1 << 10   # access / motor_vehicle = private, no, destination, customers ...
F_DUAL = 1 << 11           # one carriageway of a dual carriageway (tagged, or one-way major road)
F_TOLL = 1 << 12
F_FOREST = 1 << 13         # runs through or along forest (landuse=forest / natural=wood)
F_PARKING = 1 << 14        # service=parking_aisle or inside amenity=parking
F_COVERED = 1 << 15        # covered=yes / building passage / tunnel=building_passage
# flags2 (u8)
F2_MS_SIGN = 1 << 0        # maxspeed tag stated to come from a sign (source:maxspeed / maxspeed:type = sign)
F2_MS_ZONE = 1 << 1        # maxspeed tag is an implicit-zone value (PL:urban, PL:rural ...)
F2_MS_VARIABLE = 1 << 2    # maxspeed=signals / maxspeed:variable
F2_LANES_SPLIT = 1 << 3    # lanes:forward / lanes:backward were tagged
MS_NONE = 255              # maxspeed=none; 0 = not tagged


def tile_key(lat, lon):
    return (int(math.floor((lat + 90.0) * TILES_PER_DEG)) << 16) | int(math.floor((lon + 180.0) * TILES_PER_DEG))


def tile_origin(key):
    """(lat, lon) of the south-west corner."""
    return (key >> 16) / TILES_PER_DEG - 90.0, (key & 0xFFFF) / TILES_PER_DEG - 180.0


def _pad4(b):
    return b + b"\0" * (-len(b) % 4)


# ---- writing -------------------------------------------------------------------------------------
class RawEdge:
    """One edge handed to build_pack. `pts`: int32 array (n, 2) of absolute micro-degrees (lon, lat);
    `a`, `b`: hashable ids of its end nodes; `feats`: [(point index, feature id, dir)]."""
    __slots__ = ("a", "b", "pts", "cls", "flags", "flags2", "ms_fwd", "ms_bwd", "lanes", "lanes_fwd",
                 "lanes_bwd", "layer", "surface", "name", "ref", "feats")

    def __init__(self, a, b, pts, cls, flags=0, flags2=0, ms_fwd=0, ms_bwd=0, lanes=0, lanes_fwd=0,
                 lanes_bwd=0, layer=0, surface=0, name="", ref="", feats=()):
        self.a, self.b, self.pts = a, b, pts
        self.cls, self.flags, self.flags2 = cls, flags, flags2
        self.ms_fwd, self.ms_bwd = ms_fwd, ms_bwd
        self.lanes, self.lanes_fwd, self.lanes_bwd = lanes, lanes_fwd, lanes_bwd
        self.layer, self.surface, self.name, self.ref, self.feats = layer, surface, name, ref, feats


def pts_from_latlon(lat, lon):
    return np.stack([np.round(np.asarray(lon, dtype=np.float64) * 1e6),
                     np.round(np.asarray(lat, dtype=np.float64) * 1e6)], axis=1).astype(np.int32)


def _pt_tile(p):
    """Tile key of an absolute micro-degree point (lon, lat)."""
    return ((int(p[1]) + 90_000_000) // TILE_UNITS << 16) | ((int(p[0]) + 180_000_000) // TILE_UNITS)


def _edge_length_m(pts):
    lat = pts[:, 1] * UNIT
    mlat = 111194.9
    mlon = mlat * math.cos(math.radians(float(lat[0])))
    d = np.diff(pts.astype(np.float64), axis=0) * UNIT
    return float(np.hypot(d[:, 0] * mlon, d[:, 1] * mlat).sum())


def _edge_cells(pts):
    """Global cell ids (gy << 20 | gx) the polyline passes through (sampled every <= ~20 m)."""
    p = pts.astype(np.float64) * UNIT
    d = np.abs(np.diff(p, axis=0))
    n = np.maximum(1, np.ceil(np.maximum(d[:, 0] / 0.0003, d[:, 1] / 0.00018))).astype(np.int64)
    if int(n.sum()) == len(n):                       # short segments only: the vertices are enough
        q = p
    else:
        idx = np.repeat(np.arange(len(n)), n)
        first = np.cumsum(n) - n
        t = (np.arange(len(idx)) - first[idx]) / n[idx]
        q = np.concatenate([p[idx] + (p[idx + 1] - p[idx]) * t[:, None], p[-1:]])
    gx = np.floor((q[:, 0] + 180.0) * CELLS_PER_DEG).astype(np.int64)
    gy = np.floor((q[:, 1] + 90.0) * CELLS_PER_DEG).astype(np.int64)
    return np.unique((gy << 20) | gx)


def build_pack(path, edges, manifest, level=6, progress=None):
    """Write a pack from RawEdge objects. Returns the manifest written (with counts)."""
    tiles = collections.defaultdict(list)            # tile key -> [RawEdge] (home edges, index = position)
    node_pos = {}                                    # node id -> (lon_u, lat_u)
    for e in edges:
        if len(e.pts) < 2 or len(e.pts) > 65535:
            raise ValueError("edge with %d points" % len(e.pts))
        tiles[_pt_tile(e.pts[0])].append(e)
        node_pos.setdefault(e.a, (int(e.pts[0][0]), int(e.pts[0][1])))
        node_pos.setdefault(e.b, (int(e.pts[-1][0]), int(e.pts[-1][1])))
    node_ref = {}                                    # node id -> (tile key, idx)
    tile_nodes = collections.defaultdict(list)       # tile key -> [node id]
    for nid, p in node_pos.items():
        k = _pt_tile(p)
        node_ref[nid] = (k, len(tile_nodes[k]))
        tile_nodes[k].append(nid)
    adj = collections.defaultdict(list)              # node id -> [(tile, edge idx, end)]
    cells = collections.defaultdict(lambda: collections.defaultdict(list))   # tile -> cell -> [item]
    foreign = collections.defaultdict(dict)          # tile -> {(home tile, edge idx): foreign idx}
    n_edges = n_points = 0
    for k, lst in tiles.items():
        if len(lst) > 65535:
            raise ValueError("tile %08x has %d edges" % (k, len(lst)))
    for k in tile_nodes:
        if len(tile_nodes[k]) > 65535:
            raise ValueError("tile %08x has %d nodes" % (k, len(tile_nodes[k])))
    for k, lst in tiles.items():
        for i, e in enumerate(lst):
            adj[e.a].append((k, i, 0))
            adj[e.b].append((k, i, 1))
            n_edges += 1
            n_points += len(e.pts)
            for c in _edge_cells(e.pts).tolist():
                gy, gx = c >> 20, c & 0xFFFFF
                tk = ((gy // GRID_N) << 16) | (gx // GRID_N)
                cell = (gy % GRID_N) * GRID_N + gx % GRID_N
                if tk == k:
                    cells[tk][cell].append(("h", i))
                else:
                    f = foreign[tk]
                    cells[tk][cell].append(("f", f.setdefault((k, i), len(f))))
    keys = sorted(set(tiles) | set(tile_nodes) | set(cells))
    manifest = dict(manifest, format=VERSION, tiles=len(keys), edges=n_edges, nodes=len(node_pos), points=n_points)
    mj = json.dumps(manifest, ensure_ascii=False, sort_keys=True).encode()
    index = np.zeros(len(keys), dtype=INDEX_DT)
    tmp = path + ".tmp"
    with open(tmp, "wb") as f:
        f.write(b"\0" * HEADER_SIZE)
        f.write(mj)
        f.write(b"\0" * (-f.tell() % 8))
        for n, k in enumerate(keys):
            raw = _encode_tile(k, tiles.get(k, []), tile_nodes.get(k, []), node_pos, node_ref, adj,
                               cells.get(k, {}), foreign.get(k, {}))
            comp = zlib.compress(raw, level)
            index[n] = (k, f.tell(), len(comp), len(raw))
            f.write(comp)
            if progress and n % 2000 == 0:
                progress(n, len(keys))
        f.write(b"\0" * (-f.tell() % 8))
        index_off = f.tell()
        f.write(index.tobytes())
        f.seek(0)
        f.write(HEADER.pack(MAGIC, VERSION, len(mj), HEADER_SIZE, index_off, len(keys)))
    os.replace(tmp, path)
    return manifest


def _encode_tile(key, edges, nodes, node_pos, node_ref, adj, cells, foreign):
    lat0, lon0 = tile_origin(key)
    ox, oy = int(round(lon0 * 1e6)), int(round(lat0 * 1e6))
    strings, sidx = [], {"": 0}

    def sid(s):
        if s not in sidx:
            if len(sidx) > 65535:
                return 0
            sidx[s] = len(sidx)
            strings.append(s)
        return sidx[s]

    na = np.zeros(len(nodes), dtype=NODE_DT)
    adj_rows = []
    for i, nid in enumerate(nodes):
        p = node_pos[nid]
        lst = adj[nid]
        na[i] = (p[0] - ox, p[1] - oy, len(adj_rows), len(lst), 0)
        adj_rows += lst
    aa = np.array(adj_rows, dtype=ADJ_DT) if adj_rows else np.zeros(0, dtype=ADJ_DT)
    ea = np.zeros(len(edges), dtype=EDGE_DT)
    pts_parts, feat_rows, npts = [], [], 0
    for i, e in enumerate(edges):
        ar, br = node_ref[e.a], node_ref[e.b]
        rel = e.pts.astype(np.int64) - (ox, oy)
        d = rel.copy()
        d[1:] -= rel[:-1]                              # delta-coded: first point relative to the tile corner
        pts_parts.append(d.astype(np.int32))
        ea[i] = (ar[0], br[0], ar[1], br[1], npts, len(e.pts), len(e.feats), len(feat_rows),
                 min(0xFFFFFFFF, int(round(_edge_length_m(e.pts) * 10))), e.flags, sid(e.name), sid(e.ref),
                 e.cls, e.flags2, e.ms_fwd, e.ms_bwd, e.lanes, e.lanes_fwd, e.lanes_bwd, e.layer, e.surface, 0)
        npts += len(e.pts)
        feat_rows += [tuple(x) for x in e.feats]
    pa = np.concatenate(pts_parts) if pts_parts else np.zeros((0, 2), dtype=np.int32)
    fa = np.array(feat_rows, dtype=FEAT_DT) if feat_rows else np.zeros(0, dtype=FEAT_DT)
    fo = np.zeros(len(foreign), dtype=FOREIGN_DT)
    for (tk, ei), fi in foreign.items():
        fo[fi] = (tk, ei, 0)
    starts = np.zeros(GRID_N * GRID_N + 1, dtype="<u4")
    items = []
    for c in range(GRID_N * GRID_N):
        for kind, i in cells.get(c, ()):
            items.append(i if kind == "h" else len(edges) + i)
        starts[c + 1] = len(items)
    if items and max(items) > 65535:
        raise ValueError("tile %08x: more than 65535 grid items" % key)
    ia = np.array(items, dtype="<u2")
    sb = b"".join(struct.pack("<H", len(b)) + b for b in (s.encode()[:60000] for s in strings))
    parts = [TILE_HEADER.pack(len(sb), len(na), len(aa), len(ea), len(pa), len(fa), len(fo), GRID_N, len(ia), 0),
             _pad4(sb), na.tobytes(), aa.tobytes(), ea.tobytes(), pa.astype("<i4").tobytes(), fa.tobytes(),
             fo.tobytes(), starts.tobytes(), _pad4(ia.tobytes())]
    return b"".join(parts)


# ---- reading -------------------------------------------------------------------------------------
class Edge:
    """A decoded edge. `lat`/`lon`: float arrays; `x`/`y`: metres east/north of its first point;
    `s`: cumulative length along the points. ref = (tile key, index)."""
    __slots__ = ("ref", "a", "b", "lat", "lon", "x", "y", "s", "length", "mlat", "mlon", "bbox", "brg0", "brg1", "cls", "flags", "flags2",
                 "ms_fwd", "ms_bwd", "lanes", "lanes_fwd", "lanes_bwd", "layer", "surface", "name", "road_ref",
                 "feats", "_seg")

    @property
    def cls_name(self):
        return CLASSES[self.cls] if self.cls < len(CLASSES) else "unknown"

    @property
    def oneway(self):
        return bool(self.flags & F_ONEWAY)

    def project(self, lat, lon):
        """Nearest point of the polyline: (distance m, s along m, bearing deg of that segment a->b)."""
        px, py = (lon - self.lon[0]) * self.mlon, (lat - self.lat[0]) * self.mlat
        seg = self._seg
        if seg is None:
            dx, dy = np.diff(self.x), np.diff(self.y)
            l2 = dx * dx + dy * dy
            l2[l2 == 0] = 1e-9
            seg = self._seg = (dx, dy, l2)
        dx, dy, l2 = seg
        t = np.clip(((px - self.x[:-1]) * dx + (py - self.y[:-1]) * dy) / l2, 0.0, 1.0)
        ex, ey = self.x[:-1] + t * dx - px, self.y[:-1] + t * dy - py
        d2 = ex * ex + ey * ey
        i = int(np.argmin(d2))
        return (math.sqrt(float(d2[i])), float(self.s[i] + t[i] * (self.s[i + 1] - self.s[i])),
                math.degrees(math.atan2(float(dx[i]), float(dy[i]))) % 360.0)

    def at(self, s):
        """(lat, lon, bearing deg a->b) at distance s along the edge (clamped)."""
        s = min(max(s, 0.0), self.length)
        i = int(np.searchsorted(self.s, s, side="right")) - 1
        i = min(max(i, 0), len(self.s) - 2)
        seg = self.s[i + 1] - self.s[i]
        t = (s - self.s[i]) / seg if seg > 0 else 0.0
        return (float(self.lat[i] + t * (self.lat[i + 1] - self.lat[i])),
                float(self.lon[i] + t * (self.lon[i + 1] - self.lon[i])),
                math.degrees(math.atan2(float(self.x[i + 1] - self.x[i]), float(self.y[i + 1] - self.y[i]))) % 360.0)

    def node(self, end):
        return self.b if end else self.a


class Tile:
    def __init__(self, key, raw):
        self.key = key
        self.lat0, self.lon0 = tile_origin(key)
        nsb, nn, nadj, ne, npt, nf, nfo, gn, ni, _ = TILE_HEADER.unpack_from(raw, 0)
        off = TILE_HEADER.size
        self.strings = [""]
        end = off + nsb
        p = off
        while p < end:
            (ln,) = struct.unpack_from("<H", raw, p)
            self.strings.append(raw[p + 2:p + 2 + ln].decode("utf-8", "replace"))
            p += 2 + ln
        off += nsb + (-nsb % 4)

        def take(dt, n):
            nonlocal off
            a = np.frombuffer(raw, dtype=dt, count=n, offset=off)
            off += dt.itemsize * n
            return a

        self.nodes = take(NODE_DT, nn)
        self.adj = take(ADJ_DT, nadj)
        self.edges = take(EDGE_DT, ne)
        self.points = take(np.dtype("<i4"), npt * 2).reshape(-1, 2)
        self.feats = take(FEAT_DT, nf)
        self.foreign = take(FOREIGN_DT, nfo)
        self.grid_n = gn
        self.grid_starts = take(np.dtype("<u4"), gn * gn + 1)
        self.grid_items = take(np.dtype("<u2"), ni)
        self._cache = {}

    def edge(self, idx):
        e = self._cache.get(idx)
        if e is not None:
            return e
        r = self.edges[idx]
        e = Edge()
        e.ref = (self.key, idx)
        e.a, e.b = (int(r["a_tile"]), int(r["a_idx"])), (int(r["b_tile"]), int(r["b_idx"]))
        ps, pc = int(r["pt_start"]), int(r["pt_count"])
        p = np.cumsum(self.points[ps:ps + pc].astype(np.int64), axis=0)
        e.lon = self.lon0 + p[:, 0] * UNIT
        e.lat = self.lat0 + p[:, 1] * UNIT
        e.mlat = 111194.9
        e.mlon = e.mlat * math.cos(math.radians(float(e.lat[0])))
        e.x = (e.lon - e.lon[0]) * e.mlon
        e.y = (e.lat - e.lat[0]) * e.mlat
        e.s = np.concatenate([[0.0], np.cumsum(np.hypot(np.diff(e.x), np.diff(e.y)))])
        e.length = float(e.s[-1])
        e.bbox = (float(e.lat.min()), float(e.lat.max()), float(e.lon.min()), float(e.lon.max()))
        # bearing (a -> b) of the first and of the last segment
        e.brg0 = math.degrees(math.atan2(float(e.x[1] - e.x[0]), float(e.y[1] - e.y[0]))) % 360.0
        e.brg1 = math.degrees(math.atan2(float(e.x[-1] - e.x[-2]), float(e.y[-1] - e.y[-2]))) % 360.0
        e._seg = None
        e.cls, e.flags, e.flags2 = int(r["cls"]), int(r["flags"]), int(r["flags2"])
        e.ms_fwd, e.ms_bwd = int(r["ms_fwd"]), int(r["ms_bwd"])
        e.lanes, e.lanes_fwd, e.lanes_bwd = int(r["lanes"]), int(r["lanes_fwd"]), int(r["lanes_bwd"])
        e.layer, e.surface = int(r["layer"]), int(r["surface"])
        e.name, e.road_ref = self.strings[int(r["name"])], self.strings[int(r["ref"])]
        fs, fc = int(r["feat_start"]), int(r["feat_count"])
        e.feats = [(int(f["pt"]), int(f["kind"]), int(f["dir"])) for f in self.feats[fs:fs + fc]]
        self._cache[idx] = e
        return e


class Pack:
    """Read access to a pack file. Tiles are decompressed on demand and kept in a small LRU cache."""

    def __init__(self, path, cache_tiles=24):
        self.path = path
        self._f = open(path, "rb")
        st = os.fstat(self._f.fileno())
        self.identity = (st.st_dev, st.st_ino, st.st_size, st.st_mtime_ns)
        self._mm = mmap.mmap(self._f.fileno(), 0, access=mmap.ACCESS_READ)
        magic, ver, mlen, moff, ioff, icount = HEADER.unpack_from(self._mm, 0)
        if magic != MAGIC or ver != VERSION:
            self._mm.close()
            self._f.close()
            if magic != MAGIC:
                raise ValueError("%s: not a roadsense map pack" % path)
            raise ValueError("%s: pack format %d, this reader handles %d" % (path, ver, VERSION))
        self.manifest = json.loads(self._mm[moff:moff + mlen].decode())
        self.index = np.frombuffer(self._mm, dtype=INDEX_DT, count=icount, offset=ioff)
        self._keys = self.index["key"]
        self._tiles = collections.OrderedDict()
        self.cache_tiles = cache_tiles
        self.tile_loads = 0

    def close(self):
        self._tiles.clear()
        self.index = self._keys = None
        try:
            self._mm.close()
        except BufferError:
            pass
        self._f.close()

    def tile(self, key):
        t = self._tiles.get(key)
        if t is not None:
            self._tiles.move_to_end(key)
            return t
        i = int(np.searchsorted(self._keys, key))
        if i >= len(self._keys) or int(self._keys[i]) != key:
            return None
        r = self.index[i]
        off, clen = int(r["off"]), int(r["clen"])
        t = Tile(key, zlib.decompress(self._mm[off:off + clen]))
        self.tile_loads += 1
        self._tiles[key] = t
        while len(self._tiles) > self.cache_tiles:
            self._tiles.popitem(last=False)
        return t

    def edge(self, ref):
        t = self.tile(ref[0])
        return t.edge(ref[1]) if t is not None else None

    def node_pos(self, nref):
        t = self.tile(nref[0])
        n = t.nodes[nref[1]]
        return t.lat0 + int(n["y"]) * UNIT, t.lon0 + int(n["x"]) * UNIT

    def node_edges(self, nref):
        """[(edge ref, end)]: edges at this node; end = 0 when the node is the edge's start a, 1 when its end b."""
        t = self.tile(nref[0])
        if t is None:
            return []
        n = t.nodes[nref[1]]
        s, c = int(n["adj_start"]), int(n["adj_count"])
        return [((int(a["tile"]), int(a["edge"])), int(a["end"])) for a in t.adj[s:s + c]]

    def query(self, lat, lon, radius_m):
        """Edges that may lie within radius_m of the point (cell granularity; callers measure the distance)."""
        mlat = 111194.9
        mlon = mlat * math.cos(math.radians(lat))
        dlat, dlon = (radius_m + 30.0) / mlat, (radius_m + 30.0) / mlon
        gx0 = int(math.floor((lon - dlon + 180.0) * CELLS_PER_DEG))
        gx1 = int(math.floor((lon + dlon + 180.0) * CELLS_PER_DEG))
        gy0 = int(math.floor((lat - dlat + 90.0) * CELLS_PER_DEG))
        gy1 = int(math.floor((lat + dlat + 90.0) * CELLS_PER_DEG))
        refs = set()
        for gy in range(gy0, gy1 + 1):
            for gx in range(gx0, gx1 + 1):
                key = ((gy // GRID_N) << 16) | (gx // GRID_N)
                t = self.tile(key)
                if t is None:
                    continue
                c = (gy % GRID_N) * GRID_N + gx % GRID_N
                ne = len(t.edges)
                for it in t.grid_items[int(t.grid_starts[c]):int(t.grid_starts[c + 1])].tolist():
                    if it < ne:
                        refs.add((key, it))
                    else:
                        fo = t.foreign[it - ne]
                        refs.add((int(fo["tile"]), int(fo["edge"])))
        out = []
        dlat, dlon = radius_m / mlat, radius_m / mlon
        for r in refs:
            e = self.edge(r)
            if e is None:
                continue
            b = e.bbox                                   # cheap reject before anyone projects onto the edge
            if lat < b[0] - dlat or lat > b[1] + dlat or lon < b[2] - dlon or lon > b[3] + dlon:
                continue
            out.append(e)
        return out
