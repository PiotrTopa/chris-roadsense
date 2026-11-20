"""A small hand-made road network written as a real pack, for tests (no map data in the repository).

Metres east (x) / north (y) of ORIGIN; the main road runs east along y = 0 and crosses a tile border.

    x     0 .. 1000   primary DK1 "Main", 2 lanes, maxspeed 70 (tagged), built-up (tagged), lit
       1000 .. 2000   same road, no maxspeed, built-up (inferred)                 -> implicit 50
       2000 .. 3000   same road, no maxspeed, not built-up, forest                -> implicit 90
       3000 .. 3800   tunnel, maxspeed 80
       3800 .. 5000   after the tunnel, maxspeed 90 (tagged, from a sign)
       5000 ..        left bend, radius 150 m, 90 degrees, then 500 m north
    traffic signals at x = 1000 (a graph node), a pedestrian crossing at x = 1200, a level crossing at x = 2500
    side street "Side" (residential) north from x = 500, 400 m
    service road "Access" parallel to the main road 15 m south of it, x = 200 .. 900, joined at both ends
    a one-way pair far north (y = 2000): motorway "A9" eastbound, 2 lanes, no maxspeed   -> implicit 140
    a living street, expressway pieces (motorroad; highway=motorway with S refs) for the implicit-limit table
    far north (y = 6000): "Twin", a road with a parallel twin 8 m beside it for 1.2 km; (y = 7000): "Corner", a road
    that turns 90 degrees left at a junction where "Straight On" continues
"""
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from chris_mapper import geo, rsmap as R      # noqa: E402

ORIGIN = (52.0, 20.99)          # the tile border at lon 21.0 is ~685 m east of the origin
C = R.CLASS_ID
FT = R.FEATURE_ID


def ll(x, y):
    return geo.offset(ORIGIN[0], ORIGIN[1], y, x)


def _edge(a, b, xy, cls, **kw):
    lat, lon = zip(*[ll(x, y) for x, y in xy])
    return R.RawEdge(a, b, R.pts_from_latlon(lat, lon), C[cls], **kw)


def edges():
    main = dict(name="Main", ref="DK1", lanes=2)
    bu_t = R.F_BUILTUP | R.F_BUILTUP_TAGGED
    out = [
        _edge("m0", "m200", [(0, 0), (100, 0), (200, 0)], "primary", flags=bu_t | R.F_LIT_YES, ms_fwd=70, ms_bwd=70, **main),
        _edge("m200", "m500", [(200, 0), (350, 0), (500, 0)], "primary", flags=bu_t | R.F_LIT_YES, ms_fwd=70, ms_bwd=70, **main),
        _edge("m500", "m900", [(500, 0), (700, 0), (900, 0)], "primary", flags=bu_t | R.F_LIT_YES, ms_fwd=70, ms_bwd=70, **main),
        _edge("m900", "m1000", [(900, 0), (1000, 0)], "primary", flags=bu_t | R.F_LIT_YES, ms_fwd=70, ms_bwd=70,
              feats=[(1, FT["traffic_signals"], R.DIR_BOTH)], **main),
        _edge("m1000", "m2000", [(1000, 0), (1200, 0), (1600, 0), (2000, 0)], "primary", flags=R.F_BUILTUP,
              feats=[(0, FT["traffic_signals"], R.DIR_BOTH), (1, FT["crossing"], R.DIR_BOTH)], **main),
        _edge("m2000", "m3000", [(2000, 0), (2500, 0), (3000, 0)], "primary", flags=R.F_FOREST,
              feats=[(1, FT["level_crossing"], R.DIR_BOTH)], **main),
        _edge("m3000", "m3800", [(3000, 0), (3400, 0), (3800, 0)], "primary", flags=R.F_TUNNEL | R.F_LIT_YES,
              ms_fwd=80, ms_bwd=80, layer=-1, **main),
        _edge("m3800", "m5000", [(3800, 0), (4400, 0), (5000, 0)], "primary", ms_fwd=90, ms_bwd=90,
              flags2=R.F2_MS_SIGN, **main),
        _edge("m5000", "m_end", [(5000, 0)] + [(5000 + 150 * math.sin(math.radians(a)), 150 - 150 * math.cos(math.radians(a)))
                                             for a in range(5, 91, 5)] + [(5150, 400), (5150, 650)], "primary",
              ms_fwd=90, ms_bwd=90, **main),
        _edge("m500", "s_end", [(500, 0), (500, 200), (500, 400)], "residential", flags=R.F_BUILTUP, name="Side"),
        _edge("m200", "p200", [(200, 0), (200, -15)], "service", flags=R.F_BUILTUP, name="Access"),
        _edge("p200", "p900", [(200, -15), (550, -15), (900, -15)], "service", flags=R.F_BUILTUP, name="Access"),
        _edge("p900", "m900", [(900, -15), (900, 0)], "service", flags=R.F_BUILTUP, name="Access"),
        _edge("a0", "a1", [(0, 2000), (1500, 2000), (3000, 2000)], "motorway", flags=R.F_ONEWAY | R.F_DUAL, lanes=2,
              lanes_fwd=2, ref="A9"),
        _edge("l0", "l1", [(0, 3000), (300, 3000)], "living_street", flags=R.F_BUILTUP, name="Quiet"),
        _edge("x0", "x1", [(0, 4000), (1000, 4000)], "trunk", flags=R.F_ONEWAY | R.F_DUAL | R.F_MOTORROAD, lanes=2,
              lanes_fwd=2, ref="S9"),
        _edge("x2", "x3", [(0, 4100), (1000, 4100)], "trunk", flags=R.F_MOTORROAD, ref="S9"),
        _edge("d0", "d1", [(0, 4200), (1000, 4200)], "secondary", flags=R.F_ONEWAY | R.F_DUAL, lanes=2, lanes_fwd=2),
        _edge("k0", "k1", [(0, 4300), (300, 4300)], "motorway_link", flags=R.F_ONEWAY | R.F_LINK),
        _edge("e0", "e1", [(0, 4500), (1000, 4500)], "motorway", flags=R.F_ONEWAY | R.F_DUAL, lanes=2, lanes_fwd=2,
              ref="S7;S9"),
        _edge("t0", "t1", [(0, 4400), (500, 4400)], "tertiary", lanes=3, lanes_fwd=2, lanes_bwd=1,
              flags2=R.F2_LANES_SPLIT),
        # lane L47: a road with a twin 8 m north of it between x = 300 and 1500 (joined at both ends), and a road
        # that turns north at a junction where another one goes straight on
        _edge("w0", "w300", [(0, 6000), (150, 6000), (300, 6000)], "secondary", name="Twin"),
        _edge("w300", "w1500", [(300, 6000), (900, 6000), (1500, 6000)], "secondary", name="Twin"),
        _edge("w1500", "w2400", [(1500, 6000), (2000, 6000), (2400, 6000)], "secondary", name="Twin"),
        _edge("w300", "v300", [(300, 6000), (300, 6008)], "secondary", name="Twin North"),
        _edge("v300", "v1500", [(300, 6008), (900, 6008), (1500, 6008)], "secondary", name="Twin North"),
        _edge("v1500", "w1500", [(1500, 6008), (1500, 6000)], "secondary", name="Twin North"),
        _edge("c0", "c800", [(0, 7000), (400, 7000), (800, 7000)], "tertiary", name="Corner"),
        _edge("c800", "c_n", [(800, 7000), (800, 7300), (800, 7700)], "tertiary", name="Corner"),
        _edge("c800", "c_e", [(800, 7000), (1200, 7000), (1700, 7000)], "tertiary", name="Straight On"),
    ]
    return out


def build(path):
    return R.build_pack(path, edges(), {"region": "synthetic", "osm_timestamp": "", "built": "test",
                                        "attribution": "hand-made test network"})


def main_road_path(pack, x0=20.0, x1=5600.0):
    """[(edge, dir, s_from, s_to)] along the main road from x0 to x1 (eastwards), for sim.Drive."""
    from chris_mapper import sim
    return sim.route(pack, ll(x0, 0), ll(5150, 600) if x1 > 5200 else ll(x1, 0))


if __name__ == "__main__":
    print(build(sys.argv[1]))
