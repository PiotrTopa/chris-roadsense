#!/usr/bin/env python3
"""Synthetic position source: drive a route over a map pack and feed mapd as a real feeder would.

    # live: real time into a running mapd (python3 -m roadsense_mapd --rundir /tmp/roadsense --pack P)
    tools/synth_drive.py --pack P --demo warsaw-s2-tunnel --rundir /tmp/roadsense
    # offline: the same drive through the engine in-process, as fast as it goes, with a timeline and a summary
    tools/synth_drive.py --pack P --demo warsaw-s2-tunnel --offline
    tools/synth_drive.py --pack P --from 50.6683,17.9231 --to 50.4760,17.3340 --dropout 300:420 --offline
    # the messages only (JSON lines), e.g. to replay them later with tools/position_client.py stdin
    tools/synth_drive.py --pack P --demo krakow-city --jsonl /tmp/drive.jsonl

The vehicle follows the fastest route at the map's speed limits; GNSS gets correlated noise, loses its fix in
tunnels (and in every --dropout window) and can be pushed sideways (--bias) to provoke a parallel-road
ambiguity; motion is speed plus yaw rate, steering angle or wheel speeds (--motion).
"""
import argparse
import json
import os
import sys
import time

sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "mapd"))

from roadsense_mapd import geo, sim                  # noqa: E402
from roadsense_mapd.engine import Engine             # noqa: E402
from roadsense_mapd.rsmap import Pack                # noqa: E402

# Named routes on the Poland pack: (from, to, note). Coordinates only; the roads come from the pack.
DEMOS = {
    "warsaw-s2-tunnel": ((52.13889, 20.98064), (52.15258, 21.11298),
                         "S2 Warsaw southern bypass eastbound through the 2.3 km Ursynow tunnel (9.5 km; the road of "
                         "the ride tunnel-pow-warsaw, not time-aligned with it)"),
    "warsaw-city": ((52.2297, 21.0122), (52.1930, 21.0340), "Warsaw centre southwards along Marszalkowska and Pulawska (5.4 km)"),
    "krakow-city": ((50.0647, 19.9450), (50.0350, 19.9400), "Krakow: old town ring, Dietla, Konopnickiej (4.2 km)"),
    "opole-rural": ((50.6420, 17.6190), (50.4760, 17.3340), "Niemodlin to Nysa on DK46 and regional roads (30 km): villages, open road, forest"),
}


def latlon(s):
    a, b = s.split(",")
    return float(a), float(b)


def window(s):
    a, b = s.split(":")
    return float(a), float(b)


def build(a):
    pack = Pack(a.pack, cache_tiles=600)
    if a.demo:
        if a.demo not in DEMOS:
            sys.exit("unknown demo %r; known: %s" % (a.demo, ", ".join(sorted(DEMOS)) or "none"))
        start, goal, note = DEMOS[a.demo]
        print("demo %s: %s" % (a.demo, note), file=sys.stderr)
    elif a.start and a.goal:
        start, goal = a.start, a.goal
    else:
        sys.exit("give --demo NAME or --from LAT,LON --to LAT,LON")
    path = sim.route(pack, start, goal)
    km = sum(abs(s1 - s0) for _, _, s0, s1 in path) / 1000.0
    print("route: %d edges, %.1f km" % (len(path), km), file=sys.stderr)
    drive = sim.Drive(pack, path, lateral_m=a.lateral)
    bias = None
    if a.bias:
        t0, t1, east, north = a.bias
        bias = lambda t: (east, north) if t0 <= t < t1 else (0.0, 0.0)      # noqa: E731
    return pack, path, drive, dict(seed=a.seed, motion=a.motion, dropouts=a.dropout or (), bias_m=bias,
                                   tunnel_blocks_gnss=not a.no_tunnel_loss, explicit_none=a.explicit_none,
                                   gnss_course=not a.no_course, hacc_m=a.hacc, gnss_sigma_m=a.gnss_sigma)


def run_offline(a):
    pack, path, drive, kw = build(a)
    on_path = {e.ref for e, _, _, _ in path}
    eng = Engine(pack)
    t0 = 10 ** 12
    period = int(1e9 / a.hz)
    last = None
    prev = {}
    errs, dr_errs = [], []
    n = n_ok = 0
    states = {}
    cpu0 = time.process_time()
    o = None
    for ts, msg, o in sim.feed(drive, t0_ns=t0, tick_hz=a.hz, **kw):
        if msg is not None:
            err = eng.on_input(msg, ts)
            if err:
                print("rejected:", err, file=sys.stderr)
            continue
        if last is not None and ts - last < 0.99 * period:
            continue
        last = ts
        out = eng.step(ts)
        states[out["state"]] = states.get(out["state"], 0) + 1
        pos = out["position"]
        if pos:
            e = geo.dist_m(pos["lat"], pos["lon"], o.lat, o.lon)
            (dr_errs if out["state"] == "dead_reckoning" else errs).append(e)
        if out["available"]:
            n += 1
            n_ok += eng.match.edge.ref in on_path
        road, lim, ah = out["road"] or {}, out["speed_limit"] or {}, out["ahead"] or {}
        tun = ah.get("tunnel") or {}
        cur = {"state": out["state"], "available": out["available"],
               "road": (road.get("class"), road.get("ref"), road.get("name")) if road else None,
               "limit": (lim.get("limit_kmh"), lim.get("source"), lim.get("rule")) if lim else None,
               "tunnel": tun.get("in_tunnel"), "built_up": (road.get("built_up") or {}).get("value")}
        if cur != prev:
            extra = ""
            if tun.get("in_tunnel"):
                extra = "  tunnel exit in %s m" % tun.get("exit_m")
            elif tun.get("entry_m") is not None:
                extra = "  tunnel in %s m (%s m long)" % (tun["entry_m"], tun.get("length_m"))
            print("%7.1f s  %-14s %-9s %-38s limit %-22s built-up %-5s hacc %5.1f m%s" % (
                o.t, out["state"], "map" if out["available"] else out["reason"],
                "%s %s %s" % cur["road"] if cur["road"] else "-",
                "%s %s %s" % cur["limit"] if cur["limit"] else "-", cur["built_up"],
                pos["hacc_m"] if pos else float("nan"), extra))
            prev = cur
    cpu = time.process_time() - cpu0

    def stat(v):
        if not v:
            return "-"
        v = sorted(v)
        return "mean %.1f  p95 %.1f  max %.1f m (n=%d)" % (sum(v) / len(v), v[int(0.95 * (len(v) - 1))], v[-1], len(v))

    print("\ndrive %.0f s, %.1f km; states %s" % (o.t, drive.length / 1000.0, states))
    print("position error with a fix:      " + stat(errs))
    print("position error dead reckoning:  " + stat(dr_errs))
    print("matched on the driven route:    %d / %d steps (%.1f %%)" % (n_ok, n, 100.0 * n_ok / max(1, n)))
    print("engine CPU: %.2f s for %.0f s of driving = %.2f %% of one core; tiles loaded %d"
          % (cpu, o.t, 100.0 * cpu / o.t, pack.tile_loads))
    if a.dump_last:
        print(json.dumps(out, indent=1, ensure_ascii=False))


def run_live(a):
    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
    from position_client import PositionFeed
    pack, path, drive, kw = build(a)
    feed = PositionFeed(a.rundir, name="synth_drive")
    t0 = time.monotonic_ns()
    n = 0
    for ts, msg, o in sim.feed(drive, t0_ns=t0, source="synthetic", **kw):
        wait = (ts - time.monotonic_ns()) / 1e9
        if wait > 0:
            time.sleep(wait)
        feed.send(msg)
        n += 1
        if n % 400 == 0:
            print("%6.0f s  %.5f %.5f  %5.1f km/h  sent %d dropped %d%s" % (
                o.t, o.lat, o.lon, o.v * 3.6, feed.sent, feed.dropped,
                "  mapd: " + feed.errors[-1]["reason"] if feed.errors else ""), file=sys.stderr)
    print("done: sent %d, dropped %d" % (feed.sent, feed.dropped), file=sys.stderr)


def run_jsonl(a):
    pack, path, drive, kw = build(a)
    with open(a.jsonl, "w") as f:
        for ts, msg, o in sim.feed(drive, t0_ns=0, **kw):
            f.write(json.dumps(msg, separators=(",", ":")) + "\n")


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--pack", default=os.environ.get("ROADSENSE_MAP_PACK"), required="ROADSENSE_MAP_PACK" not in os.environ)
    ap.add_argument("--demo", help="a named route: " + ", ".join(sorted(DEMOS)))
    ap.add_argument("--from", dest="start", type=latlon, metavar="LAT,LON")
    ap.add_argument("--to", dest="goal", type=latlon, metavar="LAT,LON")
    ap.add_argument("--rundir", default=os.environ.get("ROADSENSE_RUNDIR", "/tmp/roadsense"))
    ap.add_argument("--offline", action="store_true", help="run the engine in-process instead of feeding a daemon")
    ap.add_argument("--jsonl", help="write the input messages to this file instead")
    ap.add_argument("--hz", type=float, default=2.0, help="engine steps per second (offline)")
    ap.add_argument("--motion", choices=("yaw", "steer", "wheels", "none"), default="yaw")
    ap.add_argument("--dropout", type=window, action="append", metavar="T0:T1", help="seconds without a fix")
    ap.add_argument("--bias", type=lambda s: tuple(float(x) for x in s.split(":")), metavar="T0:T1:EAST:NORTH",
                    help="shift the GNSS position by EAST/NORTH metres between T0 and T1")
    ap.add_argument("--lateral", type=float, default=0.0, help="drive this many metres right of the centre line")
    ap.add_argument("--hacc", type=float, default=5.0)
    ap.add_argument("--gnss-sigma", type=float, default=3.0)
    ap.add_argument("--no-tunnel-loss", action="store_true", help="keep the fix inside tunnels")
    ap.add_argument("--explicit-none", action="store_true", help='send fix "none" messages while the fix is lost')
    ap.add_argument("--no-course", action="store_true", help="GNSS without course")
    ap.add_argument("--seed", type=int, default=1)
    ap.add_argument("--dump-last", action="store_true", help="print the last map message (offline)")
    a = ap.parse_args()
    if a.jsonl:
        run_jsonl(a)
    elif a.offline:
        run_offline(a)
    else:
        run_live(a)


if __name__ == "__main__":
    main()
