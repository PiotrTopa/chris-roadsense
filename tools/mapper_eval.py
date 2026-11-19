#!/usr/bin/env python3
"""Lane L39: the map box (chris-mapper behind chris-integrator's ego estimate: position estimated once) against
the legacy mapd engine (its own position filter), on the same inputs.

    tools/mapper_eval.py synth [--demo NAME ...] [--pack PACK] [--lag 2.0]
        simulated drives on the real pack with truth (tools/synth_drive.py's routes): on-route share (the matched
        edge is on the driven route), position error against truth with a fix and in dead reckoning, per system.
        --lag: the receiver's fixes are this old when they arrive (the reference car's: about 2 s); 0 = a
        receiver without delay.
    tools/mapper_eval.py real SESSION ... --pos POS [--pack PACK]
        recorded drives with the car's receiver: every second fix is held out of both systems and compared with
        each system's position (at the fix's stamp minus a lag of 0 / 1 / 2 / 3 s: a fix of this receiver is about
        2 s old at its stamp); map availability, confidence, ambiguity, distance to the matched road, agreement
        of the two on the road; cost.

"legacy" = mapd/roadsense_mapd/engine.py (the engine of today's mapd daemon: GNSS + vehicle motion into its own
filter, default configuration, vision odometry off as in the unit file). "box" = the default chris-integrator
stack with the chris-mapper box in process; vehicle motion, fixes and (real drives) the road model go to the
estimator, the box reads `ego_state`.
"""
import argparse
import bisect
import math
import os
import sys
import time

HERE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
for d in ("chris-integrator", "chris-mapper", "mapd"):
    sys.path.insert(0, os.path.join(HERE, d))

from chris_integrator.io.feed import Feed                      # noqa: E402
from chris_integrator.mixer import MessageClock                # noqa: E402
from chris_integrator.plugins.mapper import Mapper             # noqa: E402
from chris_integrator.replay import load_pos, read_session    # noqa: E402
from chris_integrator.stack import build                       # noqa: E402
from chris_mapper import geo, sim                              # noqa: E402
from chris_mapper.box import MapBox                            # noqa: E402
from chris_mapper.rsmap import Pack                            # noqa: E402
from roadsense_mapd.engine import Engine                       # noqa: E402

NS = 1e9
DEMOS = {
    "warsaw-s2-tunnel": ((52.13889, 20.98064), (52.15258, 21.11298)),
    "warsaw-city": ((52.2297, 21.0122), (52.1930, 21.0340)),
    "krakow-city": ((50.0647, 19.9450), (50.0350, 19.9400)),
    "opole-rural": ((50.6420, 17.6190), (50.4760, 17.3340)),
}


def pct(v, p):
    if not v:
        return float("nan")
    v = sorted(v)
    return v[min(len(v) - 1, int(p / 100.0 * len(v)))]


def stat(v):
    return "-" if not v else "p50 %5.1f  p90 %5.1f  p95 %5.1f m (n %d)" % (pct(v, 50), pct(v, 90), pct(v, 95), len(v))


class Legacy:
    """Today's mapd engine: feeder messages in, `map` at 2 Hz of message time, the filter's position after
    every motion message."""
    name = "legacy mapd"

    def __init__(self, pack):
        self.eng = Engine(pack)
        self.track, self.maps = [], []         # (t, lat, lon), (t, map message, edge ref)
        self.last = None
        self.cpu = 0.0

    def feeder(self, m, ts):
        c = time.process_time()
        self.eng.on_input(m, ts)
        if self.last is None or ts - self.last >= 0.5 * NS:
            self.last = ts
            out = self.eng.step(ts)
            self.maps.append((ts / NS, out, self.eng.match.edge.ref if out["available"] else None))
        self.cpu += time.process_time() - c
        f = self.eng.filter
        if m.get("type") == "motion" and f.initialised and self.eng.state in ("good", "dead_reckoning"):
            self.track.append((ts / NS,) + tuple(f.latlon))

    def session(self, m):
        t = m.get("type")
        if t == "ego":
            mo = {"type": "motion", "ts_mono_ns": m["ts_mono_ns"], "speed_mps": abs(m.get("speed_mps") or 0.0),
                  "source": m.get("tag") or "recorded"}
            for k in ("gear", "yaw_rate_rps", "wheel_speeds_mps"):
                if m.get(k) is not None:
                    mo[k] = m[k]
            if m.get("reverse"):
                mo["reverse"] = True
            self.feeder(mo, m["ts_mono_ns"])
        elif t == "gnss":
            self.feeder({k: v for k, v in m.items() if not k.startswith("_")}, m["ts_mono_ns"])


class Box:
    """The chris stack with the map box: position from the estimator only."""
    name = "chris: estimator + map box"

    def __init__(self, pack):
        self.box = MapBox(pack=pack)
        self.stack = build(mapper=Mapper(self.box), clock=MessageClock())
        self.feed = Feed()
        self.track, self.maps = [], []
        self.cpu = 0.0
        self.es = None
        self.stack.mixer.add_output("eval", self._out, streams=("ego_state", "map"))

    def _out(self, e):
        m = e.msg
        if e.stream == "map":
            self.maps.append((m["ts_mono_ns"] / NS, m, self.box.match.edge.ref if m["available"] else None))
        else:
            self.es = m
            p = m["position"]
            if p["valid"]:
                self.track.append((m["ts_mono_ns"] / NS, p["lat"], p["lon"]))

    def feeder(self, m, ts):
        c = time.process_time()
        for im in self.feed.convert(m, now_ns=ts):
            self.stack.deliver(im)
        self.cpu += time.process_time() - c

    def session(self, m):
        if m.get("type") in ("map", "ego_state", "tracks", "lead", "lanes", "view", "fused", "rules", "speed_limit"):
            return
        c = time.process_time()
        self.stack.deliver({k: v for k, v in m.items() if not k.startswith("_")} if m.get("type") == "gnss" else m)
        self.cpu += time.process_time() - c

    def mapper_cost(self):
        p = next(p for p in self.stack.mixer.describe()["plugins"] if p["name"] == "mapper")
        return p["cpu_s"], p["calls"]


def at(track, t, max_gap=1.0):
    """Position of a track at time t (linear between neighbours), or None."""
    k = bisect.bisect_left(track, (t,))
    if k == 0 or k >= len(track):
        return None
    a, b = track[k - 1], track[k]
    if b[0] - a[0] > max_gap:
        return None
    w = (t - a[0]) / (b[0] - a[0]) if b[0] > a[0] else 0.0
    return a[1] + w * (b[1] - a[1]), a[2] + w * (b[2] - a[2])


# ---- simulated drives with truth ------------------------------------------------------------------------------
def cmd_synth(a):
    pack = Pack(os.path.expanduser(a.pack), cache_tiles=600)
    for demo in a.demo or sorted(DEMOS):
        path = sim.route(pack, *DEMOS[demo])
        on_path = {e.ref for e, _, _, _ in path}
        km = sum(abs(s1 - s0) for _, _, s0, s1 in path) / 1000.0
        print("== %s: %d edges, %.1f km; receiver lag %.1f s" % (demo, len(path), km, a.lag))
        for cls in (Legacy, Box):
            s = cls(pack)
            truth = []
            late = []
            for ts, m, o in sim.feed(sim.Drive(pack, path), t0_ns=10 ** 12, seed=a.seed):
                if m is None:
                    continue
                if m["type"] == "gnss" and a.lag > 0:
                    late.append(dict(m, ts_mono_ns=ts + int(a.lag * NS)))
                    continue
                while late and late[0]["ts_mono_ns"] <= ts:
                    g = late.pop(0)
                    s.feeder(g, g["ts_mono_ns"])
                s.feeder(m, ts)
                if m["type"] == "motion":
                    truth.append((ts / NS, o.lat, o.lon, o.tunnel))
            tt = [x[0] for x in truth]
            errs, dr = [], []
            n = ok = avail = tun = tun_ok = 0
            states = {}
            for t, out, ref in s.maps:
                k = min(len(truth) - 1, bisect.bisect_left(tt, t))
                if t - truth[0][0] < 20.0:
                    continue
                n += 1
                states[out["state"]] = states.get(out["state"], 0) + 1
                pos = out["position"]
                if pos:
                    (dr if out["state"] == "dead_reckoning" else errs).append(geo.dist_m(pos["lat"], pos["lon"], truth[k][1], truth[k][2]))
                if truth[k][3]:
                    tun += 1
                    tun_ok += bool(out["available"] and ref in on_path)
                if out["available"]:
                    avail += 1
                    ok += ref in on_path
            dur = truth[-1][0] - truth[0][0]
            print("  %-28s map available %5.1f %%  on route %5.1f %% of available (%5.1f %% of all steps)  in tunnels on route %s" % (
                s.name, 100.0 * avail / max(n, 1), 100.0 * ok / max(avail, 1), 100.0 * ok / max(n, 1),
                "-" if not tun else "%.1f %% (n %d)" % (100.0 * tun_ok / tun, tun)))
            print("  %-28s position error with a fix: %s" % ("", stat(errs)))
            print("  %-28s position error dead reckoning: %s   states %s" % ("", stat(dr), states))
            print("  %-28s cpu %.2f %% of one core%s" % ("", 100.0 * s.cpu / dur, "" if cls is Legacy else
                                                       " (the box alone: %.3f ms per pose, %.2f %%)" % (
                                                           1e3 * s.mapper_cost()[0] / max(1, s.mapper_cost()[1]), 100.0 * s.mapper_cost()[0] / dur)))


# ---- recorded drives ---------------------------------------------------------------------------------------------
def cmd_real(a):
    pos = load_pos(a.pos)
    for sess in a.sessions:
        pack = Pack(os.path.expanduser(a.pack), cache_tiles=600)
        systems = [Legacy(pack), Box(pack)]
        held, k_g = [], 0
        veh = []                               # (t, speed)
        t0 = t1 = None
        for m in read_session(sess, road_model="vision", pos=pos):
            ts = m.get("ts_mono_ns")
            if not isinstance(ts, int):
                continue
            t0 = ts if t0 is None else t0
            t1 = ts
            if m.get("type") == "ego":
                veh.append((ts / NS, abs(m.get("speed_mps") or 0.0)))
            if m.get("type") == "gnss" and m.get("fix") in ("2d", "3d"):
                k_g += 1
                if k_g % 2 == 0:               # held out of both systems
                    held.append((ts / NS, m["lat"], m["lon"], m.get("hacc_m")))
                    continue
            for s in systems:
                s.session(m)
        vt = [x[0] for x in veh]

        def speed(t):
            k = bisect.bisect_left(vt, t)
            return veh[min(k, len(veh) - 1)][1] if veh else 0.0
        dur = (t1 - t0) / NS
        print("== %s: %.0f s, %d fixes (%d held out), vehicle samples %d" % (os.path.basename(sess.rstrip("/")), dur, k_g, len(held), len(veh)))
        for s in systems:
            mv = [(t, out, ref) for t, out, ref in s.maps if speed(t) >= 3.0]
            av = [x for x in mv if x[1]["available"]]
            conf = [x[1]["match"]["confidence"] for x in av]
            dist = [x[1]["match"]["distance_m"] for x in av]
            amb = sum(1 for x in av if x[1]["match"]["ambiguous"])
            svc = sum(1 for x in av if x[1]["road"]["class"] in ("service", "track"))
            states = {}
            for _, out, _ in mv:
                states[out["state"]] = states.get(out["state"], 0) + 1
            sw = sum(1 for x, y in zip(av, av[1:]) if x[2] != y[2] and (x[1]["road"].get("name"), x[1]["road"].get("ref")) != (y[1]["road"].get("name"), y[1]["road"].get("ref")))
            print("  %-28s moving steps %d: map available %5.1f %%  confidence mean %.2f  ambiguous %4.1f %%  on service/track %4.1f %%  "
                  "distance to the road %s  road changes %d  states %s" % (
                      s.name, len(mv), 100.0 * len(av) / max(len(mv), 1), sum(conf) / max(len(conf), 1), 100.0 * amb / max(len(av), 1),
                      100.0 * svc / max(len(av), 1), "p50 %.1f p90 %.1f m" % (pct(dist, 50), pct(dist, 90)), sw, states))
            for lag in (0.0, 1.0, 2.0, 3.0):
                d = []
                for t, lat, lon, _ in held:
                    if speed(t) < 3.0:
                        continue
                    p = at(s.track, t - lag)
                    if p is not None:
                        d.append(geo.dist_m(lat, lon, p[0], p[1]))
                print("  %-28s held-out fixes against the position %.0f s before their stamp: %s" % ("", lag, stat(d)))
            extra = ""
            if isinstance(s, Box):
                c, n = s.mapper_cost()
                tau = (s.es or {}).get("calibration", {}).get("gnss", {}).get("tau", {})
                extra = "; the box alone %.3f ms per pose = %.2f %% of one core; receiver delay state %.2f +- %.2f s" % (
                    1e3 * c / max(n, 1), 100.0 * c / dur, tau.get("value", float("nan")), tau.get("std", float("nan")))
            print("  %-28s cpu %.2f %% of one core%s" % ("", 100.0 * s.cpu / dur, extra))
        # agreement of the two on the road, at the legacy system's steps
        L, B = systems
        bt = [x[0] for x in B.maps]
        both = same = same_name = only_l = only_b = 0
        for t, out, ref in L.maps:
            if speed(t) < 3.0 or not bt:
                continue
            k = min(len(bt) - 1, bisect.bisect_left(bt, t))
            if abs(bt[k] - t) > 0.6:
                continue
            ob, rb = B.maps[k][1], B.maps[k][2]
            if out["available"] and ob["available"]:
                both += 1
                same += ref == rb
                same_name += (out["road"].get("name"), out["road"].get("ref"), out["road"]["class"]) == (ob["road"].get("name"), ob["road"].get("ref"), ob["road"]["class"])
            elif out["available"]:
                only_l += 1
            elif ob["available"]:
                only_b += 1
        print("  agreement while moving: both have a road in %d steps: same edge %.1f %%, same road (name, ref, class) %.1f %%; "
              "only legacy %d, only the box %d" % (both, 100.0 * same / max(both, 1), 100.0 * same_name / max(both, 1), only_l, only_b))


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd", required=True)
    p = sub.add_parser("synth")
    p.add_argument("--demo", action="append")
    p.add_argument("--lag", type=float, default=2.0)
    p.add_argument("--seed", type=int, default=1)
    p.add_argument("--pack", default="~/roadsense-maps/packs/poland-20261114.rsmap")
    p = sub.add_parser("real")
    p.add_argument("sessions", nargs="+")
    p.add_argument("--pos", required=True)
    p.add_argument("--pack", default="~/roadsense-maps/packs/poland-20261114.rsmap")
    a = ap.parse_args()
    (cmd_synth if a.cmd == "synth" else cmd_real)(a)


if __name__ == "__main__":
    main()
