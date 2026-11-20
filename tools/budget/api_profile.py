#!/usr/bin/env python3
"""api_profile.py INPUT [--pos POS] [--tree DIR] [--mapper recorded|box|none] [--pack PACK] [--limit N] [--profile] [--no-ws]

The CPU of the consolidated side (since lane L39: chris-integrator's mixer, `chris_integrator.stack.build`) per
input message type, per plug-in and for the fan-out, per second of stream. The input goes through the stack as
the daemon's channels deliver it (json.loads per line, `stack.deliver`), with the output channels the daemon
has: the consolidated API (`io.api.Api`, one subscriber that counts lines like a /ws client) and the recorder
(into a temporary directory). With --no-ws: neither output channel. With --profile: cProfile, top functions by
cumulative time.

INPUT is either
  a tapped daemon stream (tools/budget/tap.py: lines {"t", "d", "m"}; the phone's stream as it was), or
  a recorder session (a directory or its messages.jsonl.gz; the recorded inputs, in recorded order;
                      --pos adds the car's GNSS fixes of sessions recorded without them).

Before L39 this tool fed `roadsense_api.state.State` (the api process). That state is gone; its numbers are
reproduced by the old tool on an old tree: `git show v2-l38:tools/budget/api_profile.py`.
"""
import argparse
import cProfile
import json
import os
import pstats
import sys
import tempfile
import time

HERE = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
INPUTS = ("road", "calib", "camera", "env", "lights", "signs", "map", "ego", "vehicle", "objects", "gnss",
          "display_lanes", "governor")


def load(a):
    """[(t_s, line)] in stream order."""
    path = a.input
    if os.path.isfile(path) and not path.endswith(".gz"):
        with open(path) as f:
            first = f.readline()
        try:
            r = json.loads(first)
        except ValueError:
            r = None
        if isinstance(r, dict) and "m" in r and "t" in r:              # a tap.py stream
            with open(path) as f:
                return [(r["t"], r["m"]) for r in map(json.loads, f)]
    from chris_integrator.replay import load_pos, read_session
    pos = load_pos(a.pos) if a.pos else None
    rows, n = [], 0
    for m in read_session(path, road_model="vision", pos=pos):
        if m.get("type") not in INPUTS or not isinstance(m.get("ts_mono_ns"), int):
            continue
        if m["type"] == "road":
            n += 1
            if a.limit and n > a.limit:
                break
        m.pop("_wall", None)
        rows.append((m["ts_mono_ns"] / 1e9, json.dumps(m, separators=(",", ":"))))
    return rows


def run(a, rows, ws=True):
    from chris_integrator.io.api import Api
    from chris_integrator.io.recorder import Recorder
    from chris_integrator.mixer import MessageClock
    from chris_integrator.stack import build
    tmp = tempfile.mkdtemp(prefix="api_profile.")
    cfg, mapper = {}, None
    if a.mapper == "box":
        cfg["mapper"] = {"pack": os.path.expanduser(a.pack)}
        mapper = "auto"
    stack = build(cfg=cfg, clock=MessageClock(), external_map=a.mapper == "recorded", mapper=mapper)
    out = {"n": {}, "bytes": {}}
    if ws:
        api = Api(stack, datadir=None)

        def push(stream, line):
            out["n"][stream] = out["n"].get(stream, 0) + 1
            out["bytes"][stream] = out["bytes"].get(stream, 0) + len(line)
        api.subscribers.add(push)
        rec = Recorder({"dir": tmp, "max_mb": 500, "frame_every_s": 0})
        stack.mixer.add_output("recorder", rec.on_envelope)
    per = {}
    for _, line in rows:
        t0 = time.process_time_ns()
        msg = json.loads(line)
        t1 = time.process_time_ns()
        if msg.get("type") == "map" and a.mapper != "recorded":
            continue
        stack.deliver(msg, line)
        t2 = time.process_time_ns()
        e = per.setdefault(msg.get("type"), [0, 0, 0, 0])
        e[0] += 1
        e[1] += t1 - t0
        e[2] += t2 - t1
        e[3] += len(line)
    desc = stack.mixer.describe()
    stack.mixer.close()
    if ws:
        rec.close()
    return per, out, desc


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("input")
    ap.add_argument("--pos", help="the car's GNSS trip log (chris_integrator.replay.load_pos) for a recorder session")
    ap.add_argument("--tree", default=HERE, help="a roadsense tree whose chris-integrator / chris-mapper are profiled (default: this one)")
    ap.add_argument("--mapper", choices=("recorded", "box", "none"), default="recorded",
                    help="`map` from the input itself (as the phone's mapd sent it), from the map box in process, or none")
    ap.add_argument("--pack", default="~/roadsense-maps/packs/poland-20261114-f2.rsmap")
    ap.add_argument("--limit", type=int, default=0, help="road frames of a session (0: all)")
    ap.add_argument("--profile", action="store_true")
    ap.add_argument("--no-ws", action="store_true")
    ap.add_argument("--repeat", type=int, default=3, help="runs; the fastest is reported")
    a = ap.parse_args()
    tree = os.path.abspath(os.path.expanduser(a.tree))
    sys.path[:0] = [os.path.join(tree, "chris-integrator"), os.path.join(tree, "chris-mapper")]
    rows = load(a)
    if not rows:
        sys.exit("no input messages in %s" % a.input)
    dur = max(rows[-1][0] - rows[0][0], 1e-9)
    if a.profile:
        pr = cProfile.Profile()
        pr.enable()
        run(a, rows, not a.no_ws)
        pr.disable()
        pstats.Stats(pr).sort_stats("cumulative").print_stats(45)
        return
    best = None
    for _ in range(max(a.repeat, 1)):
        t0 = time.process_time()
        per, out, desc = run(a, rows, not a.no_ws)
        el = time.process_time() - t0
        if best is None or el < best[0]:
            best = (el, per, out, desc)
    el, per, out, desc = best
    print("stream %.1f s, %d messages, consolidated side CPU %.3f s = %.2f %% of one core (this machine)" % (
        dur, len(rows), el, 100 * el / dur))
    print("%-13s %8s %10s %12s %12s" % ("input", "msg/s", "kB/s", "parse ms/s", "handle ms/s"))
    for k, v in sorted(per.items(), key=lambda kv: -(kv[1][1] + kv[1][2])):
        print("%-13s %8.2f %10.1f %12.2f %12.2f" % (k, v[0] / dur, v[3] / dur / 1e3, v[1] / dur / 1e6, v[2] / dur / 1e6))
    print("%-13s %8s %12s %10s" % ("plug-in", "calls/s", "ms/call", "ms/s"))
    for p in desc["plugins"]:
        print("%-13s %8.2f %12.4f %10.2f" % (p["name"], p["calls"] / dur, p["ms_per_call"] or 0.0, 1e3 * p["cpu_s"] / dur))
    if out["n"]:
        print("fan-out (msg/s, kB/s):", {k: (round(n / dur, 2), round(out["bytes"][k] / dur / 1e3, 1))
                                         for k, n in sorted(out["n"].items())})


if __name__ == "__main__":
    main()
