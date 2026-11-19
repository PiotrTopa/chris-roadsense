#!/usr/bin/env python3
"""Lane L39: the consolidated side before and after the move, message by message.

    tools/mixer_equiv.py run SESSION --pos POS --out OUT.jsonl.gz [--tree OLD_TREE] [--limit N] [--mapper box|recorded]
    tools/mixer_equiv.py diff A.jsonl.gz B.jsonl.gz [--topics lead,lanes,...]

`run` feeds a recorder session (inputs only: `road`, `calib`, `camera`, `env`, `lights`, `signs`, `map`, `ego`,
`vehicle`, `objects`, `gnss`, `display_lanes`) in recorded order through

    --tree OLD_TREE   that tree's api process state (`roadsense_api.state.State`, the trackers called by it), with
                      its clock replaced by the messages' time
    (no --tree)       this tree's chris-integrator stack (`chris_integrator.stack.build`), on a MessageClock

and writes every outgoing message of the consolidated topics as `[topic, line]`, in order. `diff` compares two
such files topic by topic (count, identical lines, the first difference) and the order of the whole stream.

Both sides get the same arrival clock: a message arrives at its own `ts_mono_ns` (never before the one before
it), the wall clock is the last `ts_real_ns` seen carried forward. The old api read the camera's intrinsics
from the frame ring; in this replay they come from the session's `calib` message on both sides.
"""
import argparse
import gzip
import hashlib
import json
import os
import sys
import time

HERE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
INPUTS = ("road", "calib", "camera", "env", "lights", "signs", "map", "ego", "vehicle", "objects", "gnss", "display_lanes")
TOPICS = ("ego_state", "tracks", "lead", "lanes", "view", "speed_limit", "rules", "fused", "lights", "signs", "map")


class Clock:
    def __init__(self):
        self.t_ns, self.r_ns = 0, 0

    def on_message(self, msg):
        ts, tr = msg.get("ts_mono_ns"), msg.get("ts_real_ns")
        if isinstance(ts, int) and ts > self.t_ns:
            if not isinstance(tr, int) and self.r_ns:
                self.r_ns += ts - self.t_ns
            self.t_ns = ts
        if isinstance(tr, int) and tr > self.r_ns:
            self.r_ns = tr

    # the `time` module as the old state.py uses it
    def monotonic(self):
        return self.t_ns / 1e9

    def monotonic_ns(self):
        return self.t_ns

    def time_ns(self):
        return self.r_ns

    def time(self):
        return self.r_ns / 1e9


def session(a, tree):
    sys.path.insert(0, os.path.join(tree, "tools"))
    import replay_lib as rl
    pos = rl.load_pos(a.pos) if a.pos else None
    n = 0
    for m in rl.read_session(a.session, road_model=a.road_model, pos=pos):
        if m.get("type") not in INPUTS:
            continue
        if m.get("type") == "road":
            n += 1
            if a.limit and n > a.limit:
                return
        yield m


def run_old(a, sink):
    tree = os.path.abspath(os.path.expanduser(a.tree))
    sys.path.insert(0, os.path.join(tree, "api"))
    import roadsense_api.state as S
    clk = Clock()
    S.time = clk
    st = S.State("/nonexistent", datadir=None, ring_path=None)

    def ring():
        c = st.latest.get("calib")
        return {"width": c.get("width"), "height": c.get("height"), "focal_px": c.get("focal_px"), "cx": c.get("cx"),
                "cy": c.get("cy")} if c else None
    st.ring = ring
    st.subscribers.add(sink)
    cpu = 0.0
    for m in session(a, tree):
        clk.on_message(m)
        c = time.process_time()
        st.on_message("replay", m, json.dumps(m, separators=(",", ":")))
        cpu += time.process_time() - c
    return cpu, None


def run_new(a, sink):
    sys.path.insert(0, os.path.join(HERE, "chris-integrator"))
    sys.path.insert(0, os.path.join(HERE, "chris-mapper"))
    from chris_integrator.mixer import MessageClock
    from chris_integrator.stack import build
    cfg = {}
    if a.mapper == "box":
        cfg["mapper"] = {"pack": os.path.expanduser(a.pack)}
    stack = build(cfg=cfg, clock=MessageClock(), external_map=a.mapper == "recorded",
                  mapper="auto" if a.mapper == "box" else None)
    stack.mixer.add_output("equiv", lambda e: sink(e.stream, e.line))
    cpu = 0.0
    for m in session(a, HERE):
        if m.get("type") == "map" and a.mapper != "recorded":
            continue
        c = time.process_time()
        stack.deliver(m)
        cpu += time.process_time() - c
    return cpu, stack.mixer.describe()


def cmd_run(a):
    n = {}
    with gzip.open(a.out, "wt", compresslevel=3) as f:
        def sink(topic, line):
            if topic in TOPICS:
                n[topic] = n.get(topic, 0) + 1
                f.write(json.dumps([topic, line]) + "\n")
        cpu, desc = (run_old if a.tree else run_new)(a, sink)
    roads = n.get("lead", 0)
    print("%s (%s): %s" % (os.path.basename(a.session.rstrip("/")), "old tree" if a.tree else "mixer", n))
    print("cpu %.1f s, %.3f ms per road frame" % (cpu, 1e3 * cpu / max(roads, 1)))
    if desc:
        for p in desc["plugins"]:
            print("  %-13s order %s calls %7d  %.4f ms/call  total %.2f s  errors %d  %s" % (
                p["name"], p["order"], p["calls"], p["ms_per_call"] or 0.0, p["cpu_s"], p["errors"], p["health"].get("state")))
        with open(a.out + ".mixer.json", "w") as f:
            json.dump(desc, f, indent=1)


def load(path):
    with gzip.open(path, "rt") as f:
        for ln in f:
            yield json.loads(ln)


def cmd_diff(a):
    A, B = list(load(a.a)), list(load(a.b))
    topics = a.topics.split(",") if a.topics else sorted({t for t, _ in A} | {t for t, _ in B})
    ok = True
    for t in topics:
        la, lb = [l for x, l in A if x == t], [l for x, l in B if x == t]
        same = sum(1 for x, y in zip(la, lb) if x == y)
        first = next((k for k, (x, y) in enumerate(zip(la, lb)) if x != y), None)
        print("%-12s %7d / %7d messages, identical %7d%s" % (t, len(la), len(lb), same,
                                                             "" if first is None else "  FIRST DIFFERENCE at #%d" % first))
        if first is not None or len(la) != len(lb):
            ok = False
            if first is not None and a.show:
                x, y = json.loads(la[first]), json.loads(lb[first])
                for k in sorted(set(x) | set(y)):
                    if x.get(k) != y.get(k):
                        print("    %s: %s  |  %s" % (k, json.dumps(x.get(k))[:300], json.dumps(y.get(k))[:300]))
    oa, ob = [t for t, _ in A if t in topics], [t for t, _ in B if t in topics]
    print("order of the stream: %s" % ("identical" if oa == ob else "DIFFERENT (first at #%d)" % next(
        (k for k, (x, y) in enumerate(zip(oa, ob)) if x != y), min(len(oa), len(ob)))))
    h = lambda X: hashlib.sha256("\n".join(t + l for t, l in X if t in topics).encode()).hexdigest()[:16]
    print("sha256 %s | %s" % (h(A), h(B)))
    return 0 if ok and oa == ob else 1


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd", required=True)
    p = sub.add_parser("run")
    p.add_argument("session")
    p.add_argument("--pos")
    p.add_argument("--out", required=True)
    p.add_argument("--tree")
    p.add_argument("--limit", type=int, default=0)
    p.add_argument("--road-model", default="vision")
    p.add_argument("--mapper", choices=("recorded", "box", "none"), default="recorded")
    p.add_argument("--pack", default="~/roadsense-maps/packs/poland-20261114.rsmap")
    p = sub.add_parser("diff")
    p.add_argument("a")
    p.add_argument("b")
    p.add_argument("--topics")
    p.add_argument("--show", action="store_true")
    a = ap.parse_args()
    sys.exit(cmd_run(a) if a.cmd == "run" else cmd_diff(a))


if __name__ == "__main__":
    main()
