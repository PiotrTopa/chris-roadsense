#!/usr/bin/env python3
"""The daemon's whole per-message path, off line and repeatable: cost by stage, and what it puts out.

    tools/cost_bench.py run INPUT [--tree DIR] [--ws TOPICS|none] [--no-recorder] [--limit N] [--repeat K]
                                  [--dump OUT.jsonl.gz] [--profile OUT.prof] [--json OUT.json] [--pack PACK]
    tools/cost_bench.py diff A.jsonl.gz B.jsonl.gz [--show N]

INPUT is a capture of tools/cost_capture.py (`*.cap.gz`: the raw lines a running integrator was fed, as the
daemons wrote them) or a recorder session directory (`messages.jsonl.gz`; its input topics only; `road` is
the recorder's reduced form there, so parsing costs less than on the device).

`run` builds the daemon object of the tree (`chris_integrator.daemon.Daemon`: the stack, the socket output,
the recorder writing into a scratch directory, the API's output channel with one WebSocket subscriber that
asks for TOPICS, default: what the car backend asks for) and pushes every line through what the daemon does
with it: decode + parse (the socket reader), the mixer with its plug-ins, the output channels. No event loop,
no sockets: what is left out is the kernel's part and asyncio's stream reader / aiohttp's frame writer.
The bus runs on the messages' clock, so two runs of one tree give identical output.

    stage `parse`     bytes -> message (io.sockets.Reader)
    stage `plugins`   each plug-in's own time, the output channels' time taken out
    stage `outputs`   each output channel (socket, recorder, api / WebSocket), serialisation included
    stage `mixer`     dispatch: the rest of Daemon.on_message

`--dump` writes every message of every stream a plug-in produced as `[stream, message]` (in order; cost is
not measured in that run), the recorder's file as `["recorder", line]` and what the socket and the WebSocket
subscriber received as digests per topic; `diff` compares two dumps: per stream the count, the identical
lines, and the largest relative difference of a number where lines differ.

`--tree DIR` runs another checkout's chris-integrator (DIR/chris-integrator, DIR/chris-mapper) with this
harness: the comparison before / after of a change. Standard library + the tree's own needs (numpy)."""
import argparse
import cProfile
import gzip
import hashlib
import json
import os
import shutil
import sys
import tempfile
import time
import zlib

HERE = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
INPUTS = ("road", "calib", "camera", "env", "lights", "signs", "ego", "vehicle", "objects", "gnss", "display_lanes",
          "governor", "road_users")
BACKEND_WS = "road,lights,speed_limit,env,rules,fused,map,lead,lanes,view"


def read_input(path, limit=0):
    """[(socket name, raw line bytes)] in arrival order."""
    out = []
    try:                                   # a session cut by the power going off ends in a broken gzip block
        if os.path.isdir(path):
            with gzip.open(os.path.join(path, "messages.jsonl.gz"), "rb") as f:
                for ln in f:
                    ln = ln.strip()
                    if not ln or not ln.endswith(b"}"):
                        continue
                    i = ln.find(b'"type":"')
                    t = ln[i + 8:ln.find(b'"', i + 8)].decode() if i >= 0 else ""
                    if t in INPUTS:
                        out.append(("feed" if t in ("ego", "vehicle", "gnss", "objects") else "session", ln))
                        if limit and len(out) >= limit:
                            break
            return out
        with gzip.open(path, "rb") as f:
            for ln in f:
                p = ln.rstrip(b"\n").split(b"\t", 2)
                if len(p) == 3:
                    out.append((p[1].decode(), p[2]))
                    if limit and len(out) >= limit:
                        break
    except (EOFError, zlib.error):
        pass
    return out


class Timed:
    """An output channel's function with its time counted, and taken out of the enclosing plug-in's own time."""

    def __init__(self, fn, mixer):
        self.fn, self.mixer, self.t, self.n, self.by = fn, mixer, 0.0, 0, {}

    def __call__(self, env):
        t0 = time.perf_counter()
        try:
            self.fn(env)
        finally:
            dt = time.perf_counter() - t0
            self.t += dt
            self.n += 1
            self.by[env.stream] = self.by.get(env.stream, 0.0) + dt
            self.mixer._child[-1] += dt


def make(a, tmp):
    tree = os.path.abspath(a.tree) if a.tree else HERE
    sys.path.insert(0, os.path.join(tree, "chris-mapper"))
    sys.path.insert(0, os.path.join(tree, "chris-integrator"))
    from chris_integrator import daemon as daemon_mod
    from chris_integrator.io.api import Api
    from chris_integrator.mixer import MessageClock
    clock = MessageClock()
    build = daemon_mod.build
    daemon_mod.build = lambda **kw: build(clock=clock, **kw)
    cfgdir, datadir, rundir = (os.path.join(tmp, d) for d in ("etc", "var", "run"))
    for d in (cfgdir, datadir, rundir):
        os.makedirs(d)
    if not a.no_recorder:
        with open(os.path.join(cfgdir, "recorder.json"), "w") as f:
            json.dump({"dir": os.path.join(tmp, "rec"), "max_mb": 4000, "frame_every_s": 1.0}, f)
        os.makedirs(os.path.join(tmp, "rec"))
    mapper = None
    if a.pack:
        with open(os.path.join(cfgdir, "mapper.json"), "w") as f:
            json.dump({"pack": os.path.expanduser(a.pack), "horizon_m": 1500, "country": "pl"}, f)
        mapper = "auto"
    d = daemon_mod.Daemon(rundir, [], None, datadir, cfgdir, None, None, mapper=mapper)
    if d.recorder is not None:
        d.recorder.clock = clock.now
        d.recorder.wall = lambda: clock.real_ns() / 1e9 or 1.7e9
    api = Api(d.stack, {}, datadir, d.feed_messages, d.recorder)
    return tree, d, api, clock


def parser(tree, stack):
    """The socket reader's bytes -> (message, line) of the tree under test."""
    from chris_integrator.io import sockets
    f = getattr(sockets, "parse_line", None)
    if f is not None:
        if getattr(stack, "trim_road", False):     # as the daemon wires its readers (default boxes only)
            f = sockets.parse_line_trimmed

        def parse_new(raw):
            try:
                return f(raw)
            except ValueError:
                return None, None
        return parse_new

    def parse(raw):                        # io.sockets.Reader.run as of 6054373
        line = raw.decode("utf-8", "replace").strip()
        if not line:
            return None, None
        try:
            msg = json.loads(line)
        except ValueError:
            return None, None
        return (msg, line) if isinstance(msg, dict) else (None, None)
    return parse


def cmd_run(a):
    lines = read_input(a.input, a.limit)
    tmp = tempfile.mkdtemp(prefix="costbench-")
    try:
        return _run(a, lines, tmp)
    finally:
        shutil.rmtree(tmp, ignore_errors=True)


def _run(a, lines, tmp):
    tree, d, api, clock = make(a, tmp)
    parse = parser(tree, d.stack)
    m = d.mixer
    ws_topics = None if a.ws == "none" else {t for t in a.ws.split(",") if t}
    ws = {"n": 0, "bytes": 0, "sha": {}}
    sock = {"sha": {}}
    dump = gzip.open(a.dump, "wt", compresslevel=3) if a.dump else None

    def digest(store, stream, data):
        h = store["sha"].get(stream)
        if h is None:
            h = store["sha"][stream] = [hashlib.sha256(), 0]
        h[0].update(data if isinstance(data, bytes) else data.encode())
        h[1] += 1

    if ws_topics is not None:
        def push(mtype, line):
            if ws_topics and mtype not in ws_topics:
                return
            ws["n"] += 1
            ws["bytes"] += len(line)
            if dump is not None:
                digest(ws, mtype, line)
        if hasattr(api, "subscribe"):          # the subscriber says which topics it wants
            api.subscribe(push, ws_topics or None)
        else:
            api.subscribers.add(push)
    if dump is not None:
        made = {n for n, s in m.bus.streams.items() if s.producer in m.slots}
        m.add_output("dump", lambda e: dump.write(json.dumps([e.stream, e.msg], separators=(",", ":")) + "\n"), streams=made)
    else:
        for name, (fn, streams, info) in list(m.outputs.items()):
            m.outputs[name] = (Timed(fn, m), streams, info)
        m._taps = list(m.outputs.values())
        if hasattr(m, "_tap_of"):
            m._tap_of = {}
    import gc
    if a.gc == "off":
        gc.disable()
    elif a.gc == "freeze":
        gc.collect()
        gc.freeze()
    prof = cProfile.Profile() if a.profile else None
    t_parse = t_mix = 0.0
    n_road = n_in = 0
    worst = []
    cpu0 = time.process_time()
    os0 = os.times()
    w0 = time.perf_counter()
    if prof:
        prof.enable()
    pc = time.perf_counter
    for _ in range(a.repeat):
        for name, raw in lines:
            t0 = pc()
            msg, line = parse(raw)
            t1 = pc()
            t_parse += t1 - t0
            if msg is None:
                continue
            n_in += 1
            if msg.get("type") == "road":
                n_road += 1
            if name == "feed":                 # the input store hands the mixer objects, no line
                d.on_message(msg)
            else:
                d.on_message(msg, line)
            dt = pc() - t1
            t_mix += dt
            if msg.get("type") == "road":
                worst.append(dt)
    if prof:
        prof.disable()
        prof.dump_stats(a.profile)
    wall = time.perf_counter() - w0
    cpu = time.process_time() - cpu0
    os1 = os.times()
    if d.recorder is not None:
        d.recorder.close()
    res = {"tree": tree, "input": a.input, "messages": n_in, "road_frames": n_road, "wall_s": round(wall, 3),
           "cpu_s": round(cpu, 3), "cpu_user_s": round(os1.user - os0.user, 2), "cpu_sys_s": round(os1.system - os0.system, 2),
           "ms_per_road_frame": round(1e3 * cpu / max(1, n_road), 3)}
    if dump is None:
        per = 1e3 / max(1, n_road)
        plug = {s.name: {"calls": s.calls, "ms_per_call": round(1e3 * s.cpu / s.calls, 4) if s.calls else None,
                         "ms_per_road_frame": round(s.cpu * per, 4), "max_ms": round(s.max_ms, 2)}
                for s in m.slots.values()}
        outs = {n: {"ms_per_road_frame": round(fn.t * per, 4),
                    "by_stream": {k: round(v * per, 4) for k, v in sorted(fn.by.items(), key=lambda kv: -kv[1])[:8]}}
                for n, (fn, _, _) in m.outputs.items() if isinstance(fn, Timed)}
        t_plug = sum(s.cpu for s in m.slots.values())
        t_out = sum(fn.t for fn, _, _ in m.outputs.values() if isinstance(fn, Timed))
        worst.sort()
        res.update(stages_ms_per_road_frame={"parse": round(t_parse * per, 4), "plugins": round(t_plug * per, 4),
                                             "outputs": round(t_out * per, 4),
                                             "mixer": round((t_mix - t_plug - t_out) * per, 4),
                                             "harness": round((wall - t_parse - t_mix) * per, 4)},
                   plugins=plug, outputs=outs,
                   road_frame_ms={"p50": round(1e3 * worst[len(worst) // 2], 2) if worst else None,
                                  "p99": round(1e3 * worst[int(len(worst) * 0.99)], 2) if worst else None,
                                  "max": round(1e3 * worst[-1], 2) if worst else None},
                   ekf_reruns=getattr(getattr(d.integ, "ekf", None), "reruns", None),
                   ws={"messages": ws["n"], "bytes": ws["bytes"]},
                   gc={"mode": a.gc, "threshold": gc.get_threshold(), "collections": [g["collections"] for g in gc.get_stats()]})
    else:
        rec = os.path.join(tmp, "rec")
        for s in sorted(os.listdir(rec)) if os.path.isdir(rec) else ():
            p = os.path.join(rec, s, "messages.jsonl.gz")
            if os.path.exists(p):
                with gzip.open(p, "rt") as f:
                    for ln in f:
                        dump.write(json.dumps(["recorder", ln.rstrip("\n")]) + "\n")
        for k, v in sorted(d.out.last.items()):
            v = v if isinstance(v, bytes) else v.line.encode() + b"\n"      # bytes, or the envelope it is made from
            dump.write(json.dumps(["socket.last", k, hashlib.sha256(v).hexdigest()]) + "\n")
        for k, (h, n) in sorted(ws["sha"].items()):
            dump.write(json.dumps(["ws", k, n, h.hexdigest()]) + "\n")
        dump.close()
        with gzip.open(a.dump, "rb") as f:
            res["dump_sha256"] = hashlib.sha256(f.read()).hexdigest()
    if a.json:
        with open(a.json, "w") as f:
            json.dump(res, f, indent=1)
    print(json.dumps(res, indent=1))


def _leaves(o, pre=""):
    if isinstance(o, dict):
        for k, v in o.items():
            yield from _leaves(v, pre + "." + str(k))
    elif isinstance(o, list):
        for i, v in enumerate(o):
            yield from _leaves(v, pre + "[%d]" % i)
    else:
        yield pre, o


def cmd_diff(a):
    def load(p):
        by = {}
        with gzip.open(p, "rt") as f:
            for ln in f:
                o = json.loads(ln)
                by.setdefault(o[0], []).append(ln)
        return by
    A, B = load(a.a), load(a.b)
    same_all = True
    for s in sorted(set(A) | set(B)):
        x, y = A.get(s, []), B.get(s, [])
        same = sum(1 for p, q in zip(x, y) if p == q)
        worst, where, other, shown = 0.0, None, 0, 0
        if same != max(len(x), len(y)):
            same_all = False
            for p, q in zip(x, y):
                if p == q:
                    continue
                lp, lq = dict(_leaves(json.loads(p))), dict(_leaves(json.loads(q)))
                for k in set(lp) | set(lq):
                    u, v = lp.get(k), lq.get(k)
                    if u == v:
                        continue
                    if isinstance(u, (int, float)) and isinstance(v, (int, float)) and not isinstance(u, bool) and not isinstance(v, bool):
                        r = abs(u - v) / max(abs(u), abs(v), 1e-300)
                        if r > worst:
                            worst, where = r, "%s: %r / %r" % (k, u, v)
                    else:
                        other += 1
                        if shown < a.show:
                            shown += 1
                            print("    %s %s: %r / %r" % (s, k, u, v))
        print("%-16s %7d / %7d  identical %7d%s" % (s, len(x), len(y), same,
              "" if same == max(len(x), len(y)) else "   largest relative difference %.3g (%s), other differences %d" % (worst, where, other)))
    print("IDENTICAL" if same_all else "DIFFERENT")
    return 0 if same_all else 1


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd", required=True)
    p = sub.add_parser("run")
    p.add_argument("input")
    p.add_argument("--tree")
    p.add_argument("--ws", default=BACKEND_WS)
    p.add_argument("--no-recorder", action="store_true")
    p.add_argument("--limit", type=int, default=0)
    p.add_argument("--repeat", type=int, default=1)
    p.add_argument("--dump")
    p.add_argument("--profile")
    p.add_argument("--json")
    p.add_argument("--pack")
    p.add_argument("--gc", choices=("default", "off", "freeze"), default="default",
                   help="an experiment: the collector off, or everything built at start frozen (gc.freeze)")
    p = sub.add_parser("diff")
    p.add_argument("a")
    p.add_argument("b")
    p.add_argument("--show", type=int, default=5)
    a = ap.parse_args()
    sys.exit(cmd_run(a) if a.cmd == "run" else cmd_diff(a))


if __name__ == "__main__":
    main()
