#!/usr/bin/env python3
"""api_profile.py TAP.jsonl [--api DIR] [--profile] [--no-ws] : feed a tapped daemon stream (tap.py) through
the api's State exactly as sockets.py does (json.loads per line, on_message, fan-out to one subscriber that
filters topics like /ws and one recorder) and report the CPU time per input message type and per derived
topic, per second of stream. With --profile: cProfile, top functions by cumulative time."""
import cProfile, json, os, pstats, sys, tempfile, time
api = sys.argv[sys.argv.index("--api") + 1] if "--api" in sys.argv else os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", "api")
sys.path.insert(0, api)
from roadsense_api.state import State
from roadsense_api import recorder as recorder_mod

def load(path):
    return [json.loads(l) for l in open(path)]

def run(rows, ws=True):
    tmp = tempfile.mkdtemp()
    st = State(tmp, datadir=tmp, ring_path=None)
    out = {"n": {}, "bytes": {}}
    if ws:
        def push(mtype, line):
            out["n"][mtype] = out["n"].get(mtype, 0) + 1; out["bytes"][mtype] = out["bytes"].get(mtype, 0) + len(line)
        st.subscribers.add(push)
        rec = recorder_mod.Recorder({"dir": tmp, "max_mb": 500, "frame_every_s": 0})
        st.subscribers.add(rec.on_message)
    per = {}
    for r in rows:
        t0 = time.process_time_ns()
        line = r["m"]
        msg = json.loads(line)
        t1 = time.process_time_ns()
        st.on_message(r["d"], msg, line)
        t2 = time.process_time_ns()
        e = per.setdefault(msg.get("type"), [0, 0, 0, 0]); e[0] += 1; e[1] += t1 - t0; e[2] += t2 - t1; e[3] += len(line)
    return per, out

rows = load(sys.argv[1]); dur = rows[-1]["t"] - rows[0]["t"]
if "--profile" in sys.argv:
    pr = cProfile.Profile(); pr.enable(); run(rows); pr.disable()
    pstats.Stats(pr).sort_stats("cumulative").print_stats(45)
else:
    best = None
    for _ in range(3):
        t0 = time.process_time(); per, out = run(rows, "--no-ws" not in sys.argv); el = time.process_time() - t0
        if best is None or el < best[0]: best = (el, per, out)
    el, per, out = best
    print("stream %.1f s, %d messages, api CPU %.3f s = %.2f %% of one core (this machine)" % (dur, len(rows), el, 100 * el / dur))
    print("%-10s %8s %10s %12s %12s" % ("input", "msg/s", "kB/s", "parse ms/s", "handle ms/s"))
    for k, v in sorted(per.items(), key=lambda kv: -(kv[1][1] + kv[1][2])):
        print("%-10s %8.2f %10.1f %12.2f %12.2f" % (k, v[0] / dur, v[3] / dur / 1e3, v[1] / dur / 1e6, v[2] / dur / 1e6))
    print("fan-out:", {k: (round(n / dur, 2), round(out["bytes"][k] / dur / 1e3, 1)) for k, n in sorted(out["n"].items())})
