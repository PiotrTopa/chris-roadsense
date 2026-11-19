#!/usr/bin/env python3
"""Stage 1 of the chain trace: a `road` stream (modeld_replay, or a recorder session: its `road` and `calib`
messages) through the consolidated side, any version: what the consumers' /ws receives.
  usage: chain_api.py ROAD.jsonl.gz|SESSION OUT.jsonl.gz [--api API_DIR | --tree TREE] [--t0 S --t1 S]
         [--ego] [--ego-yaw-bias RAD_S] [--ego-yaw-per-speed RAD_S_PER_MPS]
         --ego: a host `ego` message per road frame (speed = road.v_ego, yaw rate = pose.yaw_rate + bias)
Every output line gets "_rx_s" (frame time + 0.08 s). The clock is the stream's time.

The consolidated side is chris-integrator's stack (`chris_integrator.stack.build`, lane L39) of TREE (default
this tree). `--api API_DIR` keeps the old calling convention of chain.sh: an `api` directory of a tree from
before L39 (whose `roadsense_api.state.State` still made the consolidated topics) runs that State, as this tool
did then; an `api` directory of a later tree selects that tree's chris-integrator.
A truncated gzip (a recording cut by a power loss) yields what is complete."""
import argparse
import gzip
import json
import os
import sys
import time

HERE = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))
ap = argparse.ArgumentParser()
ap.add_argument("road"); ap.add_argument("out")
ap.add_argument("--api", help="api directory of a tree (see the header)")
ap.add_argument("--tree", help="a roadsense tree with chris-integrator (default: this one)")
ap.add_argument("--t0", type=float, default=0); ap.add_argument("--t1", type=float, default=1e9)
ap.add_argument("--ego", action="store_true"); ap.add_argument("--ego-yaw-bias", type=float, default=0.0)
ap.add_argument("--ego-yaw-per-speed", type=float, default=0.0, help="yaw-rate error proportional to speed, rad/s per m/s (a steering-angle zero offset)")
a = ap.parse_args()

tree = os.path.abspath(os.path.expanduser(a.tree or (os.path.dirname(a.api.rstrip("/")) if a.api else HERE)))
old = a.api is not None and not os.path.isdir(os.path.join(tree, "chris-integrator", "chris_integrator", "mixer"))
sys.path.insert(0, os.path.join(HERE, "chris-integrator"))
from chris_integrator.replay import read_lines  # noqa: E402  (a reader only; the same in every version)

clock = [0.0]
seen = set()
out = gzip.open(a.out, "wt")


def cb(topic, line):
    line = line.rstrip()
    seen.add(topic)
    out.write(line[:-1] + ',"_rx_s":%.4f}\n' % (clock[0] + 0.08))


if old:                                             # a tree before L39: its api State made the consolidated topics
    sys.path.insert(0, a.api)
    import roadsense_api.state as S

    class _T:
        def __getattr__(self, k): return getattr(time, k)
        def monotonic(self): return clock[0]
        def monotonic_ns(self): return int(clock[0] * 1e9)
        def time(self): return clock[0]
        def time_ns(self): return int(clock[0] * 1e9)
    S.time = _T()
    st = S.State("/nonexistent")
    st.subscribers.add(cb)

    def deliver(daemon, m, line):
        st.on_message(daemon, m, line)
        if m.get("type") in ("road", "calib") and m["type"] not in seen:   # a State that does not pass modeld's own lines on
            cb(m["type"], line)
            seen.discard(m["type"])
else:
    sys.path[:0] = [os.path.join(tree, "chris-integrator"), os.path.join(tree, "chris-mapper")]
    for k in [k for k in sys.modules if k == "chris_integrator" or k.startswith("chris_integrator.")]:
        del sys.modules[k]                          # the tree's own package, not this tree's reader's
    from chris_integrator.mixer import MessageClock
    from chris_integrator.stack import build
    stack = build(clock=MessageClock(), mapper=None)
    stack.mixer.add_output("chain", lambda env: cb(env.stream, env.line))

    def deliver(daemon, m, line):
        stack.deliver(m, line)

n = 0
for m in read_lines(a.road):
    if m.get("type") not in ("road", "calib"):
        continue
    ts = m.get("ts_mono_ns")
    ln = json.dumps(m, separators=(",", ":"))
    if m.get("type") == "road":
        t = ts * 1e-9
        if t < a.t0 or t > a.t1:
            continue
        clock[0] = t
        if a.ego:
            v = m.get("v_ego") or 0.0
            # the host's yaw rate is positive LEFT (the model's pose is positive right); bias and speed-proportional error in rad/s, + = left
            yr = -(m.get("pose") or {}).get("yaw_rate", 0.0) + a.ego_yaw_bias + a.ego_yaw_per_speed * v
            e = {"type": "ego", "seq": m["seq"], "ts_mono_ns": ts, "ts_real_ns": ts, "api": 1, "speed_mps": v, "gear": "D", "reverse": False,
                 "standstill": v < 0.1, "wheel_speeds_mps": [v] * 4, "yaw_rate_rps": yr, "turn_signal": None, "source": "vehicle", "tag": "inject"}
            deliver("mapd", e, json.dumps(e))
        n += 1
    deliver("modeld", m, ln)
out.close()
print("road frames", n, "(%s)" % ("old api State of " + a.api if old else "chris-integrator of " + tree), file=sys.stderr)
