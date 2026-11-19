#!/usr/bin/env python3
"""Stage 1 of the chain trace: a `road` stream (modeld_replay) through the api's own State (the code that runs on
the phone), any version: what the api sends to its /ws consumers.
  usage: chain_api.py ROAD.jsonl.gz OUT.jsonl.gz --api API_DIR [--t0 S --t1 S] [--ego-yaw-bias RAD_S] [--ego-steer-offset DEG]
         [--ego] (feed a host `ego` message per road frame: speed = road.v_ego, yaw rate = pose.yaw_rate + bias)
Every output line gets "_rx_s" (frame time + 0.08 s). The clock the State reads is the stream's time."""
import argparse, gzip, json, sys, time, math
ap = argparse.ArgumentParser()
ap.add_argument("road"); ap.add_argument("out"); ap.add_argument("--api", required=True)
ap.add_argument("--t0", type=float, default=0); ap.add_argument("--t1", type=float, default=1e9)
ap.add_argument("--ego", action="store_true"); ap.add_argument("--ego-yaw-bias", type=float, default=0.0)
ap.add_argument("--ego-yaw-per-speed", type=float, default=0.0, help="yaw-rate error proportional to speed, rad/s per m/s (a steering-angle zero offset)")
a = ap.parse_args()
sys.path.insert(0, a.api)
import roadsense_api.state as S
clock = [0.0]
class _T:
    def __getattr__(self, k): return getattr(time, k)
    def monotonic(self): return clock[0]
    def monotonic_ns(self): return int(clock[0] * 1e9)
    def time(self): return clock[0]
    def time_ns(self): return int(clock[0] * 1e9)
S.time = _T()
st = S.State("/nonexistent")
out = gzip.open(a.out, "wt")
def cb(topic, line):
    line = line.rstrip()
    seen.add(topic)
    out.write(line[:-1] + ',"_rx_s":%.4f}\n' % (clock[0] + 0.08))
seen = set()
st.subscribers.add(cb)
n = 0
for ln in gzip.open(a.road, "rt"):
    m = json.loads(ln)
    ts = m.get("ts_mono_ns")
    if m.get("type") == "road":
        t = ts * 1e-9
        if t < a.t0 or t > a.t1: continue
        clock[0] = t
        if a.ego:
            v = m.get("v_ego") or 0.0
            # the host's yaw rate is positive LEFT (the model's pose is positive right); bias and speed-proportional error in rad/s, + = left
            yr = -(m.get("pose") or {}).get("yaw_rate", 0.0) + a.ego_yaw_bias + a.ego_yaw_per_speed * v
            e = {"type": "ego", "seq": m["seq"], "ts_mono_ns": ts, "ts_real_ns": ts, "api": 1, "speed_mps": v, "gear": "D", "reverse": False,
                 "standstill": v < 0.1, "wheel_speeds_mps": [v] * 4, "yaw_rate_rps": yr, "turn_signal": None, "source": "vehicle", "tag": "inject"}
            st.on_message("mapd", e, json.dumps(e))
        n += 1
    st.on_message("modeld", m, ln)
    if m.get("type") in ("road", "calib") and m["type"] not in seen:   # a State that does not pass modeld's own lines on
        cb(m["type"], ln)
        seen.discard(m["type"])
out.close()
print("road frames", n, file=sys.stderr)
