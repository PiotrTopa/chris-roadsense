#!/usr/bin/env python3
"""Run the api's speed-limit state machine offline over a recorded /ws stream (JSONL, one message per line).

    tools/record_ws.py ... > ride.jsonl   (or any JSONL with road/signs/env messages)
    python3 tools/speedlimit_replay.py ride.jsonl [--rules pl] [--every 10] [--cx 960 --focal 690]
Prints every change of the served limit and a periodic status line; times are seconds from the first message.
"""
import argparse
import json
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "api"))
from roadsense_api.speedlimit import SpeedLimit, load_rules  # noqa: E402


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("jsonl")
    ap.add_argument("--rules", default="pl")
    ap.add_argument("--every", type=float, default=0, help="also print a status line every N seconds")
    ap.add_argument("--cx", type=float, default=960.0)
    ap.add_argument("--focal", type=float, default=690.0)
    ap.add_argument("--json", action="store_true", help="print events as JSON lines")
    a = ap.parse_args()
    sl = SpeedLimit(load_rules(a.rules), cfg={"cx": a.cx, "focal_px": a.focal})
    t0 = last_status = None
    n_changes = 0
    seen = set()
    with open(a.jsonl) as f:
        for line in f:
            msg = json.loads(line)
            ts = msg.get("ts_mono_ns")
            if ts is None:
                continue
            t0 = t0 if t0 is not None else ts
            t = (ts - t0) / 1e9
            changed = sl.on_message(msg)
            for i in sl.ignored:
                if id(i) not in seen:
                    seen.add(id(i))
                    print(f"{t:7.1f}s  ignored #{i['track_id']} {i['cls']}: {i['reason']}")
            if changed:
                n_changes += 1
                s = sl.snapshot(full=True)
                ev = s["recent_changes"][0]["event"] if s["recent_changes"] else ""
                src = s["source"]
                what = f"{src['sign']} #{src['track_id']}" if src["kind"] == "sign" else f"{src['kind']}:{src.get('rule')}"
                if a.json:
                    print(json.dumps({"t": round(t, 1), **s}))
                else:
                    print(f"{t:7.1f}s  odo {s['odometer_m']:7.0f} m  LIMIT {s['limit_kmh']}  conf {s['confidence']:.2f}  "
                          f"{what}  area {s['area']['type']}({s['area']['source']})  zone {s['zone']['type']}  "
                          f"road {s['road_class']}  [{ev}]")
            if a.every and (last_status is None or t - last_status >= a.every):
                last_status = t
                s = sl.snapshot()
                print(f"{t:7.1f}s  .. {s['limit_kmh']} conf {s['confidence']:.2f} v {s['v_ego_kmh']} km/h "
                      f"pending {[(p['sign'], p['value_kmh'], p['hits']) for p in s['pending']]}")
    print(f"changes {n_changes}, odometer {sl.odo:.0f} m")


if __name__ == "__main__":
    main()
