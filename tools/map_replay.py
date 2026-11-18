#!/usr/bin/env python3
"""Replay a drive-recorder session through mapd's engine, offline, and measure the map layer: position states,
map matching (confidence, road class, distance to the road, switches), the junctions and roundabouts announced
ahead, and the junction view.

    tools/map_replay.py SESSION [SESSION ...] [--pack PACK] [--fixes FIXES.jsonl] [--set KEY=VALUE ...] [--out OUT.jsonl]

SESSION     a recorder session directory (or its messages.jsonl.gz). Its `ego` messages are fed back as vehicle
            `motion` (speed, gear, the yaw rate that was in use, wheel speeds). Fixes: the session's own `map`
            messages while `good` (the filtered position at the time of a fresh fix, with the receiver's hacc;
            the host does not log raw fixes), or, with --fixes, raw `gnss` messages (JSON lines with `ts_real_ns`)
            wherever they cover the drive.
--set       engine / matcher / filter options for a variant, e.g. `--set matcher.class_speed=0` or
            `--set gnss_adaptive=false` (dotted keys go into the sub-dicts `matcher`, `filter`).
--out       the `map` messages of the replay (JSON lines), for tools further down the chain.

Everything is printed for the moving part of the drive (speed >= 3 m/s); the reference for "right road" is not
known (no ground truth position), so the figures are the matcher's own: what share of the time the map layer is
usable, how confident, how often on a service road at speed, how far the position is from the matched road.
"""
import argparse
import bisect
import collections
import gzip
import json
import math
import os
import statistics
import sys
import time
import zlib

sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "mapd"))

from roadsense_mapd.engine import Engine            # noqa: E402
from roadsense_mapd.rsmap import Pack               # noqa: E402

NS = 10 ** 9


def read_lines(path):
    if os.path.isdir(path):
        path = os.path.join(path, "messages.jsonl.gz")
    with open(path, "rb") as f:
        d, buf = (zlib.decompressobj(31), b"") if path.endswith(".gz") else (None, b"")
        while True:
            chunk = f.read(1 << 20)
            if not chunk:
                break
            if d is not None:
                try:
                    chunk = d.decompress(chunk)
                except zlib.error:          # a recording cut off by a power loss
                    break
            buf += chunk
            *full, buf = buf.split(b"\n")
            for ln in full:
                if ln.startswith(b'{"type":"ego"') or ln.startswith(b'{"type":"map"') \
                        or ln.startswith(b'{"type": "ego"') or ln.startswith(b'{"type": "map"'):
                    try:
                        yield json.loads(ln)
                    except ValueError:
                        continue


def load(session, fixes_path=None):
    """(inputs [(ts_mono, msg)], map step times [ts_mono], recorded map messages)"""
    inputs, steps, recorded = [], [], []
    off = None                                   # ts_real - ts_mono of the recorder host
    last_fix_t = None
    for m in read_lines(session):
        t = m.get("ts_mono_ns")
        if not isinstance(t, int):
            continue
        if off is None and isinstance(m.get("ts_real_ns"), int):
            off = m["ts_real_ns"] - t
        if m["type"] == "ego":
            mo = {"type": "motion", "ts_mono_ns": t, "speed_mps": abs(m.get("speed_mps") or 0.0),
                  "source": m.get("tag") or "recorded"}
            if m.get("gear"):
                mo["gear"] = m["gear"]
            if m.get("reverse"):
                mo["reverse"] = True
            if m.get("yaw_rate_rps") is not None:
                mo["yaw_rate_rps"] = m["yaw_rate_rps"]
            elif m.get("wheel_speeds_mps"):
                mo["wheel_speeds_mps"] = m["wheel_speeds_mps"]
            if m.get("turn_signal") in ("off", "left", "right", "both"):
                mo["turn_signal"] = m["turn_signal"]
            inputs.append((t, mo))
            continue
        steps.append(t)
        recorded.append(m)
        if fixes_path:
            continue
        g = (m.get("input") or {}).get("gnss") or {}
        pos = m.get("position")
        if m.get("state") == "good" and pos and g.get("fix") not in (None, "none"):
            age = g.get("age_s")
            if isinstance(age, (int, float)) and age <= 1.0:
                tf = t - int(age * NS)
                if last_fix_t is None or tf - last_fix_t > 0.5 * NS:
                    last_fix_t = tf
                    inputs.append((tf, {"type": "gnss", "ts_mono_ns": tf, "lat": pos["lat"], "lon": pos["lon"],
                                        "hacc_m": g.get("hacc_m") or 10.0, "source": "recorded-position"}))
        elif g.get("fix") == "none" and (not inputs or inputs[-1][1].get("fix") != "none"):
            inputs.append((t, {"type": "gnss", "ts_mono_ns": t, "fix": "none", "source": "recorded"}))
    if fixes_path and off is not None and steps:
        t0, t1 = steps[0], steps[-1]
        op = gzip.open if fixes_path.endswith(".gz") else open
        with op(fixes_path, "rt") as f:
            for ln in f:
                g = json.loads(ln)
                t = g.pop("ts_real_ns") - off
                if t0 <= t <= t1:
                    g["ts_mono_ns"] = t
                    inputs.append((t, g))
    inputs.sort(key=lambda x: x[0])
    return inputs, steps, recorded


def setopt(cfg, kv):
    k, _, v = kv.partition("=")
    try:
        v = json.loads(v)
    except ValueError:
        pass
    parts = k.split(".")
    d = cfg
    for p in parts[:-1]:
        d = d.setdefault(p, {})
    d[parts[-1]] = v


def replay(session, pack, cfg, fixes=None, out=None, stats=None):
    inputs, steps, recorded = load(session, fixes)
    eng = Engine(pack, cfg)
    i = 0
    cost = []
    rows = []
    for n, t in enumerate(steps):
        while i < len(inputs) and inputs[i][0] <= t:
            eng.on_input(inputs[i][1], inputs[i][0])
            i += 1
        c0 = time.perf_counter()
        m = eng.step(t)
        cost.append(time.perf_counter() - c0)
        if out:
            m["ts_real_ns"] = recorded[n].get("ts_real_ns")
            out.write(json.dumps(m) + "\n")
        rows.append(m)
    return rows, cost


def summary(name, rows, cost):
    mv = [m for m in rows if m.get("position") and abs(m["position"]["speed_mps"]) >= 3.0]
    allmv = len(mv)
    if not allmv:
        print("%s: no moving steps with a position" % name)
        return {}
    av = [m for m in mv if m["available"]]
    st = collections.Counter(m["state"] for m in mv)
    conf = [m["match"]["confidence"] for m in av]
    cls = collections.Counter(m["road"]["class"] for m in av)
    fast = [m for m in av if m["position"]["speed_mps"] >= 7.0]
    svc_fast = sum(m["road"]["class"] == "service" for m in fast)
    dist = [m["match"]["distance_m"] for m in av]
    hacc = [m["position"]["hacc_m"] for m in av]
    sw = sum(1 for a, b in zip(av, av[1:]) if a["match"]["edge"] != b["match"]["edge"])
    km = sum(m["position"]["speed_mps"] * 0.5 for m in mv) / 1000.0
    jv = [m["ahead"].get("junction_view") for m in av]
    jvk = collections.Counter(v["kind"] for v in jv if v)
    rb = sum(1 for m in av if any(j["kind"] == "roundabout" for j in m["ahead"]["junctions"]))
    q = lambda v, p: sorted(v)[min(len(v) - 1, int(p * len(v)))] if v else float("nan")    # noqa: E731
    r = {"moving_steps": allmv, "located": round(100.0 * (st["good"] + st["dead_reckoning"]) / allmv, 1),
         "available": round(100.0 * len(av) / allmv, 1),
         "conf_med": round(statistics.median(conf), 2) if conf else None,
         "conf_ge_0.5": round(100.0 * sum(c >= 0.5 for c in conf) / max(1, len(conf)), 1),
         "conf_ge_0.7": round(100.0 * sum(c >= 0.7 for c in conf) / max(1, len(conf)), 1),
         "service_at_speed": round(100.0 * svc_fast / max(1, len(fast)), 1),
         "dist_med": round(statistics.median(dist), 1) if dist else None, "dist_p90": round(q(dist, 0.9), 1),
         "hacc_med": round(statistics.median(hacc), 1) if hacc else None,
         "switches_per_km": round(sw / max(km, 0.01), 1), "km": round(km, 2),
         "jv": round(100.0 * sum(1 for v in jv if v) / allmv, 1), "jv_kinds": dict(jvk),
         "rb_ahead_steps": rb,
         "classes": {k: round(100.0 * v / len(av)) for k, v in cls.most_common(6)},
         "step_ms_mean": round(1e3 * statistics.mean(cost), 3), "step_ms_p95": round(1e3 * q(cost, 0.95), 3)}
    print("%s: %s" % (name, json.dumps(r)))
    return r


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("sessions", nargs="+")
    ap.add_argument("--pack", default=os.path.expanduser("~/roadsense-maps/packs/poland-20261114.rsmap"))
    ap.add_argument("--fixes")
    ap.add_argument("--set", action="append", default=[])
    ap.add_argument("--out")
    a = ap.parse_args()
    cfg = {}
    for kv in a.set:
        setopt(cfg, kv)
    pack = Pack(a.pack)
    out = open(a.out, "w") if a.out else None
    allrows, allcost = [], []
    for s in a.sessions:
        rows, cost = replay(s, pack, cfg, a.fixes, out)
        summary(os.path.basename(s.rstrip("/")), rows, cost)
        allrows += rows
        allcost += cost
    if len(a.sessions) > 1:
        summary("ALL", allrows, allcost)


if __name__ == "__main__":
    main()
