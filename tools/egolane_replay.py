#!/usr/bin/env python3
"""Replay recorded `road` messages through the lane tracker and the ego-lane estimator and score it against
hand labels.

    tools/egolane_replay.py ROAD_OR_SESSION --labels LANES.csv [--clock ride|session] [--map-count label|recorded|none]

ROAD_OR_SESSION  a ride's road messages (`*.road.jsonl.gz`, seq = frame number at 20 fps: --clock ride) or a
                 drive-recorder session directory (--clock session: label stamps are camera frame seqs).
--labels         CSV `source,t0,t1,lanes_own,ego_from_left,oncoming_left,confidence,note` (t0/t1 in seconds for
                 rides, camera seq for sessions; `na` where not known).
--map-count      what the estimator gets as the map's lane count: `label` (the labelled own-lane count, as a tagged
                 map would give it: the case of a matched, well-tagged road), `recorded` (the session's own `map`
                 messages), `none` (vision only).

Scored on labelled frames with 2+ own lanes and a known ego lane (confidence sure/likely), while moving:
committed share (the estimator names a lane), accuracy when committed, wrong-and-confident share, and the
time from the start of a labelled segment (a lane change, a new road) until the estimate is right. The same for
the rule `fused.lanes` used before lane L20 (rightmost lane when no lane is seen to the right, ...).
"""
import argparse
import bisect
import collections
import csv
import gzip
import json
import os
import sys
import zlib

sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "chris-integrator"))

from chris_integrator.trackers.egolane import EgoLaneEstimator      # noqa: E402
from chris_integrator.trackers.fusion import Fusion                  # noqa: E402
from chris_integrator.trackers.lanes import RoadGeometryTracker      # noqa: E402


def messages(path):
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
                except zlib.error:
                    break
            buf += chunk
            *full, buf = buf.split(b"\n")
            for ln in full:
                if ln.startswith(b"{"):
                    try:
                        yield json.loads(ln)
                    except ValueError:
                        continue


def load_labels(path):
    segs = []
    with open(path) as f:
        for r in csv.DictReader(f):
            try:
                n, e = int(r["lanes_own"]), int(r["ego_from_left"])
            except ValueError:
                n = e = None
            segs.append((float(r["t0"]), float(r["t1"]), n, e, r["confidence"].strip()))
    return segs


def fake_map(ts, n, oncoming=0):
    return {"type": "map", "ts_mono_ns": ts, "available": True, "match": {"confidence": 0.9},
            "road": {"oneway": oncoming == 0, "lanes": {"total": n + oncoming, "forward": n, "backward": oncoming,
                                                         "source": "tagged"}}}


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("src")
    ap.add_argument("--labels", required=True)
    ap.add_argument("--clock", choices=("ride", "session"), default="ride")
    ap.add_argument("--map-count", choices=("label", "recorded", "none"), default="label")
    ap.add_argument("--set", action="append", default=[], help="estimator option KEY=VALUE")
    ap.add_argument("--verbose", action="store_true")
    a = ap.parse_args()
    cfg = {}
    for kv in a.set:
        k, _, v = kv.partition("=")
        cfg[k] = json.loads(v)
    segs = load_labels(a.labels)
    seg_t0 = [s[0] for s in segs]
    tracker, est, fus = RoadGeometryTracker(), EgoLaneEstimator(cfg), Fusion()
    cam = []                                  # (ts, seq) for --clock session
    speed = 0.0
    rows = []                                 # (stamp, label n, label ego, conf, est ego, est conf, base ego)
    last_map = None
    for m in messages(a.src):
        t = m.get("type")
        if t == "camera":
            cam.append((m["ts_mono_ns"], m["seq"]))
            continue
        if t == "ego":
            speed = abs(m.get("speed_mps") or 0.0)
            tracker.on_ego(m)
            est.on_ego(m)
            continue
        if t == "map" and a.map_count == "recorded":
            last_map = m
            est.on_map(m)
            tracker.on_map(m)
            continue
        if t != "road":
            continue
        ts = m["ts_mono_ns"]
        if a.clock == "ride":
            stamp = (m["seq"] - 1) * 0.05
        else:
            if not cam:
                continue
            k = bisect.bisect_right([c[0] for c in cam[-50:]], ts) - 1
            stamp = cam[-50:][max(0, k)][1]
        i = bisect.bisect_right(seg_t0, stamp) - 1
        seg = segs[i] if i >= 0 and segs[i][0] <= stamp <= segs[i][1] else None
        if a.map_count == "label":
            n = seg[2] if seg and seg[2] else None
            if n:
                mm = fake_map(ts, n)
                est.on_map(mm)
                last_map = mm
            else:
                est.on_map(None)
                last_map = None
        ln = tracker.on_road(m)
        if ln is None:
            continue
        v = speed if speed else (m.get("v_ego") or 0.0)
        out = est.on_lanes(ln, m, speed_mps=v)
        base = fus._lanes(last_map if (last_map and last_map.get("road")) else None, m, ln, ts)["ego_index"]
        if seg is None or seg[2] is None or seg[2] < 2 or seg[4] not in ("sure", "likely") or v < 2.0:
            continue
        rows.append((stamp, seg, out["ego_index"], out["confidence"], base))
    # scoring
    res = {}
    for name, idx in (("estimator", 2), ("before_L20", 4)):
        n = len(rows)
        com = [r for r in rows if r[idx] is not None]
        right = sum(1 for r in com if r[idx] == r[1][3])
        res[name] = {"frames": n, "committed_pct": round(100.0 * len(com) / max(1, n), 1),
                     "accuracy_when_committed_pct": round(100.0 * right / max(1, len(com)), 1),
                     "wrong_confident_pct": round(100.0 * (len(com) - right) / max(1, n), 1),
                     "right_pct_of_all": round(100.0 * right / max(1, n), 1)}
        # time to correct from the start of each labelled segment
        ttc = []
        by_seg = collections.OrderedDict()
        for r in rows:
            by_seg.setdefault(r[1][:2], []).append(r)
        for (t0, t1), rs in by_seg.items():
            ok = next((r[0] for r in rs if r[idx] == r[1][3]), None)
            if ok is not None:
                ttc.append(ok - t0)
        res[name]["segments"] = len(by_seg)
        res[name]["segments_reached"] = len(ttc)
        ttc.sort()
        res[name]["time_to_correct_median"] = round(ttc[len(ttc) // 2], 1) if ttc else None
    print(json.dumps({"src": os.path.basename(a.src.rstrip("/")), "map_count": a.map_count, **res}))
    if a.verbose:
        for r in rows[::40]:
            print(r[0], r[1][2:4], r[2], r[3], r[4])


if __name__ == "__main__":
    main()
