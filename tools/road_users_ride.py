#!/usr/bin/env python3
"""road_users_ride.py: road-user classes at the phone's full rate on a ride of the library (lane L42).

    road_users_ride.py RIDE.road.jsonl.gz LIGHTS.jsonl FRAMES.list [--out OUT.json]

RIDE.road.jsonl.gz: modeld's `calib` + `road` messages of the ride (~/datasets/rides-road, frame i at i * 50 ms);
LIGHTS.jsonl: roadsense-signs `lights_replay --road-users` (the phone unit's two alternating views, ncnn-cpu) on
every 4th frame of the ride (FRAMES.list, `tools/road_users/mjpeg_frames.py`: 5 a second, the unit's rate),
one line per frame in the list's order. The light messages are put on the ride's clock and the whole goes
through the default stack (no map box, no vehicle feed): the lead's class over time, its changes per minute
(after the vote), and the display objects, as in tools/road_users_eval.py.
"""
import argparse, json, os, sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from road_users_eval import lead_stats, run  # noqa: E402
from chris_integrator.replay import read_lines  # noqa: E402
import collections  # noqa: E402


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("road")
    ap.add_argument("lights")
    ap.add_argument("frames")
    ap.add_argument("--out")
    a = ap.parse_args()
    msgs = [m for m in read_lines(a.road) if m.get("type") in ("calib", "road")]
    calib = next(m for m in msgs if m["type"] == "calib")
    files = [ln.strip() for ln in open(a.frames) if ln.strip()]
    lights = []
    with open(a.lights) as f:
        for k, ln in enumerate(f):
            if k >= len(files):
                break
            m = json.loads(ln)
            i = int(os.path.basename(files[k])[:-4])
            m["ts_mono_ns"] = i * 50_000_000 + 1          # just after the road frame of the same picture
            m["seq"] = i + 1
            m.pop("raw", None)
            m.pop("timing", None)
            lights.append(m)
    calib = dict(calib, ts_mono_ns=0)
    out_msgs = [calib] + sorted([m for m in msgs if m["type"] == "road"] + lights, key=lambda m: m["ts_mono_ns"])
    for m in out_msgs:
        if m.get("type") == "road":
            m.setdefault("road_model", "supercombo")    # ~/datasets/rides-road: supercombo v0.9.7 (L14)
    out, ms, n_road = run(out_msgs, True)
    _, ms0, _ = run(out_msgs, False)
    ru = [[o for o in m["objects"] if o.get("kind") == "road_user"] for m in out["view"]]
    n_users = collections.Counter(o["class"] for m in out["road_users"] for o in m["objects"])
    res = {"ride": os.path.basename(a.road).split(".")[0], "road_frames": n_road, "light_frames": len(lights),
           "lead": lead_stats(out["lead"]),
           "road_users_by_class": dict(n_users),
           "display": {"view_msgs": len(ru), "objects_per_msg_mean": round(sum(map(len, ru)) / max(1, len(ru)), 3),
                       "msgs_with_any": round(sum(1 for r in ru if r) / max(1, len(ru)), 3),
                       "by_class": dict(collections.Counter((o.get("cls") or {}).get("label") for r in ru for o in r)),
                       "distinct_ids": len({o["id"] for r in ru for o in r})},
           "cost_ms_per_road_frame": {"with": round(ms, 3), "without": round(ms0, 3)}}
    # the lead's label sequence per track (for a look at what changed)
    seqs, prev = [], None
    for m in out["lead"]:
        if not m.get("present"):
            continue
        key = (m.get("track_id"), (m.get("class") or {}).get("label"))
        if key != prev:
            seqs.append({"t": round(m["ts_mono_ns"] / 1e9, 2), "track": key[0], "label": key[1],
                         "conf": (m.get("class") or {}).get("confidence")})
            prev = key
    res["lead_label_runs"] = len(seqs)
    js = json.dumps(dict(res, runs=seqs), indent=1)
    if a.out:
        open(a.out, "w").write(js)
    print(json.dumps(res, indent=1))


if __name__ == "__main__":
    main()
