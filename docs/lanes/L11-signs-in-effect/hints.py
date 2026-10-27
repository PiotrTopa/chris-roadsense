#!/usr/bin/env python3
"""Detector hints for a labeller: ride times at which signd reported tracks of rule-relevant classes (raw detections,
no engine logic). usage: hints.py REC.jsonl SLUG -> prints a table (time, class, n frames, best score, suppressed)"""
import json, re, sys
rec, slug = sys.argv[1], sys.argv[2]
REL = re.compile(r"^(regulatory--(priority-road|end-of-priority-road|yield|stop$|no-stopping|no-parking|end-of-no-parking|end-of-prohibition|no-overtaking|height-limit|weight-limit|width-limit|one-way|no-left-turn|no-right-turn|no-u-turn|no-entry|roundabout|go-straight|turn-|road-closed|no-motor|no-heavy|priority-over|give-way)|information--(living-street|end-of-living-street|end-of-built-up|motorway|end-of-motorway|limited-access|end-of-limited|pedestrians-crossing|dead-end)|warning--)")
cam = None; cur = None; tracks = {}
with open(rec) as f:
    for line in f:
        m = json.loads(line)["m"]
        if m.get("type") == "camera":
            cur = m.get("ride")
            if m.get("ride_t_s") is not None:
                cam = (m["ts_mono_ns"], m["ride_t_s"])
            continue
        if cur != slug or m.get("type") != "signs" or cam is None:
            continue
        t = cam[1] + (m["ts_mono_ns"] - cam[0]) / 1e9
        for d in m.get("detections", []):
            cls = re.sub(r"--g\d+$", "", d.get("cls", ""))
            if not REL.match(cls):
                continue
            tr = tracks.setdefault(d["track_id"], {"t0": t, "t1": t, "cls": {}, "n": 0, "best": 0.0, "sup": None, "x": 0})
            tr["t1"] = t; tr["n"] += 1; tr["cls"][cls] = tr["cls"].get(cls, 0) + 1
            tr["best"] = max(tr["best"], d.get("score", 0)); tr["x"] = (d["box"][0] + d["box"][2]) / 2
            if d.get("suppressed"): tr["sup"] = d.get("reason")
print(f"# {slug}: {len(tracks)} relevant tracks (time = when last seen; x = box centre px of 1280; 'sup' = flagged by L8)")
for tid, tr in sorted(tracks.items(), key=lambda kv: kv[1]["t1"]):
    cls = max(tr["cls"], key=tr["cls"].get)
    print(f"{tr['t1']:7.1f}s  {cls:55s} frames {tr['n']:2d}  best {tr['best']:.2f}  x {tr['x']:4.0f}" + (f"  sup={tr['sup']}" if tr["sup"] else ""))
