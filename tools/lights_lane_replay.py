#!/usr/bin/env python3
"""Replay a recorder session through the ego-light selection with and without the lane assignment
(api/roadsense_api/lights.py, lane L16).

    tools/lights_lane_replay.py SESSION/messages.jsonl.gz [--events] [--out LIGHTS.jsonl]

The session's `road` lines are run through the road-geometry tracker (the `lanes` topic is not recorded), its
`lights` lines are annotated twice: "window" (the lateral window alone, as before lane L16) and "lanes" (with
the lane assignment). Printed:

* conflicts: the spans in which facing heads show stop and go at the same time (two signal groups), what
  each selection showed in them, the confidence of the assignment and the share reported ambiguous;
* a stress test, "the ego head is missed": per conflict message the heads of the group the lane assignment
  chose are removed and both selections run again on what is left (the other group's heads). Whatever they
  show then is the wrong group's state: the share of such messages with an ego light is the rate at which a
  selection would show another lane's light when the detector misses the right one;
* the usual ego-light figures for both (share of time, changes, blinks).
"""
import argparse
import copy
import gzip
import json
import zlib
import os
import sys
import time

sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "api"))

from roadsense_api import lights as L                      # noqa: E402
from roadsense_api.lanes import RoadGeometryTracker        # noqa: E402

DROP = ("dist_m", "lateral_m", "relevance", "ego", "slot")


def read_lines(path):
    op = gzip.open if path.endswith(".gz") else open
    try:
        with op(path, "rt") as f:
            for line in f:
                if line.startswith("{"):
                    try:
                        yield json.loads(line)
                    except ValueError:
                        continue
    except (EOFError, OSError) as e:
        print("note: %s: %s" % (path, e), file=sys.stderr)


def strip(m):
    m = copy.deepcopy(m)
    for k in ("ego", "map_signal"):
        m.pop(k, None)
    for li in m.get("lights") or []:
        for k in DROP:
            li.pop(k, None)
    return m


def colour(st):
    return L._colour(st)


def stats(rows, name):
    dur = rows[-1][0] - rows[0][0]
    dt = dur / (len(rows) - 1)
    on = [r[1] is not None for r in rows]
    changes = sum(1 for x, y in zip(on, on[1:]) if x != y)
    gaps, run, seen = [], 0, False
    for x in on:
        if x:
            if seen and run:
                gaps.append(run * dt)
            run, seen = 0, True
        else:
            run += 1
    flips = 0
    last = None
    for t, e in rows:
        if e is None or e.get("held"):
            continue
        c = colour(e["state"])
        if last and c and last[1] and c != last[1] and t - last[0] < 3.0 and e["track_id"] != last[2]:
            flips += 1
        last = (t, c, e["track_id"])
    print("%-7s ego light %.1f %% of the time, on/off changes %.1f/min, gaps < 1.5 s: %d, "
          "stop<->go changes with a change of track within 3 s: %d" % (
              name, 100.0 * sum(on) / len(on), changes / (dur / 60.0), sum(1 for g in gaps if g < 1.5), flips))


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("session")
    ap.add_argument("--events", action="store_true")
    ap.add_argument("--out", help="write the lights messages annotated with the lane assignment")
    a = ap.parse_args()
    trk = RoadGeometryTracker()
    sel = {"window": L.EgoLightSelector(), "lanes": L.EgoLightSelector()}
    sel["window"].lane_assign = False
    road = calib = None
    rows = {"window": [], "lanes": []}
    spans, cur = [], None
    stress = {"n": 0, "window": 0, "lanes": 0, "lanes_amb": 0}
    out = open(a.out, "w") if a.out else None
    for m in read_lines(a.session):
        ty = m.get("type")
        if ty == "calib":
            calib = m
        elif ty == "ego":
            trk.on_ego(m)
        elif ty == "road":
            road = m
            lanes = trk.on_road(m)
            if lanes is not None:
                sel["lanes"].on_lanes(lanes)
        elif ty == "lights":
            t = m["ts_mono_ns"] / 1e9
            fw, fh = (m.get("frame") or [1280, 720])[:2]
            f = (calib or {}).get("focal_px") or 690.0
            intr = (float(f), fw / 2.0, fh / 2.0, fw, fh)
            rd = road if road and abs(road["ts_mono_ns"] / 1e9 - t) < 1.0 else None
            res = {}
            for k in sel:
                mm = strip(m)
                keep = sel[k].track
                sel[k].annotate(mm, rd, calib, intr)
                res[k] = (mm, keep)
                rows[k].append((t, mm.get("ego")))
            ew, el = res["window"][0].get("ego"), res["lanes"][0].get("ego")
            if a.events and ((ew is None) != (el is None) or (ew and el and colour(ew["state"]) != colour(el["state"]))):
                print("%s  differ: window %s  lanes %s" % (
                    time.strftime("%H:%M:%S", time.localtime(m.get("ts_real_ns", 0) / 1e9)),
                    ew and "#%s %s lat %+.1f d %.0f rel %.2f%s" % (ew["track_id"], ew["state"], ew["lateral_m"], ew["dist_m"],
                                                                 ew["relevance"], " held" if ew.get("held") else ""),
                    el and "#%s %s lat %+.1f d %.0f slot %s%s" % (el["track_id"], el["state"], el["lateral_m"], el["dist_m"],
                                                                (el.get("assign") or {}).get("slot"),
                                                                " held" if el.get("held") else "")))
            if out:
                out.write(json.dumps(res["lanes"][0], separators=(",", ":")) + "\n")
            ml = res["lanes"][0]
            e = ml.get("ego")
            asg = (e or {}).get("assign") if e and not e.get("held") else None
            if asg and asg["conflict"]:
                if cur and t - cur["t1"] < 1.5:
                    cur["t1"] = t
                else:
                    cur = {"t0": t, "t1": t, "real": m.get("ts_real_ns", 0) / 1e9, "n": 0, "conf": [], "amb": 0,
                           "lanes": {}, "window": {}, "basis": {}}
                    spans.append(cur)
                cur["n"] += 1
                cur["conf"].append(asg["confidence"])
                cur["amb"] += 1 if e.get("amb") else 0
                cur["basis"][asg["basis"]] = cur["basis"].get(asg["basis"], 0) + 1
                for k in sel:
                    ek = res[k][0].get("ego")
                    st = colour(ek["state"]) if ek else None
                    cur[k][st] = cur[k].get(st, 0) + 1
                # stress test: remove the chosen group's heads
                col = colour(e["state"])
                rest = [li for li in strip(m)["lights"] if colour(li.get("state")) != col]
                stress["n"] += 1
                for k in sel:
                    mm = strip(m)
                    mm["lights"] = copy.deepcopy(rest)
                    L.annotate(mm, rd, calib, intr, lanes=sel[k].lanes if k == "lanes" else None,
                               lane_assign=(k == "lanes"))
                    if mm.get("ego"):
                        stress[k] += 1
                        if a.events:
                            ee = mm["ego"]
                            print("    stress %s: #%s %s lat %+.1f d %.0f rel %.2f slot %s" % (
                                k, ee["track_id"], ee["state"], ee["lateral_m"], ee["dist_m"], ee["relevance"],
                                (ee.get("assign") or {}).get("slot")))
                if a.events:
                    print("%s  %s  ego #%s %-6s slot %+.2f conf %.2f%s  alt %s %s (slot %+.2f)  window: %s" % (
                        time.strftime("%H:%M:%S", time.localtime(m.get("ts_real_ns", 0) / 1e9)), asg["basis"],
                        e["track_id"], e["state"], asg["slot"], asg["confidence"], " AMB" if e.get("amb") else "",
                        asg["alt"]["state"], asg["alt"]["side"], asg["alt"]["slot"],
                        (res["window"][0].get("ego") or {}).get("state")))
    if out:
        out.close()
    if not rows["lanes"]:
        sys.exit("no lights messages")
    print("%d lights messages, %.1f min" % (len(rows["lanes"]), (rows["lanes"][-1][0] - rows["lanes"][0][0]) / 60.0))
    for k in ("window", "lanes"):
        stats(rows[k], k)
    rej = sum(1 for _, e in rows["lanes"] if e is None) - sum(1 for _, e in rows["window"] if e is None)
    print("messages with an ego light: window %d, lanes %d (difference %+d)" % (
        sum(1 for _, e in rows["window"] if e), sum(1 for _, e in rows["lanes"] if e), -rej))
    tot = sum(s["n"] for s in spans)
    print("conflicts (stop and go heads facing at once): %d spans, %d messages, %.1f s" % (
        len(spans), tot, sum(s["t1"] - s["t0"] for s in spans)))
    for s in spans:
        c = sorted(s["conf"])
        print("  %s %5.1f s n %3d  lanes %s  window %s  confidence min/median %.2f/%.2f  ambiguous %d  basis %s" % (
            time.strftime("%H:%M:%S", time.localtime(s["real"])), s["t1"] - s["t0"], s["n"], s["lanes"], s["window"],
            c[0], c[len(c) // 2], s["amb"], s["basis"]))
    if stress["n"]:
        print("stress (the chosen group's heads removed, %d messages): an ego light from the other group is "
              "shown by window in %.0f %%, by lanes in %.0f %%" % (
                  stress["n"], 100.0 * stress["window"] / stress["n"], 100.0 * stress["lanes"] / stress["n"]))


if __name__ == "__main__":
    main()
