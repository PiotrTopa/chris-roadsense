#!/usr/bin/env python3
"""How much the lane boxes and their turn arrows of the display channel flicker on a recorded drive (lane L36).

    tools/turn_flicker.py SESSION[/messages.jsonl.gz] [--api DIR] [--recorded] [--min-speed 0]

Replays the session through the api's derived topics (tools/replay_lib.py: the api's order, `view` per road
frame) with the api code of `--api` (default: this checkout's api/), or with `--recorded` reads the `view`
messages as they were recorded (the recorder keeps 2 per second), and counts on `view.road`, per minute of
session time (frames at most 1.5 s apart; with `--min-speed` only frames while the car moves at least that
fast, a change counted when both frames around it are such frames):

    turn on/off        `lanes.turn` present / absent
    arrows on/off      what a lane-box consumer can draw: `turn` present, as many arrows as `lanes.forward`,
                       and on the stretch (`here`) or within 120 m of it (the MFD's rule until L36)
    boxes on/off       `position.trusted` and `lanes.forward` 2 or more (the lane boxes of a consumer; one lane
                       alone draws none)
    count changes      `lanes.forward` changes while the boxes are on
    ego on/off         `lanes.ego_index` known / unknown while the boxes are on
    trusted on/off     `position.trusted`
    dist up-jumps      `turn.dist_m` growing by more than 5 m between two frames with a turn while the car
                       moves (the distance to one junction only falls; growing = another stretch or a step)
    held               frames whose `turn` is marked held (L36 and later)
"""
import argparse
import os
import sys

# --api: the chris-integrator directory of another checkout (before / after on the same data)
_API = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "chris-integrator")
if "--api" in sys.argv:
    _API = os.path.abspath(sys.argv[sys.argv.index("--api") + 1])
sys.path.insert(0, _API)
from chris_integrator.replay import Pipeline, read_lines, read_session      # noqa: E402

ARROWS_AHEAD_M = 120.0


def views(session, recorded):
    if recorded:
        for m in read_lines(session):
            if m.get("type") == "view":
                yield m
        return
    p = Pipeline()
    for m in read_session(session):
        for o in p.feed(m):
            if o.get("type") == "view":
                yield o


def state(v):
    rd = v.get("road") or {}
    pos = rd.get("position") or {}
    ln = rd.get("lanes") or {}
    tn = ln.get("turn") if isinstance(ln.get("turn"), dict) else None
    fwd = ln.get("forward")
    trusted = bool(pos.get("trusted"))
    boxes = trusted and isinstance(fwd, int) and fwd >= 2         # one lane alone: a consumer draws no box
    arrows = bool(boxes and tn is not None and isinstance(tn.get("arrows"), list) and len(tn["arrows"]) == fwd
                  and (tn.get("here") or (isinstance(tn.get("dist_m"), (int, float)) and tn["dist_m"] <= ARROWS_AHEAD_M)))
    return {"turn": tn is not None, "arrows": arrows, "boxes": boxes,
            "count": fwd if boxes else None, "ego": boxes and ln.get("ego_index") is not None,
            "trusted": trusted, "dist": tn.get("dist_m") if tn else None, "held": bool(tn and tn.get("held"))}


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("session")
    ap.add_argument("--api")
    ap.add_argument("--recorded", action="store_true")
    ap.add_argument("--min-speed", type=float, default=0.0)
    a = ap.parse_args()
    keys = ("turn", "arrows", "boxes", "ego", "trusted")
    ch = dict.fromkeys(keys, 0)
    on_s = dict.fromkeys(keys, 0.0)
    count_ch = up = held = frames = 0
    drive_s = 0.0
    prev = None
    for v in views(a.session, a.recorded):
        ts = v.get("ts_mono_ns")
        if not isinstance(ts, int):
            continue
        sp = (v.get("ego") or {}).get("speed_mps") or 0.0
        s = state(v)
        s["t"], s["moving"], s["sp"] = ts / 1e9, sp >= a.min_speed, sp
        if prev is not None and prev["moving"] and s["moving"] and 0.0 < s["t"] - prev["t"] <= 1.5:
            dt = s["t"] - prev["t"]
            drive_s += dt
            frames += 1
            held += s["held"]
            for k in keys:
                ch[k] += s[k] != prev[k]
                on_s[k] += dt if s[k] else 0.0
            if s["boxes"] and prev["boxes"] and s["count"] != prev["count"]:
                count_ch += 1
            if s["dist"] is not None and prev["dist"] is not None and s["dist"] > prev["dist"] + 5.0 and s["sp"] >= 1.0:
                up += 1
        prev = s
    mins = max(drive_s / 60.0, 1e-9)
    print("%s  api %s  %s" % (a.session, "recorded view" if a.recorded else _API, "%.1f min, %d frames" % (mins, frames)))
    for k in keys:
        print("  %-8s on/off %5d  = %6.2f /min   on %5.1f %% of the drive" % (k, ch[k], ch[k] / mins, 100.0 * on_s[k] / max(drive_s, 1e-9)))
    print("  count    changes %4d  = %6.2f /min" % (count_ch, count_ch / mins))
    print("  dist up-jumps %d = %.2f /min; frames with the turn held %d" % (up, up / mins, held))


if __name__ == "__main__":
    main()
