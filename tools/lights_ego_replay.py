#!/usr/bin/env python3
"""Replay recorded `lights` messages through the api's ego-light selection (chris_integrator/trackers/lights.py).

    tools/lights_ego_replay.py LIGHTS.jsonl[.gz] --road ROAD.jsonl[.gz] [--step N] [--fps F] [--focal PX]
                               [--stateless] [--events]

LIGHTS: `lights` lines of a provider (roadsense-signs `lights_replay`, a /ws capture, a recorder session).
ROAD: `road` (and `calib`) lines of the same drive; the newest road message at or before each lights message
gives the planned path. For `lights_replay` output, whose seq counts processed frames, `--step N` says that
line k is ride frame (k - 1) * N + 1 and `--fps` the ride's rate; a capture carries real `ts_mono_ns` on both
and needs neither.

--stateless   the selection per message as it was before 2026-11-16 (no hysteresis, no hold; `coast` ignored
              by dropping coasting lights), for a before / after comparison on the same file

Printed: share of the time with any reported light, with an ego light (of it: held), ego on/off changes per
minute, the spans without an ego light that are shorter than 1.5 s (the blinking a consumer shows), and the
ego light's distance distribution.
"""
import argparse
import gzip
import json
import zlib
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "chris-integrator"))

from chris_integrator.trackers import lights as L      # noqa: E402


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


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("lights")
    ap.add_argument("--road", required=True)
    ap.add_argument("--step", type=int, default=0)
    ap.add_argument("--fps", type=float, default=20.0)
    ap.add_argument("--focal", type=float, help="focal length in pixels at the frame width (default: the calib line's)")
    ap.add_argument("--stateless", action="store_true")
    ap.add_argument("--events", action="store_true")
    ap.add_argument("--name", default="")
    a = ap.parse_args()

    roads, calib = [], None
    for m in read_lines(a.road):
        if m.get("type") == "calib":
            calib = m
        elif m.get("type") == "road":
            t = m["seq"] / a.fps if a.step else m["ts_mono_ns"] / 1e9
            roads.append((t, {"type": "road", "plan": m.get("plan")}))
    if not roads:
        sys.exit("no road messages in %s" % a.road)
    sel = L.EgoLightSelector()
    rows = []
    ri = 0
    n = 0
    for m in read_lines(a.lights):
        if m.get("type") != "lights":
            continue
        n += 1
        t = ((n - 1) * a.step + 1) / a.fps if a.step else m["ts_mono_ns"] / 1e9
        if t > roads[-1][0] + 1.0:
            break
        m["ts_mono_ns"] = int(t * 1e9)
        while ri + 1 < len(roads) and roads[ri + 1][0] <= t:
            ri += 1
        road = roads[ri][1] if abs(roads[ri][0] - t) < 1.0 else None
        fw, fh = (m.get("frame") or [1280, 720])[:2]
        f = a.focal or (calib or {}).get("focal_px") or 690.0
        intr = (float(f), fw / 2.0, fh / 2.0, fw, fh)
        m.pop("raw", None)
        if a.stateless:
            m["lights"] = [li for li in m.get("lights") or [] if not li.get("coast")]
            L.annotate(m, road, calib, intr)
        else:
            sel.annotate(m, road, calib, intr)
        ego = m.get("ego")
        rows.append((t, len(m.get("lights") or []), ego))
        if a.events and ego:
            print("%7.2f s  #%s %-10s %5.1f m lat %+5.1f rel %.2f%s" % (t, ego["track_id"], ego["state"], ego["dist_m"],
                                                                      ego["lateral_m"], ego["relevance"],
                                                                      "  held" if ego.get("held") else ""))
    if len(rows) < 2:
        sys.exit("no lights messages")
    dur = rows[-1][0] - rows[0][0]
    dt = dur / (len(rows) - 1)
    on = [r[2] is not None for r in rows]
    changes = sum(1 for x, y in zip(on, on[1:]) if x != y)
    gaps, run = [], 0
    seen_on = False
    for x in on:
        if x:
            if seen_on and run:
                gaps.append(run * dt)
            run, seen_on = 0, True
        else:
            run += 1
    short = sum(1 for g in gaps if g < 1.5)
    dist = sorted(r[2]["dist_m"] for r in rows if r[2] is not None)
    held = sum(1 for r in rows if r[2] is not None and r[2].get("held"))
    q = (lambda p: dist[min(len(dist) - 1, int(p * len(dist)))]) if dist else (lambda p: float("nan"))
    print("%s%d lights messages, %.1f min, %.2f Hz: any light %.1f %%, ego light %.1f %% (held %.1f %% of it), "
          "ego changes %.1f/min, gaps < 1.5 s: %d (%.1f/min), ego dist p10/p50/p90 %.0f/%.0f/%.0f m" % (
              (a.name + ": ") if a.name else "", len(rows), dur / 60.0, 1.0 / dt,
              100.0 * sum(1 for r in rows if r[1]) / len(rows), 100.0 * sum(on) / len(on),
              100.0 * held / max(1, sum(on)), changes / (dur / 60.0), short, short / (dur / 60.0),
              q(0.1), q(0.5), q(0.9)))


if __name__ == "__main__":
    main()
