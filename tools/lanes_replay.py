#!/usr/bin/env python3
"""Replay recorded `road` messages through the road-geometry tracker (api/roadsense_api/lanes.py) and measure it.

    tools/lanes_replay.py ROAD.jsonl[.gz] [--every N] [--lanes-total N --lanes-forward N] [--cfg k=v ...] [--out LANES.jsonl]

ROAD is a file of `road` JSON lines (modeld, `modeld_replay`, a /ws capture; other lines are skipped, `ego` and
`map` lines are fed to the tracker in order). `--every N` uses every Nth road message (a 20 fps replay -> 10 Hz
with N = 2, about what the reference phone delivers). `--lanes-total/--lanes-forward` feed a constant map
prior (a tagged lane count with a confident match), to see what the prior does on a ride without a position.

Printed per line, raw against tracked, only while the car moves (v_ego > 2 m/s):
  on %            share of the frames the line is shown (raw: prob >= 0.3, the threshold consumers used)
  changes/min     on/off changes per minute
  <1 s            shown or hidden spans shorter than one second (the flicker)
  |dy| p50 / p95  frame-to-frame change of the lateral position at 10 / 30 / 60 m while shown, metres
                  (for the tracked line after taking out the car's own motion, as for the raw one: both are
                  what a display draws)
  ema             the same for the raw line behind a 0.3 s low-pass (what the MFD scene did so far)
  lag             tracked minus a zero-phase reference (centred 1 s median of the raw line) at 10 / 30 m:
                  RMS over all frames and over the frames where the reference moves faster than 0.5 m/s
"""
import argparse
import gzip
import json
import zlib
import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "api"))

from roadsense_api.lanes import LINE_NAMES, RoadGeometryTracker      # noqa: E402

NS = 10 ** 9
XS = (10.0, 30.0, 60.0)
RAW_THR = 0.3


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
    except (EOFError, OSError, zlib.error) as e:     # a capture cut in the middle of a gzip stream
        print("note: %s: %s" % (path, e), file=sys.stderr)


def spans(on):
    """(changes, number of spans shorter than 1 s given as frames-per-second later) of a boolean series."""
    on = np.asarray(on, bool)
    tr = np.flatnonzero(on[1:] != on[:-1])
    return tr, np.diff(np.concatenate(([0], tr + 1, [len(on)])))


def pct(a, q):
    return float(np.percentile(a, q)) if len(a) else float("nan")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("road")
    ap.add_argument("--every", type=int, default=1)
    ap.add_argument("--fps", type=float, default=20.0)
    ap.add_argument("--lanes-total", type=int)
    ap.add_argument("--lanes-forward", type=int)
    ap.add_argument("--cfg", action="append", default=[], help="tracker constant, key=value")
    ap.add_argument("--out", help="write the `lanes` messages here (JSON lines)")
    ap.add_argument("--max", type=int, default=0)
    a = ap.parse_args()
    cfg = {}
    for kv in a.cfg:
        k, v = kv.split("=", 1)
        cfg[k] = json.loads(v)
    trk = RoadGeometryTracker(cfg)
    out = open(a.out, "w") if a.out else None

    raw_p, raw_y, trk_on, trk_y, vv, tt = [], [], [], [], [], []
    n = 0
    for msg in read_lines(a.road):
        ty = msg.get("type")
        if ty == "ego":
            trk.on_ego(msg)
            continue
        if ty == "map":
            trk.on_map(msg)
            continue
        if ty != "road":
            continue
        n += 1
        if (n - 1) % a.every:
            continue
        if a.max and len(tt) >= a.max:
            break
        if "ts_mono_ns" not in msg:
            msg["ts_mono_ns"] = int(msg.get("seq", n) / a.fps * NS)
        if a.lanes_total:
            fwd = a.lanes_forward or a.lanes_total
            trk.on_map({"type": "map", "ts_mono_ns": msg["ts_mono_ns"], "available": True,
                        "match": {"confidence": 0.9},
                        "road": {"lanes": {"total": a.lanes_total, "forward": fwd,
                                           "backward": a.lanes_total - fwd, "source": "tagged"}}})
        o = trk.on_road(msg)
        if o is None:
            continue
        if out:
            out.write(json.dumps(o, separators=(",", ":")) + "\n")
        ll = list(msg.get("lane_lines") or []) + list(msg.get("road_edges") or [])
        x = np.asarray(o["x"])
        rp, ry, ton, ty_ = [], [], [], []
        for i in range(6):
            src = ll[i] if i < len(ll) else None
            if src is None:
                rp.append(0.0)
                ry.append([math.nan] * 3)
            else:
                rp.append(float(src.get("prob", 1.0)))
                ry.append([float(np.interp(xx, src["x"], src["y"])) for xx in XS])
            t = (o["lines"] + o["edges"])[i]
            ton.append(t["present"])
            ty_.append([float(np.interp(xx, x, t["y"])) for xx in XS] if t["present"] else [math.nan] * 3)
        raw_p.append(rp)
        raw_y.append(ry)
        trk_on.append(ton)
        trk_y.append(ty_)
        vv.append(float(msg.get("v_ego") or 0.0))
        tt.append(msg["ts_mono_ns"] / NS)
    if out:
        out.close()
    if len(tt) < 10:
        sys.exit("no road messages with lines in %s" % a.road)
    raw_p, raw_y, trk_on, trk_y = map(np.asarray, (raw_p, raw_y, trk_on, trk_y))
    vv, tt = np.asarray(vv), np.asarray(tt)
    dtm = float(np.median(np.diff(tt)))
    hz = 1.0 / dtm
    minutes = (tt[-1] - tt[0]) / 60.0
    mov = vv > 2.0
    one_s = max(2, int(round(hz)))
    print("%s: %d road messages, %.1f Hz, %.1f min, moving %.0f %%, re-initialisations %d" %
          (os.path.basename(a.road), len(tt), hz, minutes, 100 * mov.mean(), trk.snaps))
    # 0.3 s low-pass of the raw line: what the display did so far
    al = 1.0 - math.exp(-dtm / 0.30)
    half = one_s // 2
    tot = {"raw": [0, 0], "trk": [0, 0]}
    for i, name in enumerate(LINE_NAMES):
        raw_on = raw_p[:, i] >= RAW_THR if i < 4 else np.ones(len(tt), bool)
        rows = []
        ema = np.full((len(tt), 3), np.nan)
        cur = None
        for k in range(len(tt)):
            if not raw_on[k]:
                cur = None
                continue
            cur = raw_y[k, i].copy() if cur is None else cur + (raw_y[k, i] - cur) * al
            ema[k] = cur
        for label, on, y in (("raw", raw_on, raw_y[:, i]), ("ema", raw_on, ema), ("trk", trk_on[:, i], trk_y[:, i])):
            tr, seg = spans(on)
            both = on[1:] & on[:-1] & mov[1:]
            dy = np.abs(np.diff(y, axis=0))[both]
            rows.append((label, 100.0 * on.mean(), len(tr) / minutes, int((seg < one_s).sum()),
                         [(pct(dy[:, k], 50), pct(dy[:, k], 95)) for k in range(3)]))
            if label in tot and i < 4:
                tot[label][0] += len(tr)
                tot[label][1] += int((seg < one_s).sum())
        # lag against a centred 1 s median of the raw line
        ref = np.full((len(tt), 3), np.nan)
        for k in range(half, len(tt) - half):
            w = raw_y[k - half:k + half + 1, i]
            if raw_on[k - half:k + half + 1].all():
                ref[k] = np.median(w, axis=0)
        err = trk_y[:, i] - ref
        rate = np.abs(np.gradient(ref, axis=0)) / dtm
        lag = []
        for k in range(2):
            e = err[:, k][np.isfinite(err[:, k]) & mov]
            ef = err[:, k][np.isfinite(err[:, k]) & mov & (rate[:, k] > 0.5)]
            lag.append((float(np.sqrt(np.mean(e ** 2))) if len(e) else math.nan,
                        float(np.sqrt(np.mean(ef ** 2))) if len(ef) else math.nan, len(ef)))
        print("  %s" % name)
        for label, onp, cpm, short, dy in rows:
            if label == "ema":
                print("    ema                                    |dy| p50/p95 @10 %.3f/%.3f @30 %.3f/%.3f @60 %.3f/%.3f" %
                      tuple(v for pair in dy for v in pair))
            else:
                print("    %s  on %5.1f %%  changes/min %6.1f  <1 s %4d  |dy| p50/p95 @10 %.3f/%.3f @30 %.3f/%.3f @60 %.3f/%.3f" %
                      ((label, onp, cpm, short) + tuple(v for pair in dy for v in pair)))
        print("    lag   rms @10 %.3f m (moving line %.3f m, n %d)   @30 %.3f m (moving line %.3f m, n %d)" %
              (lag[0] + lag[1]))
    print("  lane lines together: raw %.1f changes/min, %d spans < 1 s; tracked %.1f changes/min, %d spans < 1 s" %
          (tot["raw"][0] / minutes, tot["raw"][1], tot["trk"][0] / minutes, tot["trk"][1]))


if __name__ == "__main__":
    main()
