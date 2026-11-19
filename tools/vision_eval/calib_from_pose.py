#!/usr/bin/env python3
"""Camera pitch / yaw from the road model's recorded ego-motion: modeld's online estimate (modeld/calibrator.h)
run after the fact on `road` + `calib` messages (lane L32).

  usage: calib_from_pose.py MESSAGES.jsonl[.gz] | SESSION_DIR ... [--block 100] [--per-file]

Reads the `calib` message (the calibration the frames were warped with) and every `road` message's `pose`. The
same gates and the same arithmetic as modeld: vx >= 6.7 m/s, |yaw rate| <= 1 deg/s, atan2(vy_std, vx) <= 0.5 deg;
  yaw = yaw_used + atan2(vy, vx),   pitch = pitch_used - atan2(vz, vx);
block medians of 100 accepted samples; estimate = median of the blocks, spread = 1.4826 MAD of the blocks.
A truncated gzip (a recording cut by power-off) is read up to the damage.
Prints per input: the configured values, the estimate with its spread and the calib.json line to install by hand.
"""
import argparse
import gzip
import json
import math
import os
import statistics as st
import sys

MIN_SPEED, MAX_YAW_RATE, MAX_ANGLE_STD = 6.7, 0.01745, 0.00873


def lines(path):
    if os.path.isdir(path):
        path = os.path.join(path, "messages.jsonl.gz")
    f = gzip.open(path, "rt") if path.endswith(".gz") else open(path)
    try:
        for ln in f:
            yield ln
    except (EOFError, OSError, gzip.BadGzipFile, Exception) as e:   # cut recording: keep what was read
        print(f"   ({os.path.basename(os.path.dirname(path)) or path}: read stopped: {str(e)[:50]})", file=sys.stderr)


def estimate(paths, block):
    used = {"pitch": 0.0, "yaw": 0.0}
    cfg = None
    seen = acc = 0
    cur_p, cur_y, blocks = [], [], []
    model = None
    for p in paths:
        for ln in lines(p):
            if ln.startswith('{"type":"calib"'):
                c = json.loads(ln)
                used = {"pitch": c.get("pitch") or 0.0, "yaw": c.get("yaw") or 0.0}
                cfg = (c.get("online") or {}).get("config") or cfg or dict(used)
                cfg = dict(cfg, focal_px=c.get("focal_px"), height_m=c.get("height_m"), roll=c.get("roll"))
                continue
            if not ln.startswith('{"type":"road"'):
                continue
            m = json.loads(ln)
            q = m.get("pose")
            if not q:
                continue
            model = m.get("road_model") or "supercombo"
            seen += 1
            vx = q["vx"]
            if vx < MIN_SPEED or abs(q["yaw_rate"]) > MAX_YAW_RATE or math.atan2(q["vy_std"], vx) > MAX_ANGLE_STD:
                continue
            acc += 1
            cur_p.append(used["pitch"] - math.atan2(q["vz"], vx))
            cur_y.append(used["yaw"] + math.atan2(q["vy"], vx))
            if len(cur_p) >= block:
                blocks.append((st.median(cur_p), st.median(cur_y)))
                cur_p, cur_y = [], []
    return cfg, model, seen, acc, blocks


def report(name, cfg, model, seen, acc, blocks):
    d = math.degrees
    print(f"== {name}: road model {model}, {seen} runs, {acc} accepted, {len(blocks)} blocks")
    if cfg:
        print(f"   configured: pitch {cfg['pitch']:+.4f} rad ({d(cfg['pitch']):+.2f} deg), yaw {cfg['yaw']:+.4f} rad "
              f"({d(cfg['yaw']):+.2f} deg), focal {cfg.get('focal_px')}")
    if len(blocks) < 5:
        print("   fewer than 5 blocks: no estimate")
        return None
    ps, ys = [b[0] for b in blocks], [b[1] for b in blocks]
    p, y = st.median(ps), st.median(ys)
    sp = 1.4826 * st.median([abs(v - p) for v in ps])
    sy = 1.4826 * st.median([abs(v - y) for v in ys])
    n = len(blocks)
    print(f"   estimate:   pitch {p:+.4f} rad ({d(p):+.2f} deg), yaw {y:+.4f} rad ({d(y):+.2f} deg)")
    print(f"   blocks:     sigma pitch {d(sp):.2f} deg, yaw {d(sy):.2f} deg; range pitch {d(min(ps)):+.2f}..{d(max(ps)):+.2f}, "
          f"yaw {d(min(ys)):+.2f}..{d(max(ys)):+.2f} deg; standard error of the median about "
          f"{d(1.25 * sp / math.sqrt(n)):.2f} / {d(1.25 * sy / math.sqrt(n)):.2f} deg")
    if cfg:
        print(f"   off the configured value: pitch {d(p - cfg['pitch']):+.2f} deg, yaw {d(y - cfg['yaw']):+.2f} deg")
        print(f'   calib.json: {{"pitch": {p:.4f}, "yaw": {y:.4f}, "roll": {cfg.get("roll") or 0.0}, "height_m": {cfg.get("height_m") or 1.22}}}')
    return p, y, sp, sy


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("paths", nargs="+")
    ap.add_argument("--block", type=int, default=100)
    ap.add_argument("--per-file", action="store_true", help="one estimate per input instead of one over all")
    a = ap.parse_args()
    if a.per_file:
        for p in a.paths:
            report(os.path.basename(p.rstrip("/")), *estimate([p], a.block))
    else:
        report(", ".join(os.path.basename(p.rstrip("/")) for p in a.paths), *estimate(a.paths, a.block))


if __name__ == "__main__":
    sys.exit(main())
