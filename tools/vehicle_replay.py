#!/usr/bin/env python3
"""Replay a recorded vehicle input stream through mapd's engine, offline, and measure dead reckoning.

    tools/vehicle_replay.py --input DRIVE.ndjson --session REC_DIR [--pack PACK] [--gaps 30,60] [options]

--input     vehicle input messages (docs/VEHICLE_API.md), one JSON object per line, each with `ts_mono_ns` on
            any one clock: what a feeder sent, or what a host's own converter writes from its logs.
--session   a drive-recorder session (directory or messages.jsonl.gz) of the same drive. Its `map` messages
            give the reference: where roadsense was while it had a fix (`state: good`). When the input stream
            has no `gnss` messages (a host that does not log its fixes), those positions are also fed back as
            fixes, once a second, position only (no receiver speed, no course): the filter then has to find
            its heading and scale on its own, as with a bare receiver. Times are aligned through the
            session's `ts_real_ns`; `--input-clock real` says that the stream's `ts_mono_ns` is wall-clock ns
            (default), `mono` that it is the recorder host's CLOCK_MONOTONIC.

What it prints:

* the replay itself: messages accepted / rejected per type, engine time per message (the cost of the input
  path), position states, the calibration estimates at the end;
* with --gaps: the fix is withheld for windows of the given lengths (s), placed where the recording had a
  fix throughout (so the reference is real), and the position error at the end of each window is measured:
  dead-reckoning drift. `--variant NAME:signal=quality,..` runs the same windows with changed signal
  qualities, e.g. `gyro:yaw_rate_rps=uncalibrated` (use the gyroscope) against `wheels:yaw_rate_rps=unverified`.
  `--scale signal=factor` multiplies a numeric motion signal (what a scale error would do).
"""
import argparse
import gzip
import json
import os
import statistics
import sys
import time
import zlib

sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "mapd"))

from roadsense_mapd import geo                      # noqa: E402
from roadsense_mapd.engine import Engine            # noqa: E402
from roadsense_mapd.rsmap import Pack               # noqa: E402

NS = 10 ** 9


def read_lines(path):
    if os.path.isdir(path):
        path = os.path.join(path, "messages.jsonl.gz")
    op = gzip.open if path.endswith(".gz") else open
    with op(path, "rt", errors="replace") as f:
        try:
            for line in f:
                line = line.strip()
                if line.startswith("{"):
                    try:
                        yield json.loads(line)
                    except ValueError:
                        continue
        except (EOFError, OSError, zlib.error):  # a recording cut off by a power loss
            return


def load_session(path, clock):
    """Reference positions [(t_ns, lat, lon, hacc)] while the recording had a fix, and the pseudo fixes."""
    ref, fixes = [], []
    last_fix_t = None
    for m in read_lines(path):
        if m.get("type") != "map":
            continue
        pos, g = m.get("position"), (m.get("input") or {}).get("gnss") or {}
        t = m.get("ts_real_ns") if clock == "real" else m.get("ts_mono_ns")
        if not isinstance(t, int) or not pos or m.get("state") != "good" or g.get("fix") in (None, "none"):
            continue
        ref.append((t, pos["lat"], pos["lon"], pos.get("hacc_m")))
        age = g.get("age_s")
        if isinstance(age, (int, float)) and age <= 1.0:
            t_fix = t - int(age * NS)
            if last_fix_t is None or t_fix - last_fix_t > 0.5 * NS:
                last_fix_t = t_fix
                fixes.append({"type": "gnss", "ts_mono_ns": t, "lat": pos["lat"], "lon": pos["lon"],
                              "hacc_m": max(g.get("hacc_m") or 5.0, 3.0), "source": "recorded-position"})
    return ref, fixes


def stretches(ref, max_step_s=1.6):
    """[(t0, t1)] where the reference is continuous."""
    out, t0, prev = [], None, None
    for t, *_ in ref:
        if prev is None or t - prev > max_step_s * NS:
            if t0 is not None:
                out.append((t0, prev))
            t0 = t
        prev = t
    if t0 is not None:
        out.append((t0, prev))
    return out


def place_windows(ref, lengths, lead_s=12.0, rest_s=10.0):
    """Windows [(t0, t1)] without a fix. A window starts after `lead_s` of continuous reference and ends where
    the reference has been continuous for 3 s (so its end is not the first, jumpy fix after a real outage);
    what the recording had in between does not matter, the fixes inside are withheld anyway."""
    if not ref:
        return []
    have = {}
    for a, b in stretches(ref):
        have[a] = b
    spans = sorted(have.items())

    def inside(t0, t1):
        return any(a <= t0 and t1 <= b for a, b in spans)

    wins = []
    t, end = ref[0][0], ref[-1][0]
    i = 0
    while t < end:
        L = lengths[i % len(lengths)]
        t1 = t + int(L * NS)
        if inside(t - int(lead_s * NS), t) and inside(t1 - 3 * NS, t1):
            wins.append((t, t1))
            t = t1 + int(rest_s * NS)
            i += 1
        else:
            t += NS
    return wins


def ref_at(ref, t, tol_s=0.8):
    lo, hi = 0, len(ref) - 1
    while lo < hi:
        mid = (lo + hi) // 2
        if ref[mid][0] < t:
            lo = mid + 1
        else:
            hi = mid
    best = min((ref[i] for i in (lo - 1, lo) if 0 <= i < len(ref)), key=lambda r: abs(r[0] - t))
    return best if abs(best[0] - t) <= tol_s * NS else None


def run(stream, fixes, ref, pack, wins, quality, scale, cfg, step_s=0.5):
    eng = Engine(pack, cfg)
    msgs = sorted(stream + fixes, key=lambda m: m["ts_mono_ns"])
    counts, rejected, states = {}, {}, {}
    results = []
    win_i = 0
    nxt = None
    t_proc = 0.0
    dist_in_win = None
    for m in msgs:
        t = m["ts_mono_ns"]
        while win_i < len(wins) and t > wins[win_i][1]:
            # end of a window: where is the filter, where was the car
            w0, w1 = wins[win_i]
            out = eng.step(w1)
            r = ref_at(ref, w1)
            pos = out.get("position")
            if r is not None:
                err = geo.dist_m(pos["lat"], pos["lon"], r[1], r[2]) if pos else None
                results.append({"t0": w0, "len_s": (w1 - w0) / NS, "err_m": err, "state": out["state"],
                                "driven_m": None if dist_in_win is None else eng.odo - dist_in_win,
                                "hacc_m": pos["hacc_m"] if pos else None,
                                "matched": bool((out.get("match") or {}).get("matched"))})
            win_i += 1
            dist_in_win = None
        in_win = win_i < len(wins) and wins[win_i][0] <= t <= wins[win_i][1]
        if in_win and dist_in_win is None:
            dist_in_win = eng.odo
        if m["type"] == "gnss" and in_win:
            continue
        if m["type"] == "motion" and (quality or scale):
            m = dict(m)
            if quality:
                m["quality"] = dict(m.get("quality") or {}, **quality)
            for k, f in scale.items():
                if isinstance(m.get(k), (int, float)):
                    m[k] = m[k] * f
        if nxt is None:
            nxt = t
        while t >= nxt:
            out = eng.step(nxt)
            states[out["state"]] = states.get(out["state"], 0) + 1
            nxt += int(step_s * NS)
            if t - nxt > 10 * NS:
                nxt = t
        c0 = time.perf_counter()
        err = eng.on_input(m, t)
        t_proc += time.perf_counter() - c0
        counts[m["type"]] = counts.get(m["type"], 0) + 1
        if err:
            rejected[err] = rejected.get(err, 0) + 1
    return {"engine": eng, "counts": counts, "rejected": rejected, "states": states, "results": results,
            "us_per_msg": 1e6 * t_proc / max(1, sum(counts.values())), "t_end": msgs[-1]["ts_mono_ns"] if msgs else 0}


def pct(v, p):
    v = sorted(v)
    return v[min(len(v) - 1, int(p * len(v)))] if v else float("nan")


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--input", required=True, action="append", help="vehicle input NDJSON (may be given more than once)")
    ap.add_argument("--session", action="append", default=[], help="recorder session of the same drive (may repeat)")
    ap.add_argument("--input-clock", choices=("real", "mono"), default="real")
    ap.add_argument("--pack", help="map pack (.rsmap): map matching and the map feedback while dead reckoning")
    ap.add_argument("--gaps", default="", help="lengths (s) of the windows without a fix, e.g. 30,60")
    ap.add_argument("--variant", action="append", default=[],
                    help="NAME:signal=quality[,signal=quality] (default: one run named `as_fed`)")
    ap.add_argument("--scale", action="append", default=[], help="signal=factor applied to the motion input")
    ap.add_argument("--no-calibration", action="store_true", help="mapd `use_calibration: false`")
    ap.add_argument("--min-driven", type=float, default=50.0, help="a window counts as moving from this distance (m)")
    ap.add_argument("--json", help="write the per-window results here")
    a = ap.parse_args()

    stream = []
    for p in a.input:
        stream += [m for m in read_lines(p) if isinstance(m.get("ts_mono_ns"), int) and isinstance(m.get("type"), str)]
    ref, fixes = [], []
    for s in a.session:
        r, f = load_session(s, a.input_clock)
        ref += r
        fixes += f
    ref.sort()
    own_fixes = sum(1 for m in stream if m["type"] == "gnss")
    if own_fixes:
        fixes = []
    span = (max(m["ts_mono_ns"] for m in stream) - min(m["ts_mono_ns"] for m in stream)) / NS if stream else 0.0
    print("input: %d messages over %.0f s (%s); fixes: %s; reference: %d positions in %d stretches (%.0f s with a fix)"
          % (len(stream), span, ", ".join("%s %d" % (k, sum(1 for m in stream if m["type"] == k))
                                         for k in sorted({m["type"] for m in stream})),
             "%d in the stream" % own_fixes if own_fixes else "%d from the recorded positions" % len(fixes),
             len(ref), len(stretches(ref)), sum((b - c) / NS for c, b in stretches(ref))))
    pack = Pack(a.pack) if a.pack else None
    lengths = [float(x) for x in a.gaps.split(",") if x]
    wins = place_windows(ref, lengths) if lengths else []
    scale = {k: float(v) for k, v in (x.split("=") for x in a.scale)}
    variants = []
    for v in a.variant or ["as_fed:"]:
        name, _, spec = v.partition(":")
        variants.append((name, {k: q for k, q in (x.split("=") for x in spec.split(",") if x)}))
    cfg = {"use_calibration": False} if a.no_calibration else None
    table = {}
    for name, quality in variants:
        r = run(stream, fixes, ref, pack, wins, quality, scale, cfg)
        eng = r["engine"]
        print("\n== %s%s%s ==" % (name, " " + json.dumps(quality) if quality else "", " scale " + json.dumps(scale) if scale else ""))
        print("accepted: %s   rejected: %s" % (r["counts"], r["rejected"] or "none"))
        print("engine time per input message: %.1f us   states (steps): %s" % (r["us_per_msg"], r["states"]))
        v = eng.vehicle_message(r["t_end"], 0)
        print("yaw source at the end: %s   calibration: %s" % (v["motion"]["yaw_source"], json.dumps(v["calibration"])))
        for L in lengths:
            allw = [x for x in r["results"] if x["len_s"] == L]
            table.setdefault(name, {})[L] = allw
            for label, rows in (("moving", [x for x in allw if (x["driven_m"] or 0.0) >= a.min_driven]),
                                ("standing", [x for x in allw if (x["driven_m"] or 0.0) < 5.0])):
                lost = sum(1 for x in rows if x["err_m"] is None)
                rows = [x for x in rows if x["err_m"] is not None]
                if not rows:
                    print("gap %3.0f s, %-8s: no window" % (L, label))
                    continue
                e = [x["err_m"] for x in rows]
                rel = [100.0 * x["err_m"] / x["driven_m"] for x in rows if x["driven_m"] and x["driven_m"] >= a.min_driven]
                print("gap %3.0f s, %-8s: %2d windows, %4.0f m driven (median)  error at the end: median %.1f m, mean %.1f m, "
                      "p90 %.1f m, max %.1f m%s%s"
                      % (L, label, len(rows), statistics.median([x["driven_m"] or 0.0 for x in rows]),
                         statistics.median(e), statistics.fmean(e), pct(e, 0.9), max(e),
                         "   = %.1f %% of the distance (median)" % statistics.median(rel) if rel else "",
                         "   (%d more without a position)" % lost if lost else ""))
    if a.json:
        with open(a.json, "w") as f:
            json.dump(table, f)


if __name__ == "__main__":
    main()
