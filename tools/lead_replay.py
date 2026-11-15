#!/usr/bin/env python3
"""Replay recorded `road` messages through the lead tracker (api/roadsense_api/lead.py) and measure it.

    tools/lead_replay.py ROAD.jsonl[.gz] [--ego none|surrogate] [--every N] [--csv OUT.csv] [--events]

ROAD is a file of `road` JSON lines (modeld, `modeld_replay`, a /ws capture; other lines are skipped; only
`seq`, `ts_mono_ns`, `v_ego`, `leads[0]` are needed; a file without ts_mono_ns gets seq / --fps).
`ego` lines (mapd's relay of the vehicle motion) in the same file are fed to the tracker in order.

--ego none        vision only: the model's own speed estimate is the ego speed (what a ride video allows)
--ego surrogate   stand-in for wheel speeds where none were recorded: the model's v_ego as a 0.3 s median,
                  and exactly 0 with the standstill flag while that stays under 0.15 m/s for 0.5 s. NOT a
                  measurement: it exercises the ego path of the tracker on real lead output.
--every N         use every Nth road message (20 fps replay -> 10 Hz with N = 2)

Printed: per standstill (ego speed < 0.1 m/s for >= 3 s with a lead inside 15 m, lead speed |v| < 0.15)
the spread of the raw and the tracked distance and how often the value rounded to whole metres changes;
the tracked distance against a zero-phase reference (centred 1 s median of the raw distance) by range band
and while the gap closes fast (lag); the `departed` events with their delay; the tracker's events.
"""
import argparse
import gzip
import json
import math
import os
import statistics
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "api"))

from roadsense_api.lead import LeadTracker      # noqa: E402

NS = 10 ** 9


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
    except (EOFError, OSError) as e:                 # a capture cut in the middle of a gzip stream
        print("note: %s: %s" % (path, e), file=sys.stderr)


class Surrogate:
    """Wheel-speed stand-in from the model's v_ego (see the module docstring)."""

    def __init__(self):
        self.hist = []
        self.still_s = 0.0
        self.ts = None
        self.seq = 0

    def step(self, road):
        ts, v = road["ts_mono_ns"], float(road.get("v_ego") or 0.0)
        dt = 0.0 if self.ts is None else (ts - self.ts) / NS
        self.ts = ts
        self.hist = (self.hist + [(ts, v)])[-40:]
        win = sorted(x for t, x in self.hist if ts - t <= 0.3 * NS)        # median over 0.3 s: no single-frame blips
        vm = win[len(win) // 2]
        self.still_s = self.still_s + dt if vm < 0.15 else 0.0
        still = self.still_s >= 0.5
        self.seq += 1
        return {"type": "ego", "seq": self.seq, "ts_mono_ns": ts, "speed_mps": 0.0 if still else round(vm, 3),
                "standstill": still, "source": "vehicle", "tag": "surrogate"}


def run(path, ego="none", every=1, fps=20.0, cfg=None):
    """Returns rows: dicts with t, raw fields and the tracker output per road message used."""
    tr = LeadTracker(cfg)
    sur = Surrogate() if ego == "surrogate" else None
    rows, n = [], 0
    for m in read_lines(path):
        t = m.get("type")
        if t == "ego":
            tr.on_ego(m)
            continue
        if t != "road":
            continue
        n += 1
        if (n - 1) % every:
            continue
        if not isinstance(m.get("ts_mono_ns"), int):
            m["ts_mono_ns"] = int((m.get("seq") or n) / fps * NS)
        if sur:
            tr.on_ego(sur.step(m))
        out = tr.on_road(m)
        if out is None:
            continue
        ld = (m.get("leads") or [{}])[0]
        rows.append({"t": m["ts_mono_ns"] / NS, "v_ego": float(m.get("v_ego") or 0.0),
                     "p": ld.get("prob") or 0.0, "x": (ld.get("x") or [math.nan])[0], "v": (ld.get("v") or [math.nan])[0],
                     "out": out})
    return rows


def segments(mask, ts, min_s):
    out, i, n = [], 0, len(mask)
    while i < n:
        if mask[i]:
            j = i
            while j < n and mask[j]:
                j += 1
            if ts[j - 1] - ts[i] >= min_s:
                out.append((i, j))
            i = j
        else:
            i += 1
    return out


def flips(vals):
    r = [math.floor(v + 0.5) for v in vals]
    return sum(1 for a, b in zip(r, r[1:]) if a != b)


def standstills(rows, min_s=3.0):
    """Stops behind a standing lead, cut where the lead starts to move (raw v) so they are constant-gap spans."""
    ts = [r["t"] for r in rows]
    mask = [r["v_ego"] < 0.1 and r["p"] > 0.5 and r["x"] < 15.0 and abs(r["v"]) < 0.15 for r in rows]
    # bridge single-frame blips of the model's v_ego / v
    for i in range(1, len(mask) - 1):
        if not mask[i] and mask[i - 1] and mask[i + 1]:
            mask[i] = True
    res = []
    for i, j in segments(mask, ts, min_s):
        k = i + next((n for n in range(j - i) if ts[i + n] - ts[i] >= 1.0), 0)     # skip the first second (settling)
        raw = [rows[n]["x"] for n in range(k, j)]
        trk = [rows[n]["out"]["dist_m"] for n in range(k, j) if rows[n]["out"]["present"]]
        if len(trk) < 0.9 * len(raw) or len(raw) < 10:
            continue
        res.append({"t0": ts[k], "t1": ts[j - 1], "n": len(raw), "raw_med": statistics.median(raw),
                    "raw_sd": statistics.pstdev(raw), "raw_min": min(raw), "raw_max": max(raw), "raw_flips": flips(raw),
                    "trk_med": statistics.median(trk), "trk_sd": statistics.pstdev(trk), "trk_flips": flips(trk),
                    "trk_range": max(trk) - min(trk),
                    "hold_share": sum(1 for n in range(k, j) if rows[n]["out"]["mode"] == "hold") / (j - k)})
    return res


def reference(rows, half_s=0.5):
    """Centred median of the raw distance (zero phase) where the lead is solid; None elsewhere."""
    ts = [r["t"] for r in rows]
    ref = [None] * len(rows)
    lo = hi = 0
    for i, r in enumerate(rows):
        while ts[lo] < ts[i] - half_s:
            lo += 1
        while hi < len(rows) and ts[hi] <= ts[i] + half_s:
            hi += 1
        win = rows[lo:hi]
        if len(win) >= 5 and all(w["p"] > 0.7 for w in win) and ts[hi - 1] - ts[lo] > 1.5 * half_s:
            xs = [w["x"] for w in win]
            if max(xs) - min(xs) < max(3.0, 0.25 * r["x"]):          # no identity change inside the window
                ref[i] = statistics.median(xs)
    return ref


def accuracy(rows):
    ref = reference(rows)
    bands = ((0, 6), (6, 15), (15, 30), (30, 60), (60, 200))
    out = []
    for lo, hi in bands:
        e_raw, e_trk = [], []
        for r, x in zip(rows, ref):
            if x is None or not (lo <= x < hi) or not r["out"]["present"] or r["out"]["mode"] == "coast":
                continue
            e_raw.append(r["x"] - x)
            e_trk.append(r["out"]["dist_m"] - x)
        if len(e_raw) >= 50:
            out.append({"band": "%d-%d m" % (lo, hi), "n": len(e_raw),
                        "raw_rms": math.sqrt(sum(e * e for e in e_raw) / len(e_raw)),
                        "trk_rms": math.sqrt(sum(e * e for e in e_trk) / len(e_trk)),
                        "trk_bias": sum(e_trk) / len(e_trk)})
    # lag while the gap closes fast: bias / closing speed, near (where it matters) and far
    lags = {"under 40 m": [], "40 m and beyond": []}
    for i in range(10, len(rows) - 10):
        a, b = ref[i - 10], ref[i + 10]
        if a is None or b is None or ref[i] is None or not rows[i]["out"]["present"]:
            continue
        rate = (b - a) / (rows[i + 10]["t"] - rows[i - 10]["t"])
        if rate < -2.0 and rows[i]["out"]["mode"] == "track":
            lags["under 40 m" if ref[i] < 40.0 else "40 m and beyond"].append((rows[i]["out"]["dist_m"] - ref[i]) / -rate)
    return out, {k: ((statistics.mean(v) if len(v) >= 20 else None), len(v)) for k, v in lags.items()}


def departures(rows):
    """For every `departed` event: seconds since the raw lead speed last was under 0.05 m/s (the lead began
    to roll), and how far the raw distance (0.5 s median after the event) is from the held value."""
    out, seen = [], None
    for i, r in enumerate(rows):
        e = r["out"]["event"]
        if not e or e["kind"] != "departed" or e["ts_mono_ns"] == seen:
            continue
        seen = e["ts_mono_ns"]
        j = i
        while j > 0 and abs(rows[j]["v"]) >= 0.05:
            j -= 1
        held = rows[i - 1]["out"]["dist_m"] if i and rows[i - 1]["out"]["present"] else None
        after = [w["x"] for w in rows[i:i + 10]]
        out.append({"t": r["t"], "delay_s": r["t"] - rows[j]["t"],
                    "moved_m": None if held is None else statistics.median(after) - held,
                    "flag": r["out"]["departed"]})
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("road")
    ap.add_argument("--ego", choices=("none", "surrogate"), default="none")
    ap.add_argument("--every", type=int, default=1)
    ap.add_argument("--fps", type=float, default=20.0)
    ap.add_argument("--csv")
    ap.add_argument("--events", action="store_true")
    ap.add_argument("--set", action="append", default=[], metavar="KEY=VALUE", help="tracker config override")
    a = ap.parse_args()
    cfg = {k: json.loads(v) for k, v in (s.split("=", 1) for s in a.set)}
    rows = run(a.road, a.ego, a.every, a.fps, cfg)
    if not rows:
        raise SystemExit("no road messages in %s" % a.road)
    dur = rows[-1]["t"] - rows[0]["t"]
    present = sum(1 for r in rows if r["out"]["present"])
    print("%s: %d road messages, %.0f s (%.1f Hz), ego %s; lead present %.0f %%, in hold %.0f %%" % (
        os.path.basename(a.road), len(rows), dur, (len(rows) - 1) / dur if dur else 0.0, a.ego,
        100.0 * present / len(rows), 100.0 * sum(1 for r in rows if r["out"]["mode"] == "hold") / len(rows)))
    st = standstills(rows)
    print("standstill behind a standing lead: %d spans, %.0f s" % (len(st), sum(s["t1"] - s["t0"] for s in st)))
    for s in st:
        print("  %7.1f-%7.1f s  raw %5.2f m sd %.3f range %.2f flips %3d | tracked %5.2f m sd %.3f range %.2f flips %d  hold %.0f %%"
              % (s["t0"], s["t1"], s["raw_med"], s["raw_sd"], s["raw_max"] - s["raw_min"], s["raw_flips"],
                 s["trk_med"], s["trk_sd"], s["trk_range"], s["trk_flips"], 100 * s["hold_share"]))
    if st:
        T = sum(s["t1"] - s["t0"] for s in st)
        print("  total: raw sd %.3f m (rms over spans), %d flips (%.1f / min) | tracked sd %.3f m, %d flips (%.1f / min)" % (
            math.sqrt(sum(s["raw_sd"] ** 2 * s["n"] for s in st) / sum(s["n"] for s in st)),
            sum(s["raw_flips"] for s in st), 60.0 * sum(s["raw_flips"] for s in st) / T,
            math.sqrt(sum(s["trk_sd"] ** 2 * s["n"] for s in st) / sum(s["n"] for s in st)),
            sum(s["trk_flips"] for s in st), 60.0 * sum(s["trk_flips"] for s in st) / T))
    acc, lags = accuracy(rows)
    print("against the zero-phase reference (centred 1 s median of the raw distance):")
    for b in acc:
        print("  %-9s n %6d  raw rms %.3f m | tracked rms %.3f m  bias %+.3f m" % (
            b["band"], b["n"], b["raw_rms"], b["trk_rms"], b["trk_bias"]))
    print("  lag while closing faster than 2 m/s: " + "; ".join(
        "%s %s (n %d)" % (k, "%+.0f ms" % (1e3 * v) if v is not None else "-", n) for k, (v, n) in lags.items()))
    dep = departures(rows)
    if dep:
        print("departures (event, s after the lead began to roll, raw distance gained): " + "; ".join(
            "%.1f s: %.2f s, %s m" % (d["t"], d["delay_s"], "-" if d["moved_m"] is None else "%+.2f" % d["moved_m"]) for d in dep))
    ev = {}
    last = None
    for r in rows:
        e = r["out"]["event"]
        if e and (e["kind"], e["ts_mono_ns"]) != last:
            last = (e["kind"], e["ts_mono_ns"])
            ev[e["kind"]] = ev.get(e["kind"], 0) + 1
            if a.events:
                print("  event %7.1f s %s" % (e["ts_mono_ns"] / NS, e["kind"]))
    print("events:", ", ".join("%s %d" % kv for kv in sorted(ev.items())) or "none")
    if a.csv:
        with open(a.csv, "w") as f:
            f.write("t,v_ego,prob,raw_x,raw_v,present,mode,dist,dist_std,v_rel,v_lead,a_lead,departed,track_id\n")
            for r in rows:
                o = r["out"]
                f.write(",".join("" if v is None else str(v) for v in (
                    round(r["t"], 3), r["v_ego"], round(r["p"], 3), round(r["x"], 3), round(r["v"], 3), int(o["present"]),
                    o["mode"], o["dist_m"], o["dist_std_m"], o["v_rel_mps"], o["v_lead_mps"], o["a_lead_mps2"],
                    int(o["departed"]), o["track_id"])) + "\n")


if __name__ == "__main__":
    main()
