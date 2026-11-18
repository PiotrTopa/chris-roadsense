#!/usr/bin/env python3
"""Verdict tables from tools/vision_eval/evaluate.py results (lane L24-M).

    python tools/vision_eval/summarize.py RESULTS.json [--ref sc.13.8] [--tests vis.13.8,vis.17,vis.20]

Per metric: the mean over rides of the reference and of each test stream, the median of the per-ride ratio
test/ref, and a bootstrap 90 % interval (2000 resamples of the rides) of the mean per-ride difference. Verdict:
`better` / `worse` when the interval lies entirely on one side of zero AND the median ratio is more than 10 %
from 1; otherwise `same`. The replays are deterministic, so the spread that decides "same" is the spread between
rides (the effect has to show consistently across driving conditions), not run-to-run noise.
"""
import argparse
import json

import numpy as np

# metric -> (direction, label); direction -1 = lower is better, +1 = higher is better, 0 = report only
METRICS = [
    ("inner_line_changes_pm", -1, "inner lane-line presence changes / min (tracked)"),
    ("outer_line_changes_pm", -1, "outer lane-line presence changes / min (tracked)"),
    ("edge_changes_pm", -1, "road-edge presence changes / min (tracked)"),
    ("both_inner_present_pct", 0, "both inner lines present, % of runs"),
    ("ego_idx_changes_pm", -1, "ego-lane index changes / min (no map)"),
    ("ego_idx_known_pct", 0, "ego lane named, % of runs (no map)"),
    ("jitter_tr_10m", -1, "inner-line jitter at 10 m, m (tracked)"),
    ("jitter_tr_50m", -1, "inner-line jitter at 50 m, m (tracked)"),
    ("jitter_raw_10m", -1, "inner-line jitter at 10 m, m (raw model)"),
    ("jitter_raw_50m", -1, "inner-line jitter at 50 m, m (raw model)"),
    ("far_tracked_20-40", -1, "far-range error 20-40 m, median m (tracked)"),
    ("far_tracked_40-70", -1, "far-range error 40-70 m, median m (tracked)"),
    ("far_tracked_70-100", -1, "far-range error 70-100 m, median m (tracked)"),
    ("far_tracked_20-40/p90", -1, "far-range error 20-40 m, p90 m (tracked)"),
    ("far_tracked_40-70/p90", -1, "far-range error 40-70 m, p90 m (tracked)"),
    ("far_tracked_70-100/p90", -1, "far-range error 70-100 m, p90 m (tracked)"),
    ("far_raw_20-40", -1, "far-range error 20-40 m, median m (raw)"),
    ("far_raw_40-70", -1, "far-range error 40-70 m, median m (raw)"),
    ("far_raw_70-100", -1, "far-range error 70-100 m, median m (raw)"),
    ("lead_onoff_pm", -1, "lead on/off changes / min (tracked)"),
    ("lead_id_changes_pm", -1, "lead identity changes / min (tracked)"),
    ("lead_present_pct", 0, "lead present, % of runs"),
    ("lead_d_noise_0-15m", -1, "lead distance noise 0-15 m, m (tracked)"),
    ("lead_d_noise_15-30m", -1, "lead distance noise 15-30 m, m (tracked)"),
    ("lead_d_noise_30-60m", -1, "lead distance noise 30-60 m, m (tracked)"),
    ("lead_d_noise_60-200m", -1, "lead distance noise 60+ m, m (tracked)"),
    ("lead_v_noise", -1, "lead relative-speed noise, m/s (tracked)"),
    ("ego/label/right_pct_of_all", 1, "ego lane right, % of labelled frames (map count from label)"),
    ("ego/label/wrong_confident_pct", -1, "ego lane wrong and confident, % (map count from label)"),
    ("ego/none/right_pct_of_all", 1, "ego lane right, % of labelled frames (no map)"),
    ("ego/none/wrong_confident_pct", -1, "ego lane wrong and confident, % (no map)"),
]


def value(r, stream, key):
    if key.startswith("ego/"):
        _, mc, k = key.split("/")
        try:
            return float(r["egolane"][stream][mc][k])
        except (KeyError, TypeError, ValueError):
            return None
    s = r.get(stream) or {}
    if key.startswith("far_"):
        base, _, q = key.partition("/")
        v = s.get(base)
        if not v or v[2] < 30:
            return None
        return float(v[1] if q == "p90" else v[0])
    v = s.get(key)
    return None if v is None or (isinstance(v, float) and np.isnan(v)) else float(v)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("results")
    ap.add_argument("--ref", default="sc.13.8")
    ap.add_argument("--tests", default="vis.13.8,vis.17,vis.20")
    a = ap.parse_args()
    R = json.load(open(a.results))
    tests = a.tests.split(",")
    rng = np.random.default_rng(0)
    print(f"rides: {', '.join(R)}\n")
    hdr = "| metric | rides | " + a.ref + " | " + " | ".join(f"{t} (ratio, 90% CI of diff, verdict)" for t in tests) + " |"
    print(hdr)
    print("|" + "---|" * (3 + len(tests)))
    for key, direction, label in METRICS:
        row = [label]
        refv = None
        cells = []
        for t in tests:
            pairs = [(value(r, a.ref, key), value(r, t, key)) for r in R.values()]
            pairs = [(x, y) for x, y in pairs if x is not None and y is not None]
            if len(pairs) < 3:
                cells.append("n/a")
                continue
            x = np.array([p[0] for p in pairs]); y = np.array([p[1] for p in pairs])
            refv = (len(pairs), x.mean())
            d = y - x
            boots = [d[rng.integers(0, len(d), len(d))].mean() for _ in range(2000)]
            lo, hi = np.percentile(boots, [5, 95])
            with np.errstate(divide="ignore", invalid="ignore"):
                ratio = float(np.median(np.where(x != 0, y / x, np.nan)))
            if direction == 0:
                verdict = "-"
            elif (lo > 0 or hi < 0) and (not np.isfinite(ratio) or abs(ratio - 1) > 0.10):
                verdict = "**better**" if np.sign(d.mean()) == direction else "**worse**"
            else:
                verdict = "same"
            cells.append(f"{y.mean():.3g} (x{ratio:.2f}, [{lo:+.3g}, {hi:+.3g}], {verdict})")
        row += [str(refv[0]) if refv else "-", f"{refv[1]:.3g}" if refv else "-"] + cells
        print("| " + " | ".join(row) + " |")


if __name__ == "__main__":
    main()
