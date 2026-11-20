#!/usr/bin/env python3
"""Tables of the envd rate study (lane L48-E) from rate_sim.py JSON outputs.

    rate_report.py sim_rides.json [sim_recs.json ...] [--sessions a,b] [--md]

Per candidate, pooled over the sessions (each phase of a schedule is one run; disagreement = time share, mean
over runs and the worst run): inferences per second, disagreement with the 1 Hz baseline per consumer, label
changes (baseline / candidate), extra changes, missed changes and the delay of the matched changes
(median / p90 / worst, s), tunnel entries and exits separately.
"""
import argparse
import json

import numpy as np

KEYS = ("place", "weather", "light", "tunnel_car", "wet", "area", "limit", "fused_place", "fused_built_up",
        "fused_tunnel", "fused_limit")


def fmt_d(v):
    if not v:
        return "-"
    v = np.asarray(v)
    return "%.1f/%.1f/%.1f" % (np.median(v), np.percentile(v, 90), v.max())


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("files", nargs="+")
    ap.add_argument("--sessions", default="")
    ap.add_argument("--keys", default=",".join(KEYS))
    a = ap.parse_args()
    data = {}
    for f in a.files:
        data.update(json.load(open(f)))
    if a.sessions:
        data = {k: v for k, v in data.items() if k in a.sessions.split(",")}
    keys = a.keys.split(",")
    names = []
    for s in data.values():
        for n in s["cands"]:
            if n not in names:
                names.append(n)
    dur = sum(s["duration_s"] for s in data.values())
    print("sessions: %d, %.0f min; baseline changes: %s" % (len(data), dur / 60, {k: sum(s["base_changes"].get(k, 0) for s in data.values()) for k in keys}))
    print("| candidate | inf/s | " + " | ".join(keys) + " |")
    print("|---|---|" + "---|" * len(keys))
    for n in names:
        runs = [(s, r) for s in data.values() if n in s["cands"] for r in s["cands"][n]]
        if not runs:
            continue
        # duration-weighted mean over sessions of the mean over phases
        rate = sum(np.mean([r["rate"] for r in s["cands"][n]]) * s["duration_s"] for s in data.values() if n in s["cands"]) / \
            sum(s["duration_s"] for s in data.values() if n in s["cands"])
        cells = []
        for k in keys:
            m = sum(np.mean([r[k]["dis"] for r in s["cands"][n]]) * s["duration_s"] for s in data.values() if n in s["cands"]) / \
                sum(s["duration_s"] for s in data.values() if n in s["cands"])
            cells.append("%.2f" % (100 * m))
        print("| %s | %.3f | %s |" % (n, rate, " | ".join(cells)))
    print()
    print("changes per candidate (sum over runs / runs): baseline n, candidate n, extra, missed, delay med/p90/worst s")
    for k in keys:
        print("\n### %s" % k)
        print("| candidate | base | cand | extra | missed | delay | entry delay | exit delay |")
        print("|---|---|---|---|---|---|---|---|")
        for n in names:
            runs = [r for s in data.values() if n in s["cands"] for r in s["cands"][n]]
            if not runs:
                continue
            nr = len(runs)
            b = sum(r[k]["n_base"] for r in runs) / nr
            c = sum(r[k]["n_cand"] for r in runs) / nr
            ex = sum(r[k]["extra"] for r in runs) / nr
            mi = sum(len(r[k]["missed"]) for r in runs) / nr
            d = [x[0] for r in runs for x in r[k]["delays"]]
            if k == "place":
                ent = [x[0] for r in runs for x in r[k]["delays"] if x[2] == "tunnel"]
                exi = [x[0] for r in runs for x in r[k]["delays"] if x[1] == "tunnel"]
            elif k in ("tunnel_car", "fused_tunnel"):
                ent = [x[0] for r in runs for x in r[k]["delays"] if x[2] in (True, "True")]
                exi = [x[0] for r in runs for x in r[k]["delays"] if x[1] in (True, "True")]
            else:
                ent = exi = []
            print("| %s | %.1f | %.1f | %.1f | %.1f | %s | %s | %s |" % (n, b, c, ex, mi, fmt_d(d), fmt_d(ent), fmt_d(exi)))


if __name__ == "__main__":
    main()
