#!/usr/bin/env python3
"""Both road models through the api's 2.0 trackers on the same ride stretches (lane L24-M).

    python tools/vision_eval/evaluate.py ROAD_DIR [--rides a,b,..] [--streams sc.13.8,vis.13.8,vis.17,vis.20]
                                         [--labels ~/roadsense-labels] [--out RESULTS.json]

ROAD_DIR holds <ride>.<stream>.jsonl.gz from tools/vision_eval/ride_replay.sh (seq = frame from the ride start).
Every stream goes through tools/replay_lib.Pipeline (LeadTracker, RoadGeometryTracker, EgoLaneEstimator,
RoadGeometryFusion, ViewBuilder: the api's code, no map, no vehicle speed). Per stream:

  flicker (per minute of stream time): presence changes of the tracked inner lane lines (`lanes.lines[1..2]`),
    outer lines, road edges; lead on/off and lead identity changes; ego-lane index changes (`egolane`);
  jitter: robust sd (1.4826 MAD) of the tracked inner lines at 10 m and 50 m about their centred 1 s mean, on
    stretches the line is present; the same for the raw model lines;
  lead noise: robust sd of the tracked lead distance about its centred 1 s median, per distance band; of v_rel;
  far range: a tracked inner line point seen at 20-40 / 40-70 / 70-100 m is moved with the car's motion until it
    is 5-15 m ahead and compared with the line seen there then (|dy|, median and p90). The car's motion is the
    mean of the two models' own pose (vx, yaw rate) of the 20 Hz streams, so neither model scores on its own
    odometry; the same for the raw lines;
  coverage: share of runs with both inner lines present, with a lead;
  egolane against L20's labels (tools/egolane_replay.py, map count from the label, and none) where labels exist.
"""
import argparse
import bisect
import json
import math
import os
import subprocess
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))
from replay_lib import Pipeline, read_lines  # noqa: E402

X = np.array([192.0 * (i / 32.0) ** 2 for i in range(33)])
BANDS = ((20, 40), (40, 70), (70, 100))
LEAD_BANDS = ((0, 15), (15, 30), (30, 60), (60, 200))


def rsd(v):
    v = np.asarray(v, float)
    v = v[np.isfinite(v)]
    if len(v) < 10:
        return float("nan")
    return float(1.4826 * np.median(np.abs(v - np.median(v))))


def at(y, x):
    return float(np.interp(x, X, y))


def changes(seq):
    return sum(1 for a, b in zip(seq, seq[1:]) if a != b)


def centred(t, v, half=0.5, fn=np.mean):
    """Centred moving statistic over +-half s (NaN where the window has < 3 finite samples)."""
    t = np.asarray(t); v = np.asarray(v, float)
    out = np.full(len(v), np.nan)
    lo = np.searchsorted(t, t - half); hi = np.searchsorted(t, t + half, side="right")
    for i in range(len(v)):
        w = v[lo[i]:hi[i]]
        w = w[np.isfinite(w)]
        if len(w) >= 3:
            out[i] = fn(w)
    return out


TRACKER_CFG = {"lanes": {}, "lead": {}}
# --pre: a temporal stage in front of the trackers (exponential low-pass by message time, constants in s):
#   prob_tau  presence probabilities (lane lines, leads[0]); geo_tau  lane-line / edge y, leads[0] x / v
PRE = {"prob_tau": 0.0, "geo_tau": 0.0}
TRACKER_ON = ["all"]


class PreStage:
    """Low-pass of the model outputs before the api's trackers (the candidate 'temporal stage')."""

    def __init__(self, prob_tau, geo_tau):
        self.pt, self.gt, self.t, self.st = prob_tau, geo_tau, None, {}

    def _f(self, key, v, a):
        old = self.st.get(key)
        if old is None or a >= 1.0:
            new = np.asarray(v, float)
        else:
            new = old + a * (np.asarray(v, float) - old)
        self.st[key] = new
        return new

    def __call__(self, m):
        t = m["ts_mono_ns"] * 1e-9
        dt = 0.05 if self.t is None else min(max(t - self.t, 0.0), 1.0)
        if self.t is not None and t - self.t > 1.0:
            self.st.clear()
        self.t = t
        ap = 1.0 - math.exp(-dt / self.pt) if self.pt > 0 else 1.0
        ag = 1.0 - math.exp(-dt / self.gt) if self.gt > 0 else 1.0
        for i, l in enumerate(m["lane_lines"]):
            l["prob"] = float(self._f(f"lp{i}", l["prob"], ap))
            l["y"] = self._f(f"ly{i}", l["y"], ag).tolist()
        for i, e in enumerate(m["road_edges"]):
            e["y"] = self._f(f"ey{i}", e["y"], ag).tolist()
        L = m["leads"][0]
        L["prob"] = float(self._f("ldp", L["prob"], ap))
        for k in ("x", "v"):
            L[k] = self._f("ld" + k, L[k], ag).tolist()
        return m   # --lanes-set / --lead-set: tracker options for a tracker-side variant


def load(path, strip_plan=False):
    """Run one stream through the api pipeline; per road message the raw and tracked quantities.
    strip_plan: drop the `plan` key first (what the api sees from the vision model without the policy)."""
    p = Pipeline()
    pre = PreStage(PRE["prob_tau"], PRE["geo_tau"]) if (PRE["prob_tau"] or PRE["geo_tau"]) and "vis" in os.path.basename(path) else None
    if (TRACKER_CFG["lanes"] or TRACKER_CFG["lead"]) and (TRACKER_ON[0] == "all" or "vis" in os.path.basename(path)):
        from chris_integrator.trackers.lanes import RoadGeometryTracker
        from chris_integrator.trackers.lead import LeadTracker
        p.lanes, p.lead = RoadGeometryTracker(TRACKER_CFG["lanes"]), LeadTracker(TRACKER_CFG["lead"])
    R = {k: [] for k in ("t", "seq", "raw_in", "raw_edge", "raw_lp", "lead_raw_x", "lead_raw_p", "vx", "yr",
                         "tr_present", "tr_y", "tr_edge_present", "lead_present", "lead_d", "lead_v", "lead_id",
                         "ego_idx", "has_plan", "model_ms", "geom_from")}
    for m in read_lines(path):
        if strip_plan and m.get("type") == "road":
            m.pop("plan", None)
        if pre is not None and m.get("type") == "road":
            m = pre(m)
        outs = p.feed(m)
        if m.get("type") != "road":
            continue
        d = {o["type"]: o for o in outs}
        ln, ld = d.get("lanes"), d.get("lead")
        if ln is None:
            continue
        R["t"].append(m["ts_mono_ns"] * 1e-9); R["seq"].append(m["seq"])
        ll = m["lane_lines"]
        R["raw_in"].append([ll[1]["y"], ll[2]["y"]])
        R["raw_lp"].append([l["prob"] for l in ll])
        R["raw_edge"].append([e["y"] for e in m["road_edges"]])
        R["lead_raw_x"].append(m["leads"][0]["x"][0]); R["lead_raw_p"].append(m["leads"][0]["prob"])
        R["vx"].append(m["pose"]["vx"]); R["yr"].append(m["pose"]["yaw_rate"])
        R["model_ms"].append(m.get("model_ms") or 0)
        R["has_plan"].append("plan" in m)
        R["tr_present"].append([bool(l.get("present")) for l in ln["lines"]])
        R["tr_y"].append([l["y"] if l.get("present") and l.get("y") else [np.nan] * 33 for l in ln["lines"]])
        R["tr_edge_present"].append([bool(e.get("present")) for e in ln.get("edges") or []])
        lp = bool(ld and ld.get("present"))
        R["lead_present"].append(lp)
        R["lead_d"].append(ld["dist_m"] if lp and ld.get("dist_m") is not None else np.nan)
        R["lead_v"].append(ld["v_rel_mps"] if lp and ld.get("v_rel_mps") is not None else np.nan)
        R["lead_id"].append(ld.get("track_id") if lp else None)
        vw = d.get("view") or {}
        R["geom_from"].append(((vw.get("road") or {}).get("vision") or {}).get("from"))
        eg = p.latest.get("egolane") or {}
        R["ego_idx"].append(eg.get("ego_index"))
    for k in R:
        if k not in ("lead_id", "ego_idx", "geom_from"):
            R[k] = np.asarray(R[k], float) if k not in ("tr_present", "tr_edge_present", "lead_present", "has_plan") \
                else np.asarray(R[k], bool)
    return R


def ego_track(streams):
    """Common ego motion: mean pose of the 20 Hz streams of both models (vx, yaw rate) on the frame grid."""
    ts = sorted(set(np.round(np.concatenate([s["t"] for s in streams]), 3)))
    t = np.asarray(ts)
    vx = np.nanmean([np.interp(t, s["t"], s["vx"]) for s in streams], axis=0)
    yr = np.nanmean([np.interp(t, s["t"], s["yr"]) for s in streams], axis=0)
    dt = np.diff(t, prepend=t[0])
    dt[dt > 0.5] = 0.0
    psi = np.cumsum(yr * dt)
    X_ = np.cumsum(vx * np.cos(psi) * dt); Y_ = np.cumsum(vx * np.sin(psi) * dt)
    return t, X_, Y_, psi, np.cumsum(np.abs(vx) * dt)


def far_range(S, ego, which):
    """|dy| (m) per band: line point at x (band) at t -> same world point when it is 5-15 m ahead."""
    t_e, Xe, Ye, Pe, se = ego
    Xi = lambda tt: np.interp(tt, t_e, Xe); Yi = lambda tt: np.interp(tt, t_e, Ye)
    Pi = lambda tt: np.interp(tt, t_e, Pe); si = lambda tt: np.interp(tt, t_e, se)
    t = S["t"]; s_run = si(t)
    if which == "tracked":
        Y = S["tr_y"][:, 1:3, :]
    else:
        Y = S["raw_in"]
    errs = {b: [] for b in BANDS}
    xi_band = {b: np.nonzero((X >= b[0]) & (X < b[1]))[0] for b in BANDS}
    for i in range(0, len(t), 2):
        if not np.isfinite(t[i]):
            continue
        for b, idx in xi_band.items():
            for xi in idx:
                x0 = X[xi]
                # first later run where the point is ~10 m ahead (travelled x0 - 10 m)
                j = bisect.bisect_left(s_run, s_run[i] + x0 - 10.0, lo=i + 1)
                if j >= len(t) or t[j] - t[i] > 15.0:
                    continue
                for line in range(2):
                    y0 = Y[i, line, xi]
                    if not np.isfinite(y0):
                        continue
                    c, s_ = math.cos(Pi(t[i])), math.sin(Pi(t[i]))
                    wx = Xi(t[i]) + x0 * c - y0 * s_; wy = Yi(t[i]) + x0 * s_ + y0 * c
                    c2, s2 = math.cos(Pi(t[j])), math.sin(Pi(t[j]))
                    dx, dy = wx - Xi(t[j]), wy - Yi(t[j])
                    xp = dx * c2 + dy * s2; yp = -dx * s2 + dy * c2
                    if not 5.0 <= xp <= 15.0:
                        continue
                    y1 = Y[j, line, :]
                    if not np.isfinite(y1).all():
                        continue
                    errs[b].append(np.interp(xp, X, y1) - yp)
    out = {}
    for b, v in errs.items():
        v = np.asarray(v)
        out[f"{b[0]}-{b[1]}"] = (float(np.median(np.abs(v))) if len(v) else float("nan"),
                                 float(np.percentile(np.abs(v), 90)) if len(v) else float("nan"), len(v),
                                 float(np.median(v)) if len(v) else float("nan"))  # signed: + = seen right of prediction
    return out


def metrics(S, ego):
    t = S["t"]
    minutes = (t[-1] - t[0]) / 60.0
    out = {"runs": int(len(t)), "minutes": round(minutes, 2), "runs_per_s": round(len(t) / max(1e-9, t[-1] - t[0]), 2),
           "model_ms": round(float(np.mean(S["model_ms"])), 1)}
    P = S["tr_present"]
    out["inner_line_changes_pm"] = round((changes(list(P[:, 1])) + changes(list(P[:, 2]))) / minutes, 2)
    out["outer_line_changes_pm"] = round((changes(list(P[:, 0])) + changes(list(P[:, 3]))) / minutes, 2)
    E = S["tr_edge_present"]
    out["edge_changes_pm"] = round((changes(list(E[:, 0])) + changes(list(E[:, 1]))) / minutes, 2) if E.size else None
    out["both_inner_present_pct"] = round(100.0 * float(np.mean(P[:, 1] & P[:, 2])), 1)
    out["lead_onoff_pm"] = round(changes(list(S["lead_present"])) / minutes, 2)
    ids = [i for i in S["lead_id"] if i is not None]
    out["lead_id_changes_pm"] = round(changes(ids) / minutes, 2)
    out["lead_present_pct"] = round(100.0 * float(np.mean(S["lead_present"])), 1)
    for src in sorted(set(map(str, S["geom_from"]))):  # view.road.vision.from: where the road geometry came from
        out[f"geom_from_{src}_pct"] = round(100.0 * sum(1 for g in S["geom_from"] if str(g) == src) / len(t), 1)
    out["ego_idx_changes_pm"] = round(changes(S["ego_idx"]) / minutes, 2)
    out["ego_idx_known_pct"] = round(100.0 * sum(1 for e in S["ego_idx"] if e is not None) / len(t), 1)
    # jitter of tracked / raw inner lines about a centred 1 s mean
    for name, Y in (("tr", S["tr_y"][:, 1:3, :]), ("raw", S["raw_in"])):
        for xm in (10.0, 50.0):
            res = []
            for line in range(2):
                v = np.array([np.interp(xm, X, Y[i, line]) if np.isfinite(Y[i, line]).all() else np.nan
                              for i in range(len(t))])
                res.append(v - centred(t, v))
            out[f"jitter_{name}_{int(xm)}m"] = round(rsd(np.concatenate(res)), 3)
    # lead distance / speed noise (tracked)
    d = S["lead_d"]
    res = d - centred(t, d, fn=np.median)
    for lo, hi in LEAD_BANDS:
        sel = (d >= lo) & (d < hi)
        out[f"lead_d_noise_{lo}-{hi}m"] = round(rsd(res[sel]), 3) if sel.sum() >= 20 else None
    v = S["lead_v"]
    out["lead_v_noise"] = round(rsd(v - centred(t, v, fn=np.median)), 3)
    for which in ("tracked", "raw"):
        for band, (med, p90, n, sgn) in far_range(S, ego, which).items():
            out[f"far_{which}_{band}"] = [round(med, 3), round(p90, 3), n, round(sgn, 3)]
    return out


def agreement(A, B):
    """Median |dy| of the tracked inner lines at 10 / 50 m and |d dist| of the lead where both have them."""
    out = {}
    tb = B["t"]
    for xm in (10.0, 50.0):
        dd = []
        for i, ta in enumerate(A["t"]):
            j = int(np.argmin(np.abs(tb - ta)))
            if abs(tb[j] - ta) > 0.03:
                continue
            for line in (1, 2):
                ya, yb = A["tr_y"][i, line], B["tr_y"][j, line]
                if np.isfinite(ya).all() and np.isfinite(yb).all():
                    dd.append(abs(np.interp(xm, X, ya) - np.interp(xm, X, yb)))
        out[f"inner_dy_{int(xm)}m_median"] = round(float(np.median(dd)), 3) if dd else None
    dd = []
    for i, ta in enumerate(A["t"]):
        j = int(np.argmin(np.abs(tb - ta)))
        if abs(tb[j] - ta) <= 0.03 and np.isfinite(A["lead_d"][i]) and np.isfinite(B["lead_d"][j]):
            dd.append(abs(A["lead_d"][i] - B["lead_d"][j]))
    out["lead_dd_median"] = round(float(np.median(dd)), 3) if dd else None
    both = []
    for i, ta in enumerate(A["t"]):
        j = int(np.argmin(np.abs(tb - ta)))
        if abs(tb[j] - ta) <= 0.03:
            both.append((A["lead_present"][i], B["lead_present"][j]))
    out["lead_presence_agree_pct"] = round(100.0 * np.mean([a == b for a, b in both]), 1) if both else None
    return out


def lag_ms(A, B, key):
    """Delay of B against A (ms, + = B later) maximising the correlation of the first differences of `key`
    (e.g. the model's own speed vx, or the tracked lead distance) on a 10 ms grid, over -400..400 ms."""
    t0, t1 = max(A["t"][0], B["t"][0]) + 1, min(A["t"][-1], B["t"][-1]) - 1
    if t1 - t0 < 30:
        return None
    g = np.arange(t0, t1, 0.01)
    def series(S):
        v = np.asarray(S[key], float); ok = np.isfinite(v)
        if ok.sum() < 100:
            return None
        y = np.interp(g, S["t"][ok], v[ok])
        y = np.convolve(y, np.ones(15) / 15, mode="same")  # 150 ms smoothing
        return np.diff(y)
    a, b = series(A), series(B)
    if a is None or b is None:
        return None
    best, bl = -2, None
    for L in range(-40, 41):
        x, y = (a[:len(a) - L], b[L:]) if L >= 0 else (a[-L:], b[:len(b) + L])
        c = np.corrcoef(x, y)[0, 1]
        if np.isfinite(c) and c > best:
            best, bl = c, L * 10
    return [bl, round(float(best), 3)]


def egolane_scores(path, labels):
    res = {}
    for mc in ("label", "none"):
        r = subprocess.run([sys.executable, os.path.join(os.path.dirname(HERE), "egolane_replay.py"), path,
                            "--labels", labels, "--map-count", mc], capture_output=True, text=True)
        try:
            res[mc] = json.loads(r.stdout.strip().splitlines()[-1])["estimator"]
        except (ValueError, IndexError):
            res[mc] = {"error": r.stderr[-300:]}
    return res


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("road_dir")
    ap.add_argument("--rides", default="")
    ap.add_argument("--streams", default="sc.13.8,vis.13.8,vis.17,vis.20,vis.13.8~noplan")
    ap.add_argument("--labels", default=os.path.expanduser("~/roadsense-labels"))
    ap.add_argument("--out", default="")
    ap.add_argument("--lanes-set", action="append", default=[], help="lane tracker option KEY=JSON")
    ap.add_argument("--lead-set", action="append", default=[], help="lead tracker option KEY=JSON")
    ap.add_argument("--no-egolane", action="store_true", help="skip the label scoring")
    ap.add_argument("--pre", default="", help="vision streams only: PROB_TAU,GEO_TAU of the low-pass in front of "
                    "the trackers (s)")
    ap.add_argument("--tracker-on", default="all", choices=("all", "vis"),
                    help="apply --lanes-set / --lead-set to all streams or to the vision streams only")
    a = ap.parse_args()
    if a.pre:
        PRE["prob_tau"], PRE["geo_tau"] = map(float, a.pre.split(","))
    TRACKER_ON[0] = a.tracker_on
    for name, kvs in (("lanes", a.lanes_set), ("lead", a.lead_set)):
        for kv in kvs:
            k, _, v = kv.partition("=")
            TRACKER_CFG[name][k] = json.loads(v)
    rides = a.rides.split(",") if a.rides else sorted({f.split(".")[0] for f in os.listdir(a.road_dir)
                                                       if f.endswith(".vis.20.jsonl.gz")})
    streams = a.streams.split(",")
    allres = {}
    if a.out and os.path.exists(a.out):
        allres = json.load(open(a.out))
    for ride in rides:
        if ride in allres:
            continue
        paths = {s: os.path.join(a.road_dir, f"{ride}.{s.split('~')[0]}.jsonl.gz") for s in streams}
        if not all(os.path.exists(p) for p in paths.values()):
            print(f"{ride}: streams missing, skipped", flush=True)
            continue
        S = {s: load(p, strip_plan=s.endswith("~noplan")) for s, p in paths.items()}
        ref20 = ([S[s] for s in S if s == "vis.20"] or [S[s] for s in S if s == "vis.17"]) + [S[s] for s in S if s.startswith("sc.")]
        ego_src = os.environ.get("EGO", "mean")   # diagnostic: odometry of one model only (sc / vis)
        ego = ego_track(ref20[:2] if ego_src == "mean" else ref20[1:2] if ego_src == "sc" else ref20[:1])
        res = {s: metrics(S[s], ego) for s in streams}
        res["agree_sc_vis13.8"] = agreement(S[streams[0]], S["vis.13.8"]) if "vis.13.8" in S else None
        for s_ in streams[1:]:
            res[s_]["lag_vs_ref_ms"] = {"pose_vx": lag_ms(S[streams[0]], S[s_], "vx"),
                                        "lead_dist": lag_ms(S[streams[0]], S[s_], "lead_d")}
        if "vis.20" in S:
            res["lag_vis20_vs_sc_ms"] = {"pose_vx": lag_ms(S[streams[0]], S["vis.20"], "vx"),
                                         "lead_dist": lag_ms(S[streams[0]], S["vis.20"], "lead_d")}
        lab = os.path.join(a.labels, f"{ride}.lanes.csv")
        if os.path.exists(lab) and not a.no_egolane:
            res["egolane"] = {s: egolane_scores(p, lab) for s, p in paths.items() if "~" not in s}
        allres[ride] = res
        print(json.dumps({ride: res}), flush=True)
        if a.out:
            json.dump(allres, open(a.out, "w"), indent=1)


if __name__ == "__main__":
    main()
