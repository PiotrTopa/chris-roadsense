#!/usr/bin/env python3
"""The lead tracker on both road models' streams, after the tracker (lane L27).

    python tools/vision_eval/lead_eval.py extract ROAD_DIR LEAD_DIR
        ROAD_DIR: <ride>.<stream>.jsonl.gz of tools/vision_eval/ride_replay.sh. Writes the same names to LEAD_DIR
        with only what LeadTracker reads (leads, v_ego, pose, road_model): 40 times smaller, seconds per run.
    python tools/vision_eval/lead_eval.py score LEAD_DIR --rides a,b,.. [--streams sc.13.8,vis.13.8,vis.17,vis.20]
                                          [--set KEY=JSON ...] [--set-on vis|sc|all] [--no-profile] [--json OUT]
        Per stream and ride, and the mean over the rides:
          on/off, id     lead present <-> absent changes, and track_id changes, per minute (as L24-M's evaluate.py)
          steps          jumps of the tracked distance between two successive messages of more than max(1.5 m, 8 %),
                         per minute, whatever the track_id does (what a consumer that draws the lead sees jump)
          fstep          the tracked distance's step from one message to the next after the track's own relative
                         speed is taken out (dist[k] - dist[k-1] - v_rel dt), within one track: rms, p95 (m) and
                         steps over 0.5 m per minute. What a display that draws at the model's rate sees jump
          present        % of runs with a lead; unconf: % of those runs not confirmed by a full measurement
                         (mode coast), of which `weak` had a weak-evidence frame and the rest were prediction only
          d noise        robust sd of the tracked distance about its centred 1 s median, per distance band
          v noise        the same for the relative speed
          follow         |tracked distance - centred 1 s median of the model's raw distance| / distance, median and
                         p90 in % (frames with raw prob >= 0.5): a track that does not follow its measurement
          acq s          median delay from the first frame of a clear lead (raw prob >= 0.8 for >= 1 s, after >= 1 s
                         under 0.3) to `present`
          rel s          median time `present` stays up after the raw probability fell under 0.1 for >= 3 s
                         (how long a lead that is gone is still shown, marked coast)
          vs ref         presence agreement with the first stream's (supercombo's) default tracker, % of runs
        --set changes tracker options on the streams named by --set-on (default vis); --no-profile runs the vision
        streams without the `vision` profile (supercombo's settings, L24-M's baseline).
"""
import argparse
import gzip
import json
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(HERE)), "api"))
from roadsense_api.lead import DEFAULT_CFG, LeadTracker  # noqa: E402

BANDS = ((0, 15), (15, 30), (30, 60), (60, 200))


def extract(src, dst):
    os.makedirs(dst, exist_ok=True)
    for f in sorted(os.listdir(src)):
        out = os.path.join(dst, f)
        if not f.endswith(".jsonl.gz") or os.path.exists(out):
            continue
        with gzip.open(os.path.join(src, f), "rt") as fi, gzip.open(out + ".tmp", "wt") as fo:
            for ln in fi:
                if not ln.startswith("{"):
                    continue
                m = json.loads(ln)
                if m.get("type") != "road":
                    continue
                o = {k: m.get(k) for k in ("type", "seq", "ts_mono_ns", "ts_real_ns", "v_ego", "valid", "road_model")}
                o["pose"] = {"vx": m["pose"]["vx"], "yaw_rate": m["pose"]["yaw_rate"]}
                o["leads"] = [{k: (v[:1] if isinstance(v, list) else v) for k, v in L.items() if k != "t"}
                              for L in m["leads"]]
                fo.write(json.dumps(o, separators=(",", ":")) + "\n")
        os.rename(out + ".tmp", out)
        print(f, flush=True)


def read(path):
    with gzip.open(path, "rt") as f:
        return [json.loads(ln) for ln in f]


def rsd(v):
    v = np.asarray(v, float)
    v = v[np.isfinite(v)]
    return float(1.4826 * np.median(np.abs(v - np.median(v)))) if len(v) >= 10 else float("nan")


def centred_median(t, v, half=0.5):
    out = np.full(len(v), np.nan)
    lo, hi = np.searchsorted(t, t - half), np.searchsorted(t, t + half, side="right")
    for i in range(len(v)):
        w = v[lo[i]:hi[i]]
        w = w[np.isfinite(w)]
        if len(w) >= 3:
            out[i] = np.median(w)
    return out


def changes(seq):
    return sum(1 for a, b in zip(seq, seq[1:]) if a != b)


def run(ms, cfg, profile=True):
    tr = LeadTracker(cfg)
    outs = []
    for m in ms:
        if not profile:
            m = dict(m, road_model=None)
        outs.append(tr.on_road(m))
    return outs


def score(ms, outs, ref_present=None):
    t = np.array([m["ts_mono_ns"] * 1e-9 for m in ms])
    p = np.array([m["leads"][0]["prob"] for m in ms])
    pres = np.array([bool(o["present"]) for o in outs])
    minutes = (t[-1] - t[0]) / 60.0
    r = {"minutes": minutes, "onoff": changes(list(pres)), "id": changes([o["track_id"] for o in outs if o["present"]]),
         "present_pct": 100.0 * pres.mean()}
    unc = np.array([o["present"] and o["mode"] == "coast" for o in outs])
    weak = np.array([o["present"] and o.get("evidence") == "weak" for o in outs])
    r["unconf_pct"] = 100.0 * unc.sum() / max(1, pres.sum())
    r["weak_pct"] = 100.0 * weak.sum() / max(1, pres.sum())
    d = np.array([o["dist_m"] if o["present"] else np.nan for o in outs], float)
    v = np.array([o["v_rel_mps"] if o["present"] else np.nan for o in outs], float)
    res = d - centred_median(t, d)
    for lo, hi in BANDS:
        sel = (d >= lo) & (d < hi)
        r[f"d_noise_{lo}-{hi}"] = rsd(res[sel]) if sel.sum() >= 20 else float("nan")
        r[f"n_{lo}-{hi}"] = int(sel.sum())
    r["v_noise"] = rsd(v - centred_median(t, v))
    dd = np.abs(np.diff(d))
    r["steps"] = int((dd > np.maximum(1.5, 0.08 * d[1:])).sum())
    # what a consumer that draws the lead at the model's rate sees per frame (tools/view_replay.py's measure): the
    # distance step after the track's own relative speed is taken out, on frames of one track
    tid = np.array([o["track_id"] if o["present"] else -1 for o in outs])
    dt = np.diff(t)
    same = (tid[1:] == tid[:-1]) & (tid[1:] >= 0) & (dt < 0.5)
    fs = (np.diff(d) - v[1:] * dt)[same]
    r["fstep_rms"] = float(np.sqrt(np.mean(fs ** 2))) if len(fs) >= 20 else float("nan")
    r["fstep_p95"] = float(np.percentile(np.abs(fs), 95)) if len(fs) >= 20 else float("nan")
    r["fstep_over"] = int((np.abs(fs) > 0.5).sum())
    x = np.array([m["leads"][0]["x"][0] for m in ms])
    rawmed = centred_median(t, np.where(p >= 0.5, x, np.nan))
    fe = np.abs(d - rawmed) / rawmed
    fe = fe[np.isfinite(fe)]
    r["follow_pct"] = 100.0 * float(np.median(fe)) if len(fe) >= 20 else float("nan")
    r["follow_p90_pct"] = 100.0 * float(np.percentile(fe, 90)) if len(fe) >= 20 else float("nan")
    # acquisition delay on clear onsets, release delay on clear ends
    acq, rel = [], []
    hi8, lo3, lo1 = p >= 0.8, p < 0.3, p < 0.1
    i, n = 1, len(t)
    while i < n:
        if p[i] >= 0.5 and lo3[i - 1]:
            a = np.searchsorted(t, t[i] - 1.0)
            b = np.searchsorted(t, t[i] + 1.2)
            j = np.searchsorted(t, t[i] + 0.2)
            if a < i and b < n and lo3[a:i].all() and hi8[j:b].mean() > 0.9 and not pres[i - 1]:
                k = i
                while k < n and not pres[k]:
                    k += 1
                if k < n:
                    acq.append(t[k] - t[i])
                i = b
                continue
        if lo1[i] and not lo1[i - 1] and pres[i - 1]:
            b = np.searchsorted(t, t[i] + 3.0)
            if b < n and (p[i:b] < 0.3).all():
                k = i
                while k < n and pres[k]:
                    k += 1
                rel.append(t[min(k, n - 1)] - t[i])
                i = b
                continue
        i += 1
    r["acq_s"], r["acq_n"] = (float(np.median(acq)) if acq else float("nan")), len(acq)
    r["rel_s"], r["rel_n"] = (float(np.median(rel)) if rel else float("nan")), len(rel)
    r["rel_max_s"] = float(np.max(rel)) if rel else float("nan")
    if ref_present is not None:
        rt, rp = ref_present
        j = np.clip(np.searchsorted(rt, t), 1, len(rt) - 1)
        j = np.where(np.abs(rt[j - 1] - t) < np.abs(rt[j] - t), j - 1, j)
        ok = np.abs(rt[j] - t) < 0.04
        r["vs_ref_pct"] = 100.0 * float((pres[ok] == rp[j][ok]).mean()) if ok.any() else float("nan")
        r["only_here_pct"] = 100.0 * float((pres[ok] & ~rp[j][ok]).mean()) if ok.any() else float("nan")
        r["only_ref_pct"] = 100.0 * float((~pres[ok] & rp[j][ok]).mean()) if ok.any() else float("nan")
    return r, (t, pres)


COLS = (("on/off /min", lambda r: r["onoff"] / r["minutes"]), ("id /min", lambda r: r["id"] / r["minutes"]),
        ("steps /min", lambda r: r["steps"] / r["minutes"]),
        ("fstep rms", lambda r: r["fstep_rms"]), ("fstep p95", lambda r: r["fstep_p95"]),
        ("fstep>0.5 /min", lambda r: r["fstep_over"] / r["minutes"]),
        ("present %", lambda r: r["present_pct"]), ("unconf %", lambda r: r["unconf_pct"]),
        ("weak %", lambda r: r["weak_pct"]),
        ("d 0-15", lambda r: r["d_noise_0-15"]), ("d 15-30", lambda r: r["d_noise_15-30"]),
        ("d 30-60", lambda r: r["d_noise_30-60"]), ("d 60+", lambda r: r["d_noise_60-200"]),
        ("v noise", lambda r: r["v_noise"]), ("follow %", lambda r: r["follow_pct"]),
        ("follow p90 %", lambda r: r["follow_p90_pct"]), ("acq s", lambda r: r["acq_s"]), ("rel s", lambda r: r["rel_s"]),
        ("rel max", lambda r: r["rel_max_s"]),
        ("vs ref %", lambda r: r.get("vs_ref_pct", float("nan"))),
        ("only here %", lambda r: r.get("only_here_pct", float("nan"))),
        ("only ref %", lambda r: r.get("only_ref_pct", float("nan"))))


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd", required=True)
    e = sub.add_parser("extract")
    e.add_argument("road_dir")
    e.add_argument("lead_dir")
    s = sub.add_parser("score")
    s.add_argument("lead_dir")
    s.add_argument("--rides", required=True)
    s.add_argument("--streams", default="sc.13.8,vis.13.8,vis.17,vis.20")
    s.add_argument("--set", action="append", default=[])
    s.add_argument("--set-on", default="vis", choices=("vis", "sc", "all"))
    s.add_argument("--no-profile", action="store_true")
    s.add_argument("--per-ride", action="store_true")
    s.add_argument("--json", default="")
    a = ap.parse_args()
    if a.cmd == "extract":
        return extract(a.road_dir, a.lead_dir)
    cfg = {}
    for kv in a.set:
        k, _, v = kv.partition("=")
        if k not in DEFAULT_CFG:
            sys.exit(f"unknown tracker option {k}")
        cfg[k] = json.loads(v)
    rides, streams = a.rides.split(","), a.streams.split(",")
    allr = {}
    for ride in rides:
        ref = None
        for st in streams:
            path = os.path.join(a.lead_dir, f"{ride}.{st}.jsonl.gz")
            if not os.path.exists(path):
                continue
            ms = read(path)
            vis = st.startswith("vis")
            on = a.set_on == "all" or (a.set_on == "vis") == vis
            r, pr = score(ms, run(ms, cfg if on else None, profile=not (a.no_profile and vis)), ref)
            if ref is None:
                ref = pr
            allr.setdefault(st, {})[ride] = r
    print("| stream | rides | " + " | ".join(c for c, _ in COLS) + " |")
    print("|---|---|" + "---|" * len(COLS))
    for st, rr in allr.items():
        rows = list(rr.items()) if a.per_ride else []
        for ride, r in rows:
            print(f"| {st} | {ride} | " + " | ".join(f"{f(r):.2f}" for _, f in COLS) + " |")
        vals = [[f(r) for r in rr.values()] for _, f in COLS]
        print(f"| {st} | {len(rr)} | " + " | ".join(
            f"{np.nanmean(v):.2f}" if np.isfinite(v).any() else "-" for v in vals) + " |")
    if a.json:
        json.dump(allr, open(a.json, "w"), indent=1)


if __name__ == "__main__":
    main()
