#!/usr/bin/env python3
"""What a display would draw as lane lines, per source, measured on `view.lines` (lane L29).

    display_eval.py views ROAD.jsonl.gz OUT.npz [--laned DISPLAY_LANES.jsonl --latency 0.05] [--view-set K=JSON]
        One road stream (tools/vision_eval/ride_replay.sh) through the api's pipeline (tools/replay_lib.py); with
        --laned the `display_lanes` lines of a provider (tools/laned_replay, laned) are fed in as the api would
        get them: the message of the frame at t arrives at t + latency. Stores per road frame what `view` draws:
        the four lane lines at the view's distances (NaN where nothing is drawn), alpha, `lines_source`, and the
        road model's own speed and yaw rate.
    display_eval.py score NAME=VIEWS.npz ... [--ego A.npz,B.npz] [--ref NAME] [--labels LANES.csv --source SLUG]
                          [--paint FRAMES.nv12 --frame0 N --step K --focal F --calib calib.json [--trim-for NAME=DISPLAY_LANES.jsonl]]
                          [--json OUT]
        Per source, while the car moves (the road model's speed over 3 m/s):
          left / right / both   % of frames with the ego lane's left, right, both lines drawn (alpha >= 0.5)
          outer                 % of frames with at least one outer line drawn
          flicker in / out      appear + disappear events of the inner (outer) lines per minute
          reach                 median distance up to which the inner lines are drawn, m
          step 10 / 30          rms of an inner line's move between two frames at 10 m / 30 m, m
          jumps 30              moves over 0.3 m at 30 m per minute
          far 20-30 / 40-50     a drawn inner-line point at that distance, carried by the car's motion until it is
                                5-15 m ahead, against the line drawn there then: median / p90 of |dy|, m (correct
                                in curves and at range; --ego: the streams whose mean motion carries the point)
          vs ref                the inner lines at 10 m against the reference source's where both draw one, after
                                one scale factor per ride is taken out (the sources' metres differ with the
                                camera height they assume): median |dy|, % over 0.5 m (another marking)
          outer invented / missing   with --labels (lanes_own, ego_from_left per time span): % of labelled frames
                                with an outer line drawn on a side where the label has no lane; % with none where
                                it has one
          paint                 with --paint: the drawn inner lines against the image itself, one frame a second:
                                at 6-15 m and at 20-40 m the nearest paint stripe (a marking-width ridge of the
                                luma across the line, +-1.2 m) -> % of drawn points within 0.25 m of paint ("on"),
                                % with no paint within 1.2 m ("none"), median offset of the rest; points per frame
"""
import argparse
import bisect
import csv
import gzip
import json
import math
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))
sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(HERE)), "api"))

IDS = ("lane:0", "lane:1", "lane:2", "lane:3")


def read_jsonl(path):
    op = gzip.open if path.endswith(".gz") else open
    with op(path, "rt") as f:
        for ln in f:
            if ln.startswith("{"):
                yield json.loads(ln)


def views(a):
    from replay_lib import Pipeline, read_lines
    cfg = {}
    for kv in a.view_set:
        k, _, v = kv.partition("=")
        cfg[k] = json.loads(v)
    p = Pipeline(view_cfg=cfg or None)
    ext = sorted(read_jsonl(a.laned), key=lambda m: m["ts_mono_ns"]) if a.laned else []
    lat = int(a.latency * 1e9)
    k = 0
    T, SEQ, Y, AL, SRC, VX, YR, MZ, K = [], [], [], [], [], [], [], [], []
    xs = None
    for m in read_lines(a.road):
        if m.get("type") != "road":
            continue
        ts = m["ts_mono_ns"]
        while k < len(ext) and ext[k]["ts_mono_ns"] + lat <= ts:
            p.feed(ext[k])
            k += 1
        vw = next((o for o in p.feed(m) if o.get("type") == "view"), None)
        if vw is None:
            continue
        xs = vw["x"]
        row = np.full((4, len(xs)), np.nan)
        al = np.zeros(4)
        for ln in vw["lines"]:
            if ln["id"] in IDS:
                i = IDS.index(ln["id"])
                row[i] = [np.nan if v is None else v for v in ln["y"]]
                al[i] = ln["alpha"]
        T.append(ts * 1e-9); SEQ.append(m["seq"]); Y.append(row); AL.append(al)
        SRC.append(vw.get("lines_source") != "model")
        VX.append(m["pose"]["vx"]); YR.append(m["pose"]["yaw_rate"])
        z = [l["z"][0] for l in m["lane_lines"][1:3] if l.get("z")]
        MZ.append(float(np.mean(z)) if z else np.nan)        # the road model's own camera height (its metres)
        K.append(vw.get("lines_scale") or np.nan)
    np.savez_compressed(a.out, t=np.array(T), seq=np.array(SEQ), y=np.array(Y), alpha=np.array(AL), ext=np.array(SRC),
                        vx=np.array(VX), yr=np.array(YR), x=np.array(xs), mz=np.array(MZ), k=np.array(K))
    print(f"{a.out}: {len(T)} frames, external source on {100.0 * np.mean(SRC):.1f} %")


def ego_track(streams):
    t = np.array(sorted(set(np.round(np.concatenate([s["t"] for s in streams]), 3))))
    vx = np.mean([np.interp(t, s["t"], s["vx"]) for s in streams], axis=0)
    yr = np.mean([np.interp(t, s["t"], s["yr"]) for s in streams], axis=0)
    dt = np.diff(t, prepend=t[0])
    dt[dt > 0.5] = 0.0
    psi = np.cumsum(yr * dt)
    return t, np.cumsum(vx * np.cos(psi) * dt), np.cumsum(vx * np.sin(psi) * dt), psi, np.cumsum(np.abs(vx) * dt), vx


def far_consistency(S, ego, bands=((20, 30), (40, 50))):
    t_e, Xe, Ye, Pe, se, _ = ego
    t, X, Y = S["t"], S["x"], S["y"][:, 1:3, :]
    s_run = np.interp(t, t_e, se)
    Xi, Yi, Pi = np.interp(t, t_e, Xe), np.interp(t, t_e, Ye), np.interp(t, t_e, Pe)
    out = {}
    for lo, hi in bands:
        errs = []
        for xi in np.nonzero((X >= lo) & (X <= hi))[0]:
            x0 = X[xi]
            for i in range(0, len(t), 2):
                j = bisect.bisect_left(s_run, s_run[i] + x0 - 10.0, lo=i + 1)
                if j >= len(t) or t[j] - t[i] > 15.0:
                    continue
                for line in range(2):
                    y0 = Y[i, line, xi]
                    if not np.isfinite(y0):
                        continue
                    c, s_ = math.cos(Pi[i]), math.sin(Pi[i])
                    wx, wy = Xi[i] + x0 * c - y0 * s_, Yi[i] + x0 * s_ + y0 * c
                    c2, s2 = math.cos(Pi[j]), math.sin(Pi[j])
                    dx, dy = wx - Xi[j], wy - Yi[j]
                    xp, yp = dx * c2 + dy * s2, -dx * s2 + dy * c2
                    if not 5.0 <= xp <= 15.0:
                        continue
                    y1 = Y[j, line, :]
                    ok = np.isfinite(y1)
                    if ok.sum() < 4 or X[ok][-1] < 15.0:
                        continue
                    errs.append(np.interp(xp, X[ok], y1[ok]) - yp)
        e = np.abs(np.asarray(errs))
        out[f"far_{lo}-{hi}"] = (float(np.median(e)), float(np.percentile(e, 90)), len(e)) if len(e) >= 20 else (float("nan"), float("nan"), len(e))
    return out


def at(S, x):
    """The drawn lines at distance x: (frames, 4), NaN where not drawn."""
    X = S["x"]
    i = int(np.searchsorted(X, x))
    if X[i] == x:
        return S["y"][:, :, i]
    w = (x - X[i - 1]) / (X[i] - X[i - 1])
    return S["y"][:, :, i - 1] * (1 - w) + S["y"][:, :, i] * w


def drawn(S):
    return (S["alpha"] >= 0.5) & np.isfinite(at(S, 10.0))


class Paint:
    """The image's own evidence of paint next to a drawn line (an implementation of the marking filter that is
    independent of laned's: numpy, one image row per distance)."""

    def __init__(self, a):
        from roadsense_api.overlay import Projector
        cal = json.load(open(a.calib))
        self.cal = cal
        self.f, self.cx, self.cy = a.focal, a.cx, a.cy
        self.path, self.frame0, self.step = a.paint, a.frame0, a.step
        self.n = os.path.getsize(a.paint) // (1280 * 720 * 3 // 2)
        self.Projector = Projector
        self.trim = {}                   # frame -> (pitch trim, yaw trim) of the source that is scored

    def luma(self, k):
        with open(self.path, "rb") as f:
            f.seek(k * 1280 * 720 * 3 // 2)
            return np.frombuffer(f.read(1280 * 720), np.uint8).reshape(720, 1280).astype(np.float32)

    def offsets(self, img, xs, ys, trim=(0.0, 0.0), height=None):
        """Per (x, y) point: the offset (m) of the nearest paint stripe within +-1.2 m, NaN when there is none,
        None when the point is outside the image."""
        cal = dict(self.cal, pitch=self.cal["pitch"] + trim[0], yaw=self.cal["yaw"] + trim[1])
        if height:
            cal["height_m"] = height
        pr = self.Projector((self.f, self.cx, self.cy, 1280, 720), cal, (1280, 720))
        out = []
        d = np.arange(-1.2, 1.2001, 0.04)
        for x, y in zip(xs, ys):
            pts = pr.project(np.full(len(d), x), y + d, np.full(len(d), pr.height))
            if len(pts) != len(d):
                out.append(None)
                continue
            u = np.array([p[0] for p in pts]); v = int(round(pts[len(d) // 2][1]))
            if not (0 <= v < 720) or u.min() < 8 or u.max() > 1271:
                out.append(None)
                continue
            w = self.f * 0.15 / x                                     # marking width in pixels
            hw, gap = max(0.5, 0.5 * w), max(2.0, 1.5 * w + 1.0)
            row = img[v]
            cs = np.concatenate([[0.0], np.cumsum(row)])

            def box(c):
                a_ = np.clip(np.floor(c - hw + 0.5).astype(int), 0, 1279)
                b_ = np.clip(np.maximum(np.floor(c + hw + 0.5).astype(int), a_ + 1), 1, 1280)
                return (cs[b_] - cs[a_]) / (b_ - a_)
            c, l, r = box(u), box(u - gap), box(u + gap)
            resp = np.minimum(c - l, c - r)
            ok = resp > np.maximum(8.0, 0.08 * np.maximum(l, r))
            out.append(float(d[ok][np.argmin(np.abs(d[ok]))]) if ok.any() else float("nan"))
        return out


def score_one(S, ego, ref, labels, paint):
    t = S["t"]
    mov = np.interp(t, ego[0], ego[5]) > 3.0
    minutes = max(1e-9, float(np.sum(np.diff(t)[mov[1:] & (np.diff(t) < 0.5)])) / 60.0)
    D = drawn(S)
    r = {"frames": int(mov.sum()), "minutes": minutes, "ext_pct": 100.0 * float(S["ext"][mov].mean())}
    r["left"], r["right"] = 100.0 * D[mov, 1].mean(), 100.0 * D[mov, 2].mean()
    r["both"] = 100.0 * (D[mov, 1] & D[mov, 2]).mean()
    r["outer"] = 100.0 * (D[mov, 0] | D[mov, 3]).mean()
    ch = (D[1:] != D[:-1]) & mov[1:, None]
    r["flicker_in"] = float(ch[:, 1:3].sum()) / minutes
    r["flicker_out"] = float(ch[:, [0, 3]].sum()) / minutes
    X = S["x"]
    reach = np.where(np.isfinite(S["y"]), X[None, None, :], np.nan)
    rr = np.where(D[:, 1:3, None], reach[:, 1:3, :], -1.0)
    rr = np.where(np.isfinite(rr), rr, -1.0).max(axis=2)[mov]
    r["reach"] = float(np.median(rr[rr >= 0])) if (rr >= 0).any() else float("nan")
    for x in (10.0, 30.0):
        y = at(S, x)[:, 1:3]
        dy = np.diff(y, axis=0)
        ok = np.isfinite(dy) & mov[1:, None] & (np.diff(t) < 0.2)[:, None]
        r[f"step_{int(x)}"] = float(np.sqrt(np.mean(dy[ok] ** 2))) if ok.sum() >= 20 else float("nan")
        if x == 30.0:
            r["jumps_30"] = float((np.abs(dy[ok]) > 0.3).sum()) / minutes
    r.update(far_consistency(S, ego))
    if ref is not None:
        j = np.clip(np.searchsorted(ref["t"], t), 0, len(ref["t"]) - 1)
        okt = np.abs(ref["t"][j] - t) < 0.04
        a_, b_ = at(S, 10.0)[:, 1:3], at(ref, 10.0)[j][:, 1:3]
        both = np.isfinite(a_) & np.isfinite(b_) & (okt & mov)[:, None]
        # the two sources may differ in scale (the road model's metres rest on its own idea of the camera height,
        # a road-plane analysis on calib.json's): one factor per ride is taken out first
        ok = both & (np.abs(b_) > 0.5)
        sc = float(np.median(a_[ok] / b_[ok])) if ok.sum() >= 20 else 1.0
        e = np.abs(a_ - sc * b_)[both]
        r["ref_scale"] = sc
        r["ref_med"] = float(np.median(e)) if len(e) >= 20 else float("nan")
        r["ref_over"] = 100.0 * float((e > 0.5).mean()) if len(e) >= 20 else float("nan")
    if labels:
        inv = miss = n = exp = 0
        for t0, t1, own, efl in labels:
            sel = (t >= t0) & (t <= t1) & mov
            if not sel.any():
                continue
            want_l, want_r = efl >= 1, efl <= own - 2
            n += int(sel.sum())
            inv += int(((D[sel, 0] & (not want_l)) | (D[sel, 3] & (not want_r))).sum())
            if want_l or want_r:
                exp += int(sel.sum())
                miss += int((((~D[sel, 0]) & want_l) | ((~D[sel, 3]) & want_r)).sum())
        r["label_frames"] = n
        r["outer_invented"] = 100.0 * inv / n if n else float("nan")
        r["outer_missing"] = 100.0 * miss / exp if exp else float("nan")
    if paint is not None:
        seqs = S["seq"]
        mz = float(np.nanmedian(S["mz"])) if np.isfinite(S["mz"]).any() else None
        res = {"near": [], "far": []}
        nfr = 0
        for k in range(0, paint.n, 5):
            fr = paint.frame0 + k * paint.step
            i = int(np.searchsorted(seqs, fr + 1))
            if i >= len(seqs) or seqs[i] - (fr + 1) > 1 or not mov[i]:
                continue
            img = None
            nfr += 1
            for line in (1, 2):
                if not D[i, line]:
                    continue
                if img is None:
                    img = paint.luma(k)
                for band, pts in (("near", (6.0, 8.0, 10.0, 12.5, 15.0)), ("far", (20.0, 25.0, 30.0, 40.0))):
                    yy = [float(np.interp(x, X, S["y"][i, line])) for x in pts]
                    keep = [(x, y) for x, y in zip(pts, yy) if np.isfinite(y)]
                    if keep:
                        # the lines are in the road model's metres: its own camera height projects them (with an
                        # external source, the calibrated height times the scale the view applied to it)
                        hgt = paint.cal["height_m"] * S["k"][i] if np.isfinite(S["k"][i]) else mz
                        res[band] += paint.offsets(img, [q[0] for q in keep], [q[1] for q in keep],
                                                   paint.trim.get(fr, (0.0, 0.0)) if S["ext"][i] else (0.0, 0.0), hgt)
        for band, v in res.items():
            v = [q for q in v if q is not None]
            a_ = np.asarray(v, float)
            has = np.isfinite(a_)
            r[f"paint_{band}_n"] = len(a_) / max(1, nfr)
            r[f"paint_{band}_on"] = 100.0 * float((np.abs(a_[has]) <= 0.25).sum()) / max(1, len(a_))
            r[f"paint_{band}_none"] = 100.0 * float((~has).mean()) if len(a_) else float("nan")
            off = np.abs(a_[has])
            r[f"paint_{band}_off"] = float(np.median(off)) if len(off) else float("nan")
    return r


COLS = (("left %", "left", 0), ("right %", "right", 0), ("both %", "both", 0), ("outer %", "outer", 0),
        ("flicker in /min", "flicker_in", 1), ("flicker out /min", "flicker_out", 1), ("reach m", "reach", 0),
        ("step 10 m", "step_10", 3), ("step 30 m", "step_30", 3), ("jumps 30 /min", "jumps_30", 1),
        ("far 20-30 med", "far_20-30", 2), ("far 40-50 med", "far_40-50", 2), ("vs ref med", "ref_med", 2),
        ("vs ref >0.5 %", "ref_over", 1), ("outer invented %", "outer_invented", 1), ("outer missing %", "outer_missing", 1),
        ("paint near on %", "paint_near_on", 0), ("near none %", "paint_near_none", 0), ("near off m", "paint_near_off", 2),
        ("near pts/frame", "paint_near_n", 1),
        ("paint far on %", "paint_far_on", 0), ("far none %", "paint_far_none", 0), ("far off m", "paint_far_off", 2),
        ("far pts/frame", "paint_far_n", 1))


def fmt(r, key, nd):
    v = r.get(key)
    if isinstance(v, tuple):
        return "-" if not np.isfinite(v[0]) else f"{v[0]:.{nd}f} / {v[1]:.{nd}f}"
    return "-" if v is None or not np.isfinite(v) else f"{v:.{nd}f}"


def read_labels(path, source):
    out = []
    with open(path) as f:
        for row in csv.DictReader(f):
            if source and row["source"] != source:
                continue
            if row["confidence"] not in ("sure", "likely") or row["lanes_own"] in ("na", "") or row["ego_from_left"] in ("na", ""):
                continue
            out.append((float(row["t0"]), float(row["t1"]), int(row["lanes_own"]), int(row["ego_from_left"])))
    return out


def score(a):
    S = {}
    for spec in a.src:
        name, _, path = spec.partition("=")
        S[name] = dict(np.load(path))
    ego = ego_track([dict(np.load(p)) for p in a.ego.split(",")] if a.ego else list(S.values())[:1])
    labels = read_labels(a.labels, a.source) if a.labels else None
    paint = Paint(a) if a.paint else None
    res = {}
    for name, s in S.items():
        if paint is not None:
            paint.trim = {}
            tr = dict(kv.split("=", 1) for kv in a.trim_for)
            if name in tr:
                for m in read_jsonl(tr[name]):
                    paint.trim[m["seq"] - 1] = (m.get("pitch_trim", 0.0), m.get("yaw_trim", 0.0))
        res[name] = score_one(s, ego, S.get(a.ref) if a.ref and a.ref != name else None, labels, paint)
    print("| source | " + " | ".join(c for c, _, _ in COLS) + " |")
    print("|---|" + "---|" * len(COLS))
    for name, r in res.items():
        print(f"| {name} | " + " | ".join(fmt(r, k, nd) for _, k, nd in COLS) + " |")
    if a.json:
        json.dump(res, open(a.json, "w"), indent=1)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd", required=True)
    v = sub.add_parser("views")
    v.add_argument("road"); v.add_argument("out")
    v.add_argument("--laned", default=""); v.add_argument("--latency", type=float, default=0.05)
    v.add_argument("--view-set", action="append", default=[])
    s = sub.add_parser("score")
    s.add_argument("src", nargs="+")
    s.add_argument("--ego", default=""); s.add_argument("--ref", default="")
    s.add_argument("--labels", default=""); s.add_argument("--source", default="")
    s.add_argument("--paint", default=""); s.add_argument("--frame0", type=int, default=0); s.add_argument("--step", type=int, default=4)
    s.add_argument("--focal", type=float, default=975.0); s.add_argument("--cx", type=float, default=640.0)
    s.add_argument("--cy", type=float, default=360.0); s.add_argument("--calib", default="")
    s.add_argument("--trim-for", action="append", default=[],
                                                         help="NAME=DISPLAY_LANES.jsonl: that source's pitch trim per frame")
    s.add_argument("--json", default="")
    a = ap.parse_args()
    (views if a.cmd == "views" else score)(a)


if __name__ == "__main__":
    main()
