#!/usr/bin/env python3
"""Recorded sessions through the data integrator (docs/INTEGRATOR.md, docs/lanes/L37-integrator.md).

    integrator_replay.py characterise SESSION...          the sources as measured: vehicle speed (leave-one-out), wheels,
                                                          yaw-rate sources, accelerometer, camera lag and noise, the
                                                          camera's stated uncertainty, the feed's silences
    integrator_replay.py detect   SESSION... [--pos F]    fault detection: when, how long, by which monitor; per-source
                                                          state shares; the monitors' margins (good drives: false alarms)
    integrator_replay.py accuracy SESSION...              the ego state against held-out sources (vehicle held out
                                                          entirely, in 2 s gaps, every other sample; yaw rate)
    integrator_replay.py products SESSION --out F         the whole pipeline (integrator, lead, lanes, view) per frame
                              [--tree DIR] [--no-gate]    into F (JSON lines); --tree: another checkout's code (the
                                                          baseline before L37); --no-gate: fault detection off
    integrator_replay.py compare  A B [--label ...]       two `products` files of one session: lead / lanes / view
    integrator_replay.py summary  F...                    one `products` file: lead / lanes / view, split by the
                                                          integrator's verdict on the camera
    integrator_replay.py cost     SESSION                 CPU of the integrator alone and of the pipeline per frame
    integrator_replay.py mfd      SESSION --car-repo DIR  the api's stream through the car computer's scene builder
                              [--no-gate]                 (cyberpunk_computer/backend/roadsense_scene.py, imported
                                                          read-only): what the MFD is handed, by the camera's verdict

Sessions: recorder directories or their messages.jsonl.gz (a truncated gzip stream is read as far as it goes).
`--road-model vision` (the default) puts the model name back into `road` messages recorded before lane L37 (the
recorder dropped it; the trackers then ran their supercombo profile); `--road-model ""` leaves them alone.
`--pos F`: the car computer's trip-log GNSS lines (L35 audit, section 6) for sessions recorded before `gnss` was.
Times are printed in seconds from the first road frame of the session.
"""
import argparse
import bisect
import gzip
import importlib
import json
import os
import statistics as st
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
NS = 1e9


def tree_lib(tree=None):
    """replay_lib of a checkout (this one by default); it puts that checkout's api on sys.path."""
    sys.path.insert(0, os.path.join(tree, "tools") if tree else HERE)
    return importlib.import_module("replay_lib")


def name_of(path):
    p = os.path.normpath(path)
    return os.path.basename(os.path.dirname(p)) if p.endswith(".gz") else os.path.basename(p)


def pct(xs, ps, nd=2):
    xs = sorted(xs)
    return [round(xs[min(len(xs) - 1, int(p * len(xs)))], nd) for p in ps] if xs else None


def rsig(xs):
    """Robust sigma (1.4826 MAD) and median."""
    if len(xs) < 5:
        return None, None
    m = st.median(xs)
    return m, 1.4826 * st.median([abs(x - m) for x in xs])


class Veh:
    """The recorded vehicle speed / yaw rate as a time series (the reference a held-out test compares with)."""

    def __init__(self):
        self.t, self.v, self.r = [], [], []

    def add(self, m):
        if isinstance(m.get("ts_mono_ns"), int) and isinstance(m.get("speed_mps"), (int, float)):
            t = m["ts_mono_ns"] / NS
            if not self.t or t > self.t[-1]:
                self.t.append(t), self.v.append(float(m["speed_mps"])), self.r.append(m.get("yaw_rate_rps"))

    def at(self, t, k="v", max_gap=0.3):
        i = bisect.bisect_left(self.t, t)
        if i == 0 or i >= len(self.t) or self.t[i] - self.t[i - 1] > max_gap:
            return None
        a, b = getattr(self, k)[i - 1], getattr(self, k)[i]
        if a is None or b is None:
            return None
        return a + (b - a) * (t - self.t[i - 1]) / (self.t[i] - self.t[i - 1])


def messages(rl, a, path):
    pos = rl.load_pos(a.pos) if getattr(a, "pos", None) else None
    return rl.read_session(path, road_model=a.road_model or None, pos=pos)


# ---- characterise ---------------------------------------------------------------------------------------------
def cmd_characterise(a):
    import numpy as np
    rl = tree_lib()
    A = lambda rows: np.array([[np.nan if x is None else x for x in r] for r in rows], float)
    mad = lambda x: 1.4826 * np.median(np.abs(x - np.median(x))) if len(x) else float("nan")
    for path in a.sessions:
        ego, road = [], []
        for m in rl.read_lines(path):
            ts = m.get("ts_mono_ns")
            if not isinstance(ts, int):
                continue
            if m.get("type") == "ego":
                ws = m.get("wheel_speeds_mps") or [None] * 4
                ego.append((ts / NS, m.get("speed_mps"), m.get("yaw_rate_rps"), m.get("accel_long_mps2"), ws[0], ws[1], ws[2], ws[3]))
            elif m.get("type") == "road":
                p = m.get("pose") or {}
                road.append((ts / NS, p.get("vx"), p.get("vx_std"), p.get("yaw_rate")))
        r = A(road)
        tr, vx, vxs = r[:, 0], r[:, 1], r[:, 2]
        print("== %s: road frames %d, vehicle samples %d" % (name_of(path), len(road), len(ego)))
        own = np.abs(vx) >= 1.0
        if own.sum() > 50:
            q = (vxs / (0.25 + 0.035 * np.abs(vx)))[own]
            print("  camera's stated vx_std / characterised sigma (frames with |vx| >= 1): p5/p50/p95/p99 %s" % np.round(np.percentile(q, [5, 50, 95, 99]), 2))
        if len(ego) < 100:
            continue
        e = A(ego)
        te, ve, ye, al, ws = e[:, 0], e[:, 1], e[:, 2], e[:, 3], e[:, 4:8]
        d = np.diff(te)
        ok = (d[:-1] < 0.25) & (d[1:] < 0.25) & (ve[1:-1] > 0.5)
        pred = ve[:-2] + (ve[2:] - ve[:-2]) * (te[1:-1] - te[:-2]) / (te[2:] - te[:-2])
        res = (ve[1:-1] - pred)[ok]
        print("  vehicle speed, one sample left out and interpolated from its neighbours: robust sigma %.3f m/s, p99 |.| %.3f (n %d)" % (
            mad(res), np.percentile(np.abs(res), 99), len(res)))
        mv = ve > 3
        print("  wheel speed / speed signal, per wheel (median): %s; mean wheel speed / 0.98 - speed: robust sigma %.3f m/s" % (
            np.round(np.nanmedian(ws[mv] / ve[mv, None], axis=0), 4), mad(np.nanmean(ws[mv], axis=1) / 0.98 - ve[mv])))
        turn = mv & (np.abs(ye) > 0.08)
        if turn.sum() > 20:
            yw = (ws[:, 3] - ws[:, 2]) / 1.48
            print("  rear wheel-difference yaw rate against the gyro: gain in turns %.3f (n %d), robust sigma %.4f rad/s" % (
                np.sum(yw[turn] * ye[turn]) / np.sum(ye[turn] ** 2), turn.sum(), mad((yw - ye)[mv])))
        for lag in (0.0, 0.125, 0.25):
            vi, yi = np.interp(tr - lag, te, ve), np.interp(tr - lag, te, ye)
            gap = np.interp(tr - lag, te[1:], d)
            okf = (vi > 3) & (gap < 0.3) & (tr - lag > te[0]) & (tr - lag < te[-1])
            dv, dy = (vx - vi)[okf], (-r[:, 3] - yi)[okf]
            tf = okf & (np.abs(yi) > 0.08)
            print("  camera pose taken %.3f s before the frame stamp: speed median %+.3f robust sigma %.3f m/s; yaw rate median %+.4f "
                  "sigma %.4f rad/s, gain in turns %.3f, rms in turns %.4f (n %d)" % (
                      lag, np.median(dv), mad(dv), np.median(dy), mad(dy),
                      np.sum((-r[:, 3])[tf] * yi[tf]) / max(np.sum(yi[tf] ** 2), 1e-9), np.sqrt(np.mean(((-r[:, 3]) - yi)[tf] ** 2)) if tf.any() else float("nan"), tf.sum()))
        i = np.arange(2, len(te) - 2)
        dt = te[i + 2] - te[i - 2]
        oka = (dt < 0.5) & (ve[i] > 1) & ~np.isnan(al[i])
        dvdt = (ve[i + 2] - ve[i - 2]) / dt
        print("  accelerometer - d(speed)/dt: median %+.3f robust sigma %.3f m/s2, correlation %.2f" % (
            np.median((al[i] - dvdt)[oka]), mad((al[i] - dvdt)[oka]), np.corrcoef(al[i][oka], dvdt[oka])[0, 1]))
        for T in (0.5, 1.0, 2.0):
            vt = np.interp(te + T, te, ve)
            okb = (ve > 2) & (te + T < te[-1]) & ~np.isnan(al)
            print("  bridging %.1f s: rms error holding the speed %.3f, speed + accelerometer %.3f m/s" % (
                T, np.sqrt(np.mean((vt - ve)[okb] ** 2)), np.sqrt(np.mean((vt - ve - al * T)[okb] ** 2))))
        gi = np.where(d > 1.0)[0]
        stale = 0
        worst = 0.0
        for k in gi:
            j = np.arange(k + 1, min(k + 4, len(te)))
            j = j[te[j] - te[k + 1] <= 0.3]
            later = np.where(te - te[k + 1] >= 0.35)[0]
            if not len(later) or later[0] + 2 >= len(te):
                continue
            # the speed the feed settles to, extended back with its own slope, against its first samples
            b = later[0]
            slope = (ve[b + 2] - ve[b]) / max(te[b + 2] - te[b], 1e-3)
            err = np.abs(ve[j] - (ve[b] + slope * (te[j] - te[b])))
            if err.max() > 1.0:
                stale += 1
            worst = max(worst, err.max())
        print("  vehicle feed silences > 1 s: %d%s; resumes whose first samples (0.3 s) are more than 1 m/s off where the feed "
              "settles: %d, worst %.1f m/s" % (len(gi), "" if not len(gi) else " (length median %.1f s, max %.1f s)" % (
                  np.median(d[gi]), d[gi].max()), stale, worst))


# ---- detect ---------------------------------------------------------------------------------------------------
def cmd_detect(a):
    rl = tree_lib()
    from roadsense_api.integrator import Integrator
    for path in a.sessions:
        integ = Integrator()
        veh = Veh()
        t0 = first_ev = None
        rows = []                  # (t, vehicle speed at the frame or None, vision state, fault reason)
        states = {}
        marg = {}                  # check id -> max share / ratio seen
        gn = 0
        for m in messages(rl, a, path):
            t = m.get("type")
            if t == "ego":
                veh.add(m)
            if t == "gnss":
                gn += 1
            if t in ("ego", "vehicle", "gnss"):
                integ.on_message(m)
            if t != "road" or not isinstance(m.get("ts_mono_ns"), int):
                continue
            es = integ.on_road(m)
            T = m["ts_mono_ns"] / NS
            if t0 is None:
                t0 = T
            rows.append((T, es))
            for g, s in es["sources"].items():
                d = states.setdefault(g, {})
                d[s["state"]] = d.get(s["state"], 0) + 1
            for c in es["checks"]:
                key = "%s/%s %s %s" % (c["a"], c.get("b", "self"), c["quantity"], c["kind"])
                d = marg.setdefault(key, {"n": 0, "share": [], "ratio": [], "verdicts": {}})
                d["verdicts"][c["verdict"]] = d["verdicts"].get(c["verdict"], 0) + 1
                if c["n"] >= 15 or (c["kind"] == "witness" and c["n"] >= 10):
                    d["n"] += 1
                    if c.get("share") is not None:
                        d["share"].append(c["share"])
                    if c.get("ratio") is not None:
                        d["ratio"].append(c["ratio"])
        if not rows:
            continue
        n = len(rows)
        print("== %s: %d road frames, %.0f s, vehicle samples %d, gnss messages %d" % (
            name_of(path), n, rows[-1][0] - t0, len(veh.t), gn))
        # evidence = frames while the vehicle (the reference) moves at 3 m/s or more
        mov = [(T, es) for T, es in rows if (veh.at(T) or 0.0) >= 3.0]
        fault = [(T, es) for T, es in rows if es["sources"]["vision"]["state"] == "fault"]
        print("  per-source state, share of frames: " + "; ".join(
            "%s %s" % (g, " ".join("%s %.3f" % (k, v / n) for k, v in sorted(d.items()))) for g, d in states.items()))
        if mov:
            first_ev = mov[0][0]
            mf = sum(1 for _, es in mov if es["sources"]["vision"]["state"] == "fault")
            print("  moving frames (vehicle >= 3 m/s): %d (%.0f s); vision `fault` in %.1f %% of them, `suspect` in %.1f %%" % (
                len(mov), len(mov) / max(n, 1) * (rows[-1][0] - t0), 100.0 * mf / len(mov),
                100.0 * sum(1 for _, es in mov if es["sources"]["vision"]["state"] == "suspect") / len(mov)))
        if fault:
            f0 = fault[0]
            print("  first vision fault at %.1f s%s: %s" % (
                f0[0] - t0, "" if first_ev is None else " (%.1f s %s the first moving frame at %.1f s)" % (
                    abs(f0[0] - first_ev), "after" if f0[0] >= first_ev else "BEFORE", first_ev - t0),
                f0[1]["sources"]["vision"]["reason"]))
            # intervals
            iv, cur = [], None
            for T, es in rows:
                f = es["sources"]["vision"]["state"] == "fault"
                if f and cur is None:
                    cur = [T, T]
                elif f:
                    cur[1] = T
                elif cur is not None:
                    iv.append(cur)
                    cur = None
            if cur is not None:
                iv.append(cur)
            print("  fault intervals (s): " + ", ".join("%.1f-%.1f" % (x - t0, y - t0) for x, y in iv[:12]) +
                  (" ... (%d)" % len(iv) if len(iv) > 12 else ""))
            rules = {}
            for _, es in fault:
                k = es["sources"]["vision"].get("isolation")
                rules[k] = rules.get(k, 0) + 1
            print("  isolation rule, share of fault frames: " + ", ".join("%s %.2f" % (k, v / len(fault)) for k, v in rules.items()))
            if mov:
                after = [(T, es) for T, es in mov if T >= f0[0]]
                ok = sum(1 for _, es in after if es["sources"]["vision"]["state"] != "fault")
                print("  after the first fault: moving frames not in fault %.1f %% (%d of %d)" % (
                    100.0 * ok / max(len(after), 1), ok, len(after)))
        else:
            print("  vision fault: never")
        for key, d in sorted(marg.items()):
            line = "  check %-32s verdicts %s" % (key, d["verdicts"])
            if d["share"]:
                line += "; share beyond 4 sigma p50/p99/max %s" % pct(d["share"], (0.5, 0.99, 1.0))
            if d["ratio"]:
                line += "; ratio p1/p50/p99/max %s" % pct(d["ratio"], (0.01, 0.5, 0.99, 1.0))
            print(line)
        es = rows[-1][1]
        print("  rejected (innovation gate) %s, late %s; vehicle feed silences > 1 s (warm-ups): %s" % (
            es["rejected"], es["late"], es["sources"].get("vehicle", {}).get("warmups")))


# ---- accuracy -------------------------------------------------------------------------------------------------
def cmd_accuracy(a):
    rl = tree_lib()
    from roadsense_api.integrator import Integrator
    for path in a.sessions:
        msgs = [m for m in messages(rl, a, path) if m.get("type") in ("ego", "road", "vehicle")
                and isinstance(m.get("ts_mono_ns"), int)]
        veh = Veh()
        for m in msgs:
            if m["type"] == "ego":
                veh.add(m)
        print("== %s: vehicle samples %d" % (name_of(path), len(veh.t)))
        if len(veh.t) < 100:
            print("  no vehicle feed: nothing to hold out")
            continue

        def run(keep):
            """Integrator fed the messages `keep` lets through; per road frame (T, state, raw vision speed)."""
            integ = Integrator()
            out = []
            for i, m in enumerate(msgs):
                if m["type"] == "road":
                    es = integ.on_road(m)
                    p = m.get("pose") or {}
                    out.append((m["ts_mono_ns"] / NS, es, p.get("vx"), p.get("yaw_rate")))
                elif keep(i, m):
                    integ.on_message(m)
            return out

        def report(label, rows, use=lambda T: True, moving=1.0):
            ev, er, ey, ry, cover, z = [], [], [], [], 0, []
            n = 0
            for T, es, vx, yr in rows:
                ref = veh.at(T)
                if ref is None or abs(ref) < moving or not use(T):
                    continue
                n += 1
                sp = es["speed"]
                if sp["valid"]:
                    cover += 1
                    ev.append(sp["value"] - ref)
                    if sp["std"]:
                        z.append((sp["value"] - ref) / sp["std"])
                if vx is not None:
                    er.append(vx - ref)
                rref = veh.at(T, "r")
                if rref is not None and es["yaw_rate"]["valid"]:
                    ey.append(es["yaw_rate"]["value"] - rref)
                    if yr is not None:
                        ry.append(-yr - rref)
            if not n:
                print("  %s: no frames" % label)
                return
            m1, s1 = rsig(ev)
            m2, s2 = rsig(er)
            rms = lambda x: (sum(v * v for v in x) / len(x)) ** 0.5 if x else float("nan")
            line = "  %-46s n %5d valid %.3f | speed err median %+.3f robust sigma %.3f rms %.3f p99|.| %.2f" % (
                label, n, cover / n, m1 or 0, s1 or 0, rms(ev), pct([abs(x) for x in ev], (0.99,))[0] if ev else 0)
            if z:
                line += " | err/std rms %.2f" % rms(z)
            line += " || raw vision at the frame: median %+.3f sigma %.3f rms %.3f" % (m2 or 0, s2 or 0, rms(er))
            print(line)
            if ey:
                my, sy = rsig(ey)
                mr, sr = rsig(ry)
                print("  %-46s yaw rate err median %+.4f sigma %.4f rms %.4f || raw vision: median %+.4f sigma %.4f rms %.4f" % (
                    "", my, sy, rms(ey), mr or 0, sr or 0, rms(ry)))

        # 1. the vehicle held out entirely: the estimate is the camera's (a car without a vehicle feed)
        report("vehicle held out (camera only)", run(lambda i, m: m["type"] != "ego"))
        # 2. every other vehicle sample held out: frames nearer to a held-out sample than to a kept one
        kept, held, k = [], [], 0
        for m in msgs:
            if m["type"] == "ego":
                (kept if k % 2 == 0 else held).append(m["ts_mono_ns"] / NS)
                m["_keep2"] = k % 2 == 0
                k += 1

        def near_held(T):
            i, j = bisect.bisect_left(held, T), bisect.bisect_left(kept, T)
            dh = min(abs(T - held[x]) for x in (i - 1, i) if 0 <= x < len(held))
            dk = min(abs(T - kept[x]) for x in (j - 1, j) if 0 <= x < len(kept))
            return dh < dk

        report("every other vehicle sample held out", run(lambda i, m: m["type"] != "ego" or m["_keep2"]), near_held)
        # 3. 2 s gaps of the vehicle feed every 20 s: the camera and the accelerometer state bridge them
        t_first = veh.t[0]
        gap = lambda t: (t - t_first) % 20.0 >= 18.0
        report("2 s vehicle gaps every 20 s (inside the gaps)",
               run(lambda i, m: m["type"] != "ego" or not gap(m["ts_mono_ns"] / NS)), lambda T: gap(T) and (T - t_first) % 20.0 >= 18.3)
        # 4. everything in: the estimate against the vehicle speed interpolated at the frame time (what it is made of)
        report("all sources (against the interpolated vehicle)", run(lambda i, m: True))


# ---- products -------------------------------------------------------------------------------------------------
def cmd_products(a):
    rl = tree_lib(a.tree)
    new = hasattr(rl, "load_pos")
    p = rl.Pipeline(integrator=not a.no_gate) if new else rl.Pipeline()
    src = messages(rl, a, a.session) if new else rl.read_session(a.session)
    veh = Veh()
    n = 0
    cpu = 0.0
    with gzip.open(a.out, "wt") if a.out.endswith(".gz") else open(a.out, "w") as f:
        for m in src:
            if m.get("type") == "road" and a.road_model and "road_model" not in m:
                m["road_model"] = a.road_model
            if m.get("type") == "ego":
                veh.add(m)
            if m.get("type") == "gnss" and not new:
                continue
            c = time.process_time()
            outs = p.feed(m)
            cpu += time.process_time() - c
            if m.get("type") != "road" or not outs:
                continue
            n += 1
            row = {"t": m["ts_mono_ns"] / NS}
            raw = (m.get("leads") or [{}])[0]
            row["raw_lead"] = [raw.get("prob"), (raw.get("x") or [None])[0]]
            for o in outs:
                t = o.get("type")
                if t == "ego_state":
                    row["vision"] = o["sources"]["vision"]["state"]
                    row["speed"] = [o["speed"]["value"], o["speed"]["std"], o["speed"]["mode"]]
                elif t == "lead":
                    row["lead"] = [o.get("present"), o.get("dist_m"), o.get("v_rel_mps"), o.get("track_id"), o.get("mode"),
                                   o.get("valid", True), o.get("v_lead_mps"), (o.get("ego") or {}).get("speed_mps")]
                elif t == "lanes":
                    row["lanes"] = ["".join("|" if ln.get("present") else "." for ln in o["lines"] + o["edges"]),
                                    o.get("lane_width_m"), o.get("valid", True)]
                    lines = o["lines"]
                    row["ly"] = [ln["y"][8] if ln.get("present") and ln.get("y") else None for ln in lines[1:3]]
                elif t == "view":
                    ld = [ob for ob in o.get("objects") or [] if ob.get("role") == "lead"]
                    row["view"] = [round(ld[0]["x"], 1) if ld else None,
                                   sum(1 for ln in o.get("lines") or [] if ln.get("y")),
                                   (o.get("integrity") or {}).get("valid", True), o.get("path") is not None]
            f.write(json.dumps(row, separators=(",", ":")) + "\n")
    # the vehicle speed is written afterwards (samples arrive around the frames): one line per second is enough
    with open(a.out + ".veh", "w") as f:
        json.dump({"t": veh.t, "v": veh.v}, f)
    print("%s: %d frames -> %s; pipeline CPU %.1f s (%.3f ms per road frame)" % (name_of(a.session), n, a.out, cpu, 1e3 * cpu / max(n, 1)))


def load_products(path):
    rows = []
    with gzip.open(path, "rt") if path.endswith(".gz") else open(path) as f:
        for ln in f:
            rows.append(json.loads(ln))
    veh = Veh()
    try:
        with open(path + ".veh") as f:
            d = json.load(f)
        veh.t, veh.v, veh.r = d["t"], d["v"], [None] * len(d["t"])
    except OSError:
        pass
    return rows, veh


def lead_stats(rows):
    pres = [r for r in rows if (r.get("lead") or [False])[0]]
    d = [r["lead"][1] for r in pres if r["lead"][1] is not None]
    ids = set(r["lead"][3] for r in pres)
    # longest run of one track id
    best = cur = 0.0
    last = None
    t0 = None
    for r in rows:
        tid = r["lead"][3] if (r.get("lead") or [False])[0] else None
        if tid is not None and tid == last:
            cur = r["t"] - t0
        else:
            last, t0, cur = tid, r["t"], 0.0
        best = max(best, cur)
    return {"present": len(pres) / max(len(rows), 1), "dist": pct(d, (0.1, 0.5, 0.9), 1), "tracks": len(ids), "longest_s": round(best, 1)}


def cmd_summary(a):
    for path in a.files:
        rows, veh = load_products(path)
        print("== %s: %d frames" % (path, len(rows)))
        groups = {"all frames": rows}
        if veh.t:
            groups["vehicle >= 5 m/s"] = [r for r in rows if (veh.at(r["t"]) or 0.0) >= 5.0]
            groups["vehicle stands"] = [r for r in rows if veh.at(r["t"]) is not None and abs(veh.at(r["t"])) < 0.05]
        for vs in sorted(set(r.get("vision") for r in rows if r.get("vision"))):
            groups["vision %s" % vs] = [r for r in rows if r.get("vision") == vs]
            if veh.t:
                groups["vision %s, vehicle >= 5 m/s" % vs] = [r for r in rows if r.get("vision") == vs and (veh.at(r["t"]) or 0.0) >= 5.0]
        for k, g in groups.items():
            if not g:
                continue
            ls = lead_stats(g)
            raw = [r["raw_lead"][1] for r in g if r.get("raw_lead") and (r["raw_lead"][0] or 0) >= 0.5 and r["raw_lead"][1] is not None]
            lv = sum(1 for r in g if r.get("lead") and r["lead"][5] is False) / len(g)
            ln = [r for r in g if r.get("lanes")]
            inner = sum(1 for r in ln if r["lanes"][0][1] == "|" and r["lanes"][0][2] == "|") / max(len(ln), 1)
            lnv = sum(1 for r in ln if r["lanes"][2] is False) / max(len(ln), 1)
            vw = [r for r in g if r.get("view")]
            vl = sum(1 for r in vw if r["view"][0] is not None) / max(len(vw), 1)
            vd = pct([r["view"][0] for r in vw if r["view"][0] is not None], (0.1, 0.5, 0.9), 1)
            vlin = sum(1 for r in vw if r["view"][1] > 0) / max(len(vw), 1)
            vinv = sum(1 for r in vw if r["view"][2] is False) / max(len(vw), 1)
            print("  %-34s n %6d | raw leads[0] p>=0.5: %.2f dist p10/50/90 %s | lead present %.3f dist %s tracks %d longest %s s invalid %.3f"
                  " | lanes both inner %.3f invalid %.3f | view lead %.3f dist %s lines %.3f integrity false %.3f" % (
                      k, len(g), len(raw) / len(g), pct(raw, (0.1, 0.5, 0.9), 1), ls["present"], ls["dist"], ls["tracks"],
                      ls["longest_s"], lv, inner, lnv, vl, vd, vlin, vinv))


def cmd_compare(a):
    ra, va = load_products(a.a)
    rb, vb = load_products(a.b)
    veh = va if va.t else vb
    bt = {round(r["t"], 4): r for r in rb}
    pairs = [(r, bt[round(r["t"], 4)]) for r in ra if round(r["t"], 4) in bt]
    la, lb = a.label[0], a.label[1]
    print("frames: %s %d, %s %d, common %d" % (la, len(ra), lb, len(rb), len(pairs)))
    sets = {"all": pairs}
    if veh.t:
        sets["vehicle >= 5 m/s"] = [p for p in pairs if (veh.at(p[0]["t"]) or 0.0) >= 5.0]
        sets["vehicle stands"] = [p for p in pairs if veh.at(p[0]["t"]) is not None and abs(veh.at(p[0]["t"])) < 0.05]
    for name, ps in sets.items():
        if not ps:
            continue
        A, B = lead_stats([p[0] for p in ps]), lead_stats([p[1] for p in ps])
        both = [(x, y) for x, y in ps if x["lead"][0] and y["lead"][0]]
        only_a = sum(1 for x, y in ps if x["lead"][0] and not y["lead"][0])
        only_b = sum(1 for x, y in ps if y["lead"][0] and not x["lead"][0])
        dd = [y["lead"][1] - x["lead"][1] for x, y in both]
        dv = [y["lead"][2] - x["lead"][2] for x, y in both if x["lead"][2] is not None and y["lead"][2] is not None]
        print("-- %s (%d frames)" % (name, len(ps)))
        print("   lead %-10s present %.3f dist p10/50/90 %s tracks %d longest %.1f s" % (la, A["present"], A["dist"], A["tracks"], A["longest_s"]))
        print("   lead %-10s present %.3f dist p10/50/90 %s tracks %d longest %.1f s" % (lb, B["present"], B["dist"], B["tracks"], B["longest_s"]))
        print("   lead present in both %d, only %s %d (%.2f %%), only %s %d (%.2f %%)" % (
            len(both), la, only_a, 100.0 * only_a / len(ps), lb, only_b, 100.0 * only_b / len(ps)))
        if dd:
            print("   where both: dist %s - %s: median %+.3f m, |.| p50/p90/p99 %s m; v_rel: median %+.3f, |.| p50/p90/p99 %s m/s" % (
                lb, la, st.median(dd), pct([abs(x) for x in dd], (0.5, 0.9, 0.99)), st.median(dv) if dv else 0,
                pct([abs(x) for x in dv], (0.5, 0.9, 0.99))))
        # jitter of the tracked distance (second difference per frame), both
        for lab, k in ((la, 0), (lb, 1)):
            j = []
            prev = None
            for p in ps:
                r = p[k]
                if r["lead"][0] and prev is not None and prev["lead"][0] and prev["lead"][3] == r["lead"][3] and r["t"] - prev["t"] < 0.2:
                    j.append(abs((r["lead"][6] or 0.0) - (prev["lead"][6] or 0.0)))
                prev = r
            if j:
                print("   lead speed change per frame %-10s p50/p90/p99 %s m/s" % (lab, pct(j, (0.5, 0.9, 0.99), 3)))
        ln = [(x, y) for x, y in ps if x.get("lanes") and y.get("lanes")]
        same = sum(1 for x, y in ln if x["lanes"][0] == y["lanes"][0])
        ia = sum(1 for x, y in ln if x["lanes"][0][1:3] == "||") / max(len(ln), 1)
        ib = sum(1 for x, y in ln if y["lanes"][0][1:3] == "||") / max(len(ln), 1)
        dy = [abs(u - v) for x, y in ln for u, v in zip(x.get("ly") or [], y.get("ly") or []) if u is not None and v is not None]
        print("   lanes: presence pattern identical in %.2f %% of %d frames; both inner lines %s %.3f, %s %.3f; inner line offset "
              "at mid range |diff| p50/p99/max %s m" % (100.0 * same / max(len(ln), 1), len(ln), la, ia, lb, ib, pct(dy, (0.5, 0.99, 1.0), 3)))
        vw = [(x, y) for x, y in ps if x.get("view") and y.get("view")]
        if vw:
            print("   view: lead shown %s %.3f, %s %.3f; lines shown %s %.3f, %s %.3f" % (
                la, sum(1 for x, _ in vw if x["view"][0] is not None) / len(vw), lb, sum(1 for _, y in vw if y["view"][0] is not None) / len(vw),
                la, sum(1 for x, _ in vw if x["view"][1] > 0) / len(vw), lb, sum(1 for _, y in vw if y["view"][1] > 0) / len(vw)))


# ---- mfd ------------------------------------------------------------------------------------------------------
def cmd_mfd(a):
    rl = tree_lib()
    import importlib.util
    import logging
    logging.disable(logging.CRITICAL)
    sys.dont_write_bytecode = True       # the car repo is only read: the one module, by its file, no package import
    spec = importlib.util.spec_from_file_location(
        "car_roadsense_scene", os.path.join(a.car_repo, "cyberpunk_computer", "backend", "roadsense_scene.py"))
    mod = importlib.util.module_from_spec(spec)
    sys.modules["car_roadsense_scene"] = mod
    spec.loader.exec_module(mod)
    SceneBuilder, SceneFeedConfig = mod.SceneBuilder, mod.SceneFeedConfig
    b = SceneBuilder(SceneFeedConfig())
    p = rl.Pipeline(integrator=not a.no_gate)
    veh = Veh()
    stats = {}
    vision = None
    for m in messages(rl, a, a.session):
        if m.get("type") == "ego":
            veh.add(m)
        ts = m.get("ts_mono_ns")
        for o in p.feed(m):
            if o.get("type") == "ego_state":
                vision = o["sources"]["vision"]["state"]
            if not isinstance(ts, int):
                continue
            now = ts / NS
            b.feed(json.dumps(o, separators=(",", ":")), now)
            if o.get("type") != "view":
                continue
            sc = b.build(now)
            if sc is None:
                continue
            v = veh.at(now)
            for key in ("vision %s" % vision, "vision %s, vehicle >= 5 m/s" % vision if v is not None and v >= 5.0 else None):
                if key is None:
                    continue
                d = stats.setdefault(key, {"n": 0, "lead": 0, "vw_lead": 0, "lanes": 0, "d": []})
                d["n"] += 1
                ld = sc.get("lead")
                if ld:
                    d["lead"] += 1
                    d["d"].append(ld.get("d"))
                vw = sc.get("vw") or {}
                d["vw_lead"] += 1 if vw.get("lead") else 0
                d["lanes"] += 1 if any(vw.get("lanes") or []) or any(sc.get("lanes") or []) else 0
    print("== %s through the car's SceneBuilder (%s)" % (name_of(a.session), "fault detection off" if a.no_gate else "integrator"))
    for k, d in sorted(stats.items()):
        print("  %-36s scenes %6d | scene.lead %.3f (dist p10/50/90 %s) | drawn lead (vw.lead) %.3f | any lane line %.3f" % (
            k, d["n"], d["lead"] / d["n"], pct([x for x in d["d"] if x is not None], (0.1, 0.5, 0.9), 1), d["vw_lead"] / d["n"],
            d["lanes"] / d["n"]))


# ---- cost -----------------------------------------------------------------------------------------------------
def cmd_cost(a):
    rl = tree_lib()
    from roadsense_api.integrator import Integrator
    msgs = [m for m in messages(rl, a, a.session) if isinstance(m.get("ts_mono_ns"), int)]
    integ = Integrator()
    n = k = 0
    c0 = time.process_time()
    for m in msgs:
        t = m["type"]
        if t == "road":
            integ.on_road(m)
            n += 1
        elif t in ("ego", "vehicle", "gnss"):
            integ.on_message(m)
            k += 1
    cpu = time.process_time() - c0
    span = (msgs[-1]["ts_mono_ns"] - msgs[0]["ts_mono_ns"]) / NS
    print("%s: integrator alone: %.2f s CPU for %d road frames + %d vehicle / gnss messages over %.0f s of driving: "
          "%.3f ms per road frame (all its inputs included), %.2f %% of one core of this machine" % (
              name_of(a.session), cpu, n, k, span, 1e3 * cpu / max(n, 1), 100.0 * cpu / span))


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd", required=True)
    for name, multi in (("characterise", True), ("detect", True), ("accuracy", True), ("cost", False), ("products", False),
                        ("mfd", False)):
        p = sub.add_parser(name)
        p.add_argument("sessions" if multi else "session", **({"nargs": "+"} if multi else {}))
        p.add_argument("--pos")
        p.add_argument("--road-model", default="vision")
        if name == "products":
            p.add_argument("--out", required=True)
            p.add_argument("--tree")
            p.add_argument("--no-gate", action="store_true")
        if name == "mfd":
            p.add_argument("--car-repo", required=True)
            p.add_argument("--no-gate", action="store_true")
    p = sub.add_parser("compare")
    p.add_argument("a")
    p.add_argument("b")
    p.add_argument("--label", nargs=2, default=["A", "B"])
    p = sub.add_parser("summary")
    p.add_argument("files", nargs="+")
    a = ap.parse_args()
    globals()["cmd_" + a.cmd](a)


if __name__ == "__main__":
    main()
