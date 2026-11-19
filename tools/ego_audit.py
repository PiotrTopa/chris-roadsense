#!/usr/bin/env python3
"""Source characterisation of the ego-motion inputs on recorded drives (lane L35 audit; analysis only).

Reads recorder sessions (`messages.jsonl.gz`, tolerant of a truncated gzip stream) and, optionally, the car
computer's feeder GNSS lines (`--pos`: lines of its trip logs with `"dir":"POS"`, extracted on the phone with
`grep '"dir":"POS"' ~/cyberpunk/logs/trips/trip_YYYYMMDD_*.ndjson`; the feeder's `ts_mono_ns` is the phone's
CLOCK_MONOTONIC, the same clock as roadsense's). Nothing here is used by the stack.

    tools/ego_audit.py rates    SESSION...            stream rates, gaps, arrival order (road behind ego)
    tools/ego_audit.py vision   SESSION...            road-model pose vs vehicle speed / yaw rate: bias, noise, lag
    tools/ego_audit.py gnss     SESSION... --pos F    GNSS speed / course vs vehicle and vision, lag
    tools/ego_audit.py incident SESSION...            what a consistency check on vision speed would have seen

All times printed relative to the session's first road frame, in seconds.
"""
import argparse
import bisect
import math
import os
import statistics as st
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from replay_lib import read_lines  # noqa: E402

NS = 1e9


def q(xs, ps=(0.01, 0.05, 0.5, 0.95, 0.99), nd=3):
    xs = sorted(xs)
    n = len(xs)
    return [round(xs[min(n - 1, int(p * n))], nd) for p in ps] if xs else None


def robust(xs):
    """(median, 1.4826 * MAD)."""
    if len(xs) < 5:
        return None, None
    m = st.median(xs)
    return m, 1.4826 * st.median([abs(x - m) for x in xs])


def load(path):
    s = {"ego": [], "road": [], "map": [], "calib": [], "vehicle": [], "camera": [], "order": []}
    last_ego = None
    for m in read_lines(path):
        t = m.get("type")
        ts = m.get("ts_mono_ns")
        if t == "ego" and isinstance(ts, int):
            s["ego"].append((ts / NS, m.get("speed_mps"), m.get("yaw_rate_rps"), m.get("accel_long_mps2"),
                             bool(m.get("standstill"))))
            last_ego = ts if last_ego is None else max(last_ego, ts)
        elif t == "road" and isinstance(ts, int):
            p = m.get("pose") or {}
            ld = (m.get("leads") or [{}])[0]
            s["road"].append(dict(t=ts / NS, vx=p.get("vx"), vxs=p.get("vx_std"),
                                  r=None if p.get("yaw_rate") is None else -p["yaw_rate"],   # positive left
                                  rs=p.get("yaw_rate_std"), vy=p.get("vy"), vz=p.get("vz"),
                                  lx=(ld.get("x") or [None])[0], lp=ld.get("prob"), lv=(ld.get("v") or [None])[0],
                                  ms=m.get("model_ms"), skip=m.get("frames_skipped")))
            if last_ego is not None:
                s["order"].append((last_ego - ts) / NS)     # newest vehicle time already written when the frame was
        elif t in ("map", "calib", "vehicle", "camera") and isinstance(ts, int):
            s[t].append(m)
    s["ego"].sort()
    s["road"].sort(key=lambda r: r["t"])
    return s


class Interp:
    def __init__(self, rows, k, max_gap=0.3):
        self.t = [r[0] for r in rows]
        self.v = [r[k] for r in rows]
        self.g = max_gap

    def __call__(self, t):
        i = bisect.bisect_left(self.t, t)
        if i == 0 or i >= len(self.t):
            return None
        a, b = i - 1, i
        if self.t[b] - self.t[a] > self.g or self.v[a] is None or self.v[b] is None:
            return None
        f = (t - self.t[a]) / (self.t[b] - self.t[a])
        return self.v[a] + f * (self.v[b] - self.v[a])


def gaps(ts, lim):
    d = [b - a for a, b in zip(ts, ts[1:])]
    return d, sum(1 for x in d if x > lim), max(d) if d else None


# ---- rates ------------------------------------------------------------------------------------------
def cmd_rates(a):
    for path in a.sessions:
        s = load(path)
        rt = [r["t"] for r in s["road"]]
        print("==", os.path.basename(os.path.normpath(path)), "duration %.0f s" % (rt[-1] - rt[0] if rt else 0))
        for name, ts in (("road", rt), ("ego", [e[0] for e in s["ego"]]),
                         ("map", [m["ts_mono_ns"] / NS for m in s["map"]]),
                         ("vehicle", [m["ts_mono_ns"] / NS for m in s["vehicle"]]),
                         ("camera(1/s rec)", [m["ts_mono_ns"] / NS for m in s["camera"]])):
            if len(ts) < 2:
                print("  %-16s n %d" % (name, len(ts)))
                continue
            d, g, mx = gaps(sorted(ts), 0.5)
            print("  %-16s n %6d  rate %5.1f/s  dt p50 %.3f p99 %.3f max %.1f s  gaps>0.5s %d" % (
                name, len(ts), (len(ts) - 1) / (ts[-1] - ts[0]), st.median(d), q(d, (0.99,))[0], mx, g))
        if s["road"]:
            print("  road frames_skipped p50/p99 %s  model_ms %s" % (
                q([r["skip"] or 0 for r in s["road"]], (0.5, 0.99)), q([r["ms"] for r in s["road"] if r["ms"]], (0.5, 0.99), 1)))
        if s["order"]:
            print("  vehicle time already recorded when a road frame was written (s, = road arrival lag lower bound): %s"
                  % q(s["order"], (0.05, 0.5, 0.95, 0.99)))
        mp = s["map"]
        if mp:
            fixes = [((m.get("input") or {}).get("gnss") or {}).get("fix") for m in mp]
            hacc = [((m.get("input") or {}).get("gnss") or {}).get("hacc_m") for m in mp]
            print("  map: gnss fix share %s, hacc_m %s, states %s" % (
                {k: round(fixes.count(k) / len(fixes), 2) for k in set(fixes)},
                q([h for h in hacc if h is not None], (0.05, 0.5, 0.95)), sorted({m.get("state") for m in mp})))
        for c in s["calib"][:1]:
            print("  calib pitch %s yaw %s height %s focal %s source %s online %s" % (
                c.get("pitch"), c.get("yaw"), c.get("height_m"), c.get("focal_px"), c.get("source"),
                (c.get("online") or {}).get("state")))


# ---- vision vs vehicle --------------------------------------------------------------------------------
def xcorr_lag(f_ref, f_test, t0, t1, lags, hz=20.0, hp_s=2.0, mask=None):
    """Lag L (s) maximising the correlation of high-passed f_test(t) with f_ref(t - L): L > 0 = the test signal
    lags the reference."""
    grid = [t0 + i / hz for i in range(int((t1 - t0) * hz))]
    best = None
    out = []
    for L in lags:
        xs, ys = [], []
        for t in grid:
            if mask is not None and not mask(t):
                continue
            a, b = f_ref(t - L), f_test(t)
            if a is None or b is None:
                continue
            xs.append(a)
            ys.append(b)
        if len(xs) < 200:
            continue
        # high-pass: subtract a centred moving mean (hp_s window) of each series
        k = max(1, int(hp_s * hz / 2))
        def hp(v):
            c = [0.0]
            for x in v:
                c.append(c[-1] + x)
            o = []
            for i in range(len(v)):
                lo, hi = max(0, i - k), min(len(v), i + k + 1)
                o.append(v[i] - (c[hi] - c[lo]) / (hi - lo))
            return o
        hx, hy = hp(xs), hp(ys)
        sx = math.sqrt(sum(x * x for x in hx))
        sy = math.sqrt(sum(y * y for y in hy))
        if sx == 0 or sy == 0:
            continue
        r = sum(x * y for x, y in zip(hx, hy)) / (sx * sy)
        out.append((L, r))
        if best is None or r > best[1]:
            best = (L, r)
    return best, out


def cmd_vision(a):
    lags = [i * 0.025 for i in range(-12, 25)]
    for path in a.sessions:
        s = load(path)
        name = os.path.basename(os.path.normpath(path))
        if not s["ego"]:
            print("==", name, ": no vehicle `ego` messages, nothing to compare")
            continue
        V = Interp(s["ego"], 1)
        R = Interp(s["ego"], 2)
        rows = s["road"]
        print("==", name, "road frames", len(rows))
        tau = a.lag
        bins = {}
        for r in rows:
            v = V(r["t"] - tau)
            if v is None or r["vx"] is None:
                continue
            b = 0 if abs(v) < 0.05 else min(int(v // 3) + 1, 6)
            bins.setdefault(b, []).append((r["vx"] - v, r["vx"], v, r["vxs"]))
        print("  vision speed - vehicle speed (vehicle at frame time - %.2f s), m/s:" % tau)
        print("    %-12s %6s %8s %8s %8s  %s  %s" % ("vehicle v", "n", "bias", "sigma", "ratio50", "diff q01/05/50/95/99", "model vx_std p50/p95"))
        for b in sorted(bins):
            d = bins[b]
            m, sd = robust([x[0] for x in d])
            rat = st.median([x[1] / x[2] for x in d]) if b > 0 else None
            lab = "standstill" if b == 0 else "%d-%d" % ((b - 1) * 3, b * 3) if b < 6 else ">=15"
            print("    %-12s %6d %8.3f %8.3f %8s  %s  %s" % (lab, len(d), m or 0, sd or 0,
                                                       "-" if rat is None else "%.3f" % rat, q([x[0] for x in d]),
                                                       q([x[3] for x in d if x[3] is not None], (0.5, 0.95))))
        yd = []
        turns = []
        for r in rows:
            rr, v = R(r["t"] - tau), V(r["t"] - tau)
            if rr is None or v is None or r["r"] is None or v < 3:
                continue
            yd.append(r["r"] - rr)
            if abs(rr) > 0.08:
                turns.append((r["r"], rr))
        m, sd = robust(yd)
        if yd:
            print("  vision yaw rate - vehicle yaw rate (v >= 3 m/s), rad/s: n %d bias %.4f sigma %.4f q %s" % (
                len(yd), m, sd, q(yd, nd=4)))
        if turns:
            g = sum(x * y for x, y in turns) / sum(y * y for _, y in turns)
            print("  in turns (|vehicle yaw| > 0.08 rad/s): n %d gain vision/vehicle %.3f" % (len(turns), g))
        t0, t1 = rows[0]["t"], rows[-1]["t"]
        vis_v = Interp([(r["t"], r["vx"]) for r in rows], 1, 0.5)
        vis_r = Interp([(r["t"], r["r"]) for r in rows], 1, 0.5)
        moving = lambda t: (V(t) or 0) > 2.0
        bv, _ = xcorr_lag(V, vis_v, t0, t1, lags, mask=moving)
        br, _ = xcorr_lag(R, vis_r, t0, t1, lags, mask=moving)
        print("  lag by cross-correlation (high-passed 2 s, moving): speed %s, yaw rate %s  (s, correlation; > 0 = vision behind the vehicle)" % (
            None if bv is None else (round(bv[0], 3), round(bv[1], 3)),
            None if br is None else (round(br[0], 3), round(br[1], 3))))
        # lag by the smallest robust spread of the residual (frames with the vehicle moving > 2 m/s)
        for lab, F, k in (("speed", V, "vx"), ("yaw rate", R, "r")):
            best = None
            for L in lags:
                res = []
                for r in rows:
                    x, v = F(r["t"] - L), V(r["t"] - L)
                    if x is None or v is None or v < 2.0 or r[k] is None:
                        continue
                    res.append(r[k] - x)
                if len(res) < 200:
                    continue
                m, sd = robust(res)
                if best is None or sd < best[2]:
                    best = (L, m, sd)
            if best:
                print("  lag by the smallest robust residual spread, %s: %.3f s (bias %.4f, sigma %.4f)" % (lab, *best))


# ---- GNSS ---------------------------------------------------------------------------------------------
def load_pos(path):
    """Feeder gnss messages of the trip-log extract, each with `_wall` (s): the trip file's start (its name,
    local time Europe/Warsaw) plus the line's `ts` (ms since the trip start). The phone's CLOCK_MONOTONIC restarts
    with every boot, so a message is matched to a session by its wall time, not by `ts_mono_ns` alone."""
    import json
    import re
    import time as _time
    os.environ.setdefault("TZ", "Europe/Warsaw")
    _time.tzset()
    out = []
    with open(path) as f:
        for ln in f:
            i = ln.find("{")
            if i < 0:
                continue
            m = re.search(r"trip_(\d{8})_(\d{6})", ln[:i])
            try:
                rec = json.loads(ln[i:])
            except ValueError:
                continue
            d = rec.get("d") or {}
            if d.get("type") != "gnss" or not isinstance(d.get("ts_mono_ns"), int) or not m:
                continue
            start = _time.mktime(_time.strptime(m.group(1) + m.group(2), "%Y%m%d%H%M%S"))
            d["_wall"] = start + (rec.get("ts") or 0) / 1000.0
            out.append(d)
    out.sort(key=lambda d: d["ts_mono_ns"])
    return out


def angdiff(a, b):
    return (a - b + 180.0) % 360.0 - 180.0


def cmd_gnss(a):
    pos = load_pos(a.pos)
    for path in a.sessions:
        s = load(path)
        name = os.path.basename(os.path.normpath(path))
        rows = s["road"]
        if not rows:
            continue
        t0, t1 = rows[0]["t"], rows[-1]["t"]
        raw = [m for m in read_lines(path) if m.get("type") == "road" and isinstance(m.get("ts_real_ns"), int)][-200:]   # after any clock step
        off = st.median([(m["ts_real_ns"] - m["ts_mono_ns"]) / NS for m in raw])      # wall - mono of this boot
        g = [d for d in pos if t0 - 5 <= d["ts_mono_ns"] / NS <= t1 + 5
             and abs(d["_wall"] - (d["ts_mono_ns"] / NS + off)) < 5.0]
        if g:
            ow = [d["_wall"] - (d["ts_mono_ns"] / NS + off) for d in g]
            print("  trip-log wall time minus the message's own time: %s s (log written that much after the sample)" % q(ow, (0.05, 0.5, 0.95)))
        print("==", name, "feeder gnss messages in the session:", len(g))
        if not g:
            continue
        ts = [d["ts_mono_ns"] / NS for d in g]
        d, ng, mx = gaps(ts, 1.5)
        fixes = [d_.get("fix") for d_ in g]
        print("  rate %.2f/s, dt p50 %.2f max %.1f s, gaps > 1.5 s: %d; fix %s; hacc_m %s; sats %s" % (
            (len(g) - 1) / (ts[-1] - ts[0]), st.median(d), mx, ng, {k: fixes.count(k) for k in set(fixes)},
            q([x.get("hacc_m") for x in g if x.get("hacc_m") is not None], (0.05, 0.5, 0.95)),
            q([x.get("sats") for x in g if x.get("sats") is not None], (0.05, 0.5, 0.95), 0)))
        good = [x for x in g if x.get("fix") in ("2d", "3d", "dgps", "rtk") and x.get("speed_mps") is not None]
        rep = lambda k: sum(1 for x, y in zip(g, g[1:]) if k(x) == k(y)) / max(1, len(g) - 1)
        print("  repeated from the previous message: speed %.2f, course %.2f, position %.2f" % (
            rep(lambda x: x.get("speed_mps")), rep(lambda x: x.get("course_deg")), rep(lambda x: (x.get("lat"), x.get("lon")))))
        if s["ego"]:
            V = Interp(s["ego"], 1)
            best = None
            for L in [i * 0.1 for i in range(-20, 81)]:
                res = [x["speed_mps"] - V(x["ts_mono_ns"] / NS - L) for x in good
                       if V(x["ts_mono_ns"] / NS - L) is not None and V(x["ts_mono_ns"] / NS - L) > 2]
                if len(res) < 30:
                    continue
                rms = math.sqrt(sum(r * r for r in res) / len(res))
                if best is None or rms < best[1]:
                    best = (L, rms, len(res))
            if best:
                L = best[0]
                res, rat = [], []
                for x in good:
                    v = V(x["ts_mono_ns"] / NS - L)
                    if v is not None and v > 2:
                        res.append(x["speed_mps"] - v)
                        rat.append(x["speed_mps"] / v)
                m, sd = robust(res)
                print("  GNSS speed - vehicle speed (v > 2): best lag %.1f s (GNSS behind the vehicle), rms %.3f m/s, n %d; "
                      "at that lag bias %.3f sigma %.3f ratio50 %.4f q %s" % (L, best[1], best[2], m, sd, st.median(rat), q(res)))
                res0 = [x["speed_mps"] - V(x["ts_mono_ns"] / NS) for x in good
                        if V(x["ts_mono_ns"] / NS) is not None and V(x["ts_mono_ns"] / NS) > 2]
                m0, sd0 = robust(res0)
                print("  the same without a lag: bias %.3f sigma %.3f" % (m0, sd0))
            # speed from the displacement between consecutive fixes (independent of the receiver's speed field)
            def _dist(p, r):
                la, lb = math.radians(p["lat"]), math.radians(r["lat"])
                return 6371000.0 * math.hypot(math.radians(r["lon"] - p["lon"]) * math.cos((la + lb) / 2), lb - la)
            bd = None
            for L in [i * 0.2 for i in range(-10, 41)]:
                res = []
                for x, y in zip(good, good[1:]):
                    dt = (y["ts_mono_ns"] - x["ts_mono_ns"]) / NS
                    v = V((x["ts_mono_ns"] + y["ts_mono_ns"]) / 2 / NS - L)
                    if 0.5 < dt < 1.6 and v is not None and v > 2:
                        res.append(_dist(x, y) / dt - v)
                if len(res) > 30:
                    m, sd = robust(res)
                    if bd is None or sd < bd[2]:
                        bd = (L, m, sd, len(res))
            if bd:
                print("  displacement speed - vehicle speed: lag %.1f s (smallest robust spread), bias %.2f sigma %.2f n %d" % bd)
            still = [x["speed_mps"] for x in good if V(x["ts_mono_ns"] / NS) == 0.0]
            if still:
                print("  GNSS speed while the vehicle stands: n %d q %s" % (len(still), q(still)))
            # course rate against the vehicle yaw rate (course differences of consecutive fixes, v > 5)
            R = Interp(s["ego"], 2)
            cr = []
            for x, y in zip(good, good[1:]):
                dt = (y["ts_mono_ns"] - x["ts_mono_ns"]) / NS
                if not (0.5 < dt < 1.6) or x.get("course_deg") is None or y.get("course_deg") is None:
                    continue
                vm = V((x["ts_mono_ns"] + y["ts_mono_ns"]) / 2 / NS)
                rm = R((x["ts_mono_ns"] + y["ts_mono_ns"]) / 2 / NS)
                if vm is None or rm is None or vm < 5:
                    continue
                # course clockwise from north; yaw rate positive left
                cr.append((-math.radians(angdiff(y["course_deg"], x["course_deg"])) / dt, rm))
            if cr:
                m, sd = robust([c - r for c, r in cr])
                turns = [(c, r) for c, r in cr if abs(r) > 0.08]
                gain = sum(c * r for c, r in turns) / sum(r * r for _, r in turns) if turns else None
                print("  GNSS course rate - vehicle yaw rate (v > 5): n %d bias %.4f sigma %.4f rad/s; gain in turns %s (n %d)" % (
                    len(cr), m, sd, None if gain is None else round(gain, 3), len(turns)))
        vis = Interp([(r["t"], r["vx"]) for r in rows], 1, 0.5)
        res = []
        for x in good:
            v = vis(x["ts_mono_ns"] / NS)
            if v is not None and x["speed_mps"] > 2:
                res.append((x["speed_mps"], v))
        if res:
            m, sd = robust([v - gs for gs, v in res])
            print("  vision speed - GNSS speed (GNSS > 2, no lag): n %d bias %.3f sigma %.3f ratio50 %.3f" % (
                len(res), m, sd, st.median([v / gs for gs, v in res])))


# ---- incident -----------------------------------------------------------------------------------------
def cmd_incident(a):
    """Per session: the 5 s median of vision speed / vehicle speed over frames with the vehicle at >= 3 m/s, the
    share of frames whose difference exceeds 3 sigma of the healthy noise model (0.25 + 0.035 v), and when simple
    rules would have fired."""
    W = a.window
    for path in a.sessions:
        s = load(path)
        name = os.path.basename(os.path.normpath(path))
        if not s["ego"]:
            print("==", name, ": no vehicle `ego` messages: no reference, a check could not run")
            continue
        V = Interp(s["ego"], 1)
        rows = s["road"]
        t0 = rows[0]["t"]
        ev = []                    # (t, ratio, bad, vxs)
        for r in rows:
            v = V(r["t"] - 0.1)
            if v is None or r["vx"] is None or v < 3.0:
                continue
            sig = 0.25 + 0.035 * v
            ev.append((r["t"], r["vx"] / v, abs(r["vx"] - v) > 3 * sig, r["vxs"]))
        print("== %s: evidence frames (vehicle >= 3 m/s) %d, first at %s s" % (
            name, len(ev), None if not ev else round(ev[0][0] - t0, 1)))
        if not ev:
            continue
        # sliding windows of W s of evidence time
        med, frac, firsts = [], [], {}
        rules = {"ratio5s_out_[0.7,1.4]": lambda m_, f_, n_: n_ >= 15 and not (0.7 <= m_ <= 1.4),
                 "bad3sigma_share>=0.6": lambda m_, f_, n_: n_ >= 15 and f_ >= 0.6,
                 "ratio5s<0.5": lambda m_, f_, n_: n_ >= 15 and m_ < 0.5}
        fired_time = {k: 0.0 for k in rules}
        j = 0
        for i, (t, rt, bad, _) in enumerate(ev):
            while ev[j][0] < t - W:
                j += 1
            win = ev[j:i + 1]
            n = len(win)
            m_ = st.median([x[1] for x in win])
            f_ = sum(x[2] for x in win) / n
            if n >= 15:
                med.append(m_)
                frac.append(f_)
            dt = min(t - ev[i - 1][0], 0.5) if i else 0.0
            for k, rule in rules.items():
                if rule(m_, f_, n):
                    firsts.setdefault(k, t)
                    fired_time[k] += dt
        tot = sum(min(b[0] - a_[0], 0.5) for a_, b in zip(ev, ev[1:]))
        print("  %.0f s of evidence; %g s window median ratio q01/05/50/95/99 %s; bad share %s" % (
            tot, W, q(med), q(frac)))
        print("  per-frame ratio q %s; model vx_std q %s" % (q([x[1] for x in ev]), q([x[3] for x in ev if x[3] is not None])))
        for k in rules:
            f = firsts.get(k)
            print("  rule %-24s first fires %s s after the first evidence frame (session time %s s); true for %.0f s = %.0f %% of the evidence time" % (
                k, None if f is None else round(f - ev[0][0], 1), None if f is None else round(f - t0, 1),
                fired_time[k], 100 * fired_time[k] / tot if tot else 0))
        # lead outputs at the time
        ld = [r for r in rows if r["lp"] and r["lp"] >= 0.5 and r["lx"]]
        print("  raw leads[0] with prob >= 0.5: %d of %d frames, distance q %s" % (len(ld), len(rows), q([r["lx"] for r in ld], (0.05, 0.5, 0.95), 1)))


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd", required=True)
    for n in ("rates", "vision", "gnss", "incident"):
        p = sub.add_parser(n)
        p.add_argument("sessions", nargs="+")
        if n == "vision":
            p.add_argument("--lag", type=float, default=0.1, help="vehicle sample taken this much before the frame")
        if n == "gnss":
            p.add_argument("--pos", required=True)
        if n == "incident":
            p.add_argument("--window", type=float, default=5.0)
    a = ap.parse_args()
    {"rates": cmd_rates, "vision": cmd_vision, "gnss": cmd_gnss, "incident": cmd_incident}[a.cmd](a)


if __name__ == "__main__":
    main()
