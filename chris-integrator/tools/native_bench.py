#!/usr/bin/env python3
"""What each function of the native core costs next to its pure-Python reference, on this machine.

    tools/native_bench.py [--capture FILE.cap.gz | SESSION_DIR] [--seconds 0.4] [--json OUT.json]

Per function: microseconds per call, reference and native, and the ratio. The inputs are real messages when a
capture (tools/cost_capture.py) or a recorder session is given (the first `road` line, the stack's own
`lanes` / `view` / `lead` / `ego_state` for it), otherwise a synthetic frame of the same shape and sizes.
Run it pinned to the core in question:  taskset -c 1 python3 tools/native_bench.py
Both implementations are called directly here, whatever `CHRIS_NATIVE` says; without the compiled module the
reference column alone is printed."""
import argparse
import gzip
import json
import math
import os
import random
import sys
import time

HERE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, HERE)                    # chris-mapper, where a part needs it: the installed one

import numpy as np  # noqa: E402

from chris_integrator import native  # noqa: E402
from chris_integrator.core import ego as E  # noqa: E402
from chris_integrator.core import ekf as K  # noqa: E402
from chris_integrator.trackers import lanes as L  # noqa: E402
from chris_integrator.trackers import view as V  # noqa: E402

try:
    from chris_integrator.native import _chris_native as C
except ImportError:
    C = None


def per_call(fn, seconds):
    """Microseconds per call: the best of several timed batches (the quietest the core was)."""
    n = 1
    while True:
        t0 = time.perf_counter()
        for _ in range(n):
            fn()
        dt = time.perf_counter() - t0
        if dt > seconds / 8 or n > 1 << 22:
            break
        n *= 2
    best = dt / n
    t_end = time.perf_counter() + seconds
    while time.perf_counter() < t_end:
        t0 = time.perf_counter()
        for _ in range(n):
            fn()
        best = min(best, (time.perf_counter() - t0) / n)
    return best * 1e6


def synth_road(rng):
    xs = [192.0 * (i / 32.0) ** 2 for i in range(33)]

    def arr(lo, hi, nd=5):
        return [float("%.*g" % (nd, rng.uniform(lo, hi))) for _ in range(33)]

    def line():
        return {"x": [float("%.5g" % v) for v in xs], "y": arr(-6, 6), "z": arr(-1, 1), "y_std": arr(0.05, 2.5), "prob": round(rng.random(), 4)}
    plan = {k: arr(-5, 150) for k in ("x", "y", "z", "vx", "vy", "vz", "ax", "ay", "az", "roll", "pitch", "yaw",
                                      "roll_rate", "pitch_rate", "yaw_rate", "x_std", "y_std", "z_std")}
    return {"type": "road", "seq": 1234, "ts_mono_ns": 3226220852221, "ts_real_ns": 1795084326395365155,
            "model_ms": 64.061, "v_ego": 13.53368, "valid": True, "frames_skipped": 0, "pair_dt_ms": 66.19,
            "pose": {"vx": 13.53, "vy": 0.0021, "vz": 0.01, "wx": 0.0, "wy": 0.001, "wz": -0.0123},
            "lane_lines": [line() for _ in range(4)], "road_edges": [line() for _ in range(2)],
            "leads": [{"x": arr(5, 80)[:6], "y": arr(-1, 1)[:6], "v": arr(0, 30)[:6], "a": arr(-2, 2)[:6], "prob": 0.91}],
            "plan": plan, "meta": {"engaged": 0.01, "brake": [0.01] * 5}}


def real_road(path):
    """The first raw `road` line of a capture or a session, parsed."""
    f = os.path.join(path, "messages.jsonl.gz") if os.path.isdir(path) else path
    try:
        with gzip.open(f, "rb") as fh:
            for ln in fh:
                p = ln.rstrip(b"\n").split(b"\t", 2)
                raw = p[2] if len(p) == 3 else ln.strip()
                if raw.startswith(b'{"type":"road"') and len(raw) > 3000:
                    return json.loads(raw)
    except (EOFError, OSError):
        pass
    return None


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("input", nargs="?")
    ap.add_argument("--capture")
    ap.add_argument("--seconds", type=float, default=0.4)
    ap.add_argument("--json")
    a = ap.parse_args()
    rng = random.Random(1)
    g = np.random.default_rng(1)
    src = a.capture or a.input
    road = real_road(src) if src else None
    what = "a recorded road line" if road is not None else "a synthetic frame"
    if road is None:
        road = synth_road(rng)
    road_line = json.dumps(road, separators=(",", ":"))
    road_raw = road_line.encode()

    # the stack's own messages for this frame: the lane tracker's and the display box's outputs
    es = {"speed": {"value": 13.5, "valid": True, "sources": ["vehicle.speed"]}, "yaw_rate": {"value": 0.01, "valid": True},
          "sources": {"vision": {"state": "ok"}}}
    tr = L.RoadGeometryTracker()
    lanes = None
    for i in range(6):
        lanes = tr.on_road(dict(road, ts_mono_ns=road["ts_mono_ns"] + i * 66000000, seq=i), es) or lanes
    ego_state = {"type": "ego_state", "api": 3, "seq": 1, "ts_mono_ns": 1, "speed": {"value": 8.602, "std": 0.028, "valid": True, "sources": ["vehicle.speed"]},
                 "sources": {n: {"state": "ok", "reason": None, "usable": True, "age_s": 0.031, "rate_hz": 20.0, "nis": {"speed": 0.84, "yaw_rate": 1.02}}
                             for n in ("vehicle", "wheels", "vision", "gnss", "map")},
                 "checks": [{"kind": "nis", "a": "vehicle", "quantity": "q%d" % i, "verdict": "ok", "n": 66, "ratio": 0.05 * i, "peak": 0.08} for i in range(40)],
                 "calibration": {"s%d" % i: {"a": {"value": 0.00123 * i, "std": 0.0021, "unit": "1", "known": True}} for i in range(17)}}
    rows = []

    def add(name, ref, nat, note=""):
        r = per_call(ref, a.seconds)
        n = per_call(nat, a.seconds) if (nat is not None and C is not None) else None
        rows.append((name, r, n, note))

    # ---- text ----
    for nm, msg in (("road (whole line)", road), ("lanes", lanes), ("ego_state (tables)", ego_state)):
        if msg is None:
            continue
        text = json.dumps(msg, separators=(",", ":"))
        add("write %s, %d B" % (nm, len(text)), lambda m=msg: json.dumps(m, separators=(",", ":")), (lambda m=msg: C.dumps(m)) if C else None)
        add("read  %s, %d B" % (nm, len(text)), lambda t=text: json.loads(t), (lambda t=text: C.loads(t)) if C else None)
    from chris_integrator.fastjson import trim_road
    trimmed = trim_road(road_raw)
    add("read  road as the default stack does (trimmed, %d B)" % len(trimmed), lambda: json.loads(trimmed), (lambda: C.loads(trimmed)) if C else None)
    from chris_integrator.io import recorder as R
    from chris_integrator.fastround import warm
    warm((2, 3))

    def rec_ref():
        out, parts, digits, slots = R._slim(road)
        pieces = json.dumps(out, separators=(",", ":")).split(R._MARK_TEXT)
        texts = R._LAYOUT.text_parts(parts, digits)
        res = [pieces[0]]
        for t, p in zip(texts, pieces[1:]):
            res.append(t)
            res.append(p)
        return "".join(res)

    def rec_nat():
        return C.dumps(R._slim(road, native.Rounded)[0])
    if C is not None:
        C.set_rounded_type(native.Rounded)
        assert rec_ref() == rec_nat()
    add("the recorder's reduced road line (~700 numbers rounded and written)", rec_ref, rec_nat if C else None, "reference = L46's table method")
    vals = [rng.uniform(-200.0, 200.0) for _ in range(200)]
    add("round(x, 2), 200 values", lambda: [round(v, 2) for v in vals], (lambda: [C.round(v, 2) for v in vals]) if C else None)
    add("round of a list of 200", lambda: [round(v, 2) for v in vals], (lambda: C.round_list(vals, 2)) if C else None)
    # ---- the display box's interpolation ----
    gx, gy, qs = lanes["x"], (lanes["lines"][1]["y"] or lanes["x"]), V.DEFAULT_CFG["xs"]
    add("interpolate one line at the view's 14 distances", lambda: V._interp_all_py(gx, gy, qs), (lambda: C.interp_all(gx, gy, qs)) if C else None)
    # ---- the lane tracker's arrays ----
    c = dict(L.DEFAULT_CFG)
    srcs = list(road["lane_lines"][:4]) + list(road["road_edges"][:2])
    x = np.asarray(road["lane_lines"][0]["x"], float)
    n = len(x)
    L_NL = L._NL

    def meas_ref():
        L._NL = None
        try:
            return tr._measure(road)
        finally:
            L._NL = L_NL

    def meas_nat():
        y, s, prob, ok = np.empty((6, n)), np.empty((6, n)), np.empty(6), np.empty(6, bool)
        C.lanes_measure(srcs, x, c["edge_std_x"][0], c["edge_std_x"][1], c["edge_std_bad"], c["edge_std_good"], y, s, prob, ok)
        return x, y, s, prob, ok
    add("lanes: the frame's measurement arrays (12 lists of %d)" % n, meas_ref, meas_nat if C and len(srcs) == 6 else None)
    _, zy, zs, prob, ok = meas_ref()
    y, p = zy + 0.05 * g.standard_normal((6, n)), np.abs(g.standard_normal((6, n))) * 0.1 + 0.01
    add("lanes: noise, gate, innovation, fit of 6 lines", lambda: L._fit_py(c, x, zs, zy, prob, y, p), (lambda: L._fit_native(c, x, zs, zy, prob, y, p)) if C else None)
    r, gate, nu = L._fit_py(c, x, zs, zy, prob, y, p)[:3]
    y1, p1 = y.copy(), p.copy()
    add("lanes: update of 4 lines", lambda: L._update_py(y1, p1, r, nu, [0, 1, 2, 3]), (lambda: C.lanes_apply(y1, p1, r, nu, [0, 1, 2, 3])) if C else None)
    add("lanes: offsets and deviations rounded to lists", lambda: (L._round2_py(y), L._round2_py(p, True)), (lambda: (C.round_lists(y, 2, False), C.round_lists(p, 2, True))) if C else None)
    # ---- the estimator ----
    m = E.EgoModel()
    for i in range(17):
        m.add_aux("s%d" % i, E.AuxState("a", 0.0, 0.1, psd=1e-6) if i % 3 else E.AuxState("a", 0.0, 0.1, tau_s=40.0))
    ns = m.n
    aa = g.standard_normal((ns, ns))
    P = aa @ aa.T * 0.01 + np.eye(ns)
    xs_ = g.standard_normal(ns)
    xs_[E.V], xs_[E.SV] = 12.0, 0.01
    par = m._native_par()

    def pred_nat():
        x1, P1, Fc, rho = np.empty(ns), np.empty((ns, ns)), np.empty((10, 10)), np.empty(ns)
        C.ekf_predict(xs_, P, 0.05, par, rho, x1, P1, Fc)
        return x1, P1, Fc, rho
    add("filter: one prediction (%d states)" % ns, lambda: m._predict_py(xs_, P, 0.05), pred_nat if C else None)
    idx, coef = np.array([E.V], dtype=np.intp), np.array([1.0])
    buf = np.empty(ns)

    def row_ref():
        ph, s0 = K._project_py(P, idx, coef)
        xx = xs_.copy()
        return K._sym_py(K._apply_py(xx, P, ph, s0 + 0.1, 0.3, None, False), False)

    def row_nat():
        s0 = C.ekf_project(P, idx, coef, buf)
        xx, PP = xs_.copy(), P.copy()
        C.ekf_update(xx, PP, buf, s0 + 0.1, 0.3, None)
        C.ekf_sym(PP)
        return PP
    add("filter: one scalar row (project, update, symmetrise)", row_ref, row_nat if C else None)
    xp, Pp, F = m._predict_py(xs_, P, 0.05)
    xsm, Psm = xp + 0.01, Pp * 0.9

    def rts_ref():
        Cm = np.linalg.solve(Pp, m.apply_F(F, P)).T
        return xs_ + Cm @ (xsm - xp), P + Cm @ (Psm - Pp) @ Cm.T

    def rts_nat():
        xo, Po = np.empty(ns), np.empty((ns, ns))
        C.ekf_rts(Pp, F[0], F[1], P, xs_, xp, xsm, Psm, (), xo, Po)
        return xo, Po
    add("filter: one smoother step", rts_ref, rts_nat if C else None)

    print("inputs: %s; %s; python %s, numpy %s; native module: %s" % (
        what, os.uname().machine, sys.version.split()[0], np.__version__, "version %s" % C.VERSION if C else "not built"))
    print("%-72s %12s %12s %8s" % ("", "reference us", "native us", "ratio"))
    for name, r, n, note in rows:
        print("%-72s %12.1f %12s %8s  %s" % (name, r, "-" if n is None else "%.1f" % n, "-" if n is None else "%.1fx" % (r / n), note))
    if a.json:
        with open(a.json, "w") as f:
            json.dump({"machine": os.uname().machine, "python": sys.version.split()[0], "numpy": np.__version__, "input": what,
                       "rows": [{"what": nm, "reference_us": round(r, 2), "native_us": None if n is None else round(n, 2)} for nm, r, n, _ in rows]}, f, indent=1)


if __name__ == "__main__":
    main()
