"""`python -m chris_integrator.native`: which implementation is in use, and a self-test of the native core
against the pure-Python reference on this machine (a few seconds; exit status 0 = in use and equal,
1 = built but wrong: do not use it, 3 = not in use). The install script runs this before it keeps a binary."""
import json
import math
import random
import struct
import sys

import numpy as np

from . import PARTS, _n, status


def selftest(n=20000, seed=1):
    """Problems found (an empty list: the native functions gave the reference's results)."""
    if _n is None:
        return ["the native module is not loaded"]
    bad = []
    rng = random.Random(seed)

    def rnd_float():
        k = rng.randrange(4)
        if k == 0:
            v = struct.unpack("<d", struct.pack("<Q", rng.getrandbits(64)))[0]
            return v if math.isfinite(v) else 0.5
        if k == 1:
            return round(rng.uniform(-300.0, 300.0), rng.randrange(0, 6))
        if k == 2:
            return (rng.randrange(-10 ** 6, 10 ** 6) + 0.5) / 10.0 ** rng.randrange(0, 5)
        return rng.uniform(-1e3, 1e3)

    for _ in range(n):
        v = rnd_float()
        if _n.dumps(v) != repr(v):
            bad.append("dumps(%r) = %s" % (v, _n.dumps(v)))
        d = rng.randrange(0, 6)
        if repr(_n.round(v, d)) != repr(round(v, d)):
            bad.append("round(%r, %d) = %r" % (v, d, _n.round(v, d)))
        t = repr(v)
        if repr(_n.loads(t)) != repr(json.loads(t)):
            bad.append("loads(%s) = %r" % (t, _n.loads(t)))
        if len(bad) > 5:
            return bad
    msg = {"type": "x", "seq": 7, "ts_real_ns": 1795084321890056390, "a": [rnd_float() for _ in range(50)],
           "s": "a\"b\\c\né\U0001F600", "n": None, "t": True, "d": {"k": [1, 2.5, {"z": -0.0}]}}
    text = json.dumps(msg, separators=(",", ":"))
    if _n.dumps(msg) != text:
        bad.append("dumps of a message differs from json.dumps")
    if json.dumps(_n.loads(text), separators=(",", ":")) != text or json.dumps(_n.loads(text.encode()), separators=(",", ":")) != text:
        bad.append("loads of a message differs from json.loads")
    # the estimator's steps against numpy
    g = np.random.default_rng(seed)
    nst = 27
    a = g.standard_normal((nst, nst))
    P = a @ a.T + np.eye(nst)
    idx, coef = np.array([4], dtype=np.intp), np.array([1.0])
    ph = np.empty(nst)
    s0 = _n.ekf_project(P, idx, coef, ph)
    if not np.array_equal(ph, P[:, idx] @ coef) or s0 != float(coef @ ph[idx]):
        bad.append("ekf_project differs from numpy")
    x = g.standard_normal(nst)
    x1, P1 = x.copy(), P.copy()
    _n.ekf_update(x1, P1, ph, s0 + 0.1, 0.3, None)
    k = ph / (s0 + 0.1)
    if not np.array_equal(x1, x + k * 0.3) or not np.array_equal(P1, P - k[:, None] * ph[None, :]):
        bad.append("ekf_update differs from numpy")
    from ..core import ego as E
    m = E.EgoModel()
    for i in range(nst - E.NCORE):
        m.add_aux("s%d" % i, E.AuxState("a", 0.0, 0.1, psd=1e-6) if i % 2 else E.AuxState("a", 0.0, 0.1, tau_s=40.0))
    x[E.V], x[E.SV] = 12.0, 0.01
    xr, Pr, _ = m._predict_py(x, P, 0.05)
    xo, Po, Fc, rho = np.empty(nst), np.empty((nst, nst)), np.empty((E.NCORE, E.NCORE)), np.empty(nst)
    _n.ekf_predict(x, P, 0.05, m._native_par(), rho, xo, Po, Fc)
    if not np.array_equal(xo, xr) or float(np.max(np.abs(Po - Pr))) > 1e-9 * float(np.max(np.abs(Pr))):
        bad.append("ekf_predict differs from the reference")
    # the display box's table walk and the lane tracker's array steps against their Python / numpy forms
    from ..trackers import lanes as L
    from ..trackers import view as V
    for _ in range(300):
        xs = sorted(rng.uniform(0.0, 100.0) for _ in range(rng.randrange(2, 12)))
        ys = [None if rng.random() < 0.05 else rng.uniform(-5.0, 5.0) for _ in xs]
        qs = tuple(sorted(rng.uniform(-5.0, 110.0) for _ in range(14)))
        if _n.interp_all(xs, ys, qs) != V._interp_all_py(xs, ys, qs):
            bad.append("interp_all differs from the Python walk")
            break
    c = dict(L.DEFAULT_CFG)
    n = 33
    xl = np.array([192.0 * (i / 32.0) ** 2 for i in range(n)])
    for _ in range(50):
        zy, zs = g.standard_normal((6, n)), np.abs(g.standard_normal((6, n))) + 0.01
        prob, y, p = g.random(6), g.standard_normal((6, n)), np.abs(g.standard_normal((6, n))) + 0.01
        ref = L._fit_py(c, xl, zs, zy, prob, y, p)
        got = L._fit_native(c, xl, zs, zy, prob, y, p)
        if not all(np.array_equal(u, v) for u, v in zip(ref, got)):
            bad.append("lanes_fit differs from numpy")
            break
        if _n.round_lists(y, 2, False) != np.round(y, 2).tolist() or _n.round_lists(p, 2, True) != np.round(np.sqrt(p), 2).tolist():
            bad.append("round_lists differs from numpy")
            break
        vals = [float(v) for v in y[0]] + [None, 3]
        if _n.round_list(vals, 2) != [v if v is None else round(v, 2) for v in vals]:
            bad.append("round_list differs from round")
            break
    return bad


def main():
    st = status()
    print("chris-integrator native core: %s" % json.dumps(st))
    if _n is None:
        return 3
    if len(st["parts"]) < len(PARTS):
        print("self-test not run: only some parts are switched on (CHRIS_NATIVE=%s)" % ",".join(st["parts"]))
        return 0 if st["active"] else 3
    try:
        bad = selftest()
    except Exception as e:                 # a compiled module that raises where the reference does not
        bad = ["%s: %s" % (type(e).__name__, e)]
    if bad:
        print("SELF-TEST FAILED: the native core does not give the reference's results on this machine:")
        for b in bad:
            print("  " + b)
        return 1
    print("self-test ok: %s equal to the pure-Python reference" % ", ".join(PARTS))
    return 0 if st["active"] else 3


if __name__ == "__main__":
    sys.exit(main())
