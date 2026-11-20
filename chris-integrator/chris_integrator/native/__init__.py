"""The native core: which implementation of the hot path's arithmetic and text work is in use.

chris-integrator is pure Python + numpy and stays so: every function below has a pure-Python implementation,
which is the reference. Where the compiled module `_chris_native` (one C file, `src/chris_native.c`; build:
docs/NATIVE.md) is present and loads, the same functions run native: an accelerated implementation with the
same contract, selected here at import. Nothing else in the package knows which one it got.

    dumps(obj)            json.dumps(obj, separators=(",", ":"))            identical text
    loads(raw)            json.loads(raw)                                   identical objects
    round(x, n)           builtins.round                                    identical results
    interp_all            trackers.view's table walk                        identical results
    Rounded((list, n))    in `dumps`: the list written with every number rounded to n digits, as
                          json.dumps([round(v, n) for v in list]) writes it  (native only: `ROUNDED`)
    lanes                 the lane tracker's array work (trackers/lanes.py), or None: the same bits as the
                          numpy expressions it stands for
    ekf                   the estimator's linear algebra (core/ekf.py, core/ego.py), or None.
                          Elementwise steps give numpy's bits; sums of products are taken in index order where
                          numpy hands them to BLAS (whose order is its own): results agree to rounding
                          (docs/NATIVE.md, "Equivalence").

A native function that is not certain to give the reference's result for an input (a type it does not know, a
number outside its exact path) says so (`Fallback`) and the reference runs for that call.

`CHRIS_NATIVE` in the environment: unset or `1` = every part that is built; `0` = pure Python; a list
(`json,round,interp,lanes,ekf`) = those parts only (for comparisons). `status()` is what `/v2/health` shows.
"""
import builtins
import json
import os

PARTS = ("json", "round", "interp", "ekf", "lanes")
VERSION = 2                    # of the compiled module's functions: a module of another version is not used
_dumps_py, _loads_py = json.dumps, json.loads
_SEP = (",", ":")

_n = None
_reason = None
_want = os.environ.get("CHRIS_NATIVE", "1").strip().lower()
if _want in ("0", "off", "no", "false", "none", ""):
    _reason = "switched off (CHRIS_NATIVE=%s)" % (_want or "''")
    _want = ()
else:
    _want = PARTS if _want in ("1", "on", "yes", "true", "all") else tuple(p for p in _want.split(",") if p in PARTS)
    try:
        from . import _chris_native as _n
    except ImportError as e:               # not built for this machine: the package is complete without it
        _n = None
        if any(f.startswith("_chris_native") for f in os.listdir(os.path.dirname(os.path.abspath(__file__)))):
            _reason = "the compiled module does not load here (%s)" % str(e).splitlines()[0][:160]
        else:
            _reason = "not built (no compiled module in the package: docs/NATIVE.md)"
    else:
        if getattr(_n, "VERSION", 0) != VERSION:
            _n, _reason = None, "the compiled module is version %s, this package needs %d (rebuild it)" % (
                getattr(_n, "VERSION", "?"), VERSION)
if _n is None:
    _want = ()

AVAILABLE = _n is not None
ACTIVE = tuple(p for p in PARTS if p in _want)
Fallback = _n.Fallback if AVAILABLE else type("Fallback", (Exception,), {})


class Rounded(tuple):
    """Rounded((values, digits)): for `dumps`, the list `values` written rounded (native core only)."""
    __slots__ = ()


ROUNDED = False
if "json" in ACTIVE:
    _nd, _nl = _n.dumps, _n.loads
    _n.set_rounded_type(Rounded)
    ROUNDED = True

    def dumps(obj):
        try:
            return _nd(obj)
        except Fallback:
            return _dumps_py(obj, separators=_SEP)

    def loads(raw):
        try:
            return _nl(raw)
        except Fallback:
            return _loads_py(raw)
else:
    def dumps(obj):
        return _dumps_py(obj, separators=_SEP)

    loads = _loads_py

round = _n.round if "round" in ACTIVE else builtins.round
round_list = _n.round_list if "round" in ACTIVE else None      # [v if v is None else round(v, n) for v in list]
interp_all = _n.interp_all if "interp" in ACTIVE else None
ekf = _n if "ekf" in ACTIVE else None
lanes = _n if "lanes" in ACTIVE else None


def status():
    """For `/v2/health`: is the native core in use, which parts, and if not, why."""
    out = {"active": bool(ACTIVE), "parts": list(ACTIVE), "implementation": "native" if ACTIVE else "python"}
    if AVAILABLE:
        out["version"] = _n.VERSION
        out["module"] = os.path.basename(getattr(_n, "__file__", "") or "")
    if _reason:
        out["reason"] = _reason
    elif AVAILABLE and len(ACTIVE) < len(PARTS):
        out["reason"] = "parts selected by CHRIS_NATIVE"
    return out
