"""The native core against the pure-Python reference (chris_integrator/native).

Every native function is compared with the Python it stands for: text and objects must be identical; the
estimator's linear algebra is identical where the steps are elementwise and agrees to rounding where sums of
products are involved. Where the native module is not built, the comparisons are skipped and the selection
logic is still tested: the package is complete without a compiler.

    python -m pytest tests/test_native.py                    with the native core, if built
    CHRIS_NATIVE=0 python -m pytest tests                    the whole suite on the reference
"""
import json
import math
import os
import random
import struct
import subprocess
import sys

import numpy as np
import pytest

HERE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, HERE)

from chris_integrator import native as N  # noqa: E402

try:
    from chris_integrator.native import _chris_native as C
except ImportError:
    C = None

needs_native = pytest.mark.skipif(C is None, reason="the native core is not built (docs/NATIVE.md)")
# comparisons that go through the package's own switch points need that part switched on in this process
needs_json = pytest.mark.skipif(not N.ROUNDED, reason="the native JSON part is not in use in this process")
needs_ekf = pytest.mark.skipif(N.ekf is None, reason="the native filter part is not in use in this process")
SEP = (",", ":")


def _ref_dumps(o):
    return json.dumps(o, separators=SEP)


def _native_dumps(o):
    """The native writer, or None where it hands the object to json."""
    try:
        return C.dumps(o)
    except C.Fallback:
        return None


def _rand_double(rng):
    k = rng.randrange(8)
    if k == 0:
        return struct.unpack("<d", struct.pack("<Q", rng.getrandbits(64)))[0]       # any bit pattern
    if k == 1:
        return round(rng.uniform(-200.0, 200.0), rng.randrange(0, 7))               # what the stack writes most
    if k == 2:
        return rng.uniform(-1.0, 1.0) * 10.0 ** rng.randrange(-30, 30)
    if k == 3:
        return float(rng.randrange(-10 ** 6, 10 ** 6))
    if k == 4:
        return rng.randrange(-10 ** 17, 10 ** 17) / 10.0 ** rng.randrange(0, 23)
    if k == 5:
        return rng.choice([0.0, -0.0, 1e-4, 9.999e-5, 1e15, 1e16, 1e-5, 5e-324, 1.7976931348623157e308, 0.1, 0.3,
                           2.675, 1e22, 1e23, 123456789012345.0, 999999999999999.9, 0.30000000000000004])
    if k == 6:
        return math.ldexp(rng.randrange(1, 1 << 53), rng.randrange(-80, 30))
    return rng.uniform(-1e4, 1e4)


def _rand_str(rng):
    alphabet = ["a", "Z", "0", " ", '"', "\\", "/", "\n", "\r", "\t", "\b", "\f", "\x00", "\x1f", "\x7f", "é", "ś",
                " ", "￿", "\U0001F600", "\U0010ffff", "{", "]", ","]
    return "".join(rng.choice(alphabet) for _ in range(rng.randrange(0, 12)))


def _rand_obj(rng, depth=0):
    k = rng.randrange(14 if depth < 5 else 9)
    if k == 0:
        return None
    if k == 1:
        return rng.choice([True, False])
    if k == 2:
        return rng.randrange(-10 ** 6, 10 ** 6)
    if k == 3:
        return rng.choice([0, -1, 2 ** 63 - 1, -2 ** 63, 2 ** 63, -2 ** 63 - 1, 10 ** 30, 1795084321890056390])
    if k in (4, 5, 6):
        return _rand_double(rng)
    if k == 7:
        return rng.choice([float("nan"), float("inf"), float("-inf")])
    if k == 8:
        return _rand_str(rng)
    if k in (9, 10):
        return [_rand_obj(rng, depth + 1) for _ in range(rng.randrange(0, 6))]
    if k == 11:
        return tuple(_rand_obj(rng, depth + 1) for _ in range(rng.randrange(0, 4)))
    return {_rand_str(rng): _rand_obj(rng, depth + 1) for _ in range(rng.randrange(0, 6))}


# ---- selection ---------------------------------------------------------------------------------------------
def test_status_says_which_implementation_runs():
    st = N.status()
    assert st["implementation"] in ("native", "python")
    assert st["active"] == bool(st["parts"])
    assert set(st["parts"]) <= set(N.PARTS)
    if not st["active"]:
        assert st.get("reason")
    # the reference is always there
    assert N.dumps({"a": [1, 2.5, None]}) == '{"a":[1,2.5,null]}'
    assert N.loads('{"a":[1,2.5,null]}') == {"a": [1, 2.5, None]}
    assert N.round(2.675, 2) == round(2.675, 2)


@pytest.mark.parametrize("env,parts", [("0", []), ("json", ["json"]), ("ekf,round", ["round", "ekf"])])
def test_selection_by_environment(env, parts):
    code = "import json; from chris_integrator import native as N; print(json.dumps(N.status()))"
    out = subprocess.run([sys.executable, "-c", code], cwd=HERE, env=dict(os.environ, CHRIS_NATIVE=env),
                         capture_output=True, text=True, check=True).stdout
    st = json.loads(out)
    if C is None or env == "0":
        assert st["parts"] == [] and st["implementation"] == "python" and st["reason"]
    else:
        assert st["parts"] == parts and st["implementation"] == "native"


# ---- JSON text ---------------------------------------------------------------------------------------------
@needs_native
def test_dumps_floats_as_repr():
    rng = random.Random(1)
    taken = 0
    for _ in range(400000):
        v = _rand_double(rng)
        if not math.isfinite(v):
            continue
        assert C.dumps(v) == repr(v), v
        taken += 1
    assert taken > 300000
    # every number of two and three decimals in the range the stack writes
    for d in (2, 3):
        div = 10.0 ** d
        vals = [k / div for k in range(-40000, 40000)]
        assert C.dumps(vals) == _ref_dumps(vals)


@needs_native
def test_dumps_objects_as_json():
    rng = random.Random(2)
    native = 0
    for _ in range(20000):
        o = _rand_obj(rng)
        got = _native_dumps(o)
        if got is not None:
            native += 1
            assert got == _ref_dumps(o)
        assert N.dumps(o) == _ref_dumps(o)             # the public function: native or the reference, same text
    assert native > 10000


@needs_native
def test_dumps_leaves_to_json_what_it_does_not_know():
    class Sub(float):
        pass
    import enum

    class E(enum.IntEnum):
        A = 3
    for o in ({1: 2}, {None: 1}, {1.5: 1}, {True: 1}, 10 ** 30, "\ud800", [np.float64(1.5)], [Sub(2.5)], [E.A],
              [np.int64(3)], {"a": object()}, [set()], b"x"):
        try:
            ref = _ref_dumps(o)
        except (TypeError, ValueError) as e:
            with pytest.raises(type(e)):
                N.dumps(o)
            continue
        assert N.dumps(o) == ref
    import collections
    od = collections.OrderedDict([("a", 1), ("b", 2), ("c", 3)])
    od.move_to_end("a")                                 # its own order, not the underlying dict's
    dd = collections.defaultdict(list, {"k": [1.5]})
    for o in (od, {"x": od}, dd):
        assert N.dumps(o) == _ref_dumps(o)
    deep = cur = []
    for _ in range(500):
        cur.append([])
        cur = cur[0]
    assert N.dumps(deep) == _ref_dumps(deep)
    loop = []
    loop.append(loop)
    with pytest.raises(ValueError):
        N.dumps(loop)


@needs_json
def test_dumps_rounded_lists():
    rng = random.Random(3)
    for _ in range(3000):
        nd = rng.randrange(0, 7)
        vals = []
        for _ in range(rng.randrange(0, 40)):
            k = rng.randrange(10)
            if k < 5:
                vals.append(rng.uniform(-300.0, 300.0))
            elif k == 5:
                vals.append((rng.randrange(-10 ** 5, 10 ** 5) + 0.5) / 10.0 ** nd)      # on a decimal tie
            elif k == 6:
                vals.append(rng.randrange(-200, 200))                                   # an int stays an int
            elif k == 7:
                vals.append(rng.choice([None, 0.0, -0.0, -1e-9, 1e-9, True, 1e12, -1e300, 5e-324]))
            elif k == 8:
                vals.append(float(rng.randrange(-200, 200)))
            else:
                vals.append(_rand_double(rng))
        vals = [v for v in vals if not (isinstance(v, float) and not math.isfinite(v))]
        ref = _ref_dumps({"a": [v if v is None else round(v, nd) for v in vals], "b": 1})
        assert C.dumps({"a": N.Rounded((vals, nd)), "b": 1}) == ref


@needs_native
def test_recorder_road_line_is_the_reference_line():
    from chris_integrator.io import recorder as R
    rng = random.Random(4)

    def line(n):
        return {"x": [float(i * 3) if rng.random() < 0.5 else i * 3 for i in range(n)],
                "y": [rng.uniform(-5, 5) for _ in range(n)], "y_std": [rng.uniform(0, 2) for _ in range(n)],
                "z": [0.0] * n, "prob": rng.random()}
    for _ in range(200):
        road = {"type": "road", "seq": rng.randrange(10 ** 6), "ts_mono_ns": rng.randrange(10 ** 15), "v_ego": rng.uniform(0, 40),
                "valid": True, "pose": {"vx": rng.uniform(0, 40), "wz": rng.uniform(-1, 1), "n": 3},
                "lane_lines": [line(33) for _ in range(4)], "road_edges": [line(33) for _ in range(2)],
                "leads": [{"x": [rng.uniform(0, 100) for _ in range(6)], "prob": rng.random(), "note": "\x01"}],
                "plan": {"x": [rng.uniform(0, 200) for _ in range(33)], "y": [rng.uniform(-9, 9) for _ in range(33)],
                         "z": [rng.uniform(-2, 2) for _ in range(33)], "held": False, "age_ms": 3.25}}
        assert R.slim_road_line(road) == _ref_dumps(R.slim_road(road))


# ---- JSON parsing ------------------------------------------------------------------------------------------
def _same(a, b):
    """Equal as json.loads results: types, values, the sign of zero, NaN, key order."""
    if type(a) is not type(b):
        return False
    if isinstance(a, float):
        return (a != a and b != b) or (a == b and math.copysign(1.0, a) == math.copysign(1.0, b))
    if isinstance(a, dict):
        return list(a) == list(b) and all(_same(a[k], b[k]) for k in a)
    if isinstance(a, list):
        return len(a) == len(b) and all(_same(x, y) for x, y in zip(a, b))
    return a == b


def _both_loads(raw):
    """The native reader and json.loads give the same object or both refuse."""
    try:
        ref = json.loads(raw)
    except (ValueError, RecursionError, TypeError) as e:
        with pytest.raises((ValueError, RecursionError) if not isinstance(e, TypeError) else TypeError):
            N.loads(raw)
        return None, type(e)
    got = N.loads(raw)
    assert _same(got, ref), raw[:200]
    return got, None


@needs_native
def test_loads_numbers():
    rng = random.Random(5)
    for _ in range(200000):
        k = rng.randrange(6)
        if k == 0:
            t = repr(_rand_double(rng))
        elif k == 1:
            t = "%s%d.%s" % (rng.choice(["", "-"]), rng.randrange(0, 10 ** rng.randrange(1, 20)),
                             "".join(rng.choice("0123456789") for _ in range(rng.randrange(1, 26))))
        elif k == 2:
            t = "%.*e" % (rng.randrange(0, 20), _rand_double(rng))
        elif k == 3:
            t = str(rng.randrange(-10 ** rng.randrange(1, 25), 10 ** rng.randrange(1, 25)))
        elif k == 4:
            t = "%.*f" % (rng.randrange(0, 20), rng.uniform(-1e6, 1e6))
        else:
            t = rng.choice(["-0", "-0.0", "0.0", "0", "1E5", "1e+5", "1e-5", "01", "1.", ".5", "-", "+1", "1e", "1e+",
                            "0x10", "NaN", "Infinity", "-Infinity", "1.0e400", "-1e-400", "9007199254740993",
                            "9007199254740993.0", "0.1e1", "1_0", "9223372036854775807", "9223372036854775808",
                            "-9223372036854775808", "-9223372036854775809", "1.7976931348623157e308", "4.9e-324"])
        if t in ("nan", "inf", "-inf"):
            continue
        _both_loads(t)
        _both_loads("[%s,%s]" % (t, t))


@needs_native
def test_loads_objects_and_text():
    rng = random.Random(6)
    for _ in range(20000):
        o = _rand_obj(rng)
        try:
            text = json.dumps(o, separators=rng.choice([SEP, (", ", ": ")]), ensure_ascii=rng.random() < 0.7)
        except (TypeError, ValueError):
            continue
        _both_loads(text)
        try:
            _both_loads(text.encode("utf-8"))
        except UnicodeEncodeError:
            pass
    for text in ('{"a":1,"a":2}', ' \t\n{"a" : [ 1 , 2 ] }\r\n', "[]", "{}", '""', '"\\u0041\\ud83d\\ude00\\/"', '"\\ud83d"',
                 '"\\udc00x"', '"a\\qb"', '"a\nb"', '"\\u12"', '{"a":1,}', "[1,]", "[1 2]", '{"a" 1}', "{1:2}", "", " ",
                 "nul", "true false", "[1]x", '"abc', "[[[[", '{"a":{"b":{"c":[1,{"d":null}]}}}', "﻿[1]"):
        _both_loads(text)
        _both_loads(text.encode("utf-8", "surrogatepass"))
    for raw in (b"\xff", b'"\xff"', b'"\xc3"', b'"\xed\xa0\x80"', b"\xef\xbb\xbf[1]", "[1]".encode("utf-16"),
                bytearray(b"[1,2]"), memoryview(b'{"a":1}')):
        _both_loads(raw)
    deep = "[" * 200 + "]" * 200
    assert _same(N.loads(deep), json.loads(deep))


@needs_native
def test_loads_survives_damaged_input():
    """Lines cut short and with bytes changed: the same result as json.loads or an error, never anything else
    (the reader's bounds checks; build with -fsanitize=address to make this test a memory check)."""
    rng = random.Random(7)
    base = []
    for _ in range(300):
        try:
            base.append(json.dumps(_rand_obj(rng), separators=SEP).encode())
        except (TypeError, ValueError):
            pass
    road = json.dumps({"type": "road", "seq": 5, "ts_mono_ns": 3226220852221, "plan": {"x": [0.5 * i for i in range(33)]},
                       "lane_lines": [{"x": [1.25 * i for i in range(33)], "prob": 0.9871}] * 4, "s": "é\n"}).encode()
    base.append(road)
    for _ in range(40000):
        b = bytearray(rng.choice(base))
        if not b:
            continue
        k = rng.randrange(4)
        if k == 0:
            del b[rng.randrange(len(b)):]
        elif k == 1:
            b[rng.randrange(len(b))] = rng.randrange(256)
        elif k == 2:
            i = rng.randrange(len(b))
            b[i:i] = bytes(rng.randrange(256) for _ in range(rng.randrange(1, 4)))
        else:
            i = rng.randrange(len(b))
            del b[i:i + rng.randrange(1, 5)]
        try:
            _both_loads(bytes(b))
        except UnicodeDecodeError:
            # json.loads raises UnicodeDecodeError (a ValueError) for bytes that are not text: so does the native path
            pass


# ---- round, interpolation ----------------------------------------------------------------------------------
@needs_native
def test_round_is_builtin_round():
    rng = random.Random(8)
    for _ in range(400000):
        n = rng.randrange(0, 8)
        k = rng.randrange(5)
        if k == 0:
            x = rng.uniform(-500.0, 500.0)
        elif k == 1:
            x = (rng.randrange(-10 ** 6, 10 ** 6) + 0.5) / 10.0 ** n                   # decimal ties
        elif k == 2:
            x = _rand_double(rng)
        elif k == 3:
            x = math.nextafter((rng.randrange(-10 ** 5, 10 ** 5) + 0.5) / 10.0 ** n, rng.choice([-math.inf, math.inf]))
        else:
            x = rng.randrange(-10 ** 4, 10 ** 4) / 10.0 ** rng.randrange(0, 5)
        if not math.isfinite(x):
            continue
        a, b = C.round(x, n), round(x, n)
        assert type(a) is type(b) and a == b and math.copysign(1.0, a) == math.copysign(1.0, b), (x, n)
    for args in ((5, 2), (5,), (2.5,), (3.5, None), (2.675, -1), (1234.5, -2), (True, 1), (np.float64(2.675), 2),
                 (1e300, 3), (float("inf"), 2), (1.5, 30)):
        a, b = C.round(*args), round(*args)
        assert type(a) is type(b) and (a == b)
    assert math.isnan(C.round(float("nan"), 2))
    for _ in range(2000):
        n = rng.randrange(0, 6)
        vals = [rng.choice([None, 5, True, rng.uniform(-300, 300), (rng.randrange(-10 ** 5, 10 ** 5) + 0.5) / 10.0 ** n,
                            -1e-9, 0.0, -0.0, 1e300]) for _ in range(rng.randrange(0, 30))]
        a, b = C.round_list(vals, n), [v if v is None else round(v, n) for v in vals]
        assert len(a) == len(b) and all(type(u) is type(v) and repr(u) == repr(v) for u, v in zip(a, b)), (vals, n)
    for bad in ((), ("x", 2), (1.5, "a")):
        with pytest.raises(TypeError):
            C.round(*bad)


@needs_native
def test_interp_all_is_the_python_walk():
    from chris_integrator.trackers import view as V
    rng = random.Random(9)
    ref = V._interp_all_py
    for _ in range(20000):
        n = rng.randrange(0, 20)
        xs = [rng.uniform(0, 100) for _ in range(n)]
        if rng.random() < 0.8:
            xs.sort()
        if n > 2 and rng.random() < 0.2:
            xs[rng.randrange(1, n)] = xs[rng.randrange(0, n - 1)]                       # a zero-length segment
        ys = [rng.choice([None, float("nan")]) if rng.random() < 0.05 else rng.uniform(-5, 5)
              for _ in range(rng.randrange(max(n - 2, 0), n + 3))]
        qs = [rng.choice([float("nan"), -1.0, 200.0]) if rng.random() < 0.05 else rng.uniform(-5, 105)
              for _ in range(rng.randrange(0, 30))]
        if rng.random() < 0.7:
            qs.sort(key=lambda v: (v != v, v))
        a, b = V._interp_all(xs, ys, qs), ref(xs, ys, qs)
        assert len(a) == len(b)
        for u, v in zip(a, b):
            assert (u is None and v is None) or (u != u and v != v) or (u == v and type(u) is type(v)), (xs, ys, qs)
    # ints among the floats (the model's fixed distances): the same values and types as the Python walk
    for _ in range(20000):
        n = rng.randrange(1, 12)
        num = lambda: rng.choice([rng.randrange(-5, 120), float(rng.randrange(-5, 120)), rng.uniform(-5, 120)])  # noqa: E731
        xs = sorted(num() for _ in range(n))
        ys = [None if rng.random() < 0.05 else num() for _ in range(n)]
        qs = sorted(num() for _ in range(rng.randrange(0, 12)))
        if rng.random() < 0.5:
            xs, ys, qs = rng.choice([xs, tuple(xs)]), rng.choice([ys, tuple(ys)]), tuple(qs)     # the view's `xs` is a tuple
        a, b = V._interp_all(xs, ys, qs), ref(xs, ys, qs)
        assert len(a) == len(b) and all(type(u) is type(v) and u == v for u, v in zip(a, b)), (xs, ys, qs)
    assert V._interp_all([0, 2 ** 60], [0, 5], [5]) == ref([0, 2 ** 60], [0, 5], [5])     # big ints, other types:
    assert V._interp_all([0.0, 10.0], [np.float64(0.0), 5.0], [5]) == ref([0.0, 10.0], [np.float64(0.0), 5.0], [5])   # the reference


# ---- the estimator's linear algebra ------------------------------------------------------------------------
def _cov(rng, n):
    a = rng.standard_normal((n, n))
    return a @ a.T + np.diag(rng.uniform(0.01, 1.0, n))


def _model(rng, naux=17):
    from chris_integrator.core import ego as E
    m = E.EgoModel()
    for i in range(naux):
        k = i % 3
        if k == 0:
            m.add_aux("s%d" % i, E.AuxState("a", 0.0, 0.1, psd=1e-6))
        elif k == 1:
            m.add_aux("s%d" % i, E.AuxState("a", 0.0, 0.05, tau_s=30.0 + i))
        else:
            m.add_aux("s%d" % i, E.AuxState("a", 0.0, 0.2, psd=1e-5, lo=-0.5, hi=0.5))
    return m


def _state(rng, m):
    x = rng.standard_normal(m.n)
    x[3] = rng.uniform(-5, 40)
    x[9] = rng.uniform(-0.05, 0.05)
    return x, _cov(rng, m.n)


@needs_ekf
@pytest.mark.parametrize("naux", [0, 1, 17])
def test_predict_agrees_with_numpy(naux):
    rng = np.random.default_rng(10 + naux)
    m = _model(rng, naux)
    worst = 0.0
    for i in range(400):
        x, P = _state(rng, m)
        if i % 7 == 0:
            x[10:] = rng.uniform(-2, 2, m.n - 10)                                       # beyond the clamps
        dt = float(rng.choice([0.01, 0.02, 0.05, 0.1, 0.3, 0.7, 2.5]))
        if i % 11 == 0:
            P[3, 3] += 1e5                                                              # the caps of a long gap
        xr, Pr, (Fr, rr) = m._predict_py(x.copy(), P.copy(), dt)
        xn, Pn, (Fn, rn) = m.predict(x, P, dt)
        assert xn is not x and Pn is not P
        assert np.array_equal(xr, xn)                                                   # the state: the same steps
        assert np.array_equal(Fr, Fn)
        assert (rr is None and rn is None) or np.array_equal(rr, rn)
        scale = np.sqrt(np.outer(np.diag(Pr), np.diag(Pr)))
        worst = max(worst, float(np.max(np.abs(Pr - Pn) / scale)))
    assert worst < 1e-12, worst                                                         # the covariance: to rounding


@needs_ekf
def test_row_update_is_numpy_bit_for_bit_on_single_index_rows():
    from chris_integrator.core import ekf as K
    rng = np.random.default_rng(20)
    n = 27
    for i in range(2000):
        P = _cov(rng, n)
        x = rng.standard_normal(n)
        idx = np.array([rng.integers(0, n)], dtype=np.intp)
        coef = np.array([rng.choice([1.0, -1.0, rng.uniform(-3, 3)])])
        ph_r, s0_r = K._project_py(P, idx, coef)
        ph_n, s0_n = K._project_native(P, idx, coef)
        assert np.array_equal(ph_r, ph_n) and s0_r == s0_n
        nu, var = float(rng.standard_normal()), float(rng.uniform(1e-4, 1.0))
        freeze = None if i % 3 else np.array(sorted(set(rng.integers(0, n, 6).tolist())), dtype=np.intp)
        xr, xn = x.copy(), x.copy()
        Pr = K._apply_py(xr, P, ph_r, s0_r + var, nu, freeze, False)
        Pn = K._apply_native(xn, P, ph_n, s0_n + var, nu, freeze, False)
        assert Pn is not P
        assert np.array_equal(xr, xn) and np.array_equal(Pr, Pn)
        Pn2 = K._apply_native(xn, Pn, ph_n, s0_n + var, nu, freeze, True)               # in place on an own array
        assert Pn2 is Pn
        S = P + 1e-3 * rng.standard_normal((n, n))
        Sr = K._sym_py(S, False)
        Sn = K._sym_native(S.copy(), True)
        assert np.array_equal(Sr, Sn)


@needs_ekf
def test_row_projection_agrees_with_numpy_on_wide_rows():
    from chris_integrator.core import ekf as K
    rng = np.random.default_rng(21)
    n = 27
    for _ in range(2000):
        P = _cov(rng, n)
        m = int(rng.integers(2, 7))
        idx = rng.choice(n, m, replace=False).astype(np.intp)
        coef = rng.standard_normal(m)
        ph_r, s0_r = K._project_py(P, idx, coef)
        ph_n, s0_n = K._project_native(P, idx, coef)
        np.testing.assert_allclose(ph_n, ph_r, rtol=1e-12, atol=1e-12)
        assert abs(s0_n - s0_r) <= 1e-12 * max(abs(s0_r), 1.0)
    # what the native step does not take goes to numpy: lists, views, bad indices
    P = _cov(rng, n)
    ph, s0 = K._project_native(P, [3], [1.0])
    assert np.array_equal(ph, P[:, 3]) and s0 == P[3, 3]
    ph, s0 = K._project_native(P[::-1], np.array([3], dtype=np.intp), np.array([1.0]))
    assert np.array_equal(ph, P[::-1][:, 3])
    with pytest.raises(IndexError):
        K._project_native(P, np.array([99], dtype=np.intp), np.array([1.0]))


@needs_ekf
def test_smoother_step_agrees_with_numpy():
    rng = np.random.default_rng(30)
    m = _model(rng)
    n = m.n
    for _ in range(300):
        x, P = _state(rng, m)
        dt = float(rng.choice([0.01, 0.05, 0.1, 0.4]))
        xp, Pp, F = m._predict_py(x, P, dt)
        xs = xp + 0.01 * rng.standard_normal(n)
        Ps = Pp - 0.1 * np.diag(np.diag(Pp))
        C_ = np.linalg.solve(Pp, m.apply_F(F, P)).T
        xr, Pr = x + C_ @ (xs - xp), P + C_ @ (Ps - Pp) @ C_.T
        xn, Pn = m.rts(Pp, F, P, x, xp, xs, Ps, ())
        np.testing.assert_allclose(xn, xr, rtol=1e-9, atol=1e-9)
        np.testing.assert_allclose(Pn, Pr, rtol=1e-8, atol=1e-9)
    assert m.rts(np.zeros((n, n)), F, P, x, xp, xs, Ps, ()) is False                   # singular: the caller keeps the filtered state
    xw, _ = m.rts(Pp, F, P, x, xp, xs + np.eye(n)[2] * 2 * math.pi, Ps, (2,))          # a wrapped angle difference
    np.testing.assert_allclose(xw, xn, rtol=1e-9, atol=1e-9)


SIM = r"""
import json, sys
sys.path.insert(0, %r)
from tests.simrun import Run
from chris_integrator import native
r = Run(seed=3, keep_tracks=True)
out = {"impl": native.status()["implementation"], "es": r.es,
       "rows": [[row.get(k) for k in ("dv", "dw", "de", "dn", "dpsi", "nees_pos")] for row in r.rows],
       "leads": [ld for _, ld, _ in r.leads]}
print(json.dumps(out))
"""


@needs_native
def test_simulated_drive_native_against_reference():
    """A simulated drive through the whole integrator on both implementations (two processes: the choice is
    made at import). The estimate's errors against the truth agree far inside their own size; the published
    messages agree to the last digit they print in all but a few values."""
    res = {}
    for env in ("0", "1"):
        out = subprocess.run([sys.executable, "-c", SIM % HERE], cwd=HERE, env=dict(os.environ, CHRIS_NATIVE=env),
                             capture_output=True, text=True, check=True).stdout
        res[env] = json.loads(out)
    assert res["0"]["impl"] == "python" and res["1"]["impl"] == "native"
    a, b = res["0"]["rows"], res["1"]["rows"]
    assert len(a) == len(b) and len(a) > 300
    worst = 0.0
    for ra, rb in zip(a, b):
        for u, v in zip(ra, rb):
            assert (u is None) == (v is None)
            if u is not None:
                worst = max(worst, abs(u - v) / max(abs(u), abs(v), 1e-3))
    assert worst < 1e-6, worst
    la, lb = res["0"]["leads"], res["1"]["leads"]
    same = sum(1 for u, v in zip(la, lb) if u == v)
    assert len(la) == len(lb) and same >= 0.99 * len(la), (same, len(la))
    # the last ego state: every number equal to 1e-6 relative, every flag and string equal

    def cmp(u, v, path=""):
        assert type(u) is type(v) or {type(u), type(v)} <= {int, float}, path
        if isinstance(u, dict):
            assert list(u) == list(v), path
            for k in u:
                cmp(u[k], v[k], path + "." + k)
        elif isinstance(u, list):
            assert len(u) == len(v), path
            for i, (x, y) in enumerate(zip(u, v)):
                cmp(x, y, "%s[%d]" % (path, i))
        elif isinstance(u, float):
            assert abs(u - v) <= 1e-6 * max(abs(u), abs(v), 1.0), (path, u, v)
        else:
            assert u == v, (path, u, v)
    cmp(res["0"]["es"], res["1"]["es"])


# ---- the lane tracker's array steps ------------------------------------------------------------------------
needs_lanes = pytest.mark.skipif(N.lanes is None, reason="the native lane part is not in use in this process")


def _road_lines(rng, n=33, odd=0.0):
    x = [0.0] + sorted(rng.uniform(1.0, 190.0) for _ in range(n - 1))

    def line(edge):
        d = {"x": list(x), "y": [rng.uniform(-8.0, 8.0) for _ in range(n)], "y_std": [rng.uniform(0.0, 3.0) for _ in range(n)]}
        if not edge:
            d["prob"] = rng.choice([rng.random(), 0, 1, True, None, 1.5, -0.2, float("nan")])
        r = rng.random()
        if r < odd:
            k = rng.randrange(9)
            if k == 0:
                d["y"][rng.randrange(n)] = float("nan")
            elif k == 1:
                d["y_std"][rng.randrange(n)] = float("inf")
            elif k == 2:
                del d["y_std"]
            elif k == 3:
                d["y"] = d["y"][:-1]
            elif k == 4:
                d["y"][rng.randrange(n)] = 3                     # an int among the floats
            elif k == 5:
                d["y_std"][rng.randrange(n)] = 0.0               # below the floor
            elif k == 6:
                d["y"][rng.randrange(n)] = "1.5"
            elif k == 7:
                d["y_std"] = None
            else:
                d["prob"] = "x"
        return d
    return {"lane_lines": [line(False) for _ in range(rng.choice([4, 4, 4, 3]))],
            "road_edges": [line(True) for _ in range(rng.choice([2, 2, 2, 1]))]}


def _same_arrays(a, b):
    return all(np.array_equal(u, v, equal_nan=True) and u.dtype == v.dtype for u, v in zip(a, b))


@needs_lanes
def test_lane_measure_fit_update_round_are_numpy_bit_for_bit(monkeypatch):
    from chris_integrator.trackers import lanes as L
    rng = random.Random(41)
    g = np.random.default_rng(41)
    tr = L.RoadGeometryTracker()
    native = taken = 0
    for i in range(3000):
        road = _road_lines(rng, odd=0.3 if i % 2 else 0.0)
        got = tr._measure(road)
        with monkeypatch.context() as mp:
            mp.setattr(L, "_NL", None)
            try:
                ref = tr._measure(road)
            except (TypeError, ValueError) as e:                # what the reference refuses, the native path refuses
                with pytest.raises(type(e)):
                    tr._measure(road)
                continue
        assert (got is None) == (ref is None)
        if ref is None:
            continue
        taken += 1
        assert _same_arrays(got, ref), road
        x, zy, zs, prob, ok = ref
        n = len(x)
        y, p = g.standard_normal((6, n)), np.abs(g.standard_normal((6, n))) + 1e-3
        c = dict(tr.cfg)
        if i % 5 == 0:
            c["gate_x"] = (500.0, 600.0)                         # no point in the gate: all of them count
        elif i % 7 == 0:
            c["gate_x"] = (rng.uniform(0, 50), rng.uniform(50, 200))
        a, b = L._fit_native(c, x, zs, zy, prob, y, p), L._fit_py(c, x, zs, zy, prob, y, p)
        for name, u, v in zip(("r", "gate", "nu", "jumps", "nsigs"), a, b):
            assert u.dtype == v.dtype and np.array_equal(u, v, equal_nan=True), (name, i, c["gate_x"])
        r, gate, nu = b[0], b[1], b[2]
        upd = sorted(rng.sample(range(6), rng.randrange(0, 7)))
        y1, p1, y2, p2 = y.copy(), p.copy(), y.copy(), p.copy()
        if upd:
            L._update_native(y1, p1, r, nu, upd)
            L._update_py(y2, p2, r, nu, upd)
            assert np.array_equal(y1, y2) and np.array_equal(p1, p2)
        p1[0, 0] = -1.0                                          # a negative variance: NaN from the root, as numpy
        for arr, root in ((y1, False), (p1, True), (x, False)):
            u, v = L._round2_native(arr, root), L._round2_py(arr, root)
            assert json.dumps(u) == json.dumps(v)
        q = (c["q0"] + c["q1"] * x) ** 2 * 0.07
        pr = np.minimum(p2 + q, c["max_std"] ** 2)
        C.lanes_predict(p2, x, c["q0"], c["q1"], 0.07, c["max_std"] ** 2)
        assert np.array_equal(p2, pr)
        native += 1
    assert taken > 2500 and native > 2500


@needs_lanes
def test_lane_tracker_messages_native_against_numpy(monkeypatch):
    """The whole tracker, frame by frame, on both implementations: every `lanes` message equal as text."""
    from chris_integrator.trackers import lanes as L
    rng = random.Random(42)
    frames = []
    ts = 10 ** 12
    base = _road_lines(rng)
    for i in range(1500):
        ts += int(rng.choice([0.05, 0.066, 0.07, 0.2, 1.4]) * 1e9) if rng.random() < 0.97 else 0
        if rng.random() < 0.05:
            base = _road_lines(rng, odd=0.3)
        road = {"type": "road", "seq": i, "ts_mono_ns": ts, "pose": {"vy": rng.uniform(-0.2, 0.2)}}
        for key in ("lane_lines", "road_edges"):
            road[key] = [dict(d, y=[v + rng.gauss(0, 0.05 if rng.random() < 0.9 else 2.0) if isinstance(v, float) else v
                                    for v in d["y"]],
                              **({"prob": min(1.0, max(0.0, rng.gauss(0.6, 0.3)))} if "prob" in d else {}))
                         for d in base[key]]
        es = {"speed": {"value": rng.uniform(0, 30), "valid": True, "sources": ["vehicle.speed"]},
              "yaw_rate": {"value": rng.uniform(-0.1, 0.1), "valid": True}, "sources": {"vision": {"state": "ok"}}}
        frames.append((road, es))

    def run():
        tr = L.RoadGeometryTracker()
        out = []
        for road, es in frames:
            try:
                out.append(json.dumps(tr.on_road(road, es)))
            except (TypeError, ValueError) as e:
                out.append("raised %s" % type(e).__name__)
        return out
    a = run()
    with monkeypatch.context() as mp:
        mp.setattr(L, "_NL", None)
        mp.setattr(L, "_fit", L._fit_py)
        mp.setattr(L, "_update", L._update_py)
        mp.setattr(L, "_round2", L._round2_py)
        b = run()
    assert len(a) == len(b) == 1500
    assert a == b
    assert sum(1 for s in a if '"present": true' in s) > 500
