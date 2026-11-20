"""Rounding many numbers at once with exactly the results of Python's `round(x, n)`.

`round(x, n)` on a float is correctly rounded in decimal (it goes through the shortest-string machinery),
which costs about 2 us a call on a phone's little core; a message with several hundred rounded numbers
spends more on rounding than on everything else. numpy's own `np.round` is fast but multiplies, rounds and
divides in binary and so differs from `round` in the last digit near ties.

`round_flat(values, scale)` does the binary computation for the whole list in a few numpy calls and repairs,
element by element with `round` itself, every case in which the two could differ:

* a value within 1e-6 of a tie after scaling (the product's rounding error is far below that for |x * 10^n|
  below 1e9; larger values are repaired too),
* a value that is a whole number (x * 10^n has no fraction and is a multiple of 10^n): `round` keeps an `int`
  an `int` (JSON `0` stays `0`, not `0.0`), and every `int` is among these,
* anything not finite.

For every other element k = rint(x * 10^n) is the integer the decimal rounding picks, and k / 10^n is the
double nearest to it, which is what `round` returns. tests/test_fastround.py checks this against `round`
on random, tie, integer and extreme inputs. Lists holding anything but numbers fall back to `round` per
element (None is kept)."""
import json
import math

import numpy as np

_TIE = 1e-6
_BIG = 1e9



def _slow(values, digits):
    return [v if v is None else round(v, d) for v, d in zip(values, digits)]


_WORK = {}                      # n -> work arrays of that length


def _work(n):
    """Work arrays for lists of n numbers, kept between calls: numpy hands out small arrays from a cache of its
    own, but not ones of a few kB, and the C library of a phone's Alpine-based system (musl) gives memory of
    that size back to the kernel at once: a dozen temporaries a frame showed as system time."""
    w = _WORK.get(n)
    if w is None:
        if len(_WORK) > 32:
            _WORK.clear()
        w = _WORK[n] = (np.empty(n), np.empty(n), np.empty(n), np.empty(n, dtype=np.int64), np.empty(n, dtype=object))
    return w


def round_flat(values, scale, digits):
    """[round(v, d) for v, d in zip(values, digits)] where scale[i] == 10.0 ** digits[i] (a float array of the
    same length as `values`, made once by `scales`)."""
    try:
        a = np.array(values, dtype=float)
    except (TypeError, ValueError):
        return _slow(values, digits)
    if a.shape != scale.shape:
        return _slow(values, digits)
    y, r, t, _, _ = _work(a.size)
    with np.errstate(invalid="ignore", over="ignore"):
        np.multiply(a, scale, out=y)
        np.rint(y, out=r)
        np.subtract(y, r, out=t)
        np.abs(t, out=t)
        ok = t < 0.5 - _TIE
        np.abs(y, out=t)
        ok &= t < _BIG
        np.fmod(y, scale, out=t)
        # not (...) so that NaN (None, nan, inf - inf) lands in the repair set; whole numbers: see the module text
        bad = ~ok
        bad |= t == 0.0
        np.divide(r, scale, out=t)
        out = t.tolist()
    if bad.any():
        for i in np.flatnonzero(bad).tolist():
            v = values[i]
            out[i] = v if v is None else round(v, digits[i])
    return out


def scales(digits):
    """The scale array `round_flat` wants for a per-element list of digit counts."""
    return np.array([10.0 ** d for d in digits])


# ---- the JSON text of rounded numbers, without float.__repr__ ---------------------------------------------
# Writing a float costs as much as rounding it (the shortest-string search: about 1.5 us on a phone's little
# core, a millisecond for a road message). A number rounded to n digits is k / 10^n for an integer k, and
# its text is the same for every occurrence of k: the texts of k = -32768 .. 32767 are kept per digit count
# (about 4 MB each, built on first use: `warm`) and fetched for a whole message in one indexing step.
_OFF = 32768
_TEXT = {}                      # digits -> object array of 65536 texts


def _table(d):
    t = _TEXT.get(d)
    if t is None:
        div = 10.0 ** d
        t = _TEXT[d] = np.array([repr((j - _OFF) / div) for j in range(2 * _OFF)], dtype=object)
    return t


def warm(digits=(2, 3)):
    """Build the text tables now (a start-up cost of about 0.15 s each on a phone, instead of in a frame)."""
    for d in digits:
        _table(d)


def _num(v, _isfinite=math.isfinite, _repr_f=float.__repr__):
    """The JSON text of one number as json.dumps writes it."""
    t = type(v)
    if t is int:
        return repr(v)
    if t is float and _isfinite(v):
        return _repr_f(v)
    return json.dumps(v)


def text_flat(values, scale, digits, table, base):
    """[json text of round(v, d) for v, d in zip(values, digits)]: the strings json.dumps would write for the
    rounded numbers. `table`: the texts of every digit count in use, one after the other (`Layout`); `base`:
    per element the index of its digit count's k = 0 in `table` (a float array)."""
    try:
        a = np.array(values, dtype=float)
    except (TypeError, ValueError):
        return [_num(v if v is None else round(v, d)) for v, d in zip(values, digits)]
    if a.shape != scale.shape:
        return [_num(v if v is None else round(v, d)) for v, d in zip(values, digits)]
    y, r, t, ki, ko = _work(a.size)
    with np.errstate(invalid="ignore", over="ignore"):
        np.multiply(a, scale, out=y)
        np.rint(y, out=r)
        np.subtract(y, r, out=t)
        np.abs(t, out=t)
        ok = t < 0.5 - _TIE
        np.abs(y, out=t)
        ok &= t < _OFF - 1
        np.fmod(y, scale, out=t)
        # repaired one by one: near a tie, outside the table, not finite; whole numbers (an int stays an int);
        # a negative number that rounds to zero ("-0.0")
        bad = ~ok
        bad |= t == 0.0
        bad |= (r == 0.0) & (y < 0.0)
        r[bad] = 0.0
        r += base
        np.copyto(ki, r, casting="unsafe")
        np.take(table, ki, out=ko, mode="clip")
        out = ko.tolist()
    if bad.any():
        for i in np.flatnonzero(bad).tolist():
            v = values[i]
            out[i] = _num(v if v is None else round(v, digits[i]))
    return out


class Layout:
    """`round_flat` for messages whose lists have the same lengths frame after frame: the scale array is kept
    per layout (the tuple of (length, digits) of the parts)."""

    def __init__(self):
        self._cache = {}

    @staticmethod
    def _fill(flat, parts):
        """The parts one after the other in the layout's own list (refilled in place: no list of a few kB is
        allocated per call)."""
        i = 0
        for p in parts:
            n = len(p)
            flat[i:i + n] = p
            i += n
        return flat

    def round_parts(self, parts, digits):
        """parts: lists of numbers; digits: the digit count of each part. Returns the rounded lists."""
        key = tuple(len(p) for p in parts) + tuple(digits)
        ent = self._cache.get(key)
        if ent is None:
            if len(self._cache) > 64:
                self._cache.clear()
            dg = [d for p, d in zip(parts, digits) for _ in p]
            ent = self._cache[key] = (scales(dg), dg, None, None, [0.0] * len(dg))
        flat = self._fill(ent[4], parts)
        out = round_flat(flat, ent[0], ent[1])
        res, i = [], 0
        for p in parts:
            n = len(p)
            res.append(out[i:i + n])
            i += n
        return res

    def text_parts(self, parts, digits):
        """The JSON text of each part rounded to its digit count: `[1.5,0,-0.25]` as json.dumps writes the list
        of rounded numbers (separators without spaces)."""
        key = tuple(len(p) for p in parts) + tuple(digits)
        ent = self._cache.get(key)
        if ent is None or ent[2] is None:
            if len(self._cache) > 64:
                self._cache.clear()
            dg = [d for p, d in zip(parts, digits) for _ in p]
            used = sorted(set(digits))
            first = {d: i * 2 * _OFF + _OFF for i, d in enumerate(used)}
            table = np.concatenate([_table(d) for d in used]) if used else np.zeros(0, dtype=object)
            ent = self._cache[key] = (scales(dg), dg, table, np.array([float(first[d]) for d in dg]), [0.0] * len(dg))
        flat = self._fill(ent[4], parts)
        out = text_flat(flat, ent[0], ent[1], ent[2], ent[3])
        res, i = [], 0
        for p in parts:
            n = len(p)
            res.append("[" + ",".join(out[i:i + n]) + "]")
            i += n
        return res
