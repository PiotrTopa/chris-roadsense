"""Parse only what is needed of a large JSON line. A `road` message is about 6 kB, four fifths of it lane
lines and road edges the integrator's estimator does not read; `drop()` cuts named top-level members out of
the bytes before the JSON parser sees them. Falls back to the whole line whenever the cut is not certain."""
import json
import re

_TOK = re.compile(rb'[\[\]{}"]')


def _value_end(buf, i):
    """Index just past the JSON value that starts at buf[i] (an array or object), or -1."""
    depth = 0
    for m in _TOK.finditer(buf, i):
        c = buf[m.start()]
        if c == 0x22:                       # a string: no escapes are expected in the members cut here
            return -1
        if c in (0x5B, 0x7B):
            depth += 1
        else:
            depth -= 1
            if depth == 0:
                return m.end()
    return -1


def drop(line, keys):
    """`line` (bytes) without the top-level members `keys` whose values are arrays / objects of numbers."""
    for k in keys:
        pat = b',"' + k + b'":'
        i = line.find(pat)
        if i < 0:
            continue
        j = i + len(pat)
        if j >= len(line) or line[j] not in (0x5B, 0x7B):
            continue
        # strings inside the member (lane lines carry none): give up on this member, keep the line whole
        end = _value_end_numeric(line, j)
        if end > 0:
            line = line[:i] + line[end:]
    return line


def _value_end_numeric(buf, i):
    depth = 0
    for m in _TOK.finditer(buf, i):
        c = buf[m.start()]
        if c == 0x22:
            # a key or string inside: skip to its closing quote (keys of the stack's messages hold no escapes)
            q = buf.find(b'"', m.end())
            if q < 0 or buf.find(b"\\", m.end(), q) >= 0:
                return -1
            # continue the scan after the string: restart the iterator there
            return _resume(buf, q + 1, depth)
        if c in (0x5B, 0x7B):
            depth += 1
        else:
            depth -= 1
            if depth == 0:
                return m.end()
    return -1


def _resume(buf, i, depth):
    while True:
        m = _TOK.search(buf, i)
        if m is None:
            return -1
        c = buf[m.start()]
        if c == 0x22:
            q = buf.find(b'"', m.end())
            if q < 0 or buf.find(b"\\", m.end(), q) >= 0:
                return -1
            i = q + 1
            continue
        depth += 1 if c in (0x5B, 0x7B) else -1
        if depth == 0:
            return m.end()
        i = m.end()


ROAD_DROP = (b"lane_lines", b"road_edges", b"meta", b"desire_state", b"desire_pred")


def loads_road(line):
    """A `road` line parsed without the members the integrator's core does not read."""
    try:
        return json.loads(drop(line, ROAD_DROP))
    except ValueError:
        return json.loads(line)
