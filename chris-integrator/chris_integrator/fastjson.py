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


# ---- the `road` line as the default stack reads it (lane L46) -----------------------------------------------
# Half of a `road` line is never read by any box of the default stack: sixteen of the plan's nineteen arrays,
# the height of every lane line and road edge, `meta` and the desire outputs. Parsing them costs about 0.7 ms
# a frame on a phone's little core. `loads_road_trimmed` cuts them out of the bytes before the parser sees
# them. Consumers are not affected: the line itself is passed on whole (WebSocket, sockets, GET /v2/road), and
# the recorder writes its reduced form, which never held these members. A stack with boxes of its own (which
# may read anything) is given the whole message: see `stack.build` (`trim_road`).
PLAN_UNREAD = ("vx", "vy", "vz", "ax", "ay", "az", "roll", "pitch", "yaw", "roll_rate", "pitch_rate", "yaw_rate",
               "x_std", "y_std", "z_std", "prob")
LINE_UNREAD = ("z",)
TOP_UNREAD = (b"meta", b"desire_state", b"desire_pred")
_PLAN_PAT = tuple((',"%s":[' % k).encode() for k in PLAN_UNREAD)
_LINE_PAT = tuple((',"%s":[' % k).encode() for k in LINE_UNREAD)


def trim_road(line):
    """`line` (bytes of a `road` message) without the members listed above. Every cut is an array of numbers
    found inside its parent's own span; whatever does not look as expected is left in."""
    cuts = []
    i = line.find(b',"plan":{')
    if i >= 0:
        e = line.find(b"}", i)             # the plan holds arrays of numbers and scalars: the first } closes it
        if e > 0:
            for pat in _PLAN_PAT:
                a = line.find(pat, i, e)
                if a >= 0:
                    b = line.find(b"]", a + len(pat), e)
                    if b > 0:
                        cuts.append((a, b + 1))
    for key in (b',"lane_lines":[{', b',"road_edges":[{'):
        i = line.find(key)
        if i < 0:
            continue
        e = line.find(b"}]", i)            # objects of arrays of numbers and scalars: the first }] ends the list
        if e < 0:
            continue
        for pat in _LINE_PAT:
            a = line.find(pat, i, e)
            while a >= 0:
                b = line.find(b"]", a + len(pat), e)
                if b < 0:
                    break
                cuts.append((a, b + 1))
                a = line.find(pat, b + 1, e)
    if cuts:
        cuts.sort()
        out, at = [], 0
        for a, b in cuts:
            if a < at:                     # overlapping spans: not the layout this was written for
                return drop(line, TOP_UNREAD)
            out.append(line[at:a])
            at = b
        out.append(line[at:])
        line = b"".join(out)
    return drop(line, TOP_UNREAD)


def loads_road_trimmed(raw):
    """A `road` line (bytes) parsed without the members the default stack does not read; the whole line when
    the trimmed one does not parse."""
    try:
        return json.loads(trim_road(raw))
    except ValueError:
        return json.loads(raw)
