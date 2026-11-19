"""The bus: named streams of time-stamped messages with metadata.

A message on the bus is an `Envelope`: the stream's name, the message (a JSON object as a dict), the time the
message is valid at (`ts_mono_ns` of the message: the stack's one time axis), the time it reached the bus
(the bus clock), who put it there, and its serialised line (made at most once, reused by every output
channel; an input channel that received a line hands it over, so a message that is only passed on is never
re-serialised).

Clocks. Plug-ins never read the wall clock. The bus clock is the only "now": `LiveClock` in a running stack,
`MessageClock` in a replay (it advances with the messages), so a replay of a recording is deterministic and
runs the same code as the car."""
import json
import time

KINDS = ("raw", "measurement", "estimate", "decision", "context", "display", "status")
TIMES = ("frame", "sample", "periodic", "event")


class StreamSpec:
    """What a stream is. `kind`: raw (a source's message as it came), measurement, estimate (fused state),
    decision (what a consumer acts on), context (map and surroundings), display (informational, may be smoothed),
    status (health). `time`: frame (one message per camera frame, stamped with the frame's time of validity),
    sample (the source's own samples), periodic, event (on change). `public`: served to consumers under its
    name; a stream that is not public is wiring between plug-ins."""
    __slots__ = ("name", "kind", "time", "doc", "public", "producer")

    def __init__(self, name, kind="measurement", time="frame", doc="", public=True, producer=None):
        if kind not in KINDS or time not in TIMES:
            raise ValueError("stream %s: kind %r / time %r" % (name, kind, time))
        self.name, self.kind, self.time, self.doc, self.public, self.producer = name, kind, time, doc, public, producer

    def as_dict(self):
        return {"kind": self.kind, "time": self.time, "public": self.public, "producer": self.producer, "doc": self.doc}


class Envelope:
    __slots__ = ("stream", "msg", "ts", "t_arr", "source", "seq", "_line")

    def __init__(self, stream, msg, t_arr, source, seq, line=None):
        self.stream, self.msg, self.t_arr, self.source, self.seq, self._line = stream, msg, t_arr, source, seq, line
        ts = msg.get("ts_mono_ns") if isinstance(msg, dict) else None
        self.ts = ts if isinstance(ts, int) else None

    @property
    def line(self):
        if self._line is None:
            self._line = json.dumps(self.msg, separators=(",", ":"))
        return self._line


class LiveClock:
    """The host's clocks."""
    replay = False

    def now(self):
        return time.monotonic()

    def mono_ns(self):
        return time.monotonic_ns()

    def real_ns(self):
        return time.time_ns()

    def on_message(self, msg):
        pass


class MessageClock:
    """A replay's clock: "now" is the newest time seen in the messages (their arrival is taken as their stamp,
    never earlier than the message before)."""
    replay = True

    def __init__(self):
        self.t_ns, self.r_ns = 0, 0

    def on_message(self, msg):
        ts, tr = msg.get("ts_mono_ns"), msg.get("ts_real_ns")
        if isinstance(ts, int) and ts > self.t_ns:
            if not isinstance(tr, int) and self.r_ns:
                self.r_ns += ts - self.t_ns
            self.t_ns = ts
        if isinstance(tr, int) and tr > self.r_ns:
            self.r_ns = tr

    def now(self):
        return self.t_ns / 1e9

    def mono_ns(self):
        return self.t_ns

    def real_ns(self):
        return self.r_ns


class Bus:
    def __init__(self, clock=None):
        self.clock = clock or LiveClock()
        self.streams = {}                  # name -> StreamSpec
        self.latest = {}                   # name -> Envelope
        self.count = {}                    # name -> messages so far
        self.seq = 0

    def declare(self, spec):
        old = self.streams.get(spec.name)
        if old is not None and old.producer != spec.producer:
            raise ValueError("stream %s has two producers: %s and %s" % (spec.name, old.producer, spec.producer))
        self.streams[spec.name] = spec
        return spec

    def retract(self, producer):
        for n in [n for n, s in self.streams.items() if s.producer == producer]:
            del self.streams[n]
            self.latest.pop(n, None)

    def get(self, name):
        e = self.latest.get(name)
        return None if e is None else e.msg

    def put(self, stream, msg, source, line=None):
        self.seq += 1
        e = Envelope(stream, msg, self.clock.now(), source, self.seq, line)
        self.latest[stream] = e
        self.count[stream] = self.count.get(stream, 0) + 1
        return e
