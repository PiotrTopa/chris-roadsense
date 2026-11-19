"""The plug-in contract: a box that takes named streams from the bus and puts named streams back.

A plug-in declares

    name        unique in a mixer
    inputs      `In(stream, trigger=..., optional=..., feedback=..., same_frame=...)`
    outputs     `Out(stream, kind=..., time=..., public=..., doc=...)`
    DEFAULT_CFG its configuration with defaults (`<config dir>/<name>.json` overrides per key)

and implements `on_input(stream, msg, ctx)`: called once for every message of a *trigger* input, in the
mixer's order. Inside it reads other inputs with `ctx.get(stream)` (the newest message of that stream; with
`frame=True` only when it belongs to the same camera frame as the message being handled) and publishes with
`ctx.emit(stream, msg)`. It may touch nothing else: a stream it did not declare raises.

Time. `ctx.now` (s), `ctx.mono_ns`, `ctx.real_ns` are the bus clock (the host's clocks in a running stack,
the messages' time in a replay). A message a plug-in emits for a camera frame carries that frame's
`ts_mono_ns`; periodic outputs are paced on `ctx.now`.

Order. A plug-in runs after the producers of its inputs. An input marked `feedback` is exempt: it closes a
loop (the map match that returns to the estimator), is delivered when it is published, and is read as "the
newest there is".

Health. `health()` returns `{"state": "ok" | "idle" | "degraded" | "fault", "reason": ...}` plus anything
worth showing; the mixer adds what it measures itself (calls, errors, cost, age of the last output). An
exception in `on_input` is caught by the mixer, counted, and reported as the plug-in's fault for that
message: the other plug-ins and the fan-out go on.

Life cycle. `reset(reason)` on a new drive or a clock restart; `close()` at shutdown. `state_dump()` /
`state_load()` carry what must survive a restart (faults, calibration).

The same contract out of process: `mixer.remote.RemotePlugin` (JSON lines over a pipe or a socket; the
handshake line carries exactly these declarations). A plug-in is tested alone by `mixer.testing.Bench`."""


class In:
    __slots__ = ("name", "trigger", "optional", "feedback", "doc")

    def __init__(self, name, trigger=False, optional=False, feedback=False, doc=""):
        self.name, self.trigger, self.optional, self.feedback, self.doc = name, trigger, optional, feedback, doc

    def as_dict(self):
        return {"name": self.name, "trigger": self.trigger, "optional": self.optional, "feedback": self.feedback}


class Out:
    __slots__ = ("name", "kind", "time", "public", "doc")

    def __init__(self, name, kind="measurement", time="frame", public=True, doc=""):
        self.name, self.kind, self.time, self.public, self.doc = name, kind, time, public, doc

    def as_dict(self):
        return {"name": self.name, "kind": self.kind, "time": self.time, "public": self.public, "doc": self.doc}


class Plugin:
    name = None
    inputs = ()
    outputs = ()
    DEFAULT_CFG = {}

    def __init__(self, cfg=None, name=None):
        if name is not None:
            self.name = name
        self.cfg = dict(self.DEFAULT_CFG)
        self.configure(cfg)

    def configure(self, cfg):
        """Merge overrides into the configuration (a dict value overrides the keys it names)."""
        for k, v in (cfg or {}).items():
            self.cfg[k] = dict(self.cfg[k], **v) if isinstance(self.cfg.get(k), dict) and isinstance(v, dict) else v

    def start(self, ctx):
        """Called once when the plug-in is added to a mixer."""

    def on_input(self, stream, msg, ctx):
        raise NotImplementedError

    def health(self):
        return {"state": "ok"}

    def reset(self, reason=None):
        pass

    def close(self):
        pass

    def state_dump(self):
        return None

    def state_load(self, state):
        pass
