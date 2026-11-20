"""The mixer: wires input channels, plug-ins and output channels to one bus and runs them in order.

    m = Mixer()
    cam = m.add_input("roadsense", [StreamSpec("road", "raw", "frame"), ...])     # an input channel
    m.add(Estimator(cfg)); m.add(Lanes())                                        # plug-in transformers
    m.add_output("ws", fan_out)                                                  # an output channel: fn(envelope)
    cam.deliver("road", msg)                                                      # a message enters the bus

Dispatch is synchronous and depth-first: a message put on the bus goes to the output channels, then to every
plug-in that declared the stream as a trigger, in plug-in order; what a plug-in emits is dispatched before
the next plug-in runs. So when a later plug-in handles a camera frame, the outputs of the earlier ones for
that frame are already on the bus. The order is derived from the declarations (a producer runs before its
consumers; `feedback` inputs are exempt) and is stable in the order the plug-ins were added."""
import collections
import logging
import time

from .bus import Bus, StreamSpec

log = logging.getLogger("chris_integrator.mixer")


class MixerError(Exception):
    pass


class InputChannel:
    """Handle of an input channel: `deliver(stream, msg, line=None)` puts a message on the bus."""

    def __init__(self, mixer, name, streams, info=None):
        self.mixer, self.name, self.streams, self.info = mixer, name, tuple(s.name for s in streams), info
        self.connected = None              # None: not a connection-oriented channel
        self.last = None                   # bus time of its last message
        self.n = 0

    def deliver(self, stream, msg, line=None):
        if stream not in self.streams:
            raise MixerError("channel %s did not declare stream %s" % (self.name, stream))
        m = self.mixer
        m.bus.clock.on_message(msg)
        self.last = m.bus.clock.now()
        self.n += 1
        m.publish(stream, msg, self.name, line)

    def status(self, now):
        return {"streams": list(self.streams), "connected": self.connected, "messages": self.n,
                "last_msg_age_s": None if self.last is None else round(now - self.last, 3), "info": self.info}


class _Ctx:
    """What a plug-in sees of the mixer: its declared streams, the bus clock, shared services."""

    def __init__(self, mixer, slot):
        self._m, self._slot = mixer, slot
        self._env = None                   # the envelope being handled

    @property
    def now(self):
        return self._m.bus.clock.now()

    @property
    def mono_ns(self):
        return self._m.bus.clock.mono_ns()

    @property
    def real_ns(self):
        return self._m.bus.clock.real_ns()

    @property
    def replay(self):
        return self._m.bus.clock.replay

    def get(self, name, frame=False):
        """The newest message of a declared input (None if there is none). frame=True: only if it is stamped
        with the same time as the message being handled (it belongs to this camera frame)."""
        if name not in self._slot.reads:
            raise MixerError("plug-in %s reads %s without declaring it" % (self._slot.name, name))
        e = self._m.bus.latest.get(name)
        if e is None or (frame and (self._env is None or e.ts != self._env.ts)):
            return None
        return e.msg

    def emit(self, name, msg, line=None):
        if name not in self._slot.writes:
            raise MixerError("plug-in %s emits %s without declaring it" % (self._slot.name, name))
        s = self._slot
        s.out_n += 1
        s.out_t = self.now
        self._m.publish(name, msg, s.name, line)

    def service(self, name):
        return self._m.services.get(name)


class _Slot:
    def __init__(self, plugin, enabled):
        self.plugin, self.name, self.enabled = plugin, plugin.name, enabled
        self.reads = {i.name for i in plugin.inputs}
        self.writes = {o.name for o in plugin.outputs}
        self.calls = self.errors = self.out_n = 0
        self.cpu = 0.0
        self.max_ms = 0.0
        self.out_t = None
        self.last_error = None
        self.ctx = None
        self.busy = False
        self.pending = []                  # messages that arrived for it while it was running


class Mixer:
    def __init__(self, clock=None):
        self.bus = Bus(clock)
        self.channels = collections.OrderedDict()      # name -> InputChannel
        self.slots = collections.OrderedDict()         # name -> _Slot, in the order added
        self.order = []                                # enabled slots in running order
        self.outputs = collections.OrderedDict()       # name -> (fn, set of streams or None, info)
        self.services = {}
        self.arrivals = collections.defaultdict(lambda: collections.deque(maxlen=200))
        self._subs = {}                                # stream -> [slot, ...] in order
        self._taps = []
        self._tap_of = {}                              # stream -> the output channels' functions that take it
        self._depth = 0
        self._child = [0.0]

    # ---- wiring --------------------------------------------------------------------------------------
    def add_input(self, name, streams, info=None):
        if name in self.channels or name in self.slots:
            raise MixerError("name %s is taken" % name)
        specs = [s if isinstance(s, StreamSpec) else StreamSpec(*s) for s in streams]
        for s in specs:
            s.producer = name
            self.bus.declare(s)
        ch = self.channels[name] = InputChannel(self, name, specs, info)
        self._plan()
        return ch

    def remove_input(self, name):
        self.channels.pop(name)
        self.bus.retract(name)
        self._plan()

    def add_output(self, name, fn, streams=None, info=None):
        """An output channel: fn(envelope) for every message of a public stream (or of the listed streams)."""
        self.outputs[name] = (fn, None if streams is None else set(streams), info)
        self._taps = list(self.outputs.values())
        self._tap_of = {}

    def remove_output(self, name):
        self.outputs.pop(name, None)
        self._taps = list(self.outputs.values())
        self._tap_of = {}

    def add(self, plugin, enabled=True):
        if not plugin.name or plugin.name in self.slots or plugin.name in self.channels:
            raise MixerError("plug-in name %r is missing or taken" % (plugin.name,))
        slot = _Slot(plugin, enabled)
        slot.ctx = _Ctx(self, slot)
        self.slots[plugin.name] = slot
        try:
            if enabled:
                self._declare(slot)
            self._plan()
        except Exception:
            del self.slots[plugin.name]
            self.bus.retract(plugin.name)
            self._plan()
            raise
        plugin.start(slot.ctx)
        return plugin

    def _declare(self, slot):
        for o in slot.plugin.outputs:
            self.bus.declare(StreamSpec(o.name, o.kind, o.time, o.doc, o.public, slot.name))

    def remove(self, name):
        slot = self.slots.pop(name)
        self.bus.retract(name)
        self._plan()
        slot.plugin.close()
        return slot.plugin

    def replace(self, name, plugin):
        """Swap a plug-in for another at the same place (the new one may have another name)."""
        items = list(self.slots.items())
        k = [n for n, _ in items].index(name)
        old = items[k][1]
        self.bus.retract(name)
        slot = _Slot(plugin, old.enabled)
        slot.ctx = _Ctx(self, slot)
        items[k] = (plugin.name, slot)
        self.slots = collections.OrderedDict(items)
        if slot.enabled:
            self._declare(slot)
        self._plan()
        old.plugin.close()
        plugin.start(slot.ctx)
        return plugin

    def enable(self, name, on=True):
        slot = self.slots[name]
        if slot.enabled == bool(on):
            return
        slot.enabled = bool(on)
        if on:
            self._declare(slot)
        else:
            self.bus.retract(name)
        self._plan()

    def plugin(self, name):
        s = self.slots.get(name)
        return None if s is None else s.plugin

    def _plan(self):
        """Running order: producers before consumers (feedback inputs exempt), ties in the order added."""
        slots = [s for s in self.slots.values() if s.enabled]
        prod = {}
        for s in slots:
            for w in s.writes:
                prod[w] = s
        deps = {s.name: set() for s in slots}
        for s in slots:
            for i in s.plugin.inputs:
                p = prod.get(i.name)
                if p is not None and p is not s and not i.feedback:
                    deps[s.name].add(p.name)
        order, done = [], set()
        left = list(slots)
        while left:
            ready = [s for s in left if deps[s.name] <= done]
            if not ready:
                raise MixerError("plug-ins depend on each other in a loop: %s (mark one input `feedback`)"
                                 % ", ".join("%s<-%s" % (s.name, sorted(deps[s.name] - done)) for s in left))
            s = ready[0]
            order.append(s)
            done.add(s.name)
            left.remove(s)
        self.order = order
        self._tap_of = {}                  # a stream may have changed its producer or its `public`
        subs = {}
        for s in order:
            for i in s.plugin.inputs:
                if i.trigger:
                    subs.setdefault(i.name, []).append(s)
        self._subs = subs

    # ---- running -------------------------------------------------------------------------------------
    def publish(self, stream, msg, source, line=None):
        spec = self.bus.streams.get(stream)
        if spec is None:
            raise MixerError("stream %s was not declared (by %s)" % (stream, source))
        env = self.bus.put(stream, msg, source, line)
        self.arrivals[stream].append(env.t_arr)
        taps = self._tap_of.get(stream)
        if taps is None:                   # which output channels take this stream: worked out once per stream
            public = spec.public           # a public stream goes to every channel that did not narrow itself
            taps = self._tap_of[stream] = tuple(
                fn for fn, streams, _ in self._taps
                if (stream in streams if streams is not None else public))
        for fn in taps:
            try:
                fn(env)
            except Exception:
                log.exception("output channel failed on %s", stream)
        subs = self._subs.get(stream)
        if not subs:
            return env
        if self._depth > 24:
            raise MixerError("dispatch deeper than 24 on %s: a loop without `feedback`?" % stream)
        self._depth += 1
        try:
            for slot in subs:
                if slot.busy:              # a plug-in is never re-entered: what its own output caused to come
                    slot.pending.append(env)   # back (a feedback input) waits until it has finished
                    continue
                self._call(slot, env)
                while slot.pending:
                    self._call(slot, slot.pending.pop(0))
        finally:
            self._depth -= 1
        return env

    def _call(self, slot, env):
        ctx = slot.ctx
        prev, ctx._env = ctx._env, env
        slot.busy = True
        t0 = time.perf_counter()
        self._child.append(0.0)
        try:
            slot.plugin.on_input(env.stream, env.msg, ctx)
        except MixerError:
            raise
        except Exception as e:
            slot.errors += 1
            slot.last_error = "%s on %s: %s" % (type(e).__name__, env.stream, e)
            log.exception("plug-in %s failed on a %s message", slot.name, env.stream)
        finally:
            dt = time.perf_counter() - t0
            own = dt - self._child.pop()
            self._child[-1] += dt
            slot.calls += 1
            slot.cpu += own
            if own * 1e3 > slot.max_ms:
                slot.max_ms = own * 1e3
            ctx._env = prev
            slot.busy = False

    def fps(self, stream, window=2.0):
        q = self.arrivals.get(stream)
        if not q:
            return 0.0
        now = self.bus.clock.now()
        # arrivals are in time order (the bus clock never goes back): count from the newest until the window ends
        n, first = 0, None
        for t in reversed(q):
            if now - t > window:
                break
            n += 1
            first = t
        if n < 2:
            return 0.0
        span = now - first
        return round((n - 1) / span, 2) if span > 0 else 0.0

    def reset(self, reason=None):
        for s in self.slots.values():
            try:
                s.plugin.reset(reason)
            except Exception:
                log.exception("plug-in %s failed to reset", s.name)

    def close(self):
        for s in self.slots.values():
            try:
                s.plugin.close()
            except Exception:
                log.exception("plug-in %s failed to close", s.name)

    # ---- introspection -------------------------------------------------------------------------------
    def unsatisfied(self, slot):
        return [i.name for i in slot.plugin.inputs if not i.optional and i.name not in self.bus.streams]

    def describe(self):
        """The whole wiring and its health: channels, streams, plug-ins in running order, outputs."""
        now = self.bus.clock.now()
        plugins = []
        order = {s.name: k for k, s in enumerate(self.order)}
        for s in self.slots.values():
            try:
                h = dict(s.plugin.health() or {})
            except Exception as e:
                h = {"state": "fault", "reason": "health(): %s" % e}
            missing = self.unsatisfied(s) if s.enabled else []
            if not s.enabled:
                h = {"state": "disabled"}
            elif missing and h.get("state") == "ok":
                h.update(state="idle", reason="no producer for %s" % ", ".join(missing))
            plugins.append({
                "name": s.name, "enabled": s.enabled, "order": order.get(s.name),
                "inputs": [i.as_dict() for i in s.plugin.inputs], "outputs": [o.name for o in s.plugin.outputs],
                "health": h, "calls": s.calls, "errors": s.errors, "last_error": s.last_error,
                "ms_per_call": round(1e3 * s.cpu / s.calls, 4) if s.calls else None, "max_ms": round(s.max_ms, 3),
                "cpu_s": round(s.cpu, 3), "outputs_n": s.out_n,
                "last_output_age_s": None if s.out_t is None else round(now - s.out_t, 3),
                "remote": bool(getattr(s.plugin, "remote", False))})
        streams = {}
        for n, sp in self.bus.streams.items():
            d = sp.as_dict()
            d["consumers"] = [s.name for s in self.order if n in s.reads]
            d["messages"] = self.bus.count.get(n, 0)
            d["hz"] = self.fps(n)
            e = self.bus.latest.get(n)
            d["age_s"] = None if e is None else round(now - e.t_arr, 3)
            streams[n] = d
        return {"inputs": {n: c.status(now) for n, c in self.channels.items()},
                "streams": streams, "plugins": plugins,
                "outputs": {n: {"streams": None if st is None else sorted(st), "info": info}
                            for n, (_, st, info) in self.outputs.items()}}
