"""A plug-in alone on a bench: feed it messages of its inputs, collect what it emits.

    b = Bench(MyPlugin(cfg))
    b.put("ego_state", es)                 # a non-trigger input: just placed on the bus
    out = b.feed("road", road)             # a trigger: runs the plug-in, returns [(stream, msg), ...]

Every input the plug-in declares is provided by the bench; nothing else of a stack is needed."""
from .bus import MessageClock, StreamSpec
from .mixer import Mixer


class Bench:
    def __init__(self, plugin, clock=None):
        self.mixer = Mixer(clock or MessageClock())
        self.plugin = plugin
        own = {o.name for o in plugin.outputs}
        self.ch = self.mixer.add_input("bench", [StreamSpec(i.name, "raw", "sample") for i in plugin.inputs
                                                 if i.name not in own])
        self.out = []
        self.mixer.add(plugin)
        self.mixer.add_output("bench", lambda e: self.out.append((e.stream, e.msg)) if e.source != "bench" else None,
                              streams=own)

    def feed(self, stream, msg):
        n = len(self.out)
        self.ch.deliver(stream, msg)
        return self.out[n:]

    put = feed

    def health(self):
        return next(p for p in self.mixer.describe()["plugins"] if p["name"] == self.plugin.name)
