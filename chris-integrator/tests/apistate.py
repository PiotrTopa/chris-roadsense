"""Test helper: the default stack with the surface the tests of the consolidated side were written against
(feed a message, read the newest per topic, the health summary, the live sign tracks)."""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from chris_integrator.io.api import Api  # noqa: E402
from chris_integrator.stack import build  # noqa: E402


class State:
    def __init__(self, rundir=None, **kw):
        self.stack = build(external_map=True, mapper=None, **kw)
        self.mixer = self.stack.mixer
        self.api = Api(self.stack)
        self.subscribers = self.api.subscribers
        self.connected = {}

    def on_message(self, daemon, msg, line=None):
        self.stack.deliver(msg)

    @property
    def latest(self):
        out = {n: e.msg for n, e in self.mixer.bus.latest.items()}
        if "egolane" in out:
            out["egolane"] = out["egolane"].get("est")
        return out

    def health(self):
        return self.api.health()

    def tracks(self, suppressed=False):
        return self.mixer.plugin("sign_context").tracks(suppressed)

    speed = property(lambda self: self.mixer.plugin("rules").speed)
    live_tracks = property(lambda self: self.mixer.plugin("sign_context").live)
    ride_changes = property(lambda self: self.mixer.plugin("ride_watch").changes)
    view = property(lambda self: self.mixer.plugin("view").builder)
    lanes = property(lambda self: self.mixer.plugin("lanes").tracker)
