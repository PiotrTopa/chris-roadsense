"""Helpers shared by the stack's plug-ins."""
import collections

from ..native import round                  # builtins.round, native where built

DEFAULT_INTRINSICS = (1500.0, 960.0, 540.0, 1920, 1080)


def intrinsics(calib):
    """(focal_px, cx, cy, width, height) of a camera from its `calib` message (roadsense publishes the camera's
    intrinsics there); nominal values before the first one."""
    c = calib or {}
    w = c.get("width") or DEFAULT_INTRINSICS[3]
    h = c.get("height") or DEFAULT_INTRINSICS[4]
    f = c.get("focal_px") or DEFAULT_INTRINSICS[0]
    return float(f), float(c.get("cx") or w / 2.0), float(c.get("cy") or h / 2.0), int(w), int(h)


class Rates:
    """The marker a measurement topic carries (`rate`): measured rate and latency, and whether the consumer is
    getting less than the configured floor, with the reasons: `shed` (a roadsense instance's load governor has
    capped the stage), `limited` (the platform limiter is cutting clocks), `slow`, `late`. The governor's state
    is an input (stream `governor`); the floors are this API's promise and are configured here.

    A provider that thins itself out on purpose says so: a `lights` message with `idle: {"hz": H, ...}` comes from a
    light detector that tracks nothing and runs a sparser pattern of H passes a second until something shows up.
    While that holds (and for one rate window after the last such message, while the window still holds the idle
    arrivals), the topic's floor is IDLE_TOLERANCE x H instead of the configured one, and the rate is measured
    over the idle arrivals without the bias of a short window (first to last arrival; an arrival overdue by more
    than two announced periods counts up to now). So a stream that keeps the pattern it announces is not `slow`,
    and one that falls behind that pattern is."""
    IDLE_SOURCE = {"lights": "lights_raw"}    # topic -> the provider's stream whose messages may announce idling
    IDLE_TOLERANCE = 0.8                      # share of the announced idle rate a stream must keep
    RATE_WINDOW_S = 2.0
    FLOORS = {"road": {"hz": 10.0, "latency_ms": 250.0}, "lead": {"hz": 10.0, "latency_ms": 250.0},
              "lanes": {"hz": 10.0, "latency_ms": 250.0}, "lights": {"hz": 3.0, "latency_ms": 400.0},
              "signs": {"hz": 3.0, "latency_ms": 400.0}}
    TOPIC_CAPS = {"road": ("road_hz",), "lead": ("road_hz",), "lanes": ("road_hz",), "lights": ("lights_hz",),
                  "signs": ("signs_hz",), "env": ("env_hz",), "view": ("view_hz",)}

    def __init__(self, mixer, floors=None):
        self.mixer = mixer
        self.floors = dict(self.FLOORS, **(floors or {}))
        self._idle = {}                       # topic -> (bus time of the last idle announcement, its hz)

    def governor(self):
        return self.mixer.bus.get("governor") or {}

    def caps(self):
        return self.governor().get("caps") or {}

    def status(self, topic, msg=None, latency_ms=None, hz=None):
        if latency_ms is None and msg is not None:
            ts = msg.get("ts_real_ns")
            latency_ms = (self.mixer.bus.clock.real_ns() - ts) / 1e6 if isinstance(ts, int) and ts > 0 else None
        idle_hz, floor_hz = self.idle(topic, msg), None
        if idle_hz is not None:
            floor_hz = self.IDLE_TOLERANCE * idle_hz
            if hz is None:
                hz = self.idle_rate(topic, idle_hz)
        if hz is None:
            hz = self.mixer.fps(topic) or None
        g = self.governor()
        caps = g.get("caps") or {}
        why = []
        if any(caps.get(k, -1) >= 0 for k in self.TOPIC_CAPS.get(topic, ())):
            why.append("shed")
        if g.get("limited"):
            why.append("limited")
        fl = self.floors.get(topic) or {}
        if fl.get("hz") and floor_hz is not None:
            floor_hz = min(fl["hz"], floor_hz)
        elif fl.get("hz"):
            floor_hz = fl["hz"]
        if hz is not None and floor_hz and hz < floor_hz:
            why.append("slow")
        if latency_ms is not None and fl.get("latency_ms") and latency_ms > fl["latency_ms"]:
            why.append("late")
        out = {"hz": None if hz is None else round(hz, 1),
               "latency_ms": None if latency_ms is None else round(latency_ms, 1),
               "degraded": any(w != "limited" for w in why), "why": why, "level": g.get("level", 0)}
        if topic in self.IDLE_SOURCE:
            out["idle_hz"] = None if idle_hz is None else round(idle_hz, 2)
        return out

    def idle(self, topic, msg=None):
        """The idle rate a topic's provider announces (in this message, the provider's newest one, or one within
        the last rate window), else None."""
        src = self.IDLE_SOURCE.get(topic)
        if src is None:
            return None
        m = msg if msg is not None else self.mixer.bus.get(src)
        now = self.mixer.bus.clock.now()
        idle = (m or {}).get("idle")
        hz = idle.get("hz") if isinstance(idle, dict) else None
        if isinstance(hz, (int, float)) and not isinstance(hz, bool) and hz > 0:
            self._idle[topic] = (now, float(hz))
            return float(hz)
        seen = self._idle.get(topic)
        if seen is not None and now - seen[0] <= self.RATE_WINDOW_S:
            return seen[1]                    # the window still holds idle arrivals
        return None

    def idle_rate(self, topic, idle_hz):
        """Messages a second of the provider's stream over max(2 s, 4 announced periods): (n - 1) / (last - first),
        or up to now when the newest arrival is more than two announced periods old (a stall is not hidden)."""
        q = self.mixer.arrivals.get(self.IDLE_SOURCE.get(topic, topic)) or ()
        now = self.mixer.bus.clock.now()
        window = max(self.RATE_WINDOW_S, 4.0 / idle_hz)
        ts = [t for t in q if now - t <= window]
        if len(ts) < 2:
            return 0.0
        end = now if now - ts[-1] > 2.0 / idle_hz else ts[-1]
        span = end - ts[0]
        return (len(ts) - 1) / span if span > 0 else 0.0


class Latest(collections.abc.Mapping):
    """A read-only dict view of declared streams for logic written against a `latest` dict."""

    def __init__(self, ctx, names, unwrap=None):
        self.ctx, self.names, self.unwrap = ctx, tuple(names), unwrap or {}

    def __getitem__(self, k):
        if k not in self.names:
            raise KeyError(k)
        m = self.ctx.get(k)
        if m is None:
            raise KeyError(k)
        return m.get(self.unwrap[k]) if k in self.unwrap else m

    def __iter__(self):
        return iter(self.names)

    def __len__(self):
        return len(self.names)
