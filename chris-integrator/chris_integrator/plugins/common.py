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
    is an input (stream `governor`); the floors are this API's promise and are configured here."""
    FLOORS = {"road": {"hz": 10.0, "latency_ms": 250.0}, "lead": {"hz": 10.0, "latency_ms": 250.0},
              "lanes": {"hz": 10.0, "latency_ms": 250.0}, "lights": {"hz": 3.0, "latency_ms": 400.0},
              "signs": {"hz": 3.0, "latency_ms": 400.0}}
    TOPIC_CAPS = {"road": ("road_hz",), "lead": ("road_hz",), "lanes": ("road_hz",), "lights": ("lights_hz",),
                  "signs": ("signs_hz",), "env": ("env_hz",), "view": ("view_hz",)}

    def __init__(self, mixer, floors=None):
        self.mixer = mixer
        self.floors = dict(self.FLOORS, **(floors or {}))

    def governor(self):
        return self.mixer.bus.get("governor") or {}

    def caps(self):
        return self.governor().get("caps") or {}

    def status(self, topic, msg=None, latency_ms=None, hz=None):
        if latency_ms is None and msg is not None:
            ts = msg.get("ts_real_ns")
            latency_ms = (self.mixer.bus.clock.real_ns() - ts) / 1e6 if isinstance(ts, int) and ts > 0 else None
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
        if hz is not None and fl.get("hz") and hz < fl["hz"]:
            why.append("slow")
        if latency_ms is not None and fl.get("latency_ms") and latency_ms > fl["latency_ms"]:
            why.append("late")
        return {"hz": None if hz is None else round(hz, 1),
                "latency_ms": None if latency_ms is None else round(latency_ms, 1),
                "degraded": any(w != "limited" for w in why), "why": why, "level": g.get("level", 0)}


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
