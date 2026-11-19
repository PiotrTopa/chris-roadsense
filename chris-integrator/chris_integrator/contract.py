"""How consumers read the integrator's outputs (`ego_state`, `tracks`, `lead`). These helpers are the whole
consumer-side dependency: no consumer reaches into the estimator."""


def usable(es, group):
    """May a product be built from this source group now? False for a faulted group; True when it is absent
    from the message (an integrator that does not know it)."""
    s = ((es or {}).get("sources") or {}).get(group)
    return s is None or s.get("state") != "fault"


def inputs_health(es, groups):
    src = (es or {}).get("sources") or {}
    return {g: (src.get(g) or {}).get("state", "absent") for g in groups}


def validity(es, depends=("vision",), need=("speed",)):
    """(valid, reason, inputs) of a product made from the source groups `depends` and the ego quantities `need`."""
    src = (es or {}).get("sources") or {}
    groups = list(depends) + [g for g in ("vehicle",) if g in src and g not in depends]
    inputs = inputs_health(es, groups)
    if es is None:
        return False, "integrator: no state", inputs
    for g in depends:
        s = src.get(g) or {}
        if s.get("state") == "fault":
            return False, "%s: %s" % (g, s.get("reason")), inputs
    for q in need:
        e = (es or {}).get(q) or {}
        if not e.get("valid"):
            return False, "ego %s unknown: %s" % (q, e.get("reason")), inputs
    return True, None, inputs


def signal(es, name, max_age_s):
    s = ((es or {}).get("signals") or {}).get(name)
    if not s or s.get("age_s") is None or s["age_s"] > max_age_s:
        return None
    return s.get("value")


def quantity(es, name):
    """(value, std) of an ego quantity when valid, else (None, None)."""
    q = (es or {}).get(name) or {}
    return (q.get("value"), q.get("std")) if q.get("valid") else (None, None)


class LocalEgo:
    """A consumer that is not handed an ego state builds its own integrator from the same messages (ego part
    only), so a tracker in a test or a tool runs the same code path as in the stack."""

    def __init__(self):
        self.integ = None

    def _get(self):
        if self.integ is None:
            from .core.integrator import Integrator
            self.integ = Integrator(objects=False)
        return self.integ

    def feed(self, msg):
        self._get().on_message(msg)

    def at(self, road):
        return self._get().on_road(road)

    def state(self, ts_mono_ns):
        return self.integ.state(ts_mono_ns) if self.integ is not None and isinstance(ts_mono_ns, int) else None

    def reset(self):
        self.integ = None
