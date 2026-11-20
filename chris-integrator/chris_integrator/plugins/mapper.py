"""The map box's plug-in face. The box itself is the separate component chris-mapper (package `chris_mapper`):
in = the estimator's `ego_state` (pose with uncertainty, time), out = `map` (the map match, which returns to
the estimator as a measurement, and the map context: road attributes, limits, junctions, look-ahead,
`junction_view`, lanes). It estimates no position.

In process the box is wrapped here; out of process it speaks the plug-in wire contract itself
(`python3 -m chris_mapper --plugin`, `mixer.remote.RemotePlugin`). Without chris-mapper or without a pack the
stack runs without map streams and says so.

The feeder's `reset` (stream `position_reset`) drops the box's state (match, matcher, position state): the map is
`no_source` until the estimator has a position again; `map.input.resets` / `last_reset` say so. An out-of-process
box does not declare that input: it sees the estimator's position go and its new frame id, and drops its match
from those (state `unknown` instead of `no_source`)."""
import logging

from ..mixer import In, Out, Plugin

log = logging.getLogger("chris_integrator.mapper")


class Mapper(Plugin):
    name = "mapper"
    inputs = (In("ego_state", trigger=True), In("position_reset", trigger=True, optional=True))
    outputs = (Out("map", "context", "periodic", doc="map match (a measurement for the estimator) and map context"),)

    def __init__(self, box, name=None):
        super().__init__(None, name)
        self.box = box
        self.resets, self.last_reset = 0, None

    def on_input(self, stream, msg, ctx):
        if stream == "position_reset":
            self.box.reset(msg.get("reason"))
            self.resets += 1
            self.last_reset = {"ts_mono_ns": msg.get("ts_mono_ns"), "reason": msg.get("reason")}
            es = ctx.get("ego_state")              # the estimator's state after it forgot the position, if it sent one
            if not es or (es.get("position") or {}).get("valid"):
                return
            msg = es
        out = self.box.on_ego_state(msg)
        if out is not None:
            if isinstance(out.get("input"), dict):
                out["input"].update(resets=self.resets, last_reset=self.last_reset)
            ctx.emit("map", out)

    def reset(self, reason=None):
        self.box.reset(reason)

    def health(self):
        return self.box.health()

    def close(self):
        self.box.close()


def make_mapper(cfg=None):
    """The map box in process, or None when chris-mapper is not installed. cfg: mapper.json (`pack`: path of
    the map pack, default the installed one; `remote`: {"cmd": [...]} runs the box out of process instead)."""
    cfg = dict(cfg or {})
    if cfg.get("remote"):
        from ..mixer.remote import RemotePlugin
        return RemotePlugin(name="mapper", **cfg["remote"])
    try:
        from chris_mapper.box import MapBox
    except ImportError:
        log.info("chris-mapper is not installed: no map box")
        return None
    return Mapper(MapBox(cfg))
