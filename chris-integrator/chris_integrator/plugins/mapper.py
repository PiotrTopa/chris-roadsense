"""The map box's plug-in face. The box itself is the separate component chris-mapper (package `chris_mapper`):
in = the estimator's `ego_state` (pose with uncertainty, time), out = `map` (the map match, which returns to
the estimator as a measurement, and the map context: road attributes, limits, junctions, look-ahead,
`junction_view`, lanes). It estimates no position.

In process the box is wrapped here; out of process it speaks the plug-in wire contract itself
(`python3 -m chris_mapper --plugin`, `mixer.remote.RemotePlugin`). Without chris-mapper or without a pack the
stack runs without map streams and says so."""
import logging

from ..mixer import In, Out, Plugin

log = logging.getLogger("chris_integrator.mapper")


class Mapper(Plugin):
    name = "mapper"
    inputs = (In("ego_state", trigger=True),)
    outputs = (Out("map", "context", "periodic", doc="map match (a measurement for the estimator) and map context"),)

    def __init__(self, box, name=None):
        super().__init__(None, name)
        self.box = box

    def on_input(self, stream, msg, ctx):
        out = self.box.on_ego_state(msg)
        if out is not None:
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
