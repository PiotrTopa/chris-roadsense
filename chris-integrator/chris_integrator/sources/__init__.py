"""This stack's sources. Each file is one sensor: input description, measurement model, mount / calibration
states. The core imports none of them; `default_sources()` is the stack's default registration."""


def default_sources(cfg=None):
    from .camera_lead import CameraLeadSource
    from .gnss import GnssSource, GnssSpeedSource
    from .mapmatch import MapMatchSource
    from .vehicle import VehicleSignals, VehicleSource, WheelsSource
    from .vision import VisionSource
    cfg = cfg or {}
    out = [VehicleSource(), VehicleSignals(), WheelsSource(), VisionSource(), GnssSource(), GnssSpeedSource(),
           MapMatchSource(), CameraLeadSource(cfg.get("lead"))]
    for spec in cfg.get("sources") or ():
        out.append(build(spec))
    return out


KINDS = {}       # kind name -> factory(spec dict) -> Source; a source file registers itself here


def build(spec):
    """A source from its description in the configuration: {"kind": ..., "id": ..., "mount": {...}, ...}."""
    import importlib
    kind = spec["kind"]
    if kind not in KINDS:
        importlib.import_module("." + kind, __name__)      # sources/<kind>.py registers its factory
    return KINDS[kind](spec)
