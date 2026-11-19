"""roadsense mapd, the legacy position daemon: its own position filter plus the vehicle input API, around the
map matcher. Kept for hosts whose car backend still feeds `position.sock`; in the chris stack the position is
estimated once, by chris-integrator, and the map is the box chris-mapper (which this package now only uses:
`geo`, `rsmap`, `matcher`, `context`, `sim` live there and are aliased here)."""
import os
import sys

API_VERSION = 1      # the major version of the vehicle input API: raised only by a change of meaning
API_MINOR = 1        # additions since 1.0 (position input API v1): controls, body, vehicle, objects, quality, age_ms

try:
    import chris_mapper as _cm
except ImportError:                      # the source tree: <repo>/chris-mapper next to <repo>/mapd
    sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))), "chris-mapper"))
    import chris_mapper as _cm
from chris_mapper import context, geo, matcher, rsmap, sim  # noqa: E402,F401

for _n in ("context", "geo", "matcher", "rsmap", "sim"):
    sys.modules[__name__ + "." + _n] = getattr(_cm, _n)
