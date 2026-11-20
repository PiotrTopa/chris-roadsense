"""chris-mapper: the map box of the chris stack. In: a pose with uncertainty (`ego_state`). Out: `map` (the
map match and the map context). It estimates no position. `box.MapBox` is the box; `rsmap` the pack format;
`matcher` the map matching; `context` the road attributes, limits, junctions and look-ahead."""
from . import context, geo, matcher, rsmap, sim  # noqa: F401

__version__ = "0.2.0"
API_VERSION = 1      # the major version of the `map` message
