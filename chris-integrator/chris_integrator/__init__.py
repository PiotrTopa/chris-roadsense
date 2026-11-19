"""chris-integrator: every input enters here; decisions and the consolidated API come out (docs/DESIGN.md)."""
from .contract import LocalEgo, inputs_health, quantity, signal, usable, validity
from .core.integrator import API, DEFAULT_CFG, Integrator
from .core.registry import Meas, Mount, Source
from .core.ego import AuxState
from .core import classes as _classes
from .core import roadusers as _roadusers

_classes.register()      # the `object_classes` source kind (road-user classes as track metadata)
_roadusers.register()    # the `road_users` source kind (a camera's class detector, placed by its calibration)

__version__ = "0.2.0"
API_MAJOR = 2          # the consumer contract of the consolidated API (topics, fields, HTTP paths /v2)
__all__ = ["API", "DEFAULT_CFG", "Integrator", "Source", "Meas", "Mount", "AuxState", "LocalEgo", "usable",
           "validity", "signal", "inputs_health", "quantity"]
