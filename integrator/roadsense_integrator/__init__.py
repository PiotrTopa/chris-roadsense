"""roadsense data integrator: every input enters here; logic reads only its outputs (docs/INTEGRATOR.md)."""
from .contract import LocalEgo, inputs_health, quantity, signal, usable, validity
from .core.integrator import API, DEFAULT_CFG, Integrator
from .core.registry import Meas, Mount, Source
from .core.ego import AuxState

__version__ = "0.1.0"
__all__ = ["API", "DEFAULT_CFG", "Integrator", "Source", "Meas", "Mount", "AuxState", "LocalEgo", "usable",
           "validity", "signal", "inputs_health", "quantity"]
