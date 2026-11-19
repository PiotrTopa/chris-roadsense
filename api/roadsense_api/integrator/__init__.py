"""The data integrator (docs/INTEGRATOR.md): every input enters here; the logic reads what comes out."""
from .contract import LocalEgo, inputs_health, signal, usable, validity
from .core import DEFAULT_CFG, Integrator, Measurement, SourceSpec

__all__ = ["Integrator", "Measurement", "SourceSpec", "DEFAULT_CFG", "LocalEgo", "validity", "usable", "signal",
           "inputs_health"]
