"""The api's door to the data integrator. The integrator is its own component (`chris-integrator/`, package
`chris_integrator`, docs/INTEGRATOR.md); nothing of it lives in the api. This module only finds the package
(installed next to the api, or in the source tree) and names what the api's trackers use."""
import os
import sys

try:
    import chris_integrator as _ri
except ImportError:                      # the source tree: <repo>/integrator next to <repo>/api
    sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(
        os.path.abspath(__file__))))), "chris-integrator"))
    import chris_integrator as _ri

from chris_integrator import (API, DEFAULT_CFG, Integrator, LocalEgo, inputs_health, quantity, signal,  # noqa: E402,F401
                                  usable, validity)
