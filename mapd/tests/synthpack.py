"""The hand-made test network lives with the map box (chris-mapper's `chris_mapper/synthpack.py`); this name stays
for the tests of the legacy daemon."""
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))), "chris-mapper"))

from chris_mapper import synthpack as _m  # noqa: E402

globals().update({k: v for k, v in vars(_m).items() if not k.startswith("__")})
