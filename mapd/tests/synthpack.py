"""The hand-made test network lives with the map box (chris-mapper/tests/synthpack.py); this name stays for the
tests of the legacy daemon."""
import importlib.util
import os

_p = os.path.join(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))), "chris-mapper", "tests", "synthpack.py")
_spec = importlib.util.spec_from_file_location("chris_mapper_synthpack", _p)
_m = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(_m)
globals().update({k: v for k, v in vars(_m).items() if not k.startswith("__")})
