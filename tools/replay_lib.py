"""The replay library lives in chris-integrator (`chris_integrator.replay`): the consolidated side is that
component's. This module keeps the old import path of the tools working."""
import os
import sys

_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(_ROOT, "chris-integrator"))
sys.path.insert(0, os.path.join(_ROOT, "chris-mapper"))

from chris_integrator.replay import *  # noqa: E402,F401,F403
from chris_integrator.replay import NS, PASS, RECORDED, Pipeline, load_pos, read_lines, read_session  # noqa: E402,F401
