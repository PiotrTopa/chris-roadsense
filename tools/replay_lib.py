"""The replay library lives in chris-integrator (`chris_integrator.replay`): the consolidated side is that
component's. This module keeps the old import path of the tools working."""
import os
import sys

_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(_ROOT, "chris-integrator"))
sys.path.insert(0, os.path.join(_ROOT, "chris-mapper"))

from chris_integrator.replay import *  # noqa: E402,F401,F403
from chris_integrator.replay import NS, PASS, RECORDED, Pipeline, read_lines, read_session  # noqa: E402,F401
from chris_integrator.replay import load_pos as _load_pos  # noqa: E402

TRIP_TZ = "Europe/Warsaw"        # the car computer's trip logs are named in local time


def load_pos(path, tz=TRIP_TZ):
    """chris_integrator.replay.load_pos with the reference car's time zone (the trip-log names)."""
    return _load_pos(path, tz=tz)
