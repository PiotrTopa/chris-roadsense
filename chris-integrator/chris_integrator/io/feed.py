"""The direct feed: the car backend speaks the vehicle input API (docs/VEHICLE_INPUT.md) to the integrator's
`feed.sock` (also `position.sock`, the name a car backend written for mapd knows) and to `POST /v2/vehicle`.

The store behind it is `io.vehicle_input.VehicleInput` (validation with mapd's error reasons, quality, the
description, the sensors, the `vehicle` topic). `Feed` is the short form for replays, tests and tools: messages in,
the stack's input messages out, errors kept in `last_error` / `errors`."""
import time

from .vehicle_input import TYPES, VehicleInput, hello as _hello

ACCEPTS = list(TYPES)


def hello(now_ns=None):
    return _hello(time.monotonic_ns() if now_ns is None else now_ns)


class Feed:
    """Feeder messages -> integrator input messages (`ego`, `gnss`, `vehicle`, `objects`, `reset`).

    cfg: io.vehicle_input.DEFAULTS keys. A replay on its own clock passes `now_ns` with every message (the stamp
    checks are made against it)."""

    def __init__(self, cfg=None, calibration=None):
        self.vi = VehicleInput(cfg, calibration)
        self.errors = []                 # (message type, reason) of the last rejected messages, at most 20

    @property
    def last_error(self):
        return self.vi.last_error

    def convert(self, m, now_ns=None):
        """List of input messages for the integrator (empty for a rejected or a store-only message)."""
        now_ns = time.monotonic_ns() if now_ns is None else now_ns
        outs, err = self.vi.on_message(m, now_ns)
        if err is not None:
            self.errors = (self.errors + [((m or {}).get("type") if isinstance(m, dict) else None, err)])[-20:]
        return outs

    def check(self, m, now_ns=None):
        """(input messages, None) or ([], reason): the reply a feeder gets."""
        return self.vi.on_message(m, time.monotonic_ns() if now_ns is None else now_ns)
