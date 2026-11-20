"""A further camera as an object source: `objects` messages of kind "camera" (chris-roadsense docs/VEHICLE_API.md 5.7; what a
second roadsense instance's detector, or any camera with its own object list, publishes): positions in the
sensor's frame (x along its optical axis, y left), optionally class and probability.

A camera measures the bearing well and the range poorly: the noise is given along and across the line of
sight (range sigma grows with the range) and turned into the sensor's axes per detection. No velocity is
measured; the layer's filter derives it. Mount anywhere on the car, pointing anywhere (a wide front camera,
a rear camera): `{"kind": "camera_objects", "id": "cam_wide", "mount": {"x_m": .., "y_m": .., "yaw_deg": ..}}`.
"""
import math

import numpy as np

from ..core.ego import AuxState
from ..core.objects import X, Y, Det, Scan, SensorModel
from ..core.registry import NS, Mount, Source, fin
from . import KINDS


class CameraObjectsModel(SensorModel):
    name = "camera"
    initiates = True
    p_d = 0.85
    clutter = 0.3 / (70.0 * 70.0)            # false detections per scan over the field of view (m^2)

    def __init__(self, mount, yaw, fov, max_range):
        self.mx, self.my, self.yaw, self.fov, self.max_range = mount.x, mount.y, yaw, fov, max_range
        c, s = math.cos(yaw), math.sin(yaw)
        self.Rt = np.array([[c, s], [-s, c]])              # car axes -> sensor axes
        self.H = np.zeros((2, 5))
        self.H[:, [X, Y]] = self.Rt

    def h(self, x, ego):
        return self.Rt @ np.array([x[X] - self.mx, x[Y] - self.my]), self.H

    def initiate(self, det, ego):
        p = self.Rt.T @ det.z + np.array([self.mx, self.my])
        Pp = self.Rt.T @ det.R @ self.Rt
        P0 = np.zeros((5, 5))
        P0[:2, :2] = Pp
        P0[2, 2], P0[3, 3], P0[4, 4] = 100.0, 9.0, 4.0      # a camera says nothing about the speed
        return np.array([p[0], p[1], ego[0], 0.0, 0.0]), P0

    def coverage(self, x):
        dx, dy = x[X] - self.mx, x[Y] - self.my
        r = math.hypot(dx, dy)
        a = (math.atan2(dy, dx) - self.yaw + math.pi) % (2.0 * math.pi) - math.pi
        return self.p_d if 1.0 < r < self.max_range and abs(a) < self.fov else 0.0


class CameraObjectsSource(Source):
    rank = 1
    types = ("objects",)
    stale_s = 0.6
    info = "a further camera's object list"
    fov = math.radians(60.0)
    max_range = 70.0
    range_sigma = (0.4, 0.07)        # m + share of the range, along the line of sight
    cross_sigma = (0.12, 0.008)      # m + share of the range, across it

    def __init__(self, sensor, mount=None, **kw):
        self.id = sensor
        self.sensor = sensor
        self.mount = mount or Mount()
        self.aux = (AuxState("tau", 0.1, 0.05, 1e-9, lo=0.0, hi=0.4, unit="s", doc="a frame's detections are this old at their stamp"),)
        super().__init__(**kw)

    def scan(self, msg, ctx):
        if msg.get("sensor") != self.sensor or msg.get("kind") != "camera" or not isinstance(msg.get("objects"), list):
            return None
        dets = []
        for o in msg["objects"]:
            if not isinstance(o, dict) or not fin(o.get("x_m")) or not fin(o.get("y_m")):
                continue
            x, y = float(o["x_m"]), float(o["y_m"])
            r = math.hypot(x, y)
            if r < 1.0 or r > self.max_range:
                continue
            sr = self.range_sigma[0] + self.range_sigma[1] * r
            sc = self.cross_sigma[0] + self.cross_sigma[1] * r
            c, s = x / r, y / r
            U = np.array([[c, -s], [s, c]])
            R = U @ np.diag([sr * sr, sc * sc]) @ U.T
            dets.append(Det([x, y], R, {k: o[k] for k in ("class", "prob", "id") if k in o}))
        ctx.mem["n"] = ctx.mem.get("n", 0) + 1
        model = CameraObjectsModel(self.mount, self.mount.yaw, self.fov, self.max_range)
        ts = msg["ts_mono_ns"]
        return Scan(self.id, self.group, ts / NS - ctx.val("tau"), dets, model, ts)

    def report(self, ctx):
        return {"scans": ctx.mem.get("n", 0)}


def _factory(spec):
    m = spec.get("mount") or {}
    src = CameraObjectsSource(spec["id"], Mount(m.get("x_m", 0.0), m.get("y_m", 0.0), m.get("z_m", 0.0),
                                                math.radians(m.get("yaw_deg", 0.0))))
    for k in ("fov", "max_range", "range_sigma", "cross_sigma"):
        if k in spec:
            setattr(src, k, spec[k])
    return src


KINDS["camera_objects"] = _factory
