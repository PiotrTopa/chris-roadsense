"""Road points into one camera's picture, from that camera's `calib` message and intrinsics.

Model outputs are in the (calibrated) device frame the road model passes through: x forward, y right, z down,
origin at the camera.
  p_dev   = R(roll, pitch, yaw) p_calib        (rot_from_euler: Rz(yaw) Ry(pitch) Rx(roll))
  p_cam   = [[0,1,0],[0,0,1],[1,0,0]] p_dev    (camera: x right, y down, z forward)
  u, v    = f X / Z + cx, f Y / Z + cy, scaled from the sensor size to the picture size."""
import math

import numpy as np

VIEW_FROM_DEVICE = np.array([[0.0, 1.0, 0.0], [0.0, 0.0, 1.0], [1.0, 0.0, 0.0]])
MIN_Z = 1.0


def rot_from_euler(roll, pitch, yaw):
    cr, sr, cp, sp, cy, sy = (math.cos(roll), math.sin(roll), math.cos(pitch),
                              math.sin(pitch), math.cos(yaw), math.sin(yaw))
    rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    return rz @ ry @ rx


class Projector:
    def __init__(self, intrinsics, calib, img_size):
        f, cx, cy, w, h = intrinsics
        s = img_size[0] / float(w)
        self.f, self.cx, self.cy, self.s = f * s, cx * s, cy * s, s
        calib = calib or {}
        self.height = float(calib.get("height_m") or 1.22)
        self.m = VIEW_FROM_DEVICE @ rot_from_euler(float(calib.get("roll") or 0.0),
                                                   float(calib.get("pitch") or 0.0),
                                                   float(calib.get("yaw") or 0.0))

    def project(self, x, y, z):
        """Arrays of device-frame points -> list of (u,v) for points in front of the camera."""
        p = self.m @ np.vstack([np.asarray(x, float), np.asarray(y, float), np.asarray(z, float)])
        ok = p[2] > MIN_Z
        u = self.f * p[0, ok] / p[2, ok] + self.cx
        v = self.f * p[1, ok] / p[2, ok] + self.cy
        return list(zip(u.tolist(), v.tolist()))
