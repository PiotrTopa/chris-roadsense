"""Camera geometry shared by the calibration tools (numpy only; runs wherever the api runs).

Frames and signs are roadsense's (docs/SUPERCOMBO_SPEC.md, api/roadsense_api/overlay.py):
  calibrated frame  x forward along the road, y right, z down, origin at the camera
  device frame      p_dev = R(roll, pitch, yaw) p_calib,  R = Rz(yaw) Ry(pitch) Rx(roll)
  view frame        x right, y down, z forward:  p_view = VIEW_FROM_DEVICE p_dev
  pixel             u = f x' + cx, v = f y' + cy with (x', y') = distort(X/Z, Y/Z)
Positive pitch = camera tilted down (the horizon is higher in the picture); positive yaw = camera turned left
(the road ahead is to the right in the picture).

Distortion is OpenCV's (Brown) model with k1, k2, p1, p2, k3:
  r2 = x^2 + y^2,  radial = 1 + k1 r2 + k2 r2^2 + k3 r2^3
  x' = x radial + 2 p1 x y + p2 (r2 + 2 x^2),   y' = y radial + p1 (r2 + 2 y^2) + 2 p2 x y
"""
import json
import math
import os

import numpy as np

VIEW_FROM_DEVICE = np.array([[0.0, 1.0, 0.0], [0.0, 0.0, 1.0], [1.0, 0.0, 0.0]])
DIST_KEYS = ("k1", "k2", "p1", "p2", "k3")

# The road models' input frames (docs/SUPERCOMBO_SPEC.md 2): name -> (focal, cx, cy, width, height) in model pixels.
MODEL_FRAMES = {"medmodel": (910.0, 256.0, 47.6, 512, 256), "sbigmodel": (455.0, 256.0, 151.8, 512, 256)}


def rot_from_euler(roll, pitch, yaw):
    cr, sr, cp, sp, cy, sy = (math.cos(roll), math.sin(roll), math.cos(pitch), math.sin(pitch), math.cos(yaw),
                              math.sin(yaw))
    rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    return rz @ ry @ rx


def euler_from_rot(r):
    """(roll, pitch, yaw) of R = Rz(yaw) Ry(pitch) Rx(roll)."""
    return (math.atan2(r[2, 1], r[2, 2]), -math.asin(max(-1.0, min(1.0, r[2, 0]))), math.atan2(r[1, 0], r[0, 0]))


def distort(x, y, dist=None):
    """Ideal normalised coordinates -> distorted normalised coordinates."""
    if not dist or not any(dist.get(k) for k in DIST_KEYS):
        return x, y
    k1, k2, p1, p2, k3 = (float(dist.get(k) or 0.0) for k in DIST_KEYS)
    r2 = x * x + y * y
    rad = 1.0 + r2 * (k1 + r2 * (k2 + r2 * k3))
    return (x * rad + 2 * p1 * x * y + p2 * (r2 + 2 * x * x), y * rad + p1 * (r2 + 2 * y * y) + 2 * p2 * x * y)


def undistort(xd, yd, dist=None, iters=20):
    """Distorted normalised coordinates -> ideal ones (fixed-point iteration, as OpenCV's undistortPoints)."""
    if not dist or not any(dist.get(k) for k in DIST_KEYS):
        return xd, yd
    k1, k2, p1, p2, k3 = (float(dist.get(k) or 0.0) for k in DIST_KEYS)
    x, y = np.array(xd, float), np.array(yd, float)
    for _ in range(iters):
        r2 = x * x + y * y
        rad = 1.0 + r2 * (k1 + r2 * (k2 + r2 * k3))
        dx = 2 * p1 * x * y + p2 * (r2 + 2 * x * x)
        dy = p1 * (r2 + 2 * y * y) + 2 * p2 * x * y
        x, y = (xd - dx) / rad, (yd - dy) / rad
    return x, y


class Camera:
    """Intrinsics + mount. `intr`: focal_px, cx, cy (+ k1..k3); `calib`: pitch, yaw, roll, height_m."""

    def __init__(self, intr, calib=None, use_distortion=True):
        calib = calib or {}
        self.f = float(intr["focal_px"])
        self.cx, self.cy = float(intr["cx"]), float(intr["cy"])
        self.dist = {k: float(intr.get(k) or 0.0) for k in DIST_KEYS} if use_distortion else None
        self.roll, self.pitch, self.yaw = (float(calib.get(k) or 0.0) for k in ("roll", "pitch", "yaw"))
        self.height = float(calib.get("height_m") or 1.22)
        self.m = VIEW_FROM_DEVICE @ rot_from_euler(self.roll, self.pitch, self.yaw)

    def project(self, pts, min_z=0.5):
        """(N, 3) calibrated-frame points -> (N, 2) pixels; NaN for points behind the camera."""
        p = np.atleast_2d(np.asarray(pts, float)) @ self.m.T
        ok = p[:, 2] > min_z
        z = np.where(ok, p[:, 2], 1.0)
        x, y = distort(p[:, 0] / z, p[:, 1] / z, self.dist)
        uv = np.stack([self.f * x + self.cx, self.f * y + self.cy], axis=1)
        uv[~ok] = np.nan
        return uv

    def ray(self, uv):
        """Pixels -> unit directions in the calibrated frame."""
        uv = np.atleast_2d(np.asarray(uv, float))
        x, y = undistort((uv[:, 0] - self.cx) / self.f, (uv[:, 1] - self.cy) / self.f, self.dist)
        d = np.stack([x, y, np.ones_like(x)], axis=1) @ self.m          # m^-1 = m^T, row vectors: d_calib = m^T d_view
        return d / np.linalg.norm(d, axis=1, keepdims=True)

    def vanishing_point(self):
        """Pixel where the road direction (calibrated x axis) lands."""
        return self.project([[1e6, 0.0, 0.0]])[0]


def angles_from_vanishing_point(u, v, intr):
    """(pitch, yaw) of a camera with zero roll whose road vanishing point is at pixel (u, v)."""
    f, cx, cy = float(intr["focal_px"]), float(intr["cx"]), float(intr["cy"])
    x, y = undistort(np.array([(u - cx) / f]), np.array([(v - cy) / f]), {k: intr.get(k) for k in DIST_KEYS})
    yaw = math.atan(float(x[0]))
    return -math.atan(float(y[0]) * math.cos(yaw)), yaw


def compose_residual(calib, d_pitch, d_yaw):
    """The calibration that removes a residual seen with `calib` in use: the road direction, expressed in the
    frame the model was given, pointed d_yaw to the right and d_pitch down, d_pitch = atan2(z, |x, y|) (openpilot calibrationd:
    rot(used) . rot(observed)). Roll is kept."""
    r = rot_from_euler(float(calib.get("roll") or 0.0), float(calib.get("pitch") or 0.0),
                       float(calib.get("yaw") or 0.0)) @ rot_from_euler(0.0, -d_pitch, d_yaw)
    _, pitch, yaw = euler_from_rot(r)
    return pitch, yaw


def model_region(frame, intr, calib=None, n=9):
    """Where a model input frame lands in the camera picture, with and without the lens distortion.

    Returns (model_px (N,2), camera_px_pinhole (N,2), camera_px_real (N,2)) on an n x n grid over the model frame.
    The warp is a pure rotation (the model frame looks along the calibrated x axis)."""
    mf, mcx, mcy, mw, mh = MODEL_FRAMES[frame]
    gx, gy = np.meshgrid(np.linspace(0, mw - 1, n), np.linspace(0, mh - 1, n))
    mp = np.stack([gx.ravel(), gy.ravel()], axis=1)
    # model pixel -> direction in the calibrated frame (model view: x right, y down, z forward = calib y, z, x)
    d = np.stack([np.ones(len(mp)), (mp[:, 0] - mcx) / mf, (mp[:, 1] - mcy) / mf], axis=1) * 1e3
    return mp, Camera(intr, calib, use_distortion=False).project(d), Camera(intr, calib).project(d)


# ---- files ---------------------------------------------------------------------------------------------------------

def load_json(path, default=None):
    try:
        with open(path) as f:
            return json.load(f)
    except FileNotFoundError:
        if default is not None:
            return default
        raise


def write_json_atomic(path, obj):
    tmp = "%s.tmp.%d" % (path, os.getpid())
    with open(tmp, "w") as f:
        json.dump(obj, f, indent=1)
        f.write("\n")
        f.flush()
        os.fsync(f.fileno())
    os.replace(tmp, path)


# ---- plausibility ---------------------------------------------------------------------------------------------------

def check_intrinsics(intr, width, height):
    """Problems of an intrinsics dict as a list of strings: [] = plausible. `errors` stop an apply, `warnings` do not."""
    errors, warnings = [], []
    f, cx, cy = intr.get("focal_px"), intr.get("cx"), intr.get("cy")
    for k, v in (("focal_px", f), ("cx", cx), ("cy", cy)):
        if not isinstance(v, (int, float)) or not math.isfinite(v):
            errors.append("%s is missing or not a number" % k)
    if errors:
        return errors, warnings
    hfov = math.degrees(2 * math.atan(width / 2.0 / f))
    if not 20.0 <= hfov <= 150.0:
        errors.append("focal_px %.1f gives a horizontal field of view of %.0f deg (expected 20..150)" % (f, hfov))
    if abs(cx - width / 2.0) > 0.15 * width or abs(cy - height / 2.0) > 0.15 * height:
        errors.append("picture centre (%.1f, %.1f) is more than 15 %% of the picture away from its middle" % (cx, cy))
    elif abs(cx - width / 2.0) > 0.06 * width or abs(cy - height / 2.0) > 0.06 * height:
        warnings.append("picture centre (%.1f, %.1f) is more than 6 %% of the picture away from its middle" % (cx, cy))
    if any(intr.get(k) for k in DIST_KEYS):
        # the distortion must stay monotonic inside the picture, otherwise the fit folds the picture's edge back
        rmax = math.hypot(max(cx, width - cx), max(cy, height - cy)) / f
        r = np.linspace(0.0, rmax, 200)
        rd = np.hypot(*distort(r, np.zeros_like(r), {k: intr.get(k) for k in ("k1", "k2", "k3")}))
        if np.any(np.diff(rd) <= 0):
            errors.append("the distortion folds back inside the picture (not a physical lens: too few points near "
                          "the picture's corners)")
    return errors, warnings


def check_mount(calib):
    errors, warnings = [], []
    for k in ("pitch", "yaw", "roll"):
        v = calib.get(k, 0.0)
        if not isinstance(v, (int, float)) or not math.isfinite(v):
            errors.append("%s is not a number" % k)
        elif abs(v) > 0.35:
            errors.append("%s %.3f rad (%.1f deg) is beyond 20 deg: radians expected, and no road camera is mounted "
                          "like that" % (k, v, math.degrees(v)))
        elif abs(v) > (0.17 if k != "roll" else 0.09):
            warnings.append("%s %.3f rad (%.1f deg) is large: the model's input crop leaves the area it was trained on; "
                            "re-aim the camera if you can" % (k, v, math.degrees(v)))
    h = calib.get("height_m")
    if h is not None:
        if not isinstance(h, (int, float)) or not 0.5 <= h <= 4.0:
            errors.append("height_m %r is not a lens height above the road (0.5..4 m)" % (h,))
        elif not 0.9 <= h <= 2.0:
            warnings.append("height_m %.2f is unusual for a passenger car (0.9..2.0 m): metres, lens centre to the road?" % h)
    return errors, warnings


def check_tape(height_m=None, camera_to_front_m=None, lateral_offset_m=None, vehicle=None):
    """Tape-measured values against each other and against a vehicle description (docs/VEHICLE_API.md 5.6), if any."""
    errors, warnings = [], []
    if camera_to_front_m is not None:
        if not 0.0 <= camera_to_front_m <= 6.0:
            errors.append("camera_to_front_m %.2f: the horizontal distance from the lens to the front bumper is "
                          "between 0 and 6 m" % camera_to_front_m)
        elif not 0.8 <= camera_to_front_m <= 3.0:
            warnings.append("camera_to_front_m %.2f is unusual for a camera at the windscreen (0.8..3.0 m)" % camera_to_front_m)
    if lateral_offset_m is not None:
        if abs(lateral_offset_m) > 1.5:
            errors.append("lateral_offset_m %.2f: the lens cannot be more than 1.5 m from the car's centre line" % lateral_offset_m)
        elif abs(lateral_offset_m) > 0.6:
            warnings.append("lateral_offset_m %.2f is far from the centre line: metres, positive to the right?" % lateral_offset_m)
    v = vehicle or {}
    cam = v.get("camera") or {}
    if height_m is not None and cam.get("z_m") is not None and abs(cam["z_m"] - height_m) > 0.05:
        warnings.append("vehicle description says the camera is %.2f m up, the calibration %.2f m" % (cam["z_m"], height_m))
    if height_m is not None and v.get("height_m") is not None and height_m > v["height_m"]:
        errors.append("height_m %.2f is above the car's roof (%.2f m)" % (height_m, v["height_m"]))
    if (camera_to_front_m is not None and cam.get("x_m") is not None and v.get("rear_axle_to_front_m") is not None
            and abs(cam["x_m"] + camera_to_front_m - v["rear_axle_to_front_m"]) > 0.10):
        warnings.append("camera x_m %.2f + camera_to_front_m %.2f != rear_axle_to_front_m %.2f"
                        % (cam["x_m"], camera_to_front_m, v["rear_axle_to_front_m"]))
    if lateral_offset_m is not None and v.get("width_m") is not None and abs(lateral_offset_m) > v["width_m"] / 2:
        errors.append("lateral_offset_m %.2f is outside the car (width %.2f m)" % (lateral_offset_m, v["width_m"]))
    return errors, warnings
