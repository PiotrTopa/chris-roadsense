"""A radar as a source: `objects` messages of the vehicle input API (docs/VEHICLE_API.md 5.7) with raw
detections or the sensor's own object list, polar or Cartesian in the sensor's frame (or in the camera's
frame, as mapd relays them).

What it gives the integrator:
  * object detections (range, azimuth, range rate) with a non-linear measurement model for the object layer;
  * the car's own velocity from the range rates of the stationary world (a robust fit per scan): a speed
    measurement that is independent of the wheels and of the camera, in ground scale, so it votes in the
    isolation of a speed fault and makes the speed scale observable;
  * its own calibration states: mount yaw misalignment (from the direction of the stationary world's
    apparent motion) and time delay.

Any number of radars: one `RadarSource` per sensor id with its `Mount`. `{"kind": "radar", "id": "*"}` in the
configuration registers one per sensor id as its scans arrive (mount from the message, else at the camera).
"""
import math
import random

import numpy as np

from ..core import ego as E
from ..core.ego import AuxState
from ..core.objects import VX, VY, X, Y, Det, Scan, SensorModel
from ..core.registry import NS, Meas, Mount, Source, fin
from . import KINDS
from .vehicle import scale_reg

AXLE = 1.5                       # the car frame's origin (reference camera) is this far ahead of the rear axle


def _wrap(a):
    return (a + math.pi) % (2.0 * math.pi) - math.pi


class RadarModel(SensorModel):
    name = "radar"
    initiates = True
    p_d = 0.9
    clutter = 2.0 / (150.0 * 1.6 * 60.0)       # two false detections per scan over range x azimuth x range rate

    def __init__(self, mount, yaw, fov, max_range, sig):
        self.mx, self.my, self.yaw = mount.x, mount.y, yaw
        self.fov, self.max_range, self.sig = fov, max_range, sig

    def _sensor_vel(self, ego):
        v, w = ego[0], ego[1]
        return v - w * self.my, w * (self.mx + AXLE)

    def h(self, x, ego):
        dx, dy = x[X] - self.mx, x[Y] - self.my
        r = max(math.hypot(dx, dy), 0.5)
        vsx, vsy = self._sensor_vel(ego)
        rvx, rvy = x[VX] - vsx, x[VY] - vsy
        rr = (dx * rvx + dy * rvy) / r
        z = np.array([r, math.atan2(dy, dx) - self.yaw, rr])
        H = np.zeros((3, 5))
        H[0, X], H[0, Y] = dx / r, dy / r
        H[1, X], H[1, Y] = -dy / (r * r), dx / (r * r)
        H[2, X], H[2, Y] = rvx / r - rr * dx / (r * r), rvy / r - rr * dy / (r * r)
        H[2, VX], H[2, VY] = dx / r, dy / r
        return z, H

    def residual(self, z, zp):
        d = z - zp
        d[1] = _wrap(d[1])
        return d

    def initiate(self, det, ego):
        r, az, rr = det.z
        b = az + self.yaw
        cb, sb = math.cos(b), math.sin(b)
        vsx, vsy = self._sensor_vel(ego)
        vr = rr + cb * vsx + sb * vsy              # the target's own speed along the line of sight
        x0 = np.array([self.mx + r * cb, self.my + r * sb, vr * cb, vr * sb, 0.0])
        sr, sa, sv = self.sig
        pt = (r * sa) ** 2
        P0 = np.diag([sr * sr * cb * cb + pt * sb * sb + 0.25, sr * sr * sb * sb + pt * cb * cb + 0.25,
                      4.0 * sv * sv + 1.0 + 25.0 * sb * sb, 9.0, 4.0])
        return x0, P0

    def coverage(self, x):
        dx, dy = x[X] - self.mx, x[Y] - self.my
        r = math.hypot(dx, dy)
        if r < 0.5 or r > self.max_range or abs(_wrap(math.atan2(dy, dx) - self.yaw)) > self.fov:
            return 0.0
        return self.p_d


def ego_velocity(az, rr, rng, tol=0.35, iters=24):
    """The sensor's own velocity (vx, vy in its frame) from detections of the stationary world:
    rr_i = -(vx cos az_i + vy sin az_i). RANSAC over pairs, then least squares on the inliers.
    Returns (vx, vy, covariance 2x2, inlier mask) or None."""
    n = len(az)
    if n < 5:
        return None
    A = -np.column_stack([np.cos(az), np.sin(az)])
    # all candidate pairs at once
    ii = np.array([rng.randrange(n) for _ in range(iters)])
    jj = np.array([rng.randrange(n) for _ in range(iters)])
    a, b, c, d = A[ii, 0], A[ii, 1], A[jj, 0], A[jj, 1]
    det = a * d - b * c
    ok = np.abs(det) > 0.02
    if not ok.any():
        return None
    det = np.where(ok, det, 1.0)
    vx = (d * rr[ii] - b * rr[jj]) / det
    vy = (a * rr[jj] - c * rr[ii]) / det
    res = np.abs(np.outer(vx, A[:, 0]) + np.outer(vy, A[:, 1]) - rr[None, :]) < tol
    cnt = np.where(ok, res.sum(axis=1), -1)
    k = int(cnt.argmax())
    best = (int(cnt[k]), res[k])
    if best[0] < 5 or best[0] < 0.35 * n:
        return None
    inl = best[1]
    for _ in range(2):
        Ai, ri = A[inl], rr[inl]
        N = Ai.T @ Ai
        if np.linalg.cond(N) > 1e4:
            return None
        v = np.linalg.solve(N, Ai.T @ ri)
        res = A @ v - rr
        inl = np.abs(res) < tol
        if inl.sum() < 5:
            return None
    s2 = max(float(np.mean(res[inl] ** 2)), 0.05 ** 2)
    return float(v[0]), float(v[1]), s2 * np.linalg.inv(A[inl].T @ A[inl]), inl


class RadarSource(Source):
    rank = 2
    types = ("objects",)
    stale_s = 0.5
    info = "radar: detections for the object layer, own velocity from the stationary world"
    fov = math.radians(50.0)
    max_range = 160.0
    sigma = (0.25, math.radians(0.5), 0.15)      # range m, azimuth rad, range rate m/s
    roadside_m = 5.0             # stationary detections further than this from the car's axis start no tracks
    ego_motion = True

    def __init__(self, sensor, mount=None, **kw):
        self.id = sensor
        self.sensor = sensor
        self.mount = mount or Mount()
        self.aux = (AuxState("dyaw", 0.0, math.radians(2.0), 1e-10, unit="rad", doc="mount yaw misalignment"),
                    AuxState("tau", 0.05, 0.03, 1e-9, lo=0.0, hi=0.3, unit="s", doc="a scan is this old at its stamp"))
        super().__init__(**kw)
        self.rng = random.Random(17)

    def _parse(self, msg, ctx):
        """(r, az, rr arrays in the sensor's frame, attrs list) of a message that is this radar's, else None."""
        if msg.get("sensor") != self.sensor or msg.get("kind") not in ("radar", "lidar", None):
            return None
        mem = ctx.mem
        if "_scan" in mem and mem["_scan"][0] is msg:
            return mem["_scan"][1]
        objs = msg.get("objects")
        if not isinstance(objs, list):
            return None
        in_cam = msg.get("frame") == "camera"
        r, az, rr, at = [], [], [], []
        for o in objs:
            if not isinstance(o, dict):
                continue
            if fin(o.get("range_m")) and fin(o.get("azimuth_deg")):
                ri, ai = float(o["range_m"]), math.radians(o["azimuth_deg"])
            elif fin(o.get("x_m")):
                x, y = float(o["x_m"]), float(o["y_m"]) if fin(o.get("y_m")) else 0.0
                if in_cam:                       # mapd's relay: positions in the camera's frame
                    x, y = x - self.mount.x, y - self.mount.y
                    c, s = math.cos(self.mount.yaw), math.sin(self.mount.yaw)
                    x, y = c * x + s * y, -s * x + c * y
                ri, ai = math.hypot(x, y), math.atan2(y, x)
            else:
                continue
            v = o.get("range_rate_mps")
            if not fin(v):
                v = o.get("vx_mps")
                v = float(v) * math.cos(ai) if fin(v) else None
            if v is None or ri < 0.5 or ri > self.max_range or abs(ai) > self.fov:
                continue
            r.append(ri), az.append(ai), rr.append(float(v))
            at.append({k: o[k] for k in ("id", "rcs_dbsm", "class") if k in o})
        out = (np.array(r), np.array(az), np.array(rr), at)
        mem["_scan"] = (msg, out)
        return out

    def measure(self, msg, ctx):
        p = self._parse(msg, ctx)
        if p is None:
            return []
        ts = msg["ts_mono_ns"] / NS
        mem = ctx.mem
        mem["n"] = mem.get("n", 0) + 1
        out = [Meas("signal:%s_scan" % self.sensor, ts, len(p[0]), role="signal")]
        mem["static"] = None
        if not self.ego_motion:
            return out
        fit = ego_velocity(p[1], p[2], self.rng)
        if fit is None:
            return out
        vx, vy, C, inl = fit
        mem["static"] = inl
        mem["fits"] = mem.get("fits", 0) + 1
        c, s = math.cos(self.mount.yaw), math.sin(self.mount.yaw)
        ux, uy = c * vx - s * vy, s * vx + c * vy            # in the car's axes, by the nominal mount
        J = np.array([[c, -s], [s, c]])
        Cc = J @ C @ J.T
        idy, itau = self.ix["dyaw"], self.ix["tau"]
        x = ctx.x
        tau = float(x[itau])
        mxa, my = self.mount.x + AXLE, self.mount.y
        i_s = np.array([E.V, E.SV, E.W, itau], dtype=np.intp)

        def h_x(x, tau0=tau):
            g = 1.0 / (1.0 + x[E.SV])
            v = x[E.V] - x[E.A] * (x[itau] - tau0)
            return v * g - x[E.W] * my, i_s, np.array([g, -scale_reg(x[E.V], 3.0) * g * g, -my, -x[E.A] * g])
        i_l = np.array([E.W, idy, E.V], dtype=np.intp)

        def h_y(x):
            vg = x[E.V] / (1.0 + x[E.SV])
            return x[E.W] * mxa - vg * x[idy], i_l, np.array([mxa, -scale_reg(vg, 5.0), -x[idy]])
        sx = max(math.sqrt(max(Cc[0, 0], 0.0)), 0.08)
        sy = max(math.sqrt(max(Cc[1, 1], 0.0)), 0.25)       # (0.12 was optimistic for a corner mount: found with the second radar)
        w_now = float(x[E.W])
        ground = ux + w_now * my
        t = ts - tau
        out.append(Meas("speed", t, ux, sx, h_x, self.sensor + ".ego", robust=True,
                        mon=(ground * (1.0 + float(x[E.SV])), 0.15 + 0.01 * abs(ground))))
        out.append(Meas("radar_lat", t, uy, sy, h_y, self.sensor + ".ego", robust=True))
        return out

    def scan(self, msg, ctx):
        p = self._parse(msg, ctx)
        if p is None:
            return None
        r, az, rr, at = p
        ts = msg["ts_mono_ns"]
        yaw = self.mount.yaw + ctx.val("dyaw")
        sr, sa, sv = self.sigma
        model = RadarModel(self.mount, yaw, self.fov, self.max_range, self.sigma)
        static = ctx.mem.get("static")
        dets = []
        for i in range(len(r)):
            if static is not None and static[i]:
                # the stationary world: roadside returns start no tracks; what stands on the road ahead does
                if abs(self.mount.y + r[i] * math.sin(az[i] + yaw)) > self.roadside_m:
                    continue
            R = np.array([sr * sr + (0.005 * r[i]) ** 2, sa * sa, sv * sv])
            dets.append(Det([r[i], az[i], rr[i]], R, at[i]))
        return Scan(self.id, self.group, ts / NS - ctx.val("tau"), dets, model, ts)

    def report(self, ctx):
        m = ctx.mem
        n = m.get("n", 0)
        return {"scans": n, "ego_fit_share": round(m.get("fits", 0) / n, 2) if n else None}


class RadarAuto(Source):
    """Registers a `RadarSource` for every radar sensor id that shows up (a car that feeds object lists wants
    them used): the mount is the message's `mount` ({"x_m", "y_m", "yaw_deg"} in the car frame) or, for scans
    relayed in the camera's frame, the camera itself."""
    id = "radars"
    types = ("objects",)
    counts = False
    stale_s = 1e9
    info = "registers radars as they appear"

    def __init__(self, defaults=None, **kw):
        super().__init__(**kw)
        self.defaults = defaults or {}
        self.known = set()

    def measure(self, msg, ctx):
        sid = msg.get("sensor")
        if not isinstance(sid, str) or sid in self.known or msg.get("kind") not in ("radar", "lidar") \
                or not isinstance(msg.get("objects"), list):
            return []
        if not (msg.get("frame") == "camera" or isinstance(msg.get("mount"), dict) or sid in self.defaults):
            return []                              # a scan that cannot be placed in the car frame is not used
        self.known.add(sid)
        if any(s.id == sid for s in ctx.integ.specs):
            return []
        m = msg.get("mount") or self.defaults.get(sid) or {}
        ctx.integ.register(RadarSource(sid, _mount(m)))
        return []


def _mount(m):
    return Mount(m.get("x_m", 0.0), m.get("y_m", 0.0), m.get("z_m", 0.0), math.radians(m.get("yaw_deg", 0.0)))


def _factory(spec):
    if spec.get("id", "*") == "*":
        return RadarAuto(spec.get("mounts"))
    src = RadarSource(spec["id"], _mount(spec.get("mount") or {}))
    for k in ("fov", "max_range", "roadside_m", "ego_motion", "rank"):
        if k in spec:
            setattr(src, k, spec[k])
    return src


KINDS["radar"] = _factory
