"""Position filter: an extended Kalman filter over [east, north, heading, speed, yaw-rate bias].

east/north: metres in a local tangent plane (re-centred when the car moves far from its origin);
heading: radians clockwise from true north (the direction the car's nose points); speed: m/s along the
heading, negative in reverse; bias: rad/s, subtracted from the yaw-rate input.

Inputs: yaw rate as a control input and speed as a measurement (vehicle motion or vision odometry),
GNSS position / speed / course as measurements, and map pseudo-measurements (cross-track position and
road bearing) from the matcher while dead reckoning. Without any motion input the model is constant
velocity with larger process noise.
"""
import math

import numpy as np

from . import geo

E, N, TH, V, B = range(5)
CHI2_POS = 13.8                 # 99.9 % gate, 2 degrees of freedom
RECENTRE_M = 20000.0


class PositionFilter:
    def __init__(self, cfg=None):
        c = dict(q_pos=0.05, q_speed=1.5, q_speed_meas=0.6, q_yaw=math.radians(0.5), q_yaw_free=math.radians(12.0),
                 q_bias=math.radians(0.01), speed_sigma=0.3, reset_after_rejects=5, odo_scale_sigma=0.02)
        c.update(cfg or {})
        self.cfg = c
        self.local = None
        self.x = np.zeros(5)
        self.P = np.eye(5)
        self.heading_known = False
        self.rejects = 0            # consecutive gated-out GNSS positions
        self.rejected_total = 0
        self.resets = 0

    # ---- state access ------------------------------------------------------------------------
    @property
    def initialised(self):
        return self.local is not None

    @property
    def latlon(self):
        return self.local.to_ll(self.x[E], self.x[N])

    @property
    def heading_deg(self):
        return math.degrees(self.x[TH]) % 360.0

    @property
    def speed(self):
        return float(self.x[V])

    @property
    def hacc(self):
        """RMS radial position error, metres."""
        return math.sqrt(max(0.0, self.P[E, E] + self.P[N, N]))

    @property
    def cross_sigma(self):
        """Standard deviation of the position across the heading, metres."""
        th = self.x[TH]
        cvec = np.array([math.cos(th), -math.sin(th)])
        return math.sqrt(max(0.0, float(cvec @ self.P[:2, :2] @ cvec)))

    @property
    def heading_sigma_deg(self):
        return math.degrees(math.sqrt(max(0.0, self.P[TH, TH]))) if self.heading_known else 180.0

    def init(self, lat, lon, hacc, speed=0.0, course_deg=None):
        self.local = geo.Local(lat, lon)
        self.x = np.zeros(5)
        self.x[V] = speed
        s = max(hacc, 3.0)
        self.P = np.diag([s * s, s * s, math.pi ** 2, 25.0, math.radians(0.3) ** 2])
        self.heading_known = False
        if course_deg is not None:
            self.x[TH] = math.radians(course_deg)
            self.P[TH, TH] = math.radians(15.0) ** 2
            self.heading_known = True
        self.rejects = 0

    def _recentre(self):
        if abs(self.x[E]) > RECENTRE_M or abs(self.x[N]) > RECENTRE_M:
            lat, lon = self.latlon
            self.local = geo.Local(lat, lon)
            self.x[E] = self.x[N] = 0.0

    # ---- prediction --------------------------------------------------------------------------
    def predict(self, dt, yaw_rate_cw=None, yaw_sigma=None, speed_measured=False):
        """Advance by dt seconds. yaw_rate_cw: rad/s, positive clockwise (turning right), or None."""
        if not self.initialised or dt <= 0:
            return
        c = self.cfg
        x = self.x
        th, v = x[TH], x[V]
        r = 0.0 if yaw_rate_cw is None else yaw_rate_cw - x[B]
        if abs(v) < 0.3:
            r = 0.0                                  # standing: the heading does not move
        thm = th + 0.5 * r * dt
        sn, cs = math.sin(thm), math.cos(thm)
        x[E] += v * sn * dt
        x[N] += v * cs * dt
        x[TH] = th + r * dt
        F = np.eye(5)
        F[E, TH], F[E, V] = v * cs * dt, sn * dt
        F[N, TH], F[N, V] = -v * sn * dt, cs * dt
        if yaw_rate_cw is not None and abs(v) >= 0.3:
            F[TH, B] = -dt
        qv = c["q_speed_meas"] if speed_measured else c["q_speed"]
        if yaw_rate_cw is None:
            qth = c["q_yaw_free"] if abs(v) >= 0.3 else 0.0
        else:
            qth = max(c["q_yaw"], yaw_sigma or 0.0) if abs(v) >= 0.3 else 0.0
        # random-walk noise: the q_* values are per square root of a second
        Q = np.diag([c["q_pos"] * dt, c["q_pos"] * dt, qth ** 2 * dt, qv ** 2 * dt, c["q_bias"] ** 2 * dt])
        self.P = F @ self.P @ F.T + Q
        # The odometer scale (tyres, wheel-speed calibration, vision speed) is a bias, not noise: the
        # along-track standard deviation grows by odo_scale_sigma of the distance travelled.
        d = abs(v) * dt * c["odo_scale_sigma"]
        if d > 0:
            a = np.array([math.sin(x[TH]), math.cos(x[TH])])
            sa = math.sqrt(max(0.0, float(a @ self.P[:2, :2] @ a)))
            self.P[:2, :2] += (2.0 * sa * d + d * d) * np.outer(a, a)
        self._recentre()

    # ---- measurements ------------------------------------------------------------------------
    def _update(self, H, innov, R):
        S = H @ self.P @ H.T + R
        K = self.P @ H.T @ np.linalg.inv(S)
        self.x = self.x + K @ innov
        I_KH = np.eye(5) - K @ H
        self.P = I_KH @ self.P @ I_KH.T + K @ R @ K.T
        self.x[TH] = self.x[TH] % (2 * math.pi)

    def update_speed(self, v, sigma=None):
        if not self.initialised:
            return
        H = np.zeros((1, 5))
        H[0, V] = 1.0
        s = sigma if sigma is not None else self.cfg["speed_sigma"]
        self._update(H, np.array([v - self.x[V]]), np.array([[s * s]]))

    def innovation(self, lat, lon, hacc):
        """(squared Mahalanobis distance, along-track innovation m, cross-track innovation m) of a fix against
        the prediction, or None before the filter has a state. Read-only: the receiver-accuracy estimator uses it."""
        if not self.initialised:
            return None
        zx, zy = self.local.to_xy(lat, lon)
        s = max(hacc, 1.0) / 1.5
        S = self.P[:2, :2] + np.eye(2) * s * s
        innov = np.array([zx - self.x[E], zy - self.x[N]])
        th = self.x[TH]
        along = float(innov[0] * math.sin(th) + innov[1] * math.cos(th))
        cross = float(innov[0] * math.cos(th) - innov[1] * math.sin(th))
        return float(innov @ np.linalg.inv(S) @ innov), along, cross

    def update_position(self, lat, lon, hacc, gate=True):
        """GNSS position. hacc: 68 % horizontal radius, metres. Returns True when used."""
        if not self.initialised:
            self.init(lat, lon, hacc)
            return True
        zx, zy = self.local.to_xy(lat, lon)
        H = np.zeros((2, 5))
        H[0, E] = H[1, N] = 1.0
        s = max(hacc, 1.0) / 1.5                      # 68 % radius of a circular Gaussian = 1.515 sigma
        R = np.eye(2) * s * s
        innov = np.array([zx - self.x[E], zy - self.x[N]])
        if gate:
            S = H @ self.P @ H.T + R
            d2 = float(innov @ np.linalg.inv(S) @ innov)
            if d2 > CHI2_POS:
                self.rejects += 1
                self.rejected_total += 1
                if self.rejects < self.cfg["reset_after_rejects"]:
                    return False
                # the fixes keep disagreeing: the filter is wrong, restart on the fix
                th, known, v, b = self.x[TH], self.heading_known, self.x[V], self.x[B]
                pth, pb = self.P[TH, TH], self.P[B, B]
                self.init(lat, lon, hacc, speed=v)
                self.x[TH], self.x[B], self.heading_known = th, b, known
                self.P[TH, TH], self.P[B, B] = (min(math.pi ** 2, pth + math.radians(20) ** 2) if known
                                                else math.pi ** 2), pb
                self.resets += 1
                return True
        self.rejects = 0
        self._update(H, innov, R)
        return True

    def update_course(self, course_deg, sigma_deg, gate_deg=None):
        """Direction of travel over ground (GNSS course, or a road bearing from the matcher). With gate_deg a
        measurement that differs by more is ignored outright (map feedback must never turn the car)."""
        if not self.initialised:
            return
        z = math.radians(course_deg)
        if not self.heading_known:
            self.x[TH] = z % (2 * math.pi)
            self.P[TH, :] = 0.0
            self.P[:, TH] = 0.0
            self.P[TH, TH] = math.radians(max(sigma_deg, 5.0)) ** 2
            self.heading_known = True
            return
        H = np.zeros((1, 5))
        H[0, TH] = 1.0
        innov = geo.wrap_pi(z - self.x[TH])
        s = math.radians(sigma_deg)
        if gate_deg is not None and abs(innov) > math.radians(gate_deg):
            return
        if abs(innov) > 3.0 * math.sqrt(self.P[TH, TH] + s * s) + math.radians(10.0):
            self.P[TH, TH] += math.radians(10.0) ** 2   # disagreement: open up instead of jumping
            return
        self._update(H, np.array([innov]), np.array([[s * s]]))

    def update_on_road(self, lat, lon, bearing_deg, sigma_cross, sigma_along):
        """Map pseudo-measurement: the car is near (lat, lon) on a road of this bearing; only the
        cross-track component is trusted (sigma_along is large)."""
        if not self.initialised:
            return
        zx, zy = self.local.to_xy(lat, lon)
        b = math.radians(bearing_deg)
        along = np.array([math.sin(b), math.cos(b)])
        cross = np.array([math.cos(b), -math.sin(b)])
        R = sigma_along ** 2 * np.outer(along, along) + sigma_cross ** 2 * np.outer(cross, cross)
        H = np.zeros((2, 5))
        H[0, E] = H[1, N] = 1.0
        self._update(H, np.array([zx - self.x[E], zy - self.x[N]]), R)
