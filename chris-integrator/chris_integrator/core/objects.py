"""The object layer: one multi-target track list in the car frame (docs/INTEGRATOR.md section 6).

Track state [x, y, vx, vy, ax]: position relative to the car (x forward, y left, origin the reference camera),
velocity and longitudinal acceleration over the ground in the car's axes. The car's own motion (speed, yaw
rate from the ego estimator at that time) is the input of the process model.

Sensors are models (`SensorModel`): the layer does prediction, association (global nearest neighbour on gated
costs), existence (Bayes recursion with the sensor's detection probability and clutter density), life cycle,
cross-sensor merging, ghost / stationary marks and the choice of the lead by the path. A sensor that reports
one designated object with its own identity semantics (the road model's lead) supplies `process()` instead of
going through the generic association.
"""
import math

import numpy as np

from .registry import NS

X, Y, VX, VY, AX = range(5)

DEFAULT_CFG = dict(
    jerk_psd=30.0,               # (m/s3)^2 s: how fast a target's acceleration may change
    accel_tau_s=3.0,             # a target's acceleration decays with this time constant
    lat_acc_psd=1.0,             # (m/s2)^2 s: lateral manoeuvres of a target
    stand_jerk_psd=0.05, stand_lat_psd=0.01,
    ref_to_axle_m=1.5,           # the frame's origin (reference camera) is this far ahead of the rear axle
    gate_chi2=16.0,              # association gate (chi-square on the innovation of a generic sensor)
    confirm_exist=0.95, confirm_hits=3, delete_exist=0.05,
    new_exist=0.25,              # existence of a track started by one detection
    coast_max_s=1.0,             # a generic track without any hit for this long is dropped
    oosm_max_s=0.3,              # a scan older than the layer's time by more than this is dropped
    merge_gate=9.0, merge_s=0.3, # two tracks of different sensors this close (chi-square, 4 dof) for this long are one
    ghost_range_tol=0.12, ghost_az_rad=0.06, ghost_rate_tol=2.0,
    stationary_speed=0.8, moved_speed=1.5,
    moved_confirm_s=0.5,         # "has moved": the speed over ground above moved_speed by 2 sigma (along the velocity,
                                 # vy's uncertainty included) for this long without a break
    corridor_m=1.3, corridor_rel=0.012,      # on the path: |offset| <= corridor_m + corridor_rel * distance
    path_max_m=150.0,
    lead_radar_only=True,        # a track no camera designates may be the lead when it is on the path
    lead_min_hits=5, lead_min_exist=0.97,
    max_tracks=64,
)


class Det:
    """One detection: measurement vector, its covariance (or variances), free attributes."""
    __slots__ = ("z", "R", "attrs", "used")

    def __init__(self, z, R, attrs=None):
        self.z, self.R, self.attrs = np.asarray(z, float), np.asarray(R, float), attrs or {}
        self.used = False


class Scan:
    """What an object source returns for a message: time of validity, detections, the sensor's model."""
    __slots__ = ("src", "group", "t", "dets", "model", "ts_ns", "raw")

    def __init__(self, src, group, t, dets, model, ts_ns=None, raw=None):
        self.src, self.group, self.t, self.dets, self.model, self.ts_ns, self.raw = src, group, t, dets, model, ts_ns, raw


class SensorModel:
    """What the layer needs to know about a sensor. Subclass per sensor kind."""
    name = "sensor"
    initiates = True             # may a detection of this sensor start a track
    p_d = 0.9                    # detection probability inside the coverage
    clutter = 1e-3               # false detections per unit of measurement volume
    process = None               # process(layer, scan, ego): a sensor with its own association semantics

    def h(self, x, ego):
        """Predicted measurement and Jacobian (m x 5) for a track state."""
        raise NotImplementedError

    def residual(self, z, zp):
        return z - zp

    def initiate(self, det, ego):
        """(x0, P0) of a track started from a detection."""
        raise NotImplementedError

    def coverage(self, x):
        """Detection probability of a target at this state (0 outside the field of view / range)."""
        return self.p_d

    def gate(self, track, det, nu, S, d2):
        """Extra association rule; False rejects the pairing."""
        return True


class Track:
    __slots__ = ("id", "x", "P", "t", "t0", "state", "logodds", "hits", "sources", "last_seen", "flags", "mem",
                 "standing", "moved", "designated", "merge_t", "sig_ve", "nis", "primary")

    def __init__(self, tid, x, P, t, src, exist):
        self.id, self.x, self.P, self.t, self.t0 = tid, np.array(x, float), np.array(P, float), t, t
        self.state = "tentative"
        self.logodds = math.log(exist / (1.0 - exist))
        self.hits = 1
        self.sources = {src}
        self.last_seen = {src: t}
        self.flags = set()
        self.mem = {}                      # per-sensor policy state
        self.standing = False
        self.moved = False
        self.designated = None             # source id of the camera whose model calls this its lead
        self.merge_t = {}
        self.primary = src
        self.nis = {}

    @property
    def exist(self):
        return 1.0 / (1.0 + math.exp(-self.logodds))

    def update_scalar(self, i, z, r):
        """Scalar update of state i (a linear, direct measurement); returns the innovation."""
        P = self.P
        nu, s = z - self.x[i], P[i, i] + r
        k = P[:, i] / s
        self.x = self.x + k * nu
        P = P - np.outer(k, P[i, :])
        self.P = 0.5 * (P + P.T)
        for j in range(5):
            if self.P[j, j] < 1e-9:
                self.P[j, j] = 1e-9
        return nu

    def innovation(self, i, z, r):
        return z - self.x[i], self.P[i, i] + r


def assign(cost):
    """Optimal assignment (Hungarian, O(n^3)) on a rectangular cost matrix with np.inf for forbidden pairs.
    Returns a list of (row, col)."""
    n, m = cost.shape
    if n == 0 or m == 0:
        return []
    fin = np.isfinite(cost)
    if not fin.any():
        return []
    # the common case on a road: no two tracks compete for a detection. Then each row's best column is the
    # optimum and the cubic algorithm is not needed
    rows = np.nonzero(fin.any(axis=1))[0]
    best = np.where(fin[rows], cost[rows], np.inf).argmin(axis=1)
    if len(set(best.tolist())) == len(best) and (fin[:, best].sum(axis=0) == 1).all():
        return [(int(i), int(j)) for i, j in zip(rows, best)]
    big = 1e6
    c = np.where(np.isfinite(cost), cost, big)
    k = max(n, m)
    a = np.full((k, k), big)
    a[:n, :m] = c
    u, v = np.zeros(k + 1), np.zeros(k + 1)
    p, way = np.zeros(k + 1, dtype=int), np.zeros(k + 1, dtype=int)
    for i in range(1, k + 1):
        p[0] = i
        j0 = 0
        minv = np.full(k + 1, np.inf)
        used = np.zeros(k + 1, dtype=bool)
        while True:
            used[j0] = True
            i0, delta, j1 = p[j0], np.inf, 0
            for j in range(1, k + 1):
                if not used[j]:
                    cur = a[i0 - 1, j - 1] - u[i0] - v[j]
                    if cur < minv[j]:
                        minv[j], way[j] = cur, j0
                    if minv[j] < delta:
                        delta, j1 = minv[j], j
            for j in range(k + 1):
                if used[j]:
                    u[p[j]] += delta
                    v[j] -= delta
                else:
                    minv[j] -= delta
            j0 = j1
            if p[j0] == 0:
                break
        while True:
            j1 = way[j0]
            p[j0] = p[j1]
            j0 = j1
            if j0 == 0:
                break
    out = []
    for j in range(1, k + 1):
        i = p[j] - 1
        if i < n and j - 1 < m and np.isfinite(cost[i, j - 1]):
            out.append((i, j - 1))
    return out


class ObjectLayer:
    def __init__(self, cfg=None):
        self.cfg = dict(DEFAULT_CFG, **(cfg or {}))
        self.faulted = frozenset()
        self.lead_cfg = None
        self._ve = self._sv = self._sw = 0.0
        self.next_id = 1
        self.seq = 0
        self.reset()

    def reset(self):
        self.tracks = []
        self.queue = []                    # scans waiting for their turn, by time
        self.t = None                      # the layer's time: the newest scan / frame processed
        self.models = {}                   # source id -> its last model (coverage for misses, reporting)
        self.stats = {}                    # source id -> counters
        self.lead_id = None
        self.lead_event = None
        self.path = None

    def set_faulted(self, groups):
        groups = frozenset(groups)
        if groups != self.faulted:
            self.faulted = groups
            self.queue = [s for s in self.queue if s.group not in groups]

    def new_id(self):
        i = self.next_id
        self.next_id += 1
        return i

    def add_scan(self, scan):
        self.queue.append(scan)

    # ---- motion --------------------------------------------------------------------------------------
    def predict_track(self, tr, dt, ego):
        """ego: (v, w, sigma_v, stands)."""
        if dt <= 0.0:
            return
        c = self.cfg
        v_e, w, sig_ve = ego[0], ego[1], ego[2]
        stands = len(ego) > 3 and ego[3]
        x, y, vx, vy, ax = tr.x
        rho = math.exp(-dt / c["accel_tau_s"])
        dt2 = dt * dt
        la = c["ref_to_axle_m"]
        tr.x = np.array([x + (vx - v_e + w * y) * dt + 0.5 * ax * dt2,
                         y + (vy - w * (x + la)) * dt,
                         vx + (ax + w * vy) * dt,
                         vy - w * vx * dt,
                         ax * rho])
        F = np.array([[1.0, w * dt, dt, 0.0, 0.5 * dt2],
                      [-w * dt, 1.0, 0.0, dt, 0.0],
                      [0.0, 0.0, 1.0, w * dt, dt],
                      [0.0, 0.0, -w * dt, 1.0, 0.0],
                      [0.0, 0.0, 0.0, 0.0, rho]])
        P = F @ tr.P @ F.T
        q = c["stand_jerk_psd"] if tr.standing else c["jerk_psd"]
        ql = c["stand_lat_psd"] if tr.standing else c["lat_acc_psd"]
        dt3, dt4, dt5 = dt2 * dt, dt2 * dt2, dt2 * dt2 * dt
        P[X, X] += q * dt5 / 20.0 + (0.0 if stands else (sig_ve * dt) ** 2)
        P[X, VX] += q * dt4 / 8.0
        P[VX, X] += q * dt4 / 8.0
        P[X, AX] += q * dt3 / 6.0
        P[AX, X] += q * dt3 / 6.0
        P[VX, VX] += q * dt3 / 3.0
        P[VX, AX] += q * dt2 / 2.0
        P[AX, VX] += q * dt2 / 2.0
        P[AX, AX] += q * dt
        P[Y, Y] += ql * dt3 / 3.0
        P[Y, VY] += ql * dt2 / 2.0
        P[VY, Y] += ql * dt2 / 2.0
        P[VY, VY] += ql * dt
        tr.P = P
        tr.t += dt
        if tr.x[X] < 0.3 and tr.designated is not None:
            tr.x[X] = 0.3

    def _advance(self, t, ego):
        for tr in self.tracks:
            self.predict_track(tr, t - tr.t, ego)
        self.t = t

    # ---- one scan ------------------------------------------------------------------------------------
    def _apply(self, scan, ego_fn):
        c = self.cfg
        st = self.stats.setdefault(scan.src, {"scans": 0, "dets": 0, "assigned": 0, "started": 0, "late": 0,
                                               "dropped": 0})
        st["scans"] += 1
        st["dets"] += len(scan.dets)
        self.models[scan.src] = (scan.model, scan.t)
        model = scan.model
        if self.t is not None and scan.t < self.t - 1e-6:
            if self.t - scan.t > c["oosm_max_s"]:
                st["dropped"] += 1
                return
            st["late"] += 1
            late = self.t - scan.t           # the tracks stay where they are; the scan is compared with the
        else:                               # tracks retrodicted by their own velocity (below)
            late = 0.0
            ego = ego_fn(scan.t)
            self._advance(scan.t, ego)
        ego = ego_fn(scan.t)
        if model.process is not None:
            model.process(self, scan, ego)
            return
        tracks = [t for t in self.tracks]
        dets = scan.dets
        nt, nd = len(tracks), len(dets)
        cost = np.full((nt, nd), np.inf)
        pred = []
        if nd:
            Z = np.array([d.z for d in dets])
            Rm = np.array([d.R if d.R.ndim == 1 else np.diag(d.R) for d in dets])
            Rmean = Rm.mean(axis=0)
        for i, tr in enumerate(tracks):
            xs = tr.x if late == 0.0 else self._retro(tr, late, ego)
            if nd == 0 or model.coverage(xs) <= 0.0:
                pred.append(None)
                continue
            zp, H = model.h(xs, ego)
            PHt = tr.P @ H.T
            S0 = H @ PHt
            pred.append((zp, H, PHt, S0))
            # one innovation covariance per track (the scan's mean noise) for the gate; the update uses each
            # detection's own
            S = S0 + np.diag(Rmean)
            try:
                Si = np.linalg.inv(S)
            except np.linalg.LinAlgError:
                continue
            ld = math.log(max(np.linalg.det(S), 1e-12))
            NU = np.array([model.residual(z, zp) for z in Z]) if model.residual.__func__ is not SensorModel.residual \
                else Z - zp
            D2 = np.einsum("ij,jk,ik->i", NU, Si, NU)
            for j in np.nonzero(D2 <= c["gate_chi2"])[0]:
                if model.gate(tr, dets[j], NU[j], S, float(D2[j])):
                    cost[i, j] = D2[j] + ld
        hit = set()
        for i, j in assign(cost):
            tr, d = tracks[i], dets[j]
            zp, H, PHt, S0 = pred[i]
            R = d.R if d.R.ndim == 2 else np.diag(d.R)
            S = S0 + R
            nu = model.residual(d.z, zp)
            Si = np.linalg.inv(S)
            K = PHt @ Si
            d2 = float(nu @ Si @ nu)
            tr.x = tr.x + K @ nu
            P = tr.P - K @ S @ K.T
            tr.P = 0.5 * (P + P.T)
            d.used = True
            hit.add(i)
            st["assigned"] += 1
            tr.hits += 1
            tr.sources.add(scan.src)
            tr.last_seen[scan.src] = scan.t
            n = tr.nis.get(scan.src)
            tr.nis[scan.src] = d2 / len(nu) if n is None else n + 0.1 * (d2 / len(nu) - n)
            if tr.state != "tentative":                # the sensor's own consistency, over confirmed tracks
                st["nis"] = d2 / len(nu) if st.get("nis") is None else st["nis"] + 0.02 * (d2 / len(nu) - st["nis"])
            if d.attrs:
                tr.mem.setdefault("attrs", {}).update(d.attrs)
            # existence: a hit multiplies the odds by P_D x likelihood / clutter density (bounded)
            like = math.exp(-0.5 * d2) / math.sqrt(max((2.0 * math.pi) ** len(nu) * np.linalg.det(S), 1e-12))
            tr.logodds += min(max(math.log(max(model.p_d * like / model.clutter, 1e-9)), math.log(1.5)), math.log(20.0))
            if hasattr(model, "on_hit"):
                model.on_hit(self, tr, d, scan)
        for i, tr in enumerate(tracks):
            if i in hit or pred[i] is None:
                continue
            pd = model.coverage(tr.x)
            if pd > 0.0:
                tr.logodds += math.log(max(1.0 - pd, 1e-3))
        if model.initiates:
            for d in dets:
                if d.used or len(self.tracks) >= c["max_tracks"]:
                    continue
                x0, P0 = model.initiate(d, ego)
                tr = Track(self.new_id(), x0, P0, self.t, scan.src, c["new_exist"])
                if d.attrs:
                    tr.mem["attrs"] = dict(d.attrs)
                self.tracks.append(tr)
                st["started"] += 1

    def _retro(self, tr, late, ego):
        x = tr.x.copy()
        x[X] -= (x[VX] - ego[0]) * late
        x[Y] -= x[VY] * late
        return x

    # ---- housekeeping --------------------------------------------------------------------------------
    def _manage(self, T):
        c = self.cfg
        keep = []
        for tr in self.tracks:
            lg = max(-12.0, min(12.0, tr.logodds))
            tr.logodds = lg
            policy = tr.mem.get("policy")              # a sensor that owns the track's life cycle
            last = max(tr.last_seen.values()) if tr.last_seen else -1e18
            gen_seen = [t for s, t in tr.last_seen.items() if s != tr.designated]
            if policy is not None and policy.get("alive"):
                tr.state = policy.get("state", "confirmed")
                keep.append(tr)
                continue
            # generic life cycle (also a track whose camera let go, while other sensors still see it)
            if not tr.last_seen:
                continue                                # the camera dropped it and nobody else ever saw it
            last = max(gen_seen) if gen_seen else last
            if T - last > c["coast_max_s"] or tr.exist < c["delete_exist"] or not np.all(np.isfinite(tr.x)):
                continue
            if tr.state == "tentative":
                if tr.exist >= c["confirm_exist"] and tr.hits >= c["confirm_hits"]:
                    tr.state = "confirmed"
            elif tr.state in ("confirmed", "coast"):
                tr.state = "coast" if T - last > 0.25 else "confirmed"
            keep.append(tr)
        self.tracks = keep
        # speed marks
        for tr in self.tracks:
            sp = math.hypot(tr.x[VX], tr.x[VY])
            # the speed over ground is the relative velocity plus the car's own: the car's speed and yaw rate errors
            # (a yaw rate error w reads as a lateral speed w x range) are biases the track's covariance does not hold
            ego_var = self._sv ** 2 + (self._sw * math.hypot(tr.x[X] + c["ref_to_axle_m"], tr.x[Y])) ** 2
            if sp > 1e-3:
                u = np.array([tr.x[VX], tr.x[VY]]) / sp
                sd = math.sqrt(max(float(u @ tr.P[2:4, 2:4] @ u), 0.0) + ego_var)
            else:
                sd = math.sqrt(max(tr.P[VX, VX], tr.P[VY, VY], 0.0) + ego_var)
            # a single excursion of a young or weakly observed track (radar: vy only through the azimuth's rate)
            # is not motion; "has moved" needs a significant speed held for moved_confirm_s
            t_prev = tr.mem.get("move_T")
            dt = 0.0 if t_prev is None else max(T - t_prev, 0.0)
            tr.mem["move_T"] = T
            if sp - 2.0 * sd > c["moved_speed"]:
                tr.mem["move_s"] = tr.mem.get("move_s", 0.0) + dt
                if tr.mem["move_s"] >= c["moved_confirm_s"]:
                    tr.moved = True
            else:
                tr.mem["move_s"] = 0.0
            tr.flags.discard("stationary")
            if not tr.moved and sp < c["stationary_speed"]:
                tr.flags.add("stationary")
        self._merge(T)
        self._ghosts()

    def _merge(self, T):
        """Tracks of different sensors that are the same object: fused by covariance intersection (their errors
        share the ego motion and possibly earlier common updates: the correlation is unknown)."""
        c = self.cfg
        tr = self.tracks
        dead = set()
        for i in range(len(tr)):
            for j in range(i + 1, len(tr)):
                a, b = tr[i], tr[j]
                if a.id in dead or b.id in dead or (a.sources & b.sources):
                    continue
                if "ghost" in a.flags or "ghost" in b.flags:
                    continue
                d = (a.x - b.x)[:4]
                S = (a.P + b.P)[:4, :4]
                try:
                    d2 = float(d @ np.linalg.solve(S, d))
                except np.linalg.LinAlgError:
                    continue
                key = b.id
                if d2 > c["merge_gate"] or abs(d[0]) > 6.0 + 0.1 * abs(a.x[X]) or abs(d[1]) > 2.0:
                    a.merge_t.pop(key, None)
                    continue
                t0 = a.merge_t.setdefault(key, T)
                if T - t0 < c["merge_s"]:
                    continue
                # covariance intersection, weight by the traces
                ta, tb = np.trace(a.P), np.trace(b.P)
                w = tb / (ta + tb)
                Pa, Pb = np.linalg.inv(a.P), np.linalg.inv(b.P)
                P = np.linalg.inv(w * Pa + (1.0 - w) * Pb)
                x = P @ (w * Pa @ a.x + (1.0 - w) * Pb @ b.x)
                older, other = (a, b) if (a.designated is not None, -a.t0) >= (b.designated is not None, -b.t0) else (b, a)
                self.take_identity(older, other)
                older.x, older.P = x, 0.5 * (P + P.T)
                older.sources |= other.sources
                older.last_seen.update(other.last_seen)
                older.hits += other.hits
                older.logodds = max(older.logodds, other.logodds)
                older.moved = older.moved or other.moved
                if other.mem.get("vehicle"):
                    older.mem["vehicle"] = True
                if older.state == "tentative" and other.state != "tentative":
                    older.state = other.state
                for k, v in other.mem.items():
                    older.mem.setdefault(k, v)
                older.flags.add("merged")
                dead.add(other.id)
        if dead:
            self.tracks = [t for t in tr if t.id not in dead]

    def take_identity(self, keep, gone):
        """Two tracks found to be one object: `keep` lives on (it may carry a camera's designation), `gone` is
        removed. The object keeps the identity it has been reported under: the lead's id if either is the lead,
        else the older one's. The id of a physical object survives the loss or the arrival of one of its
        sensors."""
        if gone.id == self.lead_id or (keep.id != self.lead_id and gone.t0 < keep.t0):
            keep.id = gone.id
        keep.t0 = min(keep.t0, gone.t0)

    def _ghosts(self):
        """Double-bounce ghosts of a range sensor: about twice the range and twice the closing speed of a nearer
        track on the same bearing, seen by that sensor only."""
        c = self.cfg
        for g in self.tracks:
            g.flags.discard("ghost")
            if g.designated is not None or len(g.sources) != 1:
                continue
            rg, ag = math.hypot(g.x[X], g.x[Y]), math.atan2(g.x[Y], g.x[X])
            for n in self.tracks:
                if n is g or n.state == "tentative":
                    continue
                rn = math.hypot(n.x[X], n.x[Y])
                if rn < 2.0 or rn >= rg:
                    continue
                if abs(rg / rn - 2.0) <= 2.0 * c["ghost_range_tol"] and abs(math.atan2(n.x[Y], n.x[X]) - ag) <= c["ghost_az_rad"] \
                        and abs((g.x[VX] - self._ve) - 2.0 * (n.x[VX] - self._ve)) <= c["ghost_rate_tol"]:
                    g.flags.add("ghost")
                    break

    # ---- the path and the lead -----------------------------------------------------------------------
    def set_path(self, xs, ys):
        """The path in the car frame (y left) as a polyline, or None."""
        self.path = None if xs is None else (np.asarray(xs, float), np.asarray(ys, float))

    def path_offset(self, x, y, w, v):
        """Lateral offset of a point from the path (m, positive left): the plan when there is one, else the
        arc of the ego motion."""
        if self.path is not None and len(self.path[0]) >= 2 and x <= self.path[0][-1] + 5.0:
            return y - float(np.interp(x, self.path[0], self.path[1])), "plan"
        k = w / v if v > 2.0 else 0.0
        return y - 0.5 * k * x * x, "arc"

    def _select_lead(self, ego):
        c = self.cfg
        v_e, w = ego[0], ego[1]
        best = None
        for tr in self.tracks:
            tr.flags.discard("on_path")
            if tr.x[X] <= 0.3 or tr.x[X] > c["path_max_m"]:
                continue
            off, by = self.path_offset(tr.x[X], tr.x[Y], w, v_e)
            tr.mem["path_offset"] = off
            pol = tr.mem.get("policy") or {}
            desig = tr.designated is not None and pol.get("alive")
            # a camera's designation puts the track on the path by the model's own judgement, while the camera
            # has current evidence of it; a track only other sensors still see is judged by the path
            on = (desig and pol.get("fresh", True)) or abs(off) <= c["corridor_m"] + c["corridor_rel"] * tr.x[X]
            if on:
                tr.flags.add("on_path")
            if tr.state == "tentative" or "ghost" in tr.flags or not on:
                continue
            if not (desig and pol.get("fresh", True)):
                if not desig and (not c["lead_radar_only"] or tr.hits < c["lead_min_hits"] or tr.exist < c["lead_min_exist"]):
                    continue
                if not (tr.moved or tr.mem.get("vehicle")):
                    continue               # a return that never moved and that no camera called a vehicle is not
                                           # a vehicle ahead (a stationary-never-moved object, clutter, a ghost)
            if best is None or tr.x[X] < best.x[X]:
                best = tr
        return best

    # ---- one output frame ----------------------------------------------------------------------------
    def frame(self, integ, road, es):
        """Process every queued scan up to the frame's time, then build `tracks` and `lead`."""
        from . import lead as lead_out
        ts = es["ts_mono_ns"]
        T = ts / NS
        sp = es.get("speed") or {}
        known = bool(sp.get("valid"))
        still = (es.get("standstill") or {})
        stands = bool(still.get("value"))
        yr = es.get("yaw_rate") or {}
        e = lead_out.ego_of(es) if known else None
        if e is not None:                  # the speed is exactly 0 once the wheels have stood for the confirm time
            stands = e[1]
        ego_T = (e[0] if e is not None else 0.0, (yr.get("value") or 0.0) if yr.get("valid") else 0.0,
                 0.0 if stands else (sp.get("std") or 0.0), stands)
        self._ve = ego_T[0]
        self._sv = ego_T[2]
        self._sw = float(yr.get("std") or 0.0) if yr.get("valid") else 0.0
        self.es = es

        def ego_fn(t):
            if abs(t - T) < 1e-4:
                return ego_T
            v, w, sv, ok = integ.ego_at(t)
            return (v, w, sv, stands and abs(v) < 0.05)
        if not known:
            # no ego speed: nothing can be carried between frames; tracks are dropped and the outputs say why
            self.queue = [s for s in self.queue if s.t > T]
            had = bool(self.tracks)
            self.tracks = []
            self.t = T
            for m, _ in self.models.values():
                if hasattr(m, "on_invalid"):
                    m.on_invalid(self, ts, "ego speed unknown: %s" % sp.get("reason"))
            self.seq += 1
            return (lead_out.tracks_message(self, es, ts, valid=False, reason="ego speed unknown: %s" % sp.get("reason")),
                    lead_out.lead_message(self, es, road, None, ego_T))
        self.queue.sort(key=lambda s: s.t)
        k = 0
        while k < len(self.queue) and self.queue[k].t <= T + 1e-6:
            s = self.queue[k]
            k += 1
            if s.group in self.faulted:
                continue
            self._apply(s, ego_fn)
        del self.queue[:k]
        if self.t is None or T > self.t:
            self._advance(T, ego_T)
        for src, (m, t) in self.models.items():       # sources that own tracks and were silent or faulted
            if hasattr(m, "on_frame"):
                m.on_frame(self, T, ts, es)
        self._manage(T)
        lead = self._select_lead(ego_T)
        self.seq += 1
        return lead_out.tracks_message(self, es, ts), lead_out.lead_message(self, es, road, lead, ego_T)
