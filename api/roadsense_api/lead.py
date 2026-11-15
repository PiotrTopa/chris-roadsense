"""Lead-vehicle tracker: the api-derived topic `lead`.

modeld's `road.leads[0]` is the network's instantaneous guess, frame by frame. This module turns it into a
track: one estimate per `road` message with distance, relative speed, the lead's own speed and acceleration,
an uncertainty, and the raw vision value next to it (the fusion layer's convention).

    road (leads[0], v_ego) ──┐
                             ├─> LeadTracker ─> {"type":"lead", ...}   one per road message
    ego (vehicle motion) ────┘   (ego is optional: without it the model's own speed estimate is used)

Model: state [d, v_lead, a_lead] (distance camera -> lead rear along x, the lead's speed and acceleration
over the ground along the ego heading); the ego speed is a known input, so

    d' = v_lead - v_ego        v_lead' = a_lead        a_lead' = -a_lead / tau + white noise

and "we both stand" (v_ego = 0 from the wheels, v_lead = 0) is a different hypothesis from "the gap closes".
Measurements per frame: the distance x and the relative speed (v - the model's own v_ego), each with a noise
that follows the range and the network's own std outputs.

Modes (`mode` in the message):
    track   Kalman filter as above. A measurement that is not consistent with the track (a jump that the
            dynamics cannot produce) is not averaged in: the track coasts and a candidate collects the new
            hypothesis; when it persists the track is replaced (a nearer one fast: cut-in; a farther one
            slower), otherwise it was an outlier.
    hold    ego stands and the lead stands: the true gap is constant, so the estimate is frozen and a change
            detector (two-sided CUSUM on the lead speed, plus a slow check of the distance) watches for
            consistent evidence that it moved. Lead moving away while ego still stands = the `departed` cue.
    coast   no usable measurement for a short time (probability dropped): prediction only.

No I/O and no wall clock in here; times come from the messages (CLOCK_MONOTONIC ns).
"""
import collections
import math

NS = 1e9
DEFAULT_CFG = dict(
    # track life cycle
    acquire_prob=0.5,            # leads[0].prob at or above this starts / feeds a track
    drop_prob=0.3,               # below this the frame carries no lead
    acquire_frames=2,            # consecutive frames needed to start a track
    coast_s=0.6,                 # a track survives this long without a usable measurement
    hold_coast_s=2.5,            # ... while standing behind a standing lead
    max_gap_s=1.0,               # road messages further apart than this restart the tracker
    # measurement noise (sigma per frame), from the replayed rides (docs/lanes/L14-lead.md):
    #   distance        max(hypot(rx_floor, rx_rel * d), rx_scale * the model's x_std)
    #   relative speed  max(rv_floor, rv_scale * the model's v_std) + rv_rel * d
    # The model's lead acceleration output is not used as a measurement: on the rides it made the track lag
    # by 130-200 ms while the gap closed (it is not the derivative of the model's own lead speed).
    rx_floor=0.10, rx_rel=0.02, rx_scale=0.2,
    rv_floor=0.15, rv_scale=2.0, rv_rel=0.1,
    low_prob_inflate=2.0,        # sigma factor for a frame between drop_prob and acquire_prob
    # process noise
    jerk_psd=30.0,               # (m/s3)^2 s: how fast the lead's acceleration may change
    accel_tau_s=3.0,             # the lead's acceleration decays with this time constant
    ego_speed_sigma_vehicle=0.05, ego_speed_sigma_vision=0.4,   # m/s, uncertainty of the ego speed input
    # consistency gate (track mode)
    gate_sigma=4.0, gate_min_m=0.6, gate_rel=0.12,    # a jump: > gate_sigma sigmas and > max(gate_min_m, gate_rel * d)
    cut_in_confirm_s=0.15,       # a nearer hypothesis replaces the track after this long
    change_confirm_s=0.5,        # a farther one after this long
    candidate_tol_m=1.0, candidate_tol_rel=0.1,       # frames of one candidate agree within this
    # ego input
    ego_max_age_s=0.5,           # vehicle motion older than this (against the frame time) is not used
    ego_stop_confirm_s=0.3,      # vehicle: standstill flag held this long = ego stands
    vision_stop_speed=0.15,      # vision only: the model's v_ego below this ...
    vision_stop_confirm_s=1.0,   # ... for this long = ego stands
    vision_go_speed=0.8,         # ... and above this it moves at once (between the two: leaky)
    # hold mode
    hold_lead_speed=0.35,        # |v_lead| estimate below this and its measurement below hold_v_sigma = the lead stands
    hold_enter_s=0.5,            # both stand for this long -> hold
    hold_min_age_s=1.0,          # track age needed before a hold
    hold_v_sigma=0.12, hold_v_scale=0.5,              # sigma of the lead speed while holding: max(floor, scale * v_std)
    cusum_v_k=1.0, cusum_v_h=3.5, cusum_clip=3.0,     # lead-speed CUSUM in sigmas per 0.1 s: slack, threshold, clip
    hold_fast_tau_s=1.5, hold_break_m=0.6, hold_break_xstd=1.3,   # distance backup: 1.5 s mean of the residual
                                 # beyond max(hold_break_m, hold_break_xstd * the model's x_std)
    hold_mean_tau_s=4.0,         # the held value's slow mean
    hold_sigma_floor_m=0.15,     # per-frame clip of that mean, in sigmas of the distance (floor)
    hold_settle_s=2.0,           # the published value follows the mean this long after the hold began ...
    hold_relatch_m=0.3, hold_relatch_xstd=0.6,        # ... afterwards only when they differ by max(m, xstd * x_std)
    departed_keep_s=8.0,         # `departed` stays up this long at most (or until ego moves / a new hold)
    # derived values
    time_gap_min_speed=1.0,      # m/s; below it time_gap_s is null
    ttc_min_closing=0.5,         # m/s closing speed needed for ttc_s
    camera_to_front_m=None,      # camera -> front bumper; when set, `gap_m` = dist_m - this is added
)


def _fin(v):
    return isinstance(v, (int, float)) and not isinstance(v, bool) and math.isfinite(v)


def _first(seq):
    if isinstance(seq, (list, tuple)) and seq and _fin(seq[0]):
        return float(seq[0])
    return None


class _KF:
    """3-state Kalman filter [d, v_lead, a_lead] with scalar measurement updates (pure Python, 3x3)."""

    def __init__(self, d, v, a, pd, pv, pa):
        self.x = [d, v, a]
        self.P = [[pd, 0.0, 0.0], [0.0, pv, 0.0], [0.0, 0.0, pa]]

    def predict(self, dt, v_ego, jerk_psd, tau, sig_ve):
        d, v, a = self.x
        rho = math.exp(-dt / tau)
        self.x = [d + (v - v_ego) * dt + 0.5 * a * dt * dt, v + a * dt, a * rho]
        F = ((1.0, dt, 0.5 * dt * dt), (0.0, 1.0, dt), (0.0, 0.0, rho))
        P = self.P
        FP = [[sum(F[i][k] * P[k][j] for k in range(3)) for j in range(3)] for i in range(3)]
        Pn = [[sum(FP[i][k] * F[j][k] for k in range(3)) for j in range(3)] for i in range(3)]
        q = jerk_psd
        dt2, dt3, dt4, dt5 = dt * dt, dt ** 3, dt ** 4, dt ** 5
        Q = ((dt5 / 20.0, dt4 / 8.0, dt3 / 6.0), (dt4 / 8.0, dt3 / 3.0, dt2 / 2.0), (dt3 / 6.0, dt2 / 2.0, dt))
        for i in range(3):
            for j in range(3):
                Pn[i][j] += q * Q[i][j]
        Pn[0][0] += (sig_ve * dt) ** 2
        self.P = Pn

    def innovation(self, i, z, r):
        return z - self.x[i], self.P[i][i] + r

    def update(self, i, z, r):
        P = self.P
        nu, s = z - self.x[i], P[i][i] + r
        k = [P[0][i] / s, P[1][i] / s, P[2][i] / s]
        self.x = [self.x[j] + k[j] * nu for j in range(3)]
        row = list(P[i])
        self.P = [[P[a][b] - k[a] * row[b] for b in range(3)] for a in range(3)]
        for a in range(3):                       # keep it symmetric and positive
            for b in range(a):
                m = 0.5 * (self.P[a][b] + self.P[b][a])
                self.P[a][b] = self.P[b][a] = m
            self.P[a][a] = max(self.P[a][a], 1e-9)


class LeadTracker:
    def __init__(self, cfg=None):
        self.cfg = dict(DEFAULT_CFG, **(cfg or {}))
        self.ego = collections.deque(maxlen=64)      # (ts_ns, signed speed m/s, standstill, tag, standing since ts)
        self.track_id = 0
        self.version = 0                             # bumps on every track start / change / loss
        self._reset()

    # ---- inputs ------------------------------------------------------------------------------
    def _reset(self):
        self.kf = None
        self.mode = "none"
        self.ts = None                # frame time of the last road message
        self.t0 = None                # track start
        self.y = 0.0
        self.acq = 0
        self.miss_s = 0.0
        self.cand = None              # pending other hypothesis: {"x","v","y","t0","ts","n","closer"}
        self.hold = None              # hold state
        self.both_stand_s = 0.0
        self.vis_stop_s = 0.0
        self.departed = None          # {"ts", "dist_at_stop_m"}
        self.event = None             # last life-cycle event {"kind","ts_mono_ns",...}
        self.sx_bias = 0.0            # the model's own distance std, smoothed (absolute accuracy)

    def reset(self):
        self._reset()
        self.ego.clear()

    def on_ego(self, msg):
        """A mapd `ego` message (the vehicle motion a position feeder sent). Ignored when malformed."""
        ts, v = msg.get("ts_mono_ns"), msg.get("speed_mps")
        if not isinstance(ts, int) or not _fin(v):
            return
        if self.ego and ts <= self.ego[-1][0]:
            if ts < self.ego[-1][0] - 2 * NS:        # the clock went back: a restarted feeder
                self.ego.clear()
            else:
                return
        still = bool(msg.get("standstill")) and float(v) == 0.0
        since = None if not still else (self.ego[-1][4] if self.ego and self.ego[-1][4] is not None else ts)
        self.ego.append((ts, float(v), still, msg.get("tag"), since))

    def _ego_at(self, ts, road):
        """Ego speed at the frame time: (speed, stands, source, age_s, tag)."""
        c = self.cfg
        prev = nxt = None
        for s in reversed(self.ego):
            if s[0] <= ts:
                prev = s
                break
            nxt = s
        near = min((abs(ts - s[0]) for s in (prev, nxt) if s is not None), default=None)
        if near is not None and near <= c["ego_max_age_s"] * NS:
            a, b = prev or nxt, nxt or prev
            v = a[1] + (b[1] - a[1]) * (ts - a[0]) / (b[0] - a[0]) if b[0] > a[0] else a[1]
            # stands: the standstill flag has been up for the confirmation time at the frame, and still is
            since = a[4]
            stands = since is not None and b[4] is not None and ts - since >= c["ego_stop_confirm_s"] * NS
            return (0.0 if stands else v), stands, "vehicle", round(near / NS, 3), a[3]
        v = road.get("v_ego")
        if not _fin(v):
            v = (road.get("pose") or {}).get("vx")
        v = float(v) if _fin(v) else 0.0
        return v, self.vis_stop_s >= c["vision_stop_confirm_s"], "vision", None, None

    # ---- measurement ---------------------------------------------------------------------------
    def _measure(self, road):
        leads = road.get("leads")
        if not isinstance(leads, list) or not leads or not isinstance(leads[0], dict) or road.get("valid") is False:
            return None
        ld = leads[0]
        p, x = ld.get("prob"), _first(ld.get("x"))
        if not _fin(p) or x is None or x <= 0.0:
            return None
        v, a, y = _first(ld.get("v")), _first(ld.get("a")), _first(ld.get("y"))
        sx, sv, sa = _first(ld.get("x_std")), _first(ld.get("v_std")), _first(ld.get("a_std"))
        vm = road.get("v_ego")
        if not _fin(vm):
            vm = (road.get("pose") or {}).get("vx")
        vm = float(vm) if _fin(vm) else None
        return {"p": float(p), "x": x, "y": y if y is not None else 0.0, "v": v, "a": a,
                "sx": sx if sx is not None else 0.1 * x + 0.5, "sv": sv if sv is not None else 1.0,
                "sa": sa if sa is not None else 1.0,
                "v_rel": None if v is None or vm is None else v - vm, "v_model": vm}

    def _noise(self, z):
        c = self.cfg
        f = c["low_prob_inflate"] if z["p"] < c["acquire_prob"] else 1.0
        rx = f * max(math.hypot(c["rx_floor"], c["rx_rel"] * z["x"]), c["rx_scale"] * z["sx"])
        rv = f * (max(c["rv_floor"], c["rv_scale"] * z["sv"]) + c["rv_rel"] * z["x"])
        return rx * rx, rv * rv

    # ---- life cycle ----------------------------------------------------------------------------
    def _start(self, z, v_ego, ts, kind):
        rx, rv = self._noise(z)
        vl = (z["v_rel"] + v_ego) if z["v_rel"] is not None else v_ego
        self.kf = _KF(z["x"], vl, 0.0, 4.0 * rx, 4.0 * rv + 1.0, 1.0)
        self.y = z["y"]
        self.mode = "track"
        self.t0 = ts
        self.miss_s = 0.0
        self.cand = None
        self.hold = None
        self.both_stand_s = 0.0
        self.sx_bias = z["sx"]
        self.track_id += 1
        self.version += 1
        self.event = {"kind": kind, "ts_mono_ns": ts, "track_id": self.track_id}

    def _lose(self, ts, kind="lost"):
        self.kf = None
        self.mode = "none"
        self.cand = None
        self.hold = None
        self.acq = 0
        self.both_stand_s = 0.0
        self.version += 1
        self.event = {"kind": kind, "ts_mono_ns": ts, "track_id": self.track_id}

    def _leave_hold(self, z, v_ego, ts, moved):
        """Back to the filter. `moved`: +1 the lead went away, -1 it came nearer, 0 ego started to move."""
        h = self.hold
        self.hold = None
        self.mode = "track"
        self.both_stand_s = 0.0
        kf = self.kf
        if moved and z is not None:
            rx, rv = self._noise(z)
            vl = (z["v_rel"] + v_ego) if z["v_rel"] is not None else 0.0
            # the residuals that tripped the detector are real motion: restart on the measurement, wide
            kf.x = [z["x"], vl, 0.0]
            kf.P = [[rx + 0.25, 0.0, 0.0], [0.0, rv + 1.0, 0.0], [0.0, 0.0, 2.0]]
        else:
            kf.x = [h["mean"], 0.0, 0.0]
            kf.P = [[max(h["var"], 0.01), 0.0, 0.0], [0.0, 0.25, 0.0], [0.0, 0.0, 1.0]]
        return h

    # ---- one road message ----------------------------------------------------------------------
    def on_road(self, road):
        """Feed one `road` message; returns the `lead` message for it."""
        c = self.cfg
        ts = road.get("ts_mono_ns")
        if not isinstance(ts, int):
            return None
        dt = None if self.ts is None else (ts - self.ts) / NS
        if dt is not None and (dt <= 0.0 or dt > c["max_gap_s"]):
            if self.kf is not None:
                self._lose(ts, "reset")
            self._reset_keep_ego()
            dt = None
        self.ts = ts
        z = self._measure(road)
        vm = z["v_model"] if z else road.get("v_ego")
        # vision-only standstill: the model's speed under the threshold for the confirmation time. Leaky, so a
        # single-frame blip of the model's speed (a wiper pass) does not end it; a real start does within 0.3 s.
        if not _fin(vm) or vm >= c["vision_go_speed"]:
            self.vis_stop_s = 0.0
        elif vm < c["vision_stop_speed"]:
            self.vis_stop_s = min(self.vis_stop_s + (dt or 0.0), c["vision_stop_confirm_s"] + 0.3)
        else:
            self.vis_stop_s = max(0.0, self.vis_stop_s - 3.0 * (dt or 0.0))
        v_ego, stands, src, ego_age, ego_tag = self._ego_at(ts, road)
        usable = z is not None and z["p"] >= c["drop_prob"]
        strong = z is not None and z["p"] >= c["acquire_prob"]
        if self.departed and (not stands or ts - self.departed["ts"] > c["departed_keep_s"] * NS):
            self.departed = None

        if self.kf is None:
            self.acq = self.acq + 1 if strong else 0
            if strong and self.acq >= c["acquire_frames"]:
                self._start(z, v_ego, ts, "acquired")
        elif dt is not None:
            if self.mode == "hold":
                self._step_hold(z if usable else None, v_ego, stands, src, ts, dt)
            else:
                self._step_track(z if usable else None, v_ego, stands, src, ts, dt)
        return self._message(road, z, v_ego, stands, src, ego_age, ego_tag)

    def _reset_keep_ego(self):
        ego, tid, ver = self.ego, self.track_id, self.version
        self._reset()
        self.ego, self.track_id, self.version = ego, tid, ver

    def _step_track(self, z, v_ego, stands, src, ts, dt):
        c = self.cfg
        kf = self.kf
        sig_ve = c["ego_speed_sigma_vehicle"] if src == "vehicle" else c["ego_speed_sigma_vision"]
        kf.predict(dt, v_ego, c["jerk_psd"], c["accel_tau_s"], 0.0 if stands else sig_ve)
        if kf.x[0] < 0.3:
            kf.x[0] = 0.3
        if z is None:
            self.miss_s += dt
            self.mode = "coast"
            self.both_stand_s = 0.0
            if self.miss_s > c["coast_s"]:
                self._lose(ts)
            return
        rx, rv = self._noise(z)
        nu, s = kf.innovation(0, z["x"], rx)
        jump = abs(nu) > c["gate_sigma"] * math.sqrt(s) and abs(nu) > max(c["gate_min_m"], c["gate_rel"] * kf.x[0])
        if jump:
            self._candidate(z, v_ego, ts, nu)
            if self.kf is not kf or self.cand is None:       # the track was replaced
                return
            self.miss_s += dt
            self.both_stand_s = 0.0
            if self.miss_s > max(c["coast_s"], c["change_confirm_s"] + 0.2):
                self._start(z, v_ego, ts, "changed")
            return
        self.cand = None
        self.miss_s = 0.0
        self.mode = "track"
        kf.update(0, z["x"], rx)
        if z["v_rel"] is not None:
            kf.update(1, z["v_rel"] + v_ego, rv)
        self.y += min(1.0, dt / 0.25) * (z["y"] - self.y)
        self.sx_bias += min(1.0, dt / 1.0) * (z["sx"] - self.sx_bias)
        # both stand?
        lead_stands = abs(kf.x[1]) < c["hold_lead_speed"] and \
            (z["v_rel"] is None or abs(z["v_rel"] + v_ego) < c["hold_v_sigma"])
        if stands and lead_stands and (ts - self.t0) >= c["hold_min_age_s"] * NS:
            self.both_stand_s += dt
            if self.both_stand_s >= c["hold_enter_s"]:
                self.mode = "hold"
                self.hold = {"t0": ts, "mean": kf.x[0], "out": kf.x[0], "n": max(2.0, rx / max(kf.P[0][0], 1e-6)),
                             "var": kf.P[0][0], "fast": 0.0, "vup": 0.0, "vdn": 0.0, "miss": 0.0}
                kf.x[1] = kf.x[2] = 0.0
                self.departed = None
        else:
            self.both_stand_s = 0.0

    def _candidate(self, z, v_ego, ts, nu):
        """A measurement that does not fit the track: collect it; replace the track when it persists."""
        c = self.cfg
        cd = self.cand
        vl = (z["v_rel"] + v_ego) if z["v_rel"] is not None else self.kf.x[1]
        if cd is not None:
            pred = cd["x"] + (cd["v"] - v_ego) * (ts - cd["ts"]) / NS
            if abs(z["x"] - pred) > max(c["candidate_tol_m"], c["candidate_tol_rel"] * pred):
                cd = None
        if cd is None:
            cd = self.cand = {"x": z["x"], "v": vl, "t0": ts, "ts": ts, "n": 1, "closer": nu < 0.0}
        else:
            cd.update(x=z["x"], v=vl, ts=ts, n=cd["n"] + 1)
        need = c["cut_in_confirm_s"] if cd["closer"] else c["change_confirm_s"]
        if cd["n"] >= 2 and (ts - cd["t0"]) >= need * NS:
            self._start(z, v_ego, ts, "cut_in" if cd["closer"] else "changed")

    def _step_hold(self, z, v_ego, stands, src, ts, dt):
        c = self.cfg
        h = self.hold
        if not stands:                                   # ego moves: the filter takes over from the held value
            self._leave_hold(z, v_ego, ts, 0)
            self._step_track(z, v_ego, stands, src, ts, dt)
            return
        if z is None:
            h["miss"] += dt
            if h["miss"] > c["hold_coast_s"]:
                self._lose(ts)
            return
        h["miss"] = 0.0
        w = dt / 0.1                                     # detector statistics are per 0.1 s, whatever the frame rate
        # 1. the lead's speed (the model's own motion cue: it stays within a few mm/s of zero behind a standing
        #    car while its distance wanders by decimetres): two-sided CUSUM in sigmas
        vl = None if z["v_rel"] is None else z["v_rel"] + v_ego
        if vl is not None:
            sv = max(c["hold_v_sigma"], c["hold_v_scale"] * z["sv"])
            rvn = max(-c["cusum_clip"], min(c["cusum_clip"], vl / sv))
            h["vup"] = max(0.0, h["vup"] + (rvn - c["cusum_v_k"]) * w)
            h["vdn"] = max(0.0, h["vdn"] + (-rvn - c["cusum_v_k"]) * w)
        # 2. the distance itself, as a backup for motion the speed output does not show (a creep): a 0.7 s
        #    mean of the residual, clipped per frame so that an outlier frame (a wiper pass) cannot trip it.
        #    The threshold grows with the model's own x_std: behind a standing car the model's distance can
        #    wander by more than a metre for seconds (a vehicle passing in the next lane, wipers) while its
        #    speed output and its x_std say that nothing moved and that it is unsure.
        r = z["x"] - h["out"]
        brk = max(c["hold_break_m"], c["hold_break_xstd"] * self.sx_bias)
        h["fast"] += min(1.0, dt / c["hold_fast_tau_s"]) * (max(-2.0 * brk, min(2.0 * brk, r)) - h["fast"])
        up = h["vup"] > c["cusum_v_h"] or h["fast"] > brk
        dn = h["vdn"] > c["cusum_v_h"] or h["fast"] < -brk
        if up or dn:
            held = h["out"]
            by_speed = h["vup"] > c["cusum_v_h"] or h["vdn"] > c["cusum_v_h"]
            self._leave_hold(z, v_ego, ts, 1 if up else -1)
            self.version += 1
            if up and (by_speed or (vl is not None and vl > c["hold_v_sigma"])):
                self.departed = {"ts": ts, "dist_at_stop_m": held}
                self.event = {"kind": "departed", "ts_mono_ns": ts, "track_id": self.track_id}
            else:
                self.event = {"kind": "gap_changed", "ts_mono_ns": ts, "track_id": self.track_id}
            return
        # 3. the gap is constant: a slow mean (1/n at first, then a time constant) absorbs the model's wander;
        #    the published value follows it while settling, afterwards only when they differ by hold_relatch_m
        h["n"] += 1.0
        g = max(1.0 / h["n"], min(1.0, dt / c["hold_mean_tau_s"]))
        sig = max(c["hold_sigma_floor_m"], math.sqrt(self._noise(z)[0]))
        h["mean"] += g * max(-2.0 * sig, min(2.0 * sig, z["x"] - h["mean"]))
        h["var"] = sig * sig * max(g, 0.02)
        if (ts - h["t0"]) < c["hold_settle_s"] * NS or abs(h["mean"] - h["out"]) > max(c["hold_relatch_m"], c["hold_relatch_xstd"] * self.sx_bias):
            h["fast"] += h["out"] - h["mean"]
            h["out"] = h["mean"]
        self.y += min(1.0, dt / 1.0) * (z["y"] - self.y)
        self.sx_bias += min(1.0, dt / 1.0) * (z["sx"] - self.sx_bias)
        self.kf.x = [h["mean"], 0.0, 0.0]

    # ---- output --------------------------------------------------------------------------------
    def _message(self, road, z, v_ego, stands, src, ego_age, ego_tag):
        c = self.cfg
        ts = self.ts
        out = {"type": "lead", "seq": road.get("seq"), "ts_mono_ns": ts, "ts_real_ns": road.get("ts_real_ns"),
               "present": self.kf is not None, "mode": self.mode, "track_id": self.track_id if self.kf else None,
               "source": "vision+ego" if src == "vehicle" else "vision"}
        vis = None
        if z is not None:
            vis = {"dist_m": round(z["x"], 2), "dist_std_m": round(z["sx"], 2), "lateral_m": round(z["y"], 2),
                   "v_lead_mps": None if z["v"] is None else round(z["v"], 2),
                   "v_rel_mps": None if z["v_rel"] is None else round(z["v_rel"], 2),
                   "a_lead_mps2": None if z["a"] is None else round(z["a"], 2), "prob": round(z["p"], 3)}
        if self.kf is not None:
            d, vl, al = self.kf.x
            pd, pv = self.kf.P[0][0], self.kf.P[1][1]
            if self.mode == "hold":
                d, vl, al = self.hold["out"], 0.0, 0.0
                pd, pv = self.hold["var"], 0.0
            v_rel = vl - v_ego
            gap = d - c["camera_to_front_m"] if c["camera_to_front_m"] is not None else None
            ref = gap if gap is not None else d
            out.update(
                age_s=round((ts - self.t0) / NS, 2),
                dist_m=round(d, 2),
                # 68 %: the filter's own uncertainty and the network's stated distance std (its absolute accuracy,
                # which no amount of filtering removes) in quadrature
                dist_std_m=round(math.sqrt(max(pd, 0.0) + self.sx_bias ** 2), 2),
                dist_filter_std_m=round(math.sqrt(max(pd, 0.0)), 3),
                lateral_m=round(self.y, 2),
                v_rel_mps=round(v_rel, 2), v_rel_std_mps=round(math.sqrt(max(pv, 0.0)), 2),
                v_lead_mps=round(vl, 2), a_lead_mps2=round(al, 2),
                time_gap_s=round(max(ref, 0.0) / v_ego, 2) if v_ego >= c["time_gap_min_speed"] else None,
                ttc_s=round(max(ref, 0.0) / -v_rel, 1) if v_rel <= -c["ttc_min_closing"] else None,
                standing=self.mode == "hold",
                pending_change=self.cand is not None,
                prob=None if z is None else round(z["p"], 3))
            if gap is not None:
                out["gap_m"] = round(gap, 2)
        else:
            out.update(age_s=None, dist_m=None, dist_std_m=None, dist_filter_std_m=None, lateral_m=None,
                       v_rel_mps=None, v_rel_std_mps=None, v_lead_mps=None, a_lead_mps2=None, time_gap_s=None,
                       ttc_s=None, standing=False, pending_change=False, prob=None if z is None else round(z["p"], 3))
        dep = self.departed
        out["departed"] = dep is not None
        out["departure"] = None if dep is None else {
            "age_s": round((ts - dep["ts"]) / NS, 2), "dist_at_stop_m": round(dep["dist_at_stop_m"], 2),
            "moved_m": None if self.kf is None else round(self.kf.x[0] - dep["dist_at_stop_m"], 2)}
        ev = self.event
        out["event"] = None if ev is None else dict(ev, age_s=round((ts - ev["ts_mono_ns"]) / NS, 2))
        out["ego"] = {"speed_mps": round(v_ego, 3), "standstill": bool(stands), "source": src,
                      "age_s": ego_age, "tag": ego_tag}
        out["vision"] = vis
        return out

    def summary(self, msg):
        """Short dict for /v1/health and `fused.lead`."""
        keys = ("present", "mode", "track_id", "dist_m", "dist_std_m", "lateral_m", "v_rel_mps", "v_lead_mps",
                "a_lead_mps2", "time_gap_s", "ttc_s", "standing", "departed", "source")
        return {k: msg.get(k) for k in keys}
