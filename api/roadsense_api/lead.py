"""Lead-vehicle tracker: the api-derived topic `lead`.

modeld's `road.leads[0]` is the network's instantaneous guess, frame by frame. This module turns it into a
track: one estimate per `road` message with distance, relative speed, the lead's own speed and acceleration,
an uncertainty, and the raw vision value next to it (the fusion layer's convention).

    road (leads[0], v_ego) ──┐
    ego (vehicle motion) ────┼─> LeadTracker ─> {"type":"lead", ...}   one per road message
    objects (radar, opt.) ───┤   (everything but `road` is optional: without `ego` the model's own speed
    vehicle (brake, opt.) ───┘    estimate is used, without a radar the camera's distance)

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
            Standing (`standing: true`): ego stands and the lead stands. The estimate stays live (one update per
            frame, never frozen); what changes is the process model: the ego speed is exactly zero, the lead's
            speed is measured with the model's own (very good) standstill speed output, and the distance is a
            random walk scaled so that the filter averages the network's jitter over about `stand_tau_s`;
            frames far from the estimate are down-weighted (Huber), so a wiper pass does not move it. A
            change detector (two-sided CUSUM on the lead speed) ends the standing regime at once when the
            lead moves; moving away while ego still stands = the `departed` cue. Until 2026-11-18 (roadsense
            2.0) this was a `hold` mode that froze the value, which the owner did not want.
    coast   no usable measurement for a short time (probability dropped): prediction only, marked `coast`, with
            `meas_age_s`; it times out (`coast_s`, `stand_coast_s` while standing) and the track is lost.

Range sensors (`radar_fusion`, off by default). A host with a radar or lidar sends its object lists over the
vehicle input API; mapd relays them as `objects` with positions in the camera's frame. The tracker associates
one object with the lead (near the track, or near the camera's lead when there is no track yet; the camera
decides WHICH vehicle is the lead, a radar cannot see lanes). While an object is associated, its range and
closing speed are the measurement (decimetres and tenths of a m/s at any distance, where the camera's distance
scales with an uncalibrated focal length and pitch), `source` names `radar`, and the camera's raw value stays in
`vision`. The ratio of the two distances is learned (`vision_scale`) and applied to the camera's distance for a
minute after the radar lost the object, so that the hand-over back to vision is not a jump. When the camera
loses the lead but the radar still has the object, the track lives on the radar alone for `radar_only_s`.
Without fresh scans nothing of this runs and the tracker is the vision tracker.

Road-model profiles (`PROFILES`, lane L27). The tracker's noise model and life cycle were measured on supercombo
v0.9.7. The v0.11.1 vision model (`road.road_model == "vision"`, the default road model since L27) is a different
instrument: it has no memory, so its presence probability flickers from frame to frame and it finds and drops
distant leads; but its relative speed comes from an image pair 200 ms apart and is 2-4 times steadier than
supercombo's (0.8-1.0 m/s per frame at 50 m and beyond against 2-4 m/s), while its own `v_std` overstates that
scatter. The profile is applied by the message's `road_model` (a user's config still wins) and builds on that:

    relative speed  trusted (rv_scale 0.5, rv_rel 0.01), so the filter's speed carries the distance through gaps;
    distance        its per-frame scatter is taken at 0.4 of the model's x_std (measured: 2-4 m at 50 m and beyond),
                    and it gets a random walk of (3 % of d)^2 per second: the model's distance drifts against its
                    own speed output, and a track that trusted the speed alone stopped following the distance;
    weak evidence   a frame under `drop_prob` but at or above `weak_prob`, whose distance AND relative speed agree
                    with the track's prediction, is a measurement with inflated noise. The track is then not
                    confirmed: mode `coast`, `evidence: "weak"`, and it ends after `weak_coast_s` without a
                    confirming frame. A frame that agrees with nothing is no evidence at all (`evidence: "none"`,
                    prediction only, `coast_s`);
    acquisition     by accumulated evidence (`acquire_score`): the probabilities of successive frames that agree
                    with each other through the reported relative speed add up; a frame that does not agree
                    starts again. One confident frame pair still acquires at once (a cut-in is not delayed);
    re-acquisition  a track lost less than `reacquire_s` ago that the new lead continues (distance predicted with
                    its last speed) keeps its `track_id` (event `reacquired`): the lead was absent in between and
                    the message said so, but it is the same vehicle for a consumer that draws it.

Nothing is held: a track without evidence is predicted for a bounded time, marked, and then dropped.

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
    # standing regime (ego and lead stand; the estimate stays live, docs/lanes/L19-view-channel.md)
    hold_lead_speed=0.35,        # |v_lead| estimate below this and its measurement below hold_v_sigma = the lead stands
    hold_enter_s=0.5,            # both stand for this long -> standing
    hold_min_age_s=1.0,          # track age needed before standing
    hold_v_sigma=0.12, hold_v_scale=0.5,              # sigma of the lead speed while standing: max(floor, scale * v_std)
    cusum_v_k=1.0, cusum_v_h=3.5, cusum_clip=3.0,     # lead-speed CUSUM in sigmas per 0.1 s: slack, threshold, clip
    stand_jerk_psd=0.05,         # (m/s3)^2 s while standing: the lead's speed stays where its measurement puts it
    stand_tau_s=3.0,             # the distance averages the per-frame jitter over about this long (random walk of
                                 # the distance with variance rx^2 dt / tau^2 per step: a live value, never frozen)
    stand_huber=2.5,             # a frame more than this many sigmas off counts with a weight falling as 1 / residual
    stand_coast_s=2.5,           # a track survives this long without a usable measurement while standing
    departed_keep_s=8.0,         # `departed` stays up this long at most (or until ego moves / standing again)
    # derived values
    time_gap_min_speed=1.0,      # m/s; below it time_gap_s is null
    ttc_min_closing=0.5,         # m/s closing speed needed for ttc_s
    camera_to_front_m=None,      # camera -> front bumper; when set, `gap_m` = dist_m - this is added
    # range sensors (`objects` messages of kind radar / lidar, positions in the camera frame)
    radar_fusion=False,          # off: `objects` messages are ignored
    radar_max_age_s=0.3,         # a scan older than this (against the frame time) is not used
    radar_gate_m=3.0, radar_gate_rel=0.25,     # association: |x - lead| within max(m, rel * distance) ...
    radar_gate_lat_m=1.8,        # ... and |y - lead| within this
    radar_sigma_m=0.3, radar_sigma_rel=0.01,   # range noise: hypot(m, rel * distance)
    radar_rate_sigma=0.2,        # closing-speed noise, m/s
    radar_only_s=5.0,            # the track lives on the radar alone this long after the camera lost the lead
    radar_scale_tau_s=5.0, radar_scale_min_n=10, radar_scale_keep_s=60.0,   # camera distance scale learned from the radar
    brake_max_age_s=3.0,         # a brake state older than this is unknown
    # evidence handling for a memoryless road model (off here; the `vision` profile below turns it on)
    weak_prob=None,              # a frame with weak_prob <= prob < drop_prob that agrees with the track is weak evidence
    weak_tol_m=2.0, weak_tol_rel=0.08,     # ... agrees: |x - predicted| within max(m, rel * distance)
    weak_v_tol=2.5,              # ... and |lead speed - the track's| within this, m/s
    weak_inflate=2.5,            # sigma factor of a weak frame
    weak_coast_s=2.5,            # a track ends this long after its last confirming frame (prob >= drop_prob)
    acquire_score=None,          # a track starts when the summed probability of mutually consistent frames reaches this
    acquire_tol_m=1.5, acquire_tol_rel=0.08,   # consistent: |x - (previous x + v_rel dt)| within max(m, rel * x)
    acquire_fast_prob=0.9,       # two consistent frames in a row at or above this start the track at once (cut-in)
    acquire_decay=0.5,           # a frame under drop_prob multiplies the sum by this
    dist_walk_rel=0.0,           # the model's distance drifts against its own speed output: the distance gets a
                                 # random walk of (this * distance)^2 per second on top of the kinematics
    relocate_v_tol=None,         # a replacing hypothesis whose lead speed is within this (m/s) of the track's ...
    relocate_y_tol=1.0,          # ... and whose lateral position is within this (m) is the same vehicle seen at another
                                 # distance: the track jumps to it and keeps its id (event `relocated`)
    reacquire_s=0.0,             # a new track within this long of a lost one, where that one would be now, keeps its id
    reacquire_tol_m=3.0, reacquire_tol_rel=0.15,
)

# Per road model (`road.road_model`), on top of DEFAULT_CFG; a key set by the user's config wins. Tuned on the rides
# krakow-2024, warsaw-storm, gdansk-night-snow, tunnels-s3 and scored on four others (docs/lanes/L27-new-model-default.md).
PROFILES = {
    "vision": dict(
        rv_scale=0.5, rv_rel=0.01, rx_scale=0.4, dist_walk_rel=0.03,
        weak_prob=0.1, weak_coast_s=2.5, coast_s=0.6,   # 0.6 since L29 (owner: release about 0.7 s; was 0.8)
        acquire_score=2.5,
        relocate_v_tol=1.5,
        reacquire_s=3.0,
    ),
}


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
        self.user_cfg = dict(cfg or {})
        self.road_model = None                       # `road.road_model` of the last message (selects the profile)
        self.cfg = dict(DEFAULT_CFG, **self.user_cfg)
        self.ego = collections.deque(maxlen=64)      # (ts_ns, signed speed m/s, standstill, tag, standing since ts)
        self.scans = {}                              # range sensor id -> (ts_ns, objects) of its last scan
        self.brake = None                            # (ts_ns, pressed) from the `vehicle` topic
        self.vis_scale = None                        # [radar / camera distance ratio, samples, ts_ns of the last one]
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
        self.stand = None             # standing regime: {"t0", "vup", "vdn"} (CUSUM statistics), else None
        self.last_meas = None         # frame time of the last measurement that went into the track
        self.both_stand_s = 0.0
        self.vis_stop_s = 0.0
        self.departed = None          # {"ts", "dist_at_stop_m"}
        self.event = None             # last life-cycle event {"kind","ts_mono_ns",...}
        self.sx_bias = 0.0            # the model's own distance std, smoothed (absolute accuracy)
        self.rad = None               # the radar object associated in this frame
        self.radar_only = 0.0         # s the track has lived without the camera
        self.weak_s = 0.0             # s since the last confirming frame (prob >= drop_prob) of the track
        self.evidence = "none"        # this frame: "strong" | "weak" | "none"
        self.acq_score = 0.0          # summed probability of consistent frames (acquire_score)
        self.acq_prev = None          # (ts, x, v_rel, prob) of the last usable frame without a track
        self.ghost = None             # the last lost track: {"x","v","ts","id"} (reacquire_s)

    def _profile(self, road):
        """The road model named in the message selects the profile (a restart of modeld with the other model)."""
        rm = road.get("road_model")
        rm = rm if isinstance(rm, str) else None
        if rm == self.road_model:
            return
        self.road_model = rm
        self.cfg = dict(DEFAULT_CFG, **PROFILES.get(rm, {}))
        self.cfg.update(self.user_cfg)
        if self.kf is not None:
            self._lose(road.get("ts_mono_ns"), "reset")
        self._reset_keep_ego()

    def reset(self):
        self._reset()
        self.ego.clear()
        self.scans.clear()
        self.vis_scale = None

    def on_objects(self, msg):
        """An `objects` message (a scan of a range sensor, relayed by mapd). Used only with `radar_fusion`, from
        a radar or lidar whose positions mapd could put into the camera's frame (sensor and camera mounts known)."""
        if not self.cfg["radar_fusion"] or not isinstance(msg, dict):
            return
        ts, objs = msg.get("ts_mono_ns"), msg.get("objects")
        if msg.get("kind") not in ("radar", "lidar") or msg.get("frame") != "camera" \
                or not isinstance(ts, int) or not isinstance(objs, list):
            return
        self.scans[msg.get("sensor")] = (ts, objs)

    def on_vehicle(self, msg):
        """A `vehicle` message: the brake pedal, passed on in `ego.brake` (a consumer's "the lead moved off and
        you still hold the brake")."""
        b = ((msg or {}).get("controls") or {}).get("brake")
        ts = (msg or {}).get("ts_mono_ns")
        self.brake = (ts, b) if isinstance(b, bool) and isinstance(ts, int) else None

    def _radar(self, ts, ref_x, ref_y, wide):
        """The object of a fresh scan that is the lead at (ref_x, ref_y), or None."""
        c = self.cfg
        gx = max(c["radar_gate_m"], c["radar_gate_rel"] * ref_x) * (2.0 if wide else 1.0)
        best = None
        for sid, (sts, objs) in self.scans.items():
            if abs(ts - sts) > c["radar_max_age_s"] * NS:
                continue
            for o in objs:
                if not isinstance(o, dict):
                    continue
                x, y = o.get("x_m"), o.get("y_m")
                if not _fin(x) or x < 0.5:
                    continue
                y = float(y) if _fin(y) else 0.0
                dx, dy = abs(x - ref_x), abs(y - ref_y)
                if dx > gx or dy > c["radar_gate_lat_m"]:
                    continue
                cost = dx / gx + dy / c["radar_gate_lat_m"]
                if best is None or cost < best["cost"]:
                    v = o.get("vx_mps")
                    if not _fin(v):
                        rr = o.get("range_rate_mps")
                        v = rr / max(x / math.hypot(x, y), 0.5) if _fin(rr) else None
                    best = {"cost": cost, "sensor": sid, "id": o.get("id"), "x": float(x), "y": y,
                            "v_rel": None if v is None else float(v), "age_s": round((ts - sts) / NS, 3)}
        return best

    def _fuse_radar(self, ts, z):
        """Associate a radar object and make it the frame's measurement; returns the measurement to use (the
        camera's `z`, the camera's with the learned scale, the radar's, or None)."""
        c = self.cfg
        self.rad = None
        vs = self.vis_scale
        if vs is not None and (ts - vs[2] > c["radar_scale_keep_s"] * NS or vs[1] < c["radar_scale_min_n"]):
            scale = None
        else:
            scale = vs[0] if vs is not None else None
        seen = z is not None and z["p"] >= c["drop_prob"]
        if self.kf is not None:
            ref = (self.kf.x[0], self.y)
        elif z is not None and z["p"] >= c["acquire_prob"]:
            ref = (z["x"] * (scale or 1.0), z["y"])
        else:
            ref = None
        r = self._radar(ts, ref[0], ref[1], self.kf is None and scale is None) if ref is not None and self.scans else None
        if r is None:
            if seen:
                self.radar_only = 0.0
            if z is not None and scale is not None:      # the camera alone, on the scale the radar taught
                z = dict(z, x=z["x"] * scale, v_rel=None if z["v_rel"] is None else z["v_rel"] * scale)
            return z
        if seen:
            self.radar_only = 0.0
            if z["p"] >= c["acquire_prob"] and z["x"] > 2.0:
                ratio = min(2.0, max(0.5, r["x"] / z["x"]))
                if vs is None or ts - vs[2] > c["radar_scale_keep_s"] * NS:
                    self.vis_scale = [ratio, 1, ts]
                else:
                    dt = max((ts - vs[2]) / NS, 0.0)
                    k = max(1.0 / (vs[1] + 1), min(1.0, dt / c["radar_scale_tau_s"]))
                    self.vis_scale = [vs[0] + k * (ratio - vs[0]), vs[1] + 1, ts]
        elif self.kf is None or self.radar_only > c["radar_only_s"]:
            return z                                     # a radar alone neither starts a track nor keeps one for ever
        self.rad = r
        base = z if seen else {"p": c["acquire_prob"], "y": r["y"], "v": None, "a": None, "sa": 1.0, "v_model": None}
        sig = math.hypot(c["radar_sigma_m"], c["radar_sigma_rel"] * r["x"])
        return dict(base, x=r["x"], sx=sig, sv=c["radar_rate_sigma"],
                    v_rel=r["v_rel"] if r["v_rel"] is not None else (z["v_rel"] if seen else None),
                    r_abs=(sig * sig, c["radar_rate_sigma"] ** 2), radar_only=not seen)

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
        # the model's own speed, to turn the lead's speed into a relative one: supercombo's `v_ego` (what it was fed,
        # the previous run's pose); the vision model's pose of this very run (lead speed and pose come from the
        # same image pair, so their difference is the pair's relative speed)
        vm = (road.get("pose") or {}).get("vx") if self.road_model == "vision" else road.get("v_ego")
        if not _fin(vm):
            vm = road.get("v_ego") if self.road_model == "vision" else (road.get("pose") or {}).get("vx")
        vm = float(vm) if _fin(vm) else None
        return {"p": float(p), "x": x, "y": y if y is not None else 0.0, "v": v, "a": a,
                "sx": sx if sx is not None else 0.1 * x + 0.5, "sv": sv if sv is not None else 1.0,
                "sa": sa if sa is not None else 1.0,
                "v_rel": None if v is None or vm is None else v - vm, "v_model": vm}

    def _noise(self, z):
        c = self.cfg
        if "r_abs" in z:                                 # a radar measurement: its own noise
            return z["r_abs"]
        f = c["low_prob_inflate"] if z["p"] < c["acquire_prob"] else 1.0
        rx = f * max(math.hypot(c["rx_floor"], c["rx_rel"] * z["x"]), c["rx_scale"] * z["sx"])
        rv = f * (max(c["rv_floor"], c["rv_scale"] * z["sv"]) + c["rv_rel"] * z["x"])
        return rx * rx, rv * rv

    # ---- life cycle ----------------------------------------------------------------------------
    def _start(self, z, v_ego, ts, kind):
        rx, rv = self._noise(z)
        vl = (z["v_rel"] + v_ego) if z["v_rel"] is not None else v_ego
        old = (self.kf.x[1], self.y) if self.kf is not None else None
        self.kf = _KF(z["x"], vl, 0.0, 4.0 * rx, 4.0 * rv + 1.0, 1.0)
        self.y = z["y"]
        self.mode = "track"
        self.t0 = ts
        self.miss_s = 0.0
        self.cand = None
        self.stand = None
        self.both_stand_s = 0.0
        self.last_meas = ts
        self.sx_bias = z["sx"]
        self.weak_s = 0.0
        self.evidence = "strong"
        self.acq_score, self.acq_prev = 0.0, None
        g, self.ghost = self.ghost, None
        same = False
        if g is not None and kind == "acquired" and (ts - g["ts"]) <= self.cfg["reacquire_s"] * NS:
            pred = g["x"] + (g["v"] - v_ego) * (ts - g["ts"]) / NS
            same = abs(z["x"] - pred) <= max(self.cfg["reacquire_tol_m"], self.cfg["reacquire_tol_rel"] * max(pred, 0.0))
        if same:
            kind = "reacquired"
        elif kind in ("changed", "cut_in") and old is not None and self.cfg["relocate_v_tol"] is not None \
                and z["v_rel"] is not None and abs(vl - old[0]) <= self.cfg["relocate_v_tol"] \
                and abs(z["y"] - old[1]) <= self.cfg["relocate_y_tol"]:
            kind = "relocated"
        else:
            self.track_id += 1
        self.version += 1
        self.event = {"kind": kind, "ts_mono_ns": ts, "track_id": self.track_id}

    def _lose(self, ts, kind="lost"):
        if kind == "lost" and self.kf is not None and self.cfg["reacquire_s"] > 0 and isinstance(ts, int):
            self.ghost = {"x": self.kf.x[0], "v": self.kf.x[1], "ts": ts}
        else:
            self.ghost = None
        self.evidence = "none"
        self.kf = None
        self.mode = "none"
        self.cand = None
        self.stand = None
        self.acq = 0
        self.both_stand_s = 0.0
        self.version += 1
        self.event = {"kind": kind, "ts_mono_ns": ts, "track_id": self.track_id}

    # ---- one road message ----------------------------------------------------------------------
    def on_road(self, road):
        """Feed one `road` message; returns the `lead` message for it."""
        c = self.cfg
        ts = road.get("ts_mono_ns")
        if not isinstance(ts, int):
            return None
        self._profile(road)
        c = self.cfg
        dt = None if self.ts is None else (ts - self.ts) / NS
        if dt is not None and (dt <= 0.0 or dt > c["max_gap_s"]):
            if self.kf is not None:
                self._lose(ts, "reset")
            self._reset_keep_ego()
            dt = None
        self.ts = ts
        z = zvis = self._measure(road)
        if c["radar_fusion"]:
            if z is None or z["p"] < c["drop_prob"]:
                self.radar_only += dt or 0.0
            z = self._fuse_radar(ts, z)
        vm = zvis["v_model"] if zvis else road.get("v_ego")
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
            self.evidence = "none"
            if c["acquire_score"] is None:
                self.acq = self.acq + 1 if strong else 0
                if strong and self.acq >= c["acquire_frames"]:
                    self._start(z, v_ego, ts, "acquired")
            elif self._accumulate(z if usable else None, ts) and strong:
                self._start(z, v_ego, ts, "acquired")
        elif dt is not None:
            weak = None
            if not usable and z is not None and c["weak_prob"] is not None and z["p"] >= c["weak_prob"] \
                    and "r_abs" not in z:
                weak = z
            self._step_track(z if usable else None, v_ego, stands, src, ts, dt, weak)
        return self._message(road, zvis, v_ego, stands, src, ego_age, ego_tag)

    def _reset_keep_ego(self):
        ego, tid, ver = self.ego, self.track_id, self.version
        self._reset()
        self.ego, self.track_id, self.version = ego, tid, ver

    def _accumulate(self, z, ts):
        """Acquisition by accumulated evidence: True when the probabilities of successive frames that agree with
        each other (the distance moved as the reported relative speed says) sum to `acquire_score`."""
        c = self.cfg
        if z is None:
            self.acq_score *= c["acquire_decay"]
            if self.acq_score < 0.05:
                self.acq_score, self.acq_prev = 0.0, None
            return False
        pv = self.acq_prev
        ok = False
        if pv is not None:
            pred = pv[1] + (pv[2] if pv[2] is not None else 0.0) * (ts - pv[0]) / NS
            ok = abs(z["x"] - pred) <= max(c["acquire_tol_m"], c["acquire_tol_rel"] * z["x"])
        self.acq_score = self.acq_score + z["p"] if ok else z["p"]
        fast = ok and z["p"] >= c["acquire_fast_prob"] and pv[3] >= c["acquire_fast_prob"]
        self.acq_prev = (ts, z["x"], z["v_rel"], z["p"])
        return ok and (fast or self.acq_score >= c["acquire_score"])

    def _step_track(self, z, v_ego, stands, src, ts, dt, weak=None):
        c = self.cfg
        kf = self.kf
        if self.stand is not None and not stands:        # a wheel turns: back to the moving model at once
            self.stand = None
            kf.P[1][1] += 0.25
        st = self.stand
        sig_ve = c["ego_speed_sigma_vehicle"] if src == "vehicle" else c["ego_speed_sigma_vision"]
        kf.predict(dt, v_ego, c["stand_jerk_psd"] if st is not None else c["jerk_psd"], c["accel_tau_s"],
                   0.0 if stands else sig_ve)
        if kf.x[0] < 0.3:
            kf.x[0] = 0.3
        if c["dist_walk_rel"] and st is None:
            kf.P[0][0] += (c["dist_walk_rel"] * kf.x[0]) ** 2 * dt
        if z is None:
            self.mode = "coast"
            self.both_stand_s = 0.0
            self.weak_s += dt
            lim = c["stand_coast_s"] if st is not None else c["coast_s"]
            if weak is not None and self._weak_update(weak, v_ego):
                # not confirmed, but not blind either: the frame agrees with the track in distance and speed
                self.evidence = "weak"
                self.miss_s = 0.0
                if self.weak_s > max(lim, c["weak_coast_s"]):
                    self._lose(ts)
                return
            self.evidence = "none"
            self.miss_s += dt
            if self.miss_s > lim or (c["weak_prob"] is not None and self.weak_s > max(lim, c["weak_coast_s"])):
                self._lose(ts)
            return
        self.evidence = "strong"
        rx, rv = self._noise(z)
        vl = None if z["v_rel"] is None else z["v_rel"] + v_ego
        ended = False
        if st is not None:
            # the motion cue first, before any distance gate: a lead that rolls off makes the distance jump out of
            # the narrow standing filter, and that frame must count as motion, not as an outlier
            sv = max(c["hold_v_sigma"], c["hold_v_scale"] * z["sv"])
            if vl is not None and self._cusum(st, vl / sv, dt, ts):
                st = None                                # the filter is open again (P inflated) for this frame
                ended = True
            else:
                # standing: the distance is a slow random walk (time constant stand_tau_s for the per-frame jitter)
                kf.P[0][0] += rx * dt / (c["stand_tau_s"] ** 2)
        nu, s = kf.innovation(0, z["x"], rx)
        jump = abs(nu) > c["gate_sigma"] * math.sqrt(s) and abs(nu) > max(c["gate_min_m"], c["gate_rel"] * kf.x[0])
        if jump:
            self.evidence = "none"
            self._candidate(z, v_ego, ts, nu)
            if self.kf is not kf or self.cand is None:       # the track was replaced
                return
            self.miss_s += dt
            self.weak_s += dt
            self.both_stand_s = 0.0
            if self.miss_s > max(c["coast_s"], c["change_confirm_s"] + 0.2):
                self._start(z, v_ego, ts, "changed")
            return
        self.cand = None
        self.miss_s = 0.0
        self.weak_s = 0.0
        self.mode = "track"
        self.last_meas = ts
        if st is not None:
            # Huber: a frame far off (a wiper pass, a passing car confusing the network) counts less, not nothing
            k = c["stand_huber"] * math.sqrt(s)
            kf.update(0, z["x"], rx if abs(nu) <= k else rx * (abs(nu) / k) ** 2)
            if vl is not None:
                kf.update(1, vl, sv * sv)
        else:
            kf.update(0, z["x"], rx)
            if vl is not None:
                kf.update(1, vl, rv)
        self.y += min(1.0, dt / (1.0 if st is not None else 0.25)) * (z["y"] - self.y)
        self.sx_bias += min(1.0, dt / 1.0) * (z["sx"] - self.sx_bias)
        if st is not None or ended:
            return                                       # standing, or it ended on this very frame
        # both stand?
        lead_stands = abs(kf.x[1]) < c["hold_lead_speed"] and (vl is None or abs(vl) < c["hold_v_sigma"])
        if stands and lead_stands and (ts - self.t0) >= c["hold_min_age_s"] * NS:
            self.both_stand_s += dt
            if self.both_stand_s >= c["hold_enter_s"]:
                self.stand = {"t0": ts, "vup": 0.0, "vdn": 0.0}
                self.departed = None
        else:
            self.both_stand_s = 0.0

    def _weak_update(self, z, v_ego):
        """A frame under drop_prob: used (noise inflated) only when its distance and its relative speed both agree
        with the track's prediction. Returns whether it was used."""
        c = self.cfg
        kf = self.kf
        if abs(z["x"] - kf.x[0]) > max(c["weak_tol_m"], c["weak_tol_rel"] * kf.x[0]):
            return False
        vl = None if z["v_rel"] is None else z["v_rel"] + v_ego
        if vl is None or abs(vl - kf.x[1]) > c["weak_v_tol"]:
            return False
        rx, rv = self._noise(z)
        f = (c["weak_inflate"] / c["low_prob_inflate"]) ** 2     # _noise already applied low_prob_inflate
        kf.update(0, z["x"], rx * f)
        kf.update(1, vl, rv * f)
        return True

    def _cusum(self, st, rvn, dt, ts):
        """Standing: two-sided CUSUM on the lead speed in sigmas (the model's own motion cue: it stays within a few
        mm/s of zero behind a standing car while its distance wanders by decimetres). True when it fired: the
        standing regime ends, the filter is opened up, and `departed` / `gap_changed` is raised."""
        c = self.cfg
        w = dt / 0.1                                     # statistics per 0.1 s, whatever the frame rate
        rvn = max(-c["cusum_clip"], min(c["cusum_clip"], rvn))
        st["vup"] = max(0.0, st["vup"] + (rvn - c["cusum_v_k"]) * w)
        st["vdn"] = max(0.0, st["vdn"] + (-rvn - c["cusum_v_k"]) * w)
        up, dn = st["vup"] > c["cusum_v_h"], st["vdn"] > c["cusum_v_h"]
        if not (up or dn):
            return False
        kf = self.kf
        self.stand = None
        self.both_stand_s = 0.0
        kf.P[0][0] += 0.05
        kf.P[1][1] += 1.0
        kf.P[2][2] += 1.0
        self.version += 1
        if up:
            self.departed = {"ts": ts, "dist_at_stop_m": kf.x[0]}
            self.event = {"kind": "departed", "ts_mono_ns": ts, "track_id": self.track_id}
        else:
            self.event = {"kind": "gap_changed", "ts_mono_ns": ts, "track_id": self.track_id}
        return True

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

    # ---- output --------------------------------------------------------------------------------
    def _message(self, road, z, v_ego, stands, src, ego_age, ego_tag):
        c = self.cfg
        ts = self.ts
        out = {"type": "lead", "seq": road.get("seq"), "ts_mono_ns": ts, "ts_real_ns": road.get("ts_real_ns"),
               "present": self.kf is not None, "mode": self.mode, "track_id": self.track_id if self.kf else None,
               "source": "vision+ego" if src == "vehicle" else "vision", "road_model": self.road_model}
        rad = self.rad
        if rad is not None and self.kf is not None:
            seen = z is not None and z["p"] >= c["drop_prob"]
            out["source"] = "+".join(["radar"] + (["vision"] if seen else []) + (["ego"] if src == "vehicle" else []))
        out["radar"] = None if rad is None else {
            "sensor": rad["sensor"], "id": rad["id"], "dist_m": round(rad["x"], 2), "lateral_m": round(rad["y"], 2),
            "v_rel_mps": None if rad["v_rel"] is None else round(rad["v_rel"], 2), "age_s": rad["age_s"]}
        vs = self.vis_scale
        out["vision_scale"] = round(vs[0], 3) if vs is not None and vs[1] >= c["radar_scale_min_n"] \
            and ts - vs[2] <= c["radar_scale_keep_s"] * NS else None
        vis = None
        if z is not None:
            vis = {"dist_m": round(z["x"], 2), "dist_std_m": round(z["sx"], 2), "lateral_m": round(z["y"], 2),
                   "v_lead_mps": None if z["v"] is None else round(z["v"], 2),
                   "v_rel_mps": None if z["v_rel"] is None else round(z["v_rel"], 2),
                   "a_lead_mps2": None if z["a"] is None else round(z["a"], 2), "prob": round(z["p"], 3)}
        if self.kf is not None:
            d, vl, al = self.kf.x
            pd, pv = self.kf.P[0][0], self.kf.P[1][1]
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
                standing=self.stand is not None,
                meas_age_s=round((ts - self.last_meas) / NS, 3) if self.last_meas is not None else None,
                evidence=self.evidence,
                pending_change=self.cand is not None,
                prob=None if z is None else round(z["p"], 3))
            if gap is not None:
                out["gap_m"] = round(gap, 2)
        else:
            out.update(age_s=None, dist_m=None, dist_std_m=None, dist_filter_std_m=None, lateral_m=None,
                       v_rel_mps=None, v_rel_std_mps=None, v_lead_mps=None, a_lead_mps2=None, time_gap_s=None,
                       ttc_s=None, standing=False, meas_age_s=None, evidence="none", pending_change=False, prob=None if z is None else round(z["p"], 3))
        dep = self.departed
        out["departed"] = dep is not None
        out["departure"] = None if dep is None else {
            "age_s": round((ts - dep["ts"]) / NS, 2), "dist_at_stop_m": round(dep["dist_at_stop_m"], 2),
            "moved_m": None if self.kf is None else round(self.kf.x[0] - dep["dist_at_stop_m"], 2)}
        ev = self.event
        out["event"] = None if ev is None else dict(ev, age_s=round((ts - ev["ts_mono_ns"]) / NS, 2))
        br = self.brake
        out["ego"] = {"speed_mps": round(v_ego, 3), "standstill": bool(stands), "source": src,
                      "age_s": ego_age, "tag": ego_tag,
                      "brake": br[1] if br is not None and abs(ts - br[0]) <= c["brake_max_age_s"] * NS else None}
        out["vision"] = vis
        return out

    def summary(self, msg):
        """Short dict for /v1/health and `fused.lead`."""
        keys = ("present", "mode", "track_id", "dist_m", "dist_std_m", "lateral_m", "v_rel_mps", "v_lead_mps",
                "a_lead_mps2", "time_gap_s", "ttc_s", "standing", "departed", "source")
        return {k: msg.get(k) for k in keys}
