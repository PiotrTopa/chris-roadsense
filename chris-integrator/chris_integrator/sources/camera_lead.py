"""The road model's lead hypothesis (`road.leads[0]`) as an object source.

The model reports at most one object per frame, the vehicle it takes for the lead, with a probability, a
distance, a lateral offset and a speed. That is a detection with identity semantics of its own, so this
sensor brings its own association and life cycle (`process`) instead of the layer's generic assignment: the
rules below are the lead tracker's, measured over lanes L14, L27, L29 and L32 (noise that follows the range
and the network's own std outputs, acquisition by accumulated evidence, weak evidence, a candidate that
replaces the track when a jump persists, the standing regime with a CUSUM on the lead's speed, re-acquisition
and relocation keeping the id). What changed with the move into the integrator: the filter is the layer's
track ([x, y, vx, vy, ax], the car's yaw rate compensated), other sensors update the same track between
frames, and a track the camera lets go lives on while another sensor sees it.

Road-model profiles (`PROFILES`): the noise model and life cycle were measured on supercombo; the vision model
(`road.road_model == "vision"`) is a different instrument (docs/lanes/L27-new-model-default.md).
"""
import math

import numpy as np

from ..native import round                  # builtins.round, native where built
from ..core.objects import AX, VX, X, Y, Det, Scan, SensorModel, Track
from ..core.registry import NS, Source, fin, all_fin

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
    rx_floor=0.10, rx_rel=0.02, rx_scale=0.2,
    rv_floor=0.15, rv_scale=2.0, rv_rel=0.1,
    ry_floor=0.25, ry_scale=0.5,                      # lateral: max(floor, scale * the model's y_std)
    low_prob_inflate=2.0,        # sigma factor for a frame between drop_prob and acquire_prob
    # consistency gate (track mode)
    gate_sigma=4.0, gate_min_m=0.6, gate_rel=0.12,    # a jump: > gate_sigma sigmas and > max(gate_min_m, gate_rel * d)
    cut_in_confirm_s=0.15,       # a nearer hypothesis replaces the track after this long
    change_confirm_s=0.5,        # a farther one after this long
    candidate_tol_m=1.0, candidate_tol_rel=0.1,       # frames of one candidate agree within this
    # ego input
    ego_stop_confirm_s=0.3,      # vehicle: standstill flag held this long = ego stands
    # standing regime (ego and lead stand; the estimate stays live)
    hold_lead_speed=0.35, hold_enter_s=0.5, hold_min_age_s=1.0,
    hold_v_sigma=0.12, hold_v_scale=0.5,
    cusum_v_k=1.0, cusum_v_h=3.5, cusum_clip=3.0,
    stand_tau_s=3.0, stand_huber=2.5, stand_coast_s=2.5,
    departed_keep_s=8.0,
    # range sensors on the same track
    radar_fresh_s=0.3,           # another sensor's hit this recent "has" the track
    radar_only_s=5.0,            # the camera's designation outlives its own evidence this long while a radar has it
    scale_tau_s=5.0, scale_min_n=10, scale_keep_s=60.0,   # camera distance scale learned from a range sensor
    adopt_gate_m=3.0, adopt_gate_rel=0.25, adopt_gate_lat_m=1.8,   # an existing track this near the camera's lead is it
    adopt_gate_v=2.5,            # ... and its speed over ground agrees with the camera's within this (+ 2 sigma of the track)
    # evidence handling for a memoryless road model (off here; the `vision` profile below turns it on)
    weak_prob=None, weak_tol_m=2.0, weak_tol_rel=0.08, weak_v_tol=2.5, weak_inflate=2.5, weak_coast_s=2.5,
    acquire_score=None, acquire_tol_m=1.5, acquire_tol_rel=0.08, acquire_fast_prob=0.9, acquire_decay=0.5,
    dist_walk_rel=0.0,
    relocate_v_tol=None, relocate_y_tol=1.0,
    reacquire_s=0.0, reacquire_tol_m=3.0, reacquire_tol_rel=0.15,
)

# Per road model, on top of DEFAULT_CFG; a key set by the user's config wins (docs/lanes/L27, L29).
PROFILES = {
    "vision": dict(
        rv_scale=0.5, rv_rel=0.01, rx_scale=0.4, dist_walk_rel=0.03,
        weak_prob=0.1, weak_coast_s=1.5, coast_s=0.6,
        acquire_score=2.5,
        relocate_v_tol=1.5,
        reacquire_s=3.0,
    ),
}


def _first(seq):
    if isinstance(seq, (list, tuple)) and seq and fin(seq[0]):
        return float(seq[0])
    return None


def measure(road, road_model):
    """The frame's lead hypothesis as a dict, or None (y stays in the device frame: positive right)."""
    leads = road.get("leads")
    if not isinstance(leads, list) or not leads or not isinstance(leads[0], dict) or road.get("valid") is False:
        return None
    ld = leads[0]
    p, x = ld.get("prob"), _first(ld.get("x"))
    if not fin(p) or x is None or x <= 0.0:
        return None
    v, a, y = _first(ld.get("v")), _first(ld.get("a")), _first(ld.get("y"))
    sx, sv, sa, sy = _first(ld.get("x_std")), _first(ld.get("v_std")), _first(ld.get("a_std")), _first(ld.get("y_std"))
    # the model's own speed, to turn the lead's speed into a relative one: supercombo's `v_ego` (what it was fed);
    # the vision model's pose of this very run (lead speed and pose come from the same image pair)
    vm = (road.get("pose") or {}).get("vx") if road_model == "vision" else road.get("v_ego")
    if not fin(vm):
        vm = road.get("v_ego") if road_model == "vision" else (road.get("pose") or {}).get("vx")
    vm = float(vm) if fin(vm) else None
    return {"p": float(p), "x": x, "y": y if y is not None else 0.0, "v": v, "a": a,
            "sx": sx if sx is not None else 0.1 * x + 0.5, "sv": sv if sv is not None else 1.0,
            "sa": sa if sa is not None else 1.0, "sy": sy,
            "v_rel": None if v is None or vm is None else v - vm, "v_model": vm}


class CameraLeadModel(SensorModel):
    name = "camera_lead"
    initiates = False

    def __init__(self, src_id, user_cfg=None):
        self.src = src_id
        self.user_cfg = dict(user_cfg or {})
        self.road_model = None
        self.cfg = dict(DEFAULT_CFG, **self.user_cfg)
        self.vis_scale = None                        # [range sensor / camera distance ratio, samples, ts_ns]
        self.version = 0
        self._reset()

    # the layer calls process() per scan; `self.process` must be truthy for the layer
    def _reset(self):
        self.cur = None               # the track this camera designates as the lead
        self.mode = "none"
        self.ts = None
        self.t0 = None
        self.acq = 0
        self.miss_s = 0.0
        self.cand = None
        self.stand = None
        self.last_meas = None
        self.both_stand_s = 0.0
        self.departed = None
        self.event = None
        self.sx_bias = 0.0
        self.radar_only = 0.0
        self.weak_s = 0.0
        self.evidence = "none"
        self.acq_score = 0.0
        self.acq_prev = None
        self.ghost = None
        self.z = None                 # the frame's raw hypothesis (for the `vision` block of the lead message)
        self.other = None             # another sensor's hit on the current track in this frame: (source, age_s)

    def _profile(self, layer, rm, ts):
        rm = rm if isinstance(rm, str) else None
        if rm == self.road_model:
            return
        self.road_model = rm
        self.cfg = dict(DEFAULT_CFG, **PROFILES.get(rm, {}))
        self.cfg.update(self.user_cfg)
        if self.cur is not None:
            self._lose(layer, ts, "reset")
        self._reset()

    # ---- helpers -------------------------------------------------------------------------------------
    def _noise(self, z):
        c = self.cfg
        f = c["low_prob_inflate"] if z["p"] < c["acquire_prob"] else 1.0
        rx = f * max(math.hypot(c["rx_floor"], c["rx_rel"] * z["x"]), c["rx_scale"] * z["sx"])
        rv = f * (max(c["rv_floor"], c["rv_scale"] * z["sv"]) + c["rv_rel"] * z["x"])
        return rx * rx, rv * rv

    def _ry(self, z):
        c = self.cfg
        return max(c["ry_floor"], c["ry_scale"] * z["sy"] if z.get("sy") is not None else 0.0) ** 2

    def _has_other(self, tr, t):
        """Another sensor's hit on the track within radar_fresh_s: (source, age_s) or None."""
        best = None
        for s, ts in tr.last_seen.items():
            if s != self.src and t - ts <= self.cfg["radar_fresh_s"] and (best is None or ts > best[1]):
                best = (s, ts)
        return None if best is None else (best[0], round(t - best[1], 3))

    def _start(self, layer, z, v_ego, ts, kind):
        c = self.cfg
        rx, rv = self._noise(z)
        vl = (z["v_rel"] + v_ego) if z["v_rel"] is not None else v_ego
        cur = self.cur
        old = (cur.x[VX], -cur.x[Y]) if cur is not None else None
        x0 = np.array([z["x"], -z["y"], vl, 0.0, 0.0])
        P0 = np.diag([4.0 * rx, 4.0 * self._ry(z), 4.0 * rv + 1.0, 1.0, 1.0])
        g, self.ghost = self.ghost, None
        same = False
        if g is not None and kind == "acquired" and (ts - g["ts"]) <= c["reacquire_s"] * NS:
            pred = g["x"] + (g["v"] - v_ego) * (ts - g["ts"]) / NS
            same = abs(z["x"] - pred) <= max(c["reacquire_tol_m"], c["reacquire_tol_rel"] * max(pred, 0.0))
        # (a track another sensor holds is never moved to the camera's distance: the camera lets it go and takes
        # what is there, which is the same track when the jump was the camera's own range error)
        relocate = (not same and self.other is None
                    and kind in ("changed", "cut_in") and old is not None and c["relocate_v_tol"] is not None
                    and z["v_rel"] is not None and abs(vl - old[0]) <= c["relocate_v_tol"]
                    and abs(z["y"] - old[1]) <= c["relocate_y_tol"])
        T = ts / NS
        if relocate:
            kind = "relocated"
            cur.x, cur.P = x0, P0                          # the same vehicle seen at another distance: id kept
            tr = cur
        else:
            if cur is not None:
                self._release(cur)
            # a track another sensor holds where the camera's lead is: the camera designates that one (also on a
            # re-acquisition: the track the camera lost may have been carried by a radar meanwhile)
            tr = self._adopt(layer, z, T, vl=vl if z["v_rel"] is not None else None, vl_var=rv,
                             prefer=g["id"] if same else None)
            if tr is None:
                tid = g["id"] if same and all(t.id != g["id"] for t in layer.tracks) else layer.new_id()
                tr = Track(tid, x0, P0, T, self.src, 0.9)
                layer.tracks.append(tr)
            else:
                tr.sources.add(self.src)
            if same:
                kind = "reacquired"
        tr.designated = self.src
        tr.mem["vehicle"] = True                           # the road model calls it a vehicle
        tr.state = "confirmed"
        tr.standing = False
        tr.last_seen[self.src] = T
        tr.logodds = max(tr.logodds, math.log(0.9 / 0.1))
        tr.mem["policy"] = {"alive": True, "state": "confirmed"}
        self.cur = tr
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
        self.version += 1
        self.event = {"kind": kind, "ts_mono_ns": ts, "track_id": tr.id}

    def _adopt(self, layer, z, T, exclude=None, vl=None, vl_var=0.0, prefer=None):
        """A track another sensor already holds where the camera's lead is: the camera designates that one. vl: the
        lead's speed over ground by the camera (None: unknown), vl_var its variance; a track whose own speed
        disagrees beyond both uncertainties is another object (a stationary return beside a moving lead). prefer: a
        track id that wins any tie (the track the camera lost)."""
        c = self.cfg
        gx = max(c["adopt_gate_m"], c["adopt_gate_rel"] * z["x"])
        best = None
        for tr in layer.tracks:
            if tr is exclude or tr.designated is not None or tr.state == "tentative" or "ghost" in tr.flags:
                continue
            dx, dy = abs(tr.x[X] - z["x"]), abs(tr.x[Y] + z["y"])
            if dx > gx or dy > c["adopt_gate_lat_m"]:
                continue
            if vl is not None and abs(tr.x[VX] - vl) > c["adopt_gate_v"] + 2.0 * math.sqrt(max(tr.P[VX, VX], 0.0) + vl_var):
                continue
            cost = dx / gx + dy / c["adopt_gate_lat_m"] - (1.0 if prefer is not None and tr.id == prefer else 0.0)
            if best is None or cost < best[0]:
                best = (cost, tr)
        return None if best is None else best[1]

    def _release(self, tr):
        """The camera lets a track go: it lives on only while another sensor sees it."""
        tr.designated = None
        tr.standing = False
        tr.mem["policy"] = {"alive": False}
        tr.last_seen.pop(self.src, None)
        tr.sources.discard(self.src)

    def _lose(self, layer, ts, kind="lost"):
        cur = self.cur
        if kind == "lost" and cur is not None and self.cfg["reacquire_s"] > 0 and isinstance(ts, int):
            self.ghost = {"x": cur.x[X], "v": cur.x[VX], "ts": ts, "id": cur.id}
        else:
            self.ghost = None
        tid = cur.id if cur is not None else None
        if cur is not None:
            self._release(cur)
        self.evidence = "none"
        self.cur = None
        self.mode = "none"
        self.cand = None
        self.stand = None
        self.acq = 0
        self.both_stand_s = 0.0
        self.version += 1
        self.event = {"kind": kind, "ts_mono_ns": ts, "track_id": tid}

    def _reset_keep(self):
        ver, vs = self.version, self.vis_scale
        rm, cfg = self.road_model, self.cfg
        self._reset()
        self.version, self.vis_scale = ver, vs

    # ---- hooks of the layer --------------------------------------------------------------------------
    def on_invalid(self, layer, ts, reason):
        """The products are unknown this frame (the ego speed is unknown, or the camera is not believable)."""
        had = self.cur is not None
        if had:
            self._lose(layer, ts, "invalid")
        ev = self.event
        self._reset_keep()
        self.ts = ts
        self.event = ev if had or (ev is not None and ev.get("kind") == "invalid") else None
        layer.set_path(None, None)

    def on_frame(self, layer, T, ts, es):
        if self.ts != ts:                 # no scan of this camera went into the frame: it is silent or faulted
            vis = ((es.get("sources") or {}).get("vision") or {})
            if vis.get("state") == "fault":
                self.on_invalid(layer, ts, vis.get("reason"))
            elif self.ts is not None and ts - self.ts > self.cfg["max_gap_s"] * NS:
                if self.cur is not None:
                    self._lose(layer, ts, "reset")
                self._reset_keep()
                layer.set_path(None, None)
            self.z = None

    # ---- one frame -----------------------------------------------------------------------------------
    def process(self, layer, scan, ego):
        from ..core.lead import ego_of
        raw = scan.raw
        ts = scan.ts_ns
        self._profile(layer, raw.get("road_model"), ts)
        c = self.cfg
        dt = None if self.ts is None else (ts - self.ts) / NS
        if dt is not None and (dt <= 0.0 or dt > c["max_gap_s"]):
            if self.cur is not None:
                self._lose(layer, ts, "reset")
            self._reset_keep()
            dt = None
        self.ts = ts
        T = ts / NS
        plan = raw.get("plan")
        layer.set_path(*(plan if plan is not None else (None, None)))
        if self.cur is not None and self.cur not in layer.tracks:      # merged away or dropped by the layer
            self.cur = next((t for t in layer.tracks if t.designated == self.src), None)
            if self.cur is None:
                self._lose(layer, ts)
        z = self.z = raw.get("z")
        v_ego, stands, src = ego_of(layer.es, c["ego_stop_confirm_s"])[:3]
        cur = self.cur
        self.other = self._has_other(cur, T) if cur is not None else None
        if cur is not None and self.other is None:
            # the camera's own track and another sensor's track of the same vehicle (they started apart by the
            # camera's range error): the camera designates the other one, which keeps this track's id
            o = self._adopt(layer, {"x": float(cur.x[X]), "y": -float(cur.x[Y])}, T, exclude=cur, vl=float(cur.x[VX]),
                            vl_var=float(cur.P[VX, VX]))
            if o is not None:
                layer.tracks.remove(cur)
                layer.take_identity(o, cur)
                o.mem["vehicle"] = True
                o.designated, o.state, o.standing = self.src, "confirmed", cur.standing
                o.sources.add(self.src)
                o.last_seen[self.src] = cur.last_seen.get(self.src, T)
                o.mem["policy"] = cur.mem.get("policy")
                cur = self.cur = o
                self.other = self._has_other(cur, T)
        # the camera's distance on the scale a range sensor taught (docs/lanes/L14-lead.md)
        vs = self.vis_scale
        scale = vs[0] if vs is not None and ts - vs[2] <= c["scale_keep_s"] * NS and vs[1] >= c["scale_min_n"] else None
        usable = z is not None and z["p"] >= c["drop_prob"]
        strong = z is not None and z["p"] >= c["acquire_prob"]
        if cur is not None and self.other is not None and strong and z["x"] > 2.0:
            ratio = min(2.0, max(0.5, cur.x[X] / z["x"]))
            if vs is None or ts - vs[2] > c["scale_keep_s"] * NS:
                self.vis_scale = [ratio, 1, ts]
            else:
                k = max(1.0 / (vs[1] + 1), min(1.0, max((ts - vs[2]) / NS, 0.0) / c["scale_tau_s"]))
                self.vis_scale = [vs[0] + k * (ratio - vs[0]), vs[1] + 1, ts]
        zm = z
        if z is not None and scale is not None:
            zm = dict(z, x=z["x"] * scale, v_rel=None if z["v_rel"] is None else z["v_rel"] * scale)
        if self.departed and (not stands or ts - self.departed["ts"] > c["departed_keep_s"] * NS):
            self.departed = None
        if self.other is not None:
            if usable:
                self.radar_only = 0.0
            else:
                self.radar_only += dt or 0.0
        else:
            self.radar_only = 0.0
        if cur is None:
            self.evidence = "none"
            if c["acquire_score"] is None:
                self.acq = self.acq + 1 if strong else 0
                if strong and self.acq >= c["acquire_frames"]:
                    self._start(layer, zm, v_ego, ts, "acquired")
            elif self._accumulate(zm if usable else None, ts) and strong:
                self._start(layer, zm, v_ego, ts, "acquired")
        elif dt is not None:
            weak = None
            if not usable and zm is not None and c["weak_prob"] is not None and zm["p"] >= c["weak_prob"]:
                weak = zm
            self._step(layer, zm if usable else None, v_ego, stands, ts, dt, weak)
        cur = self.cur
        if cur is not None:
            cur.mem["policy"] = {"alive": True, "state": "confirmed" if self.mode == "track" else "coast",
                                 "fresh": self.evidence != "none" or self.other is None}
            cur.standing = self.stand is not None

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

    def _step(self, layer, z, v_ego, stands, ts, dt, weak=None):
        """One frame of the designated track (the layer has predicted it to the frame's time)."""
        c = self.cfg
        tr = self.cur
        T = ts / NS
        if self.stand is not None and not stands:        # a wheel turns: back to the moving model at once
            self.stand = None
            tr.P[VX, VX] += 0.25
        st = self.stand
        if c["dist_walk_rel"] and st is None:
            tr.P[X, X] += (c["dist_walk_rel"] * tr.x[X]) ** 2 * dt
        held = self.other is not None                    # another sensor has the track in this frame
        if z is None:
            self.mode = "coast" if not held else "track"
            self.both_stand_s = 0.0
            self.weak_s += dt
            lim = c["stand_coast_s"] if st is not None else c["coast_s"]
            if weak is not None and self._weak_update(weak, v_ego):
                # not confirmed, but not blind either: the frame agrees with the track in distance and speed
                self.evidence = "weak"
                self.miss_s = 0.0
                if self.weak_s > max(lim, c["weak_coast_s"]) and not (held and self.radar_only <= c["radar_only_s"]):
                    self._lose(layer, ts)
                return
            self.evidence = "none"
            self.miss_s += dt
            if held and self.radar_only <= c["radar_only_s"]:
                return                                   # the range sensor carries it; the camera's turn comes back
            if self.miss_s > lim or (c["weak_prob"] is not None and self.weak_s > max(lim, c["weak_coast_s"])):
                self._lose(layer, ts)
            return
        self.evidence = "strong"
        rx, rv = self._noise(z)
        vl = None if z["v_rel"] is None else z["v_rel"] + v_ego
        ended = False
        sv = None
        if st is not None:
            # the motion cue first, before any distance gate: a lead that rolls off makes the distance jump out of
            # the narrow standing filter, and that frame must count as motion, not as an outlier
            sv = max(c["hold_v_sigma"], c["hold_v_scale"] * z["sv"])
            if vl is not None and self._cusum(st, vl / sv, dt, ts):
                st = None                                # the filter is open again (P inflated) for this frame
                ended = True
            else:
                # standing: the distance is a slow random walk (time constant stand_tau_s for the per-frame jitter)
                tr.P[X, X] += rx * dt / (c["stand_tau_s"] ** 2)
        nu, s = tr.innovation(X, z["x"], rx)
        jump = abs(nu) > c["gate_sigma"] * math.sqrt(s) and abs(nu) > max(c["gate_min_m"], c["gate_rel"] * tr.x[X])
        if jump:
            self.evidence = "none"
            self._candidate(layer, z, v_ego, ts, nu)
            if self.cur is not tr or self.cand is None:      # the track was replaced
                return
            self.miss_s += dt
            self.weak_s += dt
            self.both_stand_s = 0.0
            if self.miss_s > max(c["coast_s"], c["change_confirm_s"] + 0.2):
                self._start(layer, z, v_ego, ts, "changed")
            return
        self.cand = None
        self.miss_s = 0.0
        self.weak_s = 0.0
        self.mode = "track"
        self.last_meas = ts
        tr.last_seen[self.src] = T
        tr.hits += 1
        if st is not None:
            # Huber: a frame far off (a wiper pass, a passing car confusing the network) counts less, not nothing
            k = c["stand_huber"] * math.sqrt(s)
            tr.update_scalar(X, z["x"], rx if abs(nu) <= k else rx * (abs(nu) / k) ** 2)
            if vl is not None:
                tr.update_scalar(VX, vl, sv * sv)
        else:
            tr.update_scalar(X, z["x"], rx)
            if vl is not None:
                tr.update_scalar(VX, vl, rv)
        tr.update_scalar(Y, -z["y"], self._ry(z) * (4.0 if st is not None else 1.0))
        self.sx_bias += min(1.0, dt / 1.0) * (z["sx"] - self.sx_bias)
        if st is not None or ended:
            return                                       # standing, or it ended on this very frame
        # both stand?
        lead_stands = abs(tr.x[VX]) < c["hold_lead_speed"] and (vl is None or abs(vl) < c["hold_v_sigma"])
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
        tr = self.cur
        if abs(z["x"] - tr.x[X]) > max(c["weak_tol_m"], c["weak_tol_rel"] * tr.x[X]):
            return False
        vl = None if z["v_rel"] is None else z["v_rel"] + v_ego
        if vl is None or abs(vl - tr.x[VX]) > c["weak_v_tol"]:
            return False
        rx, rv = self._noise(z)
        f = (c["weak_inflate"] / c["low_prob_inflate"]) ** 2     # _noise already applied low_prob_inflate
        tr.update_scalar(X, z["x"], rx * f)
        tr.update_scalar(VX, vl, rv * f)
        return True

    def _cusum(self, st, rvn, dt, ts):
        """Standing: two-sided CUSUM on the lead speed in sigmas (the model's own motion cue). True when it fired:
        the standing regime ends, the filter is opened up, and `departed` / `gap_changed` is raised."""
        c = self.cfg
        w = dt / 0.1                                     # statistics per 0.1 s, whatever the frame rate
        rvn = max(-c["cusum_clip"], min(c["cusum_clip"], rvn))
        st["vup"] = max(0.0, st["vup"] + (rvn - c["cusum_v_k"]) * w)
        st["vdn"] = max(0.0, st["vdn"] + (-rvn - c["cusum_v_k"]) * w)
        up, dn = st["vup"] > c["cusum_v_h"], st["vdn"] > c["cusum_v_h"]
        if not (up or dn):
            return False
        tr = self.cur
        self.stand = None
        self.both_stand_s = 0.0
        tr.P[X, X] += 0.05
        tr.P[VX, VX] += 1.0
        tr.P[AX, AX] += 1.0
        self.version += 1
        if up:
            self.departed = {"ts": ts, "dist_at_stop_m": float(tr.x[X])}
            self.event = {"kind": "departed", "ts_mono_ns": ts, "track_id": tr.id}
        else:
            self.event = {"kind": "gap_changed", "ts_mono_ns": ts, "track_id": tr.id}
        return True

    def _candidate(self, layer, z, v_ego, ts, nu):
        """A measurement that does not fit the track: collect it; replace the track when it persists."""
        c = self.cfg
        cd = self.cand
        vl = (z["v_rel"] + v_ego) if z["v_rel"] is not None else self.cur.x[VX]
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
            self._start(layer, z, v_ego, ts, "cut_in" if cd["closer"] else "changed")

    # ---- what the lead message shows of this sensor ----------------------------------------------------
    def lead_fields(self, tr, ts):
        c = self.cfg
        vs = self.vis_scale
        return {"mode": self.mode, "standing": self.stand is not None, "evidence": self.evidence,
                "pending_change": self.cand is not None, "age_s": round((ts - self.t0) / NS, 2) if self.t0 else None,
                "meas_age_s": round((ts - self.last_meas) / NS, 3) if self.last_meas is not None else None,
                "sx_bias": self.sx_bias,
                "vision_scale": round(vs[0], 3) if vs is not None and vs[1] >= c["scale_min_n"]
                and ts - vs[2] <= c["scale_keep_s"] * NS else None}


class CameraLeadSource(Source):
    id = "camera_lead"
    group = "vision"
    rank = 1
    types = ("road",)
    stale_s = 0.6
    counts = False               # the vision source counts the `road` messages of the group
    info = "the road model's lead hypothesis"

    def __init__(self, cfg=None, **kw):
        super().__init__(**kw)
        self.model = CameraLeadModel(self.id, cfg)

    def reset(self):
        self.model._reset()

    def scan(self, msg, ctx):
        ts = msg.get("ts_mono_ns")
        rm = msg.get("road_model")
        z = measure(msg, rm if isinstance(rm, str) else None)
        plan = None
        pl = msg.get("plan")
        if isinstance(pl, dict) and msg.get("valid") is not False:
            xs, ys = pl.get("x"), pl.get("y")
            if isinstance(xs, list) and isinstance(ys, list) and len(xs) == len(ys) and len(xs) >= 2 \
                    and all_fin(xs) and all_fin(ys):
                n = len(xs)
                k = 1
                while k < n and xs[k] > xs[k - 1]:
                    k += 1
                if k >= 2:
                    plan = (xs[:k], [-y for y in ys[:k]])
        dets = [] if z is None else [Det([z["x"], -z["y"]], [1.0, 1.0], {"prob": z["p"]})]
        return Scan(self.id, self.group, ts / NS, dets, self.model, ts, {"z": z, "road_model": rm, "plan": plan})
