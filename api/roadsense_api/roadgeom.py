"""Fused road geometry: the geometry inside `view.road` (until roadsense 2.0 the api-derived topic `roadgeom`).

One picture of the road ahead in the car's frame (x forward from the camera, y right, metres), for a consumer
that draws it: where the ego lane runs, from the camera as far as the camera knows it and from the map beyond,
the side roads of the junctions ahead, and a roundabout as a ring with its arms. Every part says where it
comes from and how far it can be trusted.

    lanes (tracked lane lines, lanes.py) ─┐
    road (the model's planned path) ──────┤
    ego (vehicle speed, yaw rate,         ├─> RoadGeometryFusion ─> roadview.py ─> view["road"]
         turn signal when the host has it)┤
    map (ahead.geometry, 2 per second) ───┘

How the parts are joined

* Near the car the camera decides: the centre of the ego lane is the middle of the two tracked lane lines
  (one line and the lane width, or the model's planned path, when they are missing), as far as the tracker's
  own uncertainty stays small (`vision.reach_m`).
* The map's geometry arrives twice a second in the frame of the matched position. Between two messages it is
  carried along by the distance driven (vehicle speed when a host feeds it, else the model's): the car is
  taken to be on the most likely path, that far along it, heading along it.
* What the map cannot know (where in the carriageway the car is; a few metres and degrees of position and
  heading error) is taken from the camera: the offset and the angle between the camera's lane centre and the
  map's path over the near range are fitted, low-passed and applied to all map geometry. When the two shapes
  do not fit each other (the car is not on the path the map expects, the map is wrong, the camera is lost),
  or the car turns away from the path (yaw rate against the path's curvature), the map part is withheld and
  the message says why. It comes back after a second of agreement.
* Beyond the camera's reach the centre line continues on the map's path, with the path's confidence falling
  at every junction where the way on is a guess. At a roundabout the camera's part ends at the entry (lane
  lines are not painted through a ring).
* Driver intent (optional): a turn signal relayed by the host (`ego.turn_signal`) selects the side road or
  roundabout exit on that side at the next junction; the centre line then ends there and the message names the
  chosen branch. Without the signal the way on is the map's most likely path and is marked as such.

Without a map (no position source, no pack, a poor match) the message carries the camera's part alone:
`source: "vision"`, `map.used: false` and the reason.

No I/O and no wall clock in here; times come from the messages (CLOCK_MONOTONIC ns).
"""
import math

import numpy as np

NS = 1e9

DEFAULT_CFG = dict(
    # camera part
    lane_width_m=3.4,             # used with a single lane line until the tracker has measured one
    reach_std_m=0.9,              # the camera's centre line is used while its sigma stays below this
    reach_min_m=15.0, reach_max_m=100.0,
    plan_std=(0.4, 0.03),         # sigma of the planned path as a centre line: a + b * x
    one_line_extra_std_m=0.3,
    # map part
    map_max_age_s=1.6, map_min_match=0.5, map_max_hacc_m=25.0,
    advance_max_m=40.0,           # a map message is not carried further than this
    fit_x=(4.0, 50.0), fit_min_points=4,
    fit_max_dy_m=7.0, fit_max_dyaw_deg=9.0,
    fit_rms_m=(0.5, 0.012),       # shapes agree while the residual stays below a + b * reach
    fit_tau_s=0.8,
    agree_s=1.0, disagree_s=0.5,  # the map part appears after this long in agreement, goes after this long without
    hold_m=150.0, hold_s=45.0,    # the last fit is kept this far / long without a usable camera centre (junction, ring)
    off_path_deg=35.0,            # the car's heading against the path's direction
    blend_m=15.0,                 # camera -> map over this distance before the camera's reach
    ring_margin_m=3.0,            # the camera's part ends this far before a ring
    on_ring_reach_m=10.0,
    ring_slide_m=14.0,            # on a ring the car is slid along the path by at most this, to where its heading fits
    path_max_m=400.0,
    branch_ahead_m=220.0, branch_behind_m=20.0,
    # driver intent
    intent_max_age_s=1.0, intent_ahead_m=150.0,
)


def _fin(v):
    return isinstance(v, (int, float)) and not isinstance(v, bool) and math.isfinite(v)


class RoadGeometryFusion:
    def __init__(self, cfg=None):
        self.cfg = dict(DEFAULT_CFG)
        if cfg:
            self.cfg.update(cfg)
        self.reset()

    def reset(self):
        self.ts = None                 # time of the last frame (s)
        self.odo = 0.0                 # distance driven (m), integrated per frame
        self.hdg = 0.0                 # heading driven (rad, positive right)
        self.v = 0.0
        self.width = None
        self.ego = None                # last ego message
        self.map = None                # prepared map geometry (dict) or None
        self.map_why = "no_map"
        self.fit = None                # (dy, dyaw) low-passed
        self.fit_ts = None             # time and odometer of the last fit that used the camera
        self.fit_odo = 0.0
        self.agree_for = 0.0
        self.disagree_for = 0.0
        self.used = False
        self.lane_right = False

    # ---- inputs ----------------------------------------------------------------------------------
    def on_ego(self, msg):
        if isinstance(msg, dict) and _fin(msg.get("ts_mono_ns")):
            self.ego = msg

    def on_map(self, msg):
        """Keep the geometry of a usable map message, with the odometer and heading it belongs to."""
        c = self.cfg
        # a message that cannot be used does not take the last usable one away: that one ages out by itself
        # (map_max_age_s), so a single doubtful match in a junction does not blank the picture
        if not isinstance(msg, dict) or not _fin(msg.get("ts_mono_ns")):
            self.map_why = "no_map"
            return
        ahead = msg.get("ahead") or {}
        geom = ahead.get("geometry")
        match = msg.get("match") or {}
        pos = msg.get("position") or {}
        if not msg.get("available"):
            self.map_why = msg.get("reason") or "unavailable"
            return
        if not isinstance(geom, dict) or not isinstance(geom.get("path"), dict):
            self.map_why = "no_geometry"
            return
        conf = match.get("confidence")
        if not _fin(conf) or conf < c["map_min_match"]:
            self.map_why = "low_match_confidence"
            return
        if _fin(pos.get("hacc_m")) and pos["hacc_m"] > c["map_max_hacc_m"]:
            self.map_why = "position_accuracy"
            return
        p = geom["path"]
        try:
            s = np.asarray(p["s"], dtype=float)
            xy = np.stack([np.asarray(p["x"], dtype=float), np.asarray(p["y"], dtype=float)], axis=1)
        except (KeyError, TypeError, ValueError):
            self.map_why = "no_geometry"
            return
        if len(s) < 3 or xy.shape != (len(s), 2) or not np.isfinite(xy).all():
            self.map_why = "no_geometry"
            return
        th = np.unwrap(np.arctan2(np.gradient(xy[:, 1], s), np.gradient(xy[:, 0], s)))
        # the car's heading in the map frame (whose x axis is the matched road bearing): the position filter's
        # heading when it is known, else along the road
        rel = None
        if _fin(pos.get("heading_deg")) and _fin(match.get("bearing_deg")):
            rel = math.radians((pos["heading_deg"] - match["bearing_deg"] + 180.0) % 360.0 - 180.0)
            if abs(rel) > math.radians(60.0):
                rel = None
        ts = msg["ts_mono_ns"] / NS
        lag = 0.0 if self.ts is None else max(-1.0, min(1.0, ts - self.ts))
        jn = [(j["dist_m"], j["confidence"], j.get("kind"), bool(j.get("minor")))
              for j in ahead.get("junctions") or () if _fin(j.get("dist_m")) and _fin(j.get("confidence"))]
        branches = []
        for b in geom.get("branches") or ():
            try:
                pts = np.asarray(b["pts"], dtype=float)
            except (KeyError, TypeError, ValueError):
                continue
            if pts.ndim != 2 or pts.shape[1] != 2 or len(pts) < 2:
                continue
            d = b.get("dist_m")
            side = None
            if _fin(d):
                t = float(np.interp(d, s, th))
                v = pts[-1] - pts[0]
                cr = math.cos(t) * v[1] - math.sin(t) * v[0]          # > 0: leaves to the right
                side = "right" if cr > 0 else "left"
            branches.append((b, pts, side))
        ring = geom.get("ring") if isinstance(geom.get("ring"), dict) else None
        if ring is not None and not (isinstance(ring.get("centre"), list) and _fin(ring.get("radius_m"))):
            ring = None
        self.map = {"ts": ts, "odo": self.odo + self.v * lag, "hdg": self.hdg, "s": s, "xy": xy, "th": th, "rel": rel,
                    "junctions": jn, "branches": branches, "ring": ring, "lanes": geom.get("lanes") or [],
                    "match": float(conf), "hacc": pos.get("hacc_m") if _fin(pos.get("hacc_m")) else None,
                    "state": msg.get("state"), "synthetic": bool((msg.get("input") or {}).get("synthetic")),
                    "view": ahead.get("junction_view") if isinstance(ahead.get("junction_view"), dict) else None}
        self.map_why = None

    # ---- camera part -----------------------------------------------------------------------------
    def _vision(self, lanes, road):
        """(x, y, sigma, source) of the ego-lane centre from the camera, or None."""
        c = self.cfg
        if isinstance(lanes, dict) and isinstance(lanes.get("x"), list):
            x = np.asarray(lanes["x"], dtype=float)
            ln = lanes.get("lines") or []
            if len(ln) >= 3:
                le, ri = ln[1], ln[2]
                w = lanes.get("lane_width_m")
                if _fin(w) and 2.2 <= w <= 5.0:
                    self.width = w if self.width is None else self.width + 0.1 * (w - self.width)
                w = self.width or c["lane_width_m"]
                lp, rp = bool(le.get("present")), bool(ri.get("present"))
                if lp and rp:
                    y = 0.5 * (np.asarray(le["y"], dtype=float) + np.asarray(ri["y"], dtype=float))
                    sd = 0.5 * np.hypot(np.asarray(le["y_std"], dtype=float), np.asarray(ri["y_std"], dtype=float))
                    return x, y, sd, "lanes"
                if lp or rp:
                    one = le if lp else ri
                    y = np.asarray(one["y"], dtype=float) + (0.5 * w if lp else -0.5 * w)
                    return x, y, np.asarray(one["y_std"], dtype=float) + c["one_line_extra_std_m"], "line"
        plan = (road or {}).get("plan") if isinstance(road, dict) else None
        if isinstance(plan, dict) and isinstance(plan.get("x"), list) and isinstance(plan.get("y"), list):
            try:
                px, py = np.asarray(plan["x"], dtype=float), np.asarray(plan["y"], dtype=float)
            except (TypeError, ValueError):
                return None
            ok = np.isfinite(px) & np.isfinite(py)
            px, py = px[ok], py[ok]
            if len(px) >= 4 and px[-1] - px[0] >= 8.0 and (np.diff(px) > 0).all():
                a, b = c["plan_std"]
                return px, py, a + b * px, "plan"
        return None

    def _reach(self, x, sd):
        c = self.cfg
        bad = np.nonzero((sd > c["reach_std_m"]) | (x > c["reach_max_m"]))[0]
        r = float(x[-1]) if len(bad) == 0 else float(x[max(bad[0] - 1, 0)])
        return max(min(r, c["reach_max_m"], float(x[-1])), min(c["reach_min_m"], float(x[-1])))

    # ---- one frame -------------------------------------------------------------------------------
    def on_lanes(self, lanes, road=None):
        """One `lanes` message (and the `road` message it was made from) -> the geometry for that frame
        (roadview.py puts it into `view.road`)."""
        c = self.cfg
        ts_ns = (lanes or {}).get("ts_mono_ns")
        if not _fin(ts_ns):
            return None
        ts = ts_ns / NS
        ego_l = (lanes.get("ego") or {})
        v = ego_l.get("speed_mps") if _fin(ego_l.get("speed_mps")) else 0.0
        yaw_r = ego_l.get("yaw_rate") if _fin(ego_l.get("yaw_rate")) else 0.0        # the model's: positive right
        motion = "vision"
        turn = None
        e = self.ego
        if e is not None and abs(ts - e["ts_mono_ns"] / NS) <= c["intent_max_age_s"]:
            if _fin(e.get("yaw_rate_rps")):
                yaw_r, motion = -e["yaw_rate_rps"], "vehicle"
            if e.get("turn_signal") in ("left", "right"):
                turn = e["turn_signal"]
        dt = 0.0 if self.ts is None else ts - self.ts
        if dt < 0.0 or dt > 1.0:
            keep_map, why = self.map, self.map_why
            self.reset()
            self.map, self.map_why = (keep_map, why) if 0.0 <= dt else (None, "no_map")
            if self.map is not None:
                self.map["odo"], self.map["hdg"] = 0.0, 0.0
            dt = 0.0
        self.ts = ts
        self.v = v
        self.odo += max(v, 0.0) * dt
        self.hdg += yaw_r * dt

        vis = self._vision(lanes, road)
        reach, vsrc = 0.0, "none"
        if vis is not None:
            vx, vy, vsd, vsrc = vis
            reach = self._reach(vx, vsd)

        out = {"seq": lanes.get("seq"), "ts_mono_ns": ts_ns, "ts_real_ns": lanes.get("ts_real_ns"),
               "source": "vision", "lane_width_m": round(self.width or c["lane_width_m"], 2),
               "vision": {"from": vsrc, "reach_m": round(reach, 1)},
               "motion": {"source": motion, "speed_mps": round(v, 2), "yaw_rate_rps": round(-yaw_r, 4)},
               "map": {"used": False, "why": self.map_why}, "centre": None, "branches": [], "ring": None,
               "junction": None, "way": None, "intent": None if turn is None else {"turn": turn, "source": "turn_signal"}}

        self.lane_right = bool(lanes.get("lane_to_right"))
        g = self._map_part(ts, vis, reach, dt, turn, out)
        if g is None:
            self.used = False
            if vis is not None:
                m = vx <= reach + 1e-6
                out["centre"] = self._centre(vx[m], vy[m], np.clip(1.0 - vsd[m] / 1.5, 0.1, 1.0), "v" * int(m.sum()),
                                             reach, None)
            return out
        return out

    def _held(self, ts):
        """The last fit is still good to use without a fresh one."""
        return (self.fit_ts is not None and ts - self.fit_ts <= self.cfg["hold_s"]
                and self.odo - self.fit_odo <= self.cfg["hold_m"])

    @staticmethod
    def _centre(x, y, conf, src, vision_to, end):
        s = np.concatenate([[0.0], np.cumsum(np.hypot(np.diff(x), np.diff(y)))]) + float(x[0])
        return {"s": np.round(s, 1).tolist(), "x": np.round(x, 1).tolist(), "y": np.round(y, 2).tolist(),
                "conf": np.round(conf, 2).tolist(), "src": src,       # per point: v camera, b blend, m map
                "vision_to_m": round(vision_to, 1), "end": end}

    def _map_part(self, ts, vis, reach, dt, turn, out):
        """Fill the map part of `out`; None when the map is not used (out["map"]["why"] says why)."""
        c = self.cfg
        mp = self.map
        info = out["map"]
        if mp is None:
            self.agree_for = 0.0
            return None
        age = ts - mp["ts"]
        adv = self.odo - mp["odo"]
        info.update(match_confidence=round(mp["match"], 2), age_s=round(age, 2), advance_m=round(adv, 1),
                    hacc_m=mp["hacc"], state=mp["state"], synthetic=mp["synthetic"])
        if age > c["map_max_age_s"] or age < -1.0:
            info["why"] = self.map_why or "stale"
            self.agree_for = 0.0
            self.map = None
            return None
        s, xy, th = mp["s"], mp["xy"], mp["th"]
        if adv < -2.0 or adv > c["advance_max_m"] or adv >= s[-1] - 8.0:
            info["why"] = "advance"
            return None
        adv = max(adv, 0.0)
        ring = mp["ring"]
        t0 = None if mp["rel"] is None else mp["rel"] + (self.hdg - mp["hdg"])   # the filter's heading, carried on by the yaw rate
        if ring is not None and t0 is not None:
            # on a ring the car's heading says where on it the car is, better than a position a few metres
            # off does (5 m along a ring of 14 m radius are 20 degrees): slide the car along the path to where
            # the ring runs in the car's direction
            r_in = 0.0 if ring.get("on_ring") or ring.get("entry_m") is None else ring["entry_m"]
            r_out = ring["exit_m"] if _fin(ring.get("exit_m")) else r_in + 2.0 * math.pi * ring["radius_m"]
            if r_in - 2.0 <= adv <= r_out + 2.0:
                lo, hi = max(adv - c["ring_slide_m"], r_in), min(adv + c["ring_slide_m"], r_out, float(s[-1]) - 9.0)
                if hi > lo:
                    cand = np.arange(lo, hi + 0.5, 1.0)
                    diff = np.abs((np.interp(cand, s, th) - t0 + math.pi) % (2.0 * math.pi) - math.pi)
                    cost = np.degrees(diff) + 1.0 * np.abs(cand - adv)
                    k = int(np.argmin(cost))
                    if math.degrees(diff[k]) < 12.0:
                        info["ring_slide_m"] = round(float(cand[k]) - adv, 1)
                        adv = float(cand[k])
        # the car on the path: `adv` along it
        px, py = float(np.interp(adv, s, xy[:, 0])), float(np.interp(adv, s, xy[:, 1]))
        ax, ay = float(np.interp(max(adv - 5.0, 0.0), s, xy[:, 0])), float(np.interp(max(adv - 5.0, 0.0), s, xy[:, 1]))
        bx, by = float(np.interp(adv + 5.0, s, xy[:, 0])), float(np.interp(adv + 5.0, s, xy[:, 1]))
        along = math.atan2(by - ay, bx - ax)             # the road's direction here, over 10 m
        if t0 is None:
            t0 = float(np.interp(adv, s, th))            # no heading known: along the path
        turned = (t0 - along + math.pi) % (2.0 * math.pi) - math.pi
        info["off_path_deg"] = round(math.degrees(turned), 1)
        ct, st = math.cos(t0), math.sin(t0)

        def car(p):
            d = p - (px, py)
            return np.stack([d[:, 0] * ct + d[:, 1] * st, -d[:, 0] * st + d[:, 1] * ct], axis=1)

        path = car(xy)
        ring_in = None
        if ring is not None:
            ring_in = 0.0 if ring.get("on_ring") or ring.get("entry_m") is None else ring["entry_m"] - adv
            if ring_in < 0.0:
                ring_in = 0.0
            if _fin(ring.get("exit_m")) and ring["exit_m"] - adv < -c["branch_behind_m"]:
                ring, ring_in = None, None             # left behind
        on_ring = ring is not None and ring_in == 0.0

        # fit the camera's centre line onto the path over the near range
        fitted = False
        rms = None
        if vis is not None:
            vx, vy, vsd, _ = vis
        if vis is not None and not on_ring:
            hi = min(reach, c["fit_x"][1])
            if ring_in is not None:
                hi = min(hi, ring_in - c["ring_margin_m"])
            m = (vx >= c["fit_x"][0]) & (vx <= hi)
            i1 = int(np.searchsorted(s, adv + hi + 10.0)) + 1
            i0 = max(int(np.searchsorted(s, adv)) - 1, 0)
            seg = path[i0:i1]
            if int(m.sum()) >= c["fit_min_points"] and len(seg) >= 2 and (np.diff(seg[:, 0]) > 0.5).all():
                r = vy[m] - np.interp(vx[m], seg[:, 0], seg[:, 1])
                w = 1.0 / (vsd[m] ** 2 + 0.09)
                a = np.stack([np.ones(int(m.sum())), vx[m]], axis=1)
                aw = a * w[:, None]
                try:
                    sol = np.linalg.solve(a.T @ aw, aw.T @ r)
                except np.linalg.LinAlgError:
                    sol = None
                if sol is not None:
                    rms = float(np.sqrt(np.sum(w * (r - a @ sol) ** 2) / np.sum(w)))
                    dy, dyaw = float(sol[0]), float(sol[1])
                    ok = (abs(dy) <= c["fit_max_dy_m"] and abs(math.degrees(dyaw)) <= c["fit_max_dyaw_deg"]
                          and rms <= c["fit_rms_m"][0] + c["fit_rms_m"][1] * hi)
                    fitted = True
                    if ok:
                        if self.fit is None or not self._held(ts):
                            self.fit = (dy, dyaw)
                        else:
                            k = 1.0 - math.exp(-max(dt, 1e-3) / c["fit_tau_s"])
                            self.fit = (self.fit[0] + k * (dy - self.fit[0]), self.fit[1] + k * (dyaw - self.fit[1]))
                        self.fit_ts, self.fit_odo = ts, self.odo
                        self.agree_for += dt
                        self.disagree_for = 0.0
                    else:
                        self.disagree_for += dt
                        self.agree_for = 0.0
                    info["fit"] = {"dy_m": round(dy, 2), "dyaw_deg": round(math.degrees(dyaw), 2),
                                   "rms_m": round(rms, 2), "ok": ok, "to_m": round(hi, 1)}
        off_path = abs(math.degrees(turned)) > c["off_path_deg"]
        if off_path:
            self.disagree_for += dt
            self.agree_for = 0.0
        if not fitted and not off_path:
            # no camera centre to compare with (lines gone in a junction, on a ring): the last fit is held
            if self.fit is None or not self._held(ts):
                self.disagree_for += dt
            info["fit"] = {"held_s": None if self.fit_ts is None else round(ts - self.fit_ts, 1)}
        if not self.used and self.agree_for >= c["agree_s"] and self.fit is not None:
            self.used = True
        elif self.used and self.disagree_for >= c["disagree_s"]:
            self.used = False
        if self.fit is not None:
            info["align"] = {"dy_m": round(self.fit[0], 2), "dyaw_deg": round(math.degrees(self.fit[1]), 2)}
        if not self.used or self.fit is None:
            info["why"] = "off_path" if off_path else "disagree" if fitted else "no_fit"
            return None
        dy, dyaw = self.fit
        ca, sa = math.cos(dyaw), math.sin(dyaw)

        def align(p):
            return np.stack([p[:, 0] * ca - p[:, 1] * sa, p[:, 0] * sa + p[:, 1] * ca + dy], axis=1)

        path = align(path)
        info["used"], info["why"] = True, None
        # the path was laid through the car, but it is the way's centre line (the middle of the carriageway, of
        # both directions on a two-way road), and the car drives in a lane: side roads and the ring are moved
        # by the distance between the two, from the lane count and the lane the camera sees to the right
        way_dy = 0.0
        ln = mp["lanes"][0] if mp["lanes"] and isinstance(mp["lanes"][0], dict) else None
        if ln is not None and _fin(ln.get("forward")) and _fin(ln.get("backward")):
            w = self.width or c["lane_width_m"]
            k = 1 if self.lane_right else 0
            fwd = max(int(ln["forward"]), k + 1)
            way_dy = (fwd - k - 0.5) * w if ln["backward"] > 0 else (0.5 * fwd - k - 0.5) * w
            way_dy = max(-6.0, min(6.0, way_dy))
        info["way_offset_m"] = round(way_dy, 2)

        def place(p):                                    # map objects beside the path
            q = align(car(p))
            q[:, 1] -= way_dy
            return q
        out["source"] = "vision+map"

        # driver intent: the branch on the signalled side at the next junction
        chosen = None
        if turn is not None:
            best = None
            for k, (b, pts, side) in enumerate(mp["branches"]):
                d = b.get("dist_m")
                if side != turn or not _fin(d) or not b.get("exit") or b.get("path"):
                    continue
                ahead = d - adv
                if -3.0 <= ahead <= c["intent_ahead_m"]:
                    key = (bool(b.get("minor")), ahead)
                    if best is None or key < best[0]:
                        best = (key, k, ahead)
            if best is None and turn == "left" and ring is not None:
                # a left signal at a roundabout: an exit beyond the one the map's path takes (the first of them);
                # a convention, not a certainty: the message says the choice comes from the signal
                beyond = sorted((b["ring_m"], k) for k, (b, pts, side) in enumerate(mp["branches"])
                                if b.get("ring") and b.get("dist_m") is None and b.get("exit") and not b.get("path")
                                and _fin(b.get("ring_m")))
                if beyond and _fin(ring.get("exit_m")) and ring["exit_m"] - adv <= c["intent_ahead_m"]:
                    best = (None, beyond[0][1], ring["exit_m"] - adv)
            if best is not None:
                chosen = best[1]
                out["intent"]["at_m"] = round(best[2], 1)

        # centre line: camera to its reach, map beyond
        v_to = reach
        if ring_in is not None:
            if not on_ring:
                v_to = min(v_to, max(ring_in - c["ring_margin_m"], 0.0))
            else:                                        # on the ring: lane lines if the camera has them, else the map from the car on
                v_to = min(v_to, c["on_ring_reach_m"]) if vis is not None and vis[3] in ("lanes", "line") else 0.0
        s_rel = s - adv
        cut = c["path_max_m"]
        end = "horizon" if s_rel[-1] >= cut else "path_end"
        round_to = None
        if chosen is not None:
            cb = mp["branches"][chosen]
            if _fin(cb[0].get("dist_m")):
                cut, end = cb[0]["dist_m"] - adv, "intent"
            else:                                        # further round the ring than the path goes
                cut, end, round_to = ring["exit_m"] - adv, "intent", align(car(cb[1][:1]))[0]
        if chosen is not None:
            v_to = max(min(v_to, cut), 0.0)              # the way on turns off there: the centre line ends, whatever the camera sees beyond
        pconf = np.ones(len(s))
        for d, cf, kind, minor in mp["junctions"]:
            pconf[s > d + 0.5] = cf
            if kind in ("dead_end", "no_way_on") and d - adv < cut:
                cut, end = d - adv, kind
        xs, ys, cs, src = [], [], [], ""
        if vis is not None and v_to > 0.0:
            m = vx <= v_to + 1e-6
            bx, by = vx[m], vy[m].copy()
            cf = np.clip(1.0 - vsd[m] / 1.5, 0.1, 1.0)
            i0 = max(int(np.searchsorted(s, adv)) - 1, 0)
            i1 = int(np.searchsorted(s, adv + v_to + 10.0)) + 1
            seg = path[i0:i1]
            w = np.clip((bx - (v_to - c["blend_m"])) / c["blend_m"], 0.0, 1.0)
            if len(seg) >= 2 and (np.diff(seg[:, 0]) > 0.5).all():
                by = by + w * (np.interp(bx, seg[:, 0], seg[:, 1]) - by)
            else:
                w = np.zeros(len(bx))
            xs, ys, cs = bx.tolist(), by.tolist(), cf.tolist()
            src = "".join("b" if k > 0.02 else "v" for k in w)
        # map points beyond the camera's part, along the path
        start = adv + (v_to if xs else 0.0)
        j = int(np.searchsorted(s, start + 0.5)) if xs else max(int(np.searchsorted(s, adv)) - 1, 0)
        while j < len(s) and xs and path[j, 0] <= xs[-1] + 0.5 and s[j] - adv < v_to + 30.0:
            j += 1
        jm = s_rel[j:] <= cut + 1e-6
        mxy, mcf = path[j:][jm], pconf[j:][jm]
        if len(mxy) and s_rel[j:][jm][-1] < cut - 1.0 and cut <= s_rel[-1]:
            tail = np.array([[np.interp(cut + adv, s, path[:, 0]), np.interp(cut + adv, s, path[:, 1])]])
            mxy, mcf = np.concatenate([mxy, tail]), np.concatenate([mcf, mcf[-1:]])
        if round_to is not None and len(mxy):
            # on round the ring from where the path leaves it to the chosen arm, in the path's sense of rotation
            rc = align(car(np.asarray([ring["centre"]], dtype=float)))[0]
            a0 = math.atan2(mxy[-1, 1] - rc[1], mxy[-1, 0] - rc[0])
            a1 = math.atan2(round_to[1] - rc[1], round_to[0] - rc[0])
            prev = mxy[-2] if len(mxy) >= 2 else np.array([xs[-1], ys[-1]]) if xs else mxy[-1] - (1.0, 0.0)
            ap = math.atan2(prev[1] - rc[1], prev[0] - rc[0])
            sense = 1.0 if (a0 - ap + math.pi) % (2.0 * math.pi) - math.pi >= 0.0 else -1.0
            sweep = (sense * (a1 - a0)) % (2.0 * math.pi)
            n = max(2, int(sweep / 0.25))
            ang = a0 + sense * sweep * np.arange(1, n + 1) / n
            arc = np.stack([rc[0] + ring["radius_m"] * np.cos(ang), rc[1] + ring["radius_m"] * np.sin(ang)], axis=1)
            mxy, mcf = np.concatenate([mxy, arc]), np.concatenate([mcf, np.full(n, mcf[-1])])
        x = np.asarray(xs + mxy[:, 0].tolist())
        y = np.asarray(ys + mxy[:, 1].tolist())
        if len(x) >= 2:
            out["centre"] = self._centre(x, y, np.asarray(cs + mcf.tolist()), src + "m" * len(mxy), v_to, end)

        # side roads and the ring, in the car's frame
        for k, (b, pts, side) in enumerate(mp["branches"]):
            d = b.get("dist_m")
            ahead = None if not _fin(d) else d - adv
            if ahead is not None and not -c["branch_behind_m"] <= ahead <= c["branch_ahead_m"]:
                continue
            if b.get("path"):
                continue                                 # part of the centre line
            p = place(pts)
            if ahead is None and float(np.hypot(p[0, 0], p[0, 1])) > c["branch_ahead_m"]:
                continue
            o = {"dist_m": None if ahead is None else round(ahead, 1), "pts": np.round(p, 1).tolist(),
                 "class": b.get("class"), "minor": bool(b.get("minor")), "exit": bool(b.get("exit")),
                 "entry": bool(b.get("entry")), "lanes": b.get("lanes"), "ring": bool(b.get("ring")), "side": side,
                 "way": k == chosen}
            out["branches"].append(o)
        if ring is not None:
            cxy = place(np.asarray([ring["centre"]], dtype=float))[0]
            out["ring"] = {"centre": [round(float(cxy[0]), 1), round(float(cxy[1]), 1)],
                           "radius_m": ring["radius_m"], "lanes": ring.get("lanes"), "on_ring": on_ring,
                           "entry_m": None if on_ring else round(ring_in, 1),
                           "exit_m": None if not _fin(ring.get("exit_m")) else round(ring["exit_m"] - adv, 1)}
        view = mp["view"]
        if view is not None and _fin(view.get("dist_m")):
            out["junction"] = {"kind": view.get("kind"), "dist_m": round(max(view["dist_m"] - adv, 0.0), 1),
                               "control": view.get("control"), "confidence": view.get("confidence")}
        nxt = next((cf for d, cf, kind, minor in mp["junctions"] if d - adv > 0.0 and not minor), None)
        if chosen is not None:
            idx = next((i for i, o in enumerate(out["branches"]) if o["way"]), None)
            out["way"] = {"source": "intent", "turn": turn, "branch": idx}
        else:
            out["way"] = {"source": "map", "confidence": None if nxt is None else round(nxt, 2)}
        return out

    def summary(self, msg):
        """Short form for /v1/health."""
        if not msg:
            return None
        mp = msg.get("map") or {}
        return {"source": msg.get("source"), "vision": msg.get("vision"), "map_used": mp.get("used"),
                "map_why": mp.get("why"), "branches": len(msg.get("branches") or ()),
                "ring": bool(msg.get("ring"))}
