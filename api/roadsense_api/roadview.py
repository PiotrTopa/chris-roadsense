"""The road part of the display channel: `view.road` (roadsense 2.0, lane L22).

One picture of the road for a consumer that draws it, made of parts that each say where they come from, how
sure they are, and whether they rest on a weak position:

    lanes (tracked lines) + road (plan) ─┐
    map (ahead.geometry) ────────────────┼─> RoadGeometryFusion (roadgeom.py) ─┐   geometry: the camera's lane
    ego (speed, yaw rate, turn signal) ──┘                                     │   centre near the car, the map's
    map (ahead.junction_view, ahead.turn_lanes, road.lanes, position, match) ──┼─> RoadView ─> view["road"]
    ego-lane filter (egolane.py) ──────────────────────────────────────────────┘   road beyond, side roads, ring

Until 2.0 the geometry was a topic of its own (`roadgeom`), the junction picture was read from `map`, and the
lane boxes from `fused`; a consumer that draws had to combine three topics and decide by itself when the map
was good enough. Now this is the one place where that is decided, and the measurement topics (`map`, `fused`,
`lanes`) stay what was measured.

Display rules applied here (they are display policy and live only in `view`):

* the map layer is `trusted` when the match confidence, low-passed over about four map messages (2 s), reaches
  `trust_on` and until it falls under `trust_off`; a map that drops out for a moment is held for `hold_s`;
* a junction is given when the path up to it is likely (`junction_min_conf`) and the position is good enough
  for its distance: `hacc_m` within `max_hacc_m`, or within `hacc_share` of the distance. It is marked
  `approximate` when the position is dead-reckoned, weaker than `max_hacc_m`, or its error exceeds
  `approx_share` of the distance: a consumer then draws it as "about there", without an exact distance;
* distances are carried between two map messages by the distance driven and never grow without a new message;
* the lane that is lit (`ego_index`) is the ego-lane filter's committed lane and stays until another lane is
  committed or its own probability falls under `lane_release_p` (no flicker at the commit threshold);
  `likely_index` is the most likely lane when none is committed and one holds `lane_likely_p`.

No I/O and no wall clock in here; times come from the messages (CLOCK_MONOTONIC ns).
"""
import math

NS = 1e9

DEFAULT_CFG = dict(
    map_max_age_s=3.0,
    trust_on=0.55, trust_off=0.40, trust_gain=0.25,
    hold_s=2.5,                  # the last trusted map message is kept this long when the map drops out
    max_hacc_m=20.0,             # position accuracy up to which distances are exact
    hacc_share=0.4,              # a junction is still shown while the position error is at most this share of its distance
    approx_share=0.15,           # ... and marked approximate above this share
    junction_min_conf=0.5,
    junction_max_m=400.0,
    turn_lanes_max_m=400.0,
    advance_max_m=60.0,          # a map message is not carried further than this
    lane_release_p=0.6, lane_likely_p=0.45,
)

ARROW = {"left": "l", "slight_left": "l", "sharp_left": "l", "through": "t", "right": "r", "slight_right": "r",
         "sharp_right": "r", "reverse": "u", "merge_to_left": "m", "merge_to_right": "m"}


def _fin(v):
    return isinstance(v, (int, float)) and not isinstance(v, bool) and math.isfinite(v)


def arrows(lane):
    """Arrows of one lane as letters (l left, t through, r right, u turn back, m merge), in that order, no
    duplicates: ["slight_left", "through"] -> "lt"; no arrow painted -> ""."""
    got = {ARROW[w] for w in lane if w in ARROW}
    return "".join(ch for ch in "ltrum" if ch in got)


class RoadView:
    def __init__(self, cfg=None):
        self.cfg = dict(DEFAULT_CFG, **(cfg or {}))
        self.reset()

    def reset(self):
        self.ts = None
        self.odo = 0.0
        self.v = 0.0
        self.map = None              # (message, odometer at its frame, its time s)
        self.good = None             # the last trusted one, the same triple
        self.conf = None             # low-passed match confidence
        self.trusted = False
        self.lit = None              # (forward, ego index) of the lane shown as the ego lane

    def on_map(self, msg):
        if not isinstance(msg, dict) or not _fin(msg.get("ts_mono_ns")):
            return
        c = self.cfg
        mc = (msg.get("match") or {}).get("confidence")
        if msg.get("available") and _fin(mc):
            self.conf = mc if self.conf is None else self.conf + c["trust_gain"] * (mc - self.conf)
            if self.conf >= c["trust_on"]:
                self.trusted = True
            elif self.conf < c["trust_off"]:
                self.trusted = False
        elif not msg.get("available"):
            self.trusted = False
            if msg.get("state") in ("no_source", "unknown"):
                self.conf = None     # the position was lost: trust is earned again
        ts = msg["ts_mono_ns"] / NS
        lag = 0.0 if self.ts is None else max(-1.0, min(1.0, ts - self.ts))
        self.map = (msg, self.odo + self.v * lag, ts)
        if self.trusted and msg.get("available"):
            self.good = self.map

    def on_frame(self, ts_ns, speed_mps, geom, egolane):
        """One road frame: `geom` is RoadGeometryFusion's result for it (or None), `egolane` the ego-lane
        filter's (or None). Returns the `road` object of the `view` message."""
        c = self.cfg
        ts = ts_ns / NS
        dt = 0.0 if self.ts is None else ts - self.ts
        if dt < 0.0 or dt > 1.0:                        # a gap in the frames: distances are not carried across it
            self.map = self.good = self.lit = None
            dt = 0.0
        self.ts = ts
        self.v = max(0.0, speed_mps or 0.0)
        self.odo += self.v * dt

        cur = self.map if (self.map is not None and ts - self.map[2] <= c["map_max_age_s"]) else None
        use, held = None, False
        if cur is not None and self.trusted and cur[0].get("available"):
            use = cur
        elif self.good is not None and ts - self.good[2] <= c["hold_s"] + 0.6:
            use, held = self.good, True                 # dropped out for a moment (a junction, a doubtful match)
        position = self._position(cur, use, held, ts)
        g = geom if isinstance(geom, dict) else {}
        out = {"source": g.get("source") or "vision",
               "position": position,
               "lane_width_m": g.get("lane_width_m"),
               "vision": g.get("vision"),
               "map": g.get("map") or {"used": False, "why": "no_map"},
               "centre": g.get("centre"), "branches": g.get("branches") or [], "ring": g.get("ring"),
               "way": g.get("way"), "intent": g.get("intent"),
               "junction": None, "lanes": self._lanes(use, egolane, position)}
        approx = bool(position and position["approximate"])
        if out["map"].get("used"):
            # the geometry was joined to the camera frame by frame; what a weak position leaves open is where
            # along the road the side roads are
            for b in out["branches"]:
                b["approximate"] = approx
            if out["ring"] is not None:
                out["ring"]["approximate"] = approx
        if use is not None:
            adv = self.odo - use[1]
            if 0.0 <= adv <= c["advance_max_m"]:
                out["junction"] = self._junction(use[0], adv, position)
                out["lanes"]["turn"] = self._turn(use[0], adv, out["lanes"])
        return out

    # ---- parts -----------------------------------------------------------------------------------
    def _position(self, cur, use, held, ts):
        """How good the position under the map parts is. None without a map daemon."""
        c = self.cfg
        src = use or cur
        if src is None:
            return None
        m = src[0]
        pos = m.get("position") or {}
        hacc = pos.get("hacc_m") if _fin(pos.get("hacc_m")) else None
        state = m.get("state")
        why = []
        if state == "dead_reckoning":
            why.append("dead_reckoning")
        if hacc is not None and hacc > c["max_hacc_m"]:
            why.append("position_accuracy")
        if held:
            why.append("held")
        return {"state": state, "available": bool(m.get("available")), "reason": m.get("reason"),
                "hacc_m": hacc, "match_confidence": (m.get("match") or {}).get("confidence"),
                "match_confidence_smoothed": None if self.conf is None else round(self.conf, 2),
                "trusted": use is not None, "held": held,
                "approximate": bool(why), "why": why,
                "synthetic": bool((m.get("input") or {}).get("synthetic")),
                "age_s": round(ts - src[2], 2)}

    def _junction(self, m, adv, position):
        c = self.cfg
        jv = (m.get("ahead") or {}).get("junction_view")
        if not isinstance(jv, dict) or not _fin(jv.get("dist_m")) or not _fin(jv.get("confidence")):
            return None
        ring = jv.get("ring") if isinstance(jv.get("ring"), dict) else None
        on_ring = bool(ring and ring.get("on_ring"))
        dist = max(0.0, jv["dist_m"] - adv)
        if dist > c["junction_max_m"] or (jv["confidence"] < c["junction_min_conf"] and not on_ring):
            return None
        hacc = position["hacc_m"] if position else None
        why = list(position["why"]) if position else []
        if hacc is not None and hacc > c["max_hacc_m"] and not on_ring and hacc > c["hacc_share"] * max(dist, 1.0):
            return None                                 # the error is of the size of the distance: say nothing
        if hacc is not None and hacc > c["approx_share"] * max(dist, 1.0) and "position_accuracy" not in why:
            why.append("position_accuracy")
        out = dict(jv)
        out.update(dist_m=round(dist, 1), provenance="map", approximate=bool(why), why=why)
        if ring is not None and _fin(ring.get("exit_dist_m")):
            out["ring"] = dict(ring, exit_dist_m=round(max(0.0, ring["exit_dist_m"] - adv), 1))
        return out

    def _lanes(self, use, ego, position):
        """Lane count (the map's, of the trusted match), the ego lane (the filter's) and what a consumer lights."""
        c = self.cfg
        ego = ego if isinstance(ego, dict) else {}
        ln = ((use[0].get("road") or {}).get("lanes") if use is not None else None) or {}
        fwd = int(ln["forward"]) if _fin(ln.get("forward")) else None
        probs, frm = ego.get("probs"), ego.get("probs_from")
        prov = list(ego.get("provenance") or ())
        out = {"count": ln.get("total"), "forward": fwd, "backward": ln.get("backward"),
               "count_source": ("map:" + ln["source"]) if ln.get("source") else None,
               "ego_index": None, "likely_index": None, "ego_from_right": ego.get("ego_from_right"),
               "probs": None, "probs_from": frm if probs else None, "confidence": 0.0,
               "oncoming_left": None, "provenance": prov, "approximate": bool(position and position["approximate"]),
               "turn": None}
        if probs:
            out["probs"] = probs
        if fwd is None:
            self.lit = None
            return out
        if fwd == 1:
            out.update(ego_index=0, confidence=1.0, probs=[1.0], probs_from="left")
        elif probs and frm == "left" and len(probs) == fwd:
            e = ego.get("ego_index")
            if e is None and self.lit is not None and self.lit[0] == fwd and probs[self.lit[1]] >= c["lane_release_p"]:
                e = self.lit[1]                          # still the lit lane: no flicker at the commit threshold
            self.lit = (fwd, e) if e is not None else None
            out["ego_index"] = e
            if e is not None:
                out["confidence"] = round(probs[e], 2)
            else:
                k = max(range(fwd), key=lambda i: probs[i])
                if probs[k] >= c["lane_likely_p"]:
                    out["likely_index"] = k
        else:
            self.lit = None
            out["probs"], out["probs_from"] = None, None
        if ln.get("backward") == 0:
            out["oncoming_left"] = False
        elif out["ego_index"] is not None:
            out["oncoming_left"] = out["ego_index"] == 0
        return out

    def _turn(self, m, adv, lanes):
        """Painted turn arrows for the lane boxes: on the stretch, or announced before it."""
        c = self.cfg
        tl = (m.get("ahead") or {}).get("turn_lanes")
        if not isinstance(tl, dict) or not isinstance(tl.get("lanes"), list) or not _fin(tl.get("dist_m")):
            return None
        start = max(0.0, tl["dist_m"] - adv)
        end = tl.get("junction_m")
        end = max(0.0, end - adv) if _fin(end) else None
        if start > c["turn_lanes_max_m"] or (end is not None and end <= 0.0 and adv > 0.0 and tl["dist_m"] <= 0.0
                                             and tl.get("junction_m", 0.0) - adv < -5.0):
            return None
        path = tl.get("path") if isinstance(tl.get("path"), list) and len(tl["path"]) == len(tl["lanes"]) else None
        return {"lanes": tl["lanes"], "arrows": [arrows(a) for a in tl["lanes"]], "count": len(tl["lanes"]),
                "dist_m": round(start, 1), "junction_m": None if end is None else round(end, 1),
                "path": path, "here": start <= 0.0 and len(tl["lanes"]) == lanes.get("forward"),
                "provenance": "map:turn_lanes", "confidence": tl.get("confidence")}
