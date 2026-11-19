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

* the map layer is `trusted` when the match confidence of the usable map messages (available, at least
  `min_match`: the rule of `fused`), low-passed over about four of them (2 s), reaches `trust_on`. The match
  confidence dips for a message or two wherever a side road branches off; such a message, like one that is off
  the road, is not used and the last trusted one is held for `hold_s` (its distances carried by the distance
  driven), marked approximate after `hold_approx_s`. After `hold_s` without a usable message the layer is gone
  and trust is earned again;
* a junction is given when the path up to it is likely (`junction_min_conf`) and the position is good enough
  for its distance: `hacc_m` within `max_hacc_m`, or within `hacc_share` of the distance. It is marked
  `approximate` when the position is dead-reckoned, weaker than `max_hacc_m`, or its error exceeds
  `approx_share` of the distance: a consumer then draws it as "about there", without an exact distance;
* distances are carried between two map messages by the distance driven and never grow without a new message;
  a map message that left mapd after the frame was taken (the camera frame is older than the message by the
  model's latency) is used as it is, not dropped (lane L36);
* the lane that is lit (`ego_index`) is the ego-lane filter's committed lane and stays until another lane is
  committed or its own probability falls under `lane_release_p` (no flicker at the commit threshold);
  `likely_index` is the most likely lane when none is committed and one holds `lane_likely_p`;
* the lane count shown changes only after the map's new count has held for `count_settle_s` (a short segment
  tagged with another count, a match that hops between two parallel edges); meanwhile the old count and its
  lit lane stay, marked `count_held` (lane L36);
* painted turn arrows (`lanes.turn`) are given only for as many lanes as the boxes count; a stretch appears
  after `turn_on_s` of being seen, stays through a drop-out of up to `turn_hold_s` (carried by the distance
  driven, marked `held`), and its distance never grows while it is the same stretch (lane L36). `turn.path` is
  the map's most likely path, not a route: `route` is null until a navigation source feeds one.

No I/O and no wall clock in here; times come from the messages (CLOCK_MONOTONIC ns).
"""
import math

NS = 1e9

DEFAULT_CFG = dict(
    map_max_age_s=3.0,
    min_match=0.5,               # a map message counts from this match confidence (the rule of `fused`)
    trust_on=0.55, trust_off=0.40, trust_gain=0.25,   # ... and the map layer is trusted on their low-passed value
    hold_s=2.5,                  # the last trusted map message is kept this long when the map drops out
    hold_approx_s=1.0,           # ... and what rests on it is marked approximate after this long
    max_hacc_m=20.0,             # position accuracy up to which distances are exact
    hacc_share=0.4,              # a junction is still shown while the position error is at most this share of its distance
    approx_share=0.15,           # ... and marked approximate above this share
    junction_min_conf=0.5,
    junction_max_m=400.0,
    turn_lanes_max_m=400.0,
    advance_max_m=60.0,          # a map message is not carried further than this
    lane_release_p=0.6, lane_likely_p=0.45,
    count_settle_s=2.0,          # a new lane count is shown after it held this long (L36)
    turn_on_s=0.3,               # a stretch of turn arrows is shown after it was seen this long (L36) ...
    turn_hold_s=2.0,             # ... and stays this long when it drops out (held, carried by the distance driven)
    turn_same_m=25.0,            # two sightings of the same arrows whose junction lies this close are one stretch
    gap_keep_s=1.0,              # a map message no older than this survives a gap in the frames (re-based)
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
        self.map = None              # (message, odometer at its frame, its time s, usable)
        self.good = None             # the last trusted one, the same triple
        self.conf = None             # low-passed match confidence
        self.trusted = False
        self.lit = None              # (forward, ego index) of the lane shown as the ego lane
        self.last_lit = None         # (forward, {ego_index, likely_index, probs, probs_from, confidence}) last shown
        self.lanes_shown = None      # map.road.lanes whose count is shown
        self.lanes_pend = None       # (forward, since s, lanes) a new count waiting for count_settle_s
        self.tn = None               # the turn-arrow stretch shown: dict (raw turn, start/end on the odometer, seen)
        self.tn_cand = None          # (key, since s, reference odometer) a new stretch waiting for turn_on_s

    def on_map(self, msg):
        if not isinstance(msg, dict) or not _fin(msg.get("ts_mono_ns")):
            return
        c = self.cfg
        mc = (msg.get("match") or {}).get("confidence")
        usable = bool(msg.get("available")) and _fin(mc) and mc >= c["min_match"]
        if usable:
            self.conf = mc if self.conf is None else self.conf + c["trust_gain"] * (mc - self.conf)
            if self.conf >= c["trust_on"]:
                self.trusted = True
            elif self.conf < c["trust_off"]:
                self.trusted = False
        elif msg.get("state") in ("no_source", "unknown"):
            self.conf, self.trusted = None, False      # the position was lost: trust is earned again
        ts = msg["ts_mono_ns"] / NS
        lag = 0.0 if self.ts is None else max(-1.0, min(1.0, ts - self.ts))
        self.map = (msg, self.odo + self.v * lag, ts, usable)
        if self.trusted and usable:
            self.good = self.map

    def on_frame(self, ts_ns, speed_mps, geom, egolane):
        """One road frame: `geom` is RoadGeometryFusion's result for it (or None), `egolane` the ego-lane
        filter's (or None). Returns the `road` object of the `view` message."""
        c = self.cfg
        ts = ts_ns / NS
        dt = 0.0 if self.ts is None else ts - self.ts
        self.v = max(0.0, speed_mps or 0.0)
        if dt < 0.0 or dt > 1.0:
            # a gap in the frames: distances are not carried across it. A map message from the last
            # gap_keep_s is kept, re-based to where the car is now (it was measured that long ago)
            def keep(m):
                if m is None or not 0.0 <= ts - m[2] <= c["gap_keep_s"]:
                    return None
                return (m[0], self.odo - self.v * (ts - m[2]), m[2], m[3])
            self.map, self.good = keep(self.map), keep(self.good)
            self.lit = self.tn = self.tn_cand = None
            dt = 0.0
        self.ts = ts
        self.odo += self.v * dt

        cur = self.map if (self.map is not None and ts - self.map[2] <= c["map_max_age_s"]) else None
        use, held = None, False
        if cur is not None and self.trusted and cur[3]:
            use = cur
        elif self.good is not None and ts - self.good[2] <= c["hold_s"] + 0.6:
            use, held = self.good, True                 # dropped out for a moment (a junction, a doubtful match)
        elif self.good is not None:
            self.good, self.conf, self.trusted = None, None, False    # gone for longer: trust is earned again
        position = self._position(cur, use, held, ts)
        g = geom if isinstance(geom, dict) else {}
        out = {"source": g.get("source") or "vision",
               "position": position,
               "lane_width_m": g.get("lane_width_m"),
               "vision": g.get("vision"),
               "map": g.get("map") or {"used": False, "why": "no_map"},
               "centre": g.get("centre"), "branches": g.get("branches") or [], "ring": g.get("ring"),
               "way": g.get("way"), "intent": g.get("intent"),
               "junction": None, "lanes": self._lanes(use, egolane, position, ts),
               "route": None}           # no navigation source feeds a route (yet): see the module docstring
        approx = bool(position and position["approximate"])
        if out["map"].get("used"):
            # the geometry was joined to the camera frame by frame; what a weak position leaves open is where
            # along the road the side roads are
            for b in out["branches"]:
                b["approximate"] = approx
            if out["ring"] is not None:
                out["ring"]["approximate"] = approx
        raw_turn = None
        if use is not None:
            adv = self.odo - use[1]
            if -c["advance_max_m"] <= adv <= c["advance_max_m"]:
                # adv < 0: the map message left mapd after this frame was taken (the frame is older by the
                # model's latency); its distances are used as they are (until L36 the frame had no junction)
                adv = max(0.0, adv)
                out["junction"] = self._junction(use[0], adv, position)
                raw_turn = self._turn(use[0], adv, out["lanes"])
        out["lanes"]["turn"] = self._turn_shown(raw_turn, ts, out["lanes"], use is not None)
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
        if held and ts - src[2] > c["hold_approx_s"]:
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

    def _count(self, ln, ts):
        """The map's lane object whose count is shown: a new count after count_settle_s. (lanes, held)."""
        fwd = int(ln["forward"]) if _fin(ln.get("forward")) else None
        shown = self.lanes_shown
        if fwd is None:
            self.lanes_shown = self.lanes_pend = None
            return ln, False
        if shown is None or int(shown["forward"]) == fwd:
            self.lanes_shown, self.lanes_pend = ln, None
            return ln, False
        if self.lanes_pend is None or self.lanes_pend[0] != fwd:
            self.lanes_pend = (fwd, ts)
        if ts - self.lanes_pend[1] >= self.cfg["count_settle_s"]:
            self.lanes_shown, self.lanes_pend = ln, None
            return ln, False
        return shown, True

    def _lanes(self, use, ego, position, ts):
        """Lane count (the map's, of the trusted match), the ego lane (the filter's) and what a consumer lights."""
        c = self.cfg
        ego = ego if isinstance(ego, dict) else {}
        ln = ((use[0].get("road") or {}).get("lanes") if use is not None else None) or {}
        ln, count_held = self._count(ln, ts)
        fwd = int(ln["forward"]) if _fin(ln.get("forward")) else None
        probs, frm = ego.get("probs"), ego.get("probs_from")
        prov = list(ego.get("provenance") or ())
        out = {"count": ln.get("total"), "forward": fwd, "backward": ln.get("backward"),
               "count_source": ("map:" + ln["source"]) if ln.get("source") else None,
               "ego_index": None, "likely_index": None, "ego_from_right": ego.get("ego_from_right"),
               "probs": None, "probs_from": frm if probs else None, "confidence": 0.0,
               "oncoming_left": None, "provenance": prov, "approximate": bool(position and position["approximate"]),
               "count_held": count_held, "turn": None}
        if probs:
            out["probs"] = probs
        if fwd is None:
            self.lit = None
            return out
        keys = ("ego_index", "likely_index", "probs", "probs_from", "confidence")
        if fwd == 1:
            out.update(ego_index=0, confidence=1.0, probs=[1.0], probs_from="left")
            self.last_lit = (fwd, {k: out[k] for k in keys})
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
            self.last_lit = (fwd, {k: out[k] for k in keys})
        elif count_held and self.last_lit is not None and self.last_lit[0] == fwd:
            # the old count is still shown (the filter already counts the new one): what was lit stays with it
            out.update(self.last_lit[1])
        else:
            self.lit = self.last_lit = None
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
                "path": path, "path_source": "map",
                "here": start <= 0.0 and len(tl["lanes"]) == lanes.get("forward"),
                "provenance": "map:turn_lanes", "confidence": tl.get("confidence")}

    def _turn_shown(self, raw, ts, lanes, have_map):
        """The turn arrows as given (L36): only for as many lanes as the boxes count; a new stretch after
        turn_on_s of being seen, kept through a drop-out of turn_hold_s (held), its distance never growing."""
        c = self.cfg
        fwd = lanes.get("forward")
        if not have_map or fwd is None:
            self.tn = self.tn_cand = None                # no trusted map: nothing to hold on to
            return None
        sh = self.tn
        if sh is not None and sh["raw"]["count"] != fwd:
            self.tn = sh = None                          # the boxes count other lanes now
        if raw is None:
            self.tn_cand = None
        else:
            key = tuple(raw["arrows"]) + (raw["count"],)
            start = self.odo + raw["dist_m"]
            end = None if raw["junction_m"] is None else self.odo + raw["junction_m"]
            ref = end if end is not None else start
            fits = raw["count"] == fwd                   # arrows for other lanes than the boxes show are not given
            if sh is not None and sh["key"] == key and abs(ref - sh["ref"]) <= c["turn_same_m"]:
                # the same stretch: the distance only falls (a step back of the match is not shown)
                sh.update(raw=raw, seen=ts, ref=ref, start=min(start, sh["start"]),
                          end=end if sh["end"] is None or end is None else min(end, sh["end"]))
                self.tn_cand = None
            else:
                # a new stretch is timed from its first sighting, also while the boxes still count other lanes
                # (the count settles after count_settle_s; the arrows then come with it)
                cand = self.tn_cand
                if cand is None or cand[0] != key or abs(ref - cand[2]) > c["turn_same_m"]:
                    cand = self.tn_cand = (key, ts, ref)
                if fits and ts - cand[1] >= c["turn_on_s"]:
                    self.tn = sh = {"key": key, "raw": raw, "seen": ts, "start": start, "end": end, "ref": ref}
                    self.tn_cand = None
        if sh is not None:
            # the map no longer reports it and the junction is behind the car by the distance driven (while the
            # map still reports it the car is taken to be before it: odometry runs ahead of the match). The next
            # stretch waiting for turn_on_s takes its place without a blank in between
            gone = (sh["seen"] != ts and self.tn_cand is None and sh["end"] is not None
                    and sh["end"] - self.odo < -5.0)
            hold = c["turn_hold_s"] + (c["count_settle_s"] if self.tn_cand is not None else 0.0)
            if gone or ts - sh["seen"] > hold:
                self.tn = sh = None
        if sh is None:
            return None
        start = max(0.0, sh["start"] - self.odo)
        end = None if sh["end"] is None else max(0.0, sh["end"] - self.odo)
        return dict(sh["raw"], dist_m=round(start, 1), junction_m=None if end is None else round(end, 1),
                    here=start <= 0.0, held=sh["seen"] != ts)
