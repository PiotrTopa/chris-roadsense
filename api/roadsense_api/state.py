"""In-memory state: latest message per type, rates, live/ended sign tracks, system readings."""
import collections
import glob
import json
import math
import os
import struct
import time

from . import governor as governor_mod
from . import lights as lights_mod
from . import signctx
from .fusion import Fusion
from .lanes import RoadGeometryTracker
from .lead import LeadTracker
from .roadgeom import RoadGeometryFusion
from .view import ViewBuilder
from .egolane import EgoLaneEstimator
from .rules import RulesInEffect
from .speedlimit import load_rules

DAEMONS = ("camerad", "modeld", "signd", "envd", "lightd", "mapd")
TOPICS = ("camera", "road", "signs", "calib", "env", "speed_limit", "lights", "rules", "map", "fused", "ego", "lead", "lanes",
          "vehicle", "objects", "view")
FUSED_PERIOD_S = 0.45
SPEED_LIMIT_HEARTBEAT_S = 1.0
HISTORY_MAX = 500
LIVE_TRACK_TIMEOUT_S = 2.0

# RingHeader as laid out by a C compiler (natural alignment: 4 pad bytes before write_seq).
RING_FMT = "<8s8I3f4xQQ"
RING_FIELDS = ("magic", "header_size", "slot_count", "slot_size", "width", "height",
               "y_stride", "uv_stride", "format", "focal_px", "cx", "cy",
               "write_seq", "camerad_pid")


def read_ring_header(path):
    """Return the frame ring header as a dict, or None if absent/invalid."""
    try:
        with open(path, "rb") as f:
            raw = f.read(struct.calcsize(RING_FMT))
        vals = struct.unpack(RING_FMT, raw)
    except (OSError, struct.error):
        return None
    hdr = dict(zip(RING_FIELDS, vals))
    if hdr["magic"] != b"RSFRAME1":
        return None
    hdr["magic"] = hdr["magic"].decode()
    return hdr


def cpu_temp_c():
    temps = []
    for p in glob.glob("/sys/class/thermal/thermal_zone*/temp"):
        try:
            with open(p) as f:
                temps.append(int(f.read().strip()) / 1000.0)
        except (OSError, ValueError):
            pass
    return max(temps) if temps else None


def battery():
    out = []
    for d in sorted(glob.glob("/sys/class/power_supply/*")):
        try:
            with open(os.path.join(d, "capacity")) as f:
                cap = int(f.read().strip())
        except (OSError, ValueError):
            continue
        try:
            with open(os.path.join(d, "status")) as f:
                status = f.read().strip()
        except OSError:
            status = None
        out.append({"name": os.path.basename(d), "capacity": cap, "status": status})
    return out


def _load_config(name, defaults):
    path = os.path.join(os.environ.get("ROADSENSE_CONFIG_DIR", "/etc/roadsense"), name)
    try:
        with open(path) as f:
            cfg = json.load(f)
    except (OSError, ValueError):
        return None
    if not isinstance(cfg, dict):
        return None
    return {k: v for k, v in cfg.items() if k in defaults}


def load_lead_config():
    """Optional overrides of the lead tracker's constants: `<config dir>/lead.json` (lead.py DEFAULT_CFG keys)."""
    from .lead import DEFAULT_CFG
    return _load_config("lead.json", DEFAULT_CFG)


def load_lanes_config():
    """Optional overrides of the road-geometry tracker's constants: `<config dir>/lanes.json` (lanes.py DEFAULT_CFG keys)."""
    from .lanes import DEFAULT_CFG
    return _load_config("lanes.json", DEFAULT_CFG)


def load_egolane_config():
    """Ego-lane estimator (egolane.py): DEFAULT_CFG overridden per key by <config dir>/egolane.json."""
    from .egolane import DEFAULT_CFG
    return _load_config("egolane.json", DEFAULT_CFG)


def load_view_config():
    """Display channel (view.py; its `road` object: roadview.py): view.DEFAULT_CFG overridden per key by
    <config dir>/view.json; the key `road` holds overrides of roadview.DEFAULT_CFG."""
    from .view import DEFAULT_CFG
    return _load_config("view.json", dict(DEFAULT_CFG, road={}))


def load_roadgeom_config():
    """Fused road geometry (roadgeom.py): DEFAULT_CFG overridden per key by <config dir>/roadgeom.json."""
    from .roadgeom import DEFAULT_CFG
    return _load_config("roadgeom.json", DEFAULT_CFG)


def load_lights_config():
    """Optional `<config dir>/lights.json`: {"lane_assign": false} switches the lane assignment of the ego
    light (lights.py) off, back to the lateral window alone."""
    return _load_config("lights.json", {"lane_assign": True}) or {}


class State:
    def __init__(self, rundir, datadir=None, ring_path=None, rules="pl"):
        self.rundir = rundir
        self.ring_path = ring_path
        self.latest = {}                                   # type -> message dict
        self.objects = {}                                  # range sensor id -> its last `objects` message
        self.arrivals = collections.defaultdict(lambda: collections.deque(maxlen=200))
        self.connected = {d: False for d in DAEMONS}
        self.last_msg = {d: None for d in DAEMONS}         # monotonic time of last message
        self.live_tracks = {}                              # track_id -> summary
        self.suppressed_tracks = {}                        # track_id -> summary (flagged, not live; signctx.py)
        self.signctx = signctx.SignContext()
        self.history = collections.deque(maxlen=HISTORY_MAX)
        self.history_file = None
        if datadir and os.path.isdir(datadir):
            self.history_file = os.path.join(datadir, "signs.jsonl")
        self.subscribers = set()                           # callables(type, line)
        self.version = 0                                   # bumps on every message (overlay cache key)
        self.rules_engine = RulesInEffect(load_rules(rules))   # derived topics `rules` and `speed_limit`
        self.speed = self.rules_engine.speed
        self._speed_emit = 0.0
        self._rules_emit = 0.0
        self._speed_geom = False
        self._rules = rules
        self._camera_key = None                            # (ride, ride_index) of the last camera message
        self._camera_seq = 0
        self.ride_changes = 0
        self.fusion = Fusion(load_rules(rules).get("defaults_kmh"))   # derived topic `fused` (fusion.py)
        self._fused_emit = 0.0
        self.lead = LeadTracker(load_lead_config())        # derived topic `lead` (lead.py)
        self.lanes = RoadGeometryTracker(load_lanes_config())   # derived topic `lanes` (lanes.py)
        self.roadgeom = RoadGeometryFusion(load_roadgeom_config())   # the road geometry inside `view.road` (roadgeom.py)
        self.view = ViewBuilder(load_view_config())        # derived display channel `view` (view.py, roadview.py)
        self.egolane = EgoLaneEstimator(load_egolane_config())  # ego lane over time, into `fused.lanes` (egolane.py)
        self.ego_light = lights_mod.EgoLightSelector()     # the ego traffic light over time (lights.py)
        self.ego_light.lane_assign = bool(load_lights_config().get("lane_assign", True))
        self.governor = governor_mod.Governor(governor_mod.load_config())   # load shedding under heat (governor.py)
        self._view_ts = None                               # frame time of the last `view` built (view_hz cap)
        self._road_lat_ms = None                           # camera -> api latency of the last road message

    def _rate(self, topic, msg):
        """The marker a measurement topic carries: measured rate, latency from the frame's camera time, and
        whether (and why) it is below what the consumer was promised (governor.py topic_status)."""
        ts = msg.get("ts_real_ns")
        lat = (time.time_ns() - ts) / 1e6 if isinstance(ts, int) and ts > 0 else None
        return self.governor.topic_status(topic, self.fps(topic) or None, lat)

    # ---- ingestion -------------------------------------------------------
    def on_message(self, daemon, msg, line):
        now = time.monotonic()
        mtype = msg.get("type")
        self.last_msg[daemon] = now
        if not isinstance(mtype, str):
            return
        if mtype == "lights":      # add ego-lane relevance from the road plan (lights.py), forward the result
            try:
                self.ego_light.annotate(msg, self.latest.get("road"), self.latest.get("calib"), self.intrinsics())
                msg["rate"] = self._rate("lights", msg)
                line = json.dumps(msg, separators=(",", ":"))
            except Exception:
                import logging
                logging.getLogger("roadsense_api.state").exception("lights annotation failed")
        if mtype == "signs":       # context checks on round red tracks (signctx.py), forward the result
            try:
                self.signctx.annotate(msg, self.latest.get("road"), self.latest.get("lights"), self.latest.get("calib"),
                                      self.intrinsics())
                msg["rate"] = self._rate("signs", msg)
                line = json.dumps(msg, separators=(",", ":"))
            except Exception:
                import logging
                logging.getLogger("roadsense_api.state").exception("sign context failed")
        if mtype == "camera":
            self._on_camera(msg)
        if mtype == "road":
            ts = msg.get("ts_real_ns")
            self._road_lat_ms = (time.time_ns() - ts) / 1e6 if isinstance(ts, int) and ts > 0 else None
        if mtype == "ego":         # the vehicle's own motion, relayed by mapd: input of the lead tracker
            self.view.on_ego(msg)
            self.lead.on_ego(msg)
            self.lanes.on_ego(msg)
            self.roadgeom.on_ego(msg)
            self.egolane.on_ego(msg)
        if mtype == "vehicle":     # what the host says about the car: the brake for the lead message
            self.lead.on_vehicle(msg)
        if mtype == "objects":     # a scan of a range sensor: the lead tracker's radar input (lead.json radar_fusion)
            self.objects[msg.get("sensor")] = msg
            self.lead.on_objects(msg)
        if mtype == "display_lanes":   # an external lane source for the display only (view.py, the lane-source seam)
            self.view.on_display_lanes(msg)
        if mtype == "map":         # lane count of the matched road: a soft prior for the outer lane lines
            self.lanes.on_map(msg)
            self.roadgeom.on_map(msg)
            self.view.on_map(msg)
            self.egolane.on_map(msg)
            self.ego_light.on_map(msg)
        self.latest[mtype] = msg
        self.arrivals[mtype].append(now)
        self.version += 1
        if mtype == "signs":
            self._on_signs(msg, now)
        if mtype in ("lights", "signs"):   # display channel: static objects (never feeds back into the topics)
            try:
                if mtype == "lights":
                    self.view.on_lights(msg)
                else:
                    self.view.on_signs(msg, self.intrinsics())
            except Exception:
                import logging
                logging.getLogger("roadsense_api.state").exception("view (%s) failed", mtype)
        for cb in list(self.subscribers):
            cb(mtype, line)
        if mtype == "road":
            try:
                out = self.lead.on_road(msg)
                if out is not None:
                    out["rate"] = self._rate("lead", msg)
                    self._emit("lead", out, now)
            except Exception:                 # a malformed road message must not stop the fan-out
                import logging
                logging.getLogger("roadsense_api.state").exception("lead tracker failed")
            lanes_out = geom = None
            try:
                out = lanes_out = self.lanes.on_road(msg)
                if out is not None:
                    self.ego_light.on_lanes(out)      # the lane lines the ego light is assigned by
                    out["rate"] = self._rate("lanes", msg)
                    self._emit("lanes", out, now)
                    self.latest["egolane"] = self.egolane.on_lanes(out, msg)
                    geom = self.roadgeom.on_lanes(out, msg)    # no topic of its own since 2.0: part of `view.road`
            except Exception:
                import logging
                logging.getLogger("roadsense_api.state").exception("road-geometry tracker failed")
            try:                              # the display channel, last: derived from the topics above
                out = None
                cap, ts = self.governor.caps().get("view_hz", -1), msg.get("ts_mono_ns")
                if (cap < 0 or self._view_ts is None or not isinstance(ts, int) or ts < self._view_ts
                        or (cap > 0 and ts - self._view_ts >= (1.0 / cap - 0.01) * 1e9)):
                    self._view_ts = ts
                    out = self.view.on_frame(msg, self.latest.get("lead"), lanes_out, geom, self.latest.get("egolane"))
                if out is not None:
                    self._emit("view", out, now)
            except Exception:
                import logging
                logging.getLogger("roadsense_api.state").exception("view failed")
        if mtype in ("road", "signs", "env", "lights"):
            self._on_speed(msg, now)
        if now - self._fused_emit >= FUSED_PERIOD_S and mtype in ("map", "road", "env", "signs"):
            self._fused_emit = now
            try:
                self._emit("fused", self.fusion.update(self.latest, time.monotonic_ns(), time.time_ns()), now)
            except Exception:                 # a malformed map message must not stop the fan-out
                import logging
                logging.getLogger("roadsense_api.state").exception("fusion failed")

    def _on_camera(self, msg):
        """A new ride (camerad-replay --playlist) or a camera restart (seq backwards) is not driving on:
        drop the speed-limit state (odometer, limit, area, tracks), re-read the focal from the new ring,
        and forget sign tracks and their context. Providers reset themselves on the seq restart."""
        key = (msg.get("ride"), msg.get("ride_index"))
        seq = msg.get("seq") or 0
        changed = self._camera_key is not None and key != self._camera_key
        restarted = seq < self._camera_seq
        self._camera_key, self._camera_seq = key, seq
        if changed or restarted:
            self.reset_drive()

    def reset_drive(self):
        self.ride_changes += 1
        self.rules_engine = RulesInEffect(load_rules(self._rules))
        self.speed = self.rules_engine.speed
        self._speed_geom = False
        self.live_tracks.clear()
        self.suppressed_tracks.clear()
        self.signctx = signctx.SignContext()
        self.fusion = Fusion(load_rules(self._rules).get("defaults_kmh"))
        self.lead.reset()
        self.lanes.reset()
        self.roadgeom.reset()
        self.view.reset()
        self.egolane.reset()
        self.ego_light.reset()

    def _on_speed(self, msg, now):
        if not self._speed_geom:
            ring = self.ring()
            if ring:
                f, cx, _, w, _ = self.intrinsics()
                self.speed.cfg.update(focal_px=f, cx=cx, width=w)
                self._speed_geom = True
        sv = self.speed.version
        try:
            rules_changed = self.rules_engine.on_message(msg)   # feeds the speed-limit engine too
        except Exception:                 # a malformed provider message must not stop the fan-out
            import logging
            logging.getLogger("roadsense_api.state").exception("rules/speed_limit update failed")
            return
        if self.speed.version != sv or now - self._speed_emit >= SPEED_LIMIT_HEARTBEAT_S:
            self._speed_emit = now
            self._emit("speed_limit", self.speed.snapshot(), now)
        if rules_changed or now - self._rules_emit >= SPEED_LIMIT_HEARTBEAT_S:
            self._rules_emit = now
            self._emit("rules", self.rules_engine.snapshot(), now)

    def _emit(self, topic, out, now):
        self.latest[topic] = out
        self.arrivals[topic].append(now)
        line = json.dumps(out, separators=(",", ":"))
        for cb in list(self.subscribers):
            cb(topic, line)

    def _on_signs(self, msg, now):
        for d in msg.get("detections") or []:
            tid = d.get("track_id")
            if tid is None:
                continue
            # A flagged track (provider or signctx) is listed apart with its reason, never as a live sign.
            src, other = (self.suppressed_tracks, self.live_tracks) if d.get("suppressed") else \
                (self.live_tracks, self.suppressed_tracks)
            t = other.pop(tid, None) or src.get(tid)
            if t is None:
                t = {"track_id": tid, "first_seq": msg.get("seq"), "best_score": 0.0}
            src[tid] = t
            t.update(cls=d.get("cls"), cls_id=d.get("cls_id"), score=d.get("score"),
                     box=d.get("box"), hits=d.get("hits"), age=d.get("age"),
                     last_seq=msg.get("seq"), _seen=now)
            for k in ("suppressed", "reason", "context", "glow", "verify"):
                if k in d:
                    t[k] = d[k]
                else:
                    t.pop(k, None)
            t["best_score"] = max(t["best_score"], d.get("score") or 0.0)
        for e in msg.get("tracks_ended") or []:
            self.live_tracks.pop(e.get("track_id"), None)
            self.suppressed_tracks.pop(e.get("track_id"), None)
            self.history.append(e)
            if self.history_file:
                try:
                    with open(self.history_file, "a") as f:
                        f.write(json.dumps(e, separators=(",", ":")) + "\n")
                except OSError:
                    pass
        for tracks in (self.live_tracks, self.suppressed_tracks):
            for tid in [k for k, t in tracks.items() if now - t["_seen"] > LIVE_TRACK_TIMEOUT_S]:
                del tracks[tid]

    # ---- queries ---------------------------------------------------------
    def fps(self, mtype, window=2.0):
        q = self.arrivals.get(mtype)
        if not q:
            return 0.0
        now = time.monotonic()
        recent = [t for t in q if now - t <= window]
        if len(recent) < 2:
            return 0.0
        span = now - recent[0]
        return round((len(recent) - 1) / span, 2) if span > 0 else 0.0

    def tracks(self, suppressed=False):
        src = self.suppressed_tracks if suppressed else self.live_tracks
        return [{k: v for k, v in t.items() if not k.startswith("_")}
                for t in sorted(src.values(), key=lambda t: t["track_id"])]

    def ring(self):
        return read_ring_header(self.ring_path) if self.ring_path else None

    def intrinsics(self):
        """(focal_px, cx, cy, width, height) at native sensor resolution."""
        ring = self.ring() or {}
        calib = self.latest.get("calib") or {}
        w = ring.get("width") or 1920
        h = ring.get("height") or 1080
        f = ring.get("focal_px") or calib.get("focal_px") or 1500.0
        cx = ring.get("cx") or w / 2.0
        cy = ring.get("cy") or h / 2.0
        return float(f), float(cx), float(cy), int(w), int(h)

    def calibration_summary(self):
        """The camera calibration modeld warps with, for /health (lane L32): the configured calib.json or modeld's own
        online estimate of pitch / yaw (`source` "auto"), the estimate's state, and how far it is from the configured
        value. A consumer that sees `online.state` "valid" with `applied` false (mode observe) or a large `off_config_deg`
        knows calib.json is wrong by that much."""
        c = self.latest.get("calib")
        if not c:
            return None
        out = {"source": c.get("source"), "pitch": c.get("pitch"), "yaw": c.get("yaw"), "roll": c.get("roll"),
               "height_m": c.get("height_m"), "online": None}
        o = c.get("online")
        if o:
            cfg = o.get("config") or {}
            off = None
            if o.get("pitch") is not None and o.get("yaw") is not None:
                off = {"pitch": round(math.degrees(o["pitch"] - (cfg.get("pitch") or 0.0)), 2),
                       "yaw": round(math.degrees(o["yaw"] - (cfg.get("yaw") or 0.0)), 2)}
            sp = None
            if o.get("spread_pitch") is not None and o.get("spread_yaw") is not None:
                sp = {"pitch": round(math.degrees(o["spread_pitch"]), 2), "yaw": round(math.degrees(o["spread_yaw"]), 2)}
            out["online"] = {"state": o.get("state"), "applied": o.get("applied"), "pitch": o.get("pitch"),
                             "yaw": o.get("yaw"), "off_config_deg": off, "spread_deg": sp, "blocks": o.get("blocks"),
                             "min_blocks": o.get("min_blocks"), "restored": o.get("restored")}
        return out

    def vehicle_summary(self):
        """Short dict for /v1/health: is a host feeding vehicle data, and what."""
        v = self.latest.get("vehicle")
        if not v:
            return None
        return {"present": v.get("present"), "feeders": [f.get("name") for f in v.get("feeders") or []],
                "signals": sorted(k for k, s in (v.get("signals") or {}).items()
                                  if s.get("age_s") is not None and s["age_s"] < 15.0),
                "yaw_source": (v.get("motion") or {}).get("yaw_source"),
                "sensors": [s.get("id") for s in v.get("sensors") or []]}

    def _map_summary(self, now):
        m = self.latest.get("map")
        if not m:
            return None
        pos, road, lim = m.get("position") or {}, m.get("road") or {}, m.get("speed_limit") or {}
        inp = m.get("input") or {}
        return {"state": m.get("state"), "available": m.get("available"), "reason": m.get("reason"),
                "pack": (m.get("pack") or {}).get("osm_timestamp"),
                "hacc_m": pos.get("hacc_m"), "motion_source": (inp.get("motion") or {}).get("source"),
                "gnss_age_s": (inp.get("gnss") or {}).get("age_s"),
                "match_confidence": (m.get("match") or {}).get("confidence"),
                "road": road.get("ref") or road.get("name") or road.get("class"),
                "limit_kmh": lim.get("limit_kmh"), "limit_source": lim.get("source"),
                "cpu_pct": (m.get("proc") or {}).get("cpu_pct")}

    def _governor_health(self):
        """The load governor's state plus what each measurement topic is delivering right now."""
        g = self.governor.state()
        topics = {}
        for t in ("road", "lead", "lanes", "lights", "signs"):
            m = self.latest.get(t) or {}
            if t == "road":             # modeld's own line is forwarded untouched: the marker is built here
                topics[t] = self.governor.topic_status(t, self.fps(t) or None, self._road_lat_ms)
            else:
                r = m.get("rate")
                topics[t] = self.governor.topic_status(t, self.fps(t) or None, (r or {}).get("latency_ms"))
        g["topics"] = topics
        return g

    def health(self):
        now = time.monotonic()
        daemons = {}
        for d in DAEMONS:
            last = self.last_msg[d]
            daemons[d] = {"connected": self.connected[d],
                          "last_msg_age_s": round(now - last, 3) if last else None}
        road = self.latest.get("road") or {}
        signs = self.latest.get("signs") or {}
        env = self.latest.get("env") or {}
        sl = self.latest.get("speed_limit")
        lt = self.latest.get("lights") or {}
        from . import API_MAJOR, __version__
        return {
            "api": {"major": API_MAJOR, "version": __version__},
            "daemons": daemons,
            "fps": {t: self.fps(t) for t in TOPICS},
            "model_ms": road.get("model_ms"),
            # which road model runs and where its plan comes from (lane L27): "vision" (openpilot v0.11.1
            # driving_vision + driving_policy, the default) or "supercombo" (v0.9.7); `fallback` = modeld was not told
            # a model and the default one's files were missing, so it runs the other
            "road_model": {"model": road.get("road_model"), "fallback": road.get("road_model_fallback"),
                           "plan": "plan" in road, "plan_age_ms": (road.get("plan") or {}).get("age_ms"),
                           "policy_hz": road.get("policy_hz"), "policy_ms": road.get("policy_ms")} if road else None,
            "calibration": self.calibration_summary(),
            "detect_ms": signs.get("detect_ms"),
            "env_ms": env.get("env_ms"),
            "env": {g: (env.get(g) or {}).get("label") for g in ("place", "weather", "light")} if env else None,
            "speed_limit": {"limit_kmh": sl["limit_kmh"], "source": sl["source"].get("kind"),
                            "sign": sl["source"].get("sign") or sl["source"].get("rule"),
                            "confidence": sl["confidence"], "area": sl["area"]["type"]} if sl else None,
            "lights_ms": lt.get("lights_ms"),
            "lights": lt.get("ego") if lt else None,
            "rules": self.rules_engine.summary() if "rules" in self.latest else None,
            "map": self._map_summary(now),
            "fused": self.fusion.summary(self.latest["fused"]) if "fused" in self.latest else None,
            "lead": self.lead.summary(self.latest["lead"]) if "lead" in self.latest else None,
            "lanes": self.lanes.summary(self.latest["lanes"]) if "lanes" in self.latest else None,
            "view": self.view.summary(self.latest.get("view")),
            "vehicle": self.vehicle_summary(),
            "camera_fps": (self.latest.get("camera") or {}).get("fps"),
            "cpu_temp_c": cpu_temp_c(),
            "governor": self._governor_health(),
            "battery": battery(),
        }
