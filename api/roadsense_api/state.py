"""In-memory state of the debug face: latest raw message per type, rates, the camera's frame ring, system
readings, the load governor. Nothing is tracked or fused here: the consolidated side (trackers, decisions, the
consumer API) is chris-integrator, which reads the same daemon sockets."""
import collections
import glob
import os
import struct
import time

from . import governor as governor_mod

DAEMONS = ("camerad", "modeld", "signd", "envd", "lightd")
TOPICS = ("camera", "road", "signs", "calib", "env", "lights", "display_lanes")

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


class State:
    def __init__(self, rundir, datadir=None, ring_path=None, rules=None):
        self.rundir = rundir
        self.ring_path = ring_path
        self.latest = {}                                   # type -> message dict
        self.arrivals = collections.defaultdict(lambda: collections.deque(maxlen=200))
        self.connected = {d: False for d in DAEMONS}
        self.last_msg = {d: None for d in DAEMONS}         # monotonic time of last message
        self.live_tracks = {}                              # sign track_id -> last detection (the detector's own tracks)
        self.subscribers = set()                           # callables(type, line)
        self.version = 0                                   # bumps on every message (overlay cache key)
        self.governor = governor_mod.Governor(governor_mod.load_config())   # load shedding under heat (governor.py)
        self._road_lat_ms = None                           # camera -> api latency of the last road message

    # ---- ingestion -------------------------------------------------------
    def on_message(self, daemon, msg, line):
        now = time.monotonic()
        mtype = msg.get("type")
        self.last_msg[daemon] = now
        if not isinstance(mtype, str):
            return
        if mtype == "road":
            ts = msg.get("ts_real_ns")
            self._road_lat_ms = (time.time_ns() - ts) / 1e6 if isinstance(ts, int) and ts > 0 else None
        if mtype == "signs":
            for d in msg.get("detections") or []:
                if d.get("track_id") is not None:
                    self.live_tracks[d["track_id"]] = dict(d, _seen=now)
            for e in msg.get("tracks_ended") or []:
                self.live_tracks.pop(e.get("track_id"), None)
            for tid in [k for k, t in self.live_tracks.items() if now - t["_seen"] > 2.0]:
                del self.live_tracks[tid]
        self.latest[mtype] = msg
        self.arrivals[mtype].append(now)
        self.version += 1
        for cb in list(self.subscribers):
            cb(mtype, line)

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

    def tracks(self):
        return [{k: v for k, v in t.items() if not k.startswith("_")}
                for t in sorted(self.live_tracks.values(), key=lambda t: t["track_id"])]

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

    def _governor_health(self):
        """The load governor's state plus what the raw measurement topics are delivering right now."""
        g = self.governor.state()
        g["topics"] = {"road": self.governor.topic_status("road", self.fps("road") or None, self._road_lat_ms)}
        return g

    def governor_message(self):
        """The governor's state as the `governor` message chris-integrator reads (api.sock): level, caps, whether
        the platform limiter is cutting clocks. The integrator marks the measurement topics with it."""
        g = self.governor
        return {"type": "governor", "ts_mono_ns": time.monotonic_ns(), "ts_real_ns": time.time_ns(),
                "enabled": bool(g.cfg["enabled"]), "level": g.level, "levels": len(g.steps), "shed": g.shed(),
                "caps": g.caps(), "limited": g.limited, "limiter": g.limiter, "forced": g.forced,
                "headroom_k": g.headroom_k, "hottest": g.hottest}

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
        lt = self.latest.get("lights") or {}
        from . import API_MAJOR, __version__
        return {
            "api": {"major": API_MAJOR, "version": __version__, "role": "debug face of one camera's perception"},
            "daemons": daemons,
            "fps": {t: self.fps(t) for t in TOPICS},
            "model_ms": road.get("model_ms"),
            "road_model": {"model": road.get("road_model"), "fallback": road.get("road_model_fallback"),
                           "plan": "plan" in road, "plan_age_ms": (road.get("plan") or {}).get("age_ms"),
                           "policy_hz": road.get("policy_hz"), "policy_ms": road.get("policy_ms")} if road else None,
            "calibration": self.latest.get("calib"),
            "detect_ms": signs.get("detect_ms"),
            "env_ms": env.get("env_ms"),
            "lights_ms": lt.get("lights_ms"),
            "camera_fps": (self.latest.get("camera") or {}).get("fps"),
            "cpu_temp_c": cpu_temp_c(),
            "governor": self._governor_health(),
            "battery": battery(),
        }
