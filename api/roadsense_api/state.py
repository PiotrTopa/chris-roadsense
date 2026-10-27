"""In-memory state: latest message per type, rates, live/ended sign tracks, system readings."""
import collections
import glob
import json
import os
import struct
import time

from .speedlimit import SpeedLimit, load_rules

DAEMONS = ("camerad", "modeld", "signd", "envd")
TOPICS = ("camera", "road", "signs", "calib", "env", "speed_limit")
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


class State:
    def __init__(self, rundir, datadir=None, ring_path=None, rules="pl"):
        self.rundir = rundir
        self.ring_path = ring_path
        self.latest = {}                                   # type -> message dict
        self.arrivals = collections.defaultdict(lambda: collections.deque(maxlen=200))
        self.connected = {d: False for d in DAEMONS}
        self.last_msg = {d: None for d in DAEMONS}         # monotonic time of last message
        self.live_tracks = {}                              # track_id -> summary
        self.history = collections.deque(maxlen=HISTORY_MAX)
        self.history_file = None
        if datadir and os.path.isdir(datadir):
            self.history_file = os.path.join(datadir, "signs.jsonl")
        self.subscribers = set()                           # callables(type, line)
        self.version = 0                                   # bumps on every message (overlay cache key)
        self.speed = SpeedLimit(load_rules(rules))         # derived topic `speed_limit`
        self._speed_emit = 0.0
        self._speed_geom = False

    # ---- ingestion -------------------------------------------------------
    def on_message(self, daemon, msg, line):
        now = time.monotonic()
        mtype = msg.get("type")
        self.last_msg[daemon] = now
        if not isinstance(mtype, str):
            return
        self.latest[mtype] = msg
        self.arrivals[mtype].append(now)
        self.version += 1
        if mtype == "signs":
            self._on_signs(msg, now)
        for cb in list(self.subscribers):
            cb(mtype, line)
        if mtype in ("road", "signs", "env"):
            self._on_speed(msg, now)

    def _on_speed(self, msg, now):
        if not self._speed_geom:
            ring = self.ring()
            if ring:
                f, cx, _, w, _ = self.intrinsics()
                self.speed.cfg.update(focal_px=f, cx=cx, width=w)
                self._speed_geom = True
        try:
            changed = self.speed.on_message(msg)
        except Exception:                 # a malformed provider message must not stop the fan-out
            import logging
            logging.getLogger("roadsense_api.state").exception("speed_limit update failed")
            return
        if changed or now - self._speed_emit >= SPEED_LIMIT_HEARTBEAT_S:
            self._speed_emit = now
            out = self.speed.snapshot()
            self.latest["speed_limit"] = out
            self.arrivals["speed_limit"].append(now)
            line = json.dumps(out, separators=(",", ":"))
            for cb in list(self.subscribers):
                cb("speed_limit", line)

    def _on_signs(self, msg, now):
        for d in msg.get("detections") or []:
            tid = d.get("track_id")
            if tid is None:
                continue
            t = self.live_tracks.get(tid)
            if t is None:
                t = self.live_tracks[tid] = {"track_id": tid, "first_seq": msg.get("seq"),
                                             "best_score": 0.0}
            t.update(cls=d.get("cls"), cls_id=d.get("cls_id"), score=d.get("score"),
                     box=d.get("box"), hits=d.get("hits"), age=d.get("age"),
                     last_seq=msg.get("seq"), _seen=now)
            t["best_score"] = max(t["best_score"], d.get("score") or 0.0)
        for e in msg.get("tracks_ended") or []:
            self.live_tracks.pop(e.get("track_id"), None)
            self.history.append(e)
            if self.history_file:
                try:
                    with open(self.history_file, "a") as f:
                        f.write(json.dumps(e, separators=(",", ":")) + "\n")
                except OSError:
                    pass
        for tid in [k for k, t in self.live_tracks.items() if now - t["_seen"] > LIVE_TRACK_TIMEOUT_S]:
            del self.live_tracks[tid]

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
        return {
            "daemons": daemons,
            "fps": {t: self.fps(t) for t in TOPICS},
            "model_ms": road.get("model_ms"),
            "detect_ms": signs.get("detect_ms"),
            "env_ms": env.get("env_ms"),
            "env": {g: (env.get(g) or {}).get("label") for g in ("place", "weather", "light")} if env else None,
            "speed_limit": {"limit_kmh": sl["limit_kmh"], "source": sl["source"].get("kind"),
                            "sign": sl["source"].get("sign") or sl["source"].get("rule"),
                            "confidence": sl["confidence"], "area": sl["area"]["type"]} if sl else None,
            "camera_fps": (self.latest.get("camera") or {}).get("fps"),
            "cpu_temp_c": cpu_temp_c(),
            "battery": battery(),
        }
