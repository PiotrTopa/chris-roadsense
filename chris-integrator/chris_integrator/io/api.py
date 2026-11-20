"""What the consolidated API serves besides the streams themselves: the health summary in the shape consumers
know, the sign-track history, the last scan of every range sensor. No web framework in here."""
import asyncio
import collections
import glob
import json
import math
import os

from ..native import status as native_status
from .. import API_MAJOR, __version__
from ..plugins.estimator import Estimator, lead_summary

ROADSENSE_DAEMONS = ("camerad", "modeld", "signd", "envd", "lightd")
HISTORY_MAX = 500
# the topics consumers know (api 2), in the order /v2/health lists their rates
TOPICS = ("camera", "road", "signs", "calib", "env", "speed_limit", "lights", "rules", "map", "fused", "ego", "lead", "lanes",
          "vehicle", "objects", "view", "ego_state", "gnss", "tracks")


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


def calibration_summary(c):
    """The camera calibration the road model warps with: the configured one or the model's own online estimate
    of pitch / yaw (`source` "auto"), the estimate's state, and how far it is from the configured value."""
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
        se = None
        if o.get("se_pitch") is not None and o.get("se_yaw") is not None:
            se = {"pitch": round(math.degrees(o["se_pitch"]), 2), "yaw": round(math.degrees(o["se_yaw"]), 2)}
        out["online"] = {"state": o.get("state"), "applied": o.get("applied"), "pitch": o.get("pitch"),
                         "yaw": o.get("yaw"), "off_config_deg": off, "spread_deg": sp, "error_deg": se,
                         "blocks": o.get("blocks"),
                         "min_blocks": o.get("min_blocks"), "restored": o.get("restored")}
    return out


class Api:
    """What the handlers read: the stack, the per-socket readers (for `daemons`), sign history, range scans."""

    def __init__(self, stack, readers=None, datadir=None, feed=None, recorder=None):
        self.stack, self.mixer, self.bus = stack, stack.mixer, stack.mixer.bus
        self.readers = readers if readers is not None else {}      # daemon name -> io.sockets.Reader
        self.feed = feed                   # callable(list of feeder messages) -> {"accepted", "rejected"}
        self.recorder = recorder
        self.objects = {}                  # range sensor id -> its last `objects` message
        self.subscribers = set()           # callables(stream, line) that take every public stream
        self._by_topic = {}                # stream -> [callable(stream, line)]: subscribers of named streams
        self.history = collections.deque(maxlen=HISTORY_MAX)
        self.history_file = os.path.join(datadir, "signs.jsonl") if datadir and os.path.isdir(datadir) else None
        sc = self.mixer.plugin("sign_context")
        if sc is not None:
            sc.history = self._sign_ended
        self.mixer.add_output("api", self._on_envelope, info="HTTP /v2 and WebSocket /ws")

    def latest(self, name):
        return self.bus.get(name)

    def subscribe(self, cb, topics=None):
        """`cb(stream, line)` for every message of the public streams `topics` (None: of all of them). A message
        is serialised only when a subscriber of its stream is there."""
        if not topics:
            self.subscribers.add(cb)
            return
        for t in topics:
            self._by_topic.setdefault(t, []).append(cb)

    def unsubscribe(self, cb):
        self.subscribers.discard(cb)
        for t in [t for t, cbs in self._by_topic.items() if cb in cbs]:
            self._by_topic[t].remove(cb)
            if not self._by_topic[t]:
                del self._by_topic[t]

    def _on_envelope(self, env):
        stream = env.stream
        if stream == "objects":
            self.objects[env.msg.get("sensor")] = env.msg
        if self.subscribers:
            line = env.line
            for cb in list(self.subscribers):
                cb(stream, line)
        named = self._by_topic.get(stream)
        if named:
            line = env.line
            for cb in tuple(named):
                cb(stream, line)

    def _sign_ended(self, e):
        self.history.append(e)
        if self.history_file:
            try:
                with open(self.history_file, "a") as f:
                    f.write(json.dumps(e, separators=(",", ":")) + "\n")
            except OSError:
                pass

    # ---- health ----------------------------------------------------------------------------------------
    def rate(self, topic, latency_ms=None):
        return self.mixer.services["rates"].status(topic, None, latency_ms)

    def governor_health(self):
        """The load governor of the roadsense instance (an input) plus what each measurement topic delivers."""
        g = dict(self.latest("governor") or {"enabled": False, "level": 0, "caps": {}, "limited": False,
                                             "reason": "no chris-roadsense governor connected"})
        g.pop("type", None)
        topics = {}
        clk = self.bus.clock
        for t in ("road", "lead", "lanes", "lights", "signs"):
            m = self.latest(t) or {}
            if t == "road":                # the road model's own line is passed on untouched: the marker is made here
                e = self.bus.latest.get("road")
                ts = m.get("ts_real_ns")
                lat = None
                if e is not None and isinstance(ts, int) and ts > 0:
                    lat = (clk.real_ns() - (clk.now() - e.t_arr) * 1e9 - ts) / 1e6
                topics[t] = self.rate(t, lat)
            else:
                topics[t] = self.rate(t, (m.get("rate") or {}).get("latency_ms"))
        g["topics"] = topics
        return g

    def vehicle_summary(self):
        v = self.latest("vehicle")
        if not v:
            return None
        return {"present": v.get("present"), "feeders": [f.get("name") for f in v.get("feeders") or []],
                "signals": sorted(k for k, s in (v.get("signals") or {}).items()
                                  if s.get("age_s") is not None and s["age_s"] < 15.0),
                "yaw_source": (v.get("motion") or {}).get("yaw_source"),
                "sensors": [s.get("id") for s in v.get("sensors") or []]}

    def map_summary(self):
        m = self.latest("map")
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

    def health(self):
        L, P = self.latest, self.mixer.plugin
        try:
            loop_now = asyncio.get_running_loop().time()
        except RuntimeError:               # no loop: a replay or a test
            loop_now = 0.0
        daemons = {d: (self.readers[d].status(loop_now) if d in self.readers else {"connected": False, "last_msg_age_s": None})
                   for d in ROADSENSE_DAEMONS}
        if "map" in self.bus.streams:      # the map box sits where the mapd daemon was: same key for consumers
            me = self.bus.latest.get("map")
            daemons["mapd"] = {"connected": True,
                               "last_msg_age_s": None if me is None else round(self.bus.clock.now() - me.t_arr, 3)}
        road, signs, env, sl, lt = L("road") or {}, L("signs") or {}, L("env") or {}, L("speed_limit"), L("lights") or {}
        rules, fused, lanes, view = P("rules"), P("fused"), P("lanes"), P("view")
        return {
            "api": {"major": API_MAJOR, "version": __version__, "server": "chris-integrator"},
            # which implementation of the hot path runs: the native core or pure Python (and why)
            "native": native_status(),
            "daemons": daemons,
            "fps": {t: self.mixer.fps(t) for t in TOPICS},
            "model_ms": road.get("model_ms"),
            "road_model": {"model": road.get("road_model"), "fallback": road.get("road_model_fallback"),
                           "plan": "plan" in road, "plan_age_ms": (road.get("plan") or {}).get("age_ms"),
                           "policy_hz": road.get("policy_hz"), "policy_ms": road.get("policy_ms")} if road else None,
            "calibration": calibration_summary(L("calib")),
            "detect_ms": signs.get("detect_ms"),
            "env_ms": env.get("env_ms"),
            "env": {g: (env.get(g) or {}).get("label") for g in ("place", "weather", "light")} if env else None,
            "speed_limit": {"limit_kmh": sl["limit_kmh"], "source": sl["source"].get("kind"),
                            "sign": sl["source"].get("sign") or sl["source"].get("rule"),
                            "confidence": sl["confidence"], "area": sl["area"]["type"]} if sl else None,
            "lights_ms": lt.get("lights_ms"),
            "lights": lt.get("ego") if lt else None,
            "rules": rules.engine.summary() if rules is not None and L("rules") else None,
            "map": self.map_summary(),
            "fused": fused.fusion.summary(L("fused")) if fused is not None and L("fused") else None,
            "lead": lead_summary(L("lead")) if L("lead") else None,
            "lanes": lanes.tracker.summary(L("lanes")) if lanes is not None and L("lanes") else None,
            "view": view.builder.summary(L("view")) if view is not None else None,
            "vehicle": self.vehicle_summary(),
            "integrator": Estimator.summary(L("ego_state")),
            "camera_fps": (L("camera") or {}).get("fps"),
            "cpu_temp_c": cpu_temp_c(),
            "governor": self.governor_health(),
            "battery": battery(),
            "recorder": self.recorder.status() if self.recorder is not None else None,
            "mixer": {"plugins": {p["name"]: dict(p["health"], enabled=p["enabled"], errors=p["errors"],
                                                  ms_per_call=p["ms_per_call"]) for p in self.mixer.describe()["plugins"]}},
        }
