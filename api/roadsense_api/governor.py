"""Load governor: one policy for what roadsense gives up when the platform runs out of thermal headroom.

The platform's own limiter (kernel thermal zones with passive trips, hardware clock limiters) cuts clocks
blindly: every stage slows, the road model with the rest. The governor acts before that: it reads the
platform's temperatures and limiter state through a probe, and when the headroom to the limit gets small it
sheds work in a fixed order of rising importance, one step at a time, and gives it back with hysteresis.
The road model (road / lead / lanes) is the last thing touched.

    probe (pluggable) -> Governor.step() -> <rundir>/governor (flat JSON, read by every daemon)
                                         -> /v2/health "governor", `governor` in the measurement topics

Nothing here knows a device: the default probe reads the generic Linux thermal sysfs (zones, their trip
points, cooling devices); a host with other sensors writes a reading file or names its own probe class in
`<config dir>/governor.json`.

Caps file `<rundir>/governor` (rewritten atomically at least every 2 s, ignored by the daemons when older
than `valid_s`, so a dead governor means the daemons run as configured and the platform limiter is the only
guard):

    {"v":1,"ts_mono_ns":...,"valid_s":10,"level":3,"limited":0,"headroom_k":4.2,
     "preview_hz":1,"record_frames":0,"env_hz":0.2,"view_hz":-1,"signs_hz":-1,"lights_hz":-1,"road_hz":-1}

A cap of -1 leaves the daemon's own configured rate; 0 pauses the stage; any other value is an upper bound
(the daemon runs at min(configured, cap)).
"""
import glob
import importlib
import json
import logging
import os
import time

log = logging.getLogger("roadsense_api.governor")

CAP_KEYS = ("preview_hz", "record_frames", "env_hz", "view_hz", "signs_hz", "lights_hz", "road_hz")

# Shed order, rising importance. Each step adds caps to those of the steps before it (a later step may
# tighten an earlier cap). `what` is the name reported in health and in the event list.
DEFAULT_STEPS = (
    {"what": "preview", "caps": {"preview_hz": 1}},                  # the camera picture for people: 1 a second
    {"what": "recorder_frames", "caps": {"record_frames": 0, "preview_hz": 0}},   # recorder keeps messages, no pictures
    {"what": "scenery", "caps": {"env_hz": 0.2}},                    # place / weather / light: one look in 5 s
    {"what": "view", "caps": {"view_hz": 5}},                        # display channel
    {"what": "signs", "caps": {"signs_hz": 2.5}},                    # sign detector at half rate
    {"what": "lights", "caps": {"lights_hz": 2.5}},                  # light detector at half rate
    {"what": "detectors_floor", "caps": {"env_hz": 0, "signs_hz": 1.25, "lights_hz": 1.25}},
    {"what": "road", "caps": {"road_hz": 10}},                       # last: the road model at half rate
)

DEFAULT_CFG = {
    "enabled": True,
    "probe": "sysfs",          # "sysfs" | "file:/path/reading.json" | "package.module:ClassName"
    "zones": "",               # regular expression on zone names; empty = every zone that has a limit
    "limit_c": None,           # limit for zones without a passive / hot trip point (None: such zones are ignored)
    "period_s": 1.0,
    "tau_s": 4.0,              # time constant of the per-zone average (the zones swing with every inference burst)
    "shed_at_k": 4.0,          # shed one step when the headroom to the limit falls below this
    "urgent_k": 1.5,           # ... and without waiting the full dwell below this, or while the limiter is active
    "restore_at_k": 9.0,       # give one step back after the headroom stayed above this for restore_hold_s
    "shed_dwell_s": 8.0,       # time a step is given to show before the next one
    "urgent_dwell_s": 3.0,
    "restore_hold_s": 30.0,
    "restore_backoff": 2.0,    # a step that had to be shed again within regret_s is held this much longer next time
    "regret_s": 90.0,
    "restore_hold_max_s": 600.0,
    "valid_s": 10.0,
    "max_level": None,         # None = all steps; e.g. 7 never touches the road model
    "steps": None,             # None = DEFAULT_STEPS
    # what a measurement topic must deliver not to be called degraded (rate in messages a second over the
    # last 2 s; latency from the frame's camera time to the api)
    "floors": {"road": {"hz": 10.0, "latency_ms": 250.0}, "lead": {"hz": 10.0, "latency_ms": 250.0},
               "lanes": {"hz": 10.0, "latency_ms": 250.0}, "lights": {"hz": 3.0, "latency_ms": 400.0},
               "signs": {"hz": 3.0, "latency_ms": 400.0}},
}

# topic -> the caps that shed it
TOPIC_CAPS = {"road": ("road_hz",), "lead": ("road_hz",), "lanes": ("road_hz",), "lights": ("lights_hz",),
              "signs": ("signs_hz",), "env": ("env_hz",), "view": ("view_hz",)}


# ---- probes ---------------------------------------------------------------------------------------------
class Reading:
    """One look at the platform: zones [(name, temp_c, limit_c or None)], limiter [(name, state, max_state)]."""

    def __init__(self, zones=(), limiter=()):
        self.zones = list(zones)
        self.limiter = list(limiter)

    @property
    def limited(self):
        return any(cur > 0 for _, cur, _ in self.limiter)


class SysfsThermalProbe:
    """Generic Linux: /sys/class/thermal. A zone's limit is its lowest `passive` trip point (where the kernel
    starts to cut clocks), else its lowest `hot` trip; the limiter is every cooling device (cpufreq, devfreq,
    ...) with its current state."""

    def __init__(self, cfg=None, root="/sys/class/thermal"):
        self.root = root
        self._zones = None          # [(name, temp path, limit)]
        self._cooling = None

    def _read(self, path):
        with open(path) as f:
            return f.read().strip()

    def _scan(self):
        self._zones, self._cooling = [], []
        for z in sorted(glob.glob(os.path.join(self.root, "thermal_zone*"))):
            try:
                name = self._read(os.path.join(z, "type"))
            except OSError:
                continue
            trips = {}
            for tp in glob.glob(os.path.join(z, "trip_point_*_type")):
                try:
                    kind = self._read(tp)
                    val = int(self._read(tp[:-4] + "temp")) / 1000.0
                except (OSError, ValueError):
                    continue
                if val > 0:
                    trips.setdefault(kind, []).append(val)
            limit = min(trips["passive"]) if trips.get("passive") else min(trips["hot"]) if trips.get("hot") else None
            self._zones.append((name, os.path.join(z, "temp"), limit))
        for c in sorted(glob.glob(os.path.join(self.root, "cooling_device*"))):
            try:
                self._cooling.append((self._read(os.path.join(c, "type")), os.path.join(c, "cur_state"),
                                      int(self._read(os.path.join(c, "max_state")))))
            except (OSError, ValueError):
                continue

    def read(self):
        if self._zones is None:
            self._scan()
        zones, limiter = [], []
        for name, path, limit in self._zones:
            try:
                t = int(self._read(path)) / 1000.0
            except (OSError, ValueError):
                continue
            if -40.0 < t < 200.0:
                zones.append((name, t, limit))
        for name, path, mx in self._cooling:
            try:
                limiter.append((name, int(self._read(path)), mx))
            except (OSError, ValueError):
                continue
        return Reading(zones, limiter)


class FileProbe:
    """A reading written by the host (or a test): {"zones": [{"name":..,"temp_c":..,"limit_c":..}],
    "limiter": [{"name":..,"state":..,"max":..}]}. A file older than max_age_s gives an empty reading."""

    def __init__(self, path, max_age_s=10.0):
        self.path, self.max_age_s = path, max_age_s

    def read(self):
        try:
            if time.time() - os.stat(self.path).st_mtime > self.max_age_s:
                return Reading()
            with open(self.path) as f:
                d = json.load(f)
        except (OSError, ValueError):
            return Reading()
        try:
            return Reading([(z.get("name", "?"), float(z["temp_c"]), z.get("limit_c")) for z in d.get("zones") or []],
                           [(c.get("name", "?"), int(c.get("state", 0)), int(c.get("max", 1)))
                            for c in d.get("limiter") or []])
        except (KeyError, TypeError, ValueError):
            return Reading()


def make_probe(cfg):
    spec = (cfg or {}).get("probe") or "sysfs"
    if spec == "sysfs":
        return SysfsThermalProbe(cfg)
    if spec.startswith("file:"):
        return FileProbe(spec[5:])
    mod, _, cls = spec.partition(":")
    return getattr(importlib.import_module(mod), cls)(cfg)


# ---- policy ---------------------------------------------------------------------------------------------
class Governor:
    def __init__(self, cfg=None, clock=time.monotonic, wall=time.time):
        self.cfg = dict(DEFAULT_CFG)
        for k, v in (cfg or {}).items():
            if k == "floors" and isinstance(v, dict):
                self.cfg["floors"] = {**DEFAULT_CFG["floors"], **v}
            elif k in DEFAULT_CFG:
                self.cfg[k] = v
        self.steps = [dict(s) for s in (self.cfg["steps"] or DEFAULT_STEPS)]
        self.max_level = len(self.steps) if self.cfg["max_level"] is None else max(0, min(len(self.steps),
                                                                                            int(self.cfg["max_level"])))
        self.clock, self.wall = clock, wall
        self.level = 0
        self.forced = None                 # a level held by hand (tests, measurements); None = automatic
        self.events = []                   # the last 32 steps: what was shed or restored, when, at what headroom
        self.shed_count = self.restore_count = 0
        self._ema = {}                     # zone name -> averaged temperature
        self._t_last = None
        self._t_step = -1e9                # time of the last level change
        self._t_shed = -1e9                # time of the last shed step (the dwell runs from it)
        self._above_since = None           # since when the headroom has been above restore_at_k
        self._hold = {}                    # level -> restore hold in s (grows when a restore was regretted)
        self._restored_at = {}             # level -> time it was last given back
        self.headroom_k = None
        self.hottest = None
        self.limited = False
        self.limiter = []
        self._level_since = clock()

    # -- reading -> headroom
    def _observe(self, reading, now):
        import re
        dt = self.cfg["period_s"] if self._t_last is None else max(1e-3, now - self._t_last)
        self._t_last = now
        a = 1.0 if not self.cfg["tau_s"] else min(1.0, dt / float(self.cfg["tau_s"]))
        pat = re.compile(self.cfg["zones"]) if self.cfg["zones"] else None
        best = None
        seen = set()
        for name, t, limit in reading.zones:
            if limit is None:
                limit = self.cfg["limit_c"]
            if limit is None or (pat and not pat.search(name)):
                continue
            seen.add(name)
            e = self._ema.get(name)
            e = t if e is None else e + a * (t - e)
            self._ema[name] = e
            h = float(limit) - e
            if best is None or h < best[0]:
                best = (h, name, e, float(limit))
        for name in list(self._ema):
            if name not in seen:
                del self._ema[name]
        self.limiter = [{"name": n, "state": c, "max": m} for n, c, m in reading.limiter if c > 0]
        self.limited = bool(self.limiter)
        if best is None:
            self.headroom_k, self.hottest = None, None
        else:
            self.headroom_k = round(best[0], 2)
            self.hottest = {"zone": best[1], "temp_c": round(best[2], 1), "limit_c": best[3]}

    def _event(self, action, level, why):
        what = self.steps[level - 1]["what"] if action == "shed" else self.steps[level]["what"]
        self.events.append({"ts_real_ns": int(self.wall() * 1e9), "action": action, "what": what, "level": self.level,
                            "headroom_k": self.headroom_k, "limited": self.limited, "why": why})
        del self.events[:-32]
        log.info("governor: %s %s -> level %d (headroom %s K, limiter %s, %s)", action, what, self.level,
                 self.headroom_k, "active" if self.limited else "idle", why)

    def _set_level(self, level, now, why):
        level = max(0, min(self.max_level, level))
        while self.level < level:
            self._t_shed = now
            self.level += 1
            self.shed_count += 1
            last = self._restored_at.get(self.level)
            if last is not None and now - last < self.cfg["regret_s"]:      # given back too early: hold longer
                self._hold[self.level] = min(self.cfg["restore_hold_max_s"],
                                             self._hold.get(self.level, self.cfg["restore_hold_s"])
                                             * self.cfg["restore_backoff"])
            self._event("shed", self.level, why)
        while self.level > level:
            self._restored_at[self.level] = now
            self.level -= 1
            self.restore_count += 1
            self._event("restore", self.level, why)
        self._t_step = now
        self._level_since = now
        self._above_since = None

    def force(self, level):
        """Hold a level by hand (None = back to automatic)."""
        self.forced = None if level is None else max(0, min(len(self.steps), int(level)))
        if self.forced is not None and self.forced != self.level:
            mx, self.max_level = self.max_level, len(self.steps)
            self._set_level(self.forced, self.clock(), "forced")
            self.max_level = mx

    def step(self, reading):
        """One control step on a probe reading; returns state()."""
        now = self.clock()
        self._observe(reading, now)
        if not self.cfg["enabled"] or self.forced is not None:
            return self.state()
        h = self.headroom_k
        if h is None:                      # no usable sensor: do nothing, say so (state()["headroom_k"] is None)
            return self.state()
        c = self.cfg
        urgent = self.limited or h < c["urgent_k"]
        if (h < c["shed_at_k"] or self.limited) and self.level < self.max_level:
            dwell = c["urgent_dwell_s"] if urgent else c["shed_dwell_s"]
            if now - self._t_shed >= dwell:
                self._set_level(self.level + 1, now, "limiter active" if self.limited else "headroom %.1f K" % h)
            self._above_since = None
        elif h > c["restore_at_k"] and not self.limited and self.level > 0:
            if self._above_since is None:
                self._above_since = now
            hold = self._hold.get(self.level, c["restore_hold_s"])
            if now - self._above_since >= hold and now - self._t_step >= hold:
                self._set_level(self.level - 1, now, "headroom %.1f K for %.0f s" % (h, hold))
        else:
            self._above_since = None
        return self.state()

    # -- outputs
    def caps(self):
        caps = {k: -1 for k in CAP_KEYS}
        for s in self.steps[:self.level]:
            for k, v in s["caps"].items():
                caps[k] = v
        return caps

    def shed(self):
        return [s["what"] for s in self.steps[:self.level]]

    def state(self):
        return {"enabled": bool(self.cfg["enabled"]), "level": self.level, "levels": len(self.steps),
                "shed": self.shed(), "caps": self.caps(), "headroom_k": self.headroom_k, "hottest": self.hottest,
                "limited": self.limited, "limiter": self.limiter, "forced": self.forced,
                "level_age_s": round(self.clock() - self._level_since, 1),
                "shed_count": self.shed_count, "restore_count": self.restore_count, "events": list(self.events)}

    def file_line(self):
        d = {"v": 1, "ts_mono_ns": time.monotonic_ns(), "valid_s": self.cfg["valid_s"], "level": self.level,
             "limited": int(self.limited), "headroom_k": -1000 if self.headroom_k is None else self.headroom_k}
        d.update(self.caps())
        return json.dumps(d, separators=(",", ":"))

    def topic_status(self, topic, hz, latency_ms):
        """The honest marker a measurement topic carries: measured rate and latency, and whether a consumer is
        getting less than the configured floor, with the reasons: `shed` (the governor has capped the stage),
        `limited` (the platform limiter is cutting clocks), `slow` (rate under the floor), `late` (latency over
        the floor)."""
        why = []
        caps = self.caps()
        if any(caps.get(k, -1) >= 0 for k in TOPIC_CAPS.get(topic, ())):
            why.append("shed")
        if self.limited:
            why.append("limited")
        fl = self.cfg["floors"].get(topic) or {}
        if hz is not None and fl.get("hz") and hz < fl["hz"]:
            why.append("slow")
        if latency_ms is not None and fl.get("latency_ms") and latency_ms > fl["latency_ms"]:
            why.append("late")
        out = {"hz": None if hz is None else round(hz, 1),
               "latency_ms": None if latency_ms is None else round(latency_ms, 1),
               "degraded": any(w != "limited" for w in why), "why": why,
               "level": self.level}
        return out


def write_caps(path, line):
    tmp = path + ".tmp"
    try:
        with open(tmp, "w") as f:
            f.write(line + "\n")
        os.replace(tmp, path)
        return True
    except OSError:
        return False


def load_config():
    path = os.path.join(os.environ.get("ROADSENSE_CONFIG_DIR", "/etc/roadsense"), "governor.json")
    try:
        with open(path) as f:
            cfg = json.load(f)
        return cfg if isinstance(cfg, dict) else {}
    except (OSError, ValueError):
        return {}
