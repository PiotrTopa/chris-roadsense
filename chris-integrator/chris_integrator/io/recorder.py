"""Drive recorder: what the stack saw and said, kept on disk for the replay tools.

Off unless `<config dir>/recorder.json` exists (or `make_app(record=...)`): `{"dir": "/var/lib/chris/rec"}`
switches it on. Every start of the integrator (the host starts the stack when the car is switched on) opens a session
directory `<dir>/<YYYYmmdd-HHMMSS>/` with

    messages.jsonl.gz   the topics as they went out on /ws, one JSON object per line, in order: `road` reduced to
                        what the trackers and the replay tools read (lane lines, road edges, leads[0], plan x / y,
                        speed, pose) with the numbers shortened, at most `road_hz` per second; `lights`, `signs`,
                        `env`, `speed_limit`, `rules`, `fused`, `map`, `ego` (at most `ego_hz`), `vehicle` (what
                        the host says about the car), `objects` (range-sensor scans, at most `objects_hz`),
                        `view` (the display channel as consumers got it, with the road picture `view.road`, at most `view_hz`),
                        `calib` and one `camera` message a second, as they are. `lanes` and `lead` are not written: they are
                        derived from `road` and `ego`, and `tools/lanes_replay.py` / `tools/lead_replay.py`
                        recompute them from this file.
    frames/<seq>.jpg    the camera's preview JPEG every `frame_every_s` seconds (the file camerad already
                        writes for its /preview; copied, not re-encoded; only when the recorder is given that path): the pictures a detector replay
                        (roadsense-signs `lights_replay`, `signd_replay`) needs to say what was there to see.

The file is flushed and synced to the disk every 5 s (a host that loses power keeps all but the last
seconds). A session opened while the wall clock was wrong (a boot without a clock source) is renamed to its
true start time when the clock is stepped.

Cost: about 30 kB/s of compressed messages and one 40-110 kB JPEG a second, a few milliseconds of one little
core per second. Old sessions are deleted, oldest first, when the directory exceeds `max_mb`. Nothing leaves
the device. The recorder never raises into the fan-out: a full disk stops the recording, not the api.

Config keys (all optional but `dir`): `max_mb` (4000), `road_hz` (20), `ego_hz` (25), `ego_state_hz` (2), `objects_hz` (10), `view_hz` (2),
`frame_every_s` (1.0; 0 = no frames), `topics` (list; default below).
"""
import gzip
import json
import logging
import os
import shutil
import time

from ..native import round                  # builtins.round, native where built
from .. import native
from ..fastround import Layout, warm

log = logging.getLogger("chris_integrator.recorder")

DEFAULT_TOPICS = ("road", "lights", "signs", "env", "speed_limit", "rules", "fused", "map", "ego", "calib", "camera",
                  "vehicle", "objects", "view", "gnss", "ego_state", "tracks")
DEFAULTS = dict(dir=None, max_mb=4000, road_hz=20.0, ego_hz=25.0, objects_hz=10.0, view_hz=2.0, frame_every_s=1.0,
                ego_state_hz=2.0, topics=list(DEFAULT_TOPICS))
CLOCK_STEP_S = 120.0            # the wall clock moved this much against the monotonic one: re-date the session


def load_config(config_dir=None):
    """`<config dir>/recorder.json` as a dict with a `dir`, or None (no file, no dir: the recorder stays off)."""
    path = os.path.join(config_dir or os.environ.get("CHRIS_CONFIG_DIR", "/etc/chris"), "recorder.json")
    try:
        with open(path) as f:
            cfg = json.load(f)
    except (OSError, ValueError):
        return None
    if not isinstance(cfg, dict) or not cfg.get("dir"):
        return None
    return {k: v for k, v in cfg.items() if k in DEFAULTS}


_LAYOUT = Layout()
_MARK = "\x01"                  # stands for a list of numbers in the skeleton that json.dumps writes
_MARK_TEXT = json.dumps(_MARK)


def _slim(road, rounded=None):
    """The reduced `road` with every list of numbers replaced by a marker: (dict, [(list, digits), ...] in the
    order json.dumps meets the markers, [(container, key), ...]). With `rounded` (native.Rounded) every list
    is wrapped as `rounded((list, digits))` instead, for the native writer, and the three lists stay empty."""
    # every scalar a tracker or a replay tool reads: `road_model` selects the trackers' profile (it was dropped
    # until lane L37, so replays ran the supercombo profile on vision-model sessions), `camera` names the camera
    out = {k: road.get(k) for k in ("type", "seq", "ts_mono_ns", "ts_real_ns", "model_ms", "v_ego", "valid",
                                    "frames_skipped", "pair_dt_ms", "road_model", "road_model_fallback",
                                    "policy_hz", "frame_seq_prev", "camera") if k in road}
    pose = road.get("pose")
    if isinstance(pose, dict):
        out["pose"] = {k: round(v, 4) if isinstance(v, float) else v for k, v in pose.items()}
    parts, digits, slots = [], [], []

    if rounded is None:
        def put(d, k, v, nd):
            if isinstance(v, list):
                d[k] = _MARK
                parts.append(v)
                digits.append(nd)
                slots.append((d, k))
            else:
                d[k] = v
    else:
        def put(d, k, v, nd):
            d[k] = rounded((v, nd)) if isinstance(v, list) else v
    for key in ("lane_lines", "road_edges"):
        lines = road.get(key)
        if isinstance(lines, list):
            res = []
            for ln in lines:
                if isinstance(ln, dict):
                    o = {}
                    put(o, "x", ln.get("x"), 2)
                    put(o, "y", ln.get("y"), 3)
                    put(o, "y_std", ln.get("y_std"), 3)
                    if isinstance(ln.get("prob"), (int, float)):
                        o["prob"] = round(ln["prob"], 4)
                    res.append(o)
            out[key] = res
    leads = road.get("leads")
    if isinstance(leads, list) and leads and isinstance(leads[0], dict):
        o = {}
        for k, v in leads[0].items():
            put(o, k, v, 3)
        out["leads"] = [o]
    plan = road.get("plan")
    if isinstance(plan, dict):
        o = out["plan"] = {}
        for k in ("x", "y", "z"):
            if k in plan:
                put(o, k, plan.get(k), 2)
        for k in ("held", "age_ms"):
            if k in plan:
                o[k] = plan[k]
    return out, parts, digits, slots


def slim_road(road):
    """The part of a `road` message the trackers and the replay tools read, numbers shortened."""
    out, parts, digits, slots = _slim(road)
    if parts:                   # about 700 numbers a frame, rounded in one go: fastround gives round()'s results
        for (d, k), v in zip(slots, _LAYOUT.round_parts(parts, digits)):
            d[k] = v
    return out


def slim_road_line(road):
    """json.dumps(slim_road(road), separators=(",", ":")), made without writing each float: the skeleton goes
    through json.dumps with a marker for every list, the lists' texts come from fastround's tables. With the
    native core the lists are rounded and written by its `dumps` in one pass (`native.Rounded`)."""
    if native.ROUNDED:
        try:
            return native._nd(_slim(road, native.Rounded)[0])
        except native.Fallback:            # something the native writer leaves to json: the pure path below
            pass
    out, parts, digits, slots = _slim(road)
    if parts:
        pieces = json.dumps(out, separators=(",", ":")).split(_MARK_TEXT)
        if len(pieces) == len(parts) + 1:
            texts = _LAYOUT.text_parts(parts, digits)
            res = [pieces[0]]
            for t, p in zip(texts, pieces[1:]):
                res.append(t)
                res.append(p)
            return "".join(res)
        for (d, k), v in zip(slots, _LAYOUT.round_parts(parts, digits)):      # a string that looks like the marker
            d[k] = v
    return json.dumps(out, separators=(",", ":"))


class Recorder:
    def __init__(self, cfg, preview_path=None, clock=time.monotonic, wall=time.time):
        self.cfg = dict(DEFAULTS)
        self.cfg.update(cfg or {})
        self.preview_path = preview_path
        self.clock, self.wall = clock, wall
        self.topics = set(self.cfg["topics"])
        if "road" in self.topics and not native.ROUNDED:
            warm((2, 3))            # the number texts of the reduced `road`: built at start-up, not in a frame
                                    # (the native core writes rounded lists itself and needs no tables)
        c = self.cfg
        self._caps = {"road": c["road_hz"], "ego": c["ego_hz"], "objects": c["objects_hz"], "view": c["view_hz"],
                      "camera": 1.0, "ego_state": c["ego_state_hz"], "tracks": c["ego_state_hz"]}
        self.dir = None
        self.fh = None
        self.failed = False
        self.frames_enabled = True      # the load governor switches the pictures off under heat (governor.py)
        self.written = {"messages": 0, "frames": 0, "bytes_in": 0}
        self._last = {}             # topic -> clock of the last written message (rate caps)
        self._flush_t = 0.0
        self._frame_t = -1e9
        self._prune_t = 0.0

    # ---- session ---------------------------------------------------------
    def _open(self):
        base = self.cfg["dir"]
        name = time.strftime("%Y%m%d-%H%M%S", time.localtime(self.wall()))
        self.dir = os.path.join(base, name)
        os.makedirs(os.path.join(self.dir, "frames"), exist_ok=True)
        self.fh = gzip.open(os.path.join(self.dir, "messages.jsonl.gz"), "at", compresslevel=3)
        self._opened = (self.clock(), self.wall())
        self.prune()
        log.info("recording to %s (cap %s MB)", self.dir, self.cfg["max_mb"])

    def close(self):
        if self.fh is not None:
            try:
                self.fh.close()
            except OSError:
                pass
            self.fh = None

    def _fail(self, what, exc):
        if not self.failed:
            log.warning("recorder stopped (%s): %s", what, exc)
        self.failed = True
        self.close()

    # ---- fan-out hook ----------------------------------------------------
    def on_envelope(self, env):
        """The recorder as an output channel of the mixer: the message is serialised only when it is written
        (after the rate cap), and `road` is reduced from the message the bus already holds."""
        mtype = env.stream
        if self.failed or mtype not in self.topics:
            return
        now = self.clock()
        if not self._due(mtype, now):
            return
        if mtype == "road" and isinstance(env.msg, dict):
            self._write(mtype, slim_road_line(env.msg), now)
        else:
            self._write(mtype, env.line, now)

    def on_message(self, mtype, line):
        """Called with every outgoing message (topic, serialised line)."""
        if self.failed or mtype not in self.topics:
            return
        now = self.clock()
        if not self._due(mtype, now):
            return
        if mtype == "road":
            line = json.dumps(slim_road(json.loads(line)), separators=(",", ":"))
        self._write(mtype, line, now)

    def _due(self, mtype, now):
        """The rate cap of a topic: False when its last written message is too recent."""
        cap = self._caps.get(mtype)
        if cap and now - self._last.get(mtype, -1e9) < 1.0 / cap - 0.005:
            return False
        self._last[mtype] = now
        return True

    def _write(self, mtype, line, now):
        try:
            if self.fh is None:
                self._open()
            self.fh.write(line + "\n")
            self.written["messages"] += 1
            self.written["bytes_in"] += len(line) + 1
            if now - self._flush_t > 5.0:
                self._flush_t = now
                self.fh.flush()
                try:
                    os.fsync(self.fh.fileno())
                except (OSError, ValueError, AttributeError):
                    pass
                self._redate(now)
            if mtype == "camera":
                self._frame(now, line)
            if now - self._prune_t > 300.0:
                self.prune()
        except (OSError, ValueError) as e:
            self._fail("write", e)

    def _redate(self, now):
        """The wall clock was stepped since the session was opened (it started on a stale clock): name the
        directory after the true start time. The open file follows the rename."""
        c0, w0 = self._opened
        start = self.wall() - (now - c0)
        if abs(start - w0) < CLOCK_STEP_S:
            return
        base = self.cfg["dir"]
        name = time.strftime("%Y%m%d-%H%M%S", time.localtime(start))
        new = os.path.join(base, name)
        n = 1
        while os.path.exists(new):
            new = os.path.join(base, "%s-%d" % (name, n))
            n += 1
        try:
            os.rename(self.dir, new)
        except OSError as e:
            log.warning("recorder: session not re-dated: %s", e)
            self._opened = (c0, start)
            return
        log.info("recorder: clock stepped by %.0f s, session %s is now %s", start - w0, os.path.basename(self.dir), new)
        self.dir = new
        self._opened = (c0, start)

    def _frame(self, now, camera_line):
        every = self.cfg["frame_every_s"]
        if not every or not self.frames_enabled or not self.preview_path or now - self._frame_t < every - 0.05:
            return
        self._frame_t = now
        try:
            seq = json.loads(camera_line).get("seq")
        except ValueError:
            seq = None
        dst = os.path.join(self.dir, "frames", "%s.jpg" % (seq if isinstance(seq, int) else int(now * 1000)))
        try:
            shutil.copyfile(self.preview_path, dst)
            self.written["frames"] += 1
        except FileNotFoundError:
            pass                    # no preview yet (camerad --no-preview, or still starting)

    # ---- retention -------------------------------------------------------
    def prune(self):
        """Delete the oldest sessions while the recordings exceed `max_mb` (never the current one)."""
        self._prune_t = self.clock()
        base = self.cfg["dir"]
        try:
            sessions = sorted(d for d in os.listdir(base) if os.path.isdir(os.path.join(base, d)))
        except OSError:
            return
        sizes = {}
        for d in sessions:
            total = 0
            for root, _, files in os.walk(os.path.join(base, d)):
                for fn in files:
                    try:
                        total += os.path.getsize(os.path.join(root, fn))
                    except OSError:
                        pass
            sizes[d] = total
        limit = float(self.cfg["max_mb"]) * 1e6
        cur = os.path.basename(self.dir) if self.dir else None
        for d in sessions:
            if sum(sizes.values()) <= limit:
                break
            if d == cur:
                continue
            shutil.rmtree(os.path.join(base, d), ignore_errors=True)
            log.info("recorder: removed old session %s (%.0f MB)", d, sizes[d] / 1e6)
            sizes[d] = 0

    def status(self):
        return {"dir": self.dir, "failed": self.failed, "frames_enabled": self.frames_enabled, **self.written}
