"""A recorder session through the consolidated side, offline: the one place the replay tools share.

    from chris_integrator.replay import Pipeline, read_session
    p = Pipeline()
    for m in read_session(SESSION, map_path=MAP_JSONL):
        for out in p.feed(m):        # the api's outgoing messages for this input, in the api's order
            ...

`Pipeline` is a mixer of chris-integrator (chris-integrator/chris_integrator/mixer) with the session as its
input channel: `ego` and `map` go to the boxes; a `road` message yields `road`, `ego_state`, `tracks`, `lead`,
`lanes`, `view` (with `view.road`), and `fused` twice a second of frame time; `lights`, `signs` and the other
recorded topics pass through as they were recorded (they went out annotated). Used by
tools/view_replay.py and by consumers' replay tools (the car computer's scene_feed_replay / mfd_chain_replay).
"""
import json
import os
import zlib


NS = 1e9
PASS = ("lights", "signs", "env", "speed_limit", "rules", "map", "ego", "vehicle", "camera", "calib", "gnss", "objects")
INTEGRATOR_INPUTS = ("ego", "vehicle", "gnss", "objects", "map", "calib")


def read_lines(path):
    """JSON objects of a plain or gzip file; a truncated gzip stream (a recording cut by a power loss) yields
    what is complete."""
    if os.path.isdir(path):
        path = os.path.join(path, "messages.jsonl.gz")
    with open(path, "rb") as f:
        d, buf = (zlib.decompressobj(31), b"") if path.endswith(".gz") else (None, b"")
        while True:
            chunk = f.read(1 << 20)
            if not chunk:
                break
            if d is not None:
                try:
                    chunk = d.decompress(chunk)
                except zlib.error:
                    break
            buf += chunk
            *full, buf = buf.split(b"\n")
            for ln in full:
                if ln.startswith(b"{"):
                    try:
                        yield json.loads(ln)
                    except ValueError:
                        continue
        if buf.startswith(b"{"):
            try:
                yield json.loads(buf)
            except ValueError:
                pass


def load_pos(path):
    """Feeder `gnss` messages from the car computer's trip logs (lines with `"dir":"POS"`, each prefixed by its
    trip file name; extraction: docs/lanes/L35-ego-fusion-audit.md section 6), each with `_wall` (s): the trip
    file's start (its name, local time Europe/Warsaw) plus the line's `ts` (ms since the trip start)."""
    import re
    import time as _time
    os.environ.setdefault("TZ", "Europe/Warsaw")
    _time.tzset()
    out = []
    with open(path) as f:
        for ln in f:
            i = ln.find("{")
            m = re.search(r"trip_(\d{8})_(\d{6})", ln[:i]) if i >= 0 else None
            if not m:
                continue
            try:
                rec = json.loads(ln[i:])
            except ValueError:
                continue
            d = rec.get("d") or {}
            if d.get("type") != "gnss" or not isinstance(d.get("ts_mono_ns"), int):
                continue
            start = _time.mktime(_time.strptime(m.group(1) + m.group(2), "%Y%m%d%H%M%S"))
            d["_wall"] = start + (rec.get("ts") or 0) / 1000.0
            out.append(d)
    out.sort(key=lambda d: d["ts_mono_ns"])
    return out


def read_session(session, map_path=None, road_model=None, pos=None):
    """Messages of a session in time order.

    map_path    (the `--out` of tools/map_replay.py) the session's own `map` messages are replaced by those of the
                replay.
    road_model  for sessions recorded before lane L37 (the recorder dropped `road.road_model`): the model name to
                put back into every `road` message that has none ("vision" for every recording of the car so far).
                Without it the trackers run their supercombo profile on such a session.
    pos         load_pos() output: the car's GNSS fixes of sessions recorded before `gnss` was; the fixes of this
                session's boot (matched by wall time: the monotonic clock restarts at every boot) are merged in at
                their arrival times.
    """
    src = _with_map(session, map_path) if map_path else read_lines(session)
    if pos is not None:
        src = _with_gnss(session, src, pos)
    for m in src:
        if road_model and m.get("type") == "road" and "road_model" not in m:
            m["road_model"] = road_model
        yield m


def _with_gnss(session, src, pos):
    if not pos:
        yield from src
        return
    roads = [(m["ts_mono_ns"], m["ts_real_ns"]) for m in read_lines(session)
             if m.get("type") == "road" and isinstance(m.get("ts_real_ns"), int) and isinstance(m.get("ts_mono_ns"), int)]
    if not roads:
        yield from src
        return
    offs = sorted((r - t) / NS for t, r in roads[-200:])              # wall - mono of this boot, after any clock step
    off = offs[len(offs) // 2]
    t0, t1 = roads[0][0], roads[-1][0]
    g = [dict(d) for d in pos if t0 - 5 * NS <= d["ts_mono_ns"] <= t1 + 5 * NS
         and abs(d["_wall"] - (d["ts_mono_ns"] / NS + off)) < 5.0]
    k = 0
    last = None
    for m in src:
        ts = m.get("ts_mono_ns")
        if m.get("type") == "gnss":                                   # the session has its own: nothing to merge
            g = []
        if isinstance(ts, int):
            last = ts if last is None else max(last, ts)
            # a feeder message reached mapd at about its stamp + 20 ms; it is merged in where the stream has passed it
            while k < len(g) and g[k]["ts_mono_ns"] + 20_000_000 <= last:
                d = g[k]
                d.pop("_wall", None)
                yield d
                k += 1
        yield m


def _with_map(session, map_path):
    maps = [m for m in read_lines(map_path) if m.get("type") == "map" and isinstance(m.get("ts_mono_ns"), int)]
    maps.sort(key=lambda m: m["ts_mono_ns"])
    k = 0
    for m in read_lines(session):
        if m.get("type") in ("ego_state", "tracks"):      # recomputed by the pipeline
            continue
        if m.get("type") == "map":
            continue
        ts = m.get("ts_mono_ns")
        while isinstance(ts, int) and k < len(maps) and maps[k]["ts_mono_ns"] <= ts:
            yield maps[k]
            k += 1
        yield m
    yield from maps[k:]


RECORDED = (          # a recorder session as one input channel: (message type, stream, kind, time, public, doc)
    ("road", "road", "raw", "frame", True, ""), ("calib", "calib", "raw", "event", True, ""),
    ("camera", "camera", "raw", "sample", True, ""), ("env", "env", "raw", "sample", True, ""),
    ("lights", "lights", "decision", "sample", True, "as recorded: already with the ego light"),
    ("signs", "signs", "measurement", "sample", True, "as recorded: already with the context flags"),
    ("speed_limit", "speed_limit", "decision", "event", True, "as recorded"),
    ("rules", "rules", "decision", "event", True, "as recorded"),
    ("map", "map", "context", "periodic", True, "as recorded, or of a map replay"),
    ("ego", "ego", "raw", "sample", True, ""), ("vehicle", "vehicle", "raw", "sample", True, ""),
    ("gnss", "gnss", "raw", "sample", True, ""), ("objects", "objects", "raw", "sample", True, ""),
    ("display_lanes", "display_lanes", "raw", "sample", False, ""),
)


class Pipeline:
    """The consolidated side offline, as a mixer of chris-integrator: the session is one input channel (its
    `lights`, `signs`, `speed_limit`, `rules` and `map` are taken as recorded), the plug-ins are the estimator,
    the lane tracker, ego lane, road geometry, the view builder and `fused` (twice a second of frame time)."""

    def __init__(self, view_cfg=None, egolane_cfg=None, fused_period_s=0.5, integrator_cfg=None, integrator=True):
        """integrator: an `Integrator` to use (extra sources registered), or True for the stack's default."""
        from .mixer import In, MessageClock, Mixer
        from .plugins.estimator import Estimator
        from .plugins.trackers import EgoLane, Fused, Lanes, RoadGeom, View
        from .stack import Stack

        class FusedPerFrame(Fused):          # on road frames only, stamped with the frame's own time
            inputs = tuple(In(i.name, trigger=i.name == "road", optional=True) for i in Fused.inputs)

            def on_input(self, stream, msg, ctx):
                ts = msg.get("ts_mono_ns")
                if isinstance(ts, int) and (self._emit is None or ts - self._emit >= self.cfg["period_s"] * NS):
                    self._emit = ts
                    from .plugins.common import Latest
                    ctx.emit("fused", self.fusion.update(Latest(ctx, self.READS, {"egolane": "est"}), ts, msg.get("ts_real_ns")))

        self.mixer = m = Mixer(MessageClock())
        self.stack = st = Stack(m)
        st.add_channel("recorded", RECORDED, info="a recorder session")
        # outputs per road frame only; no source besides the stack's own unless the configuration names one
        self._est = Estimator(dict({"sources": []}, **(integrator_cfg or {})), feedback=(), tick=(), stream_of={})
        if not isinstance(integrator, bool):
            self._est.integ = integrator
        self._lanes, self._egolane, self._geom = Lanes(), EgoLane(egolane_cfg), RoadGeom()
        self._view, self._fused = View(view_cfg), FusedPerFrame({"period_s": fused_period_s})
        for b in (self._est, self._lanes, self._egolane, self._geom, self._view, self._fused):
            m.add(b)
        self._out = []
        m.add_output("replay", lambda e: self._out.append(e.msg))
        self.gate = integrator is not False

    # the boxes' logic objects, as the tools know them
    integrator = property(lambda self: self._est.integ)
    view = property(lambda self: self._view.builder)
    egolane = property(lambda self: self._egolane.est)
    geom = property(lambda self: self._geom.fusion)
    fusion = property(lambda self: self._fused.fusion)

    @property
    def lanes(self):
        return self._lanes.tracker

    @lanes.setter
    def lanes(self, tracker):
        self._lanes.tracker = tracker

    @property
    def latest(self):
        """The newest message per topic (`egolane`: the ego-lane estimate itself)."""
        out = {n: e.msg for n, e in self.mixer.bus.latest.items()}
        for k, key in (("egolane", "est"), ("roadgeom", "geom")):
            if k in out:
                out[k] = out[k].get(key)
        return out

    def feed(self, m):
        """One recorded message in; the outgoing messages it causes, in the stack's order."""
        if m.get("type") in ("ego_state", "tracks"):   # recorded ones: the pipeline computes its own
            return []
        if m.get("type") == "road" and not isinstance(m.get("ts_mono_ns"), int):
            return []
        self._out = []
        self.stack.deliver(m)
        return self._out
