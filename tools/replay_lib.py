"""A recorder session through the api's derived topics, offline: the one place the replay tools share.

    from replay_lib import Pipeline, read_session
    p = Pipeline()
    for m in read_session(SESSION, map_path=MAP_JSONL):
        for out in p.feed(m):        # the api's outgoing messages for this input, in the api's order
            ...

`Pipeline.feed` mirrors api/roadsense_api/state.py: `ego` and `map` go to the trackers; a `road` message yields
`road`, `lead`, `lanes`, `view` (with `view.road`), and `fused` twice a second of message time; `lights`,
`signs` and the other recorded topics pass through as they were recorded (they went out annotated). Used by
tools/view_replay.py and by consumers' replay tools (the car computer's scene_feed_replay / mfd_chain_replay).
"""
import json
import os
import sys
import zlib

_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(_ROOT, "api"))
sys.path.insert(0, os.path.join(_ROOT, "chris-integrator"))

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


class Pipeline:
    def __init__(self, view_cfg=None, egolane_cfg=None, fused_period_s=0.5, integrator_cfg=None, integrator=True):
        """integrator: an `Integrator` to use (extra sources registered), or True for the stack's default."""
        from chris_integrator import Integrator
        from roadsense_api.egolane import EgoLaneEstimator
        from roadsense_api.fusion import Fusion
        from roadsense_api.lanes import RoadGeometryTracker
        from roadsense_api.roadgeom import RoadGeometryFusion
        from roadsense_api.view import ViewBuilder
        self.lanes, self.geom = RoadGeometryTracker(), RoadGeometryFusion()
        self.egolane, self.view, self.fusion = EgoLaneEstimator(egolane_cfg), ViewBuilder(view_cfg), Fusion()
        self.integrator = integrator if not isinstance(integrator, bool) else Integrator(integrator_cfg)
        integrator = integrator is not False
        self.gate = integrator
        self.latest = {}
        self.intr = (975.0, 640.0, 360.0, 1280, 720)
        self.fused_period_ns = int(fused_period_s * NS)
        self._fused_t = None

    def feed(self, m):
        t, ts = m.get("type"), m.get("ts_mono_ns")
        if t == "calib" and m.get("focal_px"):
            self.intr = (float(m["focal_px"]), float(m.get("cx") or 640), float(m.get("cy") or 360),
                         int(m.get("width") or 1280), int(m.get("height") or 720))
        if t in ("ego_state", "tracks"):  # recorded ones: the pipeline computes its own
            return []
        if t in INTEGRATOR_INPUTS:
            self.integrator.on_message(m)
        if t == "map":
            for x in (self.lanes, self.geom, self.view, self.egolane):
                x.on_map(m)
        elif t == "lights":
            self.view.on_lights(m)
        elif t == "signs":
            self.view.on_signs(m, self.intr)
        if t == "display_lanes":          # an external display lane source (laned): the view's lines only
            self.view.on_display_lanes(m)
            return []
        if t in PASS:
            self.latest[t] = m
            return [m]
        if t != "road" or not isinstance(ts, int):
            return []
        self.latest["road"] = m
        es, tr, ld = self.integrator.frame(m)
        self.latest["ego_state"] = es
        out = [m, es]
        if tr is not None:
            self.latest["tracks"] = tr
            out.append(tr)
        if ld is not None:
            self.latest["lead"] = ld
            out.append(ld)
        ln = self.lanes.on_road(m, es)
        geom = None
        if ln is not None:
            self.latest["lanes"] = ln
            out.append(ln)
            self.latest["egolane"] = self.egolane.on_lanes(ln, m, ego=es)
            geom = self.geom.on_lanes(ln, m, es)
        vw = self.view.on_frame(m, self.latest.get("lead"), ln, geom, self.latest.get("egolane"), ego=es)
        if vw is not None:
            self.latest["view"] = vw
            out.append(vw)
        if self._fused_t is None or ts - self._fused_t >= self.fused_period_ns:
            self._fused_t = ts
            fu = self.fusion.update(self.latest, ts, m.get("ts_real_ns"))
            self.latest["fused"] = fu
            out.append(fu)
        return out
