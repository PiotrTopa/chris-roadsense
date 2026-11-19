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

sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "api"))

NS = 1e9
PASS = ("lights", "signs", "env", "speed_limit", "rules", "map", "ego", "vehicle", "camera", "calib")


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


def read_session(session, map_path=None):
    """Messages of a session in time order. With map_path (the `--out` of tools/map_replay.py) the session's
    own `map` messages are replaced by those of the replay."""
    if not map_path:
        yield from read_lines(session)
        return
    maps = [m for m in read_lines(map_path) if m.get("type") == "map" and isinstance(m.get("ts_mono_ns"), int)]
    maps.sort(key=lambda m: m["ts_mono_ns"])
    k = 0
    for m in read_lines(session):
        if m.get("type") == "map":
            continue
        ts = m.get("ts_mono_ns")
        while isinstance(ts, int) and k < len(maps) and maps[k]["ts_mono_ns"] <= ts:
            yield maps[k]
            k += 1
        yield m
    yield from maps[k:]


class Pipeline:
    def __init__(self, view_cfg=None, egolane_cfg=None, fused_period_s=0.5):
        from roadsense_api.egolane import EgoLaneEstimator
        from roadsense_api.fusion import Fusion
        from roadsense_api.lanes import RoadGeometryTracker
        from roadsense_api.lead import LeadTracker
        from roadsense_api.roadgeom import RoadGeometryFusion
        from roadsense_api.view import ViewBuilder
        self.lead, self.lanes, self.geom = LeadTracker(), RoadGeometryTracker(), RoadGeometryFusion()
        self.egolane, self.view, self.fusion = EgoLaneEstimator(egolane_cfg), ViewBuilder(view_cfg), Fusion()
        self.latest = {}
        self.intr = (975.0, 640.0, 360.0, 1280, 720)
        self.fused_period_ns = int(fused_period_s * NS)
        self._fused_t = None

    def feed(self, m):
        t, ts = m.get("type"), m.get("ts_mono_ns")
        if t == "calib" and m.get("focal_px"):
            self.intr = (float(m["focal_px"]), float(m.get("cx") or 640), float(m.get("cy") or 360),
                         int(m.get("width") or 1280), int(m.get("height") or 720))
        if t == "ego":
            for x in (self.view, self.lead, self.lanes, self.geom, self.egolane):
                x.on_ego(m)
        elif t == "vehicle":
            self.lead.on_vehicle(m)
        elif t == "map":
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
        out = [m]
        ld = self.lead.on_road(m)
        if ld is not None:
            self.latest["lead"] = ld
            out.append(ld)
        ln = self.lanes.on_road(m)
        geom = None
        if ln is not None:
            self.latest["lanes"] = ln
            out.append(ln)
            self.latest["egolane"] = self.egolane.on_lanes(ln, m)
            geom = self.geom.on_lanes(ln, m)
        vw = self.view.on_frame(m, self.latest.get("lead"), ln, geom, self.latest.get("egolane"))
        if vw is not None:
            self.latest["view"] = vw
            out.append(vw)
        if self._fused_t is None or ts - self._fused_t >= self.fused_period_ns:
            self._fused_t = ts
            fu = self.fusion.update(self.latest, ts, m.get("ts_real_ns"))
            self.latest["fused"] = fu
            out.append(fu)
        return out
