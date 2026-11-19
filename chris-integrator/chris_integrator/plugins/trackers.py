"""Plug-in faces of the trackers and decision logic (`chris_integrator.trackers`): each takes named streams
and puts named streams back; the logic itself is in the tracker classes and knows nothing of the bus."""
from .. import contract
from ..mixer import In, Out, Plugin
from ..trackers import lights as lights_mod
from ..trackers import signctx
from ..trackers.egolane import EgoLaneEstimator
from ..trackers.fusion import Fusion
from ..trackers.lanes import RoadGeometryTracker
from ..trackers.roadgeom import RoadGeometryFusion
from ..trackers.rules import RulesInEffect
from ..trackers.speedlimit import load_rules
from ..trackers.view import ViewBuilder
from .common import Latest, intrinsics


def _rate(ctx, topic, msg):
    r = ctx.service("rates")
    return None if r is None else r.status(topic, msg)


class Lanes(Plugin):
    """The lane-line tracker: the road model's lines over time with the car's motion, the map's lane count as a
    soft prior. Measurement topic `lanes`."""
    name = "lanes"
    inputs = (In("road", trigger=True), In("ego_state", optional=True), In("map", trigger=True, optional=True),
              In("governor", optional=True))
    outputs = (Out("lanes", "measurement", "frame", doc="tracked lane lines and road edges"),)

    def __init__(self, cfg=None, name=None):
        super().__init__(None, name)
        self.tracker = RoadGeometryTracker(cfg)

    def on_input(self, stream, msg, ctx):
        if stream == "map":
            self.tracker.on_map(msg)
            return
        out = self.tracker.on_road(msg, ctx.get("ego_state", frame=True))
        if out is not None:
            rate = _rate(ctx, "lanes", msg)
            if rate is not None:
                out["rate"] = rate
            ctx.emit("lanes", out)

    def reset(self, reason=None):
        self.tracker.reset()


class EgoLane(Plugin):
    """Which lane the car is in, over time (lane changes on the tracked lines, the map's lane count)."""
    name = "egolane"
    inputs = (In("lanes", trigger=True), In("road", optional=True), In("ego_state", optional=True),
              In("map", trigger=True, optional=True))
    outputs = (Out("egolane", "estimate", "frame", public=False, doc="ego lane estimate (wiring; served inside `fused.lanes`)"),)

    def __init__(self, cfg=None, name=None):
        super().__init__(None, name)
        self.est = EgoLaneEstimator(cfg)

    def on_input(self, stream, msg, ctx):
        if stream == "map":
            self.est.on_map(msg)
            return
        out = self.est.on_lanes(msg, ctx.get("road", frame=True), ego=ctx.get("ego_state", frame=True))
        ctx.emit("egolane", {"type": "egolane", "ts_mono_ns": msg.get("ts_mono_ns"), "est": out})

    def reset(self, reason=None):
        self.est.reset()


class RoadGeom(Plugin):
    """The fused road geometry: tracked lanes + the map's shape and junctions (drawn as `view.road`)."""
    name = "roadgeom"
    inputs = (In("lanes", trigger=True), In("road", optional=True), In("ego_state", optional=True),
              In("map", trigger=True, optional=True))
    outputs = (Out("roadgeom", "estimate", "frame", public=False, doc="fused road geometry (wiring; served as `view.road`)"),)

    def __init__(self, cfg=None, name=None):
        super().__init__(None, name)
        self.fusion = RoadGeometryFusion(cfg)

    def on_input(self, stream, msg, ctx):
        if stream == "map":
            self.fusion.on_map(msg)
            return
        geom = self.fusion.on_lanes(msg, ctx.get("road", frame=True), ctx.get("ego_state", frame=True))
        ctx.emit("roadgeom", {"type": "roadgeom", "ts_mono_ns": msg.get("ts_mono_ns"), "geom": geom})

    def reset(self, reason=None):
        self.fusion.reset()


class EgoLight(Plugin):
    """The ego traffic light: a camera's light detections placed against the planned path, the tracked lanes
    and the map's signals. In: the detector's message; out: the same message with the ego light selected."""
    name = "ego_light"
    DEFAULT_CFG = {"lane_assign": True}
    inputs = (In("lights_raw", trigger=True), In("road", optional=True), In("calib", optional=True),
              In("ego_state", optional=True), In("lanes", trigger=True, optional=True),
              In("map", trigger=True, optional=True), In("governor", optional=True))
    outputs = (Out("lights", "decision", "sample", doc="traffic lights with the ego light selected"),)

    def __init__(self, cfg=None, name=None):
        super().__init__(cfg, name)
        self.sel = lights_mod.EgoLightSelector()
        self.sel.lane_assign = bool(self.cfg.get("lane_assign", True))

    def on_input(self, stream, msg, ctx):
        if stream == "lanes":
            self.sel.on_lanes(msg)
        elif stream == "map":
            self.sel.on_map(msg)
        else:
            # the plan positions a light against the path: not while the camera is not believable
            road = ctx.get("road") if contract.usable(ctx.get("ego_state"), "vision") else None
            calib = ctx.get("calib")
            self.sel.annotate(msg, road, calib, intrinsics(calib))
            rate = _rate(ctx, "lights", msg)
            if rate is not None:
                msg["rate"] = rate
            ctx.emit("lights", msg)

    def reset(self, reason=None):
        self.sel.reset()


class SignContext(Plugin):
    """Context checks on a camera's sign tracks (a round red "sign" that is the lead's tail light or a traffic
    light), and the list of live sign tracks. In: the detector's message; out: the same message annotated."""
    name = "sign_context"
    LIVE_TRACK_TIMEOUT_S = 2.0
    inputs = (In("signs_raw", trigger=True), In("lead", optional=True), In("lights", optional=True),
              In("calib", optional=True), In("governor", optional=True))
    outputs = (Out("signs", "measurement", "sample", doc="sign detections with context flags"),)

    def __init__(self, cfg=None, name=None, history=None):
        super().__init__(cfg, name)
        self.ctxcheck = signctx.SignContext()
        self.live, self.suppressed = {}, {}
        self.history = history             # callable(ended track) or None

    def on_input(self, stream, msg, ctx):
        calib = ctx.get("calib")
        # the lead that may explain a red round "sign" is the tracked, validated one
        self.ctxcheck.annotate(msg, ctx.get("lead"), ctx.get("lights"), calib, intrinsics(calib))
        rate = _rate(ctx, "signs", msg)
        if rate is not None:
            msg["rate"] = rate
        self._tracks(msg, ctx.now)
        ctx.emit("signs", msg)

    def _tracks(self, msg, now):
        for d in msg.get("detections") or []:
            tid = d.get("track_id")
            if tid is None:
                continue
            # a flagged track (provider or context check) is listed apart with its reason, never as a live sign
            src, other = (self.suppressed, self.live) if d.get("suppressed") else (self.live, self.suppressed)
            t = other.pop(tid, None) or src.get(tid)
            if t is None:
                t = {"track_id": tid, "first_seq": msg.get("seq"), "best_score": 0.0}
            src[tid] = t
            t.update(cls=d.get("cls"), cls_id=d.get("cls_id"), score=d.get("score"), box=d.get("box"),
                     hits=d.get("hits"), age=d.get("age"), last_seq=msg.get("seq"), _seen=now)
            for k in ("suppressed", "reason", "context", "glow", "verify"):
                if k in d:
                    t[k] = d[k]
                else:
                    t.pop(k, None)
            t["best_score"] = max(t["best_score"], d.get("score") or 0.0)
        for e in msg.get("tracks_ended") or []:
            self.live.pop(e.get("track_id"), None)
            self.suppressed.pop(e.get("track_id"), None)
            if self.history is not None:
                self.history(e)
        for tracks in (self.live, self.suppressed):
            for tid in [k for k, t in tracks.items() if now - t["_seen"] > self.LIVE_TRACK_TIMEOUT_S]:
                del tracks[tid]

    def tracks(self, suppressed=False):
        src = self.suppressed if suppressed else self.live
        return [{k: v for k, v in t.items() if not k.startswith("_")}
                for t in sorted(src.values(), key=lambda t: t["track_id"])]

    def reset(self, reason=None):
        self.live.clear()
        self.suppressed.clear()
        self.ctxcheck = signctx.SignContext()


class Rules(Plugin):
    """Signs in effect and the speed limit: signs + place + the car's odometer -> `rules`, `speed_limit`
    (on change, and once a second)."""
    name = "rules"
    DEFAULT_CFG = {"rules": "pl", "heartbeat_s": 1.0}
    inputs = (In("road", trigger=True), In("ego_state", optional=True), In("signs", trigger=True, optional=True),
              In("env", trigger=True, optional=True), In("lights", trigger=True, optional=True),
              In("calib", optional=True))
    outputs = (Out("speed_limit", "decision", "event", doc="the speed limit in effect from signs and rules"),
               Out("rules", "decision", "event", doc="signs in effect, recent, ahead"))

    def __init__(self, cfg=None, name=None):
        super().__init__(cfg, name)
        self._speed_emit = self._rules_emit = 0.0
        self.emitted = False
        self.reset()

    def reset(self, reason=None):
        self.engine = RulesInEffect(load_rules(self.cfg["rules"]))
        self.speed = self.engine.speed
        self._geom = False

    def on_input(self, stream, msg, ctx):
        if not self._geom:
            calib = ctx.get("calib")
            if calib:
                f, cx, _, w, _ = intrinsics(calib)
                self.speed.cfg.update(focal_px=f, cx=cx, width=w)
                self._geom = True
        es = None
        if stream == "road":
            es = ctx.get("ego_state", frame=True)
            if es is None:                 # the estimator did not answer for this frame: no odometer step
                return
        sv = self.speed.version
        changed = self.engine.on_message(msg, es)      # feeds the speed-limit engine too
        now = ctx.now
        hb = self.cfg["heartbeat_s"]
        if self.speed.version != sv or now - self._speed_emit >= hb:
            self._speed_emit = now
            ctx.emit("speed_limit", self.speed.snapshot())
        if changed or now - self._rules_emit >= hb:
            self._rules_emit = now
            ctx.emit("rules", self.engine.snapshot())
            self.emitted = True


class Fused(Plugin):
    """Map and camera decisions put together: limit, place, lanes, tunnel, lead (`fused`, periodic)."""
    name = "fused"
    DEFAULT_CFG = {"period_s": 0.45, "rules": "pl"}
    READS = ("map", "road", "env", "speed_limit", "ego_state", "lanes", "egolane", "lead")
    inputs = (In("road", trigger=True, optional=True), In("map", trigger=True, optional=True),
              In("env", trigger=True, optional=True), In("signs", trigger=True, optional=True),
              In("speed_limit", optional=True), In("ego_state", optional=True), In("lanes", optional=True),
              In("egolane", optional=True), In("lead", optional=True))
    outputs = (Out("fused", "decision", "periodic", doc="map + camera: limit, place, lanes, tunnel, lead"),)

    def __init__(self, cfg=None, name=None):
        super().__init__(cfg, name)
        self._emit = None
        self.reset()

    def reset(self, reason=None):
        self.fusion = Fusion(load_rules(self.cfg["rules"]).get("defaults_kmh"))

    def on_input(self, stream, msg, ctx):
        now = ctx.now
        if self._emit is not None and now - self._emit < self.cfg["period_s"]:
            return
        self._emit = now
        latest = Latest(ctx, self.READS, {"egolane": "est"})
        ctx.emit("fused", self.fusion.update(latest, ctx.mono_ns, ctx.real_ns))


class View(Plugin):
    """The display channel: one picture per camera frame for a screen, smooth and identity-keeping. It reads
    the measurement topics and never feeds back into them."""
    name = "view"
    inputs = (In("road", trigger=True), In("ego_state", optional=True), In("lead", optional=True),
              In("lanes", optional=True), In("roadgeom", optional=True), In("egolane", optional=True),
              In("tracks", optional=True),
              In("map", trigger=True, optional=True), In("lights", trigger=True, optional=True),
              In("signs", trigger=True, optional=True), In("display_lanes", trigger=True, optional=True),
              In("calib", optional=True), In("governor", optional=True))
    outputs = (Out("view", "display", "frame", doc="the display channel: road picture, objects, lines"),)

    def __init__(self, cfg=None, name=None):
        super().__init__(None, name)
        self.builder = ViewBuilder(cfg)
        self._ts = None                    # frame time of the last `view` built (the governor's view_hz cap)

    def on_input(self, stream, msg, ctx):
        b = self.builder
        if stream == "map":
            b.on_map(msg)
        elif stream == "lights":
            b.on_lights(msg)
        elif stream == "signs":
            b.on_signs(msg, intrinsics(ctx.get("calib")))
        elif stream == "display_lanes":    # an external lane source for the display only
            b.on_display_lanes(msg)
        else:
            cap, ts = ((ctx.get("governor") or {}).get("caps") or {}).get("view_hz", -1), msg.get("ts_mono_ns")
            if not (cap < 0 or self._ts is None or not isinstance(ts, int) or ts < self._ts
                    or (cap > 0 and ts - self._ts >= (1.0 / cap - 0.01) * 1e9)):
                return
            self._ts = ts
            geom, el = ctx.get("roadgeom", frame=True), ctx.get("egolane")
            out = b.on_frame(msg, ctx.get("lead"), ctx.get("lanes", frame=True), geom and geom.get("geom"),
                             el and el.get("est"), ego=ctx.get("ego_state", frame=True))
            if out is not None:
                ctx.emit("view", out)

    def reset(self, reason=None):
        self.builder.reset()
        self._ts = None


class RideWatch(Plugin):
    """A new ride (a replayed playlist) or a camera restart (its sequence goes backwards) is not driving on:
    every plug-in is reset."""
    name = "ride_watch"
    inputs = (In("camera", trigger=True),)

    def __init__(self, cfg=None, name=None):
        super().__init__(cfg, name)
        self._key, self._seq, self.changes = None, 0, 0

    def on_input(self, stream, msg, ctx):
        key = (msg.get("ride"), msg.get("ride_index"))
        seq = msg.get("seq") or 0
        changed = self._key is not None and key != self._key
        restarted = seq < self._seq
        self._key, self._seq = key, seq
        if changed or restarted:
            self.changes += 1
            reset = ctx.service("reset")
            if reset is not None:
                reset("new ride" if changed else "camera restart")

    def health(self):
        return {"state": "ok", "ride_changes": self.changes}
