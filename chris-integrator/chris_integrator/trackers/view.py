"""Display channel: the api-derived topic `view` (roadsense 2.0, lane L19).

The measurement topics (`road`, `lanes`, `lead`, `lights`, `signs`) say what the stack measured, frame by frame,
with honest uncertainty and no added latency; a consumer that decides something reads those. A consumer that
DRAWS needs something else: objects that keep their identity, move smoothly between two detections (a traffic
light is detected three times a second, the picture is drawn 20 times), come and go with a fade instead of a
pop, and carry a velocity so the drawing can be extrapolated to the consumer's own render time. That is this
topic. It is derived from the measurement topics and never feeds back into them; nothing in it is specific to
one display.

    lead, lanes (per road frame) ──┐
    lights (~3/s, annotated) ──────┼─> ViewBuilder ─> {"type":"view", ...}   one per road frame (10-14/s)
    signs (~5/s) ──────────────────┤
    ego (vehicle motion), road ────┘   (ego motion carries static objects between two detections)

Object model (all in the car's frame of the `road` topic: x forward from the camera, y right, metres):

* `objects[]`: {"id": "lead:12" | "light:7" | "sign:31", "kind": "vehicle|traffic_light|sign", "role": "lead|ego|
  other", "x", "y", "vx", "vy" (m/s relative to the car: x(t) = x + vx (t - ts) is the consumer's extrapolation),
  "sx", "sy" (1 sigma of the displayed position), "age_s" (since the object first appeared), "meas_age_s"
  (since the last measurement went in; > 0 = predicted), "phase" ("appear" | "steady" | "coast" | "leave"),
  "alpha" (0..1: a suggested opacity for the fade in / out), "attrs" (kind-specific: a light's state, a
  sign's class, the lead's distance read-out values), "cls" (vehicles: what kind of road user, {"label": "car" |
  "truck" | "bus" | "bike" | "human" | ..., "confidence", "source"} or null while no sensor says; non-critical: a
  consumer draws by it and must draw something sensible without it), "box" (last image box in frame pixels where
  there is one)}.
* `lines[]`: {"id": "lane:0".."lane:3" (far left, left, right, far right), "edge:0"/"edge:1", "kind":
  "lane_line|road_edge", "y" (at the shared `x` distances; null beyond `reach_m`), "dy" (lateral rate m/s per
  point, for extrapolation), "reach_m" (the line is drawn up to here: the tracker's sigma under 1 m), "conf",
  "age_s", "phase", "alpha"}.
* `path`: {"y"} the model's planned path at `x` (null beyond its end), as measured (not smoothed).
* `road`: the road as one picture (roadview.py): the ego lane's centre line from the camera near the car and
  from the map beyond, side roads, a roundabout, the junction ahead with its arms, the lane count with the
  probability per lane and the ego lane, painted turn arrows; each part with its provenance, a confidence and
  an `approximate` mark when the position under it is weak or dead-reckoned.

Display filtering, and what it costs (the measurement topics are untouched):

* Static objects (lights, signs): one Kalman filter per track in the car's frame, carried between detections
  by the car's own motion (wheel speed and yaw rate when a host feeds them, else the model's) and corrected by
  each detection with its real uncertainty (a light's distance comes from the housing size: +-25 %). So a light
  approaches smoothly at the car's speed instead of stepping by its distance error three times a second.
* Lead: the `lead` topic's tracked values as they are (no added lag), with a fade on appear / disappear and the
  relative speed as `vx`.
* Lines: the `lanes` topic's geometry through two first-order stages whose time constant grows with the
  distance (`line_tau_s` + `line_tau_rel` x: 0.1 s at the car, 0.28 s at 30 m), a blend over `shift_s` when the
  tracker restarts a line on another lane marking (a lane change, a junction), and a fade when a line comes or
  goes. The display trails the measurement by about twice that time constant: smoothness is chosen over
  latency here (owner's ruling 2026-11-18: the display is informational; latency matters on the measurement
  topics). `dy` is the rate at which each point moves, for a consumer that draws between two messages.
* Everything that is predicted, not measured, says so (`meas_age_s`, `phase: "coast"`), and leaves after a
  bounded time (`static_keep_s`, `obj_fade_s`).

No I/O and no wall clock in here; times come from the messages (CLOCK_MONOTONIC ns).
"""
import math

from ..contract import LocalEgo, validity
from ..fastround import round_flat, scales
from .roadview import RoadView

NS = 1e9

DEFAULT_CFG = dict(
    xs=(0.0, 5.0, 10.0, 15.0, 20.0, 25.0, 30.0, 40.0, 50.0, 60.0, 70.0, 80.0, 100.0, 120.0),
    line_tau_s=0.1, line_tau_rel=0.006,   # display smoothing of the line points (critically damped, two first-order
                                 # stages), per stage line_tau_s + line_tau_rel * distance: 0.1 s at the car,
                                 # 0.28 s at 30 m. The far part is where a line steps on the display (its noise
                                 # and its sweep in a turn grow with the distance); the near part stays tight
    line_motion=False,           # carry the drawn lines by the car's motion between frames (slide by the distance
                                 # driven, turn by the yaw) and smooth only the rest. Implemented and off: on the
                                 # recorded drives it predicts the next frame no better than "the line stays
                                 # where it is in the car's frame" (docs/lanes/L22-v2-integration.md)
    line_reach_std_m=1.0,        # a line point whose tracker sigma exceeds this is past the line's reach
    shift_m=0.8, shift_s=0.4,    # a line that moves this much in one frame (a track restart) blends over shift_s
    fade_s=0.3,                  # fade in / out of lines and objects
    lead_keep_s=0.4,             # a lead that disappeared fades out at its extrapolated place for this long
    lead_tau_s=0.1, lead_tau_rel=0.01,   # drawn distance: carried by the relative speed, corrections eased over
                                 # lead_tau_s + lead_tau_rel * distance (0.15 s at 5 m, 0.5 s at 40 m: the tracked
                                 # distance is good near and noisy far); the measured value stays in attrs.dist_m
    # static objects (traffic lights, signs)
    light_sx_rel=0.25, light_sx_floor=1.0,          # distance from the housing size: 25 % (1 sigma)
    light_sy_floor=0.4, light_sy_rel=0.03,          # lateral: 0.4 m + 3 % of the distance
    sign_size_m=0.7, sign_sx_rel=0.3, sign_sy_floor=0.4, sign_sy_rel=0.03,
    static_q=0.4, static_q_rel=0.02,                # process noise per sqrt(s): ego-motion error (m, + of distance)
    static_gate=3.0,             # a detection more than this many sigmas off counts with a weight falling as 1 / residual
    static_ease_s=0.25,          # the drawn position eases to the filter's over this (corrections, never motion)
    static_ease_rate=1.0, static_ease_rate_rel=0.1,   # ... at most this fast: m/s + of the distance per s
    static_keep_s=1.0,           # a static object not reported for this long leaves (fade out)
    static_min_x=1.0,            # ... and when it has passed the camera
    ego_max_age_s=0.5,           # vehicle motion older than this is not used (the model's is)
    max_gap_s=1.0,               # frames further apart than this restart the channel
    display_lanes_max_age_s=0.5, # an external display lane source (`display_lanes`) older than this is not used
    display_lanes_mode="agree",  # "agree": a line is the source's where source and road model agree on it (and
                                 # the source's extra lines while it is so anchored), the model's elsewhere;
                                 # "prefer": the source's lines where it has them, the model's elsewhere;
                                 # "replace": only the source's lines are drawn while it is fresh
    display_agree_m=0.5, display_agree_s=0.6,     # "agree": within this at 6 and 12 m, for this long
    display_agree_extra=False,   # "agree": also draw lines only the source has, while one of its lines agrees
    display_scale=True, display_scale_tau_s=8.0,   # bring the source's metres to the road model's (lane-width ratio)
    max_static=24,
)


def _cls(c):
    """An object's class for the display: label, confidence, source (non-critical metadata of the track)."""
    if not isinstance(c, dict) or not c.get("label"):
        return None
    return {"label": c["label"], "confidence": c.get("confidence"), "source": c.get("source")}


def _fin(v):
    return isinstance(v, (int, float)) and not isinstance(v, bool) and math.isfinite(v)


def _interp(xs, ys, x):
    if not xs or not ys:
        return None
    if x <= xs[0]:
        return ys[0]
    for i in range(1, min(len(xs), len(ys))):
        if x <= xs[i]:
            a, b = ys[i - 1], ys[i]
            if a is None or b is None:
                return None
            t = (x - xs[i - 1]) / ((xs[i] - xs[i - 1]) or 1.0)
            return a + (b - a) * t
    return None


def _interp_all(xs, ys, qs):
    """[_interp(xs, ys, q) for q in qs], walking the table once: for queries in rising order the segment search
    goes on where the last one ended (every earlier node was already below the smaller query); a query below
    its predecessor starts over. The same comparisons and arithmetic as `_interp`, so the same numbers."""
    if not xs or not ys:
        return [None] * len(qs)
    n = min(len(xs), len(ys))
    x0, y0 = xs[0], ys[0]
    out = []
    i, prev = 1, None
    for x in qs:
        if x <= x0:
            out.append(y0)
            continue
        if prev is not None and not x >= prev:
            i = 1
        prev = x
        while i < n and not x <= xs[i]:
            i += 1
        if i >= n:
            out.append(None)
            continue
        a, b = ys[i - 1], ys[i]
        if a is None or b is None:
            out.append(None)
            continue
        t = (x - xs[i - 1]) / ((xs[i] - xs[i - 1]) or 1.0)
        out.append(a + (b - a) * t)
    return out


class _Static:
    """A static object in the car's frame: two scalar Kalman filters (x, y), carried by ego motion."""

    def __init__(self, oid, kind, x, y, sx, sy, ts):
        self.id, self.kind = oid, kind
        self.x, self.y = x, y
        self.px, self.py = sx * sx, sy * sy
        self.t0 = self.ts = self.meas_ts = ts
        self.attrs, self.box, self.role = {}, None, "other"
        self.vx, self.vy = 0.0, 0.0
        self.alpha = 0.0
        self.leaving = False
        self.dx, self.dy = x, y                 # drawn position: carried like the filter, eased to it

    @staticmethod
    def _move(x, y, v, dpsi, dt):
        x = x - v * dt
        return x + y * dpsi, y - x * dpsi

    def predict(self, ts, v, yaw_r, cfg):
        dt = (ts - self.ts) / NS
        if dt <= 0.0:
            return
        dpsi = yaw_r * dt                       # positive right
        self.x, self.y = self._move(self.x, self.y, v, dpsi, dt)
        self.dx, self.dy = self._move(self.dx, self.dy, v, dpsi, dt)
        k = 1.0 - math.exp(-dt / cfg["static_ease_s"]) if cfg["static_ease_s"] > 0 else 1.0
        cap = (cfg["static_ease_rate"] + cfg["static_ease_rate_rel"] * abs(self.x)) * dt
        self.dx += max(-cap, min(cap, k * (self.x - self.dx)))
        self.dy += max(-cap, min(cap, k * (self.y - self.dy)))
        q = (cfg["static_q"] + cfg["static_q_rel"] * abs(self.x)) ** 2 * dt
        self.px += q + (abs(self.y) * 0.02 * dt) ** 2
        self.py += q + (abs(self.x) * 0.02 * dt) ** 2
        self.vx, self.vy = -v + self.y * yaw_r, -self.x * yaw_r
        self.ts = ts

    def update(self, x, y, sx, sy, ts, gate):
        rx, ry = sx * sx, sy * sy
        for nu, s2, which in ((x - self.x, self.px + rx, "x"), (y - self.y, self.py + ry, "y")):
            g = gate * math.sqrt(s2)
            if abs(nu) > g:                     # Huber: far off counts less (a mis-sized box, a merged pair)
                if which == "x":
                    rx *= (abs(nu) / g) ** 2
                else:
                    ry *= (abs(nu) / g) ** 2
        kx, ky = self.px / (self.px + rx), self.py / (self.py + ry)
        self.x += kx * (x - self.x)
        self.y += ky * (y - self.y)
        self.px *= (1.0 - kx)
        self.py *= (1.0 - ky)
        self.meas_ts = ts
        self.leaving = False


class _Line:
    def __init__(self, ys, ts):
        self.a = list(ys)                       # first stage
        self.b = list(ys)                       # displayed
        self.dy = [0.0] * len(ys)
        self.t0 = ts
        self.alpha = 0.0
        self.shift_t = None
        self.present = True
        self.reach = None
        self.conf = None


class ViewBuilder:
    def __init__(self, cfg=None):
        self.cfg = dict(DEFAULT_CFG, **(cfg or {}))
        self.road_cfg = self.cfg.pop("road", None)
        self._local = LocalEgo()               # the integrator of a builder that is not handed an ego state
        self.reset()

    def reset(self):
        self.ts = None
        self.es = None                         # the integrator's `ego_state` of the last frame (docs/DESIGN.md)
        self.static = {}                       # id -> _Static
        self.lines = {}                        # id -> _Line
        self.lead = None                       # {"id", "x", "y", "vx", "t0", "ts", "alpha", "msg"}
        self.plan = None                       # (x, y) of the last road plan (light positions: offset + path)
        self.ext_lanes = None                  # the last `display_lanes` message of an external lane source
        self.lines_source = "model"            # where the lines of the last `view` came from
        self.lines_scale = None                # the factor applied to the external source's metres (None: no source)
        self.agree_t = [None] * 4              # per slot: since when source and model agree on the line
        if not hasattr(self, "ext_scale"):     # kept across a reset of the channel: it describes the camera
            self.ext_scale, self.scale_ts = None, None
        self.seen = {}                         # id -> first-seen ts (age across fade-out / back in)
        self.road = RoadView(self.road_cfg)    # the road part (roadview.py)

    # ---- inputs ------------------------------------------------------------------------------------------
    def on_ego(self, msg):
        """A mapd `ego` message, for a builder that is not handed the integrator's state."""
        self._local.feed(msg)

    # ---- the lane-source seam (lane L27) -----------------------------------------------------------------
    # The lines a consumer DRAWS may come from another source than the road model: a separate, cheap lane-line
    # analysis whose only job is a good picture. This is the one place where that is decided. The measurement
    # topics (`lanes`, `fused.lanes`, the ego lane, the road geometry) never see such a source: they stay the
    # road model's. Contract of the message (docs/ARCHITECTURE.md "Display lane source"): the shape of `lanes`
    # ({"type":"display_lanes","ts_mono_ns","source","x":[..],"lines":[4 x {"present","y":[..],"y_std"?}],
    # "edges":[2 x ...]?}), device frame, y positive right, from any provider on <rundir>/laned.sock.
    def on_display_lanes(self, msg):
        if isinstance(msg, dict) and isinstance(msg.get("ts_mono_ns"), int) and isinstance(msg.get("x"), list) \
                and isinstance(msg.get("lines"), list):
            self.ext_lanes = msg

    def _display_lanes(self, lanes, ts):
        """The lane lines `view` draws for the frame at ts: from the external display source while it is fresh,
        else the road model's tracked lines (`lanes`). Sets `lines_source` and `lines_scale`.

        `display_lanes_mode` (per line, see DEFAULT_CFG): "agree" (default), "prefer", "replace".
        Road edges stay the road model's unless the source sends its own."""
        c = self.cfg
        ext = self.ext_lanes
        self.lines_scale = None
        if ext is None or abs(ts - ext["ts_mono_ns"]) > c["display_lanes_max_age_s"] * NS:
            self.lines_source = "model"
            return lanes
        have = isinstance(lanes, dict) and isinstance(lanes.get("x"), list)
        k = self._ext_scale(ext, lanes if have else None, ts)
        self.lines_scale = round(k, 3)
        ex = [x * k for x in ext["x"]]
        elines = []
        for ln in (ext.get("lines") or [])[:4]:
            if isinstance(ln, dict) and ln.get("present") and isinstance(ln.get("y"), list):
                ln = dict(ln, y=[y * k for y in ln["y"]])
            elines.append(ln)
        elines += [{"present": False}] * (4 - len(elines))
        name = str(ext.get("source") or "external")
        out = {"x": ex, "lines": elines}
        if "edges" in ext:
            out["edges"] = ext["edges"]
        elif have and lanes.get("edges"):
            out["edges"], out["edges_x"] = lanes["edges"], lanes["x"]
        mode = c["display_lanes_mode"]
        if mode != "replace" and have:
            mlines = list(lanes.get("lines") or [])[:4]
            mlines += [None] * (4 - len(mlines))

            def model_line(i):
                ml = mlines[i]
                if isinstance(ml, dict) and ml.get("present") and isinstance(ml.get("y"), list):
                    # the model's line on the source's distances (cut where they end)
                    ys = _interp_all(lanes["x"], ml["y"], ex)
                    sd = _interp_all(lanes["x"], ml.get("y_std") or [], ex)
                    if all(y is not None for y in ys):
                        return dict(ml, y=ys, y_std=[v if v is not None else 0.0 for v in sd], model=True)
                return None

            def has(ln):
                return isinstance(ln, dict) and bool(ln.get("present")) and isinstance(ln.get("y"), list)

            mod = [model_line(i) for i in range(4)]
            agree = [None] * 4                     # per slot: True / False where both have the line, None otherwise
            for i in range(4):
                if has(elines[i]) and mod[i] is not None:
                    d = max(abs(_interp(ex, elines[i]["y"], x) - _interp(ex, mod[i]["y"], x)) for x in (6.0 * k, 12.0 * k))
                    agree[i] = d <= c["display_agree_m"]
            mixed, used = [], 0
            if mode == "agree":
                # A slot is the source's once the two have agreed on it for display_agree_s (the same marking: the
                # source then gives the better geometry) and until they disagree or the source loses the line. A
                # line only the source has may be drawn while the source is anchored (at least one of its lines
                # agrees with the model's). Everything else is the model's line: where the two read the scene
                # differently nobody can tell which is right, and the picture stays what it was without a source.
                # (The extra lines are off by default, `display_agree_extra`: on the labelled rides the source's
                # extra outer lines were a kerb or a tunnel wall more often than a lane: docs/lanes/L29.)
                for i in range(4):
                    if agree[i]:
                        if self.agree_t[i] is None:
                            self.agree_t[i] = ts
                    else:
                        self.agree_t[i] = None
                ok = [self.agree_t[i] is not None and ts - self.agree_t[i] >= c["display_agree_s"] * NS for i in range(4)]
                anchored = any(ok)
                for i in range(4):
                    if ok[i] or (c["display_agree_extra"] and anchored and has(elines[i]) and mod[i] is None):
                        mixed.append(elines[i])
                        used += 1
                    else:
                        mixed.append(mod[i] or {"present": False})
            else:                                  # "prefer"
                for i in range(4):
                    if has(elines[i]):
                        mixed.append(elines[i])
                        used += 1
                    else:
                        mixed.append(mod[i] or {"present": False})
            # one lane cannot be narrower than 2 m: a source line and a model line that pinch it do not belong
            # together; the source's line of the pair is dropped in favour of the model's
            l, r = mixed[1], mixed[2]
            if has(l) and has(r) and (bool(l.get("model")) != bool(r.get("model"))) and ex[-1] >= 10.0:
                if _interp(ex, r["y"], 10.0) - _interp(ex, l["y"], 10.0) < 2.0:
                    j = 2 if l.get("model") else 1
                    mixed[j] = mod[j] or {"present": False}
                    used -= 1
            # On the model's distances. A source line ends at its own reach (the paint it saw); beyond it the line
            # goes on as the model's line, shifted by the difference at that point, so that a line the model draws
            # to 100 m does not end at 40 m because the source took it over.
            mx = lanes["x"]
            final = []
            for i, ln in enumerate(mixed):
                if not has(ln):
                    final.append({"present": False})
                    continue
                if ln.get("model"):
                    final.append(mlines[i])
                    continue
                sd = ln.get("y_std") or []
                reach = ex[-1]
                for x, v in zip(ex, sd):
                    if v is not None and v > c["line_reach_std_m"] and x > 10.0:
                        break
                    reach = x
                ml = mlines[i] if isinstance(mlines[i], dict) and mlines[i].get("present") and \
                    isinstance(mlines[i].get("y"), list) else None
                msd = (ml.get("y_std") or []) if ml else []
                y_r = _interp(ex, ln["y"], reach)
                dlt = (y_r - (_interp(mx, ml["y"], reach) or y_r)) if ml else 0.0
                ys, st = [], []
                for k2, x in enumerate(mx):
                    if x <= reach:
                        ys.append(_interp(ex, ln["y"], x))
                        v = _interp(ex, sd, x) if sd else None
                        st.append(v if v is not None else 0.1)
                    elif ml:
                        ys.append(ml["y"][k2] + dlt)
                        st.append(msd[k2] if k2 < len(msd) else 0.0)
                    else:
                        ys.append(y_r)
                        st.append(9.0)
                final.append(dict(ln, y=ys, y_std=st))
            out["x"], out["lines"] = mx, final
            out.pop("edges_x", None)
            total = sum(1 for m in mixed if has(m))
            name = "model" if used <= 0 else name if used >= total else name + "+model"
        self.lines_source = name
        return out

    def _ext_scale(self, ext, lanes, ts):
        """The factor that brings the external source's metres to the road model's. A road-plane analysis rests on
        calib.json's camera height, the road model on its own idea of it; where both see the ego lane's two lines
        the ratio of the two lane widths is that factor (low-passed over `display_scale_tau_s`, bounded). It is
        applied to the display lines only, so that they sit right next to the model's objects and path."""
        c = self.cfg
        if not c["display_scale"]:
            return 1.0
        if lanes is not None:
            try:
                el, er, ml, mr = ext["lines"][1], ext["lines"][2], lanes["lines"][1], lanes["lines"][2]
                if all(isinstance(q, dict) and q.get("present") for q in (el, er, ml, mr)):
                    we = _interp(ext["x"], er["y"], 10.0) - _interp(ext["x"], el["y"], 10.0)
                    wm = _interp(lanes["x"], mr["y"], 10.0) - _interp(lanes["x"], ml["y"], 10.0)
                    if we > 2.0 and wm > 2.0 and 0.6 < wm / we < 1.6:
                        dt = 0.0 if self.scale_ts is None else max(0.0, (ts - self.scale_ts) / NS)
                        self.scale_ts = ts
                        if self.ext_scale is None:
                            self.ext_scale = wm / we
                        else:
                            g = 1.0 - math.exp(-min(dt, 1.0) / c["display_scale_tau_s"])
                            self.ext_scale += g * (wm / we - self.ext_scale)
            except (IndexError, KeyError, TypeError):
                pass
        return self.ext_scale if self.ext_scale is not None else 1.0

    def _motion(self, ts):
        """(speed, yaw rate positive right, source) of the car from the integrator's state of the last frame; an
        unknown quantity carries nothing (0)."""
        es = self.es
        if es is None:
            return 0.0, 0.0, "none"
        sp, yr = es.get("speed") or {}, es.get("yaw_rate") or {}
        src = "vehicle" if any(x.startswith("vehicle.") for x in sp.get("sources") or ()) else \
            "vision" if sp.get("valid") else "none"
        return (sp["value"] if sp.get("valid") else 0.0), (-yr["value"] if yr.get("valid") else 0.0), src

    def on_road(self, road, usable=True):
        """The plan of the frame (light positions: offset + path); none while the camera is not believable."""
        plan = road.get("plan") or {}
        if not usable:
            self.plan = None
        elif isinstance(plan.get("x"), list) and isinstance(plan.get("y"), list):
            self.plan = (plan["x"], plan["y"])

    def _static_upsert(self, oid, kind, x, y, sx, sy, ts):
        c = self.cfg
        ob = self.static.get(oid)
        if ob is None:
            if len(self.static) >= c["max_static"]:
                return None
            ob = self.static[oid] = _Static(oid, kind, x, y, sx, sy, ts)
            self.seen.setdefault(oid, ts)
            return ob
        v, r, _ = self._motion(ts)
        ob.predict(ts, v, r, c)
        ob.update(x, y, sx, sy, ts, c["static_gate"])
        return ob

    def on_lights(self, msg):
        """An annotated `lights` message (lights.py adds dist_m / lateral_m / ego)."""
        ts = msg.get("ts_mono_ns")
        if not isinstance(ts, int):
            return
        c = self.cfg
        ego = msg.get("ego") or {}
        for li in msg.get("lights") or []:
            d, lat, tid = li.get("dist_m"), li.get("lateral_m"), li.get("track_id")
            if not _fin(d) or not _fin(lat) or tid is None:
                continue
            # lateral_m is the offset from the planned path at that distance: back to the car's frame
            yp = _interp(*self.plan, d) if self.plan else None
            y = lat + (yp if _fin(yp) else 0.0)
            sx = max(c["light_sx_floor"], c["light_sx_rel"] * d)
            sy = c["light_sy_floor"] + c["light_sy_rel"] * d
            if li.get("coast"):                  # the provider predicted this box: not a measurement
                ob = self.static.get("light:%s" % tid)
                if ob is not None:
                    ob.attrs.update(state=li.get("state"), coast=li.get("coast"))
                continue
            ob = self._static_upsert("light:%s" % tid, "traffic_light", d, y, sx, sy, ts)
            if ob is None:
                continue
            ob.attrs = {"state": li.get("state"), "flashing": bool(li.get("flashing")),
                        "relevance": li.get("relevance"), "state_conf": li.get("state_conf")}
            ob.box = li.get("box")
        for ob in self.static.values():
            if ob.kind == "traffic_light":
                ob.role = "other"
                ob.attrs.pop("ego_state", None)
        if ego:
            ob = self.static.get("light:%s" % ego.get("track_id"))
            if ob is not None:
                ob.role = "ego"
                ob.attrs.update(ego_state=ego.get("state"), held=bool(ego.get("held")), amb=bool(ego.get("amb")),
                                ego_flashing=bool(ego.get("flashing")), agree=ego.get("agree"),
                                ego_relevance=ego.get("relevance"), conf=(ego.get("assign") or {}).get("confidence"),
                                dist_m=ego.get("dist_m"))
                alt = (ego.get("assign") or {}).get("alt")
                ob.attrs["alt"] = {"state": alt.get("state"), "side": alt.get("side")} if alt else None

    def on_signs(self, msg, intrinsics=(690.0, 640.0, 360.0, 1280, 720)):
        """A `signs` message of the provider: live sign tracks as static objects (distance from the box size)."""
        ts = msg.get("ts_mono_ns")
        if not isinstance(ts, int):
            return
        c = self.cfg
        f, cx = intrinsics[0], intrinsics[1]
        for dt in msg.get("detections") or []:
            box, tid = dt.get("box"), dt.get("track_id")
            if tid is None or not box or len(box) != 4 or dt.get("suppressed"):
                continue
            size = max(1.0, box[2] - box[0], box[3] - box[1])
            d = f * c["sign_size_m"] / size
            y = (0.5 * (box[0] + box[2]) - cx) * d / f
            ob = self._static_upsert("sign:%s" % tid, "sign", d, y, c["sign_sx_rel"] * d,
                                     c["sign_sy_floor"] + c["sign_sy_rel"] * d, ts)
            if ob is not None:
                ob.attrs = {"cls": dt.get("cls"), "score": dt.get("score")}
                ob.box = box

    # ---- one frame ---------------------------------------------------------------------------------------
    def on_map(self, msg):
        self.road.on_map(msg)

    def on_frame(self, road, lead, lanes, geom=None, egolane=None, ego=None):
        """After the api's `lead` / `lanes` for a road frame: returns the `view` message for that frame. `geom`:
        RoadGeometryFusion's result for the frame; `egolane`: the ego-lane filter's; `ego`: the integrator's
        `ego_state` of the frame. While the integrator holds the vision source faulted the picture carries no
        lead, no lines and no path, and `integrity` says why."""
        c = self.cfg
        ts = road.get("ts_mono_ns")
        if not isinstance(ts, int):
            return None
        es = ego if ego is not None else self._local.at(road)
        if self.ts is not None and (ts <= self.ts or ts - self.ts > c["max_gap_s"] * NS):
            keep_road = self.road
            self.reset()
            self.road = keep_road
        self.es = es
        valid, reason, inputs = validity(es, ("vision",), ())
        self.on_road(road, valid)
        if not valid:
            lead = lanes = None
        dt = 0.0 if self.ts is None else (ts - self.ts) / NS
        self.ts = ts
        v, r, src = self._motion(ts)
        fade = dt / c["fade_s"] if c["fade_s"] > 0 else 1.0
        objects = []
        lo = self._lead(lead, ts, dt, fade)
        if lo is not None:
            objects.append(lo)
        for oid in list(self.static):
            ob = self.static[oid]
            ob.predict(ts, v, r, c)
            age = (ts - ob.meas_ts) / NS
            if age > c["static_keep_s"] or ob.x < c["static_min_x"]:
                ob.leaving = True
            ob.alpha = max(0.0, ob.alpha - fade) if ob.leaving else min(1.0, ob.alpha + fade)
            if ob.leaving and ob.alpha == 0.0:
                del self.static[oid]
                self.seen.pop(oid, None)
                continue
            phase = "leave" if ob.leaving else "appear" if ob.alpha < 1.0 else "coast" if age > 0.5 else "steady"
            objects.append({"id": oid, "kind": ob.kind, "role": ob.role, "x": round(ob.dx, 2), "y": round(ob.dy, 2),
                            "vx": round(ob.vx, 2), "vy": round(ob.vy, 2),
                            "sx": round(math.sqrt(ob.px), 2), "sy": round(math.sqrt(ob.py), 2),
                            "age_s": round((ts - self.seen.get(oid, ob.t0)) / NS, 2), "meas_age_s": round(age, 3),
                            "phase": phase, "alpha": round(ob.alpha, 2), "attrs": ob.attrs, "box": ob.box})
        lines = self._lines(self._display_lanes(lanes, ts) if valid else None, ts, dt, fade)
        path = None
        if self.plan:
            ys = _interp_all(self.plan[0], self.plan[1], c["xs"])
            end = self.plan[0][-1] if self.plan[0] else 0.0
            path = {"id": "path", "kind": "path",
                    "y": [round(y, 2) if y is not None and x <= end else None for x, y in zip(c["xs"], ys)]}
        return {"type": "view", "seq": road.get("seq"), "ts_mono_ns": ts, "ts_real_ns": road.get("ts_real_ns"),
                "x": list(c["xs"]),
                "ego": {"speed_mps": round(v, 2), "yaw_rate_rps": round(-r, 4), "source": src},   # positive left
                # the health of what the picture is made from: valid false = the camera is not believable now
                # (the reason says why); a consumer shows that instead of an empty road
                "integrity": {"valid": valid, "reason": reason, "inputs": inputs},
                "objects": objects, "lines": lines, "lines_source": self.lines_source, "lines_scale": self.lines_scale,
                "path": path,
                "road_model": road.get("road_model"),
                "road": self.road.on_frame(ts, v, geom, egolane)}

    def _lead(self, lead, ts, dt, fade):
        c = self.cfg
        cur = self.lead
        if isinstance(lead, dict) and lead.get("present") and _fin(lead.get("dist_m")):
            oid = "lead:%s" % lead.get("track_id")
            if cur is None or cur["id"] != oid:
                self.seen.setdefault(oid, ts)
                if cur is not None:
                    self.seen.pop(cur["id"], None)
                cur = self.lead = {"id": oid, "alpha": 0.0 if cur is None else cur["alpha"]}
            vx = lead.get("v_rel_mps") if _fin(lead.get("v_rel_mps")) else 0.0
            d, y = lead["dist_m"], lead.get("lateral_m") or 0.0
            if "x" in cur and dt > 0:
                k = 1.0 - math.exp(-dt / (c["lead_tau_s"] + c["lead_tau_rel"] * d))
                px = cur["x"] + vx * dt
                d = px + k * (d - px)
                y = cur["y"] + min(1.0, 2.0 * k) * (y - cur["y"])
            cur.update(x=d, y=y, vx=vx, ts=ts, msg=lead, gone=False, alpha=min(1.0, cur["alpha"] + fade))
            coast = lead.get("mode") == "coast"
            phase = "appear" if cur["alpha"] < 1.0 else "coast" if coast else "steady"
        elif cur is not None:
            # gone: fade out where it would be now (its last relative speed), for lead_keep_s at most
            gone_s = (ts - cur["ts"]) / NS
            cur["alpha"] = max(0.0, cur["alpha"] - dt / c["lead_keep_s"])
            if cur["alpha"] == 0.0 or gone_s > c["lead_keep_s"]:
                self.seen.pop(cur["id"], None)
                self.lead = None
                return None
            cur["x"] += cur["vx"] * dt
            phase = "leave"
        else:
            return None
        m = cur["msg"]
        return {"id": cur["id"], "kind": "vehicle", "role": "lead", "x": round(cur["x"], 2), "y": round(cur["y"], 2),
                "vx": round(cur["vx"], 2), "vy": 0.0,
                "sx": m.get("dist_filter_std_m"), "sy": None,
                "age_s": round((ts - self.seen.get(cur["id"], ts)) / NS, 2),
                "meas_age_s": round((ts - cur["ts"]) / NS + (m.get("meas_age_s") or 0.0), 3),
                "phase": phase, "alpha": round(cur["alpha"], 2),
                "attrs": {"dist_m": m.get("dist_m"), "dist_std_m": m.get("dist_std_m"), "v_rel_mps": m.get("v_rel_mps"),
                          "a_lead_mps2": m.get("a_lead_mps2"), "time_gap_s": m.get("time_gap_s"), "ttc_s": m.get("ttc_s"),
                          "standing": m.get("standing"), "departed": m.get("departed"), "mode": m.get("mode")},
                # what to draw it as: {"label": "car|truck|bus|bike|human|...", "confidence", "source"} or null (unknown)
                "cls": _cls(m.get("class")),
                "box": None}

    def _lines(self, lanes, ts, dt, fade):
        c = self.cfg
        xs = c["xs"]
        out = []
        src = []
        if isinstance(lanes, dict) and isinstance(lanes.get("x"), list):
            for kind, key, names in (("lane_line", "lines", 4), ("road_edge", "edges", 2)):
                for i, ln in enumerate((lanes.get(key) or [])[:names]):
                    src.append(("%s:%d" % ("lane" if kind == "lane_line" else "edge", i), kind, ln))
        gains = [1.0 - math.exp(-dt / (c["line_tau_s"] + c["line_tau_rel"] * x)) if dt > 0 and c["line_tau_s"] > 0 else 1.0
                 for x in xs]
        mv, mr, _ = self._motion(ts)
        seen = set()
        nx = len(xs)
        xs_rising = all(a <= b for a, b in zip(xs, xs[1:]))     # then "x <= reach" holds for a leading run of points
        flat, todo = [], []
        for lid, kind, ln in src:
            seen.add(lid)
            st = self.lines.get(lid)
            ok = isinstance(ln, dict) and ln.get("present") and isinstance(ln.get("y"), list)
            if ok:
                gx = lanes["x"] if kind == "lane_line" else lanes.get("edges_x") or lanes["x"]
                ys = _interp_all(gx, ln["y"], xs)
                stds = _interp_all(gx, ln.get("y_std") or [], xs)
                ok = ys[0] is not None and ys[1] is not None
            if ok:
                reach = xs[-1]
                for i, (x, s) in enumerate(zip(xs, stds)):
                    if ys[i] is None or (s is not None and s > c["line_reach_std_m"] and x > 10.0):
                        reach = xs[i - 1]
                        break
                last = ys[0]
                for i in range(len(ys)):                 # beyond the reach: carry the last value (never drawn)
                    if ys[i] is None:
                        ys[i] = last
                    last = ys[i]
                if st is None or not st.present and st.alpha == 0.0:
                    st = self.lines[lid] = _Line(ys, ts)
                    self.seen.setdefault(lid, ts)
                else:
                    # a restart on another marking moves the near part too; a turn sweeps only the far part
                    near = [y - b for y, b, x in zip(ys, st.b, xs) if 5.0 <= x <= 20.0]
                    jump = abs(sum(near) / len(near))
                    if jump > c["shift_m"] and st.shift_t is None:
                        st.shift_t = ts
                    k = None
                    if st.shift_t is not None:
                        since = (ts - st.shift_t) / NS
                        if since < c["shift_s"]:
                            k = min(1.0, dt / max(1e-3, c["shift_s"] - since + dt))   # linear over shift_s
                        else:
                            st.shift_t = None
                    ks = gains if k is None else [k] * len(xs)
                    prev = st.b
                    if c["line_motion"]:
                        # where the drawn line is now if it stands still in the world, then a correction
                        # towards the tracked line: the smoothing acts on the difference only
                        pred, rate = self._carry(prev, xs, mv, mr, dt)
                        st.b = [p + (y - p) * k for p, y, k in zip(pred, ys, ks)]
                        st.a = list(st.b)
                        st.dy = rate
                    else:
                        st.a = [p + (y - p) * k for p, y, k in zip(st.a, ys, ks)]
                        st.b = [p + (y - p) * k for p, y, k in zip(st.b, st.a, ks)] if st.shift_t is None else list(st.a)
                        if dt > 0:
                            st.dy = [0.7 * d0 + 0.3 * (b - p) / dt for d0, b, p in zip(st.dy, st.b, prev)]
                st.present, st.reach, st.conf = True, reach, ln.get("conf")
                st.alpha = min(1.0, st.alpha + fade)
            elif st is not None:
                st.present = False
                st.dy = [0.0] * len(st.dy)
                st.alpha = max(0.0, st.alpha - fade)
            if st is None:
                continue
            if not st.present and st.alpha == 0.0:
                del self.lines[lid]
                self.seen.pop(lid, None)
                continue
            phase = "leave" if not st.present else "appear" if st.alpha < 1.0 else \
                "shift" if st.shift_t is not None else "coast" if (ln or {}).get("mode") == "coast" else "steady"
            # the points within reach, offsets then rates: rounded for all lines at once below
            m = sum(1 for x in xs if x <= st.reach) if xs_rising else None
            o = {"id": lid, "kind": kind, "y": None, "dy": None,
                 "reach_m": st.reach, "conf": st.conf, "age_s": round((ts - self.seen.get(lid, st.t0)) / NS, 2),
                 "phase": phase, "alpha": round(st.alpha, 2)}
            if m is None or len(st.b) != nx or len(st.dy) != nx:
                o["y"] = [round(y, 2) if x <= st.reach else None for x, y in zip(xs, st.b)]
                o["dy"] = [round(d, 2) if x <= st.reach else None for x, d in zip(xs, st.dy)]
            else:
                flat += st.b[:m]
                flat += st.dy[:m]
                todo.append((o, m))
            out.append(o)
        if todo:
            rounded = round_flat(flat, self._scale(len(flat)), self._two(len(flat)))
            i = 0
            for o, m in todo:
                pad = [None] * (nx - m)
                o["y"] = rounded[i:i + m] + pad
                o["dy"] = rounded[i + m:i + 2 * m] + pad
                i += 2 * m
        for lid in [k for k in self.lines if k not in seen]:
            del self.lines[lid]
        return out

    _SCALES = {}

    @classmethod
    def _scale(cls, n):
        """The scale array of `fastround.round_flat` for n numbers with two digits."""
        sc = cls._SCALES.get(n)
        if sc is None:
            sc = cls._SCALES[n] = (scales([2] * n), [2] * n)
        return sc[0]

    @classmethod
    def _two(cls, n):
        return cls._SCALES[n][1]

    @staticmethod
    def _carry(ys, xs, v, r, dt):
        """A line that stands still in the world, one frame later in the car's frame: the car drove v dt along
        it (the line's values slide towards the car) and turned by r dt (positive right: every point moves left
        by its distance times the angle). Returns (ys, lateral rate per point in m/s)."""
        shift, dpsi = max(0.0, v) * dt, r * dt
        out, rate = [], []
        n = len(xs)
        j = 0
        for i, x in enumerate(xs):
            q = x + shift
            while j < n - 2 and xs[j + 1] < q:
                j += 1
            x0, x1 = xs[j], xs[j + 1]
            slope = (ys[j + 1] - ys[j]) / ((x1 - x0) or 1.0)
            y = ys[j] + slope * (min(q, x1) - x0)            # beyond the last point: its value (never drawn)
            out.append(y - x * dpsi)
            rate.append(max(0.0, v) * slope - x * r)
        return out, rate

    def summary(self, msg):
        if not msg:
            return None
        rd = msg.get("road") or {}
        jn, ln = rd.get("junction") or {}, rd.get("lanes") or {}
        return {"road": {"source": rd.get("source"), "map_used": (rd.get("map") or {}).get("used"),
                         "map_why": (rd.get("map") or {}).get("why"),
                         "trusted": (rd.get("position") or {}).get("trusted"),
                         "approximate": (rd.get("position") or {}).get("approximate"),
                         "junction": jn.get("kind"), "junction_m": jn.get("dist_m"),
                         "lanes": ln.get("forward"), "ego_index": ln.get("ego_index"),
                         "branches": len(rd.get("branches") or ()), "ring": bool(rd.get("ring"))},
                "objects": len(msg["objects"]), "lines": len(msg["lines"]), "lines_source": msg.get("lines_source"),
                "lead": next((o["x"] for o in msg["objects"] if o["role"] == "lead"), None),
                "ego_light": next((o["attrs"].get("state") for o in msg["objects"] if o["role"] == "ego"), None)}
