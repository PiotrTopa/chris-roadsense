#!/usr/bin/env python3
"""Replay a drive-recorder session through the trackers and the display channel `view`, and measure what a
consumer would draw: the measurement topics as they are (`lanes`, `lead`, the ego light of `lights`) against
the display channel (docs/lanes/L19-view-channel.md).

    tools/view_replay.py SESSION/messages.jsonl.gz [--ws OUT.jsonl.gz] [--api DIR]

The session holds `road` (reduced), `lights` (annotated as they went out), `signs`, `ego`, `map`, `calib`;
`lead`, `lanes` and `view` are recomputed here exactly as the api does (api/roadsense_api/state.py order:
road -> lead -> lanes -> view).

Printed (per drawn frame = per road frame, what a consumer that draws at the model's rate sees):

* ego traffic light: the drawn distance step per frame after the car's own motion is taken out
  (dx + v dt; a static light approaches at the car's speed), rms / p95 / frames with a step over 2 m, and the
  lateral step; measured = the last `lights.ego` held until the next lights message, view = the view object.
* lane lines (inner pair) and road edges at 10 / 30 m: step per frame rms / p95, steps over 0.3 m, and lines
  that pop in or out (present one frame, gone the next, with no fade); the display lag of view against `lanes`
  (the shift that fits best).
* lead: the distance step per frame after the relative speed is taken out.
* cost of the view builder per frame on this machine.

--ws OUT   writes the api's outgoing stream (road reduced, lead, lanes, view, lights, signs, env, speed_limit,
           rules, fused, map) in the order the api sends it, each line with "_rx_s" = the frame time plus the
           model's latency (when it left the api), for a consumer-side replay (the car computer's scene feed).
"""
import argparse
import gzip
import json
import math
import os
import sys
import time
import zlib

_API = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "api")
if "--api" in sys.argv:
    _API = sys.argv[sys.argv.index("--api") + 1]
sys.path.insert(0, _API)

from roadsense_api.lanes import RoadGeometryTracker      # noqa: E402
from roadsense_api.lead import LeadTracker               # noqa: E402
from roadsense_api.view import ViewBuilder               # noqa: E402

NS = 1e9


def lines(path):
    """JSON objects of a plain or gzip file; a truncated gzip stream yields what is complete."""
    with open(path, "rb") as f:
        if path.endswith(".gz"):
            d, buf = zlib.decompressobj(31), b""
            while True:
                chunk = f.read(1 << 20)
                if not chunk:
                    break
                try:
                    buf += d.decompress(chunk)
                except zlib.error:
                    break
                *full, buf = buf.split(b"\n")
                for ln in full:
                    if ln.startswith(b"{"):
                        try:
                            yield json.loads(ln)
                        except ValueError:
                            pass
        else:
            for ln in f:
                if ln.startswith(b"{"):
                    try:
                        yield json.loads(ln)
                    except ValueError:
                        pass


def interp(xs, ys, x):
    if not xs or not ys:
        return None
    for i in range(1, min(len(xs), len(ys))):
        if x <= xs[i]:
            a, b = ys[i - 1], ys[i]
            if a is None or b is None:
                return None
            return a + (b - a) * (x - xs[i - 1]) / ((xs[i] - xs[i - 1]) or 1.0)
    return None


def pct(v, q):
    v = sorted(v)
    return v[min(len(v) - 1, int(q * len(v)))] if v else float("nan")


def rms(v):
    return math.sqrt(sum(x * x for x in v) / len(v)) if v else float("nan")


def stats(v, big):
    return "rms %.3f  p95 %.3f  max %.2f  over %.1f: %d (n %d)" % (rms(v), pct([abs(x) for x in v], 0.95),
                                                                 max((abs(x) for x in v), default=float("nan")),
                                                                 big, sum(1 for x in v if abs(x) > big), len(v))


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("session")
    ap.add_argument("--ws")
    ap.add_argument("--api")
    a = ap.parse_args()
    lead_tr, lanes_tr, view = LeadTracker(), RoadGeometryTracker(), ViewBuilder()
    intr = (975.0, 640.0, 360.0, 1280, 720)
    out = gzip.open(a.ws, "wt") if a.ws else None
    frames = []
    last_light = None            # last lights.ego (measured, as a consumer holds it)
    cost, nview = 0.0, 0
    t_first = t_last = None

    def emit(m, rx):
        if out is not None:
            out.write(json.dumps(dict(m, _rx_s=round(rx, 4)), separators=(",", ":")) + "\n")

    for m in lines(a.session):
        t = m.get("type")
        ts = m.get("ts_mono_ns")
        if t == "calib" and m.get("focal_px"):
            intr = (float(m["focal_px"]), float(m.get("cx") or 640), float(m.get("cy") or 360),
                    int(m.get("width") or 1280), int(m.get("height") or 720))
        if t == "ego":
            lead_tr.on_ego(m)
            lanes_tr.on_ego(m)
            view.on_ego(m)
            continue
        if t == "map":
            lanes_tr.on_map(m)
        if t == "lights":
            view.on_lights(m)
            last_light = (ts, m.get("ego"))
        if t == "signs":
            view.on_signs(m, intr)
        if t in ("lights", "signs", "env", "speed_limit", "rules", "fused", "map") and isinstance(ts, int):
            emit(m, ts / NS + 0.11)
        if t != "road" or not isinstance(ts, int):
            continue
        rx = ts / NS + (m.get("model_ms") or 60.0) / 1e3 + 0.01
        emit(m, rx)
        ld = lead_tr.on_road(m)
        ln = lanes_tr.on_road(m)
        t0 = time.perf_counter()
        vw = view.on_frame(m, ld, ln)
        cost += time.perf_counter() - t0
        nview += 1
        for x in (ld, ln, vw):
            if x is not None:
                emit(x, rx + 0.001)
        t_first = t_first or ts
        t_last = ts
        frames.append({"t": ts / NS, "lead": ld, "lanes": ln, "view": vw, "light": last_light,
                       "v": (vw or {}).get("ego", {}).get("speed_mps", 0.0)})
    if out is not None:
        out.close()
    if not frames:
        raise SystemExit("no road messages")
    dur = (t_last - t_first) / NS
    print("%s: %d frames, %.0f s, view %.3f ms per frame (this machine), %.1f kB per view message" % (
        os.path.basename(os.path.dirname(os.path.abspath(a.session))), len(frames), dur, 1e3 * cost / max(1, nview),
        sum(len(json.dumps(f["view"], separators=(",", ":"))) for f in frames[::50]) / max(1, len(frames[::50])) / 1e3))

    # ---- ego traffic light
    meas_dx, meas_dy, view_dx, view_dy = [], [], [], []
    shown_meas = shown_view = 0
    for f0, f1 in zip(frames, frames[1:]):
        dt = f1["t"] - f0["t"]
        if dt <= 0 or dt > 0.3:
            continue
        v = f1["v"]
        e0, e1 = (f0["light"] or (0, None))[1], (f1["light"] or (0, None))[1]
        if e1:
            shown_meas += 1
        if e0 and e1 and e0.get("track_id") == e1.get("track_id") and e0.get("dist_m") and e1.get("dist_m"):
            meas_dx.append(e1["dist_m"] - e0["dist_m"] + v * dt)
            meas_dy.append(e1["lateral_m"] - e0["lateral_m"])
        o0 = next((o for o in f0["view"]["objects"] if o["role"] == "ego"), None)
        o1 = next((o for o in f1["view"]["objects"] if o["role"] == "ego"), None)
        if o1:
            shown_view += 1
        if o0 and o1 and o0["id"] == o1["id"]:
            view_dx.append(o1["x"] - o0["x"] + v * dt)
            view_dy.append(o1["y"] - o0["y"])
    print("ego traffic light, drawn per frame (distance step after the car's own motion, m):")
    print("  measured (lights.ego held): " + stats(meas_dx, 2.0) + "; lateral " + stats(meas_dy, 0.5))
    print("  view                      : " + stats(view_dx, 2.0) + "; lateral " + stats(view_dy, 0.5))
    print("  frames with an ego light: measured %d, view %d" % (shown_meas, shown_view))

    # ---- lines
    def series(kind, idx, x):
        out_m, out_v, pops_m, pops_v = [], [], 0, 0
        prev_m = prev_v = None
        for f in frames:
            ln = f["lanes"]
            key = "lines" if kind == "lane" else "edges"
            lm = (ln or {}).get(key, [None] * 4)[idx] if ln else None
            ym = interp(ln["x"], lm["y"], x) if lm and lm.get("present") else None
            lv = next((l for l in f["view"]["lines"] if l["id"] == "%s:%d" % (kind, idx)), None)
            yv = interp(f["view"]["x"], lv["y"], x) if lv else None
            if (ym is None) != (prev_m is None):
                pops_m += 1
            if lv is not None and lv["alpha"] in (0.0, 1.0) and (yv is None) != (prev_v is None):
                pops_v += 1
            out_m.append(ym)
            out_v.append(yv)
            prev_m, prev_v = ym, yv
        return out_m, out_v, pops_m, pops_v

    print("lines, drawn per frame (lateral step, m):")
    for kind, idx, name in (("lane", 1, "left lane line"), ("lane", 2, "right lane line"),
                            ("edge", 0, "left road edge"), ("edge", 1, "right road edge")):
        for x in (10.0, 30.0):
            ym, yv, pm, pv = series(kind, idx, x)
            dm = [b - a for a, b in zip(ym, ym[1:]) if a is not None and b is not None]
            dv = [b - a for a, b in zip(yv, yv[1:]) if a is not None and b is not None]
            # display lag: the frame shift of view against lanes that fits best
            best = None
            for k in range(0, 6):
                d = [yv[i + k] - ym[i] for i in range(len(ym) - k) if ym[i] is not None and yv[i + k] is not None]
                if len(d) > 100:
                    r = rms(d)
                    if best is None or r < best[1]:
                        best = (k, r)
            print("  %-15s %3.0f m  lanes: %s | view: %s | lag %s frames" % (
                name, x, stats(dm, 0.3), stats(dv, 0.3), "-" if best is None else best[0]))
        print("  %-15s        on/off without a fade: lanes %d, view %d" % (name, pm, pv))

    # ---- lead
    lm, lv = [], []
    for f0, f1 in zip(frames, frames[1:]):
        dt = f1["t"] - f0["t"]
        a0, a1 = f0["lead"], f1["lead"]
        if a0 and a1 and a0.get("present") and a1.get("present") and a0["track_id"] == a1["track_id"] and 0 < dt < 0.3:
            lm.append(a1["dist_m"] - a0["dist_m"] - a1["v_rel_mps"] * dt)
        o0 = next((o for o in f0["view"]["objects"] if o["role"] == "lead"), None)
        o1 = next((o for o in f1["view"]["objects"] if o["role"] == "lead"), None)
        if o0 and o1 and o0["id"] == o1["id"] and 0 < dt < 0.3:
            lv.append(o1["x"] - o0["x"] - o1["vx"] * dt)
    print("lead, drawn per frame (distance step after the relative speed, m):")
    print("  lead: " + stats(lm, 0.5))
    print("  view: " + stats(lv, 0.5))


if __name__ == "__main__":
    main()
