#!/usr/bin/env python3
"""Replay a drive-recorder session through the trackers and the display channel `view`, and measure what a
consumer would draw: the measurement topics as they are (`lanes`, `lead`, the ego light of `lights`) against
the display channel (docs/lanes/L19-view-channel.md).

    tools/view_replay.py SESSION/messages.jsonl.gz [--ws OUT.jsonl.gz] [--api DIR]

The session holds `road` (reduced), `lights` (annotated as they went out), `signs`, `ego`, `map`, `calib`;
`lead`, `lanes`, the ego lane, the road geometry, `view` (with `view.road`) and `fused` are recomputed in the
api's order (chris_integrator.replay: a mixer of chris-integrator with the session as its input).

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

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from replay_lib import Pipeline, read_session            # noqa: E402

NS = 1e9


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
    ap.add_argument("--map", help="`map` messages of a mapd replay (tools/map_replay.py --out) instead of the recorded ones")
    ap.add_argument("--set", action="append", default=[], help="view option KEY=VALUE (JSON), e.g. line_motion=false")
    a = ap.parse_args()
    vcfg = {}
    for kv in a.set:
        k, _, v = kv.partition("=")
        vcfg[k] = json.loads(v)
    pipe = Pipeline(vcfg)
    out = gzip.open(a.ws, "wt") if a.ws else None
    frames = []
    last_light = None            # last lights.ego (measured, as a consumer holds it)
    nview = 0
    t_first = t_last = None
    road_stats = {"frames": 0, "moving": 0, "map_used": 0, "trusted": 0, "approx": 0, "junction": 0,
                  "junction_approx": 0, "lanes": 0, "ego": 0, "likely": 0, "turn": 0, "bytes": 0}

    def emit(m, rx):
        if out is not None:
            out.write(json.dumps(dict(m, _rx_s=round(rx, 4)), separators=(",", ":")) + "\n")

    cost = 0.0
    for m in read_session(a.session, a.map):
        t = m.get("type")
        ts = m.get("ts_mono_ns")
        if t == "lights":
            last_light = (ts, m.get("ego"))
        if t in ("ego", "vehicle", "camera", "calib"):
            pipe.feed(m)
            continue
        t0 = time.perf_counter()
        outs = pipe.feed(m)
        if t != "road" or not isinstance(ts, int):
            for x in outs:
                if isinstance(x.get("ts_mono_ns"), int):
                    emit(x, x["ts_mono_ns"] / NS + 0.11)
            continue
        cost += time.perf_counter() - t0
        rx = ts / NS + (m.get("model_ms") or 60.0) / 1e3 + 0.01
        by = {x.get("type"): x for x in outs}
        for x in outs:
            emit(x, rx if x is m else rx + 0.001)
        ld, ln, vw = by.get("lead"), by.get("lanes"), by.get("view")
        if vw is None:
            continue
        nview += 1
        t_first = t_first or ts
        t_last = ts
        rd = vw["road"]
        v = vw["ego"]["speed_mps"]
        road_stats["frames"] += 1
        if v >= 3.0:
            road_stats["moving"] += 1
            road_stats["map_used"] += bool(rd["map"].get("used"))
            road_stats["trusted"] += bool(rd["position"] and rd["position"]["trusted"])
            road_stats["approx"] += bool(rd["position"] and rd["position"]["trusted"] and rd["position"]["approximate"])
            road_stats["junction"] += rd["junction"] is not None
            road_stats["junction_approx"] += bool(rd["junction"] and rd["junction"]["approximate"])
            road_stats["lanes"] += rd["lanes"]["forward"] is not None
            road_stats["ego"] += rd["lanes"]["ego_index"] is not None
            road_stats["likely"] += rd["lanes"]["likely_index"] is not None
            road_stats["turn"] += rd["lanes"]["turn"] is not None
        if nview % 50 == 0:
            road_stats["bytes"] += len(json.dumps(rd, separators=(",", ":")))
        raw30 = []
        for l in (m.get("lane_lines") or [])[:4]:
            y = interp(l.get("x") or [], l.get("y") or [], 30.0) if (l.get("prob") or 0.0) >= 0.5 else None
            raw30.append(y)
        frames.append({"t": ts / NS, "lead": ld, "lanes": ln, "view": vw, "light": last_light, "raw30": raw30,
                       "v": v, "r": -(vw["ego"].get("yaw_rate_rps") or 0.0)})
    if out is not None:
        out.close()
    if not frames:
        raise SystemExit("no road messages")
    dur = (t_last - t_first) / NS
    print("%s: %d frames, %.0f s, the api's derived topics (lead, lanes, ego lane, geometry, view, fused) %.3f ms per "
          "frame (this machine), %.1f kB per view message" % (
              os.path.basename(os.path.dirname(os.path.abspath(a.session))), len(frames), dur, 1e3 * cost / max(1, nview),
              sum(len(json.dumps(f["view"], separators=(",", ":"))) for f in frames[::50]) / max(1, len(frames[::50])) / 1e3))
    mv = max(1, road_stats["moving"])
    print("view.road while moving (%d frames): map layer trusted %.1f %%, of it marked approximate %.1f %%; map geometry "
          "joined %.1f %%; junction given %.1f %% (approximate %.1f %%); lane count %.1f %%, ego lane lit %.1f %%, likely "
          "lane only %.1f %%, turn arrows %.1f %%; %.1f kB per message" % (
              road_stats["moving"], 100.0 * road_stats["trusted"] / mv, 100.0 * road_stats["approx"] / max(1, road_stats["trusted"]),
              100.0 * road_stats["map_used"] / mv, 100.0 * road_stats["junction"] / mv,
              100.0 * road_stats["junction_approx"] / mv, 100.0 * road_stats["lanes"] / mv, 100.0 * road_stats["ego"] / mv,
              100.0 * road_stats["likely"] / mv, 100.0 * road_stats["turn"] / mv,
              road_stats["bytes"] / max(1, nview // 50) / 1e3))

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
            if x == 30.0:
                # what the eye sees as a jump: the step that the car's own turning does not explain (a line
                # that stands still in the world moves by -x * yaw * dt per frame); and how far the line is
                # from where the camera saw it: the reference is the raw model line at 30 m averaged over
                # +-3 frames around the frame (zero phase: no lag, less noise), in turns (|yaw rate| > 0.05
                # rad/s) and elsewhere. A filter that lags shows up in the turning number.
                ref = [None] * len(frames)
                if kind == "lane":
                    for i in range(3, len(frames) - 3):
                        w = [frames[k]["raw30"][idx] if len(frames[k]["raw30"]) > idx else None for k in range(i - 3, i + 4)]
                        if all(v is not None for v in w) and max(w) - min(w) < 1.5 \
                                and frames[i + 3]["t"] - frames[i - 3]["t"] < 0.7:
                            ref[i] = sum(w) / 7.0
                for label, ys in (("lanes", ym), ("view", yv)):
                    res, turn_err, str_err = [], [], []
                    for i in range(1, len(ys)):
                        f0, f1 = frames[i - 1], frames[i]
                        dt = f1["t"] - f0["t"]
                        if ys[i] is None or ys[i - 1] is None or not 0 < dt < 0.3:
                            continue
                        res.append(ys[i] - ys[i - 1] + x * f1["r"] * dt)
                        if ref[i] is not None:
                            (turn_err if abs(f1["r"]) > 0.05 else str_err).append(ys[i] - ref[i])
                    print("  %-15s %3.0f m  %-5s step after the car's turning: %s | off the camera's line: turning rms %.3f (n %d), else rms %.3f" % (
                        name, x, label, stats(res, 0.3), rms(turn_err), len(turn_err), rms(str_err)))
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
