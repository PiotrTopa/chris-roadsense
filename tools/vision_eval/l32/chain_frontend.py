#!/usr/bin/env python3
"""Stage 3: backend scenes through the MFD frontend's scene path (read_inputs + SceneDirector), any car repo
version, at the UI's 20 Hz. Dumps per UI frame what the director holds for the renderer.
  usage: chain_frontend.py SCENES.jsonl.gz WS.jsonl.gz ROWS.jsonl.gz --repo CAR_REPO
Row: t, v (own speed, m/s, from the stream), lead [dist, lat] (MFD sign), xs, lanes[1], lanes[2], path, view_step,
     cen = the renderer's road centre per xs (its own rule: both inner lines / one line -+ 1.75 m / path / last),
     cen_src per xs: b / l / r / p / -"""
import argparse, gzip, json, os, sys, types, inspect
ap = argparse.ArgumentParser(); ap.add_argument("scenes"); ap.add_argument("ws"); ap.add_argument("out"); ap.add_argument("--repo", required=True)
a = ap.parse_args()
sys.path.insert(0, a.repo); os.environ.setdefault("SDL_VIDEODRIVER", "dummy")
from cyberpunk_computer.ui.scene.director import SceneDirector
from cyberpunk_computer.ui.scene.inputs import SceneFeed, read_inputs
from cyberpunk_computer.ui.scene.tuning import Tuning
NS = types.SimpleNamespace
scenes = [json.loads(l) for l in gzip.open(a.scenes, "rt")]
speed = []
for ln in gzip.open(a.ws, "rt"):
    if ln.startswith('{"type":"road"'):
        m = json.loads(ln); speed.append((m["_rx_s"], abs(m.get("v_ego") or 0.0)))
tune = Tuning(); clock = [0.0]
try: feed = SceneFeed(clock=lambda: clock[0])
except TypeError: feed = SceneFeed()
director = SceneDirector(tune)
dt = 1.0 / getattr(tune, "fps_scene", 20)
nargs = len(inspect.signature(read_inputs).parameters)
t = scenes[0]["_t"] + 0.5; end = scenes[-1]["_t"]; ai = si = 0; cur_v = 0.0
out = gzip.open(a.out, "wt"); n = 0
while t < end:
    clock[0] = t
    while ai < len(scenes) and scenes[ai]["_t"] + 0.03 <= t:
        feed.push("roadsense_scene", scenes[ai]["scene"]); ai += 1
    while si < len(speed) and speed[si][0] <= t:
        cur_v = speed[si][1]; si += 1
    state = NS(vehicle=NS(real_speed_kmh=cur_v * 3.6, display_speed_kmh=cur_v * 3.6, gear="DRIVE", ready_mode=True, acc_on=True),
               roadsense=NS(state="running", api_ok=True, health={"lights": None}, scene=None))
    inp = read_inputs(state, feed, tune.map_limit) if nargs >= 3 else read_inputs(state, feed)
    director.update(inp, dt)
    xs = list(getattr(director, "xs", None) or [])
    L = [None if l is None else list(l) for l in director.lanes]
    path = None if getattr(director, "path", None) is None else list(director.path)
    cen, src = [], []
    for i in range(len(xs)):
        lv = L[1][i] if L[1] and i < len(L[1]) else None
        rv = L[2][i] if L[2] and i < len(L[2]) else None
        if lv is not None and rv is not None: cen.append((lv + rv) * 0.5); src.append("b")
        elif lv is not None: cen.append(lv - 1.75); src.append("l")
        elif rv is not None: cen.append(rv + 1.75); src.append("r")
        elif path and i < len(path): cen.append(path[i]); src.append("p")
        else: cen.append(cen[-1] if cen else 0.0); src.append("-")
    ld = director.lead
    alpha = getattr(director, "lead_alpha", 1.0)
    row = {"t": round(t, 3), "v": round(cur_v, 2), "view_step": getattr(director, "view_step", None), "vision": getattr(director, "vision", None),
           "lead": None if ld is None or alpha < 0.5 else [ld.dist_m, ld.lat_m], "xs": xs, "l1": L[1], "l2": L[2], "path": path,
           "cen": cen, "cen_src": "".join(src)}
    out.write(json.dumps(row, separators=(",", ":")) + "\n"); n += 1
    t += dt
out.close(); print("ui frames", n, file=sys.stderr)
