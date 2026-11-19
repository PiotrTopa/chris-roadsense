#!/usr/bin/env python3
"""Stage 2: the api's /ws stream through the car backend's SceneBuilder (any car repo version).
  usage: chain_backend.py WS.jsonl.gz SCENES.jsonl.gz --repo CAR_REPO [--no-view]"""
import argparse, gzip, json, sys
ap = argparse.ArgumentParser(); ap.add_argument("ws"); ap.add_argument("out"); ap.add_argument("--repo", required=True)
ap.add_argument("--no-view", action="store_true")
a = ap.parse_args()
sys.path.insert(0, a.repo)
from cyberpunk_computer.backend.roadsense_scene import SceneBuilder, SceneFeedConfig
msgs = []
for ln in gzip.open(a.ws, "rt"):
    m = json.loads(ln)
    if a.no_view and m.get("type") == "view": continue
    msgs.append((m.pop("_rx_s"), m))
msgs.sort(key=lambda p: p[0])
b = SceneBuilder(SceneFeedConfig())
out = gzip.open(a.out, "wt"); n = 0
for i, (rx, m) in enumerate(msgs):
    nxt = msgs[i + 1][0] if i + 1 < len(msgs) else rx + 1.0
    b.feed(json.dumps(m, separators=(",", ":")), rx)
    now = rx
    while True:
        sc = b.build(now)
        if sc is not None:
            out.write(json.dumps({"_t": now, "scene": sc}, separators=(",", ":")) + "\n"); n += 1
        due = b.time_to_due(now) if hasattr(b, "time_to_due") else None
        if due is None or now + due >= nxt: break
        now += max(due, 1e-4)
out.close(); print("scenes", n, file=sys.stderr)
