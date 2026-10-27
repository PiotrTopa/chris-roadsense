#!/usr/bin/env python3
"""Offline sign stream for a recorded ride: re-runs signd (signd_replay, laptop build) on exactly the frames the
phone's signd processed in an L11 recording, with any detector/classifier, and writes a copy of the recording in
which every `signs` message is replaced by the offline one (same seq / timestamps), annotated by the api's
signctx like the live api does. The result feeds tools/rules_replay.py unchanged.

usage: ride_signs.py SLUG REC.jsonl OUT.jsonl [--phase P] -- SIGND_REPLAY [signd_replay args...]
--phase P shifts every processed frame by P file frames (0..3: the other "every 4th frame" sequences the phone
could have seen; P = 0 is exactly the recorded pass).
Frames are cut from ~/datasets/rides/SLUG/*.mjpeg (cached in ~/l12/frames/SLUG/, ride files are CC BY 3.0, test
only, never committed); the MJPEG index is ~/l11/frames/SLUG/full/index.npy (L11 frame.py) or built here.
"""
import glob, json, os, subprocess, sys
import numpy as np

sys.path.insert(0, "/home/piotr/projects/roadsense/api")
from roadsense_api import signctx  # noqa: E402

slug, rec, out = sys.argv[1:4]
k = sys.argv.index("--")
phase = int(sys.argv[sys.argv.index("--phase") + 1]) if "--phase" in sys.argv[:k] else 0
cmd = sys.argv[k + 1:]
H = os.path.expanduser("~")
src = glob.glob(f"{H}/datasets/rides/{slug}/*.mjpeg")[0]
idxp = f"{H}/l11/frames/{slug}/full/index.npy"
idx = np.load(idxp)

msgs = [json.loads(l) for l in open(rec)]
cam = {}
for w in msgs:
    m = w["m"]
    if m.get("type") == "camera":
        cam[m["seq"]] = m["file_frame"]
sig = [w["m"] for w in msgs if w["m"].get("type") == "signs"]
frames = [cam.get(m["seq"]) for m in sig]
fd = f"{H}/l12/frames/{slug}"
os.makedirs(fd, exist_ok=True)
files = []
with open(src, "rb") as f:
    for i, ff in enumerate(frames):
        if ff is None:
            ff = frames[i - 1] if i else 0
        ff = min(len(idx) - 2, ff + phase)
        p = f"{fd}/{ff:06d}.jpg"
        if not os.path.exists(p):
            f.seek(idx[ff]); data = f.read(idx[ff + 1] - idx[ff])
            open(p, "wb").write(data)
        files.append(p)
r = subprocess.run(cmd + files, capture_output=True, text=True)
lines = [l for l in r.stdout.splitlines() if l.startswith("{")]
if len(lines) == len(files) + 1:              # final flush line (tracks still open at the end): fold into the last
    last, fl = json.loads(lines[-2]), json.loads(lines[-1])
    last["tracks_ended"] = (last.get("tracks_ended") or []) + (fl.get("tracks_ended") or [])
    lines = lines[:-2] + [json.dumps(last)]
if len(lines) != len(files):
    sys.exit(f"signd_replay gave {len(lines)} lines for {len(files)} frames\n{r.stderr[-2000:]}")
cr = json.load(open(f"{H}/datasets/rides/{slug}/camera-replay.json"))
try:
    calib = json.load(open(f"{H}/datasets/rides/{slug}/calib.json"))
except FileNotFoundError:
    calib = None
intr = (cr.get("focal_px", 690.0), cr.get("cx", 640.0), cr.get("cy", 360.0), 1280, 720)
ctx = signctx.SignContext()
latest = {}
it = iter(lines)
with open(out, "w") as fo:
    for w in msgs:
        m = w["m"]
        t = m.get("type")
        if t == "signs":
            n = json.loads(next(it))
            n.pop("timing", None)
            n.update(seq=m["seq"], ts_mono_ns=m["ts_mono_ns"], ts_real_ns=m.get("ts_real_ns"))
            for e in n.get("tracks_ended") or []:      # replay numbers frames 1..N: back to the ride's seq
                for f in ("first_seq", "last_seq"):
                    if isinstance(e.get(f), int) and 1 <= e[f] <= len(sig):
                        e[f] = sig[e[f] - 1]["seq"]
            ctx.annotate(n, latest.get("road"), latest.get("lights"), calib, intr)
            w = {"t": w["t"], "m": n}
        elif t in ("road", "lights"):
            latest[t] = m
        fo.write(json.dumps(w, separators=(",", ":")) + "\n")
print(f"{slug}: {len(files)} frames, {sum(len(json.loads(l).get('tracks_ended') or []) for l in lines)} tracks ended", file=sys.stderr)
