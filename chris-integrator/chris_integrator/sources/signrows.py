"""Where traffic signs sit in the picture (`signs` detections of a sign detector, boxes in raw-frame pixels): a
measurement of the camera's pitch that does not pass through the road model.

Signs stand at a few degrees above the horizon; at a given apparent size (a given distance) their image row is
a property of the roads driven and of the camera's pitch. Once a reference is learned (while the road model is
healthy and the mount is settled), a shift of the rows at equal box size is a pitch change of the camera:
rows / focal length. It needs driving past signs (about ten tracks per size class), it is coarse (about a
degree), and it keeps working when the road model has left its range, which is exactly when the model's own
pose stops being a mount measurement (measured on the reference car: a camera re-aimed by 16 degrees; the
pose's direction read "aligned", the sign rows had moved by 270 px).

Never fused: it reports (a finding on the camera's mount, and `calibration.signrows`).
"""
import collections
import math
import statistics

from ..native import round                  # builtins.round, native where built
from ..core.registry import NS, Source, fin

BINS = (10.0, 20.0, 30.0, 45.0, 70.0)        # box height classes, px
REF_TRACKS = 60                              # tracks (all classes) before the reference is fixed
RECENT = 25                                  # tracks per class the present value is taken from
MIN_TRACKS = 8                               # ... and at least this many in a class for it to speak
REPORT_DEG = 2.5


class SignRowsSource(Source):
    id = "signrows"
    group = "signrows"
    rank = 0
    types = ("signs", "calib")
    witness = True
    counts = False
    stale_s = 1e9
    info = "image rows of sign detections: a pitch reference that does not pass through the road model"

    def __init__(self, **kw):
        super().__init__(**kw)
        # kept across clock epochs (a restart of the device does not move the camera)
        self.ref = [[] for _ in BINS[:-1]]
        self.ref_med = None
        self.recent = [collections.deque(maxlen=RECENT) for _ in BINS[:-1]]
        self.seen = collections.OrderedDict()
        self.focal = None
        self.result = None

    def measure(self, msg, ctx):
        if msg.get("type") == "calib":
            if fin(msg.get("focal_px")) and msg["focal_px"] > 0:
                self.focal = float(msg["focal_px"])
            return []
        ts = msg["ts_mono_ns"] / NS
        for d in msg.get("detections") or ():
            box = d.get("box")
            if not isinstance(box, (list, tuple)) or len(box) != 4 or not all(fin(b) for b in box):
                continue
            h = box[3] - box[1]
            k = next((i for i in range(len(BINS) - 1) if BINS[i] <= h < BINS[i + 1]), None)
            key = (d.get("track_id"), k)
            if k is None or key in self.seen:
                continue                                   # one sample per track and size class
            self.seen[key] = True
            if len(self.seen) > 4000:
                self.seen.popitem(last=False)
            row = 0.5 * (box[1] + box[3])
            if self.ref_med is None:
                vis = ctx.integ.groups.get("vision")
                if vis is not None and vis.fault is None:   # the reference is learned while the camera is believed
                    self.ref[k].append(row)
                    if sum(len(r) for r in self.ref) >= REF_TRACKS:
                        self.ref_med = [statistics.median(r) if len(r) >= MIN_TRACKS else None for r in self.ref]
            else:
                self.recent[k].append(row)
                self._evaluate(ctx, ts)
        return []

    def _evaluate(self, ctx, ts):
        diffs = []
        for k, rec in enumerate(self.recent):
            if self.ref_med[k] is not None and len(rec) >= MIN_TRACKS:
                diffs += [(statistics.median(rec) - self.ref_med[k], len(rec))]
        if not diffs:
            return
        n = sum(w for _, w in diffs)
        shift = sum(d * w for d, w in diffs) / n
        spread = max(d for d, _ in diffs) - min(d for d, _ in diffs)
        self.result = {"rows_shift_px": round(shift, 1), "classes": len(diffs), "tracks": n, "class_spread_px": round(spread, 1)}
        key = ("vision", "mount", "pitch_from_signs")
        if self.focal:
            deg = math.degrees(math.atan(shift / self.focal))
            self.result["pitch_change_deg"] = round(deg, 1)       # positive: the camera points further up
            if abs(deg) >= REPORT_DEG and (len(diffs) >= 2 or n >= 2 * MIN_TRACKS) and spread < 0.6 * abs(shift) + 30.0:
                ctx.integ.fdi._set(*key, "warn", ts,
                                   "signs sit %.0f px %s in the picture than in the reference period: camera pitch changed by about "
                                   "%.0f deg %s (from %d sign tracks; does not depend on the road model)"
                                   % (abs(shift), "lower" if shift > 0 else "higher", abs(deg), "up" if deg > 0 else "down", n))
                return
        ctx.integ.fdi.findings.pop(key, None)

    def calib_state(self, ctx):
        out = {"reference_tracks": sum(len(r) for r in self.ref), "reference": "fixed" if self.ref_med is not None else "learning"}
        out.update(self.result or {})
        return out

    def report(self, ctx):
        return dict(self.result or {})
