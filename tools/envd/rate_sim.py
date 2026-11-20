#!/usr/bin/env python3
"""envd rate study (lane L48-E): replay envd's smoother and the `env` consumers on cached per-frame scores
under different inference schedules, and compare every schedule with the 1 Hz baseline AFTER the consumers.

Inputs (one "session" each):
  ride   <name>.npz of rate_extract.py: embeddings on a 2 Hz grid (frames 0, 10, 20, ... of a 20 fps ride) and
         32x18 thumbnails at 4 Hz. Phase 0 of the 1 Hz baseline = grid points 0, 2, 4, ...; phase 1 (frame
         offset 10) = 1, 3, 5, ... and is the noise floor.
  rec    a car recording (messages.jsonl.gz): the phone's own `env` messages (1 Hz, per-frame scores with
         3 decimals) and the recorded `road` / `signs` / `ego` / `lights` / `map` streams for the consumers.

Smoother: a replica of envd/smoother.h (window in seconds of frame time, hysteresis, minimum hold); checked
against the C++ labels by --check-replica (rides) and against the recorded smoothed labels (recordings).

Consumers replayed (the real code where it is pure, a marked replica where it lives in another repo):
  speed-limit engine   chris_integrator RulesInEffect (trackers/rules.py + speedlimit.py), fed with the
                       candidate `env` plus the session's road/signs/ego/lights: its `area` (type, source).
  fused                chris_integrator trackers/fusion.py Fusion: built_up, place, tunnel every 0.5 s of
                       frame time (with the recorded map when there is one).
  car tunnel rule      replica of the car computer's alerts/functions/tunnel.py vision path: envd label tunnel
                       with confidence >= 0.5 held 3 s = in, not for 5 s = out.
  car curve "wet"      replica of alerts/functions/curve.py: weather label in rain / snow / fog.
  car MFD staleness    replica of backend/roadsense_scene.py: an `env` older than 5 s (arrival) is not shown.
"""
import argparse
import collections
import gzip
import json
import math
import os
import sys
import zlib

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(os.path.dirname(HERE))
sys.path.insert(0, os.path.join(ROOT, "chris-integrator"))

GROUPS = ("place", "weather", "light")
NS = 1e9


# ---------------------------------------------------------------------------------------------------------
# class file / per-frame probabilities (as envd/pipeline.cc)
def load_classes(path):
    cf = json.load(open(path))
    groups = {}
    for g, v in cf["groups"].items():
        groups[g] = (list(v["classes"]), np.asarray(v["emb"], np.float64))
    return cf, groups


def frame_probs(emb, cf, groups):
    """emb [N,D] (L2-normalised) -> {group: (classes, probs [N,k] float32)}"""
    out = {}
    e = emb.astype(np.float64)
    e /= np.maximum(np.linalg.norm(e, axis=1, keepdims=True), 1e-6)
    for g, (cls, W) in groups.items():
        lg = (cf["logit_scale"] * (e @ W.T)).astype(np.float32)
        lg = lg - lg.max(axis=1, keepdims=True)
        p = np.exp(lg)
        p = (p / p.sum(axis=1, keepdims=True)).astype(np.float32)
        out[g] = (cls, p)
    return out


# ---------------------------------------------------------------------------------------------------------
class Smoother:
    """envd/smoother.h, line for line."""

    def __init__(self, n, window_s, hyst, hold_s, min_samples=0):
        self.n, self.w, self.h, self.hold = n, window_s, hyst, hold_s
        self.min_samples = min_samples      # candidate option: keep at least this many samples (0 = as envd)
        self.hist = collections.deque()
        self.label, self.changed_t, self.sm = -1, 0.0, np.zeros(n)

    def update(self, p, t):
        self.hist.append((t, np.asarray(p, np.float64)))
        while self.hist and t - self.hist[0][0] > self.w and len(self.hist) > max(1, self.min_samples):
            self.hist.popleft()
        self.sm = sum(s[1] for s in self.hist) / len(self.hist)
        best = int(np.argmax(self.sm))
        if self.label < 0:
            self.label, self.changed_t = best, t
        elif best != self.label and self.sm[best] > self.sm[self.label] + self.h and t - self.changed_t >= self.hold:
            self.label, self.changed_t = best, t


def with_republish(msgs, every=1.0, max_age_factor=2.5):
    """envd --republish: the last message re-sent every `every` s of arrival time until the next one (never
    once it is older than 2.5 periods); copies keep ts_mono_ns, `_arr` is their arrival time (ns)."""
    out = []
    for k, m in enumerate(msgs):
        out.append(m)
        if k + 1 >= len(msgs):
            break
        t0, t1 = m["ts_mono_ns"], msgs[k + 1]["ts_mono_ns"]
        per = (t1 - t0) / NS
        t = t0 + every * NS
        while t < t1 - 1e6 and (t - t0) / NS <= max_age_factor * max(per, every):
            out.append(dict(m, _arr=int(t)))
            t += every * NS
    return out


def publish(times, probs, idx, window=10.0, hyst=0.10, hold=3.0, min_samples=0):
    """Run the smoothers over the inferences `idx` (indices into times / probs); one `env` message each."""
    sm = {g: Smoother(len(probs[g][0]), window, hyst, hold, min_samples) for g in GROUPS}
    out = []
    last_t = None
    for i in idx:
        t = float(times[i])
        if last_t is not None and t - last_t > 60.0:   # envd resets its smoother after a 60 s gap
            for s in sm.values():
                s.hist.clear()
                s.label = -1
        last_t = t
        msg = {"type": "env", "ts_mono_ns": int(round(t * NS)), "seq": int(i)}
        for g in GROUPS:
            cls, p = probs[g]
            s = sm[g]
            s.update(p[i], t)
            msg[g] = {"label": cls[s.label], "confidence": float(s.sm[s.label]), "stable_s": t - s.changed_t,
                      "scores": {c: float(v) for c, v in zip(cls, s.sm)}, "frame_label": cls[int(np.argmax(p[i]))]}
        out.append(msg)
    return out


# ---------------------------------------------------------------------------------------------------------
# schedules: lists of indices into the session's time grid
def sched_fixed(times, period, phase=0.0):
    """One inference per `period` s of frame time, the newest frame each time (as envd's next_allowed)."""
    idx, nxt = [], times[0] + phase
    for i, t in enumerate(times):
        if t + 1e-6 >= nxt:
            idx.append(i)
            nxt = t + period
    return idx


def sched_jitter(times, dts, seed):
    """envd at "1 Hz" with the phone's real timing: the next frame is the newest one after an interval drawn
    from the recorded env intervals of the car (inference time and load make it 0.9-1.2 s)."""
    rng = np.random.default_rng(seed)
    half = 0.5 * float(np.median(np.diff(times)))      # the grid frame nearest to the drawn time (mean kept)
    idx, nxt = [], times[0] + rng.uniform(0, 1.0)
    for i, t in enumerate(times):
        if t + half + 1e-6 > nxt:
            idx.append(i)
            nxt = t + float(rng.choice(dts))
    return idx


PHONE_DT = os.path.expanduser("~/datasets/envd-cache/l48/phone_env_dt.txt")


def thumb_change(ref, cur):
    """Cheap change score of two 32x18 Y/U/V thumbnails: (|mean luma step|, luma histogram L1 / 2,
    |mean chroma step|)."""
    y0, y1 = ref[0].astype(np.float32), cur[0].astype(np.float32)
    h0 = np.histogram(y0, bins=16, range=(0, 256))[0] / y0.size
    h1 = np.histogram(y1, bins=16, range=(0, 256))[0] / y1.size
    c0 = np.abs(ref[1:].astype(np.float32) - 128).mean()
    c1 = np.abs(cur[1:].astype(np.float32) - 128).mean()
    return abs(y1.mean() - y0.mean()), 0.5 * np.abs(h1 - h0).sum(), abs(c1 - c0)


def sched_event(times, thumbs, thumb_t, period, dy=20.0, dh=0.35, dc=6.0, burst_s=10.0, burst_period=1.0,
                min_gap=1.0):
    """Event-driven: one inference per `period` s; a thumbnail change against the thumbnail of the last
    inference (mean-luma step > dy, histogram distance > dh or mean-chroma step > dc) asks for one at once and
    for a 1 Hz burst of `burst_s` s after it (so the smoothing window fills as at 1 Hz around a transition).
    The thumbnail stream is 4 Hz; a request is served by the next grid frame (<= 0.5 s later)."""
    idx = []
    ref = None
    last_t = -1e9
    burst_until = -1e9
    triggers = 0
    k = 0
    pending = False
    for i, t in enumerate(times):
        # scan the thumbnails up to this grid time
        while k < len(thumb_t) and thumb_t[k] <= t + 1e-6:
            if ref is not None and not pending and thumb_t[k] - last_t >= min_gap:
                a, b, c = thumb_change(ref, thumbs[k])
                if a > dy or b > dh or c > dc:
                    pending = True
            k += 1
        per = burst_period if t < burst_until else period
        if pending or t - last_t + 1e-6 >= per:
            if pending:
                triggers += 1
                burst_until = t + burst_s
            idx.append(i)
            last_t = t
            pending = False
            j = min(max(0, np.searchsorted(thumb_t, t + 1e-6) - 1), len(thumbs) - 1)
            ref = thumbs[j]
    return idx, triggers


# ---------------------------------------------------------------------------------------------------------
# consumers
def held(msgs, grid, key):
    """Value of key(msg) of the newest message with ts <= each grid time (None before the first)."""
    ts = np.array([m["ts_mono_ns"] / NS for m in msgs])
    out = []
    j = -1
    for t in grid:
        while j + 1 < len(ts) and ts[j + 1] <= t + 1e-9:
            j += 1
        out.append(key(msgs[j]) if j >= 0 else None)
    return out


def tunnel_rule(grid, vin, on_s=3.0, off_s=5.0):
    """Car computer tunnel rule, vision path (replica): in after vin held on_s, out after not-vin for off_s."""
    state, since, out = False, None, []
    for t, v in zip(grid, vin):
        v = bool(v)
        if v != state:
            if since is None:
                since = t
            if t - since >= (on_s if v else off_s):
                state, since = v, None
        else:
            since = None
        out.append(state)
    return out


def run_rules(env_msgs, other):
    """Speed-limit engine (RulesInEffect) over the session's messages with this env stream; returns
    [(ts_s, area_type, area_source, limit_kmh, snapshot)] after every message (snapshot at most every 0.25 s)."""
    from chris_integrator.trackers.rules import RulesInEffect
    from chris_integrator.trackers.speedlimit import load_rules
    eng = RulesInEffect(load_rules("pl"))
    stream = merge(env_msgs, other)
    out = []
    snap, snap_t = None, -1e18
    for m in stream:
        if m["type"] == "ego" and "_ego_cached" in m:
            continue
        eng.on_message(m, m.get("_ego"))
        sp = eng.speed
        if sp.now_ns is not None:
            if sp.now_ns - snap_t >= 0.25e9 or snap is None or snap["area"]["type"] != sp.area["type"]:
                snap, snap_t = sp.snapshot(), sp.now_ns
            out.append((sp.now_ns / NS, sp.area["type"], sp.area["source"], sp.current()[0], snap))
    return out


def run_fused(env_msgs, maps, area_series, grid):
    """Fusion facts every grid step (the real Fusion methods), with the session's map messages and the speed
    engine's snapshot as the vision side."""
    from chris_integrator.trackers.fusion import Fusion
    fu = Fusion()
    me = held(env_msgs, grid, lambda m: m) if env_msgs else [None] * len(grid)
    mm = held(maps, grid, lambda m: m) if maps else [None] * len(grid)
    at = np.array([a[0] for a in area_series]) if area_series else np.array([])
    out = []
    for t, e, m in zip(grid, me, mm):
        latest = {}
        if m is not None:
            latest["map"] = m
        j = np.searchsorted(at, t + 1e-9) - 1 if len(at) else -1
        vis = area_series[j][4] if j >= 0 else None
        mo, _ = fu._map(latest, int(t * NS))
        if mo:
            fu._track_junctions(mo)
        bu = fu._built_up(mo, vis)
        sl = fu._speed_limit(mo, vis, bu)
        pl = fu._place(mo, e)
        tn = fu._tunnel(mo, e)
        out.append({"built_up": bu["value"], "bu_src": bu["source"], "place": pl["label"], "limit": sl["limit_kmh"],
                    "limit_src": sl["source"], "place_src": pl["source"], "tunnel": tn["in_tunnel"],
                    "map": mo is not None})
    return out


def merge(a, b):
    """Two time-ordered message lists merged by ts_mono_ns (stable: b first on ties)."""
    out, i, j = [], 0, 0
    k = lambda m: m.get("_arr", m["ts_mono_ns"])
    while i < len(a) or j < len(b):
        if j < len(b) and (i >= len(a) or k(b[j]) <= k(a[i])):
            out.append(b[j])
            j += 1
        else:
            out.append(a[i])
            i += 1
    return out


# ---------------------------------------------------------------------------------------------------------
# metrics
def changes(grid, vals):
    out = []
    for k in range(1, len(vals)):
        if vals[k] != vals[k - 1] and vals[k - 1] is not None:
            out.append((grid[k], vals[k - 1], vals[k]))
    return out


def compare(grid, base, cand, look_back=15.0, look_ahead=60.0, mask=None):
    """Disagreement share and the delay of each baseline change in the candidate (first change of the candidate
    into the same value within [-look_back, +look_ahead] s, each candidate change used once)."""
    valid = [k for k in range(len(grid)) if base[k] is not None and cand[k] is not None]
    dis = sum(base[k] != cand[k] for k in valid) / max(1, len(valid))
    split = {}
    if mask is not None:          # disagreement while the map is in use / not (recordings)
        for nm, want in (("dis_map", True), ("dis_nomap", False)):
            v = [k for k in valid if bool(mask[k]) == want]
            split[nm] = (sum(base[k] != cand[k] for k in v) / len(v)) if v else None
    bc, cc = changes(grid, base), changes(grid, cand)
    used = set()
    delays, missed = [], []
    for t0, a, b in bc:
        best = None
        for n, (t1, x, y) in enumerate(cc):
            if n in used or y != b or t1 < t0 - look_back:
                continue
            if t1 > t0 + look_ahead:
                break
            best = n
            break
        if best is None:
            missed.append((t0, a, b))
        else:
            used.add(best)
            delays.append((cc[best][0] - t0, a, b))
    return dict({"dis": dis, "n_base": len(bc), "n_cand": len(cc), "delays": delays, "missed": missed,
                 "extra": len(cc) - len(used)}, **split)


def summarize_delays(d):
    if not d:
        return "-"
    v = np.array(d)
    return "%.1f / %.1f / %.1f" % (np.median(v), np.percentile(v, 90), v.max())


# ---------------------------------------------------------------------------------------------------------
# sessions
def load_ride(path, cf, groups):
    z = np.load(path)
    fps = float(z["fps"])
    times = z["emb_frame"] / fps
    probs = frame_probs(z["emb"], cf, groups)
    # neutral `signs` messages at 2 Hz (no detections): the speed engine's clock, as the road / signs streams
    # of the car advance it between env messages
    clock = [{"type": "signs", "ts_mono_ns": int(round(t * NS)) + 1, "detections": [], "tracks_ended": []}
             for t in np.arange(times[0], times[-1] + 0.5, 0.5)]
    return {"name": os.path.basename(path)[:-4], "kind": "ride", "times": times, "probs": probs,
            "thumbs": z["thumb"], "thumb_t": z["thumb_frame"] / fps, "other": clock, "maps": []}


def read_lines(path):
    from chris_integrator.replay import read_lines as rl
    yield from rl(path)


def load_rec(path, cache_dir=None):
    """cache_dir: keep the parsed session (with the ego states) as a pickle there (the ego pass is the slow part)."""
    import pickle
    cp = os.path.join(cache_dir, os.path.basename(os.path.normpath(path)) + ".pkl") if cache_dir else None
    if cp and os.path.exists(cp):
        return pickle.load(open(cp, "rb"))
    s = _load_rec(path)
    if cp:
        os.makedirs(cache_dir, exist_ok=True)
        pickle.dump(s, open(cp, "wb"), protocol=pickle.HIGHEST_PROTOCOL)
    return s


def _load_rec(path):
    env, other, maps = [], [], []
    for m in read_lines(path):
        t = m.get("type")
        if not isinstance(m.get("ts_mono_ns"), int):
            continue
        if t == "env":
            env.append(m)
        elif t in ("road", "signs", "ego", "lights"):
            other.append(m)
        elif t == "map":
            maps.append(m)
    env.sort(key=lambda m: m["ts_mono_ns"])
    other.sort(key=lambda m: m["ts_mono_ns"])
    # the ego state of every road frame does not depend on env: computed once (the engine's own LocalEgo path)
    from chris_integrator.contract import LocalEgo
    le = LocalEgo()
    for m in other:
        if m["type"] == "ego":
            le.feed(m)
            m["_ego_cached"] = True
        elif m["type"] == "road":
            m["_ego"] = le.at(m)
    other = [m for m in other if m["type"] != "lights" or True]
    maps.sort(key=lambda m: m["ts_mono_ns"])
    times = np.array([m["ts_mono_ns"] / NS for m in env])
    probs = {}
    for g in GROUPS:
        cls = list(env[0][g]["frame_scores"].keys())
        probs[g] = (cls, np.array([[m[g]["frame_scores"][c] for c in cls] for m in env], np.float32))
    return {"name": os.path.basename(os.path.normpath(path)), "kind": "rec", "times": times, "probs": probs,
            "thumbs": None, "thumb_t": None, "other": other, "maps": maps, "recorded_env": env}


# ---------------------------------------------------------------------------------------------------------
def consumers(sess, msgs, grid, heartbeat=None):
    """Every consumer's view of one env stream on the grid. heartbeat: envd --republish period (s): the copies
    go to the speed engine (which evaluates its area prior on every env arrival) and cap the arrival age of
    the MFD rule; None = only inference messages."""
    r = {}
    rmsgs = with_republish(msgs, heartbeat) if heartbeat else msgs
    for g in GROUPS:
        r[g] = held(msgs, grid, lambda m, g=g: m[g]["label"])
    vin = held(msgs, grid, lambda m: m["place"]["label"] == "tunnel" and m["place"]["confidence"] >= 0.5)
    r["tunnel_vis"] = vin
    r["tunnel_car"] = tunnel_rule(grid, vin)
    r["wet"] = held(msgs, grid, lambda m: m["weather"]["label"] in ("rain", "snow", "fog"))
    ts = np.array([m["ts_mono_ns"] / NS for m in msgs])
    age = []
    for t in grid:
        j = np.searchsorted(ts, t + 1e-9) - 1
        a = (t - ts[j]) if j >= 0 else None
        if a is not None and heartbeat:
            a = min(a, heartbeat)
        age.append(a)
    r["mfd_stale"] = [a is not None and a > 5.0 for a in age]
    area = run_rules(rmsgs, sess["other"])
    r["_area_series"] = area
    at = np.array([a[0] for a in area])
    av = []
    for t in grid:
        j = np.searchsorted(at, t + 1e-9) - 1
        av.append((area[j][1], area[j][2]) if j >= 0 else None)
    r["area"] = [None if a is None else a[0] for a in av]
    r["area_env"] = [None if a is None else (a[0] if a[1] == "env" else "-") for a in av]
    r["limit"] = []
    for t in grid:
        j = np.searchsorted(at, t + 1e-9) - 1
        r["limit"].append((area[j][3] if j >= 0 else None) or 0)
    fu = run_fused(msgs, sess["maps"], area, grid)
    r["fused_place"] = [f["place"] for f in fu]
    r["fused_built_up"] = [f["built_up"] for f in fu]
    r["fused_tunnel"] = [f["tunnel"] for f in fu]
    r["fused_limit"] = [f["limit"] or 0 for f in fu]
    r["fused_built_up"] = [str(v) for v in r["fused_built_up"]]
    r["fused_tunnel"] = [str(v) for v in r["fused_tunnel"]]
    r["fused_place"] = [str(v) for v in r["fused_place"]]
    r["_fused"] = fu
    return r


CONSUMER_KEYS = ("place", "weather", "light", "tunnel_vis", "tunnel_car", "wet", "area", "limit", "fused_place",
                 "fused_built_up", "fused_tunnel", "fused_limit")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--classes", default=os.path.expanduser("~/datasets/envd-cache/envd-classes-probe.json"))
    ap.add_argument("--rides", nargs="*", default=[])
    ap.add_argument("--recs", nargs="*", default=[])
    ap.add_argument("--out", required=True, help="JSON with every number")
    ap.add_argument("--cache", default="", help="directory for parsed recordings")
    a = ap.parse_args()
    cf, groups = load_classes(a.classes)
    sessions = [load_ride(p, cf, groups) for p in a.rides]
    for p in a.recs:
        sessions.append(load_rec(p, a.cache or None))
    results = run_all(sessions)
    json.dump(results, open(a.out, "w"), indent=1, default=float)


def candidates(sess):
    """(name, list of (idx, smoother kwargs, heartbeat)) per candidate; several phases where they exist."""
    T = sess["times"]
    ride = sess["kind"] == "ride"
    step = float(np.median(np.diff(T)))
    out = []
    if ride:
        b0 = list(range(0, len(T), 2))
        out.append(("base_1hz_phase1", [(list(range(1, len(T), 2)), {}, None)]))
        if os.path.exists(PHONE_DT):
            dts = np.loadtxt(PHONE_DT)
            out.append(("base_phone_jitter", [(sched_jitter(T, dts, sd), {}, None) for sd in range(4)]))
    else:
        b0 = list(range(len(T)))
    for P in (2, 3, 5, 10):
        ph = [p * step for p in range(int(round(P / step)))]
        out.append(("fixed_%ds" % P, [(sched_fixed(T, P, phase=x), {}, None) for x in ph]))
        out.append(("fixed_%ds_rep1" % P, [(sched_fixed(T, P, phase=x), {}, 1.0) for x in ph]))
        if 3 * P > 10:
            out.append(("fixed_%ds_w%d" % (P, 3 * P),
                        [(sched_fixed(T, P, phase=x), {"window": 3.0 * P}, None) for x in ph]))
    if ride:
        # pre-declared detector settings (not tuned on the scores): (base period, dy, dh, dc, burst s)
        for tag, P, dy, dh, dc, burst in (("E1", 5, 20.0, 0.35, 6.0, 10.0), ("E2", 5, 30.0, 0.50, 10.0, 10.0),
                                          ("E3", 5, 30.0, 0.50, 10.0, 6.0), ("E4", 10, 30.0, 0.50, 10.0, 10.0),
                                          ("E5", 3, 30.0, 0.50, 10.0, 6.0)):
            idx, ntr = sched_event(T, sess["thumbs"], sess["thumb_t"], P, dy=dy, dh=dh, dc=dc, burst_s=burst)
            name = "event_%s_%ds_b%d" % (tag, P, burst)
            out.append((name, [(idx, {}, None)]))
            out.append((name + "_rep1", [(idx, {}, 1.0)]))
            sess.setdefault("_triggers", {})[name] = ntr
    return b0, out


def session_grid(T, gap=60.0, warmup=15.0, step=0.5):
    """Evaluation times: every 0.5 s inside each stretch of frames (split at gaps > 60 s, where envd resets),
    from 15 s after the stretch starts (the smoother's warm-up) to its last frame."""
    cuts = [0] + [i + 1 for i in range(len(T) - 1) if T[i + 1] - T[i] > gap] + [len(T)]
    parts = [np.arange(T[a] + warmup, T[b - 1], step) for a, b in zip(cuts, cuts[1:])]
    return np.concatenate(parts) if parts else np.array([])


def run_all(sessions):
    res = {}
    for s in sessions:
        T = s["times"]
        b0, cands = candidates(s)
        grid = session_grid(T)
        base_msgs = publish(T, s["probs"], b0)
        base = consumers(s, base_msgs, grid)
        dt = np.diff(T)
        dur = float(dt[dt <= 60.0].sum())      # time with frames (gaps where envd would reset excluded)
        sres = {"duration_s": dur, "base_inferences": len(b0), "cands": {}}
        if s["kind"] == "rec":
            rec = s["recorded_env"]
            agree = {g: float(np.mean([r[g]["label"] == m[g]["label"] for r, m in zip(rec, base_msgs)])) for g in GROUPS}
            sres["replica_vs_phone"] = agree
        sres["base_changes"] = {k: len(changes(grid, base[k])) for k in CONSUMER_KEYS}
        # existence (candidate C): the consumers with no env at all, split by "map in use"
        area0 = run_rules([], s["other"])
        fu0 = run_fused([], s["maps"], area0, grid)
        mp = np.array([f["map"] for f in base["_fused"]])
        ex = {"map_share": float(mp.mean())}
        for key in ("built_up", "limit", "place", "tunnel"):
            b = np.array([str(f[key]) for f in base["_fused"]])
            z = np.array([str(f[key]) for f in fu0])
            for nm, sel in (("map", mp), ("nomap", ~mp)):
                ex["%s_%s" % (key, nm)] = float((b[sel] != z[sel]).mean()) if sel.any() else None
        bsrc = collections.Counter(f["bu_src"] for f, m in zip(base["_fused"], mp) if m)
        lsrc = collections.Counter(f["limit_src"] for f, m in zip(base["_fused"], mp) if m)
        ex["bu_src_map"] = dict(bsrc)
        ex["limit_src_map"] = dict(lsrc)
        sres["existence"] = ex
        sres["base_tunnel_share"] = float(np.mean(base["tunnel_car"]))
        for name, runs in cands:
            per = []
            for idx, kw, hb in runs:
                msgs = publish(T, s["probs"], idx, **kw)
                c = consumers(s, msgs, grid, hb)
                m = {"inferences": len(idx), "rate": len(idx) / max(1.0, dur)}
                mask = [f["map"] for f in base["_fused"]] if s["kind"] == "rec" else None
                for k in CONSUMER_KEYS:
                    cmpk = compare(grid, base[k], c[k], mask=mask)
                    m[k] = cmpk
                m["mfd_stale_share"] = float(np.mean(c["mfd_stale"]))
                per.append(m)
            sres["cands"][name] = per
            print(s["name"], name, "rate %.3f" % per[0]["rate"],
                  " ".join("%s %.3f" % (k, np.mean([p[k]["dis"] for p in per])) for k in CONSUMER_KEYS),
                  file=sys.stderr, flush=True)
        if "_triggers" in s:
            sres["triggers"] = s["_triggers"]
        res[s["name"]] = sres
    return res


if __name__ == "__main__":
    main()
