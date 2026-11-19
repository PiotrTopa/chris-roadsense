#!/usr/bin/env python3
"""Mount calibration from driving: the camera's pitch and yaw from a recorded straight drive, no board.

    mount.py SESSION_OR_MESSAGES... [--out mount.json] [--frames] [--lens lens.json] [--verbose]

SESSION_OR_MESSAGES: recorder sessions (folders with messages.jsonl.gz and frames/, docs/ARCHITECTURE.md "Drive
recorder") or message files (the output of tools/modeld_replay). Several inputs are one camera mounting: they are
combined and also listed one by one.

Method 1, the road model's own motion (openpilot selfdrive/locationd/calibrationd.py, handle_cam_odom): the model
sees frames warped with the calibration in use, and its `pose` is the camera's velocity in that frame. While the
car drives straight, the direction of that velocity IS the road direction, so what is left of it sideways and
vertically is the calibration's error:
    R_true = R_used . R(roll 0, pitch -atan2(vz, |vx, vy|), yaw atan2(vy, vx))
The tool picks the straight, steady stretches itself (speed, yaw rate, the model's own certainty, no hard
acceleration; stretches of at least --min-stretch seconds with their ends cut off), forms block medians, and
reports the median of the blocks, their spread over time, and an error estimate that also carries what the
statistics cannot see (the model answers a wrong calibration with slightly less than the full angle).

Method 2 (--frames, needs OpenCV), independent of the road model: the vanishing point of the straight lines in the
pictures taken on those stretches (lane lines, kerbs, barriers), accumulated over all pictures. Works on the
recorder's one picture a second. Printed next to method 1; a disagreement is reported, not averaged away.

Roll and the camera height are not observable from driving: roll stays as configured (check the horizon in
apply.py's view), the height is a tape measurement.

Exit 0 with "status": "ok", 2 with "status": "not_enough" and what is missing.
"""
import argparse
import json
import math
import os
import re
import sys
import zlib

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import geom  # noqa: E402

DEG = math.pi / 180.0

DEFAULTS = dict(
    min_speed=6.7,            # m/s (openpilot MIN_SPEED_FILTER, 15 mph)
    max_yaw_rate=2.0 * DEG,   # rad/s (openpilot MAX_YAW_RATE_FILTER)
    max_angle_std=0.5 * DEG,  # atan2(vy_std, vx); openpilot uses 0.25 deg, which passes little on a phone-rate model
    max_accel=0.7,            # m/s^2 along the road: braking and accelerating pitch the car
    min_stretch=2.0,          # s: a straight stretch must last this long
    trim=0.5,                 # s cut from both ends of a stretch
    block=100,                # accepted samples per block (openpilot BLOCK_SIZE)
    min_blocks=5,             # blocks for an estimate (openpilot INPUTS_NEEDED)
    max_spread=0.5 * DEG,     # robust sigma of the block medians, per angle
    max_halves=0.4 * DEG,     # first half of the blocks against the second half
    gain_loss=0.10,           # share of a calibration error the model's pose does not show (measured on rides:
                              # 0.03-0.12 of a yaw error, about nothing of a pitch error; lane report L33)
)


# What the picture method's bootstrap cannot see: where in the lane the car drove, the road's crossfall, an
# uncorrected lens distortion. Measured (lane report L33): the two methods differ by 0.02-0.19 deg on rides whose
# calibration leaves no residual, and straightening the car camera's nominal distortion moves the result by 0.07-0.2 deg.
PICTURE_FLOOR = 0.15 * DEG
AGREE = 0.7 * DEG            # the two methods agree when they are this close (or within twice their combined error)


def raw_lines(path):
    """Lines (bytes) of a plain or gzip message file; a recording cut by a power loss yields what is complete."""
    if os.path.isdir(path):
        path = os.path.join(path, "messages.jsonl.gz")
    with open(path, "rb") as f:
        d = zlib.decompressobj(31) if path.endswith(".gz") else None
        buf = b""
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
            yield from full
        if buf:
            yield buf


_POSE = re.compile(rb'"pose":(\{[^{}]*\})')
_TS = re.compile(rb'"ts_mono_ns":(-?\d+)')
_SEQ = re.compile(rb'"seq":(\d+)')
_MODEL = re.compile(rb'"road_model":"([^"]*)"')


def read_samples(path):
    """(samples, calibs, model) of one input. samples: array of rows
    [t_s, seq, vx, vy, vz, yaw_rate, vy_std, used_pitch, used_yaw, used_roll]; calibs: the calib messages.
    Only the head and the `pose` object of a road message are parsed (the lines are long)."""
    rows, calibs, used, model = [], [], None, None
    for ln in raw_lines(path):
        if ln.startswith(b'{"type":"calib"'):
            try:
                c = json.loads(ln)
            except ValueError:
                continue
            used = c
            calibs.append(c)
        elif ln.startswith(b'{"type":"road"'):
            pm = _POSE.search(ln)
            if not pm or used is None:
                continue
            head = ln[:pm.start()]
            ts, sq = _TS.search(head), _SEQ.search(head)
            if not ts or b'"valid":false' in head:
                continue
            try:
                q = json.loads(pm.group(1))
                row = (int(ts.group(1)) * 1e-9, int(sq.group(1)) if sq else 0, q["vx"], q["vy"], q["vz"], q["yaw_rate"],
                       q.get("vy_std", 0.0), used.get("pitch") or 0.0, used.get("yaw") or 0.0, used.get("roll") or 0.0)
            except (ValueError, KeyError):
                continue
            if model is None:
                mm = _MODEL.search(ln)
                model = mm.group(1).decode() if mm else "supercombo"
            rows.append(row)
    return np.array(rows, float).reshape(-1, 10), calibs, model


def select(s, o):
    """Boolean mask of the samples on straight, steady stretches."""
    n = len(s)
    if n == 0:
        return np.zeros(0, bool)
    t, vx = s[:, 0], s[:, 2]
    ok = (vx >= o["min_speed"]) & (np.abs(s[:, 5]) <= o["max_yaw_rate"])
    ok &= np.arctan2(s[:, 6], np.maximum(vx, 1e-3)) <= o["max_angle_std"]
    # acceleration along the road: mean speed of the second after against the second before (the model's speed is
    # noisy from frame to frame, a plain difference would reject everything)
    cs = np.concatenate([[0.0], np.cumsum(vx)])
    a0, a1 = np.searchsorted(t, t - 1.5), np.searchsorted(t, t - 0.5)
    b0, b1 = np.searchsorted(t, t + 0.5), np.searchsorted(t, t + 1.5)
    good = (a1 - a0 >= 3) & (b1 - b0 >= 3) & (t - t[np.minimum(a0, n - 1)] <= 1.6) & (t[np.minimum(b1, n) - 1] - t <= 1.6)
    before = (cs[a1] - cs[a0]) / np.maximum(a1 - a0, 1)
    after = (cs[b1] - cs[b0]) / np.maximum(b1 - b0, 1)
    acc = np.where(good, (after - before) / 2.0, 0.0)
    ok &= good & (np.abs(acc) <= o["max_accel"])
    # stretches: runs of accepted samples without a gap, long enough, ends trimmed
    out = np.zeros(n, bool)
    i = 0
    while i < n:
        if not ok[i]:
            i += 1
            continue
        j = i
        while j + 1 < n and ok[j + 1] and t[j + 1] - t[j] < 0.5 and s[j + 1, 7] == s[i, 7] and s[j + 1, 8] == s[i, 8]:
            j += 1
        if t[j] - t[i] >= o["min_stretch"]:
            out[i:j + 1] = (t[i:j + 1] >= t[i] + o["trim"]) & (t[i:j + 1] <= t[j] - o["trim"])
        i = j + 1
    return out


def absolute_angles(s):
    """Per sample: the (pitch, yaw) this sample points at, and how far that is from the calibration in use."""
    d_yaw = np.arctan2(s[:, 3], s[:, 2])
    d_pitch = np.arctan2(s[:, 4], np.hypot(s[:, 2], s[:, 3]))
    out = np.zeros((len(s), 2))
    cache = {}
    for i in range(len(s)):
        key = (s[i, 7], s[i, 8], s[i, 9])
        # small-angle composition is exact enough per sample only for small used angles: do it properly
        r = cache.get(key)
        if r is None:
            r = cache[key] = geom.rot_from_euler(s[i, 9], s[i, 7], s[i, 8])
        rr = r @ geom.rot_from_euler(0.0, -d_pitch[i], d_yaw[i])
        _, out[i, 0], out[i, 1] = geom.euler_from_rot(rr)
    return out, out - s[:, 7:9]


def robust_sigma(v):
    v = np.asarray(v, float)
    return float(1.4826 * np.median(np.abs(v - np.median(v)))) if len(v) > 1 else float("nan")


def blocks_of(t, ang, block):
    """Block medians of consecutive accepted samples: rows [t_mid, n, pitch, yaw]."""
    out = []
    for i in range(0, len(ang) - block + 1, block):
        out.append((float(np.median(t[i:i + block])), block, float(np.median(ang[i:i + block, 0])),
                    float(np.median(ang[i:i + block, 1]))))
    return np.array(out, float).reshape(-1, 4)


def estimate(inputs, opts=None):
    """The mount estimate from message files / sessions. Returns the report dict."""
    o = dict(DEFAULTS, **(opts or {}))
    o["min_blocks"] = max(3, int(o["min_blocks"]))        # a spread needs at least three blocks
    rep = dict(status="not_enough", method="pose", reasons=[], notes=[], options={k: round(v, 5) for k, v in o.items()},
               inputs=[])
    all_blocks, all_abs, all_res, t_off, calib_last, models = [], [], [], 0.0, None, set()
    seen = accepted = 0
    time_accepted = 0.0
    for p in inputs:
        s, calibs, model = read_samples(p)
        info = dict(path=os.path.abspath(p), road_messages=int(len(s)), model=model)
        rep["inputs"].append(info)
        if len(s) == 0:
            info["note"] = "no road messages with a pose"
            continue
        models.add(model)
        calib_last = calibs[-1]
        mask = select(s, o)
        sa = s[mask]
        seen += len(s)
        accepted += len(sa)
        rate = len(s) / max(1e-3, s[-1, 0] - s[0, 0])
        info.update(duration_s=round(float(s[-1, 0] - s[0, 0]), 1), rate_hz=round(rate, 1), accepted=int(len(sa)),
                    accepted_s=round(len(sa) / rate, 1),
                    used=dict(pitch=float(s[-1, 7]), yaw=float(s[-1, 8]), roll=float(s[-1, 9])))
        time_accepted += len(sa) / rate
        if len(sa) == 0:
            continue
        ab, res = absolute_angles(sa)
        b = blocks_of(sa[:, 0], ab, o["block"])
        info["blocks"] = int(len(b))
        if len(ab) >= 30:
            info.update(pitch=float(np.median(ab[:, 0])), yaw=float(np.median(ab[:, 1])),
                        residual_pitch=float(np.median(res[:, 0])), residual_yaw=float(np.median(res[:, 1])))
        if len(b):
            info.update(spread_pitch=robust_sigma(b[:, 2]), spread_yaw=robust_sigma(b[:, 3]))
            b[:, 0] += t_off - s[0, 0]
            all_blocks.append(b)
        t_off += s[-1, 0] - s[0, 0] + 60.0
        all_abs.append(ab)
        all_res.append(res)
        info["_seq"] = sa[:, 1].astype(np.int64)          # accepted frame numbers, for the picture method
    rep.update(samples_seen=int(seen), samples_accepted=int(accepted), accepted_s=round(time_accepted, 1),
               models=sorted(m for m in models if m))
    if calib_last is not None:
        rep["intrinsics_assumed"] = {k: calib_last.get(k) for k in ("focal_px", "cx", "cy", "width", "height")}
        rep["used_calib"] = {k: calib_last.get(k) for k in ("pitch", "yaw", "roll", "height_m")}
    if not all_abs:
        rep["reasons"].append("no straight, steady driving found (%d road messages): needed is driving above %.0f km/h "
                              "on a straight road" % (seen, o["min_speed"] * 3.6))
        return rep
    ab = np.concatenate(all_abs)
    res = np.concatenate(all_res)
    b = np.concatenate(all_blocks) if all_blocks else np.zeros((0, 4))
    rep["blocks"] = [dict(t_s=round(r[0], 1), pitch=round(r[2], 5), yaw=round(r[3], 5)) for r in b]
    need_s = o["min_blocks"] * o["block"] / max(1e-3, accepted / max(1e-3, time_accepted))
    if len(b) < o["min_blocks"]:
        rep["reasons"].append("%.0f s of straight, steady driving found, at least %.0f s are needed (%d blocks of %d "
                              "samples, have %d): drive a straight road above %.0f km/h for a few minutes"
                              % (time_accepted, need_s, o["min_blocks"], o["block"], len(b), o["min_speed"] * 3.6))
        if len(ab) >= 30:
            rep["rough"] = dict(pitch=float(np.median(ab[:, 0])), yaw=float(np.median(ab[:, 1])))
        return rep

    pitch, yaw = float(np.median(b[:, 2])), float(np.median(b[:, 3]))
    sp, sy = robust_sigma(b[:, 2]), robust_sigma(b[:, 3])
    nb = len(b)
    half = nb // 2
    hp = abs(float(np.median(b[:half, 2]) - np.median(b[half:, 2])))
    hy = abs(float(np.median(b[:half, 3]) - np.median(b[half:, 3])))
    # error estimate: the median's standard error, never below half the difference between the two halves of the
    # drive (drift, a change of road or load), plus the share of the residual the model does not show
    rp, ry = float(np.median(res[:, 0])), float(np.median(res[:, 1]))
    stat_p = max(1.2533 * sp / math.sqrt(nb), hp / 2.0)
    stat_y = max(1.2533 * sy / math.sqrt(nb), hy / 2.0)
    sys_p, sys_y = o["gain_loss"] * abs(rp), o["gain_loss"] * abs(ry)
    rep.update(pitch=pitch, yaw=yaw, roll=None,
               residual=dict(pitch=rp, yaw=ry),
               spread=dict(pitch=sp, yaw=sy), halves=dict(pitch=hp, yaw=hy),
               sigma=dict(pitch=math.hypot(stat_p, sys_p), yaw=math.hypot(stat_y, sys_y)),
               sigma_parts=dict(statistical=dict(pitch=stat_p, yaw=stat_y), model_response=dict(pitch=sys_p, yaw=sys_y)),
               n_blocks=nb)
    if sp > o["max_spread"] or sy > o["max_spread"]:
        rep["reasons"].append("the blocks disagree: spread pitch %.2f deg, yaw %.2f deg (limit %.2f): not one steady "
                              "mounting, or not a straight road" % (sp / DEG, sy / DEG, o["max_spread"] / DEG))
    if hp > o["max_halves"] or hy > o["max_halves"]:
        rep["reasons"].append("the first and the second half of the driving disagree: pitch %.2f deg, yaw %.2f deg "
                              "(limit %.2f): the camera moved, or the load / the road changed" % (
                                  hp / DEG, hy / DEG, o["max_halves"] / DEG))
    per = [i for i in rep["inputs"] if i.get("blocks", 0) >= 3]
    for k in ("pitch", "yaw"):
        vals = [i[k] for i in per]
        if len(vals) >= 2 and max(vals) - min(vals) > o["max_halves"]:
            rep["reasons"].append("the recordings disagree on the %s: %s deg: the camera was moved between them; give "
                                  "only recordings made after the last change" % (
                                      k, ", ".join("%+.2f" % (v / DEG) for v in vals)))
    if max(abs(rp), abs(ry)) > 1.0 * DEG:
        rep["notes"].append("the calibration in use was off by pitch %+.2f deg, yaw %+.2f deg; the model shows a little "
                            "less than the full error, so after applying this estimate record another straight drive "
                            "and run this again: it should then report a residual near zero" % (rp / DEG, ry / DEG))
    if rep.get("intrinsics_assumed", {}).get("focal_px"):
        ia = rep["intrinsics_assumed"]
        cam = geom.Camera(dict(focal_px=ia["focal_px"], cx=ia["cx"], cy=ia["cy"]), dict(pitch=pitch, yaw=yaw))
        rep["vp_px"] = [round(float(v), 2) for v in cam.vanishing_point()]
    if not rep["reasons"]:
        rep["status"] = "ok"
        del rep["reasons"]
    return rep


# ---- method 2: the vanishing point of the pictures ---------------------------------------------------------------------

def frame_files(session, seqs, max_gap=3):
    """Recorder pictures (frames/<seq>.jpg) taken on accepted samples: [(seq, path)]."""
    d = os.path.join(session, "frames")
    if not os.path.isdir(d):
        return []
    have = []
    for f in os.listdir(d):
        stem, ext = os.path.splitext(f)
        if ext.lower() in (".jpg", ".jpeg", ".png", ".pgm") and stem.isdigit():
            have.append((int(stem), os.path.join(d, f)))
    have.sort()
    if seqs is None:
        return have
    seqs = np.sort(np.asarray(seqs))
    out = []
    for q, path in have:
        i = np.searchsorted(seqs, q)
        near = min(abs(int(seqs[j]) - q) for j in (max(0, i - 1), min(len(seqs) - 1, i))) if len(seqs) else 1 << 30
        if near <= max_gap:
            out.append((q, path))
    return out


def picture_segments(cv2, gray, min_len=25.0, flat_deg=8.0):
    """Straight segments of a picture as rows [x1, y1, x2, y2], without the nearly horizontal ones."""
    g = cv2.GaussianBlur(gray, (0, 0), 1.0)
    lsd = cv2.createLineSegmentDetector(cv2.LSD_REFINE_STD)
    lines = lsd.detect(g)[0]
    if lines is None:
        return np.zeros((0, 4))
    seg = lines.reshape(-1, 4).astype(float)
    dx, dy = seg[:, 2] - seg[:, 0], seg[:, 3] - seg[:, 1]
    ln = np.hypot(dx, dy)
    ang = np.degrees(np.arctan2(np.abs(dy), np.abs(dx)))
    return seg[(ln >= min_len) & (ang >= flat_deg)]


def static_segments(segs, frame_of, share=0.3, cell=8.0, angle_deg=3.0):
    """Mask of the segments that sit at the same place in many pictures: the bonnet, the dashboard's reflection, a
    burned-in text. They belong to the car, not to the road, and two of them would always "meet" somewhere."""
    n_frames = len(np.unique(frame_of))
    if n_frames < 10:
        return np.zeros(len(segs), bool)
    mx, my = (segs[:, 0] + segs[:, 2]) / 2.0, (segs[:, 1] + segs[:, 3]) / 2.0
    ang = np.degrees(np.arctan2(segs[:, 3] - segs[:, 1], segs[:, 2] - segs[:, 0])) % 180.0
    key = (np.floor(mx / cell).astype(np.int64) * 100003 + np.floor(my / cell).astype(np.int64)) * 1009 \
        + np.floor(ang / angle_deg).astype(np.int64)
    pairs = np.unique(np.stack([key, frame_of.astype(np.int64)], axis=1), axis=0)
    keys, counts = np.unique(pairs[:, 0], return_counts=True)
    static = keys[counts >= max(5, share * n_frames)]
    return np.isin(key, static)


def vanishing_point(segs, frame_of, guess, sigma=4.0, window=(360.0, 240.0), rng=None, boot=40):
    """The point most segments aim at. segs: rows [x1, y1, x2, y2] (pinhole pixels), frame_of: the picture each came
    from, guess: (u, v) to search around. Only segments below a candidate count for it (the road is below the
    horizon). Returns dict(u, v, sigma_u, sigma_v, support, frames) or None."""
    if len(segs) < 20:
        return None
    x1, y1, x2, y2 = segs.T
    ln = np.hypot(x2 - x1, y2 - y1)
    a, b = (y2 - y1) / ln, -(x2 - x1) / ln
    c = -(a * x1 + b * y1)
    my = np.minimum(y1, y2)                            # the upper end of the segment
    w = np.minimum(ln, 150.0)

    def score(pu, pv, sg, wt):
        d = a[None, :] * pu[:, None] + b[None, :] * pv[:, None] + c[None, :]
        below = my[None, :] > pv[:, None] + 5.0
        return np.sum(wt[None, :] * below * np.exp(-0.5 * (d / sg) ** 2), axis=1)

    def search(wt):
        u0, v0 = guess
        best = None
        for step, half, sg in ((12.0, window, 3 * sigma), (3.0, (24.0, 24.0), 1.5 * sigma), (0.75, (6.0, 6.0), sigma)):
            us = np.arange(u0 - half[0], u0 + half[0] + 1e-6, step)
            vs = np.arange(v0 - half[1], v0 + half[1] + 1e-6, step)
            gu, gv = np.meshgrid(us, vs)
            sc = score(gu.ravel(), gv.ravel(), sg, wt)
            k = int(np.argmax(sc))
            u0, v0, best = float(gu.ravel()[k]), float(gv.ravel()[k]), float(sc[k])
        # least squares on the segments that agree
        for _ in range(3):
            d = a * u0 + b * v0 + c
            inl = (np.abs(d) < 2.0 * sigma) & (my > v0 + 5.0) & (wt > 0)
            if inl.sum() < 10:
                break
            A = np.stack([a[inl], b[inl]], axis=1) * np.sqrt(wt[inl])[:, None]
            sol, *_ = np.linalg.lstsq(A, -c[inl] * np.sqrt(wt[inl]), rcond=None)
            u0, v0 = float(sol[0]), float(sol[1])
        return u0, v0, best

    u, v, best = search(w)
    d = a * u + b * v + c
    inl = (np.abs(d) < 2.0 * sigma) & (my > v + 5.0)
    frames = np.unique(frame_of)
    rng = rng or np.random.default_rng(1)
    bs = []
    if len(frames) >= 8:
        for _ in range(boot):
            pick = rng.integers(0, len(frames), len(frames))
            cnt = np.bincount(pick, minlength=len(frames))
            wt = w * cnt[np.searchsorted(frames, frame_of)]
            # refine around the full estimate only
            u0, v0 = u, v
            for _ in range(3):
                dd = a * u0 + b * v0 + c
                il = (np.abs(dd) < 2.0 * sigma) & (my > v0 + 5.0) & (wt > 0)
                if il.sum() < 10:
                    break
                A = np.stack([a[il], b[il]], axis=1) * np.sqrt(wt[il])[:, None]
                sol, *_ = np.linalg.lstsq(A, -c[il] * np.sqrt(wt[il]), rcond=None)
                u0, v0 = float(sol[0]), float(sol[1])
            bs.append((u0, v0))
    bs = np.array(bs) if bs else np.full((1, 2), np.nan)
    return dict(u=u, v=v, sigma_u=float(np.std(bs[:, 0])), sigma_v=float(np.std(bs[:, 1])),
                support=float(np.sum(w[inl]) / max(1e-9, np.sum(w))), segments=int(len(segs)), inliers=int(inl.sum()),
                frames=int(len(frames)), frames_supporting=int(len(np.unique(frame_of[inl]))))


def picture_method(rep, lens=None, max_frames=400, pictures=None, intr=None, verbose=False):
    """Vanishing point from the recorder pictures on the accepted stretches (or from `pictures`: [(id, path)])."""
    try:
        import cv2
    except ImportError:
        return dict(status="unavailable", reason="needs OpenCV (run with the calibration venv's python)")
    ia = intr or rep.get("intrinsics_assumed") or {}
    if not ia.get("focal_px"):
        return dict(status="unavailable", reason="the camera's focal length is not known (no calib message)")
    lens = lens or {}
    k_lens = dict(focal_px=lens.get("focal_px", ia["focal_px"]), cx=lens.get("cx", ia["cx"]), cy=lens.get("cy", ia["cy"]),
                  **{q: lens.get(q, 0.0) for q in geom.DIST_KEYS})
    files = list(pictures or [])
    if pictures is None:
        for info in rep["inputs"]:
            if os.path.isdir(info["path"]) and info.get("_seq") is not None:
                files.extend(frame_files(info["path"], info["_seq"]))
    if len(files) > max_frames:
        files = [files[i] for i in np.linspace(0, len(files) - 1, max_frames).astype(int)]
    if len(files) < 8:
        return dict(status="not_enough", reason="%d pictures on straight stretches (need 8 or more; the recorder keeps "
                    "one a second)" % len(files))
    segs, frame_of = [], []
    for n, (q, path) in enumerate(files):
        gray = cv2.imread(path, cv2.IMREAD_GRAYSCALE)
        if gray is None:
            continue
        s = picture_segments(cv2, gray)
        if len(s) == 0:
            continue
        if any(k_lens[q2] for q2 in geom.DIST_KEYS):         # straighten the end points: pinhole pixels of the same lens
            for c0 in (0, 2):
                x, y = geom.undistort((s[:, c0] - k_lens["cx"]) / k_lens["focal_px"],
                                      (s[:, c0 + 1] - k_lens["cy"]) / k_lens["focal_px"], k_lens)
                s[:, c0], s[:, c0 + 1] = x * k_lens["focal_px"] + k_lens["cx"], y * k_lens["focal_px"] + k_lens["cy"]
        segs.append(s)
        frame_of.append(np.full(len(s), n))
    if not segs:
        return dict(status="not_enough", reason="no straight segments in the pictures")
    segs, frame_of = np.concatenate(segs), np.concatenate(frame_of)
    keep = ~static_segments(segs, frame_of)
    n_static = int((~keep).sum())
    segs, frame_of = segs[keep], frame_of[keep]
    guess = (k_lens["cx"], k_lens["cy"])
    vp = vanishing_point(segs, frame_of, guess)
    if vp is None:
        return dict(status="not_enough", reason="too few straight segments (%d)" % len(segs))
    pitch, yaw = geom.angles_from_vanishing_point(vp["u"], vp["v"], dict(focal_px=k_lens["focal_px"], cx=k_lens["cx"],
                                                                        cy=k_lens["cy"]))
    f = k_lens["focal_px"]
    out = dict(status="ok", pictures=len(files), vp_px=[round(vp["u"], 2), round(vp["v"], 2)], pitch=pitch, yaw=yaw,
               sigma=dict(pitch=math.hypot(vp["sigma_v"] / f, PICTURE_FLOOR), yaw=math.hypot(vp["sigma_u"] / f, PICTURE_FLOOR)),
               sigma_bootstrap=dict(pitch=vp["sigma_v"] / f, yaw=vp["sigma_u"] / f), support=round(vp["support"], 3),
               segments=vp["segments"], inliers=vp["inliers"], frames_supporting=vp["frames_supporting"],
               static_segments_dropped=n_static)
    if vp["frames_supporting"] < 8 or vp["support"] < 0.03:
        out["status"] = "weak"
        out["reason"] = ("few pictures agree on one vanishing point (%d pictures, %.0f %% of the segment length): no "
                         "clear lane lines on the straight stretches" % (vp["frames_supporting"], 100 * vp["support"]))
    return out


def recommend(rep):
    """Which estimate to apply, per angle: the one with the smaller error when the two methods agree; the road
    model's motion when there are no usable pictures; nothing when they contradict each other."""
    p = rep.get("pictures") or {}
    pose_ok = rep.get("status") == "ok" and rep.get("method") != "pictures" and "pitch" in rep
    pic_ok = p.get("status") == "ok"
    if not pose_ok and not pic_ok:
        return None
    if pose_ok and not pic_ok:
        return dict(status="ok", pitch=rep["pitch"], yaw=rep["yaw"], sigma=dict(rep["sigma"]),
                    source=dict(pitch="pose", yaw="pose"), why="no usable pictures: the road model's motion alone")
    if pic_ok and not pose_ok:
        if rep.get("method") == "pictures":
            return dict(status="ok", pitch=p["pitch"], yaw=p["yaw"], sigma=dict(p["sigma"]),
                        source=dict(pitch="pictures", yaw="pictures"), why="pictures only")
        return None                                   # the recordings themselves were judged not good enough
    out = dict(status="ok", source={}, sigma={})
    for k in ("pitch", "yaw"):
        diff = abs(rep[k] - p[k])
        if diff > max(AGREE, 2.0 * math.hypot(rep["sigma"][k], p["sigma"][k])):
            return dict(status="conflict", why="the two methods differ by %.2f deg in %s: neither is recommended; look at "
                        "the view with each (apply.py --mount-source pose | pictures --dry-run)" % (diff / DEG, k))
        src = "pose" if rep["sigma"][k] <= p["sigma"][k] else "pictures"
        out[k] = rep[k] if src == "pose" else p[k]
        out["sigma"][k] = min(rep["sigma"][k], p["sigma"][k])
        out["source"][k] = src
    out["why"] = "both methods agree; per angle the one with the smaller error"
    return out


def print_report(rep, out=None):
    w = (out or sys.stdout).write
    d = lambda v: "%+.2f" % (v / DEG)   # noqa: E731
    for i in rep["inputs"]:
        line = "%-44s %6d road msgs" % (i["path"][-44:], i["road_messages"])
        if "duration_s" in i:
            line += ", %5.0f s, straight %5.0f s, blocks %2d" % (i["duration_s"], i["accepted_s"], i.get("blocks", 0))
        if "pitch" in i:
            line += " | pitch %s yaw %s deg" % (d(i["pitch"]), d(i["yaw"]))
        if "spread_pitch" in i and math.isfinite(i["spread_pitch"]):
            line += " (spread %.2f / %.2f)" % (i["spread_pitch"] / DEG, i["spread_yaw"] / DEG)
        w(line + "\n")
    u = rep.get("used_calib")
    if u:
        w("calibration in use (last): pitch %.4f yaw %.4f roll %.4f rad\n" % (u.get("pitch") or 0, u.get("yaw") or 0,
                                                                            u.get("roll") or 0))
    if "pitch" in rep:
        s, sp = rep["sigma"], rep["spread"]
        w("road model's motion, %d blocks over %.0f s of straight driving (%s):\n" % (
            rep["n_blocks"], rep["accepted_s"], ", ".join(rep["models"])))
        w("  pitch %+.4f rad (%s deg) +- %.2f deg   spread over time %.2f deg, halves differ %.2f deg\n" % (
            rep["pitch"], d(rep["pitch"]), s["pitch"] / DEG, sp["pitch"] / DEG, rep["halves"]["pitch"] / DEG))
        w("  yaw   %+.4f rad (%s deg) +- %.2f deg   spread over time %.2f deg, halves differ %.2f deg\n" % (
            rep["yaw"], d(rep["yaw"]), s["yaw"] / DEG, sp["yaw"] / DEG, rep["halves"]["yaw"] / DEG))
        w("  the calibration in use is off by: pitch %s deg, yaw %s deg (estimate minus used)\n" % (
            d(rep["residual"]["pitch"]), d(rep["residual"]["yaw"])))
        if rep.get("vp_px"):
            w("  road vanishing point at pixel %.1f, %.1f\n" % tuple(rep["vp_px"]))
        w("  roll: not observable from driving (kept as configured)\n")
    elif rep.get("rough"):
        w("rough only (not usable): pitch %s deg, yaw %s deg\n" % (d(rep["rough"]["pitch"]), d(rep["rough"]["yaw"])))
    p = rep.get("pictures")
    if p:
        if p.get("pitch") is not None:
            w("pictures' vanishing point (%d pictures, %d agree, support %.0f %%): pixel %.1f, %.1f\n" % (
                p["pictures"], p["frames_supporting"], 100 * p["support"], p["vp_px"][0], p["vp_px"][1]))
            w("  pitch %s +- %.2f deg, yaw %s +- %.2f deg%s\n" % (
                d(p["pitch"]), p["sigma"]["pitch"] / DEG, d(p["yaw"]), p["sigma"]["yaw"] / DEG,
                "" if p["status"] == "ok" else "   [%s]" % p["status"]))
            if "pitch" in rep:
                w("  difference to the road model's motion: pitch %s deg, yaw %s deg\n" % (
                    d(p["pitch"] - rep["pitch"]), d(p["yaw"] - rep["yaw"])))
        if p.get("reason"):
            w("pictures' vanishing point: %s\n" % p["reason"])
    r = rep.get("recommended")
    if r and r.get("status") == "ok":
        w("recommended: pitch %+.4f rad (%s deg, +- %.2f, from %s), yaw %+.4f rad (%s deg, +- %.2f, from %s)\n" % (
            r["pitch"], d(r["pitch"]), r["sigma"]["pitch"] / DEG, r["source"]["pitch"],
            r["yaw"], d(r["yaw"]), r["sigma"]["yaw"] / DEG, r["source"]["yaw"]))
    elif r:
        w("recommended: nothing: %s\n" % r["why"])
    for t in rep.get("notes", []):
        w("NOTE: %s\n" % t)
    if rep["status"] == "ok":
        w("OK: mount estimate usable\n")
    else:
        for t in rep.get("reasons", []):
            w("NOT ENOUGH: %s\n" % t)


def strip(rep):
    for i in rep["inputs"]:
        i.pop("_seq", None)
    return rep


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("inputs", nargs="*", help="recorder sessions or message files (jsonl, jsonl.gz)")
    ap.add_argument("--pictures", help="instead of recordings: a folder of road pictures from the camera (e.g. frames of "
                    "a dashcam video of a straight road); only the pictures' vanishing point is estimated; needs --focal")
    ap.add_argument("--focal", type=float, help="--pictures: the camera's focal length in pixels")
    ap.add_argument("--cx", type=float)
    ap.add_argument("--cy", type=float)
    ap.add_argument("--out", help="report (JSON); apply.py takes it when its status is ok")
    ap.add_argument("--frames", action="store_true", help="also the vanishing point of the sessions' pictures (OpenCV)")
    ap.add_argument("--lens", help="lens.json: straighten the pictures' segments with its distortion (--frames)")
    ap.add_argument("--max-frames", type=int, default=400)
    ap.add_argument("--min-speed", type=float, help="m/s (default %.1f)" % DEFAULTS["min_speed"])
    ap.add_argument("--max-angle-std-deg", type=float, help="the model's own sideways uncertainty limit (default 0.5)")
    ap.add_argument("--block", type=int, help="samples per block (default %d)" % DEFAULTS["block"])
    ap.add_argument("--min-blocks", type=int, help="blocks needed (default %d, at least 3)" % DEFAULTS["min_blocks"])
    ap.add_argument("--verbose", action="store_true", help="print every block")
    a = ap.parse_args(argv)
    opts = {}
    if a.min_speed is not None:
        opts["min_speed"] = a.min_speed
    if a.max_angle_std_deg is not None:
        opts["max_angle_std"] = a.max_angle_std_deg * DEG
    if a.block:
        opts["block"] = a.block
    if a.min_blocks:
        opts["min_blocks"] = a.min_blocks
    lens = None
    if a.lens:
        lj = geom.load_json(a.lens)
        lens = lj.get("calibration") or lj
    if a.pictures:
        if a.inputs or not (a.focal or lens):
            ap.error("--pictures goes without recordings and needs --focal (or --lens)")
        files = sorted(os.path.join(a.pictures, f) for f in os.listdir(a.pictures)
                       if f.lower().endswith((".jpg", ".jpeg", ".png", ".pgm")))
        if not files:
            ap.error("no pictures in %s" % a.pictures)
        try:
            from PIL import Image
            with Image.open(files[0]) as im:
                w, h = im.size
        except ImportError:
            import cv2
            h, w = cv2.imread(files[0], 0).shape
        intr = dict(focal_px=a.focal or lens["focal_px"], cx=a.cx if a.cx is not None else w / 2.0,
                    cy=a.cy if a.cy is not None else h / 2.0, width=w, height=h)
        rep = dict(status="not_enough", method="pictures", inputs=[], notes=[], intrinsics_assumed=intr)
        p = rep["pictures"] = picture_method(rep, lens, a.max_frames, pictures=list(enumerate(files)), intr=intr)
        if p.get("status") == "ok":
            rep["status"] = "ok"
        else:
            rep["reasons"] = [p.get("reason") or p.get("status")]
        rep["angles_for"] = {k: (lens or intr).get(k, intr[k]) for k in ("focal_px", "cx", "cy")}
        rep["notes"].append("these pictures were not selected for straight driving: the result is only as good as the "
                            "road in them is straight (curved tunnels and winding roads gave 2-3 deg of error on rides "
                            "while looking fine); prefer a recording, where the straight stretches are picked")
        rep["recommended"] = recommend(rep)
        print_report(rep)
        if a.out:
            geom.write_json_atomic(a.out, rep)
            print("report: %s" % a.out)
        return 0 if rep["status"] == "ok" else 2
    if not a.inputs:
        ap.error("give recordings, or --pictures DIR")
    rep = estimate(a.inputs, opts)
    if a.frames:
        rep["pictures"] = picture_method(rep, lens, a.max_frames, verbose=a.verbose)
    rep["angles_for"] = {k: (rep.get("intrinsics_assumed") or {}).get(k) for k in ("focal_px", "cx", "cy")}
    if lens and "pitch" in rep and rep["angles_for"]["focal_px"]:
        # the recording was made with other lens values than the ones given: the road model's estimate fixes a pixel
        # (where the road vanishes); express it as angles of the given lens, like the pictures' estimate
        new = {k: lens.get(k, rep["angles_for"][k]) for k in ("focal_px", "cx", "cy")}
        if any(abs(new[k] - rep["angles_for"][k]) > 0.05 for k in new):
            vp = geom.Camera(rep["angles_for"], dict(pitch=rep["pitch"], yaw=rep["yaw"])).vanishing_point()
            rep["as_recorded"] = dict(pitch=rep["pitch"], yaw=rep["yaw"], intrinsics=dict(rep["angles_for"]))
            rep["pitch"], rep["yaw"] = geom.angles_from_vanishing_point(vp[0], vp[1], new)
            rep["notes"].append("the angles are given for the lens values of --lens (focal %.1f, centre %.1f, %.1f); the "
                                "recording was made with focal %.1f, centre %.1f, %.1f" % (
                                    new["focal_px"], new["cx"], new["cy"], rep["angles_for"]["focal_px"],
                                    rep["angles_for"]["cx"], rep["angles_for"]["cy"]))
        rep["angles_for"] = new
    rep["recommended"] = recommend(rep)
    print_report(rep)
    if a.verbose and rep.get("blocks"):
        for b in rep["blocks"]:
            print("  block t %7.1f s  pitch %+.2f  yaw %+.2f deg" % (b["t_s"], b["pitch"] / DEG, b["yaw"] / DEG))
    if a.out:
        geom.write_json_atomic(a.out, strip(rep))
        print("report: %s" % a.out)
    return 0 if rep["status"] == "ok" else 2


if __name__ == "__main__":
    sys.exit(main())
