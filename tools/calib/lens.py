#!/usr/bin/env python3
"""Lens calibration: focal length, picture centre and distortion from pictures of the calibration board.

    lens.py solve  PICTURES... --board charuco:8x5 [--square-mm 34.5] [--out lens.json] [--debug-dir DIR]
    lens.py impact (--lens lens.json | --focal F [--cx X --cy Y --k1 K ...]) [--size 1280x720] [--calib calib.json]
    lens.py synth  --out DIR [--views 24] ...        pictures of the board through a known camera (rehearsal, tests)

PICTURES: picture files (jpg, png, pgm), folders of them, a recorder session (its frames/), or a video (every
--every-th frame). The board is the one drawn by tools/calib/board.html: `charuco:COLSxROWS` (squares) or
`checker:COLSxROWS`. Needs OpenCV with the aruco module (laptop: /home/piotr/datasets/l33/venv, docs/CALIBRATION.md);
the pictures are taken on the camera host with tools/calib/capture.py.

`solve` reports focal length, picture centre, distortion (k1, k2; OpenCV's model, tools/calib/geom.py), the
reprojection error, how much of the picture the board corners covered, and an uncertainty for every value (the
larger of the fit's own and a bootstrap over the views). It REFUSES (exit 2, `"status": "refused"`, no values to
apply) when the pictures cannot carry a calibration: too few distinct views, no tilted views (focal length and
distance cannot be told apart), too little of the picture covered, a poor fit, or an uncertainty beyond the limits.

`impact` prints how far the distortion moves points inside the road models' input frames, i.e. what roadsense's
pinhole input warp gets wrong while it ignores the distortion.
"""
import argparse
import glob
import json
import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import geom  # noqa: E402

PICTURE_EXT = (".jpg", ".jpeg", ".png", ".pgm", ".ppm", ".bmp")
VIDEO_EXT = (".mjpeg", ".mjpg", ".mp4", ".mkv", ".avi", ".mov", ".webm")
GRID = (8, 6)                      # coverage cells over the picture, columns x rows

LIMITS = dict(
    min_views=10,                  # distinct views
    min_corners=8,                 # board corners in a view
    similar_px=12.0,               # two views whose common corners moved less than this (mean) are the same view
    min_tilted=3,                  # views with the board tilted by at least tilt_deg against the camera
    tilt_deg=15.0,
    min_coverage=0.6,              # fraction of the coverage cells holding a corner
    max_rms_px=1.0,                # reprojection error of the fit
    max_focal_rel=0.015,           # sigma(focal) / focal
    max_centre_rel=0.008,          # sigma(cx), sigma(cy) as a fraction of the focal length (0.008 = 0.46 deg)
)


def cv():
    try:
        import cv2
    except ImportError:
        sys.exit("lens.py needs OpenCV (cv2 with the aruco module): run it with the calibration venv's python, "
                 "see docs/CALIBRATION.md")
    if not hasattr(cv2, "aruco"):
        sys.exit("this OpenCV has no aruco module: install opencv-contrib-python-headless")
    return cv2


class Board:
    def __init__(self, spec, square_mm=1.0):
        try:
            kind, dims = spec.split(":")
            cols, rows = (int(v) for v in dims.lower().split("x"))
        except ValueError:
            raise ValueError("board must look like charuco:8x5 or checker:8x5 (squares, columns x rows)")
        if kind not in ("charuco", "checker") or cols < 3 or rows < 3:
            raise ValueError("board must look like charuco:8x5 or checker:8x5 (squares, columns x rows)")
        self.kind, self.cols, self.rows, self.square = kind, cols, rows, float(square_mm)
        self.spec = "%s:%dx%d" % (kind, cols, rows)
        # inner corners, id = row * (cols - 1) + col, as OpenCV's CharucoBoard numbers them
        gx, gy = np.meshgrid(np.arange(1, cols), np.arange(1, rows))
        self.corners = np.stack([gx.ravel(), gy.ravel(), np.zeros(gx.size)], axis=1).astype(np.float32) * self.square
        self._det = None

    def charuco(self, cv2):
        d = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        return cv2.aruco.CharucoBoard((self.cols, self.rows), 1.0, 0.75, d)

    def image(self, cv2, px_per_square=80):
        """The board as a grey picture (the same drawing as board.html)."""
        px = int(px_per_square) // 8 * 8
        if self.kind == "charuco":
            return self.charuco(cv2).generateImage((self.cols * px, self.rows * px), marginSize=0, borderBits=1)
        r, c = np.indices((self.rows * px, self.cols * px))
        return (((r // px + c // px) % 2) * 255).astype(np.uint8)

    def detect(self, cv2, gray):
        """(ids (N,), pixels (N, 2)) of the board corners found in a grey picture; ids index self.corners."""
        if self.kind == "charuco":
            if self._det is None:
                dp = cv2.aruco.DetectorParameters()
                dp.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
                self._det = cv2.aruco.CharucoDetector(self.charuco(cv2), detectorParams=dp)
            cc, ci, _mc, _mi = self._det.detectBoard(gray)
            if ci is None or len(ci) == 0:
                return np.zeros(0, int), np.zeros((0, 2), np.float32)
            return ci.ravel().astype(int), cc.reshape(-1, 2).astype(np.float32)
        size = (self.cols - 1, self.rows - 1)
        ok, cc = cv2.findChessboardCornersSB(gray, size, flags=cv2.CALIB_CB_NORMALIZE_IMAGE | cv2.CALIB_CB_EXHAUSTIVE
                                             | cv2.CALIB_CB_ACCURACY)
        if not ok:
            return np.zeros(0, int), np.zeros((0, 2), np.float32)
        return np.arange(size[0] * size[1]), cc.reshape(-1, 2).astype(np.float32)


# ---- pictures -------------------------------------------------------------------------------------------------------

def iter_pictures(cv2, inputs, every=10):
    """(name, grey picture) of every input picture / sampled video frame."""
    for p in inputs:
        if os.path.isdir(p):
            d = os.path.join(p, "frames") if os.path.isdir(os.path.join(p, "frames")) else p
            files = sorted(f for f in glob.glob(os.path.join(d, "*")) if f.lower().endswith(PICTURE_EXT))
            if not files:
                print("lens: no pictures in %s" % d, file=sys.stderr)
            for f in files:
                img = cv2.imread(f, cv2.IMREAD_GRAYSCALE)
                if img is not None:
                    yield f, img
        elif p.lower().endswith(VIDEO_EXT):
            cap = cv2.VideoCapture(p)
            n = 0
            while True:
                if not cap.grab():
                    break
                if n % max(1, every) == 0:
                    ok, img = cap.retrieve()
                    if ok:
                        yield "%s#%d" % (p, n), cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
                n += 1
            cap.release()
        else:
            img = cv2.imread(p, cv2.IMREAD_GRAYSCALE)
            if img is None:
                print("lens: cannot read %s" % p, file=sys.stderr)
                continue
            yield p, img


def well_spread(board, ids):
    """The corners of a view must span two directions of the board (a single row gives no pose)."""
    if len(ids) < 4:
        return False
    p = board.corners[ids][:, :2]
    s = np.linalg.svd(p - p.mean(axis=0), compute_uv=False)
    return s[1] > 0.2 * board.square


def distinct(views, similar_px):
    """Drop views that repeat an earlier one: the corners they share moved by less than similar_px on average."""
    kept, dropped = [], []
    for v in views:
        same = False
        for k in kept:
            common, ia, ib = np.intersect1d(v["ids"], k["ids"], return_indices=True)
            if len(common) >= 0.5 * min(len(v["ids"]), len(k["ids"])) and len(common) >= 4:
                if float(np.mean(np.linalg.norm(v["px"][ia] - k["px"][ib], axis=1))) < similar_px:
                    same = True
                    break
        (dropped if same else kept).append(v)
    return kept, dropped


# ---- the fit --------------------------------------------------------------------------------------------------------

def _calibrate(cv2, board, views, size, full=False, guess=None):
    obj = [board.corners[v["ids"]].reshape(-1, 1, 3) for v in views]
    img = [v["px"].reshape(-1, 1, 2).astype(np.float32) for v in views]
    flags = cv2.CALIB_FIX_K3
    k0 = np.array([[1.0, 0, size[0] / 2.0], [0, 1.0, size[1] / 2.0], [0, 0, 1]])
    if not full:
        flags |= cv2.CALIB_FIX_ASPECT_RATIO | cv2.CALIB_ZERO_TANGENT_DIST
    d0 = np.zeros(5)
    if guess is not None:
        k0, d0 = guess[0].copy(), guess[1].copy()
        flags |= cv2.CALIB_USE_INTRINSIC_GUESS
    rms, k, d, rv, tv, s_in, _s_ex, per = cv2.calibrateCameraExtended(obj, img, size, k0, d0, flags=flags)
    return dict(rms=float(rms), k=k, d=d.ravel()[:5], rv=rv, tv=tv, s_in=s_in.ravel(), per=per.ravel())


def _params(fit):
    k, d = fit["k"], fit["d"]
    return dict(focal_px=float(k[0, 0]), fy=float(k[1, 1]), cx=float(k[0, 2]), cy=float(k[1, 2]), k1=float(d[0]),
                k2=float(d[1]), p1=float(d[2]), p2=float(d[3]), k3=float(d[4]))


def coverage(views, size):
    grid = np.zeros((GRID[1], GRID[0]), int)
    w, h = size
    rmax = 0.0
    for v in views:
        gx = np.clip((v["px"][:, 0] / w * GRID[0]).astype(int), 0, GRID[0] - 1)
        gy = np.clip((v["px"][:, 1] / h * GRID[1]).astype(int), 0, GRID[1] - 1)
        np.add.at(grid, (gy, gx), 1)
        rmax = max(rmax, float(np.max(np.hypot(v["px"][:, 0] - w / 2.0, v["px"][:, 1] - h / 2.0))))
    corners = {"top_left": int(grid[0, 0]), "top_right": int(grid[0, -1]), "bottom_left": int(grid[-1, 0]),
               "bottom_right": int(grid[-1, -1])}
    return dict(fraction=float(np.mean(grid > 0)), grid=grid.tolist(), corners=corners,
                radius_fraction=rmax / math.hypot(w / 2.0, h / 2.0))


def solve_views(cv2, board, views, size, limits=None, bootstrap=60, seed=1):
    """Fit the camera to detected views [{name, ids, px}]. Returns the report dict (`status` ok | refused)."""
    lim = dict(LIMITS, **(limits or {}))
    rep = dict(status="refused", reasons=[], warnings=[], board=board.spec, square_mm=board.square,
               image=dict(width=size[0], height=size[1]), views_given=len(views), limits=lim)
    views = [v for v in views if len(v["ids"]) >= lim["min_corners"] and well_spread(board, v["ids"])]
    rep["views_detected"] = len(views)
    views, same = distinct(views, lim["similar_px"])
    rep["views_distinct"], rep["views_repeated"] = len(views), len(same)
    if len(views) < lim["min_views"]:
        rep["reasons"].append("only %d distinct views with the board found (%d pictures given, %d repeat another "
                              "view); at least %d are needed: move and tilt the board between pictures"
                              % (len(views), rep["views_given"], len(same), lim["min_views"]))
        return rep

    fit = _calibrate(cv2, board, views, size)
    for _ in range(2):                                   # views that do not fit (smeared, board bent, wrong ids)
        per = fit["per"]
        bad = per > max(3.0 * float(np.median(per)), 1.0)
        if not bad.any() or (~bad).sum() < lim["min_views"]:
            break
        rep.setdefault("views_rejected", []).extend(
            dict(name=v["name"], rms_px=round(float(e), 2)) for v, e, b in zip(views, per, bad) if b)
        views = [v for v, b in zip(views, bad) if not b]
        fit = _calibrate(cv2, board, views, size)
    p = _params(fit)
    rep["views_used"] = len(views)
    rep["rms_px"] = round(fit["rms"], 3)

    # every view's pose: tilt of the board against the camera axis, distance
    tilts, vinfo = [], []
    for v, rv, tv, e in zip(views, fit["rv"], fit["tv"], fit["per"]):
        r, _ = cv2.Rodrigues(rv)
        tilt = math.degrees(math.acos(min(1.0, abs(float(r[2, 2])))))
        tilts.append(tilt)
        vinfo.append(dict(name=v["name"], corners=int(len(v["ids"])), rms_px=round(float(e), 3),
                          tilt_deg=round(tilt, 1), distance=round(float(np.linalg.norm(tv)), 1)))
    rep["views"] = vinfo
    n_tilted = int(sum(t >= lim["tilt_deg"] for t in tilts))
    rep["tilted_views"] = n_tilted

    # uncertainty: the fit's own (assumes independent corner noise: optimistic) and a bootstrap over the views
    formal = dict(focal_px=fit["s_in"][0], cx=fit["s_in"][2], cy=fit["s_in"][3], k1=fit["s_in"][4], k2=fit["s_in"][5])
    rng = np.random.default_rng(seed)
    boot = []
    for _ in range(bootstrap):
        idx = rng.integers(0, len(views), len(views))
        if len(set(idx.tolist())) < 4:
            continue
        try:
            b = _params(_calibrate(cv2, board, [views[i] for i in idx], size, guess=(fit["k"], fit["d"])))
        except cv2.error:
            continue
        boot.append([b[k] for k in ("focal_px", "cx", "cy", "k1", "k2")])
    bs = np.std(np.array(boot), axis=0) if len(boot) >= 10 else np.full(5, np.nan)
    sigma = {}
    for i, k in enumerate(("focal_px", "cx", "cy", "k1", "k2")):
        sigma[k] = float(np.nanmax([formal[k], bs[i]]))
    rep["sigma"] = {k: round(v, 5 if k[0] == "k" else 3) for k, v in sigma.items()}
    rep["sigma_fit"] = {k: round(float(v), 5 if k[0] == "k" else 3) for k, v in formal.items()}
    rep["sigma_bootstrap"] = {k: (None if not np.isfinite(bs[i]) else round(float(bs[i]), 5 if k[0] == "k" else 3))
                              for i, k in enumerate(("focal_px", "cx", "cy", "k1", "k2"))}

    cov = coverage(views, size)
    rep["coverage"] = cov

    # a freer model as a check: separate fx / fy and tangential terms
    try:
        full = _calibrate(cv2, board, views, size, full=True)
        fp = _params(full)
        rep["check_free_model"] = dict(rms_px=round(full["rms"], 3), fx=round(fp["focal_px"], 2), fy=round(fp["fy"], 2),
                                       cx=round(fp["cx"], 2), cy=round(fp["cy"], 2), k1=round(fp["k1"], 5),
                                       k2=round(fp["k2"], 5), p1=round(fp["p1"], 5), p2=round(fp["p2"], 5))
        asp = fp["fy"] / fp["focal_px"] - 1.0
        asp_sigma = math.hypot(full["s_in"][0], full["s_in"][1]) / fp["focal_px"]
        if abs(asp) > max(3.0 * asp_sigma, 0.004):
            rep["warnings"].append("the pixels do not look square: fy / fx = %.4f (roadsense has one focal length; "
                                   "the single-focal fit is used)" % (1.0 + asp))
        if full["rms"] < 0.8 * fit["rms"]:
            rep["warnings"].append("a model with tangential distortion fits clearly better (rms %.3f against %.3f px): "
                                   "the lens may sit tilted on the sensor" % (full["rms"], fit["rms"]))
    except cv2.error:
        pass

    intr = dict(focal_px=p["focal_px"], cx=p["cx"], cy=p["cy"], k1=p["k1"], k2=p["k2"], p1=0.0, p2=0.0, k3=0.0)
    errors, warns = geom.check_intrinsics(intr, size[0], size[1])
    rep["warnings"].extend(warns)
    reasons = rep["reasons"]
    reasons.extend(errors)
    if n_tilted < lim["min_tilted"]:
        reasons.append("only %d views show the board tilted by %.0f deg or more (need %d): with the board always "
                       "facing the camera, focal length and distance cannot be told apart"
                       % (n_tilted, lim["tilt_deg"], lim["min_tilted"]))
    if cov["fraction"] < lim["min_coverage"]:
        reasons.append("the board corners covered %.0f %% of the picture (need %.0f %%): show the board in the "
                       "empty areas too" % (100 * cov["fraction"], 100 * lim["min_coverage"]))
    if fit["rms"] > lim["max_rms_px"]:
        reasons.append("reprojection error %.2f px is above %.2f px: smeared or bent board, or the wrong --board"
                       % (fit["rms"], lim["max_rms_px"]))
    elif fit["rms"] > 0.5 * lim["max_rms_px"]:
        rep["warnings"].append("reprojection error %.2f px is high for a still, flat board" % fit["rms"])
    if sigma["focal_px"] / p["focal_px"] > lim["max_focal_rel"]:
        reasons.append("focal length %.1f +- %.1f px is uncertain by more than %.1f %%: more views, more tilt, the "
                       "board larger in the picture" % (p["focal_px"], sigma["focal_px"], 100 * lim["max_focal_rel"]))
    for k in ("cx", "cy"):
        if sigma[k] / p["focal_px"] > lim["max_centre_rel"]:
            reasons.append("%s %.1f +- %.1f px is uncertain by more than %.2f deg: more tilted views in both "
                           "directions" % (k, p[k], sigma[k], math.degrees(lim["max_centre_rel"])))
    empty = [k.replace("_", " ") for k, n in cov["corners"].items() if n == 0]
    if empty:
        rep["warnings"].append("no board corner in the picture's %s corner%s: the distortion there is extrapolated "
                               "(points reached %.0f %% of the way to the picture's corners)"
                               % (", ".join(empty), "" if len(empty) == 1 else "s", 100 * cov["radius_fraction"]))

    values = dict(focal_px=round(p["focal_px"], 2), cx=round(p["cx"], 2), cy=round(p["cy"], 2), k1=round(p["k1"], 5),
                  k2=round(p["k2"], 5), p1=0.0, p2=0.0, k3=0.0)
    rep["hfov_deg"] = round(math.degrees(2 * math.atan(size[0] / 2.0 / p["focal_px"])), 1)
    if reasons:
        rep["measured_not_usable"] = values        # for the record; apply.py does not take it
    else:
        rep["status"] = "ok"
        rep["calibration"] = values
        del rep["reasons"]
    return rep


def detect_all(cv2, board, inputs, every=10, debug_dir=None, log=None):
    views, size, n = [], None, 0
    for name, gray in iter_pictures(cv2, inputs, every):
        n += 1
        sz = (gray.shape[1], gray.shape[0])
        if size is None:
            size = sz
        elif sz != size:
            print("lens: %s is %dx%d, the first picture %dx%d: skipped" % (name, sz[0], sz[1], size[0], size[1]),
                  file=sys.stderr)
            continue
        ids, px = board.detect(cv2, gray)
        if log:
            log("%-60s %3d corners" % (name[-60:], len(ids)))
        if len(ids):
            views.append(dict(name=name, ids=ids, px=px))
            if debug_dir:
                vis = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
                for (x, y) in px:
                    cv2.circle(vis, (int(round(x)), int(round(y))), 4, (0, 220, 0), 1)
                cv2.imwrite(os.path.join(debug_dir, "det-%04d.jpg" % n), vis)
    return views, size, n


def draw_coverage(cv2, views, size, rep, path):
    w, h = size
    img = np.full((h, w, 3), 255, np.uint8)
    grid = np.array(rep["coverage"]["grid"])
    for gy in range(GRID[1]):
        for gx in range(GRID[0]):
            x0, y0, x1, y1 = gx * w // GRID[0], gy * h // GRID[1], (gx + 1) * w // GRID[0], (gy + 1) * h // GRID[1]
            if grid[gy, gx] == 0:
                cv2.rectangle(img, (x0, y0), (x1, y1), (200, 200, 255), -1)
            cv2.rectangle(img, (x0, y0), (x1, y1), (170, 170, 170), 1)
    for v in views:
        for (x, y) in v["px"]:
            cv2.circle(img, (int(round(x)), int(round(y))), 2, (160, 90, 0), -1)
    cv2.putText(img, "board corners used; red cells are empty (%.0f %% covered)" % (100 * rep["coverage"]["fraction"]),
                (12, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 1, cv2.LINE_AA)
    cv2.imwrite(path, img)


def print_report(rep, out=sys.stdout):
    w = out.write
    w("board %s, pictures %d, with the board %d, distinct %d, used %s\n" % (
        rep["board"], rep["views_given"], rep.get("views_detected", 0), rep.get("views_distinct", 0),
        rep.get("views_used", "-")))
    vals = rep.get("calibration") or rep.get("measured_not_usable")
    if vals:
        s = rep["sigma"]
        w("  focal length  %8.2f +- %.2f px   (horizontal field of view %.1f deg)\n" % (vals["focal_px"], s["focal_px"],
                                                                                    rep["hfov_deg"]))
        w("  picture centre %7.2f +- %.2f, %.2f +- %.2f px   (picture %dx%d, middle %.0f, %.0f)\n" % (
            vals["cx"], s["cx"], vals["cy"], s["cy"], rep["image"]["width"], rep["image"]["height"],
            rep["image"]["width"] / 2, rep["image"]["height"] / 2))
        w("  distortion    k1 %+.4f +- %.4f   k2 %+.4f +- %.4f\n" % (vals["k1"], s["k1"], vals["k2"], s["k2"]))
        w("  reprojection error %.3f px rms over %d views; %d views tilted >= %.0f deg\n" % (
            rep["rms_px"], rep["views_used"], rep["tilted_views"], rep["limits"]["tilt_deg"]))
        c = rep["coverage"]
        w("  coverage %.0f %% of the picture, out to %.0f %% of the corner radius; corners: %s\n" % (
            100 * c["fraction"], 100 * c["radius_fraction"],
            ", ".join("%s %s" % (k.replace("_", " "), "yes" if n else "NO") for k, n in c["corners"].items())))
        for row in c["grid"]:
            w("    " + " ".join("%4d" % n if n else "   ." for n in row) + "\n")
        f = rep.get("check_free_model")
        if f:
            w("  check, free model: fx %.2f fy %.2f centre %.2f, %.2f p1 %+.5f p2 %+.5f rms %.3f px\n" % (
                f["fx"], f["fy"], f["cx"], f["cy"], f["p1"], f["p2"], f["rms_px"]))
    for t in rep.get("warnings", []):
        w("  WARNING: %s\n" % t)
    if rep["status"] == "ok":
        w("OK: calibration usable\n")
    else:
        for t in rep["reasons"]:
            w("  REFUSED: %s\n" % t)
        w("REFUSED: no calibration written to apply\n")


def cmd_solve(a):
    cv2 = cv()
    board = Board(a.board, a.square_mm)
    if a.debug_dir:
        os.makedirs(a.debug_dir, exist_ok=True)
    views, size, n = detect_all(cv2, board, a.pictures, a.every, a.debug_dir,
                                log=(lambda s: print(s, file=sys.stderr)) if a.verbose else None)
    if size is None:
        sys.exit("lens: no pictures read")
    limits = {"min_views": a.min_views} if a.min_views else None
    rep = solve_views(cv2, board, views, size, limits=limits, bootstrap=a.bootstrap)
    rep["views_given"] = n
    rep["inputs"] = [os.path.abspath(p) for p in a.pictures]
    print_report(rep)
    if a.out:
        geom.write_json_atomic(a.out, rep)
        print("report: %s" % a.out)
    if a.debug_dir and "coverage" in rep:
        used = {v["name"] for v in rep.get("views", [])}
        draw_coverage(cv2, [v for v in views if v["name"] in used], size, rep, os.path.join(a.debug_dir, "coverage.png"))
        print("coverage picture: %s" % os.path.join(a.debug_dir, "coverage.png"))
    return 0 if rep["status"] == "ok" else 2


# ---- impact of the distortion on the model inputs --------------------------------------------------------------------

def impact(intr, size, calib=None):
    """How far the distortion moves points of the road models' input frames. Returns rows for a table."""
    w, h = size
    out = {}
    for frame in ("medmodel", "sbigmodel"):
        mf = geom.MODEL_FRAMES[frame][0]
        mp, pin, real = geom.model_region(frame, intr, calib, n=33)
        inside = (pin[:, 0] >= 0) & (pin[:, 0] <= w - 1) & (pin[:, 1] >= 0) & (pin[:, 1] <= h - 1)
        d_cam = np.linalg.norm(real - pin, axis=1)
        scale = mf / intr["focal_px"]                         # camera pixel -> model pixel
        # the best a single pinhole focal length could do: scale the picture about the centre to fit this frame
        x = pin[inside] - [intr["cx"], intr["cy"]]
        y = real[inside] - [intr["cx"], intr["cy"]]
        s = float(np.sum(x * y) / np.sum(x * x))
        res = np.linalg.norm(y - s * x, axis=1)
        mcx, mcy = geom.MODEL_FRAMES[frame][1], geom.MODEL_FRAMES[frame][2]
        centre = np.hypot(mp[:, 0] - mcx, mp[:, 1] - mcy) < 0.1 * 512       # around the vanishing point
        out[frame] = dict(
            inside_fraction=float(np.mean(inside)),
            cam_px_max=float(d_cam[inside].max()), cam_px_rms=float(np.sqrt(np.mean(d_cam[inside] ** 2))),
            model_px_max=float(d_cam[inside].max() * scale), model_px_rms=float(np.sqrt(np.mean(d_cam[inside] ** 2)) * scale),
            model_px_near_vp=float(d_cam[inside & centre].max() * scale) if (inside & centre).any() else None,
            best_pinhole_focal=float(s * intr["focal_px"]), best_pinhole_model_px_max=float(res.max() * scale),
            best_pinhole_model_px_rms=float(np.sqrt(np.mean(res ** 2)) * scale))
    return out


def cmd_impact(a):
    if a.lens:
        rep = geom.load_json(a.lens)
        intr = rep.get("calibration") or rep.get("measured_not_usable") or rep
        size = (rep.get("image", {}).get("width", 1280), rep.get("image", {}).get("height", 720))
    else:
        w, h = (int(v) for v in a.size.lower().split("x"))
        size = (w, h)
        intr = dict(focal_px=a.focal, cx=a.cx if a.cx is not None else w / 2.0, cy=a.cy if a.cy is not None else h / 2.0,
                    k1=a.k1, k2=a.k2, p1=0.0, p2=0.0, k3=a.k3)
    calib = geom.load_json(a.calib) if a.calib else None
    r = impact(intr, size, calib)
    print("lens: focal %.1f centre %.1f, %.1f k1 %+.4f k2 %+.4f, picture %dx%d, mount %s" % (
        intr["focal_px"], intr["cx"], intr["cy"], intr.get("k1", 0), intr.get("k2", 0), size[0], size[1],
        "pitch %.4f yaw %.4f" % (calib.get("pitch", 0), calib.get("yaw", 0)) if calib else "level"))
    print("how far the distortion moves a point of the model's input frame while the warp assumes a pinhole:")
    print("%-10s %7s | %-21s | %-21s | %-9s | %s" % ("frame", "inside", "camera px  max / rms", "model px   max / rms",
                                                     "near VP", "best single focal: px, left over max / rms (model px)"))
    for frame, v in r.items():
        print("%-10s %6.0f%% | %9.1f / %-9.1f | %9.1f / %-9.1f | %9s | %7.1f, %5.1f / %.1f" % (
            frame, 100 * v["inside_fraction"], v["cam_px_max"], v["cam_px_rms"], v["model_px_max"], v["model_px_rms"],
            "%.2f" % v["model_px_near_vp"] if v["model_px_near_vp"] is not None else "-",
            v["best_pinhole_focal"], v["best_pinhole_model_px_max"], v["best_pinhole_model_px_rms"]))
    print("(inside = part of the frame that lies inside the camera picture; near VP = within 51 model px of the "
          "vanishing point, where far vehicles are; model frames are 512x256)")
    if a.json:
        geom.write_json_atomic(a.json, dict(intrinsics=intr, size=size, impact=r))
    return 0


# ---- synthetic pictures ---------------------------------------------------------------------------------------------

def render_view(cv2, board, intr, rvec, tvec, size, px_per_square=64, blur=0.8, noise=1.5, rng=None):
    """A picture of the board (pose rvec, tvec in board units) through the camera `intr` (with its distortion)."""
    w, h = size
    tex = board.image(cv2, px_per_square).astype(np.float32)
    k = np.array([[intr["focal_px"], 0, intr["cx"]], [0, intr["focal_px"], intr["cy"]], [0, 0, 1.0]])
    d = np.array([intr.get(q, 0.0) for q in geom.DIST_KEYS])
    gx, gy = np.meshgrid(np.arange(w, dtype=np.float32), np.arange(h, dtype=np.float32))
    pts = np.stack([gx.ravel(), gy.ravel()], axis=1).reshape(-1, 1, 2)
    nrm = cv2.undistortPoints(pts, k, d).reshape(-1, 2)
    rays = np.concatenate([nrm, np.ones((len(nrm), 1), np.float32)], axis=1)
    r, _ = cv2.Rodrigues(np.asarray(rvec, float))
    t = np.asarray(tvec, float).reshape(3)
    rr = rays @ r                                           # R^T ray, row vectors
    rt = r.T @ t
    s = rt[2] / np.where(np.abs(rr[:, 2]) < 1e-9, 1e-9, rr[:, 2])
    xb = s[:, None] * rr - rt                               # point on the board plane, board units
    scale = tex.shape[1] / (board.cols * board.square)
    mx = (xb[:, 0] * scale - 0.5).reshape(h, w).astype(np.float32)
    my = (xb[:, 1] * scale - 0.5).reshape(h, w).astype(np.float32)
    bad = (s <= 0).reshape(h, w)
    mx[bad] = -10
    # a white quiet zone of one square around the board, grey elsewhere
    pad = int(tex.shape[1] / board.cols)
    texp = cv2.copyMakeBorder(tex, pad, pad, pad, pad, cv2.BORDER_CONSTANT, value=255)
    img = cv2.remap(texp, mx + pad, my + pad, cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT, borderValue=110)
    img = img * 0.8 + 20
    if blur > 0:
        img = cv2.GaussianBlur(img, (0, 0), blur)
    if noise > 0:
        img = img + (rng or np.random.default_rng(0)).normal(0, noise, img.shape)
    return np.clip(img, 0, 255).astype(np.uint8)


def synth_poses(board, intr, size, n, rng, tilt_max_deg=40.0, fill=(0.3, 0.6)):
    """n board poses spread over the picture, facing the camera within tilt_max_deg."""
    import cv2
    w, h = size
    bw, bh = board.cols * board.square, board.rows * board.square
    poses = []
    for i in range(n):
        z = bw * intr["focal_px"] / (w * rng.uniform(*fill))           # the board spans `fill` of the picture width
        u, v = rng.uniform(0.12, 0.88) * w, rng.uniform(0.15, 0.85) * h
        c = np.array([(u - intr["cx"]) / intr["focal_px"] * z, (v - intr["cy"]) / intr["focal_px"] * z, z])
        ax, ay = np.radians(rng.uniform(-tilt_max_deg, tilt_max_deg, 2)) if i % 4 else (0.0, 0.0)
        az = math.radians(rng.uniform(-10, 10))
        r = (cv2.Rodrigues(np.array([0, 0, az]))[0] @ cv2.Rodrigues(np.array([ax, 0, 0]))[0]
             @ cv2.Rodrigues(np.array([0, ay, 0]))[0])
        t = c - r @ np.array([bw / 2, bh / 2, 0])
        poses.append((cv2.Rodrigues(r)[0].ravel(), t))
    return poses


def synth_views(board, intr, size, poses, noise_px=0.15, rng=None, min_corners=8):
    """Detected views computed, not rendered: the board corners projected through `intr` plus pixel noise."""
    import cv2
    rng = rng or np.random.default_rng(0)
    k = np.array([[intr["focal_px"], 0, intr["cx"]], [0, intr["focal_px"], intr["cy"]], [0, 0, 1.0]])
    d = np.array([intr.get(q, 0.0) for q in geom.DIST_KEYS])
    views = []
    for i, (rv, tv) in enumerate(poses):
        px, _ = cv2.projectPoints(board.corners.reshape(-1, 1, 3).astype(np.float64), rv, tv, k, d)
        px = px.reshape(-1, 2) + rng.normal(0, noise_px, (len(board.corners), 2))
        ok = (px[:, 0] > 2) & (px[:, 0] < size[0] - 3) & (px[:, 1] > 2) & (px[:, 1] < size[1] - 3)
        if ok.sum() >= min_corners:
            views.append(dict(name="synth-%03d" % i, ids=np.nonzero(ok)[0], px=px[ok].astype(np.float32)))
    return views


def cmd_synth(a):
    cv2 = cv()
    board = Board(a.board, a.square_mm)
    w, h = (int(v) for v in a.size.lower().split("x"))
    intr = dict(focal_px=a.focal, cx=a.cx if a.cx is not None else w / 2.0, cy=a.cy if a.cy is not None else h / 2.0,
                k1=a.k1, k2=a.k2, p1=0.0, p2=0.0, k3=a.k3)
    rng = np.random.default_rng(a.seed)
    os.makedirs(a.out, exist_ok=True)
    for i, (rv, tv) in enumerate(synth_poses(board, intr, (w, h), a.views, rng)):
        cv2.imwrite(os.path.join(a.out, "synth-%03d.png" % i), render_view(cv2, board, intr, rv, tv, (w, h), rng=rng))
    geom.write_json_atomic(os.path.join(a.out, "truth.json"), intr)
    print("%d pictures in %s (truth.json holds the camera they were drawn with)" % (a.views, a.out))
    return 0


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    sub = ap.add_subparsers(dest="cmd", required=True)
    s = sub.add_parser("solve", help="pictures of the board -> lens.json")
    s.add_argument("pictures", nargs="+")
    s.add_argument("--board", required=True, help="charuco:COLSxROWS or checker:COLSxROWS, as board.html shows it")
    s.add_argument("--square-mm", type=float, default=1.0, help="side of one square (only scales the distances)")
    s.add_argument("--out", help="report (JSON); apply.py takes it when its status is ok")
    s.add_argument("--every", type=int, default=10, help="video input: use every N-th frame (default 10)")
    s.add_argument("--min-views", type=int, default=0, help="raise the least number of distinct views (default %d)"
                   % LIMITS["min_views"])
    s.add_argument("--bootstrap", type=int, default=60)
    s.add_argument("--debug-dir", help="write the detections and a coverage picture here")
    s.add_argument("--verbose", action="store_true")
    s.set_defaults(fn=cmd_solve)

    def lens_args(p, required):
        p.add_argument("--focal", type=float, required=required)
        p.add_argument("--cx", type=float)
        p.add_argument("--cy", type=float)
        p.add_argument("--k1", type=float, default=0.0)
        p.add_argument("--k2", type=float, default=0.0)
        p.add_argument("--k3", type=float, default=0.0)
        p.add_argument("--size", default="1280x720")

    s = sub.add_parser("impact", help="how far the distortion moves points in the road models' input frames")
    s.add_argument("--lens", help="lens.json from solve")
    lens_args(s, False)
    s.add_argument("--calib", help="calib.json (mount angles; default level)")
    s.add_argument("--json")
    s.set_defaults(fn=cmd_impact)

    s = sub.add_parser("synth", help="pictures of the board through a known camera")
    s.add_argument("--out", required=True)
    s.add_argument("--board", default="charuco:8x5")
    s.add_argument("--square-mm", type=float, default=30.0)
    s.add_argument("--views", type=int, default=24)
    s.add_argument("--seed", type=int, default=1)
    lens_args(s, True)
    s.set_defaults(fn=cmd_synth)
    a = ap.parse_args(argv)
    if a.cmd == "impact" and not a.lens and a.focal is None:
        ap.error("impact needs --lens or --focal")
    return a.fn(a)


if __name__ == "__main__":
    sys.exit(main())
