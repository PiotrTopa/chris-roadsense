#!/usr/bin/env python3
"""Apply a camera calibration and draw the picture that shows whether it is right.

    apply.py --config-dir /etc/roadsense --camera-file camera-dvr.json
             [--lens lens.json] [--mount mount.json] [--mount-source pose|pictures]
             [--pitch RAD --yaw RAD --roll RAD]
             [--height-m H] [--camera-to-front-m X] [--lateral-offset-m Y]
             (--frame PICTURE | --session RECORDING) [--view OUT.jpg] [--dry-run]
    apply.py --config-dir /etc/roadsense --rollback [STAMP]
    apply.py --config-dir /etc/roadsense --camera-file camera-dvr.json --frame PICTURE --view-only

What goes where (docs/CALIBRATION.md, docs/ARCHITECTURE.md):
  <camera file>   focal_px, cx, cy       read by the camera daemon at its start (frame ring header) -> restart it
                  k1, k2, p1, p2, k3     the lens distortion, recorded; no daemon reads it yet
  calib.json      pitch, yaw, roll (rad), height_m     re-read by modeld and laned within 2 s
                  camera_to_front_m, lateral_offset_m  tape measurements, recorded here (lens to front bumper;
                                                       lens right of the car's centre line, m)
  lead.json       camera_to_front_m      the lead tracker's `gap_m` (api; read at its start)

Nothing is overwritten without a copy: the files as they were go to <config-dir>/calibration-backups/<stamp>/
first, and `--rollback` puts the newest (or the named) copy back, after saving the current files the same way.
A lens.json / mount.json whose status is not "ok" is not applied, and values that are not plausible stop the run.

The view (default <config-dir>/calibration-backups/<stamp>/view.jpg, or --view): on a picture of the road it
draws the horizon, the road's vanishing point, a ground grid at 10, 20 and 50 m with a 3.5 m lane, and the
outline of the road model's input. On a straight, level road the vanishing point must sit where the lane lines
meet, the horizon on the real horizon, and the lane's lines along the painted ones.
Needs numpy and Pillow (what the api needs); reading a session's straight stretches needs nothing more.
"""
import argparse
import datetime
import math
import os
import shutil
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import geom  # noqa: E402

BACKUP_DIR = "calibration-backups"
GRID_X = (10.0, 20.0, 50.0)
LANE_HALF = 1.75
DEG = math.pi / 180.0


def stamp_now():
    return datetime.datetime.now().strftime("%Y%m%d-%H%M%S")


def picture_size(path):
    from PIL import Image
    with Image.open(path) as im:
        return im.size


# ---- the view --------------------------------------------------------------------------------------------------------

def _font(size):
    from PIL import ImageFont
    try:
        return ImageFont.load_default(size=size)
    except TypeError:
        return ImageFont.load_default()


def _polyline(draw, cam, pts, scale, colour, width):
    uv = cam.project(pts) * scale
    run = []
    for u, v in uv:
        if not (math.isfinite(u) and math.isfinite(v)) or abs(u) > 1e5 or abs(v) > 1e5:
            if len(run) > 1:
                draw.line(run, fill=colour, width=width)
            run = []
        else:
            run.append((float(u), float(v)))
    if len(run) > 1:
        draw.line(run, fill=colour, width=width)


def draw_view(img, intr, calib, size, title=None, tape=None):
    """Draw the calibration onto a PIL picture taken by the camera (any scale of the camera's size)."""
    from PIL import ImageDraw
    img = img.convert("RGB")
    d = ImageDraw.Draw(img, "RGBA")
    sc = img.size[0] / float(size[0])
    cam = geom.Camera(intr, calib)
    h = cam.height
    lw = max(1, int(round(2 * sc)))
    # horizon: directions in the road plane
    az = np.radians(np.linspace(-60, 60, 121))
    _polyline(d, cam, np.stack([np.cos(az), np.sin(az), np.zeros_like(az)], axis=1) * 1e5, sc, (255, 220, 0, 230), lw)
    # ground grid
    xs = np.concatenate([np.linspace(4, 20, 33), np.linspace(21, 120, 100)])
    for y, col in ((-LANE_HALF, (0, 255, 120, 230)), (LANE_HALF, (0, 255, 120, 230)), (0.0, (0, 255, 120, 110))):
        _polyline(d, cam, np.stack([xs, np.full_like(xs, y), np.full_like(xs, h)], axis=1), sc, col, lw)
    ys = np.linspace(-6, 6, 25)
    f = _font(max(11, int(16 * sc)))
    for x in GRID_X:
        _polyline(d, cam, np.stack([np.full_like(ys, x), ys, np.full_like(ys, h)], axis=1), sc, (0, 200, 255, 230), lw)
        p = cam.project([[x, 6.0, h]])[0] * sc
        if np.all(np.isfinite(p)):
            d.text((float(p[0]) + 4, float(p[1]) - 8 * sc), "%.0f m" % x, fill=(0, 200, 255, 255), font=f)
    front = (tape or {}).get("camera_to_front_m")
    if front:
        for x in GRID_X:                                  # the same distances measured from the front bumper
            _polyline(d, cam, np.stack([np.full(2, x + front), [-0.5, 0.5], np.full(2, h)], axis=1), sc,
                      (255, 255, 255, 230), lw)
    # the road model's input frame (the picture area the model looks at)
    _mp, pin, _real = geom.model_region("medmodel", intr, calib, n=2)
    q = [tuple(float(v) for v in pin[i] * sc) for i in (0, 1, 3, 2, 0)]
    if all(math.isfinite(v) for pt in q for v in pt):
        d.line(q, fill=(255, 255, 255, 120), width=1)
    # vanishing point and the picture centre
    u, v = (float(t) for t in cam.vanishing_point() * sc)
    r = 14 * sc
    d.line([(u - r, v), (u + r, v)], fill=(255, 40, 40, 255), width=lw)
    d.line([(u, v - r), (u, v + r)], fill=(255, 40, 40, 255), width=lw)
    d.ellipse([u - r / 2, v - r / 2, u + r / 2, v + r / 2], outline=(255, 40, 40, 255), width=lw)
    cx, cy = intr["cx"] * sc, intr["cy"] * sc
    d.line([(cx - 6 * sc, cy - 6 * sc), (cx + 6 * sc, cy + 6 * sc)], fill=(255, 255, 255, 200), width=1)
    d.line([(cx - 6 * sc, cy + 6 * sc), (cx + 6 * sc, cy - 6 * sc)], fill=(255, 255, 255, 200), width=1)
    lines = []
    if title:
        lines.append(title)
    lines.append("focal %.1f px  centre %.1f, %.1f%s" % (
        intr["focal_px"], intr["cx"], intr["cy"],
        "  k1 %+.3f k2 %+.3f" % (intr.get("k1", 0), intr.get("k2", 0)) if intr.get("k1") or intr.get("k2") else ""))
    lines.append("pitch %+.2f  yaw %+.2f  roll %+.2f deg  height %.2f m" % (
        cam.pitch / DEG, cam.yaw / DEG, cam.roll / DEG, cam.height))
    lines.append("yellow: horizon   red: where a straight road must vanish   green: 3.5 m lane   blue: 10 / 20 / 50 m")
    fs = max(11, int(15 * sc))
    f2 = _font(fs)
    d.rectangle([0, 0, img.size[0], (fs + 4) * len(lines) + 6], fill=(0, 0, 0, 150))
    for i, t in enumerate(lines):
        d.text((8, 4 + i * (fs + 4)), t, fill=(255, 255, 255, 255), font=f2)
    return img


def session_frames(session, n=4):
    """Pictures of a recorder session taken on straight, steady stretches (up to n, spread over the drive)."""
    import mount
    s, _calibs, _model = mount.read_samples(session)
    if len(s) == 0:
        files = mount.frame_files(session, None)
    else:
        files = mount.frame_files(session, s[mount.select(s, mount.DEFAULTS), 1].astype(np.int64))
        if not files:
            files = mount.frame_files(session, None)
    if not files:
        return []
    pick = sorted(set(np.linspace(0, len(files) - 1, min(n, len(files))).astype(int).tolist()))
    return [files[i][1] for i in pick]


def make_view(frames, intr, calib, out, title=None, tape=None):
    from PIL import Image
    tiles, size = [], None
    for p in frames:
        im = Image.open(p)
        size = size or im.size
        tiles.append(draw_view(im, intr, calib, size, title=title if not tiles else os.path.basename(p), tape=tape))
    if len(tiles) == 1:
        sheet = tiles[0]
    else:
        w, h = tiles[0].size
        cols = 2
        rows = (len(tiles) + 1) // 2
        sheet = Image.new("RGB", (w * cols, h * rows))
        for i, t in enumerate(tiles):
            sheet.paste(t.resize((w, h)), ((i % cols) * w, (i // cols) * h))
    sheet.save(out, quality=90)
    return size


# ---- the files -------------------------------------------------------------------------------------------------------

def plan(a, cur_calib, cur_cam):
    """New contents of the files and the notes about them: (calib, camera, lead_patch, notes, errors, warnings)."""
    notes, errors, warnings = [], [], []
    calib, cam = dict(cur_calib), dict(cur_cam)
    lens = None
    if a.lens:
        rep = geom.load_json(a.lens)
        if rep.get("status") != "ok" or "calibration" not in rep:
            errors.append("%s: status %r, not a usable lens calibration (%s)" % (
                a.lens, rep.get("status"), "; ".join(rep.get("reasons", [])) or "no values"))
        else:
            lens = rep
            for k, v in rep["calibration"].items():
                cam[k] = v
            notes.append("lens: focal %.2f -> %.2f px, centre %.1f, %.1f -> %.1f, %.1f, k1 %+.4f k2 %+.4f (rms %.2f px, "
                         "%d views)" % (cur_cam.get("focal_px", float("nan")), cam["focal_px"],
                                        cur_cam.get("cx", float("nan")), cur_cam.get("cy", float("nan")), cam["cx"],
                                        cam["cy"], cam["k1"], cam["k2"], rep["rms_px"], rep["views_used"]))
    if a.mount:
        rep = geom.load_json(a.mount)
        src = rep if a.mount_source == "pose" else (rep.get("pictures") or {})
        if src.get("status") != "ok" or src.get("pitch") is None:
            errors.append("%s (%s): status %r, not a usable mount estimate (%s)" % (
                a.mount, a.mount_source, src.get("status"), "; ".join(rep.get("reasons", []) or [src.get("reason") or ""])))
        else:
            pitch, yaw = src["pitch"], src["yaw"]
            ia = rep.get("intrinsics_assumed") or {}
            new = (cam.get("focal_px"), cam.get("cx"), cam.get("cy"))
            old = (ia.get("focal_px"), ia.get("cx"), ia.get("cy"))
            if a.mount_source == "pose" and all(v is not None for v in old + new) and any(
                    abs(o - n) > 0.05 for o, n in zip(old, new)):
                # the estimate fixes where the road vanishes in the picture; keep that pixel under the new lens values
                vp = geom.Camera(dict(focal_px=old[0], cx=old[1], cy=old[2]), dict(pitch=pitch, yaw=yaw)).vanishing_point()
                pitch, yaw = geom.angles_from_vanishing_point(vp[0], vp[1], dict(focal_px=new[0], cx=new[1], cy=new[2]))
                notes.append("mount: the drive was recorded with focal %.1f centre %.1f, %.1f; the angles are re-expressed "
                             "for the new lens values so that the road vanishes at the same pixel (%.1f, %.1f). Record "
                             "a straight drive with the new values and run mount.py again to confirm"
                             % (old[0], old[1], old[2], vp[0], vp[1]))
            calib["pitch"], calib["yaw"] = round(float(pitch), 5), round(float(yaw), 5)
            sg = src.get("sigma") or {}
            notes.append("mount (%s): pitch %+.4f rad (%+.2f deg) +- %.2f, yaw %+.4f rad (%+.2f deg) +- %.2f deg; was pitch "
                         "%+.4f, yaw %+.4f" % (a.mount_source, calib["pitch"], calib["pitch"] / DEG,
                                              sg.get("pitch", float("nan")) / DEG, calib["yaw"], calib["yaw"] / DEG,
                                              sg.get("yaw", float("nan")) / DEG, cur_calib.get("pitch", 0.0),
                                              cur_calib.get("yaw", 0.0)))
    for k in ("pitch", "yaw", "roll"):
        v = getattr(a, k)
        if v is not None:
            calib[k] = v
            notes.append("%s set by hand to %+.4f rad" % (k, v))
    lead_patch = {}
    if a.height_m is not None:
        calib["height_m"] = a.height_m
        notes.append("height_m %.3f (tape)" % a.height_m)
    if a.camera_to_front_m is not None:
        calib["camera_to_front_m"] = a.camera_to_front_m
        lead_patch["camera_to_front_m"] = a.camera_to_front_m
        notes.append("camera_to_front_m %.3f (tape) -> calib.json and lead.json" % a.camera_to_front_m)
    if a.lateral_offset_m is not None:
        calib["lateral_offset_m"] = a.lateral_offset_m
        notes.append("lateral_offset_m %+.3f (tape; lens right of the centre line)" % a.lateral_offset_m)
    for k in ("pitch", "yaw", "roll"):
        calib.setdefault(k, 0.0)
    e, w = geom.check_mount(calib)
    errors += e
    warnings += w
    e, w = geom.check_tape(calib.get("height_m"), calib.get("camera_to_front_m"), calib.get("lateral_offset_m"),
                           geom.load_json(a.vehicle) if a.vehicle else None)
    errors += e
    warnings += w
    return calib, cam, lead_patch, lens, notes, errors, warnings


def comment(stamp, what):
    # no double quotes in here: the daemons find keys by their quoted names
    return ("written by tools/calib/apply.py %s: %s. Angles in radians, openpilot convention (docs/CALIBRATION.md). "
            "Previous files: calibration-backups/%s" % (stamp, what.replace('"', "'"), stamp))


def backup(config_dir, names, stamp, manifest):
    d = os.path.join(config_dir, BACKUP_DIR, stamp)
    n = 1
    while os.path.exists(d):
        n += 1
        d = os.path.join(config_dir, BACKUP_DIR, "%s-%d" % (stamp, n))
    os.makedirs(d)
    kept = []
    for name in names:
        src = os.path.join(config_dir, name)
        if os.path.exists(src):
            shutil.copy2(src, os.path.join(d, name))
            kept.append(name)
    manifest = dict(manifest, stamp=os.path.basename(d), files_saved=kept, files_absent=[n for n in names if n not in kept])
    geom.write_json_atomic(os.path.join(d, "manifest.json"), manifest)
    return d


def flat_number(text, key):
    """The number a daemon's flat JSON reader finds for `key` (first quoted occurrence), or None."""
    import re
    m = re.search(r'"%s"\s*:\s*(-?[0-9.eE+-]+)' % re.escape(key), text)
    return float(m.group(1)) if m else None


def verify_written(config_dir, name, expect):
    """Read a written file back the way the daemons do. Returns a list of problems."""
    path = os.path.join(config_dir, name)
    with open(path) as f:
        text = f.read()
    obj = geom.load_json(path)
    bad = []
    for k, v in expect.items():
        got = flat_number(text, k)
        if got is None or abs(got - v) > 1e-9 + 1e-6 * abs(v) or obj.get(k) != v:
            bad.append("%s: %s reads back as %r, wanted %r" % (name, k, got, v))
    return bad


def cmd_rollback(a):
    root = os.path.join(a.config_dir, BACKUP_DIR)
    stamps = sorted(d for d in os.listdir(root) if os.path.isfile(os.path.join(root, d, "manifest.json"))) \
        if os.path.isdir(root) else []
    if not stamps:
        print("apply: no backups in %s" % root, file=sys.stderr)
        return 1
    want = a.rollback if a.rollback != "latest" else stamps[-1]
    if want not in stamps:
        print("apply: no backup %s (have: %s)" % (want, ", ".join(stamps)), file=sys.stderr)
        return 1
    src = os.path.join(root, want)
    man = geom.load_json(os.path.join(src, "manifest.json"))
    names = man.get("files_saved", []) + man.get("files_absent", [])
    if a.dry_run:
        print("apply: would restore %s from %s" % (", ".join(man.get("files_saved", [])), src))
        return 0
    cur = backup(a.config_dir, names, stamp_now(), dict(action="before rollback to %s" % want))
    for name in man.get("files_saved", []):
        tmp = os.path.join(a.config_dir, name + ".tmp.%d" % os.getpid())
        shutil.copy2(os.path.join(src, name), tmp)
        os.replace(tmp, os.path.join(a.config_dir, name))
    for name in man.get("files_absent", []):       # files the apply created
        p = os.path.join(a.config_dir, name)
        if os.path.exists(p):
            os.remove(p)
    print("apply: restored %s from %s (the files as they were just now: %s)" % (
        ", ".join(man.get("files_saved", [])) or "nothing", src, cur))
    print("apply: restart the camera daemon if the camera file changed; calib.json is re-read within 2 s")
    return 0


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--config-dir", default=os.environ.get("ROADSENSE_CONFIG_DIR", "/etc/roadsense"))
    ap.add_argument("--camera-file", help="the running camera daemon's file in the config dir (camera-dvr.json, "
                    "camera.json, camera-replay.json)")
    ap.add_argument("--lens", help="lens.json from lens.py solve")
    ap.add_argument("--mount", help="mount.json from mount.py")
    ap.add_argument("--mount-source", choices=("pose", "pictures"), default="pose",
                    help="which of mount.py's two estimates to apply (default: the road model's motion)")
    ap.add_argument("--pitch", type=float, help="rad, by hand")
    ap.add_argument("--yaw", type=float, help="rad, by hand")
    ap.add_argument("--roll", type=float, help="rad, by hand")
    ap.add_argument("--height-m", type=float, help="tape: lens centre above the road")
    ap.add_argument("--camera-to-front-m", type=float, help="tape: lens to the front bumper, along the car")
    ap.add_argument("--lateral-offset-m", type=float, help="tape: lens right (+) / left (-) of the car's centre line")
    ap.add_argument("--vehicle", help="a vehicle description (JSON, docs/VEHICLE_API.md 5.6) to check the tape values against")
    ap.add_argument("--frame", action="append", help="a picture from the camera to draw the view on (repeatable)")
    ap.add_argument("--session", help="a recorder session: pictures from its straight stretches are used")
    ap.add_argument("--view", help="where to write the view (default: next to the backup)")
    ap.add_argument("--view-only", action="store_true", help="draw the view for the files as they are; change nothing")
    ap.add_argument("--dry-run", action="store_true", help="show what would be written and draw the view; change nothing")
    ap.add_argument("--rollback", nargs="?", const="latest", help="restore the newest backup, or the named one")
    a = ap.parse_args(argv)
    if a.rollback:
        return cmd_rollback(a)

    changes_lens = bool(a.lens)
    if (changes_lens or a.view_only or a.frame or a.session) and not a.camera_file:
        ap.error("--camera-file is needed (which file the running camera daemon reads: camera-dvr.json, camera.json, ...)")
    cur_calib = geom.load_json(os.path.join(a.config_dir, "calib.json"), default={})
    cur_cam = geom.load_json(os.path.join(a.config_dir, a.camera_file), default={}) if a.camera_file else {}
    calib, cam, lead_patch, lens, notes, errors, warnings = plan(a, cur_calib, cur_cam)

    frames = list(a.frame or [])
    if a.session:
        frames += session_frames(a.session)
        if not frames:
            warnings.append("no pictures in %s" % a.session)
    size = None
    if lens:
        size = (lens["image"]["width"], lens["image"]["height"])
    if frames:
        fsize = picture_size(frames[0])
        if size and abs(fsize[0] / fsize[1] - size[0] / size[1]) > 0.01:
            errors.append("the picture %s is %dx%d, the lens calibration is for %dx%d" % (frames[0], fsize[0], fsize[1],
                                                                                         size[0], size[1]))
        size = size or fsize
    if cam.get("focal_px") is not None and size:
        e, w = geom.check_intrinsics(cam, size[0], size[1])
        errors += e
        warnings += w
    elif a.camera_file and cam.get("focal_px") is None:
        errors.append("%s has no focal_px and no --lens was given" % a.camera_file)

    for t in notes:
        print("  " + t)
    for t in warnings:
        print("  WARNING: " + t)
    if errors:
        for t in errors:
            print("  REFUSED: " + t)
        print("apply: nothing written")
        return 2

    stamp = stamp_now()
    names = ["calib.json"] + ([a.camera_file] if changes_lens else []) + (["lead.json"] if lead_patch else [])
    changed = calib != cur_calib or (changes_lens and cam != cur_cam) or bool(lead_patch)
    view_dir = None
    if a.view_only or a.dry_run or not changed:
        if not changed and not (a.view_only or a.dry_run):
            print("apply: nothing to change")
    else:
        what = "; ".join(notes)
        view_dir = backup(a.config_dir, names, stamp, dict(action="apply", notes=notes, warnings=warnings,
                                                          lens=os.path.abspath(a.lens) if a.lens else None,
                                                          mount=os.path.abspath(a.mount) if a.mount else None))
        stamp = os.path.basename(view_dir)
        calib["_comment"] = comment(stamp, what)
        geom.write_json_atomic(os.path.join(a.config_dir, "calib.json"), calib)
        bad = verify_written(a.config_dir, "calib.json", {k: calib[k] for k in ("pitch", "yaw", "roll", "height_m")
                                                           if calib.get(k) is not None})
        if changes_lens:
            cam["_comment"] = comment(stamp, "lens calibration, rms %.2f px over %d views; k1..k3 are recorded, the "
                                             "daemons do not use them yet" % (lens["rms_px"], lens["views_used"]))
            geom.write_json_atomic(os.path.join(a.config_dir, a.camera_file), cam)
            bad += verify_written(a.config_dir, a.camera_file, {k: cam[k] for k in ("focal_px", "cx", "cy")})
        if lead_patch:
            lead = geom.load_json(os.path.join(a.config_dir, "lead.json"), default={})
            lead.update(lead_patch)
            geom.write_json_atomic(os.path.join(a.config_dir, "lead.json"), lead)
        if bad:
            for t in bad:
                print("  ERROR: " + t)
            print("apply: the written files do not read back right: run  apply.py --config-dir %s --rollback %s"
                  % (a.config_dir, stamp))
            return 3
        print("apply: written %s in %s; the previous files are in %s" % (", ".join(names), a.config_dir, view_dir))
        if changes_lens:
            print("apply: restart the camera daemon to load the focal length and centre (e.g. sudo systemctl restart "
                  "roadsense-camerad-dvr); calib.json is re-read by modeld and laned within 2 s")
        if lead_patch:
            print("apply: restart the api to load lead.json (sudo systemctl restart roadsense-api)")
        print("apply: go back with  apply.py --config-dir %s --rollback %s" % (a.config_dir, stamp))

    if frames and cam.get("focal_px") is not None:
        out = a.view or os.path.join(view_dir or ".", "view.jpg" if view_dir else "calibration-view.jpg")
        title = "calibration %s%s" % (stamp if view_dir else "as it is" if a.view_only else "NOT written (dry run)", "")
        make_view(frames, cam, calib, out, title=title, tape=calib)
        print("apply: view %s : the red mark must sit where a straight road vanishes, the yellow line on the horizon, "
              "the green lines along a 3.5 m lane" % os.path.abspath(out))
    elif not frames:
        print("apply: no view drawn (give --frame PICTURE or --session RECORDING)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
