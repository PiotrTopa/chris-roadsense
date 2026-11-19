"""Calibration tools on synthetic data with known answers (lane L33).

    /home/piotr/datasets/l33/venv/bin/python -m unittest discover -s tools/calib/tests -v      (everything)
    python3 -m unittest discover -s tools/calib/tests -v     (without OpenCV: the lens and picture tests are skipped)
"""
import gzip
import json
import math
import mmap
import os
import shutil
import struct
import subprocess
import sys
import tempfile
import unittest

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))
import apply as apply_tool  # noqa: E402
import capture  # noqa: E402
import geom  # noqa: E402
import mount  # noqa: E402

try:
    import cv2
    HAVE_CV = hasattr(cv2, "aruco")
except ImportError:
    cv2, HAVE_CV = None, False
if HAVE_CV:
    import lens

DEG = math.pi / 180.0


def rd(path, mode="r"):
    with open(path, mode) as f:
        return f.read()


TRUE_LENS = dict(focal_px=988.0, cx=652.0, cy=349.0, k1=-0.33, k2=0.12, p1=0.0, p2=0.0, k3=0.0)
SIZE = (1280, 720)


class GeomTest(unittest.TestCase):
    def test_signs(self):
        # positive pitch = camera tilted down = the horizon is higher in the picture; positive yaw = camera turned
        # left = the road is to the right
        intr = dict(focal_px=1000.0, cx=640.0, cy=360.0)
        u, v = geom.Camera(intr, dict(pitch=0.05, yaw=0.0)).vanishing_point()
        self.assertAlmostEqual(u, 640.0, 6)
        self.assertLess(v, 360.0)
        u, v = geom.Camera(intr, dict(pitch=0.0, yaw=0.05)).vanishing_point()
        self.assertGreater(u, 640.0)
        self.assertAlmostEqual(v, 360.0, 6)

    def test_matches_api_overlay(self):
        sys.path.insert(0, os.path.join(HERE, "..", "..", "..", "api"))
        try:
            from roadsense_api import overlay
        except ImportError:
            self.skipTest("api overlay (Pillow) not importable")
        calib = dict(pitch=-0.07, yaw=0.03, roll=0.01, height_m=1.3)
        pr = overlay.Projector((975.0, 640.0, 360.0, 1280, 720), calib, (1280, 720))
        pts = np.array([[10.0, -1.5, 1.3], [40.0, 2.0, 1.3], [80.0, 0.0, 0.0]])
        want = np.array(pr.project(pts[:, 0], pts[:, 1], pts[:, 2]))
        got = geom.Camera(dict(focal_px=975.0, cx=640.0, cy=360.0), calib).project(pts)
        np.testing.assert_allclose(got, want, atol=1e-9)

    def test_distortion_round_trip(self):
        x, y = np.meshgrid(np.linspace(-0.6, 0.6, 9), np.linspace(-0.35, 0.35, 7))
        xd, yd = geom.distort(x, y, TRUE_LENS)
        xu, yu = geom.undistort(xd, yd, TRUE_LENS)
        np.testing.assert_allclose(xu, x, atol=1e-6)
        np.testing.assert_allclose(yu, y, atol=1e-6)
        cam = geom.Camera(TRUE_LENS, dict(pitch=0.02, yaw=-0.05, height_m=1.2))
        pts = np.array([[20.0, 3.0, 1.2], [8.0, -2.0, 1.2], [60.0, 0.5, -2.0]])
        rays = cam.ray(cam.project(pts))
        np.testing.assert_allclose(rays, pts / np.linalg.norm(pts, axis=1, keepdims=True), atol=1e-6)

    def test_vanishing_point_angles(self):
        intr = dict(focal_px=975.0, cx=640.0, cy=360.0)
        for pitch, yaw in ((0.02, -0.06), (-0.1, 0.04), (0.0, 0.0)):
            u, v = geom.Camera(intr, dict(pitch=pitch, yaw=yaw)).vanishing_point()
            p, y = geom.angles_from_vanishing_point(u, v, intr)
            self.assertAlmostEqual(p, pitch, 9)
            self.assertAlmostEqual(y, yaw, 9)

    def test_compose_residual(self):
        used = dict(pitch=0.03, yaw=-0.02, roll=0.0)
        true = geom.rot_from_euler(0.0, -0.01, -0.07)
        t = geom.rot_from_euler(0.0, used["pitch"], used["yaw"]).T @ true @ np.array([1.0, 0, 0])   # travel, used frame
        p, y = geom.compose_residual(used, math.atan2(t[2], math.hypot(t[0], t[1])), math.atan2(t[1], t[0]))
        self.assertAlmostEqual(p, -0.01, 9)
        self.assertAlmostEqual(y, -0.07, 9)

    def test_plausibility(self):
        self.assertEqual(geom.check_intrinsics(dict(focal_px=975.0, cx=640.0, cy=360.0), 1280, 720), ([], []))
        e, _ = geom.check_intrinsics(dict(focal_px=120.0, cx=640.0, cy=360.0), 1280, 720)
        self.assertTrue(e)
        e, _ = geom.check_intrinsics(dict(focal_px=975.0, cx=900.0, cy=360.0), 1280, 720)
        self.assertTrue(e)
        e, _ = geom.check_intrinsics(dict(focal_px=975.0, cx=640.0, cy=360.0, k1=-1.5), 1280, 720)   # folds back
        self.assertTrue(e)
        e, _ = geom.check_mount(dict(pitch=1.3, yaw=0.0))      # degrees given as radians
        self.assertTrue(e)
        e, w = geom.check_mount(dict(pitch=0.02, yaw=-0.06, roll=0.0, height_m=1.25))
        self.assertEqual((e, w), ([], []))
        e, _ = geom.check_mount(dict(height_m=125.0))          # centimetres
        self.assertTrue(e)
        e, w = geom.check_tape(1.25, 1.6, 0.02, dict(height_m=1.49, width_m=1.72, rear_axle_to_front_m=3.6,
                                                      camera=dict(x_m=2.0, z_m=1.25)))
        self.assertEqual((e, w), ([], []))
        e, _ = geom.check_tape(1.6, 1.6, 0.0, dict(height_m=1.49))          # above the roof
        self.assertTrue(e)
        _, w = geom.check_tape(1.25, 1.2, 0.0, dict(rear_axle_to_front_m=3.6, camera=dict(x_m=2.0, z_m=1.25)))
        self.assertTrue(w)
        e, _ = geom.check_tape(1.25, 160.0, 0.0)                            # centimetres
        self.assertTrue(e)


@unittest.skipUnless(HAVE_CV, "needs OpenCV with aruco")
class LensTest(unittest.TestCase):
    def views(self, n=24, seed=3, tilt=40.0, noise=0.15, fill=(0.3, 0.6)):
        board = lens.Board("charuco:8x5", 30.0)
        rng = np.random.default_rng(seed)
        poses = lens.synth_poses(board, TRUE_LENS, SIZE, n, rng, tilt_max_deg=tilt, fill=fill)
        return board, lens.synth_views(board, TRUE_LENS, SIZE, poses, noise_px=noise, rng=rng)

    def test_recovers_known_camera(self):
        board, views = self.views()
        rep = lens.solve_views(cv2, board, views, SIZE, bootstrap=40)
        self.assertEqual(rep["status"], "ok", rep.get("reasons"))
        c, s = rep["calibration"], rep["sigma"]
        self.assertLess(abs(c["focal_px"] - TRUE_LENS["focal_px"]) / TRUE_LENS["focal_px"], 0.003)
        self.assertLess(abs(c["cx"] - TRUE_LENS["cx"]), 2.0)
        self.assertLess(abs(c["cy"] - TRUE_LENS["cy"]), 2.0)
        self.assertLess(abs(c["k1"] - TRUE_LENS["k1"]), 0.01)
        self.assertLess(abs(c["k2"] - TRUE_LENS["k2"]), 0.03)
        self.assertLess(rep["rms_px"], 0.4)
        # the truth lies within the stated uncertainty (5 sigma: a test must not flake)
        for k in ("focal_px", "cx", "cy", "k1", "k2"):
            self.assertGreater(s[k], 0.0)
            self.assertLess(abs(c[k] - TRUE_LENS[k]), 5.0 * s[k] + 1e-3, k)
        self.assertGreater(rep["coverage"]["fraction"], 0.6)
        self.assertEqual(len(rep["coverage"]["grid"]), 6)

    def test_refuses_too_few_views(self):
        board, views = self.views(n=7)
        rep = lens.solve_views(cv2, board, views, SIZE, bootstrap=10)
        self.assertEqual(rep["status"], "refused")
        self.assertNotIn("calibration", rep)
        self.assertIn("distinct views", rep["reasons"][0])

    def test_refuses_repeated_views(self):
        board, views = self.views(n=3)
        rng = np.random.default_rng(0)
        many = [dict(name="rep-%d" % i, ids=v["ids"], px=v["px"] + rng.normal(0, 0.3, v["px"].shape).astype(np.float32))
                for i in range(10) for v in views]
        rep = lens.solve_views(cv2, board, many, SIZE, bootstrap=10)
        self.assertEqual(rep["status"], "refused")
        self.assertEqual(rep["views_distinct"], 3)
        self.assertEqual(rep["views_repeated"], 27)

    def test_refuses_board_always_facing_the_camera(self):
        board, views = self.views(n=24, tilt=1.0)
        rep = lens.solve_views(cv2, board, views, SIZE, bootstrap=20)
        self.assertEqual(rep["status"], "refused")
        self.assertTrue(any("tilted" in r for r in rep["reasons"]), rep["reasons"])
        self.assertNotIn("calibration", rep)

    def test_refuses_small_coverage(self):
        board = lens.Board("charuco:8x5", 30.0)
        rng = np.random.default_rng(5)
        poses = lens.synth_poses(board, TRUE_LENS, SIZE, 40, rng, fill=(0.15, 0.2))
        # keep only the views in the picture's middle
        views = [v for v in lens.synth_views(board, TRUE_LENS, SIZE, poses, rng=rng)
                 if abs(v["px"][:, 0].mean() - 640) < 200 and abs(v["px"][:, 1].mean() - 360) < 120]
        self.assertGreaterEqual(len(views), 10)
        rep = lens.solve_views(cv2, board, views, SIZE, bootstrap=20)
        self.assertEqual(rep["status"], "refused")
        self.assertTrue(any("covered" in r for r in rep["reasons"]), rep["reasons"])

    def test_pictures_through_the_detector(self):
        board = lens.Board("charuco:8x5", 30.0)
        rng = np.random.default_rng(11)
        d = tempfile.mkdtemp(prefix="l33-lens-")
        try:
            for i, (rv, tv) in enumerate(lens.synth_poses(board, TRUE_LENS, SIZE, 16, rng)):
                cv2.imwrite(os.path.join(d, "p%02d.png" % i), lens.render_view(cv2, board, TRUE_LENS, rv, tv, SIZE, rng=rng))
            views, size, n = lens.detect_all(cv2, board, [d])
            self.assertEqual((size, n), (SIZE, 16))
            self.assertGreaterEqual(len(views), 14)
            rep = lens.solve_views(cv2, board, views, size, bootstrap=20)
            self.assertEqual(rep["status"], "ok", rep.get("reasons"))
            c = rep["calibration"]
            self.assertLess(abs(c["focal_px"] - TRUE_LENS["focal_px"]) / TRUE_LENS["focal_px"], 0.006)
            self.assertLess(abs(c["cx"] - TRUE_LENS["cx"]), 4.0)
            self.assertLess(abs(c["cy"] - TRUE_LENS["cy"]), 4.0)
            self.assertLess(abs(c["k1"] - TRUE_LENS["k1"]), 0.02)
        finally:
            shutil.rmtree(d)

    def test_chessboard_board(self):
        board = lens.Board("checker:8x5", 30.0)
        rng = np.random.default_rng(2)
        rv, tv = lens.synth_poses(board, TRUE_LENS, SIZE, 1, rng, fill=(0.5, 0.5))[0]
        ids, px = board.detect(cv2, lens.render_view(cv2, board, TRUE_LENS, rv, tv, SIZE, rng=rng))
        self.assertEqual(len(ids), 7 * 4)

    def test_impact_of_the_distortion(self):
        r = lens.impact(dict(TRUE_LENS, cx=640.0, cy=360.0), SIZE)
        m, b = r["medmodel"], r["sbigmodel"]
        self.assertLess(m["model_px_near_vp"], 0.2)             # far vehicles do not move
        self.assertGreater(m["model_px_max"], 5.0)              # the crop's corners do
        self.assertGreater(b["model_px_max"], m["model_px_max"])
        self.assertLess(m["best_pinhole_focal"], TRUE_LENS["focal_px"])      # barrel: a shorter focal fits the crop
        none = lens.impact(dict(focal_px=988.0, cx=640.0, cy=360.0), SIZE)
        self.assertEqual(none["medmodel"]["model_px_max"], 0.0)

    def test_board_page_draws_opencv_board(self):
        node = shutil.which("node")
        if not node:
            self.skipTest("no node")
        html = rd(os.path.join(HERE, "..", "board.html"))
        js = html[html.index("<script>") + 8:html.index("</script>")]
        d = tempfile.mkdtemp(prefix="l33-board-")
        try:
            with open(os.path.join(d, "b.js"), "w") as f:
                f.write(js + "\nconsole.log(JSON.stringify(Array.from(boardCells('charuco', 8, 5), r => Array.from(r))));\n"
                             "console.log(JSON.stringify(Array.from(boardCells('checker', 6, 4), r => Array.from(r))));\n")
            out = subprocess.run([node, os.path.join(d, "b.js")], capture_output=True, text=True, check=True).stdout
        finally:
            shutil.rmtree(d)
        cells, checker = (np.array(json.loads(x), np.uint8) for x in out.strip().splitlines())
        ref = lens.Board("charuco:8x5").image(cv2, 64)          # 8 px per cell
        np.testing.assert_array_equal(np.kron(cells, np.ones((8, 8), np.uint8)) * 255, ref)
        np.testing.assert_array_equal(np.kron(checker, np.ones((1, 1), np.uint8)) * 255, lens.Board("checker:6x4").image(cv2, 8))


def write_drive(path, true, used, parts, rate=13.5, gain=1.0, noise=0.004, seed=0, t0=1000.0, seq0=1):
    """A message file of a drive. true / used: (pitch, yaw) rad; parts: [(seconds, speed m/s, yaw rate rad/s)]."""
    rng = np.random.default_rng(seed)
    r_used = geom.rot_from_euler(0.0, used[0], used[1])
    # the model shows `gain` of the error
    pt, yt = used[0] + gain * (true[0] - used[0]), used[1] + gain * (true[1] - used[1])
    t_c = r_used.T @ geom.rot_from_euler(0.0, pt, yt) @ np.array([1.0, 0.0, 0.0])
    op = gzip.open if path.endswith(".gz") else open
    t, seq = t0, seq0
    with op(path, "wt") as f:
        f.write(json.dumps(dict(type="calib", seq=seq, ts_mono_ns=int(t * 1e9), pitch=used[0], yaw=used[1], roll=0.0,
                                height_m=1.22, focal_px=975.0, cx=640.0, cy=360.0, width=1280, height=720,
                                source="config"), separators=(",", ":")) + "\n")
        for dur, v, yr in parts:
            for _ in range(int(dur * rate)):
                t += 1.0 / rate
                seq += 1
                vv = v + rng.normal(0, 0.15)
                pose = dict(vx=vv * t_c[0], vy=vv * t_c[1] + rng.normal(0, noise * max(v, 1.0)),
                            vz=vv * t_c[2] + rng.normal(0, noise * max(v, 1.0)), roll_rate=0.0, pitch_rate=0.0,
                            yaw_rate=yr + rng.normal(0, 0.002), vx_std=0.3, vy_std=0.005 * max(v, 1.0), vz_std=0.05,
                            roll_rate_std=0.001, pitch_rate_std=0.001, yaw_rate_std=0.001)
                f.write(json.dumps(dict(type="road", seq=seq, ts_mono_ns=int(t * 1e9), valid=True, pose=pose,
                                        lane_lines=[{"x": [0, 1], "y": [0, 1], "note": "{nested}"}]),
                                   separators=(",", ":")) + "\n")
    return t, seq


DRIVE = [(20, 3.0, 0.0), (60, 20.0, 0.0), (15, 15.0, 0.15), (70, 25.0, 0.0), (10, 0.0, 0.0), (50, 14.0, 0.0)]


class MountTest(unittest.TestCase):
    def setUp(self):
        self.d = tempfile.mkdtemp(prefix="l33-mount-")

    def tearDown(self):
        shutil.rmtree(self.d)

    def test_recovers_known_mount(self):
        true = (1.3 * DEG, -3.5 * DEG)
        p = os.path.join(self.d, "m.jsonl.gz")
        write_drive(p, true, (0.0, 0.0), DRIVE)
        rep = mount.estimate([p])
        self.assertEqual(rep["status"], "ok", rep.get("reasons"))
        self.assertLess(abs(rep["pitch"] - true[0]), 0.05 * DEG)
        self.assertLess(abs(rep["yaw"] - true[1]), 0.05 * DEG)
        self.assertLess(abs(rep["residual"]["yaw"] - true[1]), 0.05 * DEG)
        # only the straight, fast parts were used: 60 + 70 + 50 s less the trimmed ends
        self.assertGreater(rep["accepted_s"], 150)
        self.assertLess(rep["accepted_s"], 181)
        self.assertGreaterEqual(rep["n_blocks"], 20)
        # the error estimate covers the model-response term (10 % of 3.5 deg) and little else
        self.assertGreater(rep["sigma"]["yaw"], 0.3 * DEG)
        self.assertLess(rep["sigma"]["yaw"], 0.45 * DEG)
        u, v = rep["vp_px"]
        self.assertAlmostEqual(u, 640 + 975 * math.tan(true[1]), delta=1.5)

    def test_absolute_whatever_calibration_was_in_use(self):
        true = (-2.0 * DEG, 2.5 * DEG)
        out = []
        for i, used in enumerate(((0.0, 0.0), (-3.5 * DEG, 0.5 * DEG), true)):
            p = os.path.join(self.d, "u%d.jsonl" % i)
            write_drive(p, true, used, DRIVE, seed=i)
            out.append(mount.estimate([p]))
        for rep in out:
            self.assertEqual(rep["status"], "ok")
            self.assertLess(abs(rep["pitch"] - true[0]), 0.06 * DEG)
            self.assertLess(abs(rep["yaw"] - true[1]), 0.06 * DEG)
        self.assertLess(abs(out[2]["residual"]["yaw"]), 0.05 * DEG)          # calibrated: nothing left
        self.assertLess(out[2]["sigma"]["yaw"], 0.1 * DEG)

    def test_model_showing_less_than_the_error_is_inside_the_estimate(self):
        true = (0.0, -3.0 * DEG)
        p = os.path.join(self.d, "g.jsonl")
        write_drive(p, true, (0.0, 0.0), DRIVE, gain=0.92)
        rep = mount.estimate([p])
        self.assertEqual(rep["status"], "ok")
        self.assertAlmostEqual(rep["yaw"], 0.92 * true[1], delta=0.05 * DEG)
        self.assertLess(abs(rep["yaw"] - true[1]), rep["sigma"]["yaw"])
        self.assertTrue(rep["notes"])

    def test_not_enough_driving(self):
        p = os.path.join(self.d, "s.jsonl")
        write_drive(p, (0.0, -0.05), (0.0, 0.0), [(30, 3.0, 0.0), (20, 20.0, 0.0), (60, 15.0, 0.2)])
        rep = mount.estimate([p])
        self.assertEqual(rep["status"], "not_enough")
        self.assertNotIn("pitch", rep)
        self.assertIn("straight", rep["reasons"][0])
        p2 = os.path.join(self.d, "t.jsonl")
        write_drive(p2, (0.0, -0.05), (0.0, 0.0), [(120, 4.0, 0.0), (120, 12.0, 0.3)])      # town: slow or turning
        rep = mount.estimate([p2])
        self.assertEqual(rep["status"], "not_enough")
        self.assertEqual(rep["samples_accepted"], 0)

    def test_camera_moved_between_recordings(self):
        a, b = os.path.join(self.d, "a.jsonl"), os.path.join(self.d, "b.jsonl")
        write_drive(a, (1.3 * DEG, -3.4 * DEG), (0.0, 0.0), DRIVE, seed=1)
        write_drive(b, (-1.1 * DEG, -3.4 * DEG), (0.0, 0.0), DRIVE, seed=2)
        rep = mount.estimate([a, b])
        self.assertEqual(rep["status"], "not_enough")
        self.assertTrue(any("moved" in r for r in rep["reasons"]), rep["reasons"])

    def test_unsteady_blocks(self):
        p = os.path.join(self.d, "n.jsonl")
        # the direction wanders by degrees: not a mounting
        t, seq = write_drive(p, (0.0, -1.0 * DEG), (0.0, 0.0), [(40, 20.0, 0.0)], seed=1)
        for i, yaw in enumerate((2.0, -3.0, 1.0, -2.0, 3.0)):
            q = os.path.join(self.d, "n%d.jsonl" % i)
            t, seq = write_drive(q, (0.0, yaw * DEG), (0.0, 0.0), [(40, 20.0, 0.0)], seed=i, t0=t + 1, seq0=seq)
            with open(p, "a") as f:
                f.write("".join(ln for ln in rd(q).splitlines(True) if '"road"' in ln))
        rep = mount.estimate([p])
        self.assertEqual(rep["status"], "not_enough")

    def test_recommendation(self):
        pose = dict(status="ok", pitch=0.020, yaw=-0.058, sigma=dict(pitch=0.002, yaw=0.006))
        pic = dict(status="ok", pitch=0.021, yaw=-0.062, sigma=dict(pitch=0.003, yaw=0.003))
        r = mount.recommend(dict(pose, pictures=pic))
        self.assertEqual((r["status"], r["source"]), ("ok", dict(pitch="pose", yaw="pictures")))
        self.assertEqual((r["pitch"], r["yaw"], r["sigma"]), (0.020, -0.062, dict(pitch=0.002, yaw=0.003)))
        r = mount.recommend(dict(pose, pictures=dict(status="weak", pitch=0.05, yaw=0.0, sigma=dict(pitch=0.01, yaw=0.01))))
        self.assertEqual(r["source"], dict(pitch="pose", yaw="pose"))
        r = mount.recommend(dict(pose))
        self.assertEqual(r["source"], dict(pitch="pose", yaw="pose"))
        r = mount.recommend(dict(pose, pictures=dict(pic, yaw=-0.030)))       # 1.6 deg apart
        self.assertEqual(r["status"], "conflict")
        self.assertNotIn("pitch", r)
        self.assertIsNone(mount.recommend(dict(status="not_enough", pictures=pic)))   # the drive itself was refused
        self.assertIsNone(mount.recommend(dict(status="not_enough")))

    def test_truncated_gzip_is_read(self):
        p = os.path.join(self.d, "c.jsonl.gz")
        write_drive(p, (0.0, -3.0 * DEG), (0.0, 0.0), DRIVE)
        data = rd(p, "rb")
        with open(p, "wb") as f:
            f.write(data[:int(len(data) * 0.8)])
        rep = mount.estimate([p])
        self.assertEqual(rep["status"], "ok")
        self.assertLess(abs(rep["yaw"] + 3.0 * DEG), 0.06 * DEG)

    @unittest.skipUnless(HAVE_CV, "needs OpenCV")
    def test_pictures_vanishing_point(self):
        # a session: messages + pictures of a straight road whose lines meet at a known pixel, with clutter
        true = (1.2 * DEG, -3.6 * DEG)
        intr = dict(focal_px=975.0, cx=640.0, cy=360.0)
        s = os.path.join(self.d, "sess")
        os.makedirs(os.path.join(s, "frames"))
        write_drive(os.path.join(s, "messages.jsonl.gz"), true, (0.0, 0.0), DRIVE)
        rng = np.random.default_rng(4)
        seqs = mount.read_samples(s)[0]
        ok = seqs[mount.select(seqs, mount.DEFAULTS), 1].astype(int)
        for q in ok[::40]:
            off = rng.uniform(-0.6, 0.6)                        # where in the lane the car is
            cam = geom.Camera(intr, dict(pitch=true[0], yaw=true[1], height_m=1.25))
            img = np.full((720, 1280), 70, np.uint8)
            xs = np.linspace(5, 150, 60)
            for y in (-5.25, -1.75, 1.75, 5.25):
                uv = cam.project(np.stack([xs, np.full_like(xs, y + off), np.full_like(xs, 1.25)], axis=1))
                cv2.polylines(img, [np.round(uv).astype(np.int32)], False, 230, 3, cv2.LINE_AA)
            for _ in range(6):                                  # clutter: poles, shadows, a car's edge
                a, b = rng.uniform(0, 1280, 2), rng.uniform(0, 720, 2)
                cv2.line(img, (int(a[0]), int(b[0])), (int(a[1]), int(b[1])), int(rng.uniform(120, 200)), 2, cv2.LINE_AA)
            img = np.clip(img + rng.normal(0, 3, img.shape), 0, 255).astype(np.uint8)
            cv2.imwrite(os.path.join(s, "frames", "%d.jpg" % q), img)
        rep = mount.estimate([s])
        pic = mount.picture_method(rep)
        self.assertEqual(pic["status"], "ok", pic)
        self.assertLess(abs(pic["pitch"] - true[0]), 0.1 * DEG)
        self.assertLess(abs(pic["yaw"] - true[1]), 0.1 * DEG)
        self.assertLess(abs(pic["yaw"] - rep["yaw"]), 0.15 * DEG)


def args(**kw):
    base = dict(lens=None, mount=None, mount_source="pose", pitch=None, yaw=None, roll=None, height_m=None,
                camera_to_front_m=None, lateral_offset_m=None, vehicle=None)
    base.update(kw)
    return type("A", (), base)()


class ApplyTest(unittest.TestCase):
    def setUp(self):
        self.d = tempfile.mkdtemp(prefix="l33-apply-")
        self.cfg = os.path.join(self.d, "cfg")
        os.makedirs(self.cfg)
        geom.write_json_atomic(os.path.join(self.cfg, "calib.json"), dict(pitch=0.0, yaw=0.0, roll=0.0, height_m=1.22,
                                                                      _comment="nominal"))
        geom.write_json_atomic(os.path.join(self.cfg, "camera-dvr.json"), dict(focal_px=975.0, cx=640.0, cy=360.0,
                                                                           preview_every=2, poll_us=2000))
        self.lens = os.path.join(self.d, "lens.json")
        geom.write_json_atomic(self.lens, dict(status="ok", image=dict(width=1280, height=720), rms_px=0.2, views_used=22,
                                               calibration=dict(focal_px=991.5, cx=652.3, cy=349.1, k1=-0.33, k2=0.12,
                                                                p1=0.0, p2=0.0, k3=0.0)))
        self.mount = os.path.join(self.d, "mount.json")
        geom.write_json_atomic(self.mount, dict(status="ok", pitch=0.0227, yaw=-0.0610, sigma=dict(pitch=0.002, yaw=0.006),
                                                intrinsics_assumed=dict(focal_px=975.0, cx=640.0, cy=360.0, width=1280,
                                                                        height=720),
                                                pictures=dict(status="ok", pitch=0.0230, yaw=-0.0690,
                                                              sigma=dict(pitch=0.003, yaw=0.003)),
                                                recommended=dict(status="ok", pitch=0.0227, yaw=-0.0690,
                                                                 sigma=dict(pitch=0.002, yaw=0.003),
                                                                 source=dict(pitch="pose", yaw="pictures"))))
        from PIL import Image
        self.frame = os.path.join(self.d, "frame.jpg")
        Image.new("RGB", (1280, 720), (60, 60, 60)).save(self.frame)

    def tearDown(self):
        shutil.rmtree(self.d)

    def run_apply(self, *argv):
        with open(os.devnull, "w") as null:
            old, sys.stdout = sys.stdout, null
            try:
                return apply_tool.main(["--config-dir", self.cfg] + list(argv))
            finally:
                sys.stdout = old

    def read(self, name):
        return geom.load_json(os.path.join(self.cfg, name))

    def backups(self):
        return sorted(os.listdir(os.path.join(self.cfg, apply_tool.BACKUP_DIR)))

    def test_apply_keeps_previous_and_rolls_back(self):
        before = {n: rd(os.path.join(self.cfg, n)) for n in ("calib.json", "camera-dvr.json")}
        rc = self.run_apply("--camera-file", "camera-dvr.json", "--lens", self.lens, "--mount", self.mount, "--height-m",
                            "1.25", "--camera-to-front-m", "1.62", "--lateral-offset-m", "0.03", "--frame", self.frame)
        self.assertEqual(rc, 0)
        cam, cal = self.read("camera-dvr.json"), self.read("calib.json")
        self.assertEqual((cam["focal_px"], cam["cx"], cam["cy"], cam["k1"]), (991.5, 652.3, 349.1, -0.33))
        self.assertEqual((cam["preview_every"], cam["poll_us"]), (2, 2000))            # other keys stay
        self.assertEqual((cal["height_m"], cal["camera_to_front_m"], cal["lateral_offset_m"]), (1.25, 1.62, 0.03))
        self.assertEqual(self.read("lead.json"), {"camera_to_front_m": 1.62})
        # the lens values changed, so the angles keep the road's vanishing pixel, not the numbers
        vp_old = geom.Camera(dict(focal_px=975.0, cx=640.0, cy=360.0), dict(pitch=0.0227, yaw=-0.0690)).vanishing_point()
        vp_new = geom.Camera(dict(focal_px=991.5, cx=652.3, cy=349.1), cal).vanishing_point()
        np.testing.assert_allclose(vp_new, vp_old, atol=0.05)
        self.assertNotAlmostEqual(cal["yaw"], -0.0690, 3)
        # the daemons' flat readers find the numbers (no quoted key names inside the comment)
        text = rd(os.path.join(self.cfg, "calib.json"))
        for k in ("pitch", "yaw", "roll", "height_m"):
            self.assertEqual(apply_tool.flat_number(text, k), cal[k])
        b = self.backups()
        self.assertEqual(len(b), 1)
        bdir = os.path.join(self.cfg, apply_tool.BACKUP_DIR, b[0])
        for n, text in before.items():
            self.assertEqual(rd(os.path.join(bdir, n)), text)
        self.assertTrue(os.path.getsize(os.path.join(bdir, "view.jpg")) > 1000)
        self.assertEqual(self.run_apply("--rollback"), 0)
        for n, text in before.items():
            self.assertEqual(rd(os.path.join(self.cfg, n)), text)
        self.assertFalse(os.path.exists(os.path.join(self.cfg, "lead.json")))
        self.assertEqual(len(self.backups()), 2)                 # the rollback kept what it replaced

    def test_two_applies_never_share_a_backup(self):
        self.assertEqual(self.run_apply("--pitch", "0.01"), 0)
        self.assertEqual(self.run_apply("--pitch", "0.02"), 0)
        b = self.backups()
        self.assertEqual(len(b), 2)
        first = geom.load_json(os.path.join(self.cfg, apply_tool.BACKUP_DIR, b[0], "calib.json"))
        second = geom.load_json(os.path.join(self.cfg, apply_tool.BACKUP_DIR, b[1], "calib.json"))
        self.assertEqual((first["pitch"], second["pitch"]), (0.0, 0.01))
        self.assertEqual(self.run_apply("--rollback", b[0]), 0)
        self.assertEqual(self.read("calib.json")["pitch"], 0.0)

    def test_refuses_unusable_inputs(self):
        before = rd(os.path.join(self.cfg, "calib.json"))
        bad = os.path.join(self.d, "bad.json")
        geom.write_json_atomic(bad, dict(status="refused", reasons=["only 4 distinct views"],
                                         measured_not_usable=dict(focal_px=900.0, cx=640.0, cy=360.0)))
        self.assertEqual(self.run_apply("--camera-file", "camera-dvr.json", "--lens", bad), 2)
        geom.write_json_atomic(bad, dict(status="not_enough", reasons=["18 s of straight driving"]))
        self.assertEqual(self.run_apply("--mount", bad), 2)
        self.assertEqual(self.run_apply("--pitch", "1.3"), 2)                 # degrees as radians
        self.assertEqual(self.run_apply("--height-m", "125"), 2)              # centimetres
        self.assertEqual(self.run_apply("--camera-to-front-m", "-1"), 2)
        self.assertEqual(rd(os.path.join(self.cfg, "calib.json")), before)
        self.assertFalse(os.path.isdir(os.path.join(self.cfg, apply_tool.BACKUP_DIR)))

    def test_dry_run_and_pictures_source(self):
        before = rd(os.path.join(self.cfg, "calib.json"))
        view = os.path.join(self.d, "v.jpg")
        self.assertEqual(self.run_apply("--camera-file", "camera-dvr.json", "--mount", self.mount, "--frame", self.frame,
                                        "--view", view, "--dry-run"), 0)
        self.assertEqual(rd(os.path.join(self.cfg, "calib.json")), before)
        self.assertTrue(os.path.getsize(view) > 1000)
        self.assertEqual(self.run_apply("--mount", self.mount, "--mount-source", "pose"), 0)
        self.assertEqual(self.read("calib.json")["yaw"], -0.061)
        self.assertEqual(self.run_apply("--mount", self.mount, "--mount-source", "pictures"), 0)
        self.assertEqual((self.read("calib.json")["pitch"], self.read("calib.json")["yaw"]), (0.023, -0.069))
        self.assertEqual(self.run_apply("--mount", self.mount), 0)                # what mount.py recommends
        self.assertEqual((self.read("calib.json")["pitch"], self.read("calib.json")["yaw"]), (0.0227, -0.069))
        # two methods that contradict each other: nothing is applied without an explicit choice
        m = geom.load_json(self.mount)
        m["recommended"] = dict(status="conflict", why="the two methods differ by 1.20 deg in yaw")
        geom.write_json_atomic(self.mount, m)
        self.assertEqual(self.run_apply("--mount", self.mount), 2)
        self.assertEqual(self.read("calib.json")["yaw"], -0.069)

    def test_view_marks_the_vanishing_point(self):
        from PIL import Image
        intr, calib = dict(focal_px=975.0, cx=640.0, cy=360.0), dict(pitch=0.02, yaw=-0.06, height_m=1.25)
        img = apply_tool.draw_view(Image.new("RGB", (1280, 720), (0, 0, 0)), intr, calib, (1280, 720))
        u, v = geom.Camera(intr, calib).vanishing_point()
        r, g, b = img.getpixel((int(round(u)) + 3, int(round(v))))          # on the red cross
        self.assertGreater(r, 200)
        self.assertLess(g, 90)
        half = apply_tool.draw_view(Image.new("RGB", (640, 360), (0, 0, 0)), intr, calib, (1280, 720))   # scaled picture
        r, g, b = half.getpixel((int(round(u / 2)) + 2, int(round(v / 2))))
        self.assertGreater(r, 200)


def make_ring(path, w, h, frames):
    """A frame ring file (docs/ARCHITECTURE.md) holding `frames` (list of luma arrays) as seq 1..n."""
    slots, data_size = 6, w * h * 3 // 2
    slot_size = (4096 + data_size + 4095) // 4096 * 4096
    with open(path, "wb") as f:
        f.truncate(4096 + slots * slot_size)
    fd = os.open(path, os.O_RDWR)
    m = mmap.mmap(fd, 4096 + slots * slot_size)
    m[0:struct.calcsize(capture.HDR_FMT)] = struct.pack(capture.HDR_FMT, b"RSFRAME1", 4096, slots, slot_size, w, h, w, w,
                                                         1, 975.0, w / 2, h / 2, 0, os.getpid())

    def put(seq, y):
        slot = 4096 + (seq % slots) * slot_size
        m[slot:slot + 48] = struct.pack(capture.SLOT_FMT, seq, seq * 50_000_000, seq * 50_000_000, 0, 0.0, 0.0, 0.0, 4096,
                                        data_size)
        m[slot + 4096:slot + 4096 + w * h] = y.tobytes()
        m[56:64] = struct.pack("<Q", seq)

    for i, y in enumerate(frames):
        put(i + 1, y)
    return m, put


class CaptureTest(unittest.TestCase):
    def setUp(self):
        self.d = tempfile.mkdtemp(prefix="l33-cap-")

    def tearDown(self):
        shutil.rmtree(self.d)

    def test_ring_reader_and_pgm(self):
        w, h = 320, 180
        rng = np.random.default_rng(0)
        y = rng.integers(0, 255, (h, w), dtype=np.uint8)
        ring_path = os.path.join(self.d, "ring")
        m, put = make_ring(ring_path, w, h, [y])
        r = capture.Ring(ring_path)
        self.assertEqual(r.info(), {"width": w, "height": h, "focal_px": 975.0, "cx": 160.0, "cy": 90.0})
        seq, _real, got = r.latest()
        self.assertEqual(seq, 1)
        out = os.path.join(self.d, "f.pgm")
        capture.write_pgm(out, got, r.y_stride, w, h)
        data = rd(out, "rb")
        self.assertTrue(data.startswith(b"P5\n320 180\n255\n"))
        self.assertEqual(data[-w * h:], y.tobytes())
        # a slot being overwritten (its seq already moved on) is not returned as the frame
        slot = 4096 + (1 % 6) * r.slot_size
        m[slot:slot + 8] = struct.pack("<Q", 7)
        self.assertIsNone(r.latest())
        m.close()

    def test_keeps_still_and_new_pictures_only(self):
        w, h = 320, 180
        a = np.full((h, w), 40, np.uint8)
        b = a.copy()
        b[40:140, 60:260] = 220                     # "the board"
        ring_path = os.path.join(self.d, "ring")
        m, put = make_ring(ring_path, w, h, [a])
        out = os.path.join(self.d, "shots")
        env = dict(os.environ, PYTHONPATH="")
        p = subprocess.Popen([sys.executable, "-I", os.path.join(HERE, "..", "capture.py"), "--out", out, "--ring",
                              ring_path, "--count", "2", "--every", "0.3", "--timeout", "20"], env=env,
                             stdout=subprocess.PIPE, text=True)
        import time
        seq = 1
        t_end = time.monotonic() + 15
        phase = [a] * 8 + [b] * 12                  # the same still scene, then the board appears and holds still
        i = 0
        while p.poll() is None and time.monotonic() < t_end:
            seq += 1
            put(seq, phase[min(i, len(phase) - 1)])
            i += 1
            time.sleep(0.12)
        self.assertEqual(p.wait(timeout=10), 0)
        p.stdout.close()
        m.close()
        shots = sorted(f for f in os.listdir(out) if f.endswith(".pgm"))
        self.assertEqual(len(shots), 2)             # scene a once, scene b once: no repeats of a still scene
        first = rd(os.path.join(out, shots[0]), "rb")[-w * h:]
        second = rd(os.path.join(out, shots[1]), "rb")[-w * h:]
        self.assertEqual(first, a.tobytes())
        self.assertEqual(second, b.tobytes())
        self.assertIn("320x180", rd(os.path.join(out, "capture.txt")))

    def test_preview_copy(self):
        prev = os.path.join(self.d, "preview.jpg")
        with open(prev, "wb") as f:
            f.write(b"\xff\xd8 one \xff\xd9")
        out = os.path.join(self.d, "shots")
        rc = capture.main(["--out", out, "--source", "preview", "--preview", prev, "--count", "1", "--every", "0",
                           "--timeout", "5"])
        self.assertEqual(rc, 0)
        self.assertEqual(rd(os.path.join(out, "shot-0001.jpg"), "rb"), b"\xff\xd8 one \xff\xd9")

    def test_no_ring(self):
        self.assertEqual(capture.main(["--out", os.path.join(self.d, "o"), "--ring", os.path.join(self.d, "none")]), 1)


if __name__ == "__main__":
    unittest.main()
