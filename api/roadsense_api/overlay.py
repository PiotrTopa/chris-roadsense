"""Draw road/sign overlays on a preview JPEG with Pillow.

Model outputs are in openpilot's (calibrated) device frame, passed through by modeld:
x forward, y right, z down, origin at the camera. Projection:
  p_dev   = R(roll,pitch,yaw) p_calib          (openpilot rot_from_euler: Rz(yaw) Ry(pitch) Rx(roll))
  p_cam   = [[0,1,0],[0,0,1],[1,0,0]] p_dev    (camera: x right, y down, z forward)
  u,v     = f*X/Z + cx, f*Y/Z + cy, then scaled from sensor size to preview size.
"""
import io
import math

import numpy as np
from PIL import Image, ImageDraw, ImageFont

VIEW_FROM_DEVICE = np.array([[0.0, 1.0, 0.0], [0.0, 0.0, 1.0], [1.0, 0.0, 0.0]])
MIN_Z = 1.0
LEAD_W, LEAD_H = 1.8, 1.5
PATH_HALF_W = 0.9


def rot_from_euler(roll, pitch, yaw):
    cr, sr, cp, sp, cy, sy = (math.cos(roll), math.sin(roll), math.cos(pitch),
                              math.sin(pitch), math.cos(yaw), math.sin(yaw))
    rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    return rz @ ry @ rx


def _font(size):
    try:
        return ImageFont.load_default(size=size)
    except TypeError:
        return ImageFont.load_default()


class Projector:
    def __init__(self, intrinsics, calib, img_size):
        f, cx, cy, w, h = intrinsics
        s = img_size[0] / float(w)
        self.f, self.cx, self.cy, self.s = f * s, cx * s, cy * s, s
        calib = calib or {}
        self.height = float(calib.get("height_m") or 1.22)
        self.m = VIEW_FROM_DEVICE @ rot_from_euler(float(calib.get("roll") or 0.0),
                                                   float(calib.get("pitch") or 0.0),
                                                   float(calib.get("yaw") or 0.0))

    def project(self, x, y, z):
        """Arrays of device-frame points -> list of (u,v) for points in front of the camera."""
        p = self.m @ np.vstack([np.asarray(x, float), np.asarray(y, float), np.asarray(z, float)])
        ok = p[2] > MIN_Z
        u = self.f * p[0, ok] / p[2, ok] + self.cx
        v = self.f * p[1, ok] / p[2, ok] + self.cy
        return list(zip(u.tolist(), v.tolist()))


def _prob_colour(p, alpha=230):
    p = max(0.0, min(1.0, float(p or 0.0)))
    return (int(255 * (1 - p)), int(255 * p), 40, alpha)


def _short_cls(cls):
    return (cls or "?").split("--")[-1]


LIGHT_COLOURS = {"red": (255, 40, 40), "yellow": (255, 200, 0), "red_yellow": (255, 120, 0), "green": (40, 255, 90),
                 "off": (150, 150, 150), "unknown": (150, 150, 150)}


def draw(img, road=None, signs=None, calib=None, intrinsics=(1500.0, 960.0, 540.0, 1920, 1080), hud=None, lights=None):
    """Return a new RGB image with overlays drawn on `img` (PIL.Image)."""
    base = img.convert("RGBA")
    layer = Image.new("RGBA", base.size, (0, 0, 0, 0))
    d = ImageDraw.Draw(layer)
    pr = Projector(intrinsics, calib, base.size)
    font = _font(14)
    lw = max(2, round(4 * pr.s))

    if road:
        plan = road.get("plan") or {}
        if plan.get("x"):
            x = np.asarray(plan["x"], float)
            y = np.asarray(plan.get("y") or np.zeros_like(x), float)
            z = np.asarray(plan.get("z") or np.zeros_like(x), float) + pr.height   # path on the ground (z down)
            left = pr.project(x, y + PATH_HALF_W, z)
            right = pr.project(x, y - PATH_HALF_W, z)
            if len(left) >= 2 and len(right) >= 2:
                d.polygon(left + right[::-1], fill=(40, 120, 255, 70), outline=(80, 160, 255, 160))
        for ll in road.get("lane_lines") or []:
            pts = pr.project(ll["x"], ll["y"], ll["z"])
            if len(pts) >= 2:
                p = ll.get("prob") or 0.0
                d.line(pts, fill=_prob_colour(p), width=max(1, round(lw * (0.3 + p))))
        for re in road.get("road_edges") or []:
            pts = pr.project(re["x"], re["y"], re["z"])
            if len(pts) >= 2:
                d.line(pts, fill=(255, 80, 220, 200), width=lw)
        for lead in (road.get("leads") or [])[:1]:     # hypothesis 0 = lead now; +2s/+4s not drawn
            p = lead.get("prob") or 0.0
            if p < 0.3 or not lead.get("x"):
                continue
            lx, ly = float(lead["x"][0]), float((lead.get("y") or [0.0])[0])
            zg = pr.height                              # ground, z down
            c = pr.project([lx] * 4, [ly - LEAD_W / 2, ly + LEAD_W / 2] * 2, [zg, zg, zg - LEAD_H, zg - LEAD_H])
            if len(c) == 4:
                us, vs = [q[0] for q in c], [q[1] for q in c]
                col = (255, 200, 0, int(80 + 175 * p))
                d.rectangle([min(us), min(vs), max(us), max(vs)], outline=col, width=lw)
                v = (lead.get("v") or [None])[0]
                label = f"{lx:.1f} m p{p:.2f}" + (f" {v:.1f} m/s" if v is not None else "")
                d.text((min(us), min(vs) - 16), label, fill=col, font=font)

    if signs:
        for det in signs.get("detections") or []:
            box = det.get("box")
            if not box or len(box) != 4:
                continue
            b = [v * pr.s for v in box]
            d.rectangle(b, outline=(0, 255, 255, 255), width=2)
            d.text((b[0], b[3] + 2), f"#{det.get('track_id')} {_short_cls(det.get('cls'))} {det.get('score', 0):.2f}",
                   fill=(0, 255, 255, 255), font=font)

    if lights:   # traffic lights: box in the state colour, the ego light thick with its distance
        for li in lights.get("lights") or []:
            box = li.get("box")
            if not box or len(box) != 4:
                continue
            b = [v * pr.s for v in box]
            c = LIGHT_COLOURS.get(li.get("state"), (150, 150, 150))
            ego = bool(li.get("ego"))
            d.rectangle([b[0] - 2, b[1] - 2, b[2] + 2, b[3] + 2], outline=c + (255,), width=4 if ego else 2)
            label = f"#{li.get('track_id')} {li.get('state')}" + (" flash" if li.get("flashing") else "")
            if ego and li.get("dist_m") is not None:
                label = "EGO " + label + f" {li['dist_m']:.0f} m"
            d.text((b[0], b[3] + 3), label, fill=c + (255,), font=font)

    if hud:
        d.rectangle([0, 0, 8 + 7 * max(len(s) for s in hud), 6 + 16 * len(hud)], fill=(0, 0, 0, 120))
        for i, s in enumerate(hud):
            d.text((4, 3 + 16 * i), s, fill=(255, 255, 255, 230), font=font)

    return Image.alpha_composite(base, layer).convert("RGB")


def render_jpeg(jpeg_bytes, quality=80, **kw):
    img = Image.open(io.BytesIO(jpeg_bytes))
    out = io.BytesIO()
    draw(img, **kw).save(out, "JPEG", quality=quality)
    return out.getvalue()
