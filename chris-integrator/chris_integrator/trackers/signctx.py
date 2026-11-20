"""Context checks for round red sign tracks (lane L8: stop lamps and red traffic lights read as `no-entry`).

The sign provider judges a track by its pixels (roadsense-signs: per-class minimum score, glow share, optional
verifier) and flags it `suppressed` + `reason`. The api adds what only it knows, per detection of a round red
class (prohibitions, speed limits, stop; ROUND_RED below):

  traffic_light  the box centre lies inside a light box of the latest `lights` message (<= 0.4 s apart),
                 grown by 25 % on each side;
  lead_vehicle   the box centre lies inside the lead vehicle's projected rear (modeld `leads[0]`, prob >= 0.5,
                 a 2.6 m wide, 0-2 m high rectangle at the lead's distance; `road` <= 0.3 s apart) - where stop
                 lamps sit, below any sign plate.

Per track the api counts checked detections and context hits; with >= 2 checked and a share >= 0.5 of either
the track gets `context` {"traffic_light": share, "lead_vehicle": share} and, unless the provider's appearance
numbers vouch for it (verifier mean `verify` >= 0.9 and glow share < 0.5), `suppressed: true` with reason
"traffic_light" or "lead_vehicle". Suppressed tracks are not live tracks and not used by the speed-limit state;
they stay in the messages, `/v1/signs` (`suppressed_tracks`) and the history with their reason.
"""
import re

from ..native import round                  # builtins.round, native where built
from .projection import Projector

ROUND_RED = re.compile(r"^regulatory--(no-|maximum-speed-limit|road-closed|height-limit|weight-limit|width-limit|stop$)")
LIGHT_DT_NS = 400_000_000
ROAD_DT_NS = 300_000_000
LIGHT_GROW = 0.25
LEAD_W, LEAD_H, LEAD_MIN_PROB = 2.6, 2.0, 0.5
MIN_CHECKED, MIN_SHARE = 2, 0.5
VOUCH_VERIFY, VOUCH_GLOW = 0.9, 0.5
MAX_TRACKS = 500


def is_round_red(cls):
    return bool(ROUND_RED.match(cls or ""))


def _inside(p, b, grow=0.0):
    w, h = b[2] - b[0], b[3] - b[1]
    return b[0] - grow * w <= p[0] <= b[2] + grow * w and b[1] - grow * h <= p[1] <= b[3] + grow * h


def lead_rect(road, calib, intrinsics, frame_wh):
    """Image rectangle (frame px) of the lead vehicle's rear, or None."""
    if (road or {}).get("type") == "lead":          # the tracked lead (lead.py): present, valid, with a distance
        if not road.get("present") or road.get("valid") is False or road.get("dist_m") is None:
            return None
        lead = {"x": [road["dist_m"]], "y": [road.get("lateral_m") or 0.0]}
    else:
        leads = (road or {}).get("leads") or []
        if not leads:
            return None
        lead = leads[0]
        if (lead.get("prob") or 0.0) < LEAD_MIN_PROB or not lead.get("x"):
            return None
    f, cx, cy, w, h = intrinsics
    fw, fh = frame_wh
    pr = Projector((f * fw / w, cx * fw / w, cy * fw / w, fw, fh), calib, (fw, fh))
    lx, ly = float(lead["x"][0]), float((lead.get("y") or [0.0])[0])
    zg = pr.height
    c = pr.project([lx] * 4, [ly - LEAD_W / 2, ly + LEAD_W / 2] * 2, [zg, zg, zg - LEAD_H, zg - LEAD_H])
    if len(c) != 4:
        return None
    us, vs = [q[0] for q in c], [q[1] for q in c]
    return [min(us), min(vs), max(us), max(vs)]


class SignContext:
    def __init__(self):
        self.tracks = {}      # track_id -> [checked, light hits, lead hits]

    def _decide(self, tid, item):
        c = self.tracks.get(tid)
        if not c or c[0] < MIN_CHECKED:
            return
        share = {"traffic_light": c[1] / c[0], "lead_vehicle": c[2] / c[0]}
        hit = [k for k in ("traffic_light", "lead_vehicle") if share[k] >= MIN_SHARE]
        if not hit:
            return
        item["context"] = {k: round(v, 2) for k, v in share.items()}
        if item.get("suppressed"):
            return
        v, g = item.get("verify"), item.get("glow")
        if v is not None and v >= VOUCH_VERIFY and (g is None or g < VOUCH_GLOW):
            return                                   # the provider's appearance checks vouch for a real sign
        item["suppressed"] = True
        item["reason"] = hit[0]

    def annotate(self, msg, road=None, lights=None, calib=None, intrinsics=(690.0, 640.0, 360.0, 1280, 720)):
        """Adds `context` / `suppressed` / `reason` to round red detections and ended tracks of a `signs` message.
        `road`: a `road` message, or (the api, since lane L37) the tracked `lead` message: the lead vehicle that
        explains a red round "sign" is then the validated track, not the model's raw guess."""
        ts = msg.get("ts_mono_ns") or 0
        lt = lights if lights and abs((lights.get("ts_mono_ns") or 0) - ts) <= LIGHT_DT_NS else None
        rd = road if road and abs((road.get("ts_mono_ns") or 0) - ts) <= ROAD_DT_NS else None
        fw, fh = (lt or {}).get("frame") or intrinsics[3:5]
        lead = None
        lead_done = False
        for d in msg.get("detections") or []:
            tid, box = d.get("track_id"), d.get("box")
            if tid is None or not box or len(box) != 4 or not is_round_red(d.get("cls")):
                continue
            if not lead_done:
                lead, lead_done = (lead_rect(rd, calib, intrinsics, (fw, fh)) if rd else None), True
            p = ((box[0] + box[2]) / 2, (box[1] + box[3]) / 2)
            c = self.tracks.setdefault(tid, [0, 0, 0])
            c[0] += 1
            if lt and any(_inside(p, li["box"], LIGHT_GROW) for li in lt.get("lights") or [] if li.get("box")):
                c[1] += 1
            if lead and _inside(p, lead):
                c[2] += 1
            self._decide(tid, d)
        for e in msg.get("tracks_ended") or []:
            tid = e.get("track_id")
            if is_round_red(e.get("cls")):
                self._decide(tid, e)
            self.tracks.pop(tid, None)
        while len(self.tracks) > MAX_TRACKS:          # tracks whose end we never saw (provider restart)
            self.tracks.pop(next(iter(self.tracks)))
        return msg
