#!/usr/bin/env python3
"""The integrator's object layer on real drives with injected sensors, and against the simulator (lane L41-O;
docs/lanes/L38-integrator.md 5.5).

    integrator_objects_eval.py inject SESSION [--pos POS.txt] [--no-lead] [--cam2] [--two] [--baseline]
                                      [--statics path|ahead] [--no-link] [--no-split] [--frames OUT.jsonl]
        (--statics ahead --no-link --no-split is the injection of L38)
        synthetic radar(s) / second camera injected into a recording (chris_integrator/sim/inject.py); every
        lead frame and every change of the lead's track id attributed to what the injector put there: the
        injected vehicle, a roadside object (on the driven road or beside it), a double-bounce ghost, nothing
        (clutter). The injected detections' `synthetic` tags and the injector's own description of its world are
        used here only; the integrator never reads them.
    integrator_objects_eval.py sim [--seeds 1 2] [--scenario town]
        radar + second camera against simulated truth: the lead (tracked, false, ghost / static as the lead,
        id switches), the track list per vehicle (tracked, duplicates, id switches, rms), tracks on no object

Pseudo-truth on a recording: the recording's own camera-only lead, smoothed non-causally, exactly as injected
("the injected vehicles": since L41 split at the camera track's distance steps and joined across gaps where
consistent, sim/inject.py). Also reported against the camera tracks unsplit (what L38 used). It is the camera's
own estimate: it tests association, continuity and false leads, not accuracy.
"""
import argparse
import json
import math
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(os.path.dirname(HERE), "chris-integrator"))
import replay_lib as rl  # noqa: E402
from chris_integrator import Integrator  # noqa: E402

NS = 1e9
INPUTS = ("ego", "vehicle", "gnss", "map", "calib", "objects")
RADAR = [{"kind": "radar", "id": "*"}]
COAST_GRACE_S = 1.5          # a lead this soon after the injected vehicle vanished is counted as its coast


def name_of(p):
    return os.path.basename(os.path.normpath(p)).replace("20261119-", "")


def session(path, a):
    pos = rl.load_pos(a.pos) if getattr(a, "pos", None) else None
    return rl.read_session(path, road_model="vision", pos=pos)


class Veh:
    def __init__(self, msgs):
        self.t, self.v = [], []
        for m in msgs:
            if m.get("type") == "ego" and isinstance(m.get("speed_mps"), (int, float)):
                self.t.append(m["ts_mono_ns"] / NS), self.v.append(m["speed_mps"])

    def at(self, t, max_gap=0.3):
        import bisect
        i = bisect.bisect_left(self.t, t)
        if i == 0 or i >= len(self.t) or self.t[i] - self.t[i - 1] > max_gap:
            return None
        u = (t - self.t[i - 1]) / (self.t[i] - self.t[i - 1])
        return self.v[i - 1] + u * (self.v[i] - self.v[i - 1])


def track_at(tracks, t):
    import bisect
    for a in tracks:
        if a[0, 0] <= t <= a[-1, 0]:
            i = min(bisect.bisect_left(a[:, 0].tolist(), t), len(a) - 1)
            if abs(a[i, 0] - t) < 0.3:
                return a[i]
    return None


def attribute(W, T, x, y, radar_mount=(2.2, 0.0)):
    """What the injector had near (x, y) (car frame, y left) at T: ("vehicle", k) / ("static", j) / ("ghost", k) /
    ("none", None), with the normalised distance."""
    best = ("none", None, 9.0)
    for k, g, lat, v in W.vehicle_at(T):
        d = (abs(x - g) / (3.0 + 0.1 * g)) ** 2 + (abs(y - lat) / 2.0) ** 2
        if d < 1.0 and d < best[2]:
            best = ("vehicle", k, d)
        mx, my = radar_mount
        r, az = math.hypot(g - mx, lat - my), math.atan2(lat - my, g - mx)
        gx, gy = mx + 2 * r * math.cos(az), my + 2 * r * math.sin(az)
        d = (abs(x - gx) / (6.0 + 0.1 * gx)) ** 2 + (abs(y - gy) / 2.5) ** 2
        if d < 1.0 and d < best[2]:
            best = ("ghost", k, d)
    for j, sx, sy in W.statics_at(T):
        d = (abs(x - sx) / 3.0) ** 2 + (abs(y - sy) / 2.0) ** 2
        if d < 1.0 and d < best[2]:
            best = ("static", j, d)
    return best[:2]


def vehicle_ended(W, T, grace=COAST_GRACE_S):
    """Index of an injected vehicle that ended within `grace` before T (and is not present now), else None."""
    for k, a in enumerate(W.vehicles):
        if 0.0 < T - a[-1, 0] <= grace:
            return k
    return None


def cmd_inject(a):
    from chris_integrator.sim.inject import inject, lead_truth
    path = a.session
    msgs = [m for m in session(path, a) if m.get("type") in INPUTS + ("road",)]
    veh = Veh(msgs)
    t0 = min(m["ts_mono_ns"] for m in msgs) / NS
    raw_tracks = []
    cam_tracks = [] if a.no_lead else lead_truth(msgs, unsplit=raw_tracks)
    radars = [{"sensor": "radar_front", "mount": (2.2, 0.0, 0.0)}]
    if a.two:
        radars.append({"sensor": "radar_fl", "mount": (2.0, 0.8, math.radians(40.0))})
    cam2 = {"sensor": "cam_wide", "mount": (0.0, 0.0, 0.0)} if a.cam2 else None
    world = {}
    ms = list(inject(msgs, radars=radars, camera2=cam2, lead=raw_tracks if a.no_split else cam_tracks,
                     statics_mode=a.statics, link=not a.no_link, world=world))
    W = world["injected"]
    vehicles, members = W.vehicles, W.members          # the pseudo-truth is what was injected
    cfg = {"sources": RADAR + ([{"kind": "camera_objects", "id": "cam_wide"}] if cam2 else [])}
    n_stat = sum(1 for p in W.stat_world if p is not None)
    print("== %s: injected statics %s (%d placed, %d dropped near another stretch of the path), vehicles %s (%d camera tracks, "
          "%d pieces after splitting at steps -> %d vehicles)%s" % (
              name_of(path), a.statics, n_stat, len(W.stat_world) - n_stat if a.statics == "path" else 0,
              "joined" if not a.no_link else "as camera tracks", len(raw_tracks), len(cam_tracks), len(vehicles),
              "; radars %s%s" % ([r["sensor"] for r in radars], ", second camera" if cam2 else "")))
    runs = [("with injected sensors", ms, cfg, a.frames)]
    if a.baseline:
        runs.insert(0, ("recording as it is (the recording's camera only)", msgs, {}, None))
    for label, mlist, c, frames in runs:
        rows = run_one(mlist, c, W, veh, t0, raw_tracks, vehicles, frames)
        print("-- %s" % label)
        report(rows, W, vehicles, members, cam_tracks, a)


def run_one(ms, cfg, W, veh, t0, raw_tracks, vehicles, frames):
    I = Integrator(cfg)
    rows = []
    fout = open(frames, "w") if frames else None
    for m in ms:
        if m["type"] != "road":
            I.on_message(m)
            continue
        es, tr, ld = I.frame(m)
        T = m["ts_mono_ns"] / NS
        v = veh.at(T)
        row = {"t": round(T - t0, 3), "v": v, "present": bool(ld["present"]), "valid": bool(ld["valid"]),
               "cam": track_at(raw_tracks, T) is not None, "veh": track_at(vehicles, T) is not None,
               "inj": [[k, round(g, 2), round(y, 2)] for k, g, y, _ in W.vehicle_at(T)], "vision": es["sources"]["vision"]["state"],
               "z": (ld.get("vision") or {}).get("dist_m"), "vs": ld.get("vision_scale")}
        if ld["present"]:
            x, y = ld["dist_m"], -ld["lateral_m"]
            kind, oid = attribute(W, T, x, y)
            trk = next((t for t in I.objects.tracks if t.id == ld["track_id"]), None)
            row.update(id=ld["track_id"], src=ld["source"], dist=x, lat=ld["lateral_m"], kind=kind, oid=oid,
                       mode=ld["mode"], ev=(ld.get("event") or {}).get("kind") if (ld.get("event") or {}).get("age_s", 9) == 0 else None,
                       desig=trk is not None and trk.designated is not None,
                       moved=bool(trk.moved) if trk is not None else None,
                       stationary=trk is not None and "stationary" in trk.flags,
                       sources=sorted(trk.sources) if trk is not None else [],
                       hits=trk.hits if trk is not None else None,
                       vx=round(float(trk.x[2]), 2) if trk is not None else None,
                       age=round(T - trk.t0, 2) if trk is not None else None,
                       ended=vehicle_ended(W, T))
            if kind == "static":
                row["static_path_m"] = round(W.path_distance(oid), 2)
            # the track that held the previous lead: does it still exist, who sees it
            row["ids_alive"] = [t.id for t in I.objects.tracks if t.state != "tentative"]
        rows.append(row)
        if fout:
            fout.write(json.dumps(row) + "\n")
    if fout:
        fout.close()
    return rows


def injected_source(r):
    parts = (r.get("src") or "").split("+")
    return "radar" in parts or "camera" in parts


def classify_lead(r):
    """Cause of a lead frame where the pseudo-truth ("vehicles") has none."""
    k = r["kind"]
    if not injected_source(r) and k != "vehicle":
        return "the recording's own camera, no injected sensor on the lead (camera tracks under 1 s or after a step are not in the pseudo-truth)"
    if k == "vehicle":
        return "injected vehicle, not in the camera-only pseudo-truth (injector)" if not r["veh"] else "vehicle"
    if k == "static":
        on = r.get("static_path_m", 9) < 2.5
        return "static object ON the driven road (injector)" if on else \
            "static roadside object, %s (tracker)" % ("moved flag set" if r["moved"] else "never moved")
    if k == "ghost" and r.get("ended") is None:
        return "ghost of the vehicle (tracker)"
    if r.get("ended") is not None:
        return "coast after the injected vehicle vanished (injector: vehicles end with the camera's track)"
    return "no injected object near: clutter-born or drifted track (tracker)"


def report(rows, W, vehicles, members, cam_tracks, a):
    nfr = len(rows)
    mv = [r for r in rows if r["v"] is not None and r["v"] >= 5.0]
    pres = [r for r in rows if r["present"]]
    print("frames %d; lead present %.3f (vehicle >= 5 m/s: %.3f); valid false %.3f" % (
        nfr, len(pres) / nfr, sum(1 for r in mv if r["present"]) / max(len(mv), 1), sum(1 for r in rows if not r["valid"]) / nfr))
    src = {}
    for r in pres:
        src[r["src"]] = src.get(r["src"], 0) + 1
    print("lead by source %s" % {k: round(v / nfr, 3) for k, v in sorted(src.items(), key=lambda kv: -kv[1])})
    for label, key in (("camera tracks unsplit (L38's pseudo-truth)", "cam"), ("the injected vehicles", "veh")):
        nt = sum(1 for r in rows if r[key])
        if not nt and not pres:
            continue
        tracked = sum(1 for r in rows if r[key] and r["present"] and r["kind"] == "vehicle")
        without = [r for r in pres if not r[key]]
        print("against %s: frames with one %d; tracked (lead on the injected vehicle) %.3f; lead present without one %.4f of frames (%d)" % (
            label, nt, tracked / max(nt, 1), len(without) / nfr, len(without)))
    ids = {r["id"] for r in pres}
    print("lead track ids %d" % len(ids))
    ro = [r for r in pres if "vision" not in r["src"].split("+")]
    if ro:
        print("lead with no camera evidence (no 'vision' in its source): %.4f of frames, dist p10/50/90 %s m; of those on the injected vehicle %.3f" % (
            len(ro) / nfr, np.round(np.percentile([r["dist"] for r in ro], [10, 50, 90]), 1), sum(1 for r in ro if r["kind"] == "vehicle") / len(ro)))
    # ---- frames with a lead where the pseudo-truth (vehicles) has none, or on something else than the vehicle
    print("lead frames by cause (pseudo-truth = the injected vehicles; share of all frames, count, dist p10/50/90 m):")
    causes = {}
    for r in pres:
        if r["veh"] and r["kind"] == "vehicle":
            continue
        c = classify_lead(r) if not r["veh"] else "pseudo-truth has a vehicle, lead on: " + classify_lead(r)
        causes.setdefault(c, []).append(r)
    for c, rs in sorted(causes.items(), key=lambda kv: -len(kv[1])):
        d = np.percentile([r["dist"] for r in rs], [10, 50, 90])
        srcs = {}
        for r in rs:
            srcs[r["src"]] = srcs.get(r["src"], 0) + 1
        print("  %-100s %.4f %5d  %5.1f %5.1f %5.1f  %s" % (c, len(rs) / nfr, len(rs), *d, dict(sorted(srcs.items(), key=lambda kv: -kv[1])[:3])))
    # ---- changes of the lead's track id
    print("lead id changes by cause (consecutive lead frames, or the lead back within 2 s with another id):")
    ch = {}
    prev = None
    for r in rows:
        if not r["present"]:
            continue
        if prev is not None and r["id"] != prev["id"] and r["t"] - prev["t"] <= 2.0:
            a_, b_ = (prev["kind"], prev["oid"]), (r["kind"], r["oid"])
            if a_ == b_ and a_[0] == "vehicle":
                held = prev["id"] in r["ids_alive"]
                c = "same injected vehicle, new id (tracker): old track %s, camera event %s, new track sources %s" % (
                    "still alive" if held else "gone", r["ev"], "+".join(s.replace("radar_front", "radar") for s in r["sources"]))
            elif a_[0] == "vehicle" and b_[0] == "vehicle":
                c = "another injected vehicle (a camera track the joining did not link)"
            elif a_ == b_:
                c = "same %s object, new id" % a_[0]
            else:
                c = "%s -> %s" % (a_[0], b_[0])
            ch.setdefault(c, []).append((prev, r))
        prev = r
    for c, v in sorted(ch.items(), key=lambda kv: -len(kv[1])):
        print("  %-120s %4d" % (c, len(v)))
    per = {}
    for r in pres:
        if r["kind"] == "vehicle":
            per.setdefault(r["oid"], set()).add(r["id"])
    if per:
        n = np.array([len(s) for s in per.values()])
        print("track ids per injected vehicle (lead frames on it): vehicles %d, ids total %d, mean %.2f, max %d" % (
            len(per), n.sum(), n.mean(), n.max()))


# ---- sim -----------------------------------------------------------------------------------------------------------
def cmd_sim(a):
    from tests.simrun import Run
    from tests.tracks_eval import per_vehicle
    from chris_integrator.sim import default_sensors
    from chris_integrator.sim.radar import RadarSim
    from chris_integrator.sim.camera2 import CameraObjectsSim
    true = dict(vehicle=dict(gyro_bias=0.004, gyro_scale=0.04, accel_bias=0.15, silences=[(80.0, 87.0)]),
                vision=dict(mount_yaw=-0.06, mount_pitch=0.05, speed_scale=0.03, range_scale=1.08), gnss=dict(lag=2.2))
    print("sim %s, radar (misaligned 1.0 deg) + second camera; the same set-up as integrator_eval.py sim --radar --cam2" % a.scenario)
    for seed in a.seeds:
        tk = {k: dict(v, seed=100 * seed + i) for i, (k, v) in enumerate(dict(true, map={}).items())}
        sens = default_sensors(**tk)
        sens.append(RadarSim(misalign_deg=1.0, seed=100 * seed + 7))
        sens.append(CameraObjectsSim(seed=100 * seed + 8))
        cfg = {"gnss_absolute": True, "sources": RADAR + [{"kind": "camera_objects", "id": "cam_wide"}]}
        r = Run(sensors=sens, seed=seed, scenario=a.scenario, speed_scale=0.015, integ=Integrator(cfg), keep_tracks=True)
        w = r.w
        rows = [(t, ld, tl) for t, ld, tl in r.leads if t >= 5.0]
        tmap = {round(t, 4): tr for t, tr in r.tracks}
        both = [(ld["dist_m"] - tl["x"], ld["v_rel_mps"] - (tl["vx"] - w.truth_at(t + w.t0)["v"])) for t, ld, tl in rows
                if ld["present"] and tl is not None]
        nt = sum(1 for _, _, tl in rows if tl is not None)
        false = [(t, ld) for t, ld, tl in rows if ld["present"] and tl is None]
        # what a lead is on: the nearest true object
        on = {"vehicle": 0, "static": 0, "ghost": 0, "none": 0}
        for t, ld in false:
            objs = w.targets_at(t + w.t0)
            st = w.statics_at(t + w.t0)
            x, y = ld["dist_m"], -ld["lateral_m"]
            if any(abs(x - o["x"]) < 4 and abs(y - o["y"]) < 2.5 for o in objs):
                on["vehicle"] += 1
            elif any(abs(x - o["x"]) < 4 and abs(y - o["y"]) < 2.5 for o in st):
                on["static"] += 1
            elif any(abs(x - 2 * o["x"]) < 8 and abs(o["y"]) < 1.5 for o in objs):
                on["ghost"] += 1
            else:
                on["none"] += 1
        ghost_lead = 0
        for t, ld, tl in rows:
            if ld["present"]:
                k = next((k for k in tmap.get(round(t, 4), {"tracks": []})["tracks"] if k["id"] == ld["track_id"]), None)
                if k is not None and "ghost" in k["flags"]:
                    ghost_lead += 1
        # id switches of the lead against the true lead vehicle
        seq = [(tl["id"], ld["track_id"]) for t, ld, tl in rows if ld["present"] and tl is not None]
        sw = sum(1 for p, q in zip(seq, seq[1:]) if p[0] == q[0] and p[1] != q[1])
        print("seed %d | lead: tracked %.3f, false %.4f (on vehicle off the truth's lead %d, static %d, ghost %d, nothing %d), "
              "ghost-flagged lead %d, rms %.2f m %.2f m/s, ids %d, id switches on one true lead %d" % (
                  seed, len(both) / max(nt, 1), len(false) / len(rows), on["vehicle"], on["static"], on["ghost"], on["none"],
                  ghost_lead, np.sqrt(np.mean([d * d for d, _ in both])), np.sqrt(np.mean([v * v for _, v in both])),
                  len({ld["track_id"] for _, ld, _ in rows if ld["present"]}), sw))
        for label, pred in (("in lane", lambda o: abs(o["d"]) < 1.5 and 3 < o["x"] < 100),
                            ("next lanes", lambda o: abs(o["d"]) > 2 and 3 < o["x"] < 100)):
            pv = per_vehicle(r, pred, gate=(3.5, 1.6))
            print("       vehicles %-10s: n %d tracked %.3f duplicate %.3f id switches %d rms x %.2f y %.2f" % (
                label, pv["n"], pv["tracked"], pv["dup"], pv["switches"], pv["rms_x"], pv["rms_y"]))
        conf = [k for t, tr in r.tracks[200:] for k in tr["tracks"] if k["state"] != "tentative"]
        n_fr = len(r.tracks[200:])
        nf = 0
        for t, tr in r.tracks[200::3]:
            objs = w.targets_at(t + w.t0) + w.statics_at(t + w.t0)
            for k in tr["tracks"]:
                if k["state"] == "tentative" or "ghost" in k["flags"]:
                    continue
                if not any(abs(k["x_m"] - o["x"]) < 4.0 and abs(k["y_m"] - o["y"]) < 2.5 for o in objs):
                    nf += 1
        print("       confirmed tracks per frame %.2f; ghost-flagged per frame %.3f; confirmed on no true object %.4f per frame" % (
            len(conf) / n_fr, sum(1 for k in conf if "ghost" in k["flags"]) / n_fr, nf / len(r.tracks[200::3])))


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd", required=True)
    p = sub.add_parser("inject")
    p.add_argument("session")
    p.add_argument("--pos")
    p.add_argument("--no-lead", action="store_true")
    p.add_argument("--cam2", action="store_true")
    p.add_argument("--two", action="store_true")
    p.add_argument("--statics", choices=("path", "ahead"), default="path")
    p.add_argument("--no-link", action="store_true")
    p.add_argument("--no-split", action="store_true", help="inject the camera tracks smoothed across their steps (L38)")
    p.add_argument("--frames")
    p.add_argument("--baseline", action="store_true", help="also the recording as it is, against the same pseudo-truth")
    p = sub.add_parser("sim")
    p.add_argument("--scenario", default="town")
    p.add_argument("--seeds", type=int, nargs="+", default=[1, 2])
    a = ap.parse_args()
    globals()["cmd_" + a.cmd](a)


if __name__ == "__main__":
    main()
