#!/usr/bin/env python3
"""Evaluation of the data integrator (docs/INTEGRATOR.md section 10; results in docs/lanes/L38-integrator.md).

    integrator_eval.py sim [--scenario town|highway] [--seeds 1 2 3] [--radar] [--cam2]
        ego accuracy, NEES and coverage, calibration states and object tracks against simulated truth
    integrator_eval.py real SESSION... [--pos POS.txt] [--no-map]
        the estimator on recordings: source states, innovation consistency per row class, position against
        receiver fixes held out (every second one), calibration states, cost
    integrator_eval.py heldout SESSION...
        the vehicle feed held out entirely: speed and yaw rate from the camera against the recorded vehicle
    integrator_eval.py inject SESSION [--pos POS.txt] [--fault T0 T1 FACTOR] [--no-lead] [--cam2] [--two]
        synthetic radar(s) / second camera injected into the recording: isolation, lead with and without them
    integrator_eval.py gnss SESSION... --pos POS.txt
        the receiver against the odometer: repeats, distance over long chords, displacement speed, speed field
    integrator_eval.py cost SESSION [--pos POS.txt] [--radar]
        CPU per message class and peak memory of the integrator alone

Sessions: recorder directories; recordings made before `road_model` / `gnss` were recorded get them restored
(read_session(road_model="vision", pos=...)).
"""
import argparse
import math
import os
import resource
import statistics as st
import sys
import time

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(os.path.dirname(HERE), "chris-integrator"))
import replay_lib as rl  # noqa: E402
from chris_integrator import Integrator  # noqa: E402
from chris_integrator.core import ego as E  # noqa: E402

NS = 1e9
INPUTS = ("ego", "vehicle", "gnss", "map", "calib", "objects")
RADAR = [{"kind": "radar", "id": "*"}]


def mad(x):
    x = np.asarray(x, float)
    return 1.4826 * float(np.median(np.abs(x - np.median(x)))) if len(x) else float("nan")


def name_of(p):
    return os.path.basename(os.path.normpath(p)).replace("20261119-", "")


def session(path, a):
    pos = rl.load_pos(a.pos) if getattr(a, "pos", None) else None
    return rl.read_session(path, road_model="vision", pos=pos)


class Veh:
    def __init__(self, msgs=None):
        self.t, self.v, self.w = [], [], []
        for m in msgs or ():
            self.add(m)

    def add(self, m):
        if m.get("type") == "ego" and isinstance(m.get("speed_mps"), (int, float)):
            self.t.append(m["ts_mono_ns"] / NS), self.v.append(m["speed_mps"]), self.w.append(m.get("yaw_rate_rps") or 0.0)

    def at(self, t, max_gap=0.3):
        import bisect
        i = bisect.bisect_left(self.t, t)
        if i == 0 or i >= len(self.t) or self.t[i] - self.t[i - 1] > max_gap:
            return None
        u = (t - self.t[i - 1]) / (self.t[i] - self.t[i - 1])
        return self.v[i - 1] + u * (self.v[i] - self.v[i - 1]), self.w[i - 1] + u * (self.w[i] - self.w[i - 1])


# ---- sim ---------------------------------------------------------------------------------------------------------
def cmd_sim(a):
    sys.path.insert(0, os.path.join(os.path.dirname(HERE), "chris-integrator"))
    from tests.simrun import Run
    from tests.tracks_eval import per_vehicle
    from chris_integrator.sim import default_sensors
    from chris_integrator.sim.radar import RadarSim
    from chris_integrator.sim.camera2 import CameraObjectsSim
    true = dict(vehicle=dict(gyro_bias=0.004, gyro_scale=0.04, accel_bias=0.15, silences=[(80.0, 87.0)]),
                vision=dict(mount_yaw=-0.06, mount_pitch=0.05, speed_scale=0.03, range_scale=1.08), gnss=dict(lag=2.2))
    print("scenario %s; truth: speed scale 0.015, gyro bias 0.004 scale 0.04, accel bias 0.15, camera mount yaw -0.06 pitch 0.05 "
          "scale 0.03 delay 0.125 range scale 1.08, receiver lag 2.2%s" % (a.scenario, ", radar misaligned 1.0 deg" if a.radar else ""))
    print("seed | speed rms NEES cover2s | yaw rate rms NEES | heading rms(deg) NEES | position rms(m) NEES(2 dof) | grade NEES |"
          " lead: tracked false rms(m) rms v(m/s) ids")
    cal = {}
    for seed in a.seeds:
        sens = default_sensors(**true)
        cfg = {}
        src = []
        if a.radar:
            sens.append(RadarSim(misalign_deg=1.0))
            src += RADAR
        if a.cam2:
            sens.append(CameraObjectsSim())
            src.append({"kind": "camera_objects", "id": "cam_wide"})
        if src:
            cfg["sources"] = src
        r = Run(sensors=sens, seed=seed, scenario=a.scenario, speed_scale=0.015, integ=Integrator(cfg), keep_tracks=True)
        t0 = 30.0
        rows = [(t, ld, tl) for t, ld, tl in r.leads if t >= 5.0]
        both = [(ld["dist_m"] - tl["x"], ld["v_rel_mps"] - (tl["vx"] - r.w.truth_at(t + r.w.t0)["v"])) for t, ld, tl in rows
                if ld["present"] and tl is not None]
        nt = sum(1 for _, _, tl in rows if tl is not None)
        print("%4d | %.3f %.2f %.2f | %.4f %.2f | %.2f %.2f | %.2f %.2f | %.2f | %.3f %.3f %.2f %.2f %d" % (
            seed, np.sqrt(np.mean(r.col("dv", t0) ** 2)), r.nees("dv", "sv", t0), np.mean(np.abs(r.col("dv", t0)) <= 2 * r.col("sv", t0)),
            np.sqrt(np.mean(r.col("dw", t0) ** 2)), r.nees("dw", "sw", t0),
            math.degrees(np.sqrt(np.mean(r.col("dpsi", t0) ** 2))), r.nees("dpsi", "spsi", t0),
            np.sqrt(np.mean(r.col("along", t0) ** 2 + r.col("cross", t0) ** 2)), np.mean(r.col("nees_pos", t0)),
            r.nees("dth", "sth", t0), len(both) / max(nt, 1),
            sum(1 for _, ld, tl in rows if ld["present"] and tl is None) / len(rows),
            np.sqrt(np.mean([d * d for d, _ in both])), np.sqrt(np.mean([v * v for _, v in both])),
            len({ld["track_id"] for _, ld, _ in rows if ld["present"]})))
        for s, d in r.es["calibration"].items():
            for k, v in d.items():
                if isinstance(v, dict) and "value" in v:
                    cal.setdefault("%s.%s" % (s, k), []).append((v["value"], v["std"]))
        if a.radar or a.cam2:
            for label, pred in (("in lane", lambda o: abs(o["d"]) < 1.5 and 3 < o["x"] < 100), ("next lanes", lambda o: abs(o["d"]) > 2 and 3 < o["x"] < (55 if not a.radar else 100))):
                pv = per_vehicle(r, pred, gate=(3.5 if a.radar else 8.0, 1.6))
                print("       tracks, vehicles %s: n %d tracked %.3f duplicate %.3f id switches %d rms x %.2f y %.2f" % (
                    label, pv["n"], pv["tracked"], pv["dup"], pv["switches"], pv["rms_x"], pv["rms_y"]))
            conf = [k for t, tr in r.tracks[200:] for k in tr["tracks"] if k["state"] != "tentative"]
            n_fr = len(r.tracks[200:])
            w = r.w
            false = 0
            for t, tr in r.tracks[200::3]:
                objs = w.targets_at(t + w.t0) + w.statics_at(t + w.t0)
                for k in tr["tracks"]:
                    if k["state"] == "tentative" or "ghost" in k["flags"]:
                        continue
                    if not any(abs(k["x_m"] - o["x"]) < 4.0 and abs(k["y_m"] - o["y"]) < 2.5 for o in objs):
                        false += 1
            print("       confirmed tracks per frame %.1f; flagged ghost %.2f per frame; confirmed tracks on no true object %.3f per frame; sensors %s" % (
                len(conf) / n_fr, sum(1 for k in conf if "ghost" in k["flags"]) / n_fr, false / len(r.tracks[200::3]),
                {s: {q: v.get(q) for q in ("scans", "assigned", "started", "nis")} for s, v in r.tracks[-1][1]["sensors"].items()}))
    print("calibration states at the end (value +- std per seed):")
    for k, v in cal.items():
        print("  %-22s %s" % (k, "  ".join("%+.4f +- %.4f" % x for x in v)))


# ---- real ----------------------------------------------------------------------------------------------------------
def cmd_real(a):
    for path in a.sessions:
        msgs = [m for m in session(path, a) if m.get("type") in INPUTS + ("road",)]
        veh = Veh(msgs)
        srcs = None
        if a.no_map:
            from chris_integrator.sources import default_sources
            srcs = [s for s in default_sources() if s.id != "map"]
        I = Integrator(sources=srcs)
        inn = {}
        I.ekf.stat = lambda row, d2, t, f=I._nis_stat: (f(row, d2, t), inn.setdefault((row.group, row.q), []).append(d2))
        states, held, n, cpu = {}, [], 0, 0.0
        pos_valid = pos_fused = 0
        k_g = 0
        last = None
        moving = 0
        for m in msgs:
            t = m["type"]
            c = time.process_time()
            if t == "road":
                es, tr, ld = I.frame(m)
                cpu += time.process_time() - c
                n += 1
                last = es
                vv = veh.at(m["ts_mono_ns"] / NS)
                mv = vv is not None and vv[0] >= 3.0
                moving += mv
                for g, s in es["sources"].items():
                    d = states.setdefault(g, {})
                    d[s["state"]] = d.get(s["state"], 0) + 1
                    if mv:
                        d["_mv_" + s["state"]] = d.get("_mv_" + s["state"], 0) + 1
                pos_valid += es["position"]["valid"]
                pos_fused += es["position"]["valid"] and es["position"]["mode"] == "fused"
                continue
            if t == "gnss" and m.get("fix") in ("2d", "3d"):
                k_g += 1
                if k_g % 2 == 0 and I.anchored:          # held out: compared with the estimate, never fused
                    sp = I.speed_now()
                    if sp is not None and sp[0] > 3.0:
                        itau = I.model.index["gnss.tau"]
                        T = m["ts_mono_ns"] / NS - float(I.x_now[itau])
                        sm = I.ekf.at(T)
                        if sm is not None:
                            x, P = sm
                            e, nn = E.to_local(I.origin, m["lat"], m["lon"])
                            de, dn = e - x[E.PE], nn - x[E.PN]
                            sig = max(float(m.get("hacc_m") or 10.0), 3.0) * (last["sources"]["gnss"].get("noise_scale") or 1.0)
                            S = P[:2, :2] + np.eye(2) * (sig ** 2 + 2.5 ** 2)
                            c_, s_ = math.cos(x[E.PSI]), math.sin(x[E.PSI])
                            held.append((math.hypot(de, dn), float(np.array([de, dn]) @ np.linalg.solve(S, np.array([de, dn]))),
                                         c_ * de + s_ * dn, -s_ * de + c_ * dn, math.sqrt(max(P[0, 0] + P[1, 1], 0.0))))
                    continue
            I.on_message(m)
            cpu += time.process_time() - c
        print("== %s: %d road frames (%d with the vehicle >= 3 m/s), integrator CPU %.3f ms per road frame (all inputs), re-run entries %d"
              % (name_of(path), n, moving, 1e3 * cpu / max(n, 1), I.ekf.reruns))
        for g, d in states.items():
            tot = sum(v for k, v in d.items() if not k.startswith("_"))
            print("   source %-12s %s" % (g, "  ".join("%s %.1f %%" % (k, 100.0 * v / tot) for k, v in sorted(d.items()) if not k.startswith("_"))
                                           + ("   | moving: " + "  ".join("%s %.1f %%" % (k[4:], 100.0 * v / max(moving, 1)) for k, v in sorted(d.items()) if k.startswith("_mv_")) if moving else "")))
        print("   innovation consistency (first-pass normalised innovations squared; median / 0.455 = 1 when the stated uncertainty is right):")
        for (g, q), v in sorted(inn.items()):
            if len(v) >= 30 and not q.startswith("standstill"):
                v = np.array(v)
                print("     %-8s %-12s n %6d  median/0.455 %5.2f  mean %6.2f  within 2 sigma %.3f" % (g, q, len(v), np.median(v) / 0.4549, v.mean(), np.mean(v <= 4.0)))
        print("   position valid %.1f %% of frames (fused %.1f %%); anchored %s" % (100.0 * pos_valid / max(n, 1), 100.0 * pos_fused / max(n, 1), I.anchored))
        if held:
            h = np.array(held)
            print("   receiver fixes held out (every second one, car moving): n %d, distance to the estimate p50 %.1f p90 %.1f m; normalised (2 dof, expect 2): mean %.2f, "
                  "within 95 %% ellipse %.3f; along-track median %+.1f m, cross-track median %+.1f m; estimator's own position std p50 %.1f m" % (
                      len(h), np.median(h[:, 0]), np.percentile(h[:, 0], 90), h[:, 1].mean(), np.mean(h[:, 1] <= 5.99), np.median(h[:, 2]), np.median(h[:, 3]), np.median(h[:, 4])))
        if last:
            for s, d in last["calibration"].items():
                print("   calibration %-9s %s" % (s, "  ".join("%s %+.4f+-%.4f%s" % (k, v["value"], v["std"], "" if v.get("known") else "(prior)") for k, v in d.items() if isinstance(v, dict) and "value" in v)
                                               + ("  state %s suggested %s" % (d.get("state"), d.get("suggested")) if "state" in d else "")))


# ---- heldout -------------------------------------------------------------------------------------------------------
def cmd_heldout(a):
    from chris_integrator.sources import default_sources
    for path in a.sessions:
        msgs = [m for m in session(path, a) if m.get("type") in ("ego", "road", "calib")]
        veh = Veh(msgs)
        if len(veh.t) < 100:
            print("== %s: no vehicle feed" % name_of(path))
            continue
        I = Integrator(sources=[s for s in default_sources() if s.group in ("vision",)], objects=False)
        ev, ew, raw = [], [], []
        for m in msgs:
            if m["type"] == "ego":
                continue
            if m["type"] != "road":
                I.on_message(m)
                continue
            es = I.on_road(m)
            T = m["ts_mono_ns"] / NS
            v = veh.at(T)
            if v is None or v[0] < 1.0 or not es["speed"]["valid"]:
                continue
            ev.append((es["speed"]["value"] - v[0], es["speed"]["std"]))
            vr = veh.at(T - 0.125)
            if vr is not None:
                raw.append(m["pose"]["vx"] - vr[0])
            if es["yaw_rate"]["valid"]:
                ew.append((es["yaw_rate"]["value"] - v[1], es["yaw_rate"]["std"]))
        e = np.array(ev)
        w = np.array(ew)
        print("== %s: camera only against the recorded vehicle (vehicle >= 1 m/s), n %d: speed error robust sigma %.3f rms %.3f p99 %.2f; "
              "error / stated std rms %.2f, within 2 sigma %.3f; raw camera at its own time: sigma %.3f rms %.3f | yaw rate error sigma %.4f, "
              "error / std rms %.2f | vision state %s" % (
                  name_of(path), len(e), mad(e[:, 0]), np.sqrt(np.mean(e[:, 0] ** 2)), np.percentile(np.abs(e[:, 0]), 99),
                  np.sqrt(np.mean((e[:, 0] / np.maximum(e[:, 1], 1e-3)) ** 2)), np.mean(np.abs(e[:, 0]) <= 2 * e[:, 1]),
                  mad(raw), np.sqrt(np.mean(np.square(raw))), mad(w[:, 0]), np.sqrt(np.mean((w[:, 0] / np.maximum(w[:, 1], 1e-4)) ** 2)),
                  es["sources"]["vision"]["state"]))


# ---- inject --------------------------------------------------------------------------------------------------------
def cmd_inject(a):
    from chris_integrator.sim.inject import inject, lead_truth
    path = a.session
    msgs = [m for m in session(path, a) if m.get("type") in INPUTS + ("road",)]
    veh = Veh(msgs)
    t0 = min(m["ts_mono_ns"] for m in msgs) / NS
    truth = [] if a.no_lead else lead_truth(msgs)

    def truth_at(t):
        import bisect
        for tr in truth:
            if tr[0, 0] <= t <= tr[-1, 0]:
                i = min(bisect.bisect_left(tr[:, 0].tolist(), t), len(tr) - 1)
                if abs(tr[i, 0] - t) < 0.3:
                    return tr[i]
        return None
    radars = [{"sensor": "radar_front", "mount": (2.2, 0.0, 0.0)}]
    if a.two:
        radars.append({"sensor": "radar_fl", "mount": (2.0, 0.8, math.radians(40.0))})
    cam2 = {"sensor": "cam_wide", "mount": (0.0, 0.0, 0.0)} if a.cam2 else None
    fault = tuple(a.fault) if a.fault else None
    runs = {"recording as it is": (msgs, {}),
            "with injected sensors": (list(inject(msgs, radars=radars, camera2=cam2, lead=truth, speed_fault=fault)),
                                      {"sources": RADAR + ([{"kind": "camera_objects", "id": "cam_wide"}] if cam2 else [])})}
    if fault:
        runs = {"speed fault, recording's own sensors": (list(inject(msgs, radars=[], lead=truth, speed_fault=fault)), {}),
                "speed fault, with injected radar": runs["with injected sensors"]}
    print("== %s%s; pseudo-truth lead tracks %d" % (name_of(path), " (vehicle speed x %.2f from %.0f to %.0f s)" % (fault[2], fault[0], fault[1]) if fault else "", len(truth)))
    for label, (ms, cfg) in runs.items():
        I = Integrator(cfg)
        changes, prev, rows = [], None, []
        src_share = {}
        nfr = 0
        tracks_n = []
        for m in ms:
            if m["type"] != "road":
                I.on_message(m)
                continue
            es, tr, ld = I.frame(m)
            T = m["ts_mono_ns"] / NS
            nfr += 1
            stt = {g: (s["state"], s.get("isolation")) for g, s in es["sources"].items() if s["state"] in ("fault",)}
            if stt != prev:
                changes.append((T - t0, dict(stt)))
                prev = stt
            v = veh.at(T)
            tl = truth_at(T)
            rows.append((T - t0, ld, tl, v[0] if v else None, es["speed"]["value"] if es["speed"]["valid"] else None))
            if ld["present"]:
                src_share[ld["source"]] = src_share.get(ld["source"], 0) + 1
            tracks_n.append(sum(1 for k in tr["tracks"] if k["state"] != "tentative"))
        print("-- %s: %d frames" % (label, nfr))
        print("   faults over time: %s" % ("; ".join("%.1f s %s" % (t, {g: v[1] for g, v in s.items()} or "none") for t, s in changes[:12])))
        mv = [r for r in rows if r[3] is not None and r[3] >= 5.0]
        both = [(r[1]["dist_m"] - r[2][1]) for r in rows if r[1]["present"] and r[2] is not None]
        ntr = sum(1 for r in rows if r[2] is not None)
        print("   lead: present %.3f of frames (vehicle >= 5 m/s: %.3f, dist p10/50/90 %s); valid false %.3f; by source %s" % (
            sum(1 for r in rows if r[1]["present"]) / nfr, sum(1 for r in mv if r[1]["present"]) / max(len(mv), 1),
            np.round(np.percentile([r[1]["dist_m"] for r in mv if r[1]["present"]] or [float("nan")], [10, 50, 90]), 1),
            sum(1 for r in rows if not r[1]["valid"]) / nfr, {k: round(v / nfr, 3) for k, v in sorted(src_share.items())}))
        if ntr:
            print("   against the pseudo-truth (%d frames with one): tracked %.3f; where both |diff| p50 %.2f p90 %.2f p99 %.2f m; lead present without one %.4f of frames; ids %d" % (
                ntr, len(both) / ntr, *np.percentile(np.abs(both), [50, 90, 99]),
                sum(1 for r in rows if r[1]["present"] and r[2] is None) / nfr, len({r[1]["track_id"] for r in rows if r[1]["present"]})))
        sp = [(r[4] - r[3]) for r in rows if r[3] is not None and r[4] is not None and r[3] >= 3.0]
        if fault:
            fw = [abs(r[4] - r[3] / 1.0) for r in rows if r[3] is not None and r[4] is not None and fault[0] <= r[0] < fault[1]]
            tw = [r for r in rows if fault[0] <= r[0] < fault[1] and r[4] is not None and r[3] is not None]
            # inside the window the recorded value IS the faulty one: the true speed is recorded / factor
            err = [abs(r[4] - r[3] / fault[2]) for r in tw if r[3] / fault[2] >= 3.0]
            print("   speed in the fault window against the true speed (recorded / factor), true >= 3 m/s: n %d, |error| p50 %.2f p90 %.2f m/s; share of those frames within 1 m/s %.3f" % (
                len(err), *np.percentile(err or [float("nan")], [50, 90]), float(np.mean(np.array(err) < 1.0)) if err else float("nan")))
        print("   confirmed tracks per frame p50 %d p90 %d; sources at the end: %s" % (
            np.median(tracks_n), np.percentile(tracks_n, 90), {g: s["state"] for g, s in es["sources"].items()}))
        for s in ("radar_front", "radar_fl"):
            if s in es["calibration"]:
                c = es["calibration"][s]
                print("   %s: mount yaw error %+.2f +- %.2f deg (injected 0), ego-velocity fit on %.0f %% of scans; speed scale %+.4f +- %.4f" % (
                    s, math.degrees(c["dyaw"]["value"]), math.degrees(c["dyaw"]["std"]), 100 * (es["sources"][s].get("ego_fit_share") or 0),
                    es["calibration"]["odometry"]["speed_scale"]["value"], es["calibration"]["odometry"]["speed_scale"]["std"]))


# ---- gnss ----------------------------------------------------------------------------------------------------------
def cmd_gnss(a):
    for path in a.sessions:
        et, ev, g = [], [], []
        for m in session(path, a):
            if m.get("type") == "ego" and m.get("speed_mps") is not None:
                et.append(m["ts_mono_ns"] / NS), ev.append(m["speed_mps"])
            elif m.get("type") == "gnss" and m.get("lat") is not None:
                g.append(m)
        if len(et) < 100 or len(g) < 50:
            print("== %s: vehicle samples %d, fixes %d: nothing to compare" % (name_of(path), len(et), len(g)))
            continue
        et, ev = np.array(et), np.array(ev)
        d = np.diff(et)
        ok = d < 0.5
        odo = np.concatenate([[0], np.cumsum(np.where(ok, 0.5 * (ev[1:] + ev[:-1]) * d, 0))])
        gaps = np.concatenate([[0], np.cumsum(~ok)])
        gt = np.array([m["ts_mono_ns"] / NS for m in g])
        gv = np.array([m["speed_mps"] if m.get("speed_mps") is not None else np.nan for m in g])
        lat, lon = np.array([m["lat"] for m in g]), np.array([m["lon"] for m in g])
        x = np.radians(lon - lon[0]) * 6371000.0 * math.cos(math.radians(lat[0]))
        y = np.radians(lat - lat[0]) * 6371000.0
        v3 = np.interp(gt - 3.0, et, ev)
        rep = np.concatenate([[False], (lat[1:] == lat[:-1]) & (lon[1:] == lon[:-1])])
        print("== %s: %d fixes, period p50 %.2f s; position repeated: %.2f of all, %.2f while moving (>3 m/s), %.2f while standing" % (
            name_of(path), len(g), np.median(np.diff(gt)), rep.mean(), rep[v3 > 3].mean(), rep[v3 < 0.05].mean() if (v3 < 0.05).any() else float("nan")))
        for L in (100, 300):
            rat = []
            i0 = 0
            for i in range(1, len(g)):
                if gt[i] - gt[i0] > 90:
                    i0 = i
                    continue
                dd = math.hypot(x[i] - x[i0], y[i] - y[i0])
                if dd >= L:
                    o = np.interp(gt[i] - 3, et, odo) - np.interp(gt[i0] - 3, et, odo)
                    if np.interp(gt[i] - 3, et, gaps) == np.interp(gt[i0] - 3, et, gaps) and o > 1:
                        rat.append(dd / o)
                    i0 = i
            if rat:
                print("   chords >= %d m: receiver distance / odometer distance: n %d median %.3f p10 %.3f p90 %.3f" % (L, len(rat), np.median(rat), *np.percentile(rat, [10, 90])))
        dv = np.hypot(np.diff(x), np.diff(y)) / np.maximum(np.diff(gt), 1e-3)
        tm = 0.5 * (gt[1:] + gt[:-1])
        okk = (np.diff(gt) < 1.6) & (np.diff(gt) > 0.5)
        best = None
        for lag in np.arange(0, 9, 0.25):
            v = np.interp(tm - lag, et, ev)
            k = okk & (v > 3)
            r = dv[k] - v[k]
            if best is None or mad(r) < best[0]:
                best = (mad(r), lag, np.median(r), np.median(dv[k] / v[k]), int(k.sum()))
        print("   displacement speed between fixes against the vehicle: best lag %.2f s, sigma %.2f m/s, ratio %.3f (n %d)" % (best[1], best[0], best[3], best[4]))
        lags = []
        for ta in np.arange(gt[0], gt[-1], 60):
            k0 = (gt >= ta) & (gt < ta + 60) & np.isfinite(gv)
            if k0.sum() < 30 or np.interp(gt[k0], et, ev).std() < 1.0:
                continue
            bl = min(((np.mean(np.abs(gv[k0] - np.interp(gt[k0] - lag, et, ev))), lag) for lag in np.arange(0, 9, 0.25)))
            lags.append((bl[1], bl[0], np.median(gv[k0]) / max(np.median(np.interp(gt[k0] - bl[1], et, ev)), 0.1)))
        if lags:
            Lg = np.array(lags)
            print("   speed FIELD per 60 s window (n %d): best lag p10/p50/p90 %s s; field / vehicle ratio p10/p50/p90 %s; field while the car stands p50 %.2f m/s" % (
                len(Lg), np.round(np.percentile(Lg[:, 0], [10, 50, 90]), 2), np.round(np.percentile(Lg[:, 2], [10, 50, 90]), 2),
                np.nanmedian(gv[v3 < 0.05]) if (v3 < 0.05).any() else float("nan")))


# ---- cost ----------------------------------------------------------------------------------------------------------
def cmd_cost(a):
    from chris_integrator.sim.inject import inject
    msgs = [m for m in session(a.session, a) if m.get("type") in INPUTS + ("road",)]
    cfg = {}
    if a.radar:
        msgs = list(inject(msgs, radars=[{"sensor": "radar_front", "mount": (2.2, 0.0, 0.0)}]))
        cfg = {"sources": RADAR}
    r0 = resource.getrusage(resource.RUSAGE_SELF).ru_maxrss
    I = Integrator(cfg)
    cpu, cnt = {}, {}
    t_first = t_last = None
    for m in msgs:
        t = m["type"]
        c = time.process_time()
        if t == "road":
            I.frame(m)
        else:
            I.on_message(m)
        cpu[t] = cpu.get(t, 0.0) + time.process_time() - c
        cnt[t] = cnt.get(t, 0) + 1
        ts = m.get("ts_mono_ns")
        if isinstance(ts, int):
            t_first = ts if t_first is None else min(t_first, ts)
            t_last = ts if t_last is None else max(t_last, ts)
    dur = (t_last - t_first) / NS
    tot = sum(cpu.values())
    print("== %s%s: %.0f s of driving, integrator CPU %.1f s = %.2f %% of one core of this machine; %.3f ms per road frame all inputs included" % (
        name_of(a.session), " + injected radar" if a.radar else "", dur, tot, 100.0 * tot / dur, 1e3 * tot / cnt["road"]))
    for t in sorted(cpu, key=lambda k: -cpu[k]):
        print("   %-8s %6d messages  %.3f ms each  %.1f %% of the time" % (t, cnt[t], 1e3 * cpu[t] / cnt[t], 100.0 * cpu[t] / tot))
    print("   state size %d, buffer entries %d, re-run entries %d; peak resident memory of this process %.0f MB (before the run: %.0f MB)" % (
        I.model.n, len(I.ekf.entries), I.ekf.reruns, resource.getrusage(resource.RUSAGE_SELF).ru_maxrss / 1024.0, r0 / 1024.0))


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd", required=True)
    p = sub.add_parser("sim")
    p.add_argument("--scenario", default="town")
    p.add_argument("--seeds", type=int, nargs="+", default=[1, 2, 3])
    p.add_argument("--radar", action="store_true")
    p.add_argument("--cam2", action="store_true")
    for name in ("real", "heldout", "gnss"):
        p = sub.add_parser(name)
        p.add_argument("sessions", nargs="+")
        p.add_argument("--pos")
        p.add_argument("--no-map", action="store_true")
    p = sub.add_parser("inject")
    p.add_argument("session")
    p.add_argument("--pos")
    p.add_argument("--fault", type=float, nargs=3)
    p.add_argument("--no-lead", action="store_true")
    p.add_argument("--cam2", action="store_true")
    p.add_argument("--two", action="store_true")
    p = sub.add_parser("cost")
    p.add_argument("session")
    p.add_argument("--pos")
    p.add_argument("--radar", action="store_true")
    a = ap.parse_args()
    globals()["cmd_" + a.cmd](a)


if __name__ == "__main__":
    main()
