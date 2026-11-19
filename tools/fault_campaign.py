#!/usr/bin/env python3
"""The fault campaign of the data integrator: every fault class of the taxonomy injected into the simulator
(known truth) and into real recordings (the unfaulted run of the same recording is the reference), scored on
what the integrator *reported* and on what its outputs *were* meanwhile.

    fault_campaign.py sim  [--radar] [--only SUBSTR] [--seeds 1] [--out table.json] [--tree OTHER_TREE] [-j 6]
    fault_campaign.py real SESSION --pos POS.txt [--from S --to S] [--only SUBSTR] [--out table.json]
    fault_campaign.py healthy [--seeds 1 2 3 4 5 6] [--scenario town highway]     false alarms without a fault
    fault_campaign.py show table.json [other.json]                                the table (or before / after)

Per fault: `det` seconds from onset to the first alarm naming any source (a source `fault` / `suspect` / `stale`
/ `absent`, a diagnostics finding, the mount `moved`), `who` the first source declared faulted or named by a
finding (ok = the injected one), `wrong-valid` seconds during which an output was valid and wrong by more than
its tolerance AND more than 3 of its own stated sigma (the cost that matters: confidently wrong), per quantity,
`max` the largest such error, `rec` seconds from the end of the fault until alarms are gone and outputs are back.
`--tree` runs another checkout's integrator under the same faults (the before column).
"""
import argparse
import importlib.util
import json
import math
import multiprocessing as mp
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
NS = 1e9
T0, T1, T_END = 95.0, 150.0, 240.0          # town: the car moves from 66 to 156 s; recovery is judged to the end
TOL = {"speed": 0.5, "yaw_rate": 0.02, "position": 15.0, "heading": 6.0, "lead": 4.0}     # m/s, rad/s, m, deg, m
ALARM = ("fault", "suspect", "stale", "absent")


def _faults_mod():
    spec = importlib.util.spec_from_file_location("l41_faults", os.path.join(ROOT, "chris-integrator", "chris_integrator", "sim", "faults.py"))
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def catalogue(radar=False):
    """[(label, [fault spec tuples])]: the taxonomy, per source and quantity."""
    W = (T0, T1)
    c = []

    def add(label, *fs):
        c.append((label, [f if len(f) == 5 else f + ({},) for f in fs]))
    S = "vehicle.speed"
    add("vehicle speed: bias +2 m/s", (S, "bias", *W, dict(b=2.0)))
    add("vehicle speed: scale 0.7", (S, "scale", *W, dict(k=0.7)))
    add("vehicle speed: scale 0.9", (S, "scale", *W, dict(k=0.9)))
    add("vehicle speed: slow scale drift to +17 %", (S, "drift", *W, dict(rate=0.003, rel=True)))
    add("vehicle speed: stuck", (S, "stuck", *W))
    add("vehicle speed: each sample repeated 5x", (S, "repeat", *W, dict(n=5)))
    add("vehicle speed: noise burst 1 m/s", (S, "noise", *W, dict(sigma=1.0)))
    add("vehicle speed: outliers 10 % of 5 m/s", (S, "outliers", *W, dict(p=0.1, mag=5.0)))
    add("vehicle speed: unit (km/h read as m/s)", (S, "unit", *W, dict(k=3.6)))
    add("vehicle speed: sign", (S, "sign", *W))
    add("vehicle speed signal only: scale 0.7 (wheels right)", ("vehicle.speed_only", "scale", *W, dict(k=0.7)))
    add("vehicle wheels only: scale 0.8", ("vehicle.wheels", "scale", *W, dict(k=0.8)))
    G = "vehicle.gyro"
    add("gyro: bias 0.02 rad/s", (G, "bias", *W, dict(b=0.02)))
    add("gyro: scale 1.3", (G, "scale", *W, dict(k=1.3)))
    add("gyro: slow bias drift to 0.03 rad/s", (G, "drift", *W, dict(rate=0.00055)))
    add("gyro: stuck", (G, "stuck", *W))
    add("gyro: noise burst 0.03 rad/s", (G, "noise", *W, dict(sigma=0.03)))
    add("gyro: sign", (G, "sign", *W))
    add("gyro: unit (deg/s read as rad/s)", (G, "unit", *W, dict(k=57.3)))
    add("accelerometer: bias 1 m/s2", ("vehicle.accel", "bias", *W, dict(b=1.0)))
    add("accelerometer: sign", ("vehicle.accel", "sign", *W))
    add("accelerometer: stuck", ("vehicle.accel", "stuck", *W))
    F = "vehicle.feed"
    add("vehicle feed: drop-out", (F, "dropout", *W))
    add("vehicle feed: silent 7 s, 3 stale samples on return", (F, "stale", T0, T0 + 7.0, dict(n=3)))
    add("vehicle feed: content 0.5 s old (delay jump)", (F, "delay", *W, dict(d=0.5)))
    add("vehicle feed: stamps 0.5 s late", (F, "stamp", *W, dict(d=0.5)))
    add("vehicle feed: stamps 0.5 s early", (F, "stamp", *W, dict(d=-0.5)))
    V = "vision.speed"
    add("camera speed: scale 0.3", (V, "scale", *W, dict(k=0.3)))
    add("camera speed: scale 0.85", (V, "scale", *W, dict(k=0.85)))
    add("camera speed: bias +1.5 m/s", (V, "bias", *W, dict(b=1.5)))
    add("camera speed: slow scale drift to -17 %", (V, "drift", *W, dict(rate=-0.003, rel=True)))
    add("camera speed: stuck", (V, "stuck", *W))
    add("camera speed: noise burst 2 m/s", (V, "noise", *W, dict(sigma=2.0)))
    add("camera speed: noise 1.5 m/s and stated std x 0.2 (lies)", (V, "noise", *W, dict(sigma=1.5)), ("vision.std", "lie", *W, dict(k=0.2)))
    add("camera yaw rate: sign", ("vision.yaw", "sign", *W))
    add("camera yaw rate: scale 1.5", ("vision.yaw", "scale", *W, dict(k=1.5)))
    add("camera feed: drop-out", ("vision.feed", "dropout", *W))
    add("camera feed: content 0.3 s old (delay jump)", ("vision.feed", "delay", *W, dict(d=0.3)))
    add("camera feed: stamps 0.3 s late", ("vision.feed", "stamp", *W, dict(d=0.3)))
    add("camera mount: pitch step 1.7 deg", ("vision.mount", "mount", T0, 1e9, dict(pitch=0.03)))
    add("camera mount: pitch step 8 deg (model out of range)", ("vision.mount", "mount", T0, 1e9, dict(pitch=0.14, collapse=True)))
    add("camera mount: slow pitch creep 0.03 deg/s", ("vision.mount", "drift", *W, dict(rate=0.0005)))
    add("camera lead: range scale 1.3", ("vision.lead", "scale", *W, dict(k=1.3)))
    add("camera lead: phantom lead at 12 m", ("vision.lead", "ghost", *W, dict(x=12.0)))
    P = "gnss.pos"
    add("receiver: position jump 30 m", (P, "bias", *W, dict(e=21.0, n=21.0)))
    add("receiver: slow position drift 1 m/s", (P, "drift", *W, dict(e=0.7, n=0.7)))
    add("receiver: noise burst 15 m", (P, "noise", *W, dict(sigma=15.0)))
    add("receiver: outliers 20 % of 60 m", (P, "outliers", *W, dict(p=0.2, mag=60.0)))
    add("receiver: frozen fix while moving", (P, "stuck", *W))
    add("receiver: lat / lon swapped (frame)", (P, "frame", *W))
    add("receiver: noise 15 m and stated accuracy x 0.3 (lies)", (P, "noise", *W, dict(sigma=15.0)), ("gnss.hacc", "lie", *W, dict(k=0.3)))
    add("receiver: drop-out", ("gnss.feed", "dropout", *W))
    add("receiver: fixes 2 s older (delay jump)", ("gnss.feed", "delay", *W, dict(d=2.0)))
    add("map: matcher on a parallel road 25 m away", ("map.match", "bias", *W, dict(b=25.0)))
    add("map: bearing turned 90 deg (frame)", ("map.match", "frame", *W, dict(deg=90.0)))
    add("clock: epoch change (all stamps step back 600 s)", ("clock", "epoch", T0, 1e9, dict(back=600.0)))
    add("two together: vehicle and camera speed x 0.7", (S, "scale", *W, dict(k=0.7)), (V, "scale", *W, dict(k=0.7)))
    add("two together: vehicle speed stuck, receiver out", (S, "stuck", *W), ("gnss.feed", "dropout", *W))
    add("two together: camera out of range, receiver jump 30 m", ("vision.mount", "mount", T0, 1e9, dict(pitch=0.14, collapse=True)),
        (P, "bias", *W, dict(e=21.0, n=21.0)))
    if radar:
        add("radar: range bias 3 m", ("radar.range", "bias", *W, dict(b=3.0)))
        add("radar: range scale 1.1", ("radar.range", "scale", *W, dict(k=1.1)))
        add("radar: range rate bias 1 m/s", ("radar.rate", "bias", *W, dict(b=1.0)))
        add("radar: mount knocked 3 deg", ("radar.mount", "bias", T0, 1e9, dict(b=3.0)))
        add("radar: azimuth sign (frame)", ("radar.mount", "frame", *W))
        add("radar: drop-out", ("radar.feed", "dropout", *W))
        add("radar: content 0.2 s old", ("radar.feed", "delay", *W, dict(d=0.2)))
        add("radar: ghosts", ("radar.feed", "ghost", *W, dict(x=25.0)))
    return c


def _alarms(es):
    """{group: (level, text)} the integrator names at this frame."""
    out = {}
    for g, s in es["sources"].items():
        if s["state"] in ALARM:
            out[g] = (s["state"], s.get("reason") or s["state"])
    for d in es.get("diagnostics") or ():
        if d.get("level") in ("fault", "warn") and d.get("source") not in out:
            out[d["source"]] = ("finding", "%s: %s" % (d.get("kind"), d.get("text", "")))
    cv = (es.get("calibration") or {}).get("vision") or {}
    if cv.get("state") in ("moved",) and "vision" not in out:
        out["vision"] = ("finding", "mount moved")
    return out


def _blamed(es):
    """Groups declared faulted (the integrator's isolation), and those a finding at fault level names."""
    b = [g for g, s in es["sources"].items() if s["state"] == "fault"]
    b += [d["source"] for d in es.get("diagnostics") or () if d.get("level") == "fault" and d["source"] not in b]
    return b


def _frame_row(t, es, ld):
    def q(d):
        return (d["value"], d["std"]) if d.get("valid") else None
    pos = es["position"]
    return {"t": t, "al": _alarms(es), "bl": _blamed(es), "speed": q(es["speed"]), "yaw_rate": q(es["yaw_rate"]),
            "pos": (pos["local"]["e_m"], pos["local"]["n_m"], pos["hacc_m"], pos["local"]["frame"]) if pos.get("valid") else None,
            "head": q(es["heading"]), "lead": (ld["dist_m"], ld.get("dist_std_m") or 0.0) if ld and ld.get("present") and ld.get("valid", True) else None,
            "lead_valid": bool(ld and ld.get("valid", True)), "mount": ((es.get("calibration") or {}).get("vision") or {}).get("state"),
            "events": [d for d in es.get("diagnostics") or () if d.get("kind") in ("mount_step",)]}


def _score(rows, ref, faults, groups, t0, t1, quiet_alarms=()):
    """rows: per frame _frame_row + "err": {quantity: (error, sigma)}; ref: None (errors already against truth)."""
    res = {"n": len(rows)}
    pre = set()
    for r in rows:
        if t0 - 20.0 <= r["t"] < t0:
            pre |= set(r["al"])
    det = who = None
    named = []
    for r in rows:
        if r["t"] < t0:
            continue
        new = [g for g in r["al"] if g not in pre or r["al"][g][0] == "fault"]
        if new and det is None:
            det = r["t"] - t0
            res["first"] = {g: r["al"][g][1][:90] for g in new}
        for g in r["bl"]:
            if g not in named:
                named.append(g)
        if who is None and r["bl"]:
            who = list(r["bl"])
    res["det_s"] = None if det is None else round(det, 1)
    res["blamed"] = named
    ok = [g for g in named if g in groups]
    res["isolated"] = "ok" if ok and len(named) == len(ok) else "ok+" if ok else ("none" if not named else "wrong")
    wv, mx = {}, {}
    last = rows[0]["t"]
    for r in rows:
        dt = min(max(r["t"] - last, 0.0), 0.5)
        last = r["t"]
        if not (t0 <= r["t"] < t1 + 10.0):
            continue
        for q, (e, s) in r["err"].items():
            if abs(e) > TOL[q] and abs(e) > 3.0 * s:
                wv[q] = wv.get(q, 0.0) + dt
                mx[q] = max(mx.get(q, 0.0), abs(e))
    res["wrong_valid_s"] = {q: round(v, 1) for q, v in wv.items()}
    res["max_err"] = {q: round(v, 2) for q, v in mx.items()}
    # recovery: the last frame after the fault's end with an alarm that was not there before, or an output wrong
    bad_t = None
    for r in rows:
        if r["t"] < t1:
            continue
        if any(g not in pre for g in r["al"]) or any(abs(e) > TOL[q] and abs(e) > 3.0 * s for q, (e, s) in r["err"].items()):
            bad_t = r["t"]
    end = rows[-1]["t"]
    res["recovery_s"] = 0.0 if bad_t is None else (None if end - bad_t < 5.0 else round(bad_t - t1, 1))
    ev = [e for r in rows for e in r.get("events", ())]
    if ev:
        res["event"] = ev[-1].get("text")
    return res


# ---- simulation ----------------------------------------------------------------------------------------------------
def _sim_stream(seed, scenario, radar):
    from chris_integrator.sim import World, default_sensors
    true = dict(vehicle=dict(gyro_bias=0.004, gyro_scale=0.04, accel_bias=0.15), vision=dict(mount_yaw=-0.06, mount_pitch=0.05, speed_scale=0.03),
                gnss=dict(lag=2.2))
    tk = {k: dict(v, seed=100 * seed + i) for i, (k, v) in enumerate(dict(true, map={}).items())}
    w = World(seed=seed, scenario=scenario, speed_scale=0.015)
    sens = default_sensors(**tk)
    if radar:
        from chris_integrator.sim.radar import RadarSim
        sens.append(RadarSim(misalign_deg=1.0, seed=100 * seed + 7))
    return w, [m for _, m in w.messages(sens)]


def _run_sim(job):
    label, specs, seed, scenario, radar, tree = job
    sys.path.insert(0, os.path.join(tree or ROOT, "chris-integrator"))
    from chris_integrator import Integrator
    from chris_integrator.core import ego as E
    from chris_integrator.sim.world import geo
    FM = _faults_mod()
    w, msgs = _sim_stream(seed, scenario, radar)
    faults = [FM.Fault(s[0], s[1], s[2], s[3], **s[4]) for s in specs]
    cfg = {"gnss_absolute": True}
    if radar:
        cfg["sources"] = [{"kind": "radar", "id": "*"}]
    I = Integrator(cfg)
    rows = []
    shift = 0.0                                              # an epoch fault moves the stamps: truth follows
    for m in FM.apply(msgs, faults):
        if m["type"] != "road":
            I.on_message(m)
            continue
        es, tr, ld = I.frame(m)
        if es is None:
            continue
        T = m["ts_mono_ns"] / NS
        for f in faults:
            if f.kind == "epoch" and T < w.t0 - 1.0:
                shift = f.p.get("back", 600.0)
        tt = T + shift
        tru = w.truth_at(tt)
        r = _frame_row(tt - w.t0, es, ld)
        err = {}
        if r["speed"]:
            err["speed"] = (r["speed"][0] - 1.015 * tru["v"], r["speed"][1])
        if r["yaw_rate"]:
            err["yaw_rate"] = (r["yaw_rate"][0] - tru["w"], r["yaw_rate"][1])
        if r["pos"] and I.origin is not None:
            oe, on = E.to_local(I.origin, *geo(0.0, 0.0))
            err["position"] = (math.hypot(r["pos"][0] - tru["pe"] - oe, r["pos"][1] - tru["pn"] - on), r["pos"][2])
        if r["head"]:
            d = (r["head"][0] - (90.0 - math.degrees(tru["psi"])) + 180.0) % 360.0 - 180.0
            err["heading"] = (d, r["head"][1])
        tl = w.lead_at(tt)
        if r["lead"] and tl is not None:
            err["lead"] = (r["lead"][0] - tl["x"], max(r["lead"][1], 0.08 * tl["x"]))
        elif r["lead"] and tl is None:
            err["lead"] = (r["lead"][0], 0.0)               # a lead that is not there
        r["err"] = err
        rows.append(r)
    groups = {f.group for f in faults if f.group} | ({"wheels", "vehicle"} if any(f.group in ("wheels",) for f in faults) else set())
    if any(f.target == "clock" for f in faults):
        groups = set()
    res = _score(rows, None, faults, groups, T0, min(max(f.t1 for f in faults), T_END))
    res.update(label=label, seed=seed, target=sorted(groups))
    return res


def cmd_sim(a):
    cat = [c for c in catalogue(a.radar) if not a.only or a.only.lower() in c[0].lower()]
    jobs = [(label, specs, seed, a.scenario, a.radar, a.tree) for label, specs in cat for seed in a.seeds]
    with mp.Pool(a.jobs) as pool:
        out = pool.map(_run_sim, jobs, chunksize=1)
    _save(a, out, dict(mode="sim", radar=a.radar, scenario=a.scenario, tree=a.tree or "this"))


def _healthy(job):
    seed, scenario, radar, tree = job
    r = _run_sim(("healthy", [], seed, scenario, radar, tree)) if False else None
    sys.path.insert(0, os.path.join(tree or ROOT, "chris-integrator"))
    from chris_integrator import Integrator
    w, msgs = _sim_stream(seed, scenario, radar)
    cfg = {"gnss_absolute": True}
    if radar:
        cfg["sources"] = [{"kind": "radar", "id": "*"}]
    I = Integrator(cfg)
    n = 0
    al = {}
    ep = {}
    prev = set()
    for m in msgs:
        if m["type"] != "road":
            I.on_message(m)
            continue
        es, tr, ld = I.frame(m)
        t = m["ts_mono_ns"] / NS - w.t0
        if t < 20.0:
            continue
        n += 1
        cur = _alarms(es)
        for g, (lvl, txt) in cur.items():
            k = "%s:%s" % (g, "fault" if g in _blamed(es) else lvl)
            al[k] = al.get(k, 0) + 1
            if g not in prev:
                ep[k] = ep.get(k, 0) + 1
        prev = set(cur)
    return {"seed": seed, "scenario": scenario, "frames": n, "seconds": w.T - 20.0, "alarm_frames": al, "episodes": ep}


def cmd_healthy(a):
    jobs = [(s, sc, a.radar, a.tree) for sc in a.scenario for s in a.seeds]
    with mp.Pool(a.jobs) as pool:
        out = pool.map(_healthy, jobs, chunksize=1)
    tot = sum(r["seconds"] for r in out)
    print("healthy runs: %d, %.0f s in all (after the first 20 s of each)" % (len(out), tot))
    agg, epi = {}, {}
    for r in out:
        for k, v in r["alarm_frames"].items():
            agg[k] = agg.get(k, 0) + v / r["frames"] * r["seconds"]
        for k, v in r["episodes"].items():
            epi[k] = epi.get(k, 0) + v
    for k in sorted(agg):
        print("  %-28s %6.1f s (%.2f %% of the time), %d episodes = %.1f per hour" % (k, agg[k], 100.0 * agg[k] / tot, epi[k], epi[k] * 3600.0 / tot))
    if not agg:
        print("  no alarm of any kind")
    if a.out:
        json.dump({"meta": dict(mode="healthy", tree=a.tree or "this"), "runs": out}, open(a.out, "w"), indent=1)


# ---- real recordings -----------------------------------------------------------------------------------------------
def _run_real(job):
    label, specs, path, pos, t_from, t_to, tree, f0, f1 = job
    sys.path.insert(0, HERE)
    sys.path.insert(0, os.path.join(tree or ROOT, "chris-integrator"))
    import replay_lib as rl
    from chris_integrator import Integrator
    FM = _faults_mod()
    msgs = [m for m in rl.read_session(path, road_model="vision", pos=rl.load_pos(pos) if pos else None)
            if m.get("type") in ("ego", "vehicle", "gnss", "map", "calib", "objects", "road") and isinstance(m.get("ts_mono_ns"), int)]
    ts0 = msgs[0]["ts_mono_ns"] / NS
    msgs = [m for m in msgs if t_from <= m["ts_mono_ns"] / NS - ts0 < t_to]

    def run(faults):
        I = Integrator()
        rows = []
        for m in FM.apply(msgs, faults):
            if m["type"] != "road":
                I.on_message(m)
                continue
            es, tr, ld = I.frame(m)
            if es is not None:
                rows.append(_frame_row(m["ts_mono_ns"] / NS - ts0 - t_from, es, ld))
        return rows
    faults = [FM.Fault(s[0], s[1], f0 if s[2] == T0 else s[2] - T0 + f0, f1 if s[3] == T1 else (s[3] if s[3] > 1e8 else s[3] - T0 + f0), **s[4]) for s in specs]
    ref = run([])
    rows = run(faults)
    by_t = {round(r["t"], 3): r for r in ref}
    for r in rows:
        o = by_t.get(round(r["t"], 3))
        err = {}
        if o is not None:
            for q in ("speed", "yaw_rate"):
                if r[q] and o[q]:
                    err[q] = (r[q][0] - o[q][0], r[q][1])
            if r["pos"] and o["pos"] and r["pos"][3] == o["pos"][3]:
                err["position"] = (math.hypot(r["pos"][0] - o["pos"][0], r["pos"][1] - o["pos"][1]), r["pos"][2])
            if r["head"] and o["head"]:
                err["heading"] = ((r["head"][0] - o["head"][0] + 180.0) % 360.0 - 180.0, r["head"][1])
            if r["lead"] and o["lead"]:
                err["lead"] = (r["lead"][0] - o["lead"][0], max(r["lead"][1], 0.08 * o["lead"][0]))
            elif r["lead"] and not o["lead"] and o["lead_valid"]:
                err["lead"] = (r["lead"][0], 0.0)
        r["err"] = err
    groups = {f.group for f in faults if f.group} | ({"wheels", "vehicle"} if any(f.group == "wheels" for f in faults) else set())
    if any(f.target == "clock" for f in faults):
        groups = set()
    res = _score(rows, ref, faults, groups, f0, f1)
    res.update(label=label, target=sorted(groups), session=os.path.basename(os.path.normpath(path)))
    # the reference run's own alarms in the window: what "healthy" looks like on this slice
    res["ref_alarms"] = sorted({g for r in ref if f0 <= r["t"] < f1 + 10 for g in r["al"]})
    return res


def cmd_real(a):
    cat = [c for c in catalogue(False) if not a.only or a.only.lower() in c[0].lower()]
    jobs = [(label, specs, a.session, a.pos, a.t_from, a.t_to, a.tree, a.f0, a.f1) for label, specs in cat]
    with mp.Pool(a.jobs) as pool:
        out = pool.map(_run_real, jobs, chunksize=1)
    _save(a, out, dict(mode="real", session=os.path.basename(os.path.normpath(a.session)), window=[a.t_from, a.t_to], fault=[a.f0, a.f1], tree=a.tree or "this"))


# ---- table ---------------------------------------------------------------------------------------------------------
def _cell(r):
    wv = r["wrong_valid_s"]
    return "%s | %s | %s | %s | %s" % (
        "-" if r["det_s"] is None else "%.1f" % r["det_s"],
        {"ok": "ok", "ok+": "ok + " + ",".join(g for g in r["blamed"] if g not in r["target"]), "none": "none",
         "wrong": "WRONG: " + ",".join(r["blamed"])}[r["isolated"]],
        ", ".join("%s %.0f s" % (q, v) for q, v in sorted(wv.items()) if v >= 0.5) or "0",
        ", ".join("%s %.3g" % (q, v) for q, v in sorted(r["max_err"].items())) or "-",
        "-" if r["recovery_s"] is None else "%.0f" % r["recovery_s"])


def table(runs, other=None):
    by = {}
    for r in runs:
        by.setdefault(r["label"], []).append(r)
    ob = {}
    for r in other or ():
        ob.setdefault(r["label"], []).append(r)
    lines = []
    if other:
        lines.append("| fault | before: detected after (s) | isolated | wrong-but-valid | after: detected after (s) | isolated | wrong-but-valid | largest such error | recovered after (s) |")
        lines.append("|---|---|---|---|---|---|---|---|---|")
    else:
        lines.append("| fault | detected after (s) | isolated | wrong-but-valid | largest such error | recovered after (s) |")
        lines.append("|---|---|---|---|---|---|")
    for label, rs in by.items():
        for r in rs:
            if other:
                o = (ob.get(label) or [None])[0]
                oc = " | ".join(_cell(o).split(" | ")[:3]) if o else "| |"
                lines.append("| %s | %s | %s |" % (label, oc, _cell(r)))
            else:
                lines.append("| %s | %s |" % (label, _cell(r)))
    return "\n".join(lines)


def summary(runs):
    n = len(runs)
    det = sum(1 for r in runs if r["det_s"] is not None)
    ok = sum(1 for r in runs if r["isolated"] in ("ok", "ok+"))
    wrong = sum(1 for r in runs if r["isolated"] == "wrong")
    clean = sum(1 for r in runs if not any(v >= 0.5 for v in r["wrong_valid_s"].values()))
    wv = sum(sum(r["wrong_valid_s"].values()) for r in runs)
    return "%d faults: detected %d, isolated correctly %d, wrong source blamed %d, no output confidently wrong %d; confidently-wrong output time in all %.0f s" % (
        n, det, ok, wrong, clean, wv)


def _save(a, out, meta):
    print(table(out))
    print(summary(out))
    if a.out:
        json.dump({"meta": meta, "runs": out}, open(a.out, "w"), indent=1)


def cmd_show(a):
    d = json.load(open(a.table))
    o = json.load(open(a.other)) if a.other else None
    if o:
        print(table(o["runs"], d["runs"]))
        print("before: " + summary(d["runs"]))
        print("after:  " + summary(o["runs"]))
    else:
        print(table(d["runs"]))
        print(summary(d["runs"]))


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd", required=True)
    for name in ("sim", "healthy", "real"):
        p = sub.add_parser(name)
        p.add_argument("--tree")
        p.add_argument("--out")
        p.add_argument("-j", "--jobs", type=int, default=6)
        p.add_argument("--radar", action="store_true")
        p.add_argument("--only")
        if name == "sim":
            p.add_argument("--seeds", type=int, nargs="+", default=[1])
            p.add_argument("--scenario", default="town")
        elif name == "healthy":
            p.add_argument("--seeds", type=int, nargs="+", default=[1, 2, 3, 4, 5, 6])
            p.add_argument("--scenario", nargs="+", default=["town", "highway"])
        else:
            p.add_argument("session")
            p.add_argument("--pos")
            p.add_argument("--from", dest="t_from", type=float, default=0.0)
            p.add_argument("--to", dest="t_to", type=float, default=1e9)
            p.add_argument("--f0", type=float, default=95.0, help="fault onset, s into the slice")
            p.add_argument("--f1", type=float, default=150.0)
    p = sub.add_parser("show")
    p.add_argument("table")
    p.add_argument("other", nargs="?")
    a = ap.parse_args()
    globals()["cmd_" + a.cmd](a)


if __name__ == "__main__":
    main()
