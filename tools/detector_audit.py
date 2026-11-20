#!/usr/bin/env python3
"""Every detector of the integrator on recordings: how often it spoke, how near it came to its firing level, and
what was published as invalid meanwhile (lane L43; the table behind the gate / report defaults, docs/FAULTS.md).

    detector_audit.py SESSION... [--pos POS.txt] [--cfg JSON] [--gate-all] [--episodes]

Per session and detector: its mode; `peak` = the largest value its statistic reached as a fraction of its firing
level (below 1: it never fired; 0.5 = a margin of 2); the share of the frames with the car moving (vehicle
>= 3 m/s; all frames when there is no vehicle feed) during which it held a verdict; the number of such spells.
Then: sources set aside (group faults, single quantities), outputs published invalid, with shares and spells.
`--episodes` lists every spell of a set-aside source or an invalid output with its time and reason.
"""
import argparse
import bisect
import collections
import json
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(os.path.dirname(HERE), "chris-integrator"))
import replay_lib as rl  # noqa: E402
from chris_integrator import Integrator  # noqa: E402
from chris_integrator.core.integrator import DETECTORS, GATE_ALL  # noqa: E402

NS = 1e9
INPUTS = ("ego", "vehicle", "gnss", "map", "calib", "objects", "road", "signs")


def audit(path, pos, cfg, episodes):
    msgs = [m for m in rl.read_session(path, road_model="vision", pos=pos) if m.get("type") in INPUTS]
    vt, vv = [], []
    for m in msgs:
        if m.get("type") == "ego" and isinstance(m.get("speed_mps"), (int, float)):
            vt.append(m["ts_mono_ns"] / NS)
            vv.append(m["speed_mps"])
    I = Integrator(cfg or None)
    n = mv = 0
    t0 = None
    cnt, spells, open_, eps = collections.Counter(), collections.Counter(), {}, []
    for m in msgs:
        if m["type"] != "road":
            I.on_message(m)
            continue
        es, tr, ld = I.frame(m)
        if es is None:
            continue
        t = m["ts_mono_ns"] / NS
        t0 = t if t0 is None else t0
        i = bisect.bisect_left(vt, t)
        moving = not vt or (0 < i < len(vt) and vt[i] - vt[i - 1] < 0.3 and vv[i] >= 3.0)
        n += 1
        mv += moving
        cur = {}
        for (hi, lo, q), p in I.pairs.items():
            if p.verdict == "disagree" and p.c["mode"] != "ratio":
                cur[("detector", "%s.%s" % (q, p.decided_by()), "%s~%s" % (hi, lo))] = p.describe()
        for (g, q), sc in I.selfs.items():
            if sc.fault:
                cur[("detector", "self_check", g)] = sc.describe()
        for (g, q), nm in I.nis.items():
            if nm.high and q in I.cfg["quarantine"]:
                cur[("detector", "innovation", "%s.%s" % (g, q))] = nm.describe()
        for d in es["diagnostics"]:
            if d["kind"] in ("stuck", "noise", "invalid") and d["level"] == "fault":
                cur[("detector", "signal." + d["kind"], "%s.%s" % (d["source"], d["quantity"]))] = d["text"]
            if d["kind"] == "conflict":
                cur[("detector", "map.conflict", "map")] = d["text"]
        for g, f in I.faults().items():
            cur[("set aside", g, f["rule"])] = f["reason"]
        for (g, q), f in I.qfaults.items():
            cur[("set aside", "%s.%s" % (g, q), f["rule"])] = f["reason"]
        for q in I.disputed:
            cur[("disputed", q, "")] = ""
        for g, s in es["sources"].items():
            if s["state"] == "suspect":
                cur[("suspect", g, "")] = s.get("reason") or ""
        for k in ("speed", "yaw_rate", "position", "heading"):
            if not es[k]["valid"]:
                cur[("invalid", k, es[k].get("reason") or "")] = ""
        if ld is not None and not ld.get("valid", True):
            cur[("invalid", "lead", (ld.get("reason") or "").split(":")[0])] = ld.get("reason") or ""
        for k, txt in cur.items():
            cnt[k] += moving
            if k not in open_:
                open_[k] = [t, t, txt]
                spells[k] += 1
            else:
                open_[k][1] = t
        for k in [k for k in open_ if k not in cur]:
            s, e, txt = open_.pop(k)
            eps.append((s - t0, e - s, k, txt))
    eps += [(s - t0, e - s, k, txt) for k, (s, e, txt) in open_.items()]
    name = os.path.basename(os.path.normpath(path))
    print("== %s: %d road frames, %d with the car moving; position lost and anchored again %d times"
          % (name, n, mv, getattr(I, "reanchors", 0)))
    peak = collections.defaultdict(float)
    for (hi, lo, q), p in I.pairs.items():
        for k, v in p.peak.items():
            peak["%s.%s" % (q, k)] = max(peak["%s.%s" % (q, k)], v)
    for sc in I.selfs.values():
        peak["self_check"] = max(peak["self_check"], sc.peak)
    for (g, q), nm in I.nis.items():
        if q in I.cfg["quarantine"]:
            peak["innovation"] = max(peak["innovation"], nm.peak)
    for st in I.fdi.streams.values():
        for k, v in st.peak.items():
            peak["signal." + k] = max(peak["signal." + k], v)
    for s in I.specs:
        v = I.ctx[s.id].mem.get("conflict_peak")
        if v:
            peak["map.conflict"] = max(peak["map.conflict"], v)
    print("   detector          mode    peak / level   verdict held (moving frames)   spells")
    for det in sorted(set(DETECTORS) | set(I.detectors)):
        c = sum(v for k, v in cnt.items() if k[0] == "detector" and k[1] == det)
        sp = sum(v for k, v in spells.items() if k[0] == "detector" and k[1] == det)
        if det == "position.lost":
            sp, c = getattr(I, "reanchors", 0), 0
        print("   %-17s %-7s %-14s %-30s %d" % (det, I.detectors.get(det, "report"), ("%.2f" % peak[det]) if det in peak else "-",
                                               "%.2f %%" % (100.0 * c / max(mv, 1)), sp))
    for kind in ("set aside", "disputed", "invalid", "suspect"):
        rows = sorted(((k, v) for k, v in spells.items() if k[0] == kind), key=lambda x: -cnt[x[0]])
        if not rows:
            print("   %s: nothing" % kind)
        for k, v in rows:
            print("   %-9s %-28s %6.2f %% of moving frames, %d spells" % (kind, " ".join(x for x in k[1:] if x)[:28], 100.0 * cnt[k] / max(mv, 1), v))
    if episodes:
        for s, d, k, txt in sorted(eps):
            if k[0] in ("set aside", "invalid", "disputed"):
                print("     t %7.1f s for %6.1f s  %s :: %s" % (s, d, " | ".join(k), txt[:200]))


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("sessions", nargs="+")
    ap.add_argument("--pos")
    ap.add_argument("--cfg")
    ap.add_argument("--gate-all", action="store_true")
    ap.add_argument("--episodes", action="store_true")
    a = ap.parse_args()
    cfg = json.loads(a.cfg or "{}")
    if a.gate_all:
        cfg["detectors"] = dict(GATE_ALL, **cfg.get("detectors", {}))
    pos = rl.load_pos(a.pos) if a.pos else None
    for p in a.sessions:
        audit(p, pos, cfg, a.episodes)


if __name__ == "__main__":
    main()
