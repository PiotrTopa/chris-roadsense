#!/usr/bin/env python3
"""Lead's lateral place minus the centre of the ego lane at the lead's distance, at every stage of the chain, by
own speed.   usage: chain_table.py NAME WS.jsonl.gz SCENES.jsonl.gz ROWS.jsonl.gz [--json OUT]
Stages (all converted to: + = lead to the RIGHT of the lane centre, metres):
  raw     road: leads[0] (prob >= 0.5) against lane_lines 1, 2 (prob >= 0.5)
  topics  `lead` (present) against `lanes` lines 1, 2 (present)                      (measurement topics)
  view    view.objects role lead against view.lines lane:1, lane:2                    (api 2 only)
  scene   the backend's scene: top-level lead / lanes, and its "vw" part (view)        (what goes to the Pi)
  mfd     the frontend director: lead against both inner lines; and against the renderer's road centre rule
          (both lines, else one line -+ 1.75 m, else the path) = what is drawn, with the share of each rule
Per speed bin: frames with a lead / with lead and both lines, median offset, median angle."""
import sys, json, gzip, math, statistics as st, argparse
ap = argparse.ArgumentParser(); ap.add_argument("name"); ap.add_argument("ws"); ap.add_argument("scenes"); ap.add_argument("rows"); ap.add_argument("--json")
a = ap.parse_args()
BINS = ((0, 0.3), (0.3, 3), (3, 7), (7, 11), (11, 15), (15, 99))
def at(xs, ys, x):
    if ys is None: return None
    n = min(len(xs), len(ys))
    if n < 2 or x < xs[0]: return None
    if x > xs[n - 1]: return None
    for i in range(n - 1):
        if xs[i] <= x <= xs[i + 1]:
            if ys[i] is None or ys[i + 1] is None: return None
            f = (x - xs[i]) / (xs[i + 1] - xs[i]) if xs[i + 1] > xs[i] else 0
            return ys[i] + f * (ys[i + 1] - ys[i])
S = {}   # stage -> bin -> dict
G = {}   # mfd: distance band -> speed bin -> [(offset to the drawn centre, lead lateral, drawn centre direction 5-20 m, centre source)]
def rec(stage, v, lead, off, x, extra=None):
    b = next(b for b in BINS if b[0] <= v < b[1])
    r = S.setdefault(stage, {}).setdefault(b, {"n": 0, "lead": 0, "off": [], "ang": [], "x": [], "extra": {}})
    r["n"] += 1
    if lead: r["lead"] += 1
    if off is not None:
        r["off"].append(off); r["ang"].append(math.degrees(math.atan2(off, x))); r["x"].append(x)
    if extra: r["extra"][extra] = r["extra"].get(extra, 0) + 1
speed = {}  # seq -> v
DX = [0.0, 5.0, 10.0, 15.0, 20.0, 30.0, 40.0, 50.0, 65.0, 80.0, 100.0]
for ln in gzip.open(a.ws, "rt"):
    m = json.loads(ln); t = m.get("type")
    if t == "road":
        v = abs(m.get("v_ego") or 0.0); speed[m["seq"]] = v
        L = m["leads"][0]; ll = m["lane_lines"]; lead = L["prob"] >= 0.5 and L["x"][0] > 3; off = None
        if lead and ll[1]["prob"] >= 0.5 and ll[2]["prob"] >= 0.5:
            p, q = at(ll[1]["x"], ll[1]["y"], L["x"][0]), at(ll[2]["x"], ll[2]["y"], L["x"][0])
            if p is not None and q is not None: off = L["y"][0] - 0.5 * (p + q)
        rec("1 raw road", v, lead, off, L["x"][0])
    elif t == "lead":
        cur_lead = m
    elif t == "lanes":
        v = speed.get(m["seq"])
        if v is None: continue
        ld = cur_lead if cur_lead.get("seq") == m["seq"] else None
        lead = bool(ld and ld.get("present")); off = None
        l1, l2 = m["lines"][1], m["lines"][2]
        if lead and l1.get("present") and l2.get("present"):
            p, q = at(m["x"], l1["y"], ld["dist_m"]), at(m["x"], l2["y"], ld["dist_m"])
            if p is not None and q is not None: off = (ld.get("lateral_m") or 0.0) - 0.5 * (p + q)
        rec("2 topics lead/lanes", v, lead, off, ld["dist_m"] if lead else 0)
    elif t == "view":
        v = speed.get(m["seq"])
        if v is None: continue
        ob = next((o for o in m.get("objects") or [] if o.get("role") == "lead"), None)
        ls = {l["id"]: l for l in m.get("lines") or []}
        off = None
        if ob and "lane:1" in ls and "lane:2" in ls:
            p, q = at(m["x"], ls["lane:1"]["y"], ob["x"]), at(m["x"], ls["lane:2"]["y"], ob["x"])
            if p is not None and q is not None: off = ob["y"] - 0.5 * (p + q)
        rec("3 view", v, ob is not None, off, ob["x"] if ob else 0)
vt = sorted((0, 0) for _ in ())
tv = []
for ln in gzip.open(a.ws, "rt"):
    if ln.startswith('{"type":"road"'):
        m = json.loads(ln); tv.append((m["_rx_s"], abs(m.get("v_ego") or 0.0)))
import bisect
tt = [x[0] for x in tv]
def v_at(t):
    k = bisect.bisect_right(tt, t) - 1
    return tv[max(0, k)][1]
def scene_off(lead, lanes, xs, sign):
    if not lead: return False, None, 0
    d = lead.get("d", lead.get("x")); y = lead.get("y") or 0.0
    l1 = lanes[1] if lanes and len(lanes) > 2 else None; l2 = lanes[2] if lanes and len(lanes) > 2 else None
    off = None
    if l1 and l2:
        p, q = at(xs, l1["y"], d), at(xs, l2["y"], d)
        if p is not None and q is not None: off = sign * (y - 0.5 * (p + q))
    return True, off, d
for ln in gzip.open(a.scenes, "rt"):
    m = json.loads(ln); sc = m["scene"]; v = v_at(m["_t"])
    xs = sc.get("xs") or DX
    lead, off, d = scene_off(sc.get("lead"), sc.get("lanes"), xs, 1.0)   # the scene's y is RIGHT positive (the frontend negates)
    rec("4a scene (tracked part)", v, lead, off, d)
    vw = sc.get("vw")
    if vw is not None:
        lead, off, d = scene_off(vw.get("lead"), vw.get("lanes"), vw.get("xs") or xs, 1.0)
        rec("4b scene vw (view part)", v, lead, off, d)
for ln in gzip.open(a.rows, "rt"):
    r = json.loads(ln); v = r["v"]; ld = r["lead"]; off = off2 = None; src = None
    if ld:
        p, q = at(r["xs"], r["l1"], ld[0]), at(r["xs"], r["l2"], ld[0])
        if p is not None and q is not None: off = -(ld[1] - 0.5 * (p + q))
        c = at(r["xs"], r["cen"], ld[0])
        if c is not None:
            off2 = -(ld[1] - c)
            k = min(range(len(r["xs"])), key=lambda i: abs(r["xs"][i] - ld[0])); src = r["cen_src"][k] if k < len(r["cen_src"]) else "-"
    if ld and off2 is not None:
        db = "lead < 15 m" if ld[0] < 15 else "lead 15-30 m" if ld[0] < 30 else "lead 30-60 m" if ld[0] < 60 else "lead 60+ m"
        b = next(b for b in BINS if b[0] <= v < b[1])
        c5, c20 = at(r["xs"], r["cen"], 5.0), at(r["xs"], r["cen"], 20.0)
        G.setdefault(db, {}).setdefault(b, []).append((off2, -ld[1], None if c5 is None or c20 is None else -math.degrees(math.atan2(c20 - c5, 15.0)), src))
    rec("5a mfd director, both lines", v, bool(ld), off, ld[0] if ld else 0)
    rec("5b mfd, renderer's centre rule", v, bool(ld), off2, ld[0] if ld else 0, src)
res = {}
print(f"=== {a.name}")
for stage in sorted(S):
    print(f"  {stage}")
    print("     v m/s      frames  lead   lead+centre   offset m   angle deg   lead dist m   " + ("centre from: both / one line / path" if stage.startswith("5b") else ""))
    for b in BINS:
        r = S[stage].get(b)
        if not r: continue
        ok = len(r["off"]) >= 8
        ex = ""
        if stage.startswith("5b") and r["off"]:
            e = r["extra"]; tot = max(1, sum(e.values()))
            ex = f"{100*e.get('b',0)/tot:3.0f} / {100*(e.get('l',0)+e.get('r',0))/tot:3.0f} / {100*(e.get('p',0)+e.get('-',0))/tot:3.0f} %"
        print(f"     {b[0]:4.1f}-{b[1]:4.1f} {r['n']:7d} {r['lead']:5d} {len(r['off']):8d}      " + (f"{st.median(r['off']):+7.2f}    {st.median(r['ang']):+7.2f}     {st.median(r['x']):7.0f}" if ok else "      -          -           -") + "       " + ex)
        res.setdefault(stage, {})[f"{b[0]}-{b[1]}"] = {"frames": r["n"], "lead": r["lead"], "n": len(r["off"]), "off": st.median(r["off"]) if ok else None,
                                                     "ang": st.median(r["ang"]) if ok else None, "x": st.median(r["x"]) if ok else None}
print("  6 mfd, what is drawn, by lead distance and own speed: lead minus drawn centre (m, + right) | the lead's own lateral (m, + right) | direction of the drawn centre 5-20 m (deg, + right) | centre from lines %")
for db in ("lead < 15 m", "lead 15-30 m", "lead 30-60 m", "lead 60+ m"):
    for b in BINS:
        v = G.get(db, {}).get(b)
        if not v or len(v) < 8: continue
        cd = [x[2] for x in v if x[2] is not None]
        print(f"     {db:13s} v {b[0]:4.1f}-{b[1]:4.1f}  n {len(v):5d}   {st.median([x[0] for x in v]):+6.2f} | {st.median([x[1] for x in v]):+6.2f} | {(st.median(cd) if cd else float('nan')):+6.2f} | {100*sum(1 for x in v if x[3] in ('b','l','r'))/len(v):3.0f}")
        res.setdefault("6 " + db, {})[f"{b[0]}-{b[1]}"] = {"n": len(v), "off": st.median([x[0] for x in v]), "lead_y": st.median([x[1] for x in v]), "centre_dir": st.median(cd) if cd else None}
if a.json: json.dump(res, open(a.json, "w"), indent=1)
