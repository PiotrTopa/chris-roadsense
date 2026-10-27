#!/usr/bin/env python3
"""Score the engine's offline timeline (tools/rules_replay.py --out) against human labels (labels/<ride>.json).

usage: score.py timeline.json labels_dir [--rides a,b] [--md]
Per ride and pooled: time agreement per lasting rule (excluding unclear stretches), start/end delays per matched interval,
misses (human intervals with < 50 % overlap) attributed to detector/engine via the ignored list, false rules; momentary
and warning precision/recall (+-6 s); junction-ending delay statistics by source.
"""
import argparse
import json
import os
import statistics as st

LASTING = ("priority_road", "no_stopping", "no_parking", "no_overtaking", "height_limit", "weight_limit", "width_limit",
           "living_street", "built_up_area", "one_way", "motorway", "expressway")
MOMENT_MAP = {"B-2": "no_entry", "B-20": "stop", "A-7": "yield", "B-21": "no_left_turn", "B-22": "no_right_turn",
              "B-23": "no_u_turn", "C-12": "roundabout", "D-6": "pedestrian_crossing", "B-26": "no_overtaking_hgv",
              "B-5": "no_trucks", "D-4a": "dead_end", "D-5": "priority_over_oncoming", "B-31": "give_way_oncoming",
              "D-3": "one_way_side", "D-2": "end_of_priority_road", "B-42": "end_of_prohibitions", "B-1": "road_closed",
              "B-3": "no_motor_vehicles", "C-2": "mandatory_direction", "C-4": "mandatory_direction", "C-5": "mandatory_direction"}
WARN_MAP = {"A-5": "crossroads_ahead", "A-6": "side_road_ahead", "A-8": "roundabout_ahead", "A-16": "pedestrian_crossing_ahead",
            "A-17": "children_ahead", "A-29": "traffic_signals_ahead", "A-9": "level_crossing_ahead", "A-10": "level_crossing_ahead",
            "A-14": "roadworks_ahead", "A-1": "curve_ahead", "A-2": "curve_ahead", "A-3": "curve_ahead", "A-4": "curve_ahead",
            "A-11": "uneven_road_ahead", "A-12": "road_narrows_ahead", "A-15": "slippery_ahead", "A-18": "animals_ahead",
            "A-20": "two_way_ahead", "A-21": "tram_crossing_ahead", "A-24": "cyclists_ahead", "A-30": "other_danger_ahead"}
TOL = 6.0


def overlap(a, b):
    return max(0.0, min(a[1], b[1]) - max(a[0], b[0]))


def norm_sign(s):
    s = (s or "").split()[0].rstrip("?")
    for k in sorted(MOMENT_MAP, key=len, reverse=True):
        if s.startswith(k):
            return MOMENT_MAP[k]
    return None


def norm_warn(s):
    s = (s or "").split()[0].rstrip("?")
    for k in sorted(WARN_MAP, key=len, reverse=True):
        if s.startswith(k) and (len(s) == len(k) or not s[len(k)].isdigit()):
            return WARN_MAP[k]
    return "warning_other"


def score_ride(tl, lab):
    dur = tl["duration_s"]
    res = {"ride": tl["ride"], "duration_s": dur, "rules": {}, "momentary": {}, "warnings": {}, "misses": [], "false": [],
           "delays": [], "ends": []}
    eng = [i for i in tl["intervals"] if i["id"] in LASTING]
    hum = [i for i in lab.get("intervals", []) if i["rule"] in LASTING]
    cls_ignored = {}
    for ig in tl.get("ignored", []):
        cls_ignored.setdefault(ig.get("cls"), []).append(ig)
    for rule in sorted(set(i["id"] for i in eng) | set(i["rule"] for i in hum)):
        e = [(i["start_t"], i["end_t"], i) for i in eng if i["id"] == rule]
        h = [(i["start_t"], i["end_t"], i) for i in hum if i["rule"] == rule]
        # time agreement on a 0.5 s grid, excluding unclear stretches (between end_t and alt_end_t)
        unclear = [(min(i["end_t"], i.get("alt_end_t") or i["end_t"]), max(i["end_t"], i.get("alt_end_t") or i["end_t"]))
                   for _, _, i in h if i.get("end_unclear")]
        agree = total = 0
        t = 0.0
        while t < dur:
            if not any(a <= t < b for a, b in unclear):
                ein = any(a <= t < b for a, b, _ in e)
                hin = any(a <= t < b for a, b, _ in h)
                total += 1
                agree += (ein == hin)
            t += 0.5
        matched_e = set()
        for a, b, hi in h:
            best = max(e, key=lambda x: overlap((a, b), (x[0], x[1])), default=None)
            ov = overlap((a, b), (best[0], best[1])) if best else 0.0
            if best and ov == 0:          # short human interval, engine a few seconds late: match on the start instead
                near = [x for x in e if abs(x[0] - a) <= TOL]
                if near:
                    best, ov = min(near, key=lambda x: abs(x[0] - a)), 1e-9
            if best and ov > 0 and (ov >= 0.5 * min(b - a, best[1] - best[0]) or ov == 1e-9):
                if id(best[2]) in matched_e:      # the engine kept one rule where the human saw a new one after a junction
                    res.setdefault("merged", []).append({"rule": rule, "human": [a, b], "engine": [best[0], best[1]]})
                    continue
                matched_e.add(id(best[2]))
                res["delays"].append({"rule": rule, "start": round(best[0] - a, 1), "end": round(best[1] - b, 1),
                                      "end_reason": best[2]["end_reason"], "human_end": hi.get("end_reason"),
                                      "end_unclear": bool(hi.get("end_unclear"))})
                res["ends"].append({"rule": rule, "source": best[2]["end_reason"].split(" (")[0], "delay": round(best[1] - b, 1),
                                    "human_end": hi.get("end_reason"), "unclear": bool(hi.get("end_unclear"))})
            else:
                cause = "detector"
                t0 = hi.get("start_sign_t") or a
                twin = {"no_stopping": "no_parking", "no_parking": "no_stopping"}.get(rule)
                if twin and any(abs(i["start_t"] - t0) <= TOL for i in eng if i["id"] == twin):
                    cause = "classifier: B-35/B-36 confusion"
                near = [ig for igs in cls_ignored.values() for ig in igs if abs(ig["t"] - t0) <= 5 and
                        ig.get("sign", "") and any(k in ig.get("cls", "") for k in rule.split("_") if len(k) > 3)]
                if near:
                    cause = "engine: " + "; ".join(sorted(set(n["reason"] for n in near)))[:120]
                res["misses"].append({"rule": rule, "start_t": a, "end_t": b, "cause": cause})
        for a, b, ei in e:
            if id(ei) not in matched_e:
                twin = {"no_stopping": "no_parking", "no_parking": "no_stopping"}.get(rule)
                conf = twin and any(abs(i["start_t"] - a) <= TOL for i in hum if i["rule"] == twin)
                res["false"].append({"rule": rule, "start_t": a, "end_t": b, "track": ei["track_id"], "end_reason": ei["end_reason"],
                                     "cause": "classifier: B-35/B-36 confusion" if conf else "?"})
        res["rules"][rule] = {"agreement": round(agree / max(1, total), 3), "human_n": len(h), "engine_n": len(e),
                              "human_s": round(sum(b - a for a, b, _ in h), 1), "engine_s": round(sum(b - a for a, b, _ in e), 1)}
    # momentary / warnings: precision and recall within TOL
    def match(evs, labs):
        used, tp = set(), 0
        for lt in labs:
            c = [k for k, ev in enumerate(evs) if k not in used and abs(ev - lt) <= TOL]
            if c:
                used.add(c[0]); tp += 1
        return tp
    eng_m = {}
    for p in tl["passed"]:
        eng_m.setdefault(p["id"], []).append(p["t"])
    for i in tl["intervals"]:
        if i["id"] not in LASTING:
            eng_m.setdefault(i["id"], []).append(i["start_t"])
    hum_m = {}
    for m in lab.get("momentary", []):
        rid = norm_sign(m["sign"])
        if rid:
            hum_m.setdefault(rid, []).append(m["t"])
    for rid in sorted(set(eng_m) | set(hum_m)):
        tp = match(eng_m.get(rid, []), hum_m.get(rid, []))
        res["momentary"][rid] = {"tp": tp, "engine": len(eng_m.get(rid, [])), "human": len(hum_m.get(rid, []))}
    eng_w, hum_w = {}, {}
    for w in tl["warnings"]:
        eng_w.setdefault(w["id"], []).append(w["t"])
    for w in lab.get("warnings", []):
        hum_w.setdefault(norm_warn(w["sign"]), []).append(w["t"])
    for rid in sorted(set(eng_w) | set(hum_w)):
        tp = match(eng_w.get(rid, []), hum_w.get(rid, []))
        res["warnings"][rid] = {"tp": tp, "engine": len(eng_w.get(rid, [])), "human": len(hum_w.get(rid, []))}
    # junction cues vs human junctions
    hj = [j["t"] for j in lab.get("junctions", []) if j.get("public") is not False]
    ej = tl["junctions"]
    res["junction_cues"] = {"human": len(hj), "engine": len(ej),
                            "by_source": {s: sum(1 for j in ej if j["source"].split(" ")[0] == s) for s in ("sign", "traffic", "turn")},
                            "engine_hit": sum(1 for j in ej if any(abs(j["t"] - t) <= 8 for t in hj)),
                            "human_found": sum(1 for t in hj if any(abs(j["t"] - t) <= 8 for j in ej))}
    return res


def pooled(results):
    out = {"rules": {}, "momentary": {}, "warnings": {}, "delays": {}, "ends": {}}
    for rule in sorted(set(r for res in results for r in res["rules"])):
        rows = [res["rules"][rule] for res in results if rule in res["rules"]]
        out["rules"][rule] = {"agreement_mean": round(st.mean(r["agreement"] for r in rows), 3),
                              "human_n": sum(r["human_n"] for r in rows), "engine_n": sum(r["engine_n"] for r in rows),
                              "matched": sum(1 for res in results for d in res["delays"] if d["rule"] == rule),
                              "misses": sum(1 for res in results for m in res["misses"] if m["rule"] == rule),
                              "merged": sum(1 for res in results for m in res.get("merged", []) if m["rule"] == rule),
                              "false": sum(1 for res in results for f in res["false"] if f["rule"] == rule)}
    for key in ("momentary", "warnings"):
        for rid in sorted(set(r for res in results for r in res[key])):
            rows = [res[key][rid] for res in results if rid in res[key]]
            tp, e, h = (sum(r[k] for r in rows) for k in ("tp", "engine", "human"))
            out[key][rid] = {"tp": tp, "engine": e, "human": h, "precision": round(tp / e, 2) if e else None,
                             "recall": round(tp / h, 2) if h else None}
    d = [x for res in results for x in res["delays"]]
    if d:
        out["delays"] = {"n": len(d), "start_median": st.median(x["start"] for x in d),
                         "end_median_clear": st.median([x["end"] for x in d if not x["end_unclear"]] or [0]),
                         "end_abs_median_clear": st.median([abs(x["end"]) for x in d if not x["end_unclear"]] or [0])}
    ends = [x for res in results for x in res["ends"]]
    for src in sorted(set(x["source"] for x in ends)):
        rows = [x for x in ends if x["source"] == src]
        out["ends"][src] = {"n": len(rows), "abs_delay_median": st.median(abs(x["delay"]) for x in rows),
                            "delay_median": st.median(x["delay"] for x in rows)}
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("timeline")
    ap.add_argument("labels_dir")
    ap.add_argument("--rides")
    ap.add_argument("--out")
    a = ap.parse_args()
    tls = {t["ride"]: t for t in json.load(open(a.timeline))}
    rides = a.rides.split(",") if a.rides else sorted(tls)
    results = []
    for r in rides:
        p = os.path.join(a.labels_dir, f"{r}.json")
        if r not in tls or not os.path.exists(p):
            print(f"skip {r}: timeline {r in tls}, labels {os.path.exists(p)}")
            continue
        results.append(score_ride(tls[r], json.load(open(p))))
    out = {"rides": results, "pooled": pooled(results)}
    if a.out:
        json.dump(out, open(a.out, "w"), indent=1)
    for res in results:
        print(f"== {res['ride']} ({res['duration_s']:.0f} s)")
        for rule, v in res["rules"].items():
            print(f"  {rule:16s} agree {v['agreement']:.3f}  human {v['human_n']} ({v['human_s']} s)  engine {v['engine_n']} ({v['engine_s']} s)")
        for m in res["misses"]:
            print(f"  MISS  {m['rule']} {m['start_t']}-{m['end_t']}: {m['cause']}")
        for f in res["false"]:
            print(f"  FALSE {f['rule']} {f['start_t']}-{f['end_t']} track {f['track']}: {f['end_reason']}  [{f['cause']}]")
        for m in res.get("merged", []):
            print(f"  merged {m['rule']}: human {m['human']} inside engine {m['engine']}")
        for d in res["delays"]:
            print(f"  delay {d['rule']}: start {d['start']:+.1f} s  end {d['end']:+.1f} s  ({d['end_reason']} | human {d['human_end']}{' unclear' if d['end_unclear'] else ''})")
        print("  momentary:", {k: (v['tp'], v['engine'], v['human']) for k, v in res["momentary"].items()})
        print("  warnings: ", {k: (v['tp'], v['engine'], v['human']) for k, v in res["warnings"].items()})
        print("  junctions:", res["junction_cues"])
    print("== pooled")
    print(json.dumps(out["pooled"], indent=1))


if __name__ == "__main__":
    main()
