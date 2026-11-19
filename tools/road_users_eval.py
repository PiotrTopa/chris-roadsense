#!/usr/bin/env python3
"""road_users_eval.py: road-user classes through chris-integrator on a recorded drive (lane L42).

    road_users_eval.py SCAN.npz SESSION_DIR [--calib pitch_deg,yaw_deg] [--labels LEAD_DIR] [--views 0,1]
                       [--thr person=0.4,...] [--out OUT.json]

SCAN.npz is roadsense-signs `tools/road_users/car_scan.py` output for this session's recorder frames (one JPEG a
second): every road-user box of the light detector in the views of the phone's unit. The boxes of a frame are
attached, as `road_users` (the key `signd --lights-road-users` writes), to the recorded `lights` message of the same
view nearest in time (within 0.25 s), with signd's per-class thresholds and rider pairing, and the session goes
through the default stack (`chris_integrator.stack.build`, no map box) on its own clock. Reported:

* the lead's class: share of lead frames with a class, class changes per minute of lead time while the track
  stays the same (after the vote), and, with --labels (roadsense-signs tools/road_users/sheets.py lead), the
  voted class and the per-frame detector class at the labelled frames against the labels;
* the display objects (`view.objects` of kind `road_user`): how many per view message, by class;
* the range agreement between the boxes' ground distance and the road model's tracked lead distance (a check of
  the calibration: --calib replaces the session's pitch / yaw, in degrees);
* the cost: the stack's wall time per road frame with and without the road users.

The recorder's frames come once a second, so the class votes here come at about 2 a second (two views), against
5 on the phone (lights at 5 Hz): the vote is slower to start here, and per-frame flicker has fewer chances.
"""
import argparse, bisect, collections, copy, json, math, os, sys, time

_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(_ROOT, "chris-integrator"))

import numpy as np  # noqa: E402

from chris_integrator.mixer import MessageClock  # noqa: E402
from chris_integrator.replay import read_lines  # noqa: E402
from chris_integrator.stack import build  # noqa: E402

NAMES = ["person", "rider", "car", "truck", "bus", "train", "motor", "bike"]
THR = {"person": 0.40, "rider": 0.35, "car": 0.40, "truck": 0.40, "bus": 0.40, "train": 0.50, "motor": 0.35, "bike": 0.35}
NS = 1e9


def pair_riders(dets):
    gone, taken = set(), set()
    for i, r in enumerate(dets):
        if r["cls"] != "rider":
            continue
        best, best_ov = None, 0.0
        for j, b in enumerate(dets):
            if b["cls"] not in ("motor", "bike") or j in taken:
                continue
            rb, bb = r["box"], b["box"]
            ox = min(rb[2], bb[2]) - max(rb[0], bb[0])
            ov = ox / max(1.0, min(rb[2] - rb[0], bb[2] - bb[0]))
            if ov < 0.5 or rb[3] < bb[1] or (rb[1] + rb[3]) > (bb[1] + bb[3]) or ov <= best_ov:
                continue
            best, best_ov = j, ov
        if best is None:
            continue
        b = dets[best]
        b["box"] = [min(b["box"][0], r["box"][0]), min(b["box"][1], r["box"][1]), max(b["box"][2], r["box"][2]),
                    max(b["box"][3], r["box"][3])]
        b["score"] = max(b["score"], r["score"])
        b["rider"] = True
        taken.add(best)
        gone.add(i)
    return [d for i, d in enumerate(dets) if i not in gone]


def iou(a, b):
    ix = max(0.0, min(a[2], b[2]) - max(a[0], b[0])); iy = max(0.0, min(a[3], b[3]) - max(a[1], b[1]))
    i = ix * iy
    return i / ((a[2] - a[0]) * (a[3] - a[1]) + (b[2] - b[0]) * (b[3] - b[1]) - i + 1e-9)


def load_scan(npz, session, views, thr):
    """{seq: {view: [road user dicts as signd writes them]}} for the frames of this session."""
    d = np.load(npz)
    frames = list(d["frames"])
    keep = {i: int(os.path.basename(f)[:-4]) for i, f in enumerate(frames)
            if os.path.dirname(os.path.dirname(f)).rstrip("/") == session.rstrip("/")}
    out = collections.defaultdict(lambda: collections.defaultdict(list))
    for r, s in zip(d["rows"], d["scores"]):
        fi, v = int(r[0]), int(r[1])
        if fi not in keep or v not in views:
            continue
        c = int(np.argmax(s))
        if s[c] <= thr[NAMES[c]]:
            continue
        roi = r[7:11]
        b = [float(x) for x in r[3:7]]
        clip = ("l" if b[0] <= roi[0] + 2 else "") + ("t" if b[1] <= roi[1] + 2 else "") + \
               ("r" if b[2] >= min(roi[2], 1280) - 2 else "") + ("b" if b[3] >= min(roi[3], 720) - 2 else "")
        out[keep[fi]][v].append({"cls": NAMES[c], "score": round(float(s[c]), 3), "box": [round(x, 1) for x in b],
                                 "clip": clip, "rider": False, "p": [round(float(x), 2) for x in s]})
    for seq in out:
        for v in out[seq]:
            lst = sorted(out[seq][v], key=lambda x: -x["score"])
            kept = []
            for x in lst:                                 # signd: class-agnostic NMS 0.5 (the scan used 0.45)
                if all(iou(x["box"], k["box"]) <= 0.5 for k in kept):
                    kept.append(x)
            out[seq][v] = pair_riders(kept)
    return out


def messages(session, scan, calib_override, road_model="vision"):
    """The session's messages with `road_users` attached to the recorded lights messages."""
    msgs = [m for m in read_lines(session) if isinstance(m.get("ts_mono_ns"), int)
            and m.get("type") not in ("ego_state", "tracks", "lead", "view", "fused", "map", "lanes", "speed_limit", "rules")]
    msgs.sort(key=lambda m: m["ts_mono_ns"])
    for m in msgs:                                    # recordings before lane L37 lost `road_model`; every recording
        if m.get("type") == "road" and "road_model" not in m:     # of the car is the vision model's (L37)
            m["road_model"] = road_model
    road_ts = {m["seq"]: m["ts_mono_ns"] for m in msgs if m.get("type") == "road"}
    lights = collections.defaultdict(list)            # view -> [(ts, index)]
    for i, m in enumerate(msgs):
        if m.get("type") == "lights":
            lights[m.get("view") or 0].append((m["ts_mono_ns"], i))
    attached = 0
    for seq, per_view in scan.items():
        ts = road_ts.get(seq)
        if ts is None:
            continue
        for v, dets in per_view.items():
            lst = lights.get(v) or []
            k = bisect.bisect_left(lst, (ts, -1))
            best = min((lst[j] for j in (k - 1, k) if 0 <= j < len(lst)), key=lambda e: abs(e[0] - ts), default=None)
            if best is None or abs(best[0] - ts) > 0.25 * NS:
                continue
            m = msgs[best[1]]
            if "road_users" in m:
                continue
            m["road_users"] = dets
            m["_frame_seq"] = seq
            attached += 1
    if calib_override is not None:
        for m in msgs:
            if m.get("type") == "calib":
                m["pitch"], m["yaw"] = math.radians(calib_override[0]), math.radians(calib_override[1])
    return msgs, attached


def run(msgs, with_users):
    st = build(cfg={"estimator": {"sources": [{"kind": "radar", "id": "*"}, {"kind": "road_users", "id": "*"}]}},
               mapper=None, clock=MessageClock())
    out = {"lead": [], "view": [], "road_users": []}
    st.mixer.add_output("e", lambda e: out[e.stream].append(e.msg), streams=tuple(out))
    n_road = 0
    t0 = time.process_time()
    for m in msgs:
        if not with_users and "road_users" in m:
            m = dict(m)
            m.pop("road_users")
        if m.get("type") == "road":
            n_road += 1
        st.deliver(m)
    return out, (time.process_time() - t0) / max(1, n_road) * 1000.0, n_road


def lead_stats(leads):
    present = [m for m in leads if m.get("present")]
    with_cls = [m for m in present if m.get("class")]
    changes, nulls, prev = 0, 0, None
    for m in present:
        lab = (m.get("class") or {}).get("label")
        key = (m.get("track_id"), lab)
        if prev is not None and prev[0] == key[0]:
            if prev[1] is not None and lab is not None and prev[1] != lab:
                changes += 1
            elif (prev[1] is None) != (lab is None):
                nulls += 1
        prev = key
    dur = 0.0
    for a, b in zip(present, present[1:]):
        dt = (b["ts_mono_ns"] - a["ts_mono_ns"]) / NS
        if dt < 0.5:
            dur += dt
    labels = collections.Counter((m.get("class") or {}).get("label") for m in present)
    return {"lead_frames": len(present), "lead_minutes": round(dur / 60.0, 1),
            "with_class_share": round(len(with_cls) / max(1, len(present)), 3),
            "label_changes_same_track": changes, "changes_per_min": round(changes / max(1e-9, dur / 60.0), 3),
            "null_label_transitions_same_track": nulls, "labels": dict(labels)}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("scan")
    ap.add_argument("session")
    ap.add_argument("--calib", help="pitch_deg,yaw_deg replacing the session's calibration")
    ap.add_argument("--labels", help="directory of sheets.py lead (lead_sample.json + labels.txt)")
    ap.add_argument("--views", default="0,1")
    ap.add_argument("--thr", default="")
    ap.add_argument("--out")
    ap.add_argument("--no-cost", action="store_true")
    ap.add_argument("--road-model", default="vision", help="put into road messages without one (recordings before L37)")
    a = ap.parse_args()
    thr = dict(THR)
    for kv in filter(None, a.thr.split(",")):
        k, v = kv.split("=")
        thr[k] = float(v)
    cal = tuple(float(x) for x in a.calib.split(",")) if a.calib else None
    scan = load_scan(a.scan, a.session, [int(v) for v in a.views.split(",")], thr)
    msgs, attached = messages(a.session, scan, cal, a.road_model)
    out, ms_users, n_road = run(msgs, True)
    res = {"session": os.path.basename(a.session.rstrip("/")), "frames_with_boxes": len(scan), "lights_msgs_with_users": attached,
           "road_frames": n_road, "calib_override_deg": cal, "thr": thr, "lead": lead_stats(out["lead"])}
    if not a.no_cost:
        _, ms_plain, _ = run(msgs, False)
        res["cost_ms_per_road_frame"] = {"with": round(ms_users, 3), "without": round(ms_plain, 3)}
    # display objects
    ru = [[o for o in m["objects"] if o.get("kind") == "road_user"] for m in out["view"]]
    res["display"] = {"view_msgs": len(ru), "objects_per_msg_mean": round(sum(map(len, ru)) / max(1, len(ru)), 3),
                      "msgs_with_any": round(sum(1 for r in ru if r) / max(1, len(ru)), 3),
                      "by_class": dict(collections.Counter((o.get("cls") or {}).get("label") for r in ru for o in r)),
                      "distinct_ids": len({o["id"] for r in ru for o in r})}
    # range agreement: boxes with a ground range near the lead's bearing vs the tracked lead distance
    leads = {m["ts_mono_ns"]: m for m in out["lead"] if m.get("present") and m.get("dist_m")}
    lts = sorted(leads)
    rel = []
    for m in out["road_users"]:
        k = bisect.bisect_left(lts, m["ts_mono_ns"])
        cand = [lts[j] for j in (k - 1, k) if 0 <= j < len(lts)]
        if not cand:
            continue
        t = min(cand, key=lambda x: abs(x - m["ts_mono_ns"]))
        if abs(t - m["ts_mono_ns"]) > 0.15 * NS:
            continue
        ld = leads[t]
        lb = math.degrees(math.atan2(ld.get("lateral_m") or 0.0, ld["dist_m"]))
        # the nearest vehicle box on the lead's bearing (the largest box: one behind the lead is smaller); only
        # when that box has a ground distance
        near = [o for o in m["objects"] if o["class"] in ("car", "truck", "bus", "van")
                and abs(o["bearing_deg"] - lb) < max(2.0, math.degrees(math.atan2(1.2, ld["dist_m"])))]
        if near:
            o = max(near, key=lambda o: o["box"][3] - o["box"][1])
            if o.get("range_m") is not None:
                rel.append((ld["dist_m"], o["range_m"], o["range_std_m"]))
    if rel:
        r = np.array(rel)
        ratio = r[:, 1] / r[:, 0]
        z = (r[:, 1] - r[:, 0]) / r[:, 2]
        res["range_vs_lead"] = {"n": len(rel), "ratio_median": round(float(np.median(ratio)), 3),
                                "ratio_p10_p90": [round(float(np.percentile(ratio, 10)), 3), round(float(np.percentile(ratio, 90)), 3)],
                                "within_3sigma": round(float(np.mean(np.abs(z) <= 3.0)), 3),
                                "by_dist": {f"{lo}-{hi}": round(float(np.median(ratio[(r[:, 0] >= lo) & (r[:, 0] < hi)])), 3)
                                            for lo, hi in ((5, 15), (15, 30), (30, 60)) if np.any((r[:, 0] >= lo) & (r[:, 0] < hi))}}
    # labelled lead frames
    if a.labels:
        sample = json.load(open(os.path.join(a.labels, "lead_sample.json")))
        labels = {}
        for ln in open(os.path.join(a.labels, "labels.txt")):
            p = ln.split()
            if len(p) >= 2:
                labels[int(p[0])] = p[1]
        by_seq = {m.get("seq"): m for m in out["lead"]}
        rows = []
        um_by_seq = collections.defaultdict(list)
        for m in out["road_users"]:
            um_by_seq[m.get("seq")].append(m)
        frame_of_lights = {m.get("seq"): m.get("_frame_seq") for m in msgs if m.get("type") == "lights" and "_frame_seq" in m}
        per_frame = collections.defaultdict(list)
        for m in out["road_users"]:
            fs = frame_of_lights.get(m.get("seq"))
            if fs is not None:
                per_frame[fs].append(m)
        for n, s in enumerate(sample):
            if s["session"] != res["session"] or n not in labels:
                continue
            ld = by_seq.get(s["seq"])
            voted = ((ld or {}).get("class") or {}).get("label") if ld and ld.get("present") else None
            # per-frame: the detection of this JPEG nearest the lead's bearing (the source's gate)
            pf = None
            if ld and ld.get("present") and ld.get("dist_m"):
                lb = math.atan2(ld.get("lateral_m") or 0.0, ld["dist_m"])
                gate = max(math.radians(1.5), math.atan2(1.2, ld["dist_m"]))
                best = None
                for m in per_frame.get(s["seq"], ()):
                    for o in m["objects"]:
                        e = abs(math.radians(o["bearing_deg"]) - lb)
                        g = max(gate, 3 * math.radians(o["bearing_std_deg"]))
                        if e <= g and (best is None or e / g < best[0]):
                            best = (e / g, o["class"])
                pf = best[1] if best else None
            rows.append({"n": n, "seq": s["seq"], "label": labels[n], "voted": voted, "per_frame": pf,
                         "lead_present": bool(ld and ld.get("present"))})
        res["labelled"] = rows
    js = json.dumps(res, indent=1)
    if a.out:
        open(a.out, "w").write(js)
    print(json.dumps({k: v for k, v in res.items() if k != "labelled"}, indent=1))


if __name__ == "__main__":
    main()
