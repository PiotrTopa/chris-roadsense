"""The `tracks` and `lead` messages (docs/INTEGRATOR.md sections 6, 8)."""
import math

from ..contract import signal, validity
from .objects import AX, VX, VY, X, Y, class_of
from .registry import NS

LEAD_CFG = dict(time_gap_min_speed=1.0, ttc_min_closing=0.5, camera_to_front_m=None, brake_max_age_s=3.0,
                ego_stop_confirm_s=0.3)


def ego_of(es, stop_confirm_s=0.3):
    """The ego motion of a frame as the object layer uses it: (speed, stands, source, age_s, tag), or None when
    the speed is unknown. `source` is "vehicle" when a vehicle measurement is in the estimate; then the car
    stands when the vehicle's standstill has held for stop_confirm_s, and the speed is exactly 0."""
    sp, st = es["speed"], es.get("standstill") or {}
    if not sp.get("valid"):
        return None
    veh = any(x.startswith("vehicle.") for x in sp.get("sources") or ())
    if veh:
        stands = bool(st.get("value")) and st.get("source") == "vehicle" and st.get("for_s", 0.0) >= stop_confirm_s
        return (0.0 if stands else sp["value"]), stands, "vehicle", sp.get("age_s"), \
            ((es.get("sources") or {}).get("vehicle") or {}).get("tag")
    return sp["value"], bool(st.get("value")) and st.get("source") != "vehicle", "vision", None, None


def _std(P, i):
    return math.sqrt(max(float(P[i, i]), 0.0))


def tracks_message(layer, es, ts, valid=True, reason=None):
    T = ts / NS
    out = []
    for tr in layer.tracks:
        seen = {s: round(max(0.0, T - t), 3) for s, t in tr.last_seen.items()}
        out.append({
            "id": tr.id, "state": tr.state, "exist": round(tr.exist, 3),
            "x_m": round(float(tr.x[X]), 2), "y_m": round(float(tr.x[Y]), 2),
            "vx_mps": round(float(tr.x[VX]), 2), "vy_mps": round(float(tr.x[VY]), 2),
            "ax_mps2": round(float(tr.x[AX]), 2),
            "std": [round(_std(tr.P, i), 2) for i in range(5)],
            "v_rel_mps": round(float(tr.x[VX]) - layer._ve, 2),
            "age_s": round(T - tr.t0, 2), "meas_age_s": round(min(seen.values()), 3) if seen else None,
            "sources": sorted(tr.sources), "seen": seen, "hits": tr.hits,
            "flags": sorted(tr.flags | ({"standing"} if tr.standing else set())
                            | ({"designated"} if tr.designated else set())),
            "path_offset_m": None if tr.mem.get("path_offset") is None else round(tr.mem["path_offset"], 2),
            # what kind of road user (non-critical metadata: label, confidence, the sensor that said so); null: unknown
            "class": class_of(tr, T),
            "attrs": tr.mem.get("attrs") or {}})
    sensors = {}
    for src, (m, t) in layer.models.items():
        st = layer.stats.get(src, {})
        sensors[src] = dict(st, age_s=round(max(0.0, T - t), 3), kind=m.name)
        if sensors[src].get("nis") is not None:
            sensors[src]["nis"] = round(sensors[src]["nis"], 2)
    return {"type": "tracks", "api": 3, "seq": layer.seq, "ts_mono_ns": ts, "ts_real_ns": es.get("ts_real_ns"),
            "frame": "car", "valid": valid, "reason": reason, "tracks": out, "sensors": sensors,
            "path": None if layer.path is None else "plan"}


def lead_message(layer, es, road, lead, ego_T, cfg=LEAD_CFG):
    """The `lead` topic: the track on the path, in the shape the api has served since L14 (present, mode,
    standing, departed, events, the camera's raw value), with validity and sources."""
    c = dict(cfg, **{k: v for k, v in (layer.lead_cfg or {}).items() if k in cfg})
    ts = es["ts_mono_ns"]
    T = ts / NS
    cam = next((m for m, _ in layer.models.values() if m.name == "camera_lead"), None)
    e = ego_of(es, c["ego_stop_confirm_s"])
    v_ego, stands, src, ego_age, ego_tag = e if e is not None else (None, False, None, None, None)
    # which object sources are alive and believable now
    live = [s for s, (m, t) in layer.models.items() if T - t <= 1.0 and m.name != "camera_lead"
            and not getattr(m, "annotates_only", False)]
    vis_ok = ((es.get("sources") or {}).get("vision") or {}).get("state") != "fault"
    valid, reason, inputs = validity(es, () if live else ("vision",), ("speed",))
    if valid and not live and cam is None and road is None:
        valid, reason = False, "no object source"
    out = {"type": "lead", "api": 3, "seq": road.get("seq") if road else None, "ts_mono_ns": ts,
           "ts_real_ns": es.get("ts_real_ns"), "present": lead is not None, "mode": "none", "track_id": None,
           "source": "vision+ego" if src == "vehicle" else "vision",
           "road_model": cam.road_model if cam is not None else None,
           "valid": valid, "reason": reason, "inputs": inputs}
    z = cam.z if cam is not None and vis_ok else None
    desig = lead is not None and cam is not None and lead is cam.cur
    f = cam.lead_fields(lead, ts) if desig else None
    # life-cycle events of a lead no camera designates (a range sensor's track on the path)
    prev = layer.lead_id
    cur_id = None if lead is None else lead.id
    if cur_id != prev:
        if not desig and (cam is None or cam.event is None or cam.event["ts_mono_ns"] != ts):
            kind = "acquired" if prev is None else "lost" if cur_id is None else "changed"
            layer.lead_event = {"kind": kind, "ts_mono_ns": ts, "track_id": cur_id if cur_id is not None else prev}
        layer.lead_id = cur_id
    ev = cam.event if cam is not None else None
    if layer.lead_event is not None and (ev is None or layer.lead_event["ts_mono_ns"] >= ev["ts_mono_ns"]):
        ev = layer.lead_event
    others = []
    if lead is not None:
        others = sorted(s for s, t in lead.last_seen.items() if T - t <= 0.3 and (cam is None or s != cam.src))
        cam_seen = desig and z is not None and f["evidence"] != "none"
        kinds = []                          # by sensor kind ("radar"), as the topic has named them; ids are in `sources`
        for o in others:
            k = layer.models[o][0].name if o in layer.models else o
            if k not in kinds:
                kinds.append(k)
        parts = kinds + (["vision"] if cam_seen or (desig and not others) else []) + (["ego"] if src == "vehicle" else [])
        out["source"] = "+".join(parts)
        out["mode"] = f["mode"] if desig else ("track" if T - max(lead.last_seen.values()) <= 0.25 else "coast")
        out["track_id"] = lead.id
    rad = None
    if lead is not None and others:
        s = others[0]
        rad = {"sensor": s, "id": (lead.mem.get("attrs") or {}).get("id", lead.id), "dist_m": round(float(lead.x[X]), 2), "lateral_m": round(-float(lead.x[Y]), 2),
               "v_rel_mps": round(float(lead.x[VX]) - v_ego, 2), "age_s": round(T - lead.last_seen[s], 3)}
    out["radar"] = rad
    out["vision_scale"] = (f or {}).get("vision_scale") if f else (cam.lead_fields(None, ts)["vision_scale"] if cam else None)
    vis = None
    if z is not None:
        vis = {"dist_m": round(z["x"], 2), "dist_std_m": round(z["sx"], 2), "lateral_m": round(z["y"], 2),
               "v_lead_mps": None if z["v"] is None else round(z["v"], 2),
               "v_rel_mps": None if z["v_rel"] is None else round(z["v_rel"], 2),
               "a_lead_mps2": None if z["a"] is None else round(z["a"], 2), "prob": round(z["p"], 3)}
    if lead is not None:
        d, vl, al = float(lead.x[X]), float(lead.x[VX]), float(lead.x[AX])
        pd, pv = float(lead.P[X, X]), float(lead.P[VX, VX])
        v_rel = vl - v_ego
        gap = d - c["camera_to_front_m"] if c["camera_to_front_m"] is not None else None
        ref = gap if gap is not None else d
        sxb = f["sx_bias"] if desig and not others else 0.0
        last = max(lead.last_seen.values())
        out.update(
            age_s=f["age_s"] if desig else round(T - lead.t0, 2),
            dist_m=round(d, 2),
            # 68 %: the filter's own uncertainty and, for a camera-only lead, the network's stated distance std
            # (its absolute accuracy, which no amount of filtering removes) in quadrature
            dist_std_m=round(math.sqrt(max(pd, 0.0) + sxb ** 2), 2),
            dist_filter_std_m=round(math.sqrt(max(pd, 0.0)), 3),
            lateral_m=round(-float(lead.x[Y]), 2), lateral_std_m=round(_std(lead.P, Y), 2),
            v_rel_mps=round(v_rel, 2), v_rel_std_mps=round(math.sqrt(max(pv, 0.0)), 2),
            v_lead_mps=round(vl, 2), a_lead_mps2=round(al, 2),
            time_gap_s=round(max(ref, 0.0) / v_ego, 2) if v_ego >= c["time_gap_min_speed"] else None,
            ttc_s=round(max(ref, 0.0) / -v_rel, 1) if v_rel <= -c["ttc_min_closing"] else None,
            standing=bool(f["standing"]) if desig else False,
            meas_age_s=(f["meas_age_s"] if desig and not others else round(T - last, 3)),
            evidence=f["evidence"] if desig else "strong",
            pending_change=bool(f["pending_change"]) if desig else False,
            prob=None if z is None else round(z["p"], 3),
            exist=round(lead.exist, 3), sources=sorted(lead.sources),
            on_path=lead.mem.get("path_offset") is not None and "on_path" in lead.flags,
            path_offset_m=None if lead.mem.get("path_offset") is None else round(lead.mem["path_offset"], 2))
        if gap is not None:
            out["gap_m"] = round(gap, 2)
    else:
        out.update(age_s=None, dist_m=None, dist_std_m=None, dist_filter_std_m=None, lateral_m=None, lateral_std_m=None,
                   v_rel_mps=None, v_rel_std_mps=None, v_lead_mps=None, a_lead_mps2=None, time_gap_s=None,
                   ttc_s=None, standing=False, meas_age_s=None, evidence="none", pending_change=False,
                   prob=None if z is None else round(z["p"], 3), exist=None, sources=[], on_path=False,
                   path_offset_m=None)
    dep = cam.departed if cam is not None else None
    out["departed"] = dep is not None
    out["departure"] = None if dep is None else {
        "age_s": round((ts - dep["ts"]) / NS, 2), "dist_at_stop_m": round(dep["dist_at_stop_m"], 2),
        "moved_m": None if lead is None else round(float(lead.x[X]) - dep["dist_at_stop_m"], 2)}
    out["event"] = None if ev is None else dict(ev, age_s=round((ts - ev["ts_mono_ns"]) / NS, 2))
    out["ego"] = {"speed_mps": None if v_ego is None else round(v_ego, 3), "standstill": bool(stands), "source": src,
                  "std_mps": (es.get("speed") or {}).get("std"), "age_s": ego_age, "tag": ego_tag,
                  "brake": signal(es, "brake", c["brake_max_age_s"])}
    out["vision"] = vis
    out["class"] = None if lead is None else class_of(lead, T)     # non-critical: what kind of road user, if a sensor says
    return out
