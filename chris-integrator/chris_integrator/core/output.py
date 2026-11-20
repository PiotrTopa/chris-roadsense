"""The `ego_state` message (docs/DESIGN.md section 8): every quantity with value, std, valid / reason,
mode, age and contributing sources; calibration states; per-source health; the monitors' table."""
import math

from ..native import round                  # builtins.round, native where built
from . import ego as E
from .registry import NS

TABLES_S = 0.2
_SPEED_KINDS = ("speed", "standstill")
_YAW_KINDS = ("yaw_rate", "standstill_w")


def _health(I, g, T):
    age = None if g.last_t is None else max(0.0, I.now - g.last_t)
    usable = True
    reason = None
    if g.fault is not None:
        state, reason, usable = "fault", g.fault["reason"], False
    elif g.last_t is None:
        state, usable = "absent", False
    elif age > g.stale_s:
        state, usable = "stale", False
    else:
        checked = suspect = False
        for (hi, lo, q), p in I.pairs.items():
            if g.name not in (hi, lo):
                continue
            other = I.groups[lo if hi == g.name else hi]
            if p.verdict == "agree" and not other.witness and other.fault is None:
                checked = True
            if other.fault is not None:
                continue               # the culprit of that disagreement is known, and it is the other one
            if p.verdict == "disagree" and p.c["mode"] != "ratio" and not I.gates(I._detector(p)):
                continue               # a detector in report mode: listed under `reported`, the state is not touched
            if p.suspect or (p.verdict == "disagree" and (lo == g.name or not other.witness)):
                suspect = True
                reason = p.describe() if lo == g.name else "%s of %s disagrees" % (q, lo)
        for (gname, q), sc in I.selfs.items():
            if gname == g.name and sc.median is not None and sc.median > sc.c["recover_ratio"] \
                    and sc.n >= sc.c["min_n"]:
                suspect, reason = True, sc.describe()
        for (gname, q), nm in I.nis.items():
            if gname != g.name or nm.value is None:
                continue
            if nm.high:
                suspect, reason = True, nm.describe() + (" (set aside)" if (gname, q) in I.ekf.quarantine else "")
            elif q in ("pos", "heading", "map_cross"):
                checked = True             # no raw partner exists for a position: its innovations are its check
        state = "suspect" if suspect else "ok" if checked else "unchecked"
    qf = {q: f for (gg, q), f in I.qfaults.items() if gg == g.name}
    if qf and g.fault is None:
        # single quantities of the source are set aside; the rest of it is used
        if usable:
            state = "suspect"
        reason = "; ".join("%s set aside: %s" % (q, f["reason"]) for q, f in sorted(qf.items()))
    n = sum(1 for a in g.arrivals if a > I.now - 2.0)
    out = {"state": state, "reason": reason, "usable": usable,
           "age_s": None if age is None else round(age, 3), "rate_hz": round(n / 2.0, 1)}
    if qf:
        out["set_aside"] = sorted(qf)
    rep = sorted({det for (gg, _, det) in I.fdi.reported if gg == g.name})
    if rep:
        out["reported"] = rep          # detectors in report mode that name this source now (texts: `diagnostics`)
    if g.fault is not None:
        out["isolation"] = g.fault["rule"]
        out["since_s"] = None if g.fault["since"] is None else round(max(0.0, I.now - g.fault["since"]), 1)
    nis = {q: round(nm.value, 2) for (gname, q), nm in I.nis.items() if gname == g.name and nm.value is not None}
    if nis:
        out["nis"] = nis
    for s in g.sources:
        rep = dict(s.report(I.ctx[s.id]) or {})
        why = rep.pop("suspect_reason", None)      # a source's own check holds its rows back
        if why and out["state"] in ("ok", "unchecked"):
            out["state"], out["reason"] = "suspect", why
        out.update(rep)
    return out


def _standstill(I, T):
    """Standstill at T: {"value", "for_s", "source"} from the best usable group that reports it."""
    best = None
    for name, q in I.still.items():
        g = I.groups[name]
        if g.fault is not None or not q or abs(T - q[-1][0]) > g.stale_s and T > q[-1][0]:
            continue
        prev = nxt = None
        for s in reversed(q):
            if s[0] <= T:
                prev = s
                break
            nxt = s
        a, b = prev or nxt, nxt or prev
        if min(abs(T - s[0]) for s in (prev, nxt) if s is not None) > max(g.stale_s, 0.5):
            continue
        stands = a[1] is not None and b[1] is not None
        cand = (g.rank, {"value": stands, "for_s": round(max(0.0, T - a[1]), 3) if stands else 0.0, "source": name})
        if best is None or cand[0] > best[0]:
            best = cand
    return best[1] if best else {"value": False, "for_s": 0.0, "source": None}


def _none(reason, age=None):
    return {"value": None, "std": None, "valid": False, "reason": reason, "mode": "none",
            "age_s": None if age is None else round(age, 3), "sources": []}


def _quantity(I, T, sm, i, kinds, digits, scale=1.0):
    c = I.cfg
    if sm is None:
        return _none("no_measurement")
    x, P = sm
    age, srcs = I.ekf.near(T, c["fresh_s"], kinds, c["max_coast_s"] + 0.01)
    if age is None or age > c["max_coast_s"]:
        why = "no_measurement"
        if any(g.fault is not None and g.last_t is not None and I.now - g.last_t <= g.stale_s
               for g in I.groups.values()):
            why = "no_healthy_source"
        return _none(why, age)
    return {"value": round(float(x[i]) * scale, digits), "std": round(math.sqrt(max(float(P[i, i]), 0.0)) * scale, digits),
            "valid": True, "reason": None, "mode": "fused" if age <= c["fresh_s"] else "coast",
            "age_s": round(age, 3), "sources": srcs}


def _position(I, T, sm, speed):
    c = I.cfg
    if sm is None or not speed["valid"] and not I.anchored:
        return _pos_none("no_measurement"), _none("no_measurement")
    x, P = sm
    local = {"frame": I.frame_id, "e_m": round(float(x[E.PE]), 2), "n_m": round(float(x[E.PN]), 2),
             "heading_rad": round(float(x[E.PSI]), 4)}
    if not I.anchored:
        p = _pos_none("no_absolute_source")
        p["local"] = local
        return p, _none("no_absolute_source")
    # a fix describes the car `delay` seconds before it arrives: the newest row is that much older than the newest
    # fix. Freshness is about arrival (before L43 a receiver with a 2 s delay and one fix a second sat on the edge
    # of the window, and the mode flipped between `fused` and `coast` every second).
    fresh = c["position_fresh_s"] + getattr(I, "abs_delay", 0.0)
    age, srcs = I.ekf.near(T, fresh, ("pos",), 1e9)
    cee, cen, cnn = float(P[E.PE, E.PE]), float(P[E.PE, E.PN]), float(P[E.PN, E.PN])
    tr, det = cee + cnn, cee * cnn - cen * cen
    hacc = math.sqrt(max(0.5 * tr + math.sqrt(max(0.25 * tr * tr - det, 0.0)), 0.0))     # 1-sigma semi-major
    lat, lon = E.to_geo(I.origin, float(x[E.PE]), float(x[E.PN]))
    hstd = math.degrees(math.sqrt(max(float(P[E.PSI, E.PSI]), 0.0)))
    hsrc = I.ekf.near(T, fresh, ("pos", "heading"), 1e9)[1]
    head = {"value": round(E.psi_to_course(float(x[E.PSI])), 2), "std": round(hstd, 2), "valid": hstd <= 45.0,
            "reason": None if hstd <= 45.0 else "heading_uncertain", "mode": "fused" if hsrc else "coast",
            "age_s": None if age is None else round(age, 3), "sources": hsrc, "unit": "deg clockwise from north"}
    if hacc > c["position_max_hacc_m"]:
        p = _pos_none("dead_reckoning_ran_out")
        p["local"] = local
        return p, head
    mode = "fused" if age is not None and age <= fresh else "coast"
    return {"valid": True, "reason": None, "mode": mode, "lat": round(lat, 7), "lon": round(lon, 7),
            "alt_m": round(float(x[E.Z]), 1), "hacc_m": round(hacc, 2),
            "cov_en": [round(cee, 3), round(cen, 3), round(cnn, 3)], "fix_age_s": None if age is None else round(age, 2),
            "sources": srcs, "local": local,
            "origin": {"lat": I.origin[0], "lon": I.origin[1]}}, head


def _pos_none(reason):
    return {"valid": False, "reason": reason, "mode": "none", "lat": None, "lon": None, "alt_m": None, "hacc_m": None,
            "cov_en": None, "fix_age_s": None, "sources": [], "local": None, "origin": None}


def calibration(I, x, P):
    sd = math.sqrt(max(float(P[E.SV, E.SV]), 0.0))
    out = {"odometry": {"speed_scale": {"value": round(float(x[E.SV]), 5), "std": round(sd, 5), "unit": "1",
                                        "known": sd < 0.5 * I.model.c["scale_sigma0"]},
                        "note": "speed is in the vehicle speed signal's scale; ground distance = speed / (1 + speed_scale)"}}
    for s in I.specs:
        d = {}
        for a in s.aux:
            i = s.ix[a.name]
            sd = math.sqrt(max(float(P[i, i]), 0.0))
            d[a.name] = {"value": round(float(x[i]), 5), "std": round(sd, 5), "unit": a.unit,
                         "known": sd < 0.5 * a.sigma0}
        d.update(s.calib_state(I.ctx[s.id]) or {})
        if not d:
            continue
        if s.mount is not None:
            d["mount"] = s.mount.as_dict()
        out[s.id] = d
    return out


def ego_state(I, ts_mono_ns, ts_real_ns=None):
    T = ts_mono_ns / NS
    if I.now is None:
        I.now = T
    I.seq += 1
    sm = I.ekf.at(T)
    speed = _quantity(I, T, sm, E.V, _SPEED_KINDS, 3)
    accel = _quantity(I, T, sm, E.A, None, 3)
    if not speed["valid"]:
        accel = dict(accel, value=None, std=None, valid=False, reason=speed["reason"], mode="none")
    yr = _quantity(I, T, sm, E.W, _YAW_KINDS, 5)
    still = _standstill(I, T)
    if still["value"] and still["source"] is not None and speed["valid"] and I.still_exact.get(still["source"]) and \
            any(s.startswith(still["source"] + ".") for s in speed["sources"]) and abs(speed["value"]) < 0.05:
        speed = dict(speed, value=0.0, std=0.0)      # the wheels stand: exactly zero, not a small number
    dsp = I.disputed.get("speed")
    if dsp is not None and speed["valid"]:
        # two sources of the speed disagree and no third has decided yet: the value is the higher-ranked one's,
        # its uncertainty covers both readings
        speed = dict(speed, std=round(max(speed["std"], 0.5 * dsp[2]), 3), mode="disputed",
                     reason="%s and %s disagree, vote pending" % (dsp[0], dsp[1]))
    pos, head = _position(I, T, sm, speed)
    grade = _quantity(I, T, sm, E.TH, None, 4) if speed["valid"] else _none(speed["reason"])
    # odometer: the path length of the published speed (a consumer's distance base)
    if speed["valid"]:
        if I._odo_t is not None and 0.0 < T - I._odo_t[0] <= 1.0:
            I.odometer += 0.5 * (abs(speed["value"]) + abs(I._odo_t[1])) * (T - I._odo_t[0])
        if I._odo_t is None or T > I._odo_t[0]:
            I._odo_t = (T, speed["value"])
    sig = {}
    for name, (t, v) in I.signals.items():
        sig[name] = {"value": v, "age_s": round(max(0.0, T - t), 3)}
    x, P = sm if sm is not None else (I.x_now, I.P_now)
    rej = dict(I.ekf.rejected)
    # the tables (calibration, per-source health, the monitors' verdicts, findings) are rebuilt five times a
    # second and at once when anything about a fault changes; the quantities above are fresh every frame
    key = (I.epoch, id(I.ekf), tuple(sorted((n, g.fault["rule"]) for n, g in I.groups.items() if g.fault is not None)),
           tuple(sorted(I.qfaults)), tuple(sorted(I.disputed)), len(I.fdi.findings), len(I.fdi.events), len(I.pairs),
           tuple(sorted(I.fdi.reported)))
    tb = getattr(I, "_tables", None)
    if tb is None or tb[0] != key or not (0.0 <= T - tb[1] < TABLES_S):
        tb = I._tables = (key, T, {
            "calibration": calibration(I, x, P),
            "sources": {n: _health(I, g, T) for n, g in I.groups.items()},
            "checks": [p.report() for p in I.pairs.values()] + [s.report() for s in I.selfs.values()]
            + [m.report() for m in I.nis.values() if m.value is not None],
            "diagnostics": I.fdi.report(T),
            "detectors": dict(I.detectors)})
    tables = tb[2]
    return {"type": "ego_state", "api": 3, "seq": I.seq, "ts_mono_ns": ts_mono_ns, "ts_real_ns": ts_real_ns,
            "epoch": I.epoch, "speed": speed, "accel": accel, "yaw_rate": yr, "standstill": still,
            "position": pos, "heading": head, "grade": grade, "odometer_m": round(I.odometer, 2),
            "signals": sig, "calibration": tables["calibration"], "sources": tables["sources"], "checks": tables["checks"],
            "diagnostics": tables["diagnostics"], "detectors": tables["detectors"], "tables_age_s": round(T - tb[1], 3),
            "reanchors": getattr(I, "reanchors", 0), "rejected": rej, "late": dict(I.counts["late"]), "dropped": dict(I.counts["dropped"])}
