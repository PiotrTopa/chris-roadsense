"""Shared helper of the integrator's tests: run a simulated world through an `Integrator` and collect the
estimates next to the truth."""
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import numpy as np  # noqa: E402

from chris_integrator import Integrator  # noqa: E402
from chris_integrator.core import ego as E  # noqa: E402
from chris_integrator.sim import World, default_sensors  # noqa: E402


def wrap(a):
    return (a + math.pi) % (2.0 * math.pi) - math.pi


class Run:
    """rows: per road frame a dict of errors and stds; leads: per frame (t, lead message, true lead or None);
    tracks: per frame (t, tracks message)."""

    def __init__(self, world=None, sensors=None, integ=None, keep_tracks=False, order=None, on_frame=None, **wkw):
        self.w = world or World(**wkw)
        self.I = integ or Integrator({"gnss_absolute": True})     # the simulated receiver is a decent one
        self.rows, self.leads, self.tracks, self.es = [], [], [], None
        w, I = self.w, self.I
        msgs = list(w.messages(sensors if sensors is not None else default_sensors()))
        if order is not None:
            msgs = order(msgs)
        for arr, m in msgs:
            if m["type"] != "road":
                I.on_message(m)
                continue
            es, tr, ld = I.frame(m)
            if es is None:
                continue
            self.es = es
            T = m["ts_mono_ns"] / 1e9
            tru = w.truth_at(T)
            row = {"t": T - w.t0, "es": es if on_frame else None, "v_true": tru["v"], "w_true": tru["w"],
                   "vision": es["sources"]["vision"]["state"], "vehicle": es["sources"]["vehicle"]["state"]}
            sm = I.ekf.at(T)
            if es["speed"]["valid"] and sm is not None:
                x, P = sm
                row.update(dv=es["speed"]["value"] - (1.0 + w.speed_scale) * tru["v"], sv=max(es["speed"]["std"], 1e-3),
                           dw=x[E.W] - tru["w"], sw=math.sqrt(P[E.W, E.W]), dth=x[E.TH] - tru["th"],
                           sth=math.sqrt(P[E.TH, E.TH]), mode=es["speed"]["mode"])
                if I.anchored:
                    c, s = math.cos(tru["psi"]), math.sin(tru["psi"])
                    de, dn = x[E.PE] - tru["pe"], x[E.PN] - tru["pn"]
                    Pp = P[:2, :2]
                    d = np.array([de, dn])
                    row.update(de=de, dn=dn, along=c * de + s * dn, cross=-s * de + c * dn,
                               nees_pos=float(d @ np.linalg.solve(Pp, d)), dpsi=wrap(x[E.PSI] - tru["psi"]),
                               spsi=math.sqrt(P[E.PSI, E.PSI]), pos_valid=es["position"]["valid"],
                               hacc=es["position"]["hacc_m"], pos_mode=es["position"]["mode"])
            self.rows.append(row)
            if ld is not None:
                self.leads.append((T - w.t0, ld, w.lead_at(T)))
            if keep_tracks and tr is not None:
                self.tracks.append((T - w.t0, tr))
            if on_frame:
                on_frame(self, T - w.t0, es, tr, ld)

    def col(self, k, t0=0.0, t1=1e9):
        return np.array([r[k] for r in self.rows if k in r and t0 <= r["t"] < t1], float)

    def nees(self, d, s, t0=0.0, t1=1e9):
        e, sd = self.col(d, t0, t1), self.col(s, t0, t1)
        return float(np.mean((e / sd) ** 2))

    def cal(self, src, name):
        c = self.es["calibration"][src][name]
        return c["value"], c["std"]
