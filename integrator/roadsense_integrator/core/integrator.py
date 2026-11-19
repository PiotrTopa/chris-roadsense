"""The integrator: source registry, ego estimator, monitors and fault isolation, object layer, outputs
(docs/INTEGRATOR.md). No I/O and no wall clock: times come from the messages (`ts_mono_ns`)."""
import collections
import math

import numpy as np

from . import ego as E
from .ekf import Filter, Row, wrap_pi
from .monitors import NisMonitor, Pair, SelfCheck, Series
from .registry import NS, Ctx, Meas, lin

API = 3

DEFAULT_CFG = dict(
    horizon_s=10.0,              # measurements are kept (and may still be inserted) this long
    gate_chi2=23.9,              # hard gate of robust rows (chi-square 1 dof), while an exact source anchors them
    process={},                  # overrides of ego.DEFAULT_PROCESS
    fresh_s=0.3,                 # a quantity with a measurement this near the requested time is `fused`
    max_coast_s=1.0,             # ... further than this it is unknown
    epoch_back_s=2.0,            # a source's clock going back by more than this starts a new epoch
    camera_silent_s=0.3,         # without road frames for this long the state is also published on vehicle samples
    position_fresh_s=3.0,        # position `fused` while an absolute fix is this fresh, else `coast` (dead reckoning)
    position_max_hacc_m=75.0,    # dead reckoning beyond this is unknown
    anchor_min_speed=3.0,        # a course initialises the heading only above this speed
    anchor_chord_m=15.0,         # without a course: two fixes this far apart do
    epoch_pos_sigma_m=10.0, epoch_heading_sigma=0.3,
    pair=dict(                   # pair monitors (defaults; per quantity below)
        mode="share", lag_s=0.0, max_gap_s=0.5, pending_s=1.0, max_step_s=0.25,
        z_bad=4.0, fire_window_s=5.0, min_n=15, fire_share=0.5, suspect_share=0.35,
        recover_window_s=10.0, recover_min_n=30, recover_share=0.25),
    pair_quantity=dict(
        speed=dict(min_evidence=3.0, fault=True),
        yaw_rate=dict(min_evidence=0.05, fault=False),      # a disagreement makes the source `suspect` only
    ),
    witness=dict(                # sources with poor timing: ratio of medians over a long window
        mode="ratio", lag_s=3.0, max_gap_s=0.5, pending_s=1.0, max_step_s=3.0, z_bad=4.0,
        fire_window_s=30.0, min_n=10, ratio_lo=0.6, ratio_hi=1.6, min_evidence=3.0,
        recover_window_s=30.0, recover_min_n=10, recover_share=1.0, fire_share=1.0, suspect_share=1.0, fault=False),
    self_check=dict(fire_window_s=5.0, min_n=15, fire_ratio=4.0, recover_window_s=10.0, recover_min_n=30,
                    recover_ratio=3.0, max_step_s=0.25),
    nis=dict(window_s=8.0, max_n=80, min_n=8, fire=6.0, recover=3.0),
    # row classes that are set aside while their innovations are far beyond the stated uncertainty (their rows
    # are still evaluated, so the class comes back by itself): the classes with no raw partner to be judged by
    quarantine=("pos", "heading", "alt", "map_cross", "map_heading", "mount"),
    objects={},                  # overrides of objects.DEFAULT_CFG
    lead={},                     # the camera-lead source's constants (sources/camera_lead.py) and the lead message's
    sources=None,                # extra sources of this car: [{"kind": "radar", "id": ..., "mount": {...}}, ...]
)

_IV, _IA, _IW, _IAL = (np.array([i], dtype=np.intp) for i in (E.V, E.A, E.W, E.AL))
_ONE = np.array([1.0])


def _zero(idx):
    def h(x):
        return float(x[idx[0]]), idx, _ONE
    return h


_H0 = {"v": _zero(_IV), "a": _zero(_IA), "w": _zero(_IW), "al": _zero(_IAL)}


class _Group:
    def __init__(self, name, rank, witness, stale_s, family=None):
        self.name, self.rank, self.witness, self.stale_s = name, rank, witness, stale_s
        self.family = family or name
        self.arrivals = collections.deque(maxlen=64)
        self.last_t = None
        self.fault = None                  # {"since", "reason", "rule"} or None
        self.carried = False
        self.series = {}                   # quantity -> Series
        self.sources = []

    def reset_data(self):
        self.arrivals.clear()
        self.last_t = None
        for s in self.series.values():
            s.clear()


class Integrator:
    def __init__(self, cfg=None, sources=None, objects=True):
        self.cfg = dict(DEFAULT_CFG)
        for k, v in (cfg or {}).items():
            self.cfg[k] = dict(self.cfg[k], **v) if isinstance(self.cfg.get(k), dict) and isinstance(v, dict) else v
        self.model = E.EgoModel(self.cfg["process"])
        self.specs, self.by_type, self.groups, self.ctx = [], collections.defaultdict(list), {}, {}
        self.pairs, self.selfs, self.nis = {}, {}, {}
        self.seq = 0
        self.epoch = 0
        self.frame_id = 0                  # bumps when the local position frame is re-anchored
        self.origin = None                 # (lat, lon) of the local frame
        self.anchored = False
        self.counts = {"late": {}, "dropped": {}}
        self.odometer = 0.0
        self._odo_t = None
        self.objects = None
        if sources is None:
            from ..sources import default_sources
            sources = default_sources(self.cfg)
        for s in sources:
            self.register(s, _build=False)
        self._build()
        if objects:
            from .objects import ObjectLayer
            self.objects = ObjectLayer(self.cfg["objects"])
            self.objects.lead_cfg = self.cfg["lead"]

    # ---- registry ------------------------------------------------------------------------------------
    def register(self, src, _build=True):
        """Add a source: its input description, measurement model and calibration states."""
        self.specs.append(src)
        for t in src.types:
            self.by_type[t].append(src)
        g = self.groups.get(src.group)
        if g is None:
            g = self.groups[src.group] = _Group(src.group, src.rank, src.witness, src.stale_s, src.family)
        g.sources.append(src)
        for a in src.aux:
            src.ix[a.name] = self.model.add_aux(src.id, a)
        self.ctx[src.id] = Ctx(self, src)
        if _build:
            self._build(keep=True)

    def _build(self, keep=False):
        old = getattr(self, "ekf", None)
        x, P = self.model.initial()
        if keep and old is not None and old.latest() is not None:      # a source added while running
            t, xo, Po = old.latest()
            n = len(xo)
            x[:n] = xo
            P[:n, :n] = Po
        c = self.cfg
        self._foreign_ix = {}
        self.ekf = Filter(self.model, x, P, c["horizon_s"], c["gate_chi2"])
        self.ekf.stat = self._nis_stat
        if keep and old is not None and old.latest() is not None:
            self.ekf.rebase(t, x, P)
        if not keep:
            self._init_state()
        self._cache()

    def _init_state(self):
        self.now = None
        self.src_t = {}
        self.still = {}                    # group -> deque of (t, standing since t or None)
        self.still_exact = {}              # group -> its standstill is known (wheels), not inferred
        self.signals = {}                  # name -> (t, value)
        self.last_frame_t = None
        self._anchor_first = None

    def _cache(self):
        l = self.ekf.latest()
        if l is None:
            self.x_now, self.P_now = self.ekf.x0, self.ekf.P0
        else:
            self.x_now, self.P_now = l[1], l[2]

    def carry_fault(self, group, reason):
        """A fault known from before this run: the group starts faulted until its monitors see agreement."""
        g = self.groups.get(group)
        if g is not None and g.fault is None:
            g.fault = {"since": None, "reason": reason, "rule": "carried"}
            g.carried = True
            self._apply_faults()

    def calibration_snapshot(self):
        """Calibration states and their covariance block, to carry over a restart."""
        l = self.ekf.latest()
        if l is None:
            return None
        _, x, P = l
        names = ["sv"] + [name for name, _ in self.model.aux]
        return {name: [float(x[self.model.index[name]]), float(P[self.model.index[name], self.model.index[name]])]
                for name in names}

    def calibration_restore(self, snap):
        if not snap or self.ekf.base is not None:
            return
        x, P = self.ekf.x0, self.ekf.P0
        for name, (val, var) in snap.items():
            i = self.model.index.get(name)
            if i is not None and math.isfinite(val) and var > 0:
                x[i], P[i, i] = val, var
        self._cache()

    # ---- input ---------------------------------------------------------------------------------------
    def on_message(self, msg):
        """Feed any input message; messages of types no source claims are ignored."""
        specs = self.by_type.get(msg.get("type")) if isinstance(msg, dict) else None
        if not specs:
            return
        ts = msg.get("ts_mono_ns")
        if not isinstance(ts, int):
            return
        t = ts / NS
        fused = False
        for spec in specs:
            last = self.src_t.get(spec.id)
            if last is not None and t <= last:
                if t < last - self.cfg["epoch_back_s"]:
                    self._new_epoch()
                else:
                    self.counts["dropped"][spec.id] = self.counts["dropped"].get(spec.id, 0) + 1
                    continue
            self.src_t[spec.id] = t
            if self.now is None or t > self.now:
                self.now = t
            g = self.groups[spec.group]
            ctx = self.ctx[spec.id]
            ms = spec.measure(msg, ctx)
            if ms:
                if spec.counts:
                    g.arrivals.append(t)
                    g.last_t = t if g.last_t is None else max(g.last_t, t)
                for m in ms:
                    fused |= self._measure(spec, g, m)
            if self.objects is not None:
                sc = spec.scan(msg, ctx)
                if sc is not None:
                    if spec.counts and not ms:
                        g.arrivals.append(t)
                        g.last_t = t if g.last_t is None else max(g.last_t, t)
                    if g.fault is None:
                        self.objects.add_scan(sc)
        if fused:
            self.ekf.prune(self.now)
            self._cache()

    def _new_epoch(self):
        """The device clock restarted: drop what is tied to the old time axis. Calibration states and their
        covariance stay; a fault survives as carried; the position stays with its uncertainty inflated."""
        self.epoch += 1
        l = self.ekf.latest()
        x, P = self.model.initial()
        if l is not None:
            _, xo, Po = l
            n0 = E.NCORE
            x[n0:] = xo[n0:]
            P[n0:, n0:] = Po[n0:, n0:]
            x[E.SV], P[E.SV, E.SV] = xo[E.SV], Po[E.SV, E.SV]
            if self.anchored:
                c = self.cfg
                x[E.PE], x[E.PN], x[E.PSI], x[E.Z] = xo[E.PE], xo[E.PN], xo[E.PSI], xo[E.Z]
                P[E.PE, E.PE] = Po[E.PE, E.PE] + c["epoch_pos_sigma_m"] ** 2
                P[E.PN, E.PN] = Po[E.PN, E.PN] + c["epoch_pos_sigma_m"] ** 2
                P[E.PE, E.PN] = P[E.PN, E.PE] = Po[E.PE, E.PN]
                P[E.PSI, E.PSI] = Po[E.PSI, E.PSI] + c["epoch_heading_sigma"] ** 2
                P[E.Z, E.Z] = Po[E.Z, E.Z] + 25.0
        self.ekf.reset(x, P)
        self._init_state()
        self._odo_t = None
        for s in self.specs:
            self.ctx[s.id].mem = {}
            s.reset()
        for g in self.groups.values():
            g.reset_data()
            if g.fault is not None:
                g.fault = dict(g.fault, since=None, rule="carried")
                g.carried = True
        for p in self.pairs.values():
            p.reset_data()
            if p.verdict == "agree":
                p.verdict = "unknown"
        for s in list(self.selfs.values()) + list(self.nis.values()):
            s.reset_data()
        if self.objects is not None:
            self.objects.reset()
        self._apply_faults()
        self._cache()

    def _measure(self, spec, g, m):
        """Returns True when a row went into the filter."""
        role = m.role
        if role == "signal":
            self.signals[m.q[7:]] = (m.t, m.z)
            return False
        if role == "flag":                 # standstill
            q = self.still.setdefault(g.name, collections.deque(maxlen=64))
            self.still_exact[g.name] = m.sigma is None
            since = None
            if m.z:
                since = q[-1][1] if q and q[-1][1] is not None else m.t
            q.append((m.t, since))
            if m.z and m.src is not None and not g.witness:
                # exact for a source that knows (wheels at zero); a source that infers it gives its sigma
                k = 1.0 if m.sigma is None else (m.sigma / 1e-3) ** 2
                for name, var in (("v", 1e-6), ("a", 1e-4), ("w", 1e-8), ("al", 1e-4)):
                    self._fuse(m.t, Row("standstill" if name == "v" else "standstill_" + name, 0.0, var * k, _H0[name],
                                        m.src, g.name, robust=m.sigma is not None))
                return True
            return False
        if role == "self":
            q = m.q[5:]
            sc = self.selfs.get((g.name, q))
            if sc is None:
                sc = self.selfs[(g.name, q)] = SelfCheck(g.name, q, self.cfg["self_check"])
            if sc.offer(m.t, m.z):
                self._isolate()
            return False
        fused = False
        if role == "fuse" and not g.witness:
            self._fuse(m.t, Row(m.q, m.z, m.sigma * m.sigma, m.h, m.src, g.name, robust=m.robust, wrap=m.wrap,
                                cb=m.cb, freeze=self._freeze(spec, m)))
            fused = True
        if m.mon is not None or (role == "monitor"):
            v, s = m.mon if m.mon is not None else (m.z, m.sigma)
            self._monitor(g, m.q, m.t, v, s)
        return fused

    def _freeze(self, spec, m):
        f = self._foreign(spec) if m.local else None
        if m.consider:
            c = np.array(list(m.consider), dtype=np.intp)
            f = c if f is None else np.concatenate([f, c])
        return f

    def _foreign(self, spec):
        """Indices of the calibration states that belong to other sources."""
        f = self._foreign_ix.get(spec.id)
        if f is None:
            own = set(spec.ix.values())
            f = self._foreign_ix[spec.id] = np.array([E.SV] + [i for i in range(E.NCORE, self.model.n) if i not in own],
                                                     dtype=np.intp)
        return f

    def _fuse(self, t, row):
        if not self.ekf.insert(t, row):
            self.counts["late"][row.src] = self.counts["late"].get(row.src, 0) + 1

    def _nis_stat(self, row, d2, t):
        key = (row.group, row.q)
        m = self.nis.get(key)
        if m is None:
            m = self.nis[key] = NisMonitor(row.group, row.q, self.cfg["nis"])
        if m.offer(t, d2) and row.q in self.cfg["quarantine"]:
            self.ekf.set_quarantine(k for k, n in self.nis.items() if n.high and k[1] in self.cfg["quarantine"])

    def _monitor(self, g, q, t, v, s):
        ser = g.series.get(q)
        if ser is None:
            ser = g.series[q] = Series()
            for o in self.groups.values():           # a new (group, quantity): pair it with the others that have it
                if o is g or q not in o.series or (o.witness and g.witness):
                    continue
                hi, lo = (o, g) if (o.rank, o.name) > (g.rank, g.name) else (g, o)
                wit = hi.witness or lo.witness
                cfg = dict(self.cfg["witness"]) if wit else dict(self.cfg["pair"], **self.cfg["pair_quantity"].get(q, {}))
                self.pairs[(hi.name, lo.name, q)] = Pair(hi.name, lo.name, q, cfg)
        ser.add(t, v, s)
        changed = False
        for (hi, lo, pq), p in self.pairs.items():
            if pq != q:
                continue
            if lo == g.name:
                changed |= p.offer(t, v, s, self.groups[hi].series[q], self.now)
            elif hi == g.name and p.pending:
                changed |= p.flush(ser, self.now)
        if changed:
            self._isolate()

    # ---- absolute position (any receiver) --------------------------------------------------------------
    def abs_position(self, src, t_stamp, lat, lon, sigma, itau=None, course_deg=None, course_ok=False, cb=None,
                     bias=None):
        """Rows of an absolute position fix (list of Meas). Sets the local origin on the first fix and anchors
        the local frame (position and heading) once a heading is known: a usable course, or two fixes far
        enough apart. Until then nothing is fused. `itau`: index of the source's delay state; `bias`: indices
        of its east / north error states (a receiver's error correlated over tens of seconds)."""
        c = self.cfg
        if self.origin is None:
            self.origin = (lat, lon)
        e, n = E.to_local(self.origin, lat, lon)
        x = self.x_now
        tau = float(x[itau]) if itau is not None else 0.0
        if not self.anchored:
            psi = None
            if course_ok and course_deg is not None:
                psi = E.course_to_psi(course_deg)
            else:
                f = self._anchor_first
                if f is None:
                    self._anchor_first = (e, n)
                elif math.hypot(e - f[0], n - f[1]) >= c["anchor_chord_m"]:
                    psi = math.atan2(n - f[1], e - f[0])
            if psi is None:
                return []
            self._anchor(e, n, psi, sigma, tau)
            return []
        v, ps = float(x[E.V]) / (1.0 + float(x[E.SV])), float(x[E.PSI])
        idx = [E.PE] if itau is None else [E.PE, itau]
        idn = [E.PN] if itau is None else [E.PN, itau]
        ce = [1.0] if itau is None else [1.0, -v * math.cos(ps)]
        cn = [1.0] if itau is None else [1.0, -v * math.sin(ps)]
        if bias is not None:
            idx, idn, ce, cn = idx + [bias[0]], idn + [bias[1]], ce + [1.0], cn + [1.0]
        ke = tau * v * math.cos(ps) if itau is not None else 0.0      # so that the prediction is pe at insertion
        kn = tau * v * math.sin(ps) if itau is not None else 0.0
        t = t_stamp - tau
        return [Meas("pos", t, e, sigma, lin(idx, ce, ke), src + ".pos", robust=True, cb=cb),
                Meas("pos", t, n, sigma, lin(idn, cn, kn), src + ".pos", robust=True)]

    def _anchor(self, e, n, psi, sigma, tau):
        l = self.ekf.latest()
        if l is None:
            return
        t, x, P = l
        x, P = x.copy(), P.copy()
        v = float(x[E.V])
        dpsi = psi - x[E.PSI]
        x[E.PE], x[E.PN], x[E.PSI] = e + v * tau * math.cos(psi), n + v * tau * math.sin(psi), psi
        for i, var in ((E.PE, (2.0 * sigma) ** 2 + 4.0), (E.PN, (2.0 * sigma) ** 2 + 4.0), (E.PSI, 0.35 ** 2)):
            P[i, :] = 0.0
            P[:, i] = 0.0
            P[i, i] = var
        self.ekf.rebase(t, x, P)
        self.anchored = True
        self.frame_id += 1
        self._cache()

    def reset_aux(self, i, x0, sigma0):
        """Put one calibration state back to a prior (its source says what it describes has changed)."""
        l = self.ekf.latest()
        if l is None:
            self.ekf.x0[i] = x0
            self.ekf.P0[i, :] = 0.0
            self.ekf.P0[:, i] = 0.0
            self.ekf.P0[i, i] = sigma0 ** 2
        else:
            t, x, P = l
            x, P = x.copy(), P.copy()
            x[i] = x0
            P[i, :] = 0.0
            P[:, i] = 0.0
            P[i, i] = sigma0 ** 2
            self.ekf.rebase(t, x, P)
        self._cache()

    # ---- isolation -----------------------------------------------------------------------------------
    def _verdict(self, a, b, q):
        p = self.pairs.get((a, b, q)) or self.pairs.get((b, a, q))
        return p.verdict if p is not None else "unknown"

    def _isolate(self):
        """Which groups are faulted, from the monitors' verdicts (docs/INTEGRATOR.md section 5)."""
        found = {}
        for (gname, q), sc in self.selfs.items():
            if sc.fault:
                found.setdefault(gname, []).append(("self", sc.describe()))
        for (hi, lo, q), p in self.pairs.items():
            if p.verdict != "disagree" or not p.c.get("fault") or p.c["mode"] == "ratio":
                continue
            against_hi = against_lo = 0
            for c, cg in self.groups.items():
                # a witness is reported, it does not vote; neither does a group that shares its sensors with one
                # of the two
                if c in (hi, lo) or cg.witness or cg.family in (self.groups[hi].family, self.groups[lo].family):
                    continue
                v_lo, v_hi = self._verdict(c, lo, q), self._verdict(c, hi, q)
                if v_lo == "agree" and v_hi == "disagree":
                    against_hi += 1
                elif v_hi == "agree" and v_lo == "disagree":
                    against_lo += 1
            if against_hi > against_lo:
                culprit, other, rule = hi, lo, "vote"
            else:
                culprit, other, rule = lo, hi, "vote" if against_lo else "rank"
            txt = "%s disagrees with %s" % (q, other) + p.describe()[len("%s disagrees with %s" % (q, p.ref)):]
            found.setdefault(culprit, []).append((rule, txt))
        for name, g in self.groups.items():
            f = found.get(name)
            if f:
                rules = [r for r, _ in f]
                rule = "vote" if "vote" in rules else "rank" if "rank" in rules else "self"
                reason = "; ".join(t for _, t in f)
                since = g.fault["since"] if g.fault is not None and not g.carried else self.now
                g.fault = {"since": since, "reason": reason, "rule": rule}
                g.carried = False
            elif g.fault is not None and (not g.carried or self._cleared(name)):
                g.fault = None
                g.carried = False
        self._apply_faults()

    def _cleared(self, name):
        """A carried fault ends when a ranked monitor of the group has evidence of agreement."""
        ok = False
        for (hi, lo, q), p in self.pairs.items():
            if name in (hi, lo) and p.c.get("fault") and p.c["mode"] != "ratio":
                if p.verdict == "agree":
                    ok = True
        for (gname, q), sc in self.selfs.items():
            if gname == name and not sc.fault and sc.n >= sc.c["min_n"] and not any(
                    name in (hi, lo) and p.c.get("fault") for (hi, lo, _), p in self.pairs.items()):
                ok = True                  # no ranked partner at all: the self monitor's quiet is what there is
        return ok

    def _apply_faults(self):
        bad = [n for n, g in self.groups.items() if g.fault is not None]
        self.ekf.set_disabled(bad)
        if self.objects is not None:
            self.objects.set_faulted(bad)
        self._cache()

    def faults(self):
        return {n: dict(g.fault) for n, g in self.groups.items() if g.fault is not None}

    # ---- queries for sources and the object layer ------------------------------------------------------
    def speed_now(self):
        """(speed, std) of the newest filtered state, None before any speed measurement."""
        if self.ekf.base is None or self.P_now[E.V, E.V] > 100.0:
            return None
        return float(self.x_now[E.V]), math.sqrt(max(float(self.P_now[E.V, E.V]), 0.0))

    def ego_at(self, t):
        """(v, w, sigma_v, known) at time t for the object layer's prediction between frames (the filtered
        state: scans come many times a frame, and the frame's own output is the smoothed one)."""
        sm = self.ekf.filtered(t)
        if sm is None or sm[1][E.V, E.V] > 100.0:
            return 0.0, 0.0, 0.0, False
        x, P = sm
        return float(x[E.V]), float(x[E.W]), math.sqrt(max(float(P[E.V, E.V]), 0.0)), True

    def camera_silent(self):
        return self.last_frame_t is None or self.now - self.last_frame_t > self.cfg["camera_silent_s"]

    # ---- output --------------------------------------------------------------------------------------
    def state(self, ts_mono_ns, ts_real_ns=None):
        from . import output
        return output.ego_state(self, ts_mono_ns, ts_real_ns)

    def on_road(self, road):
        """Feed a `road` message and return the `ego_state` for its frame time (None without a time stamp)."""
        self.on_message(road)
        ts = road.get("ts_mono_ns")
        if not isinstance(ts, int):
            return None
        self.last_frame_t = ts / NS
        return self.state(ts, road.get("ts_real_ns"))

    def frame(self, road):
        """One road frame through the whole box: (`ego_state`, `tracks`, `lead`)."""
        es = self.on_road(road)
        if es is None:
            return None, None, None
        if self.objects is None:
            return es, None, None
        tr, ld = self.objects.frame(self, road, es)
        return es, tr, ld

    def tick(self, ts_mono_ns, ts_real_ns=None):
        """Outputs at a time without a road frame (the camera is silent): (`ego_state`, `tracks`, `lead`)."""
        es = self.state(ts_mono_ns, ts_real_ns)
        if self.objects is None:
            return es, None, None
        tr, ld = self.objects.frame(self, None, es)
        return es, tr, ld
