"""The integrator: source registry, ego estimator, monitors and fault isolation, object layer, outputs
(docs/INTEGRATOR.md). No I/O and no wall clock: times come from the messages (`ts_mono_ns`)."""
import collections
import math

import numpy as np

from . import ego as E
from .ekf import Filter, Row, wrap_pi
from .fdi import Diagnostics
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
        # `fault`: True = a disagreement is isolated by vote, else by rank; "vote" = by vote only (else `suspect`)
        # The CUSUM limits were first set on the simulator, where they catch a 10 % scale error and a gyro bias of
        # 0.02 rad/s. On the reference car's two healthy drives those limits raised alarms (the camera's speed
        # carries a slow bias of up to 0.9 m/s, the three yaw sensors differ by 5-9 % in scale and more in
        # transients): the limits below are the ones that are quiet on those drives. They are tuned to that
        # thin data; what they cost in sensitivity is in the fault table.
        speed=dict(min_evidence=3.0, fault=True, cusum_k=1.5, cusum_h=25.0, abs_delta=1.2, min_delta=0.10),
        yaw_rate=dict(min_evidence=0.05, fault="vote", cusum_k=1.0, cusum_h=20.0, abs_delta=0.015, min_delta=0.30),
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
    quarantine=("pos", "heading", "alt", "map_cross", "map_heading", "mount_yaw"),
    reanchor_s=8.0,              # absolute positions set aside this long = the position is lost: anchor again
    vote_wait_s=25.0,            # a disagreement of two sources waits this long at most for a third source's evidence
    vote_settle_s=1.0, vote_fresh_n=2,     # ... which must be this many samples valid this long after the disagreement began
    gnss_absolute=False,         # may the receiver's positions teach the speed scale and the yaw scales
                                 # (a good receiver: yes; one that scatters 10-15 m with a claimed 3 m: no)
    fdi={},                      # overrides of fdi.DEFAULT_CFG (signal, timing and trend detectors)
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
        self.fdi = Diagnostics(self.cfg["fdi"])
        self.specs, self.by_type, self.groups, self.ctx = [], collections.defaultdict(list), {}, {}
        self.pairs, self.selfs, self.nis = {}, {}, {}
        self.qfaults = {}                  # (group, quantity) -> {"since", "reason", "rule"}: that quantity's rows are out
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
        self.ekf.stat_nu = self._innovation
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
        self._pos_high_t = None
        self._lost = False
        self.disputed = {}
        self._dispute_t = -1e18
        self.qfaults = getattr(self, "qfaults", {})

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
        self.fdi.stamp((msg["type"], msg.get("sensor")), specs[0].group, t)
        for spec in specs:
            # ordering is per stream: several sensors may share a message type (`objects` of each radar and camera)
            key = spec.id if "sensor" not in msg else (spec.id, msg["sensor"])
            last = self.src_t.get(key)
            if last is not None and t <= last:
                if t < last - self.cfg["epoch_back_s"]:
                    self._new_epoch()
                else:
                    self.counts["dropped"][spec.id] = self.counts["dropped"].get(spec.id, 0) + 1
                    continue
            self.src_t[key] = t
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
        if self.disputed and self.now - self._dispute_t > 0.5:      # a pending vote has a deadline
            self._dispute_t = self.now
            self._isolate()
        if fused:
            self.ekf.prune(self.now)
            self._cache()
            if self._lost:
                self._reanchor()

    def _new_epoch(self):
        """The device clock restarted: drop what is tied to the old time axis. Calibration states and their
        covariance stay; a fault survives as carried; the position stays with its uncertainty inflated."""
        self.epoch += 1
        self.fdi.reset()
        self.fdi.event("clock", "epoch", None, "the device clock restarted (epoch %d): buffers dropped, calibration kept" % self.epoch,
                       quantity="time")
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
                # a standstill says what the motion is, nothing about any calibration: scale and bias states are
                # consider states of these rows (the sources' own rows learn their biases from the known zero).
                # Correlations built at speed would otherwise be cashed in at v = 0, where they mean nothing.
                fz = self._aux_ix()
                for name, var in (("v", 1e-6), ("a", 1e-4), ("w", 1e-8), ("al", 1e-4)):
                    self._fuse(m.t, Row("standstill" if name == "v" else "standstill_" + name, 0.0, var * k, _H0[name],
                                        m.src, g.name, robust=m.sigma is not None, freeze=fz))
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
            sig = m.sigma * self.fdi.noise_scale(g.name, m.q)     # a noise burst: weighted by its present noise
            self._fuse(m.t, Row(m.q, m.z, sig * sig, m.h, m.src, g.name, robust=m.robust, wrap=m.wrap,
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

    def _aux_ix(self):
        f = self._foreign_ix.get(None)
        if f is None:
            f = self._foreign_ix[None] = np.array([E.SV] + list(range(E.NCORE, self.model.n)), dtype=np.intp)
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
            if row.q == "pos":
                self._pos_high_t = t if m.high else None
        if row.q == "pos" and m.high and self._pos_high_t is not None and t - self._pos_high_t > self.cfg["reanchor_s"]:
            self._lost = True              # acted on after this filter pass (on_message)

    def _innovation(self, row, nu, s, t):
        if not row.q.startswith("standstill"):
            self.fdi.innovation(row.group, row.q, t, nu, s)

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
                for gg in (hi, lo):          # a source may say how it is to be compared (a slow, noisy one: by its mean)
                    if not wit:
                        cfg.update(getattr(gg.sources[0], "pair", None) or {})
                self.pairs[(hi.name, lo.name, q)] = Pair(hi.name, lo.name, q, cfg)
        ser.add(t, v, s)
        changed = self.fdi.raw(g.name, q, t, v, s)
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
            self._anchor(e, n, psi, sigma, tau, itau)
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
        loc = not c["gnss_absolute"]
        return [Meas("pos", t, e, sigma, lin(idx, ce, ke), src + ".pos", robust=True, cb=cb, local=loc),
                Meas("pos", t, n, sigma, lin(idn, cn, kn), src + ".pos", robust=True, local=loc)]

    def _reanchor(self):
        """The absolute positions have disagreed with the estimate for too long to be the receiver's fault alone:
        the position is declared lost and anchored again on the next usable fix (a new local frame id)."""
        self._lost = False
        self._pos_high_t = None
        self.anchored = False
        self._anchor_first = None
        self.reanchors = getattr(self, "reanchors", 0) + 1
        for sp in self.specs:
            if hasattr(sp, "on_reanchor"):
                sp.on_reanchor(self.ctx[sp.id])
        self.fdi.event("position", "reanchor", None, "the absolute position disagreed with the estimate for %.0f s: position "
                       "declared lost and anchored again (which of the two was wrong is not known)" % self.cfg["reanchor_s"], quantity="position")
        for k, m in self.nis.items():
            if k[1] in ("pos", "heading", "map_cross", "map_heading"):
                m.reset_data()
        self.ekf.set_quarantine(k for k, n in self.nis.items() if n.high and k[1] in self.cfg["quarantine"])

    def _anchor(self, e, n, psi, sigma, tau, itau=None):
        """Place the local frame on a fix. The position is the fix carried forward by speed x delay along the
        heading, so it inherits the uncertainty of the delay (along the track) and of the heading (across it),
        correlated with those states: a delay and an along-track offset cannot be told apart at constant speed,
        and the covariance has to say so (the filter then learns the delay when the speed changes, and the
        position's uncertainty stays honest until it does)."""
        l = self.ekf.latest()
        if l is None:
            return
        t, x, P = l
        x, P = x.copy(), P.copy()
        v = float(x[E.V]) / (1.0 + float(x[E.SV]))
        c, s = math.cos(psi), math.sin(psi)
        x[E.PE], x[E.PN], x[E.PSI] = e + v * tau * c, n + v * tau * s, psi
        var_psi = 0.35 ** 2
        var_tau = float(P[itau, itau]) if itau is not None else 0.0
        if itau is not None:                       # the delay keeps its variance, loses stale correlations
            P[itau, :] = 0.0
            P[:, itau] = 0.0
            P[itau, itau] = var_tau
        for i, var in ((E.PE, (2.0 * sigma) ** 2 + 4.0), (E.PN, (2.0 * sigma) ** 2 + 4.0), (E.PSI, var_psi)):
            P[i, :] = 0.0
            P[:, i] = 0.0
            P[i, i] = var
        d = v * tau
        u, nrm = (c, s), (-s, c)                   # along the track, across it
        for a_, ia in ((u[0], E.PE), (u[1], E.PN)):
            for b_, ib in ((u[0], E.PE), (u[1], E.PN)):
                P[ia, ib] += v * v * var_tau * a_ * b_
        for a_, ia in ((nrm[0], E.PE), (nrm[1], E.PN)):
            for b_, ib in ((nrm[0], E.PE), (nrm[1], E.PN)):
                P[ia, ib] += d * d * var_psi * a_ * b_
            P[ia, E.PSI] = P[E.PSI, ia] = d * var_psi * a_
        if itau is not None:
            for a_, ia in ((u[0], E.PE), (u[1], E.PN)):
                P[ia, itau] = P[itau, ia] = v * var_tau * a_
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

    def _indep(self, c, a, b, q):
        """Is group c an independent opinion on quantity q beside a and b? Groups of one family share sensors,
        unless a source says its reading of q comes from a sensor of its own (`independent`)."""
        cg = self.groups[c]
        if c in (a, b) or cg.witness:
            return False
        for o in (a, b):
            if cg.family == self.groups[o].family and not (
                    q in getattr(cg.sources[0], "independent", ()) or q in getattr(self.groups[o].sources[0], "independent", ())):
                return False
        return True

    def _critical(self, group, q):
        """Does a fault of quantity q take the whole group out (its main signal), or only that quantity's rows?"""
        return q in getattr(self.groups[group].sources[0], "critical", ("speed",))

    def _isolate(self):
        """Which groups (or single quantities of a group) are faulted, from the monitors' verdicts and the
        detectors' findings (docs/DESIGN.md sections 5, 5a)."""
        found = {}                         # group -> [(rule, text)]: the whole group is out
        qfound = {}                        # (group, quantity) -> [(rule, text)]: only that quantity's rows
        self.disputed = {}

        def blame(group, q, rule, txt):
            if q is None or self._critical(group, q):
                found.setdefault(group, []).append((rule, txt))
            else:
                qfound.setdefault((group, q), []).append((rule, txt))
        for (gname, q), sc in self.selfs.items():
            if sc.fault:
                blame(gname, None, "self", sc.describe())
        # a source that shows a fault by itself (a frozen value, a noise burst) needs no second opinion, and no rank
        evident = set()
        for name, g in self.groups.items():
            for q, txt in (() if g.witness else self.fdi.self_evident(name)):
                evident.add((name, q))
                blame(name, q, "self", "%s %s" % (q, txt))
        for (hi, lo, q), p in self.pairs.items():
            mode = p.c.get("fault")
            if p.verdict != "disagree" or not mode or p.c["mode"] == "ratio":
                continue
            against_hi = against_lo = 0
            for c in self.groups:
                if not self._indep(c, hi, lo, q) or self.groups[c].fault is not None or (c, q) in self.qfaults:
                    continue                   # (a source that is itself set aside has no vote)
                v_lo, v_hi = self._verdict(c, lo, q), self._verdict(c, hi, q)
                if v_lo == "agree" and v_hi == "disagree":
                    against_hi += 1
                elif v_hi == "agree" and v_lo == "disagree":
                    against_lo += 1
            if against_hi > against_lo:
                culprit, other, rule = hi, lo, "vote"
            else:
                culprit, other, rule = lo, hi, "vote" if against_lo else "rank"
            if (other, q) in evident and (culprit, q) not in evident:
                continue                   # the partner is already known to be the faulty one
            cg = self.groups[culprit]
            known = (cg.fault is not None and not cg.carried) or (culprit, q) in self.qfaults
            if rule == "rank" and not known:
                # two sources disagree and rank alone would decide. If the higher-ranked one shows a condition of
                # its own, or an independent third source is about to have an opinion, nobody is blamed yet: the
                # quantity is published as disputed (its uncertainty covers both readings) until the vote is in
                if self.fdi.suspected(other) or self._third_pending(hi, lo, q, p):
                    self.disputed[q] = (hi, lo, abs(p.last_delta or 0.0))
                    continue
            if rule == "rank" and mode == "vote":
                continue                   # this quantity is isolated by vote only: suspect, nobody blamed
            if not known:
                # blaming anew needs a disagreement that is still going on (a verdict held only by its
                # recovery hysteresis is not one) ...
                sh = p.stats.get("share")
                if sh is not None and sh <= p.c["recover_share"] and not p.by_cusum:
                    continue
                # ... and, by rank alone, no independent third source that sides with the lower-ranked one and
                # is drifting away from the higher: then the vote is still forming and nobody is blamed yet
                if rule == "rank" and self._vote_forming(hi, lo, q):
                    continue
            txt = "%s disagrees with %s" % (q, other) + p.describe()[len("%s disagrees with %s" % (q, p.ref)):]
            blame(culprit, q, rule, txt)
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
        old = self.qfaults
        self.qfaults = {k: {"since": old[k]["since"] if k in old else self.now, "reason": "; ".join(t for _, t in f),
                            "rule": "vote" if any(r == "vote" for r, _ in f) else f[0][0]} for k, f in qfound.items()}
        self._apply_faults()

    def _third_pending(self, hi, lo, q, p):
        """An independent third source of the quantity exists, has been delivering, and has not yet gathered
        fresh evidence since this disagreement began (bounded wait)."""
        c = self.cfg
        since = p.changed_t if p.changed_t is not None else self.now
        waited = self.now - since > c["vote_wait_s"]
        for name, cg in self.groups.items():
            if cg.fault is not None or not self._indep(name, hi, lo, q):
                continue
            p_lo = self.pairs.get((name, lo, q)) or self.pairs.get((lo, name, q))
            p_hi = self.pairs.get((name, hi, q)) or self.pairs.get((hi, name, q))
            if p_lo is None or p_hi is None or (p_lo.verdict == "unknown" and p_hi.verdict == "unknown"):
                continue
            newest = max([p_lo.times[-1]] if p_lo.times else [] + [p_hi.times[-1]] if p_hi.times else [], default=None)
            if newest is None or self.now - newest > 15.0:
                continue                   # it has stopped delivering: nothing to wait for
            if max(p_lo.fresh(since + c["vote_settle_s"]), p_hi.fresh(since + c["vote_settle_s"])) < c["vote_fresh_n"]:
                return True
            # the third source has spoken and sides with neither against the other: the disagreement is inside
            # its resolution. Rank would be a guess; the quantity stays disputed (its uncertainty covers both)
            if not waited or (p_lo.verdict == "agree" and p_hi.verdict == "agree"):
                return True
        return False

    def _vote_forming(self, hi, lo, q):
        for c, cg in self.groups.items():
            if cg.fault is not None or not self._indep(c, hi, lo, q):
                continue
            p_lo = self.pairs.get((c, lo, q)) or self.pairs.get((lo, c, q))
            p_hi = self.pairs.get((c, hi, q)) or self.pairs.get((hi, c, q))
            if p_lo is None or p_hi is None or p_lo.verdict != "agree":
                continue
            s_lo, s_hi = p_lo.stats.get("share") or 0.0, p_hi.stats.get("share")
            if p_hi.verdict == "unknown" or (s_hi is not None and s_hi > s_lo + 0.1):
                return True
        return False

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
        self.ekf.set_disabled(bad + list(self.qfaults))
        self.fdi.qfaults = self.qfaults
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

    # ---- L43-V: the feeder's `reset` (vehicle input API 5.3). One method, appended; nothing above changed. -------
    def new_position_source(self, why):
        """(Name and signature agreed with the integration lead's branch, which has its own body: at the merge keep
        the lead's body and check it also drops what this one drops, or merge the two.)
        Forget the position (a synthetic drive ended, the receiver was swapped): the input so far must not live
        on as the car's position. Dropped: the local frame (origin, anchor), position, heading and altitude with
        their covariance, the states and memories of the absolute sources (receiver delay and correlated error, the
        map's lane offset, the receiver's track), their monitors and arrival history. Kept: the motion (speed,
        yaw rate, accelerations, grade), every calibration state of the car's own sensors and of the cameras, the
        faults, the object layer, the odometer. Position comes back with the next usable fix (a new frame id)."""
        absolute = [s for s in self.specs if s.group in ("gnss", "map") or s.family == "gnss"]
        x0, P0 = self.model.initial()
        drop = [E.PE, E.PN, E.PSI, E.Z] + [i for s in absolute for i in s.ix.values()]
        l = self.ekf.latest()
        if l is None:
            x, P = self.ekf.x0, self.ekf.P0
        else:
            t, x, P = l
            x, P = x.copy(), P.copy()
        for i in drop:
            x[i] = x0[i]
            P[i, :] = 0.0
            P[:, i] = 0.0
            P[i, i] = P0[i, i]
        if l is not None:
            self.ekf.rebase(t, x, P)
        self.origin = None
        self.anchored = False
        self._anchor_first = None
        self._lost = False
        self._pos_high_t = None
        self.frame_id += 1
        groups = {s.group for s in absolute}
        for s in absolute:
            self.ctx[s.id].mem = {}
        for g in groups:
            self.groups[g].reset_data()
        for k, m in self.nis.items():
            if k[0] in groups or k[1] in ("pos", "heading", "alt", "map_cross", "map_heading"):
                m.reset_data()
        self.ekf.set_quarantine(k for k in self.ekf.quarantine if k[0] not in groups
                                and k[1] not in ("pos", "heading", "alt", "map_cross", "map_heading"))
        self.fdi.event("position", "reset", None, "the feeder reset the position (%s): position, heading and the "
                       "receiver's states dropped, calibration of the car's sensors kept" % (why or "requested"),
                       quantity="position")
        self._cache()
