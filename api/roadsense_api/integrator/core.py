"""The integrator: source registry, ego estimator, consistency / isolation, output (docs/INTEGRATOR.md).

No I/O and no wall clock: times come from the messages (`ts_mono_ns`, CLOCK_MONOTONIC of the device).
"""
import collections
import math

import numpy as np

from .filters import Channel, Row, fq_speed, fq_yaw
from .monitors import Pair, SelfCheck, Series

NS = 1e9

DEFAULT_CFG = dict(
    horizon_s=2.0,               # measurements are kept (and may still be inserted) this long
    gate_sigma=5.0,              # a fused measurement further than this from the prediction is not applied
    jerk_psd=2.0,                # (m/s3)^2 s: how fast the longitudinal acceleration changes
    accel_bias_psd=0.0025,       # (m/s2)^2 / s: drift of the accelerometer's offset (grade, calibration)
    yaw_acc_psd=1.0,             # (rad/s2)^2 / s... of the yaw acceleration's rate of change
    fresh_s=0.3,                 # a quantity with a measurement this near the requested time is `fused`
    max_coast_s=1.0,             # ... further than this it is unknown
    epoch_back_s=2.0,            # a source's clock going back by more than this starts a new epoch
    camera_silent_s=0.3,         # without road frames for this long the state is also published on vehicle samples
    pair=dict(                   # pair monitors (defaults; per quantity below)
        mode="share", lag_s=0.0, max_gap_s=0.5, pending_s=1.0, max_step_s=0.25,
        z_bad=4.0, fire_window_s=5.0, min_n=15, fire_share=0.5, suspect_share=0.35,
        recover_window_s=10.0, recover_min_n=30, recover_share=0.25),
    pair_quantity=dict(
        speed=dict(min_evidence=3.0, fault=True),
        yaw_rate=dict(min_evidence=0.05, fault=False),      # a disagreement makes the source `suspect` only
    ),
    witness=dict(                # sources with poor timing (GNSS): ratio of medians over a long window
        mode="ratio", lag_s=3.0, max_gap_s=0.5, pending_s=1.0, max_step_s=3.0, z_bad=4.0,
        fire_window_s=30.0, min_n=10, ratio_lo=0.6, ratio_hi=1.6, min_evidence=3.0,
        recover_window_s=30.0, recover_min_n=10, recover_share=1.0, fire_share=1.0, suspect_share=1.0, fault=False),
    self_check=dict(fire_window_s=5.0, min_n=15, fire_ratio=4.0, recover_window_s=10.0, recover_min_n=30,
                    recover_ratio=3.0, max_step_s=0.25),
)


class Measurement:
    """What a measurement model returns.

    q      "speed" | "accel" | "yaw_rate" (role fuse / monitor), "self:<quantity>" (stated / characterised sigma),
           "standstill" (z True / False), "signal:<name>" (pass-through value)
    t      time of validity, seconds on the device's monotonic clock
    z      value;  sigma  1-sigma noise (fuse / monitor);  src  measurement id ("vehicle.speed")
    role   "fuse" | "monitor" | "self" | "flag" | "signal"
    msigma the sigma the monitors judge the source by: its characterised noise, not what it states about itself
           (a source that inflates its own uncertainty must not pass its consistency check that way)
    gate   True for a heavy-tailed source: a fused value further than `gate_sigma` from the prediction is not
           applied (single-sample protection; a trusted source is never gated, the state follows it)
    """
    __slots__ = ("q", "t", "z", "sigma", "src", "role", "gate", "msigma")

    def __init__(self, q, t, z, sigma=None, src=None, role="fuse", gate=False, msigma=None):
        self.q, self.t, self.z, self.sigma, self.src, self.role, self.gate = q, t, z, sigma, src, role, gate
        self.msigma = sigma if msigma is None else msigma


class SourceSpec:
    """An input description. `model(msg, mem)` turns a message of one of `types` into Measurements (`mem` is a
    dict the model may keep state in). `rank` orders the groups for isolation by rank; `witness` sources are
    never fused and can never start a fault; `stale_s` is how long the group may be silent."""

    def __init__(self, id, group, rank, types, model, stale_s=1.0, witness=False, info=None, counts=True,
                 family=None):
        self.id, self.group, self.rank, self.types, self.model = id, group, rank, tuple(types), model
        # groups of one family share their physical sensors (wheel speeds and the speed signal of one car): they
        # are checked against each other, but neither is an independent vote for the other
        self.family = family or group
        self.stale_s, self.witness, self.info = stale_s, witness, info
        self.counts = counts               # False: a side channel of the group (its messages do not show it alive)


_H = {"speed": np.array([1.0, 0.0, 0.0]), "accel": np.array([0.0, 1.0, 1.0]), "accel_state": np.array([0.0, 1.0, 0.0]),
      "yaw_rate": np.array([1.0, 0.0]), "yaw_acc": np.array([0.0, 1.0])}


class _Group:
    def __init__(self, name, rank, witness, stale_s, family=None):
        self.name, self.rank, self.witness, self.stale_s = name, rank, witness, stale_s
        self.family = family or name
        self.arrivals = collections.deque(maxlen=64)
        self.last_t = None                 # newest stamp
        self.fault = None                  # {"since", "reason", "rule"} or None
        self.carried = False
        self.series = {}                   # quantity -> Series
        self.info = {}                     # what the models want reported (tag, repeats, ...)

    def reset_data(self):
        self.arrivals.clear()
        self.last_t = None
        for s in self.series.values():
            s.clear()


class Integrator:
    def __init__(self, cfg=None, sources=None):
        self.cfg = dict(DEFAULT_CFG)
        for k, v in (cfg or {}).items():
            self.cfg[k] = dict(self.cfg[k], **v) if isinstance(self.cfg.get(k), dict) and isinstance(v, dict) else v
        self.specs, self.by_type, self.groups, self.mem = [], collections.defaultdict(list), {}, {}
        self.pairs, self.selfs = {}, {}
        self.seq = 0
        self.epoch = 0
        self.counts = {"late": {}, "dropped": {}}
        self._init_state()
        if sources is None:
            from .sources import default_sources
            sources = default_sources()
        for s in sources:
            self.register(s)

    def _init_state(self):
        c = self.cfg
        self.lon = Channel("lon", fq_speed(c["jerk_psd"], c["accel_bias_psd"]), [0.0, 0.0, 0.0], [1e6, 9.0, 0.09],
                           c["horizon_s"], c["gate_sigma"])
        self.yaw = Channel("yaw", fq_yaw(c["yaw_acc_psd"]), [0.0, 0.0], [1.0, 1.0], c["horizon_s"], c["gate_sigma"])
        self.now = None
        self.src_t = {}                    # source id -> its newest stamp (out-of-order / epoch detection)
        self.still = {}                    # group -> deque of (t, standing since t or None)
        self.signals = {}                  # name -> (t, value)
        self.last_frame_t = None

    # ---- registry ------------------------------------------------------------------------------------
    def register(self, spec):
        """Add a source: an input description and its measurement model. Pair monitors against the groups already
        known are created as their quantities appear."""
        self.specs.append(spec)
        for t in spec.types:
            self.by_type[t].append(spec)
        if spec.group not in self.groups:
            self.groups[spec.group] = _Group(spec.group, spec.rank, spec.witness, spec.stale_s, spec.family)
        self.mem[spec.id] = {}

    def carry_fault(self, group, reason):
        """A fault known from before this run (the previous session ended with it): the group starts as faulted
        and stays so until its monitors have evidence that it agrees again."""
        g = self.groups.get(group)
        if g is not None and g.fault is None:
            g.fault = {"since": None, "reason": reason, "rule": "carried"}
            g.carried = True
            self._apply_faults()

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
            ms = spec.model(msg, self.mem[spec.id])
            if not ms:
                continue                   # a message without data (a status line, a fix without a position)
            if spec.counts:
                g.arrivals.append(t)
                g.last_t = t if g.last_t is None else max(g.last_t, t)
            for m in ms:
                self._measure(spec, g, m)
            info = self.mem[spec.id].get("info")
            if info:
                g.info.update(info)
        self.lon.prune(self.now)
        self.yaw.prune(self.now)

    def _new_epoch(self):
        """The device clock restarted (a boot, or a replay that joins sessions): drop everything that is tied to
        the old time axis; a fault survives as carried, an `ok` falls back to unchecked."""
        self.epoch += 1
        self._init_state()
        for k in self.mem:
            self.mem[k] = {}
        for g in self.groups.values():
            g.reset_data()
            if g.fault is not None:
                g.fault = dict(g.fault, since=None, rule="carried")
                g.carried = True
        for p in self.pairs.values():
            p.reset_data()
            if p.verdict == "agree":
                p.verdict = "unknown"
        for s in self.selfs.values():
            s.reset_data()
        self._apply_faults()

    def _measure(self, spec, g, m):
        role = m.role
        if role == "signal":
            self.signals[m.q[7:]] = (m.t, m.z)
            return
        if role == "flag":                 # standstill
            q = self.still.setdefault(g.name, collections.deque(maxlen=64))
            since = None
            if m.z:
                since = q[-1][1] if q and q[-1][1] is not None else m.t
            q.append((m.t, since))
            if m.z and m.src is not None and not g.witness:
                self._fuse(self.lon, m.t, Row("standstill", _H["speed"], 0.0, 1e-6, m.src, g.name, force=True))
                self._fuse(self.lon, m.t, Row("standstill", _H["accel_state"], 0.0, 1e-4, m.src, g.name, force=True))
                self._fuse(self.yaw, m.t, Row("standstill", _H["yaw_rate"], 0.0, 1e-8, m.src, g.name, force=True))
            return
        if role == "self":
            q = m.q[5:]
            sc = self.selfs.get((g.name, q))
            if sc is None:
                sc = self.selfs[(g.name, q)] = SelfCheck(g.name, q, self.cfg["self_check"])
            if sc.offer(m.t, m.z):
                self._isolate()
            return
        if role == "fuse" and not g.witness:
            ch = self.yaw if m.q == "yaw_rate" else self.lon
            self._fuse(ch, m.t, Row(m.q, _H[m.q], m.z, m.sigma * m.sigma, m.src, g.name, force=not m.gate))
        if m.q in ("speed", "yaw_rate"):
            self._monitor(g, m)

    def _fuse(self, ch, t, row):
        if not ch.insert(t, row):
            self.counts["late"][row.src] = self.counts["late"].get(row.src, 0) + 1

    def _monitor(self, g, m):
        ser = g.series.get(m.q)
        if ser is None:
            ser = g.series[m.q] = Series()
            for o in self.groups.values():           # a new (group, quantity): pair it with the others that have it
                if o is g or m.q not in o.series or (o.witness and g.witness):
                    continue
                hi, lo = (o, g) if (o.rank, o.name) > (g.rank, g.name) else (g, o)
                wit = hi.witness or lo.witness
                cfg = dict(self.cfg["witness"]) if wit else dict(self.cfg["pair"], **self.cfg["pair_quantity"].get(m.q, {}))
                self.pairs[(hi.name, lo.name, m.q)] = Pair(hi.name, lo.name, m.q, cfg)
        ser.add(m.t, m.z, m.msigma)
        changed = False
        for (hi, lo, q), p in self.pairs.items():
            if q != m.q:
                continue
            if lo == g.name:
                changed |= p.offer(m.t, m.z, m.msigma, self.groups[hi].series[q], self.now)
            elif hi == g.name and p.pending:
                changed |= p.flush(ser, self.now)
        if changed:
            self._isolate()

    # ---- isolation -----------------------------------------------------------------------------------
    def _verdict(self, a, b, q):
        p = self.pairs.get((a, b, q)) or self.pairs.get((b, a, q))
        return p.verdict if p is not None else "unknown"

    def _isolate(self):
        """Which groups are faulted, from the monitors' verdicts (docs/INTEGRATOR.md section 2)."""
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
                # of the two (INTEGRATOR.md 2)
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
        self.lon.set_disabled(bad)
        self.yaw.set_disabled(bad)

    # ---- output --------------------------------------------------------------------------------------
    def _health(self, g, T):
        age = None if g.last_t is None else max(0.0, self.now - g.last_t)
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
            for (hi, lo, q), p in self.pairs.items():
                if g.name not in (hi, lo):
                    continue
                other = self.groups[lo if hi == g.name else hi]
                if p.verdict == "agree" and not other.witness and other.fault is None:
                    checked = True
                if other.fault is not None:
                    continue               # the culprit of that disagreement is known, and it is the other one
                if p.suspect or (p.verdict == "disagree" and (lo == g.name or not other.witness)):
                    suspect = True
                    reason = p.describe() if lo == g.name else "%s of %s disagrees" % (q, lo)
            for (gname, q), sc in self.selfs.items():
                if gname == g.name and sc.median is not None and sc.median > sc.c["recover_ratio"] \
                        and sc.n >= sc.c["min_n"]:
                    suspect, reason = True, sc.describe()
            state = "suspect" if suspect else "ok" if checked else "unchecked"
        n = sum(1 for a in g.arrivals if a > self.now - 2.0)
        out = {"state": state, "reason": reason, "usable": usable,
               "age_s": None if age is None else round(age, 3), "rate_hz": round(n / 2.0, 1)}
        if g.fault is not None:
            out["isolation"] = g.fault["rule"]
            out["since_s"] = None if g.fault["since"] is None else round(max(0.0, self.now - g.fault["since"]), 1)
        out.update(g.info)
        return out

    def _standstill(self, T):
        """Standstill at T: {"value", "for_s", "source"} from the best usable group that reports it."""
        best = None
        for name, q in self.still.items():
            g = self.groups[name]
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

    def _quantity(self, ch, T, sm, h, kinds, digits):
        c = self.cfg
        if sm is None:
            return {"value": None, "std": None, "valid": False, "reason": "no_measurement", "mode": "none",
                    "age_s": None, "sources": []}
        x, P = sm
        age, srcs, floor = ch.near(T, c["fresh_s"], kinds, c["max_coast_s"] + 0.01)
        val, std = float(h @ x), math.sqrt(max(float(h @ P @ h), 0.0))
        if floor is not None and kinds is not None:
            std = max(std, floor)
        if age is None or age > c["max_coast_s"]:
            why = "no_measurement"
            if any(g.fault is not None and g.last_t is not None and self.now - g.last_t <= g.stale_s
                   for g in self.groups.values()):
                why = "no_healthy_source"
            return {"value": None, "std": None, "valid": False, "reason": why, "mode": "none",
                    "age_s": None if age is None else round(age, 3), "sources": []}
        return {"value": round(val, digits), "std": round(std, digits), "valid": True, "reason": None,
                "mode": "fused" if age <= c["fresh_s"] else "coast", "age_s": round(age, 3), "sources": srcs}

    def state(self, ts_mono_ns, ts_real_ns=None):
        """The `ego_state` message for the time of validity ts_mono_ns (smoothed over what has arrived)."""
        T = ts_mono_ns / NS
        if self.now is None:
            self.now = T
        self.seq += 1
        lon, yaw = self.lon.at(T), self.yaw.at(T)
        speed = self._quantity(self.lon, T, lon, _H["speed"], ("speed", "standstill"), 3)
        accel = self._quantity(self.lon, T, lon, _H["accel_state"], None, 3)
        if not speed["valid"]:
            accel = dict(accel, value=None, std=None, valid=False, reason=speed["reason"], mode="none")
        yr = self._quantity(self.yaw, T, yaw, _H["yaw_rate"], ("yaw_rate", "standstill"), 5)
        still = self._standstill(T)
        if still["value"] and still["source"] is not None and speed["valid"] and \
                any(s.startswith(still["source"] + ".") for s in speed["sources"]) and abs(speed["value"]) < 0.05:
            speed = dict(speed, value=0.0, std=0.0)      # the wheels stand: exactly zero, not a small number
        sig = {}
        for name, (t, v) in self.signals.items():
            sig[name] = {"value": v, "age_s": round(max(0.0, T - t), 3)}
        return {"type": "ego_state", "seq": self.seq, "ts_mono_ns": ts_mono_ns, "ts_real_ns": ts_real_ns,
                "epoch": self.epoch, "speed": speed, "accel": accel, "yaw_rate": yr, "standstill": still,
                "signals": sig,
                "sources": {n: self._health(g, T) for n, g in self.groups.items()},
                "checks": [p.report() for p in self.pairs.values()] + [s.report() for s in self.selfs.values()],
                "rejected": dict(self.lon.rejected, **self.yaw.rejected), "late": dict(self.counts["late"])}

    def on_road(self, road):
        """Feed a `road` message and return the state for its frame time (None without a time stamp)."""
        self.on_message(road)
        ts = road.get("ts_mono_ns")
        if not isinstance(ts, int):
            return None
        self.last_frame_t = ts / NS
        return self.state(ts, road.get("ts_real_ns"))

    def camera_silent(self):
        return self.last_frame_t is None or self.now - self.last_frame_t > self.cfg["camera_silent_s"]

    def faults(self):
        return {n: dict(g.fault) for n, g in self.groups.items() if g.fault is not None}
