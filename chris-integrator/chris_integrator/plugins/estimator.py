"""The estimator box: every motion, position and object input -> `ego_state`, `tracks`, `lead`.

The box is `core.Integrator` (source registry, one EKF with calibration states, monitors and fault isolation,
the object layer); this file is its plug-in face. Its inputs are the message types its registered sources
read, so a source added to the configuration adds an input here and nothing else changes."""
from ..core.integrator import Integrator
from ..mixer import In, Out, Plugin


class Estimator(Plugin):
    name = "estimator"
    DEFAULT_CFG = {}                       # core.integrator.DEFAULT_CFG keys (integrator.json)
    FRAME = "road"                         # the stream whose messages are camera frames
    # a source reads message types; on the bus the raw detections of a camera carry another stream name than the
    # decided topic of the same message type (`signs` is the sign-context box's output, downstream of this box)
    STREAM_OF = {"signs": "signs_raw"}
    outputs = (Out("ego_state", "estimate", "frame", doc="the car's motion and position with uncertainty, calibration, per-source health"),
               Out("tracks", "estimate", "frame", doc="one multi-target track list in the car frame"),
               Out("lead", "decision", "frame", doc="the track on the path"))

    def __init__(self, cfg=None, name=None, sources=None, tick=("ego",), feedback=("map",), stream_of=None):
        """sources: Source objects instead of the default registration. tick: streams whose samples drive the
        outputs while no camera frames arrive. feedback: inputs produced downstream of this box. stream_of:
        {message type: stream} where a source's message type is carried by a stream of another name."""
        self._sources, self.tick, self.feedback = sources, tuple(tick), tuple(feedback)
        self.stream_of = dict(self.STREAM_OF if stream_of is None else stream_of)
        super().__init__(cfg, name)
        self.saved = None
        self._new()

    def _new(self):
        cfg = dict(self.cfg)
        if cfg.get("sources") is None:               # a car that feeds range-sensor scans gets them fused; a camera's
            cfg["sources"] = [{"kind": "radar", "id": "*"},       # road users annotate the tracks with a class
                              {"kind": "road_users", "id": "*"}]
        self.integ = Integrator(cfg, sources=self._sources)
        types = [self.FRAME] + sorted(self.stream_of.get(t, t) for t in self.integ.by_type if t != self.FRAME)
        self.inputs = tuple(In(t, trigger=True, optional=True, feedback=t in self.feedback) for t in types) + \
            (In("governor", optional=True),)

    def register(self, src):
        """A source added while running (its message type becomes an input at the next wiring)."""
        self.integ.register(src)

    def on_input(self, stream, msg, ctx):
        I = self.integ
        if stream == self.FRAME:
            try:
                outs = I.frame(msg)
            except Exception:              # the frame still gets an answer: unknown, never the raw model output
                ctx.emit("lead", self._rated(lead_unknown(msg, "integrator_timeout"), msg, ctx))
                raise
            self._publish(outs, msg, ctx, frame=True)
            return
        I.on_message(msg)
        if stream in self.tick and I.camera_silent() and isinstance(msg.get("ts_mono_ns"), int):
            # no camera frames: the ego state (and the tracks other sensors hold) still go out, at the sample rate
            self._publish(I.tick(msg["ts_mono_ns"], msg.get("ts_real_ns")), msg, ctx)

    def _publish(self, outs, msg, ctx, frame=False):
        es, tr, ld = outs
        if es is not None:
            ctx.emit("ego_state", es)
            self._save(ctx)
        if tr is not None:
            ctx.emit("tracks", tr)
        if ld is None and frame and self.integ.objects is not None:
            ld = lead_unknown(msg, "integrator_timeout")
        if ld is not None:
            ctx.emit("lead", self._rated(ld, msg, ctx))

    @staticmethod
    def _rated(ld, msg, ctx):
        rates = ctx.service("rates")
        if rates is not None:
            ld["rate"] = rates.status("lead", msg)
        return ld

    # ---- carried state: faults and calibration survive a restart ----------------------------------------
    def _save(self, ctx):
        store = ctx.service("store")
        if store is None:
            return
        key = sorted(self.integ.faults())
        if key != self.saved:
            self.saved = key
            store.save(self.name, self.state_dump())

    def state_dump(self):
        faults = self.integ.faults()
        return {"faults": {g: {"reason": r.get("reason"), "rule": r.get("rule")} for g, r in faults.items()},
                "calibration": self.integ.calibration_snapshot()}

    def state_load(self, saved):
        for g, rec in ((saved or {}).get("faults") or {}).items():
            if isinstance(rec, dict):
                self.integ.carry_fault(g, "carried over from the previous run: %s" % rec.get("reason"))
        self.integ.calibration_restore((saved or {}).get("calibration"))
        self.saved = sorted(self.integ.faults())

    def reset(self, reason=None):
        self._new()                        # another drive: nothing of the old one is evidence

    def health(self):
        faults = sorted(self.integ.faults())
        return {"state": "degraded" if faults else "ok", "reason": ("faulted: " + ", ".join(faults)) if faults else None,
                "states": self.integ.model.n, "sources": [s.id for s in self.integ.specs], "epoch": self.integ.epoch,
                "late": dict(self.integ.counts["late"]), "dropped": dict(self.integ.counts["dropped"])}

    @staticmethod
    def summary(es):
        """/health: the verdict per source, the faults, and what the ego state is made from."""
        if not es:
            return None
        src = es.get("sources") or {}
        return {"sources": {g: {k: s.get(k) for k in ("state", "reason", "isolation", "since_s", "age_s", "rate_hz")
                                if k in s} for g, s in src.items()},
                "faults": sorted(g for g, s in src.items() if s.get("state") == "fault"),
                "speed": {k: (es.get("speed") or {}).get(k) for k in ("value", "std", "valid", "reason", "mode", "sources")},
                "yaw_rate": {k: (es.get("yaw_rate") or {}).get(k) for k in ("value", "std", "valid", "mode", "sources")},
                "standstill": es.get("standstill"), "epoch": es.get("epoch"), "checks": es.get("checks")}


def lead_unknown(msg, reason):
    """The `lead` of a frame nothing answered for: unknown with the reason, never the raw model output."""
    return {"type": "lead", "seq": msg.get("seq"), "ts_mono_ns": msg.get("ts_mono_ns"),
            "ts_real_ns": msg.get("ts_real_ns"), "present": False, "mode": "none", "track_id": None,
            "valid": False, "reason": reason, "inputs": {}, "standing": False,
            "departed": False, "event": None, "vision": None, "radar": None}


LEAD_SUMMARY_KEYS = ("present", "valid", "reason", "mode", "track_id", "dist_m", "dist_std_m", "lateral_m", "v_rel_mps",
                     "v_lead_mps", "a_lead_mps2", "time_gap_s", "ttc_s", "standing", "departed", "source")


def lead_summary(msg):
    return {k: msg.get(k) for k in LEAD_SUMMARY_KEYS}
