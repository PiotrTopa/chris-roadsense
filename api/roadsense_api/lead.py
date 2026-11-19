"""Lead vehicle: the api-side face of the topic `lead`.

Since lane L38 the lead is tracked inside the data integrator's object layer (docs/INTEGRATOR.md section 6;
`chris-integrator/chris_integrator/core/objects.py`, `sources/camera_lead.py`): one multi-target track list in
the car frame, the road model's lead hypothesis as one of its sensors (with the measured noise and life-cycle
rules of lanes L14, L27, L29, L32), radars and further cameras as others, and the lead picked from the tracks
by the path. The integrator emits the `lead` message; the api adds the rate marker and serves it.

`LeadTracker` here is the stand-alone form for tools and tests: it owns a private integrator, is fed the same
messages (`on_ego`, `on_vehicle`, `on_objects`, `on_road`) and returns the same `lead` message. `DEFAULT_CFG`
and `PROFILES` are the camera-lead source's (lead.json keys keep working).
"""
from .integrator import Integrator
from chris_integrator.core.lead import LEAD_CFG
from chris_integrator.sources.camera_lead import DEFAULT_CFG as _CAM_CFG, PROFILES  # noqa: F401

NS = 1e9
DEFAULT_CFG = dict(_CAM_CFG, **LEAD_CFG)
DEFAULT_CFG.update(radar_fusion=False)      # stand-alone tracker only: register the radar source for `objects` scans


class LeadTracker:
    def __init__(self, cfg=None, integrator_cfg=None):
        self.user_cfg = dict(cfg or {})
        self.integrator_cfg = dict(integrator_cfg or {})
        self.reset()

    def reset(self):
        cfg = dict(self.integrator_cfg, lead=self.user_cfg)
        if self.user_cfg.get("radar_fusion"):        # an old lead.json switch: range sensors are sources now
            cfg["sources"] = list(cfg.get("sources") or []) + [{"kind": "radar", "id": "*"}]
        self.integ = Integrator(cfg)
        self.tracks = None

    @property
    def cfg(self):
        """The constants in use: the camera-lead source's (with the road model's profile applied) and the lead
        message's."""
        cam = next((s.model.cfg for s in self.integ.specs if s.id == "camera_lead"), _CAM_CFG)
        return dict(DEFAULT_CFG, **dict(cam, **{k: v for k, v in self.user_cfg.items() if k in LEAD_CFG}))

    def on_objects(self, msg):
        self.integ.on_message(msg)

    def on_vehicle(self, msg):
        self.integ.on_message(msg)

    def on_ego(self, msg):
        self.integ.on_message(msg)

    def on_road(self, road, ego=None):
        """Feed one `road` message; returns the `lead` message. (`ego` is ignored: the private integrator makes
        its own state from what it was fed.)"""
        if not isinstance(road.get("ts_mono_ns"), int):
            return None
        es, self.tracks, ld = self.integ.frame(road)
        self.ego_state = es
        return ld

    @staticmethod
    def summary(msg):
        """Short dict for /v1/health and `fused.lead`."""
        keys = ("present", "valid", "reason", "mode", "track_id", "dist_m", "dist_std_m", "lateral_m", "v_rel_mps",
                "v_lead_mps", "a_lead_mps2", "time_gap_s", "ttc_s", "standing", "departed", "source")
        return {k: msg.get(k) for k in keys}
