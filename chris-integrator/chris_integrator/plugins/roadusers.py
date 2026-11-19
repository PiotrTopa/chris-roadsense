"""The `road_users` box (lane L42): a camera's road users, from the class detector's boxes in `lights_raw`
(roadsense-signs `signd --lights-road-users`), placed with the camera's calibration -> stream `road_users`.

    lights_raw (+ calib) -> road_users     {"type": "road_users", "sensor", "seq", "ts_mono_ns", "view", "roi",
                                            "shed" (the load governor has paused the detector's road users),
                                            "objects": [{"class", "class_conf", "det_class", "rider", "clip", "box",
                                                         "bearing_deg", "bearing_std_deg", "range_m", "range_std_m",
                                                         "x_m", "y_m" (car frame, y LEFT), "method", "weight"}]}

Non-critical: the estimator reads it only to annotate its tracks with a class (`road_users` source kind,
core/roadusers.py), the view to draw the road users it cannot place otherwise (trackers/roadusers_view.py).
A light message without the key (road users off, or not decoded on this frame) gives nothing. The provider's
optional road-user view (`signd --lights-ru-roi`: an extra detector run on a lower band of the picture, where
the light views have no ground point) sends `{"type": "road_users", "view": 2, "roi", "road_users": [...]}` on
the same socket (stream `road_users_raw`); it is placed the same way."""
from ..core.roadusers import PROJ_CFG, from_lights
from ..mixer import In, Out, Plugin


class RoadUsers(Plugin):
    name = "road_users"
    DEFAULT_CFG = dict(PROJ_CFG, sensor="cam")
    inputs = (In("lights_raw", trigger=True), In("road_users_raw", trigger=True, optional=True), In("calib", optional=True))
    outputs = (Out("road_users", "measurement", "sample",
                   doc="road users of one camera's class detector: class, box, bearing and ground position with uncertainty"),)

    def on_input(self, stream, msg, ctx):
        out = from_lights(msg, ctx.get("calib"), self.cfg["sensor"], self.cfg)
        if out is not None:
            ctx.emit("road_users", out)
