"""The chris stack as a mixer: which input channels, plug-ins and services, in which order.

    stack = build(config_dir="/etc/chris")            # or build(cfg={...}) / build() for the defaults
    stack.deliver(msg)                                # a message from any source, routed by its `type`
    stack.mixer.add_output("mine", fn)                # a consumer of the consolidated streams

Everything here is configuration of the mixer; nothing here is needed to use the mixer or a plug-in alone.

Configuration: one JSON file per plug-in in the configuration directory, named after the plug-in
(`integrator.json` for the estimator, `road_users.json`, `lead.json`, `lanes.json`, `egolane.json`, `roadgeom.json`, `view.json`,
`lights.json`, `rules.json`, `fused.json`, `mapper.json`, `rates.json`), and `mixer.json` for the wiring:

    {"disable": ["fused"],                                        plug-ins of the default stack to leave out
     "plugins": [{"class": "mypkg.mymod:MyPlugin", "cfg": {...}},   further plug-ins (in process)
                 {"remote": {"name": "x", "cmd": ["python3", "-m", "x"]}}],   ... or out of process
     "replace": {"lanes": {"class": "mypkg.lanes2:Lanes2"}},       another box in a default one's place
     "road_whole": true}                                           parse every member of `road` (see below)

With only the default boxes, the daemon parses a `road` line without the members none of them reads (most of the
plan's arrays, the lines' heights, `meta`, the desire outputs: half of the line; `fastjson.trim_road`). The line
itself is passed on whole to every consumer. A wiring with `plugins` or `replace` gets the whole message, as does
`"road_whole": true`.
"""
import importlib
import json
import logging
import os

from .mixer import Mixer, Plugin, StreamSpec
from .plugins.common import Rates
from .plugins.estimator import Estimator
from .plugins.roadusers import RoadUsers
from .plugins.trackers import EgoLane, EgoLight, Fused, Lanes, RideWatch, RoadGeom, Rules, SignContext, View

log = logging.getLogger("chris_integrator.stack")

# input channels of the default stack: channel -> [(message type, stream, kind, time, public, doc)]
ROADSENSE = (
    ("road", "road", "raw", "frame", True, "road model output of one camera: pose, lane lines, leads, plan"),
    ("calib", "calib", "raw", "event", True, "the camera's calibration in use and its intrinsics"),
    ("camera", "camera", "raw", "sample", True, "camera status"),
    ("env", "env", "raw", "sample", True, "place / weather / light"),
    ("lights", "lights_raw", "raw", "sample", False, "traffic-light detections of one camera"),
    ("road_users", "road_users_raw", "raw", "sample", False, "road users of the light provider's extra road-user view (L42)"),
    ("signs", "signs_raw", "raw", "sample", False, "sign detections of one camera"),
    ("display_lanes", "display_lanes", "raw", "sample", True, "a lane source for the display only"),
    ("governor", "governor", "status", "periodic", True, "the roadsense instance's load governor: level, caps"),
)
VEHICLE = (
    ("ego", "ego", "raw", "sample", True, "the car's motion: speed, wheels, gyro, accelerometers"),
    ("vehicle", "vehicle", "raw", "sample", True, "the car as its host describes it: every signal with age, quality "
                                                  "and source, the description, the sensors (docs/VEHICLE_INPUT.md)"),
    ("gnss", "gnss", "raw", "sample", True, "receiver fixes"),
    ("objects", "objects", "raw", "sample", True, "scans of range sensors and further cameras"),
    ("reset", "position_reset", "raw", "event", False, "the feeder's `reset`: forget the position (estimator, map box)"),
)
EXTERNAL_MAP = (("map", "map", "context", "periodic", True, "map match and context from an external matcher"),)


def load_json(config_dir, name):
    if not config_dir:
        return None
    try:
        with open(os.path.join(config_dir, name)) as f:
            cfg = json.load(f)
    except (OSError, ValueError):
        return None
    return cfg if isinstance(cfg, dict) else None


def load_class(spec):
    mod, _, cls = spec.partition(":")
    return getattr(importlib.import_module(mod), cls)


class Store:
    """State that survives a restart (faults, calibration): one JSON file per plug-in in the data directory."""

    def __init__(self, datadir):
        self.dir = datadir if datadir and os.path.isdir(datadir) else None

    def path(self, name):
        return os.path.join(self.dir, ("integrator" if name == "estimator" else name) + ".json")

    def load(self, name):
        if not self.dir:
            return None
        try:
            with open(self.path(name)) as f:
                d = json.load(f)
            return d if isinstance(d, dict) else None
        except (OSError, ValueError):
            return None

    def save(self, name, state):
        if not self.dir or state is None:
            return
        p = self.path(name)
        try:
            with open(p + ".tmp", "w") as f:
                json.dump(dict(state, ts_real_ns=__import__("time").time_ns()), f)
            os.replace(p + ".tmp", p)
        except OSError:
            pass


class Stack:
    def __init__(self, mixer):
        self.mixer = mixer
        self.route = {}                    # (instance or None, message type) -> (channel handle, stream)
        # True: only the default boxes run, and an input channel may parse `road` without the members none of
        # them reads (fastjson.trim_road). False as soon as the wiring adds or replaces a box.
        self.trim_road = False

    def add_channel(self, name, table, instance=None, info=None):
        """An input channel from a table of (type, stream, kind, time, public, doc). A further instance of a
        source (a second camera's roadsense) gets its streams suffixed: `road@rear`."""
        sfx = "" if instance is None else "@" + instance
        ch = self.mixer.add_input(name, [StreamSpec(st + sfx, kind, time, doc, public) for _, st, kind, time, public, doc in table], info)
        for t, st, *_ in table:
            self.route[(instance, t)] = (ch, st + sfx)
        return ch

    def deliver(self, msg, line=None, instance=None):
        """Route a message of any source to its stream. Returns False for a type no channel carries."""
        r = self.route.get((instance, msg.get("type")))
        if r is None:
            return False
        r[0].deliver(r[1], msg, line)
        return True

    def plugin(self, name):
        return self.mixer.plugin(name)

    def save(self):
        store = self.mixer.services.get("store")
        for s in self.mixer.slots.values():
            st = s.plugin.state_dump()
            if st is not None and store is not None:
                store.save(s.name, st)


def build(cfg=None, config_dir=None, datadir=None, clock=None, mapper="auto", external_map=False, rules="pl",
          plugins=None):
    """The default stack.

    cfg          {plug-in name: config dict}; overrides what the configuration directory says
    mapper       "auto": the map box in process if chris-mapper and a pack are there; a Plugin: that one;
                 None / False: no map box
    external_map an input channel for `map` messages of an external matcher (a legacy mapd, a recording)
    plugins      further Plugin objects, added after the default ones
    """
    cfg = dict(cfg or {})

    def conf(name, file=None):
        c = dict(load_json(config_dir, (file or name) + ".json") or {})
        for k, v in (cfg.get(name) or {}).items():
            c[k] = dict(c[k], **v) if isinstance(c.get(k), dict) and isinstance(v, dict) else v
        return c

    wiring = conf("mixer")
    m = Mixer(clock)
    st = Stack(m)
    st.add_channel("roadsense", ROADSENSE, info="a chris-roadsense instance: one camera's perception")
    st.add_channel("vehicle", VEHICLE, info="the car's direct feed (vehicle input API) and relays")
    m.services["rates"] = Rates(m, conf("rates").get("floors"))
    m.services["store"] = store = Store(datadir)
    m.services["reset"] = m.reset

    est_cfg = conf("estimator", "integrator")
    lead = conf("lead")
    if lead:
        est_cfg["lead"] = dict(lead, **(est_cfg.get("lead") or {}))
    view_cfg = conf("view")
    boxes = [RideWatch(), RoadUsers(conf("road_users") or None), Estimator(est_cfg)]
    if external_map:
        st.add_channel("map_in", EXTERNAL_MAP, info="an external map matcher")
    elif mapper not in (None, False):
        if mapper == "auto":
            from .plugins.mapper import make_mapper
            mapper = make_mapper(conf("mapper"))
        if mapper is not None:
            boxes.append(mapper)
    boxes += [Lanes(conf("lanes") or None), EgoLane(conf("egolane") or None), RoadGeom(conf("roadgeom") or None),
              EgoLight(conf("lights")), SignContext(), View(view_cfg or None),
              Rules(dict(conf("rules"), rules=rules)), Fused(dict(conf("fused"), rules=rules))]
    disable = set(wiring.get("disable") or ())
    replace = wiring.get("replace") or {}
    for b in boxes:
        if b.name in replace:
            b = _make(replace[b.name])
        m.add(b, enabled=b.name not in disable)
    for spec in wiring.get("plugins") or ():
        m.add(_make(spec))
    for p in plugins or ():
        m.add(p)
    st.trim_road = not (wiring.get("plugins") or replace or plugins or wiring.get("road_whole")
                        or isinstance(mapper, Plugin) and getattr(mapper, "remote", False))
    for s in m.slots.values():
        saved = store.load(s.name)
        if saved:
            s.plugin.state_load(saved)
    return st


def _make(spec):
    if "remote" in spec:
        from .mixer.remote import RemotePlugin
        return RemotePlugin(**spec["remote"])
    return load_class(spec["class"])(spec.get("cfg"))
