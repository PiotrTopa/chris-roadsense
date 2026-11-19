"""Road users of a camera's class detector (lane L42): boxes in `lights` messages (signd --lights-road-users) ->
`road_users` (placed by the calibration) -> the class vote of the track they point at, and display objects for
those no track holds. Non-critical: the tracks, the lead and the ego state are the same, number for number,
with and without them; the view differs only by the class and by the road users' own objects."""
import copy
import math
import os
import sys
import unittest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from chris_integrator.core import classvote  # noqa: E402
from chris_integrator.core.roadusers import Camera, from_lights, project  # noqa: E402
from chris_integrator.mixer import MessageClock  # noqa: E402
from chris_integrator.sim import World, default_sensors  # noqa: E402
from chris_integrator.stack import build  # noqa: E402
from chris_integrator.trackers.projection import Projector  # noqa: E402

NS = 10 ** 9
CALIB = {"type": "calib", "pitch": 0.02, "yaw": -0.01, "roll": 0.005, "height_m": 1.22, "focal_px": 975, "cx": 640,
         "cy": 360, "width": 1280, "height": 720, "source": "config"}
SIZE = {"car": (1.8, 1.5), "truck": (2.5, 3.4), "human": (0.5, 1.75)}


def box_of(x, y_left, kind, roi_bottom=720.0):
    """The image box of a road user standing at (x, y left) in the car frame, its bottom clipped at roi_bottom."""
    w, h = SIZE[kind]
    pr = Projector((CALIB["focal_px"], CALIB["cx"], CALIB["cy"], 1280, 720), CALIB, (1280, 720))
    yr = -y_left
    pts = pr.project([x, x, x, x], [yr - w / 2, yr + w / 2, yr - w / 2, yr + w / 2],
                     [CALIB["height_m"], CALIB["height_m"], CALIB["height_m"] - h, CALIB["height_m"] - h])
    us, vs = [p[0] for p in pts], [p[1] for p in pts]
    b = [min(us), min(vs), max(us), max(vs)]
    clip = ""
    if b[3] > roi_bottom - 2:
        b[3], clip = roi_bottom, "b"
    return b, clip


def drive(label=None, seconds=40.0, wrong_every=0, person=False, roi_bottom=720.0):
    """A simulated town drive through the default stack, with a `lights` message per road frame. label: what the
    detector calls the nearest vehicle in the ego lane (every wrong_every-th time "car"), as a road user of the
    message; person: a pedestrian 25 m ahead, 6 m to the right, that no track holds."""
    w = World(seed=1, scenario="town")
    st = build(cfg={"estimator": {"sources": [{"kind": "radar", "id": "*"}, {"kind": "road_users", "id": "*"}]}},
               mapper=None, clock=MessageClock())
    out = {"tracks": [], "lead": [], "view": [], "ego_state": [], "road_users": []}
    st.mixer.add_output("t", lambda e: out[e.stream].append(copy.deepcopy(e.msg)), streams=tuple(out))
    k = 0
    calib_sent = False
    for t, m in w.messages(default_sensors()):
        if t - w.t0 > seconds:
            break
        if not calib_sent:
            st.deliver(dict(CALIB, ts_mono_ns=m["ts_mono_ns"]))
            calib_sent = True
        if m["type"] == "road":
            lm = {"type": "lights", "seq": m.get("seq"), "ts_mono_ns": m["ts_mono_ns"] - 1, "frame": [1280, 720],
                  "roi": [0, 0, 1280, 720], "view": 0, "lights": []}
            if label is not None:
                ru = []
                near = [o for o in w.targets_at(m["ts_mono_ns"] / NS) if o["x"] > 2.0 and abs(o["y"]) < 1.5]
                if near:
                    o = min(near, key=lambda o: o["x"])
                    k += 1
                    lab = "car" if wrong_every and k % wrong_every == 0 else label
                    b, clip = box_of(o["x"], o["y"], "truck" if lab == "truck" else "car", roi_bottom)
                    ru.append({"cls": lab, "score": 0.8, "box": b, "clip": clip, "rider": False})
                if person:
                    b, clip = box_of(25.0, -6.0, "human", roi_bottom)
                    ru.append({"cls": "person", "score": 0.7, "box": b, "clip": clip, "rider": False})
                lm["road_users"] = ru
            st.deliver(lm)
        st.deliver(m)
    return st, out


def strip(msgs):
    out = []
    for m in msgs:
        m = copy.deepcopy(m)
        m.pop("class", None)
        m.pop("sensors", None)
        for tr in m.get("tracks") or []:
            tr.pop("class", None)
        if "objects" in m:
            m["objects"] = [o for o in m["objects"] if not str(o.get("id", "")).startswith("ru:")]
        for o in m.get("objects") or []:
            o.pop("cls", None)
        out.append(m)
    return out


class Geometry(unittest.TestCase):
    def test_ground_point_round_trip_through_roadsense_projection(self):
        cam = Camera(CALIB)
        pr = Projector((975, 640, 360, 1280, 720), CALIB, (1280, 720))
        for x, yl in ((8.0, 0.0), (20.0, 3.5), (45.0, -6.0)):
            (u, v), = pr.project([x], [-yl], [CALIB["height_m"]])
            gx, gy = cam.ground(u, v)
            self.assertAlmostEqual(gx, x, delta=0.01 * x)
            self.assertAlmostEqual(-gy, yl, delta=0.05)
            self.assertAlmostEqual(math.degrees(cam.bearing(u)), math.degrees(math.atan2(yl, x)), delta=0.6)

    def test_range_and_uncertainty_from_the_bottom_edge(self):
        cam = Camera(CALIB)
        for x, rel in ((20.0, (0.08, 0.25)), (40.0, (0.2, 0.5))):
            b, clip = box_of(x, 0.0, "car")
            p = project({"box": b, "clip": clip}, cam)
            self.assertEqual(p["method"], "ground")
            self.assertAlmostEqual(p["range_m"], x, delta=0.03 * x)
            self.assertTrue(rel[0] < p["range_std_m"] / x < rel[1], p)

    def test_a_box_cut_at_the_bottom_has_a_bearing_only(self):
        b, clip = box_of(8.0, 1.0, "car", roi_bottom=446.0)
        self.assertEqual(clip, "b")
        p = project({"box": b, "clip": clip}, Camera(CALIB))
        self.assertIsNone(p["range_m"])
        self.assertAlmostEqual(p["bearing_deg"], math.degrees(math.atan2(1.0, 8.0)), delta=1.0)

    def test_the_road_user_view_message_is_placed_too(self):
        from chris_integrator.mixer import MessageClock as MC
        st = build(cfg={"estimator": {"sources": []}}, mapper=None, clock=MC())
        got = []
        st.mixer.add_output("t", lambda e: got.append(e.msg), streams=("road_users",))
        st.deliver(dict(CALIB, ts_mono_ns=1))
        b, clip = box_of(9.0, 3.0, "human")
        self.assertTrue(st.deliver({"type": "road_users", "seq": 4, "ts_mono_ns": 10, "frame": [1280, 720], "view": 2,
                                    "roi": [0, 238, 1280, 684], "road_users": [{"cls": "person", "score": 0.8, "box": b,
                                                                                "clip": clip}]}))
        self.assertEqual(len(got), 1)
        o = got[0]["objects"][0]
        self.assertEqual((o["class"], o["method"], got[0]["view"]), ("human", "ground", 2))
        self.assertAlmostEqual(o["x_m"], 9.0, delta=0.3)

    def test_lights_message_without_with_and_shed(self):
        base = {"type": "lights", "seq": 3, "ts_mono_ns": 5, "frame": [1280, 720], "lights": []}
        self.assertIsNone(from_lights(base, CALIB))
        shed = from_lights(dict(base, road_users=None), CALIB)
        self.assertTrue(shed["shed"])
        self.assertEqual(shed["objects"], [])
        b, clip = box_of(30.0, 2.0, "truck")
        m = from_lights(dict(base, road_users=[{"cls": "truck", "score": 0.9, "box": b, "clip": clip},
                                               {"cls": "rider", "score": 0.6, "box": [600, 300, 620, 340], "clip": ""}]), CALIB)
        self.assertEqual([o["class"] for o in m["objects"]], ["truck", "bike"])
        self.assertAlmostEqual(m["objects"][0]["x_m"], 30.0, delta=1.0)
        self.assertAlmostEqual(m["objects"][0]["y_m"], 2.0, delta=0.3)


class Vote(unittest.TestCase):
    def test_sticky_label_and_unknown_until_evidence(self):
        mem = {}
        classvote.vote(mem, "car", 0.9, "s", 0.0)
        self.assertIsNone(classvote.result(mem, 0.0))                 # one observation: unknown
        seq = ["car", "truck", "car", "car", "truck", "car", "bus", "car"] * 5
        shown = []
        for i, lab in enumerate(seq):
            classvote.vote(mem, lab, 0.7, "s", 0.2 * (i + 1))
            shown.append((classvote.result(mem) or {}).get("label"))
        self.assertEqual(set(shown), {"car"})                         # 30 % other labels: no hop
        for i in range(40):                                           # the track is now another vehicle
            classvote.vote(mem, "truck", 0.8, "s", 10.0 + 0.2 * i)
        self.assertEqual(classvote.result(mem)["label"], "truck")

    def test_aliases(self):
        self.assertEqual([classvote.norm_label(x) for x in ("person", "motor", "rider", "Bicycle", "van", "train")],
                         ["human", "motorbike", "bike", "bike", "van", "tram"])


class RoadUsersInTheStack(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.st0, cls.plain = drive(None, person=False)
        cls.st1, cls.cls = drive("truck", wrong_every=4, person=True)

    def test_measurement_outputs_equal_with_and_without_road_users(self):
        for topic in ("tracks", "lead"):
            self.assertEqual(strip(self.plain[topic]), strip(self.cls[topic]), topic)
        a, b = copy.deepcopy(self.plain["ego_state"]), copy.deepcopy(self.cls["ego_state"])
        for m in a + b:
            m["sources"].pop("roadusers", None)
        self.assertEqual(a, b)

    def test_view_differs_only_by_class_and_road_user_objects(self):
        self.assertEqual(strip(self.plain["view"]), strip(self.cls["view"]))

    def test_lead_class_is_voted_and_stable(self):
        ld = [m for m in self.cls["lead"] if m["present"]]
        labels = [(m["class"] or {}).get("label") for m in ld]
        self.assertGreater(labels.count("truck") / len(ld), 0.9)      # one "car" in four does not flip it
        changes = sum(1 for a, b in zip(labels, labels[1:]) if a != b and a is not None and b is not None)
        self.assertEqual(changes, 0)
        self.assertEqual(ld[-1]["class"]["source"], "roadusers")

    def test_the_pedestrian_is_a_display_object_not_a_track(self):
        self.assertFalse(any((t["class"] or {}).get("label") == "human" for m in self.cls["tracks"] for t in m["tracks"]))
        ru = [o for m in self.cls["view"] for o in m["objects"] if o["id"].startswith("ru:")]
        hum = [o for o in ru if (o["cls"] or {}).get("label") == "human"]
        self.assertTrue(hum)
        o = hum[-1]
        self.assertAlmostEqual(o["x"], 25.0, delta=3.0)
        self.assertAlmostEqual(o["y"], 6.0, delta=1.0)                # view frame: y right
        self.assertTrue(0.5 < o["sx"] < 6.0 and o["sy"] < 2.0, o)
        self.assertEqual((o["kind"], o["role"]), ("road_user", "other"))
        # the lead's own detection is not drawn a second time
        trucks = [o for o in ru if (o["cls"] or {}).get("label") == "truck"]
        self.assertLess(len(trucks), 0.05 * len(self.cls["view"]))
        # nothing pops: the first frame an object is in it fades in
        first = {}
        for m in self.cls["view"]:
            for o in m["objects"]:
                if o["id"].startswith("ru:") and o["id"] not in first:
                    first[o["id"]] = o
        self.assertTrue(all(o["phase"] == "appear" and o["alpha"] < 1.0 for o in first.values()))

    def test_road_users_stream_carries_bearing_and_range(self):
        m = [m for m in self.cls["road_users"] if m["objects"]][-1]
        o = m["objects"][0]
        for k in ("bearing_deg", "bearing_std_deg", "range_m", "range_std_m", "x_m", "y_m", "class", "class_conf", "box"):
            self.assertIn(k, o)
        self.assertEqual(self.cls["tracks"][-1]["sensors"]["roadusers"]["kind"], "road_users")


if __name__ == "__main__":
    unittest.main()
