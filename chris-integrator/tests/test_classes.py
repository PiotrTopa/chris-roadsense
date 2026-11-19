"""Road-user classes as non-critical metadata: the class attribute on tracks, the lead and the view's objects;
never a gate. A later detector only adds an input (`objects` of kind "classes", or `class` on the objects of a
`camera_objects` source)."""
import copy
import os
import sys
import unittest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from chris_integrator.mixer import MessageClock  # noqa: E402
from chris_integrator.sim import World, default_sensors  # noqa: E402
from chris_integrator.stack import build  # noqa: E402

NS = 10 ** 9


def drive(classes=None, seconds=40.0, cfg=None, wrong_every=0, fault_at=None):
    """A simulated town drive through the default stack. classes: label for the vehicle ahead (a class detector's
    message per camera frame, pointing at the truth position of the nearest vehicle in the ego lane)."""
    w = World(seed=1, scenario="town")
    est = {"sources": [{"kind": "radar", "id": "*"}] + ([{"kind": "object_classes", "id": "roadusers"}] if classes else [])}
    st = build(cfg={"estimator": dict(est, **(cfg or {}))}, mapper=None, clock=MessageClock())
    out = {"tracks": [], "lead": [], "view": [], "ego_state": []}
    st.mixer.add_output("t", lambda e: out[e.stream].append(copy.deepcopy(e.msg)), streams=tuple(out))
    k = 0
    for t, m in w.messages(default_sensors()):
        if t - w.t0 > seconds:
            break
        if fault_at is not None and t - w.t0 > fault_at:
            st.mixer.plugin("estimator").integ.carry_fault("vision", "test")
            fault_at = None
        if m["type"] == "road" and classes:
            near = [o for o in w.targets_at(m["ts_mono_ns"] / NS) if o["x"] > 2.0 and abs(o["y"]) < 1.5]
            if near:
                o = min(near, key=lambda o: o["x"])
                k += 1
                label = "car" if wrong_every and k % wrong_every == 0 else classes
                st.deliver({"type": "objects", "kind": "classes", "sensor": "roadusers", "ts_mono_ns": m["ts_mono_ns"] - 1,
                            "objects": [{"class": label, "class_conf": 0.8, "x_m": o["x"], "y_m": o["y"]},
                                        {"class": "human", "class_conf": 0.9, "bearing_deg": 35.0}]})   # points at nothing
        st.deliver(m)
    return st, out


def strip(msgs):
    """The messages without the class attribute (and without what names the class sensor)."""
    out = []
    for m in msgs:
        m = copy.deepcopy(m)
        m.pop("class", None)
        m.pop("sensors", None)
        for tr in m.get("tracks") or []:
            tr.pop("class", None)
        for o in m.get("objects") or []:
            o.pop("cls", None)
        out.append(m)
    return out


class ClassSlot(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.st0, cls.plain = drive(None)
        cls.st1, cls.cls = drive("truck", wrong_every=7)

    def test_the_slot_is_always_there_and_null_without_a_detector(self):
        tr = [t for m in self.plain["tracks"] for t in m["tracks"]]
        self.assertTrue(tr and all("class" in t and t["class"] is None for t in tr))
        self.assertTrue(all("class" in m and m["class"] is None for m in self.plain["lead"]))
        leads = [o for m in self.plain["view"] for o in m["objects"] if o["role"] == "lead"]
        self.assertTrue(leads and all("cls" in o and o["cls"] is None for o in leads))

    def test_class_with_confidence_and_source_on_track_lead_and_view(self):
        ld = [m for m in self.cls["lead"] if m["present"]]
        with_cls = [m for m in ld if m["class"]]
        self.assertGreater(len(with_cls) / len(ld), 0.9)
        c = with_cls[-1]["class"]
        self.assertEqual((c["label"], c["source"]), ("truck", "roadusers"))    # one "car" in seven does not flip it
        self.assertGreater(c["confidence"], 0.6)
        self.assertLessEqual(c["confidence"], 1.0)
        self.assertLess(c["age_s"], 0.5)
        tid = with_cls[-1]["track_id"]
        tr = next(t for t in self.cls["tracks"][-1]["tracks"] if t["id"] == tid)
        self.assertEqual(tr["class"]["label"], "truck")
        vo = [o for o in self.cls["view"][-1]["objects"] if o["role"] == "lead"][0]
        self.assertEqual(vo["cls"], {"label": "truck", "confidence": c["confidence"], "source": "roadusers"})
        self.assertEqual(vo["kind"], "vehicle")
        # the detection that pointed at nothing started nothing
        self.assertFalse(any((t["class"] or {}).get("label") == "human" for m in self.cls["tracks"] for t in m["tracks"]))

    def test_it_gates_nothing(self):
        """Tracks, the lead, the view and the ego state are the same, number for number, with and without the
        class detector (the class attribute apart)."""
        for topic in ("tracks", "lead", "view"):
            self.assertEqual(strip(self.plain[topic]), strip(self.cls[topic]), topic)
        a, b = copy.deepcopy(self.plain["ego_state"]), copy.deepcopy(self.cls["ego_state"])
        for m in a + b:
            m["sources"].pop("roadusers", None)
        self.assertEqual(a, b)
        self.assertEqual(self.cls["tracks"][-1]["sensors"]["roadusers"]["kind"], "classes")
        self.assertGreater(self.cls["tracks"][-1]["sensors"]["roadusers"]["assigned"], 100)

    def test_a_class_detector_alone_is_not_an_object_source(self):
        """With the camera faulted the lead is unknown; a class detector that keeps talking does not make it valid."""
        st, out = drive("truck", seconds=14.0, fault_at=8.0)
        ld = out["lead"][-1]
        self.assertEqual((ld["valid"], ld["present"]), (False, False))
        self.assertTrue(ld["reason"].startswith("vision"), ld["reason"])

    def test_class_from_a_tracking_camera_feeds_the_same_attribute(self):
        from chris_integrator.sim.camera2 import CameraObjectsSim
        w = World(seed=1, scenario="town")
        st = build(cfg={"estimator": {"sources": [{"kind": "camera_objects", "id": "cam_wide"}]}}, mapper=None, clock=MessageClock())
        tracks = []
        st.mixer.add_output("t", lambda e: tracks.append(e.msg), streams=("tracks",))
        for t, m in w.messages(default_sensors() + [CameraObjectsSim()]):
            if t - w.t0 > 30.0:
                break
            st.deliver(m)
        cl = [t["class"] for t in tracks[-1]["tracks"] if t["class"]]
        self.assertTrue(cl)
        self.assertEqual({c["label"] for c in cl}, {"car"})
        self.assertEqual({c["source"] for c in cl}, {"cam_wide"})


if __name__ == "__main__":
    unittest.main()
