"""The display lane source (lane L29): `display_lanes` from a provider (laned) on <rundir>/laned.sock.

* the measurement topics (`lead`, `lanes`, `fused` with its lanes and ego lane, the ego-lane filter, the road
  geometry) are the same, message for message, with and without a provider: only `view.lines` changes;
* the seam's three modes: "agree" (default: a line is the provider's where provider and road model agree on it),
  "prefer", "replace"; the scale that brings the provider's metres to the model's; road edges stay the model's;
* a provider that stops, or that the load governor pauses, returns the display to the model's lines.
"""
import copy
import json
import os
import sys
import unittest


from chris_integrator.replay import Pipeline  # noqa: E402
from .apistate import State  # noqa: E402
from .test_vision_road import X, road  # noqa: E402

DX = [0, 5, 10, 20, 40, 80]


def dl(s, left=-1.6, right=2.0, scale=1.0, far_left=None, edges=None):
    """A provider message for frame s. The road model of test_vision_road draws its lines at -1.8 and +1.8."""
    def ln(y):
        return {"present": False} if y is None else {"present": True, "y": [y * scale] * len(DX), "y_std": [0.1] * len(DX)}
    m = {"type": "display_lanes", "seq": s, "ts_mono_ns": s * 50_000_000, "source": "laned", "x": [x * scale for x in DX],
         "lines": [ln(far_left), ln(left), ln(right), ln(None)]}
    if edges is not None:
        m["edges"] = edges
    return m


def line_y(vw, lid, i=2):
    return next((ln["y"][i] for ln in vw["lines"] if ln["id"] == lid), None)


class MeasurementTopicsDoNotSeeIt(unittest.TestCase):
    def run_pipeline(self, with_provider):
        p = Pipeline()
        outs = []
        for s in range(1, 120):
            if with_provider and s >= 20:
                # a provider that disagrees wildly with the model (another marking, an extra line): the worst case
                p.feed(dl(s, left=-0.9, right=3.4, far_left=-4.4))
            for o in p.feed(road(s, plan=True)):
                outs.append(copy.deepcopy(o))
            outs.append(("egolane", copy.deepcopy(p.latest.get("egolane"))))
        return outs

    def test_same_measurement_topics_with_and_without_a_provider(self):
        a, b = self.run_pipeline(False), self.run_pipeline(True)
        self.assertEqual(len(a), len(b))
        views = 0
        for x, y in zip(a, b):
            if isinstance(x, dict) and x.get("type") == "view":
                views += 1
                # the road picture of the view is built from the measurement topics: unchanged too
                self.assertEqual(x["road"], y["road"])
                self.assertEqual(x["objects"], y["objects"])
                continue
            self.assertEqual(x, y)                    # road, lead, lanes, fused, the ego-lane filter: identical
        self.assertGreater(views, 100)
        kinds = {o["type"] for o in a if isinstance(o, dict)}
        self.assertTrue({"lead", "lanes", "fused", "view"} <= kinds)

    def test_in_the_api_state(self):
        def run(with_provider, mode="replace"):
            st = State("/nonexistent")
            st.view.cfg.update(display_lanes_mode=mode, display_scale=False)
            got = []
            st.subscribers.add(lambda t, line: got.append((t, json.loads(line))))
            for s in range(1, 60):
                if with_provider and s >= 10:
                    m = dl(s, left=-0.9, right=3.4)
                    st.on_message("laned", m, json.dumps(m))
                m = road(s, plan=True)
                st.on_message("modeld", m, json.dumps(m))
            return st, got
        st0, g0 = run(False)
        st1, g1 = run(True)
        for topic in ("lead", "lanes"):
            m0 = [m for t, m in g0 if t == topic]
            m1 = [m for t, m in g1 if t == topic]
            for x, y in zip(m0, m1):
                x.pop("rate", None), y.pop("rate", None)
            self.assertEqual(m0, m1, topic)
        self.assertEqual(st0.latest.get("egolane"), st1.latest.get("egolane"))
        v0, v1 = [m for t, m in g0 if t == "view"][-1], [m for t, m in g1 if t == "view"][-1]
        self.assertEqual((v0["lines_source"], v1["lines_source"]), ("model", "laned"))
        self.assertAlmostEqual(line_y(v0, "lane:1"), -1.8, delta=0.1)
        self.assertAlmostEqual(line_y(v1, "lane:1"), -0.9, delta=0.1)       # only the display follows the provider
        self.assertEqual(st1.health()["view"]["lines_source"], "laned")


class SeamModes(unittest.TestCase):
    def run_mode(self, mode, n=80, **kw):
        p = Pipeline(view_cfg={"display_lanes_mode": mode, "display_scale": kw.pop("display_scale", True),
                               "display_agree_extra": kw.pop("extra", False)})
        for s in range(1, 20):
            p.feed(road(s, plan=True))
        for s in range(20, n):
            p.feed(dl(s, **kw))
            p.feed(road(s, plan=True))
        return p

    def test_agree_is_the_default_and_takes_agreeing_lines(self):
        p = self.run_mode("agree", left=-1.6, right=2.0)                     # 0.2 m from the model's on both sides
        vw = p.latest["view"]
        self.assertEqual(Pipeline().view.cfg["display_lanes_mode"], "agree")
        self.assertEqual(vw["lines_source"], "laned")
        self.assertAlmostEqual(line_y(vw, "lane:1"), -1.6, delta=0.05)
        self.assertAlmostEqual(line_y(vw, "lane:2"), 2.0, delta=0.05)

    def test_agree_keeps_the_models_line_where_they_disagree(self):
        p = self.run_mode("agree", left=-0.9, right=2.0, display_scale=False)   # left: another marking; right agrees
        vw = p.latest["view"]
        self.assertEqual(vw["lines_source"], "laned+model")
        self.assertAlmostEqual(line_y(vw, "lane:1"), -1.8, delta=0.05)       # the model's
        self.assertAlmostEqual(line_y(vw, "lane:2"), 2.0, delta=0.05)        # the provider's
        p = self.run_mode("agree", left=-0.9, right=2.9, display_scale=False)   # nothing agrees: the model's picture
        vw = p.latest["view"]
        self.assertEqual(vw["lines_source"], "model")
        self.assertAlmostEqual(line_y(vw, "lane:2"), 1.8, delta=0.05)

    def test_agree_draws_an_extra_line_only_on_request_and_while_anchored(self):
        p = self.run_mode("agree", left=-1.7, right=1.9, far_left=-5.2)      # the model has no far-left line
        self.assertIsNone(line_y(p.latest["view"], "lane:0"))                # by default: only lines the model has too
        self.assertEqual(p.latest["view"]["lines_source"], "laned")
        p = self.run_mode("agree", left=-1.7, right=1.9, far_left=-5.2, extra=True)
        self.assertAlmostEqual(line_y(p.latest["view"], "lane:0"), -5.2, delta=0.05)
        p = self.run_mode("agree", left=-0.9, right=2.9, far_left=-5.2, display_scale=False, extra=True)
        self.assertIsNone(line_y(p.latest["view"], "lane:0"))                # not anchored: not drawn

    def test_prefer_and_replace(self):
        p = self.run_mode("prefer", left=-0.9, right=None, display_scale=False)
        vw = p.latest["view"]
        self.assertEqual(vw["lines_source"], "laned+model")
        self.assertAlmostEqual(line_y(vw, "lane:1"), -0.9, delta=0.05)
        self.assertAlmostEqual(line_y(vw, "lane:2"), 1.8, delta=0.05)        # the provider has none: the model's
        p = self.run_mode("replace", left=-0.9, right=None, display_scale=False)
        vw = p.latest["view"]
        self.assertEqual(vw["lines_source"], "laned")
        self.assertIsNone(line_y(vw, "lane:2"))                              # absent in the provider: not drawn

    def test_scale_brings_the_providers_metres_to_the_models(self):
        # the provider measures a 4.5 m lane where the model has 3.6 m (another camera height): factor 0.8
        p = self.run_mode("agree", n=400, left=-2.25, right=2.25, scale=1.0)
        vw = p.latest["view"]
        self.assertAlmostEqual(vw["lines_scale"], 0.8, delta=0.02)
        self.assertEqual(vw["lines_source"], "laned")
        self.assertAlmostEqual(line_y(vw, "lane:1"), -1.8, delta=0.06)
        self.assertIsNone(Pipeline().view.on_frame(road(1, plan=True), None, None)["lines_scale"])

    def test_edges_stay_the_models_unless_the_provider_sends_them(self):
        p = self.run_mode("agree", left=-1.6, right=2.0)
        ids = {ln["id"] for ln in p.latest["view"]["lines"]}
        base = Pipeline()
        for s in range(1, 80):
            base.feed(road(s, plan=True))
        self.assertEqual({i for i in ids if i.startswith("edge")}, {ln["id"] for ln in base.latest["view"]["lines"] if ln["id"].startswith("edge")})

    def test_a_provider_that_stops_returns_the_display_to_the_model(self):
        p = self.run_mode("agree", left=-1.6, right=2.0)
        for s in range(80, 100):                                             # 1 s without a message (limit 0.5 s)
            p.feed(road(s, plan=True))
        vw = p.latest["view"]
        self.assertEqual((vw["lines_source"], vw["lines_scale"]), ("model", None))
        self.assertAlmostEqual(line_y(vw, "lane:1"), -1.8, delta=0.05)

    def test_malformed_messages_are_ignored(self):
        p = Pipeline()
        for bad in ({}, {"type": "display_lanes"}, {"type": "display_lanes", "ts_mono_ns": "x", "x": [], "lines": []},
                    {"type": "display_lanes", "ts_mono_ns": 50_000_000, "x": [0, 10], "lines": [None, {"present": True}]}):
            p.view.on_display_lanes(bad)
            for s in range(1, 4):
                self.assertIsNotNone(p.view.on_frame(road(s, plan=True), None, None))


if __name__ == "__main__":
    unittest.main()
