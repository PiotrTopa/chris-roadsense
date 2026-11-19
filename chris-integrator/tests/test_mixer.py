"""The mixer and the plug-in contract: declarations, order, feedback, isolation of failures, adding / removing /
replacing / disabling a plug-in, a plug-in alone on the bench, a plug-in out of process."""
import json
import os
import sys
import unittest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from chris_integrator.mixer import In, MessageClock, Mixer, MixerError, Out, Plugin, StreamSpec  # noqa: E402
from chris_integrator.mixer.remote import RemotePlugin  # noqa: E402
from chris_integrator.mixer.testing import Bench  # noqa: E402
from chris_integrator.plugins.trackers import Lanes  # noqa: E402
from chris_integrator.stack import build  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))


class Double(Plugin):
    name = "double"
    inputs = (In("a", trigger=True),)
    outputs = (Out("b"),)

    def on_input(self, stream, msg, ctx):
        ctx.emit("b", {"type": "b", "ts_mono_ns": msg["ts_mono_ns"], "v": 2 * msg["v"]})


class Sum(Plugin):
    """Reads `b` of the same frame as the `a` it is triggered by."""
    name = "sum"
    inputs = (In("a", trigger=True), In("b"))
    outputs = (Out("c"),)

    def on_input(self, stream, msg, ctx):
        b = ctx.get("b", frame=True)
        ctx.emit("c", {"type": "c", "ts_mono_ns": msg["ts_mono_ns"], "v": msg["v"] + (b["v"] if b else 0), "had_b": b is not None})


class Boom(Plugin):
    name = "boom"
    inputs = (In("a", trigger=True),)
    outputs = (Out("x"),)

    def on_input(self, stream, msg, ctx):
        raise ValueError("broken box")


def mixer(*plugins):
    m = Mixer(MessageClock())
    ch = m.add_input("src", [StreamSpec("a", "raw", "sample")])
    got = []
    m.add_output("test", lambda e: got.append((e.stream, e.msg)))
    for p in plugins:
        m.add(p)
    return m, ch, got


def a(k, v=1):
    return {"type": "a", "ts_mono_ns": k * 1000, "v": v}


class Contract(unittest.TestCase):
    def test_order_follows_the_declarations_not_the_order_added(self):
        m, ch, got = mixer(Sum(), Double())                 # the consumer added first
        self.assertEqual([s.name for s in m.order], ["double", "sum"])
        ch.deliver("a", a(1, 5))
        self.assertEqual([t for t, _ in got], ["a", "b", "c"])       # input, then each box's output for that frame
        self.assertEqual(got[-1][1], {"type": "c", "ts_mono_ns": 1000, "v": 15, "had_b": True})

    def test_undeclared_streams_are_refused(self):
        class Sneaky(Plugin):
            name = "sneaky"
            inputs = (In("a", trigger=True),)
            outputs = (Out("y"),)

            def on_input(self, stream, msg, ctx):
                ctx.get("b")
        m, ch, _ = mixer(Double(), Sneaky())
        with self.assertRaises(MixerError):
            ch.deliver("a", a(1))
        with self.assertRaises(MixerError):
            ch.deliver("nope", a(1))
        with self.assertRaises(ValueError):                 # two producers of one stream
            class Double2(Double):
                name = "double2"
            m.add(Double2())
        self.assertNotIn("double2", m.slots)

    def test_a_failing_plugin_does_not_stop_the_others(self):
        m, ch, got = mixer(Boom(), Double(), Sum())
        ch.deliver("a", a(1))
        ch.deliver("a", a(2))
        self.assertEqual([t for t, _ in got].count("c"), 2)
        d = {p["name"]: p for p in m.describe()["plugins"]}
        self.assertEqual(d["boom"]["errors"], 2)
        self.assertIn("broken box", d["boom"]["last_error"])
        self.assertEqual(d["sum"]["errors"], 0)

    def test_loops_need_feedback_and_a_plugin_is_never_re_entered(self):
        class Est(Plugin):
            name = "est"
            inputs = (In("a", trigger=True), In("m", trigger=True, optional=True, feedback=True))
            outputs = (Out("e"), Out("e2"))

            def __init__(self):
                super().__init__()
                self.log = []

            def on_input(self, stream, msg, ctx):
                self.log.append("in:" + stream)
                if stream == "a":
                    ctx.emit("e", {"type": "e", "ts_mono_ns": msg["ts_mono_ns"]})
                    self.log.append("between")
                    ctx.emit("e2", {"type": "e2", "ts_mono_ns": msg["ts_mono_ns"]})
                    self.log.append("out:a")

        class Map(Plugin):
            name = "map"
            inputs = (In("e", trigger=True),)
            outputs = (Out("m"),)

            def on_input(self, stream, msg, ctx):
                ctx.emit("m", {"type": "m", "ts_mono_ns": msg["ts_mono_ns"]})
        est = Est()
        m, ch, got = mixer(est, Map())
        ch.deliver("a", a(1))
        self.assertEqual([t for t, _ in got], ["a", "e", "m", "e2"])
        # the map's answer came back while the estimator was still publishing: delivered after it had finished
        self.assertEqual(est.log, ["in:a", "between", "out:a", "in:m"])

        class Est2(Est):
            inputs = (In("a", trigger=True), In("m", trigger=True))        # the same loop, not declared as one
        with self.assertRaises(MixerError):
            mixer(Est2(), Map())

    def test_add_remove_replace_disable_at_run_time(self):
        m, ch, got = mixer(Double())
        ch.deliver("a", a(1))
        m.add(Sum())                                         # added while running
        ch.deliver("a", a(2, 3))
        self.assertEqual(got[-1][1]["v"], 9)
        m.enable("double", False)                            # disabled: its stream is gone, the consumer says so
        ch.deliver("a", a(3, 3))
        self.assertEqual(got[-1][1], {"type": "c", "ts_mono_ns": 3000, "v": 3, "had_b": False})
        d = {p["name"]: p for p in m.describe()["plugins"]}
        self.assertEqual(d["double"]["health"]["state"], "disabled")
        self.assertEqual((d["sum"]["health"]["state"], d["sum"]["health"]["reason"]), ("idle", "no producer for b"))
        m.enable("double", True)

        class Triple(Double):
            name = "triple"

            def on_input(self, stream, msg, ctx):
                ctx.emit("b", {"type": "b", "ts_mono_ns": msg["ts_mono_ns"], "v": 3 * msg["v"]})
        m.replace("double", Triple())                        # another box in its place: same stream, same consumers
        ch.deliver("a", a(4, 1))
        self.assertEqual(got[-1][1]["v"], 4)
        self.assertEqual(m.bus.streams["b"].producer, "triple")
        m.remove("sum")
        n = len(got)
        ch.deliver("a", a(5))
        self.assertEqual([t for t, _ in got[n:]], ["a", "b"])

    def test_describe_lists_the_wiring_and_the_cost(self):
        m, ch, _ = mixer(Double(), Sum())
        for k in range(1, 20):
            ch.deliver("a", a(k * 100000))
        d = m.describe()
        self.assertEqual(d["streams"]["b"]["producer"], "double")
        self.assertEqual(d["streams"]["b"]["consumers"], ["sum"])
        self.assertEqual(d["streams"]["a"]["producer"], "src")
        self.assertEqual(d["inputs"]["src"]["messages"], 19)
        p = {x["name"]: x for x in d["plugins"]}
        self.assertEqual((p["double"]["calls"], p["double"]["order"], p["sum"]["order"]), (19, 0, 1))
        self.assertGreater(p["sum"]["ms_per_call"], 0.0)
        json.dumps(d)                                        # serialisable: it is served as /v2/mixer


class Alone(unittest.TestCase):
    def test_a_real_plugin_on_the_bench(self):
        from .test_vision_road import road
        b = Bench(Lanes())
        out = []
        for s in range(1, 30):
            out += b.feed("road", road(s))
        self.assertEqual({t for t, _ in out}, {"lanes"})
        self.assertEqual(len(out), 29)
        self.assertEqual(sum(1 for ln in out[-1][1]["lines"] if ln["present"]), 2)
        self.assertEqual(b.health()["errors"], 0)


class DefaultStack(unittest.TestCase):
    def test_order_and_what_each_box_declares(self):
        st = build(mapper=None)
        names = [s.name for s in st.mixer.order]
        for before, after in (("estimator", "lanes"), ("lanes", "egolane"), ("lanes", "roadgeom"), ("roadgeom", "view"),
                              ("ego_light", "sign_context"), ("sign_context", "rules"), ("rules", "fused"), ("estimator", "view")):
            self.assertLess(names.index(before), names.index(after), (before, after))
        d = st.mixer.describe()
        pub = {n for n, s in d["streams"].items() if s["public"]}
        self.assertTrue({"road", "lead", "lanes", "view", "fused", "rules", "speed_limit", "lights", "signs", "ego_state",
                         "tracks"} <= pub)
        self.assertFalse({"lights_raw", "signs_raw", "egolane", "roadgeom"} & pub)      # wiring, not API
        self.assertEqual(d["streams"]["lights"]["producer"], "ego_light")

    def test_a_plugin_from_the_configuration_and_one_left_out(self):
        import tempfile
        d = tempfile.mkdtemp(prefix="chris-cfg-")
        with open(os.path.join(d, "mixer.json"), "w") as f:
            json.dump({"disable": ["fused"], "plugins": [{"class": "tests.test_mixer:Double"}]}, f)
        sys.path.insert(0, os.path.dirname(HERE))
        st = build(config_dir=d, mapper=None)
        names = [s.name for s in st.mixer.order]
        self.assertNotIn("fused", names)
        self.assertNotIn("fused", st.mixer.bus.streams)
        self.assertIn("double", st.mixer.slots)


class OutOfProcess(unittest.TestCase):
    """The same box behind the same contract in another process."""
    CMD = [sys.executable, "-c", "import sys; sys.path.insert(0, %r); from tests.test_mixer import Double; "
           "from chris_integrator.mixer.remote import serve; serve(Double(), sys.stdin.buffer, sys.stdout.buffer)" % os.path.dirname(HERE)]

    def test_sync_remote_plugin_behaves_as_the_local_one(self):
        r = RemotePlugin(cmd=self.CMD, timeout_s=5.0)
        self.assertEqual((r.name, [i.name for i in r.inputs], [o.name for o in r.outputs]), ("double", ["a"], ["b"]))
        m, ch, got = mixer(Sum(), r)
        try:
            self.assertEqual([s.name for s in m.order], ["double", "sum"])
            for k in range(1, 6):
                ch.deliver("a", a(k, k))
            self.assertEqual([msg["v"] for t, msg in got if t == "c"], [3, 6, 9, 12, 15])
            h = next(p for p in m.describe()["plugins"] if p["name"] == "double")
            self.assertTrue(h["remote"])
            self.assertEqual((h["health"]["state"], h["health"]["transport"]["timeouts"]), ("ok", 0))
        finally:
            m.close()

    def test_async_remote_plugin_publishes_when_it_answers(self):
        import select
        r = RemotePlugin(cmd=self.CMD, sync=False)
        m, ch, got = mixer(r)
        try:
            ch.deliver("a", a(1, 4))                         # sent, not waited for
            self.assertEqual([t for t, _ in got], ["a"])
            self.assertTrue(select.select([r.fileno()], [], [], 5.0)[0])     # what an event loop would watch
            r.pump()
            self.assertEqual(got[-1], ("b", {"type": "b", "ts_mono_ns": 1000, "v": 8}))
        finally:
            m.close()

    def test_a_dead_remote_plugin_is_a_fault_not_a_stall(self):
        r = RemotePlugin(cmd=self.CMD, timeout_s=0.3, retry_s=0.0)
        m, ch, got = mixer(Sum(), r)
        try:
            ch.deliver("a", a(1, 1))
            r.proc.kill()
            r.proc.wait()
            ch.deliver("a", a(2, 1))                         # the frame goes on without it
            self.assertEqual(got[-1][1]["had_b"], False)
            self.assertEqual(next(p for p in m.describe()["plugins"] if p["name"] == "double")["health"]["state"], "fault")
            ch.deliver("a", a(3, 1))                         # restarted on the next message
            self.assertEqual(got[-1][1], {"type": "c", "ts_mono_ns": 3000, "v": 3, "had_b": True})
        finally:
            m.close()


if __name__ == "__main__":
    unittest.main()
