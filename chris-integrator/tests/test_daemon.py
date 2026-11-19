"""The daemon: sources over Unix sockets in, `ego_state` / `tracks` / `lead` lines out; the car's direct feed."""
import asyncio
import json
import os
import tempfile
import unittest

from . import simrun  # noqa: F401  (path)
from chris_integrator import fastjson
from chris_integrator.daemon import Daemon
from chris_integrator.feed import Feed
from chris_integrator.sim import World, default_sensors


class DaemonTest(unittest.TestCase):
    def test_sockets_end_to_end(self):
        w = World(seed=1)
        msgs = [(t, m) for t, m in w.messages(default_sensors()) if t - w.t0 < 20.0]

        async def go():
            tmp = tempfile.mkdtemp(prefix="chris-test-")
            rs, run = os.path.join(tmp, "rs"), os.path.join(tmp, "chris")
            os.makedirs(rs)
            clients = []

            async def modeld(reader, writer):
                clients.append(writer)
                await reader.read()
            srv = await asyncio.start_unix_server(modeld, path=os.path.join(rs, "modeld.sock"))
            d = Daemon(run, [(rs, ["modeld.sock"])], {}, tmp)
            task = asyncio.create_task(d.run())
            for _ in range(100):
                await asyncio.sleep(0.02)
                if clients and os.path.exists(os.path.join(run, "feed.sock")):
                    break
            fr, fw = await asyncio.open_unix_connection(os.path.join(run, "feed.sock"))
            hello = json.loads(await fr.readline())
            out_r, out_w = await asyncio.open_unix_connection(os.path.join(run, "integrator.sock"), limit=1 << 24)
            for t, m in msgs:
                if m["type"] == "road":
                    clients[0].write(json.dumps(m, separators=(",", ":")).encode() + b"\n")
                else:
                    fw.write(json.dumps(dict(m, type="motion") if m["type"] == "ego" else m).encode() + b"\n")
                await asyncio.sleep(0)
            await asyncio.sleep(0.3)
            got = {}
            try:
                while True:
                    raw = await asyncio.wait_for(out_r.readline(), 0.2)
                    if not raw:
                        break
                    m = json.loads(raw)
                    got.setdefault(m["type"], []).append(m)
            except asyncio.TimeoutError:
                pass
            task.cancel()
            srv.close()
            return hello, got, d
        hello, got, d = asyncio.run(go())
        self.assertIn("motion", hello["accepts"])
        frames = {m["ts_mono_ns"] for _, m in msgs if m["type"] == "road"}
        for t in ("ego_state", "tracks", "lead"):               # every road frame is answered by all three
            self.assertTrue(frames <= {m["ts_mono_ns"] for m in got[t]}, t)
        self.assertGreater(len(got["ego_state"]), len(frames))      # and the car's samples before the first frame too
        es, ld = got["ego_state"][-1], got["lead"][-1]
        self.assertEqual(es["sources"]["vehicle"]["state"], "ok")            # the car's feed arrived over feed.sock
        self.assertTrue(es["speed"]["valid"])
        self.assertEqual(ld["ts_mono_ns"], es["ts_mono_ns"])
        self.assertTrue(ld["present"])
        self.assertTrue(os.path.exists(d.state_file))                        # calibration and faults carried on disk

    def test_feed_and_partial_parse(self):
        f = Feed()
        ego = f.convert({"type": "motion", "ts_mono_ns": 5, "speed_mps": 3.0, "gear": "R", "yaw_rate_rps": 0.1})[0]
        self.assertEqual((ego["type"], ego["speed_mps"], ego["reverse"], ego["yaw_source"]), ("ego", -3.0, True, "yaw_rate"))
        v = f.convert({"type": "controls", "brake": True}, now_ns=77)[0]
        self.assertEqual((v["type"], v["controls"]["brake"], v["ts_mono_ns"]), ("vehicle", True, 77))
        self.assertEqual(f.convert({"type": "gnss", "age_ms": 500, "lat": 1.0}, now_ns=10 ** 9)[0]["ts_mono_ns"], 5 * 10 ** 8)
        road = {"type": "road", "seq": 1, "pose": {"vx": 1.0}, "lane_lines": [{"x": [1, 2], "y": [0.5, 1], "prob": 0.3}],
                "road_edges": [{"y": [1.0]}], "leads": [{"prob": 0.9, "x": [10.0]}], "plan": {"x": [0, 1], "y": [0, 0]},
                "meta": {"a": [1, 2], "b": {"c": 3}}}
        line = json.dumps(road, separators=(",", ":")).encode()
        got = fastjson.loads_road(line)
        self.assertEqual({k: road[k] for k in ("type", "seq", "pose", "leads", "plan")}, got)
        odd = json.dumps(dict(road, lane_lines=[{"note": 'a " quote'}]), separators=(",", ":")).encode()
        self.assertEqual(fastjson.loads_road(odd)["pose"], road["pose"])     # anything unexpected: the whole line is parsed


if __name__ == "__main__":
    unittest.main()
