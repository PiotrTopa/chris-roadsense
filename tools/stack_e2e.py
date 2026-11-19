#!/usr/bin/env python3
"""Lane L39: the whole stack end to end on a laptop, from a recording.

    tools/stack_e2e.py SESSION --pos POS [--start S] [--seconds N] [--car-repo DIR] [--pack PACK] [--remote-mapper]

    a roadsense instance replaying the session    its daemons' recorded messages on real Unix sockets, at the
                                                  recorded pace: camerad.sock, modeld.sock (road, calib), signd.sock,
                                                  lightd.sock, envd.sock
    the car                                       the recorded vehicle motion, controls and fixes as a feeder
                                                  would send them: vehicle input API on chris-integrator's feed.sock
    chris-integrator (a real process)             python3 -m chris_integrator --roadsense <run dir> --http-port P
    a consumer                                    the car computer's own scene feed (its RoadsenseSceneFeed, read
                                                  from --car-repo, unmodified) on ws://127.0.0.1:P/ws, as the MFD's
                                                  backend connects; plus a plain WebSocket reader that times the hop

Printed: what the consumer got (scenes per second, with a lead / lane lines / a limit / the road picture), the
latency from a `road` line written on modeld.sock to its `view` line on the WebSocket, the daemon's CPU and
memory, and the mixer's own table (order, health, cost per plug-in) from GET /v2/mixer.
"""
import argparse
import asyncio
import json
import os
import socket
import subprocess
import sys
import tempfile
import threading
import time
import urllib.request

HERE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(HERE, "chris-integrator"))
from chris_integrator.replay import load_pos, read_session  # noqa: E402

NS = 1e9
SOCK = {"camera": "camerad", "road": "modeld", "calib": "modeld", "signs": "signd", "lights": "lightd", "env": "envd"}


def free_port():
    with socket.socket() as s:
        s.bind(("127.0.0.1", 0))
        return s.getsockname()[1]


class Pub:
    def __init__(self, path):
        self.path, self.clients, self.hello = path, set(), None

    async def start(self):
        async def client(reader, writer):
            if self.hello:
                writer.write(self.hello)
            self.clients.add(writer)
            try:
                await reader.read()
            except OSError:
                pass
            finally:
                self.clients.discard(writer)
        await asyncio.start_unix_server(client, path=self.path)

    def send(self, line):
        for w in list(self.clients):
            w.write(line)


def pct(v, p):
    v = sorted(v)
    return v[min(len(v) - 1, int(p / 100.0 * len(v)))] if v else float("nan")


async def play(a, rundir, chris_run, sent, at_end=None):
    """The session at its own pace: daemon messages on the roadsense sockets, the car's on feed.sock."""
    pubs = {d: Pub(os.path.join(rundir, d + ".sock")) for d in set(SOCK.values())}
    for p in pubs.values():
        await p.start()
    feed = None
    for _ in range(100):
        try:
            _, feed = await asyncio.open_unix_connection(os.path.join(chris_run, "feed.sock"))
            break
        except OSError:
            await asyncio.sleep(0.1)
    for _ in range(100):                               # until the integrator has connected to every socket
        if all(p.clients for p in pubs.values()):
            break
        await asyncio.sleep(0.1)
    t0 = w0 = r_off = m_off = None
    n = 0
    for m in read_session(a.session, road_model="vision", pos=load_pos(a.pos) if a.pos else None):
        ts, t = m.get("ts_mono_ns"), m.get("type")
        if not isinstance(ts, int):
            continue
        if t0 is None:
            t0 = ts
        rel = (ts - t0) / NS
        if t == "calib":                               # modeld sends it to every client that connects
            pubs["modeld"].hello = (json.dumps(m, separators=(",", ":")) + "\n").encode()
        if rel < a.start:
            continue
        if rel > a.start + a.seconds:
            break
        if w0 is None:
            w0 = time.monotonic() - rel
        wait = w0 + rel - time.monotonic()
        if wait > 0:
            await asyncio.sleep(wait)
        # the recording's clocks moved onto this host's (one constant offset each), as if the daemons ran now:
        # the stack compares stamps with its own monotonic clock (staleness of the map, latency markers)
        if m_off is None:
            m_off = time.monotonic_ns() - ts
        ts = m["ts_mono_ns"] = ts + m_off
        if isinstance(m.get("ts_real_ns"), int):
            if r_off is None:
                r_off = time.time_ns() - m["ts_real_ns"]
            m["ts_real_ns"] += r_off
        if t in SOCK:
            line = (json.dumps(m, separators=(",", ":")) + "\n").encode()
            if t == "road":
                sent[m.get("seq")] = time.monotonic()
                n += 1
            pubs[SOCK[t]].send(line)
        elif t == "ego" and feed is not None:          # the car backend's `motion`
            mo = {"type": "motion", "ts_mono_ns": ts, "speed_mps": abs(m.get("speed_mps") or 0.0), "source": m.get("tag") or "replay"}
            for k in ("gear", "reverse", "standstill", "wheel_speeds_mps", "yaw_rate_rps", "accel_long_mps2", "accel_lat_mps2", "turn_signal"):
                if m.get(k) is not None:
                    mo[k] = m[k]
            feed.write((json.dumps(mo, separators=(",", ":")) + "\n").encode())
        elif t == "gnss" and feed is not None:
            feed.write((json.dumps({k: v for k, v in m.items() if not k.startswith("_")}, separators=(",", ":")) + "\n").encode())
        elif t == "vehicle" and feed is not None and isinstance(m.get("controls"), dict):
            c = {k: v for k, v in m["controls"].items() if v is not None}
            if c:
                feed.write((json.dumps(dict(c, type="controls", ts_mono_ns=ts), separators=(",", ":")) + "\n").encode())
    await asyncio.sleep(0.5)
    if at_end is not None:                             # while the sockets are still up
        await asyncio.get_running_loop().run_in_executor(None, at_end)
    return n


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("session")
    ap.add_argument("--pos")
    ap.add_argument("--start", type=float, default=0.0)
    ap.add_argument("--seconds", type=float, default=120.0)
    ap.add_argument("--car-repo", default=os.path.expanduser("~/projects/cyberpank-prius-gen2-computer"))
    ap.add_argument("--pack", default=os.path.expanduser("~/roadsense-maps/packs/poland-20261114.rsmap"))
    ap.add_argument("--python", default=os.environ.get("PYTHON", sys.executable),
                    help="interpreter for the chris-integrator process (it needs numpy and aiohttp)")
    ap.add_argument("--remote-mapper", action="store_true", help="run the map box as its own process (mapper.json remote)")
    a = ap.parse_args()
    tmp = tempfile.mkdtemp(prefix="chris-e2e-")
    rundir, chris_run, etc = os.path.join(tmp, "roadsense"), os.path.join(tmp, "chris"), os.path.join(tmp, "etc")
    for d in (rundir, chris_run, etc):
        os.makedirs(d)
    env = dict(os.environ, PYTHONPATH=os.pathsep.join([os.path.join(HERE, "chris-integrator"), os.path.join(HERE, "chris-mapper")]),
               CHRIS_MAP_PACK=a.pack)
    if a.remote_mapper:
        with open(os.path.join(etc, "mapper.json"), "w") as f:
            json.dump({"remote": {"cmd": [a.python, "-m", "chris_mapper", "--plugin", "--pack", a.pack], "timeout_s": 0.25}}, f)
    port = free_port()
    proc = subprocess.Popen([a.python, "-m", "chris_integrator", "--rundir", chris_run, "--datadir", tmp, "--config-dir", etc,
                             "--roadsense", rundir, "--host", "127.0.0.1", "--http-port", str(port)], env=env,
                            stderr=open(os.path.join(tmp, "integrator.log"), "w"))
    base = "http://127.0.0.1:%d" % port
    for _ in range(100):
        try:
            urllib.request.urlopen(base + "/v2/version", timeout=1).read()
            break
        except OSError:
            time.sleep(0.1)
    # consumer 1: the car computer's scene feed, as its backend runs it
    # the one module, loaded from its file (the package pulls in the whole backend); nothing is written there
    import importlib.util
    sys.dont_write_bytecode = True
    spec = importlib.util.spec_from_file_location(
        "car_roadsense_scene", os.path.join(a.car_repo, "cyberpunk_computer", "backend", "roadsense_scene.py"))
    car = importlib.util.module_from_spec(spec)
    sys.modules["car_roadsense_scene"] = car
    spec.loader.exec_module(car)
    MiniWebSocket, RoadsenseSceneFeed, SceneFeedConfig, TOPICS = car.MiniWebSocket, car.RoadsenseSceneFeed, car.SceneFeedConfig, car.TOPICS
    scenes = []
    feed = RoadsenseSceneFeed(SceneFeedConfig(host="127.0.0.1", port=port), lambda sc: scenes.append((time.monotonic(), sc)))
    feed.set_active(True)
    feed.start()
    # consumer 2: a plain reader of the same WebSocket that times road -> view
    sent, seen = {}, {}
    stop = threading.Event()

    def reader():
        ws = MiniWebSocket.connect("127.0.0.1", port, "/ws?topics=view,lead", 3.0)
        n = {"view": 0, "lead": 0, "bytes": 0}
        while not stop.is_set():
            try:
                line = ws.recv(0.5)
            except ConnectionError:
                break
            if line is None:
                continue
            now = time.monotonic()
            n["bytes"] += len(line)
            if line.startswith('{"type":"view"'):
                n["view"] += 1
                i = line.find('"seq":')
                seen[int(line[i + 6:line.index(",", i)])] = now
            else:
                n["lead"] += 1
        seen["_n"] = n
    th = threading.Thread(target=reader, daemon=True)
    th.start()
    t_start = time.monotonic()
    try:
        got = {}

        def at_end():
            got["health"] = json.loads(urllib.request.urlopen(base + "/v2/health", timeout=3).read())
            got["mixer"] = json.loads(urllib.request.urlopen(base + "/v2/mixer", timeout=3).read())
        frames = asyncio.run(play(a, rundir, chris_run, sent, at_end))
        dur = time.monotonic() - t_start
        with open("/proc/%d/stat" % proc.pid) as f:
            st = f.read().split()
        cpu = (int(st[13]) + int(st[14])) / os.sysconf("SC_CLK_TCK")
        with open("/proc/%d/status" % proc.pid) as f:
            rss = next(int(ln.split()[1]) for ln in f if ln.startswith("VmRSS")) / 1024.0
        health, mixer = got["health"], got["mixer"]
    finally:
        stop.set()
        feed.stop()
        proc.terminate()
        proc.wait(10)
    th.join(2)
    n = seen.pop("_n", {})
    lat = [1e3 * (seen[s] - sent[s]) for s in seen if s in sent]
    sc = [s for _, s in scenes]
    print("== %s from %.0f s for %.0f s: %d road frames played; chris-integrator pid ran %.1f s" % (
        os.path.basename(a.session.rstrip("/")), a.start, a.seconds, frames, dur))
    print("consumer (the car's RoadsenseSceneFeed, topics %s):" % ",".join(TOPICS))
    print("  scenes %d = %.1f per second; with a lead %.1f %%; lead source tracked %.1f %%; with >= 1 lane line %.1f %%; "
          "with a speed limit %.1f %%; with the map layer %.1f %% (ok %.1f %%); with the road picture %.1f %%; display channel in use %.1f %%" % (
              len(sc), len(sc) / max(a.seconds, 1e-9),
              100.0 * sum(1 for s in sc if s.get("lead")) / max(len(sc), 1),
              100.0 * sum(1 for s in sc if s.get("lead_src") == "tracked") / max(len(sc), 1),
              100.0 * sum(1 for s in sc if any(s.get("lanes") or [])) / max(len(sc), 1),
              100.0 * sum(1 for s in sc if s.get("limit")) / max(len(sc), 1),
              100.0 * sum(1 for s in sc if isinstance(s.get("map"), dict)) / max(len(sc), 1),
              100.0 * sum(1 for s in sc if isinstance(s.get("map"), dict) and s["map"].get("ok")) / max(len(sc), 1),
              100.0 * sum(1 for s in sc if s.get("geo")) / max(len(sc), 1),
              100.0 * sum(1 for s in sc if s.get("vw")) / max(len(sc), 1)))
    print("WebSocket reader: view %d, lead %d messages (%.0f kB/s); road line written -> its view line read: p50 %.1f  p90 %.1f  p99 %.1f ms (n %d)" % (
        n.get("view", 0), n.get("lead", 0), n.get("bytes", 0) / 1e3 / max(dur, 1e-9), pct(lat, 50), pct(lat, 90), pct(lat, 99), len(lat)))
    print("daemon: cpu %.1f s = %.1f %% of one core (%.2f ms per road frame, all inputs and outputs included); resident memory %.0f MB" % (
        cpu, 100.0 * cpu / dur, 1e3 * cpu / max(frames, 1), rss))
    print("health: daemons %s; integrator sources %s; map %s" % (
        {k: v["connected"] for k, v in health["daemons"].items()},
        {k: v["state"] for k, v in (health["integrator"] or {}).get("sources", {}).items()},
        {k: (health["map"] or {}).get(k) for k in ("state", "available", "road", "limit_kmh", "motion_source")}))
    print("mixer (GET /v2/mixer):")
    for p in sorted(mixer["plugins"], key=lambda p: (p["order"] is None, p["order"])):
        print("  %2s %-13s %-8s calls %6d  %.3f ms/call  max %.1f ms  errors %d  outputs %d%s" % (
            p["order"], p["name"], p["health"].get("state"), p["calls"], p["ms_per_call"] or 0.0, p["max_ms"], p["errors"], p["outputs_n"],
            "  remote %s" % p["health"].get("transport") if p["remote"] else ""))
    print("  inputs: %s" % {k: v["messages"] for k, v in mixer["inputs"].items()})
    print("  outputs: %s" % list(mixer["outputs"]))


if __name__ == "__main__":
    main()
