#!/usr/bin/env python3
"""What the integrator costs as a process of its own: starts the daemon, plays a simulated drive into it over
Unix sockets at real pace (the road model on a `modeld.sock`, the car on `feed.sock`), reads `integrator.sock`,
and reports the added latency (road line written -> its `lead` line read) and the daemon's CPU.

    python3 tools/hop_cost.py [--seconds 60] [--radar]
"""
import argparse
import asyncio
import json
import os
import subprocess
import sys
import tempfile
import time

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)
from chris_integrator.sim import World, default_sensors  # noqa: E402
from chris_integrator.sim.radar import RadarSim  # noqa: E402


async def main(a):
    w = World(seed=1)
    sens = default_sensors() + ([RadarSim()] if a.radar else [])
    msgs = [(t, m) for t, m in w.messages(sens) if t - w.t0 < a.seconds]
    tmp = tempfile.mkdtemp(prefix="chris-hop-")
    rs, run = os.path.join(tmp, "roadsense"), os.path.join(tmp, "chris")
    os.makedirs(rs)
    clients = []

    async def modeld(reader, writer):
        clients.append(writer)
        await reader.read()
    srv = await asyncio.start_unix_server(modeld, path=os.path.join(rs, "modeld.sock"))
    proc = subprocess.Popen([sys.executable, "-m", "chris_integrator", "--rundir", run, "--datadir", tmp, "--roadsense", rs,
                             "--no-mapd", "--config", os.path.join(tmp, "none.json")], cwd=ROOT,
                            env=dict(os.environ, PYTHONPATH=ROOT), stderr=subprocess.DEVNULL)
    for _ in range(100):
        if os.path.exists(os.path.join(run, "feed.sock")) and clients:
            break
        await asyncio.sleep(0.1)
    fr, fw = await asyncio.open_unix_connection(os.path.join(run, "feed.sock"))
    await fr.readline()
    out_r, _ = await asyncio.open_unix_connection(os.path.join(run, "integrator.sock"), limit=1 << 24)
    sent, lat, sizes = {}, [], {}

    async def reader():
        while True:
            raw = await out_r.readline()
            if not raw:
                return
            now = time.perf_counter()
            t0 = time.perf_counter()
            m = json.loads(raw)
            sizes.setdefault(m["type"], []).append((len(raw), time.perf_counter() - t0))
            if m["type"] == "lead" and m["ts_mono_ns"] in sent:
                lat.append(now - sent.pop(m["ts_mono_ns"]))
    task = asyncio.create_task(reader())
    clk = os.sysconf("SC_CLK_TCK")

    def cpu():
        with open("/proc/%d/stat" % proc.pid) as f:
            p = f.read().rsplit(")", 1)[1].split()
        return (int(p[11]) + int(p[12])) / clk
    c0, t_start, sim0 = cpu(), time.perf_counter(), msgs[0][0]
    n_road = 0
    for t, m in msgs:
        d = (t - sim0) - (time.perf_counter() - t_start)
        if d > 0:
            await asyncio.sleep(d)
        if m["type"] == "road":
            line = json.dumps(m, separators=(",", ":")).encode() + b"\n"
            sent[m["ts_mono_ns"]] = time.perf_counter()
            n_road += 1
            for c in clients:
                c.write(line)
        else:
            if m["type"] == "ego":
                m = dict(m, type="motion")
            fw.write(json.dumps(m, separators=(",", ":")).encode() + b"\n")
    await asyncio.sleep(0.5)
    c1, wall = cpu(), time.perf_counter() - t_start
    with open("/proc/%d/status" % proc.pid) as f:
        rss = next(int(x.split()[1]) for x in f if x.startswith("VmRSS"))
    proc.terminate()
    task.cancel()
    srv.close()
    lat.sort()
    q = lambda p: 1e3 * lat[min(int(p * len(lat)), len(lat) - 1)]
    print("%d s of simulated driving at real pace%s: %d road frames, %d answered" % (a.seconds, " with a radar" if a.radar else "", n_road, len(lat)))
    print("  added latency, road line written -> lead line read: p50 %.2f ms, p90 %.2f ms, p99 %.2f ms" % (q(0.5), q(0.9), q(0.99)))
    print("  daemon CPU %.2f s over %.1f s = %.2f %% of one core; %.2f ms per road frame (all inputs); resident memory %.0f MB" % (
        c1 - c0, wall, 100.0 * (c1 - c0) / wall, 1e3 * (c1 - c0) / max(n_road, 1), rss / 1024.0))
    for k, v in sizes.items():
        print("  output %-9s %5.0f bytes, a consumer's parse %.1f us" % (k, sum(x for x, _ in v) / len(v), 1e6 * sum(y for _, y in v) / len(v)))


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--seconds", type=float, default=60.0)
    ap.add_argument("--radar", action="store_true")
    asyncio.run(main(ap.parse_args()))
