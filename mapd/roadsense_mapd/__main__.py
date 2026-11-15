"""mapd: the roadsense map daemon.

    python3 -m roadsense_mapd [--rundir DIR] [--pack FILE] [--hz 2]

Sockets in the run directory (newline-delimited JSON, docs/ARCHITECTURE.md):
    mapd.sock       output: one `map` message per step; the last one is sent to a client when it connects
    position.sock   input:  `gnss` and `motion` messages from a feeder (position input API v1)
    modeld.sock     read (optional): `road` messages as vision odometry while no vehicle motion arrives
"""
import argparse
import asyncio
import json
import logging
import os
import re
import signal
import time

from . import API_VERSION
from .engine import DEFAULTS, Engine
from .rsmap import Pack

log = logging.getLogger("mapd")
RE_TS = re.compile(r'"ts_mono_ns":(\d+)')
RE_V = re.compile(r'"v_ego":(-?[\d.eE+-]+)')
RE_YAW = re.compile(r'"pose":\{[^}]*?"yaw_rate":(-?[\d.eE+-]+)')
LINE_LIMIT = 16 * 1024 * 1024


class Mapd:
    def __init__(self, a, cfg):
        self.a = a
        self.engine = Engine(None, cfg)
        self.pack_path = a.pack
        self.pack_id = None
        self.clients = set()
        self.last_line = None
        self.feeders = 0
        self.step_ms = 0.0
        self._vision_next = 0

    # ---- pack ----------------------------------------------------------------------------------
    def check_pack(self):
        """(Re)load the pack when the file behind the path appeared or changed (a new symlink target too)."""
        try:
            st = os.stat(self.pack_path)
            ident = (st.st_dev, st.st_ino, st.st_size, st.st_mtime_ns)
        except OSError:
            ident = None
        if ident == self.pack_id:
            return
        old = self.engine.pack
        if ident is None:
            log.warning("pack %s is gone; map layer unavailable", self.pack_path)
            self.engine.set_pack(None)
        else:
            try:
                pack = Pack(self.pack_path, cache_tiles=self.a.cache_tiles)
            except (OSError, ValueError) as e:
                log.error("cannot load pack %s: %s", self.pack_path, e)
                self.pack_id = ident
                return
            m = pack.manifest
            log.info("pack %s: %s, OSM %s, %d tiles, %d edges", self.pack_path, m.get("region"),
                     m.get("osm_timestamp"), m.get("tiles", 0), m.get("edges", 0))
            self.engine.set_pack(pack)
        self.pack_id = ident
        if old is not None:
            old.close()

    # ---- output socket ---------------------------------------------------------------------------
    async def out_client(self, reader, writer):
        self.clients.add(writer)
        if self.last_line:
            writer.write(self.last_line)
        try:
            await reader.read()
        except (OSError, asyncio.CancelledError):
            pass
        finally:
            self.clients.discard(writer)
            writer.close()

    def publish(self, msg):
        self.last_line = (json.dumps(msg, separators=(",", ":"), ensure_ascii=False) + "\n").encode()
        for w in list(self.clients):
            if w.transport.get_write_buffer_size() > 1 << 20:      # slow client: drop it
                w.close()
                self.clients.discard(w)
            else:
                w.write(self.last_line)

    # ---- input socket ----------------------------------------------------------------------------
    async def in_client(self, reader, writer):
        self.feeders += 1
        n = 0

        def reply(obj):
            writer.write((json.dumps(obj, separators=(",", ":")) + "\n").encode())

        try:
            while True:
                raw = await reader.readline()
                if not raw:
                    break
                n += 1
                line = raw.strip()
                if not line:
                    continue
                try:
                    msg = json.loads(line)
                except ValueError:
                    self.engine.counts["errors"] += 1
                    self.engine.last_error = "bad JSON"
                    reply({"type": "error", "line": n, "reason": "bad JSON"})
                    continue
                if isinstance(msg, dict) and msg.get("type") == "hello":
                    reply({"type": "hello", "api": API_VERSION, "server": "roadsense-mapd",
                           "accepts": ["gnss", "motion"], "clock": "CLOCK_MONOTONIC",
                           "now_mono_ns": time.monotonic_ns()})
                    continue
                err = self.engine.on_input(msg, time.monotonic_ns())
                if err:
                    reply({"type": "error", "line": n, "reason": err})
        except (OSError, ValueError, asyncio.LimitOverrunError) as e:
            log.warning("feeder: %s", e)
        finally:
            self.feeders -= 1
            writer.close()

    # ---- vision odometry -------------------------------------------------------------------------
    async def vision(self, path):
        eng = self.engine
        period = int(1e9 / self.a.vision_hz)
        while True:
            try:
                reader, writer = await asyncio.open_unix_connection(path, limit=LINE_LIMIT)
            except OSError:
                await asyncio.sleep(2.0)
                continue
            try:
                while True:
                    raw = await reader.readline()
                    if not raw:
                        break
                    now = time.monotonic_ns()
                    mo = eng.motion
                    if now < self._vision_next or (mo and mo["source"] == "vehicle" and now - mo["ts"] < 1.5e9):
                        continue                                    # not needed: skip without parsing
                    if not raw.startswith(b'{"type":"road"'):
                        continue
                    s = raw.decode("utf-8", "replace")
                    ts, v, yaw = RE_TS.search(s), RE_V.search(s), RE_YAW.search(s)
                    if not ts or not v:
                        continue
                    self._vision_next = now + period
                    try:
                        eng.on_vision({"ts_mono_ns": int(ts.group(1)), "v_ego": float(v.group(1)),
                                       "pose": {"yaw_rate": float(yaw.group(1))} if yaw else {}}, now)
                    except ValueError:
                        pass
            except (OSError, ValueError, asyncio.LimitOverrunError) as e:
                log.warning("modeld socket: %s", e)
            finally:
                writer.close()
            await asyncio.sleep(1.0)

    # ---- main loop -------------------------------------------------------------------------------
    async def run(self):
        a = self.a
        os.makedirs(a.rundir, exist_ok=True)
        out_path, in_path = os.path.join(a.rundir, "mapd.sock"), os.path.join(a.rundir, "position.sock")
        for p in (out_path, in_path):
            try:
                os.unlink(p)
            except FileNotFoundError:
                pass
        out_srv = await asyncio.start_unix_server(self.out_client, path=out_path)
        in_srv = await asyncio.start_unix_server(self.in_client, path=in_path, limit=1 << 20)
        os.chmod(in_path, a.input_mode)
        tasks = []
        if a.vision_odometry:
            tasks.append(asyncio.create_task(self.vision(os.path.join(a.rundir, "modeld.sock"))))
        stop = asyncio.Event()
        loop = asyncio.get_running_loop()
        for sig in (signal.SIGINT, signal.SIGTERM):
            loop.add_signal_handler(sig, stop.set)
        self.check_pack()
        period = 1.0 / a.hz
        nxt = time.monotonic()
        next_pack = nxt + a.pack_poll
        next_stats = nxt + a.stats if a.stats else None
        cpu0, wall0, steps = time.process_time(), nxt, 0
        cpu_pct = 0.0
        log.info("mapd up: %s (out), %s (in), %.1f Hz", out_path, in_path, a.hz)
        while not stop.is_set():
            t0 = time.perf_counter()
            msg = self.engine.step(time.monotonic_ns(), time.time_ns())
            self.step_ms = 0.8 * self.step_ms + 0.2 * (time.perf_counter() - t0) * 1e3
            msg["proc"] = {"step_ms": round(self.step_ms, 2), "cpu_pct": round(cpu_pct, 2), "feeders": self.feeders,
                           "tile_loads": self.engine.pack.tile_loads if self.engine.pack else 0}
            self.publish(msg)
            steps += 1
            now = time.monotonic()
            if now >= next_pack:
                next_pack = now + a.pack_poll
                self.check_pack()
                cpu, cpu0_, wall = time.process_time(), cpu0, now - wall0
                if wall >= 10.0:
                    cpu_pct = 100.0 * (cpu - cpu0_) / wall
                    cpu0, wall0 = cpu, now
            if next_stats and now >= next_stats:
                next_stats = now + a.stats
                c = self.engine.counts
                log.info("state %s available %s  gnss %d (used %d) motion %d vision %d errors %d  step %.2f ms  cpu %.2f %%",
                         msg["state"], msg["available"], c["gnss"], c["gnss_used"], c["motion"], c["vision"],
                         c["errors"], self.step_ms, cpu_pct)
            nxt += period
            if nxt < now:
                nxt = now + period
            try:
                await asyncio.wait_for(stop.wait(), max(0.0, nxt - time.monotonic()))
            except asyncio.TimeoutError:
                pass
        for t in tasks:
            t.cancel()
        out_srv.close()
        in_srv.close()
        for w in list(self.clients):
            w.close()
        for p in (out_path, in_path):
            try:
                os.unlink(p)
            except OSError:
                pass


def load_config(path):
    try:
        with open(path) as f:
            cfg = json.load(f)
    except FileNotFoundError:
        return {}
    if not isinstance(cfg, dict):
        raise SystemExit("%s: not a JSON object" % path)
    return cfg


def main():
    cfgdir = os.environ.get("ROADSENSE_CONFIG_DIR", "/etc/roadsense")
    cfg = load_config(os.path.join(cfgdir, "mapd.json"))
    ap = argparse.ArgumentParser(prog="roadsense_mapd", description="roadsense map daemon (position filter, map matching, map context)")
    ap.add_argument("--rundir", default=os.environ.get("ROADSENSE_RUNDIR", "/run/roadsense"))
    ap.add_argument("--pack", default=cfg.pop("pack", os.environ.get("ROADSENSE_MAP_PACK", "/var/lib/roadsense/maps/current.rsmap")),
                    help="map pack file (a symlink is followed; a changed file is reloaded)")
    ap.add_argument("--hz", type=float, default=cfg.pop("hz", 2.0), help="`map` messages per second")
    ap.add_argument("--cache-tiles", type=int, default=cfg.pop("cache_tiles", 24))
    ap.add_argument("--pack-poll", type=float, default=cfg.pop("pack_poll_s", 10.0), help="seconds between pack file checks")
    ap.add_argument("--no-vision-odometry", dest="vision_odometry", action="store_false",
                    default=cfg.pop("vision_odometry", True), help="do not read modeld.sock for speed / yaw rate")
    ap.add_argument("--vision-hz", type=float, default=cfg.pop("vision_hz", 10.0))
    ap.add_argument("--input-mode", type=lambda s: int(s, 8), default=int(str(cfg.pop("input_mode", "666")), 8),
                    help="permissions of position.sock (octal, default 666)")
    ap.add_argument("--stats", type=float, default=cfg.pop("stats_s", 60.0), help="log a status line every N seconds (0 = off)")
    ap.add_argument("-v", "--verbose", action="store_true")
    a = ap.parse_args()
    logging.basicConfig(level=logging.DEBUG if a.verbose else logging.INFO, format="%(asctime)s %(levelname)s %(name)s: %(message)s")
    unknown = [k for k in cfg if k not in DEFAULTS and k not in ("filter", "matcher")]
    if unknown:
        log.warning("mapd.json: unknown keys ignored: %s", ", ".join(unknown))
    asyncio.run(Mapd(a, {k: v for k, v in cfg.items() if k not in unknown}).run())


if __name__ == "__main__":
    main()
