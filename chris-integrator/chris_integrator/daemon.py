"""chris-integrator as a daemon: reads the sources' sockets, runs the integrator, serves its outputs.

    inputs   each roadsense instance's run directory (modeld.sock: `road`, `calib`; while the car still feeds
             roadsense's mapd also ego.sock, vehicle.sock, mapd.sock) and the integrator's own feed.sock (the
             car backend, vehicle input API: `motion`, `gnss`, `controls`, `body`, `objects`)
    output   <run dir>/integrator.sock: one `ego_state`, `tracks`, `lead` line per road frame (per vehicle sample
             while no camera frames arrive); a client gets the last of each when it connects

JSON lines over Unix stream sockets (the stack's convention). A slow client is cut at 1 MiB of backlog.
"""
import argparse
import asyncio
import json
import logging
import os
import time

from . import fastjson
from .core.integrator import Integrator
from .feed import Feed, hello

log = logging.getLogger("chris_integrator")
LINE_LIMIT = 16 * 1024 * 1024
BACKLOG = 1 << 20
OUT = ("ego_state", "tracks", "lead")


def load_config(path):
    try:
        with open(path) as f:
            cfg = json.load(f)
        return cfg if isinstance(cfg, dict) else {}
    except (OSError, ValueError):
        return {}


class Daemon:
    def __init__(self, rundir, sources, cfg=None, datadir=None):
        """sources: list of (run directory, [socket names]) to read."""
        self.rundir, self.sources, self.datadir = rundir, sources, datadir
        self.integ = Integrator(cfg)
        self.feed = Feed()
        self.clients = set()
        self.last = {}
        self.stats = {"road": 0, "in": 0, "out_bytes": 0, "cpu_s": 0.0, "t0": time.monotonic()}
        self.state_file = os.path.join(datadir, "integrator.json") if datadir and os.path.isdir(datadir) else None
        self._saved = None
        self._restore()

    # ---- carried state (faults, calibration) -----------------------------------------------------------
    def _restore(self):
        if not self.state_file:
            return
        saved = load_config(self.state_file)
        for g, rec in (saved.get("faults") or {}).items():
            if isinstance(rec, dict):
                self.integ.carry_fault(g, "carried over from the previous run: %s" % rec.get("reason"))
        self.integ.calibration_restore(saved.get("calibration"))
        self._saved = sorted(self.integ.faults())

    def _save(self, force=False):
        if not self.state_file:
            return
        faults = self.integ.faults()
        if not force and sorted(faults) == self._saved:
            return
        self._saved = sorted(faults)
        try:
            with open(self.state_file + ".tmp", "w") as f:
                json.dump({"ts_real_ns": time.time_ns(), "calibration": self.integ.calibration_snapshot(),
                           "faults": {g: {"reason": r.get("reason"), "rule": r.get("rule")} for g, r in faults.items()}}, f)
            os.replace(self.state_file + ".tmp", self.state_file)
        except OSError:
            pass

    # ---- one input message -------------------------------------------------------------------------------
    def on_line(self, raw):
        c = time.process_time()
        try:
            if raw.startswith(b'{"type":"road"'):
                msg = fastjson.loads_road(raw)
            else:
                msg = json.loads(raw)
        except ValueError:
            return
        if isinstance(msg, dict):
            self.on_message(msg)
        self.stats["cpu_s"] += time.process_time() - c

    def on_message(self, msg):
        I = self.integ
        t = msg.get("type")
        self.stats["in"] += 1
        try:
            if t == "road":
                self.stats["road"] += 1
                self.publish(I.frame(msg))
                self._save()
            else:
                I.on_message(msg)
                if t == "ego" and I.camera_silent() and isinstance(msg.get("ts_mono_ns"), int):
                    self.publish(I.tick(msg["ts_mono_ns"], msg.get("ts_real_ns")))
        except Exception:
            log.exception("integrator failed on a %s message", t)

    def publish(self, outs):
        data = b""
        for o in outs:
            if o is None:
                continue
            line = json.dumps(o, separators=(",", ":")).encode() + b"\n"
            self.last[o["type"]] = line
            data += line
        self.stats["out_bytes"] += len(data)
        for w in list(self.clients):
            if w.transport.get_write_buffer_size() > BACKLOG:
                self.clients.discard(w)
                w.close()
                continue
            w.write(data)

    # ---- sockets -----------------------------------------------------------------------------------------
    async def out_client(self, reader, writer):
        for t in OUT:
            if t in self.last:
                writer.write(self.last[t])
        self.clients.add(writer)
        try:
            await reader.read()
        except OSError:
            pass
        finally:
            self.clients.discard(writer)
            writer.close()

    async def feed_client(self, reader, writer):
        writer.write(json.dumps(hello(), separators=(",", ":")).encode() + b"\n")
        try:
            while True:
                raw = await reader.readline()
                if not raw:
                    break
                try:
                    m = json.loads(raw)
                except ValueError:
                    continue
                c = time.process_time()
                for im in self.feed.convert(m):
                    self.on_message(im)
                self.stats["cpu_s"] += time.process_time() - c
        except (OSError, ValueError):
            pass
        finally:
            writer.close()

    async def read_source(self, path):
        backoff = 0.2
        while True:
            try:
                reader, writer = await asyncio.open_unix_connection(path, limit=LINE_LIMIT)
            except OSError:
                await asyncio.sleep(backoff)
                backoff = min(backoff * 2, 3.0)
                continue
            backoff = 0.2
            log.info("reading %s", path)
            try:
                while True:
                    raw = await reader.readline()
                    if not raw:
                        break
                    self.on_line(raw)
            except (OSError, ValueError, asyncio.LimitOverrunError) as e:
                log.warning("%s: %s", path, e)
            finally:
                writer.close()
            await asyncio.sleep(backoff)

    async def run(self):
        os.makedirs(self.rundir, exist_ok=True)
        servers = []
        for name, cb in (("integrator.sock", self.out_client), ("feed.sock", self.feed_client)):
            p = os.path.join(self.rundir, name)
            try:
                os.unlink(p)
            except OSError:
                pass
            servers.append(await asyncio.start_unix_server(cb, path=p, limit=LINE_LIMIT))
            os.chmod(p, 0o666)
        tasks = [asyncio.create_task(self.read_source(os.path.join(d, s))) for d, socks in self.sources for s in socks]
        log.info("chris-integrator: %d source sockets, state size %d", len(tasks), self.integ.model.n)
        try:
            await asyncio.gather(*tasks)
        finally:
            self._save(force=True)
            for s in servers:
                s.close()


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--rundir", default=os.environ.get("CHRIS_RUNDIR", "/run/chris"))
    ap.add_argument("--datadir", default=os.environ.get("CHRIS_DATADIR", "/var/lib/chris"))
    ap.add_argument("--config", default=os.path.join(os.environ.get("CHRIS_CONFIG_DIR", "/etc/chris"), "integrator.json"))
    ap.add_argument("--roadsense", action="append", default=None,
                    help="run directory of a roadsense instance (repeat for more cameras; default /run/roadsense)")
    ap.add_argument("--no-mapd", action="store_true", help="do not read roadsense's mapd relays (the car feeds feed.sock)")
    a = ap.parse_args(argv)
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(name)s %(levelname)s %(message)s")
    cfg = load_config(a.config)
    socks = ["modeld.sock"] + ([] if a.no_mapd else ["ego.sock", "vehicle.sock", "mapd.sock"])
    sources = [(d, socks if i == 0 else ["modeld.sock"]) for i, d in enumerate(a.roadsense or ["/run/roadsense"])]
    if cfg.get("sources") is None:
        cfg["sources"] = [{"kind": "radar", "id": "*"}]
    asyncio.run(Daemon(a.rundir, sources, cfg, a.datadir).run())


if __name__ == "__main__":
    main()
