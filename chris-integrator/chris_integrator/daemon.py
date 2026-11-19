"""chris-integrator as a daemon: the mixer with its channels.

    input channels
      each chris-roadsense instance's run directory: camerad.sock (`camera`), modeld.sock (`road`, `calib`),
        signd.sock (`signs`), lightd.sock (`lights`), envd.sock (`env`), laned.sock (`display_lanes`),
        api.sock (`governor`)
      <run dir>/feed.sock   the car backend (vehicle input API: `motion`, `gnss`, `controls`, `body`, `objects`);
                            also under the name position.sock
      POST /v2/vehicle      the same over HTTP
      --legacy-mapd         also a roadsense mapd's relays (ego.sock, vehicle.sock) while a car still feeds it
    plug-ins                the stack (`chris_integrator.stack`): estimator, map box, trackers, decisions, view
    output channels
      HTTP + WebSocket      the consolidated API (/v2/..., /ws), --http-port (0: off)
      <run dir>/integrator.sock   JSON lines of the public streams (a first line {"subscribe": [...]} narrows;
                            default: `ego_state`, `tracks`, `lead`)
      the recorder          <config dir>/recorder.json

JSON lines over Unix stream sockets (the stack's convention). A slow client is cut at 1 MiB of backlog.
"""
import argparse
import asyncio
import json
import logging
import os
import time

from .io.feed import Feed, hello
from .io.sockets import LINE_LIMIT, LineServer, Reader
from .stack import build

log = logging.getLogger("chris_integrator")
OUT = ("ego_state", "tracks", "lead")
ROADSENSE_SOCKETS = ("camerad", "modeld", "signd", "lightd", "envd", "laned", "api")
LEGACY_SOCKETS = ("ego", "vehicle")


def load_config(path):
    try:
        with open(path) as f:
            cfg = json.load(f)
        return cfg if isinstance(cfg, dict) else {}
    except (OSError, ValueError):
        return {}


class Daemon:
    def __init__(self, rundir, sources, cfg=None, datadir=None, config_dir=None, http=None, roadsense_http=None,
                 mapper="auto", out_streams=OUT, record="config"):
        """sources: list of (run directory of a roadsense instance, [socket names]); the first is the default
        instance, a further one's streams are suffixed with its directory's name (`road@rear`).
        cfg: the estimator's configuration (integrator.json). http: (host, port) or None."""
        self.rundir, self.sources, self.datadir, self.http, self.roadsense_http = rundir, sources, datadir, http, roadsense_http
        self.stack = build(cfg={"estimator": cfg or {}}, config_dir=config_dir, datadir=datadir, mapper=mapper)
        self.mixer = self.stack.mixer
        self.integ = self.mixer.plugin("estimator").integ
        self.feed = Feed()
        self.readers = {}
        self.out = LineServer(os.path.join(rundir, "integrator.sock"), streams=None)
        self.out_default = set(out_streams)
        self.mixer.add_output("socket", self._to_socket, info="integrator.sock: JSON lines for other processes")
        self.recorder = None
        if record:
            from .io import recorder as recorder_mod
            rec_cfg = recorder_mod.load_config(config_dir) if record == "config" else record
            if rec_cfg:
                preview = os.path.join(sources[0][0], "preview.jpg") if sources else None
                self.recorder = recorder_mod.Recorder(rec_cfg, preview)
                self.mixer.add_output("recorder", self._record, info="drive recorder")
        self.stats = {"road": 0, "in": 0, "cpu_s": 0.0, "t0": time.monotonic()}
        st = self.mixer.services["store"]
        self.state_file = st.path("estimator") if st.dir else None

    # ---- output channels ---------------------------------------------------------------------------------
    def _to_socket(self, env):
        out = self.out
        if not out.clients and env.stream not in self.out_default:
            return
        data = None
        for w, want in list(out.clients.items()):
            if env.stream not in (self.out_default if want is None else want):
                continue
            if data is None:
                data = env.line.encode() + b"\n"
            if w.transport.get_write_buffer_size() > (1 << 20):
                del out.clients[w]
                w.close()
                continue
            w.write(data)
            out.out_bytes += len(data)
        if env.stream in self.out_default:
            out.last[env.stream] = data if data is not None else env.line.encode() + b"\n"

    def _record(self, env):
        rec = self.recorder
        if env.stream == "governor":
            rec.frames_enabled = (env.msg.get("caps") or {}).get("record_frames", -1) != 0
        rec.on_envelope(env)

    # ---- input channels ----------------------------------------------------------------------------------
    def on_message(self, msg, line=None, instance=None):
        c = time.process_time()
        self.stats["in"] += 1
        if msg.get("type") == "road":
            self.stats["road"] += 1
        try:
            self.stack.deliver(msg, line, instance)
        except Exception:
            log.exception("the mixer failed on a %s message", msg.get("type"))
        self.stats["cpu_s"] += time.process_time() - c

    def feed_messages(self, msgs):
        """Feeder messages (vehicle input API) from HTTP: {"accepted", "rejected"}."""
        rejected = []
        for k, m in enumerate(msgs):
            out = self.feed.convert(m)
            if not out and m.get("type") not in ("hello", "reset", "vehicle"):
                rejected.append({"index": k, "reason": "unknown type %r" % (m.get("type"),)})
            for im in out:
                self.on_message(im)
        return {"accepted": len(msgs) - len(rejected), "rejected": rejected}

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
                for im in self.feed.convert(m):
                    self.on_message(im)
        except (OSError, ValueError):
            pass
        finally:
            writer.close()

    async def run(self):
        os.makedirs(self.rundir, exist_ok=True)
        await self.out.start()
        feed_srvs = []
        for name in ("feed.sock", "position.sock"):     # position.sock: the name a car backend written for mapd knows
            p = os.path.join(self.rundir, name)
            try:
                os.unlink(p)
            except OSError:
                pass
            feed_srvs.append(await asyncio.start_unix_server(self.feed_client, path=p, limit=LINE_LIMIT))
            os.chmod(p, 0o666)
        tasks = []
        for i, (d, socks) in enumerate(self.sources):
            inst = None if i == 0 else os.path.basename(d.rstrip("/"))
            if inst is not None:
                from .stack import ROADSENSE
                self.stack.add_channel("roadsense@" + inst, ROADSENSE, instance=inst, info="a further chris-roadsense instance")
            for s in socks:
                name = s[:-5] if s.endswith(".sock") else s
                r = Reader(os.path.join(d, name + ".sock"), lambda m, ln, inst=inst: self.on_message(m, ln, inst), name)
                if i == 0:
                    self.readers[name] = r
                tasks.append(asyncio.create_task(r.run()))
        runner = None
        if self.http:
            from aiohttp import web
            from .io.http import make_app
            app = make_app(self.stack, self.readers, self.datadir, self.feed_messages, self.recorder, self.roadsense_http)
            runner = web.AppRunner(app, access_log=None)
            await runner.setup()
            await web.TCPSite(runner, self.http[0], self.http[1]).start()
        loop = asyncio.get_running_loop()
        for slot in self.mixer.slots.values():          # out-of-process plug-ins in async mode: publish as they answer
            pl = slot.plugin
            if getattr(pl, "remote", False) and not pl.sync and pl.fileno() is not None:
                loop.add_reader(pl.fileno(), pl.pump)
        log.info("chris-integrator: %d source sockets, %d plug-ins (%s), state size %d, http %s", len(tasks),
                 len(self.mixer.order), ", ".join(s.name for s in self.mixer.order), self.integ.model.n, self.http)
        try:
            await asyncio.gather(*tasks) if tasks else await asyncio.Event().wait()
        finally:
            self.stack.save()
            self.mixer.close()
            self.out.close()
            for srv in feed_srvs:
                srv.close()
            if self.recorder is not None:
                self.recorder.close()
            if runner is not None:
                await runner.cleanup()


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    env = os.environ.get
    ap.add_argument("--rundir", default=env("CHRIS_RUNDIR", "/run/chris"))
    ap.add_argument("--datadir", default=env("CHRIS_DATADIR", "/var/lib/chris"))
    ap.add_argument("--config-dir", default=env("CHRIS_CONFIG_DIR", "/etc/chris"))
    ap.add_argument("--config", default=None, help="the estimator's configuration file (default <config dir>/integrator.json)")
    ap.add_argument("--roadsense", action="append", default=None,
                    help="run directory of a chris-roadsense instance (repeat for more cameras; default /run/roadsense)")
    ap.add_argument("--roadsense-http", default=env("CHRIS_ROADSENSE_HTTP"),
                    help="base URL of the first instance's debug face; /preview, /v2/frame.jpg, /v2/governor are forwarded to it")
    ap.add_argument("--legacy-mapd", action="store_true",
                    help="also read a roadsense mapd's relays (ego.sock, vehicle.sock) while a car still feeds its position.sock")
    ap.add_argument("--no-mapd", action="store_true", help="(default now; kept for older unit files)")
    ap.add_argument("--no-mapper", action="store_true", help="run without the map box")
    ap.add_argument("--host", default=env("CHRIS_API_HOST", "0.0.0.0"))
    ap.add_argument("--http-port", type=int, default=int(env("CHRIS_API_PORT", "8080")), help="consolidated API port (0: none)")
    a = ap.parse_args(argv)
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(name)s %(levelname)s %(message)s")
    cfg = load_config(a.config) if a.config else None
    socks = list(ROADSENSE_SOCKETS) + (list(LEGACY_SOCKETS) if a.legacy_mapd else [])
    sources = [(d, socks if i == 0 else list(ROADSENSE_SOCKETS)) for i, d in enumerate(a.roadsense or ["/run/roadsense"])]
    d = Daemon(a.rundir, sources, cfg, a.datadir, a.config_dir, (a.host, a.http_port) if a.http_port else None,
               a.roadsense_http, mapper=None if a.no_mapper else "auto")
    asyncio.run(d.run())


if __name__ == "__main__":
    main()
