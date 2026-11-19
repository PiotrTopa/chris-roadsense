"""aiohttp application: REST endpoints, WebSocket fan-out, MJPEG preview with overlays."""
import asyncio
import json
import logging
import os
import time
import unicodedata
import weakref

from aiohttp import WSCloseCode, WSMsgType, web

from . import governor as governor_mod
from . import overlay
from .sockets import start_clients
from .state import DAEMONS, State

log = logging.getLogger("roadsense_api.server")
PREVIEW_POLL_S = 0.015
WS_QUEUE_MAX = 512
BOUNDARY = "rsframe"


def _json(data, status=200):
    return web.json_response(data, status=status, dumps=lambda o: json.dumps(o, separators=(",", ":")))


class PreviewRenderer:
    """Watches preview.jpg; while /preview clients exist, renders one overlay frame per update."""

    def __init__(self, state, path):
        self.state, self.path = state, path
        self.clients = 0
        self.frame = None              # latest rendered JPEG bytes
        self.frame_id = 0
        self.event = asyncio.Event()
        self._mtime = None

    def render(self, raw):
        st = self.state
        road, signs, cam = st.latest.get("road"), st.latest.get("signs"), st.latest.get("camera") or {}
        hud = [f"seq {cam.get('seq', '-')}  cam {st.fps('camera'):.0f} fps",
               f"road {st.fps('road'):.1f} Hz  {road.get('model_ms', 0):.0f} ms" if road else "road: no data",
               f"signs {st.fps('signs'):.1f} Hz  {len(st.live_tracks)} live" if signs else "signs: no data"]
        lights = st.latest.get("lights")
        if lights:
            ego = lights.get("ego")
            hud.append(f"lights {st.fps('lights'):.1f} Hz  " + (
                f"ego {ego['state'].upper()}{' flashing' if ego.get('flashing') else ''} #{ego['track_id']} {ego['dist_m']:.0f} m"
                if ego else "ego none"))
        ride = ride_line(cam, ascii_only=True)
        if ride:
            hud.append(ride)
        return overlay.render_jpeg(raw, road=road, signs=signs, calib=st.latest.get("calib"),
                                   intrinsics=st.intrinsics(), hud=hud, lights=lights)

    async def run(self):
        loop = asyncio.get_running_loop()
        while True:
            await asyncio.sleep(PREVIEW_POLL_S)
            if not self.clients:
                continue
            try:
                mtime = os.stat(self.path).st_mtime_ns
            except OSError:
                continue
            if mtime == self._mtime:
                continue
            self._mtime = mtime
            try:
                with open(self.path, "rb") as f:
                    raw = f.read()
                self.frame = await loop.run_in_executor(None, self.render, raw)
            except Exception as e:  # truncated/foreign file: skip this update
                log.warning("preview render failed: %s", e)
                continue
            self.frame_id += 1
            ev, self.event = self.event, asyncio.Event()
            ev.set()


def ride_line(cam, ascii_only=False):
    """'Ride: <slug> — <author>, <licence>' for a camerad-replay source (attribution wherever frames are shown).
    ascii_only: for the /preview HUD, whose bitmap font has no Polish letters or dashes."""
    if not cam.get("ride"):
        return None
    s = f"Ride: {cam['ride']} \u2014 {cam.get('author') or 'unknown author'}, {cam.get('licence') or 'licence unknown'}"
    if cam.get("playlist"):
        s += f"  ({cam.get('ride_index', 0) + 1}/{cam.get('ride_count', 1)}, " \
             f"{cam.get('ride_t_s', 0) / 60:.0f}/{cam.get('ride_duration_s', 0) / 60:.0f} min)"
    if ascii_only:
        s = _ascii(s)
    return s


def _ascii(s):
    s = unicodedata.normalize("NFKD", s.replace("\u2014", "-").replace("\u0142", "l").replace("\u0141", "L"))
    return s.encode("ascii", "ignore").decode()


def _raw(name):
    async def h(req):
        msg = req.app["state"].latest.get(name)
        return _json(msg or {"error": "no %s message yet" % name}, 200 if msg else 503)
    return h


async def h_signs(req):
    st = req.app["state"]
    msg = dict(st.latest.get("signs") or {})
    msg["live_tracks"] = st.tracks()
    return _json(msg)


async def h_moved(req):
    """The consolidated topics (tracked, fused, decided) are served by chris-integrator since api 3."""
    return _json({"error": "gone from the roadsense debug face: this topic is served by chris-integrator "
                           "(the consolidated API)", "topic": req.match_info.get("name") or req.path.rsplit("/", 1)[-1]}, 410)


async def h_version(req):
    from . import API_MAJOR, __version__
    from .state import TOPICS
    return _json({"name": "roadsense-api", "version": __version__, "api": API_MAJOR, "topics": list(TOPICS),
                  "role": "debug face: one camera's raw topics, /preview, the load governor",
                  "paths": "/v2/...; the /v1/... paths are aliases of the same handlers and will go"})


async def h_camera(req):
    st = req.app["state"]
    msg = dict(st.latest.get("camera") or {})
    ring = st.ring()
    if ring:
        ring = {k: v for k, v in ring.items() if k != "magic"}
    msg["ring"] = ring
    return _json(msg)


async def h_health(req):
    return _json(req.app["state"].health())


async def h_governor(req):
    """GET: the load governor's state (the `governor` object of health). POST {"force": N | null}: hold shed
    level N by hand (measurements, tests), null gives the governor back to its policy."""
    st = req.app["state"]
    if req.method == "POST":
        try:
            body = await req.json()
            force = body["force"]
            if force is not None and (isinstance(force, bool) or not isinstance(force, int)):
                raise ValueError
        except Exception:
            return _json({"error": 'body must be {"force": level or null}'}, 400)
        st.governor.force(force)
        governor_mod.write_caps(os.path.join(st.rundir, "governor"), st.governor.file_line())
        req.app["governor_pub"].send(st.governor_message())
    return _json(st._governor_health())


async def governor_task(app):
    """One probe reading and one policy step a period; the caps file is rewritten on every change and at least
    every 2 s (the daemons ignore a stale file)."""
    st = app["state"]
    gov = st.governor
    try:
        probe = governor_mod.make_probe(gov.cfg)
    except Exception:
        logging.getLogger("roadsense_api.governor").exception("governor probe not available, governor off")
        return
    path = os.path.join(st.rundir, "governor")
    last_line_caps, last_write = None, 0.0
    while True:
        try:
            gov.step(probe.read())
            caps = (gov.level, gov.limited)
            now = time.monotonic()
            if caps != last_line_caps or now - last_write >= 2.0:
                governor_mod.write_caps(path, gov.file_line())
                last_line_caps, last_write = caps, now
            app["governor_pub"].send(st.governor_message())
        except Exception:
            logging.getLogger("roadsense_api.governor").exception("governor step failed")
        await asyncio.sleep(float(gov.cfg["period_s"]))


async def h_frame(req):
    try:
        with open(req.app["preview_path"], "rb") as f:
            data = f.read()
    except OSError:
        return _json({"error": "no preview.jpg"}, 503)
    return web.Response(body=data, content_type="image/jpeg", headers={"Cache-Control": "no-store"})


async def h_preview(req):
    pr = req.app["renderer"]
    resp = web.StreamResponse(headers={
        "Content-Type": f"multipart/x-mixed-replace; boundary={BOUNDARY}",
        "Cache-Control": "no-store"})
    await resp.prepare(req)
    pr.clients += 1
    sent = 0
    try:
        while True:
            if pr.frame_id == sent:
                await pr.event.wait()
            sent, frame = pr.frame_id, pr.frame
            await resp.write(f"--{BOUNDARY}\r\nContent-Type: image/jpeg\r\nContent-Length: {len(frame)}\r\n\r\n"
                             .encode() + frame + b"\r\n")
    except (ConnectionResetError, asyncio.CancelledError):
        pass
    finally:
        pr.clients -= 1
    return resp


async def h_ws(req):
    st = req.app["state"]
    topics = {t for t in req.query.get("topics", "").split(",") if t}
    ws = web.WebSocketResponse(heartbeat=20)
    await ws.prepare(req)
    req.app["websockets"].add(ws)          # closed on shutdown, so a restart does not wait for clients
    q = asyncio.Queue(maxsize=WS_QUEUE_MAX)

    def push(mtype, line):
        if topics and mtype not in topics:
            return
        try:
            q.put_nowait(line)
        except asyncio.QueueFull:          # slow client: disconnect, like the daemons do
            st.subscribers.discard(push)
            asyncio.ensure_future(ws.close(message=b"slow consumer"))

    async def reader():
        async for m in ws:
            if m.type == WSMsgType.ERROR:
                break

    st.subscribers.add(push)
    rt = asyncio.create_task(reader())
    try:
        while not ws.closed:
            get = asyncio.create_task(q.get())
            done, _ = await asyncio.wait({get, rt}, return_when=asyncio.FIRST_COMPLETED)
            if get not in done:
                get.cancel()
                break
            await ws.send_str(get.result())
    except (ConnectionResetError, RuntimeError):
        pass
    finally:
        st.subscribers.discard(push)
        rt.cancel()
        await ws.close()
    return ws


async def h_index(req):
    return web.Response(text=INDEX_HTML, content_type="text/html")


async def h_options(req):
    return web.Response(status=204, headers={
        "Access-Control-Allow-Methods": "GET, POST, OPTIONS",
        "Access-Control-Allow-Headers": "*"})


async def _cors(req, resp):
    resp.headers["Access-Control-Allow-Origin"] = "*"


class LinePublisher:
    """<rundir>/api.sock: the debug face's own output for other processes (the `governor` message), JSON lines;
    a client gets the last line when it connects."""

    def __init__(self, path):
        self.path, self.clients, self.last, self.server = path, set(), None, None

    def send(self, msg):
        self.last = (json.dumps(msg, separators=(",", ":")) + "\n").encode()
        for w in list(self.clients):
            if w.transport is None or w.transport.is_closing() or w.transport.get_write_buffer_size() > 1 << 20:
                self.clients.discard(w)
                continue
            w.write(self.last)

    async def _client(self, reader, writer):
        self.clients.add(writer)
        if self.last:
            writer.write(self.last)
        try:
            await reader.read()
        except (OSError, asyncio.CancelledError):
            pass
        finally:
            self.clients.discard(writer)
            writer.close()

    async def start(self):
        try:
            os.unlink(self.path)
        except OSError:
            pass
        try:
            self.server = await asyncio.start_unix_server(self._client, path=self.path)
            os.chmod(self.path, 0o666)
        except OSError as e:
            log.warning("no %s: %s", self.path, e)

    def close(self):
        if self.server is not None:
            self.server.close()


MOVED = ("speed_limit", "rules", "map", "fused", "lead", "ego_state", "tracks", "lanes", "view", "vehicle", "signs/history")


def make_app(rundir, datadir=None, ring_path=None, connect=True, rules=None, record=None, governor=None):
    """The debug face of one roadsense instance. (`datadir`, `rules`, `record` are accepted and unused: the
    consolidated side they configured is chris-integrator.)"""
    state = State(rundir, ring_path=ring_path)
    if governor is None:          # the governor runs with the daemons; an app without them (tests) has none
        governor = connect
    app = web.Application()
    app["state"] = state
    app["preview_path"] = os.path.join(rundir, "preview.jpg")
    app["governor_pub"] = LinePublisher(os.path.join(rundir, "api.sock"))
    app["websockets"] = weakref.WeakSet()
    app.on_response_prepare.append(_cors)

    async def startup(app):
        app["renderer"] = PreviewRenderer(state, app["preview_path"])
        app["tasks"] = [asyncio.create_task(app["renderer"].run())]
        if governor or connect:
            await app["governor_pub"].start()
            app["governor_pub"].send(state.governor_message())
        if governor:
            app["tasks"].append(asyncio.create_task(governor_task(app)))
        if connect:
            app["tasks"] += start_clients(state, rundir, DAEMONS)

    async def cleanup(app):
        for t in app["tasks"]:
            t.cancel()
        await asyncio.gather(*app["tasks"], return_exceptions=True)
        app["governor_pub"].close()

    async def shutdown(app):      # wake /preview handlers and close /ws clients so they finish (else systemd waits 90 s)
        app["renderer"].event.set()
        for ws in set(app["websockets"]):
            await ws.close(code=WSCloseCode.GOING_AWAY, message=b"api shutdown")

    app.on_startup.append(startup)
    app.on_shutdown.append(shutdown)
    app.on_cleanup.append(cleanup)
    app.router.add_get("/", h_index)
    routes = [("GET", "road", _raw("road")), ("GET", "signs", h_signs), ("GET", "env", _raw("env")),
              ("GET", "lights", _raw("lights")), ("GET", "calib", _raw("calib")), ("GET", "camera", h_camera),
              ("GET", "health", h_health), ("GET", "governor", h_governor), ("POST", "governor", h_governor),
              ("GET", "frame.jpg", h_frame), ("GET", "version", h_version)]
    routes += [("GET", name, h_moved) for name in MOVED]
    for method, name, handler in routes:
        for prefix in ("/v2/", "/v1/"):
            app.router.add_route(method, prefix + name, handler)
    app.router.add_get("/preview", h_preview)
    app.router.add_get("/ws", h_ws)
    app.router.add_route("OPTIONS", "/{tail:.*}", h_options)
    return app


INDEX_HTML = """<!doctype html>
<html><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>roadsense (debug face)</title>
<style>
body{margin:0;background:#111;color:#ddd;font:14px/1.4 monospace}
main{display:flex;flex-wrap:wrap;gap:12px;padding:12px}
img{max-width:100%;width:960px;background:#000}
section{min-width:280px;flex:1}
h2{font-size:14px;color:#8cf;margin:8px 0 4px}
pre{margin:0;white-space:pre-wrap}
.bad{color:#f66}.ok{color:#6f6}
</style></head><body><main>
<img src="/preview" alt="preview">
<section><p>One camera's raw perception. Tracked and fused topics, the map and the consumer API: chris-integrator.</p>
<h2>health</h2><pre id="health">...</pre>
<h2>road (raw)</h2><pre id="road">...</pre>
<h2>signs (detector tracks)</h2><pre id="signs">...</pre>
<h2>traffic lights (raw)</h2><pre id="lights">...</pre>
<h2>governor</h2><pre id="gov">...</pre></section>
</main><script>
const $ = id => document.getElementById(id);
const f = (v, d=1) => v == null ? "-" : Number(v).toFixed(d);
async function get(u){ const r = await fetch(u, {cache:"no-store"}); return r.ok ? r.json() : null; }
async function tick(){
  try {
    const h = await get("/v2/health");
    if (h) {
      let s = "";
      for (const [d, v] of Object.entries(h.daemons))
        s += `<span class="${v.connected ? "ok" : "bad"}">${d.padEnd(8)}</span> age ${f(v.last_msg_age_s, 2)} s\\n`;
      s += "fps " + Object.entries(h.fps).map(([k, v]) => k + " " + f(v)).join("  ") + "\\n";
      s += `model ${f(h.model_ms, 0)} ms  detect ${f(h.detect_ms, 0)} ms  env ${f(h.env_ms, 0)} ms\\n`;
      s += `cpu ${f(h.cpu_temp_c)} C  ` + h.battery.map(b => `${b.name} ${b.capacity}% ${b.status || ""}`).join("  ");
      $("health").innerHTML = s;
      $("gov").textContent = `level ${h.governor.level}/${h.governor.levels}  shed ${h.governor.shed.join(", ") || "-"}  limited ${h.governor.limited}`;
    }
    const r = await get("/v2/road");
    if (r) {
      const names = ["far left", "left", "right", "far right"];
      let s = (r.lane_lines || []).map((l, i) => `${names[i] || i}: p ${f(l.prob, 2)}`).join("\\n");
      const lead = (r.leads || [])[0];
      s += "\\nlead (raw): " + (lead ? `${f(lead.x[0])} m  p ${f(lead.prob, 2)}  v ${f(lead.v[0])} m/s` : "-");
      s += `\\nseq ${r.seq}  model ${f(r.model_ms, 0)} ms`;
      $("road").textContent = s;
    } else $("road").textContent = "no data";
    const sg = await get("/v2/signs");
    $("signs").textContent = sg && sg.live_tracks.length
      ? sg.live_tracks.map(t => `#${t.track_id} ${t.cls} ${f(t.score, 2)} hits ${t.hits}`).join("\\n") : "none";
    const lt = await get("/v2/lights");
    $("lights").textContent = lt ? (lt.lights || []).map(l => `#${l.track_id} ${l.state} ${f(l.state_conf, 2)}  ${f(l.dist_m, 0)} m`).join("\\n") || "none" : "no data";
  } catch (e) { $("health").textContent = "api unreachable"; }
}
tick(); setInterval(tick, 1000);
</script></body></html>
"""
