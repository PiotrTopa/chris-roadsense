"""aiohttp application: REST endpoints, WebSocket fan-out, MJPEG preview with overlays."""
import asyncio
import json
import logging
import os
import time

from aiohttp import WSMsgType, web

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
        return overlay.render_jpeg(raw, road=road, signs=signs, calib=st.latest.get("calib"),
                                   intrinsics=st.intrinsics(), hud=hud)

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


async def h_road(req):
    return _json(req.app["state"].latest.get("road") or {"error": "no road message yet"},
                 200 if "road" in req.app["state"].latest else 503)


async def h_signs(req):
    st = req.app["state"]
    msg = dict(st.latest.get("signs") or {})
    msg["live_tracks"] = st.tracks()
    return _json(msg)


async def h_signs_history(req):
    try:
        limit = max(0, int(req.query.get("limit", 100)))
    except ValueError:
        return _json({"error": "limit must be an integer"}, 400)
    hist = list(req.app["state"].history)[::-1][:limit]
    return _json({"count": len(hist), "tracks": hist})


async def h_env(req):
    st = req.app["state"]
    msg = st.latest.get("env")
    return _json(msg or {"error": "no env message yet"}, 200 if msg else 503)


async def h_speed_limit(req):
    st = req.app["state"]
    if "speed_limit" not in st.latest:
        return _json({"error": "no road/signs message yet"}, 503)
    return _json(st.speed.snapshot(full=True))


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
        "Access-Control-Allow-Methods": "GET, OPTIONS",
        "Access-Control-Allow-Headers": "*"})


async def _cors(req, resp):
    resp.headers["Access-Control-Allow-Origin"] = "*"


def make_app(rundir, datadir=None, ring_path=None, connect=True, rules="pl"):
    state = State(rundir, datadir=datadir, ring_path=ring_path, rules=rules)
    app = web.Application()
    app["state"] = state
    app["preview_path"] = os.path.join(rundir, "preview.jpg")
    app.on_response_prepare.append(_cors)

    async def startup(app):
        app["renderer"] = PreviewRenderer(state, app["preview_path"])
        app["tasks"] = [asyncio.create_task(app["renderer"].run())]
        if connect:
            app["tasks"] += start_clients(state, rundir, DAEMONS)

    async def cleanup(app):
        for t in app["tasks"]:
            t.cancel()
        await asyncio.gather(*app["tasks"], return_exceptions=True)

    async def shutdown(app):      # wake /preview handlers so they finish
        app["renderer"].event.set()

    app.on_startup.append(startup)
    app.on_shutdown.append(shutdown)
    app.on_cleanup.append(cleanup)
    app.router.add_get("/", h_index)
    app.router.add_get("/v1/road", h_road)
    app.router.add_get("/v1/signs", h_signs)
    app.router.add_get("/v1/signs/history", h_signs_history)
    app.router.add_get("/v1/env", h_env)
    app.router.add_get("/v1/speed_limit", h_speed_limit)
    app.router.add_get("/v1/camera", h_camera)
    app.router.add_get("/v1/health", h_health)
    app.router.add_get("/v1/frame.jpg", h_frame)
    app.router.add_get("/preview", h_preview)
    app.router.add_get("/ws", h_ws)
    app.router.add_route("OPTIONS", "/{tail:.*}", h_options)
    return app


INDEX_HTML = """<!doctype html>
<html><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>roadsense</title>
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
<section><h2>health</h2><pre id="health">...</pre>
<h2>road</h2><pre id="road">...</pre>
<h2>signs (live)</h2><pre id="signs">...</pre>
<h2>environment</h2><pre id="env">...</pre>
<h2>speed limit</h2><pre id="speed">...</pre></section>
</main><script>
const $ = id => document.getElementById(id);
const f = (v, d=1) => v == null ? "-" : Number(v).toFixed(d);
async function get(u){ const r = await fetch(u, {cache:"no-store"}); return r.ok ? r.json() : null; }
async function tick(){
  try {
    const h = await get("/v1/health");
    if (h) {
      let s = "";
      for (const [d, v] of Object.entries(h.daemons))
        s += `<span class="${v.connected ? "ok" : "bad"}">${d.padEnd(8)}</span> age ${f(v.last_msg_age_s, 2)} s\\n`;
      s += "fps " + Object.entries(h.fps).map(([k, v]) => k + " " + f(v)).join("  ") + "\\n";
      s += `model ${f(h.model_ms, 0)} ms  detect ${f(h.detect_ms, 0)} ms  env ${f(h.env_ms, 0)} ms\\n`;
      s += `cpu ${f(h.cpu_temp_c)} C  ` + h.battery.map(b => `${b.name} ${b.capacity}% ${b.status || ""}`).join("  ");
      $("health").innerHTML = s;
    }
    const r = await get("/v1/road");
    if (r) {
      const names = ["far left", "left", "right", "far right"];
      let s = (r.lane_lines || []).map((l, i) => `${names[i] || i}: p ${f(l.prob, 2)}`).join("\\n");
      const lead = (r.leads || [])[0];
      s += "\\nlead: " + (lead ? `${f(lead.x[0])} m  p ${f(lead.prob, 2)}  v ${f(lead.v[0])} m/s` : "-");
      s += `\\nseq ${r.seq}  model ${f(r.model_ms, 0)} ms`;
      $("road").textContent = s;
    } else $("road").textContent = "no data";
    const sg = await get("/v1/signs");
    $("signs").textContent = sg && sg.live_tracks.length
      ? sg.live_tracks.map(t => `#${t.track_id} ${t.cls} ${f(t.score, 2)} hits ${t.hits}`).join("\\n") : "none";
    const ev = await get("/v1/env");
    $("env").textContent = ev ? ["place", "weather", "light"].map(g => {
        const r = ev[g] || {};
        const top = Object.entries(r.scores || {}).sort((a, b) => b[1] - a[1]).slice(0, 3)
          .map(([k, v]) => `${k} ${f(v, 2)}`).join(", ");
        return `${g.padEnd(8)} ${(r.label || "-").padEnd(20)} conf ${f(r.confidence, 2)}  stable ${f(r.stable_s, 0)} s  [${top}]`;
      }).join("\\n") + `\\nluma ${f(ev.stats && ev.stats.luma, 0)}  ${ev.model || ""}  ${f(ev.env_ms, 0)} ms` : "no data";
    const sl = await get("/v1/speed_limit");
    $("speed").textContent = sl ? [
        `limit ${sl.limit_kmh == null ? "unknown" : sl.limit_kmh + " km/h"}  conf ${f(sl.confidence, 2)}  ` +
          (sl.source.kind === "sign" ? `${sl.source.sign} #${sl.source.track_id}` : `${sl.source.kind} ${sl.source.rule || ""}`),
        `since ${f(sl.since.age_s, 0)} s / ${f(sl.since.distance_m, 0)} m  area ${sl.area.type} (${sl.area.source})  road ${sl.road_class}  zone ${sl.zone.type}`,
        sl.expiry ? `expires: ${sl.expiry.rule}, cap in ${f(sl.expiry.remaining_m, 0)} m` : "",
        `v ${f(sl.v_ego_kmh, 0)} km/h  odo ${f(sl.odometer_m / 1000, 2)} km` + (sl.advisory_kmh ? `  advisory ${sl.advisory_kmh}` : ""),
        ...sl.pending.map(p => `pending #${p.track_id} ${p.sign} ${p.value_kmh || ""} hits ${p.hits} q ${f(p.quality, 2)}`),
        ...(sl.recent_changes || []).slice(0, 3).map(c => `  ${f(c.odo_m, 0)} m: ${c.event}`)
      ].filter(Boolean).join("\n") : "no data";
  } catch (e) { $("health").textContent = "api unreachable"; }
}
tick(); setInterval(tick, 1000);
</script></body></html>
"""
