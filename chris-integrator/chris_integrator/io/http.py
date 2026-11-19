"""The consolidated API: an output channel (HTTP + WebSocket) over the bus, and one input (vehicle data by POST).

    GET  /v2/<topic>            the last message of a public stream (`road`, `lead`, `lanes`, `view`, `fused`,
                                `rules`, `speed_limit`, `lights`, `signs`, `map`, `ego_state`, `tracks`, `env`,
                                `vehicle`, `camera`, ...); /v1/... are aliases of the same handlers
    GET  /v2/health             the stack's health in the shape consumers know, plus `mixer` (the wiring)
    GET  /v2/mixer              channels, streams, plug-ins in running order with health and cost
    POST /v2/vehicle            vehicle input API over HTTP (one message, an array, or JSON lines)
    GET  /ws?topics=a,b         every message of the public streams as it is put on the bus
    GET  /preview, /v2/frame.jpg, /v2/governor   forwarded to a chris-roadsense instance's debug face when one
                                is configured (they are that camera's own; the integrator draws nothing)

Message shapes, paths and topic names are those the roadsense api served before the consolidated side moved
here (api 2); additions only. Needs aiohttp (`pip install chris-integrator[http]`)."""
import asyncio
import json
import logging
import weakref

from aiohttp import ClientSession, ClientTimeout, WSCloseCode, WSMsgType, web

from .. import API_MAJOR, __version__
from .api import Api

log = logging.getLogger("chris_integrator.http")
WS_QUEUE_MAX = 512


def _json(data, status=200):
    return web.json_response(data, status=status, dumps=lambda o: json.dumps(o, separators=(",", ":")))


def _topic(name, missing="no message yet"):
    async def h(req):
        msg = req.app["api"].latest(name)
        return _json(msg or {"error": missing}, 200 if msg else 503)
    return h


async def h_signs(req):
    api = req.app["api"]
    msg = dict(api.latest("signs") or {})
    sc = api.mixer.plugin("sign_context")
    msg["live_tracks"] = sc.tracks() if sc is not None else []
    msg["suppressed_tracks"] = sc.tracks(suppressed=True) if sc is not None else []
    return _json(msg)


async def h_signs_history(req):
    try:
        limit = max(0, int(req.query.get("limit", 100)))
    except ValueError:
        return _json({"error": "limit must be an integer"}, 400)
    hist = list(req.app["api"].history)[::-1][:limit]
    return _json({"count": len(hist), "tracks": hist})


async def h_speed_limit(req):
    api = req.app["api"]
    r = api.mixer.plugin("rules")
    if r is None or not api.latest("speed_limit"):
        return _json({"error": "no road/signs message yet"}, 503)
    return _json(r.speed.snapshot(full=True))


async def h_rules(req):
    api = req.app["api"]
    r = api.mixer.plugin("rules")
    if r is None or not api.latest("rules"):
        return _json({"error": "no road/signs message yet"}, 503)
    return _json(r.engine.snapshot(full=True))


async def h_roadgeom(req):
    """The topic `roadgeom` of api 1 is the `road` object of `view` since 2.0."""
    return _json({"error": "gone in api 2: the fused road geometry is view.road (GET /v2/view, /ws?topics=view)",
                  "see": "/v2/view"}, 410)


async def h_version(req):
    api = req.app["api"]
    return _json({"name": "chris-integrator", "version": __version__, "api": API_MAJOR,
                  "topics": [n for n, s in api.bus.streams.items() if s.public],
                  "paths": "/v2/...; the /v1/... paths are aliases of the same handlers and will go"})


async def h_vehicle(req):
    """The car as the host describes it: the `vehicle` message plus the last scan of every range sensor."""
    api = req.app["api"]
    msg = api.latest("vehicle")
    if not msg:
        return _json({"error": "no vehicle message yet (nothing feeds the car's data)"}, 503)
    return _json(dict(msg, objects=list(api.objects.values())))


async def h_position(req):
    """Vehicle input over HTTP (`POST /v2/vehicle`, and under its first name `POST /v2/position`): the body is
    one message, a JSON array of messages, or JSON lines (the vehicle input API)."""
    api = req.app["api"]
    try:
        text = (await req.text()).strip()
    except Exception:
        return _json({"error": "unreadable body"}, 400)
    try:
        if text.startswith("["):
            msgs = json.loads(text)
        else:
            msgs = [json.loads(ln) for ln in text.splitlines() if ln.strip()]
    except ValueError:
        return _json({"error": "body must be a JSON object, a JSON array or JSON lines"}, 400)
    if not msgs or not all(isinstance(m, dict) for m in msgs):
        return _json({"error": "no messages"}, 400)
    if api.feed is None:
        return _json({"error": "this integrator takes no vehicle feed"}, 503)
    return _json(dict(api.feed(msgs), api=1))


async def h_camera(req):
    api = req.app["api"]
    up = await _forward_json(req, "/v2/camera")        # the camera's own face knows its frame ring
    if up is not None:
        return _json(up)
    msg = dict(api.latest("camera") or {})
    msg["ring"] = None
    return _json(msg)


async def h_health(req):
    return _json(req.app["api"].health())


async def h_mixer(req):
    return _json(req.app["api"].mixer.describe())


# ---- what belongs to one camera is forwarded to that camera's debug face -------------------------------
async def _forward_json(req, path):
    base = req.app["roadsense_http"]
    if not base:
        return None
    try:
        async with req.app["client"].get(base + path, timeout=ClientTimeout(total=2.0)) as r:
            return await r.json() if r.status == 200 else None
    except Exception:
        return None


async def h_governor(req):
    """The load governor belongs to the roadsense instance: GET / POST are forwarded to its debug face; without
    one, GET answers from the `governor` input."""
    app = req.app
    base = app["roadsense_http"]
    if base:
        try:
            body = await req.read() if req.method == "POST" else None
            async with app["client"].request(req.method, base + "/v2/governor", data=body,
                                             timeout=ClientTimeout(total=2.0)) as r:
                return web.Response(body=await r.read(), status=r.status, content_type="application/json")
        except Exception:
            pass
    if req.method == "POST":
        return _json({"error": "no chris-roadsense debug face configured (--roadsense-http)"}, 503)
    return _json(app["api"].governor_health())


async def h_frame(req):
    base = req.app["roadsense_http"]
    if not base:
        return _json({"error": "no chris-roadsense debug face configured (--roadsense-http)"}, 503)
    try:
        async with req.app["client"].get(base + "/v2/frame.jpg", timeout=ClientTimeout(total=3.0)) as r:
            if r.status != 200:
                return _json({"error": "no preview.jpg"}, 503)
            return web.Response(body=await r.read(), content_type="image/jpeg", headers={"Cache-Control": "no-store"})
    except Exception:
        return _json({"error": "chris-roadsense debug face unreachable"}, 503)


async def h_preview(req):
    base = req.app["roadsense_http"]
    if not base:
        return _json({"error": "no chris-roadsense debug face configured (--roadsense-http)"}, 503)
    try:
        up = await req.app["client"].get(base + "/preview", timeout=ClientTimeout(total=None, sock_connect=2.0))
    except Exception:
        return _json({"error": "chris-roadsense debug face unreachable"}, 503)
    resp = web.StreamResponse(headers={"Content-Type": up.headers.get("Content-Type", "multipart/x-mixed-replace"),
                                       "Cache-Control": "no-store"})
    await resp.prepare(req)
    try:
        async for chunk in up.content.iter_any():
            await resp.write(chunk)
    except (ConnectionResetError, asyncio.CancelledError, Exception):
        pass
    finally:
        up.close()
    return resp


async def h_ws(req):
    api = req.app["api"]
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
            api.subscribers.discard(push)
            asyncio.ensure_future(ws.close(message=b"slow consumer"))

    async def reader():
        async for m in ws:
            if m.type == WSMsgType.ERROR:
                break

    api.subscribers.add(push)
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
        api.subscribers.discard(push)
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


def make_app(stack, readers=None, datadir=None, feed=None, recorder=None, roadsense_http=None):
    """The aiohttp application over a stack. `roadsense_http`: base URL of a chris-roadsense debug face
    (`http://127.0.0.1:8081`) that `/preview`, `/v2/frame.jpg`, `/v2/camera` and `/v2/governor` are forwarded to."""
    app = web.Application()
    app["api"] = Api(stack, readers, datadir, feed, recorder)
    app["roadsense_http"] = roadsense_http.rstrip("/") if roadsense_http else None
    app["websockets"] = weakref.WeakSet()
    app.on_response_prepare.append(_cors)

    async def startup(app):
        app["client"] = ClientSession()

    async def cleanup(app):
        await app["client"].close()

    async def shutdown(app):      # close /ws clients so they finish (else a service manager waits for them)
        for ws in set(app["websockets"]):
            await ws.close(code=WSCloseCode.GOING_AWAY, message=b"api shutdown")

    app.on_startup.append(startup)
    app.on_shutdown.append(shutdown)
    app.on_cleanup.append(cleanup)
    app.router.add_get("/", h_index)
    # api 2: every resource under /v2; /v1 is an alias of the same handlers and will go
    for method, name, handler in (
            ("GET", "road", _topic("road", "no road message yet")),
            ("GET", "signs", h_signs),
            ("GET", "signs/history", h_signs_history),
            ("GET", "env", _topic("env", "no env message yet")),
            ("GET", "speed_limit", h_speed_limit),
            ("GET", "rules", h_rules),
            ("GET", "lights", _topic("lights", "no lights message yet")),
            ("GET", "map", _topic("map", "no map message yet (no map box, or no pack)")),
            ("GET", "fused", _topic("fused", "no fused message yet")),
            ("GET", "lead", _topic("lead", "no road message yet")),
            ("GET", "ego_state", _topic("ego_state", "no input yet")),
            ("GET", "tracks", _topic("tracks", "no input yet")),
            ("GET", "lanes", _topic("lanes", "no road message yet")),
            ("GET", "view", _topic("view", "no road message yet")),
            ("GET", "calib", _topic("calib", "no calib message yet")),
            ("GET", "gnss", _topic("gnss", "no gnss message yet")),
            ("GET", "roadgeom", h_roadgeom),
            ("POST", "position", h_position),
            ("GET", "vehicle", h_vehicle),
            ("POST", "vehicle", h_position),
            ("GET", "camera", h_camera),
            ("GET", "health", h_health),
            ("GET", "mixer", h_mixer),
            ("GET", "governor", h_governor),
            ("POST", "governor", h_governor),
            ("GET", "frame.jpg", h_frame),
            ("GET", "version", h_version),
    ):
        for prefix in ("/v2/", "/v1/"):
            app.router.add_route(method, prefix + name, handler)
    app.router.add_get("/preview", h_preview)
    app.router.add_get("/ws", h_ws)
    app.router.add_route("OPTIONS", "/{tail:.*}", h_options)
    return app


INDEX_HTML = """<!doctype html>
<html><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>chris</title>
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
<section><pre id="ride"></pre><h2>health</h2><pre id="health">...</pre>
<h2>road</h2><pre id="road">...</pre>
<h2>signs (live)</h2><pre id="signs">...</pre>
<h2>environment</h2><pre id="env">...</pre>
<h2>rules in effect</h2><pre id="rules">...</pre>
<h2>speed limit</h2><pre id="speed">...</pre>
<h2>traffic lights</h2><pre id="lights">...</pre>
<h2>map</h2><pre id="map">...</pre>
<h2>fused (map + camera)</h2><pre id="fused">...</pre>
<h2>vehicle (from the host)</h2><pre id="vehicle">...</pre></section>
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
    }
    const cam = await get("/v2/camera");
    $("ride").textContent = cam && cam.ride ? `Ride: ${cam.ride} \u2014 ${cam.author || "unknown author"}, ` +
      `${cam.licence || "licence unknown"}` + (cam.playlist ? `  (${cam.ride_index + 1}/${cam.ride_count}, ` +
      `${f(cam.ride_t_s / 60, 0)}/${f(cam.ride_duration_s / 60, 0)} min)` : "") : "";
    const r = await get("/v2/road");
    if (r) {
      const names = ["far left", "left", "right", "far right"];
      let s = (r.lane_lines || []).map((l, i) => `${names[i] || i}: p ${f(l.prob, 2)}`).join("\\n");
      const tl = await get("/v2/lanes");
      if (tl) s += "\\ntracked: lines " + tl.lines.map(l => l.present ? (l.mode === "coast" ? "c" : "|") : ".").join("") +
        "  edges " + tl.edges.map(l => l.present ? "|" : ".").join("") + `  width ${f(tl.lane_width_m)} m  (${tl.source})`;
      const lead = (r.leads || [])[0];
      s += "\\nlead (raw): " + (lead ? `${f(lead.x[0])} m  p ${f(lead.prob, 2)}  v ${f(lead.v[0])} m/s` : "-");
      const ld = await get("/v2/lead");
      if (ld && ld.valid === false) s += "\\nlead: UNKNOWN (" + ld.reason + ")";
      const es = await get("/v2/ego_state");
      if (es) s += "\\nego: " + (es.speed.valid ? `${f(es.speed.value)} +- ${f(es.speed.std, 2)} m/s (${es.speed.sources.join(", ")}; ${es.speed.mode})` : "speed unknown: " + es.speed.reason) +
        "  sources: " + Object.entries(es.sources).map(([k, v]) => k + " " + v.state).join(", ");
      s += "\\nlead (tracked): " + (ld && ld.present ? `${f(ld.dist_m)} +- ${f(ld.dist_std_m)} m  v_rel ${f(ld.v_rel_mps)} m/s  ` +
        `gap ${f(ld.time_gap_s)} s  ${ld.mode}${ld.departed ? " DEPARTED" : ""}  ego ${f(ld.ego.speed_mps)} m/s (${ld.ego.source})` : "-");
      s += `\\nseq ${r.seq}  model ${f(r.model_ms, 0)} ms`;
      $("road").textContent = s;
    } else $("road").textContent = "no data";
    const sg = await get("/v2/signs");
    $("signs").textContent = sg && sg.live_tracks.length
      ? sg.live_tracks.map(t => `#${t.track_id} ${t.cls} ${f(t.score, 2)} hits ${t.hits}`).join("\\n") : "none";
    const ev = await get("/v2/env");
    $("env").textContent = ev ? ["place", "weather", "light"].map(g => {
        const r = ev[g] || {};
        const top = Object.entries(r.scores || {}).sort((a, b) => b[1] - a[1]).slice(0, 3)
          .map(([k, v]) => `${k} ${f(v, 2)}`).join(", ");
        return `${g.padEnd(8)} ${(r.label || "-").padEnd(20)} conf ${f(r.confidence, 2)}  stable ${f(r.stable_s, 0)} s  [${top}]`;
      }).join("\\n") + `\\nluma ${f(ev.stats && ev.stats.luma, 0)}  ${ev.model || ""}  ${f(ev.env_ms, 0)} ms` : "no data";
    const sl = await get("/v2/speed_limit");
    $("speed").textContent = sl ? [
        `limit ${sl.limit_kmh == null ? "unknown" : sl.limit_kmh + " km/h"}  conf ${f(sl.confidence, 2)}  ` +
          (sl.source.kind === "sign" ? `${sl.source.sign} #${sl.source.track_id}` : `${sl.source.kind} ${sl.source.rule || ""}`),
        `since ${f(sl.since.age_s, 0)} s / ${f(sl.since.distance_m, 0)} m  area ${sl.area.type} (${sl.area.source})  road ${sl.road_class}  zone ${sl.zone.type}`,
        sl.expiry ? `expires: ${sl.expiry.rule}, cap in ${f(sl.expiry.remaining_m, 0)} m` : "",
        `v ${f(sl.v_ego_kmh, 0)} km/h  odo ${f(sl.odometer_m / 1000, 2)} km` + (sl.advisory_kmh ? `  advisory ${sl.advisory_kmh}` : ""),
        ...sl.pending.map(p => `pending #${p.track_id} ${p.sign} ${p.value_kmh || ""} hits ${p.hits} q ${f(p.quality, 2)}`),
        ...(sl.recent_changes || []).slice(0, 3).map(c => `  ${f(c.odo_m, 0)} m: ${c.event}`)
      ].filter(Boolean).join("\\n") : "no data";
    const ru = await get("/v2/rules");
    $("rules").textContent = ru ? [
        ...(ru.in_effect.length ? ru.in_effect.map(e => `${e.sign.padEnd(6)} ${(e.name + (e.value != null ? " " + e.value + " " + (e.unit || "") : "")).padEnd(34)} ` +
          `${e.side ? "(" + e.side + ") " : ""}since ${f(e.since.distance_m, 0)} m / ${f(e.since.age_s, 0)} s  conf ${f(e.confidence, 2)}  ` +
          `${e.source.kind === "sign" ? "#" + e.source.track_id : e.source.kind}${e.uncertain && e.uncertain.length ? "  ? " + e.uncertain.join(", ") : ""}` +
          `${e.ends ? "\\n       until: " + e.ends + (e.remaining_m != null ? " (cap in " + f(e.remaining_m, 0) + " m)" : "") : ""}`) : ["nothing in effect"]),
        "recent: " + (ru.recent.length ? ru.recent.map(e => `${e.sign} ${e.name} ${f(e.passed.age_s, 0)} s ago` + (e.applies_to_car ? "" : " (not this car)") +
          (e.uncertain && e.uncertain.length ? " ?" : "")).join("; ") : "-"),
        "ahead:  " + (ru.ahead.length ? ru.ahead.map(w => `${w.sign} ${w.name} ~${f(w.expected_in_m, 0)} m (${f(w.range_m[0], 0)}-${f(w.range_m[1], 0)})`).join("; ") : "-"),
        `junction: ${ru.junction.last ? ru.junction.last.source + " " + f(ru.junction.last.age_m, 0) + " m ago" : "none seen"}  ` +
          `area ${ru.area.type} (${ru.area.source})  road ${ru.road_class}  zone ${ru.zone.type}`,
        ...(ru.pending || []).map(p => `pending #${p.track_id} ${p.sign} ${p.id} hits ${p.hits} q ${f(p.quality, 2)}`),
        ...(ru.recent_changes || []).slice(0, 4).map(c => `  ${f(c.odo_m, 0)} m: ${c.event}`)
      ].join("\\n") : "no data";
    const lt = await get("/v2/lights");
    $("lights").textContent = lt ? [
        lt.ego ? `ego ${lt.ego.state}${lt.ego.flashing ? " (flashing)" : ""}  #${lt.ego.track_id}  ${f(lt.ego.dist_m, 0)} m  ` +
          `lat ${f(lt.ego.lateral_m, 1)} m  rel ${f(lt.ego.relevance, 2)}  agree ${lt.ego.agree}/${lt.ego.n_relevant}  (${lt.ego.source})` : "ego: none",
        ...(lt.lights || []).map(l => `#${l.track_id} ${l.state.padEnd(10)} ${f(l.state_conf, 2)}  ${f(l.dist_m, 0)} m  ` +
          `lat ${f(l.lateral_m, 1)}  rel ${f(l.relevance, 2)}  hits ${l.hits}${l.ego ? "  EGO" : ""}`),
        `${f(lt.lights_ms, 0)} ms`
      ].join("\\n") : "no data";
    const mp = await get("/v2/map");
    $("map").textContent = mp ? [
        `position ${mp.state}` + (mp.position ? `  ${f(mp.position.lat, 5)} ${f(mp.position.lon, 5)}  +-${f(mp.position.hacc_m, 0)} m  ` +
          `hdg ${f(mp.position.heading_deg, 0)}  ${f(mp.position.speed_mps * 3.6, 0)} km/h  motion ${mp.position.motion_source}` : ""),
        mp.available ? `road ${mp.road.class} ${mp.road.ref || ""} ${mp.road.name || ""}  match ${f(mp.match.confidence, 2)} (${f(mp.match.distance_m, 0)} m)` +
          `  lanes ${mp.road.lanes.forward}+${mp.road.lanes.backward} (${mp.road.lanes.source})` : `map unavailable: ${mp.reason}`,
        mp.available ? `limit ${mp.speed_limit.limit_kmh == null ? "-" : mp.speed_limit.limit_kmh} (${mp.speed_limit.source} ${mp.speed_limit.rule || mp.speed_limit.detail || ""})` +
          `  built-up ${mp.road.built_up.value} (${mp.road.built_up.source})` +
          (mp.ahead.tunnel.in_tunnel ? `  TUNNEL exit ${f(mp.ahead.tunnel.exit_m, 0)} m` : mp.ahead.tunnel.entry_m != null ? `  tunnel in ${f(mp.ahead.tunnel.entry_m, 0)} m` : "") : "",
        mp.available ? "ahead: " + (mp.ahead.features.slice(0, 5).map(x => `${x.kind} ${f(x.dist_m, 0)} m`).join(", ") || "-") +
          (mp.ahead.curves.length ? `  curve ${mp.ahead.curves[0].direction} R ${f(mp.ahead.curves[0].min_radius_m, 0)} m in ${f(mp.ahead.curves[0].start_m, 0)} m` : "") : "",
        mp.pack ? `pack ${mp.pack.region} OSM ${mp.pack.osm_timestamp}  (c) OpenStreetMap contributors` : "no pack"
      ].filter(Boolean).join("\\n") : "no map box";
    const fu = await get("/v2/fused");
    $("fused").textContent = fu ? [
        `limit ${fu.speed_limit.limit_kmh == null ? "unknown" : fu.speed_limit.limit_kmh + " km/h"}  ${fu.speed_limit.source}  conf ${f(fu.speed_limit.confidence, 2)}  (${fu.speed_limit.agreement})` +
          (fu.speed_limit.note ? `  ${fu.speed_limit.note}` : ""),
        `place ${fu.place.label || "-"}  ${fu.place.source}  conf ${f(fu.place.confidence, 2)}   built-up ${fu.built_up.value} (${fu.built_up.source})`,
        `lanes ${fu.lanes.count == null ? "-" : fu.lanes.count}  ego ${fu.lanes.ego_index == null ? "-" : fu.lanes.ego_index}  oncoming left ${fu.lanes.oncoming_left == null ? "-" : fu.lanes.oncoming_left}  (${fu.lanes.source})`,
        `tunnel ${fu.tunnel.in_tunnel}` + (fu.tunnel.exit_in_m != null ? `  exit in ${f(fu.tunnel.exit_in_m, 0)} m` : "") + `  (${fu.tunnel.source})`
      ].join("\\n") : "no data";
    const vh = await get("/v2/vehicle");
    if (vh && vh.present) {
      const sg = k => vh.signals[k] ? ` [${vh.signals[k].quality}, ${vh.signals[k].source || "-"}]` : "";
      const row = (g, k) => vh[g][k] == null ? null : `${k} ${JSON.stringify(vh[g][k])}${sg(k)}`;
      $("vehicle").textContent = [
        `feeders: ${vh.feeders.map(x => x.name || "?").join(", ") || "-"}`,
        `speed ${f(vh.motion.speed_mps * 3.6, 0)} km/h${vh.motion.standstill ? " (standstill)" : ""}  gear ${vh.motion.gear || "-"}  ` +
          `yaw ${f(vh.motion.yaw_rate_rps, 3)} rad/s (${vh.motion.yaw_source || "-"})`,
        ...["steering_wheel_deg", "accel_long_mps2", "accel_lat_mps2"].map(k => row("motion", k)),
        ...Object.keys(vh.controls).map(k => row("controls", k)),
        ...Object.keys(vh.body).map(k => row("body", k)),
        ...vh.sensors.map(x => `sensor ${x.id} (${x.kind}): ${x.objects} objects, ${f(x.hz)} Hz, age ${f(x.age_s, 2)} s`),
        `calibration: yaw scale ${f(vh.calibration.yaw_rate.scale.value, 3)} (n ${vh.calibration.yaw_rate.scale.n})  ` +
          `speed scale ${f(vh.calibration.speed.scale.value, 3)} (n ${vh.calibration.speed.scale.n})`
      ].filter(Boolean).join("\\n");
    } else $("vehicle").textContent = vh ? "no host feeds vehicle data (vision only)" : "no map box";
  } catch (e) { $("health").textContent = "api unreachable"; }
}
tick(); setInterval(tick, 1000);
</script></body></html>
"""
