#!/usr/bin/env python3
"""wsprobe.py SECONDS [URL] : a /ws consumer on every topic; per topic the message rate, bytes per
second and the latency from the frame's camera timestamp (ts_real_ns) to reception (median, p90)."""
import asyncio, json, sys, time, statistics as st
import aiohttp
D = float(sys.argv[1]) if len(sys.argv) > 1 else 60
URL = sys.argv[2] if len(sys.argv) > 2 else "http://127.0.0.1:8090/ws"
async def main():
    lat, nb = {}, {}
    async with aiohttp.ClientSession() as s:
        async with s.ws_connect(URL, max_msg_size=0) as ws:
            t_end = time.time() + D
            while time.time() < t_end:
                try: m = await asyncio.wait_for(ws.receive(), timeout=max(0.1, t_end - time.time()))
                except asyncio.TimeoutError: break
                if m.type != aiohttp.WSMsgType.TEXT: continue
                now = time.time_ns(); j = json.loads(m.data); t = j.get("type")
                nb[t] = nb.get(t, 0) + len(m.data)
                ts = j.get("ts_real_ns")
                lat.setdefault(t, []).append((now - ts) / 1e6 if ts else None)
    out = {}
    for t, v in sorted(lat.items()):
        v2 = sorted(x for x in v if x is not None)
        out[t] = dict(rate=round(len(v) / D, 2), kb_s=round(nb[t] / D / 1000, 1))
        if v2: out[t].update(lat_med=round(st.median(v2), 1), lat_p90=round(v2[int(0.9 * (len(v2) - 1))], 1))
    print("WS", json.dumps(out))
asyncio.run(main())
