#!/usr/bin/env python3
"""tap.py OUT.jsonl SECONDS [RUNDIR] : record what the daemons publish on their sockets, one line per
message `{"t": seconds, "d": daemon, "m": <the line>}`. Input for api_profile.py (the api's cost per
stage on exactly the stream the phone produced)."""
import asyncio, json, os, sys, time
OUT, SECS = sys.argv[1], float(sys.argv[2]); RUN = sys.argv[3] if len(sys.argv) > 3 else "/run/l21"
SOCKS = {"camerad": "camerad", "modeld": "modeld", "signd": "signd", "envd": "envd", "lightd": "lightd", "mapd": "mapd", "ego": "mapd", "vehicle": "mapd"}
async def tap(name, daemon, fh, t0):
    try: r, w = await asyncio.open_unix_connection(os.path.join(RUN, name + ".sock"), limit=1 << 24)
    except OSError: return
    while True:
        raw = await r.readline()
        if not raw: break
        fh.write(json.dumps({"t": round(time.monotonic() - t0, 4), "d": daemon, "m": raw.decode("utf-8", "replace").strip()}) + "\n")
async def main():
    with open(OUT, "w") as fh:
        t0 = time.monotonic()
        tasks = [asyncio.create_task(tap(n, d, fh, t0)) for n, d in SOCKS.items()]
        await asyncio.sleep(SECS)
        for t in tasks: t.cancel()
asyncio.run(main())
