#!/usr/bin/env python3
"""Record the phone api's /ws (selected topics) to JSONL with laptop receive time. usage: rec_ws.py URL OUT SECONDS [topics]"""
import asyncio, json, sys, time, aiohttp
url, out, secs = sys.argv[1], sys.argv[2], float(sys.argv[3])
topics = sys.argv[4] if len(sys.argv) > 4 else "camera,road,signs,lights,speed_limit"
async def main():
    t_end = time.time() + secs
    n = 0
    async with aiohttp.ClientSession() as s:
        with open(out, "a") as f:
            while time.time() < t_end:
                try:
                    async with s.ws_connect(f"{url}/ws?topics={topics}", heartbeat=10, max_msg_size=0) as ws:
                        while time.time() < t_end:
                            m = await ws.receive(timeout=max(1, t_end - time.time()))
                            if m.type != aiohttp.WSMsgType.TEXT:
                                break
                            f.write(json.dumps({"t": time.time(), "m": json.loads(m.data)}, separators=(",", ":")) + "\n")
                            n += 1
                except Exception as e:
                    print("ws error", repr(e), file=sys.stderr, flush=True)
                    await asyncio.sleep(2)
    print("messages", n)
asyncio.run(main())
