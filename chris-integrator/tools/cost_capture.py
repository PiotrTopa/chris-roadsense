#!/usr/bin/env python3
"""Capture what a running chris-integrator is fed, as raw lines with arrival times, for tools/cost_bench.py.

    tools/cost_capture.py OUT.cap.gz [--seconds 180] [--roadsense /run/roadsense] [--url http://127.0.0.1:8090]

Connects to the roadsense daemons' sockets like the integrator does (camerad, modeld, signd, lightd, envd,
laned, api: each daemon serves every client the same lines) and writes `<mono_ns>\\t<socket>\\t<line>`.
What the vehicle feed delivered is not on a socket this tool can read (the feeder connects to the
integrator); it is taken from the integrator's WebSocket (`vehicle`, `ego`, `gnss`, `objects`: the messages
as the input store handed them to the mixer) when aiohttp is there and --url is given, socket name `feed`.
Run it on other cores than the integrator's (`taskset -c 4-7 nice`): it must not load what it observes."""
import argparse
import gzip
import os
import selectors
import socket
import threading
import time

SOCKETS = ("camerad", "modeld", "signd", "lightd", "envd", "laned", "api")
FEED_TOPICS = "vehicle,ego,gnss,objects"


def ws_feed(url, write, stop):
    try:
        import asyncio
        import aiohttp
    except ImportError:
        return

    async def run():
        async with aiohttp.ClientSession() as s:
            async with s.ws_connect(url.rstrip("/") + "/ws?topics=" + FEED_TOPICS, max_msg_size=0) as ws:
                async for m in ws:
                    if stop.is_set():
                        break
                    if m.type == aiohttp.WSMsgType.TEXT:
                        write("feed", m.data.encode())
    try:
        asyncio.run(run())
    except Exception as e:
        print("feed capture ended: %s" % e)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("out")
    ap.add_argument("--seconds", type=float, default=180)
    ap.add_argument("--roadsense", default="/run/roadsense")
    ap.add_argument("--url", default=None)
    a = ap.parse_args()
    out = gzip.open(a.out, "wb", compresslevel=1)
    lock = threading.Lock()
    n = {}

    def write(name, line):
        with lock:
            out.write(b"%d\t%s\t%s\n" % (time.monotonic_ns(), name.encode(), line))
            n[name] = n.get(name, 0) + 1

    stop = threading.Event()
    if a.url:
        threading.Thread(target=ws_feed, args=(a.url, write, stop), daemon=True).start()
    sel = selectors.DefaultSelector()
    for name in SOCKETS:
        p = os.path.join(a.roadsense, name + ".sock")
        s = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        try:
            s.connect(p)
        except OSError:
            continue
        sel.register(s, selectors.EVENT_READ, [name, b""])
    end = time.monotonic() + a.seconds
    while time.monotonic() < end:
        for key, _ in sel.select(0.5):
            data = key.fileobj.recv(1 << 16)
            if not data:
                sel.unregister(key.fileobj)
                continue
            buf = key.data[1] + data
            *lines, key.data[1] = buf.split(b"\n")
            for ln in lines:
                if ln.strip():
                    write(key.data[0], ln)
    stop.set()
    with lock:
        out.close()
    print(n)


if __name__ == "__main__":
    main()
