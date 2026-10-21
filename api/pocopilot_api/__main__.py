import argparse
import logging
import os

from aiohttp import web

from .server import make_app


def main():
    ap = argparse.ArgumentParser(prog="pocopilot_api", description="pocopilot HTTP/WebSocket API")
    ap.add_argument("--rundir", default=os.environ.get("POCOPILOT_RUNDIR", "/run/pocopilot"))
    ap.add_argument("--datadir", default=os.environ.get("POCOPILOT_DATADIR"),
                    help="where signs.jsonl is appended (default <rundir>/../var/lib/pocopilot; skipped if missing)")
    ap.add_argument("--ring", default=os.environ.get("POCOPILOT_RING", "/dev/shm/pocopilot-frames"),
                    help="frame ring file, read for its header only")
    ap.add_argument("--host", default="0.0.0.0")
    ap.add_argument("--port", type=int, default=int(os.environ.get("POCOPILOT_API_PORT", 8080)))
    ap.add_argument("-v", "--verbose", action="store_true")
    a = ap.parse_args()
    logging.basicConfig(level=logging.DEBUG if a.verbose else logging.INFO,
                        format="%(asctime)s %(levelname)s %(name)s: %(message)s")
    datadir = a.datadir or os.path.join(a.rundir, "..", "var", "lib", "pocopilot")
    web.run_app(make_app(a.rundir, datadir=datadir, ring_path=a.ring), host=a.host, port=a.port,
                access_log=None, print=None)


if __name__ == "__main__":
    main()
