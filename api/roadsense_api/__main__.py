import argparse
import logging
import os

from aiohttp import web

from .server import make_app


def main():
    ap = argparse.ArgumentParser(prog="roadsense_api", description="roadsense debug face: raw topics, /preview, load governor")
    ap.add_argument("--rundir", default=os.environ.get("ROADSENSE_RUNDIR", "/run/roadsense"))
    ap.add_argument("--datadir", default=os.environ.get("ROADSENSE_DATADIR", "/var/lib/roadsense"),
                    help="unused since api 3 (the sign history is chris-integrator's)")
    ap.add_argument("--ring", default=os.environ.get("ROADSENSE_RING", "/dev/shm/roadsense-frames"),
                    help="frame ring file, read for its header only")
    ap.add_argument("--rules", default=os.environ.get("ROADSENSE_RULES", "pl"),
                    help="unused since api 3 (the rules engine is chris-integrator's)")
    ap.add_argument("--host", default=os.environ.get("ROADSENSE_API_HOST", "0.0.0.0"))
    ap.add_argument("--port", type=int, default=int(os.environ.get("ROADSENSE_API_PORT", 8091)))
    ap.add_argument("-v", "--verbose", action="store_true")
    a = ap.parse_args()
    logging.basicConfig(level=logging.DEBUG if a.verbose else logging.INFO,
                        format="%(asctime)s %(levelname)s %(name)s: %(message)s")
    web.run_app(make_app(a.rundir, datadir=a.datadir, ring_path=a.ring, rules=a.rules), host=a.host, port=a.port,
                access_log=None, print=None)


if __name__ == "__main__":
    main()
