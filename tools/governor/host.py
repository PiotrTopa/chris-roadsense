#!/usr/bin/env python3
"""host.py COMMAND [JSON]: one command to the host controller's REST api (the car computer's backend on the
reference phone: roadsense_mode {"mode":"on"|"auto"}, set_fan {"pct":N}, fan_auto {}). Bench tooling."""
import sys
import urllib.request

body = (sys.argv[2] if len(sys.argv) > 2 else "{}").encode()
req = urllib.request.Request("http://127.0.0.1:8080/api/v1/commands/" + sys.argv[1], data=body,
                             headers={"Content-Type": "application/json"}, method="POST")
try:
    print(sys.argv[1], urllib.request.urlopen(req, timeout=5).read().decode()[:200])
except Exception as e:
    print(sys.argv[1], "FAILED", e)
    sys.exit(1)
