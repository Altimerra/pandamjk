# /// script
# requires-python = ">=3.9"
# dependencies = []
# ///

"""Serve the custom Panda MJK web dashboard (panda_dashboard/) on the host.

This is a plain static file server -- the dashboard connects straight to
the rosbridge websocket from the browser via roslibjs, so no backend or
ROS 2 install is needed here. panda_control.launch.py already exposes
rosbridge on port 9090 (host networking makes that port reachable from
the Mac host).
"""

import functools
import http.server
import os
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent
DASHBOARD_DIR = REPO_ROOT / "panda_dashboard"

if __name__ == "__main__":
    if not DASHBOARD_DIR.exists():
        print(f"Could not find {DASHBOARD_DIR}", file=sys.stderr)
        sys.exit(1)

    port = int(os.environ.get("PANDA_DASHBOARD_PORT", "8080"))
    handler = functools.partial(http.server.SimpleHTTPRequestHandler, directory=str(DASHBOARD_DIR))
    server = http.server.ThreadingHTTPServer(("0.0.0.0", port), handler)
    print(f"Serving panda_dashboard on http://localhost:{port}")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
