"""
Lightweight HTTP listener that receives page visit URLs from the browser extension.
Runs on port 5004. Can be run standalone or imported and started in a thread.
"""

import json
import threading
from datetime import datetime
from http.server import HTTPServer, BaseHTTPRequestHandler
from typing import Callable

DEFAULT_PORT = 5004

_visits: list[dict] = []
_callback: Callable[[dict], None] | None = None


class _Handler(BaseHTTPRequestHandler):
    def do_POST(self):
        if self.path != "/page-visit":
            self.send_response(404)
            self.end_headers()
            return

        length = int(self.headers.get("Content-Length", 0))
        body = json.loads(self.rfile.read(length)) if length else {}

        # Accept single entry or batch (list) from buffer flush
        entries = body if isinstance(body, list) else [body]
        for entry in entries:
            entry.setdefault("received_at", datetime.now().isoformat())
            _visits.append(entry)
            if _callback:
                _callback(entry)

        self.send_response(200)
        self.send_header("Content-Type", "application/json")
        self.send_header("Access-Control-Allow-Origin", "*")
        self.end_headers()
        self.wfile.write(json.dumps({"queued": len(entries)}).encode())

    def do_OPTIONS(self):
        self.send_response(204)
        self.send_header("Access-Control-Allow-Origin", "*")
        self.send_header("Access-Control-Allow-Methods", "POST, OPTIONS")
        self.send_header("Access-Control-Allow-Headers", "Content-Type")
        self.end_headers()

    def log_message(self, format, *args):
        # Quiet by default; override to enable logging
        pass


def get_visits() -> list[dict]:
    """Return all received visits."""
    return list(_visits)


def pop_visits() -> list[dict]:
    """Return and clear all received visits."""
    result = list(_visits)
    _visits.clear()
    return result


def start(port: int = DEFAULT_PORT, callback: Callable[[dict], None] | None = None, daemon: bool = True):
    """Start the listener in a background thread. Returns the thread."""
    global _callback
    _callback = callback
    server = HTTPServer(("127.0.0.1", port), _Handler)
    t = threading.Thread(target=server.serve_forever, name="url-listener", daemon=daemon)
    t.start()
    print(f"URL listener started on port {port}")
    return t


if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser(description="Browser URL listener")
    parser.add_argument("--port", type=int, default=DEFAULT_PORT)
    parser.add_argument("--verbose", action="store_true")
    args = parser.parse_args()

    def on_visit(entry):
        print(f"  [{entry.get('timestamp', '?')}] {entry.get('url', '?')}")

    if args.verbose:
        _Handler.log_message = BaseHTTPRequestHandler.log_message

    start(port=args.port, callback=on_visit, daemon=False)
