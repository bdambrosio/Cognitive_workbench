"""Bridge: Zenoh subscriber on cognitive/{character}/canvas → WebSocket fanout
plus a localhost HTTP image proxy on a sibling port.

The WebSocket side fans out canvas content (markdown / HTML) to the browser
page. The HTTP side answers GET /proxy?url=<url>: it fetches the remote
image server-side, caches the bytes in memory, and returns them with the
upstream Content-Type. Model-authored HTML can therefore reference an image
via <img src="http://127.0.0.1:<port>/proxy?url=..."> without the bytes ever
flowing through the agent's LLM context.

Run:
    CANVAS_CHARACTER=jill python -m canvas.display
Then open src/canvas/display/static/index.html in a browser, or use
launcher.py with --canvas to do both automatically.

Env vars:
    CANVAS_CHARACTER   character to subscribe for (required; sets key)
    CANVAS_ZENOH_KEY   override the full key (rare; takes precedence)
    CANVAS_WS_HOST     default 127.0.0.1
    CANVAS_WS_PORT     default 8788
    CANVAS_HTTP_HOST   default 127.0.0.1 (image-proxy bind)
    CANVAS_HTTP_PORT   default 8789      (image-proxy bind)
"""
from __future__ import annotations

import asyncio
import collections
import ipaddress
import json
import logging
import os
import signal
import socket
import sys
import threading
import time
import urllib.parse
import urllib.request
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Any, Optional, Set
from urllib.error import HTTPError, URLError

_THIS_DIR = Path(__file__).resolve().parent
_SRC_DIR = _THIS_DIR.parent.parent
if str(_SRC_DIR) not in sys.path:
    sys.path.insert(0, str(_SRC_DIR))

import websockets
import zenoh  # noqa: E402

from utils.zenoh_utils import make_localhost_config  # noqa: E402

logger = logging.getLogger('canvas.display')

WS_HOST = os.environ.get('CANVAS_WS_HOST', '127.0.0.1')
WS_PORT = int(os.environ.get('CANVAS_WS_PORT', '8788'))

HTTP_HOST = os.environ.get('CANVAS_HTTP_HOST', '127.0.0.1')
HTTP_PORT = int(os.environ.get('CANVAS_HTTP_PORT', '8789'))

# Proxy bounds. The cache is for repeated views of the same image (Jill
# tends to re-render the same hero image as a turn iterates). The byte
# caps stop a misbehaving model from filling RAM or fetching a 4K video
# file as if it were an icon.
_PROXY_CACHE_MAX_ENTRIES = 100
_PROXY_CACHE_MAX_BYTES = 10 * 1024 * 1024   # 10 MiB total resident
_PROXY_CACHE_TTL_S = 3600                   # 1 h
_PROXY_FETCH_TIMEOUT_S = 15
_PROXY_MAX_RESPONSE_BYTES = 8 * 1024 * 1024  # 8 MiB per image

# /local route — serves files produced by Jill's tools (rendered CAD
# meshes, generated images, etc.). Hard-allowlisted to a single root so
# a path-traversal trick can't escape into $HOME. The cad helper writes
# under ~/.cache/cognitive/cad/<character>/; other tools that want to
# expose artifacts should write under ~/.cache/cognitive/<their-name>/.
_LOCAL_ROOT = Path(
    os.environ.get('CANVAS_LOCAL_ROOT', '~/.cache/cognitive')
).expanduser().resolve()
_LOCAL_MAX_RESPONSE_BYTES = 50 * 1024 * 1024  # 50 MiB — meshes can be bigger than images

# Scrollback: the bridge keeps the last N canvas keyframes and replays them
# to each connecting client, so the page can offer paging back through
# recent entries (and a reloaded page repopulates). Publishes carrying the
# same `turn` (ReAct episode counter) coalesce into one keyframe — the
# turn's final render wins; drafts still stream to live clients untouched.
# In-memory only — dies with the bridge process; cross-session persistence
# deliberately deferred.
_HISTORY_MAX = 50

# Minimal MIME map. Anything else is served as octet-stream, which model-
# viewer / browsers handle fine for known-extension files.
_LOCAL_MIME = {
    '.glb': 'model/gltf-binary',
    '.gltf': 'model/gltf+json',
    '.stl': 'model/stl',
    '.obj': 'text/plain',
    '.png': 'image/png',
    '.jpg': 'image/jpeg',
    '.jpeg': 'image/jpeg',
    '.svg': 'image/svg+xml',
    '.json': 'application/json',
    '.txt': 'text/plain; charset=utf-8',
}


# ── Image proxy ────────────────────────────────────────────────────
#
# Runs in a daemon thread alongside the asyncio websocket loop. We use
# stdlib's http.server rather than aiohttp to avoid a new dep — this is a
# single-route GET endpoint so the thread model is fine.
#
# SSRF: we restrict to http/https, then resolve the hostname and reject
# any address that is loopback / private / link-local / reserved /
# multicast. Redirects re-run the same check. The proxy binds to
# 127.0.0.1 only, so the residual attack surface is the model itself —
# this guard prevents a model-supplied URL (or a malicious redirect
# chain) from reaching cloud-metadata endpoints or LAN services.


def _host_is_blocked(host: str) -> bool:
    """True if `host` resolves to any address we won't fetch from.
    Returns True (block) on resolution failure as well — we'd rather
    fail-closed on a typo'd host than open the SSRF door."""
    if not host:
        return True
    try:
        infos = socket.getaddrinfo(host, None)
    except socket.gaierror as e:
        logger.info(f"proxy: getaddrinfo({host!r}) failed: {e}")
        return True
    for info in infos:
        addr = info[4][0]
        try:
            ip = ipaddress.ip_address(addr)
        except ValueError:
            continue
        if (ip.is_loopback or ip.is_private or ip.is_link_local
                or ip.is_reserved or ip.is_multicast or ip.is_unspecified):
            return True
    return False


class _SSRFGuardedRedirectHandler(urllib.request.HTTPRedirectHandler):
    """Re-validate scheme + host on every redirect hop. urllib's default
    handler follows blindly, which would let `https://evil.tld/x` 302 to
    `http://169.254.169.254/...` and bypass the up-front check."""

    def redirect_request(self, req, fp, code, msg, hdrs, newurl):
        parsed = urllib.parse.urlparse(newurl)
        if parsed.scheme not in ('http', 'https'):
            raise URLError(f"redirect to unsupported scheme {parsed.scheme!r}")
        if _host_is_blocked(parsed.hostname or ''):
            raise URLError(f"redirect to disallowed host {parsed.hostname!r}")
        return super().redirect_request(req, fp, code, msg, hdrs, newurl)


_proxy_opener = urllib.request.build_opener(_SSRFGuardedRedirectHandler())


class _ProxyCache:
    """LRU + total-byte cap + TTL. Thread-safe (handler runs in
    ThreadingHTTPServer worker threads)."""

    def __init__(self, max_entries: int, max_bytes: int, ttl_s: float):
        self._entries: "collections.OrderedDict[str, tuple[float, str, bytes]]" = (
            collections.OrderedDict()
        )
        self._bytes = 0
        self._max_entries = max_entries
        self._max_bytes = max_bytes
        self._ttl_s = ttl_s
        self._lock = threading.Lock()

    def get(self, key: str) -> Optional[tuple[str, bytes]]:
        with self._lock:
            entry = self._entries.get(key)
            if entry is None:
                return None
            ts, ctype, body = entry
            if time.time() - ts > self._ttl_s:
                self._entries.pop(key, None)
                self._bytes -= len(body)
                return None
            self._entries.move_to_end(key)
            return ctype, body

    def put(self, key: str, ctype: str, body: bytes) -> None:
        with self._lock:
            if key in self._entries:
                _, _, old_body = self._entries.pop(key)
                self._bytes -= len(old_body)
            self._entries[key] = (time.time(), ctype, body)
            self._bytes += len(body)
            while (len(self._entries) > self._max_entries
                   or self._bytes > self._max_bytes):
                if not self._entries:
                    break
                _, (_, _, evicted) = self._entries.popitem(last=False)
                self._bytes -= len(evicted)


_proxy_cache = _ProxyCache(
    _PROXY_CACHE_MAX_ENTRIES, _PROXY_CACHE_MAX_BYTES, _PROXY_CACHE_TTL_S,
)


class _ProxyHandler(BaseHTTPRequestHandler):
    server_version = 'CanvasProxy/0.1'

    def log_message(self, fmt: str, *args: Any) -> None:
        logger.info(f"proxy: {self.address_string()} {fmt % args}")

    def _reject(self, code: int, msg: str) -> None:
        body = msg.encode('utf-8', errors='replace')
        self.send_response(code)
        self.send_header('Content-Type', 'text/plain; charset=utf-8')
        self.send_header('Content-Length', str(len(body)))
        self.end_headers()
        try:
            self.wfile.write(body)
        except Exception as e:
            logger.info(f"proxy: reject write failed: {e}")

    def do_GET(self) -> None:  # noqa: N802 (stdlib signature)
        parsed = urllib.parse.urlparse(self.path)
        if parsed.path == '/local':
            self._handle_local(parsed)
            return
        if parsed.path != '/proxy':
            self._reject(404, "route not found (try /proxy?url=... or /local?path=...)")
            return
        qs = urllib.parse.parse_qs(parsed.query)
        urls = qs.get('url') or []
        if not urls:
            self._reject(400, "missing 'url' query parameter")
            return
        url = urls[0]
        target = urllib.parse.urlparse(url)
        if target.scheme not in ('http', 'https'):
            self._reject(400, f"unsupported scheme {target.scheme!r}")
            return
        if _host_is_blocked(target.hostname or ''):
            self._reject(403, "host rejected (private/loopback/unresolvable)")
            return
        cached = _proxy_cache.get(url)
        if cached is not None:
            ctype, body = cached
            self._respond(ctype, body)
            return
        try:
            req = urllib.request.Request(url, headers={
                'User-Agent': 'cognitive-canvas-proxy/0.1',
                'Accept': 'image/*,*/*;q=0.8',
            })
            with _proxy_opener.open(req, timeout=_PROXY_FETCH_TIMEOUT_S) as resp:
                ctype = resp.headers.get('Content-Type', 'application/octet-stream')
                # Read one byte over the cap to detect oversize without
                # blowing memory if the server lies about Content-Length.
                body = resp.read(_PROXY_MAX_RESPONSE_BYTES + 1)
        except HTTPError as e:
            logger.info(f"proxy: upstream {e.code} for {url}: {e.reason}")
            self._reject(502, f"upstream HTTP {e.code}: {e.reason}")
            return
        except URLError as e:
            logger.info(f"proxy: URLError for {url}: {e.reason}")
            self._reject(502, f"fetch failed: {e.reason}")
            return
        except socket.timeout as e:
            logger.info(f"proxy: timeout for {url}: {e}")
            self._reject(504, "upstream timeout")
            return
        except Exception as e:
            logger.warning(f"proxy: unexpected error fetching {url}: {e}")
            self._reject(502, f"fetch failed: {e}")
            return
        if len(body) > _PROXY_MAX_RESPONSE_BYTES:
            self._reject(413, "upstream response exceeds proxy size cap")
            return
        # The /proxy route exists to feed <img src=...> tags. If upstream
        # returns anything other than image bytes (most commonly text/html
        # because the caller pasted a webpage URL by mistake), refuse —
        # the browser would render the bytes as a broken-image icon. The
        # 415 surfaces in the bridge log so the user can diagnose; for a
        # caller that needs the actionable hint mid-loop, fold a preflight
        # into the publishing tool itself (deferred).
        primary_ctype = ctype.split(';', 1)[0].strip().lower()
        if not primary_ctype.startswith('image/'):
            self._reject(
                415,
                f"upstream returned {primary_ctype!r}, not an image. "
                "If this is a webpage URL, fetch_text it first and use "
                "metadata.html_metadata.images.og_image rather than piping "
                "the page URL through this proxy."
            )
            return
        _proxy_cache.put(url, ctype, body)
        self._respond(ctype, body)

    def _respond(self, ctype: str, body: bytes) -> None:
        try:
            self.send_response(200)
            self.send_header('Content-Type', ctype)
            self.send_header('Content-Length', str(len(body)))
            # Allow the page (loaded from file://) to read the bytes.
            self.send_header('Access-Control-Allow-Origin', '*')
            self.send_header('Cache-Control', 'public, max-age=3600')
            self.end_headers()
            self.wfile.write(body)
        except Exception as e:
            logger.info(f"proxy: response write failed: {e}")

    def _handle_local(self, parsed: urllib.parse.ParseResult) -> None:
        """Serve a file from under _LOCAL_ROOT. Resolves symlinks before
        the allowlist check, so a hostile symlink inside the root can't
        escape it. Files at _LOCAL_ROOT itself are allowed; the parent
        is not."""
        qs = urllib.parse.parse_qs(parsed.query)
        paths = qs.get('path') or []
        if not paths:
            self._reject(400, "missing 'path' query parameter")
            return
        raw = paths[0]
        try:
            resolved = Path(raw).expanduser().resolve(strict=True)
        except FileNotFoundError:
            self._reject(404, "file not found")
            return
        except (OSError, RuntimeError) as e:
            # resolve() raises RuntimeError on symlink loops.
            logger.info(f"local: resolve failed for {raw!r}: {e}")
            self._reject(400, f"path resolve failed: {e}")
            return
        try:
            resolved.relative_to(_LOCAL_ROOT)
        except ValueError:
            self._reject(
                403,
                f"path outside allowed root ({_LOCAL_ROOT})",
            )
            return
        if not resolved.is_file():
            self._reject(404, "not a regular file")
            return
        try:
            size = resolved.stat().st_size
        except OSError as e:
            self._reject(500, f"stat failed: {e}")
            return
        if size > _LOCAL_MAX_RESPONSE_BYTES:
            self._reject(413, "file exceeds local-route size cap")
            return
        try:
            body = resolved.read_bytes()
        except OSError as e:
            self._reject(500, f"read failed: {e}")
            return
        ctype = _LOCAL_MIME.get(
            resolved.suffix.lower(), 'application/octet-stream',
        )
        # No caching headers: files change in place (e.g. current.stl
        # gets overwritten each render), and we want the browser to
        # always see the latest bytes on a fresh load.
        try:
            self.send_response(200)
            self.send_header('Content-Type', ctype)
            self.send_header('Content-Length', str(len(body)))
            self.send_header('Access-Control-Allow-Origin', '*')
            self.send_header('Cache-Control', 'no-cache')
            self.end_headers()
            self.wfile.write(body)
        except Exception as e:
            logger.info(f"local: response write failed: {e}")


def _start_proxy_server() -> ThreadingHTTPServer:
    # Ensure the local-route root exists so resolve()/relative_to() have
    # a stable target. Tools that write under it create their own
    # subdirs; this just guarantees the parent.
    try:
        _LOCAL_ROOT.mkdir(parents=True, exist_ok=True)
    except OSError as e:
        logger.warning(f"local-route root {_LOCAL_ROOT} not creatable: {e}")
    server = ThreadingHTTPServer((HTTP_HOST, HTTP_PORT), _ProxyHandler)
    thread = threading.Thread(
        target=server.serve_forever, name='canvas-proxy', daemon=True,
    )
    thread.start()
    logger.info(
        f"image proxy: http://{HTTP_HOST}:{HTTP_PORT}/proxy?url=<image-url>"
    )
    logger.info(
        f"local files: http://{HTTP_HOST}:{HTTP_PORT}/local?path=<{_LOCAL_ROOT}/...>"
    )
    return server


def resolve_zenoh_key() -> str:
    override = os.environ.get('CANVAS_ZENOH_KEY')
    if override:
        return override
    character = os.environ.get('CANVAS_CHARACTER')
    if not character:
        raise SystemExit(
            "CANVAS_CHARACTER env var required (or set CANVAS_ZENOH_KEY).")
    return f'cognitive/{character}/canvas'


class Bridge:
    def __init__(self, zenoh_key: str) -> None:
        self._zenoh_key = zenoh_key
        self._clients: Set[Any] = set()
        # Entries are (turn, payload); turn is None when the payload has no
        # usable turn stamp (pre-turn publisher, non-JSON) — never coalesced.
        self._history: "collections.deque[tuple[Optional[int], str]]" = (
            collections.deque(maxlen=_HISTORY_MAX))
        self._loop: Optional[asyncio.AbstractEventLoop] = None

    async def serve(self) -> None:
        self._loop = asyncio.get_running_loop()
        session = zenoh.open(make_localhost_config())
        sub = session.declare_subscriber(self._zenoh_key, self._on_zenoh_sample)
        logger.info(f"subscribed to {self._zenoh_key}; ws://{WS_HOST}:{WS_PORT}")

        stop = asyncio.Event()
        for sig in (signal.SIGINT, signal.SIGTERM):
            try:
                self._loop.add_signal_handler(sig, stop.set)
            except NotImplementedError as e:
                logger.debug(f"signal handler unavailable for {sig}: {e}")

        try:
            async with websockets.serve(self._on_client, WS_HOST, WS_PORT):
                await stop.wait()
        finally:
            try:
                sub.undeclare()
            except Exception as e:
                logger.warning(f"subscriber undeclare failed: {e}")
            try:
                session.close()
            except Exception as e:
                logger.warning(f"zenoh session close failed: {e}")

    def _on_zenoh_sample(self, sample: Any) -> None:
        try:
            payload = bytes(sample.payload).decode('utf-8', errors='replace')
        except Exception as e:
            logger.warning(f"decode failed: {e}")
            return
        turn: Optional[int] = None
        try:
            t = json.loads(payload).get('turn')
            if isinstance(t, int) and t > 0:
                turn = t
        except Exception as e:
            logger.info(f"payload turn parse failed (kept as own entry): {e}")
        if turn is not None and self._history and self._history[-1][0] == turn:
            self._history[-1] = (turn, payload)   # same-turn draft → keyframe
        else:
            self._history.append((turn, payload))
        if self._loop is None:
            return
        try:
            asyncio.run_coroutine_threadsafe(self._broadcast(payload), self._loop)
        except Exception as e:
            logger.warning(f"schedule broadcast failed: {e}")

    async def _broadcast(self, payload: str) -> None:
        if not self._clients:
            return
        dead = []
        for ws in list(self._clients):
            try:
                await ws.send(payload)
            except Exception as e:
                logger.info(f"client send failed: {e}")
                dead.append(ws)
        for ws in dead:
            self._clients.discard(ws)

    async def _on_client(self, ws: Any) -> None:
        self._clients.add(ws)
        logger.info(f"client connected (n={len(self._clients)})")
        try:
            try:
                # Replay recent history oldest-first; the page dedups by
                # (seq, ts) so a reconnect doesn't duplicate entries.
                for _, payload in list(self._history):
                    await ws.send(payload)
            except Exception as e:
                logger.info(f"history replay failed: {e}")
            async for _ in ws:
                pass
        except Exception as e:
            logger.info(f"client loop ended: {e}")
        finally:
            self._clients.discard(ws)
            logger.info(f"client disconnected (n={len(self._clients)})")


def main() -> None:
    logging.basicConfig(
        level=logging.INFO,
        format='%(asctime)s %(levelname)s %(name)s %(message)s',
    )
    key = resolve_zenoh_key()
    proxy_server: Optional[ThreadingHTTPServer] = None
    try:
        proxy_server = _start_proxy_server()
    except OSError as e:
        # Port already bound by an earlier instance, or permission denied.
        # The websocket side is independently useful, so log and carry on.
        logger.warning(
            f"image proxy disabled — bind {HTTP_HOST}:{HTTP_PORT} failed: {e}"
        )
    try:
        asyncio.run(Bridge(key).serve())
    except KeyboardInterrupt:
        logger.info("interrupted")
    finally:
        if proxy_server is not None:
            try:
                proxy_server.shutdown()
            except Exception as e:
                logger.info(f"proxy server shutdown failed: {e}")


if __name__ == '__main__':
    main()
