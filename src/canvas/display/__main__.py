"""Bridge: Zenoh subscriber on cognitive/{character}/canvas → WebSocket fanout.

Run:
    CANVAS_CHARACTER=jill python -m canvas.display
Then open src/canvas/display/static/index.html in a browser, or use
launcher.py with --canvas to do both automatically.

Env vars:
    CANVAS_CHARACTER   character to subscribe for (required; sets key)
    CANVAS_ZENOH_KEY   override the full key (rare; takes precedence)
    CANVAS_WS_HOST     default 127.0.0.1
    CANVAS_WS_PORT     default 8788
"""
from __future__ import annotations

import asyncio
import logging
import os
import signal
import sys
from pathlib import Path
from typing import Any, Optional, Set

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
        self._last_payload: str = '{}'
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
        self._last_payload = payload
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
                await ws.send(self._last_payload)
            except Exception as e:
                logger.info(f"initial send failed: {e}")
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
    try:
        asyncio.run(Bridge(key).serve())
    except KeyboardInterrupt:
        logger.info("interrupted")


if __name__ == '__main__':
    main()
