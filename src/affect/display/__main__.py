"""Bridge: Zenoh subscriber on cognitive/affect/state → WebSocket fanout.

Run:
    python -m affect.display
Then open src/affect/display/static/index.html in a browser (file:// is fine).
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

logger = logging.getLogger('affect.display')

WS_HOST = os.environ.get('AFFECT_WS_HOST', '127.0.0.1')
WS_PORT = int(os.environ.get('AFFECT_WS_PORT', '8787'))
ZENOH_KEY = os.environ.get('AFFECT_ZENOH_KEY', 'cognitive/affect/state')


class Bridge:
    def __init__(self) -> None:
        self._clients: Set[Any] = set()
        self._last_payload: str = '{}'
        self._loop: Optional[asyncio.AbstractEventLoop] = None

    async def serve(self) -> None:
        self._loop = asyncio.get_running_loop()
        session = zenoh.open(make_localhost_config())
        sub = session.declare_subscriber(ZENOH_KEY, self._on_zenoh_sample)
        logger.info(f"subscribed to {ZENOH_KEY}; ws://{WS_HOST}:{WS_PORT}")

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
    try:
        asyncio.run(Bridge().serve())
    except KeyboardInterrupt:
        logger.info("interrupted")


if __name__ == '__main__':
    main()
