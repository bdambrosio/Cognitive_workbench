"""Bridge: a character's replies → one row each → WebSocket.

Subscribes to cognitive/{character}/action. A turn that ended silent
publishes nothing there, so it gets no row. Canvas updates get no row: the
canvas shows either a copy of the reply or a display made in a turn that
also replies, so the reply's row already covers it.

Run:
    OUTPUTS_CHARACTER=Jill python -m outputs.display
Then open src/outputs/display/static/index.html in a browser (file:// is fine).

Every WebSocket message is a JSON list of rows. A new client gets the rows
held so far; after that each message carries the one row that was added.
A row is {id, ts, turn_seq, trigger, text}.
"""
from __future__ import annotations

import asyncio
import json
import logging
import os
import signal
import sys
import threading
from datetime import datetime
from pathlib import Path
from typing import Any, Dict, List, Optional, Set

_THIS_DIR = Path(__file__).resolve().parent
_SRC_DIR = _THIS_DIR.parent.parent
if str(_SRC_DIR) not in sys.path:
    sys.path.insert(0, str(_SRC_DIR))

import websockets
import zenoh  # noqa: E402

from utils.zenoh_utils import make_localhost_config  # noqa: E402

logger = logging.getLogger('outputs.display')

WS_HOST = os.environ.get('OUTPUTS_WS_HOST', '127.0.0.1')
WS_PORT = int(os.environ.get('OUTPUTS_WS_PORT', '8792'))
CHARACTER = os.environ.get('OUTPUTS_CHARACTER', 'Jill')

# Rows held for a client that connects or reconnects. Older rows are dropped.
MAX_ROWS = 300


class Bridge:
    def __init__(self) -> None:
        self._clients: Set[Any] = set()
        self._rows: List[Dict[str, Any]] = []
        self._next_id = 1
        self._lock = threading.Lock()
        self._loop: Optional[asyncio.AbstractEventLoop] = None

    async def serve(self) -> None:
        self._loop = asyncio.get_running_loop()
        session = zenoh.open(make_localhost_config())
        action_key = f'cognitive/{CHARACTER}/action'
        sub = session.declare_subscriber(action_key, self._on_action)
        logger.info(f"subscribed to {action_key}; ws://{WS_HOST}:{WS_PORT}")

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

    def _on_action(self, sample: Any) -> None:
        try:
            msg = json.loads(bytes(sample.payload).decode('utf-8', errors='replace'))
        except Exception as e:
            logger.warning(f"action decode failed: {e}")
            return
        if msg.get('type') != 'say':
            return
        try:
            ts = datetime.fromisoformat(msg['timestamp']).timestamp()
        except Exception as e:
            logger.warning(f"action timestamp unreadable: {e}")
            return
        row = {
            'ts': ts,
            'turn_seq': msg.get('turn_seq'),
            'trigger': msg.get('trigger') or '',
            'text': str(msg.get('text') or ''),
        }
        with self._lock:
            row['id'] = self._next_id
            self._next_id += 1
            self._rows.append(row)
            del self._rows[:-MAX_ROWS]
        if self._loop is None:
            return
        try:
            asyncio.run_coroutine_threadsafe(
                self._broadcast(json.dumps([row])), self._loop)
        except Exception as e:
            logger.warning(f"schedule broadcast failed: {e}")

    async def _broadcast(self, payload: str) -> None:
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
            with self._lock:
                backlog = json.dumps(self._rows)
            try:
                await ws.send(backlog)
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
