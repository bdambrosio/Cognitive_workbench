"""CanvasPublisher: in-process display-surface publisher for Jill.

Constructed in ChatLoop.__init__ with no session; attach_session(zenoh) is
called later once the Zenoh session is open. set_content() publishes the
latest payload to a single latest-wins key. Mirrors AffectPublisher.

Keying: `cognitive/{character}/canvas` — one key per character, so
multiple agents can each drive their own display window.
"""
from __future__ import annotations

import logging
import threading
import time
from typing import Any, Optional

from .state import CanvasState, Format

logger = logging.getLogger('canvas.publisher')


def default_key(character: str) -> str:
    return f'cognitive/{character}/canvas'


class CanvasPublisher:
    def __init__(self, key: str, session: Optional[Any] = None):
        self._key = key
        self._pub: Optional[Any] = None
        self._state = CanvasState()
        self._seq = 0
        self._lock = threading.Lock()
        if session is not None:
            self.attach_session(session)

    def attach_session(self, session: Any) -> None:
        try:
            self._pub = session.declare_publisher(self._key)
            logger.info(f"canvas: publishing to {self._key}")
        except Exception as e:
            logger.warning(
                f"canvas: declare_publisher({self._key}) failed: {e}; running headless")
            self._pub = None

    def close(self) -> None:
        if self._pub is None:
            return
        try:
            self._pub.undeclare()
        except Exception as e:
            logger.warning(f"canvas: undeclare failed: {e}")
        self._pub = None

    def set_content(self, content: str, fmt: str = Format.MARKDOWN.value) -> int:
        """Publish a new content payload. Returns content byte length so the
        ReAct tool dispatcher can report a useful observation."""
        if fmt not in (Format.MARKDOWN.value, Format.HTML.value):
            logger.warning(f"canvas: unknown format {fmt!r}, defaulting to markdown")
            fmt = Format.MARKDOWN.value
        body = content if isinstance(content, str) else str(content)
        with self._lock:
            self._state.content = body
            self._state.format = fmt
            self._seq += 1
            self._state.seq = self._seq
            self._state.ts = time.time()
            payload = self._state.to_json()
        if self._pub is not None:
            try:
                self._pub.put(payload)
            except Exception as e:
                logger.warning(f"canvas: publish failed: {e}")
        return len(body.encode('utf-8'))
