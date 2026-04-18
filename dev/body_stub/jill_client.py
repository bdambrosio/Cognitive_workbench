"""Minimal zenoh client for chatting with the Jill cognitive agent.

Publishes freeform text to `cognitive/{character}/sense_data` and subscribes
to `cognitive/{character}/action` to pick up her "say" replies. Kept separate
from `controller.py` so the core body stub has no Jill dependency; the UI
instantiates this only when the Jill chat mode is selected.

Uses its own zenoh session (same router as the body stub by default).
"""
from __future__ import annotations

import json
import logging
import threading
import time
from collections import deque
from datetime import datetime
from typing import Any, Deque, List, Optional, Tuple

from .transport import open_session

logger = logging.getLogger(__name__)


class JillClient:
    def __init__(
        self,
        router: str,
        character: str = "Jill",
        max_queued_replies: int = 64,
    ):
        self._router = router
        self._character = character
        self._session: Optional[Any] = None
        self._sense_pub: Optional[Any] = None
        self._action_sub: Optional[Any] = None
        self._lock = threading.Lock()
        self._replies: Deque[Tuple[str, float]] = deque(maxlen=max_queued_replies)

    @property
    def character(self) -> str:
        return self._character

    @property
    def connected(self) -> bool:
        return self._session is not None

    def connect(self) -> Optional[str]:
        """Open the session and declare pub/sub. None on success, err str otherwise."""
        if self._session is not None:
            return None
        try:
            self._session = open_session(self._router)
        except Exception as e:
            logger.exception("jill zenoh open failed")
            return f"{type(e).__name__}: {e}"
        try:
            self._sense_pub = self._session.declare_publisher(
                f"cognitive/{self._character}/sense_data"
            )
            self._action_sub = self._session.declare_subscriber(
                f"cognitive/{self._character}/action", self._on_action,
            )
        except Exception as e:
            logger.exception("jill declare pub/sub failed")
            self.close()
            return f"{type(e).__name__}: {e}"
        return None

    def close(self) -> None:
        for obj in (self._action_sub, self._sense_pub):
            if obj is not None:
                try:
                    obj.undeclare()
                except Exception:
                    pass
        self._action_sub = None
        self._sense_pub = None
        if self._session is not None:
            try:
                self._session.close()
            except Exception:
                pass
            self._session = None

    def publish_chat(self, text: str, image_path: Optional[str] = None) -> bool:
        """Send a freeform chat turn to Jill. Returns False if not connected."""
        if self._sense_pub is None:
            return False
        body = text
        if image_path:
            body = f"{text}\n[camera image available at: {image_path}]"
        content = {"source": "User", "text": body}
        payload = {
            "timestamp": datetime.now().isoformat(),
            "sequence_id": 0,
            "mode": "text",
            "content": json.dumps(content),
        }
        try:
            self._sense_pub.put(json.dumps(payload))
        except Exception:
            logger.exception("jill sense_data publish failed")
            return False
        return True

    def drain_replies(self) -> List[Tuple[str, float]]:
        """Pop all queued Jill replies (text, wall-clock ts). UI calls each tick."""
        with self._lock:
            out = list(self._replies)
            self._replies.clear()
        return out

    def _on_action(self, sample: Any) -> None:
        try:
            raw = bytes(sample.payload.to_bytes())
        except AttributeError:
            raw = bytes(sample.payload)
        try:
            msg = json.loads(raw.decode("utf-8"))
        except Exception:
            return
        if msg.get("type") != "say" and msg.get("action_type") != "say":
            return
        if msg.get("source") == "User":
            return
        text = msg.get("text") or ""
        if not text:
            return
        with self._lock:
            self._replies.append((text, time.time()))
