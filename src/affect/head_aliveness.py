"""HeadAliveness — gentle idle micro-gaze for the ChatterBot head.

Embodied affect: while Jill is *idle* (no turn in flight), this nudges the head
in small, slow movements so she looks alive rather than frozen. It is the
lowest-risk form of autonomy — non-interruptive, no perception, no speech.

Design / safety:
- **Idle only.** It moves the head ONLY when affect mode is `idle`. During a
  turn the deliberate head tools (`head-move`, `look-at-target`) own the head;
  this never fights them.
- **Drifts around an anchor**, not a fixed pose: the anchor is wherever the head
  was left when idle began, so it never *undoes* a deliberate "look at X" — it
  just adds subtle life around it. Bounded to a tight box and the safe envelope.
- **No-op when the bot is absent.** It only moves if fresh `head/status` is
  arriving (the Pi is up and streaming); otherwise it does nothing and never
  blocks on a dead link.
- **Off unless enabled** (launcher `--head-aliveness`).

Observes affect state by subscribing to the in-process `cognitive/affect/state`
topic (same session as the chat loop), so it stays fully decoupled from
AffectPublisher. Drives the head via the shared ChatterLink singleton.
"""
from __future__ import annotations

import json
import logging
import random
import threading
from typing import Any, Optional

from affect.publisher import DEFAULT_KEY as _AFFECT_KEY
from utils.chatter_link import clamp_pose, get_link

logger = logging.getLogger('affect.head_aliveness')

# Idle drift: small offsets around the anchor pose, slow cadence.
_DRIFT_PAN = 6          # max ± degrees, horizontal
_DRIFT_TILT = 4         # max ± degrees, vertical
_INTERVAL_MIN_S = 8.0
_INTERVAL_MAX_S = 18.0
# Only animate if head/status arrived within this many seconds (bot present).
_PRESENCE_MAX_AGE_S = 5.0
# Per-move wait for the Pi to settle; short — aliveness needs no confirmation.
_MOVE_TIMEOUT_S = 2.0


def _payload_bytes(sample: Any) -> bytes:
    payload = getattr(sample, "payload", sample)
    to_bytes = getattr(payload, "to_bytes", None)
    if callable(to_bytes):
        return bytes(to_bytes())
    return bytes(payload)


class HeadAliveness:
    def __init__(self, enabled: bool = False) -> None:
        self._enabled = enabled
        self._sub: Optional[Any] = None
        self._thread: Optional[threading.Thread] = None
        self._stop = threading.Event()
        self._mode_lock = threading.Lock()
        self._mode = 'idle'
        self._anchor: Optional[tuple] = None  # (pan, tilt) drift centre

    def attach_session(self, session: Any) -> None:
        """Subscribe to affect state and start the idle-drift thread. No-op if
        disabled. Mirrors AffectPublisher.attach_session — safe if it fails."""
        if not self._enabled:
            return
        try:
            self._sub = session.declare_subscriber(_AFFECT_KEY, self._on_state)
        except Exception as e:
            logger.warning(f"head_aliveness: subscribe failed: {e}; disabled")
            return
        self._thread = threading.Thread(
            target=self._loop, name='head-aliveness', daemon=True)
        self._thread.start()
        logger.info("head_aliveness: idle micro-gaze active")

    def close(self) -> None:
        self._stop.set()
        if self._sub is not None:
            try:
                self._sub.undeclare()
            except Exception as e:
                logger.warning(f"head_aliveness: sub undeclare failed: {e}")
            self._sub = None

    # -- affect state ----------------------------------------------------

    def _on_state(self, sample: Any) -> None:
        try:
            mode = json.loads(_payload_bytes(sample).decode('utf-8')).get('mode')
        except Exception as e:
            logger.debug(f"head_aliveness: undecodable affect state: {e}")
            return
        if not mode:
            return
        with self._mode_lock:
            self._mode = mode
            if mode != 'idle':
                # Turn started — relinquish the head and re-anchor next idle.
                self._anchor = None

    # -- drift loop ------------------------------------------------------

    def _loop(self) -> None:
        while not self._stop.wait(random.uniform(_INTERVAL_MIN_S,
                                                 _INTERVAL_MAX_S)):
            with self._mode_lock:
                if self._mode != 'idle':
                    continue
            link = get_link()
            if link.ensure() is not None:
                continue
            status = link.latest_head_status()
            if status is None or status.get('age_s', 1e9) > _PRESENCE_MAX_AGE_S:
                continue  # bot absent / not streaming — do nothing, don't block
            with self._mode_lock:
                if self._anchor is None:
                    self._anchor = (status.get('pan', 90), status.get('tilt', 113))
                anchor = self._anchor
            pan = anchor[0] + random.uniform(-_DRIFT_PAN, _DRIFT_PAN)
            tilt = anchor[1] + random.uniform(-_DRIFT_TILT, _DRIFT_TILT)
            pan, tilt = clamp_pose(round(pan), round(tilt))
            try:
                link.send_head_cmd(pan=pan, tilt=tilt, smooth=True,
                                   timeout=_MOVE_TIMEOUT_S)
            except Exception as e:
                logger.debug(f"head_aliveness: move failed: {e}")
