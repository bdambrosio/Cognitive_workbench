"""AffectPublisher: in-process state publisher for Jill.

Construction is always safe: AffectPublisher(session=None) gives a headless
no-op publisher. Call attach_session(zenoh_session) once the session is
open to start publishing. Every mutating method auto-publishes the full
latest state to a single latest-wins key.
"""
from __future__ import annotations

import logging
import threading
import time
from typing import Any, Optional

from .state import Mode, MemoryOp, ProcState, Trigger

logger = logging.getLogger('affect.publisher')

DEFAULT_KEY = 'cognitive/affect/state'


class AffectPublisher:
    def __init__(self, session: Optional[Any] = None, key: str = DEFAULT_KEY):
        self._key = key
        self._pub: Optional[Any] = None
        self._state = ProcState()
        self._state.mode_entered_at = time.time()
        self._seq = 0
        self._lock = threading.Lock()
        self._last_tool_sig: Optional[str] = None
        if session is not None:
            self.attach_session(session)

    def attach_session(self, session: Any) -> None:
        try:
            self._pub = session.declare_publisher(self._key)
            logger.info(f"affect: publishing to {self._key}")
        except Exception as e:
            logger.warning(f"affect: declare_publisher({self._key}) failed: {e}; running headless")
            self._pub = None
            return
        self._publish()

    def close(self) -> None:
        if self._pub is None:
            return
        try:
            self._pub.undeclare()
        except Exception as e:
            logger.warning(f"affect: undeclare failed: {e}")
        self._pub = None

    # ── mode / trigger ─────────────────────────────────────────────

    def set_mode(self, mode: str) -> None:
        with self._lock:
            if self._state.mode == mode:
                return
            self._state.mode = mode
            self._state.mode_entered_at = time.time()
        self._publish()

    def set_trigger(self, trigger: str) -> None:
        with self._lock:
            if self._state.trigger == trigger:
                return
            self._state.trigger = trigger
        self._publish()

    # ── react loop lifecycle ───────────────────────────────────────

    def enter_loop(self) -> None:
        now = time.time()
        with self._lock:
            self._state.loop_started_at = now
            self._state.react_iter = 0
            self._state.tool_errors = 0
            self._state.repeat_tool_call = 0
            self._state.llm_retries = 0
            self._last_tool_sig = None
        self._publish()

    def exit_loop(self) -> None:
        with self._lock:
            self._state.loop_started_at = None
            self._state.react_iter = 0
            self._state.tool_name = None
            self._state.waiting_for_llm = False
            self._state.tool_errors = 0
            self._state.repeat_tool_call = 0
            self._state.llm_retries = 0
            self._state.memory_op = MemoryOp.NONE.value
            self._last_tool_sig = None
        self._publish()

    def set_react_iter(self, i: int) -> None:
        with self._lock:
            self._state.react_iter = i
        self._publish()

    # ── LLM signal ─────────────────────────────────────────────────

    def set_waiting_for_llm(self, waiting: bool) -> None:
        with self._lock:
            if self._state.waiting_for_llm == waiting:
                return
            self._state.waiting_for_llm = waiting
        self._publish()

    def incr_llm_retry(self) -> None:
        with self._lock:
            self._state.llm_retries += 1
        self._publish()

    # ── tool ───────────────────────────────────────────────────────

    def set_tool(self, name: Optional[str], args_sig: str = '') -> int:
        """Set current tool and auto-set mode. Returns repeat count for
        this (name+args_sig) within the current loop. Pass name=None to
        clear (mode goes back to thinking)."""
        with self._lock:
            self._state.tool_name = name
            if name is None:
                self._state.mode = Mode.THINKING.value
                self._state.mode_entered_at = time.time()
                rc = 0
            else:
                self._state.mode = Mode.TOOL_EXEC.value
                self._state.mode_entered_at = time.time()
                sig = f'{name}|{args_sig}'
                if sig == self._last_tool_sig:
                    self._state.repeat_tool_call += 1
                else:
                    self._state.repeat_tool_call = 0
                    self._last_tool_sig = sig
                rc = self._state.repeat_tool_call
        self._publish()
        return rc

    def incr_tool_error(self) -> None:
        with self._lock:
            self._state.tool_errors += 1
        self._publish()

    # ── memory / concerns ──────────────────────────────────────────

    def set_memory_op(self, op: str) -> None:
        with self._lock:
            if self._state.memory_op == op:
                return
            self._state.memory_op = op
        self._publish()

    def set_active_concerns(self, n: int) -> None:
        with self._lock:
            if self._state.active_concerns == n:
                return
            self._state.active_concerns = n
        self._publish()

    def mark_completion(self) -> None:
        """Stamp a successful ReAct completion (respond exit). Display
        decays this into a transient happy gesture."""
        with self._lock:
            self._state.last_completed_at = time.time()
        self._publish()

    # ── publish ────────────────────────────────────────────────────

    def _publish(self) -> None:
        with self._lock:
            self._seq += 1
            self._state.ts = time.time()
            self._state.seq = self._seq
            payload = self._state.to_json()
        if self._pub is None:
            return
        try:
            self._pub.put(payload)
        except Exception as e:
            logger.warning(f"affect: publish failed: {e}")
