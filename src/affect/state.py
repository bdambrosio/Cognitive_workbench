"""ProcState: state vector published by AffectPublisher.

Wire format is the JSON-serialized dataclass. Keep field names stable —
the P5 sketch reads them by name. All time fields are wall-clock seconds
(time.time()) so a separate display process can compute ages without a
shared monotonic clock.
"""
from __future__ import annotations

import json
from dataclasses import dataclass, asdict
from enum import Enum
from typing import Optional


class Mode(str, Enum):
    IDLE = 'idle'
    THINKING = 'thinking'
    TOOL_EXEC = 'tool_exec'


class Trigger(str, Enum):
    USER = 'user'
    AUTONOMOUS = 'autonomous'
    SCHEDULED = 'scheduled'


class MemoryOp(str, Enum):
    NONE = 'none'
    RECALL = 'recall'
    COMPANION_UPDATE = 'companion_update'


@dataclass
class ProcState:
    mode: str = Mode.IDLE.value
    mode_entered_at: float = 0.0           # wall-clock seconds
    trigger: str = Trigger.USER.value
    react_iter: int = 0
    loop_started_at: Optional[float] = None
    waiting_for_llm: bool = False
    tool_name: Optional[str] = None
    llm_retries: int = 0
    tool_errors: int = 0
    repeat_tool_call: int = 0
    active_concerns: int = 0
    memory_op: str = MemoryOp.NONE.value
    # Wall-clock seconds of the most recent successful ReAct completion
    # (respond exit). Display decays this into a transient happy gesture.
    last_completed_at: Optional[float] = None
    ts: float = 0.0                        # publish wall-clock seconds
    seq: int = 0                           # monotonic per publisher

    def to_json(self) -> str:
        return json.dumps(asdict(self), separators=(',', ':'))
