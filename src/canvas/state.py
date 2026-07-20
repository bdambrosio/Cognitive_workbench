"""CanvasState: payload published by CanvasPublisher.

Wire format is the JSON-serialized dataclass — `content` carries the
markdown/HTML body, `format` selects how the shim renders it. seq is
monotonic per publisher; ts is wall-clock seconds. turn is the ReAct
episode counter (see CanvasPublisher.new_turn) — the display layer
coalesces same-turn publishes into one scrollback keyframe; 0 means
"no turn info", never coalesced.
"""
from __future__ import annotations

import json
from dataclasses import dataclass, asdict
from enum import Enum


class Format(str, Enum):
    MARKDOWN = 'markdown'
    HTML = 'html'


@dataclass
class CanvasState:
    content: str = ''
    format: str = Format.MARKDOWN.value
    ts: float = 0.0
    seq: int = 0
    turn: int = 0

    def to_json(self) -> str:
        return json.dumps(asdict(self), separators=(',', ':'))
