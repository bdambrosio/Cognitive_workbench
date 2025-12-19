from dataclasses import dataclass
from typing import Optional


class SegmentBoundaryError(RuntimeError):
    """Raised when adapter updates are attempted mid-segment."""


@dataclass
class SegmentState:
    """Tracks whether a generation segment is active."""

    active: bool = False
    segment_index: int = 0

    def start(self):
        if self.active:
            raise SegmentBoundaryError(
                "Segment already active; reset KV cache before starting a new one."
            )
        self.active = True
        self.segment_index += 1

    def end(self):
        if not self.active:
            return
        self.active = False

    def ensure_boundary(self, action: str):
        if self.active:
            raise SegmentBoundaryError(
                f"{action} requires a segment boundary (KV cache rebuild) per musing_spec."
            )
