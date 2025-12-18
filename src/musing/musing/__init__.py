"""Musing adapter scaffolds and segment guards."""

from .config import AdapterConfig, AdapterMagnitude
from .qwen3_adapter import AdapterSiteShape, Qwen3AdapterScaffold
from .segment import SegmentBoundaryError, SegmentState

__all__ = [
    "AdapterConfig",
    "AdapterMagnitude",
    "AdapterSiteShape",
    "Qwen3AdapterScaffold",
    "SegmentBoundaryError",
    "SegmentState",
]
