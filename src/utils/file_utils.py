"""Atomic file-write helpers.

All full-rewrite persistence in the codebase should go through these so a
crash mid-write can never leave a truncated file behind: content is written
to a sibling ``<name>.tmp`` in the same directory (same filesystem, so the
rename is atomic), fsynced, then moved into place with ``os.replace``.

Append-only logs (jsonl, conversation logs) do not need this.
"""

import json
import logging
import os
from pathlib import Path
from typing import Any, Union

logger = logging.getLogger(__name__)


def atomic_write_bytes(path: Union[str, Path], data: bytes) -> None:
    """Atomically replace ``path`` with ``data``. Raises on failure
    (after cleaning up the temp file) so callers keep their existing
    error handling."""
    path = Path(path)
    tmp = path.with_name(path.name + '.tmp')
    try:
        with open(tmp, 'wb') as f:
            f.write(data)
            f.flush()
            os.fsync(f.fileno())
        os.replace(tmp, path)
    except Exception:
        try:
            tmp.unlink(missing_ok=True)
        except OSError as cleanup_err:
            logger.warning(f"atomic_write_bytes: temp cleanup failed for {tmp}: {cleanup_err}")
        raise


def atomic_write_text(path: Union[str, Path], content: str,
                      encoding: str = 'utf-8') -> None:
    """Atomically replace ``path`` with ``content``."""
    atomic_write_bytes(path, content.encode(encoding))


def atomic_write_json(path: Union[str, Path], obj: Any, indent: int = 2) -> None:
    """Atomically replace ``path`` with ``obj`` serialized as JSON."""
    atomic_write_text(path, json.dumps(obj, indent=indent))
