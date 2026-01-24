"""
Filesystem head tool.
Reads the first N lines of a text file under scenarios/<world_name>/fs.
"""
from __future__ import annotations

import itertools
import logging
from pathlib import Path
from typing import Any, Dict, Optional

logger = logging.getLogger(__name__)

try:
    from ..fs_common import (
        build_text_content,
        file_metadata,
        is_binary_file,
        resolve_fs_path,
    )
except ImportError:
    import sys
    sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
    from fs_common import (
        build_text_content,
        file_metadata,
        is_binary_file,
        resolve_fs_path,
    )


def _create_note(resource_manager, agent_name: str, content: Dict[str, Any], rel_path: str, note_name: str) -> Optional[str]:
    if not resource_manager:
        return None
    success, note_id, error_msg, _ = resource_manager.create_note(
        agent_name,
        content,
        "json",
        "fs-head",
        rel_path,
        note_name,
        {"kind": "fs-slice"}
    )
    if not success:
        logger.error(f"fs-head: failed to create Note for {rel_path}: {error_msg}")
        return None
    return note_id


def tool(input_value=None, **kwargs):
    """
    Read the first N lines of a file under scenarios/<world_name>/fs.
    """
    executor = kwargs.get("executor")
    if not executor:
        return {"status": "failed", "reason": "executor not available", "value": None, "resource_id": None}

    resource_manager = kwargs.get("resource_manager")
    agent_name = kwargs.get("agent_name", "fs-head")
    world_name = kwargs.get("world_name", "")

    path_arg = kwargs.get("path") or input_value or kwargs.get("value")
    lines = int(kwargs.get("lines", 20))
    if lines <= 0:
        return executor._create_uniform_return("failed", reason="lines must be positive")

    abs_path, rel_path = resolve_fs_path(path_arg, world_name, Path(__file__))
    if abs_path is None:
        return executor._create_uniform_return("failed", reason="fs root not available or invalid path")
    if not abs_path.exists():
        return executor._create_uniform_return("failed", reason="path not found")
    if not abs_path.is_file():
        return executor._create_uniform_return("failed", reason="fs-head requires a file path")
    if is_binary_file(abs_path):
        return executor._create_uniform_return("failed", reason="fs-head does not support binary files")

    with abs_path.open("r", encoding="utf-8", errors="replace") as f:
        chunk = list(itertools.islice(f, lines + 1))

    truncated = len(chunk) > lines
    if truncated:
        chunk = chunk[:lines]

    text = "".join(chunk)
    meta = file_metadata(abs_path, rel_path)
    meta.update({"line_start": 1, "line_end": len(chunk), "truncated": truncated})
    content = build_text_content(text, meta)
    note_name = f"{rel_path or '/'}#head:{lines}"
    note_id = _create_note(resource_manager, agent_name, content, rel_path, note_name)
    if not note_id:
        return executor._create_uniform_return("failed", reason="fs-head failed to create note")

    summary = f"head {rel_path or '/'} ({len(chunk)} lines)"
    return executor._create_uniform_return("success", value=summary, resource_id=note_id)
