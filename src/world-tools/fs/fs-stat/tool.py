"""
Filesystem stat tool.
Returns metadata for a file or directory under scenarios/<world_name>/fs.
"""
from __future__ import annotations

import logging
from pathlib import Path
from typing import Any, Dict, Optional

logger = logging.getLogger(__name__)

try:
    from ..fs_common import (
        build_json_content,
        file_metadata,
        resolve_fs_path,
    )
except ImportError:
    import sys
    sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
    from fs_common import (
        build_json_content,
        file_metadata,
        resolve_fs_path,
    )


def _create_note(resource_manager, agent_name: str, content: Dict[str, Any], rel_path: str, note_name: str) -> Optional[str]:
    if not resource_manager:
        return None
    success, note_id, error_msg, _ = resource_manager.create_note(
        agent_name,
        content,
        "json",
        "fs-stat",
        rel_path,
        note_name,
        {"kind": "fs-stat"}
    )
    if not success:
        logger.error(f"fs-stat: failed to create Note for {rel_path}: {error_msg}")
        return None
    return note_id


def tool(input_value=None, **kwargs):
    """
    Return metadata for a file or directory under scenarios/<world_name>/fs.
    """
    executor = kwargs.get("executor")
    if not executor:
        return {"status": "failed", "reason": "executor not available", "value": None, "resource_id": None}

    resource_manager = kwargs.get("resource_manager")
    agent_name = kwargs.get("agent_name", "fs-stat")
    world_name = kwargs.get("world_name", "")

    path_arg = kwargs.get("path") or input_value or kwargs.get("value")
    abs_path, rel_path = resolve_fs_path(path_arg, world_name, Path(__file__))
    if abs_path is None:
        return executor._create_uniform_return("failed", reason="fs root not available or invalid path")
    if not abs_path.exists():
        return executor._create_uniform_return("failed", reason="path not found")

    meta = file_metadata(abs_path, rel_path)
    content = build_json_content(meta, {"path": rel_path})
    note_name = f"{rel_path or '/'}#stat"
    note_id = _create_note(resource_manager, agent_name, content, rel_path, note_name)
    if not note_id:
        return executor._create_uniform_return("failed", reason="fs-stat failed to create note")

    summary = f"stat {rel_path or '/'}"
    return executor._create_uniform_return("success", value=summary, resource_id=note_id)
