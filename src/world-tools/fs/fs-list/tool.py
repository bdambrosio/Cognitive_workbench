"""
Filesystem list tool.
Lists files and directories under scenarios/<world_name>/fs.
"""
from __future__ import annotations

import logging
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

logger = logging.getLogger(__name__)

try:
    from ..fs_common import (
        build_binary_content,
        build_json_content,
        build_text_content,
        file_metadata,
        get_fs_root,
        is_binary_file,
        is_documentation_file,
        list_dir_entries,
        resolve_fs_path,
        _find_project_root,
    )
except ImportError:
    import sys
    sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
    from fs_common import (
        build_binary_content,
        build_json_content,
        build_text_content,
        file_metadata,
        get_fs_root,
        is_binary_file,
        is_documentation_file,
        list_dir_entries,
        resolve_fs_path,
        _find_project_root,
    )


def _detect_file_format(abs_path: Path) -> str:
    """
    Detect file format from extension and sample.
    Returns: 'text', 'json', 'binary', or 'pdf'
    """
    suffix_lower = abs_path.suffix.lower()
    if suffix_lower == ".pdf":
        return "pdf"
    elif suffix_lower == ".json":
        return "json"
    elif is_binary_file(abs_path):
        return "binary"
    else:
        return "text"


def _create_placeholder_note(resource_manager, agent_name: str, abs_path: Path, rel_path: str, meta: Dict[str, Any]) -> Optional[str]:
    """
    Create a placeholder Note for a file with full metadata structure.
    The content will be filled in when fs-read is called.
    """
    if not resource_manager:
        return None
    
    # Detect file format
    file_format = _detect_file_format(abs_path)
    
    # Create placeholder content structure matching fs-read format
    if file_format == "pdf":
        content = {
            "text": None,  # Placeholder - will be filled by fs-read
            "format": "pdf",
            "metadata": meta,
            "page_count": 0,  # Unknown until read
            "char_count": 0
        }
    elif file_format == "json":
        content = {
            "data": None,  # Placeholder - will be filled by fs-read
            "format": "json",
            "metadata": meta
        }
    elif file_format == "binary":
        content = {
            "format": "binary",
            "metadata": meta
        }
    else:  # text
        content = {
            "text": None,  # Placeholder - will be filled by fs-read
            "format": "text",
            "metadata": meta,
            "char_count": 0
        }
    
    success, note_id, error_msg, _ = resource_manager.create_note(
        agent_name,
        content,
        "json",
        "fs-list",
        rel_path,
        rel_path,
        {"kind": "fs-file", "placeholder": True}
    )
    if not success:
        logger.error(f"fs-list: failed to create placeholder Note for {rel_path}: {error_msg}")
        return None
    return note_id


def _create_collection(resource_manager, agent_name: str, items: List[str], rel_path: str, meta: Dict[str, Any]) -> Optional[str]:
    if not resource_manager:
        return None
    collection_name = rel_path or "/"
    success, collection_id, error_msg, _ = resource_manager.create_collection(
        agent_name,
        items,
        "list",
        "fs-list",
        rel_path or "/",
        collection_name,
        {"kind": "fs-dir", "doc_meta": meta}
    )
    if not success:
        logger.error(f"fs-list: failed to create Collection for {rel_path}: {error_msg}")
        return None
    return collection_id


def tool(input_value=None, **kwargs):
    """
    List files and directories under scenarios/<world_name>/fs.
    Returns a Collection containing Notes (files) and Collections (subdirectories).
    """
    executor = kwargs.get("executor")
    if not executor:
        return {"status": "failed", "reason": "executor not available", "value": None, "resource_id": None}

    resource_manager = kwargs.get("resource_manager")
    agent_name = kwargs.get("agent_name", "fs-list")
    world_name = kwargs.get("world_name", "") or (executor.world_name if executor else "")

    path_arg = kwargs.get("path") or input_value or kwargs.get("value")
    recursive = bool(kwargs.get("recursive", False))
    include_files = kwargs.get("include_files", True)
    include_dirs = kwargs.get("include_dirs", True)
    max_entries = kwargs.get("max_entries", 200)

    abs_path, rel_path = resolve_fs_path(path_arg, world_name, Path(__file__))
    if abs_path is None:
        fs_root = get_fs_root(world_name, Path(__file__))
        project_root = _find_project_root(Path(__file__))
        logger.error(f"fs-list: resolve_fs_path failed - world_name='{world_name}', fs_root={fs_root}, project_root={project_root}, tool_file={Path(__file__)}")
        return executor._create_uniform_return("failed", reason=f"fs root not available (world_name='{world_name}', fs_root={fs_root})")
    if not abs_path.exists():
        return executor._create_uniform_return("failed", reason="path not found")
    if not abs_path.is_dir():
        return executor._create_uniform_return("failed", reason="fs-list requires a directory path")

    entry_counter = {"count": 0}

    def build_dir(dir_path: Path, dir_rel: str) -> Tuple[Optional[str], int]:
        items: List[str] = []
        entries = list(list_dir_entries(dir_path))
        
        # Performance warning for large directories
        if len(entries) > 16:
            logger.warning(f"fs-list: Directory {dir_rel or '/'} has {len(entries)} entries (>16), this may be slow")
        
        for entry in entries:
            if max_entries is not None and entry_counter["count"] >= int(max_entries):
                break
            # Skip documentation files (metadata for planner, not content)
            if entry.is_file() and is_documentation_file(entry):
                continue
            entry_rel = f"{dir_rel}/{entry.name}" if dir_rel else entry.name
            meta = file_metadata(entry, entry_rel)
            if entry.is_dir():
                if not include_dirs:
                    continue
                if recursive:
                    child_id, _ = build_dir(entry, entry_rel)
                else:
                    child_id = _create_collection(resource_manager, agent_name, [], entry_rel, meta)
                if child_id:
                    items.append(child_id)
                    entry_counter["count"] += 1
            else:
                if not include_files:
                    continue
                # Create placeholder Note with full metadata structure
                note_id = _create_placeholder_note(resource_manager, agent_name, entry, entry_rel, meta)
                if note_id:
                    items.append(note_id)
                    entry_counter["count"] += 1

        meta = file_metadata(dir_path, dir_rel or "/")
        collection_id = _create_collection(resource_manager, agent_name, items, dir_rel, meta)
        return collection_id, len(items)

    collection_id, _ = build_dir(abs_path, rel_path)
    if not collection_id:
        return executor._create_uniform_return("failed", reason="fs-list failed to create collection")

    summary = executor._format_collection_value(collection_id)
    return executor._create_uniform_return("success", value=summary, resource_id=collection_id)
