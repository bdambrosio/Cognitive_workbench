"""
Filesystem find tool.
Find files by filename pattern (glob) under scenarios/<world_name>/fs.
"""
from __future__ import annotations

import fnmatch
import logging
from pathlib import Path
from typing import Any, Dict, List, Optional

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
        "fs-find",
        rel_path,
        rel_path,
        {"kind": "fs-file", "placeholder": True}
    )
    if not success:
        logger.error(f"fs-find: failed to create placeholder Note for {rel_path}: {error_msg}")
        return None
    return note_id


def _create_collection(resource_manager, agent_name: str, items: List[str], pattern: str) -> Optional[str]:
    if not resource_manager:
        return None
    collection_name = f"fs-find:{pattern}"
    success, collection_id, error_msg, _ = resource_manager.create_collection(
        agent_name,
        items,
        "list",
        "fs-find",
        pattern,
        collection_name,
        {"kind": "fs-find", "pattern": pattern}
    )
    if not success:
        logger.error(f"fs-find: failed to create Collection for pattern {pattern}: {error_msg}")
        return None
    return collection_id


def _iter_files(root: Path, recursive: bool) -> List[Path]:
    """Iterate all files under root, optionally recursively."""
    files = []
    if recursive:
        for file_path in root.rglob("*"):
            if file_path.is_file():
                files.append(file_path)
    else:
        for entry in root.iterdir():
            if entry.is_file():
                files.append(entry)
    return files


def tool(input_value=None, **kwargs):
    """
    Find files by filename pattern (glob) under scenarios/<world_name>/fs.
    Returns a Collection containing Notes (matching files).
    """
    executor = kwargs.get("executor")
    if not executor:
        return {"status": "failed", "reason": "executor not available", "value": None, "resource_id": None}

    resource_manager = kwargs.get("resource_manager")
    agent_name = kwargs.get("agent_name", "fs-find")
    world_name = kwargs.get("world_name", "") or (executor.world_name if executor else "")

    pattern_arg = kwargs.get("pattern") or input_value or kwargs.get("value")
    if not pattern_arg:
        return executor._create_uniform_return("failed", reason="pattern is required")
    
    pattern = str(pattern_arg).strip()
    
    # Reject patterns with path separators (use path parameter instead)
    if '/' in pattern or '\\' in pattern:
        return executor._create_uniform_return("failed", reason="pattern cannot contain path separators; use 'path' parameter to search in specific directory")
    
    path_arg = kwargs.get("path", ".")
    recursive = bool(kwargs.get("recursive", True))
    max_results = int(kwargs.get("max_results", 200))

    abs_path, rel_path = resolve_fs_path(path_arg, world_name, Path(__file__))
    if abs_path is None:
        fs_root = get_fs_root(world_name, Path(__file__))
        project_root = _find_project_root(Path(__file__))
        logger.error(f"fs-find: resolve_fs_path failed - world_name='{world_name}', fs_root={fs_root}, project_root={project_root}, tool_file={Path(__file__)}")
        return executor._create_uniform_return("failed", reason=f"fs root not available (world_name='{world_name}', fs_root={fs_root})")
    if not abs_path.exists():
        return executor._create_uniform_return("failed", reason="path not found")
    if not abs_path.is_dir():
        return executor._create_uniform_return("failed", reason="fs-find requires a directory path")

    # Get fs_root for relative path calculation
    fs_root = get_fs_root(world_name, Path(__file__))
    if not fs_root:
        return executor._create_uniform_return("failed", reason="fs root not available")

    # Find matching files
    note_ids = []
    files = _iter_files(abs_path, recursive)
    
    for file_path in files:
        if len(note_ids) >= max_results:
            break
        
        # Skip documentation files (metadata for planner, not content)
        if is_documentation_file(file_path):
            continue
        
        # Match filename only (not full path)
        filename = file_path.name
        if not fnmatch.fnmatch(filename, pattern):
            continue
        
        # Calculate relative path from fs_root
        try:
            entry_rel = file_path.relative_to(fs_root).as_posix()
        except ValueError:
            # File outside fs_root (shouldn't happen, but handle gracefully)
            continue
        
        meta = file_metadata(file_path, entry_rel)
        # Create placeholder Note with full metadata structure
        note_id = _create_placeholder_note(resource_manager, agent_name, file_path, entry_rel, meta)
        if note_id:
            note_ids.append(note_id)

    # Create Collection
    collection_id = _create_collection(resource_manager, agent_name, note_ids, pattern)
    if not collection_id:
        return executor._create_uniform_return("failed", reason="fs-find failed to create collection")

    summary = executor._format_collection_value(collection_id)
    return executor._create_uniform_return("success", value=summary, resource_id=collection_id)
