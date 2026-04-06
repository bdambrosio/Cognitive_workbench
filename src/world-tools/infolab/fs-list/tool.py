"""
Filesystem list tool.
Lists files and directories under scenarios/<world_name>/fs.
Returns a single Note with a text listing (like ls).
"""
from __future__ import annotations

import logging
from pathlib import Path
from typing import Any, Dict, List, Optional

logger = logging.getLogger(__name__)

try:
    from ..fs_common import (
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
        file_metadata,
        get_fs_root,
        is_binary_file,
        is_documentation_file,
        list_dir_entries,
        resolve_fs_path,
        _find_project_root,
    )


def _format_size(size_bytes: int) -> str:
    """Format file size for display."""
    if size_bytes < 1024:
        return f"{size_bytes}B"
    elif size_bytes < 1024 * 1024:
        return f"{size_bytes / 1024:.1f}K"
    else:
        return f"{size_bytes / (1024 * 1024):.1f}M"


def _build_listing(dir_path: Path, dir_rel: str, recursive: bool,
                   include_files: bool, include_dirs: bool,
                   max_entries: int) -> List[str]:
    """Build text listing lines for a directory."""
    lines = []
    count = 0
    entries = list(list_dir_entries(dir_path))

    for entry in entries:
        if count >= max_entries:
            lines.append(f"... (truncated at {max_entries} entries)")
            break

        if entry.is_file() and is_documentation_file(entry):
            continue

        entry_rel = f"{dir_rel}/{entry.name}" if dir_rel else entry.name

        if entry.is_dir():
            if not include_dirs:
                continue
            lines.append(f"{entry.name}/")
            count += 1
            if recursive:
                sub_lines = _build_listing(entry, entry_rel, recursive,
                                           include_files, include_dirs,
                                           max_entries - count)
                for sl in sub_lines:
                    lines.append(f"  {sl}")
                    count += 1
                    if count >= max_entries:
                        break
        else:
            if not include_files:
                continue
            meta = file_metadata(entry, entry_rel)
            size_str = _format_size(meta["size_bytes"])
            lines.append(f"{entry.name}  ({size_str})")
            count += 1

    return lines


def tool(input_value=None, **kwargs):
    """
    List files and directories under scenarios/<world_name>/fs.
    Returns a Note with a text listing.
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
    max_entries = int(kwargs.get("max_entries", 200))

    abs_path, rel_path = resolve_fs_path(path_arg, world_name, Path(__file__))
    if abs_path is None:
        fs_root = get_fs_root(world_name, Path(__file__))
        project_root = _find_project_root(Path(__file__))
        logger.error(f"fs-list: resolve_fs_path failed - world_name='{world_name}', fs_root={fs_root}, project_root={project_root}")
        return executor._create_uniform_return("failed", reason=f"fs root not available (world_name='{world_name}', fs_root={fs_root})")
    if not abs_path.exists():
        return executor._create_uniform_return("failed", reason="path not found")
    if not abs_path.is_dir():
        return executor._create_uniform_return("failed", reason="fs-list requires a directory path")

    lines = _build_listing(abs_path, rel_path, recursive,
                           include_files, include_dirs, max_entries)

    display_path = rel_path or "/"
    listing_text = f"{display_path}\n" + "\n".join(lines) if lines else f"{display_path}\n(empty)"

    # Create a single Note with the listing
    if not resource_manager:
        return executor._create_uniform_return("success", value=listing_text)

    success, note_id, error_msg, _ = resource_manager.create_note(
        agent_name, listing_text, "text", "fs-list",
        display_path, display_path, {}
    )
    if not success:
        return executor._create_uniform_return("failed", reason=f"Failed to create listing Note: {error_msg}")

    return executor._create_uniform_return("success", value=listing_text, resource_id=note_id)
