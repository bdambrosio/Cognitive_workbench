"""
Filesystem grep tool.
Searches text files under scenarios/<world_name>/fs and returns match snippets.
"""
from __future__ import annotations

import logging
import re
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

logger = logging.getLogger(__name__)

try:
    from ..fs_common import (
        build_text_content,
        file_metadata,
        get_fs_root,
        is_binary_file,
        is_documentation_file,
        list_dir_entries,
        read_text_file,
        resolve_fs_path,
    )
except ImportError:
    import sys
    sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
    from fs_common import (
        build_text_content,
        file_metadata,
        get_fs_root,
        is_binary_file,
        is_documentation_file,
        list_dir_entries,
        read_text_file,
        resolve_fs_path,
    )


def _create_note(resource_manager, agent_name: str, content: Dict[str, Any], rel_path: str, note_name: str) -> Optional[str]:
    if not resource_manager:
        return None
    success, note_id, error_msg, _ = resource_manager.create_note(
        agent_name,
        content,
        "json",
        "fs-grep",
        rel_path,
        note_name,
        {"kind": "fs-grep"}
    )
    if not success:
        logger.error(f"fs-grep: failed to create Note for {rel_path}: {error_msg}")
        return None
    return note_id


def _create_collection(resource_manager, agent_name: str, items: List[str], name: str, meta: Dict[str, Any]) -> Optional[str]:
    if not resource_manager:
        return None
    success, collection_id, error_msg, _ = resource_manager.create_collection(
        agent_name,
        items,
        "list",
        "fs-grep",
        name,
        name,
        {"kind": "fs-grep", "doc_meta": meta}
    )
    if not success:
        logger.error(f"fs-grep: failed to create Collection {name}: {error_msg}")
        return None
    return collection_id


def _iter_files(root: Path, recursive: bool) -> List[Path]:
    files: List[Path] = []
    if root.is_file():
        # Skip documentation files
        if is_documentation_file(root):
            return []
        return [root]
    if recursive:
        for path in root.rglob("*"):
            if path.is_file() and not is_documentation_file(path):
                files.append(path)
    else:
        for path in list_dir_entries(root):
            if path.is_file() and not is_documentation_file(path):
                files.append(path)
    return files


def tool(input_value=None, **kwargs):
    """
    Grep text files under scenarios/<world_name>/fs.
    Returns a Collection of Notes, one per match.
    """
    executor = kwargs.get("executor")
    if not executor:
        return {"status": "failed", "reason": "executor not available", "value": None, "resource_id": None}

    resource_manager = kwargs.get("resource_manager")
    agent_name = kwargs.get("agent_name", "fs-grep")
    world_name = kwargs.get("world_name", "")

    path_arg = kwargs.get("path") or input_value or kwargs.get("value")
    pattern = kwargs.get("pattern")
    if not pattern:
        return executor._create_uniform_return("failed", reason="pattern is required")

    recursive = bool(kwargs.get("recursive", True))
    max_matches = int(kwargs.get("max_matches", 20))
    context_lines = int(kwargs.get("context", 0))
    case_insensitive = bool(kwargs.get("case_insensitive", False))
    max_file_bytes = int(kwargs.get("max_file_bytes", 2000000))

    abs_path, rel_path = resolve_fs_path(path_arg, world_name, Path(__file__))
    if abs_path is None:
        return executor._create_uniform_return("failed", reason="fs root not available or invalid path")
    if not abs_path.exists():
        return executor._create_uniform_return("failed", reason="path not found")

    flags = re.IGNORECASE if case_insensitive else 0
    try:
        regex = re.compile(pattern, flags=flags)
    except re.error as e:
        return executor._create_uniform_return("failed", reason=f"invalid pattern: {e}")

    note_ids: List[str] = []
    matches = 0
    files_scanned = 0

    fs_root = get_fs_root(world_name, Path(__file__))
    for file_path in _iter_files(abs_path, recursive):
        if max_matches and matches >= max_matches:
            break
        try:
            if file_path.stat().st_size > max_file_bytes:
                continue
        except Exception:
            continue
        if is_binary_file(file_path):
            continue

        try:
            text = read_text_file(file_path)
        except Exception:
            continue

        files_scanned += 1
        lines = text.splitlines()

        for idx, line in enumerate(lines):
            if max_matches and matches >= max_matches:
                break
            if not regex.search(line):
                continue

            start = max(0, idx - context_lines)
            end = min(len(lines), idx + context_lines + 1)
            snippet = "\n".join(lines[start:end])

            rel = None
            if fs_root:
                try:
                    rel = file_path.relative_to(fs_root).as_posix()
                except Exception:
                    rel = None
            if not rel:
                root = abs_path if abs_path.is_dir() else abs_path.parent
                rel = file_path.relative_to(root).as_posix() if root in file_path.parents or root == file_path else file_path.name
            meta = file_metadata(file_path, rel)
            meta.update({"line": idx + 1, "pattern": pattern, "context": context_lines})
            content = build_text_content(snippet, meta)
            note_name = f"{rel}#L{idx + 1}"
            note_id = _create_note(resource_manager, agent_name, content, rel, note_name)
            if note_id:
                note_ids.append(note_id)
                matches += 1

    collection_meta = {"pattern": pattern, "path": rel_path or "/", "matches": matches, "files_scanned": files_scanned}
    collection_name = f"fs-grep:{pattern}"
    collection_id = _create_collection(resource_manager, agent_name, note_ids, collection_name, collection_meta)
    if not collection_id:
        return executor._create_uniform_return("failed", reason="fs-grep failed to create collection")

    summary = executor._format_collection_value(collection_id)
    return executor._create_uniform_return("success", value=summary, resource_id=collection_id)
