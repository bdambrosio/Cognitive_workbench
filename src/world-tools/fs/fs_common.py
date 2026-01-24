"""
Shared helpers for filesystem tools.
"""
from __future__ import annotations

import json
import logging
from pathlib import Path
from typing import Any, Dict, Iterable, Optional, Tuple

logger = logging.getLogger(__name__)


def _find_project_root(start: Path) -> Optional[Path]:
    """Walk up from start to find project root (contains both scenarios/ and src/)."""
    current = start.resolve()
    for _ in range(10):
        # Project root should contain both scenarios/ and src/ directories
        if (current / "scenarios").exists() and (current / "src").exists():
            return current
        if current.parent == current:
            break
        current = current.parent
    return None


def get_fs_root(world_name: str, start_path: Path) -> Optional[Path]:
    """Resolve scenarios/<world_name>/fs root."""
    if not world_name:
        return None
    project_root = _find_project_root(start_path)
    if not project_root:
        return None
    return (project_root / "scenarios" / world_name / "fs").resolve()


def resolve_fs_path(path_value: Optional[str], world_name: str, start_path: Path) -> Tuple[Optional[Path], Optional[str]]:
    """
    Resolve a user path against the fs root and enforce boundary.
    Creates fs root directory if it doesn't exist.
    Returns (absolute_path, rel_path_posix).
    """
    fs_root = get_fs_root(world_name, start_path)
    if not fs_root:
        project_root = _find_project_root(start_path)
        logger.error(f"resolve_fs_path: get_fs_root returned None - world_name='{world_name}', start_path={start_path}, project_root={project_root}")
        return None, None
    if not fs_root.exists():
        try:
            fs_root.mkdir(parents=True, exist_ok=True)
            logger.info(f"resolve_fs_path: Created fs root directory: {fs_root}")
        except Exception as e:
            logger.warning(f"Failed to create fs root {fs_root}: {e}")
            return None, None
    logger.debug(f"resolve_fs_path: fs_root={fs_root}, path_value='{path_value}', world_name='{world_name}'")

    rel_input = (path_value or "").strip()
    if rel_input in ("", ".", "./"):
        candidate = fs_root
    else:
        rel_path = Path(rel_input)
        if rel_path.is_absolute():
            return None, None
        candidate = (fs_root / rel_path).resolve()

    if not _is_within_root(candidate, fs_root):
        return None, None

    rel_path_posix = candidate.relative_to(fs_root).as_posix()
    if rel_path_posix == ".":
        rel_path_posix = ""
    return candidate, rel_path_posix


def _is_within_root(path: Path, root: Path) -> bool:
    try:
        path.relative_to(root)
        return True
    except Exception:
        return False


def is_binary_file(path: Path, sample_size: int = 2048) -> bool:
    """Detect binary files by null byte heuristic."""
    try:
        with path.open("rb") as f:
            sample = f.read(sample_size)
        return b"\x00" in sample
    except Exception:
        return True


def file_metadata(path: Path, rel_path: str) -> Dict[str, Any]:
    """Collect basic file metadata."""
    try:
        stat = path.stat()
        size = int(stat.st_size)
        mtime = float(stat.st_mtime)
    except Exception:
        size = 0
        mtime = 0.0
    return {
        "name": path.name,
        "path": rel_path,
        "is_dir": path.is_dir(),
        "is_file": path.is_file(),
        "size_bytes": size,
        "mtime": mtime,
        "suffix": path.suffix
    }


def build_text_content(text: str, metadata: Dict[str, Any]) -> Dict[str, Any]:
    return {"text": text, "format": "text", "metadata": metadata, "char_count": len(text)}


def build_json_content(data: Any, metadata: Dict[str, Any]) -> Dict[str, Any]:
    return {"data": data, "format": "json", "metadata": metadata}


def build_binary_content(metadata: Dict[str, Any]) -> Dict[str, Any]:
    return {"format": "binary", "metadata": metadata}


def read_text_file(path: Path, max_chars: Optional[int] = None) -> str:
    with path.open("r", encoding="utf-8", errors="replace") as f:
        if max_chars is None:
            return f.read()
        return f.read(max_chars)


def read_json_file(path: Path) -> Optional[Any]:
    try:
        with path.open("r", encoding="utf-8") as f:
            return json.load(f)
    except Exception:
        return None


def list_dir_entries(path: Path) -> Iterable[Path]:
    return sorted(path.iterdir(), key=lambda p: (not p.is_dir(), p.name.lower()))
