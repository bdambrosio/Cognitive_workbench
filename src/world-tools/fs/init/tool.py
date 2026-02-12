"""
Filesystem initialization tool.
Builds filesystem catalog for prompt context.
"""
from __future__ import annotations

import logging
from pathlib import Path
from typing import Any, Dict, List, Optional

logger = logging.getLogger(__name__)

try:
    from ..fs_common import (
        get_fs_root,
        list_dir_entries,
        file_metadata,
        is_binary_file,
    )
except ImportError:
    import sys
    sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
    from fs_common import (
        get_fs_root,
        list_dir_entries,
        file_metadata,
        is_binary_file,
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


def _read_directory_doc(dir_path: Path) -> Optional[str]:
    """
    Read directory documentation file if present.
    Checks for Skill.md, README.md, or DIRECTORY.md.
    """
    for doc_name in ["Skill.md", "README.md", "DIRECTORY.md"]:
        doc_path = dir_path / doc_name
        if doc_path.exists() and doc_path.is_file():
            try:
                return doc_path.read_text(encoding="utf-8", errors="replace")
            except Exception as e:
                logger.debug(f"Failed to read {doc_path}: {e}")
    return None


def _format_directory_entry(entry: Path, rel_path: str, depth: int = 0, max_depth: int = 2, max_entries: int = 50) -> List[str]:
    """
    Format a directory or file entry for the catalog.
    Returns list of lines.
    """
    lines = []
    indent = "  " * depth
    
    meta = file_metadata(entry, rel_path)
    
    if entry.is_dir():
        lines.append(f"{indent}- {entry.name}/ (directory)")
        
        # Read directory doc if available
        if depth < max_depth:
            doc_content = _read_directory_doc(entry)
            if doc_content:
                # Extract first paragraph or first few lines
                doc_lines = doc_content.split("\n")
                summary_lines = []
                for line in doc_lines[:5]:  # First 5 lines
                    line = line.strip()
                    if line and not line.startswith("#"):
                        summary_lines.append(line)
                        if len(summary_lines) >= 3:  # Max 3 lines
                            break
                if summary_lines:
                    for doc_line in summary_lines:
                        lines.append(f"{indent}  {doc_line}")
    else:
        file_format = _detect_file_format(entry)
        size_kb = meta.get("size_bytes", 0) / 1024
        size_str = f"{size_kb:.1f}KB" if size_kb >= 1 else f"{meta.get('size_bytes', 0)}B"
        lines.append(f"{indent}- {entry.name} ({file_format}, {size_str})")
    
    return lines


def build_filesystem_catalog(world_name: str, executor) -> str:
    """
    Build filesystem catalog text for prompt context.
    
    Performs lightweight directory scan and includes directory documentation
    where available (Skill.md, README.md, DIRECTORY.md).
    
    Args:
        world_name: World name (e.g., "fs")
        executor: InfospaceExecutor instance
        
    Returns:
        Formatted catalog text string
    """
    if not world_name:
        return ""
    
    fs_root = get_fs_root(world_name, Path(__file__))
    if not fs_root or not fs_root.exists():
        logger.debug(f"fs init: fs_root not available: {fs_root}")
        return ""
    
    lines = []
    lines.append("Filesystem Sandbox Structure:")
    lines.append("=" * 50)
    lines.append("")
    lines.append(f"Root: scenarios/{world_name}/fs/")
    lines.append("")
    
    # Lightweight scan: limit depth and entries
    max_depth = 2
    max_entries_per_dir = 30
    
    def scan_directory(dir_path: Path, rel_path: str, depth: int = 0) -> int:
        """Recursively scan directory, return count of entries processed."""
        if depth > max_depth:
            return 0
        
        entries = list(list_dir_entries(dir_path))
        if len(entries) > max_entries_per_dir:
            logger.warning(f"fs init: Directory {rel_path} has {len(entries)} entries (>30), truncating")
            entries = entries[:max_entries_per_dir]
        
        entry_count = 0
        for entry in entries:
            if entry_count >= max_entries_per_dir:
                break
            
            entry_rel = f"{rel_path}/{entry.name}" if rel_path else entry.name
            entry_lines = _format_directory_entry(entry, entry_rel, depth, max_depth, max_entries_per_dir)
            lines.extend(entry_lines)
            entry_count += 1
            
            # Recurse into subdirectories
            if entry.is_dir() and depth < max_depth:
                sub_count = scan_directory(entry, entry_rel, depth + 1)
                entry_count += sub_count
        
        return entry_count
    
    try:
        total_entries = scan_directory(fs_root, "")
        lines.append("")
        lines.append(f"Total entries scanned: {total_entries}")
        lines.append("")
        lines.append("Note: Use fs-list, fs-find, fs-read to explore filesystem in detail.")
    except Exception as e:
        logger.error(f"fs init: Error building filesystem catalog: {e}")
        lines.append("")
        lines.append(f"Error building catalog: {str(e)}")
    
    return "\n".join(lines)


def tool(input_value=None, **kwargs):
    """
    Initialize filesystem world by building catalog for prompt context.
    
    This tool is automatically executed during executor initialization.
    It scans the filesystem sandbox and builds a catalog that will be
    included in planner prompts to give the agent awareness of available files.
    
    Returns:
        Uniform return format dict with success status.
    """
    executor = kwargs.get("executor")
    if not executor:
        return {
            "status": "failed",
            "reason": "executor not available",
            "value": None,
            "resource_id": None
        }
    
    world_name = kwargs.get("world_name", "")
    if not world_name:
        world_name = executor.world_name
    
    try:
        # Build filesystem catalog
        fs_catalog = build_filesystem_catalog(world_name, executor)
        
        # Store in world_state for executor to retrieve generically
        executor.set_world_state("_prompt_sections", {
            "filesystem_catalog": fs_catalog
        })
        
        logger.info(f"fs init: Built filesystem catalog ({len(fs_catalog)} chars)")
        return executor._create_uniform_return(
            "success",
            value=f"Filesystem catalog built ({len(fs_catalog)} chars)",
            resource_id=None
        )
    except Exception as e:
        logger.error(f"fs init: Failed to build catalog: {e}")
        return executor._create_uniform_return(
            "failed",
            reason=f"Failed to build filesystem catalog: {str(e)}"
        )
