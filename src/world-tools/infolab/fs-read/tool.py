"""
Filesystem read tool.
Reads text, JSON, or PDF files under scenarios/<world_name>/fs.
"""
from __future__ import annotations

import json
import logging
from datetime import datetime
from pathlib import Path
from typing import Any, Dict, Optional, Tuple

logger = logging.getLogger(__name__)

try:
    import pymupdf
    HAS_PYMUPDF = True
except ImportError:
    HAS_PYMUPDF = False

try:
    from utils.grobid import parse_pdf_grobid
    HAS_GROBID = True
except ImportError:
    HAS_GROBID = False

try:
    from ..fs_common import (
        file_metadata,
        get_fs_root,
        is_binary_file,
        read_json_file,
        read_text_file,
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
        read_json_file,
        read_text_file,
        resolve_fs_path,
        _find_project_root,
    )


def _extract_pdf_text(pdf_path: Path, grobid_url: Optional[str] = None, pdf_parser: Optional[str] = None) -> Dict[str, Any]:
    """
    Extract text from PDF file using GROBID (if available) or pymupdf fallback.
    When pdf_parser is "pymupdf", use pymupdf only (skip GROBID even if available).
    Returns dict with text, format, metadata, page_count, char_count.
    """
    if not HAS_PYMUPDF:
        raise ValueError("pymupdf not available for PDF extraction")
    
    # Read PDF bytes
    with pdf_path.open("rb") as f:
        content = f.read()
    
    # Use pymupdf only when pdf_parser is explicitly "pymupdf"
    use_pymupdf_only = (pdf_parser or "").lower() == "pymupdf"
    
    # Try GROBID first if available and not forcing pymupdf (use grobid_url from config or module default)
    if HAS_GROBID and not use_pymupdf_only:
        try:
            # Extract title from filename
            title = pdf_path.stem or "document"
            
            # Parse with GROBID
            grobid_result = parse_pdf_grobid(pdf_filepath=str(pdf_path), title=title, grobid_url=grobid_url)
            
            if grobid_result:
                # Concatenate chunks with section headers
                chunks = grobid_result.get('chunks', [])
                if chunks:
                    chunk_texts = []
                    for section_title, section_text in chunks:
                        chunk_texts.append(f"{section_title}\n{section_text}")
                    full_text = "\n\n".join(chunk_texts)
                elif grobid_result.get('abstract'):
                    full_text = grobid_result['abstract']
                else:
                    full_text = ""
                
                result = {
                    "text": full_text,
                    "format": "pdf",
                    "metadata": {
                        "pdf_metadata": {
                            "title": grobid_result.get("title", ""),
                            "author": grobid_result.get("authors", ""),
                            "subject": "",
                            "creator": ""
                        }
                    },
                    "page_count": len(chunks) if chunks else 0,
                    "char_count": len(full_text)
                }
                
                logger.info(f"Extracted {len(full_text)} chars from PDF using GROBID ({len(chunks)} chunks)")
                return result
        except Exception as e:
            logger.warning(f"GROBID parsing failed, falling back to pymupdf: {str(e)}")
            # Fall through to pymupdf extraction
    
    # Fallback to pymupdf extraction
    try:
        doc = pymupdf.open(stream=content, filetype="pdf")
        pages_text = []
        for page_num in range(len(doc)):
            page = doc[page_num]
            text = page.get_text()
            pages_text.append(text)
        
        full_text = "\n\n".join(pages_text)
        metadata = doc.metadata or {}
        page_count = len(doc)
        doc.close()
        
        result = {
            "text": full_text,
            "format": "pdf",
            "metadata": {
                "pdf_metadata": {
                    "title": metadata.get("title", ""),
                    "author": metadata.get("author", ""),
                    "subject": metadata.get("subject", ""),
                    "creator": metadata.get("creator", "")
                }
            },
            "page_count": page_count,
            "char_count": len(full_text)
        }
        
        logger.info(f"Extracted {len(full_text)} chars from {page_count} page PDF using pymupdf")
        return result
    except Exception as e:
        logger.error(f"PDF extraction failed: {str(e)}")
        raise ValueError(f"PDF extraction failed: {str(e)}")


def _find_existing_note(resource_manager, rel_path: str, file_mtime: float, max_chars: Optional[int]) -> Tuple[Optional[str], bool]:
    """
    Find existing Note for a file path, verifying it's still valid.
    
    Args:
        resource_manager: Resource manager instance
        rel_path: Relative file path
        file_mtime: File modification time
        max_chars: Requested max_chars (None for full read)
        
    Returns:
        Tuple of (Note ID if found, is_placeholder flag)
        Returns (None, False) if no Note found
    """
    if not resource_manager:
        return None, False
    
    # Accept Notes created by fs-read, fs-list, or fs-find
    valid_source_skills = {'fs-read', 'fs-list', 'fs-find'}
    
    # Search resource_registry for matching Notes
    for note_id, resource_data in resource_manager.resource_registry.items():
        if not note_id.startswith('Note_'):
            continue
        
        props = resource_data.get('properties', {})
        
        # Check if this Note matches our file (by path and kind)
        if (props.get('source_skill') in valid_source_skills and 
            props.get('source_value') == rel_path and
            props.get('kind') == 'fs-file'):
            
            # Verify Note still exists in resource manager
            existing_resource = resource_manager.get_resource(note_id)
            if not existing_resource:
                continue
            
            # Check if this is a placeholder Note
            is_placeholder = props.get('placeholder', False)
            tool_meta = props.get('tool_metadata', {})
            
            # If it's a placeholder, we can always update it (don't check mtime)
            if is_placeholder:
                logger.debug(f"fs-read: found placeholder Note {note_id} for {rel_path}")
                return note_id, True
            
            # For filled Notes, check if max_chars differs (cache miss if different)
            if max_chars is not None and tool_meta.get('truncated'):
                # Cached Note was truncated, re-read to match requested max_chars
                continue
            
            # Check file modification time vs Note creation time
            created_at_str = props.get('created_at', '')
            if created_at_str:
                try:
                    created_at = datetime.fromisoformat(created_at_str).timestamp()
                    # If file is newer than Note creation, cache is stale
                    if file_mtime > created_at:
                        logger.debug(f"fs-read: cache miss for {rel_path} - file modified after Note creation")
                        continue
                except (ValueError, TypeError):
                    # Invalid timestamp, skip cache
                    continue
            
            # Cache hit - return existing filled Note
            logger.debug(f"fs-read: cache hit for {rel_path} - returning existing Note {note_id}")
            return note_id, False
    
    return None, False


def _create_note(resource_manager, agent_name: str, text_content: str, rel_path: str, tool_metadata: Optional[Dict] = None) -> Optional[str]:
    if not resource_manager:
        return None
    success, note_id, error_msg, _ = resource_manager.create_note(
        agent_name,
        text_content,
        "text",
        "fs-read",
        rel_path,
        rel_path,
        {"kind": "fs-file", "tool_metadata": tool_metadata or {}}
    )
    if not success:
        logger.error(f"fs-read: failed to create Note for {rel_path}: {error_msg}")
        return None
    return note_id


def tool(input_value=None, **kwargs):
    """
    Read a file under scenarios/<world_name>/fs.
    Returns a text Note with file content; metadata is linked separately.
    """
    executor = kwargs.get("executor")
    if not executor:
        return {"status": "failed", "reason": "executor not available", "value": None, "resource_id": None}

    resource_manager = kwargs.get("resource_manager")
    agent_name = kwargs.get("agent_name", "fs-read")
    world_name = kwargs.get("world_name", "") or (executor.world_name if executor else "")
    grobid_url = kwargs.get("grobid_url")
    pdf_parser = kwargs.get("pdf_parser")

    path_arg = kwargs.get("path") or input_value or kwargs.get("value")
    max_chars = kwargs.get("max_chars")
    force_json = bool(kwargs.get("as_json", False))

    abs_path, rel_path = resolve_fs_path(path_arg, world_name, Path(__file__))
    if abs_path is None:
        fs_root = get_fs_root(world_name, Path(__file__))
        project_root = _find_project_root(Path(__file__))
        logger.error(f"fs-read: resolve_fs_path failed - world_name='{world_name}', fs_root={fs_root}, project_root={project_root}, path_arg='{path_arg}'")
        return executor._create_uniform_return("failed", reason=f"fs root not available or invalid path (world_name='{world_name}')")
    if not abs_path.exists():
        return executor._create_uniform_return("failed", reason="path not found")
    if not abs_path.is_file():
        return executor._create_uniform_return("failed", reason="fs-read requires a file path")

    meta = file_metadata(abs_path, rel_path)
    file_mtime = meta.get('mtime', 0.0)
    
    # Check for existing Note (filled or placeholder)
    existing_note_id, is_placeholder = _find_existing_note(resource_manager, rel_path, file_mtime, max_chars)
    
    # If filled Note exists and is valid, return it
    if existing_note_id and not is_placeholder:
        summary = f"read {rel_path or '/'} (cached)"
        return executor._create_uniform_return("success", value=summary, resource_id=existing_note_id)
    
    # Save original placeholder state if updating
    original_content = None
    original_placeholder_flag = None
    if existing_note_id and is_placeholder:
        existing_note = resource_manager.get_resource(existing_note_id)
        if existing_note:
            props = existing_note.get('properties', {})
            original_content = props.get('content')
            original_placeholder_flag = props.get('placeholder', False)

    # Read file content (text-only; metadata in tool_metadata)
    try:
        text_content = ""
        tool_meta = dict(meta)
        
        if abs_path.suffix.lower() == ".pdf":
            if not HAS_PYMUPDF:
                return executor._create_uniform_return("failed", reason="pymupdf not available for PDF extraction")
            try:
                pdf_result = _extract_pdf_text(abs_path, grobid_url=grobid_url, pdf_parser=pdf_parser)
                text_content = pdf_result.get("text", "") or ""
                pdf_meta = pdf_result.get("metadata", {})
                tool_meta.update({"format": "pdf", "page_count": pdf_result.get("page_count", 0),
                                 "pdf_metadata": pdf_meta.get("pdf_metadata", {})})
                for k, v in pdf_meta.items():
                    if k != "pdf_metadata":
                        tool_meta[k] = v
                if max_chars and len(text_content) > int(max_chars):
                    text_content = text_content[:int(max_chars)]
                    tool_meta["truncated"] = True
            except Exception as e:
                logger.error(f"fs-read: PDF extraction failed for {rel_path}: {e}")
                return executor._create_uniform_return("failed", reason=f"PDF extraction failed: {str(e)}")
        elif is_binary_file(abs_path):
            text_content = "(binary file)"
            tool_meta["format"] = "binary"
        else:
            if force_json or abs_path.suffix.lower() == ".json":
                data = read_json_file(abs_path)
                if data is not None:
                    text_content = json.dumps(data, ensure_ascii=False)
                    tool_meta["format"] = "json"
                else:
                    text_content = read_text_file(abs_path, max_chars=int(max_chars) if max_chars else None)
                    tool_meta["format"] = "text"
                    if max_chars is not None and len(text_content) >= int(max_chars):
                        tool_meta["truncated"] = True
            else:
                text_content = read_text_file(abs_path, max_chars=int(max_chars) if max_chars else None)
                tool_meta["format"] = "text"
                if max_chars is not None and len(text_content) >= int(max_chars):
                    tool_meta["truncated"] = True
        
        # Update existing placeholder Note or create new Note
        if existing_note_id and is_placeholder:
            success, error_msg = resource_manager.update_note_content(
                existing_note_id, text_content, reindex=True, tool_metadata=tool_meta
            )
            if not success:
                logger.error(f"fs-read: failed to update placeholder Note {existing_note_id}: {error_msg}")
                # Revert to original placeholder state
                if original_content is not None:
                    resource_manager.update_note_content(existing_note_id, original_content, reindex=False)
                    # Restore placeholder flag
                    note_data = resource_manager.resource_registry.get(existing_note_id)
                    if note_data and original_placeholder_flag is not None:
                        note_data['properties']['placeholder'] = original_placeholder_flag
                return executor._create_uniform_return("failed", reason=f"Failed to update Note: {error_msg}")
            
            # Remove placeholder flag
            note_data = resource_manager.resource_registry.get(existing_note_id)
            if note_data:
                note_data['properties'].pop('placeholder', None)
            
            summary = f"read {rel_path or '/'}"
            return executor._create_uniform_return("success", value=summary, resource_id=existing_note_id)
        else:
            note_id = _create_note(resource_manager, agent_name, text_content, rel_path, tool_metadata=tool_meta)
            if not note_id:
                return executor._create_uniform_return("failed", reason="fs-read failed to create note")
            
            summary = f"read {rel_path or '/'}"
            return executor._create_uniform_return("success", value=summary, resource_id=note_id)
    
    except Exception as e:
        logger.error(f"fs-read: error reading {rel_path}: {e}")
        # Revert to original placeholder state if we were updating
        if existing_note_id and is_placeholder and original_content is not None:
            resource_manager.update_note_content(existing_note_id, original_content, reindex=False)
            # Restore placeholder flag
            note_data = resource_manager.resource_registry.get(existing_note_id)
            if note_data and original_placeholder_flag is not None:
                note_data['properties']['placeholder'] = original_placeholder_flag
            logger.info(f"fs-read: reverted {existing_note_id} to placeholder state after error")
        return executor._create_uniform_return("failed", reason=f"Error reading file: {str(e)}")
