#!/usr/bin/env python3
"""
Extract bibliography/references from PDF files using GROBID.
"""
import logging
import os
from pathlib import Path
from typing import Any, Dict, List, Optional
from infospace_executor import InfospaceExecutor

import sys
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '../..')))
from utils.grobid import extract_references_grobid

logger = logging.getLogger(__name__)


def _fail(executor: InfospaceExecutor, reason: str, value: Optional[str] = None, extra: Optional[Dict[str, Any]] = None):
    return executor._create_uniform_return(
        "failed",
        value=value or reason,
        reason=reason,
        extra=extra,
    )


def _success(
    executor: InfospaceExecutor,
    value: str,
    resource_id: Optional[str],
    extra: Optional[Dict[str, Any]] = None,
):
    return executor._create_uniform_return("success", value=value, resource_id=resource_id, extra=extra)


def _create_note(content: Any, agent_name: str, resource_manager, source_skill: str = 'extract-references') -> Optional[str]:
    """Create a Note via resource_manager and return its ID."""
    if not resource_manager:
        logger.error("resource_manager required for creating Notes")
        return None
    
    success, note_id, error_msg, _ = resource_manager.create_note(
        character_name=agent_name,
        content=content,
        format_type='json' if isinstance(content, dict) else 'text',
        source_skill=source_skill,
        source_value=str(content)[:100],
        note_name='',
        extra_props={}
    )
    
    if success:
        return note_id
    else:
        logger.error(f"Failed to create Note: {error_msg}")
        return None


def _create_collection(note_ids: List[str], agent_name: str, resource_manager, source_skill: str = 'extract-references') -> Optional[str]:
    """Create a Collection via resource_manager and return its ID."""
    if not note_ids:
        note_ids = []
    
    if not resource_manager:
        logger.error("resource_manager required for creating Collections")
        return None
    
    success, collection_id, error_msg, _ = resource_manager.create_collection(
        agent_name,
        note_ids,
        'list',
        f'{len(note_ids)} references',
        '',
        '',
        {}
    )
    
    if success:
        logger.info(f"Created Collection {collection_id} with {len(note_ids)} references")
        return collection_id
    else:
        logger.error(f"Failed to create Collection: {error_msg}")
        return None


def tool(input_value=None, runtime=None, **kwargs):
    """
    Extract bibliography/references from PDF file.
    
    Args:
        input_value: PDF file path (absolute) or Note ID containing PDF content
        **kwargs:
            - path: PDF file path (if input_value not provided)
            - grobid_url: Optional GROBID server URL (from world_config)
            - agent_name: Required agent name
            - resource_manager: Required resource manager
            - executor: Required executor
            
    Returns:
        Collection ID containing Notes (one per reference)
    """
    executor: InfospaceExecutor = kwargs.get("executor")
    if not executor:
        return {"status": "failed", "reason": "executor not available", "value": None, "resource_id": None}
    
    resource_manager = kwargs.get('resource_manager')
    if not resource_manager:
        return _fail(executor, 'resource_manager required in kwargs')
    
    agent_name = kwargs.get('agent_name')
    if not agent_name:
        return _fail(executor, 'agent_name required in kwargs')
    
    grobid_url = kwargs.get('grobid_url')
    
    # Get PDF path
    pdf_path = kwargs.get('path') or input_value
    if not pdf_path:
        return _fail(executor, 'path parameter required (PDF file path or Note ID)')
    
    # Handle Note ID input
    if isinstance(pdf_path, str) and pdf_path.startswith('Note_'):
        # Get Note content and extract PDF path or URL
        note_resource = resource_manager.get_resource(pdf_path)
        if not note_resource:
            return _fail(executor, f'Note {pdf_path} not found')
        
        note_content = note_resource.get('properties', {}).get('content', {})
        if isinstance(note_content, dict):
            # Check for pdf_url or source_url in metadata
            metadata = note_content.get('metadata', {})
            pdf_path = metadata.get('pdf_url') or metadata.get('source_url') or metadata.get('uri')
            if not pdf_path:
                return _fail(executor, f'Note {pdf_path} does not contain PDF URL in metadata')
        else:
            return _fail(executor, f'Note {pdf_path} content is not structured')
    
    if not isinstance(pdf_path, str):
        return _fail(executor, 'path must be a string (file path or URL)')
    
    # Check if it's a file path
    if os.path.isfile(pdf_path):
        pdf_filepath = pdf_path
        pdf_url = None
    elif pdf_path.startswith(('http://', 'https://')):
        pdf_filepath = None
        pdf_url = pdf_path
    else:
        return _fail(executor, f'Invalid path: {pdf_path} (must be file path or URL)')
    
    # Extract references
    references = extract_references_grobid(
        pdf_filepath=pdf_filepath,
        pdf_url=pdf_url,
        grobid_url=grobid_url
    )
    
    if references is None:
        return _fail(executor, 'Failed to extract references from PDF (GROBID error or no references found)')
    
    if not references:
        # Return empty Collection
        empty_coll_id = _create_collection([], agent_name, resource_manager)
        if not empty_coll_id:
            return _fail(executor, 'Failed to create empty Collection')
        return _success(
            executor,
            '0 items []',
            empty_coll_id,
            {"item_count": 0, "source": pdf_path},
        )
    
    # Create a Note for each reference
    note_ids = []
    for idx, ref in enumerate(references):
        # Structure Note content to match format-citation expectations (top-level fields)
        note_content = {
            "title": ref.get('title', ''),
            "authors": ref.get('authors', []),
            "year": ref.get('year', 0),
            "venue": ref.get('venue', ''),
            "doi": ref.get('doi', ''),
            "metadata": {
                "source_pdf": pdf_path,
                "reference_index": idx,
                "raw_citation": ref.get('raw_citation', ''),
                "url": ref.get('url', ''),
                "extraction_method": "grobid"
            }
        }
        
        note_id = _create_note(note_content, agent_name, resource_manager)
        if note_id:
            note_ids.append(note_id)
        else:
            logger.warning(f"Failed to create Note for reference {idx}")
    
    if not note_ids:
        return _fail(executor, 'Failed to create any Notes from references')
    
    # Create Collection
    collection_id = _create_collection(note_ids, agent_name, resource_manager)
    if not collection_id:
        return _fail(executor, 'Failed to create Collection')
    
    item_count = len(note_ids)
    display_ids = note_ids[:5]
    note_list_str = ', '.join(display_ids)
    if item_count > 5:
        note_list_str += ', ...'
    collection_value = f"{item_count} items [{note_list_str}]"
    
    logger.info(f"extract-references created Collection {collection_id} with {item_count} references from {pdf_path}")
    return _success(
        executor,
        collection_value,
        collection_id,
        {"item_count": item_count, "source": pdf_path, "note_ids": note_ids},
    )


if __name__ == "__main__":
    # Test
    import sys
    if len(sys.argv) > 1:
        pdf_path = sys.argv[1]
        print(f"Testing extract-references with: {pdf_path}")
        # Would need executor/resource_manager for full test
    else:
        print("Usage: python tool.py <pdf_file_path>")
