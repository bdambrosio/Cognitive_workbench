#!/usr/bin/env python3
"""
Format citations from semantic-scholar Notes/Collections to BibTeX format.
"""
import logging
import re
from typing import Any, Dict, List, Optional
from infospace_executor import InfospaceExecutor

logger = logging.getLogger(__name__)


def _get_note_resource(resource_id: str, resource_manager) -> Optional[Dict]:
    """Fetch resource (with properties) for a Note ID."""
    if resource_id == "Note_null" or not resource_manager:
        return None
    return resource_manager.get_resource(resource_id)


def _get_content(resource_id: str, resource_manager) -> Any:
    """Fetch content for a resource ID (Note: string, Collection: list of note_ids)."""
    resource = resource_manager.get_resource(resource_id) if resource_manager else None
    if not resource:
        return None
    return resource.get('properties', {}).get('content')


def _create_note(text_content: str, agent_name: str, resource_manager, source_skill: str = 'format-citation',
                 tool_metadata: Optional[Dict] = None) -> Optional[str]:
    """Create a Note. Content is text-only."""
    if not resource_manager:
        logger.error("resource_manager required for creating Notes")
        return None
    success, note_id, error_msg, _ = resource_manager.create_note(
        character_name=agent_name,
        content=text_content,
        format_type='text',
        source_skill=source_skill,
        source_value=(text_content or '')[:100],
        note_name='',
        extra_props={'tool_metadata': tool_metadata or {}}
    )
    
    if success:
        return note_id
    else:
        logger.error(f"Failed to create Note: {error_msg}")
        return None


def _create_collection(note_ids: List[str], agent_name: str, resource_manager, source_skill: str = 'format-citation') -> Optional[str]:
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
        f'{len(note_ids)} citations',
        '',
        '',
        {}
    )
    
    if success:
        logger.info(f"Created Collection {collection_id} with {len(note_ids)} citations")
        return collection_id
    else:
        logger.error(f"Failed to create Collection: {error_msg}")
        return None


def _generate_bibtex_key(title: str, authors: List[str], year: int) -> str:
    """Generate a BibTeX citation key from title, authors, and year."""
    # Use first author's last name and year
    if authors and len(authors) > 0:
        first_author = authors[0].strip()
        # Try to extract last name (assume "Last, First" or "First Last")
        if ',' in first_author:
            last_name = first_author.split(',')[0].strip()
        else:
            parts = first_author.split()
            last_name = parts[-1] if parts else first_author
        # Clean and lowercase
        last_name = re.sub(r'[^a-zA-Z0-9]', '', last_name).lower()
    else:
        last_name = "unknown"
    
    # Use year if available
    year_str = str(year) if year and year > 0 else ""
    
    # Get first significant word from title
    title_words = re.findall(r'\b\w+\b', title.lower())
    first_word = title_words[0] if title_words else "paper"
    
    # Combine: lastname_year_titleword
    key = f"{last_name}{year_str}{first_word}"
    # Limit length and clean
    key = re.sub(r'[^a-zA-Z0-9]', '', key)[:50]
    return key or "citation"


def _format_author_bibtex(author: str) -> str:
    """Format a single author name for BibTeX."""
    author = author.strip()
    if not author:
        return ""
    
    # If already in "Last, First" format, use as-is
    if ',' in author:
        return author
    
    # Otherwise, assume "First Last" and try to format
    # For BibTeX, "Last, First" is preferred but "First Last" also works
    return author


def _format_bibtex(metadata: Dict[str, Any]) -> str:
    """
    Format paper metadata as BibTeX citation.
    
    Args:
        metadata: Dict with title, authors, year, venue, doi, etc.
        
    Returns:
        BibTeX citation string
    """
    title = metadata.get('title', 'Untitled')
    authors = metadata.get('authors', [])
    year = metadata.get('year', 0)
    venue = metadata.get('venue', '')
    doi = metadata.get('doi') or metadata.get('DOI')
    
    # Determine entry type based on venue
    # Conference venues often contain keywords like "Conference", "Workshop", "Symposium"
    venue_lower = venue.lower() if venue else ""
    if any(keyword in venue_lower for keyword in ['conference', 'workshop', 'symposium', 'proceedings', 'icml', 'neurips', 'iclr', 'aaai', 'ijcai']):
        entry_type = "inproceedings"
    elif any(keyword in venue_lower for keyword in ['journal', 'transactions', 'letters']):
        entry_type = "article"
    else:
        # Default to article if venue unclear
        entry_type = "article"
    
    # Generate citation key
    bibtex_key = _generate_bibtex_key(title, authors, year)
    
    # Format authors
    if authors:
        formatted_authors = " and ".join([_format_author_bibtex(a) for a in authors if a.strip()])
    else:
        formatted_authors = "Unknown"
    
    # Build BibTeX entry
    lines = [f"@{entry_type}{{{bibtex_key},"]
    lines.append(f"  title={{{title}}},")
    lines.append(f"  author={{{formatted_authors}}},")
    
    if year and year > 0:
        lines.append(f"  year={{{year}}},")
    
    if venue:
        if entry_type == "inproceedings":
            lines.append(f"  booktitle={{{venue}}},")
        else:
            lines.append(f"  journal={{{venue}}},")
    
    if doi:
        lines.append(f"  doi={{{doi}}},")
    
    # Remove trailing comma from last line
    lines[-1] = lines[-1].rstrip(',')
    lines.append("}")
    
    return "\n".join(lines)


def _format_note_citation(tool_metadata: Dict) -> Optional[str]:
    """
    Extract metadata from tool_metadata and format as BibTeX citation.
    
    Args:
        tool_metadata: tool_metadata from Note properties (title, authors, year, etc.)
        
    Returns:
        BibTeX citation string, or None if formatting fails
    """
    try:
        metadata = tool_metadata or {}
        
        # Validate required fields
        if not metadata.get('title'):
            logger.warning("Note missing title; cannot format citation")
            return None
        
        return _format_bibtex(metadata)
    
    except Exception as e:
        logger.error(f"Error formatting citation: {e}")
        return None


def _fail(executor: InfospaceExecutor, reason: str, extra: Optional[Dict[str, Any]] = None):
    return executor._create_uniform_return("failed", value=reason, reason=reason, extra=extra)


def _success(
    executor: InfospaceExecutor,
    value: str,
    resource_id: Optional[str],
    extra: Optional[Dict[str, Any]] = None,
):
    return executor._create_uniform_return("success", value=value, resource_id=resource_id, extra=extra)


def tool(input_value: Any, runtime=None, **kwargs):
    """
    Format citations from semantic-scholar Notes/Collections to BibTeX format.
    
    Args:
        input_value: Collection ID (list of note_ids), Note ID, or variable name
        **kwargs:
            - target: Collection/Note ID or variable (if input_value not provided)
            - format: Citation format (default: "bibtex")
            - agent_name: Required agent name
            - resource_manager: Required resource manager
            - executor: Required executor
            
    Returns:
        Collection ID (if input was Collection) or Note ID (if input was Note) with formatted citations
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
    
    # Get target (executor resolves 'target' field and passes as input_value)
    target = input_value or kwargs.get('target')
    if not target:
        return _fail(executor, 'target parameter required (Collection ID, Note ID, or variable)')
    
    # Get format (default to bibtex)
    citation_format = kwargs.get('format', 'bibtex').lower()
    if citation_format != 'bibtex':
        return _fail(executor, f'Unsupported format: {citation_format}. Currently only "bibtex" is supported')
    
    # Check if target is a Collection or Note
    is_collection = False
    note_ids = []
    
    if isinstance(target, list):
        # Already a list of note_ids (from Collection, already resolved by executor)
        note_ids = [nid for nid in target if isinstance(nid, str) and nid.startswith('Note_')]
        is_collection = True
    elif isinstance(target, str):
        # Could be Collection ID, Note ID, or variable name (should be resolved by executor)
        if target.startswith('Collection_'):
            # Collection ID - fetch note_ids
            collection_content = _get_content(target, resource_manager)
            if isinstance(collection_content, list):
                note_ids = [nid for nid in collection_content if isinstance(nid, str) and nid.startswith('Note_')]
                is_collection = True
            else:
                return _fail(executor, f'Collection {target} content is not a list')
        elif target.startswith('Note_'):
            # Single Note ID
            note_ids = [target]
            is_collection = False
        else:
            # Try to resolve as resource ID (might be a variable that wasn't resolved)
            resource = resource_manager.get_resource(target)
            if resource:
                content = resource.get('properties', {}).get('content')
                if isinstance(content, list):
                    note_ids = [nid for nid in content if isinstance(nid, str) and nid.startswith('Note_')]
                    is_collection = True
                else:
                    # Single Note
                    note_ids = [target]
                    is_collection = False
            else:
                return _fail(executor, f'Could not resolve target: {target}')
    
    if not note_ids:
        # Empty Collection or invalid input
        if is_collection:
            empty_coll_id = _create_collection([], agent_name, resource_manager)
            if not empty_coll_id:
                return _fail(executor, 'Failed to create empty Collection')
            return _success(executor, '0 items []', empty_coll_id, {"item_count": 0})
        else:
            return _fail(executor, 'No valid Note IDs found')
    
    # Process each Note and format citation
    formatted_note_ids = []
    failed_count = 0
    
    for note_id in note_ids:
        if not isinstance(note_id, str) or not note_id.startswith('Note_'):
            logger.warning(f"Skipping invalid note_id: {note_id}")
            failed_count += 1
            continue
        
        # Get Note resource (content + tool_metadata)
        resource = _get_note_resource(note_id, resource_manager)
        if not resource:
            logger.warning(f"Note {note_id} not found; skipping")
            failed_count += 1
            continue
        props = resource.get('properties', {})
        tool_meta = props.get('tool_metadata', {})
        
        # Format citation from tool_metadata (semantic-scholar stores title, authors, year there)
        bibtex_citation = _format_note_citation(tool_meta)
        if bibtex_citation is None:
            logger.warning(f"Failed to format citation for Note {note_id} (missing metadata); skipping")
            failed_count += 1
            continue
        
        citation_note_id = _create_note(
            bibtex_citation, agent_name, resource_manager,
            tool_metadata={"citation_format": "bibtex", "source_note": note_id, "original_metadata": tool_meta}
        )
        if citation_note_id:
            formatted_note_ids.append(citation_note_id)
        else:
            logger.warning(f"Failed to create citation Note for {note_id}")
            failed_count += 1
    
    # Return result
    if is_collection:
        # Return Collection of formatted citations
        if not formatted_note_ids:
            empty_coll_id = _create_collection([], agent_name, resource_manager)
            if not empty_coll_id:
                return _fail(executor, 'Failed to create empty Collection')
            return _success(
                executor,
                '0 items []',
                empty_coll_id,
                {"item_count": 0, "failed_count": failed_count}
            )
        
        collection_id = _create_collection(formatted_note_ids, agent_name, resource_manager)
        if not collection_id:
            return _fail(executor, 'Failed to create Collection')
        
        item_count = len(formatted_note_ids)
        display_ids = formatted_note_ids[:5]
        note_list_str = ', '.join(display_ids)
        if item_count > 5:
            note_list_str += ', ...'
        collection_value = f"{item_count} items [{note_list_str}]"
        
        return _success(
            executor,
            collection_value,
            collection_id,
            {"item_count": item_count, "failed_count": failed_count, "format": citation_format}
        )
    else:
        # Return single Note
        if not formatted_note_ids:
            return _fail(executor, 'Failed to format citation', extra={"failed_count": failed_count})
        
        note_id = formatted_note_ids[0]
        # Get the citation text for display (content is string)
        citation_note_content = _get_content(note_id, resource_manager)
        citation_text = str(citation_note_content) if citation_note_content else ""
        
        display_text = f"Citation formatted: {citation_text[:100]}..." if citation_text else "Citation formatted"
        return _success(
            executor,
            display_text,
            note_id,
            {"format": citation_format, "failed_count": failed_count}
        )


if __name__ == "__main__":
    # Test with sample metadata
    test_metadata = {
        "title": "Attention Is All You Need",
        "authors": ["Vaswani, Ashish", "Shazeer, Noam", "Parmar, Niki"],
        "year": 2017,
        "venue": "Advances in Neural Information Processing Systems",
        "doi": "10.48550/arXiv.1706.03762"
    }
    print(_format_bibtex(test_metadata))
