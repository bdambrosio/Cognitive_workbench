#!/usr/bin/env python3
"""
Filter items in a Collection based on natural-language predicate, returning a new Collection.
"""
import logging
import json
from typing import Any, Dict, List, Optional
from infospace_executor import InfospaceExecutor

logger = logging.getLogger(__name__)

# Resource manager will be passed via kwargs


def _get_content(resource_id: str, resource_manager) -> Any:
    """Fetch content for a resource ID."""
    if resource_id == "Note_null":
        return None
    
    if not resource_manager:
        logger.error("Resource manager not available")
        return None
    
    resource = resource_manager.get_resource(resource_id)
    if not resource:
        return None
    
    return resource.get('properties', {}).get('content')


def _create_collection(note_ids: List[str], agent_name: str, resource_manager, source_skill: str = 'filter-collection') -> str:
    """Create a new Collection with the given note_ids."""
    if not note_ids:
        note_ids = []
    
    if not resource_manager:
        logger.error("Resource manager not available")
        return None
    
    success, collection_id, error_msg, location = resource_manager.create_collection(
        agent_name, note_ids, 'list', source_skill, f'{len(note_ids)} filtered items', '', {}
    )
    
    if success:
        logger.info(f"Created filtered Collection {collection_id} with {len(note_ids)} items")
        return collection_id
    else:
        logger.error(f"Failed to create Collection: {error_msg}")
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


def tool(input_value: Any, runtime=None, **kwargs) -> str:
    """
    Filter Collection items by predicate, return new Collection ID.
    
    Args:
        input_value: List of note_ids from input Collection
        **kwargs: 
            - predicate (required): Filtering condition
            - mode ('include'/'exclude', default 'include')
            - llm_generate: Required LLM generation callback
            - map_name: Map name (for logging)
            - agent_name: Agent name (for collection creation, defaults to 'system')
    
    Returns:
        New Collection ID (str) or None on error
    """
    executor: InfospaceExecutor = kwargs.get("executor")
    if not executor:
        return {"status": "failed", "reason": "executor not available", "value": None, "resource_id": None}

    resource_manager = kwargs.get('resource_manager')
    predicate = kwargs.get('predicate')
    # If no predicate but value is in kwargs (planner mistake: used 'value' instead of 'predicate'), use value as predicate
    if not predicate and 'value' in kwargs:
        predicate = kwargs.get('value')
    
    if not predicate:
        logger.warning("No predicate provided; returning empty Collection")
        agent_name = kwargs.get('agent_name', 'system')
        empty_coll_id = _create_collection([], agent_name, resource_manager)
        if not empty_coll_id:
            return _fail(executor, 'Failed to create empty Collection')
        return _success(
            executor,
            '0 items []',
            empty_coll_id,
            {"item_count": 0, "predicate": predicate, "mode": "include"},
        )
    
    mode = kwargs.get('mode', 'include')
    agent_name = kwargs.get('agent_name', 'system')
    llm_generate = kwargs.get('llm_generate')
    if not llm_generate:
        return _fail(executor, 'llm_generate callback is required')
    
    if not isinstance(input_value, list):
        logger.warning("Input not a list; treating as empty Collection")
        empty_coll_id = _create_collection([], agent_name, resource_manager)
        if not empty_coll_id:
            return _fail(executor, 'Failed to create empty Collection')
        return _success(
            executor,
            '0 items []',
            empty_coll_id,
            {"item_count": 0, "predicate": predicate, "mode": mode},
        )
    
    filtered_ids = []
    for note_id in input_value:
        if not isinstance(note_id, str) or not note_id.startswith('Note_'):
            # Skip invalid IDs (could be sub-collections in future)
            continue
        
        # Fetch note content
        note_content = _get_content(note_id, resource_manager)
        if note_content is None:
            logger.warning(f"Note {note_id} not found or has no content; skipping")
            continue
        
        content_str = str(note_content)
        
        # LLM evaluation
        prompt = f"Does this match '{predicate}'? Respond 'true' or 'false' only.\n\nContent: {content_str}"
        try:
            # Use unified llm_generate callback (required)
            response = llm_generate(messages=[prompt], max_tokens=10, temperature=0.0, is_json=False)
            if not response.success:
                logger.error(f"LLM evaluation failed for {note_id}: {response.error}")
                continue
            result = response.text.strip().lower()
            matches = True if 'true' in result else False
        except Exception as e:
            logger.error(f"LLM evaluation failed for {note_id}: {e}")
            continue
        
        if (mode == 'include' and matches) or (mode == 'exclude' and not matches):
            filtered_ids.append(note_id)
    
    # Create new Collection with filtered note_ids
    new_coll_id = _create_collection(filtered_ids, agent_name, resource_manager)
    if not new_coll_id:
        logger.error("Failed to create filtered Collection")
        return _fail(executor, 'Failed to create filtered Collection')
    
    # Format collection value as "X items [Note_1, ...]"
    item_count = len(filtered_ids)
    display_ids = filtered_ids[:5]
    note_list_str = ', '.join(display_ids)
    if item_count > 5:
        note_list_str += ', ...'
    collection_value = f"{item_count} items [{note_list_str}]"
    
    return _success(
        executor,
        collection_value,
        new_coll_id,
        {
            "item_count": item_count,
            "predicate": predicate,
            "mode": mode,
            "filtered_ids": filtered_ids,
        },
    )
