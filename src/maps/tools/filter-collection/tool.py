#!/usr/bin/env python3
"""
Filter items in a Collection based on natural-language predicate, returning a new Collection.
"""
import logging
import json
import zenoh
from typing import List, Any
from zenoh import QueryTarget, ConsolidationMode
from llm_client import ZenohLLMClient

logger = logging.getLogger(__name__)

# Open zenoh session for fetching Note content and creating Collections
config = zenoh.Config()
zenoh_session = zenoh.open(config)

# LLM client (use default from kwargs if provided, else fallback)
default_llm_client = ZenohLLMClient(server_name='vllm', model_name='models/Qwen3-Next:1.5B')


def _get_content(resource_id: str) -> Any:
    """Fetch content from map_node for a resource ID."""
    if resource_id == "Note_null":
        return None
    
    for reply in zenoh_session.get(
        f"cognitive/map/resource/{resource_id}",
        target=QueryTarget.BEST_MATCHING,
        consolidation=ConsolidationMode.NONE,
        timeout=5.0
    ):
        if reply.ok:
            response = json.loads(reply.ok.payload.to_bytes().decode('utf-8'))
            if response.get('success'):
                if 'resource' in response:
                    resource_data = response.get('resource')
                    if resource_data:
                        return resource_data.get('properties', {}).get('content')
                else:
                    return response.get('content')
        break
    return None


def _create_collection(note_ids: List[str], agent_name: str, source_skill: str = 'filter-collection') -> str:
    """Create a new Collection with the given note_ids via Zenoh."""
    if not note_ids:
        note_ids = []
    
    for reply in zenoh_session.get(
        "cognitive/map/collection/create",
        target=QueryTarget.BEST_MATCHING,
        consolidation=ConsolidationMode.NONE,
        payload=json.dumps({
            'character_name': agent_name,
            'content': note_ids,
            'format': 'list',
            'source_skill': source_skill,
            'source_value': f'{len(note_ids)} filtered items'
        }).encode('utf-8'),
        timeout=5.0
    ):
        if reply.ok:
            response = json.loads(reply.ok.payload.to_bytes().decode('utf-8'))
            if response.get('success'):
                collection_id = response.get('info_id')
                logger.info(f"Created filtered Collection {collection_id} with {len(note_ids)} items")
                return collection_id
        break
    
    logger.error("Failed to create Collection via Zenoh")
    return None


def tool(value: Any, **kwargs) -> str:
    """
    Filter Collection items by predicate, return new Collection ID.
    
    Args:
        value: List of note_ids from input Collection
        **kwargs: 
            - predicate (required): Filtering condition
            - mode ('include'/'exclude', default 'include')
            - llm_client: Optional LLM client (uses default if not provided)
            - map_name: Map name (for logging)
            - agent_name: Agent name (for collection creation, defaults to 'system')
    
    Returns:
        New Collection ID (str) or None on error
    """
    predicate = kwargs.get('predicate')
    if not predicate:
        logger.warning("No predicate provided; returning empty Collection")
        agent_name = kwargs.get('agent_name', 'system')
        return _create_collection([], agent_name) or ""
    
    mode = kwargs.get('mode', 'include')
    agent_name = kwargs.get('agent_name', 'system')
    llm_client = kwargs.get('llm_client', default_llm_client)
    
    if not isinstance(value, list):
        logger.warning("Input not a list; treating as empty Collection")
        return _create_collection([], agent_name) or ""
    
    filtered_ids = []
    for note_id in value:
        if not isinstance(note_id, str) or not note_id.startswith('Note_'):
            # Skip invalid IDs (could be sub-collections in future)
            continue
        
        # Fetch note content via Zenoh
        note_content = _get_content(note_id)
        if note_content is None:
            logger.warning(f"Note {note_id} not found or has no content; skipping")
            continue
        
        content_str = str(note_content)
        
        # LLM evaluation
        prompt = f"Does this match '{predicate}'? Respond 'true' or 'false' only.\n\nContent: {content_str}"
        try:
            response = llm_client.generate([prompt], max_tokens=10)
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
    new_coll_id = _create_collection(filtered_ids, agent_name)
    if not new_coll_id:
        logger.error("Failed to create filtered Collection")
        return ""
    
    return new_coll_id
