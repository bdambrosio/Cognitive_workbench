#!/usr/bin/env python3
"""
Entity extraction tool with automatic chunking for long documents.
"""
import logging
import json
from llm_client import ZenohLLMClient

llm_client = ZenohLLMClient(server_name='vllm', model_name='models/Qwen3-Next:1.5B')
logger = logging.getLogger(__name__)


def tool(value, **kwargs):
    """
    Extract entities from text with automatic chunking for long documents.
    
    Args:
        value: Text content to extract entities from
        **kwargs: Optional parameters (entity_types, include_confidence, max_entities_per_type)
    
    Returns:
        JSON string with extracted entities
    """
    if not value:
        return json.dumps({"people": [], "organizations": [], "locations": [], 
                          "topics": [], "dates": [], "key_concepts": [], "relationships": []})
    
    # Check if segmentation needed
    chunks = _segment_text(value, max_chunk_size=16000)
    
    if len(chunks) == 1:
        # Single chunk - direct extraction
        return _extract_from_chunk(value)
    
    # Multiple chunks - extract and merge
    logger.info(f"extract-entities: long document ({len(chunks)} chunks), extracting and merging")
    
    all_entities = {
        "people": set(),
        "organizations": set(),
        "locations": set(),
        "topics": set(),
        "dates": [],  # Keep as list to preserve temporal info
        "key_concepts": set(),
        "relationships": []
    }
    
    # Extract from each chunk
    for i, (chunk_text, _) in enumerate(chunks):
        chunk_result = _extract_from_chunk(chunk_text)
        
        # Parse JSON result
        chunk_entities = json.loads(chunk_result)
        
        # Merge entities (deduplicate)
        for key in ["people", "organizations", "locations", "topics", "key_concepts"]:
            if key in chunk_entities:
                all_entities[key].update(chunk_entities[key])
        
        # Append dates and relationships without deduplication
        if "dates" in chunk_entities:
            all_entities["dates"].extend(chunk_entities["dates"])
        if "relationships" in chunk_entities:
            all_entities["relationships"].extend(chunk_entities["relationships"])
    
    # Convert sets back to lists and deduplicate dates
    result = {
        "people": sorted(list(all_entities["people"])),
        "organizations": sorted(list(all_entities["organizations"])),
        "locations": sorted(list(all_entities["locations"])),
        "topics": sorted(list(all_entities["topics"])),
        "dates": list(set(all_entities["dates"])),  # Deduplicate dates
        "key_concepts": sorted(list(all_entities["key_concepts"])),
        "relationships": all_entities["relationships"]
    }
    
    return json.dumps(result, indent=2)


def _extract_from_chunk(text):
    """Extract entities from a single chunk of text."""
    prompt = f"""Extract entities from the following text. Return JSON with these fields:
- people: list of person names
- organizations: list of organizations/companies
- locations: list of places
- topics: list of main topics/subjects
- dates: list of dates or temporal references
- key_concepts: list of important concepts
- relationships: list of {{subject, predicate, object}} triples

Text:
{text}

Return only valid JSON, no explanation."""
    
    response = llm_client.generate(
        messages=[prompt],
        max_tokens=1000,
        temperature=0.3,
        is_json=True
    )
    
    # Send heartbeat after LLM call
    heartbeat = kwargs.get('heartbeat')
    if heartbeat:
        heartbeat()
    
    if not response.success:
        logger.error(f"extract-entities failed: {response.error}")
        return json.dumps({"people": [], "organizations": [], "locations": [], 
                          "topics": [], "dates": [], "key_concepts": [], "relationships": []})
    
    return response.text if isinstance(response.text, str) else json.dumps(response.text)


def _segment_text(text, max_chunk_size=16000):
    """
    Segment text into chunks at sentence boundaries.
    Returns list of (chunk_text, delimiter) tuples.
    """
    if len(text) <= max_chunk_size:
        return [(text, None)]
    
    chunks = []
    pos = 0
    
    while pos < len(text):
        end_pos = min(pos + max_chunk_size, len(text))
        
        if end_pos >= len(text):
            chunks.append((text[pos:], None))
            break
        
        # Find sentence boundary
        search_start = max(pos, end_pos - 500)
        best_split = -1
        best_delimiter = None
        
        for i in range(end_pos, search_start, -1):
            if i < len(text) - 1 and text[i] == '.' and text[i+1] in (' ', '\n'):
                best_split = i + 1
                best_delimiter = text[i+1]
                break
        
        # Fall back to word boundary
        if best_split == -1:
            for i in range(end_pos, search_start, -1):
                if text[i] in (' ', '\n', '\t'):
                    best_split = i
                    best_delimiter = text[i]
                    break
        
        # Hard split if needed
        if best_split == -1:
            best_split = end_pos
            best_delimiter = ''
        
        chunk_text = text[pos:best_split]
        chunks.append((chunk_text, best_delimiter))
        pos = best_split + (1 if best_delimiter else 0)
    
    return chunks

