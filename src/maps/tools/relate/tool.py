#!/usr/bin/env python3
"""
Relate tool - Compare Notes and Collections with automatic flattening and context management.
"""
import logging
import json
import zenoh
from zenoh import QueryTarget, ConsolidationMode
from llm_client import ZenohLLMClient

llm_client = ZenohLLMClient(server_name='vllm', model_name='models/Qwen3-Next:1.5B')
logger = logging.getLogger(__name__)

# Open zenoh session for fetching Collection/Note content
config = zenoh.Config()
zenoh_session = zenoh.open(config)


def _get_content(resource_id: str) -> any:
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


def _flatten_list(items: list, separator: str = '\n\n') -> str:
    """Flatten a list (Collection content) into concatenated Note content."""
    note_contents = []
    for item in items:
        if isinstance(item, str) and item.startswith('Note_'):
            note_content = _get_content(item)
            if note_content is not None:
                note_contents.append(str(note_content))
        elif isinstance(item, str) and item.startswith('Collection_'):
            # Recursively flatten nested Collections
            flattened = _flatten_collection(item, separator)
            if flattened:
                note_contents.append(flattened)
        else:
            note_contents.append(str(item))
    
    return separator.join(note_contents)


def _flatten_collection(collection_id: str, separator: str = '\n\n') -> str:
    """Flatten a Collection into concatenated Note content."""
    content = _get_content(collection_id)
    if not isinstance(content, list):
        return str(content) if content else ""
    
    return _flatten_list(content, separator)


def _summarize_note(content: str, focus: str = '', heartbeat=None) -> str:
    """
    Summarize a Note if it exceeds context limits. 
    Chunks content > 16k chars before summarizing.
    Returns truncated content if summarize fails.
    """
    # If content is small enough, summarize directly
    if len(content) <= 16000:
        focus_guidance = f"\nFocus on: {focus}" if focus else ""
        
        prompt = f"""Summarize the following content concisely for comparison purposes.{focus_guidance}
Provide a detailed summary (8-12 sentences) preserving key themes, facts, and perspectives.

Content:
{content}

Summary:"""
        
        response = llm_client.generate(
            messages=[prompt],
            max_tokens=1000,
            temperature=0.3,
            is_json=False
        )
        
        # Send heartbeat after LLM call
        if heartbeat:
            heartbeat()
        
        if not response.success:
            logger.warning(f"summarize failed: {response.error}, truncating to 16000 chars")
            return content[:16000]  # Truncate if summarization fails
        
        return response.text if response.text else content[:16000]
    
    # Content > 16k: chunk and summarize each chunk
    logger.info(f"summarize: content ({len(content)} chars) exceeds 16k, chunking before summarization")
    
    # Chunk size: 12k chars (leaving room for prompt overhead)
    chunk_size = 12000
    chunks = []
    for i in range(0, len(content), chunk_size):
        chunks.append(content[i:i + chunk_size])
    
    logger.info(f"summarize: split into {len(chunks)} chunks")
    
    # Summarize each chunk
    chunk_summaries = []
    focus_guidance = f"\nFocus on: {focus}" if focus else ""
    
    for i, chunk in enumerate(chunks):
        prompt = f"""Summarize the following content chunk concisely for comparison purposes.{focus_guidance}
Provide a detailed summary (6-10 sentences) preserving key themes, facts, and perspectives.
This is chunk {i+1} of {len(chunks)}.

Content:
{chunk}

Summary:"""
        
        response = llm_client.generate(
            messages=[prompt],
            max_tokens=1000,
            temperature=0.3,
            is_json=False
        )
        
        # Send heartbeat after LLM call
        if heartbeat:
            heartbeat()
        
        if response.success and response.text:
            chunk_summaries.append(response.text)
        else:
            logger.warning(f"summarize: chunk {i+1} failed, using truncated chunk")
            chunk_summaries.append(chunk[:chunk_size // 2])  # Use partial chunk if summarize fails
    
    # Combine chunk summaries
    combined_summary = '\n\n'.join(chunk_summaries)
    
    # If combined summary is still > 16k, recursively summarize it
    if len(combined_summary) > 16000:
        logger.info(f"summarize: combined summary ({len(combined_summary)} chars) still exceeds 16k, recursively summarizing")
        return _summarize_note(combined_summary, focus, heartbeat)
    
    return combined_summary


def _preprocess_element(element: any, focus: str = '', heartbeat=None) -> str:
    """
    Preprocess a single element for comparison.
    
    Steps:
    1. If Collection ID string → fetch and flatten
    2. If Note ID string → fetch content
    3. If list → flatten directly
    4. Otherwise → use as-is
    5. If > 16k chars → summarize
    6. Ensure < 16k chars (truncate if needed)
    
    Returns:
        Preprocessed string content (always < 16k chars)
    """
    # Step 1-4: Resolve and flatten
    if isinstance(element, str) and element.startswith('Collection_'):
        content = _flatten_collection(element)
        content = str(content) if content is not None else ""
    elif isinstance(element, str) and element.startswith('Note_'):
        content = _get_content(element)
        content = str(content) if content is not None else ""
    elif isinstance(element, list):
        content = _flatten_list(element)
        content = str(content) if content is not None else ""
    else:
        content = str(element) if element else ""
    
    # Ensure content is always a string (safety check)
    if content is None:
        content = ""
    
    # Step 5: Summarize if > 16k chars
    if len(content) > 16000:
        logger.info(f"relate: element exceeds 16k ({len(content)} chars), summarizing")
        summarized = _summarize_note(content, focus, heartbeat)
        content = str(summarized) if summarized is not None else content[:16000]
    
    # Ensure content is still a string after summarization
    if content is None:
        content = ""
    
    # Step 6: Final validation - truncate if still too large
    if len(content) > 16000:
        logger.warning(f"relate: element still exceeds 16k after summarization ({len(content)} chars), truncating")
        content = content[:16000]
    
    return content


def tool(value, **kwargs):
    """
    Compare exactly two Notes/Collections to find similarities, differences, and relationships.
    
    Args:
        value: List with exactly 2 elements (Note/Collection IDs or content)
        **kwargs: Optional parameters
            - comparison_mode: 'similarity' | 'contradiction' | 'comprehensive' (default)
            - threshold: Minimum similarity to report (default: 0.3)
            - focus_aspects: String describing specific dimensions to compare
    
    Returns:
        Comparison analysis as JSON string
    """
    if not value:
        return json.dumps({"error": "No content to compare"})
    
    # Validate: must be list with exactly 2 elements
    if not isinstance(value, list):
        return json.dumps({"error": "relate requires a list with exactly 2 elements"})
    
    if len(value) != 2:
        return json.dumps({"error": f"relate requires exactly 2 elements, got {len(value)}"})
    
    comparison_mode = kwargs.get('comparison_mode', 'comprehensive')
    threshold = kwargs.get('threshold', 0.3)
    focus_aspects = kwargs.get('focus_aspects', '')
    heartbeat = kwargs.get('heartbeat')
    
    # Build focus string for summarization
    focus = focus_aspects if focus_aspects else "key themes, facts, and perspectives for comparison"
    
    # Preprocess each element independently
    element_a = _preprocess_element(value[0], focus, heartbeat)
    element_b = _preprocess_element(value[1], focus, heartbeat)
    
    # Format for LLM comparison
    formatted_input = f"""## Item A
{element_a}

## Item B
{element_b}"""
    
    # Build comparison prompt
    mode_guidance = ""
    if comparison_mode == 'similarity':
        mode_guidance = "\nFocus on identifying similarities and overlaps."
    elif comparison_mode == 'contradiction':
        mode_guidance = "\nFocus on identifying contradictions and conflicts."
    
    focus_guidance = f"\nFocus aspects: {focus_aspects}" if focus_aspects else ""
    
    prompt = f"""Compare the following content to identify similarities, differences, and relationships.

{mode_guidance}{focus_guidance}

Content:
{formatted_input}

Provide a comparison analysis in JSON format with:
- similarity_score: float (0.0 to 1.0)
- shared_themes: array of strings
- unique_to_first: array of strings (unique to Item A)
- unique_to_second: array of strings (unique to Item B)
- contradictions: array of objects with "aspect", "first", "second" fields
- relationship: string describing relationship type
- summary: string summary of comparison"""
    
    response = llm_client.generate(
        messages=[prompt],
        max_tokens=1000,
        temperature=0.5,
        is_json=True
    )
    
    # Send heartbeat after LLM call
    heartbeat = kwargs.get('heartbeat')
    if heartbeat:
        heartbeat()
    
    if not response.success:
        logger.error(f"relate failed: {response.error}")
        return json.dumps({"error": response.error})
    
    return response.text

