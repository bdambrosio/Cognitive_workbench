#!/usr/bin/env python3
"""
Relate tool - Compare two Notes or Collections to find similarities, differences, and relationships.
"""
import logging
import json

logger = logging.getLogger(__name__)


def _get_content(resource_id: str, resource_manager) -> any:
    """Fetch content from resource_manager for a resource ID."""
    if resource_id == "Note_null":
        return None
    
    resource = resource_manager.get_resource(resource_id)
    if resource:
        return resource.get('properties', {}).get('content')
    return None


def _flatten_list(items: list, resource_manager, separator: str = '\n\n') -> str:
    """Flatten a list (Collection content) into concatenated Note content."""
    note_contents = []
    for item in items:
        if isinstance(item, str) and item.startswith('Note_'):
            note_content = _get_content(item, resource_manager)
            if note_content is not None:
                note_contents.append(str(note_content))
        elif isinstance(item, str) and item.startswith('Collection_'):
            # Recursively flatten nested Collections
            flattened = _flatten_collection(item, resource_manager, separator)
            if flattened:
                note_contents.append(flattened)
        else:
            note_contents.append(str(item))
    
    return separator.join(note_contents)


def _flatten_collection(collection_id: str, resource_manager, separator: str = '\n\n') -> str:
    """Flatten a Collection into concatenated Note content."""
    content = _get_content(collection_id, resource_manager)
    if not isinstance(content, list):
        return str(content) if content else ""
    
    return _flatten_list(content, resource_manager, separator)


def _summarize_note(content: str, focus: str = '', heartbeat=None, kwargs=None) -> str:
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
Do not include any introductory, reasoning, or explanatory text in your response. Only provide the summary, followed by the </end> tag.
End your response with:
</end>
Summary:"""
        
        # Use unified llm_generate callback (required)
        llm_generate = kwargs.get('llm_generate') if kwargs else None
        if not llm_generate:
            raise ValueError("llm_generate callback is required")
        response = llm_generate(
            messages=[prompt],
            max_tokens=1000,
            temperature=0.3,
            is_json=False,
            stops=['</end>']
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

Do not include any introductory, reasoning, or explanatory text in your response. Only provide the summary, followed by the </end> tag.
End your response with:
</end>
Summary:"""
        
        # Use unified llm_generate callback (required)
        llm_generate = kwargs.get('llm_generate') if kwargs else None
        if not llm_generate:
            raise ValueError("llm_generate callback is required")
        response = llm_generate(
            messages=[prompt],
            max_tokens=1000,
            temperature=0.3,
            is_json=False,
            stops=['</end>']
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
        return _summarize_note(combined_summary, focus, heartbeat, kwargs)
    
    return combined_summary


def _preprocess_element(element: any, resource_manager, focus: str = '', heartbeat=None, kwargs=None) -> str:
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
        content = _flatten_collection(element, resource_manager)
        content = str(content) if content is not None else ""
    elif isinstance(element, str) and element.startswith('Note_'):
        content = _get_content(element, resource_manager)
        content = str(content) if content is not None else ""
    elif isinstance(element, list):
        content = _flatten_list(element, resource_manager)
        content = str(content) if content is not None else ""
    else:
        content = str(element) if element else ""
    
    # Ensure content is always a string (safety check)
    if content is None:
        content = ""
    
    # Step 5: Summarize if > 16k chars
    if len(content) > 16000:
        logger.info(f"relate: element exceeds 16k ({len(content)} chars), summarizing")
        summarized = _summarize_note(content, focus, heartbeat, kwargs)
        content = str(summarized) if summarized is not None else content[:16000]
    
    # Ensure content is still a string after summarization
    if content is None:
        content = ""
    
    # Step 6: Final validation - truncate if still too large
    if len(content) > 16000:
        logger.warning(f"relate: element still exceeds 16k after summarization ({len(content)} chars), truncating")
        content = content[:16000]
    
    return content


def tool(input_value, runtime=None, **kwargs):
    """
    Relate exactly two Notes/Collections to find similarities, differences, and relationships.
    
    Args:
        input_value: First Note/Collection to compare (target)
        **kwargs: Optional parameters
            - other: Second Note/Collection to compare (REQUIRED)
            - instruction: Natural language guidance for comparison focus (optional)
    
    Returns:
        Comparison analysis as JSON string
    """
    other = kwargs.get('other')
    if not input_value or not other:
        return json.dumps({"error": "relate requires both target and other parameters"})
    
    instruction = kwargs.get('instruction', '')
    heartbeat = kwargs.get('heartbeat')
    resource_manager = kwargs.get('resource_manager')
    
    # Build focus string for summarization
    focus = instruction if instruction else "key themes, facts, and perspectives for comparison"
    
    # Preprocess each element independently
    element_a = _preprocess_element(input_value, resource_manager, focus, heartbeat, kwargs)
    element_b = _preprocess_element(other, resource_manager, focus, heartbeat, kwargs)
    
    # Format for LLM comparison
    formatted_input = f"""## Item A
{element_a}

## Item B
{element_b}"""
    
    # Build comparison prompt
    instruction_guidance = f"\n{instruction}" if instruction else ""
    
    prompt = f"""Compare the following content to identify similarities, differences, and relationships.{instruction_guidance}

Content:
{formatted_input}

Provide a comparison analysis in JSON format with:
- similarity_score: float (0.0 to 1.0)
- shared_themes: array of strings
- unique_to_first: array of strings (unique to Item A)
- unique_to_second: array of strings (unique to Item B)
- contradictions: array of objects with "aspect", "first", "second" fields
- relationship: string describing relationship type
- summary: string summary of comparison
Do not include any introductory, reasoning, or explanatory text in your response. Only provide the JSON.

"""
    
    # Use unified llm_generate callback (required)
    llm_generate = kwargs.get('llm_generate')
    if not llm_generate:
        raise ValueError("llm_generate callback is required")
    response = llm_generate(
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

