#!/usr/bin/env python3
"""
Summarization tool with automatic chunking for long documents.
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


def tool(value, **kwargs):
    """
    Summarize content with automatic chunking for long documents.
    
    Args:
        value: Text content to summarize, or list (Collection) which will be flattened
        **kwargs: Optional parameters
            - focus: Topic to guide summarization (optional)
    
    Returns:
        Summary as string
    """
    if not value:
        return "No content to summarize"
    
    # Flatten Collection if input is a list
    if isinstance(value, list):
        logger.info(f"summarize: flattening Collection with {len(value)} items")
        value = _flatten_list(value)
    
    focus = kwargs.get('focus', '')
    focus_guidance = f"\nFocus on: {focus}" if focus else ""
    
    # Check if segmentation needed
    chunks = _segment_text(value, max_chunk_size=16000)
    
    if len(chunks) == 1:
        # Single chunk - direct summarization
        prompt = f"""Summarize the following content concisely.{focus_guidance}

Content:
{value}

Provide a brief summary (3-5 sentences) highlighting key points."""
        
        response = llm_client.generate(
            messages=[prompt],
            max_tokens=500,
            temperature=0.3,
            is_json=False
        )
        
        # Send heartbeat after LLM call
        heartbeat = kwargs.get('heartbeat')
        if heartbeat:
            heartbeat()
        
        if not response.success:
            logger.error(f"summarize failed: {response.error}")
            return f"Error: {response.error}"
        
        return response.text
    
    # Multiple chunks - hierarchical summarization
    logger.info(f"summarize: long document ({len(chunks)} chunks), using hierarchical approach")
    
    # Step 1: Summarize each chunk
    chunk_summaries = []
    for i, (chunk_text, _) in enumerate(chunks):
        prompt = f"""Summarize this section concisely.{focus_guidance}

Section:
{chunk_text}

Provide key points (2-4 sentences)."""
        
        response = llm_client.generate(
            messages=[prompt],
            max_tokens=300,
            temperature=0.3,
            is_json=False
        )
        
        # Send heartbeat after LLM call
        heartbeat = kwargs.get('heartbeat')
        if heartbeat:
            heartbeat()
        
        if not response.success:
            logger.error(f"summarize chunk {i+1}/{len(chunks)} failed: {response.error}")
            return f"Error: {response.error}"
        
        chunk_summaries.append(response.text)
    
    # Step 2: Synthesize chunk summaries into final summary
    combined = "\n\n".join([f"Section {i+1}: {s}" for i, s in enumerate(chunk_summaries)])
    
    synthesis_prompt = f"""Synthesize these section summaries into a coherent overall summary.{focus_guidance}

Section Summaries:
{combined}

Provide a unified summary (4-6 sentences) that captures the key themes and findings."""
    
    response = llm_client.generate(
        messages=[synthesis_prompt],
        max_tokens=500,
        temperature=0.3,
        is_json=False
    )
    
    # Send heartbeat after LLM call
    heartbeat = kwargs.get('heartbeat')
    if heartbeat:
        heartbeat()
    
    if not response.success:
        logger.error(f"summarize synthesis failed: {response.error}")
        return f"Error: {response.error}"
    
    return response.text


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

