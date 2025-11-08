#!/usr/bin/env python3
"""
Summarization tool with focus-aware compression and adaptive styling.
"""
import logging
import json
import zenoh
import time
import uuid
from zenoh import QueryTarget, ConsolidationMode
from llm_client import ZenohLLMClient

llm_client = ZenohLLMClient(server_name='vllm', model_name='models/Qwen3-Next:1.5B')
logger = logging.getLogger(__name__)

# Open zenoh session for fetching Collection/Note content and calling map_node
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


def _estimate_tokens(text):
    """Rough token estimation: ~4 chars per token"""
    return len(text) // 4


def _wait_for_response(topic: str, timeout: float = 10.0):
    """Wait for response on Zenoh topic."""
    response_data = None
    start_time = time.time()
    
    def response_handler(sample):
        nonlocal response_data
        if not response_data:  # Only take first response
            response_data = json.loads(sample.payload.to_bytes().decode('utf-8'))
    
    subscriber = zenoh_session.declare_subscriber(topic, response_handler)
    
    try:
        while not response_data and (time.time() - start_time) < timeout:
            time.sleep(0.1)
    finally:
        subscriber.undeclare()
    
    return response_data


def _create_temp_collection(text_content: str, map_name: str = 'infolab') -> str:
    """Create temporary Collection with text content for indexing."""
    # Create a Note with the text content
    note_id = None
    for reply in zenoh_session.get(
        "cognitive/map/note/create",
        target=QueryTarget.BEST_MATCHING,
        consolidation=ConsolidationMode.NONE,
        payload=json.dumps({
            'character_name': 'summarize_tool',
            'content': text_content,
            'format': 'text',
            'source_skill': 'summarize_temp'
        }).encode('utf-8'),
        timeout=5.0
    ):
        if reply.ok:
            response = json.loads(reply.ok.payload.to_bytes().decode('utf-8'))
            if response.get('success'):
                note_id = response.get('info_id')
                break
        break
    
    if not note_id:
        logger.error("Failed to create temporary Note")
        return None
    
    # Create Collection containing the Note
    collection_id = None
    for reply in zenoh_session.get(
        "cognitive/map/collection/create",
        target=QueryTarget.BEST_MATCHING,
        consolidation=ConsolidationMode.NONE,
        payload=json.dumps({
            'character_name': 'summarize_tool',
            'content': [note_id],
            'format': 'list',
            'source_skill': 'summarize_temp',
            'collection_name': f'summarize_temp_{uuid.uuid4().hex[:8]}'
        }).encode('utf-8'),
        timeout=5.0
    ):
        if reply.ok:
            response = json.loads(reply.ok.payload.to_bytes().decode('utf-8'))
            if response.get('success'):
                collection_id = response.get('info_id')
                break
        break
    
    if not collection_id:
        logger.error("Failed to create temporary Collection")
        return None
    
    return collection_id


def _index_collection(collection_id: str, map_name: str = 'infolab') -> bool:
    """Index a Collection via map_node."""
    request = {
        'agent_name': 'summarize_tool',
        'collection_id': collection_id,
        'index_type': 'semantic',
        'fields': {'content': 'embed'}
    }
    
    # Publish index request
    zenoh_session.put(
        f"map/{map_name}/index_request/summarize_tool",
        json.dumps(request)
    )
    
    # Wait for response
    response = _wait_for_response(
        f"map/{map_name}/index_response/summarize_tool",
        timeout=15.0
    )
    
    if not response:
        logger.error("Index request timeout")
        return False
    
    if response.get('status') != 'success':
        logger.error(f"Index failed: {response.get('reason')}")
        return False
    
    return True


def _search_collection(collection_id: str, query: str, limit: int, map_name: str = 'infolab') -> list:
    """Search indexed Collection and return chunk results."""
    request = {
        'agent_name': 'summarize_tool',
        'collection_id': collection_id,
        'query': query,
        'mode': 'semantic',
        'limit': limit,
        'threshold': 0.0,
        'return_mode': 'chunks'
    }
    
    # Publish search request
    zenoh_session.put(
        f"map/{map_name}/search_request/summarize_tool",
        json.dumps(request)
    )
    
    # Wait for response
    response = _wait_for_response(
        f"map/{map_name}/search_response/summarize_tool",
        timeout=10.0
    )
    
    if not response:
        logger.error("Search request timeout")
        return []
    
    if response.get('status') != 'success':
        logger.error(f"Search failed: {response.get('reason')}")
        return []
    
    return response.get('results', [])


def _compute_target_length(effective_tokens, style, compression_ratio):
    """Compute target summary length based on style and compression ratio."""
    if style == 'executive':
        # Fixed cap for executive summaries
        return min(500, effective_tokens // 3)
    elif style == 'comprehensive':
        # Low compression for detailed summaries
        return effective_tokens // 2.0
    else:  # technical (default)
        # Use specified compression ratio
        target = effective_tokens / compression_ratio
        # Floor: never go below 300 tokens for very small inputs
        return max(target, 300)


def tool(value, **kwargs):
    """
    Summarize content with focus-aware compression and adaptive styling.
    
    Args:
        value: Text content to summarize, or list (Collection) which will be flattened
        **kwargs: Optional parameters
            - focus: Topic to guide summarization (optional)
            - style: Output style - 'technical' (default), 'executive', or 'comprehensive'
            - compression_ratio: Compression factor (default 3.0), applied to focused content
    
    Returns:
        Summary as string
    """
    if not value:
        return "No content to summarize"
    
    # Extract parameters
    focus = kwargs.get('focus', '')
    style = kwargs.get('style', 'technical')
    compression_ratio = kwargs.get('compression_ratio', 3.0)
    heartbeat = kwargs.get('heartbeat')
    
    # Validate style
    if style not in ['technical', 'executive', 'comprehensive']:
        logger.warning(f"Unknown style '{style}', using 'technical'")
        style = 'technical'
    
    # Flatten Collection if input is a list
    if isinstance(value, list):
        logger.info(f"summarize: flattening Collection with {len(value)} items")
        value = _flatten_list(value)
    
    # Measure input
    input_tokens = _estimate_tokens(value)
    
    # Apply focus filtering via index+search if focus provided
    effective_tokens = input_tokens
    inclusion_pct = 100
    
    if focus:
        logger.info(f"summarize: applying semantic search filter for '{focus}'")
        
        # Get map_name from kwargs (default to 'infolab' for infospace)
        map_name = kwargs.get('map_name', 'infolab')
        
        # Create temporary Collection with content
        temp_collection_id = _create_temp_collection(value, map_name)
        if not temp_collection_id:
            logger.warning(f"Failed to create temp collection, using all content")
            filtered_text = value
        else:
            try:
                # Index the Collection
                if not _index_collection(temp_collection_id, map_name):
                    logger.warning(f"Failed to index temp collection, using all content")
                    filtered_text = value
                else:
                    # Compute target length to determine how many chunks to retrieve
                    target_tokens = int(_compute_target_length(effective_tokens, style, compression_ratio))
                    avg_chunk_tokens = 128  # Based on paragraph-first chunking
                    target_chunks = max(1, int(target_tokens / avg_chunk_tokens))
                    limit = target_chunks * 3  # 3x overshoot
                    
                    # Search for relevant chunks
                    search_results = _search_collection(temp_collection_id, focus, limit, map_name)
                    
                    if not search_results:
                        logger.warning(f"Search returned no results for '{focus}', using all content")
                        filtered_text = value
                    else:
                        # Extract chunk content from search results
                        chunk_contents = []
                        cumulative_tokens = 0
                        max_content_tokens = target_tokens * 3  # Allow up to 3x target for LLM
                        
                        for result in search_results:
                            # Extract chunk text from result (search returns 'document' field)
                            chunk_text = result.get('document', '')
                            if not chunk_text:
                                continue
                            
                            chunk_tokens = _estimate_tokens(chunk_text)
                            if cumulative_tokens + chunk_tokens > max_content_tokens:
                                break
                            
                            chunk_contents.append(chunk_text)
                            cumulative_tokens += chunk_tokens
                        
                        if chunk_contents:
                            filtered_text = "\n\n".join(chunk_contents)
                            effective_tokens = _estimate_tokens(filtered_text)
                            inclusion_pct = int((effective_tokens / input_tokens) * 100) if input_tokens > 0 else 100
                            logger.info(f"Retrieved {len(chunk_contents)} chunks ({effective_tokens}t, {inclusion_pct}% of input)")
                        else:
                            logger.warning(f"No chunks extracted from search results, using all content")
                            filtered_text = value
            finally:
                # Note: Temporary Collection cleanup could be added here if needed
                # For now, leaving it (map_node may clean up unused resources)
                pass
            
            value = filtered_text
    
    # Check if segmentation needed (after filtering)
    chunks = _segment_text(value, max_chunk_size=16000)
    chunk_count = len(chunks)
    
    # Compute target length
    target_tokens = int(_compute_target_length(effective_tokens, style, compression_ratio))
    
    # Style-specific instructions
    style_instructions = {
        'executive': 'Provide a high-level executive summary focused on key findings and implications.',
        'technical': 'Provide a technical summary preserving key details, methodology, and caveats.',
        'comprehensive': 'Provide a comprehensive summary preserving nuance, technical details, and supporting evidence.'
    }
    style_instruction = style_instructions.get(style, style_instructions['technical'])
    
    focus_guidance = f"\nFocus on: {focus}" if focus else ""
    
    # Log summarization parameters
    logger.info(f"summarize: input={input_tokens}t, focus={'yes' if focus else 'no'}, "
               f"filtered={effective_tokens}t ({inclusion_pct}%), target={target_tokens}t, "
               f"style={style}, ratio={compression_ratio:.1f}")
    
    if chunk_count == 1:
        # Single chunk - direct summarization
        chunk_text = chunks[0][0]
        
        prompt = f"""{style_instruction}{focus_guidance}

Content:
{chunk_text}

Target length: approximately {target_tokens} tokens. Provide a summary highlighting key points.
Do not include any introductory, reasoning, or explanatory text in your response. Only provide the summary, followed by the </end> tag.
End your response with:
</end>
"""
        
        # max_tokens = 1.5x final output limit (2000 tokens)
        response = llm_client.generate(
            messages=[prompt],
            max_tokens=3000,
            temperature=0.3,
            is_json=False,
            stops=['</end>']
        )
        
        if heartbeat:
            heartbeat()
        
        if not response.success:
            logger.error(f"summarize failed: {response.error}")
            return f"Error: {response.error}"
        
        output_tokens = _estimate_tokens(response.text)
        logger.info(f"summarize: output={output_tokens}t")
        
        return response.text
    
    # Multiple chunks - hierarchical summarization
    logger.info(f"summarize: hierarchical summarization ({chunk_count} chunks)")
    
    # Distribute target across chunks
    tokens_per_chunk = target_tokens // chunk_count
    tokens_per_chunk = max(tokens_per_chunk, 100)  # Floor per chunk
    
    # Step 1: Summarize each chunk
    chunk_summaries = []
    for i, (chunk_text, _) in enumerate(chunks):
        prompt = f"""{style_instruction}{focus_guidance}

Section:
{chunk_text}

Target length: approximately {tokens_per_chunk} tokens. Provide key points.
Do not include any introductory, reasoning, or explanatory text in your response. Only provide the summary, followed by the </end> tag.
End your response with:
</end>
"""
        
        # max_tokens = 1.5x target per chunk
        max_chunk_tokens = int(tokens_per_chunk * 1.5)
        response = llm_client.generate(
            messages=[prompt],
            max_tokens=max_chunk_tokens,
            temperature=0.3,
            is_json=False,
            stops=['</end>']
        )
        
        if heartbeat:
            heartbeat()
        
        if not response.success:
            logger.error(f"summarize chunk {i+1}/{chunk_count} failed: {response.error}")
            return f"Error: {response.error}"
        
        chunk_summaries.append(response.text)
    
    # Step 2: Synthesize chunk summaries into final summary
    combined = "\n\n".join([f"Section {i+1}: {s}" for i, s in enumerate(chunk_summaries)])
    
    synthesis_prompt = f"""{style_instruction}{focus_guidance}

Synthesize these section summaries into a coherent overall summary.

Section Summaries:
{combined}

Target length: approximately {target_tokens} tokens. Provide a unified summary that captures the key themes and findings.
Do not include any introductory, reasoning, or explanatory text in your response. Only provide the summary, followed by the </end> tag.
End your response with:
</end>
"""
    
    # max_tokens = 1.5x final output limit (2000 tokens)
    response = llm_client.generate(
        messages=[synthesis_prompt],
        max_tokens=3000,
        temperature=0.3,
        is_json=False,
        stops=['</end>']
    )
    
    if heartbeat:
        heartbeat()
    
    if not response.success:
        logger.error(f"summarize synthesis failed: {response.error}")
        return f"Error: {response.error}"
    
    output_tokens = _estimate_tokens(response.text)
    logger.info(f"summarize: output={output_tokens}t")
    
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

