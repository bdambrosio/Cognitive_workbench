#!/usr/bin/env python3
"""
Boolean predicate testing with automatic chunking for long documents.
"""
import logging
from llm_client import ZenohLLMClient

llm_client = ZenohLLMClient(server_name='vllm', model_name='models/Qwen3-Next:1.5B')
logger = logging.getLogger(__name__)


def tool(value, **kwargs):
    """
    Test Note content against natural language predicate with automatic chunking.
    
    Args:
        value: Note content to test
        **kwargs: Optional parameters
            - predicate: Natural language question/test (required)
    
    Returns:
        "true" or "false" as string
    """
    predicate = kwargs.get('predicate')
    if not predicate:
        return "false"
    
    if not value:
        return "false"
    
    # Check if segmentation needed
    chunks = _segment_text(value, max_chunk_size=16000)
    
    if len(chunks) == 1:
        # Single chunk - direct test
        return _test_chunk(value, predicate, kwargs)
    
    # Multiple chunks - OR aggregation (true if ANY chunk matches)
    logger.info(f"assess: long document ({len(chunks)} chunks), using OR aggregation")
    
    for i, (chunk_text, _) in enumerate(chunks):
        result = _test_chunk(chunk_text, predicate, kwargs)
        if result == "true":
            logger.info(f"assess: predicate matched in chunk {i+1}/{len(chunks)}")
            return "true"
    
    return "false"


def _test_chunk(text, predicate, kwargs=None):
    """Test a single chunk against predicate."""
    prompt = f"""Answer this question about the content with ONLY "true" or "false":

Question: {predicate}

Content:
{text}

Do not include any introductory, reasoning, or explanatory text in your response. Only provide the summary, followed by the </end> tag.
End your response with:
</end>

Answer (true or false):"""
    
    response = llm_client.generate(
        [prompt],
        max_tokens=10,
        temperature=0.0,
        is_json=False,
        stops=['</end>']
    )
    
    # Send heartbeat after LLM call
    if kwargs:
        heartbeat = kwargs.get('heartbeat')
        if heartbeat:
            heartbeat()
    
    result = response.text if hasattr(response, 'text') else str(response)
    result = result.strip().lower()
    
    # Normalize to boolean string
    if 'true' in result:
        return "true"
    elif 'false' in result:
        return "false"
    else:
        logger.warning(f"assess ambiguous response: '{result}' for predicate: {predicate[:50]}")
        return "false"


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

