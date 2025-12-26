#!/usr/bin/env python3
"""
Boolean predicate testing with automatic chunking for long documents.
"""
import logging
from utils.text_chunking import segment_text_boundary_aware

logger = logging.getLogger(__name__)


def tool(input_value, runtime=None, **kwargs):
    """
    Test Note content against natural language predicate with automatic chunking.
    
    Args:
        input_value: Note content to test
        **kwargs: Optional parameters
            - predicate: Natural language question/test (required)
    
    Returns:
        "true" or "false" as string
    """
    predicate = kwargs.get('predicate')
    # If no predicate but value is in kwargs (planner mistake: used 'value' instead of 'predicate'), use value as predicate
    if not predicate and 'value' in kwargs:
        predicate = kwargs.get('value')
    
    if not predicate:
        return "false"
    
    if not input_value:
        return "false"
    
    # Check if segmentation needed
    chunks = segment_text_boundary_aware(input_value, max_chunk_size=16000)
    
    if len(chunks) == 1:
        # Single chunk - direct test
        return _test_chunk(input_value, predicate, kwargs)
    
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
    
    # Use unified llm_generate callback (required)
    llm_generate = kwargs.get('llm_generate') if kwargs else None
    if not llm_generate:
        raise ValueError("llm_generate callback is required")
    response = llm_generate(
        messages=[prompt],
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

