#!/usr/bin/env python3
"""
Universal LLM-based Note predicate testing tool.
"""
import logging

logger = logging.getLogger(__name__)


def execute(value: str, predicate: str = None, llm_client=None, **kwargs) -> str:
    """
    Test Note content against natural language predicate.
    
    Args:
        value: Note content to test
        predicate: Natural language question/test
        llm_client: LLM client instance
        
    Returns:
        "true" or "false" as string
    """
    if not predicate:
        return "false"
    
    if not value:
        return "false"
    
    if not llm_client:
        logger.error("test-note: llm_client not available")
        return "false"
    
    # Build prompt
    prompt = f"""Answer this question about the content with ONLY "true" or "false":

Question: {predicate}

Content:
{value}

Answer (true or false):"""
    
    # Call LLM with low temperature for consistent boolean output
    response = llm_client.generate(
        [prompt],
        max_tokens=10,
        temperature=0.0,
        is_json=False
    )
    
    result = response.text if hasattr(response, 'text') else str(response)
    result = result.strip().lower()
    
    # Normalize to boolean string
    if 'true' in result:
        answer = "true"
    elif 'false' in result:
        answer = "false"
    else:
        # Default to false if unclear
        logger.warning(f"test-note ambiguous response: '{result}' for predicate: {predicate[:50]}")
        answer = "false"
    
    logger.info(f"test-note: predicate='{predicate[:50]}...' → {answer}")
    
    return answer

