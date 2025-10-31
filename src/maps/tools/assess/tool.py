#!/usr/bin/env python3
"""
Universal LLM-based Note predicate testing tool.
"""
import logging
from llm_client import ZenohLLMClient
llm_client = ZenohLLMClient(server_name='vllm', model_name='models/Qwen3-Next:1.5B')
logger = logging.getLogger(__name__)

def tool(value, **kwargs):
    """
    Test Note content against natural language predicate.
    
    Args:
        value: Note content to test
        **kwargs: Optional parameters
            - predicate: Natural language question/test (required)
            - llm_client: LLM client instance (required)
    
    Returns:
        "true" or "false" as string
    """
    predicate = kwargs.get('predicate')
    if not predicate:
        return "false"
    
    if not value:
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
        logger.warning(f"assess ambiguous response: '{result}' for predicate: {predicate[:50]}")
        answer = "false"
    
    logger.info(f"assess: predicate='{predicate[:50]}...' → {answer}")
    
    return answer

