#!/usr/bin/env python3
"""
Universal LLM-based Note transformation tool.
"""
import logging
import logging
from llm_client import ZenohLLMClient
llm_client = ZenohLLMClient(server_name='vllm', model_name='models/Qwen3-Next:1.5B')
logger = logging.getLogger(__name__)


def tool(value, **kwargs):
    """
    Transform Note content using natural language instruction.
    
    Args:
        value: Note content to transform
        **kwargs: Optional parameters
            - instruction: Natural language instruction (required)
            - llm_client: LLM client instance (optional, creates default if not provided)
    
    Returns:
        Transformed content as string
    """
    instruction = kwargs.get('instruction')
    if not instruction:
        return "Error: instruction parameter required"
    
    if not value:
        return "Error: value parameter required"
    if len(value) > 32000:
        logger.warning(f"refine: value too long, truncating to 32000 characters")
        value = value[:32000]
    
    # Build prompt
    prompt = f"""Transform the following content according to this instruction:

Instruction: {instruction}

Content:
{value}

Return only the transformed result, no explanation."""
    
    logger.info(f"refine: {instruction[:50]}...")
    
    # Call LLM
    response = llm_client.generate(
        messages=[prompt],
        max_tokens=2000,
        temperature=0.5,
        is_json=False
    )
    
    if not response.success:
        logger.error(f"refine failed: {response.error}")
        return f"Error: {response.error}"
    
    result = response.text
    logger.info(f"refine complete: output_len={len(result)}")
    
    return result


