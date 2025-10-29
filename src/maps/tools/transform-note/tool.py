#!/usr/bin/env python3
"""
Universal LLM-based Note transformation tool.
"""
import logging

logger = logging.getLogger(__name__)


def execute(value: str, command: str = None, model: str = "fast", llm_client=None, **kwargs) -> str:
    """
    Transform Note content using natural language command.
    
    Args:
        value: Note content to transform
        command: Natural language instruction
        model: LLM model selection ("fast" or "sonnet")
        llm_client: LLM client instance
        
    Returns:
        Transformed content as string
    """
    if not command:
        return "Error: command parameter required"
    
    if not value:
        return "Error: value parameter required (Note content to transform)"
    
    if not llm_client:
        return "Error: llm_client not available"
    
    # Build prompt
    prompt = f"""Transform the following content according to this instruction:

Instruction: {command}

Content:
{value}

Return only the transformed result, no explanation."""
    
    # Select model and temperature based on type
    if model == "sonnet":
        # Use higher quality model with lower temperature
        max_tokens = 4000
        temperature = 0.3
        logger.info(f"transform-note using premium model: {command[:50]}...")
    else:
        # Use default fast model
        max_tokens = 2000
        temperature = 0.5
        logger.info(f"transform-note using fast model: {command[:50]}...")
    
    # Call LLM
    response = llm_client.generate(
        [prompt],
        max_tokens=max_tokens,
        temperature=temperature,
        is_json=False
    )
    
    result = response.text if hasattr(response, 'text') else str(response)
    
    # Log for usage analysis
    logger.info(f"transform-note complete: command={command[:50]}, output_len={len(result)}")
    
    return result

