#!/usr/bin/env python3
"""
Universal LLM-based Note transformation tool.
"""
import logging
from utils.text_chunking import segment_text_boundary_aware

logger = logging.getLogger(__name__)


def tool(input_value, runtime=None, **kwargs):
    """
    Transform Note content using natural language instruction.
    
    Args:
        input_value: Note content to transform
        **kwargs: Additional parameters
            - instruction: Natural language instruction (REQUIRED)
            - llm_client: LLM client instance (optional, creates default if not provided)
    
    Returns:
        Transformed content as string
    """
    instruction = kwargs.get('instruction')
    # If no instruction but value is in kwargs (planner mistake: used 'value' instead of 'instruction'), use value as instruction
    if not instruction and 'value' in kwargs:
        instruction = kwargs.get('value')
    
    if not instruction:
        return "Error: instruction parameter required"
    
    if not input_value:
        return "Error: input_value parameter required"
    
    # Convert input_value to string if it's not already
    text_value = str(input_value) if not isinstance(input_value, str) else input_value
    
    # Segment long content into chunks
    chunks = segment_text_boundary_aware(text_value, max_chunk_size=16000)
    logger.info(f"refine: {instruction[:50]}... ({len(chunks)} chunks)")
    
    results = []
    for i, (chunk_text, delimiter) in enumerate(chunks):
        # Build prompt for this chunk
        prompt = f"""Transform the following content according to this instruction:

Instruction: {instruction}

Content:
{chunk_text}

Return only the transformed result, no explanation.
Do not include any introductory, reasoning, code fences, or explanatory text in your response. Only provide the transformed result, followed by the </end> tag.
End your response with:
</end>
"""
        
        # Calculate max_tokens based on chunk size
        max_tokens = len(chunk_text) // 2
        
        # Use unified llm_generate callback (required)
        llm_generate = kwargs.get('llm_generate')
        if not llm_generate:
            raise ValueError("llm_generate callback is required")
        response = llm_generate(
            messages=[prompt],
            max_tokens=max_tokens,
            temperature=0.4,
            is_json=False,
            stops=['</end>']
        )
        
        # Send heartbeat after LLM call to reset timeout
        heartbeat = kwargs.get('heartbeat')
        if heartbeat:
            heartbeat()
        
        if not response.success:
            logger.error(f"refine chunk {i+1}/{len(chunks)} failed: {response.error}")
            return f"Error: {response.error}"
        
        results.append(response.text)
        # Append delimiter if it's not the last chunk
        if delimiter and i < len(chunks) - 1:
            results.append(delimiter)
    
    result = ''.join(results)
    logger.info(f"refine complete: output_len={len(result)}")
    
    return result


