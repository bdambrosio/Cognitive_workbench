#!/usr/bin/env python3
"""
LLM-based Note extraction/transformation tool.
Derives content from a single Note — no new information introduced, no cross-document synthesis.
"""
import logging
from typing import Any, Dict, Optional
from infospace_executor import InfospaceExecutor
from utils.text_chunking import segment_text_boundary_aware

logger = logging.getLogger(__name__)


def _fail(executor: InfospaceExecutor, reason: str, value: Optional[str] = None, extra: Optional[Dict[str, Any]] = None):
    return executor._create_uniform_return(
        "failed",
        value=value or reason,
        reason=reason,
        extra=extra,
    )


def _success(executor: InfospaceExecutor, result: str, extra: Optional[Dict[str, Any]] = None):
    return executor._create_uniform_return("success", value=result, extra=extra)


def tool(input_value, runtime=None, **kwargs):
    """
    Derive content from a single Note via LLM-guided extraction or transformation.
    
    Args:
        input_value: Note content to extract from / transform
        **kwargs: Additional parameters
            - instruction: Natural language instruction (REQUIRED)
            - llm_client: LLM client instance (optional, creates default if not provided)
    
    Returns:
        Extracted/transformed content as string
    """
    executor: InfospaceExecutor = kwargs.get("executor")
    if not executor:
        return {"status": "failed", "reason": "executor not available", "value": None, "resource_id": None}

    # Reject Collection targets
    if isinstance(input_value, list):
        return _fail(executor, "target must be a single Note, not a Collection; use map(extract) for Collections")

    instruction = kwargs.get('instruction')
    # If no instruction but value is in kwargs (planner mistake: used 'value' instead of 'instruction'), use value as instruction
    if not instruction and 'value' in kwargs:
        instruction = kwargs.get('value')
    
    if not instruction:
        return _fail(executor, "instruction parameter required")
    
    if not input_value:
        return _fail(executor, "target is empty")
    
    # Convert input_value to string if it's not already
    text_value = str(input_value) if not isinstance(input_value, str) else input_value
    
    # Segment long content into chunks
    chunks = segment_text_boundary_aware(text_value, max_chunk_size=16000)
    logger.info(f"extract: {instruction[:50]}... ({len(chunks)} chunks)")
    
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
            return _fail(executor, "llm_generate callback is required")
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
            logger.error(f"extract chunk {i+1}/{len(chunks)} failed: {response.error}")
            return _fail(
                executor,
                "llm_generate_failed",
                value=f"Error: {response.error}",
                extra={"chunk_index": i + 1, "failure_reason": response.error},
            )
        
        results.append(response.text)
        # Append delimiter if it's not the last chunk
        if delimiter and i < len(chunks) - 1:
            results.append(delimiter)
    
    result = ''.join(results)
    logger.info(f"extract complete: output_len={len(result)}")
    
    return _success(
        executor,
        result,
        {
            "chunks": len(chunks),
            "instruction": instruction,
        },
    )
