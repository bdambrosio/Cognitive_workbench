#!/usr/bin/env python3
"""
LLM-based content generation tool.
Generates new text or code from scratch using natural language prompts.
No source documents — for generation from source material, use synthesize.
"""
import logging
from typing import Any, Dict, Optional
from infospace_executor import InfospaceExecutor

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


def tool(input_value=None, runtime=None, **kwargs):
    """
    Generate new content using natural language prompt.
    Creates content from scratch using the LLM's own knowledge — no source documents.
    For generation from source material, use synthesize instead.
    
    Args:
        input_value: Unused (for compatibility with executor interface)
        **kwargs: Tool parameters
            - prompt: Generation instruction (REQUIRED)
            - style: "code" or "text" (optional, default: "text")
    
    Returns:
        Generated content as string
    """
    executor: InfospaceExecutor = kwargs.get("executor")
    if not executor:
        return {"status": "failed", "reason": "executor not available", "value": None, "resource_id": None}

    prompt = kwargs.get('prompt', '')
    # If no prompt but value is in kwargs (planner mistake: used 'value' instead of 'prompt'), use value as prompt
    if not prompt and 'value' in kwargs:
        prompt = kwargs.get('value')
    
    if not prompt:
        return _fail(executor, "prompt parameter required")
    
    style = kwargs.get('style', 'text').lower()

    # Deprecation: if context is provided, log warning and ignore
    context_arg = kwargs.get('context', '')
    if context_arg:
        logger.warning(f"generate-note: 'context' parameter is deprecated. Use synthesize for generation from source material. Ignoring context.")
    
    # Build generation prompt based on style
    if style == 'code':
        generation_prompt = f"""Generate code according to the following instruction.

Instruction: {prompt}

Return only the code, no explanation, no code fences, no markdown formatting.
Do not include any introductory text, reasoning, or commentary.
Only provide the code itself, followed by the </end> tag.
End your response with:
</end>
"""
        max_tokens = 4000
        temperature = 0.2
    else:
        generation_prompt = f"""Generate text content according to the following instruction.

Instruction: {prompt}

Return only the generated content, no explanation, no code fences, no markdown formatting.
Do not include any introductory text, reasoning, or commentary.
Only provide the generated content itself, followed by the </end> tag.
End your response with:
</end>
"""
        max_tokens = 2000
        temperature = 0.7
    

    logger.info(f"generate-note: {prompt[:50]}... (style={style})")
    
    # Use unified llm_generate callback (required)
    llm_generate = kwargs.get('llm_generate')
    if not llm_generate:
        return _fail(executor, "llm_generate callback is required")
    response = llm_generate(
        messages=[generation_prompt],
        max_tokens=max_tokens,
        temperature=temperature,
        is_json=False,
        stops=['</end>']
    )
    
    # Send heartbeat after LLM call to reset timeout
    heartbeat = kwargs.get('heartbeat')
    if heartbeat:
        heartbeat()
    
    if not response.success:
        logger.error(f"generate-note failed: {response.error}")
        return _fail(executor, "llm_generate_failed", value=f"Error: {response.error}", extra={"llm_error": response.error})
    
    result = response.text.strip()
    logger.info(f"generate-note complete: output_len={len(result)}")
    
    return _success(
        executor,
        result,
        {
            "style": style,
            "prompt_length": len(prompt),
        },
    )
