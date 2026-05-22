#!/usr/bin/env python3
"""
LLM-based reflective content generation tool.
Like generate-note, but with access to the planner's internal reasoning state
(system prompt, goal analysis, tool selection, situation context) as background.
Enables the agent to generate content informed by its current orientation.
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
    Generate content with awareness of the agent's current reasoning state.

    Works like generate-note but includes the planner's internal state
    (character context, goal analysis, tool selection, situation awareness)
    as background context for the LLM, enabling reflective or
    state-aware generation.

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
    if not prompt and 'value' in kwargs:
        prompt = kwargs.get('value')

    if not prompt:
        return _fail(executor, "prompt parameter required")

    style = kwargs.get('style', 'text').lower()

    # Retrieve planner reasoning state (snapshotted at Stage 1 by the planner)
    reflective_state = getattr(executor, '_planner_reflective_state', '') or ''
    if reflective_state:
        state_len = len(reflective_state)
        logger.info(f"generate-reflective-note: using reflective state ({state_len} chars)")
        reflective_block = (
            f"\n## Internal Reflective State\n"
            f"The following is the agent's current reasoning context — goal analysis, "
            f"situation awareness, character orientation, and planning state. "
            f"Use this as background when responding to the instruction.\n\n"
            f"{reflective_state}\n"
            f"## End Internal Reflective State\n\n"
        )
    else:
        logger.warning("generate-reflective-note: no reflective state available, proceeding without")
        reflective_block = ""

    # Build generation prompt
    if style == 'code':
        generation_prompt = f"""{reflective_block}Generate code according to the following instruction.

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
        generation_prompt = f"""{reflective_block}Generate text content according to the following instruction.
Use the Internal Reflective State above as background context to inform your response.

Instruction: {prompt}

Return only the generated content, no explanation, no code fences, no markdown formatting.
Do not include any introductory text, reasoning, or commentary.
Do not describe or summarize the reflective state — use it to inform your content.
Only provide the generated content itself, followed by the </end> tag.
End your response with:
</end>
"""
        max_tokens = 2000
        temperature = 0.7

    # Allow output sizing guidance to override default max_tokens
    guided_tokens = kwargs.get('target_tokens')
    if guided_tokens and isinstance(guided_tokens, (int, float)) and int(guided_tokens) > 0:
        max_tokens = int(guided_tokens)

    logger.info(f"generate-reflective-note: {prompt[:50]}... (style={style}, state={len(reflective_block)} chars)")

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

    heartbeat = kwargs.get('heartbeat')
    if heartbeat:
        heartbeat()

    if not response.success:
        logger.error(f"generate-reflective-note failed: {response.error}")
        return _fail(executor, "llm_generate_failed", value=f"Error: {response.error}", extra={"llm_error": response.error})

    result = response.text.strip()
    logger.info(f"generate-reflective-note complete: output_len={len(result)}")

    return _success(
        executor,
        result,
        {
            "style": style,
            "prompt_length": len(prompt),
            "reflective_state_length": len(reflective_block),
        },
    )
