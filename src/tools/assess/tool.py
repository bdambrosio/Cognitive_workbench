#!/usr/bin/env python3
"""
Boolean predicate testing with automatic chunking for long documents.
"""
import logging
from typing import Any, Dict
# infospace_executor was the planner-side runtime; the chat ReAct loop
# bypasses it. Keep the import optional so this module loads in chat-only
# builds — the InfospaceExecutor annotations degrade to Any there.
try:
    from infospace_executor import InfospaceExecutor
except ImportError:
    InfospaceExecutor = Any  # type: ignore[assignment,misc]
from utils.text_chunking import segment_text_boundary_aware

logger = logging.getLogger(__name__)


def _fail(executor: InfospaceExecutor, reason: str, value: str | None = None, extra: Dict[str, Any] | None = None):
    return executor._create_uniform_return(
        "failed",
        value=value or reason,
        reason=reason,
        extra=extra,
    )


def _succeed(executor: InfospaceExecutor, result: str, extra: Dict[str, Any] | None = None):
    return executor._create_uniform_return("success", value=result, extra=extra)


def react_invoke(args, *, character_name=None, backend=None, logger=None):
    """ReAct entry-point — see Skill.md for the args contract."""
    from utils.chat_tool_stub import build_tool_kwargs, CapturingResourceManager, translate_result
    source = args.get("source", "")
    predicate = args.get("predicate", "")
    if not isinstance(source, str) or not source:
        return {"status": "error", "text": "assess requires non-empty `source`"}
    if not isinstance(predicate, str) or not predicate:
        return {"status": "error", "text": "assess requires non-empty `predicate`"}
    if backend is None:
        return {"status": "error", "text": "assess requires backend (LLM access not configured)"}

    mgr = CapturingResourceManager()
    result = tool(source, **build_tool_kwargs(
        character_name=character_name, backend=backend, manager=mgr,
        predicate=predicate,
    ))
    return translate_result(result, manager=mgr)


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
    executor: InfospaceExecutor = kwargs.get("executor")
    if not executor:
        return {"status": "failed", "reason": "executor not available", "value": None, "resource_id": None}

    predicate = kwargs.get('predicate')
    # If no predicate but value is in kwargs (planner mistake: used 'value' instead of 'predicate'), use value as predicate
    if not predicate and 'value' in kwargs:
        predicate = kwargs.get('value')
    
    if not predicate:
        return _fail(executor, "missing_predicate")
    
    if not input_value:
        return _fail(executor, "missing_input")
    
    # Check if segmentation needed
    chunks = segment_text_boundary_aware(input_value, max_chunk_size=16000)
    
    if len(chunks) == 1:
        # Single chunk - direct test. _test_chunk returns a bare "true"/"false";
        # box it the way the multi-chunk path below does, so both paths return
        # the uniform dict every caller expects (the ReAct wrapper's
        # translate_result rejects a bare string).
        return _succeed(executor, _test_chunk(input_value, predicate, kwargs),
                        {"matched_chunk": 1, "total_chunks": 1})
    
    # Multiple chunks - OR aggregation (true if ANY chunk matches)
    logger.info(f"assess: long document ({len(chunks)} chunks), using OR aggregation")
    
    for i, (chunk_text, _) in enumerate(chunks):
        result = _test_chunk(chunk_text, predicate, kwargs)
        if result == "true":
            logger.info(f"assess: predicate matched in chunk {i+1}/{len(chunks)}")
            return _succeed(executor, "true", {"matched_chunk": i + 1, "total_chunks": len(chunks)})
    
    return _succeed(executor, "false", {"matched_chunk": None, "total_chunks": len(chunks)})


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
    # enable_thinking=False: a reasoning model would spend its whole
    # 10-token budget inside the think block and return nothing (Qwen3.8
    # wants ~100 thinking tokens for this one-word answer), and any stop
    # marker it mentions while thinking truncates the turn. Non-thinking
    # backends ignore the flag — it is a chat-template kwarg unused by
    # their templates.
    response = llm_generate(
        messages=[prompt],
        max_tokens=10,
        temperature=0.0,
        is_json=False,
        stops=['</end>'],
        enable_thinking=False
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

