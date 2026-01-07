"""
Word count tool - example Python tool.
"""

from typing import Any, Dict
from infospace_executor import InfospaceExecutor


def _fail(executor: InfospaceExecutor, reason: str, extra: Dict[str, Any] | None = None):
    return executor._create_uniform_return("failed", value=reason, reason=reason, extra=extra)


def _success(executor: InfospaceExecutor, value: str, extra: Dict[str, Any] | None = None):
    return executor._create_uniform_return("success", value=value, extra=extra)


def tool(value, runtime=None, **kwargs):
    """
    Count words in input text.
    
    Args:
        value: Input text (string)
        **kwargs: Optional parameters (none used)
    
    Returns:
        String with word count
    """
    executor: InfospaceExecutor = kwargs.get("executor")
    if not executor:
        return {"status": "failed", "reason": "executor not available", "value": None, "resource_id": None}

    if not isinstance(value, str):
        return _fail(executor, "Input must be a string")
    
    # Count words
    words = value.split()
    count = len(words)
    result_text = f"Word count: {count}"
    
    return _success(executor, result_text, {"count": count})

