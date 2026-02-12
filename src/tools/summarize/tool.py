#!/usr/bin/env python3
"""
Backward-compatibility wrapper: summarize → synthesize.
Use 'synthesize' directly for new code.
"""
import logging
import importlib.util
from pathlib import Path

logger = logging.getLogger(__name__)

_synthesize_tool_func = None


def _load_synthesize():
    global _synthesize_tool_func
    if _synthesize_tool_func is not None:
        return _synthesize_tool_func
    synthesize_path = Path(__file__).parent.parent / 'synthesize' / 'tool.py'
    spec = importlib.util.spec_from_file_location("tool_synthesize", str(synthesize_path))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    _synthesize_tool_func = module.tool
    return _synthesize_tool_func


def tool(input_value, runtime=None, **kwargs):
    """Deprecated: use synthesize instead. Routes to synthesize tool.
    Maps old 'style' parameter to 'format' for backward compatibility.
    """
    logger.warning("summarize is deprecated — use synthesize instead. Routing to synthesize.")
    # Map 'style' → 'format' if present (old summarize used 'style', synthesize uses 'format')
    if 'style' in kwargs and 'format' not in kwargs:
        kwargs['format'] = kwargs.pop('style')
    return _load_synthesize()(input_value, runtime=runtime, **kwargs)
