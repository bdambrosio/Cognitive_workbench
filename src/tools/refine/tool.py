#!/usr/bin/env python3
"""
Backward-compatibility wrapper: refine → extract.
Use 'extract' directly for new code.
"""
import logging
import importlib.util
from pathlib import Path

logger = logging.getLogger(__name__)

_extract_tool_func = None


def _load_extract():
    global _extract_tool_func
    if _extract_tool_func is not None:
        return _extract_tool_func
    extract_path = Path(__file__).parent.parent / 'extract' / 'tool.py'
    spec = importlib.util.spec_from_file_location("tool_extract", str(extract_path))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    _extract_tool_func = module.tool
    return _extract_tool_func


def tool(input_value, runtime=None, **kwargs):
    """Deprecated: use extract instead. Routes to extract tool."""
    logger.warning("refine is deprecated — use extract instead. Routing to extract.")
    return _load_extract()(input_value, runtime=runtime, **kwargs)
