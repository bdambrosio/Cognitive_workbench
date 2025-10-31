# Tool Development Guide

## Python Tool Requirements

**All Python tools MUST define a `tool()` function as the entry point.**

### Function Signature

```python
def tool(value, **kwargs):
    """
    Tool entry point.
    
    Args:
        value: Main input data (Note content, Collection, etc.)
        **kwargs: Additional parameters from tool invocation
        
    Returns:
        Result value (string, dict, or structured response)
    """
    # Tool implementation
    return result
```

### Why `tool()` and not `execute()`?

The executor (`infospace_executor.py`) expects a function named `tool()`:

```python
if not hasattr(tool_module, 'tool'):
    return {'status': 'failed', 
           'reason': f'No tool() function in {python_path.name}'}
tool_func = tool_module.tool
```

**Current state**: All tools use `tool()` - this is the standard pattern.

### Internal Implementation Pattern

Some tools use `execute()` internally for organization, then wrap it with `tool()`:

```python
def execute(value: str, command: str = None, **kwargs) -> str:
    """Internal implementation."""
    # ... tool logic ...
    return result

def tool(value: str, command: str = None, **kwargs) -> str:
    """Tool entry point - wraps execute() for compatibility."""
    return execute(value, command=command, **kwargs)
```

This pattern is acceptable, but **`tool()` must exist** and be the entry point.

### Examples

**Simple tool** (`word-count/tool.py`):
```python
def tool(value, **kwargs):
    count = len(value.split())
    return f"Word count: {count}"
```

**Tool with internal `execute()`** (`refine/tool.py`, `assess/tool.py`):
```python
def execute(value: str, command: str = None, **kwargs) -> str:
    # Implementation
    return result

def tool(value: str, command: str = None, **kwargs) -> str:
    return execute(value, command=command, **kwargs)
```

### Verification

To verify your tool has the correct function:
```bash
grep "^def tool" src/maps/tools/your-tool/tool.py
```

Should return: `def tool(...)`

### Common Errors

**Error**: `No tool() function in tool.py`
**Cause**: Tool defines `execute()` but not `tool()`
**Fix**: Add `tool()` wrapper function (see pattern above)

