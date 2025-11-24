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

---

## Plan Tool Requirements

**Plan tools use a `plan.json` file to define reusable plan sequences.**

### Structure

Plan tools must have:
- `SKILL.md` or `Skill.md` with `type: plan` in frontmatter
- `plan.json` or `tool.json` file with plan definition

### plan.json Format

```json
{
  "plan": [
    {"type": "action_name", "...": "..."}
  ],
  "out": "$result"
}
```

**Required fields:**
- `plan`: Array of plan actions (primitives or tool invocations)
- `out`: Output variable name (with or without `$` prefix)

### Parameter Passing

Plan tools receive parameters as bound variables:

**Main input** → `$input` variable
**Args dict entries** → `$key` variables (one per key)

**Example invocation:**
```json
{
  "type": "compare-papers",
  "target": "$paper1",
  "args": {"baseline": "$paper2", "focus": "methodology"},
  "out": "$comparison"
}
```

**Variables available inside the plan tool:**
- `$input` = content of `$paper1`
- `$baseline` = content of `$paper2`
- `$focus` = `"methodology"`

**Example plan.json:**
```json
{
  "plan": [
    {"type": "create-collection", "value": ["$input", "$baseline"], "out": "$both"},
    {"type": "relate", "target": "$both", "args": {"focus": "$focus"}, "out": "$result"}
  ],
  "out": "$result"
}
```

### Best Practices

1. **Document parameters** in SKILL.md frontmatter:
   ```yaml
   ---
   name: compare-papers
   type: plan
   description: Compare two papers
   parameters:
     - name: input
       description: First paper (main input)
     - name: baseline
       description: Second paper for comparison
     - name: focus
       description: Aspect to focus on (optional)
   ---
   ```

2. **Use descriptive variable names** in args instead of generic names
3. **Keep plan tools focused** - compose smaller plan tools rather than large monolithic ones
4. **Test with literal values** before using in larger plans

