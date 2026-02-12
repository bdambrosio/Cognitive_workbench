# Spec: Phase-Aware Execution — Trace-Informed Fixes

**Version 3 — incorporates findings from first live trace**

The v2 spec was implemented and tested. This document captures issues found in the
first live trace and specifies fixes. Changes here are INCREMENTAL to v2 — they do
not repeat the full v2 spec.


## Trace Summary

Goal: "Find recent papers on transformer architecture improvements..."
Model: Qwen3-Coder-30B via SGLang
Steps used: 14 of 16
Outcome: Completed, but 8 steps wasted on infrastructure failures.

What worked:
- Stage 1 correctly identified pipeline phase, selected tools, generated concise reasoning
- Phase-hint from ABSTRACT_PLAN correctly propagated through PHASE_TYPE
- Code block generation triggered on pipeline phases (steps 1, 2, 12, 13, 14)
- NEXT_PHASE_TYPE transitions (pipeline → deliberative → pipeline) worked correctly
- Stage 2-PRE correctly skipped for pipeline phases

What failed:
- Step 1: Code block too long (48 lines > 45 limit)
- Steps 4–11: TOOL_ARGS_JSON generation truncated — `load` called 5 times with
  empty args `{` due to SGLang stop token `"\n"` firing after opening brace
- Model unnecessarily tried to `load` a persisted collection when the variable
  `$raw_papers` was still in scope from step 2


## Fix 1: SGLang TOOL_ARGS_JSON Stop Token (CRITICAL)

### Root Cause

Line 1975 in SGLang path:

```python
+ gen(f"tool_args_{step}", max_tokens=1024, temperature=GEN_TEMPERATURE, stop="\n")
```

The model generates `{"target": "transformer_papers_raw", "out": "$loaded"}` but
formats it with a newline after the opening brace:

```
TOOL_ARGS_JSON: {
  "target": "transformer_papers_raw",
  "out": "$loaded"
}
```

The `stop="\n"` fires immediately after `{`, truncating the JSON to `{`.
This caused **5 consecutive identical failures** (steps 4-8) consuming 31% of the
step budget.

### Fix

Change the stop token to allow multi-line JSON:

```python
# OLD (line 1975):
+ gen(f"tool_args_{step}", max_tokens=1024, temperature=GEN_TEMPERATURE, stop="\n")

# NEW:
+ gen(f"tool_args_{step}", max_tokens=1024, temperature=GEN_TEMPERATURE, 
      stop=["\n\n", "\nCODE:", "\nSTAGE", "\nTOOL_NAME:"])
```

Rationale for each stop token:
- `"\n\n"` — blank line after JSON (normal formatting)
- `"\nCODE:"` — transition to code block (Option B)
- `"\nSTAGE"` — next stage marker
- `"\nTOOL_NAME:"` — shouldn't appear, but defensive

### Same-Issue Check: AGENT_STATE_SUPPORT (line 1955)

```python
# OLD (line 1955):
stop="\n"

# NEW:
stop=["\n\n", "\nSTAGE", "\nTOOL_NAME:"]
```

The AGENT_STATE_SUPPORT field is multi-line (one line per hypothesis). The `"\n"`
stop token truncates it to the first line. This was masked in the trace because
the model happened to put all support on one line, but it will fail on multi-line
support blocks.

### vLLM Path

The vLLM path (line ~2986) uses single-block generation with post-hoc label parsing
(`_extract_braced_json_object`) and does NOT have this issue. No change needed.


## Fix 2: Code Block Line Limit

### Problem

The model's natural code style uses comments, blank lines, and per-step error
checks. A 4-step pipeline with guards:

```python
# Step 1: Search
r1 = executor.execute_action_tracked({...}, "codegen")    # 5 lines with dict
if r1["status"] != "success":                              # 2 lines
    return executor._create_uniform_return(...)

# Step 2: Filter                                           # repeat
...
```

This runs ~12 lines per step. A 4-step pipeline = ~48 lines + summary line.

### Fix

**Option A (simple):** Raise the limit from 45 to 60.

```python
# OLD (line 1214):
if len(lines) > 45:
    return False, f"Code block too long ({len(lines)} lines, max 45)"

# NEW:
if len(lines) > 60:
    return False, f"Code block too long ({len(lines)} lines, max 60)"
```

**Option B (better):** Count only lines containing executable code, not comments
and blanks. This lets the model write readable code without gaming the limit:

```python
# NEW validation:
code_lines = [l for l in lines if l.strip() and not l.strip().startswith('#')]
if len(code_lines) > 45:
    return False, f"Code block too long ({len(code_lines)} code lines, max 45)"
# Also enforce a hard total limit to prevent abuse
if len(lines) > 80:
    return False, f"Code block too long ({len(lines)} total lines, max 80)"
```

**Recommendation:** Option B. It separates the safety concern (limiting executable
code) from readability (comments are free).

### Docstring Update

Line 1202 still says "Max 30 lines" — update to match actual limit.


## Fix 3: Variable Lifetime Guidance

### Problem

In step 2, the model bound `$raw_papers` via code block. In step 3, instead of
using `$raw_papers` directly with `filter-structured`, it persisted the collection
and then spent steps 4-11 trying to `load` it back by name. The model doesn't know
that `$raw_papers` is still in scope.

### Fix

Add variable lifetime note to Stage 2 instructions (both paths). Insert after the
Stage 2 FORMAT section, before NUMERIC ARGUMENTS:

```python
"#Stage 2 VARIABLE LIFETIME:\n"
"  Variables created via 'out' parameters in prior steps (including inside\n"
"  code blocks) remain in scope for the entire plan. You do NOT need to\n"
"  persist and reload them. Use $variable_name directly.\n"
"  Example: if step 1 created $papers, step 2 can use target: '$papers'.\n"
"\n"
```

This goes in BOTH the SGLang Stage 2/3 format instructions (~line 1767) and the
vLLM equivalent (~line 2830).


## Fix 4: Spiral Detection Note

### Problem

Steps 4-8 repeated the identical failing action 5 times. The model knew it was
failing (THOUGHTS correctly diagnosed the issue each time) but couldn't escape
because the infrastructure was mangling its output. The Stage 3 reflection
system correctly identified the error but the generated NEXT_TASK was always
"try the same thing with correct format" — which produced the same truncated output.

### Analysis

This is primarily Fix 1 (stop token) — once the JSON isn't truncated, the spiral
won't occur. However, as defense-in-depth, consider adding to Stage 3 instructions:

```python
"SPIRAL DETECTION:\n"
"- If the same tool has failed 2+ times consecutively with the same error,\n"
"  do NOT retry the same approach. Either:\n"
"  (a) Use a different tool to achieve the same sub-goal, or\n"
"  (b) Skip the failing step and proceed with available data, or\n"
"  (c) Use a code block to call the tool with explicit parameters.\n"
"\n"
```

This is a low-priority addition. The stop token fix eliminates the root cause.


## Fix 5: Code Block Error Handling Pattern Guidance

### Problem

The model generates verbose error handling:

```python
r1 = executor.execute_action_tracked({...}, "codegen")
if r1["status"] != "success":
    return executor._create_uniform_return("failed", reason="search failed")
```

This pattern is correct but adds 2-3 lines per step. For a 4-step pipeline,
that's 8-12 lines just for guards. Combined with comments and the summary line,
it pushes past limits.

### Fix

Add a compact error handling pattern to the Option B examples in the Stage 2
FORMAT instructions:

```python
"  Option B - multi-step code block:\n"
"    ...\n"
"    Compact error handling pattern:\n"
"    ```python\n"
"    r1 = executor.execute_action_tracked({...}, \"codegen\")\n"
"    if r1[\"status\"] != \"success\": return executor._create_uniform_return(\"failed\", reason=\"step 1 failed\")\n"
"    r2 = executor.execute_action_tracked({...}, \"codegen\")\n"
"    if r2[\"status\"] != \"success\": return executor._create_uniform_return(\"failed\", reason=\"step 2 failed\")\n"
"    return executor._create_uniform_return(\"success\", value=\"pipeline complete\")\n"
"    ```\n"
```

This shows the model that single-line guards are acceptable, saving ~1 line per step.


## Priority Order

1. **Fix 1 (stop token)** — Critical. 5/14 steps wasted. Must fix before next test.
2. **Fix 2 (line limit)** — High. First step failed. Blocks all 4+ step pipelines.
3. **Fix 3 (variable lifetime)** — Medium. Caused unnecessary persist/load cycle.
4. **Fix 5 (compact guards)** — Medium. Helps stay within line limits.
5. **Fix 4 (spiral detection)** — Low. Defense-in-depth, root cause is Fix 1.


## Verification

After applying fixes 1-3, re-run the same goal:
"Find recent papers on transformer architecture improvements, identify significant
changes and directions, and report"

Expected:
- Step 1: Code block with search → filter → project (should fit in 60 lines or
  45 code lines)
- Step 2: Deliberative analysis using summarize on `$extracted_metadata` (variable
  still in scope, no load needed)
- Step 3: Pipeline code block with generate-note → say → done
- Total: 3-5 steps, no spirals, no load failures
