# Spec: Phase-Aware Execution for Incremental Planner

**Version 2 — incorporates Stage 1 scoping analysis**

## Problem

Three related issues in the incremental planner:

1. **Code blocks never get chosen**: Stage 1 decomposes the goal into a single-step
   FIRST_TASK. At Stage 2, the model always picks Option A because the task is
   already scoped to one tool. Option B exists but is never selected.

2. **Deliberative overhead on deterministic work**: Each step incurs a full cycle
   (Stage 2-PRE hypotheses + Stage 2 tool selection + Stage 3 reflection/audit).
   For a 5-step linear pipeline, that's ~15 LLM inference calls for what could be
   1 code block + 1 reflection.

3. **Stage 1 analysis is redundant with _preplan**: The `_preplan` method already
   analyzes the goal, identifies operation types, matches to tools, and produces a
   phased ABSTRACT_PLAN. Stage 1's `<analysis>` block then re-derives this same
   decomposition, spending ~192 tokens restating what the model was already told.
   The only non-redundant work Stage 1 does is: (a) tool selection for doc loading,
   (b) noticing available resources from Stage 0, and (c) extracting the first task.

4. **Primitive gaps cause spirals**: When a pipeline step needs an operation not in
   the primitive catalog (e.g., positional pairing of two collections), the agent
   burns remaining steps trying to approximate with available primitives instead of
   expressing the logic directly in code.


## Approach

Two coordinated changes:

**A. Narrow Stage 1** — Remove the redundant `<analysis>` block. Stage 1 becomes a
   focused extraction step: read the ABSTRACT_PLAN, select tools, determine phase
   type, extract first task. No re-analysis of the goal.

**B. Phase-aware execution** — Pipeline phases are presented as multi-step tasks
   with strong guidance to use code blocks. Deliberative steps retain the full
   hypothesis/audit machinery.


## Files to Modify

All changes are in `src/incremental_planner.py`.

There are two parallel planner implementations that MUST be kept consistent:

- **SGLang path**: function `tool_planner_infospace` (lines ~1400–2133)
  Uses `gen()` calls with stop tokens for structured generation.
- **vLLM path**: function `tool_planner_infospace_vllm` (lines ~2400–3077)
  Uses `vllm_gen()` calls with label parsing for structured generation.

Both paths share identical prompt text but differ in generation mechanics.
Every prompt change described below must be applied to BOTH paths.

Line numbers reference the vLLM path; corresponding SGLang locations are noted.


## Change 1: Narrow Stage 1 — Remove Redundant Analysis

### Rationale

`_preplan` (line ~3780) already performs goal analysis: identifies operation types,
matches to available tools, produces a phased step sequence. The ABSTRACT_PLAN is
injected into the system prompt before Stage 1 runs. Stage 1's `<analysis>` block
re-derives this decomposition. The model spends its first generation restating what
it was already told.

What Stage 1 uniquely contributes:
- **Tool selection** → drives Stage 1.5 doc loading. Keep this.
- **Resource awareness** → can notice Stage 0 resources. But this can be folded
  into the first task extraction without a separate analysis block.
- **Phase type + first task** → new with this spec. Should be framed as *reading*
  the preplan, not re-analyzing the goal.

### Current Stage 1 prompt (vLLM path, lines ~2576–2590)

```python
prompt += format_user(
    "#Stage 1: Analyze goal and identify relevant tools from the Complete primitive and tool catalog.\n"
    "Include tools you might need AND related/supporting tools.\n"
    "Then, decompose the goal into a FIRST high-level task/subgoal to focus on.\n"
    "In doing so, consider the tools you have selected, the goal you are trying to achieve, "
    "and the downstream tasks that will be required to achieve the goal.\n"
    "Respond using the following XML format:\n"
    "<analysis>\n"
    "YOUR REASONING AND THOUGHTS HERE.\n"
    "</analysis>\n"
    "<tools>\n"
    "JSON LIST OF TOOLS HERE\n"
    "</tools>\n"
    "<first_task>\n"
    "YOUR FIRST TASK HERE\n"
    "</first_task>\n"
)
```

### Current SGLang equivalent (lines ~1488–1502)

Same prompt text, different generation: three separate `gen()` calls with stop tokens
for `stage1_analysis`, `selected_tools_json`, and `first_task`.

### New Stage 1 prompt (BOTH paths)

```python
prompt_text = (
    "#Stage 1: Read the ABSTRACT_PLAN above and prepare for execution.\n"
    "\n"
    "1. TOOLS: Select tools you will need from the Complete primitive and tool catalog.\n"
    "   Include tools for the full plan, not just the first step.\n"
    "\n"
    "2. PHASE TYPE: Examine the ABSTRACT_PLAN's first phase.\n"
    "   - pipeline: 2-5 consecutive steps where each feeds the next,\n"
    "     no observation-dependent branching. Execute as a code block.\n"
    "   - deliberative: a single step where you must observe the result\n"
    "     before deciding what to do next.\n"
    "\n"
    "3. FIRST TASK: State what to do first.\n"
    "   - For pipeline phases: describe the ENTIRE phase as one task\n"
    "     (e.g., 'Search for papers, filter to recent, extract metadata,\n"
    "     and build comparison table').\n"
    "   - For deliberative steps: describe just the single next step.\n"
    "\n"
    "Respond using this format:\n"
    "<tools>\n"
    "JSON LIST OF TOOLS\n"
    "</tools>\n"
    "<phase_type>\n"
    "pipeline OR deliberative\n"
    "</phase_type>\n"
    "<first_task>\n"
    "TASK DESCRIPTION\n"
    "</first_task>\n"
)
```

Note: `<analysis>` tag is removed entirely. The model no longer generates
free-form reasoning about the goal at Stage 1. If it needs to reason, that
reasoning already happened in `_preplan`.

### vLLM path generation change (lines ~2593–2603)

**Current**: Single generation with stop on `</first_task>`, then parse three tags.

```python
prompt += format_assistant("")
stage1_block = vllm_gen("stage1_block", prompt, state, max_tokens=384,
                        temperature=GEN_TEMPERATURE, stop="</first_task>", executor=executor)
```

**New**: Same single-generation approach, but shorter (no analysis block to generate).
Reduce max_tokens since the analysis block consumed most of the budget.

```python
prompt += format_assistant("")
stage1_block = vllm_gen("stage1_block", prompt, state, max_tokens=256,
                        temperature=GEN_TEMPERATURE, stop="</first_task>", executor=executor)
# ... existing defensive truncation ...
prompt += stage1_block + "</first_task>\n"
stage1_full = stage1_block + "</first_task>"

# Parse tags — no more stage1_analysis
state["selected_tools_json"] = _extract_tag_block(stage1_full, "tools")
state["phase_type"] = _extract_tag_block(stage1_full, "phase_type").strip().lower()
state["first_task"] = _extract_tag_block(stage1_full, "first_task")

# Normalize phase_type
if state["phase_type"] not in ("pipeline", "deliberative"):
    state["phase_type"] = "deliberative"
```

### SGLang path generation change (lines ~1505–1514)

**Current**: Three separate `gen()` calls: `stage1_analysis` (192 tokens),
`selected_tools_json` (96 tokens), `first_task` (96 tokens).

**New**: Replace with generation for tools, phase_type, and first_task.
Remove the `stage1_analysis` gen() call entirely.

```python
s += assistant(
    "<tools>\n"
    + gen("selected_tools_json", max_tokens=96, temperature=GEN_TEMPERATURE, stop="</tools>")
    + "</tools>\n"
    "<phase_type>\n"
    + gen("phase_type", max_tokens=16, temperature=GEN_TEMPERATURE, stop="</phase_type>")
    + "</phase_type>\n"
    "<first_task>\n"
    + gen("first_task", max_tokens=128, temperature=GEN_TEMPERATURE, stop="</first_task>")
    + "</first_task>\n"
)
```

Note: `first_task` max_tokens increased from 96 to 128 because pipeline phase
descriptions are longer than single-step tasks.

### Logging changes (BOTH paths)

Remove the `stage1_analysis` log line. Add `phase_type`:

```python
logger.info(f"SELECTED_TOOLS_JSON: {state.get('selected_tools_json', 'N/A')}")
logger.info(f"PHASE_TYPE: {state.get('phase_type', 'N/A')}")
logger.info(f"FIRST_TASK: {state.get('first_task', 'N/A')}")
```


## Change 2: Process Overview Summary

### Current (vLLM line ~2564; SGLang line ~1477)

```python
system_parts.append("""Follow this process to achieve the goal:
 - Stage 1 (once): Analyze goal, select relevant tools, decompose into FIRST_TASK.
 - Stage 1.5 (once): Load and inject detailed docs for selected tools.
Then you will work in repeated cycles to achieve the goal:
 - Stage 2: Pick a single tool and JSON args for CURRENT_TASK. Be concise in text value arguments.
 - Stage 3: Reflect on result, decide if goal done, set NEXT_TASK.
ALWAYS follow all formatting instructions exactly.
""")
```

### New (BOTH paths)

```python
system_parts.append("""Follow this process to achieve the goal:
 - Stage 1 (once): Read the ABSTRACT_PLAN, select tools, identify first PHASE type (pipeline or deliberative).
 - Stage 1.5 (once): Load and inject detailed docs for selected tools.
Then you will work in repeated cycles:
 - Stage 2: For PIPELINE phases, write a code block (Option B) executing the full phase.
            For DELIBERATIVE steps, pick a single tool (Option A).
 - Stage 3: Reflect on result, decide if goal done, set NEXT_TASK and NEXT_PHASE_TYPE.
ALWAYS follow all formatting instructions exactly.
""")
```


## Change 3: Stage 2 Instructions — Phase-Aware Guidance

### Current (vLLM lines ~2675–2700; SGLang lines ~1715–1740)

Option A and Option B presented as equals, with Option B described as
"use when CURRENT_TASK needs 2+ sequential tool calls."

### New (BOTH paths)

Replace the entire Stage 2 FORMAT section:

```python
"#Stage 2 FORMAT:\n"
"  The format depends on the current PHASE_TYPE.\n"
"\n"
"  PIPELINE phases (CURRENT_TASK describes multiple known steps):\n"
"    You SHOULD use Option B (code block) to execute the entire phase.\n"
"    Only fall back to Option A if the phase truly requires just one tool call.\n"
"\n"
"  DELIBERATIVE steps (you need to observe before deciding next):\n"
"    Use Option A (single tool call).\n"
"    Option B is available if you realize 2-3 calls can be safely chained.\n"
"\n"
"  Option A - single tool call:\n"
"    TOOL_NAME: <name from the Complete primitive and tool catalog>\n"
"    TOOL_ARGS_JSON: <json object>\n"
"\n"
"  Option B - multi-step code block:\n"
"    TOOL_NAME: _code_block_\n"
"    TOOL_ARGS_JSON: {}\n"
"    CODE:\n"
"    ```python\n"
"    r1 = executor.execute_action_tracked({\"type\": \"semantic-scholar\", \"query\": \"transformers\", \"limit\": 20, \"out\": \"$papers\"}, \"codegen\")\n"
"    if r1[\"status\"] != \"success\":\n"
"        return executor._create_uniform_return(\"failed\", reason=\"search failed\")\n"
"    r2 = executor.execute_action_tracked({\"type\": \"filter-structured\", \"target\": \"$papers\", \"where\": \"metadata.year >= 2020\", \"out\": \"$recent\"}, \"codegen\")\n"
"    r3 = executor.execute_action_tracked({\"type\": \"summarize\", \"target\": \"$recent\", \"focus\": \"key findings\", \"out\": \"$summary\"}, \"codegen\")\n"
"    return executor._create_uniform_return(\"success\", value=\"pipeline complete\")\n"
"    ```\n"
"    Rules for Option B:\n"
"    - Max 6 tool calls via executor.execute_action_tracked(action_dict, \"codegen\")\n"
"    - if/else control flow is allowed.\n"
"    - Bounded iteration over KNOWN SMALL collections is allowed:\n"
"      e.g., `for item in [item1, item2, item3]:` (max 5 items, list must be literal or a variable\n"
"      whose length was observed in a prior step).\n"
"    - No while loops, imports, function defs, or unbounded iteration.\n"
"    - Must end with: return executor._create_uniform_return(status, value=..., extra=...)\n"
"\n"
```


## Change 4: Inject PHASE_TYPE into Stage 2 Step Prompt

### Current Stage 2 step prompt (vLLM lines ~2840–2844; SGLang lines ~1889–1893)

```python
# vLLM:
prompt += format_user(
    f"STAGE 2 (step {step + 1}/{max_steps}):\n"
    f"#GOAL: {goal_for_step}\n#END GOAL\n"
    f"CURRENT_TASK: {current_task}\n"
    "Choose tool and JSON args using Stage 2 FORMAT.\n"
)

# SGLang:
s += user(
    f"STAGE 2 (step {step + 1}/{max_steps}):\n"
    f"#GOAL: {goal_for_step}\n#END GOAL\n"
    f"CURRENT_TASK: {current_task}\n"
    "Choose tool and JSON args using Stage 2 FORMAT.\n"
)
```

### New (BOTH paths)

Add `PHASE_TYPE:` line:

```python
f"STAGE 2 (step {step + 1}/{max_steps}):\n"
f"#GOAL: {goal_for_step}\n#END GOAL\n"
f"CURRENT_TASK: {current_task}\n"
f"PHASE_TYPE: {current_phase_type}\n"
"Choose tool and JSON args using Stage 2 FORMAT.\n"
```

Where `current_phase_type` is a loop variable initialized and updated as described
in Change 7.


## Change 5: Stage 2-PRE — Skip for Pipeline Phases

### Rationale

Stage 2-PRE agent state hypotheses are for deliberative steps where the model
reasons about constraints before choosing a tool. For pipeline phases, the model
already knows what it's going to do — hypothesis generation is wasted inference.

### Implementation (BOTH paths)

In the main loop, wrap Stage 2-PRE in a conditional.

**vLLM path** (lines ~2793–2837):

```python
if current_phase_type == "deliberative":
    # Full Stage 2-PRE: Agent-State Hypotheses
    prompt_parts = [
        f"STAGE 2-PRE (step {step + 1}/{max_steps}):\n",
        f"#GOAL: {goal_for_step}\n#END GOAL\n",
        f"CURRENT_TASK: {current_task}\n\n"
    ]
    # ... existing voxel grid / AF1 injection for minecraft ...
    prompt_parts.append(
        "Before choosing any tool, infer AGENT-STATE HYPOTHESES and "
        "AGENT-STATE-SUPPORT. Refer to STAGE 2-PRE Instructions.\n"
    )
    prompt += format_user("".join(prompt_parts))
    prompt += format_assistant("")
    stage2pre_block = vllm_gen(f"stage2pre_block_{step}", prompt, state,
                               max_tokens=512, temperature=GEN_TEMPERATURE, executor=executor)
    prompt += stage2pre_block + "\n"
    state[f"agent_state_hypotheses_{step}"] = _extract_between_labels(
        stage2pre_block, "AGENT_STATE_HYPOTHESES:", "AGENT_STATE_SUPPORT:")
    state[f"agent_state_support_{step}"] = _extract_after_label(
        stage2pre_block, "AGENT_STATE_SUPPORT:").strip()
else:
    # Pipeline phase: skip Stage 2-PRE entirely
    logger.info(f"Step {step}: Pipeline phase — skipping Stage 2-PRE")
    state[f"agent_state_hypotheses_{step}"] = "[]"
    state[f"agent_state_support_{step}"] = ""
```

**SGLang path** (lines ~1854–1887): Same conditional wrapping the `gen()` calls
for `agent_state_hypotheses_{step}` and `agent_state_support_{step}`.

```python
if current_phase_type == "deliberative":
    s += user("".join(prompt_parts))
    s += assistant(
        "AGENT_STATE_HYPOTHESES:\n"
        + gen(f"agent_state_hypotheses_{step}", max_tokens=256,
              temperature=GEN_TEMPERATURE, stop="\nAGENT_STATE_SUPPORT:")
        + "\nAGENT_STATE_SUPPORT:\n"
        + gen(f"agent_state_support_{step}", max_tokens=256,
              temperature=GEN_TEMPERATURE, stop="\n")
    )
else:
    logger.info(f"Step {step}: Pipeline phase — skipping Stage 2-PRE")
```


## Change 6: Stage 3 — Add NEXT_PHASE_TYPE

### Current Stage 3 FORMAT (vLLM lines ~2707–2716; SGLang lines ~1747–1756)

```
THOUGHTS: <text>
HYPOTHESES: [...]
AUDIT: ...
DONE: <YES or NO>
NEXT_TASK: <next high-level subgoal or blank>
REQUEST_TOOLS: <json array>
```

### New Stage 3 FORMAT (BOTH paths)

Insert `NEXT_PHASE_TYPE` between `NEXT_TASK` and `REQUEST_TOOLS`:

```
THOUGHTS: <text>
HYPOTHESES: [...]
AUDIT: ...
DONE: <YES or NO>
NEXT_TASK: <next high-level subgoal or blank>
NEXT_PHASE_TYPE: <pipeline or deliberative>
REQUEST_TOOLS: <json array>
```

### Add to Stage 3 INSTRUCTIONS (BOTH paths)

After the existing NEXT_TASK instructions, add:

```python
"NEXT_PHASE_TYPE:\n"
"- If NEXT_TASK describes 2+ steps that form a deterministic pipeline\n"
"  (each feeds the next, no branching on results), set: pipeline\n"
"- If NEXT_TASK requires observing a result before deciding what follows,\n"
"  set: deliberative\n"
"- Default: deliberative\n"
"\n"
```

### Parsing changes

**vLLM path** (lines ~2943–2944):

Current:
```python
next_task_val = _extract_between_labels(stage3_block, "NEXT_TASK:", "REQUEST_TOOLS:")
request_tools_val = _extract_after_label(stage3_block, "REQUEST_TOOLS:").strip()
```

New:
```python
next_task_val = _extract_between_labels(stage3_block, "NEXT_TASK:", "NEXT_PHASE_TYPE:")
next_phase_type_val = _extract_between_labels(stage3_block, "NEXT_PHASE_TYPE:", "REQUEST_TOOLS:")
request_tools_val = _extract_after_label(stage3_block, "REQUEST_TOOLS:").strip()

state[f"next_phase_type_{step}"] = next_phase_type_val.strip().lower() \
    if next_phase_type_val.strip().lower() in ("pipeline", "deliberative") \
    else "deliberative"
```

If parsing fails (model omits NEXT_PHASE_TYPE), fall back:
```python
# Fallback: if NEXT_PHASE_TYPE not found, try direct extraction from NEXT_TASK to REQUEST_TOOLS
if not next_task_val.strip():
    next_task_val = _extract_between_labels(stage3_block, "NEXT_TASK:", "REQUEST_TOOLS:")
    state[f"next_phase_type_{step}"] = "deliberative"
```

**SGLang path** (lines ~1992–2004):

Add a `gen()` call for `next_phase_type_{step}` between `next_task_{step}` and
`request_tools_{step}`:

```python
+ "\nNEXT_PHASE_TYPE: "
+ gen(
    f"next_phase_type_{step}",
    max_tokens=16,
    temperature=GEN_TEMPERATURE,
    stop=["\nREQUEST_TOOLS:", "\n\n"]
)
+ "\nREQUEST_TOOLS: "
```

Update the stop tokens for `next_task_{step}` to stop on `\nNEXT_PHASE_TYPE:`
instead of `\nREQUEST_TOOLS:`.


## Change 7: Main Loop — Phase Type Variable

### Initialization (BOTH paths)

Before the main `for step in range(max_steps):` loop:

**vLLM** (after line ~2782):
```python
current_task = state.get("first_task", "").strip()
current_phase_type = state.get("phase_type", "deliberative")  # NEW
if not current_task:
    logger.warning("No first_task found, using goal as initial task")
    current_task = goal_for_step
```

**SGLang** (after line ~1822):
```python
current_task = s["first_task"].strip()
current_phase_type = s.get("phase_type", "deliberative") if hasattr(s, 'get') else "deliberative"  # NEW
```

Note: SGLang's ProgramState uses bracket notation and may not have `.get()`.
Use a try/except fallback if needed:
```python
try:
    current_phase_type = s["phase_type"].strip().lower()
    if current_phase_type not in ("pipeline", "deliberative"):
        current_phase_type = "deliberative"
except (KeyError, TypeError, AttributeError):
    current_phase_type = "deliberative"
```

### Update at end of loop iteration (BOTH paths)

At the point where `current_task` is updated from NEXT_TASK (vLLM line ~3054;
SGLang line ~2112):

```python
# Update current task for next iteration
next_task_raw = state.get(f"next_task_{step}", "").strip()  # or s[f"next_task_{step}"]
if next_task_raw and next_task_raw.lower() not in ["", "none", "null", "n/a"]:
    current_task = next_task_raw
    # Update phase type from Stage 3 output
    current_phase_type = state.get(f"next_phase_type_{step}", "deliberative")  # NEW
    logger.info(f"Step {step}: Next task: {current_task} (phase: {current_phase_type})")
else:
    logger.warning(f"Step {step}: No NEXT_TASK provided, stopping")
```


## Change 8: Relax Bounded Iteration in validate_codegen_block

### Current forbidden patterns (line ~1179)

```python
_CODEGEN_FORBIDDEN_PATTERNS = [
    r'\bimport\b',
    r'\bfrom\b\s+\w+\s+import\b',
    r'\bexec\b\s*\(', r'\beval\b\s*\(', r'\bcompile\b\s*\(',
    r'\b__\w+__\b',
    r'\bopen\b\s*\(',
    r'\bos\b\.', r'\bsys\b\.', r'\bsubprocess\b',
    r'\bwhile\b', r'\bfor\b',               # ← both banned
    r'\bclass\b', r'\bdef\b',
    r'\bglobals\b\s*\(', r'\blocals\b\s*\(',
    r'\bgetattr\b\s*\(', r'\bsetattr\b\s*\(', r'\bdelattr\b\s*\(',
]
```

### New forbidden patterns

Remove `r'\bfor\b'` from the list. Keep `r'\bwhile\b'`.

```python
_CODEGEN_FORBIDDEN_PATTERNS = [
    r'\bimport\b',
    r'\bfrom\b\s+\w+\s+import\b',
    r'\bexec\b\s*\(', r'\beval\b\s*\(', r'\bcompile\b\s*\(',
    r'\b__\w+__\b',
    r'\bopen\b\s*\(',
    r'\bos\b\.', r'\bsys\b\.', r'\bsubprocess\b',
    r'\bwhile\b',
    # Note: 'for' is allowed but validated separately for bounded iteration
    r'\bclass\b', r'\bdef\b',
    r'\bglobals\b\s*\(', r'\blocals\b\s*\(',
    r'\bgetattr\b\s*\(', r'\bsetattr\b\s*\(', r'\bdelattr\b\s*\(',
]
```

### Add bounded-for validation in validate_codegen_block (after line ~1220)

After the forbidden pattern scan, before the execute_action_tracked count check:

```python
# Check for loops: allow bounded 'for' over short literal lists, reject all else
for_matches = list(re.finditer(r'\bfor\b\s+\w+\s+in\s+', code))
for match in for_matches:
    rest_of_line = code[match.end():].split('\n')[0].strip().rstrip(':')

    # Allow: for X in [literal1, literal2, ...]
    if re.match(r'\[.*\]$', rest_of_line):
        items = rest_of_line[1:-1].split(',')
        if len(items) > 5:
            return False, f"For loop over list with {len(items)} items (max 5)"
        continue

    # Allow: for i, X in enumerate([...])
    if re.match(r'enumerate\s*\(\s*\[.*\]\s*\)$', rest_of_line):
        inner = re.search(r'\[(.*)\]', rest_of_line)
        if inner:
            items = inner.group(1).split(',')
            if len(items) > 5:
                return False, f"For loop with enumerate over {len(items)} items (max 5)"
        continue

    # Allow: for X in $variable (variable from prior step, bounded by context)
    if re.match(r'\$\w+', rest_of_line):
        continue

    # Reject all other for loops (range(), generator expressions, etc.)
    return False, f"Unbounded or disallowed for loop: for ... in {rest_of_line[:50]}"
```

### Increase limits in validate_codegen_block

```python
# Line count: 30 → 45
if len(lines) > 45:
    return False, f"Code block too long ({len(lines)} lines, max 45)"

# execute_action_tracked calls: 6 → 10
# (bounded loop over 3 items × 2 calls each = 6 from loop alone)
if call_count > 10:
    return False, f"Too many execute_action_tracked() calls ({call_count}, max 10)"
```


## Change 9: ToolModel trace parser (if applicable)

The `ToolModel` class in `tool_model.py` parses traces to extract step records.
If it currently keys on `FIRST_TASK:` or `stage1_analysis`, update to handle the
new tag structure (no `<analysis>`, added `<phase_type>`). This is a low-priority
cosmetic change since it only affects post-hoc trace analysis.


## Consistency Checklist

| Component | vLLM path | SGLang path |
|-----------|-----------|-------------|
| Stage 1 prompt text | Change 1 | Change 1 (same text) |
| Stage 1 generation / parsing | `vllm_gen` + tag extraction | `gen()` with stop tokens |
| Process overview | Change 2 | Change 2 (same text) |
| Stage 2 format instructions | Change 3 | Change 3 (same text) |
| Stage 2 step prompt (PHASE_TYPE) | Change 4 | Change 4 (same text) |
| Stage 2-PRE conditional | Change 5 (format_user/vllm_gen) | Change 5 (user/gen) |
| Stage 3 format + instructions | Change 6 | Change 6 (same text) |
| Stage 3 parsing (NEXT_PHASE_TYPE) | `_extract_between_labels` | New `gen()` call |
| Main loop phase_type variable | Change 7 (state dict) | Change 7 (ProgramState) |
| validate_codegen_block | Change 8 | Change 8 (shared function) |


## Testing

### Test 1: Pipeline Detection

Goal: "Search for three recent papers on recursive language models. For each,
extract title, authors, main contribution. Compile into comparison table."

Expected: Stage 1 emits `<phase_type>pipeline</phase_type>`. Stage 2 receives
`PHASE_TYPE: pipeline` and produces a code block. Stage 2-PRE is skipped.

### Test 2: Deliberative Fallback

Goal: "Find information about [obscure topic] and determine if it's real."

Expected: Stage 1 emits `<phase_type>deliberative</phase_type>`. Full Stage 2-PRE
runs. Option A chosen at Stage 2.

### Test 3: Phase Transition

Goal: "Compare weather in Berkeley and SF. If either has rain, search for indoor
activities there."

Expected: First phase is pipeline (search both). After Stage 3, model sets
`NEXT_PHASE_TYPE: deliberative` (need to check rain results before deciding).

### Test 4: Bounded Iteration

Goal: "For each of these three topics [A, B, C], search and extract the main finding."

Expected: Code block with `for topic in ["A", "B", "C"]:` containing search + refine.
`validate_codegen_block` accepts this.

### Test 5: Backward Compatibility

Goal: Any goal where ABSTRACT_PLAN lacks PHASE annotations (e.g., retry path
where preplan="No preplan provided").

Expected: `phase_type` defaults to "deliberative". Planner behaves identically to
current behavior. No regression.

### Test 6: Analysis Removal Doesn't Break Tool Selection

Goal: Any multi-tool goal.

Expected: Stage 1 still correctly selects tools in `<tools>` tag without the
`<analysis>` reasoning block. Stage 1.5 doc loading works as before.


## Non-Goals

- Does NOT add new infospace primitives (zip, get-nth, etc.). Separate improvement.
- Does NOT change `_preplan()` prompt. Already updated separately.
- Does NOT change the reflection system, world model, or verification logic.
- Does NOT change trace file output format (traces will naturally show the new
  fields but existing parsers should degrade gracefully).


## Migration Notes

- `current_phase_type` variable must be initialized before the main loop and
  updated each iteration. Defaults to "deliberative" throughout.

- If `_preplan` output lacks PHASE annotations (old format or "No preplan
  provided"), Stage 1 will default to deliberative. No regression.

- The `state["stage1_analysis"]` key will no longer be populated. Any code
  that reads this key (logging, trace parsing) should use `.get()` with a
  default or be updated.

- The SGLang path's ProgramState may not support `.get()` with defaults.
  Use try/except for all new key accesses.
