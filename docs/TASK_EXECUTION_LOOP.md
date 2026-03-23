# Task Execution Loop: Current State & Analysis

## Overview

This document describes the current operational task execution loop
(`_advance_task_execution`) — how it works, how it compares to the
incremental planner it dispatches goals into, and the gap between
the two that explains the looping/quality issues observed in practice.

---

## Current Operational Loop: How It Works

### The Outer Loop (Task Level)

Each tick, the executive node:

1. **Selects a task** via `_select_next_task()` — round-robin by staleness,
   running tasks get priority, budget-gated
2. **Calls `_advance_task_execution(task_note_name)`** — the "outer planner"
3. This method:
   - Reads the task WIP (intention, cycle state, findings, history)
   - Formats `_OPERATIONAL_TASK_PROMPT` with accumulated context
   - Makes **one LLM call** (400 tokens, temperature 0.3)
   - Parses: `ACTION: SUBMIT_GOAL | CYCLE_DONE` + `GOAL_TEXT: ...`
   - If SUBMIT_GOAL: dispatches goal text via `parse_and_set_goal()`
   - If CYCLE_DONE: finalizes cycle, enters cooldown
4. Goal executes via the **incremental planner** (inner loop)
5. On completion, `_record_operational_goal_result()` captures the result
6. Next tick: re-advances the same task (if mid-cycle) or selects another

### The Inner Loop (Goal Level — Incremental Planner)

Each dispatched goal runs through the full incremental planner pipeline:

```
Stage 0: Resource retrieval (semantic search for relevant Notes/Collections)
Stage 1: Tool selection + first task planning (3 structured outputs)
Stage 1.5: Detailed tool documentation injection (SKILL.md loading)
Stage 2/3 Loop (up to 16 steps):
  Stage 2: Code block generation (Python, temperature 0.2)
           Code block execution (scoped Python environment)
  Stage 3: Structured reflection (6 fields: THOUGHTS, EVAL_TARGET,
           DONE, NEXT_TASK, REQUEST_TOOLS, ASK_USER)
  Stage 3.1: Resource commentary update
  Stage 3.5: Dynamic tool loading (if REQUEST_TOOLS=YES)
  Stage 3.6: ASK_USER (if stuck, at most once)
  Done Gate: Verification + deep vision eval (quality assurance)
Post-loop: Last-chance synthesis if step limit reached
```

---

## The Gap: Outer vs Inner Sophistication

### What the Outer Loop Does

| Aspect | Operational Loop |
|--------|-----------------|
| Prompt | 18-line template |
| LLM calls per decision | 1 |
| Decision output | ACTION + GOAL_TEXT (free text) |
| Context provided | Intention, findings, history |
| Tool awareness | None |
| Code generation | None |
| Vision evaluation | None |
| Done verification | None |
| Stall detection | Hard cap only (5 goals) |
| Reflection | None |
| Result evaluation | Appends raw result summary |
| Quality control | None |

### What the Inner Loop Does

| Aspect | Incremental Planner |
|--------|---------------------|
| Prompt | Multi-page system prompt with type system, rules |
| LLM calls per step | 3-5 (code gen, reflection, eval) |
| Decision output | Executable Python code block |
| Context provided | Full infospace state, bindings, tool docs, world model |
| Tool awareness | Explicit catalog, dynamic doc loading |
| Code generation | Yes (temperature 0.2, stop tokens, validation) |
| Vision evaluation | Yes (shallow + deep, cross-reference) |
| Done verification | Yes (VERIFICATION_ANSWER gate) |
| Stall detection | Signature-based repeat detection + nudge |
| Reflection | 6-field structured assessment per step |
| Result evaluation | Ground truth comparison + THOUGHTS |
| Quality control | Vision criteria, deep eval, revision loop |

### The Practical Consequence

The outer loop generates **a sentence** ("Run the discovery script to scan
the workspace for stale notes..."). The inner loop then takes that sentence
and runs a full multi-step planning process with code generation, tool
selection, execution, reflection, and quality gates.

The problem: the outer loop has no way to evaluate whether the goal it
generated was appropriate, achievable, or well-scoped. It just sees a
result summary and decides what to do next. When the inner loop fails
(loops on metadata extraction, hits `target_tokens` bug, etc.), the outer
loop can't diagnose the failure — it just generates another similar goal.

---

## Key Lessons from the Inner Planner

### 1. The Product is Code, Not Aspirational Text

The planner's basic unit of work is a **Python code block** — procedural,
executable, testable. Each step produces concrete actions:

```python
r = tool("check-health", out="$health")
health = get_json("$health")
report = f"Status: {health.get('overall_status')}"
return executor._create_uniform_return("success", value=report)
```

The task loop's basic unit is a **goal sentence** — aspirational, vague,
open to interpretation:

```
"Run the discovery script to scan the workspace for stale notes..."
```

The planner succeeds because code is unambiguous. The task loop struggles
because goal sentences leave too much to the inner planner's interpretation.

**Question for task planning**: Could the outer loop generate something
more structured than free text? Not full code, but perhaps structured
goal specifications with explicit inputs, expected outputs, and success
criteria?

### 2. Structured Reflection After Every Step

The planner evaluates every step with 6 structured fields:

```
THOUGHTS: What happened? What did we learn?
EVAL_TARGET: What artifact represents progress?
DONE: Is the entire goal satisfied?
NEXT_TASK: What should we do next?
REQUEST_TOOLS: Do we need more tool information?
ASK_USER: Are we stuck?
```

The task loop has **no reflection**. It just appends the raw result summary
and asks the LLM what to do next. There's no structured assessment of:
- Did the goal achieve what we intended?
- Was the result useful or garbage?
- Should we retry, pivot, or move on?
- Is the overall task making progress or spinning?

### 3. Vision Criteria and Quality Gates

The planner generates **testable failure criteria** before execution:

```
1. output_empty: "len(output.strip()) == 0"
2. topic_mismatch: "not any(keyword in output.lower() for keyword in [...])"
3. missing_schema_fields: "output is missing required fields"
```

Then at the done gate, it evaluates the artifact against these criteria
with cross-referencing against upstream sources. If it fails, it gets
one retry.

The task loop has **no quality criteria**. It can't tell if a cycle goal
produced useful output or garbage. It can't verify that the audit log
was populated correctly. It relies entirely on the inner planner's quality
gates, which operate at the wrong level (checking the goal artifact, not
the task-level progress).

### 4. Result Evaluation is Ground Truth

The planner injects the actual execution result as ground truth:

```
>> RESULT (ground truth) <<
{tool_result}
>> END RESULT <<

Evaluate: Is the GOAL fully achieved? Use ONLY the result above.
```

The task loop captures `clean_response[:300]` — a truncated summary.
The LLM sees "Goal result: audit log created and persisted" but has no
way to verify whether the log actually contains useful data. The planner
would load the artifact and check.

### 5. Stall Detection Based on Behavior Signatures

The planner detects stalls by comparing **normalized task + action type
signatures**:

```python
signature = (next_task_sig[:220], tuple(action_types[:8]))
if signature == prev_signature:
    repeat_count += 1
```

After 2 repeats, it nudges. After 3, it breaks out.

The task loop has **no equivalent**. It can't detect that it's generating
the same goal text with the same result pattern. The 5-goal cap is a blunt
instrument — it doesn't distinguish between 5 productive goals and 5
repetitions of the same failed approach.

### 6. Dynamic Tool Loading

The planner can request detailed documentation mid-execution:

```
REQUEST_TOOLS: YES
REQUEST_TOOLS_LIST: ["extract", "map", "filter-structured"]
```

This loads SKILL.md files with full parameter specs, examples, and
constraints. This is why the planner eventually learns that `map(extract)`
needs an `instruction` parameter — the doc injection tells it.

The task loop has no analog. When it generates "Run the discovery script..."
it has no control over which tools the inner planner will attempt to use
or whether it has the documentation to use them correctly.

---

## What a Better Outer Loop Would Look Like

Drawing from the inner planner's architecture:

### Structured Goal Specification

Instead of:
```
GOAL_TEXT: Run the discovery script to scan workspace for stale notes
```

Generate:
```
GOAL_SPEC:
  objective: Discover and catalog stale workspace resources
  inputs:
    - audit-stale-resources-config (load by name)
  expected_output:
    - Updated audit-stale-resources-log with discovered items
  success_criteria:
    - audit-stale-resources-log is non-empty after execution
    - At least 1 discovered item has metadata populated
  tools_hint: [discover-notes, load, create-note, persist]
  max_steps: 8
```

### Structured Result Evaluation

After each goal, instead of just appending the summary, evaluate:

```
CYCLE_REFLECTION:
  goal_achieved: YES | PARTIAL | NO
  evidence: "audit-stale-resources-log now contains 5 items with metadata"
  progress_toward_intention: "Discovery complete, archival pending"
  next_action: SUBMIT_GOAL | RETRY | PIVOT | CYCLE_DONE
  retry_reason: "" (if RETRY)
  pivot_reason: "" (if PIVOT)
```

### Task-Level Quality Criteria

Generated once at cycle start (like the planner's vision criteria):

```
CYCLE_QUALITY_CRITERIA:
  1. "audit-stale-resources-log has at least 1 non-empty discovered_items entry"
  2. "Each item has last_modified, status, and path fields populated"
  3. "No duplicate items in the log"
```

Checked after each goal to measure progress.

### Stall Detection

Track goal signatures across the cycle:

```python
goal_signature = normalize(goal_text)[:200]
if goal_signature == prev_goal_signature:
    stall_count += 1
if stall_count >= 2:
    # Force CYCLE_DONE or PIVOT
```

---

## Open Questions

1. **Code vs Text Goals**: The inner planner's strength is code generation.
   Should the outer loop generate code-like specifications rather than
   free text? Or is the two-level separation (task decides WHAT, planner
   decides HOW) the right abstraction?

2. **Quality Criteria Placement**: Should quality criteria live at the task
   level (checking cycle progress) or the goal level (checking goal output)?
   Currently they only exist at the goal level inside the planner.

3. **Result Fidelity**: The outer loop sees a 300-char summary. Should it
   load the actual artifact to verify? This would add an LLM call but
   catch the "audit log has placeholder entries" problem that the current
   loop misses.

4. **Retry vs Pivot**: The current loop has no concept of retrying a
   failed goal with different parameters vs pivoting to a different
   approach. It just generates another goal, which may be identical.

5. **Procedural Hints**: Should the outer loop provide the inner planner
   with hints about HOW to accomplish the goal (e.g., "use synthesize
   on the collection, not extract on individual notes")? This bridges
   the gap between task-level knowledge and planner-level execution.
