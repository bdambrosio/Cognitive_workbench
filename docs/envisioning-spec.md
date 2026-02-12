# Envisioning: Design Spec

## Problem

The incremental planner executes goals competently — it recovers from tool failures, diagnoses empty results, retries with adjusted parameters. But it has no sense of whether its *output* is any good. It checks tool return codes, not artifact quality. The result is structurally correct but substantively shallow work.

The root cause: the agent has no image of the target artifact. A human assistant hearing "find recent papers on transformer improvements and report" immediately forms a concrete vision — length, depth, audience, what "good" looks like. That vision is the reference signal for continuous quality assessment. Without it, the agent has no way to detect the gap between what it's producing and what's needed.

## Design

Two components, minimal surface area. No new tool parameters. The existing planner loop handles error recovery and replanning — don't replace it, feed it better information.

### 1. Vision Generator

At task start, before planning or execution, the LLM generates 2–4 concrete, evaluable criteria from the goal statement. These are code-evaluable or `assess`-evaluable metrics that make the LLM's implicit model of "done" explicit.

**Input:** Goal string + any available context (user model, prior interactions, domain).

**Output:** A small set of criteria, each one a testable predicate.

Example — given goal "Find recent papers on transformer architecture improvements, identify significant changes and directions, and report":

```
min_sources: 8
detail_test: "contains specific method names and performance characteristics, not just category labels"
coverage_test: "addresses at least 3 distinct architectural themes"  
depth_test: "includes comparative or contrastive analysis, not just enumeration"
```

**Key constraint:** Criteria should be concrete on *quality dimensions* (depth, specificity, audience expectations) and deliberately open on *content* (which themes, which methods). Content specifics emerge from the material; quality standards are known upfront.

**Implementation:** Single LLM call with a focused prompt. The goal string already implies what "done" looks like — we're just asking the LLM to surface that as checkable predicates. Numeric criteria (min_sources) are checked in code. Qualitative criteria (detail_test, depth_test) are checked via `assess` against the current output.

**Where the vision lives:** System prompt injection. The criteria are 2–6 lines of text appended to the planner system prompt. The token cost is negligible and they're always visible to the planner throughout execution. No new data structures, no separate storage.

### 2. Vision Evaluator

After steps that produce artifacts (synthesize, extract, code blocks that bind new variables), a lightweight check asks whether the output meets the vision criteria. This uses `call_subplanner` which has access to variable bindings and can run `assess` against actual content.

**When:** After steps where a new artifact is bound to a variable — specifically after `synthesize`, `extract`, and code blocks that produce new bindings. Not at every step.

**Heuristic:** Only evaluate if the step produced something that could plausibly be the final or near-final artifact. In practice: check `action_type in ("synthesize", "extract", "_code_block_")` and confirm new bindings were created.

**How:** A short `call_subplanner` invocation with goal: "Evaluate $variable against these criteria: {vision_criteria}. For each criterion, report pass/fail and a one-sentence gap description if failing." The subplanner has access to bindings and can use `assess` for qualitative checks and code for numeric checks.

**Output:** Pass/fail per criterion plus gap descriptions. Fed directly into the next Stage 3 reflection as additional context. The planner's existing recovery loop can then respond — e.g., re-running extraction with a more detailed instruction, or re-running synthesis with a different focus.

**Vision updates:** Not in v1. The criteria are set once at the start. If we find they need to evolve, that's a natural extension but adds complexity we don't need yet.

## Integration with Existing Architecture

The planner loop is unchanged. Two additions plug in at defined points:

```
[goal received]
    |
    v
[vision generator] --> 2-6 lines of criteria (single LLM call)
    |
    v
[criteria injected into planner system prompt]
    |
    v
[planner generates + executes steps as normal]
    |
    v
[after artifact-producing steps: lightweight eval via subplanner]
    |                   pass/fail + gap descriptions
    |                   (fed back as Stage 3 context)
    v
[planner decides: next phase, retry, or done]
```

No new control flow. No new tool parameters. The vision criteria and evaluation results become part of the planner's reasoning context alongside tool results and error messages.

## What This Doesn't Address (Yet)

**Detail parameter.** The existing tools (extract, synthesize) already have `instruction`, `format`, `compression_ratio`, and `focus` parameters. The planner can steer these in response to evaluation feedback without needing a new `detail` parameter. Zero implementation cost on the tool side.

**Recursive envisioning.** The vision could decompose recursively — a report vision spawns section visions spawns paragraph visions. This spec keeps it flat: one set of criteria for the whole task.

**Vision updates.** The vision could co-evolve with the work. Deferred — set once, evaluate against the same criteria throughout.

## Success Criteria for This Spec

Applied to the transformer architecture trace, this design should produce:

1. Upfront criteria that would catch the shallow output (specifically: detail_test and depth_test)
2. A post-extraction evaluation that flags "16 one-sentence summaries don't meet depth_test"
3. A planner response that re-runs extraction with a more detailed `instruction` parameter
4. A final output that contains specific architectural details, method comparisons, and performance characteristics — not just category labels
