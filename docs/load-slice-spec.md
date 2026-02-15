# Unified Load with Slicing: Design Spec

## Problem

Three tools — `load`, `head`, `display` — provide overlapping access to content with different truncation behaviors, none controllable by the agent. `load` returns 1024 chars. `head` returns N items from a Collection. `display` is a UI side-effect that also truncates. The agent cannot control how much content enters its reasoning context, leading to:

- Vision evaluator checking truncated output and missing key sections
- Agent auditing against imagined content it has never read
- Repeated load/display calls trying to see more of an artifact
- `head` existing as a separate tool for what is just sliced access to a Collection

## Design

Consolidate `load` and `head` into a single `load` primitive with a `slice` parameter. Eliminate `head`. Keep `display` as a separate UI-only operation.

### Unified `load`

```
load(target, slice?, out)
```

**target**: Note ID/name, Collection ID/name, or $variable.

**slice**: Optional. Python-style slice syntax as a string. Behavior determined by target type:

| Target type | Slice units | Default | Example |
|-------------|-------------|---------|---------|
| Note | Characters | `"0:4096"` | `"500:1500"`, `"-1000:"`, `":"` |
| Collection | Items | `"0:5"` | `"3:8"`, `"-3:"`, `"7"` |

**Slice syntax**:
- `"0:1000"` — first 1000 units (chars or items)
- `"500:1500"` — middle range
- `"-500:"` — last 500 (Python negative-index semantics)
- `":"` — everything (no limit)
- `"5"` — single item (Collection) or char at position (Note, unlikely use)
- Omitted — default slice applies

**Validation:** Rejects only when both start and stop are non-negative and `stop < start`. Standard Python semantics supported, including negative indices (e.g. `"-500:"` for last 500).

**Chunked processing pattern** (for large Notes):
```
load(target=$doc, slice="0:500", out=$chunk1)   → process $chunk1
load(target=$doc, slice="500:1000", out=$chunk2)  → process $chunk2
load(target=$doc, slice="1500:2000", out=$chunk3)  → process $chunk3
```

**Default slices**:
- Note: `"0:4096"` (4x current 1024 default — enough for most single-document evaluation)
- Collection: `"0:5"` (first 5 items)

**Max ceiling**: Ceilings apply only when slice is omitted (default). When slice is explicit (e.g. `":"` for full, `"0:10000"` for first 10k chars), no ceiling — the requested range is returned in full. Default slice `"0:4096"` for Notes and `"0:5"` for Collections use `LOAD_MAX_NOTE_CHARS` and `LOAD_MAX_COLLECTION_ITEMS` respectively.

### Output: two layers

The `out` binding and the `value` string serve different roles:

- **Binding (`out`)**: The real resource. For Notes, the same Note resource ID. For Collections, a new Collection resource ID containing the sliced items. This is what downstream tools (`synthesize`, `map`, `filter-structured`, etc.) consume.

- **Value string**: A content preview for planner context. What the planner sees in the truncated Stage 3 result.
  - **Note**: Prefixed content string (`"Note Content: <text>"`) — the sliced character range.
  - **Collection**: Each item rendered as Note ID + first 200 chars of content. Not concatenated prose, not a bare ID list.

This matches `head`'s existing contract exactly: the bound variable is the real resource, the result string is just for the planner to reason about.

**Collection preview format example** (for `load($papers, slice="0:3")`):
```
Collection Content (3 items):
- Note_42: Attention Is All You Need. We propose a new simple network architecture, the Transformer, based solely on attention mechanisms, dispensing with recurrence and convolutions entirely...
- Note_43: BERT: Pre-training of Deep Bidirectional Transformers for Language Understanding. We introduce a new language representation model called BERT...
- Note_44: Language Models are Few-Shot Learners. Recent work has demonstrated substantial gains on many NLP tasks...
```

### What `head` did, and how `load` replaces it

| Old pattern | New pattern |
|-------------|-------------|
| `head($papers, count=5, out=$top5)` | `load($papers, slice="0:5", out=$top5)` |
| `head($sorted, count=1, out=$first)` | `load($sorted, slice="0", out=$first)` |
| `head($ranked, count=10, out=$top10)` | `load($ranked, slice="0:10", out=$top10)` |

Type flows through: slicing a Collection returns a Collection. Slicing a Note returns text. This means `load($sorted_papers, slice="0:5")` returns a Collection of 5 items that can feed directly into `synthesize`, `map`, `filter-structured`, or any other Collection operation — exactly as `head` did.

All existing `head` usage migrates directly to sliced `load` with no behavioral change. `head` is kept as a deprecated alias (routes to load internally) so in-flight plans don't break.

### `display` remains separate

`display` is a UI side-effect — it renders content for the user in a popup. It does NOT return content into the agent's reasoning context. This distinction is real: `load` is "let me see this so I can think about it," `display` is "show this to the user."

`display` retains its current interface unchanged.

### Vision evaluator integration

The vision evaluator uses `load` with `slice=":"` (full content up to ceiling) to check criteria against the candidate artifact. This replaces the current pattern of truncated-at-512 display results that the evaluator cannot actually read.

**Evaluator targeting — planner provides `eval_target`:**

The planner is in the best position to know which artifact represents progress toward the goal. The evaluation framework should not guess. The planner declares what to evaluate via an `eval_target` field in its step output:

- **Code blocks**: The `return` statement includes `eval_target="$variable_name"` in its `extra` dict:
  ```python
  return executor._create_uniform_return("success", value="done", 
      extra={"eval_target": "$final_report"})
  ```
- **Single tool calls**: The planner's Stage 3 output includes `EVAL_TARGET: $variable_name` (or blank to skip).

**Behavior:**
- If `eval_target` is provided: evaluator loads that artifact with `slice=":"` and checks all vision criteria against it.
- If `eval_target` is blank or omitted: evaluator **skips**. No evaluation runs.
- The evaluator never guesses what to evaluate.

This eliminates two problems from the trace:
1. Evaluating intermediate Collections (note ID lists) that aren't report candidates — the planner simply omits `eval_target` for intermediate pipeline steps.
2. Evaluating the wrong output after a multi-step code block — the planner explicitly names the final artifact.

**Typical pattern**: `eval_target` is set only at phase boundaries where the output is a candidate for the goal artifact. During sourcing and extraction phases, it's omitted. After synthesis, it names the report.

## Eliminated

- `head` primitive — deprecated alias, removed from catalog
- The 1024-char hardcoded truncation in `load` — replaced by configurable default (4096)

## Unchanged

- `display` — UI-only, no changes
- `size` — still returns item count for Collections
- All other Collection operations (`filter-structured`, `sort`, `project`, etc.)

## Additional fix: `ask` tool catalog entry

The planner has minimal one-line documentation for every tool in its upfront catalog. The current `ask` entry is insufficient — it doesn't name the required parameters, causing the agent to guess (`question` instead of `value`) and burn multiple steps on parameter discovery.

**Current catalog entry:**
```
- ask: Ask user a question and wait for response (suspends plan execution).
```

**Revised catalog entry:**
```
- ask: Ask user a question and wait for response (suspends plan execution). 
  Requires: value (the question text), out (variable to bind user's response).
  Example: {"type": "ask", "value": "Should I continue?", "out": "$answer"}
```

**Principle**: Any tool that the planner might call without loading full docs should have its required parameters named in the minimal catalog entry. This is especially true for interactive tools (`ask`, `display`, `say`) that tend to be called ad-hoc rather than planned in advance. The existing entries for `say` and `display` should also be checked for parameter completeness.

## Examples

```json
// Read full report for evaluation (up to 4096 char ceiling)
{"type": "load", "target": "$final_report", "slice": ":", "out": "$report_text"}

// Read last section of a report
{"type": "load", "target": "$final_report", "slice": "-2000:", "out": "$ending"}

// Read first 3 papers from a collection (content preview, not just IDs)
{"type": "load", "target": "$papers", "slice": "0:3", "out": "$sample_papers"}

// Read a single specific item
{"type": "load", "target": "$papers", "slice": "5", "out": "$paper_5"}

// Default load (first 4096 chars of Note, first 5 items of Collection)
{"type": "load", "target": "$final_report", "out": "$report_preview"}
```
