# Plan Review Prompt

This file guides the review of a Cognitive Workbench goal plan. It is used by
a reviewer (human or LLM) to critique and improve cached plan actions before
replay. Update this file after each review cycle with new patterns discovered.

## How to Use

1. Run `/goal plan review <goal_id>` in Jill's CLI to generate a review bundle.
2. Open the review bundle (`goal_review_<goal_id>.md`) alongside this prompt.
3. If a trace exists (`goal_trace_<goal_id>.txt`), use it to understand planner
   reasoning — grep for specific tool calls or variable names rather than
   reading the whole trace.
4. Review the plan in `goal_plan_<goal_id>.py` (run `/goal plan edit <goal_id>`
   if the file doesn't exist).
5. Apply the checks below. Fix issues directly in the plan file.
6. Run `/goal plan load <goal_id>` to load the fixed plan.
7. Add any new failure patterns you discover to this file.

---

## Review Checklist

### 1. Goal Alignment

- Read the goal text in the review bundle header.
- Does the plan actually accomplish what the goal describes?
- Are there goal requirements that no step addresses?
- Are there steps that do work the goal didn't ask for?

### 2. Inter-Step Data Flow

Plans are sequences of independent code blocks executed by `execute_plan_sync`.
Each step runs in its own scope. The ONLY way to pass data between steps is
through infospace Notes via `tool("create-note", ..., out="$var")`.

Common failures:
- **Return values don't cross steps.** A step's `return` dict (including `extra`)
  is stored in goal metadata but is NOT accessible to the next step. If step 2
  needs a list computed by step 1, step 1 must write it to a Note.
- **Variable reuse in loops.** Using the same `out="$var"` in a loop overwrites
  the previous iteration's value. Either use the value immediately before the
  next iteration, or use unique variable names per iteration.

### 3. Tool API Correctness

- `fs-write` does NOT exist. Use `tool("obsidian", action="write", ...)` for
  writing files to obsidian vaults, or `tool("fs-append", ...)` for filesystem
  append.
- `get_json("$var")` returns parsed JSON. `get_text("$var")` returns raw text.
  Know which the tool produces — `fetch-text` returns a Note whose content may
  be JSON with a `"text"` key, or may be plain text.
- `tool("bind", target=resource_id, out="$var")` is needed to create a readable
  binding for resources returned by tools that don't accept an `out` parameter
  directly.

### 4. Naming vs Binding Confusion

- `out="$var"` creates a variable BINDING — it controls how subsequent code
  references the Note. It is NOT the Note's display name.
- `name="foo"` (on tools that support it) sets the Note's human-readable name
  in the infospace. These are different concepts.
- When the goal says "name the Note X", that means `name="X"`, not `out="$X"`.

### 5. Error Handling Across Steps

- Steps continue executing even when a previous step fails.
  `execute_plan_sync` does NOT abort on step failure.
- If step N depends on step N-1 succeeding, step N must check for the
  existence/validity of the data it expects (e.g., check if `$var` has content
  before using it).
- Conditional updates (like appending to a skip-list) must only include items
  that were actually processed successfully, not all items that were attempted.

### 7. Preserving Generality

The plan is instantiated for a specific goal with specific inputs. When
rewriting, distinguish between:

- **Instance parameters** (file paths, note names from the goal text) — keep
  as-is. These match the goal text and are correct for this goal.
- **Strategy logic** (the approach to solving the problem) — keep general.
  Ask: "Would this plan still work if the goal text named a different file?"
- **Fallback strategies** (alternative approaches the planner tried) — don't
  discard these. Consolidate them as fallback branches within a single step
  (try approach A; if that fails, try approach B) rather than removing them.
  The planner tried multiple strategies for a reason — the first one failed.
- **Hardcoded values from a specific run** (literal strings from one document,
  magic numbers like character counts) — replace with computed values or
  parameterize them.
- **Structural exploration** (e.g., parsing YAML frontmatter, checking for
  metadata fields) — preserve as a fast path even if it didn't help on this
  particular input. It may be the right approach for a different input.

Example: if the planner tried (1) frontmatter parsing, (2) extract on full
content, and (3) extract on a preview, don't collapse to just (3). Instead,
write a single step that tries (1) first, falls back to (3) if no frontmatter.

### 6. Structural Checks (Automated)

The `/goal plan review` command runs these automatically:
- Any step that returned `status: failed`
- Notes created with empty content
- Variables bound but never read in a subsequent step
- `say` actions dumping raw content (debug leftovers)

---

## Known Antipatterns

### "Optimistic bookkeeping"
A step updates a tracking file (e.g., `_processed.md`) with ALL attempted items
rather than only successfully processed ones. Failed items are then permanently
skipped on subsequent runs.

**Fix:** Write only successful items to a `$newly_processed` Note in the
processing step; read that in the bookkeeping step.

### "Debug say in production"
`tool("say", value="Processed: " + str(content)[:40])` left in from debugging.
Noisy and unhelpful in replay.

**Fix:** Remove or replace with meaningful status messages.

### "Retry steps masquerading as a pipeline"
The planner generates multiple steps that are really successive retry attempts
at the same subtask. Each step overwrites the same variable and returns. On
replay, ALL steps execute sequentially — the final result is always the last
step's output, wasting time on the earlier retries.

**Fix:** Collapse retry steps into a single step. If the first approach might
fail, handle it within the step (try approach A, fall back to approach B).

### "Hardcoded keyword matching in plans"
When the planner gives up on a semantic approach, it may resort to keyword
matching (e.g., `if "specific phrase" in content`). This violates the project
rule against keyword matching and produces plans that only work for the
specific data seen during the initial run.

**Fix:** Replace with a semantic approach. For classification tasks, use
`extract` on a small context window, then post-process the output in Python
(e.g., truncate to N words).

### "Passing full content to extract when a preview suffices"
The `extract` tool produces unreliable output on large Notes (>10K chars). It
may return repeated phrases, partial content, or ignore length constraints.

**Fix:** Create a `$preview` Note from the first ~500 chars of the content and
pass that to `extract` instead of the full Note.

### "Unquoted paths in exec-script"
Shell commands built with f-strings don't quote paths. Filenames with spaces
(e.g., "Trading Agents.md") cause the shell to split arguments incorrectly.

**Fix:** Always single-quote paths in exec-script targets:
`tool("exec-script", target=f"mv '{source}' '{dest}/'")`.

### "Classification without truncation before use"
The `extract` tool often returns more than N words even when instructed to
limit output. If the raw classification is used directly in a directory name
or filename, it creates absurdly long paths.

**Fix:** Always truncate to the required word count in Python *before* using
the classification for normalization, directory creation, or file operations.

### "Fragile line parsing"
Parsing `fs-list` output by splitting on whitespace or specific delimiters.
The output format may change or contain filenames with spaces.

**Fix:** Use more robust parsing, or prefer `fs-find` which returns structured
results.

---

## Revision History

- 2026-04-07: Initial version. Patterns from goal_11 review (clippings
  processing pipeline). Key findings: inter-step data flow via return values
  fails silently; optimistic bookkeeping marks failed items as processed.
- 2026-04-07: Added patterns from goal_12 review (note classification).
  Key findings: retry steps masquerading as pipeline; hardcoded keyword
  matching when extract fails; passing large Notes to extract produces
  unreliable output.
- 2026-04-07: Added "Preserving Generality" checklist item. Reviewer should
  consolidate fallback strategies as branches within a step, not discard them.
- 2026-04-07: Added patterns from goal_12 multi-file classify+move review.
  New: unquoted paths in exec-script; classification used without truncation
  creating absurd directory names. Also reinforced: fs-find placeholder Notes
  can't be passed to fs-read (use fs-list + path instead).
