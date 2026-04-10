# Plan Review Prompt

This file guides the review of a Cognitive Workbench goal plan. It is used by
a reviewer (human or LLM) to critique and improve cached plan actions before
replay. Update this file after each review cycle with new patterns discovered.

## How to Use

1. Run `/goal plan review <goal_id>` in Jill's CLI to generate a review bundle.
2. Open the review bundle (`goal_review_<goal_id>.md`) alongside this prompt.
3. Check the **Available Tools** section in the review bundle to verify that
   every tool call in the plan actually exists. This is the authoritative list
   of primitives and world/external tools for the current configuration.
4. If a trace exists (`goal_trace_<goal_id>.txt`), use it to understand planner
   reasoning — grep for specific tool calls or variable names rather than
   reading the whole trace.
5. Review the plan in `goal_plan_<goal_id>.py` (run `/goal plan edit <goal_id>`
   if the file doesn't exist).
6. Apply the checks below. Fix issues directly in the plan file.
7. Run `/goal plan commit <goal_id>` to load the plan, set replay mode, and inject learnings.
8. Add any new failure patterns you discover to this file.

---

## Review Checklist

### 0. Triage: Fix or Rewrite?

Before reviewing individual steps, assess whether the plan is worth fixing:

- Check **last_result** and **status** in the review bundle header. If the
  result says "step limit reached", "loop guard", or the status is
  "completed" with no primary product, the plan likely failed entirely.
- Scan the plan steps: if 3+ steps are near-identical (retry spiral), or
  the plan never progressed past discovery/verification, **rewrite from
  the goal text** rather than patching the cached steps.
- The goal text is the specification. The cached plan is one failed
  implementation attempt — don't be anchored by its approach. A different
  strategy may be simpler and more robust.
- If the plan succeeded and produced correct output, minor issues can be
  fixed in place. Only rewrite when the approach is fundamentally broken.

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

- `fs-write` and `fs-append` do NOT exist. Use `tool("obsidian", action="write", ...)`
  for writing files. To append, read with `fs-read`, concatenate in Python, then
  write back with `obsidian write`.
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

**Critical:** Split on BOTH spaces AND underscores when counting words:
`re.split(r'[\s_]+', text)[:3]`. If you only split on spaces,
`"financial_kline_model"` counts as 1 word and truncation has no effect —
the planner joins multiple such "words" into an absurdly long underscore
chain. Also sanitize with `re.sub(r'[^a-z0-9_]', '', text)` to prevent
shell metacharacters in directory names.

### "Fragile line parsing"
Parsing `fs-list` output by splitting on whitespace or specific delimiters.
The output format may change or contain filenames with spaces.

**Fix:** Use more robust parsing, or prefer `fs-find` which returns structured
results.

### "Missing reason on failed return"
`_create_uniform_return("failed", value=..., extra=...)` without an explicit
`reason=` kwarg. The executor fills in `'Unknown error'`, which masks the real
cause in execution logs and review bundles.

**Fix:** Always pass `reason=` when returning `"failed"`. Split the return into
explicit success/failure branches rather than using a ternary for the status.

### "Unguarded cross-step variable read"
Step N calls `get_text("$var")` for a variable that step N-1 was supposed to
create. If step N-1 failed or crashed, the variable doesn't exist and step N
throws an unhandled exception (or reads stale data from a prior run).

**Fix:** Wrap `get_text("$var")` in a try/except for variables created by a
prior step. Check for empty content before proceeding. This is especially
important for the bookkeeping step that updates tracking files.

### "Verification spiral"
The planner generates a step that verifies the previous step's output, then
generates the same verification step again and again because the eval target
doesn't reflect the change. Symptoms: 3+ consecutive steps with identical code
or near-identical NEXT_TASK text in the trace.

**Fix:** Collapse all verification/retry steps into a single step. If the plan
needs to confirm success, do it at the end of the processing step itself (check
the Note content inline) rather than as a separate step. Steps 3+ that are
copies of step 3 should be deleted entirely.

### "Literal variable name passed as tool argument"
`tool("obsidian", action="write", content="$new_paper")` passes the literal
string `"$new_paper"` — NOT the Note's content. Variable bindings (`$var`) are
only resolved by `get_text("$var")` or `get_json("$var")`, not by string
interpolation in tool kwargs.

**Fix:** Always dereference: `content=get_text("$new_paper")` or assign to a
local first: `paper_text = get_text("$new_paper")` then pass `content=paper_text`.

### "Hardcoded single-file processing instead of loop"
The planner processes one specific file by name in a step that should iterate
over a dynamic list. This produces a plan that only works for the exact files
present during the planning run.

**Fix:** Read the unprocessed list from the Note created in the discovery step
and loop over it. The loop body should handle each file identically.

### "Paywalled/403 URLs treated as permanent failures"
`fetch-text` returns HTTP 403 for paywalled or bot-blocked sites. If the plan
marks these files as processed (via optimistic bookkeeping), they are
permanently skipped and never retried.

**Fix:** Only append successfully fetched files to the processed list. Log the
domain and HTTP status for world-model learning. Consider adding domain-level
skip lists if the same domain consistently 403s.

### "fs-find IDs passed to fs-read"
`fs-find` returns Collection Notes containing placeholder Note IDs. These IDs
cannot be passed to `fs-read` as paths — `fs-read` expects filesystem-relative
paths like `KnowledgeBases/Untitled.md`. The planner often tries this, gets a
"path not found" error, then burns steps discovering the issue.

**Fix:** Use `fs-list` for discovery (returns human-readable filenames), parse
the output to extract filenames, then construct explicit paths for `fs-head` or
`fs-read`.

### "Verification looks for wrong filename after title extraction"
The planner writes a note using an extracted title (e.g., `KnowledgeBases/Paper_Title.md`),
then verifies by reading the original name (e.g., `KnowledgeBases/Untitled.md`). The read
fails, and the planner concludes the write failed — triggering a reprocess spiral.

**Fix:** If the step writes a file under a dynamically computed name, verify by listing
the parent directory (not by reading a hardcoded path). Or skip verification entirely —
if `obsidian write` returned success, trust it.

### "Bookkeeping file format destroyed by verbose logging"
The planner overwrites a simple bookkeeping file (e.g., `_processed.md` containing one
filename per line) with a verbose "processing log" to pass the `output_truncated` quality
gate. Future runs can't parse the file, permanently breaking the pipeline.

**Fix:** Never overwrite bookkeeping files with summaries. To append filenames,
read with `fs-read`, concatenate in Python, write back with `obsidian write`.
(`fs-append` does not exist.) Set `$eval_target` to the real deliverable, not
the bookkeeping artifact.

### "Regex \\S+ breaks on filenames with spaces"
`fs-list` output lines look like `Untitled 1.md  (219B)`. A regex like
`r'^(\S+\.md)\s+\('` stops at the first space, capturing `1.md` instead of
`Untitled 1.md`. The planner then silently drops filenames with spaces from
the list, producing an empty unprocessed set and spiraling.

**Fix:** Parse `fs-list` output by finding the size suffix with
`line.rfind('  (')` and taking everything before it as the filename. Do not
use `\S+` for filenames — Obsidian and many filesystems allow spaces.

### "Spiral guard triggered across steps"
The spiral guard counts consecutive failures per tool globally, not per step.
If step 1 fails `fs-read` twice, step 2's first `fs-read` call is immediately
blocked — even though step 2 might use a correct path. This causes cascading
failures across retry steps.

**Fix:** Collapse retry steps into one step with fallback branches (try
`fs-read`, catch failure, fall back to `fs-head`). This avoids accumulating
cross-step failure counts. Also, if a step uses `fs-list` for discovery, skip
`fs-read` entirely and go straight to `fs-head` for previews.

### "Binding preview mistaken for truncated content"
The RESULT text shown to the planner after each code block includes a
200-char preview of each binding's content. Smaller models interpret this
preview as the *actual* Note content, conclude it's "truncated", and
spiral trying to fix a non-existent problem. Symptoms: repeated steps
trying `create-note`, `obsidian write`, chunking, and `synthesize` to
"fix truncation", all producing the same 200-char preview.

**Fix:** For plans that create long-form content, build the text in Python
(f-strings, list join) and pass it to `create-note` directly. Do NOT
verify the Note content by reading the RESULT preview — trust that
`create-note` stores the full value. If verification is needed, use
`get_text("$var")` inside the code block before returning, not as a
separate step that reads the binding preview.

### "Infinite comment loop on tool gap"
When the LLM can't find a tool to accomplish a subtask (e.g., no `fs-write`),
it sometimes enters an infinite loop of comments reasoning about the problem
instead of generating executable code. The code block fills with dozens of
lines like `# Actually, fs-read is read-only. To write, need to use fs-read?
No.` and never returns.

**Fix:** The reviewer must verify every tool call in the plan against the
Available Tools section of the review bundle. If the plan needs to write a
file and no `fs-write` exists, use `obsidian write` (for vault files) or
`exec-script` with the correct relative path.

### "fs-find Note IDs vs filenames"
`fs-find` returns a Collection of Note IDs (e.g., `Note_5052`). To get the
actual filename, you must call `get-metadata` on each Note and extract the
`path` field. This is expensive (one tool call per file) and fragile. The
planner often hardcodes specific Note IDs or fails to extract metadata
correctly.

**Fix:** Use `fs-list` instead of `fs-find` for discovery. `fs-list` returns
a text listing with filenames visible, parseable in Python without extra tool
calls.

### "Missing $eval_target binding"
The plan returns `extra={"eval_target": output}` in the return dict, but the
system requires an explicit `out="$eval_target"` binding on a tool call (e.g.,
`create-note`). Without the binding, no primary product is identified, the
quality gate doesn't run, and post-completion cleanup may delete all outputs.

**Fix:** Create the deliverable as a Note bound to `$eval_target`:
`tool("create-note", content=report_text, out="$eval_target")`. The return
dict's `extra` field is for metadata, not for declaring the primary product.

### "Re-fetching content that's already loaded"
`semantic-scholar` returns a Collection with full paper text (via GROBID). The
planner then tries to extract `pdf_url` from metadata and pass it to
`extract-references` or `fetch-text` to re-fetch the same content. This wastes
steps, introduces failure points (URL extraction, HTTP fetches), and often
spirals when the URL plumbing doesn't work.

**Fix:** Use `map(extract)` or `synthesize` directly on the Collection that
already contains the text. Check each tool's Skill.md for what its output
already includes before adding fetch steps.

### "pluck on tool_metadata fields"
`pluck(target="$paper", field="metadata.pdf_url")` always fails because `pluck`
operates on Note **content**, not on the separate `tool_metadata` property.
semantic-scholar stores metadata (title, authors, pdf_url, etc.) in a relation
Note, not in the content field. The planner tries `pluck` on various
`metadata.*` field paths, gets empty results, and spirals.

**Fix:** To access tool_metadata fields: `get-metadata` on the Note →
`get_text("$metadata")` → `json.loads()` in Python. But often the metadata
isn't needed at all — if the goal is content extraction, the Note body already
has the full text (when GROBID is available).

### "Split exec-script calls lose computed values on partial failure"
A step computes a directory name in Python, then calls `exec-script` twice
(mkdir, then mv). If `mkdir` succeeds but `mv` is denied by the user, the step
returns failure and the computed directory name is lost. The retry step has to
guess the name — and typically guesses wrong (uses a hardcoded fallback or
reads the source code literal instead of the runtime value).

**Fix:** Combine mkdir + mv in a single exec-script call:
`tool("exec-script", target=f"mkdir -p '{dir}' && mv '{src}' '{dir}/'")`.
This ensures both succeed or both fail atomically. If the user denies once,
the retry has the same combined command.

### "Synthesize for structured extraction"
The planner uses `synthesize` expecting a structured list (e.g., citation list,
file inventory) but gets a narrative overview ("the literature reveals a clear
trajectory..."). Symptoms: repeated synthesize calls with increasingly specific
`focus` instructions, all returning prose instead of lists.

**Fix:** `synthesize` produces narratives by design. For structured extraction,
use `map(extract)` on the Collection to get per-item structured output, then
`flatten` into a single Note. Report the flattened text directly — do not pipe
it through `synthesize` again.

### "Repeated extract-on-Collection"
The planner calls `extract(target="$collection_var")`, gets an error that
extract requires a Note not a Collection, then retries the same call 2-3 steps
later. The type error is deterministic — retrying can't help.

**Fix:** When the target is a Collection, use `map(extract)` (processes each
item) or index into it with `get_items("$var")[0]` to get a single Note.
If the reviewer sees `extract` called on a known Collection binding, fix it
on first occurrence — don't leave it for the spiral guard to catch.

### "Missing named Note triggers filesystem hunt"
A goal references a sensor-written named Note (e.g. `_rss_pending_titles`,
written by `rss-watcher`). `tool("load", target="_rss_pending_titles", ...)`
returns `failed` because the sensor has not run yet. Instead of treating the
empty input as the empty case, the planner concludes the resource must live
on disk and burns 4-6 steps calling `fs-find` and `fs-list` against
`Clippings/`, `daily_posts/`, `Analysis/`, `src/`, etc. — all places where
infospace Notes never live. The world model usually already records the
producer-consumer fact, but the planner ignores it.

**Fix:** Names starting with `_` (and most names referenced in goal text
that don't look like file paths) are infospace Notes, not files. If `load`
fails on such a name AND `load` succeeds on a sibling note in the same goal
(e.g. `rss-interests` works but `_rss_pending_titles` doesn't), the missing
target is another named Note, not a file. Do not pivot to the filesystem.
Treat the missing Note per the next antipattern.

### "Missing sensor-written Note treated as failure"
Sensor-written Notes follow a producer/consumer pattern: the producer only
writes when there is fresh data to hand off. A missing Note means "no fresh
data", which for a filter-and-report goal is the empty case ("say nothing"),
not an error. Returning `failed` here causes the goal to retry forever and
masks the actual semantics from the user.

**Fix:** When `load` fails on a name that follows the `_*` sensor-output
convention, return `success` with `value="No pending ... to filter."` and
exit the step. Reserve `failed` for *expected* inputs that the user is
supposed to maintain (e.g. an interests note the user curated).

### "Hardcoded predicate from one observed run"
The planner reads an interests/criteria Note in the planning run, sees its
content (e.g. "AI, Agents, Markets"), and bakes that string literal into
the plan as `predicate = "mentions AI, Agents, or Markets"`. On replay, if
the user edits the criteria Note (which is the entire reason it lives in a
Note rather than the plan), the predicate is stale. This also drifts toward
keyword matching: a literal string list with "or" between terms is the
keyword-matching shape, even though `filter-semantic` could do better with
the raw text.

**Fix:** Build the predicate from `get_text("$interests_var")` every run.
A natural-language wrapper like
`f"matches one or more of these user interests: {interests_text}"` keeps
the plan general and lets `filter-semantic`'s LLM evaluate semantically
against whatever the user wrote.

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
- 2026-04-08: Second review of goal_11 after replay failure. New patterns:
  missing reason= on failed return causes "Unknown error"; unguarded cross-step
  variable reads crash when prior step fails; paywalled URLs (HTTP 403) need
  domain-level tracking, not permanent skip.
- 2026-04-08: Third review of goal_11 after replan run. Planner spiraled badly
  (12 steps, 10 identical verification retries). New patterns: verification
  spiral; literal $var passed as tool content; hardcoded single-file processing
  instead of loop. Also: bare say() crash, planner reasoning leaked into ask.
- 2026-04-08: Review of goal_12 (KnowledgeBases classify+move). 6 steps
  collapsed to 2. New patterns: fs-find IDs can't be passed to fs-read (use
  fs-list instead); spiral guard triggers across steps causing cascading blocks.
  Reinforced: extract truncation, quoted exec-script paths, preview over full
  content for classification.
- 2026-04-08: Fifth review of goal_11 (gemma-4-31B replan). Spiraled 8 steps
  on discovery — regex \S+ couldn't match filenames with spaces ("Untitled 1.md").
  Planner never progressed past step 1. New pattern: regex filename parsing.
- 2026-04-08: Sixth review of goal_11 (gemma-4-31B replan). 11 steps, mostly
  spiraling. Step 4 had infinite comment loop (~50 lines "fs-read is read-only")
  when LLM couldn't find fs-write. Used fs-find+get-metadata instead of fs-list.
  Hardcoded Note IDs. Passed "$var" literal to obsidian content. Duplicate entries
  in _processed.md. New patterns: infinite comment loop on tool gap; fs-find Note
  IDs vs filenames.
- 2026-04-09: Review of goal_15 (paper citation fetch). 13 steps, massive retry
  spiral. Planner forgot extract can't operate on Collections 3 times; used
  synthesize expecting structured lists but got narratives; hit spiral guard.
  Collapsed to 2 steps: semantic-scholar → map(extract) + flatten + say.
  New patterns: synthesize for structured extraction; repeated extract-on-Collection.
- 2026-04-09: Second review of goal_15 (replan run after file edit). 17 steps,
  loop guard triggered. Planner used extract-references on Collection (needs PDF
  URL), then spiraled reading JSON Notes with pluck(field="text"). No $eval_target
  binding so cleanup deleted all output. Collapsed to 2 steps: semantic-scholar →
  map(extract) + flatten + create-note($eval_target) + say. New patterns: missing
  $eval_target binding; re-fetching content already loaded by semantic-scholar;
  pluck on tool_metadata fields (must use get-metadata + json.loads instead).
- 2026-04-09: Review of goal_12 (classify+move .md files). 19 steps, massive
  spiral. extract returned multi-phrase output, .replace(" ", "_") joined it
  all into one huge directory name. Planner spent 16 steps trying to fix it.
  Collapsed to 2 steps: fs-list → loop(fs-head + extract + truncate + mv).
  Refined antipattern: split on spaces AND underscores when counting words.
- 2026-04-08: Review of goal_14 (3-hop citation chain). 20 steps, hit step
  limit. Planner spiraled 13 steps on "truncation" that was actually the
  200-char binding preview. Built chain via synthesize (LLM-generated, lossy)
  instead of Python f-strings (deterministic). New pattern: binding preview
  mistaken for truncated content.
- 2026-04-08: Fourth review of goal_11 after replan with new goal text. 8 steps
  collapsed to 3. New patterns: verification reads wrong filename after title
  extraction (triggers reprocess spiral); bookkeeping file format destroyed by
  verbose logging to pass quality gate. Title had colons — need filename
  sanitization beyond just spaces.
- 2026-04-10: Review of goal_15 (filter RSS titles by user interests). 6 steps,
  spiraled on missing `_rss_pending_titles`. Planner ignored the world-model
  fact that this is a sensor-written named Note and spent steps 2-6 hunting
  through `Clippings/`, `daily_posts/`, `Analysis/`, `src/` with `fs-find`.
  Also hardcoded `predicate = "mentions AI, Agents, or Markets"` from one
  observed read of `rss-interests`, defeating replay generality. Collapsed
  to 1 step that treats missing pending-titles as the empty case (return
  success silently per "if no matches, say nothing"), and builds the
  predicate dynamically from `get_text("$interests")`. New patterns: missing
  named Note triggers filesystem hunt; missing sensor-written Note treated
  as failure; hardcoded predicate from one observed run.
