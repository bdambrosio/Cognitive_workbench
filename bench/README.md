# bench/ — Cognitive Workbench benchmark harness

Modules for running and measuring goals against a live Cognitive Workbench
launcher. Used to evaluate planner quality, the impact of plan review, and
other cross-validation experiments.

Stages currently implemented:

- **Stage 1** — `bench/harvester.py`: single-goal harvester. Submits one
  goal, waits for the run to terminate, computes a row of metrics from
  the goal record + the bracketed slice of `executive_node.log`, writes
  JSONL.

- **Stage 2** — `bench/verdict.py`: independent quality verdict. Takes a
  stage 1 row, decomposes the goal into atomic objectives, asks an
  Anthropic model (default `claude-sonnet-4-6`) to judge each objective
  against the run's actual output, and decorates the row with a
  structured `opus_verdict` sub-object. Can be invoked standalone on a
  saved row, or inline via `harvester.py --opus`.

- **Stage 3** — `bench/snapshot.py`, `bench/launcher.py`,
  `bench/reviewer.py`, `bench/trial.py`: snapshot/restore primitive,
  launcher lifecycle helper, automated plan reviewer with dry-run mode,
  and the per-goal trial driver that composes `run` and `review` steps
  into one experimental record against a single goal.

- **Stage 4** — `bench/baseline.py`, `bench/experiment.py`: batch
  experiment runner with overlay-based state management.
  `baseline.py` builds a truly-empty baseline snapshot by wiping the
  character resource dir, booting Jill once to write the framework
  scaffold, seeding goals from a YAML file via the new create-only
  `/goal add` semantic, and snapshotting. `experiment.py` accepts an
  explicit goal list and trial type (A/B/C), produces a plan, and
  executes it with per-goal reset = baseline restore + orthogonal
  overlays (world_model.json, cached_plan_actions). Captures
  accumulating state between goals as sidecar files. Writes per-goal
  records to a JSONL file as they complete.

## Prerequisites

- A running launcher with the target character available on Zenoh.
  The harvester uses the same `make_localhost_config()` helper as the CLI,
  so it will find the router on `localhost:7447` automatically.
- The harvester reads `src/logs/executive_node.log` for the bracketed log
  slice. The log is written by the running agent; no additional setup
  needed beyond the launcher being up.
- Run from the repo root (or anywhere — the harvester resolves paths
  relative to its own location).
- For **stage 2 (Opus verdict)** only: `CLAUDE_API_KEY` (or
  `ANTHROPIC_API_KEY`) in the environment, plus the `anthropic` Python
  SDK in the venv (`pip install anthropic`).

## Stage 1 usage

```bash
# Run a goal in its current execution_mode and write the row to stdout
python bench/harvester.py --goal goal_15

# Force a specific mode for this run (overrides the stored execution_mode)
python bench/harvester.py --goal goal_15 --mode replay

# Append the row to a JSONL file instead of stdout
python bench/harvester.py --goal goal_15 --out runs/2026-04-10.jsonl

# Harvest the current goal record without submitting a new run
# (useful for collecting a row from a goal that already completed)
python bench/harvester.py --goal goal_15 --no-submit
```

## Stage 2 usage

Three ways to invoke the verdict tool:

```bash
# Inline: harvest + grade in one call (uses harvester's Zenoh session)
python bench/harvester.py --goal goal_18 --opus --out runs.jsonl

# Inline with model override
python bench/harvester.py --goal goal_18 --opus --opus-model claude-opus-4-6

# Standalone: grade an existing row from stdin
echo '{"goal_id":"goal_18","goal_text":"...",...}' | python bench/verdict.py --stdin

# Standalone: grade all rows in a JSONL file
python bench/verdict.py --jsonl-file runs.jsonl --out runs_graded.jsonl

# Standalone with explicit model
python bench/verdict.py --jsonl-file runs.jsonl --model claude-opus-4-6 --out graded.jsonl
```

The decorated row gains an `opus_verdict` sub-object:

```json
{
  "model": "claude-sonnet-4-6",
  "verdict_ts": "2026-04-10T22:00:00+00:00",
  "elapsed_ms": 4521,
  "input_tokens": 1832,
  "output_tokens": 612,
  "stop_reason": "end_turn",
  "primary_product_loaded": true,
  "objectives": [
    {"id": "obj1", "text": "Search for parameter count",
     "achieved": true, "evidence": "search-web returned a Note...",
     "confidence": 0.95}
  ],
  "achieved_count": 3,
  "total_count": 3,
  "score_percent": 100,
  "verdict": "satisfied",
  "notable_issues": [],
  "criteria_alignment": {
    "non_empty": "PASS",
    "on_topic": "PASS",
    "includes_source": "PASS"
  }
}
```

If the API key is missing, the SDK isn't installed, the API call fails,
or the model returns malformed JSON, the verdict subdict carries an
`error` field instead of the verdict fields. The row is still written
with whatever stage 1 data was captured.

### Re-grading saved rows

You can re-grade a saved JSONL file with a different model or after a
prompt change without re-running the goals:

```bash
# Original capture (sonnet)
python bench/harvester.py --goal goal_2 --opus --out runs/today.jsonl

# Later, regrade with opus
python bench/verdict.py --jsonl-file runs/today.jsonl --model claude-opus-4-6 \
                       --out runs/today.opus.jsonl
```

### Cost guidance

Approximate cost per verdict at current pricing (Sonnet 4.6):

- Goal text + criteria + plan + last result + primary product ≈ 2-4K input tokens
- Verdict output ≈ 0.5-1.5K output tokens
- Sonnet: ~$0.01-0.04 per verdict
- Opus: ~$0.05-0.20 per verdict

For an overnight A/B benchmark of 17 goals × 3 trials × 2 conditions:
- 102 verdicts × Sonnet ≈ $1-4 total
- 102 verdicts × Opus ≈ $5-20 total

Use Sonnet during development, Opus for the final cross-validation pass.

## Exit codes

- `0` — success, row written
- `2` — goal not found (or launcher not running for this character)
- `3` — failed to send the run command via Zenoh
- `4` — completed but `_harvest_error` is set (timeout, missing record)
- `5` — verdict tool: no API key in environment
- `6` — verdict tool: invalid input row JSON
- `7` — verdict tool: at least one row had a grading error

## What ends up in the row

One JSON object per row. Stable keys (always present):

- `harvest_ts`, `harvester_version`
- Goal identity: `goal_id`, `goal_name`, `goal_text`
- Run identity: `submitted_mode`, `last_run_mode`, `execution_mode_setting`
- Outcome: `status`, `quality_status`, `last_result`, `primary_product`
- Plan structure: `cached_plan_step_count`
- Per-step instrumentation (replay only — planning leaves these at 0):
  `step_results_count`, `step_failures`, `step_exceptions`,
  `step_durations_ms_total`
- Tool calls (from log slice): `tool_call_count`, `tool_call_types` (dict),
  `code_block_failures`
- Vision criteria provenance: `vision_criteria`, `vision_criteria_source`,
  `vision_criteria_version`, `vision_criteria_model`,
  `vision_criteria_generated_at`, `vision_criteria_text_hash`,
  `last_quality_eval`
- Bracket-derived (from log END line): `bracket_mode`, `bracket_status`,
  `bracket_steps`, `bracket_duration_ms`, `bracket_quality`
- Timestamps: `created`, `updated`
- Harvest meta: `log_slice_lines`, `_harvest_error` (null on success)
- (When `--opus` flag set) `opus_verdict` sub-object with the model's
  per-objective grading; see Stage 2 usage above for the schema.

## Stage 4 usage

The experiment runner expects a `baseline-clean` snapshot to already
exist. See **Baseline setup** below for the one-time manual procedure.

```bash
# Smoke: one goal, replan only
python bench/experiment.py trial-A --goals G01 --seed 42 --out runs/smoke.jsonl

# Trial A: baseline measurement, 10 goals
python bench/experiment.py trial-A \
    --goals G01 G02 G03 G04 G05 G06 G07 G08 G09 G10 \
    --seed 42 --out runs/a.jsonl

# Trial B: accumulating review, same 10 goals
python bench/experiment.py trial-B \
    --goals G01 G02 G03 G04 G05 G06 G07 G08 G09 G10 \
    --seed 42 --out runs/b.jsonl

# Trial C: replay where a Trial B first-half plan exists, replan otherwise.
# --trial-b-dir is the experiment dir produced by the Trial B run above.
python bench/experiment.py trial-C \
    --goals G01 G02 G03 G04 G05 G06 G07 G08 G09 G10 \
    --seed 42 \
    --trial-b-dir exp/trial-B-seed-42 \
    --reviewed-in-b-from runs/b.jsonl \
    --out runs/c.jsonl
```

`--goals` accepts either display names (`G01`) or internal goal_ids
(`goal_17`) or a mix. The same invocation works for 1 goal or 20.

Trial C's `--reviewed-in-b-from` reads the first-half goal refs out of
the Trial B JSONL file and derives the `cached_plan_<goal>.json` paths
from `--trial-b-dir`, so you don't hand-maintain either list.

### Overlay-based state management (the reset model)

Every per-goal reset in every trial is a **full restore of the empty
baseline** followed by an **explicit list of overlays** to carry
forward specific slices of accumulated state. The overlay list is
either empty (Trial A), or one or two items (Trial B phase 2, Trial
C), or grows per-goal (Trial B phase 1 as world_model and cached_plans
accumulate).

Overlay operations — applied after `snapshot.restore()`, before
`launcher.start()`:

- **world_model**: copies a saved `world_model.json` over the
  baseline's empty one. Simple file copy. Used to carry forward
  world-model learnings from prior goals in the same trial.
- **cached_plan**: offline JSON surgery on `resources.json`. Locates
  the scheduled-goal Note by `note_name`, parses its `content` field,
  sets `cached_plan_actions`, writes back. Used to inject a reviewed
  plan from Trial B phase 1 into a Trial C replay.

Capture operations — applied after a goal run completes, saving
named sidecar files for subsequent overlays:

- **world_model** → copy current `world_model.json` to
  `exp/<trial>/world_model_after_<i>.json`
- **cached_plan** → read `cached_plan_actions` from the scheduled-goal
  Note, write as JSON list to `exp/<trial>/cached_plan_<goal>.json`

### Trial semantics (summary)

| Trial | Per-goal protocol | Reset | Overlays | Captures |
|---|---|---|---|---|
| A | `[replan]` | `baseline-empty` (full) | none | none |
| B phase 1 goal 0 | `[replan, review commit, replay]` | `baseline-empty` | none | world_model, cached_plan for this goal |
| B phase 1 goal N>0 | `[replan, review commit, replay]` | `baseline-empty` | world_model from goal N-1 + cached_plans for goals 0..N-1 | world_model, cached_plan for this goal |
| B phase 2 | `[replan]` | `baseline-empty` | world_model from end of phase 1 | none |
| C (reviewed in B) | `[replay]` | `baseline-empty` | world_model + cached_plan for this goal | none |
| C (unreviewed) | `[replan]` | `baseline-empty` | world_model only | none |

Every goal starts from **identical** framework scaffold (empty
`_scheduled_goals` collection + seeded `_derived_concerns`) plus
whatever overlays apply. There is no cross-goal contamination of
persistent Notes, `_user_concerns`, `_situation`, `_ooda_state`, or
`conversation_history` — those are lazily created and wiped on every
reset.

## Baseline setup (one-time, scripted)

The experiment runner assumes a snapshot named `baseline-empty` already
exists. `bench/baseline.py create` builds it end-to-end: wipes the
character resource dir, boots Jill once so the framework scaffold gets
written (two persistent resources: the seeded `_derived_concerns` Note
from the scenario YAML, and an empty `_scheduled_goals` Collection),
sends `/goal add` + `/goal rename` for each entry in a goals YAML file
to seed G01..Gnn without running them, stops Jill, and takes the
snapshot.

```bash
python bench/baseline.py create \
    --label baseline-empty \
    --goals-file bench/goals-infolab-bench.yaml
```

Prerequisites:

- Jill must be stopped before invoking this.
- The agent-side `/goal add` command must support create-only
  semantics (default: `run=False`). The `_cmd_goal_add` handler in
  `executive_node.py` returns `"Goal <goal_id> created"` without
  starting the goal on the worker thread.
- The goals YAML file (`bench/goals-infolab-bench.yaml`) is the
  canonical taskset definition. Edit it to change goals; re-run
  `baseline.py create` afterward to rebuild.

The baseline can be inspected after creation:

```bash
python bench/snapshot.py describe baseline-empty
```

A `_baseline_seed_manifest.json` sidecar is written inside the
snapshot directory, listing the goals seeded and their assigned
goal_ids.

Rebuild any time you change the goal set or want a truly clean start:

```bash
python bench/snapshot.py delete baseline-empty
python bench/baseline.py create --label baseline-empty \
    --goals-file bench/goals-infolab-bench.yaml
```

### Why not a manual baseline procedure?

The older `baseline-clean` approach (delete `world_model.json`, keep
`resources.json`) preserved the framework scaffold but also preserved
accumulated session history: prior `_user_concerns`, `_situation`,
`_ooda_state`, `conversation_history` items, prior FAISS-indexed
persistent notes, cross-session `theory_of_mind` paragraphs, and
closed-concerns lists that the planner reads on startup. Direct
inspection of a post-run trace showed all of those leaking into planner
behavior in ways that biased per-goal outcomes — in one case, a closed
concern labeled "PDF fetch for arXiv paper 2604.08455 incomplete due to
missing Note persistence" from a prior run was priming the current
planner into a truncation spiral on the same goal.

The overlay model eliminates this by design: every goal starts from the
empty-scaffold baseline, with exactly the state the experiment says
should carry forward and nothing else.

## Known limitations

- **Tool call counts come from log scraping**, not from a structured
  per-call list. Reasonable for stages 1-2; will be replaced when the
  executor exposes a structured per-run action list.
- **Planning runs leave `step_results` empty** because per-step capture
  only fires in `execute_plan_sync` (replay path). Planning-mode rows
  will show `step_results_count=0`; use `cached_plan_step_count` and
  `tool_call_count` (or planner_tool_call_count) to characterize
  planning runs.
- **`primary_product` may be empty even on successful runs** if the
  planner persists the deliverable via `name=` instead of binding it to
  `out="$eval_target"`. Stage 2 falls back to grading against
  `last_result` alone in that case, with lower confidence per objective.
- **No automatic experiment resume.** If `run_experiment` crashes
  mid-trial, partial results are on disk (JSONL appends per-goal), but
  restarting manually with an updated goal list is the user's job.
- **Single seed per invocation.** Running multiple seeds for statistical
  strength is a shell loop; no built-in multi-seed orchestration.
