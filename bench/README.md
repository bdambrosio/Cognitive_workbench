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

Later stages will add: snapshot/restore between trials, A/B comparison
(planning vs replay against the same goal), batch runners, randomization,
and overnight execution.

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

## Known limitations

- **No state isolation between runs.** The harvester does not snapshot or
  restore the infospace. Re-running the same goal accumulates state.
- **Tool call counts come from log scraping**, not from a structured
  per-call list. Reasonable for stages 1-2; will be replaced when the
  executor exposes a structured per-run action list.
- **Single goal per invocation.** Use a shell loop or wait for stage 4.
- **Planning runs leave `step_results` empty** because per-step capture
  only fires in `execute_plan_sync` (replay path). Planning-mode rows
  will show `step_results_count=0`; use `cached_plan_step_count` and
  `tool_call_count` (or planner_tool_call_count) to characterize
  planning runs.
- **`primary_product` may be empty even on successful runs** if the
  planner persists the deliverable via `name=` instead of binding it to
  `out="$eval_target"`. Stage 2 falls back to grading against
  `last_result` alone in that case, with lower confidence per objective.
