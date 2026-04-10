# bench/ — Cognitive Workbench benchmark harness

Modules for running and measuring goals against a live Cognitive Workbench
launcher. Used to evaluate planner quality, the impact of plan review, and
other cross-validation experiments.

Stage 1 (current): single-goal harvester. Submits one goal, waits for the
run to terminate, computes a row of metrics from the goal record + the
bracketed slice of `executive_node.log`, writes JSONL.

Later stages will add: an Opus quality verdict, snapshot/restore between
trials, A/B comparison (planning vs replay against the same goal), batch
runners, randomization, and overnight execution.

## Prerequisites

- A running launcher with the target character available on Zenoh.
  The harvester uses the same `make_localhost_config()` helper as the CLI,
  so it will find the router on `localhost:7447` automatically.
- The harvester reads `src/logs/executive_node.log` for the bracketed log
  slice. The log is written by the running agent; no additional setup
  needed beyond the launcher being up.
- Run from the repo root (or anywhere — the harvester resolves paths
  relative to its own location).

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

## Exit codes

- `0` — success, row written
- `2` — goal not found (or launcher not running for this character)
- `3` — failed to send the run command via Zenoh
- `4` — completed but `_harvest_error` is set (timeout, missing record)

## Known stage 1 limitations

- **No state isolation between runs.** The harvester does not snapshot or
  restore the infospace. Re-running the same goal accumulates state.
- **Tool call counts come from log scraping**, not from a structured
  per-call list. Reasonable for stage 1; will be replaced when the executor
  exposes a structured per-run action list.
- **No quality verdict beyond what the system computes.** `quality_status`
  comes from the agent (planner verification or vision_eval); the harness
  does not run an independent Opus verdict yet.
- **Single goal per invocation.** Use a shell loop or wait for stage 4.
- **Planning runs leave `step_results` empty** because per-step capture
  only fires in `execute_plan_sync` (replay path). Planning-mode rows
  will show `step_results_count=0`; use `cached_plan_step_count` and
  `tool_call_count` to characterize planning runs.
