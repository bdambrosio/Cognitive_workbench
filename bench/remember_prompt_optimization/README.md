# Remember-subagent prompt optimization

Hand-curated A/B harness for the `chat.remember.remember()` subagent
system prompt. Reuses `bench/memory_recall`'s probes and judge but tests
multiple candidate prompts against identical primer-populated memory
state per probe — slash readout is read-only, so all variants are
apples-to-apples.

Born from analyzing the 2026-05-05 memory_recall baseline on local Gemma
4 31B: slash mean 0.483 vs agent mean 0.931 → **the subagent is the
single largest benchmark gap in the codebase**, and ~60% of slash
failures are JSON-output reliability (model emits empty/malformed
output across all 10 iters), not exploration strategy. v0.1 variants
target prompt brevity + output-discipline-first framing.

## What's here

```
bench/remember_prompt_optimization/
├── README.md          # this file
├── runner.py          # multi-variant primer + slash + agent-ask harness
├── score.py           # runs memory_recall judge per variant + writes compare.md
├── variants/
│   ├── v0_baseline.py # current production prompt (chat.remember._build_system_prompt)
│   ├── v1_*.py        # candidate variants
│   └── ...
└── results/
    └── <stamp>_<scenario>/
        ├── run_index.json
        ├── v0_baseline/
        │   ├── raw.jsonl       # written by runner.py
        │   ├── run_summary.json
        │   ├── scored.jsonl    # written by judge.py
        │   └── summary.md
        ├── v1_minimalist/
        │   ├── raw.jsonl
        │   └── ...
        ├── ...
        ├── compare.md          # written by score.py
        └── compare.json
```

## Variant contract

Each `variants/v*.py` must export:

```python
def build(memory_dir: pathlib.Path) -> str:
    """Return the system prompt the /remember subagent will use."""
```

The harness imports every `v*.py` it finds and runs all of them per
probe — a slash readout is read-only, so the variants don't contaminate
each other within a session.

`v0_baseline.py` re-exports `chat.remember._build_system_prompt` so the
current production prompt is treated as just another variant.

## Running

Two-step: runner produces per-variant `raw.jsonl`; score wraps the
existing memory_recall judge per variant and writes a comparison report.

```bash
# 1. Run primer + multi-variant slash + agent-ask, all subsets.
cd src
python ../bench/remember_prompt_optimization/runner.py \
    --scenario ../scenarios/jill-benchmark-chat.yaml

# 2. Score each variant via memory_recall/judge.py and write compare.md.
python ../bench/remember_prompt_optimization/score.py \
    --run-dir ../bench/remember_prompt_optimization/results/<stamp>_jill-benchmark-chat
```

Subset filter for fast iteration during variant development:

```bash
python ../bench/remember_prompt_optimization/runner.py \
    --scenario ../scenarios/jill-benchmark-chat.yaml \
    --subsets 01-direct-recall 03-negative-fact 09-last-on-topic
```

`CLAUDE_API_KEY` must be set for the score step (Sonnet judge).

## Cost shape

Per full run (14 subsets × ~25-turn primers + 23 probes × N variants
slash + 23 agent asks):

- Primer: ~1 hour wall-clock on local Gemma 4 31B (one-time per run).
- Slash: ~10–30 s per variant per probe × 23 probes × N variants.
  At N=4: ~15–45 min total slash work.
- Agent ask: ~10 s × 23 probes ≈ 4 min.
- Judge: ~2 s × 23 probes × N variants ≈ 1–2 min/variant.

Total at N=4: ~1.5–2 hours per full pass.

## Why this layout (vs reusing existing memory dirs)

A previous design considered replaying variants against the
2026-05-05 memory_recall run's persisted memory dirs — much faster, no
primer re-runs. Rejected because:

- The persisted memory dirs contain the FULL run's state — tells +
  ALL probes' agent-asks (which mutate memory). Replaying probe N's
  slash readout against that dir gives state visibility different
  from the original run, where probe N's slash saw only tells +
  probes 1..N-1.
- Multi-variant testing needs per-probe state matched to the original
  probe ordering. Re-running primer once + lock-step multi-variant
  slash is the cleanest way to guarantee that.

If primer cost ever becomes the bottleneck, a snapshot/restore pattern
(snapshot ChatLoop state after each probe's agent-ask) would let us
re-run individual probes against captured state without redoing
primer work. Not in v0.1.
