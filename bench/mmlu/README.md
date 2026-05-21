# MMLU benchmark

Drives Jill's chat ReAct loop through MMLU multiple-choice questions in
closed-book mode. One question per ChatLoop instance — questions don't
share state.

## Run

From `src/` (so `launcher` imports resolve):

```bash
cd src

# Quick smoke (default 6-subject subset, 50 questions each, k=5):
python ../bench/mmlu/runner.py --scenario ../scenarios/jill-benchmark-mmlu.yaml

# Specific subjects:
python ../bench/mmlu/runner.py \
    --scenario ../scenarios/jill-benchmark-mmlu.yaml \
    --subjects high_school_physics formal_logic

# Full MMLU (all 57 subjects):
python ../bench/mmlu/runner.py \
    --scenario ../scenarios/jill-benchmark-mmlu.yaml --subjects all

# Zero-shot:
python ../bench/mmlu/runner.py \
    --scenario ../scenarios/jill-benchmark-mmlu.yaml --k-shot 0
```

The local chat backend (default `http://127.0.0.1:5000`) must be up — the
scenario points at it via `llm_config.vllm_url`. Swap in an alternate
scenario YAML to test other backends.

## Output

`bench/mmlu/results/<timestamp>_<scenario_stem>/`:

- `raw.jsonl` — one row per question with `subject`, `question_index`,
  `question`, `choices`, `gold`, `prediction`, `correct`, `reply`,
  `error`, `duration_s`, `world_name`.
- `summary.md` — overall accuracy + per-subject table.
- `summary.json` — same numbers, machine-readable.

## Design notes

**Closed-book.** The scenario sets `chat.omitted_tools` to drop every
external-info tool (`search`, `fetch_text`, `inspect`,
`inspect_external`, `recall`, `display`, `security`). Only
`process_text` and `respond` remain. This is a test of model knowledge
through Jill's ReAct loop, not retrieval.

**Per-question isolation.** Each question runs in a fresh `ChatLoop`
with a unique `world_name` (`bench-mmlu-<stamp>-<subject>-<idx>`), so
memory dirs and resource state don't carry over. Slower than reusing a
loop, but it's the same isolation pattern the other in-process benches
use and it removes any chance of cross-question contamination.

**No LLM judge.** Gold is one of A/B/C/D; the runner extracts the
letter from `ANSWER: X` (with fallbacks for "choice B" / "option B" /
trailing letter). Scoring is exact-letter match.

**Few-shot examples** come from the dev split per subject; they live
in the user-turn prompt, not in Jill's conversation memory.

**Scenario flags.** `chat.benchmark_mode: true` makes post-turn
reflection inline so persistence completes before teardown.
`discourse` and `orientation` are off — they add prompt overhead and
latency with no value on independent one-shot questions.
