# HLE (Humanity's Last Exam) benchmark

Drives Jill's chat ReAct loop through HLE research questions. One
question per ChatLoop instance — questions don't share state. Open-book:
search, fetch, and process_text are enabled.

Two phases. The runner produces `raw.jsonl`; the judge scores it.

## Run

From `src/` (so `launcher` imports resolve):

```bash
cd src

# Smoke run — 10 questions:
python ../bench/hle/runner.py --scenario ../scenarios/jill-benchmark-hle.yaml

# More questions, or specific slice:
python ../bench/hle/runner.py \
    --scenario ../scenarios/jill-benchmark-hle.yaml \
    --max-questions 50

# Resume from an offset:
python ../bench/hle/runner.py \
    --scenario ../scenarios/jill-benchmark-hle.yaml \
    --start 50 --max-questions 50
```

The local chat backend must be up. HLE will exercise `search` and
`fetch_text` — make sure the relevant API keys (e.g. for the search
provider used by `src/tools/search-web/tool.py`) are set.

## Score

```bash
export CLAUDE_API_KEY=sk-ant-...
python bench/hle/judge.py --run-dir bench/hle/results/<timestamp>_<scenario>
```

Judge model is `claude-sonnet-4-6` (override with `--judge-model`).
Sonnet is the working judge for the related memory_recall and cspred
benches; staying on it here means cross-bench comparisons aren't
contaminated by judge-model drift. Opus 4.7 is more capable but enough
more expensive at HLE volumes to make it the wrong default — revisit
once we want a tighter ceiling on judge noise.

## Output

`bench/hle/results/<timestamp>_<scenario_stem>/`:

- `raw.jsonl` — one row per question with `question_id`, `question`,
  `gold`, `category`, `answer_type`, `reply`, `error`, `duration_s`,
  `world_name`.
- `run_meta.json` — run parameters + skipped-multimodal count.
- `scored.jsonl` (after judge) — raw row + `judge: {verdict, score,
  rationale}`.
- `summary.md` / `summary.json` (after judge) — overall +
  per-category breakdown.

## Design notes

**Per-question isolation.** Same as MMLU: fresh `ChatLoop` with a
unique `world_name`. One question can't pollute another's context or
memory.

**Multimodal questions are skipped.** HLE includes some questions with
images. The runner's `image` field check filters them; the count is
reported in `run_meta.json`. Image support is straightforward to add
later (ChatLoop already accepts `image_url`), but not in v1.

**Judge.** Single three-way verdict (correct / partial / incorrect)
mapped to a 0/0.5/1 score. We report both `score_mean` and
`exact_correct_rate`; the latter is what's usually compared across
HLE leaderboards. The judge prompt is strict — near-misses on
technical/scientific questions count as incorrect, not partial.

**Scenario flags.** `chat.benchmark_mode: true` makes post-turn
reflection inline. `discourse` and `orientation` are off — same
rationale as MMLU. Tools are NOT omitted; HLE expects retrieval where
useful.
