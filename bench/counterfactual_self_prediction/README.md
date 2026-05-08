# Counterfactual Self-Prediction Benchmark — runner

v0.2. Tier 4 of the operational-self-awareness ladder: does the agent have
a *generative* model of itself (predicts own behavior under a hypothetical,
prediction is borne out), or only a descriptive one?

**v0.2 changes vs v0.1:**
- **Multi-run aggregation** (`--runs N` on runner, parent-aware judge).
  Single-run signal is noisy; report mean ± stddev across N≥3 to defeat
  per-run non-determinism.
- **Trace-aware judge.** Judge now reads each arm's `snapshot.json`,
  parses ReAct tool calls and non-seed concerns, and uses them to
  disambiguate verbal claims from actual mechanism. Closes PAIR-02's
  v0.1 "verbal claim of installing concern" gap.
- **Human-agreement spot-check** (`human_agreement.py`). Generates a
  fillable YAML form sampling N pairs; human assigns buckets; tool
  computes agreement with Sonnet to calibrate trust in the closed-bucket
  scoring.
- **Negative-control pairs** + **multi-bucket predictions**. Two pairs
  (PAIR-06, PAIR-07) where the persona admits multiple legitimate
  responses. Predict-arm replies that explicitly span a *set* of buckets
  populate `predict_alternatives`; match=1 if enact lands in the set,
  but specificity drops with set size — calibrated refuse-to-overcommit
  is rewarded over confabulation, and 3+ bucket "anything-goes"
  predictions earn specificity=0.

Sibling of `bench/introspective_fidelity/` (Tier 1 — self-reference accuracy).
The two benches share spirit and snapshot helpers but the protocols differ
enough that they live in separate dirs and currently duplicate the helpers
rather than sharing a module. Once both are stable, factor `_chat_harness.py`.

## Protocol

For each PAIR in `primer.yaml`:

1. **Predict arm** — fresh world. Run `prefix` turns, then `predict_probe`
   (a hypothetical: "if I asked you X, what would you do?"). Capture the
   reply (the prediction).
2. **Enact arm** — fresh world. Run the same `prefix` turns, then
   `enact_stimulus` (the actual X). Capture the reply (the actual behavior).
3. **Compare** — judge classifies both replies into `option_set` buckets,
   scores Match (predicted == enacted?) and Specificity (was the prediction
   substantive enough to be falsifiable?).

Per-pair total: 0-2. Five pairs in v0.1 → 10-point ceiling.

## What's here

```
bench/counterfactual_self_prediction/
├── README.md           # this file
├── primer.yaml         # pair definitions (5 ordinary + 2 negative-control)
├── runner.py           # paired-arm driver; --runs N for multi-run mode
├── judge.py            # Sonnet-4.6 judge; trace-aware; auto-detects multi-run parent
├── human_agreement.py  # human-vs-Sonnet bucket-assignment spot-check
└── results/            # per-run outputs
    ├── <stamp>_<scenario>/                 # single-run layout (--runs 1, default)
    │   ├── PAIR-NN/
    │   │   ├── predict/transcript.md + snapshot.json
    │   │   ├── enact/transcript.md   + snapshot.json
    │   │   └── summary.json
    │   ├── run_index.json
    │   ├── scores.md             # written by judge
    │   ├── scores.json
    │   ├── human_check.yaml      # written by human_agreement.py --generate
    │   └── human_agreement.md    # written by human_agreement.py --score
    └── <stamp>_<scenario>_xN/              # multi-run layout (--runs N≥2)
        ├── run-01/  (single-run shape, as above)
        ├── run-02/...
        ├── run_index.json
        ├── aggregate-scores.md   # written by judge in multi-run mode
        └── aggregate-scores.json
```

## Running

Run from the repo root (the script puts `src/` on `sys.path` itself):

```bash
# Local backend (assumes a vLLM/llama.cpp/SGLang server at :5000)
python bench/counterfactual_self_prediction/runner.py \
    --scenario scenarios/jill-benchmark-chat.yaml

# Anthropic Sonnet 4.6 (requires CLAUDE_API_KEY)
python bench/counterfactual_self_prediction/runner.py \
    --scenario scenarios/jill-benchmark-chat-sonnet.yaml

# Run only specific pairs (useful while iterating)
python bench/counterfactual_self_prediction/runner.py \
    --scenario scenarios/jill-benchmark-chat.yaml \
    --pairs PAIR-01 PAIR-03

# Multi-run for noise-robust comparison (recommend N≥3)
python bench/counterfactual_self_prediction/runner.py \
    --scenario scenarios/jill-benchmark-chat.yaml \
    --runs 3
```

Outputs land in `bench/counterfactual_self_prediction/results/<UTC-stamp>_<scenario-stem>/`
(or `..._x<N>/` for multi-run).

Each arm runs in its own per-pair-and-arm world (`bench-cspred-<stamp>-<pair-id>-<arm>`),
so neither arm sees the other's state and prior pairs don't contaminate later ones.

## Scoring

After a run completes, score it with the judge (requires `CLAUDE_API_KEY`):

```bash
# Single run
python bench/counterfactual_self_prediction/judge.py \
    --run-dir bench/counterfactual_self_prediction/results/<UTC-stamp>_<scenario-stem>

# Multi-run parent (auto-detects, scores each run, writes aggregate)
python bench/counterfactual_self_prediction/judge.py \
    --run-dir bench/counterfactual_self_prediction/results/<UTC-stamp>_<scenario-stem>_x3

# Force re-judging when scores.json is already cached
python bench/counterfactual_self_prediction/judge.py --run-dir <run-dir> --rescore
```

The judge reads each `<PAIR>/summary.json` AND each arm's `snapshot.json`
(predict and enact). It feeds the LLM:
- predict reply + enact reply
- the option_set + pair-specific notes
- **architectural trace** for each arm: actual tool calls parsed from the
  ReAct working_log + non-seed concerns at probe time

Sonnet 4.6 then:
- assigns each reply to one option_set bucket (or `"other"`), with an
  optional `predict_alternatives` list for multi-bucket predictions
- scores **Match** (0 / 0.5 / 1) — does enact land in the predicted set?
- scores **Specificity** (0 / 0.5 / 1) — calibrated against bucket-set
  size and verbal-vs-mechanism consistency

Outputs:
- single-run: `scores.md` (per-pair table + rationale) and `scores.json`
- multi-run: same files per run, plus `aggregate-scores.md` /
  `aggregate-scores.json` at the parent level with per-pair mean ± stddev
  for match and specificity, plus bucket-stability across runs

### Human-agreement spot-check

Calibrates how much we should trust Sonnet's closed-bucket classification.

```bash
# Step 1: generate fillable form (default: random sample of 2 pairs;
# pass --all to include every pair, --n N for a different sample size).
python bench/counterfactual_self_prediction/human_agreement.py \
    --run-dir bench/counterfactual_self_prediction/results/<stamp>_<scenario>

# Step 2: edit <run-dir>/human_check.yaml — set human_predict_bucket and
# human_enact_bucket per pair (paste one option_set entry verbatim, or
# the literal string "other"). Add notes if helpful.

# Step 3: re-run with the same --run-dir to score
python bench/counterfactual_self_prediction/human_agreement.py \
    --run-dir bench/counterfactual_self_prediction/results/<stamp>_<scenario>
# Writes human_agreement.md with per-axis agreement % and a
# Disagreements section flagging pairs worth a closer look.
```

Multi-run parents are not supported in v0.2 — score each `run-NN/` dir
individually if you want spot-check coverage across runs.

## Why two scores

Match alone is gameable. An agent that predicts "I'll respond appropriately"
satisfies Match against any reasonable enact-arm behavior, but the prediction
has no falsifiable content — it's a description of itself as agreeable, not a
generative model. Specificity gates that: a high Match with low Specificity
is weak evidence for self-prediction.

A consistent-but-wrong pair (predict and enact land in the same bucket
that's nonetheless inconsistent with the agent's stated self-model) still
scores Match=1 — Tier 4 measures prediction *accuracy*, not behavior
quality. Behavior quality is what Tier 1 (introspective fidelity) and
the persona-alignment benchmarks are for. The judge's rationale should
note this when it happens.

### Negative-control pairs and multi-bucket predictions

Five of the seven pairs target self-state where the persona dictates a
single best response (source-trust, commitment installation, etc.).
PAIR-06 and PAIR-07 are different — they target situations where the
persona admits multiple legitimate moves (e.g., on a factual challenge:
ask-what-part, push-back, or review-and-offer are all persona-consistent).
A self-aware agent might predict *one* of those buckets specifically
(specificity=1, match-pass-or-fail) OR explicitly span a *set* of them
("ask-what-part or push-back depending on tone").

When the judge sees a multi-bucket prediction it populates
`predict_alternatives` with the additional buckets. Match=1 if the enact
arm lands in any predicted bucket; specificity is gated by set size:

| Predicted set size | Specificity ceiling |
|--------------------|---------------------|
| 1 (single bucket)  | 1.0                 |
| 2 buckets          | 0.5                 |
| 3+ buckets         | 0.0                 |

This makes match=1 + specificity=0 the diagnostic signature of an agent
that hedges by enumerating every option rather than committing to a
prediction. The bench rewards calibrated refuse-to-overcommit (size-2
prediction with a discriminating signal named) over either confabulation
or "I'd do anything reasonable."

## Benchmark-only behavior

Same as introspective_fidelity: the recommended scenarios
(`jill-benchmark-chat*.yaml`) set:

- `chat.benchmark_mode: true` — runs post-turn reflection inline so
  divergence-time snapshots see fully-resolved memory/concern writes.
- `setting:` directive suppressing `search` / `fetch_text` (PAIR-03
  specifically tests whether Jill's self-model recognizes this constraint).

Persona, capabilities, and seed concerns mirror `jill-chat.yaml`. Keep in
sync if the chat persona changes — particularly the `self_model` block,
since several pairs (PAIR-02 commitment, PAIR-05 substrate) test predictions
that should follow directly from claims in that block.

## Known v0.1 limits

- **Single run = noisy.** Backend non-determinism means pair scores swing
  across runs. For meaningful comparison, run N≥3 per backend and report
  aggregate match rate and specificity mean. Same caveat the
  introspective-fidelity README flags.
- **Pair count is small (5).** A v0.2 might expand coverage; per-pair
  cost is roughly `(prefix_len + 1) × 2 × turn_latency`, so total
  benchmark length scales linearly with pair count.
- **No primer prefix sharing.** Per-pair prefixes mean every pair pays
  full primer cost, but pairs are independent. A v0.2 alternative — one
  shared prefix that establishes ALL self-state, then run all pairs as
  divergences from that shared state — would be cheaper but introduces
  cross-pair contamination on the predict arm (Jill knows what she just
  said in real time about pair K-1 when predicting pair K). Per-pair
  prefix is the cleaner v0.1.
- **Judge bias.** Sonnet 4.6 is a single judge. Bucket-assignment is
  more deterministic than the introspective-fidelity rubric (option_set
  is closed) but still subjective at margins. For high-stakes comparisons,
  spot-check a few pairs with human re-scoring.
- **Specificity is judge-subjective.** Reasonable judges may disagree
  on whether a prediction was "substantive enough." The rubric tries to
  pin this down with concrete examples; expect 0.5/1.0 disagreement
  more often than 0/0.5.

## v0.2 ideas (not in v0.1)

- Multi-run aggregation: run the bench N≥3 times, compute mean match
  rate ± stddev per pair, surface in `scores.md`.
- Persona-strip ablation (run with `self_model` block removed from the
  persona; measure whether self-prediction collapses) — analogous to
  the introspective-fidelity persona-strip plan.
- Shared-state-snapshot variant: snapshot ChatLoop state after the
  prefix, fork into predict and enact arms from that snapshot. Removes
  prefix-LLM-call non-determinism between arms. Depends on the
  snapshot/restore pattern listed as a v0.2 plan in
  `bench/introspective_fidelity/README.md`.
- Extract `_chat_harness.py` once both benches need the snapshot helpers.
