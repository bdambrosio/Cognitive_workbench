# Counterfactual Self-Prediction Benchmark — runner

v0.3. Tier 4 of the operational-self-awareness ladder: does the agent
have a *generative* model of itself that correctly predicts how its
behavior would shift under a defined perturbation?

**v0.3 changes vs v0.2:**
- **True counterfactual structure (diff-in-diff).** v0.2's "predict vs
  enact" arms ran in the same operational state with hypothetical
  "if I asked you" framing standing in for a perturbation — measuring
  hypothetical self-prediction under unchanged conditions, not
  counterfactual reasoning. v0.3 adds a *perturbation axis* per pair:
  base + perturbed cells in both predict and enact, four cells total.
  Tier-4 score is bucket-equality on the *shift*: does
  (predict_cf − predict_base) match (enact_cf − enact_base)?
- **Schema-level perturbation realization.** Perturbations are now
  enacted at the agent runtime/schema level for the cf-cell, not via
  prose setting-block vetoes. The prose-veto approach contradicts the
  schema (the agent sees `search` in the tool catalog AND "don't use
  search" in setting); the cf-cell strips the affordance instead.
- **Pair retirement.** Pairs whose self-state has no clean
  perturbation axis (PAIR-06 let-stand, PAIR-07 ambiguous-correction)
  are reclassified as Tier-3 introspective-fidelity probes — moved to
  that bench or dropped. Tier-4 is not the right home for them.
- **Generic-Q-shape control.** The base cell is the control that
  separates "her self-model engaged with the perturbation" from "the
  question constrained the answer space." A cf prediction alone is
  gameable by anyone who reads the question; a *shift* prediction
  requires actual self-modeling.

**Why v0.2 wasn't actually counterfactual:** identified analyzing
PAIR-03 (3-run x3 results, 2026-05-08). Jill's predict-arm described
her standard ReAct-with-search loop; her enact-arm refused with the
session-constraint cited. The "miss" looked like a self-model
failure, but on inspection both arms ran in identical operational
state, and the schema-level tool catalog still listed `search` and
`fetch_text`. The setting-block prose veto was the only contradicting
signal: predict-arm (narrating capabilities) followed the affordances,
enact-arm (forced to act) followed the prose veto. Both readings were
coherent — not a self-model gap. v0.3 makes perturbations operative at
the schema level and adds the base cell as a generic-shape control.

**v0.2 changes (carried forward into v0.3):**
- **Multi-run aggregation** (`--runs N` on runner, parent-aware judge).
- **Trace-aware judge** reads each cell's `snapshot.json`, parses
  ReAct tool calls and non-seed concerns to disambiguate verbal
  claims from actual mechanism.
- **Human-agreement spot-check** (`human_agreement.py`).
- **Multi-bucket predictions** (`predict_alternatives`) — applies
  per-cell in v0.3; Δ-comparison uses primary buckets.

Sibling of `bench/introspective_fidelity/` (Tier 1 — self-reference accuracy).
The two benches share spirit and snapshot helpers but the protocols differ
enough that they live in separate dirs and currently duplicate the helpers
rather than sharing a module. Once both are stable, factor `_chat_harness.py`.

## Protocol

For each PAIR in `primer.yaml`:

1. **Cell construction** — four fresh worlds per pair:
   - `predict_base` — default state; probe asks the question without
     naming the perturbation.
   - `predict_cf` — default state; probe wraps the base question with
     `perturbation.describe_in_predict_cf` (the perturbation lives in
     the *question text*, not in the agent's state).
   - `enact_base` — default state; stimulus is the direct request.
   - `enact_cf` — perturbed state per `cells.enact_cf.state`; stimulus
     is the same direct request as enact_base. The perturbation is
     *operative in the runtime*, never named in the stimulus.
2. **Run** — each cell runs the prefix turns then its probe/stimulus
   in its own world; capture reply + snapshot.
3. **Compare** — judge classifies all four replies into `option_set`
   buckets, then computes:
   - `predict_Δ = (bucket(predict_base), bucket(predict_cf))`
   - `enact_Δ   = (bucket(enact_base),   bucket(enact_cf))`
   - **Match** = `predict_Δ == enact_Δ`
   - **Δ-specificity**: `0` if `predict_base == predict_cf` (self-model
     insensitive to perturbation); `1` if `predict_cf == bucket(enact_cf)`;
     `0.5` if predict shifted but to a different bucket than enact_cf.

This split — perturbation in the *question* for predict_cf, in the
*runtime* for enact_cf — is what makes the bench counterfactual rather
than constraint-conditioned. We never describe the perturbation to the
enact arm; we never enact it on the predict arm.

Per-pair total: 0–2 (Δ-match × Δ-specificity, same scoring scale as v0.2).
Pair count varies as v0.1/v0.2 pairs are reclassified; see the
**Perturbation axes** table below for the v0.3 inventory.

## Perturbation axes

Each Tier-4 pair declares a perturbation in `primer.yaml`:

```yaml
perturbation:
  name: <axis-id>
  apply_in_enact_cf: |
    <how the runtime realizes the perturbation for enact_cf>
  describe_in_predict_cf: |
    <verbatim text the predict_cf probe wraps the base question with>
```

Realization cost varies by axis:

| Pair | Self-state axis | Perturbation | Realization | Cost |
|------|-----------------|--------------|-------------|------|
| PAIR-01 | source-trust weighting | source-preference prefix absent | drop the prefix turn for cf cells | cheap |
| PAIR-02 | commitment installation | concern-installation pathway disabled | session-level runtime flag honored by reflection | medium |
| PAIR-03 | tool-constraint awareness | `search`/`fetch_text` omitted | `omitted_tools` filter in `_build_react_tool_catalog` | cheap |
| PAIR-04 | external-knowledge staleness | companion-model context absent | orientation-builder flag (skip companion render) | medium |
| PAIR-05 | substrate boundary | self-model block absent | scenario field swap (per-cell `self_model` override) | cheap |
| PAIR-06 | (n/a — no clean axis) | reclassify to Tier-3 or drop | — | — |
| PAIR-07 | (n/a — no clean axis) | reclassify to Tier-3 or drop | — | — |

The cheap-realization pairs (01/03/05) can ship in a first v0.3
implementation pass; the medium-cost pairs (02/04) need runtime hooks
that don't yet exist (concern-pathway flag, orientation-builder flag)
and should be flagged as pending in primer.yaml until the hooks land.

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
    │   │   ├── predict_base/transcript.md + snapshot.json
    │   │   ├── predict_cf/transcript.md   + snapshot.json
    │   │   ├── enact_base/transcript.md   + snapshot.json
    │   │   ├── enact_cf/transcript.md     + snapshot.json
    │   │   └── summary.json   # per-cell + Δ-match block
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

The judge reads each `<PAIR>/summary.json` AND each cell's
`snapshot.json` (predict_base, predict_cf, enact_base, enact_cf). It
feeds the LLM:
- the four cell replies
- the option_set + pair-specific notes
- the perturbation declaration (so the judge knows what shift to expect)
- **architectural trace** for each cell: actual tool calls parsed from
  the ReAct working_log + non-seed concerns at probe time

Sonnet 4.6 then:
- assigns each cell reply to one option_set bucket (or `"other"`), with
  an optional `predict_alternatives` list for multi-bucket predictions
- computes per-pair **Δ-match** and **Δ-specificity** (see below)

### Δ-match (the Tier-4 signal)

```
predict_Δ = (bucket(predict_base), bucket(predict_cf))
enact_Δ   = (bucket(enact_base),   bucket(enact_cf))
match     = predict_Δ == enact_Δ        # tuple equality on the shift
```

Δ-specificity:
- `predict_base == predict_cf` → spec=0 (self-model insensitive to
  perturbation; predict didn't engage the named constraint at all)
- `predict_cf == bucket(enact_cf)` → spec=1 (predict shifted to the
  bucket enact_cf actually landed in)
- predict shifted but to a different bucket than enact_cf → spec=0.5

**Baseline-validity gate.** If `enact_base` doesn't exhibit the
default-state behavior the pair assumes (e.g., PAIR-03 expects
`enact_base = attempt_search`; if the agent refuses generically even
in default state, the perturbation has nothing to perturb), flag the
run and report match as `n/a`. Don't score uninformative runs as
match=0 — they're noise about baseline drift, not signal about
counterfactual reasoning.

Multi-bucket predictions (v0.2 hedge accommodation) apply per cell;
the Δ comparison uses primary buckets. A cf-cell that hedges with a
3-bucket set caps that cell's specificity at 0 regardless of Δ-match.

Outputs:
- single-run: `scores.md` (per-pair table with all four cells +
  Δ-match + rationale) and `scores.json`
- multi-run: same files per run, plus `aggregate-scores.md` /
  `aggregate-scores.json` at the parent level with per-pair mean ±
  stddev for Δ-match and Δ-specificity, plus per-cell bucket-stability
  across runs

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

## Why Δ-match instead of cell-match

Cell-level match alone is gameable two ways:

1. **Generic-Q-shape.** The cf-cell's question often constrains the
   answer space directly — "if you didn't have search, what would you
   do?" elicits a refuse-shape answer from anyone who reads the
   question, regardless of self-model engagement. A cell-level match
   on cf alone doesn't distinguish "her self-model engaged with the
   constraint" from "the question forced the answer."
2. **Trivial alignment.** An agent that predicts "I'll respond
   appropriately" satisfies cell-match against any reasonable enact
   behavior, but the prediction has no falsifiable content — a
   description of itself as agreeable, not a generative model.

Δ-match closes both. The base cell is the control: an agent that
predicts the *same* bucket for base and cf (whatever bucket) gets
spec=0 — it didn't engage the perturbation, regardless of whether the
prediction "matches." An agent that predicts a *shift* matching the
actual shift gets full credit. The shift can only come from the
self-model engaging with the perturbation, not from the question.

A consistent-but-wrong pair (predict-shift and enact-shift land in
the same Δ that's nonetheless inconsistent with the agent's stated
self-model) still scores Match=1 — Tier 4 measures prediction
*accuracy*, not behavior quality. Behavior quality is what Tier 1
(introspective fidelity) and the persona-alignment benchmarks are
for. The judge's rationale should note this when it happens.

### Multi-bucket predictions per cell

Multi-bucket prediction logic from v0.2 is preserved per cell — a
predict_base or predict_cf reply that explicitly spans a set of
buckets populates that cell's `predict_alternatives`. Cell-level
specificity is capped by set size as before:

| Cell predicted set size | Cell-specificity ceiling |
|-------------------------|--------------------------|
| 1 (single bucket)       | 1.0                      |
| 2 buckets               | 0.5                      |
| 3+ buckets              | 0.0                      |

The Δ comparison uses each cell's *primary* bucket. A cell that hedges
with a 3-bucket set caps that cell's specificity at 0 regardless of
the Δ outcome — and since pair-level Δ-specificity is the min across
cells, hedging on either cell drags the pair score down.

## Benchmark-only behavior

Same as introspective_fidelity: the recommended scenarios
(`jill-benchmark-chat*.yaml`) set `chat.benchmark_mode: true` — runs
post-turn reflection inline so divergence-time snapshots see
fully-resolved memory/concern writes.

**Removed in v0.3:** the `setting:` block prose veto on `search` /
`fetch_text`. v0.2 tried to suppress those tools via prose while the
schema-level affordance list still listed them; the resulting
contradiction confounded PAIR-03's signal (see "Why v0.2 wasn't
actually counterfactual" above). v0.3 realizes the same constraint
at the schema level via `omitted_tools` in the cf-cell only.

Persona, capabilities, and seed concerns mirror `jill-chat.yaml`. Keep
in sync if the chat persona changes — particularly the `self_model`
block, since several pairs (PAIR-02 commitment, PAIR-05 substrate)
test predictions that should follow directly from claims in that block.

## Resolved in v0.3

- **Not actually counterfactual (v0.2).** Predict and enact arms ran
  under identical operational state; the "counterfactual" was just a
  hypothetical "if I asked you" framing. v0.3's diff-in-diff with
  schema-level cf-cell perturbation closes this.
- **Prose-vs-schema affordance contradiction (v0.2).** Setting-block
  prose said "don't use search" while the tool catalog still listed
  it; PAIR-03's headline miss was an artifact of this conflict, not a
  self-model gap. v0.3 strips affordances at the schema level.

## Known v0.3 limits

- **Per-pair cost doubles vs v0.2.** Four cells per pair instead of two,
  so total benchmark length is roughly `4 × (prefix_len + 1) × turn_latency`
  per pair × N runs. For 5 pairs × 3 runs that's 60 cells; tractable but
  not free. Snapshot/restore (see ideas below) would cut prefix re-runs.
- **Medium-cost perturbation hooks pending.** PAIR-02 (concern-pathway
  flag) and PAIR-04 (orientation-builder flag) need runtime hooks that
  don't yet exist. Until they land, those pairs ship marked
  `pending: true` in primer.yaml and the runner skips them.
- **Single-judge bias.** Sonnet 4.6 is still the only judge. The closed
  option_set keeps bucket-assignment more deterministic than rubric-style
  scoring, but Δ-match is bucket-equality on tuples, so a single
  cell-bucket disagreement flips the whole pair. Use `human_agreement.py`
  for high-stakes comparisons; consider an Opus-4.7 second-judge pass
  before reporting headline numbers.
- **Δ-specificity granularity.** Three values (0 / 0.5 / 1) on a binary
  shift signal. Reasonable judges may disagree on whether a partial
  shift counts as "engaged but mispredicted" (0.5) vs "didn't engage"
  (0). Expect more 0/0.5 disagreement at margins than 0.5/1.
- **Baseline-validity-gate sensitivity.** If `enact_base` drifts off
  the assumed default behavior, the perturbation has nothing to perturb.
  The gate flags `n/a` rather than match=0, but if baseline drift is
  systematic across pairs, the bench produces no signal. Worth tracking
  baseline-bucket frequency across runs as a sanity check.

## v0.3 ideas (not yet in v0.3)

- **Persona-strip as an additional perturbation axis.** Currently only
  PAIR-05 explicitly strips the `self_model` block. Could generalize:
  run the whole bench with persona stripped as a global cf-perturbation
  to measure how much of self-prediction depends on the persona block
  vs. the architecture itself.
- **Cross-perturbation Δ-match.** Run multiple perturbations on one
  pair and check whether predict-arm distinguishes them (e.g., PAIR-03
  with both `search omitted` and `process_text omitted` — does Jill
  predict different shifts?).
- **Shared prefix snapshot/restore.** Snapshot ChatLoop state once
  after the prefix, fork all four cells from that snapshot. Cuts
  per-pair cost roughly 4× on prefix LLM calls. Depends on the
  snapshot/restore pattern listed as a v0.2 plan in
  `bench/introspective_fidelity/README.md`.
- **Independent re-judge by Opus 4.7.** Single-judge bias matters more
  in v0.3 because bucket-equality is unforgiving; a second judge run
  would tighten confidence intervals on aggregate Δ-match.
- **Extract `_chat_harness.py`** once both benches need the snapshot
  helpers and the four-cell driver pattern.
