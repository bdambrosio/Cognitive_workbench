# Measuring the workbench — score traces, not tasks

Written 2026-08-22. Implementation: `measure/`. Replaces the retired
`bench/` and the M0–M5 harness roadmap.

## Why the previous approach stopped working

`bench/` v2 was built to settle one question — Gemma-local vs Luna-cloud,
keep or sell the PRO 6000 — and it could not. Final campaign: **29 of 34
rows scored exactly 1.0**. Two of the three others were arms disagreeing
with themselves (qwen tictactoe 0.3333 vs 1.0; qwen_reasoning convergence
0.8125 vs 1.0). No between-arm signal on score, measurable within-arm noise,
and only wall clock separating anything.

The deeper problem is that task success is the wrong observable. It is
binary, it saturates, and it is not commensurable: there is no axis on which
a coding diff, a venture memo and a two-player tic-tac-toe game are the same
kind of achievement. Asking for the system's "capability frontier" on that
basis is not a well-posed question.

Meanwhile the provenance instrument in `src/chat/claims.py` had been running
in production the whole time, and it discriminates hard — **4% → 97%
`model_prior` across the archived `coord_search` arms on the same task.**

## The reframe

**Trace properties are the same measurements on every task.** Where a claim
came from, whether evidence preceded authoring, whether an agent waited for
its partner — these are computable from any run's trace regardless of what
the task was. So the heterogeneity dissolves. You do not compare the tasks;
you measure one set of properties across all of them.

Tasks become **fixtures**: chosen to elicit behaviour, never scored. Their
only design requirement is difficulty, because a fixture every arm completes
perfectly produces no behaviour worth measuring.

## Separating harness from model

Every run is tagged **(backend arm × harness revision)**. Because the metrics
are trace-derived, one instrument answers both questions:

- hold harness, vary arm → model comparison
- hold arm, vary harness → **harness regression** (the current priority)

The v2 suite only ever varied the arm, which is why a harness edit has never
been measurable. `measure/harness_rev.py` supplies the other axis by joining
each turn to the commit live when it ran. Retrospectively the live trace is
a natural experiment: jill_chat spans 2026-05-03 → 08-22 against 121 commits
touching `src/`.

## Metric families

1. **Provenance** — grounding mix, `model_prior` fraction, extraction
   density, quote-gate pass rate, ref validity, volatility-weighted
   `model_prior`. Cross-agent attribution ("checked by X" verified against
   X's own trace) is the multi-agent case.
2. **Process shape** — arc phases from tools called, evidence before
   authoring, re-authoring (the runaway shape), stalls vs silent, exits.
3. **Coordination** — ordering, premature reply, duplication.
4. **Cost** — wall clock, iterations, post-turn tail.

One capability anchor (HLE pin-30) is reported beside these and never folded
into them, purely to catch a backend that is simply weaker.

## How saturation is avoided

- **Distributional metrics, not pass/fail.** Grounding mix has an observed
  93-point spread; it has no ceiling because it is not "did you succeed".
- **Graded difficulty where ground truth exists.** `measure/fixtures/dataroom/`
  has a ladder — *stated* → *cross-document* → *derived* — with an answer
  key. Report the highest tier reached.
- **Fixture retirement.** Borrowed from SPADE's difficulty anchor
  (arXiv 2608.19197, which pays environments whose win rate falls in
  [0.4, 0.6]): retire any fixture where every arm reaches the top tier.
- **Never average a discriminating fact away.** v2 scored `turn_taking` 1.0
  for an arm whose trace recorded `premature_reply: True` — the one case in
  the campaign where a probe separated the arms, lost to the mean.

## The three failure classes

Established 2026-08-22 by checking two labelled fabrications from the venture
runs. Only the third is caught by checking refs, and it is the one *neither*
labelled case turned out to be:

| class | example | ref-check sees it? |
|---|---|---|
| **extraction miss** | "Alpha Vantage key is already configured" was never extracted as a claim | no — nothing to check |
| **stale source** | a benchmark suite deleted in `dcd9e85d`, read from a design doc that still existed | no — the ref is genuine |
| **ref counterfeit** | the cited step does not support the claim | yes |

**Extraction recall is the ceiling on everything downstream.** No detector
can fire on a sentence that never became a claim. First evidence that it
degrades: median density falls 8.65 → 2.88 claims per 1k reply chars as
replies lengthen (n=267 turns), and the 27.6k-char venture reply sits at
1.34. Confounded with genuine prose density — a hand-labelled sample is what
would settle it, and that is verification step 1.

Stale-source is detectable only for *self-referential* claims, by re-checking
the cited source against current state. Not generally detectable for external
claims; say so rather than implying coverage.

## The grader is a large part of the measurement

The live grader calls `self.backend.chat` (`src/chat/claims.py`), so the
backend under test grades its own output. `measure/regrade.py` fixes this
for measurement by re-grading offline against a pinned instrument
(`gpt-5.6-luna`, `reasoning_effort=low`, matching the cloud arm). Production
grading stays as it is — it feeds the agent's own verification concern and
works.

**How much this matters, measured 2026-08-22.** The same venture_solo reply,
graded by the run's own backend and then by the pinned grader:

| grounding | self-graded | pinned luna |
|---|---|---|
| retrieved | 19 (51%) | 16 (64%) |
| **model_prior** | **12 (32%)** | **0 (0%)** |
| inferred | 3 (8%) | 5 (20%) |
| user_asserted | 3 (8%) | 3 (12%) |
| context | 0 | 1 (4%) |
| **total extracted** | **37** | **25** |

Both extraction count *and* attribution move on that one turn.

### The full experiment — all 15 coord_search arms re-graded

Run 2026-08-22, 67 cloud calls, `measure/grader_delta.py`. It corrects the
n=1 worry above rather than confirming it.

**The aggregate rate is robust.** Across 15 arms, `model_prior` is **25.9%
self-graded and 25.9% pinned** — identical. The headline spread survives
intact: 0.0% → 97.3% both ways.

**The extreme is real.** `Gemma4_1/Jack` reproduces exactly — 37 claims both
ways, 97.3% `model_prior` both ways, 0% retrieved both ways. Two independent
instruments agree that arm fabricated rather than searched.

**But per-arm readings from self-grading are not trustworthy, and the reason
is extraction, not attribution.** The pinned grader extracts **+62% more
claims** (355 → 576). Seven of 15 arms extracted at most half what one
consistent instrument found:

    coord_search_luna.run1/Jack     0 ->  44 claims   (self-grading produced nothing)
    coord_search_luna.run1/Jill     1 ->  34
    coord_search.luna2/Jill         2 ->  28
    coord_search.Gemma4_1/Jill      6 ->  41
    coord_search.luna3/Jill         3 ->  16

Under-extraction reads as innocence. Three arms that scored a clean **0.0%
`model_prior`** carry substantial background-knowledge claims once a
consistent instrument looks:

    coord_search.Gemma4_1/Jill    0.0% -> 48.8%   (6 -> 41 claims)
    coord_search.gemma3/Jill      0.0% -> 30.6%   (36 -> 36 claims — same count!)
    coord_search.gemma1/Jill      0.0% -> 25.0%

`gemma3/Jill` is the sharp case: identical claim count, and attribution
still moves 0.0 → 30.6. So both mechanisms are live; extraction is merely
the larger one.

**What this settles.** Aggregate provenance rates are safe to quote from
self-graded data. Per-arm comparisons and any "this arm was clean" reading
are not — those require the pinned instrument. A zero is the least
trustworthy number in the self-graded table.

What never depended on a grader: task scores saturated at 1.0 on 29 of 34
rows. That is mechanical, and it is why task outcome was abandoned as the
observable.

## Verification before trust

1. **Extraction recall** against a hand-labelled sample. Until this number
   exists, every provenance metric has unknown recall.
2. **Landmark-commit response** — metrics must move at changes known to have
   had an effect (`dd366453`, `96e03f54`, `9444c579`).
3. **Quote-gate regression** — on uncapped turns the gate is currently 100%
   (n=257). Any change must reproduce that.
4. **Grader disagreement** — re-grade a sample twice; report the
   disagreement rate as the instrument's own noise floor.
5. **Replicate variance** — run over the 9 archived `coord_search` arms.
   Run-to-run variance under fixed conditions is still unknown, and is
   required before any difference is called real.

## Deliberately not decided

No ship gate and no composite score. Run as a reporting instrument until the
numbers demonstrably separate. Inventing a gate before seeing the spread is
how the old mean−1.5×band formula produced a 0.660 threshold that would have
waved through real regressions.
