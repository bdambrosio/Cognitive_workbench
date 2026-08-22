# measure — score traces, not tasks

Replaces `bench/` (deleted 2026-08-22). Design: `docs/measurement-v3.md`.

## The idea in one paragraph

Task success is binary, saturates, and is not comparable across a coding
task, a venture memo and tic-tac-toe. Trace properties — where a claim came
from, whether evidence preceded authoring, whether an agent waited for its
partner — are **the same measurements on every task**. So the heterogeneity
problem dissolves: you don't compare the tasks, you measure one set of
properties across all of them. Tasks become *fixtures* that elicit
behaviour; they are not scored.

## Running

```bash
python3 measure/report.py --world venture_solo
python3 measure/report.py --world jill_chat --agent Jill --since 2026-08-02
python3 measure/report.py --worlds-matching 'coord_search.*'
python3 measure/report.py --world jill_chat --since 2026-08-14 --commits
```

Read-only with respect to agent state. Needs no GPU, no server, no network.

## What is here

| file | role |
|---|---|
| `trace.py` | turn-level view of a run. Absorbs the join traps (below). |
| `process.py` | arc shape, re-authoring, evidence-before-authoring, stalls vs silent, exits |
| `provenance.py` | grounding mix, extraction density, quote gate, ref validity |
| `harness_rev.py` | joins a turn to the harness commit live when it ran |
| `report.py` | prints the vector |
| `regrade.py` | offline re-grade against the PINNED grader (costs money) |

`provenance.py` reuses `src/chat/claims.py` (`valid_refs_for`,
`_restore_observations`) rather than reimplementing it. The live grader is
production code and works; this reads what it produces.

## Two rules that are not negotiable

**Report a vector, never a score.** The v2 suite collapsed `turn_taking` to
1.0 for an arm whose own trace recorded `premature_reply: True` — the single
case in the whole campaign where a probe separated the arms, discarded by
the mean.

**Every rate carries its denominator.** Coverage varies enormously:
`observations_full` is on 150 of 3,010 jill_chat rows; `claims.jsonl` starts
2026-08-02 while the trace goes back to 05-03. A rate without its
denominator is not a fact.

## Join traps (all found on live data, all absorbed by the code)

- **`turn_seq` is not unique.** One value repeats 235 times in jill_chat.
- **`claims.ts` is the grading time, not the turn time** — grading runs
  after the turn, so the timestamps never match. Joining on ts equality
  silently matches nothing, which reads as "no claims" rather than as a bug.
  The join is turn_seq + nearest-preceding-ts; see `provenance.join_turn`.
- **`working_log` stores observations capped.** Matching quotes against the
  raw field gives a false ~35% failure rate. Use `_restore_observations`,
  and count capped-without-`observations_full` as *uncheckable*, not failed.
- **`respond` and `yield` are not tools.** They sit in the action stream
  where a tool name does; counting them inverts evidence-before-authoring.

## The three failure classes

Established 2026-08-22 against two labelled fabrications. Only the third is
detectable by checking refs, and it is the one *neither* labelled case was:

| class | example | ref-check sees it? |
|---|---|---|
| extraction miss | "Alpha Vantage key is already configured" never became a claim | no — nothing to check |
| stale source | a deleted benchmark suite, read from a doc that still exists | no — the ref is genuine |
| ref counterfeit | cited step does not support the claim | yes |

**Extraction recall is the ceiling on everything downstream.** No detector
fires on a sentence that never became a claim. `extraction_density` is a
proxy for it, not a measure — first evidence: density falls 8.65 → 2.88
claims/1k chars as replies lengthen (n=267), and the 27.6k-char venture
reply sits at 1.34. Confounded with genuine prose density; a hand-labelled
sample is what would settle it.

## Historical note — what `bench/` was, and why it went

Eleven v1 suites were cut 2026-08-18 (`dcd9e85d`) because they measured
capability while every failure that cost time was harness behaviour. The v2
replacement — six probes, mechanical scoring — was built to settle
Gemma-local vs Luna-cloud and **failed to**: 29 of 34 rows scored exactly
1.0, and two of the three others were arms disagreeing with themselves. No
between-arm signal, measurable within-arm noise; only wall clock separated.

Meanwhile the provenance instrument had been running in production the whole
time and spans **4% → 97% `model_prior` across the archived coord_search
arms** on the same task. It discriminates; the task scorers did not.

**Caveat on that spread, added the same day it was cited.** Each arm graded
itself, so an unknown share of the range is the grader. On one venture_solo
reply, re-grading with the pinned instrument moved `model_prior` from 32% to
0% and the extracted-claim count from 37 to 25. Re-grading the nine archived
arms through `regrade.py` is the first experiment, not a tidy-up. What does
not depend on a grader: 29 of 34 task rows scored exactly 1.0.

Worth keeping from the retired work, and carried forward here:

- mechanical scoring, no judge where a judge can be avoided
- hold the measuring instrument constant across arms
- n ≥ 3; a single run is anecdote (two conclusions were retracted for this)
- never change the instrument mid-campaign
- a suspiciously uniform result is a bug until proven otherwise — six metric
  bugs were found this way, and every one punished *correct* behaviour
- no ship gate until the numbers demonstrably separate

Deleted with `bench/`: `docs/harness-behaviour-suite.md`,
`docs/harness-roadmap.md`, `docs/harness-m0-m1-status.md`,
`docs/self-awareness-benchmarks.md`,
`docs/introspective_fidelity_benchmark_v01.md`. The last two described
suites removed in `dcd9e85d` and are the stale sources a live venture run
read and reported as current — deleting them closes that trap.

`bench/dataroom/` is kept at `measure/fixtures/dataroom/`: a synthetic
corpus with an answer key whose defects form a difficulty ladder (*stated*
→ *cross-document* → *derived*). That ladder is how a fixture avoids
saturating — report the highest tier reached, not pass/fail.
