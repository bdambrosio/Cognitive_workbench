# Harness-behaviour suite (benchmarks v2)

Written 2026-08-18. Companion to `docs/harness-roadmap.md`, not a
replacement for it.

## Why this exists

The composite suite measures four things — memory recall, HLE, discourse
reflection, introspective fidelity. All four measure **capability**.

Now list what actually cost time between 2026-07-20 and 2026-08-18:

- `inspect` burning all 12 iterations without answering, 40–44% of calls
- `--reasoning` breaking the yield: past the nudge at 9 *and* the cap at 12
- a stated plan becoming an inert deferred concern (`Note_19`, activation 0.000)
- a 452s turn answering from a 7.5-minute-stale prompt
- six Steam titles claimed "verified" on 3 searches and 0 fetches
- simultaneous dispatch with no turn-taking; one agent authoring the other's line
- a 6-minute, 24-call post-turn tail settling three visible turns
- 160 `unknown tool None` from a `response_format` behind a local-only guard

**Not one is measured by anything in the composite.** Every one is harness
behaviour, and the composite would have scored identically through all of
them.

That is the gap. Not breadth of capability coverage — there is plenty of
that — but zero gated instrumentation on the layer where the failures
actually happen. It is also why the Gemma/Qwen/Luna question keeps failing
to resolve: a harness-behaviour question has been getting settled with
capability instruments plus n=1 anecdotes.

## Design commitments

In priority order. These are the parts to argue with, before the probes.

1. **Mechanical scoring wherever possible — no judge.** Judges cost money,
   inject roughly 10pp of noise, and have killed three run attempts on
   availability alone. The `concerns.py` mapping probe already proved the
   pattern: a 16-cell rubric against grep-checkable ground truth cleanly
   separated Gemma (12/16) from Luna (16/16) with no judge in the loop.
   `bench/coord_search/score.py` is the existing reference implementation
   and the template for everything here.

2. **Run per *backend*, not per harness change.** Different cadence and a
   different question from the composite. This is the instrument that
   decides local-vs-cloud on evidence.

3. **~20 minutes, not 104.** It has to be cheap enough to run three times.
   Any probe that cannot hold that budget gets cut, not extended.

4. **Add nothing external.** MMLU sits at 95% for both backends —
   saturated, no signal. HLE pin-30 remains the single capability anchor.
   Published leaderboards already describe models; nothing external
   describes *this harness on* a model.

5. **n≥3 or it does not count.** Gemma's three no-reasoning runs on
   2026-08-16 disagreed with each other more than the backends disagreed
   on most measures. Single runs are anecdote and have twice produced
   conclusions that had to be retracted.

6. **Hold the instrument constant.** Where a probe needs an LLM at all
   (extraction, not judging), it uses one fixed backend regardless of which
   backend is under test — otherwise extraction quality shows up as the
   metric. `coord_search/score.py` already does this deliberately; keep it.

## The six probes

| # | probe | measures | mechanical score | status |
|---|---|---|---|---|
| 1 | **Convergence** — `concerns.py` creation-site mapping | tool-loop convergence, search completeness | 16-cell rubric (4 facts × 4 paths) · `max_iters` rate · wall clock | ground truth exists; 4 archived runs give a free baseline |
| 2 | **State across turns** — tic-tac-toe vs scripted opponent | multi-turn state with no prompt scaffolding | legal-move rate · forced-block rate · board-render accuracy · terminal agreement | new; seeded by the verified 2026-08-18 game |
| 3 | **Yield** — `scenarios/yield_test.yaml` | turn-boundary handoff | `exit_reason == yield` · iters at exit · continuation correctness | scenario exists |
| 4 | **Claim honesty** — task needing N verifications, tools instrumented | the Steam-titles failure | claimed-verified vs actually-fetched, from the trace | partly exists as `model_prior` fraction in `coord_search/score.py` |
| 5 | **Turn-taking** — two agents, explicit ordering | multi-agent coordination | did B wait for A · did either author the other's line · stalls vs silent turns | `coord_search` scaffolding + scorer exist |
| 6 | **Cost rider** — attached to 1–5, not standalone | the thing that keeps being a surprise | wall clock · LLM call count · post-turn queue depth | instrument once, free thereafter |

Probes 1, 3, 5 reuse existing scaffolding. Only 2 and 4 are genuinely new,
and both stay small because they are mechanically scored.

### Probe 1 — ground truth

Lines drift; **re-extract before every campaign.** As of `96e03f54`:

| path | call | activation | rhythm_hours |
|---|---|---|---|
| seed | 484 | 0.0 (constructor default) | from YAML |
| hop-budget carrier | 2180 | **0.70** (primed :2196) | 1 |
| user-yield | 2219 | **0.70** (primed :2238) | 1 |
| successor | 2290 | **0.70** (primed :2311) | 1 |

Rubric cells: activation value · rhythm_hours · rhythm_source ·
autonomy-to-fire, for each of the 4 paths. Deliberately excludes breadth,
citation accuracy and prose — those are judgement, and scoring "found 6
paths not 4" as simultaneously a plus and a minus already happened once.

The priming cell is the discriminating one, and it is an honest miss: the
priming is not among the call's named arguments, it sits in an
`extra_properties={...}` dict 16–21 lines below. Read the argument list and
stop, and "constructor default applies" is the correct inference.

### Probe 2 — why tic-tac-toe

Verified on Gemma-4-31B with no reasoning, 2026-08-18, `jill_chat` turns
2794–2803: nine moves of board state carried across nine turn boundaries
with no board in the prompt, interleaved with three autonomous concern
fires. Correct unprompted forced-block at move 3, correct terminal-draw
deduction before the opponent announced it, and a canvas-update claim that
the trace confirms was true.

It is the ideal mechanical probe: forced blocks and terminal states are
computable, so the whole thing scores without a judge, and instances are
unbounded. Against a scripted opponent it is also fully deterministic.

## What this buys

Three runs each on Gemma-local and Luna-cloud and the GPU question stops
being a judgement call:

- Luna wins probes 1–5 decisively → the capability-floor caveat in the
  roadmap is proven, and selling the PRO 6000 is the correct call.
- Gemma holds probes 2–5 and loses probe 1 only on completeness → the
  problem is `inspect` scope, not the model, and the card stays.

Either way it resolves inside a day for roughly the cost of two composite
runs.

## Deliberate non-goals

- **Not folded into the composite.** Different cadence, different question;
  mixing them re-creates the incomparable-rows problem that the 2026-07-24
  HLE re-pin and the 2026-08-15 unequal-parser runs both caused.
- **No ship gate yet.** Run it as a reporting instrument across a few
  backends first. Attach floors only once the numbers demonstrably
  separate — inventing a gate before seeing the spread is how the
  mean−1.5×band formula produced a 0.660 gate that would have waved through
  real regressions.
