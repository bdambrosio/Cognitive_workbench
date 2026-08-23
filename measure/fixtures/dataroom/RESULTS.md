# dataroom — campaign state

Updated 2026-08-23. **The instrument changed today**, so the three rows
below are not a comparison — see the banner under the table.

## Scored runs

| arm | rung | Tier 1 | Tier 2 | T3 | unsup | report | Gap Map | legs / wall | harness |
|---|---|---|---|---|---|---|---|---|---|
| **Qwen3.8-27B** (local) | **derived** | **3/3** | 1/2 | 5 | 1 | 2,127w | 163w | 2 / 474s | old |
| **Gemma4-31B-it** (local) | cross_document | 2/3 | 0/2 | 4 | **0** | 494w | 81w | 4 / 220s | old |
| **gpt-5.6-luna** (OpenAI) | **derived** | 2/3 | 1/2 | 4 | **0** | 1,210w | 121w | 4 / 503s | **fixed** |

> **DO NOT READ THIS TABLE AS A COMPARISON.** Luna ran on a harness with
> three fixes the other two never saw: the iteration cap raised 12→16, the
> `inspect` geofence narrowed, and the empty-answer sentinel corrected. Any
> of the three could move a score. Qwen and Gemma must be re-run on the
> fixed harness before these rows sit beside each other. This is exactly the
> mid-campaign instrument change the previous edition of this file warned
> against — it happened anyway, because the bugs were worse than the
> discontinuity.

All three led with P2, which the key ranks #1 of 12, so placement discipline
held everywhere. Luna sits between the local arms on the axes that differ:
it reaches the derived rung like Qwen, and carries zero unsupported claims
like Gemma. Luna is ALSO the campaign's pinned grader, so its row is
self-graded — an upper bound, not a measurement.

Deliverables live beside each run:
`results/<ts>_<world>/{report.md,gap_map.md,full_reply.md,run_meta.json}`

## Not scored, and why

| arm | outcome |
|---|---|
| nemotron-ultra (OpenRouter) | 429 DeepInfra + 503 Together — shared-pool contention |
| nemotron-super (OpenRouter) ×2 | 429, then 429 through all 4 retries |
| nemotron-ultra (DeepInfra direct, 1st) | 404 — doubled `/v1` in the arm's base_url, my bug |
| nemotron-ultra (DeepInfra direct, 2nd) | `max_iters` on turn 1 — burned 12 iterations without answering |

Neither nemotron arm has produced a valid run. The infrastructure problems
are solved; the `max_iters` exit is the only one that might be behavioural,
and a single occurrence is a draw, not a property.

## Pick up here

1. **Re-run Qwen and Gemma on the fixed harness.** Nothing in the table is
   comparable until this is done. Nemotron is parked by decision, not by
   failure.
2. **Replicate.** n>=3 per arm. Two conclusions in this project have already
   been retracted for treating n=1 as a result.
3. **Add the §9 taxonomy check to `score.py`, then rescore.** Both Qwen and
   Luna wrote the buyer's action vocabulary instead of the method's: Qwen
   "DO NOT PROCEED as presented", Luna "Pause; do not close pending material
   verification". §9 requires Clear / Clear with caveats / Conditional /
   Material / Walk. Two arms out of three failing the same way is a finding
   about the brief, not about either model.
4. **Persist NOTE lines in the stored working log.** `chat_loop.py:1667`
   keeps only `$step*` labels, so the budget nudge is stripped from every
   trace. No post-hoc analysis can currently tell whether an agent was
   warned before it ran out of iterations — see the yield-adherence finding
   below, which took driving the live loop to establish.

## Environment note that will bite again

`~/.bashrc` returns early for non-interactive shells (the standard
`case $- in *i*) ;; *) return;; esac` guard), so `export DEEPINFRA` at line
159 never runs for anything launched from a script. Extract it explicitly:

    eval "$(grep -m1 '^export DEEPINFRA=' ~/.bashrc)"; export DEEPINFRA

## What this campaign actually measured, honestly

Eight apparent findings turned out to be plumbing before any model finding
survived: the summariser dropping figures, OpenRouter's shared-pool rate
limits, a doubled `/v1`, a stopping rule that scored an unfinished turn as a
completed audit, two local arms that would have been mislabelled once the
served model changed, an empty-answer sentinel that reported success, and a
geofence that put the answer key inside the exam. Each is fixed and recorded
in the commit that fixed it.

Two behavioural findings have survived, and both are about **yield-adherence
under budget pressure** rather than about audit skill:

**1. Gemma ended a turn with a stated-but-unexecuted plan** — "I will now
begin working the priority order" — which its own character block forbids.
Found 2026-08-22; it is why the runner now treats a missing Gap Map marker,
not the exit reason, as the signal that work is unfinished.

**2. Nemotron Ultra was warned and kept starting new work.** `react.py:418`
injects a NOTE three iterations before the cap: *do NOT start anything new,
emit `yield` NOW*. At iteration 10 of 12 Nemotron received it, then read
doc8, read doc9, and began claim extraction — hitting the cap with 11 of 12
iterations spent on ingestion it could not use. Its reasoning was otherwise
clean: no repeats, no confusion, correct tool for every call. The single
thing separating it from a valid run was reading one document per call where
Gemma and Qwen batched three.

Luna is the contrast, on the fixed harness: leg 1 ran 14 iterations and
yielded on iteration 14 — the exact iteration the nudge fires at cap 16 —
and legs 2 and 3 yielded voluntarily at 7 and 10, well before any warning.

**Caveat on how #2 was established.** The NOTE is stripped from the stored
working log (`chat_loop.py:1667` keeps only `$step*` labels), so it appears
in no trace and its absence looks like evidence it never fired. It was
confirmed instead by driving the real loop with a scripted backend that
never yields: the NOTE lands in the in-memory log, which is the same string
that reaches the prompt. Until that filter is changed, no run's trace can
answer "was it warned?" — see pick-up item 4.

**Raising the cap does not fix this.** 12→16 moves the wall; a model that
ignores the nudge hits 16 the way it hit 12. The cap was raised because it
was selecting for read-batching strategy and scoring the difference as a
failed run, which is a separate defect.
