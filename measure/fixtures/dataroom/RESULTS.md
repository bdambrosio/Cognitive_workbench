# dataroom — campaign state

Started 2026-08-23, on the workflow harness. Everything before it was
discarded, deliberately — see "Why every earlier run was discarded".

## Scored runs

| arm | rung | Tier 1 | Tier 2 | T3 | unsup | §9 verdict | report | Gap Map | legs / wall |
|---|---|---|---|---|---|---|---|---|---|
| **Qwen3.8-27B** (local) | cross_document | **3/3** | 0/2 | 5 | 1 | **Material** | 1,923w | 153w | 2 / 472s |

Deliverables live beside each run:
`results/<ts>_<world>/{report.md,gap_map.md,full_reply.md,run_meta.json}`

Qwen took every cross-document item — 6/6, the first clean sweep of that
rung — and reached no derived finding. It led with P2, which the key ranks
#1 of 12.

**n=1. Nothing here is a result yet.**

## What the workflow harness changed

The method now loads verbatim into the static system prompt
(`scenarios/audit.yaml` -> `workflow: audit/METHOD.md`) instead of being
fetched with `inspect`. Three things reached an output for the first time:

- **§9's taxonomy.** The report is headed `### §9 Recommendation: Material`.
  Every earlier run invented a recommendation vocabulary instead — "DO NOT
  PROCEED as presented", "Pause; do not close pending material
  verification" — because the taxonomy never survived to the turn that
  needed it.
- **§15's Gap Map format.** Coverage line, "Full report with citations
  available on request", and the scope disclaimer verbatim.
- **§16's deliverable contract**, which until today existed only inside this
  runner's brief.

## Why every earlier run was discarded

They ran against a different instrument, and the differences are not
adjustments — they change what the agent is given and what counts as
finished.

| change | effect |
|---|---|
| method in the static prompt | §3-§16 reach the agent at all; before, the surviving fragment ended mid-sentence in §2 |
| iteration cap 12 -> 16 | an arm reading one document per call can finish ingesting and still have budget to work |
| `inspect` geofenced to `audit/` | the answer key and prior arms' reports are out of reach |
| empty-answer sentinel fixed | a subagent returning nothing says so, instead of reporting `OK` |
| Gap Map marker as the stopping rule | a turn that stops with a stated plan is no longer scored as a finished audit |
| engagement ledger on each `continue` | legs, minutes and documents-opened are carried by the runner |

Comparing across that line would be comparing two instruments. Those runs did
their job — every fix above was found by running them — and they remain in
git history if a specific claim ever needs checking.

## Pick up here

1. **Luna and Ultra**, one run each, then read all three together.
2. **Replicate to n>=3 per arm.** Two conclusions in this project have
   already been retracted for treating n=1 as a result.
3. **Add the §9 taxonomy check to `score.py`.** Qwen now conforms, but the
   scorer still cannot tell — it would have passed the non-conforming runs
   too.
4. **Persist NOTE lines in the stored working log.** `chat_loop.py:1667`
   keeps only `$step*` labels, so the budget nudge is stripped from every
   trace and no post-hoc analysis can say whether an agent was warned before
   it ran out of iterations.

## Environment note that will bite again

`~/.bashrc` returns early for non-interactive shells (the standard
`case $- in *i*) ;; *) return;; esac` guard), so `export DEEPINFRA` and
`export OPENAI_API_KEY` never run for anything launched from a script.
Extract them explicitly:

    eval "$(grep -m1 '^export DEEPINFRA=' ~/.bashrc)"; export DEEPINFRA
    eval "$(grep -m1 '^export OPENAI_API_KEY=' ~/.bashrc)"; export OPENAI_API_KEY

## Yield-adherence, carried forward

Two observations outlive the discard because they are about behaviour rather
than scores. Both deserve a probe of their own rather than being folded into
this fixture:

- An arm ended a turn with a stated-but-unexecuted plan — "I will now begin
  working the priority order" — which its own character block forbids.
- An arm received the budget nudge at iteration 10 of 12 (`react.py:418`:
  *do NOT start anything new, emit `yield` NOW*) and then started new work
  three more times.
