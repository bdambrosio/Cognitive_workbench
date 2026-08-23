# dataroom — campaign state

Started 2026-08-23, on the workflow harness. Everything before it was
discarded, deliberately — see "Why every earlier run was discarded".

## Scored runs

| run | arm | wf_mode | rung | T1 | T2 | T3 | stated | cross | unsup | report | legs / wall |
|---|---|---|---|---|---|---|---|---|---|---|---|
| wm1 | Qwen3.8-27B | on | derived | 3/3 | 1/2 | 3 | 1/3 | 5/6 | 2 | 1,519w | 2 / 858s* |
| wm2 | Qwen3.8-27B | on | derived | 3/3 | 1/2 | 6 | 3/3 | 6/6 | **7** | 2,214w† | 2 / 433s |
| wm3 | Qwen3.8-27B | on | derived | 3/3 | 1/2 | 2 | 0/3 | 5/6 | 0 | 1,510w | 2 / 387s |
| wm4 | Qwen3.8-27B | on | derived | 3/3 | 1/2 | 5 | 2/3 | 6/6 | 2 | 1,637w | 2 / 512s |
| wm1 | gpt-5.6-luna | on | derived | 3/3 | 2/2 | 5 | 3/3 | 5/6 | 0 | 1,206w | 4 / 729s |
| wm2 | gpt-5.6-luna | on | cross_doc | **2/3** | **0/2** | 2 | 0/3 | 4/6 | 0 | 1,244w | 9 / 813s |

\* contended — the live agent shared the GPU. † over the 2,000-word guide.

**n=4 Qwen, n=2 Luna, one configuration. Read the spread, not the rows.**

## What replication actually showed

Two things are stable across all six runs, both arms:

- **Placement.** P2 first, every run. The key ranks it #1 of 12.
- **Coverage.** 9 of 9 documents opened, every run.

Everything else moves, and the two arms move in different places:

**Qwen is deterministic at the top of the ladder and noisy at the bottom.**
Four for four on Tier 1, four for four on Tier 2 (always F2, never F1). Then
the stated rung spans its entire possible range 0/3 to 3/3 across four runs
of one config, Tier 3 spans 2 to 6, and unsupported claims span **0 to 7**.

**Luna is noisy at the top.** Between two runs of the identical config it
dropped a must-find item (3/3 to 2/3) and both derived findings (2/2 to 0/2),
while doing more work — 9 legs and 64 iterations against 4 and 39.

That split is worth more than any single score. The method's §4 priority
order — safety, architecture, operations, micro — is doing real work on the
arm that follows it: the ranked top is deterministic and the unranked bottom
is noise, which is exactly what §4 argues for on the grounds that a report
cut short must still be decidable.

## What this retires

**Tier 3 and the stated rung are not measurements at n<3.** A 2-to-6 spread
on four identical runs means any between-arm comparison on those axes was
reading noise. Several were made in this session and were wrong.

**No claim about `workflow_mode`, in either direction.** Two conclusions were
drawn today — "Qwen degraded under it", "Luna improved under it" — and
replication killed both. Each arm's own spread on one config covers the
entire difference that had been attributed to the change. What IS verified is
that the lean config carries less: 0 chars of discourse, companion and
orientation state per turn against 1,252 / 2,791 / 460, and two fewer LLM
calls per turn. Whether that is free in quality terms is not answerable from
this data.

**Unsupported claims are the least stable thing scored** — 0 to 7 on one
config — and also the one with the worst product consequence. An unsupported
claim in a delivered report is precisely the §5 provenance failure the method
exists to prevent. That instability deserves attention before any of these
numbers are quoted.

## One finding that survived replication

**F1 never lands. F2 always does.** Across four Qwen runs, the backup-expiry
derivation (30-day retention against a 2026-07-30 last-good backup) appears
every time; the revenue derivation ($16k Stripe against $24k wired, 60% of
revenue unverifiable from the payment processor) appears never. Two derived
findings of the same shape, one reliably reachable and one not. That is
consistent enough across runs to be a property rather than variance, and it
is the most concrete open question the fixture has produced.

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

## The Tier 2 collapse — the method has no slot for a derived finding

All three arms scored 0/2 on Tier 2, and all three found the underlying
evidence. They stopped one step short, in the same place.

The answer key's F2 is *"30-day retention against 21 days of failures puts the
last recoverable backup at total loss around 2026-08-29"* — described there as
**the strongest single finding available**. Qwen got to the doorstep: *"The
last successful backup is 24 days old. The 30-day retention window is being
consumed by 21 consecutive days of failure."* Every fact needed is on the
page. It never computes the date. Luna declined explicitly — *"the supplied
lines do not establish 30-day usable retention"* — and Ultra filed it as
`(duplicate)` of the backup finding.

**§5's finding format has nowhere to put it.** The format is:

    Claim (<source>): <the stated claim>
    Evidence: <file:lines> — <what the code actually does>
    Delta: <None, or the specific gap>

and "a finding must cite its source, both halves: the document making the
claim, and the file and line range showing the implementation."

A derived finding tests no stated claim. Nobody claimed a backup expiry date;
it is a consequence of two facts in two documents. Under §5 it has only one
home — the finding for the claim it escalates — which is exactly where all
three arms put it. Ultra's `(duplicate)` label is the method working as
written.

Searching the whole document for a slot: no occurrence of *derived*,
*arithmetic as a finding type*, *projection*, *forecast*, or *implication*.
The one place arithmetic appears is §5's warning that a prose summary drops
"the arithmetic that made the finding material" — the method knows the
arithmetic is what makes a finding material, and gives it no structure.

**So the fixture rewards what the method's format excludes.** That conflict
was invisible while the method was not reaching the agent. It appeared the
moment the workflow harness delivered §5 intact and the arms started obeying
it.

This is documentary inference, not a controlled test. The test is to give §5
a derived-finding shape — a claim slot that names the two facts rather than a
stated claim — and re-run. Until then, Tier 2 is measuring whether an arm
will break the format it was given.

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

## Open: the auditor cannot talk to the client

§12 step 4 requires it — *"if a delta is found, stop and confirm with the
client before continuing; the audit is a collaboration, not a surprise"* —
and §8 defines the working recap as living "in the auditor-client
conversation". Neither is reachable. The runner sends `continue` and nothing
else; there is no channel back.

The deliverable has "what I should ask Dave before closing", but Dave is the
seller and that advice is post-audit. The missing thing is different:
**questions for the client, raised during the audit, whose answers would
materially change it.** Scope questions ("is the source repo available?"),
direction questions ("should the eight pilot accounts count as revenue?"),
and the §12-step-4 delta confirmations.

Candidate shape: a third output alongside the report and the Gap Map — open
questions, each with what it would change. Cheap, needs no new channel, and
turns an unanswerable requirement into a deliverable. Whether it belongs in
§12, §16, or as its own section is a method decision and is not made.

Noted 2026-08-23. Not built, deliberately.

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
