# dataroom — campaign state

Started 2026-08-23, on the workflow harness. Everything before it was
discarded, deliberately — see "Why every earlier run was discarded".

## Scored runs

| arm | rung | Tier 1 | Tier 2 | T3 | unsup | §9 verdict | report | Gap Map | legs / iters / wall |
|---|---|---|---|---|---|---|---|---|---|
| **Qwen3.8-27B** (local) | cross_document | **3/3** | 0/2 | 5 | 1 | Material | 1,923w | 153w | 2 / 15 / 472s |
| **gpt-5.6-luna** (OpenAI) | cross_document | **3/3** | 0/2 | 3 | **0** | Conditional | 1,284w | 158w | 6 / 34 / 412s |
| **Nemotron-3-Ultra** (DeepInfra) | cross_document | **3/3** | 0/2 | 4 | 1 | Material | 1,738w | 169w | 5 / 16 / 477s |

Deliverables live beside each run:
`results/<ts>_<world>/{report.md,gap_map.md,full_reply.md,run_meta.json}`

**Every arm found all three must-find items, used §9's vocabulary, opened all
nine documents, and led with P2 — which the key ranks #1 of 12. No arm
reached a derived finding.**

Three arms agreeing on everything the fixture makes easy, and failing
identically on the one thing it makes hard, is a statement about the fixture
rather than about the models. See "The Tier 2 collapse" below.

Ultra's run is its first valid one in six attempts — the previous five died
on 429, 429, 404, and twice on `max_iters` after spending eleven of twelve
iterations reading documents in order to read the method. It no longer has to
read the method.

Luna is the outlier in shape rather than score: the most legs and iterations
for the shortest report, the only arm with zero unsupported claims, and the
only one to land on Conditional rather than Material. Whether that is better
calibration or under-reading is not answerable at n=1.

**n=1 per arm. Nothing here is a result yet.**

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
