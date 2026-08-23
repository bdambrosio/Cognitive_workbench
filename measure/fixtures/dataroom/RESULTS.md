# dataroom — campaign state

Started 2026-08-23, on the workflow harness. Everything before it was
discarded, deliberately — see "Why every earlier run was discarded".

## Scored runs

| run | arm | temp | rung | T1 | T2 | T3 | uns | words | wall | §9 | THRESH |
|---|---|---|---|---|---|---|---|---|---|---|---|
| wm1 | Qwen3.8-27B | 0.7 | derived | 3/3 | 1/2 | 3 | 0 | 1,519 | 858s* | Material | **PASS** |
| wm2 | Qwen3.8-27B | 0.7 | derived | 3/3 | 1/2 | 4 | 0 | 2,214 | 433s | Material | FAIL |
| wm3 | Qwen3.8-27B | 0.7 | derived | 3/3 | 1/2 | 2 | 0 | 1,510 | 387s | Conditional | **PASS** |
| wm4 | Qwen3.8-27B | 0.7 | derived | 3/3 | 1/2 | 5 | 1 | 1,637 | 512s | — | FAIL |
| r1 | Qwen3.8-27B **+reasoning** | 0.7 | derived | 3/3 | 1/2 | 4 | 0 | 1,275 | 1,015s | Material | **PASS** |
| r2 | Qwen3.8-27B **+reasoning** | 0.7 | derived | 3/3 | 1/2 | 3 | 0 | 1,965 | 812s | Material | **PASS** |
| wm1 | gpt-5.6-luna | 0.7 | derived | 3/3 | **2/2** | 4 | 0 | 1,206 | 729s | Material | **PASS** |
| wm2 | gpt-5.6-luna | 0.7 | cross_doc | 2/3 | 0/2 | 2 | 0 | 1,244 | 813s | Conditional | FAIL |
| t025_1 | gpt-5.6-luna | 0.25 | derived | 3/3 | 1/2 | 2 | 0 | 1,227 | 548s | Material | **PASS** |
| t025_2 | gpt-5.6-luna | 0.25 | derived | 3/3 | 1/2 | 2 | 0 | 1,176 | 570s | Material | **PASS** |
| t025_3 | gpt-5.6-luna | 0.25 | derived | 2/3 | 0/2 | 5 | 0 | 1,302 | 650s | Material | FAIL |

\* contended — the live agent shared the GPU.

**7 of 11 clear the threshold.** Every run is workflow_mode. Graded by
`gpt-5.6-terra`, effort high, temp 0.1, 32k budget.

## The threshold, and why it is the number that matters

Tier counts are a score; a business case needs a floor. `score.py` reports
PASS/FAIL against six criteria a delivered report would be judged on: all
three must-find items, a Gap Map, a §9 recommendation, leading with a top-3
finding, no unsupported claims, and the word ceiling. What separates the four
failures is not depth — it is a missing must-find item (luna wm2, t025_3), a
recommendation in the wrong vocabulary (qwen wm4), and a report over length
(qwen wm2).

## The grader was validated, and swapped

`gpt-5.6-luna` over-credited. Checked by hand against the key on three
judgements:

- **B7** credited from a line in the memo's *questions for the seller*
  section — which the match prompt already forbids in as many words.
- **B6** credited on a finding that quoted the right claim and concluded
  "Delta: None", missing the co-located database that is the whole finding.
- **F1** credited on a report that states the substance ($24k of revenue
  outside the payment processor, flagged for verification) without computing
  the percentage.

`gpt-5.6-terra`, same prompt and settings, drops B6 and B7 and keeps F1 —
a finer distinction than the rule proposed at the time, which would have
thrown away all three. Two of those three calls were the grader being wrong;
the third was the reviewer being wrong.

**Accuracy improved; precision did not.** Three passes over one run on Terra:
Tier 3 = 5, 4, 5 and unsupported = 1, 1, 2. Tier 1, Tier 2 and the threshold
verdict held, but that run fails on §9 regardless — a run sitting at
unsupported 0 or 1 would flip PASS/FAIL on grader noise alone. **Any
threshold result within one unsupported claim of the boundary needs more than
one grading pass.**

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
