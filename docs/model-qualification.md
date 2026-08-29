# Model qualification — the frozen pass criteria

What a model must do before it is trusted with a delivered report. The
pre-screen ([model-prescreen.md](model-prescreen.md)) asks whether a model can
be *used* at all; this document asks whether its output can be *shipped*.

**Nothing in this document is computed. Every criterion below is read by a
person off an artifact a run already writes**, and the artifact and field are
named so the reading can be checked. Re-derived from the codebase 2026-08-29:
the criteria had been written as though something applied them, and no code
does. What follows says where each number comes from.

**Frozen means the criteria are fixed before a campaign's first run and are not
changed during it.** A change to them invalidates comparison across the boundary
exactly as a change to the instrument does — three campaigns were discarded on
2026-08-24 for instrument drift. Freezing is a discipline, not a mechanism: no
code checks it, and `harness_rev` in `run_meta.json` is what makes a violation
visible afterwards.

## What the code applies on its own

Two gates, both in `measure/prescreen.py`, both run before a model is a
candidate at all:

1. **Structured output** — does the endpoint accept `response_format=json_schema`
   with the real schema and return a schema-valid action.
2. **Tokens per call for a one-line action** — this architecture makes 35–187
   calls per run, so tokens per call sets wall clock. It excluded Qwen3.8-2.4T
   at 2,902 tokens for a one-line answer.

`prescreen.py` also names a third gate — one fixture run graded on
admissibility — and says outright that it is not in the file, because it is a
real run. Everything below is that third gate, expanded, and performed by hand.

## The unit of qualification

**Three valid runs of one model on the dataroom fixture, on one instrument
revision.** Where more than one model is under test, interleave by round.

Three is the floor, not a target. At n=1 the m1 campaign looked saturated —
three models, eight of eight criteria each — and at n=3 it separated them. Two
conclusions in this project have been retracted for treating a single run as a
result, and `score.py` prints "n=1 is anecdote" on every run for this reason.

**No code aggregates runs.** `score.py` scores one run; `measure/report.py`
reads chat-trace worlds rather than audit runs and declines a composite by
design. A campaign result is assembled by hand from three `run_meta.json` files,
three `score.py` outputs and three `review/` directories.
`measure/fixtures/dataroom/RESULTS.md` is that assembly, maintained by hand.

Record `git rev-parse HEAD` per run and keep the tree clean while a campaign
runs. `run_meta.json` records `harness_rev` and `target_rev` for you; a dirty
tree makes both meaningless, and after the fact the boundary is unrecoverable.

## What a run produces

Three instruments answering different questions. Only two exist at a paying
client, and the gating criteria are built on those two.

| instrument | artifact | fields the criteria read | needs the key | exists at a client |
|---|---|---|---|---|
| **the audit runner** — `workflows/claims_audit/runner.py` | `run_meta.json` | `error`, `blocks_delivered`, `blocks_prompted`, `blocks_closed`, `harness_rev`, `target_rev`, `resolved_model`, `resolved_temperature`, `top_p`, `workflow_mode`, `workflow_suppressed`, `wall_clock_s`, `finish_length_events` | no | **yes** |
| **the review** — `workflows/audit_review/runner.py` | `review/conformance.json`, `review/citations.json`, `review/summary.md` | markers, closed vocabularies, claim surface; every cited line fetched, references past end-of-file, `scheme`; ADMISSIBLE / INADMISSIBLE, supported N of M, exceptions with their standing | no | **yes** |
| **the fixture scorer** — `measure/fixtures/dataroom/score.py` | stdout | `MEASUREMENT INVALID`, Tier 1/2/3 recall, ladder rung, placement, subagent no-answer rate, report length, eight criteria and their conjunction | **yes** | no |

## The reviewer is pinned, and it is never the model under test

The review's dispositions are a model's judgement, so varying the reviewer
across a campaign varies the instrument. Hold one reviewer for the whole
campaign and name it in the campaign record.

**Nothing enforces this.** The reviewer is whatever `--model` was passed;
`review/review_meta.json` records `model_config` and `resolved_model` per
review, so a violation is visible after the fact and not before.

`runner.py` does enforce the weaker rule mechanically: it refuses to review a
run that already holds a review, because a reviewer whose `inspect` can reach an
earlier one is not independent of it. Renaming that review does not satisfy the
check — move it out of the run directory.

**A model reviewing its own report is not disqualified.** Reviewer quality
dominates the intra-model effect, and the citation and conformance checks run
before the reviewer and are file operations, so the half most exposed to
self-favour is decided by neither model. Choose the best available reviewer and
record which it was.

Current choice: `grok-4.6`. Grok's own runs therefore need a different reviewer,
and picking one is part of setting up any campaign in which grok is an arm.

## Q0 — does the run count at all

A run is **VOID** and re-run rather than scored if any of:

- **the answer key was opened** — `score.py` prints `MEASUREMENT INVALID — the
  trace shows answer_key was opened` and refuses to score. A run that read the
  answers measured nothing.
- **the review could not obtain its retest** — read off `review/summary.md`,
  where the exceptions are marked *not retested*. An exception whose standing is
  unknown is not one that did not stand. This is our infrastructure failing, not
  the model.
- **the instrument moved** between this run and the others in its set — compare
  `harness_rev` across the three `run_meta.json` files.

A void run is replaced. Qualification needs three valid runs, not three
attempts.

**Non-delivery is not on this list and must not be added to it.** A model that
did not produce the deliverable has returned a result; see Q1.

## Q1 — delivered, 3 of 3

**Every run must deliver every block METHOD §16 requires.** Read
`run_meta.json`: `error` is `no_deliverable: …` when the leg cap was reached
with blocks outstanding, and `blocks_delivered` names which arrived.

Assessed before admissibility, because **the citations of a report that does not
exist cannot be judged.** Until 2026-08-27 there was no such outcome: a run that
never produced a report went to the review anyway, the review had nothing it
could place, and the result was INADMISSIBLE — a delivery failure wearing a
citation failure's name, which disqualified GLM-5.3-Flash for a defect it does
not have.

**It is not a void.** Q0 voids *our* failures. A model told which block was
missing on every leg up to the cap, that still did not produce it, has returned
a result; replacing that run would discard the finding.

**One occurrence is enough**, because on the block instrument non-delivery means
the model failed to emit a named, copyable token after being asked for it by
name, repeatedly. A model that forgot once and delivered when told is recorded
in `blocks_prompted` and passes.

## Q2 — admissible, 3 of 3

**Every run must be ADMISSIBLE**, read off `review/summary.md`'s first result
line. One INADMISSIBLE run disqualifies the model for this workflow.

**THE JUSTIFICATION FOR ZERO TOLERANCE IS DISPROVEN. The threshold is left as
written because changing it is a decision, not a correction.**

It read: admissibility is the one review signal measured to hold still — three
precision legs on one report returned INADMISSIBLE three of three with the
mechanical layer byte-identical, against a supported ratio that moved 19/23,
21/23, 23/23 on identical text. But three legs on **one report** with **one
reviewer** is reviewer-side stability, and it was used to justify zero tolerance
across runs and across reviewers. Neither holds:

- **Run to run.** GLM-5.3-Flash returned ADMISSIBLE, INADMISSIBLE, ADMISSIBLE
  over three runs of one fixture.
- **Reviewer to reviewer.** grok-4.6 and GLM-5.3-Flash split on one report — 13
  of 47 evidence fields pointing nowhere, 27%, which REVIEW.md §4.0 calls either
  "a few fields among many" or "a substantial share" without fixing a boundary.
  grok listed thirteen `[uncited]` exceptions and proceeded; GLM stopped.

Two ways out when someone decides: soften to two of three, or read
`citations.json`'s `scheme` block rather than the reviewer's verdict — `scheme`
is a file operation and does hold still, which is the property this criterion
thought it was buying.

The ambiguity is **not** a defect in REVIEW.md, which says outright that the
practice decides. It is a defect here, in consuming that aid as a gate.

**What it gates is not quality.** A report whose citations cannot be placed
cannot be checked by anyone. Admissibility and quality are orthogonal:
`w1_qwen_1` cleared this gate and then failed on substance.

Qwen3.8-27B is ruled out on this criterion on every instrument it has run — two
of three inadmissible on b2, INADMISSIBLE again on b4 under three separate
reviews. That is a citation defect rather than a delivery one, so neither the
block instrument nor the METHOD amendment addresses it.

## Q3 — no standing exception, in at least 2 of 3

**At least two of the three runs must carry no standing exception**, read off
`review/summary.md`'s exception list.

An exception a second, uninformed reviewer did not confirm does not stand and
does not count here: a report two reviewers divided over is not a report that
failed. An exception marked *not retested* is Q0's business, not this
criterion's.

Not 3 of 3, and the reason is measured. One clean report reviewed five times on
2026-08-26 came back supported 12 of 12 four times and 11 of 12 once — the
dissent having rebutted a claim the audit never made. A per-review false-defect
rate near 20% puts a genuinely clean model at roughly a coin flip over three
runs under a 3-of-3 rule. That rule would measure the reviewer.

**Report every standing exception by disposition, not only how many runs were
clean.** What the exception was is the input to the next decision.

## Q4 — the mechanical criteria, read one at a time

`score.py` prints eight criteria and their conjunction: all three must-find
items, Gap Map produced, §9 recommendation used, leads with a top-3 finding, no
unsupported claims, §6 verdicts only, limitations statement, claim surface
closed.

**Gate: any single criterion failing in 2 or more of the three runs
disqualifies. The conjunction verdict is reported and does not gate.**

The distinction is the whole point. Eight criteria ANDed at 90% reliability each
pass 43% of the time regardless of which model is under test, so the THRESHOLD
line at n=3 mostly measures the conjunction. A *single* criterion failing twice
in three runs is a different quantity: at the same 90% that happens 2.8% of the
time by chance, so it is model behaviour.

Read against the m1 campaign this isolates the finding worth having — all three
of that campaign's failures were the claim surface, and no other criterion
failed more than once across nine runs.

**`score.py` computes the conjunction and not this rule.** The per-criterion
lines are printed per run; the 2-of-3 reading is done by hand across three
outputs.

## Reported, never gated

Measured, written into the campaign record, and disqualifying at no value:

- **report length** — no assurance standard bounds it, and a longer report
  covers more findings, so gating it penalised coverage. Removed 2026-08-24.
- **placement** — saturated; eight of nine m1 runs led with the key's top finding.
- **Tier 3 count and report thickness** — a score, not a floor. Tier 3 spans 2
  to 6 on one configuration.
- **`blocks_prompted`** — how often the runner had to name a missing block. New,
  no baseline, gates nothing until its variance is known.
- **subagent no-answer rate** — see the open question below.
- **wall clock**.

## The open question this document surfaced

Applying Q1–Q3 to the m1 numbers reproduces the recorded judgement for two
models and **not for the third**. Those runs were reviewed when the review still
returned a PASS/FAIL grade; read "PASS" as "no standing exception". Grok
qualifies (3 of 3, no criterion failing at all). Qwen does not (1 of 3, and out
on Q1 besides). **Luna qualifies under these criteria** — 2 of 3, its two
failing criteria each failing once — where the working judgement on 2026-08-25
was that it did not.

The signal that separates luna and is currently ungated is its **subagent
no-answer rate: 19%, 31%, 20%, against 0% for both other models across all six
of their runs.** Replicated, model-specific, bimodal rather than continuous, and
evidence lost in transit rather than a report-quality defect.

**Whether that becomes a fourth gate is Bruce's decision and is not frozen
here.** Any threshold between 0% and 19% is equally defensible on current
evidence, which is a reason to choose it deliberately.

The m1 numbers predate the review layer, so they carry Q2 and Q3 but say nothing
about Q1. They calibrate a rule; they are not evidence about those models on the
current instrument.

## What is deliberately not here

**No composite score.** A campaign result is a vector — admissibility, standing
exceptions, the per-criterion table, the reported columns — and a single number
would invite argument about the number instead of about the runs.
`measure/report.py` declines a composite for the same reason, and REVIEW.md
stopped returning a grade on 2026-08-29.

**No criterion the runner could enforce mid-run.** See
[workflow-concern-layers.md](workflow-concern-layers.md): the runner accepts
delivery, the review judges the work. Everything here is read after a run has
finished, on artifacts that exist.
