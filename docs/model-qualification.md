# Model qualification — the frozen pass criteria

What a model must do before it is trusted with a delivered report. The
pre-screen ([model-prescreen.md](model-prescreen.md)) asks whether a model can
be *used* at all — structured output, tokens per call, one admissible fixture
run. This document asks whether its output can be *shipped*.

Frozen 2026-08-27, after three sessions in which the criteria were applied
without ever being written down.

**Frozen means the criteria are fixed before a campaign's first run and are not
changed during it.** A change to them invalidates comparison across the
boundary exactly as a change to the instrument does. Three campaigns were
discarded on 2026-08-24 for instrument drift; the same rule applies here, and
for the same reason — a bar that moves is not a bar.

## Which instrument these criteria apply to

**The instrument changed on 2026-08-27** — METHOD §16 went to four
self-delimiting blocks and the runner stopped reading a turn boundary as
delivery. Every run on the `b2` board predates that and is historical: `b2`
results and `b3` results are not comparable, by the same rule that discarded
three campaigns for drift on 2026-08-24.

Two consequences for what follows.

**A model disqualified on `b2` is not thereby disqualified.** GLM-5.3-Flash
failed Q1 on a run where no report was ever delivered, and the review layer —
the only thing that noticed — labelled that INADMISSIBLE. On the block
instrument the same behaviour is a named missing block and a prompt, so the
question it was disqualified on can no longer be asked the same way.

**`blocks_prompted` is new and has no baseline.** It counts how many times the
runner had to name a block that had not arrived. It gates nothing, and it will
not until its variance is known — three runs on one model is the minimum, and
`b3_grok_1` is the first.

## The unit of qualification

**Three valid runs of one model on the dataroom fixture, on one instrument
revision.** Where more than one model is under test, interleave by round.

Three is the floor and not a target. At n=1 the m1 campaign looked saturated —
three models, eight of eight criteria each — and at n=3 it separated them
cleanly. Two conclusions in this project have been retracted for treating a
single run as a result.

Record `git rev-parse HEAD` per run and keep the tree clean while a campaign is
running. A commit is not the edit: a file can change under a run, and after the
fact the boundary is unrecoverable.

## What a run produces

Three instruments, answering different questions. Only two of them exist at a
paying client, and the criteria below are built so that the gating ones are the
two that do.

| instrument | produces | needs the answer key | exists at a client |
|---|---|---|---|
| **mechanical** — `workflows/audit_review/runner.py` → `conformance.json`, `citations.json`, `scheme` | markers, closed vocabularies, claim surface, every cited line fetched, references past end-of-file, evidence fields pointing nowhere | no | **yes** |
| **review** — `workflows/audit_review`, a second model working to `REVIEW.md` | ADMISSIBLE / INADMISSIBLE, supported N of M, PASS / FAIL / INCONCLUSIVE | no | **yes** |
| **fixture scorer** — `measure/fixtures/dataroom/score.py` | Tier 1/2/3 recall against the key, placement, subagent no-answer rate, the eight-criterion threshold | yes | no |

## The reviewer is pinned, and it is never the model under test

The review verdict is a model's judgement. Varying the reviewer across a
campaign varies the instrument, so the reviewer is **one model held constant
for the whole campaign**, named in the campaign record.

It must not be the model being qualified. A model reviewing its own report is
not an independent review, which is the entire purpose of the layer —
`runner.py` already refuses a second review of one run for the weaker version
of this reason.

Current choice: `grok-4.6`. The consequence to plan for is that **grok's own
runs need a different reviewer**, and picking one is part of setting up any
campaign in which grok is an arm rather than the baseline.

## Q0 — does the run count at all

A run is **VOID** and is re-run rather than scored if any of:

- **the answer key was opened** — `score.py` returns MEASUREMENT INVALID. A run
  that read the answers measured nothing, and scoring it as a failure would
  misrepresent it as evidence.
- **the review returned INCONCLUSIVE** — a retest that could not be run is not
  a retest that agreed. This is our infrastructure failing, not the model.
- **the instrument moved** between this run and the others in its set.

A void run is replaced. Qualification needs three valid runs, not three
attempts.

## Q1 — admissible, 3 of 3

**Every run must be ADMISSIBLE. One INADMISSIBLE run disqualifies the model for
this workflow.**

Zero tolerance applies here and nowhere else, for two reasons.

**It is the one review signal measured to hold still.** Three precision legs on
one report returned INADMISSIBLE three times out of three with the mechanical
layer byte-identical; the baseline run put 0 of 35 references past end-of-file.
Against that, the supported ratio moved 19/23, 21/23 and 23/23 on identical
text. A gate can be absolute only where the signal is stable, and this is the
only place that is true.

**What it gates is not quality.** A report whose citations cannot be placed
cannot be checked by anyone — so what it found does not matter, and a supported
ratio computed over a misreading is worse than no ratio at all. Admissibility
and quality are orthogonal: `w1_qwen_1` cleared the gate and then failed the
review on substance, which is the demonstration that this criterion measures
something of its own.

Qwen3.8-27B is out on this criterion: two of its three runs on the current
method are inadmissible, and the ordinal-versus-line defect recurs at a 48%
rate.

## Q2 — review verdict, PASS in at least 2 of 3

**At least two of the three runs must return PASS.**

Not 3 of 3, and the reason is measured. REVIEW.md §9 fails a whole report on a
*single* surviving exception, and one clean report reviewed five times on
2026-08-26 came back supported 12 of 12 four times and 11 of 12 once — the
dissent having rebutted a claim the audit never made. A per-review false-FAIL
rate near 20% puts a genuinely clean model at roughly a coin flip over three
runs under a 3-of-3 rule. That rule would measure the reviewer.

The retest rule in §9 already applies before this criterion is read: a
`[unsupported]` or `[indeterminate]` that a second, uninformed reviewer does not
confirm is a fail that does not stand, and does not count toward FAIL.

**Report every surviving exception by verdict, not only the count of PASSes.**
Two clean runs and one FAIL is a qualifying result, and what the FAIL was is
the input to the next decision.

## Q3 — the mechanical criteria, read one at a time

`score.py` prints eight criteria and a conjunction verdict.

**Gate: any single criterion failing in 2 or more of the three runs
disqualifies. The conjunction verdict is reported and does not gate.**

The distinction is the whole point. Eight criteria ANDed at 90% reliability
each pass 43% of the time regardless of which model is under test, so the
THRESHOLD line at n=3 mostly measures the conjunction. A *single* criterion
failing twice in three runs is a different quantity: at the same 90% it happens
2.8% of the time by chance, so it is model behaviour.

Read against the m1 campaign, this is the criterion that isolates the finding
worth having — all three of that campaign's failures were the claim surface,
and no other criterion failed more than once across nine runs.

## Reported, never gated

These are measured and written into the campaign record, and no value of them
disqualifies a model:

- **report length** — no assurance standard bounds report length, and a longer
  report covers more findings, so gating it penalised coverage. Removed from
  the threshold 2026-08-24.
- **placement** — saturated. Eight of nine m1 runs led with the key's
  top-ranked finding.
- **Tier 3 count and report thickness** — a score, not a floor. Tier 3 spans 2
  to 6 on one configuration.
- **subagent no-answer rate** — see the open question below.
- **wall clock**.

## The open question this document surfaced

Applying Q1–Q3 to the m1 numbers reproduces the recorded judgement for two
models and **not for the third**. Grok qualifies (3/3 PASS, no criterion
failing at all). Qwen does not (1/3 PASS, and out on Q1 besides). **Luna
qualifies under these criteria** — 2 of 3 PASS, and its two failing criteria
each fail only once — where the working judgement on 2026-08-25 was that it did
not.

The signal that separates luna and is currently ungated is its **subagent
no-answer rate: 19%, 31%, 20%, against 0% for both other models across all six
of their runs.** That is replicated, model-specific, bimodal rather than
continuous, and it is evidence lost in transit rather than a report-quality
defect — the same shape as the four other transit failures on record.

**Whether that becomes a fourth gate is Bruce's decision and is not frozen
here.** It would be a new criterion rather than a transcription of an existing
one, and any threshold between 0% and 19% is equally defensible on the current
evidence, which is a reason to choose it deliberately rather than in passing.

Note also that the m1 numbers predate the review layer, so they carry Q2 and Q3
but say nothing about Q1. They are used above to calibrate a rule, not as
evidence about those models on the current instrument.

## What is deliberately not here

**No composite score.** The result of a campaign is a vector — admissibility,
PASS count, the per-criterion table, the reported columns — and a single number
would invite argument about the number instead of about the runs. REVIEW.md §9
declines a severity grade for the same reason.

**No criterion the runner could enforce mid-run.** See
[workflow-concern-layers.md](workflow-concern-layers.md): the runner accepts
delivery, the review judges the work. Everything in this document is read after
a run has finished, on artifacts that exist.
