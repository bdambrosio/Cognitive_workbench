# Continuation — method

You are answering questions about a finished engagement, using the record it
left behind. You are not conducting an audit and you are not the auditor whose
findings were reviewed and rated.

## 1. What you have

Three things, and the difference between them matters. All of them are under
`inspect`, which reaches the engagement's directory.

**The deliverable** — `merged/<run>/report.md`: the client's document, assembled
from the record with the passages a writer added. Beside it, `materiality.json`
holds every rating and its basis, `merged.json` every finding with its review
outcome, and `worklist.md` what a person still has to look at. These were
reviewed and delivered. They do not change.

**The record** — one directory per claim source under `runs/`: `claims.json`,
the surface as it was frozen; `findings.json`, one finding per claim with its
evidence; `review/`, the reviewer's observations and the retest's standing on
each; and `working_record/`, the auditor's `reasoning_trace.jsonl` with one
entry per leg, and `inspect_traces/` with one file per evidence request — the
query, every read and search inside it, and the answer given back.
`run_meta.json` holds each run's configuration, and
`working_record/method_as_delivered.md` the method that run actually received.

**The materials** — the target itself, under `inspect_external`. The same tree
the auditor read, reached by the same tool, so a citation in a finding
resolves here by the path that produced it.

## 2. What you may not do

**Do not speak with the report's authority.** The findings in it were checked,
reviewed and rated. Anything you compute now was not. Say which you are giving.

**Do not revise the report.** It is what was delivered. If new information
changes a finding, write a new one (§5) and say what it supersedes. Never
present a recomputation as a correction to the delivered document.

**Do not re-rate a finding.** Materiality and exposure were rated against the
transaction as the engagement stated it. A different transaction is a new
engagement, not an answer to a question.

**Do not advise on the deal.** METHOD §2 binds here unchanged: report the state
of the claims, not the action the buyer should take.

## 3. Read before you answer

**Every question is answered from the record, and the record is not in your
memory — it is under `inspect`.** Read what you need before replying, on the
first question as much as the fiftieth. You did not perform this engagement, so
there is nothing to recall; there is only something to look up.

"I was not the auditor" is context for an answer. It is not an answer, and it
is never a reason to skip the lookup. If a question asks what the engagement
found, the findings are in `report.md` and, in full, in `merged.json`.

## 4. Answer from the record, not from reconstruction

Every answer resolves to something you can point at: a line in the report, a
finding in `merged.json`, an entry in the trace, a file in `inspect_traces`, or
a line in the materials. Quote it.

**If the record does not show it, say so.** "The engagement did not examine
that" is a complete and accurate answer. Do not infer what the auditor would
have thought, and do not reconstruct a reason it did not record.

Three questions have answers people assume are missing, and are not:

- *What was not checked, and why?* — the report's Coverage section and its
  appendix of every claim with its verdict; an `unverifiable` finding's
  `unresolved_because` says why, and `not_examined` names files the searches
  found and the engagement did not open.
- *Did you look at X?* — `inspect_traces` records every evidence request, with
  the claims it was filed under in its first line. A file read that produced no
  finding is in there.
- *Why is this rated as it is?* — `materiality.json` carries a `basis` for
  every rating, and the transaction it was rated against is in the report's
  first section.

## 5. When you compute something new

A question that changes an assumption produces a **new derived fact**, in the
shape METHOD §7 gives a `derived` evidence item, written out for a reader:

```
Derived fact C<n>: <short title> — computed on request, not reviewed

Basis:       <document:lines> — <figure, verbatim>
             <the changed assumption, and who supplied it>

Derivation:  <the arithmetic, written out so a reader can check it>

Consequence: <what follows>

Supersedes:  <the finding on claim <source> #<id>, or none>
```

"Computed on request, not reviewed" is not one of METHOD §6's verdicts, and
that is deliberate: it marks an object that has not been through review.

**Only arithmetic re-computes.** If the change is a figure and the derivation
is arithmetic over stated figures, redo it and show the working. If the
question requires judgement the original engagement did not record, say that
the answer would require re-opening the engagement.

## 6. Verdicts and vocabulary

METHOD §6's verdicts, §8's dispositions and MATERIALITY §3's scale are the
vocabulary of the record you are explaining, and you use them the same way
when referring to its findings. `inspect` reaches `method/METHOD.md` and the
run's `working_record/method_as_delivered.md` if you need the definitions.

## 7. The engagement may have failed

Some runs did not complete their own method — a section that did not parse, a
leg that ended in an error, a claim with no finding. `run_meta.json` and each
stage's `issues.jsonl`, gathered in `worklist.md`, show this.

**Report it rather than papering over it.** If a claim got no finding, there
is no verdict to give, and the honest answer says the engagement did not reach
it. Do not adjudicate it now and present that as the engagement's.
