# Derived findings, and what `[uncited]` actually catches

Understanding recorded 2026-08-27. **No instrument change is proposed and none
should be made on the strength of this note.** The last section says what would
have to be observed first.

## The observation that started it

`b3_qwen_1` Finding 14, verdict `[real]`:

```
Claim (doc1:L9): "revenue-positive"
Evidence: MRR $40k. Costs: Heroku $25/mo, Redis $50/mo, DataEnrich $400/mo,
          Twilio $100/mo, GoDaddy $12/yr. Total visible costs ~$587/mo.
Gap: None. Revenue $40k/mo exceeds costs.
```

The review returned `[uncited]`, and that verdict was the standing fail that
failed the report. The figures are real and almost certainly correct — the cost
lines are in doc8, the external dependency list. The finding is good. It simply
never says where anything came from, so a reader has nothing to follow.

It reads as complete on the page, which is the whole problem. Five concrete
numbers and an arithmetic conclusion do not look like a defective citation.

## This is not a gap in the method

§5's second shape exists for precisely this finding:

```
**Finding N: <short title> — [derived]**

Basis: <document:lines> — <the first stated figure, verbatim>
       <document:lines> — <the second, verbatim>

Derivation: <the arithmetic, written out so a reader can check it>

Consequence: <what follows, and why a buyer cares>

Escalates: <Finding N, or None>
```

**A composite finding's evidence is not uncitable. It is multi-cited.** The
inputs each have a locator; the thing that spans documents — the arithmetic —
is the auditor's own work and needs no citation, only to be written out. §5
rule 2 says so directly: *"Write the arithmetic out. Arithmetic left implicit is
an opinion with a citation attached."*

So Finding 14 is a derived finding written in the simple shape. Had it used the
shape the method already provides, it would have passed and the report would not
have failed. **The report was rejected for a conformance failure, not for the
finding being good.**

## What `[uncited]` gets right, and what it reports badly

Right: the burden is on the report to be checkable, not on the reviewer to make
it so. The reviewer could have found the cost lines in doc8 — they are in the
materials it holds — and §2 and §10 forbid it, because a review that
reconstructs missing citations grades reports on whether the reviewer happened
to locate the evidence, stops being reproducible, and removes any incentive to
cite. `[uncited]` is also not retested, correctly: a field carries a pointer or
it does not, and asking a second model to re-derive a fact is not a second
opinion.

Badly: `[uncited]` is accurate about the artifact and silent about the cause.
*"You used the simple shape for a derived finding"* is the fixable statement,
and nothing in the review's vocabulary can say it.

## The one genuine gap, stated narrowly

A derivation is the auditor's own construction. Each Basis line resolves by file
operation; the inference from those lines to the consequence does not. Writing
the arithmetic out makes it **reader-checkable**, which is the honest ceiling —
but the review has no verdict for that state. A correctly-cited derived finding
would come back `[supported]`, and the reviewer would have verified the *inputs*
rather than the *inference*.

If that is ever worth fixing, the shape is:

- a verdict distinct from `[supported]` meaning **basis resolves, derivation is
  the auditor's reasoning** — reported, and not fatal;
- **earned by a fully cited Basis, never available as an alternative to
  citing**;
- retest extended to adjudicate the derivation, reusing the existing mechanism
  where a second, uninformed reviewer checks a finding without seeing the first
  verdict.

The second bullet is the load-bearing one. A marker that means *"not mechanically
verifiable"* is what a model reaches for whenever citing is inconvenient. The
precedent is on record: when §5 was relaxed to document-plus-quote on
2026-08-26, one report came back with 10 of 33 evidence fields carrying nothing
searchable, against 1 of 45 before, and the change was reverted the same day
(`afd5b2bd`). Gate the marker on a fully cited Basis and it cannot become that
escape hatch.

## Why nothing is being built

**The failure has been inferred, not observed.** No properly-shaped `[derived]`
finding has yet gone through review, so nobody has seen the over-crediting this
note describes. Every instrument change in this project that was justified by
reasoning rather than by a run has cost more than it returned, `afd5b2bd` most
recently.

**The run that produced the observation was a model that does not qualify.**
Qwen3.8-27B is disqualified on Q3 for reasons that include this one. A defect
seen only in a rejected model's output is not yet evidence about the instrument.

**The trigger that would justify acting:** a report from a qualifying model
containing a derived finding with every Basis line cited, whose derivation is
wrong, returning `[supported]`. That is one observation, it is falsifiable, and
until it exists this note is the whole of the change.
