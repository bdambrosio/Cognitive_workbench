# Multi-run merge, and weeding by review

Idea recorded 2026-08-27. **Not built, not designed in detail, not scheduled.**
Written down because it may be a better use of effort than the thing it would
replace.

## The thing it would replace

Chasing single-run reliability on open-weights models. Three days of campaigns
say that is unrewarding: every measured property of a qualified model moves run
to run. On one fixture, one model, three runs of identical material —

| | run 1 | run 2 | run 3 |
|---|---|---|---|
| claims enumerated | 20 | 25 | 22 |
| line references in the report | 37 | 0 | 44 |
| harness prompts needed | 1 | 0 | 2 |

— and it still passed review three times out of three. A second model
enumerated 47 findings and lost 15 of them to checking. A third enumerated 51
claims where the first found 20.

The instinct is to find a model that does not do this, or to tune one until it
stops. That is tail-chasing: the variance is not obviously a defect to be fixed,
and the models that have it are otherwise producing admissible, passing work.

## The idea

Run the same engagement N times. Merge. Weed.

**Merge the claim surface by union.** A claim either appears in the named claim
sources or it does not, so the union across runs is a strictly better estimate
of the surface than any single run's, and the combining rule is principled
rather than a heuristic. `measure/fixtures/dataroom/overlap.py` already compares
surfaces **by citation rather than by count**, which is the primitive this
needs.

**Merge the findings, then weed them by review.** An audit report is not prose;
it is a set of findings, each carrying its own citation, each independently
checkable. That is the property that makes union-then-filter sound here and
unsound for most generated text. The review layer already issues per-finding
verdicts — `[supported]`, `[unsupported]`, `[uncited]`, `[overstated]` — and
already retests a fail with a second, uninformed reviewer. Weeding is the
existing mechanism pointed at a larger pile.

**Pre-merge or post-merge weeding are different designs.** Weeding each run
before merging keeps reviews small and independent; weeding after merging lets
the reviewer see that three runs found the same thing, which is evidence a
single review cannot have. Corroboration across independent runs is probably the
more interesting signal, and it is the one that argues for post-merge.

## Why it might be better than the alternative

**It plays to what the models are good at and against what they are bad at.**
Per-run behaviour is high-recall and low-precision: qwen produced 47 findings
and 32 held. Union raises recall further; review raises precision. Neither
operation asks a model to be more consistent than it is.

**It supplies a stopping rule and a coverage estimate, which a single run
structurally cannot.** Marginal yield is measurable: if run N+1 adds no claims
the surface has saturated, and the saturation curve is evidence about coverage
rather than an assertion of it. The largest open problem in the method — that
every coverage figure divides by a denominator nobody can validate — becomes an
empirical question instead of an act of faith.

**It decouples the product from model choice.** If merge works, the requirement
on a model drops from *reliably produces a complete audit* to *reliably
delivers, and contributes findings that survive checking*. That is a much
weaker requirement and a much larger candidate pool.

## What would have to be settled first

**Deduplication is the hard part. Dedup on SOURCE, not on finding text.**
(Bruce, 2026-08-27.) Two runs will word the same finding differently, but the
evidence quotes are not free text — they are substrings of a corpus we hold, so
two findings quoting the same passage are pointing at the same evidence, and
that is a file fact rather than a judgement. This is not keyword matching: it
measures co-reference to shared source text, not classification from a word
list.

**Compare resolved positions, not quote strings.**
`audit_review/runner.py:resolve_quotes` already locates every quote in a
document — it has to, to decide `contiguous` against `split` — but
`citations.json` stores only the quote text, the document and segment counts.
No offsets. Recording start/end offsets is a small change and it turns
"percentage overlap between two strings" into "overlap between two intervals in
one document", which is exact. Paraphrase, truncation and reordering vanish once
both sides are located, and the only threshold left is how much interval overlap
counts as the same evidence.

**Source identity is not finding identity, so use it as a BLOCKING KEY.** In the
current corpus doc4's Backups section supports at least two distinct findings —
"failures recorded for the last 21 days" and "no alerting configured for backup
failures". Overlapping quotes, different findings; merging them loses one. The
converse also occurs: two runs supporting the same gap from different evidence
read as distinct. Overlap should therefore generate candidate pairs cheaply and
mechanically, leaving a small, well-posed same-or-different question with both
quotes in hand — not decide the merge outright.

**Tune toward under-merging.** False-distinct is cheap: redundancy the reader
can see. False-merge is expensive: a finding silently disappears.

**Near-exact where it matters most.** For the claim surface the unit of merge
*is* a source assertion, so interval overlap approaches identity — and that is
where the payoff is largest, because it is the denominator problem. Mechanical
union for the surface; blocking key plus a judgement step for findings.

**Unciteable findings drop out, and that must be explicit.** A finding with no
resolvable quote cannot be deduped on source at all — one run this week had 20
of 47 evidence fields pointing nowhere and another had 9 `[uncited]` findings.
Requiring a resolvable quote before merge is defensible, since an unciteable
finding cannot be verified either, but it must be a stated rule with a reported
count rather than a silent filter.

**Calibration is real work.** `resolve_quotes` rejected a similarity threshold
on the grounds that it "would put a judgement in the layer that exists to keep
judgement out, and would need calibrating against failures we do not have". The
first half does not apply — this is a different layer answering a different
question — but the second half does. A threshold needs hand-labelled
same/different pairs, and the runs already on disk are the material for that
set.

`measure/fixtures/dataroom/overlap.py` compares surfaces by citation rather than
by count and is the existing primitive to build on. Citation alone will not
carry it: citation forms vary between runs of the same model — 37, 0 and 44
line references across three runs.

**Cost multiplies.** N audits plus N reviews plus a merge step. On the fixture
that is cents; on a real engagement it is the wall clock that matters, and the
audits are independent so they parallelise.

**It interacts with the measurement problem.** A merged report is a better
product and a worse instrument: it tells you less about any individual model
than a single run does. If merge ships, model qualification needs to keep
running single-run, or the fixture stops discriminating.

**The client receives one report.** That is unchanged and unproblematic — a
practice that has three people read a data room delivers one set of findings.
But the working record should show that it was assembled from N runs, because
§14's whole posture is that the record shows what was actually done.

## Where this does not help

Delivery. A model that fails to produce the deliverable contributes nothing to a
merge, so the per-run delivery contract still has to hold. Merge improves
recall and precision of findings; it does nothing for a model that will not
finish.
