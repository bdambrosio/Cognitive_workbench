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

**Deduplication is the hard part and it must not be keyword matching.** Two runs
will state the same finding in different words. Matching by citation is the
obvious first cut — same document, same lines, same claim — and `overlap.py`
does exactly that, but citation forms vary between runs of the same model (37,
0 and 44 line references across three runs), so citation alone will not carry
it. Anything beyond that is a meaning-based match and belongs to an LLM call or
an embedding, per the project rule.

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
