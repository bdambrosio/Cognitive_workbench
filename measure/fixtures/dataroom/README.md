# dataroom — recovered corpus, PARKED

**Not the venture benchmark, and not a candidate for it.** Handing an agent
this corpus pre-decides the business idea, which replaces sub-problem
identification with document analysis — a narrower capability, and not the one
under study. Bruce ruled it out on those grounds 2026-08-21; the extended-
reasoning bench is `bench/venture/`, which keeps the brief unbounded.

Kept because it cost four hours of agent time to produce and is a usable
answer key for a *different* instrument, should one ever be wanted: a
document-analysis probe with planted defects at graded discoverability. Nothing
here is wired to anything.

A synthetic acquisition data room with known defects, for measuring the class of
task the Jill/Jack exchange of 2026-08-20 represents: sub-problem
identification, subplan generation, data gathering, cross-document analysis,
and synthesis into a deliverable.

**Not runnable, by decision.** Corpus and answer key only — no scenario, no
runner, no grader, and none planned.

## What is here

    corpus/          nine documents, ~11.3k chars, the material under analysis
    answer_key.md    what is planted, where the evidence is, severity order
                     — keep out of any agent's reach

## Provenance

Recovered 2026-08-21 from `scenarios/jill_chat.bak`, trace rows 2411–2634 —
the artifact Jill and Jack built during the 130-turn unattended exchange. It
was authored as a *format* test, so one leak had to be removed (Document 9's
"Omissions Note" listed the answers); see the key.

## Why this corpus is worth keeping

The defects form a difficulty ladder rather than a flat list:

- **stated** — a fact sitting in one document (backup failures in Doc 4)
- **cross-document** — a seller claim contradicted by a config or contract
  (failover, test coverage, uptime monitoring, DataEnrich's absence)
- **derived** — true only after arithmetic on stated figures (60% of revenue
  outside Stripe; the last good backup expiring under 30-day retention)

The third tier is the interesting one: cross-document contradiction and
arithmetic-on-evidence were what the two agents actually did well in the
original run.

## Deliverable contract (the termination half)

Bruce's point, and it is the right one: this class of task does have a
well-understood terminal product — a due-diligence memo, a one-pager, a
five-pager. A bounded deliverable supplies what the original task lacked:

- a stopping rule the agent can check for itself, and
- a forced choice about what earns space, which is itself the measurement.

Note what the original run demonstrates, though: Jill and Jack *had* a document
structure (three parts, nine sections, four deliverables) and still did not
stop. A section list alone is not a stopping rule — each section needs a
satisfaction condition, and there must be a rule against re-revising a section
already accepted. In the trace Jack had reached that rule unilaterally ("I'm
not re-running it a fourth time") while the exchange continued regardless.

## Suggested scoring

Recall against Tier 1 and 2; Tier 3 credited, never penalised; false positives
counted only for claims the corpus does not support. For a length-bounded
deliverable, score placement against the key's severity ordering, not mere
presence.

Per-finding expectations also make a loss funnel possible — retrievable →
retrieved → read → noticed → recorded → survived → reported — which localises a
miss to a subsystem instead of reporting one number.
