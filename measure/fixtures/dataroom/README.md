# dataroom — the fixed-task fixture

A synthetic acquisition data room with planted defects at graded
discoverability, plus a bounded deliverable. **This is the fixture for
answering "did my harness change help?"**

## Why a fixture and not the live trace

Retrospective analysis of jill_chat cannot attribute a behaviour change to a
harness change. Its two largest step changes over three months — median
iterations per turn 4.0 → 2.0, yield rate 42.6% → 1.9% — turned out to be
Factorio being shelved, not any commit. In an experimental workbench the task
never holds still.

A fixture holds it still. Same nine documents, same brief, same deliverable
contract, run before a change and after, with the harness as the only thing
varying.

## Running

```bash
# fresh world every trial — bump world_name in the scenario first
python3 launcher.py dataroom.yaml --cli --autonomy

# then, mechanical metrics only, free:
python3 measure/fixtures/dataroom/score.py --world dataroom_1 --dry-run
# with finding-matching (one pinned LLM call):
python3 measure/fixtures/dataroom/score.py --world dataroom_1
```

Give `TASK.md` **verbatim** as the opening turn. Do not paraphrase it and do
not answer questions afterwards — the brief says no user is available, and
answering makes the run incomparable with every other run.

`scenarios/dataroom.yaml` is `venture.yaml` with a different `world_name` and
header, deliberately: matched line for line so the two fixtures differ in the
task and nothing else.

## What is scored

A vector, never a single number.

**Mechanical, no model:** turns, iterations, which of the nine corpus
documents were actually opened, memo word count against the 900 ceiling,
and whether the answer key was read.

**One pinned LLM call:** matching the memo's findings onto key ids. This is
a judgement about meaning, so it cannot be mechanical — but it runs on the
same pinned grader `regrade.py` uses, held constant across arms. If the arm
under test did its own matching, a difference in matching quality would
surface as the metric.

## The difficulty ladder — why this fixture should not saturate

Two different axes, and conflating them was the first bug in `score.py`.

**Provenance of the finding** (the answer key's grouping):

| tier | what | scoring |
|---|---|---|
| 1 | planted, must-find (P1–P3) | recall |
| 2 | planted, derived (F1–F2) | recall |
| 3 | supported but unplanted (B1–B7) | **credit, never penalise** |

Tier 3 is never penalised because entries 4–5 of the original exchange were
discovered rather than seeded. An agent finding something real that nobody
planted is being correct, and a grader counting that as a false positive is
measuring the wrong thing. Only claims the corpus does not support count
against it.

**Difficulty of noticing it** (the anti-saturation axis):

| rung | what |
|---|---|
| `stated` | a fact sitting in one document |
| `cross_document` | a seller claim contradicted by another document |
| `derived` | true only after arithmetic on stated figures |

Report the highest rung reached, not pass/fail. **Retire the fixture when
every arm tops out on `derived`** — a probe everyone passes is not an
instrument, and that is precisely how the retired `bench/` died with 29 of
34 rows at 1.0.

## Placement is a measurement, not a formatting rule

The memo is capped at 900 words and the corpus supports twelve findings. The
cap forces a choice about what earns space, and that choice is the thing
worth measuring. The key carries a severity ordering; the scorer reports the
memo's order against it. Score whether the top-ranked findings made the page,
not merely whether they were found somewhere.

The stopping rule and the no-reopen clause in `TASK.md` exist because of the
2026-08-20 runaway: Jill and Jack *had* a document structure — three parts,
nine sections, four deliverables — and still produced "Revised Part 2 is in
hand. Different document from the one I reviewed last turn." A heading is not
a finish line. Each section needs a satisfaction condition and a rule against
re-revising one already accepted.

## The answer key is inside the agent's reach

`inspect` is geofenced to the repo root (`176b4029`), so `answer_key.md` is
readable by the agent under test. Rather than hide it, `score.py` looks for
it in the trace and returns **MEASUREMENT INVALID** if it was opened.

That is a third outcome, not a zero. A run that read the answers measured
nothing, and reporting it as a bad score would misrepresent it as evidence.

## Provenance of the corpus

Recovered 2026-08-21 from `scenarios/jill_chat.bak`, trace rows 2411–2634 —
the artifact Jill and Jack built during the 130-turn unattended exchange. It
was authored as a *format* test ("does the report template handle three
finding types"), not a discovery test, with two consequences:

1. **A leak was removed.** Document 9 originally ended with an "Omissions
   Note" enumerating the planted answers. It is preserved at the bottom of
   `answer_key.md` and stripped from `corpus/`. If you regenerate the corpus
   from the trace, strip it again.
2. **The corpus supports more findings than were planted** — hence Tier 3.

Nine documents, ~11.3k characters. Company `flowmetrics`, seller `dave`.

## Before trusting a number from this

Run-to-run variance under fixed conditions is **still unmeasured** for this
agent. Three runs per arm minimum. Two conclusions in this project have
already been retracted for treating a single run as a result.
