# dataroom — campaign state

Started 2026-08-23, on the workflow harness. Everything before it was
discarded, deliberately — see "Why every earlier run was discarded".

## Scored runs — campaign m1, 2026-08-25

Nine runs, three per model, interleaved by round. Instrument frozen at
`bc4ae148` (METHOD.md, run.py, audit.yaml); scorer at `3b1d6e07`. Every run
completed with `error=None`. No retries were needed.

| run | T1 | T2 | T3 | unsup | findings | verdicts | surface | subagent no-ans | leads | threshold |
|---|---|---|---|---|---|---|---|---|---|---|
| grok 1 | 3/3 | 2/2 | 4 | 0 | 12 | 6 | 32 | 0% | F2 | PASS |
| grok 2 | 3/3 | 2/2 | 5 | 0 | 16 | 7 | 31 | 0% | P2 | PASS |
| grok 3 | 3/3 | 2/2 | 5 | 0 | 12 | 6 | 34 | 0% | P2 | PASS |
| qwen 1 | 3/3 | 1/2 | 5 | 0 | 29 | 6 | 40 | 0% | P2 | PASS |
| qwen 2 | 3/3 | 1/2 | 5 | 0 | 15 | 3 | not closed | 0% | P2 | **FAIL** |
| qwen 3 | 3/3 | 1/2 | 4 | 1 | 16 | 6 | not closed | 0% | P2 | **FAIL** |
| luna 1 | 3/3 | 1/2 | 3 | 0 | 8 | 3 | 62 | 19% | P2 | PASS |
| luna 2 | 2/3 | 2/2 | 4 | 0 | 11 | 4 | not closed | 31% | P2 | **FAIL** |
| luna 3 | 3/3 | 1/2 | 3 | 0 | 8 | 3 | 90 | 20% | P2 | PASS |

**grok 3/3 PASS · qwen 1/3 · luna 2/3.**

### The threshold does discriminate. Yesterday's saturation was an n=1 artifact

On 2026-08-24 all three models passed all eight criteria once each, and this file
recorded that the threshold had stopped discriminating. At n=3 it separates the
models cleanly. One run per model was not enough to see it, which is the reason the
scorer prints "n=1 is anecdote" after every run.

### Every failure is the claim surface, and the marker is simply absent

All three FAILs are the same criterion, and no other criterion failed more than
once across nine runs. In all three the marker appears **nowhere** in the trace
— not truncated, not echoed from a tool, not present without a count. The model
did not enumerate.

**Both of qwen's failures are single-leg runs**, and its one two-leg run closed
the surface. An model that compresses the whole engagement into one turn appears
to skip enumeration and go straight to verification. Two observations is not a
finding, and luna 2 is a counterexample at three legs, so leg count is not the
whole mechanism.

This is now the most interesting open question in the suite. It is also the
criterion that took the most work to build, and it is the only one earning its
place in the threshold.

### What replicated

- **luna's subagent no-answer rate: 19%, 31%, 20%.** Predicted 25-36%. Real,
  model-specific, and slightly below the band. Both other models are 0% across all
  six of their runs.
- **grok's stability.** Tier 2 2/2 on all three runs, claim surface 31/32/34,
  zero subagent failures, threshold PASS every time. No other model is stable on
  any of those axes.
- **Report thickness tracks the model.** luna 8, 11, 8 findings with 3-4 distinct
  verdicts; grok 12-16 with 6-7; qwen 15-29 with 3-6.
- **Placement is saturated.** Eight of nine runs lead with P2, the key's
  top-ranked finding. This column no longer discriminates and should be
  reported, not gated.

### One genuine miss

luna 2 is the only run in nine to drop a must-find item (2/3), and one of only
two to register anything as `unsupported`.

### Trust the mechanical columns

Tier 2 and Tier 3 are grader-sampled, and Tier 2 was observed flipping on
identical text earlier the same day. The claim-surface, subagent and
verdict-conformance columns are read straight off the trace and are the ones to
argue from.

### The retry estimate was pessimistic

This file predicted roughly one run in four would need a retry, from six
enumeration runs that produced two failures. Nine scored runs needed none. The
failures were in the claims-only harness and against repository-sized targets,
not the fixture. Keep the warning for real targets; drop it for fixture runs.

## Instrument drift — the check that has to run before every campaign

The check, kept. What it concluded about specific runs is gone with them.

**Join run timestamps to commit timestamps before comparing anything.**
Established 2026-08-24, when a table being read as one experiment turned out
to span three instruments. Four commits landed *between* runs inside 30
hours, each changing agent behaviour:

| UTC | commit | change |
|---|---|---|
| 08-23 21:48 | `544858ce` | METHOD.md §12 **6b** — verify every claim against its citations before shipping |
| 08-23 23:20 | `aae2f5e7` | `top_p` default **1.0 -> 0.95** on every non-Anthropic call |
| 08-24 01:25-01:38 | `ef2e4fa8` `1240b460` `42dc3fe5` | METHOD.md +24 lines: an absence needs the search, not the conclusion |
| 08-24 02:14 | `a6c5c466` | subagent no longer discards a found answer — `inspect_external` IS a subagent |

Two traps found doing it, both worth keeping:

- **Timezone.** `git log --date=format-local` renders in local time; the
  result directories are UTC. Comparing them directly inverted the ordering
  and made a post-run commit look like a pre-run one. Use `TZ=UTC`.
- **A commit is not the edit.** `luna_t025_3` started before `544858ce` and
  finished after it. Commits follow edits, so the file may change under a
  run and the boundary is unrecoverable after the fact. Prefer a clean tree
  and a recorded `git rev-parse HEAD` per run over reconstructing it later.

**Comparisons within one instrument are clean; across instruments they are
not.** That rule discarded the pre-workflow campaign, then the campaign that
replaced it. The next one runs on a frozen instrument or it will discard too.

## The §9 column was broken — fixed 2026-08-24

`recommendation_of()` matched four hardcoded spellings of the label and
returned None for anything else. Hand-checked against all 19 reports in the
campaign: **every one stated a valid §9 term**, and the check missed four —
every `—` that was ever in the column. It had never once discriminated
between models; it only ever reported label formatting.

Those reports are deleted. The fix is not evidence about models, it is a
repair to the instrument, and it is covered by unit tests that do not depend
on them. The four spellings below are kept because they are the lesson.

| run | wrote | why the old matcher missed it |
|---|---|---|
| run A | `**Section 1: RECOMMENDATION**` / `**Verdict: Material.**` | caps, and `Verdict:` not `Recommendation:` |
| run B | `### Recommendation: **Material**` | `**` between the colon and the term |
| run C | `## 1. Recommendation` then `**Material.**` | term on the next line |
| run D | `## 1. Recommendation: **Material**` | `**` between the colon and the term |

**The closed vocabulary was never the bug, and is kept.** §9 defines five
literal values and the method requires one verbatim, so testing for them is
conformance, not classification of free text — the distinction the existing
comment already drew, and it was right. Replacing it with an LLM call was
considered and rejected for a stronger reason than that: this gates a
PASS/FAIL, and the campaign has already watched grader noise flip
`unsupported` between 0 and 1 over identical text. Putting a sampled model
behind a second gate would add noise to the threshold, not remove it.

Only the LOCATOR changed: find each mention of "recommendation", then look
for a vocabulary term standing as a verdict in the 120 characters after it —
earliest match wins, longest term breaks the tie so "Clear with caveats" is
never read as "Clear".

**Two bugs in the fix itself, both found by running it, not reading it.**
The first guard accepted `Recommendation: Clear the backup failures` as a
verdict of `Clear` — `Clear` and `Walk` are ordinary English verbs. The
second, tightened to require the clause to END on the term, used `\s*` and
no `re.M`, so the newline was swallowed before `$` could match end-of-line:
that turned **13 correct rows into false negatives**. The working form is
horizontal whitespace only, plus `re.M`. Re-scoring all 19 after each attempt
is what caught both; neither was visible in the diff.

**No verdicts flipped** when the fix was applied: all four corrected runs
failed another criterion anyway, so the bug had never changed a verdict — only
made the §9 column look informative when it was not. Those runs are now
deleted; the four spellings survive as the shape of the bug.

**The 19 rows were not re-graded.** §9 is mechanical and was recomputed
offline for free; the tier counts come from a sampled grader, and re-running
it over historical rows would have shifted them by grader noise while
correcting nothing. The record keeps the numbers it was scored with.

## The preamble boundary — decided 2026-08-24

**The deliverable is the whole final turn.** Prose before the report counts
as report: against the word budget, and against the ordering.

Decided this way because the fixture already worked this way in the half
nobody questioned — `run.py` writes everything before `=== GAP MAP ===` into
`report.md`, so a leg-opening preamble was already inside the word count.
Scoring placement on a narrower span than word count would have made one
deliverable two different documents depending on which criterion was asking.

The alternative was a `=== REPORT ===` marker mirroring the Gap Map's. It was
rejected as machinery bought to spare the agent one explicit act of
restraint: a new marker is a new compliance surface and a new way to fail,
where a sentence in §16 costs nothing and says what was always meant.

§16 now carries that sentence, which changes the instrument — every run from
here is on the post-2026-08-24 method.

## Pick models by output tokens per call, not parameter count

Kept as a selection rule; the model that produced it is dropped.

Qwen3.8-2.4T-A95B spent 2,902 completion tokens answering a one-line question
against a 6k-character prompt, where DeepSeek used 18. At ~21 tok/s that is
140 seconds per call, and this architecture makes 35-187 calls per audit.
Neither throttling nor cache — both were tested. **A model that cannot answer
briefly cannot be used in a call-heavy agent, whatever its parameter count.**
Measure tokens-per-call on a trivial prompt before committing an model to a
campaign; it costs one call and it is the cheapest disqualifier available.

## The threshold, and why it is the number that matters

Tier counts are a score; a business case needs a floor. `score.py` reports
PASS/FAIL against six criteria a delivered report would be judged on: all
three must-find items, a Gap Map, a §9 recommendation, leading with a top-3
finding, no unsupported claims, and the word ceiling.

**The arithmetic that makes this hard to read.** Six criteria ANDed: even at
90% reliability each, the conjunction passes 53% of the time. A ~50% pass
rate is therefore roughly what six good-but-imperfect gates produce
*regardless of the model*, so pass rate alone is close to uninformative at
small n. Report the per-criterion failures, not just the verdict.

## The grader was validated, and swapped

> The three hand-checked judgements below were made against the answer
> key, which still exists, but the reports they were made on are deleted.
> The conclusion — terra, not luna — stands and is configured. The noise
> figures are indicative only and would need re-measuring.

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

> Architecture stands — the method does load verbatim into the static
> system prompt. The claim that three contract elements reached an output
> "for the first time" rested on runs now deleted, and is unverified
> until the next campaign reproduces it.

The method now loads verbatim into the static system prompt
(`workflows/claims_audit/scenario.yaml` -> `workflow: workflows/claims_audit/method/METHOD.md`) instead of being
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

## Why every earlier run was discarded

They ran against a different instrument, and the differences are not
adjustments — they change what the agent is given and what counts as
finished.

| change | effect |
|---|---|
| method in the static prompt | §3-§16 reach the agent at all; before, the surviving fragment ended mid-sentence in §2 |
| iteration cap 12 -> 16 | an model reading one document per call can finish ingesting and still have budget to work |
| `inspect` geofenced to `audit/` | the answer key and prior models' reports are out of reach |
| empty-answer sentinel fixed | a subagent returning nothing says so, instead of reporting `OK` |
| Gap Map marker as the stopping rule | a turn that stops with a stated plan is no longer scored as a finished audit |
| engagement ledger on each `continue` | legs, minutes and documents-opened are carried by the runner |

Comparing across that line would be comparing two instruments. Those runs did
their job — every fix above was found by running them — and they remain in
git history if a specific claim ever needs checking.

## Campaign m1 is closed. A new stage starts here — 2026-08-25

**The base campaign is done.** Nine runs, three models, one frozen instrument.
grok 3/3 PASS, qwen 1/3, luna 2/3, and the working judgement is that grok
clears the first gate for production use and the other two do not. Two full
audits against real targets — ChatterMate and Body — completed cleanly on the
same instrument. Results at the top of this file.

Nothing further extends that campaign. Work from here is in three streams, and
none of them is "one more run of the same thing":

1. **Continuation.** Answering questions about a finished engagement from the
   record it left. Built at `workflows/claims_audit/continuation.py`, `workflows/claims_audit/method/CONTINUATION.md`,
   `workflows/claims_audit/continuation.yaml`. One capability verified live — recomputing a
   derived finding under a changed assumption, with the authority boundary
   holding unprompted. Three untested: retrieval of a claim that held and never
   reached the report, being argued out of a finding, and behaviour on a run
   that never closed its claim surface.
2. **New models and new fixtures.** Other models against this data room, and new
   client data rooms against this method.
3. **New methods.** The audit is one workflow. `audit.yaml` binds a method file
   through `workflow:`, so a second workflow is a second method document and a
   second scenario, not a second harness.

**What that changes about the instrument.** METHOD.md has been treated as *the*
method. It is now one of several, and the machinery around it — the workflow
loader, the practice-audience split, the claim-source declaration, the working
record — is the part that generalises. Freezing was the right discipline for a
single campaign; from here the question is which pieces are shared and which
belong to one workflow.

### QUEUED, and it must land before any further run

**One deliverable per leg.** The report in one leg, the Gap Map in the next.
Turn boundaries are structure the harness already maintains, and no model has
to emit them correctly.

This is a Tier 1 change — it alters what the agent does, so runs before and
after are not comparable. That is acceptable now and would not have been
yesterday.

What it touches:

- **METHOD §16** currently reads "Two documents, produced together in the final
  turn... The final turn carries these two documents and nothing else," with a
  rationale about anything written before the report becoming the report. The
  rationale is still right and needs rewriting for two legs, not deleting.
- **`run.py`'s stopping rule** is `GAP_MARK not in reply -> continue`. With the
  Gap Map in its own leg there is no marker to stop on, and this is the hard
  part of the change: the runner needs a completion signal that is not a string
  in prose. Decide it before writing anything.
- **The brief** says "Produce both deliverables together in your final reply."
- **`score.py:split_deliverables`** becomes unnecessary for runs made after the
  change, and must keep working for the ones already on disk.
- **The runner should stop judging completeness.** `GAP_MARK not in reply` is
  there because "done is a deliverable, not an exit reason" — an model once wrote
  "I will now begin working the priority order", ended the turn, and scored as
  complete. That check belongs in the scorer, which already has the criteria: a
  241-word report with zero findings fails `_FINDING_RE`, fails Tier 1, fails
  the threshold. The runner drives a fixed protocol and accepts what comes
  back; the scorer judges. Truncation has a structural signal already —
  `backend.last_finish_reason` — and needs no sentinel in the text, though note
  `backend.py:456` leaves it None on the legacy cloud route.

**Keep `=== GAP MAP ===` as a fallback rather than deleting it.** An model will
sometimes produce both documents in one leg regardless of instruction, and a
run that does should be salvageable rather than lost.

**What this does not fix**, and the reason it is a structural improvement
rather than a marker fix: `=== LIMITATIONS ===` is a section inside the report
and `=== CLAIM SURFACE ===` lives in the working log mid-engagement. Neither is
a boundary between deliverables, so leg-splitting does not reach them. Their
whitespace fragility was fixed separately in `5a3945ce`.

### And the half that makes the leg split worth doing: check §15's elements

**A marker is not evidence that a Gap Map was produced.** `=== GAP MAP ===`
currently does three jobs and is competent at one. It delimits two documents in
one reply — fine. It stands in for "the engagement finished" — weak. And it is
taken as proof a valid Gap Map followed, which it does not attempt: `score.py`
checks that the marker appeared and counts the words after it. That is all.

§15 requires six elements: target name and a one-line description, the §9
recommendation, three to five key items, a coverage line, the report-link line,
and the scope disclaimer. **None is checked.** A run emitting the marker
followed by 150 words of anything scores "Gap Map produced".

Measured 2026-08-25 across 25 runs: **two shipped a Gap Map missing a required
element** — one with no report-link line, one with no scope disclaimer — and
both passed the threshold.

When the Gap Map is its own leg the reply IS the Gap Map, and the question
stops being "is the marker there" and becomes "does this document have its six
elements". That is checkable, and it is the same move as §12a's sign-off: stop
detecting compliance with a string, test the thing itself.

**Write the test against §15's shape, not §16's.** A first attempt at this
check reported nine of twenty-five Gap Maps as missing the recommendation. All
nine were one model, and all nine were wrong: `recommendation_of` requires the
term to open the document, because §16 puts the recommendation first in a
REPORT. §15 puts target identity first, so that model's recommendation sits on
line 2 and the report-shaped locator could not see it. The measurement was
worse than the thing it measured. A Gap Map check needs its own locator.

### Queued, larger: `workflows` as a capability

The decision taken 2026-08-25: this is worth preserving and generalising. The
audit is one workflow; there will be others. The directory layout has not
caught up, and the import direction shows it.

**`workflows/claims_audit/continuation.py` imports `latest_reply` from
`workflows.claims_audit.runner`.** Product code reaching into a fixture for a
general utility — dependencies pointing from the general thing into the
specific one, which is the reliable smell.

`run.py` was born as a benchmark harness for one fixture, which is why it lives
where it does. Then `--external-repo` and `--brief-file` were added so it could
audit ChatterMate and Body, and it became the engagement driver without moving.
Its own help text records the split: *"audit a real target instead of the
fixture corpus. Scoring against the answer key is meaningless for one."*

About 25 lines of its 549 are fixture-specific — `BRIEF`, `CORPUS`, and
`engagement_state`'s document counting, which already carries the comment
"Fixture-only. A real target has a claim surface, not a document list."
Everything else knows nothing about FlowMetrics.

**Rough shape, to be planned properly rather than assumed:**

- a driver package for the engagement machinery, with `continuation.py` in it —
  that is product, not measurement
- `claims-audit` as one workflow beneath it, holding METHOD.md, CONTINUATION.md
  and the scenario, perhaps parallel to how `scenarios/` is organised
- `measure/fixtures/dataroom/` keeps the fixture: corpus, answer key, results,
  and `score.py`, which is genuinely fixture code because the answer key is
  compiled into it. Its brief becomes a file beside the corpus instead of a
  constant in the driver.
- `claims.py` and `overlap.py` are study tools and stay in `measure/`

**Two things that make it more than a file move:**

- **Paths are computed from `HERE`.** `CORPUS = HERE / "corpus"` and
  `out = HERE / "results" / ...`. Moving the file relocates where results are
  written. These have to become parameters.
- **Name collision.** `src/chat/workflow.py` already exists — the loader that
  strips practice-audience sections. A `src/workflow/` package beside it will
  confuse imports and conversation. Either pick another name or move the loader
  in with it, since they are the same concern.

**Open, and deliberately not decided:** whether one driver generalises cleanly
across workflows and METHOD files at all. The leg protocol, the deliverable
count and the working record are plausibly shared; the stopping rule and the
brief shape may not be. That is a question for the refactor plan, not an
assumption going into it.

**Sequence.** Review and settle `run.py` first. The refactor plans after that.

### Also queued, smaller

- **The claims-only harness measures something adjacent.** grok enumerated
  ChatterMate at 244 and Body at 164 in enumeration-only runs, and both at 118
  inside a full audit. Enumeration is drawn coarser when 118 claims then have
  to be verified. Do not quote claims-only counts as engagement denominators.
- **74 dropped quotes across nine runs** on the claim-attribution verbatim
  check — 20 in one run. Designed degradation, never examined. Either models
  paraphrase evidence heavily or the check is too strict after normalising.

## The claim surface: named by the engagement, not inferred

**The problem.** Three models given the same nine documents enumerated 62, 67 and
273 claims. Every coverage figure in a report divides by that number.

**Two attempts to fix it by defining a claim more precisely both failed.**
"One claim is one assertion that can take exactly one §6 verdict" widened the
spread to 44, 21 and 321 — "one assertion" has no fixed size, and two models read
it at different scales without either misreading it. A second, procedural
version produced 66 and 108 and left the third model unable to close the surface
at all: it was longer, and the model relayed the instruction text into a subagent
query instead of executing it.

**What worked.** §2 defines a claim as an assertion the seller makes to the
buyer, which makes source code, comments and a data room's evidence documents
evidence rather than claims. §12 step 1 has the engagement name which documents
carry the assertions. For the fixture that is doc1, doc2 and doc9.

**Measured after the change.** grok 33, qwen 70, luna 88 — but all three read
only the three named documents, and grok and qwen cited an identical 21-line
set (Jaccard 1.00; luna 0.75, differing only on where doc2's paragraph breaks).
Scope is settled. The residual is grain, and it is immaterial: a denominator
only has to be internally consistent and disclosed, and no client compares two
auditors' denominators. §12a, practice-only, has a person confirm the surface
before delivery.

**The fixture was flattering us.** On ChatterMate — a 625-line README —
grok enumerated 244 claims and qwen 58, agreeing on only 129 of 311 cited lines
(Jaccard 0.41). The 1.00 is substantially a property of three short,
claim-dense documents, not of the method. Do not report fixture agreement as a
general result.

### Claim sources, settled per target

| target | claim sources | grok count |
|---|---|---|
| FlowMetrics (fixture) | doc1, doc2, doc9 | 33 |
| ChatterMate | README.md, llms.txt, HELP_CENTER_INFRA.md | 244 |
| Body | README.md, body_project_spec.md | 164 |

`claims.py` enumerates and stops — one leg, no verification, about a fifth of a
scored run. `overlap.py` compares two surfaces by what they cited, because two
models both reporting 28 claims may have enumerated two different sets of 28.

## The scorer read a tool's output as the auditor's closure

An model reported 28 findings against a 14-claim surface. That is impossible, and
the 14 was the scorer's number rather than the model's. It came from a
`process_text` call whose observation arrived truncated at the 1000-char trace
cap, announcing 14 while listing 18. The model said so in its next thought — "the
output is unreliable... it says 14 claims but lists 18" — discarded it,
re-enumerated, and closed at 32 in its own words. `claim_surface` took the
first marker with a count and recorded the one the agent had rejected.

**The 1000-char cap is not the fix.** `chat_loop.py:1719` states that the
stored record is re-injected in full into the next few turns' prompts, so the
cap bounds prompt text as well as trace size. Raising it changes what every
model reads on every later turn.

**Excluding tool output was tried first and was wrong.** An model may close the
surface through a tool — delegate the enumeration and adopt the result — and
that is a real closure. Across the 15 runs on disk, excluding observations
turned three legitimate tool-delegated closures into no count at all, which
fails the threshold. Position does not separate the good case from the bad;
adoption does, and that needs judgement.

The rule is now: **read the agent's own `ACTION:` lines first, fall back to
what it was handed.** Exactly one number moved across 15 runs.

## `unsupported` gated a PASS/FAIL on an unstable definition

The same memo scored 2 unsupported claims, then 0, on consecutive grader calls.
The definition described the category rather than testing for it. It now reads:
unsupported only where the memo states a fact that contradicts the data room or
appears nowhere in it; where the facts are in the documents and the memo has
drawn its own conclusion, that is "other". Ties break to "other", because the
uncertain case belongs on the ungated side. Three consecutive calls afterwards:
0, 0, 0.

## METHOD rewritten for its reader — 2026-08-25

Agent-visible text went from 32,333 to about 20,000 characters. Removed: a
scope rule describing a codebase audit for engagements that have no codebase,
and which excluded the financial claims that produce the most material
findings; a §3 telling a SaaS auditor to put security "front and centre" while
§11 says this is not a penetration test; a §8 titled "two recaps" that
described one, and which used "§1" to mean the report's first section rather
than this file's; every pointer into a practice-stripped section; and three
target facts carried from a previous engagement, which §14 forbids.

Four terms had been defined precisely and then reused loosely: `delta` named a
verdict, a form field and a synonym for findings; `walk` sat in both the
audit's vocabulary and the buyer's, two lines apart, inside the section
forbidding the conflation; `micro-claim` meant both the whole surface and its
lowest tier; `statement` meant a sentence in one rule and an assertion in the
next, which made the two rules contradict. The general rule is now in
`CLAUDE.md` — write for the reader, not for yourself.

**Validated, not just read.** grok on the rewritten text: 8/8, Tier 1 3/3,
Tier 2 2/2, 0 unsupported, top finding the key's #1, claim surface 33 — the
same 33 an independent enumeration-only run produced.

## Open

- **One model's enumeration varies run to run.** qwen has closed the same three
  documents at 70 and at 32. grok gave 33 twice; luna 88 and 86. Not
  understood.
- **ChatterMate agreement is 0.41.** Whether that is the target's size, its
  prose style, or the models, is untested. Two more models on ChatterMate would
  say.
- **`score.py` checks the §9 taxonomy was used, never that coverage supports
  it.** An model verifying 15 of 273 claims and returning "Material" scores the
  same as one verifying 45. §1a forbids the conclusion without the coverage;
  nothing enforces it.

## Superseded: pick up here — 2026-08-24

**The instrument is finished and frozen at `27353583`.** Tree clean, tests
pass, zero runs on the board. Everything below is measurement, not more
instrument work.

### Tomorrow: three runs per model

```bash
R=~/Downloads/Cognitive_workbench
eval "$(grep -m1 '^export XAI_API_KEY=' ~/.bashrc)";    export XAI_API_KEY
eval "$(grep -m1 '^export OPENAI_API_KEY=' ~/.bashrc)"; export OPENAI_API_KEY
for i in 1 2 3; do for a in grok:grok_4p6 qwen:qwen_local luna:luna_openai; do
  n=${a%%:*}; f=${a##*:}
  python3 $R/workflows/claims_audit/runner.py --world m1_${n}_$i \
          --model $R/measure/models/${f}.yaml
done; done
```

Interleave by round, not blocked by model, so a mid-campaign disturbance hits
all three equally. Nine runs, roughly 90 minutes on last night's timings
(grok ~440s, qwen ~650s, luna ~950s). Then
`score.py --world <w>` per run; the grader costs one call each.

**Do not edit METHOD.md, run.py or score.py once the first run starts.** Two
campaigns were discarded for exactly that. If something needs fixing, finish
the nine, then fix, then re-run.

### The threshold is eight criteria

three must-find items · Gap Map · §9 recommendation · leads with a top-3
finding · no unsupported claims · §6 verdicts only · limitations statement ·
claim surface closed

Three of those are new and were exercised once each. Report length is in the
vector and **not** gated.

### What to expect, so a surprise is legible

- **Luna's subagent no-answer rate.** 36% before the harness fixes, 25% after,
  against 0% for both other models. The only model-specific effect that has
  replicated across two instruments. If it vanishes, suspect the instrument.
- **grok's derived-finding recall** was 2/2 in three consecutive runs on the
  old instrument. Untested since.
- **The claim-surface marker is newly mandated in shape.** On first exposure
  only qwen produced a parseable count; grok listed without counting and luna
  quoted the instruction. Watch whether the tightened §12.2 fixes both.

### Known, deliberately not fixed

- `difflib` view for `process_text` — needs a test matrix across
  edit/summarise/translate shapes before it goes near a campaign.
- Grader misses B6 three times in four; Tier 3 noise 6/5/5/5. **Trust the
  threshold, not Tier 3.**
- The fixture and `answer_key.md` still share a lineage. METHOD now has an
  external reference (`workflows/claims_audit/method/gap-analysis.md`); the key does not.
- Saturation: all models cleared every criterion before the three new ones
  landed. If all nine pass tomorrow, the threshold has stopped discriminating
  and the vector is the only signal left.
- DeepSeek, both Nemotrons and gemma-4-31B are in the settings table and have
  never run under it.
- `chat_loop.py:1667` still strips NOTE lines from the stored working log.

## What the 2026-08-24 harness fixes were supposed to change

The runs behind this are deleted; the observations are kept because they are
what the fixes were aimed at, and a fix nobody stated a prediction for cannot
be shown to have worked. **Predictions, written before the re-run.**

Pre-change behaviour, one run per model, all at the corrected per-model
temperatures:

| | legs | iters | `process_text` | `inspect_external` | words |
|---|---|---|---|---|---|
| grok-4.6 | 2 | 9 | **0** | 5 | 1,262 |
| Qwen3.8-27B | 2 | 24 | 3 | 16 | 1,706 |
| gpt-5.6-luna | 3 | **40** | **13** | 24 | 1,019 |

Three behaviours, and what each fix should do to them:

**1. The phantom trailing line (fix: the working log is now closed).** Luna
spent ten of leg 3's sixteen iterations instructing `process_text` to delete a
line reading `Emit next action:` — the ReAct prompt's own trailer, which the
unclosed working log left sitting one newline below an 8,000-character
document. The string appears **zero** times in the delivered report and ten
times in the trace, every one inside an instruction asking for its removal.

> **Predicted:** no instruction referencing `Emit next action:`; Luna's leg-3
> iteration count well below the cap of 16. **Falsified if** the phantom
> recurs, which would mean the boundary was not the cause.

**2. Dangling section references (fix: the method crosses into
`process_text`).** Qwen instructed a sub-call to "use the §5 finding format,
the §6 verdict vocabulary, the §9 taxonomy, a coverage statement per §4"
against a system prompt containing only a framing line, the persona and a
citation rule. Four pointers to a document the sub-call had never seen.

> **Predicted:** Qwen's §-references now resolve, so its report conforms to
> §5/§6 without the parent reformatting. **Falsified if** conformance is
> unchanged — which would mean the sub-model was already guessing the format
> correctly and the references never mattered.

**3. Contract inlining (same fix, opposite model).** Luna compensated correctly
for the same gap by pasting ~3,000 characters of taxonomy and finding
structure into every composition instruction.

> **Predicted:** Luna's `process_text` instructions get materially shorter,
> because the contract is now in the sub-call's context. **This is the cheap
> falsifiable one** — instruction length is mechanical to measure.

**A caution against reading these as model comparisons.** n=1 per model, and
the three models differ in temperature by design. What is being tested is
whether two harness defects stopped happening, not which model is better.

## Subagent contract compliance — counted, not repaired

Added to `score.py`'s mechanical block 2026-08-24. Free, no grader, no change
to anything the agent sees.

The subagent contract is one line: finish by emitting `respond` with your
answer in `text`. A call that returns nothing has failed it. The parent sees
only `EMPTY:`, retries narrower, and the cost lands in the iteration count
where nothing distinguishes it from thorough work.

First measurement, one run per model:

| model | `inspect_external` calls | returned NO answer | exits |
|---|---|---|---|
| grok-4.6 | 9 | **0 (0%)** | respond 8, **llm_error 1** |
| Qwen3.8-27B | 7 | **0 (0%)** | respond 7 |
| gpt-5.6-luna | 33 | **12 (36%)** | respond 32, max_iters 1 |

All twelve of Luna's are `exit=respond` — the subagent chose to finish and
finished with an empty `text`. Hand-read, they split evenly:

- **Six declare the tools broken without calling them**, several at
  `iters=1`: *"Unable to inspect the repository because tool results were not
  returned."*
- **Six announce an answer and emit nothing**: *"doc7 has been read in full;
  I'll provide its exact line-numbered contents…"* — then no text.

**Why this is counted rather than fixed.** `subagent._salvage` reconstructs an
answer for the paths where the loop never GOT to answer (`max_iters`,
`format_failed`). These are not those paths: the subagent chose to respond and
responded with nothing. Salvaging there would substitute reconstructed
observations for an answer the model declined to give — manufacturing evidence
to cover a contract violation, in a product whose entire value is provenance.

**The line this draws.** Fix the harness where the harness is wrong; do not fix
it where the model is wrong. The two changes shipped on 2026-08-24 were
defects — the working log was unclosed, so the prompt's own trailer read as
document content; and §-references reached a sub-call that had never been given
the method. A model that reports tools failed without calling them is not a
defect in either. It is a finding, and it now has a denominator.

Also worth its own note: the counter surfaced an `llm_error` subagent exit on
grok that nobody had seen, on a run that scored the best of the three. Failures
that get recovered from are still failures worth counting.

## Grader revalidated at top_p 0.95 — 2026-08-24

The un-pinning left the grader's hand-validation stale, because that had been
done at top_p 1.0. Redone. **Precision improved; accuracy is uneven, and all
of the unevenness sits in the tier that cannot change a verdict.**

**Precision.** Four passes over one identical report:

| | pass 1 | 2 | 3 | 4 |
|---|---|---|---|---|
| Tier 1 / Tier 2 / unsupported / THRESHOLD | identical across all four | | | |
| Tier 3 | 6 | 5 | 5 | 5 |

Against the documented baseline at top_p 1.0 — Tier 3 = 5, 4, 5 and
**unsupported = 1, 1, 2** — this is a real improvement, and the mechanism is
plain: top_p 0.95 with temperature 0.1 is near-greedy where 1.0 kept the tail.
The failure that mattered — `unsupported` flapping 0/1/2 and turning PASS/FAIL
on grader noise — did not recur on that report. It DID recur on another, so
the improvement is not universal.

**Accuracy, hand-checked against the answer key.** B6 is a genuine finding the
report states correctly — claim cited, `doc4 lines 3-8` for the co-located
database, delta stated — and **three of four passes missed it**. B7 is credited
on partial support: the report nails the renewal-inside-notice-window half and
never states the concentration half. Not the old error (crediting from a
forbidden questions section) but the looser reading.

**Verdict: trust it for the threshold, not for Tier 3.** All observed noise is
in the credit-only tier, which is never penalised and cannot move a verdict.
Report Tier 3 as an indicator, never as a score.

## Runs deleted 2026-08-24, and what they showed

Seven runs (three models) deleted when METHOD.md changed — §8's working recap
and §12's delta-confirmation removed as unperformable, `[real, with a
structural note]` removed as unused, and `run.py`'s brief corrected where it
still instructed both removed procedures. Everything they measured is
downstream of that.

Kept because they are the only observations of these behaviours:

| model | n | threshold | Tier 2 | subagent no-answer | words |
|---|---|---|---|---|---|
| grok-4.6 | 3 | **3/3 PASS** | **2/2 every run** | 0 of 22 | 928-1,194 |
| Qwen3.8-27B | 3 | 1/3 PASS | 2/2 once, 1/2 twice | 0 of 24 | 1,804-2,851 |
| gpt-5.6-luna | 1 | PASS | 1/2 | **12 of 33** | 1,156 |

**A correction worth keeping.** Qwen's two failures were first reported here as
length failures. They were not — removing the word gate flipped neither. They
fail on placement, and one also on §9 and unsupported. The length reading came
from seeing "OVER the 2,000 guide" and not reading the criteria list.

**Predictions carried forward, still untested against the current method:**
grok's derived-finding recall (2/2 in three consecutive runs) and Luna's
subagent no-answer rate (~36%) are the two effects large enough to expect
again. If either vanishes on the next campaign, suspect the instrument before
the model.

## Outstanding work — needs careful, extensive testing before shipping

**A diff view for `process_text`.** The observation would carry a
`difflib.unified_diff` of source-vs-result instead of the whole rewritten
document, while the `$stepN` binding keeps the full text. The split is known
to work: on 2026-08-24 a `respond text="$step15"` delivered all 8,114
characters while the stored observation held 8,000, so a binding already
resolves to more than the observation shows.

**Why it is not shipped.** It changes what the agent sees on every
`process_text` call — a workhorse tool — and a diff is only the right view
for edit-shaped instructions. For a summarisation the result IS the answer
and a diff against the source is noise. Choosing between them means
classifying the instruction, which the house rule forbids doing by keyword;
the alternative is a similarity heuristic, which is a new failure mode on a
hot path. Needs deliberate design and a test matrix across edit / summarise /
translate / extract shapes before it goes anywhere near a campaign.

The cheap half of the benefit shipped instead: `process_text` now reports
`EMPTY: … returned the source UNCHANGED` on a no-op, which is what actually
ends the observed loop.

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

Same for `XAI_API_KEY`. Note that `~/.bashrc` also exports `GROK_API_KEY`, a
**different** key value that is equally valid against api.x.ai — both return
200. `grok_4p6.yaml` names `XAI_API_KEY`; if a quota or billing question ever
comes up, that duplication is the first place to look.

## Yield-adherence, carried forward

> Survived one discard already, and survives this one on the same
> reasoning: these are observations about harness behaviour, not scores.
> The runs are deleted; the two behaviours still want a probe of their own.

Two observations outlive the discard because they are about behaviour rather
than scores. Both deserve a probe of their own rather than being folded into
this fixture:

- An model ended a turn with a stated-but-unexecuted plan — "I will now begin
  working the priority order" — which its own character block forbids.
- An model received the budget nudge at iteration 10 of 12 (`react.py:418`:
  *do NOT start anything new, emit `yield` NOW*) and then started new work
  three more times.
