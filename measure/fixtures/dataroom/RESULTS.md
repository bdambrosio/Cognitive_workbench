# dataroom — campaign state

Started 2026-08-23, on the workflow harness. Everything before it was
discarded, deliberately — see "Why every earlier run was discarded".

## The board is empty — 2026-08-27

**Every run was deleted when METHOD §4 and §16 were amended**, late on
2026-08-27. Campaigns b2, b3 and the `cm_glm_1` ChatterMate run all predate the
amendment and are not comparable to anything run after it. Fourteen run
directories and forty scenario worlds removed; the delivered ChatterMate
engagement under `engagements/chattermate/delivered/` is a client artifact, not
a run, and was kept.

**What changed and why it invalidates them.** §4 and §6 contradicted each other
on whether a claim that holds is a finding. Resolved to §6's reading: every
examined claim now produces a finding, so a report that recorded forty held
claims as a consistency rate and five gaps as findings is a different artifact
from one that records forty-five findings. §16's 2,000-word target went with
it, replaced by a per-finding budget. Report length, finding count, the review's
enumerated surface and its supported ratio all move.

**The rule this follows is the one that discarded three campaigns on
2026-08-24**: comparisons within one instrument are clean, across instruments
they are not. It has now cost a full day's runs twice, which is the price of
the rule rather than an argument against it.

**Verdicts withdrawn.** No model currently holds a qualification result.
GLM-5.3-Flash's and Qwen3.8-27B's results are gone with the runs that produced
them, and grok has no baseline on the current method. The next campaign starts
from nothing.

**Lessons kept, and where.** The findings from those runs are behaviour of
models and of the instrument, not scores, so they survive the runs:
`docs/held-claims-are-unreviewed.md`, `docs/derived-findings-and-uncited-evidence.md`,
`docs/multi-run-merge.md`, `docs/model-prescreen.md`, and
`docs/Workflow/essay-01-a-method-is-not-a-script-v2.md`. Each cites numbers
from runs that no longer exist; treat those as illustrations of a mechanism,
never as current measurements.

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

> The threshold is a **run-level** result. What a *model* has to do across
> three runs to qualify is frozen in `docs/model-qualification.md`, and it
> reads this table one criterion at a time rather than through the
> conjunction verdict.

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

## Pick up here — 2026-08-26, end of session

> **Read the retraction below before this section.** Both paragraphs that
> follow were written from the review's verdict and are wrong in their
> central claim. They are kept because the correction is only legible
> against them.

**The campaign table is in question, and that is the headline.** An independent
review of `m1_qwen_2` returned **supported 1 of 15 findings, FAIL**. That run
passed every check the threshold has — valid §9 recommendation, closed §6
vocabulary, limitations statement, complete Gap Map with every §15 element,
Tier 1 3/3 against the answer key — and four of its findings cite lines that do
not exist while ten cite lines that say something else. Verified by hand:
`doc2` has 5 lines and the report cites `doc2:6, :11, :12, :13`; `doc9:10` is
cited for "managed DNS" and is about backups.

It found the right things and fabricated the evidence trail for nearly all of
them. **If one run in nine does that while passing, the m1 table measures
format compliance more than it measures work.** Nothing in it should be quoted
until the other eight are reviewed.

### That headline is wrong. Retracted 2026-08-26

**The review's facts are all correct, its verdict is defensible, and its number
is an artifact of the reviewer's harness.** Fourteen of the fifteen findings
are defensible from the materials. What failed is the citation *pointers*, and
the reviewer could see only the half of the citation surface that fails.

Checked by hand against the corpus, not taken from the review:

| | result |
|---|---|
| claim-side pointers resolving correctly **as line numbers** | 5 of 23 |
| evidence-side quotes reproduced verbatim from doc3–doc8 | **every one** |
| findings defensible from the materials | 14 of 15 |

Every evidence quote checks out exactly: "Failures recorded for the last 21
days", "Last Successful Backup: 2026-07-30", "Uptime Monitor: None (No Pingdom,
UptimeRobot, or custom checks)", "Unit Tests: 12 (all located in `test/utils/`)",
"Payment-Path Tests: 0", "Managed personally by 'dave'. No secondary DNS
provider". The audit did the work. It could not say where it got it.

#### The auditor was citing claim ordinals, not line numbers

`doc2` has five lines and the report cites `doc2:11, :12, :13`. Those are not
broken line references. They are claim indices, and they are correct ones.

The report declares the scheme in its own coverage statement — *"48 claims
identified from three claim sources (doc1: 25, **doc2: 13**, **doc9: 10**)"* —
and every reference fits it in document order:

| cited | claim | position in doc2 |
|---|---|---|
| `doc2:2` | automatic scaling and failover at the platform level | para 1 |
| `doc2:3` | remains available under varying load | para 1 |
| `doc2:4` | no manual intervention for routine operations | para 1 end |
| `doc2:6` | configured with daily scheduled backups | para 2 early |
| `doc2:11` | the entire stack is managed through Heroku's dashboard | para 2 late |
| `doc2:12` | deployments, scaling and monitoring are streamlined | para 2 late |
| `doc2:13` | standard SaaS stack with platform-level redundancy | final phrase |

Perfectly monotone, and it terminates at 13 — the count the report declared.
doc9's ten claims land the same way, `:1` redundancy through `:10` managed DNS,
with five of six distinct references correct under the scheme.

**Inference, not verification.** `working_record/method_as_delivered.md` is
absent from this run, so the auditor's intent cannot be confirmed. The fit is
tight — two independently declared counts, both matching, both monotone in
document order — but it is a reconstruction.

**It inverts the flagship example.** "`doc9:10` is cited for managed DNS and is
about backups" was recorded above as the clearest instance of a fabricated
trail. Under claim indexing `doc9:10` **is** managed DNS, the tenth of the ten
claims the report declares for that document. F15 is correct. The real error is
F1 and F2 reusing `:10` for the backup claim, which is `:7`.

**Why this was hard to see:** doc1 declares 25 claims and doc1 has exactly 25
lines. Its references are ambiguous between the two schemes, and three of them
happen to resolve correctly as lines. The coincidence made the report look like
a document that was citing lines and getting them wrong.

#### For most documents, line numbers were destroyed in transit

`inspect`'s `read` returns numbered lines (`19|*   **Reliability:** …`). The
subagent sees them. What reaches the auditor is the subagent's `respond`.
Measured across this run's three traces:

| trace | documents | numbered lines surviving into the answer |
|---|---|---|
| 1 | doc1 | **0** — reformatted as prose, closing "That is the entire file — 25 lines" |
| 2 | doc2, doc9 | 16 — passed through intact |
| 3 | doc3–doc8 (every evidence document) | **0** |

So for doc1 and for all six evidence documents, METHOD §5's *"each with line
numbers"* was not an instruction the auditor could follow — it held no line
numbers. That is why the evidence half cites section names (`Evidence (doc4,
Backups section)`): not sloppiness, the only locator it had.

**This is the fourth instance of the shape.** Retrieval was reliable; transit
was not. See `docs/RESUME-2026-08-24.md`, "evidence was retrieved and then lost
in transit".

Note what it does **not** explain. doc2 and doc9 arrived fully numbered and the
auditor still used ordinals. Where the numbers were present it did not want
them, because it was working in the claim surface §4 had made it build.

#### The reviewer saw one of three citation dialects, and it was the worst one

`runner.py:70`'s `_CITE` matches `docN:NN`. The report emits three forms:

| dialect | where | count | in `citations.json` |
|---|---|---|---|
| claim ordinals, `docN:NN` | claim half | 23 | **all 23, resolved as line numbers** |
| section names, `(doc4, Backups section)` | evidence half | 15 | none |
| line numbers | nowhere | 0 | — |

The brief then forbids re-fetching (`runner.py:199`: *"Citations have already
been resolved for you… Do not re-fetch them"*), so `citations.json` became the
reviewer's whole evidence base. Exception 5 states the consequence outright:
F2 is ruled unsupported because *"last-success / doc4 is not in the resolved
citation set"* — reasoning from absence in a partial index. `doc4:18` reads
`**Last Successful Backup:** 2026-07-30`, the report quotes it exactly, and the
derivation to 2026-08-29 is arithmetically right.

**Two symmetric errors, from one regex.** It over-reported the claim half to 14
false exceptions, and never saw that the evidence half violates §5 too —
fifteen real violations, unreported.

#### The failure mode, named

**Not malformed syntax. Well-formed syntax over an undeclared referent.**

All 23 citations parsed cleanly. There was no syntactic complaint available.
The regex succeeded on every one and resolved them against the wrong axis,
producing eighteen confident false readings with verbatim quoted text attached.
A parse *failure* would have been the safer outcome: a parser can report a
malformed citation, and nothing reports a correct-looking integer indexing a
different coordinate system.

The generalisation, which predicts the other queued items: **a syntactic
contract holds when the required token is copyable from something in front of
the model at generation time, and fails when it must be reconstructed from a
rendering the model no longer holds.** `=== LIMITATIONS ===` and the
`[delta]`/`[partial]` vocabulary held in this run because they sit in the
prompt. Line numbers failed because they are a property of a display two hops
upstream. Apply the same test to the leg-split completion signal now queued: it
holds if the harness emits it, and fails if the model must synthesise it.

#### What this does to the m1 table

Less than the retracted headline claimed, and not nothing.

- **The audit's work was sound.** 14 of 15 findings defensible, evidence quoted
  verbatim. "Fabricated the evidence trail" is withdrawn: the quotes are real
  and locatable, the pointers are unresolvable by a third party.
- **The report is still not deliverable.** A citation trail no reader outside
  the run can follow is not provenance, and §10's negligence defence rests on
  exactly that trail. FAIL is the right verdict for the wrong reason.
- **"The m1 table measures format compliance more than work" is withdrawn.**
  It was drawn from this review. What the run shows is a citation-hygiene
  failure, not a work failure. Whether that holds across the other eight is
  open and untested.

#### Admissibility is a real gate, and it is not decidable syntactically

This run is the proof: **the syntactic check passed 23 of 23 and the document
was still not reviewable.** Whether a report can be reviewed at all is a
separate question from whether its findings hold, and it cannot be settled by
code, because a citation can be well-formed, resolve to real text, and still
not permit a reader to establish what the auditor meant.

**Recommendation: a declared phase and a missing verdict, not a third process.**

The reviewer already held everything it needed to say "I cannot tell what
coordinate system this is." It did not say so because §6 gave it no way to.
All five verdicts — `[supported]`, `[overstated]`, `[understated]`,
`[unsupported]`, `[broken citation]` — presuppose the reviewer knows what the
citation points at. Add the sixth:

> `[indeterminate]` — the citation is well-formed and resolves to text, and the
> referent it indexes cannot be established from the report.

With that verdict this run reports 23 indeterminate citations and INADMISSIBLE,
instead of 14 unsupported and FAIL. That is the correct reading, and it costs
one table row.

Then make admissibility **§4.0**, before any finding is checked, with the
authority to end the run: state the citation scheme the report uses and whether
it can be established; if it cannot, stop and say so. It sits naturally beside
§4's existing "close the review surface" phase, needs no second scenario,
runner or method document, and shares the read the reviewer must do anyway.

**Why not a separate pre-screen process.** The independence it appears to buy
is not independence, it is a fresh context — and a fresh context is precisely
what loses the reading being judged. The cost is a new method document, a new
output contract and a new failure surface, against a gate that is one phase and
one verdict inside a process that already reads the report.

**The cheap half belongs on the audit side.** The auditor *did* declare its
scheme — in prose, in the coverage statement, at the end, far from the
citations it governs. METHOD §5 should require the citation scheme to be stated
where the citations are used, in one line. Admissibility is not only "can the
information be extracted"; it is "is the coordinate system declared".

#### Two confounds, both real

**The method is missing.** REVIEW.md §3 makes
`working_record/method_as_delivered.md` a mandated input, and it is absent from
this run. So it cannot be determined whether the auditor violated §5 or
followed a version that specified claim indices. **The reviewer noted the file
was missing and returned a verdict anyway** — a review inadmissible under its
own method, delivering a number. §4.0 would have caught its own case.

**The models differ**, qwen audited and grok reviewed. That is the documented
policy and it worked as intended. A same-model reviewer might have shared the
claim-ordinal convention and read the citations as the auditor meant them —
passing a report whose trail no third party can follow. Cross-model review is
what exposed this, and this is evidence for the policy rather than against it.

### The sweep answered it: six of 29, and quotes hold across legs — 2026-08-26

The new resolver was run over every report on disk, 29 runs, no model calls.

**Six runs carry at least one reference past the end of its document**, and
they are not one model's problem: `s4_qwen_1`, `m1_grok_1`, `m1_grok_2`,
`m1_qwen_2`, `cm_audit_grok`, `t3_grok_1`. Three of the nine m1 runs, including
two grok runs that passed.

**But the rate separates them, and the rate is the finding.** `m1_grok_1` has
one bad reference in 42, `doc7:94` against a 45-line document — a typo.
`m1_qwen_2` has four of its seven doc2 references past the end of a five-line
file, topping out at 13, against a coverage statement declaring "doc2: 13
claims". One is a slip; the other is a different coordinate system. §4.0 divides
on the rate for this reason, and `by_document` in the `scheme` block is what it
divides on.

**Quote resolution across all 29 runs: 692 of 733, 94%.** This settles the open
risk that would have blocked the citation-contract change: quote accuracy does
**not** degrade across legs. `m1_qwen_2` was single-leg, so its verbatim
evidence proved nothing about multi-leg runs; the multi-leg grok runs resolve at
the same rate (`m1_grok_3` 33/33, `m1_grok_2` 56/58, `m1_qwen_1` 70/73).

**One caveat that shapes the METHOD pass.** Quote density varies enormously —
`m1_qwen_1` carries 73 quoted spans, `s1_grok_1` carries five. A report can be
line-heavy and quote-light, so the quote channel is only reliable once METHOD
§5 asks for it. Today it asks for line numbers, which is why some reports barely
quote at all.

### The gate is live, and it fires on the right one — 2026-08-26

Three reviews under the amended REVIEW.md, all grok-4.6, all `error=None`.

| report | instrument | scheme | verdict | wall |
|---|---|---|---|---|
| `m1_qwen_2` | METHOD `bc4ae148` | 4 of 7 doc2 refs past EOF | **INADMISSIBLE** | 94s |
| `w1_grok_2` | current | 41 refs, 0 past EOF | ADMISSIBLE, 15/15, PASS | 304s |
| `w1_qwen_1` | current | 58 refs, 0 past EOF | ADMISSIBLE, 19/23, **FAIL** | 931s |

**The stop path works, and the reviewer reasoned its way there.** It was given
the mechanical signal only — that four integers exceed their document — and
found the corroboration itself: *"The same report states coverage as 'doc1: 25,
doc2: 13, doc9: 10.' The clustering and the stated claim count match each other.
They do not match line numbering."* It reported no ratio and enumerated nothing,
and it stopped in 94 seconds where the review it replaces spent 309 producing a
number. It also made an argument this file had not: the three doc2 integers that
*do* fall inside the file cannot be read as lines while the other four are
ordinals, because that is two schemes at once.

**Admissibility and quality are orthogonal, and `w1_qwen_1` proves it.** It
passes the gate on citation form and then fails the review on substance — one
`[unsupported]`, three `[overstated]`, all of the shape "the cited line shows
the add-on, not the use". A gate that only ever fired together with FAIL would
be measuring nothing new.

**The negative control holds.** `w1_grok_2` is admissible and clean. Two lines
of its summary are direct evidence the other two fixes landed: *"F4's
enterprise-contract figures were confirmed in doc7 **outside the line-index
keys**"* — the reviewer going to the materials for evidence the index does not
carry, which is exactly what Exception 5 refused to do — and a quoted span
correctly noted as non-contiguous rather than treated as a failure.

**One flap, recorded rather than explained.** `w1_grok_2` reviewed 14 of 15 with
one `[understated]` before the change and 15 of 15 with none after, on identical
text with the same model. `[understated]` is calibration and cannot move a
threshold, and PASS held both times — but it is a one-finding move on identical
input, which is the instability this file already documents for the grader. The
precision check has since been run; see the next section.

**Qwen's ordinals look like the old METHOD, not the model.** The same
Qwen3.8-27B that cited claim ordinals in `m1_qwen_2` cited 58 line references
with zero overruns under the current METHOD. n=1, and worth one more run before
it is believed.

### The admissibility verdict reproduces — 2026-08-26

Three independent reviews of the same report (`m1_qwen_2`), same reviewer
model (grok-4.6), each from a clean copy of the run directory with `review/`
and `review.pre-admissibility/` removed so no reviewer could see a predecessor
through `inspect`. Archived under `results/precision_m1_qwen_2/`.

| leg | verdict | findings enumerated | legs | wall |
|---|---|---|---|---|
| 1 (shipped) | **INADMISSIBLE** | 0 | 2 | 94s |
| 2 | **INADMISSIBLE** | 0 | 2 | 247s |
| 3 | **INADMISSIBLE** | 0 | 2 | 380s |

**3 of 3, and the reasoning reproduces too, not just the verdict.** The runner
hands the reviewer one mechanical signal — four integers exceed their document.
All three legs went looking for corroboration and all three found the same
thing without being pointed at it: the report's own coverage line says
"doc2: 13", and 13 is the largest integer cited for a five-line file. Leg 1 and
leg 3 each added the two-schemes-at-once argument — that the three doc2
integers falling inside the file cannot be lines while the other four are
ordinals. Each leg stopped in two turns, enumerated nothing, and reported no
ratio.

**The mechanical layer is byte-identical across legs.** `citations.json` and
`conformance.json` hash the same in all three. That is expected — they are file
operations — but it means the only thing varying across these rows is the
judgement, which is what the check was for.

**What this does not establish.** One report, one direction. A verdict that
reproduces on a report where the signal is loud says nothing about a report
where four of seven references overrun by one line instead of eight. The false
*positive* — a gate firing on an admissible report — remains untested, and
`w1_grok_2` (41 refs, 0 overruns) passing once is the only evidence against it.

**Wall clock spread 4x on identical input**: 94s, 247s, 380s, same two-leg
shape and same three `inspect` calls each. Cost per review is not predictable
from the input; budget from the slow end.

### METHOD §5 was rewritten and reverted the same day — 2026-08-26

**What was changed.** §5's citation contract went from `document:lines` on both
halves of every finding to document plus verbatim quote, on the argument that a
line number is not copyable at generation time: the inspect subagent numbers
the lines it reads, then answers in prose with the numbers gone, so writing one
means reconstructing it. Quotes were measured to resolve 692/733 across 29
reports where line references overran their document in 6 of 29 runs.

**Why it was reverted.** Two audits under the new contract, one grok and one
Qwen3.8-27B on the fixture, against the two runs it replaced:

| report | evidence fields | resolving quote | quote that fails | line ref only | **nothing** |
|---|---|---|---|---|---|
| `q1_grok_1` new §5 | 37 | 33 | 0 | 1 | **3** |
| `q1_qwen_1` new §5 | 33 | 16 | 7 | 0 | **10** |
| `w1_grok_2` §5 as it stands | 29 | 14 | 0 | 14 | **1** |
| `w1_qwen_1` §5 as it stands | 45 | 34 | 2 | 8 | **1** |

The two `nothing` cells in the bottom rows are fields asserting that no
evidence exists — "no source code provided to verify exact version" — which is
the one legitimate case. The ten in `q1_qwen_1` are not: `Evidence (doc4):
Single standard-1x dyno, no read replicas, no separate DB instance` had carried
`doc4:5`. **Seventeen of its 33 evidence fields, 52%, gave a reader nothing to
search for.**

**The mechanism, and it is general.** `Evidence (doc4:17–19)` has a slot that
looks wrong when empty. `Evidence (doc4)` looks complete while naming a whole
document. A required token became an encouraged behaviour, because a quote is
content and not a slot. Grok kept quoting; Qwen took the format at its word.

**Three further findings from the excursion.**

- **The claim-ordinal defect relocated rather than being fixed** — Qwen wrote
  `Claim (doc9, claim 44)`, moving the ordinal into the document slot — and
  §4.0 went blind to it, because that report emits zero `docN:NN` references
  for the gate to test.
- **The premise was false where it mattered.** Cross-checking the two channels
  against each other on the runs that carry both, the quoted text sits inside
  the lines the same field cites in **28 of 29** and **53 of 55** cases, and
  every exception is a paraphrase rather than a misdirected reference. There
  was no epidemic of wrong-but-resolving citations.
- **"A quote is text you are looking at" is false across a leg boundary.**
  Qwen's seven failing quotes are all reconstructions — `"database backed up
  daily to Heroku managed storage"` against doc9's `"The database is backed up
  daily to Heroku's managed storage"`. That is the same failure line numbers
  have. Grok resolved 64 of 64. The channel's reliability is model-dependent,
  so a quote-only contract measures transcription discipline as much as audit
  work.

**The rule this cost.** A citation that resolves to the wrong line is what the
review layer exists to catch — REVIEW.md §6 decides whether a cited line
SUPPORTS a claim, and §4.0 stopped `m1_qwen_2` for exactly this class three
times out of three the same morning. **Do not weaken a base-instrument
requirement to solve a review-layer problem.** The defect was already handled
in the layer that owns it.

**The methodological error, separately.** The new contract was validated by
running the CHECKER over reports written under the OLD one. Every one of them
carried line references, so the failure that appears when the pointer
obligation is removed was structurally invisible to every measurement taken
before the change shipped.

**What was kept**, all in `workflows/audit_review/runner.py`, none of it
touching what the auditor must produce:

- **Quote marks pair in document order.** The old extractor scanned for an
  opening mark followed by a closing one, slid past a pair too short to keep,
  and matched from that pair's closing mark to the next pair's opening one.
  `"Streamlined" is not a fair description of "not present."` was reported as a
  quotation of the materials that resolved to nothing — the auditor's own prose
  scored as fabricated evidence.
- **Quotes are read from `Claim`, `Evidence` and `Basis` only.** `Gap:` is
  where quoting a word back at the seller is legitimate writing. This removed
  every false miss: five became three, and all three survivors are real
  paraphrases.
- **One quote, one document.** A quote resolves when some single document holds
  every segment, not when each segment is found somewhere. Fragments from two
  documents joined into one sentence read as continuous evidence and are true
  of neither. It costs nothing — no quote in any report on disk needed two
  documents — and it settles attribution without reading the citation label.
- **`evidence fields pointing nowhere` in `conformance.json`.** The counter
  that was missing: `resolve_citations` and `resolve_quotes` score the pointers
  a report makes, and neither notices a field making none. `q1_qwen_1` read
  "16 of 23 quotes resolving" beside ten fields with nothing. Nothing in
  REVIEW.md reads this counter yet; wiring it in is a method decision and is
  not made.

### The restored instrument, verified — 2026-08-26

One grok and one Qwen3.8-27B audit on the fixture after the revert, then a
review of each. `err=None` on all four.

| run | model | findings | evidence fields | line refs | broken | quotes | resolving | **pointing nowhere** |
|---|---|---|---|---|---|---|---|---|
| `r1_grok_1` | grok-4.6 | 12 | 46 | 47 | 1 | 53 | 53 | **0** |
| `r1_qwen_1` | Qwen3.8-27B | 13 | 25 | 27 | 13 | 12 | 7 | **0** |
| `w1_grok_2` | grok-4.6 | 15 | 29 | 41 | 0 | 29 | 28 | 1 |
| `w1_qwen_1` | Qwen3.8-27B | 23 | 45 | 58 | 0 | 54 | 52 | 1 |

**The verifiability requirement holds.** No evidence field on either new run
points nowhere; the two `1`s are the legitimate case, a field stating that no
evidence exists. The quote-only contract produced ten.

**Qwen3.8-27B cannot reliably follow this METHOD, and that is the finding.**
`r1_qwen_1` puts 13 of 27 references past the end of their document, across
four documents at once — doc9 six of six, max cited 32 against an 11-line file;
doc2 three of three, max 23 against five lines. The review returned
**INADMISSIBLE** and reasoned its way there from the mechanical signal alone:
*"doc9 topping out at 32 against the report's stated 32-claim surface. Claim
and Evidence use the same form, so the citations cannot be read."*

**This settles the open watch item, the other way.** The note carried forward
was that Qwen cited 58 clean references on `w1_qwen_1` after citing claim
ordinals on `m1_qwen_2`, and that if it held on a second run the original
defect was never a model property. It did not hold. `w1_qwen_1` was the
outlier. State it as what the evidence supports — this model does not reliably
follow this method — rather than as a property of the model at large.

**And it is the argument for the layer rule, run forward.** The defect that
prompted rewriting §5 is real and recurs at a 48% rate on this model. The base
instrument still requires the pointer; the review layer catches the bad
pointer and stops the report. Removing the requirement would have hidden the
same defect behind prose.

**For this workflow, Qwen3.8-27B is not usable.** Two of its three runs on the
current method are inadmissible.

### The review layer got two rules and a retest — 2026-08-26

**REVIEW.md §4.0 asked one of the two questions it promised.** It opens "Two
questions, and this one comes first" and then only ever asks whether the
citations can be placed. The missing one is whether the report cites at all.
A report with three of eighteen evidence fields pointing nowhere was reviewed
**ADMISSIBLE, supported 10 of 10, PASS** — the reviewer wrote "cited lines say
what the findings say they say", which was true: it checked the twelve
references that existed while a sixth of the evidence carried none. A field
that names a document and then writes prose makes no citation, so it is
invisible in every citation count.

`[uncited]` is now a sixth verdict and §9 fails on it. It is not a stretched
`[broken citation]` — that has a reference which does not resolve — nor an
`[indeterminate]`, which has one that resolves to text the reader cannot
place. `[uncited]` has none at all. Re-reviewed under the rule, that report
returned **7 of 10, FAIL**, with three `[uncited]`.

**§9 fails a report on a single exception, which turns reviewer noise into
verdict noise.** `r1_grok_1` reviewed five times: supported 12 of 12 PASS four
times, 11 of 12 FAIL once. The dissent rebutted a finding by attacking a claim
the audit never made — the report said Rails 7 was "not re-verified from a
Gemfile", a statement about its own coverage, and the review read it as a
claim about the target. At a 2% per-finding error rate a 12-finding report
flips ~21% of the time and a 23-finding one ~37%: the more thorough the audit,
the likelier it is failed for nothing.

**So a claim-check fail is retested once.** `[unsupported]` and
`[indeterminate]` rest on a reading of a line against a claim, and the fail
stands only if a retest — not told the first verdict, unable to see the review
— reaches the same verdict. A single disagreement means it does not stand.
Where the two divide the finding is genuinely borderline, so the tally is
reported either way and a fail that does not stand stays in the review as
found but not upheld. `[broken citation]` and `[uncited]` are not retested: a
reference resolves or it does not, and asking a model to re-derive a file fact
is not another opinion.

**Renaming is not moving.** The one-review-per-run guard tested for the
literal name `review/`, so a previous review parked alongside as
`review.8192-era/` was exactly as visible to `inspect` as `review/` had been.
Four re-reviews ran under that hole and were discarded. The guard now refuses
on any `review*` directory holding a `review.md`.

**The supported ratio does not hold still, and that limits what the model
search can grade on.** `w1_qwen_1` went 19 of 23 FAIL, then 21 of 23 FAIL,
then 23 of 23 PASS across three reviews of identical text. Admissibility has
been measured stable at 3 of 3; PASS/FAIL has now been measured unstable
twice. A candidate reviewer should be graded on admissibility, with the ratio
reported and not scored.

### Ceilings were the model's problem, not the model — 2026-08-26

Four emission ceilings sit on the audit path and only one is the 8192 anyone
talks about. Two candidates were cut mid-emission from two different limits:
`inspect_external` at the subagent's 4096, and the main loop at 8192. Raised
to 8192 and 16384 respectively, with `_PROCESS_TEXT_MAX_TOKENS` and the stored
observation cap (1000 -> 4096) alongside. The truncation marker now says where
the elided text went: `observations_full` was already in the live trace and
`recall` already reads `reasoning_trace.jsonl`, so the retrieval path existed
and nothing pointed at it.

**What that was worth, measured on one model.** DeepSeek-V4-Flash, same
fixture:

| | provider | ceiling | wall | calls | s/call | evidence fields | pointing nowhere |
|---|---|---|---|---|---|---|---|
| `r1_deepseek_1` | DeepInfra | 8192 | 1432s | 43 | 33.3 | 18 | **3** |
| `b2_deepseek_or` | Baidu | 16384 | **226s** | 28 | **8.1** | 23 | **0** |

**6.3x faster and now quicker per call than grok** (8.1s against 11.1s), with
citations more than doubled — 12 line references became 29, 19 quotes became
40. **The three pointerless fields were our ceiling, not its citation habit**,
and they would have eliminated it under the screening bar. Two variables moved
at once, so provider and ceiling cannot be separated from this.

### Provider selection is not what the benchmarks measure — 2026-08-26

**Structured output eliminates the fast endpoints.** Probing every fp8
provider of DeepSeek-V4-Flash-0731 with the real payload (`json_schema` +
`reasoning_effort`):

    baidu 69 tok/s · parasail 58 · siliconflow 50 · deepinfra 49 · akashml 46
    gmicloud, novita, streamlake, coreweave, baseten -> 404, no json_schema

The two fastest on paper — coreweave ~166 tok/s, gmicloud ~117 — refuse
`response_format` outright, and published throughput rankings are measured
without it. Among providers that accept our payload the spread is 46-69 and
DeepInfra sits mid-pack. `response_format` is load-bearing (it is what stopped
luna emitting its answer as JSON instead of an action, 160 `unknown tool
None`), so this is a real constraint rather than a setting to drop.

**`supported_parameters` from the endpoints API is not reliable.** It lists
`response_format` for gmicloud, which then 404s on a `json_schema` request.
Probe with the real payload; the endpoints API is for shortlisting.

**Three ways of asking for one model give three different artifacts.** On
2026-08-26:

    deepseek-v4-flash-0731, unpinned        -> OpenInference, fp4
    deepseek-v4-flash (base) + coreweave    -> CoreWeave, fp8, snapshot 20260423
    deepseek-v4-flash-0731 + gmicloud/fp8   -> GMICloud, fp8, snapshot 20260731

An unpinned run would have used 4-bit weights; the base id an April build.
Pin by TAG (`baidu/fp8`), which fixes precision as well as provider, and turn
fallbacks off — a failover mid-run can change quantization without changing
the model id.

**Rate limits are per-model shared pools, not per-account.** Two runs launched
together on one DeepInfra key: Nemotron-3-Super took eight 429s between
12:36:54 and 12:52:23 while DeepSeek took none. A 429 recorded on 2026-08-22
named it directly — `limit_source: upstream_provider_shared_pool`. OpenRouter
does throttle on its own account, but only on `:free` variants (20 req/min,
which a 20-43 call audit would trip); paid models pass upstream limits
through. `error.metadata.provider_code` marks an upstream refusal and
`X-RateLimit-*` headers a gateway one. None of this is captured in a run
record today.

### Two things the runs exposed

**The instrument could not run at all.** `scenario.yaml` pointed `inspect_repo`
at `audit/`, a tree the workflow refactor moved, so `chat_loop` refused to
launch. Every audit after `w1_grok_2` would have died at launch and none was
attempted, so the refactor went unexercised for a day. Fixed at `1e39ba83`,
pointing at `workflows/claims_audit/method` — METHOD.md stays at the root of
what `inspect` reads, and `engagements/` stays outside a fence that now guards
delivered client reports as well as the answer key.

**Run output stopped being tracked.** `measure/fixtures/dataroom/results/` is
gitignored and 27 stale-instrument runs were deleted; `m1_qwen_2` is kept as the
only report on disk that trips §4.0, and reports the tests need are copied into
`tests/fixtures/audit_review/`. The cost, named: the campaign table above cites
paths that now resolve only on the machine that made them.

### Do this first — revised 2026-08-26

**Do not review the other eight runs yet.** That was the instruction here, and
running it against the current reviewer would produce eight more numbers with
the same defect. Fix the instrument first, in this order:

1. **Add `[indeterminate]` to REVIEW.md §6, and admissibility as §4.0** with
   the authority to end the run. See the retraction above.
2. **Decide the citation contract in METHOD §5** — the one line stating which
   coordinate system the report uses, and whether line numbers are asked for at
   all given that the subagent boundary strips them from most documents.
3. **Re-review `m1_qwen_2`.** Its `review/` directory must be moved or deleted
   first; the runner refuses a second review in place, deliberately.
4. **Then the other eight**, `workflows/audit_review/runner.py --run <dir>
   --model measure/models/grok_4p6.yaml`, about five minutes each.

Only then decide what the m1 table means. Options are unchanged: report
supported-ratio alongside the existing vector, gate on it, or retire the
campaign.

### The rest, in rough order

1. **§9's number is not extracted mechanically.** "Supported 14 of 15, PASS" is
   prose in `summary.md`. A small parser makes it the dev benchmark that was
   the point of §9. Now that two reviews exist, the shape is known rather than
   guessed.
2. **REVIEW.md §7 is too conservative.** The reviewer declined to flag "all 48
   claims were examined" on a run that never closed a claim surface — "not
   re-litigated here as a new finding about the business". Correct instinct
   about §2, wrong result: a coverage statement the record does not support is
   an exception, and §7 must say so more plainly.
3. **Review a big report.** ChatterMate, 42 findings. Both reviews so far fit
   every finding and citation into one context, so checking needed no tool
   calls after the fetch — four `inspect` calls, all retrieval. That will stop
   holding at some size, and where it stops is worth knowing.
4. **`[incoherent]` is off the table** and still unresolved: cross-finding
   contradiction is a property of the set, not a per-finding verdict.
5. **33 worlds, 1.4 GB in `scenarios/`**, genuinely discardable since runs read
   their own archives. Nothing deletes them because nothing ever did.
6. **Runs live in two places** — 28 under `measure/fixtures/dataroom/results/`,
   new ones under `engagements/*/runs/`. The old ones were left because their
   paths are in the campaign table above.

### Two properties established today, worth not breaking

**A run directory is self-sufficient.** Delete its world and it still scores
and still answers questions. Verified by deleting one.

**The model is not the reviewer's problem.** Use the best available; diversity
only helps between peers, and a weaker reviewer's disagreements are mostly its
own errors. Review is currently about as expensive as the audit (309s vs 249s),
which should improve with report size: audit cost tracks the claim surface,
review cost tracks findings.

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
