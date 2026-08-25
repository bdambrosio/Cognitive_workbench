# dataroom — campaign state

Started 2026-08-23, on the workflow harness. Everything before it was
discarded, deliberately — see "Why every earlier run was discarded".

## Scored runs

**All rows wiped 2026-08-24.** Every run in the campaign was made at
temperature 0.7 — the code default in `chat_loop.py`, inherited by every
scenario and every arm because none of them set one. That was never the
intended house setting, and nothing in the config, the arm files or the
runner would have revealed it: `run.py --help` reports the scenario default
as if it were a decision. Temperature is now a per-model setting (see
`docs/model-settings.md`), and no result predating that is trusted.

The three DeepInfra arms are the sharpest illustration. Their yaml carries a
comment stating temperature 1.0 as the publisher's number, explains that it
is passed by the runner rather than set in the file, and warns in as many
words about "the silent second variable this exists to avoid" — and then the
runner was invoked without it. Documenting a setting in a file the runner
does not read is not setting it.

*(No table. The next campaign starts from zero, on one frozen instrument,
temperature and top_p resolved per model and recorded in every `run_meta`.)*

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
between arms; it only ever reported label formatting.

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

## Pick arms by output tokens per call, not parameter count

Kept as a selection rule; the model that produced it is dropped.

Qwen3.8-2.4T-A95B spent 2,902 completion tokens answering a one-line question
against a 6k-character prompt, where DeepSeek used 18. At ~21 tok/s that is
140 seconds per call, and this architecture makes 35-187 calls per audit.
Neither throttling nor cache — both were tested. **A model that cannot answer
briefly cannot be used in a call-heavy agent, whatever its parameter count.**
Measure tokens-per-call on a trivial prompt before committing an arm to a
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
(`scenarios/audit.yaml` -> `workflow: audit/METHOD.md`) instead of being
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
| iteration cap 12 -> 16 | an arm reading one document per call can finish ingesting and still have budget to work |
| `inspect` geofenced to `audit/` | the answer key and prior arms' reports are out of reach |
| empty-answer sentinel fixed | a subagent returning nothing says so, instead of reporting `OK` |
| Gap Map marker as the stopping rule | a turn that stops with a stated plan is no longer scored as a finished audit |
| engagement ledger on each `continue` | legs, minutes and documents-opened are carried by the runner |

Comparing across that line would be comparing two instruments. Those runs did
their job — every fix above was found by running them — and they remain in
git history if a specific claim ever needs checking.

## Pick up here

Everything below the line is instrument work. No campaign runs until it is
done, because a run made before it is a run that will be discarded.

1. **Land the model-settings layer.** `src/chat/model_params.py` as the single
   source of truth (per-model temperature, global `top_p` 0.95), resolved and
   enforced in `_ChatBackend`, with the `0.7` literals deleted so an unknown
   model raises instead of inheriting. Plus `docs/model-settings.md` for
   provenance, a `CLAUDE.md` rule, and a test that walks every arm and
   scenario asserting each resolves. See "Scored runs" above.
2. **Remove the `top_p=1.0` pin in `measure/regrade.py`.** The grader is not
   exempt from the global setting.
3. **Freeze the instrument, then run.** Clean tree, recorded `git rev-parse
   HEAD` per run, no method edits mid-campaign. Two campaigns have now been
   discarded for drifting under their own runs.
4. **n=5 per arm, all on one frozen instrument.** Arms: grok-4.6,
   Qwen3.8-27B local, gpt-5.6-luna.
   Nemotron excluded (two valid runs in seven attempts, cannot assemble a
   report). DeepSeek is the obvious fourth, held back only for run time.
5. **Persist NOTE lines in the stored working log.** `chat_loop.py:1667`
   keeps only `$step*` labels, so the budget nudge is stripped from every
   trace and no post-hoc analysis can say whether an agent was warned before
   it ran out of iterations.

## What the 2026-08-24 harness fixes were supposed to change

The runs behind this are deleted; the observations are kept because they are
what the fixes were aimed at, and a fix nobody stated a prediction for cannot
be shown to have worked. **Predictions, written before the re-run.**

Pre-change behaviour, one run per arm, all at the corrected per-model
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

**3. Contract inlining (same fix, opposite arm).** Luna compensated correctly
for the same gap by pasting ~3,000 characters of taxonomy and finding
structure into every composition instruction.

> **Predicted:** Luna's `process_text` instructions get materially shorter,
> because the contract is now in the sub-call's context. **This is the cheap
> falsifiable one** — instruction length is mechanical to measure.

**A caution against reading these as model comparisons.** n=1 per arm, and
the three arms differ in temperature by design. What is being tested is
whether two harness defects stopped happening, not which model is better.

## Subagent contract compliance — counted, not repaired

Added to `score.py`'s mechanical block 2026-08-24. Free, no grader, no change
to anything the agent sees.

The subagent contract is one line: finish by emitting `respond` with your
answer in `text`. A call that returns nothing has failed it. The parent sees
only `EMPTY:`, retries narrower, and the cost lands in the iteration count
where nothing distinguishes it from thorough work.

First measurement, one run per arm:

| arm | `inspect_external` calls | returned NO answer | exits |
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

Seven runs (three arms) deleted when METHOD.md changed — §8's working recap
and §12's delta-confirmation removed as unperformable, `[real, with a
structural note]` removed as unused, and `run.py`'s brief corrected where it
still instructed both removed procedures. Everything they measured is
downstream of that.

Kept because they are the only observations of these behaviours:

| arm | n | threshold | Tier 2 | subagent no-answer | words |
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

- An arm ended a turn with a stated-but-unexecuted plan — "I will now begin
  working the priority order" — which its own character block forbids.
- An arm received the budget nudge at iteration 10 of 12 (`react.py:418`:
  *do NOT start anything new, emit `yield` NOW*) and then started new work
  three more times.
