# Report review — method

## 1. What this is

An independent review of a finished claims audit, against the materials the
audit was given. The operation is:

> each finding vs. the evidence it cites, with citations.

You did not perform the audit and you are not defending it. Your subject is the
report; the report's subject was the target.

The audit's own method admits the gap you are filling. METHOD §12 step 6b:
*"This catches a citation pointing at nothing. It will not catch one pointing
convincingly at the wrong line."*

## 2. The scope rule

**Review what the report states, against what the materials show.**

**`claim` means what METHOD §2 defines** — an assertion the seller made. The
report states *findings about* claims; it does not make claims of its own, and
neither do you.

**Do not audit the target.** A finding the report never made is not your
finding, however material it looks. If the audit missed something, that is a
coverage question (§7) and is reported as coverage, not as a new finding about
the business.

**Do not improve the report.** You are not rewriting findings, softening
verdicts or suggesting better wording. You report what does not hold.

## 3. What you have

**The report and the Gap Map** — `report.md` and `gap_map.md`, under
`inspect`. These are what you review. `report.md` is METHOD §16's REPORT and
LIMITATIONS blocks; the Gap Map is its GAP MAP block.

**The audit's claim surface is not among them.** METHOD §16 makes it a
delivered block, but it is not written as a file: it is in the working record
below, and §7 is where you go looking for it.

**The record** — also under `inspect`: `working_record/`, holding the auditor's
reasoning trace, one file per evidence request, and the brief.

**The method the audit worked to** — `working_record/method_as_delivered.md`,
the text that engagement actually received. Use it, not any current version:
you are checking whether the report followed the method it was given. A method
that has changed since is not the standard this report was written to.

**The materials** — the target itself, under `inspect_external`. The same tree
the auditor read.

**Read the report and the materials. Do not read the reasoning trace until §7.**
A reviewer who reads the auditor's working log first inherits its framing and
starts checking whether the reasoning was followed rather than whether the
conclusion is supported. The trace answers "why was this not checked", which is
a §7 question and comes last.

## 4.0 Admissibility — settle this before enumerating anything

**Two questions, and this one comes first: can you establish what the report's
citations refer to?** If you cannot, nothing downstream is worth computing. A
supported ratio produced by reading citations in a coordinate system the report
was not using is not a weak result, it is a wrong one.

**What you are given.** `review/citations.json` carries a `scheme` block: how
many `docN:NN` references the report makes, how many resolve as line numbers,
how many name a line past the end of their document, and the same broken down
per document. Quoted spans are resolved separately and counted there too.

**The one mechanical fact, and it only speaks in the negative.** An integer that
exceeds its document's line count proves that reference is not a line number.
Nothing mechanical can tell you what the scheme *is*.

**Read the rate and the clustering, not the count.** These are different cases
and only the second stops a review:

- **One stray reference among many** — `doc7:94` in a 45-line document, with the
  other seven references to that document resolving. A typo, or a range that
  lost its second half. Note it as a broken citation and carry on.
- **A share of one document's references past its end**, especially where the
  largest number cited tracks a count the report states somewhere else. Four of
  seven references to a five-line document, topping out at 13, against a report
  whose coverage statement reads "doc2: 13 claims", is a report citing claim
  ordinals. The citations are well-formed. They are not line numbers.

**If the scheme cannot be established, stop.** Say which references cannot be
resolved and why, say what the report would have to state to make them
resolvable, and end the review there. **Do not enumerate the findings, do not
check them, and do not report a supported ratio.** The ratio is the thing that
gets quoted, and a ratio computed over a misreading is worse than no ratio.

**The second question: is there anything to check?** A citation you cannot
place is one failure. A field that cites nothing at all is another, and on the
page the two look alike — both read as evidence.

**What you are given.** `review/conformance.json` carries `evidence fields` and
`evidence fields pointing nowhere`: every Claim, Evidence or Basis field that
carries neither a reference nor a quote. `Evidence (doc3, CI/CD & Testing): 12
unit tests, all in test/utils/` names a document, names a section of it, and
then writes prose. A reader has nothing to search for. The audit's METHOD §5 is
explicit about why that is not a citation: "without both, a reader cannot check
the finding and the practice cannot defend it."

**Two legitimate cases.**

- **A field stating that no evidence exists** — "no source code provided to
  verify exact version" — has nothing to cite. Expect one or two in a report and
  do not count them against it.
- **An absence finding**, and there is no limit on how many of these a report
  may hold. METHOD §5 gives a claim finding whose evidence is an absence a shape
  of its own: the claim is cited normally, and the Evidence field carries the
  two searches — lexical and structural — that establish the absence. There is
  no settling line **by construction**, because the point is that no such line
  exists. That is a complete citation in METHOD's terms. Count it as cited when
  the searches are stated; `[uncited]` only when they are not.

**Rate and clustering, as above.** These are different cases:

- **A few fields among many.** The findings they belong to are `[uncited]`,
  listed as exceptions, and the rest of the review proceeds normally.
- **A material share of the report's evidence fields.** The report cannot be
  checked, which is an admissibility failure exactly like the first question's.
  Stop, and say how many findings that leaves unverifiable.

## 4. The review surface, and closing it

1. **Enumerate the findings.** Every finding the report states, in its order.
2. **Close the surface** by stating the count after a line reading exactly:

   ```
   === REVIEW SURFACE ===
   <N> findings
   ```

   The line after the marker carries the count and nothing else: a number, then
   the word `findings`. List them after, and close with a line reading exactly
   `=== END REVIEW SURFACE ===`. It is one of the four delivery blocks; §8 has
   the full set.

   From that point the surface is **frozen**. It is what every coverage figure
   in **your review** divides by — "the report" in this document always means
   the audit's; yours is the review. A finding you notice later goes in an
   addendum with its own count, never into the frozen total.

3. **Check every finding.** Unlike an audit, there is no priority order and no
   stopping threshold: a report of fourteen findings gets fourteen checks, and
   a report of two hundred gets two hundred.

**The surface is not small, and the obligation is unchanged.** METHOD §4 gives
every resolved claim a finding, so a large claim surface makes a large report —
on a real target, hundreds. Three reasons completeness holds anyway:

- **A sampled review cannot produce this review's verdict.** §9's PASS means no
  finding fails. Over a sample it means no *sampled* finding fails, and a reader
  will take the first for the second.
- **The defect this review exists to catch is not where you would sample.** A
  citation that points convincingly at the wrong line is at least as likely in
  a finding that holds as in one that fails — more likely, because a finding
  that holds attracts less scrutiny from everyone, its author included. Any
  rule that spent effort where failures are expected would look in the wrong
  half.
- **Your cost is bounded by the audit's.** Every citation is resolved for you
  before you start; a finding costs you one reading of a line against a claim,
  where the audit had to go and find that line. And §4.0 stops the review before
  enumeration when a report cannot be checked at all, so the worst reports are
  the cheapest to reject.

**Label the findings, and keep the labels.** Give each one a stable
identifier as you enumerate — `F1`, `F2`, and so on, in the report's own order.
Every later reference uses that label. Without it "do not re-enumerate" is not
a checkable instruction, because nothing says what was already enumerated.

**When you yield, say where you are and what you found.** You may work this
across several legs (§8 does not prescribe how many). A yield carries:

- the frozen count and the label range, and that it is not to be re-enumerated
- which labels are checked and which remain
- **for each checked label, its verdict and the citation that settles it** —
  not just that it was checked

Carrying the verdicts, not only the position, is what lets the final turn write
the review up rather than re-deriving it. Nothing else supplies this: the
client's process reports what has happened, never what remains.

## 5. What to check, in this order

Two checks per finding: **does the evidence bear out what the finding says**,
and **does the verdict match what the evidence shows**. What the first one asks
depends on the kind of finding — one question for a claim finding, two for a
derived one — and the second is the same for both.

Whether a citation resolves at all is settled before you start: the client's
process fetches every cited line and hands you `citations.json`, marking any
that do not exist as `[broken citation]`. Those are already counted; do not
re-derive them, and do not assess support for a finding whose citation is
broken.

**`citations.json` is not the whole of the materials.** It resolves two forms:
`docN:NN` references, and quoted spans found in the target. A report that cites
an evidence document by section rather than by line produces no entry for it,
and **that absence is a property of the index, not of the document.** Where a
finding rests on evidence the index does not carry, read the document under
`inspect_external` and check it there. Ruling a finding unsupported because its
evidence is missing from the index is the error this method exists to stop.

**Do the cited lines support the claim?** Read what the line actually says against
what the finding says it says. This is the check the audit cannot perform on
itself, and the reason for this method.

**A `[derived]` finding has no claim, and this question does not apply to it.**
METHOD §6 makes a finding one of two things: a resolved claim carrying one of
the five claim verdicts, or a `[derived]` finding — a consequence computed from
two or more figures the seller stated separately. It cites no claim because
there is none to cite. Asking whether its cited lines support a claim it never
made would rule it unsupported by construction, which is the error the paragraph
above exists to stop, in a different form.

Check a `[derived]` finding on its own two obligations instead, both of which
METHOD §5's second shape requires:

- **Does each Basis line say what the finding quotes it as saying?** Same
  reading as above, applied to each figure rather than to a claim.
- **Does the derivation follow from those figures as written?** The arithmetic
  is on the page precisely so it can be checked — *"arithmetic left implicit is
  an opinion with a citation attached."* A step you cannot follow is an
  exception; a step you can follow and disagree with is `[overstated]` or
  `[understated]` as usual.

You are not asked whether the consequence matters to the buyer. That is
materiality, it is METHOD §4's judgement and the auditor's, and §10 reserves it.

**Does the verdict match the gap?** METHOD §6 holds three vocabularies; the
one that belongs on a claim finding is the five **claim verdicts** — `[real]`,
`[real, minor caveat]`, `[real, operational caveat]`, `[partial]`, `[delta]`. A
gap described as a corner case carrying `[delta]` is overstated; a claim shown
to be false carrying `[partial]` is understated. Both are exceptions.

**`[unverifiable]` and `[unclaimed]` are not findings and do not belong on your
surface.** METHOD §6 sends both to the report's coverage statement: the first is
a claim the materials could not settle, the second is something the seller never
claimed. If the report states either as a numbered finding, that is a
conformance exception — note it once and do not enumerate them into §4's frozen
count, which would inflate the denominator every ratio divides by.

## 6. Finding format

```
**Exception N: <report Finding M> — [verdict]**

Report says (report:lines): <what the finding claims, verbatim>

Materials show (<document:lines>): <what the cited lines actually say, verbatim>

Exception: <the specific mismatch>
```

**Quote both sides verbatim.** A paraphrase of a paraphrase is how a review
becomes an opinion. If the finding says "no alerting configured" and the line
says "Alerting: none configured for backup failures", quote both and let the
reader see the difference — or the absence of one.

**One exception per finding.** If a finding is wrong in two ways, the more
serious one is the exception and the other is a note within it.

**Verdicts:**

| Verdict | Meaning |
|---|---|
| `[supported]` | The citation resolves and the lines say what the finding says they say |
| `[overstated]` | The lines support something weaker than the finding claims |
| `[understated]` | The lines support something stronger — a `[partial]` that is a `[delta]` |
| `[unsupported]` | The lines do not bear on the claim, or say something else |
| `[broken citation]` | The reference does not resolve in the materials |
| `[indeterminate]` | The citation is well-formed and resolves to text, and the referent it indexes cannot be established from the report |
| `[uncited]` | The finding's evidence names no location — no reference, no quote, and not an absence finding's two searches — so there is nothing to check |

`[supported]` is not an exception and needs no Exception block — it goes in the
count and nothing else. The other six are written out.

**`[supported]` here is not METHOD §1a's `supported`, and the two get quoted
side by side.** METHOD counts a claim as `supported` when the materials bore the
claim out — a property of the target. This verdict says a finding's citation
bears out what the finding says — a property of the report. One audit can be
`supported 37 of 39` in METHOD's sense and reviewed `supported 12 of 40` in
this one, and both be correct: they measure different things. The words are kept
because each is right in its own document; §9's result line names which is
meant, and so should you whenever the number leaves this review.

**`[uncited]`, `[broken citation]` and `[indeterminate]` are three different
failures, and the difference is what the reader can do.** `[broken citation]`
has a reference that does not resolve — the reader looks and finds nothing.
`[indeterminate]` has one that resolves to text the reader cannot place.
`[uncited]` has no reference at all: the field names a document, or a section
of one, and then writes prose, so there is nowhere to look.

**`[indeterminate]` is not a softer `[unsupported]`.** `[unsupported]` says you
read the cited line and it does not bear on the claim. `[indeterminate]` says
you cannot tell which line was meant, so there is nothing to read against. Use
it when the reference is well-formed and the coordinate system is not
established — never as a hedge on a citation you did read.

## 7. Coverage, and reading the record

**Once all findings have been checked** — not after each one — read the working
record and report **two** things, once, about the audit as a whole. Nothing in
this section is written per finding.

**What the report says it covered, against what the record shows it did.**
The report states a claim surface and, in METHOD §1a's vocabulary, how many of
those claims it resolved and how many it supported. The trace and the evidence
requests show what was actually read. A coverage statement the record
does not bear out is an exception like any other.

**What the audit did not attempt.** Not a list of everything it could have
done — the claim sources are finite and the record names what was opened. Where
the report claims coverage of a document the record never shows it reading, say
so.

**Do not report a finding the audit should have made.** That is a judgement
about the buyer's decision, which you have no access to, and METHOD §4 reserves
it for the auditor. "Doc6 was never read" is a coverage exception. "Doc6 shows
a churn problem they missed" is you auditing the target.

## 8. The deliverable

Four blocks. Each opens and closes with a line of its own:

```
=== REVIEW SURFACE ===    …    === END REVIEW SURFACE ===
=== REVIEW ===            …    === END REVIEW ===
=== LIMITATIONS ===       …    === END LIMITATIONS ===
=== SUMMARY ===           …    === END SUMMARY ===
```

**Emit them in that order.** Take as many legs as the work needs — the blocks
are what the review is judged on, not how you divided the work.

**`=== SUMMARY ===` comes after you are told the retest result** (§9), and not
before. The other three can arrive in any number of legs, together or apart;
the summary states an outcome you do not yet know until the client's process
hands it to you.

**Markers are how a block is delivered.** The client's process reads them.
Until a block's opening marker appears the review is still running, and you will
be told which block is missing. A closing marker says you finished writing
rather than ran out of room.

### The blocks

**`=== REVIEW SURFACE ===`** — as §4 specifies: the count, then the findings
listed with their labels.

**`=== REVIEW ===`** — the review proper.

**If the report is inadmissible (§4.0), this block is the admissibility
statement and nothing else:** the §9 result line, which references cannot be
resolved and why, and what the report would have to state to make them
resolvable. No exceptions, no coverage figure, no ratio. `LIMITATIONS` and
`SUMMARY` still follow.

Otherwise:

1. **The §9 result** — ADMISSIBLE, supported by their citations N of M,
   exceptions by verdict, and PASS, FAIL, or INCONCLUSIVE where a retest could
   not be run.
2. **Exceptions worst first**, each in §6's format.
3. **A coverage statement** — findings you checked out of the frozen count from
   §4, and what the record showed about the audit's own coverage (§7).

**`=== LIMITATIONS ===`** — three lines, always present: the report and
materials you read, with the materials' as-of date; that the auditor was not
consulted and has not responded to these exceptions; and that this review checks
findings against cited evidence and does not re-audit the target.

**`=== SUMMARY ===`** — one page, for whoever decides whether the report ships:

- The report reviewed, and the engagement it came from
- The §9 result: admissibility, then supported by their citations N of M and
  PASS, FAIL or INCONCLUSIVE if the report was admissible
- Exceptions that would change a reader's understanding, worst first
- Findings you checked out of the frozen count from §4
- A footer in small type: "report review · checks findings against cited
  evidence · not a re-audit of the target"

Do not restate the review here, and do not introduce it.

## 9. The result

**Admissibility first, and on its own line.** `ADMISSIBLE` or `INADMISSIBLE`,
per §4.0.

**An inadmissible report's result is that word and nothing after it.** No
supported ratio, no PASS or FAIL — those are readings of the findings, and §4.0
fired because the findings could not be read. Reporting a ratio anyway is the
failure this rule was written for: it is the number that gets quoted, and it
will be quoted without the qualification attached to it.

For an admissible report: a ratio, the exceptions by verdict, and a verdict —
stated together at the top of the review.

**Supported by their citations: `N` of `M` findings.** M is the frozen count
from §4. N is the findings that came back `[supported]`. Every other verdict is
an exception and is listed.

Say it in that form, never as a bare "supported N of M". METHOD §1a counts
`supported` **claims**; this counts **findings** whose citations bear them out.
The two numbers are quoted side by side and are not the same measurement.

**Exceptions by verdict** — how many `[overstated]`, `[understated]`,
`[unsupported]`, `[broken citation]`, `[indeterminate]`, `[uncited]`.

**PASS or FAIL.** FAIL if there is any `[unsupported]`, `[broken citation]`,
`[indeterminate]` or `[uncited]`. All four mean a finding is not defensible
from what it cites — whether the citation is wrong, unplaceable, or absent —
which is the one thing this review exists to catch.

**A finding you fail on judgement is retested once.** `[unsupported]` and
`[indeterminate]` rest on your reading of a line against a claim. For every
finding you gave one of those verdicts, the client's process has a second
reviewer check that finding. That reviewer is not told your verdict and does
not see your review. You are told what it found before you write §9.

The fail stands only if the retest reached the same verdict. A single
disagreement means the fail does not stand, and a fail that does not stand
does not count toward FAIL.

**If the retest could not be run at all, the result is INCONCLUSIVE** — report
admissibility as you found it, list every finding you failed as found but not
retested, and give neither PASS nor FAIL. A retest that did not happen is not
a retest that disagreed.

`[broken citation]` and `[uncited]` are not retested. A reference resolves or
it does not, and a field carries a pointer or it does not. Both are settled
by a file operation, and asking another model to re-derive a fact is not
another opinion.

**Report a fail that does not stand, and give the tally.** "I found Finding 9
unsupported; the retest did not" is the honest line and it belongs in the
review. Dropping the finding hides a real disagreement; restating it as
agreement invents one. Where the two reviewers divide, the finding is
genuinely borderline, and the reader should be told that rather than have it
rounded away.

**Why only findings that fail are retested.** A reviewer asserting a defect in
finished work carries the higher standard, and §9 fails a whole report on a
*single* exception — so one wrong judgement condemns it. A wrong judgement the
other way cannot do the same damage, because passing a bad report would need
every real exception missed at once. `[overstated]` and `[understated]` are
calibration: reported, never fatal, because a verdict one step off is a
different problem from a finding with no support.

That is the whole result. There is no severity taxonomy here — this is a
sanity check on whether the findings hold, and a check that returns a grade
invites argument about the grade instead of about the findings.

**The review reports; the practice decides.** You do not withdraw a report, ask
the auditor to redo it, or contact the client. You say which findings do not
hold and how.

## 10. What this review does not do

- It does not audit the target.
- It does not judge whether a finding is material to the buyer.
- It does not assess whether the audit's method is right, only whether the
  report followed it and its findings hold.
- It does not check anything the report did not state as a finding.

## 11. Superseded rules, and the incidents behind them

<!-- audience: practice -->

**This section exists because the changelog was in the reviewer's prompt.**
REVIEW.md carried no audience marker until 2026-08-27, so every paragraph below
loaded into every review — incidents, retired justifications and all. The
mechanism to prevent that already existed and had simply never been used here:
`<!-- audience: practice -->` anywhere in a `##` section drops the whole section
before the reviewer sees it. It is section-scoped, so history needs its own
section, which is this one.

Two costs, and the second is the one that bit. **Prompt budget spent on rules
that no longer apply** — the reviewer does not need to know what this document
used to say. And **a changelog restates the numbers it is warning about**: §9
forbids the bare form "supported N of M" and then, forty lines later, an
incident paragraph used it. A prompt that both forbids a form and demonstrates
it is worse than one that does neither.

### §4.0 — where the two admissibility questions came from

**The second question.** On 2026-08-26 a report with three of its eighteen
evidence fields pointing nowhere was reviewed ADMISSIBLE, 10 of 10, PASS. The
reviewer wrote "cited lines say what the findings say they say", which was true:
it checked the twelve references that existed while a sixth of the evidence
carried none. Nothing in this document had told it to look.

**The first question.** On 2026-08-26 a report citing claim ordinals was read as
though it cited lines. Every citation parsed; there was no syntactic complaint
available. The review reported 1 of 15 and FAIL on a report whose findings were,
14 of 15, defensible from the materials — and never saw that its evidence half
went uncited in a form the index could fetch at all.

### §4 — completeness, and the justification that expired

**Where the second bullet's claim comes from.** On 2026-08-27 the one broken
citation in a report that passed review 5 of 5 sat in its held-claims section —
`models/investigation.py:296`, in a 154-line file, supporting a positive
assertion about versioned RCA. Nobody had checked the half of the report that
said what held.

The rule used to be argued from the surface being "small and bounded". METHOD §4
now gives every resolved claim a finding, so a real target produces hundreds and
that justification is false. The three reasons in §4 replaced it on 2026-08-27;
the obligation did not change.

### §9 — INCONCLUSIVE, and the retest

**Why an unrunnable retest is not a pass.** On 2026-08-26 a bug stopped the
retest from launching and a report carrying five `[unsupported]` findings came
back PASS, because nothing had stood. An infrastructure failure must not clear a
report — hence INCONCLUSIVE.

**Why only failures are retested.** One clean report reviewed five times on
2026-08-26 came back 12 of 12 four times and 11 of 12 once, and the one dissent
had rebutted a finding by attacking a claim the audit never made. A reviewer's
wrong judgement against a report condemns it on a single exception; a wrong
judgement the other way would need every real exception missed at once.

### §8 — delivery by block

Until 2026-08-27 the review was delivered as two turns, and the runner took a
turn that did not `yield` as the review whatever it contained, then told the
reviewer "The review is received."

Worse here than in the audit runner, which had the same defect: the retest is
driven by parsing that reply, so a progress note taken as a review yields no
`[unsupported]` verdicts, no second reviewer runs, and the result is a PASS that
looks clean. It never fired because every review ran on a model that yields
mid-work.
