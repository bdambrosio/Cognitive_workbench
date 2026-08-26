# Report review — method

## 1. What this is

An independent review of a finished claims audit, against the materials that
audit examined. The operation is:

> each finding vs. the evidence it cites, with citations.

You did not perform the audit and you are not defending it. Your subject is the
report; the report's subject was the target.

The audit's own method admits the gap you are filling. METHOD §12 step 6b:
*"This catches a citation pointing at nothing. It will not catch one pointing
convincingly at the wrong line."*

## 2. The scope rule

**Review what the report claims, against what the materials show.**

**Do not audit the target.** A claim the report never made is not your finding,
however material it looks. If the audit missed something, that is a coverage
question (§7) and is reported as coverage, not as a new finding about the
business.

**Do not improve the report.** You are not rewriting findings, softening
verdicts or suggesting better wording. You report what does not hold.

## 3. What you have

**The report and the Gap Map** — the delivered artifacts, under `inspect`.
These are what you review.

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

**Where this rule came from.** On 2026-08-26 a report citing claim ordinals was
read as though it cited lines. Every citation parsed; there was no syntactic
complaint available. The review reported supported 1 of 15 and FAIL on a report
whose findings were, 14 of 15, defensible from the materials — and never saw
that its evidence half went uncited in a form the index could fetch at all.

## 4. The review surface, and closing it

1. **Enumerate the findings.** Every finding the report states, in its order.
2. **Close the surface** by stating the count after a line reading exactly:

   ```
   === REVIEW SURFACE ===
   <N> findings
   ```

   The line after the marker carries the count and nothing else: a number, then
   the word `findings`. List them after.

   From that point the surface is **frozen**. It is what every coverage figure
   in your report divides by. A finding you notice later goes in an addendum
   with its own count, never into the original total.

3. **Check every finding.** Unlike an audit, there is no priority order and no
   stopping threshold: a report of fourteen findings gets fourteen checks. The
   surface is small and bounded, so partial coverage has no excuse.

**Label the findings, and keep the labels.** Give each one a stable
identifier as you enumerate — `F1`, `F2`, and so on, in the report's own order.
Every later reference uses that label. Without it "do not re-enumerate" is not
a checkable instruction, because nothing says what was already enumerated.

**Say where you are, and what you found.** You will work this across several
turns. End each turn by yielding with:

- the frozen count and the label range, and that it is not to be re-enumerated
- which labels are checked and which remain
- **for each checked label, its verdict and the citation that settles it** —
  not just that it was checked

Carrying the verdicts, not only the position, is what lets the final turn write
the review up rather than re-deriving it. Nothing else supplies this: the
client's process reports what has happened, never what remains.

## 5. What to check, in this order

For each finding, two questions. Whether a citation resolves at all is settled
before you start — the client's process fetches every cited line and hands you
`citations.json`, marking any that do not exist as `[broken citation]`. Those
are already counted; do not re-derive them, and do not assess support for a
finding whose citation is broken.

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

**Does the verdict match the gap?** METHOD §6 defines the vocabulary. A gap
described as a corner case carrying `[delta]` is overstated; a claim shown to
be false carrying `[partial]` is understated. Both are exceptions.

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

`[supported]` is not an exception and needs no Exception block — it goes in the
count and nothing else. Only the other five are written out.

**`[indeterminate]` is not a softer `[unsupported]`.** `[unsupported]` says you
read the cited line and it does not bear on the claim. `[indeterminate]` says
you cannot tell which line was meant, so there is nothing to read against. Use
it when the reference is well-formed and the coordinate system is not
established — never as a hedge on a citation you did read.

## 7. Coverage, and reading the record

**Once all findings have been checked** — not after each one — read the working
record and report **two** things, once, about the audit as a whole. Nothing in
this section is written per finding.

**What the report says it covered, against what the record shows it examined.**
The report states a claim surface and a number verified. The trace and the
evidence requests show what was actually read. A coverage statement the record
does not bear out is an exception like any other.

**What the audit did not examine.** Not a list of everything it could have
done — the claim sources are finite and the record names what was opened. Where
the report claims coverage of a document the record never shows it reading, say
so.

**Do not report a finding the audit should have made.** That is a judgement
about the buyer's decision, which you have no access to, and METHOD §4 reserves
it for the auditor. "Doc6 was never read" is a coverage exception. "Doc6 shows
a churn problem they missed" is you auditing the target.

## 8. The deliverable

Two documents, in two turns, in this order. A turn carries its document and
nothing else.

**Turn one: the review.**

**If the report is inadmissible (§4.0), turn one is the admissibility statement
and nothing else:** the §9 result line, which references cannot be resolved and
why, what the report would have to state to make them resolvable, and the
limitations statement. No exceptions, no coverage figure, no ratio. Turn two is
still the summary.

Otherwise:

- The §9 result: ADMISSIBLE, supported N of M, exceptions by verdict, PASS or FAIL.
- Exceptions worst first, each in §6's format.
- A coverage statement: findings checked out of findings enumerated, and what
  the record showed about the audit's own coverage (§7).
- **The limitations statement**, after a line reading exactly:

  ```
  === LIMITATIONS ===
  ```

  Three lines, always present: the report and materials examined, with the
  materials' as-of date; that the auditor was not consulted and has not
  responded to these exceptions; and that this review checks findings against
  cited evidence and does not re-audit the target.

**Turn two: the review summary.** One page, for whoever decides whether the
report ships:

- The report reviewed, and the engagement it came from
- The §9 result: admissibility, then supported N of M and PASS or FAIL if the
  report was admissible
- Exceptions that would change a reader's understanding, worst first
- Findings checked out of findings enumerated
- A footer in small type: "report review · checks findings against cited
  evidence · not a re-audit of the target"

## 9. The result

**Admissibility first, and on its own line.** `ADMISSIBLE` or `INADMISSIBLE`,
per §4.0.

**An inadmissible report's result is that word and nothing after it.** No
supported ratio, no PASS or FAIL — those are readings of the findings, and §4.0
fired because the findings could not be read. Reporting a ratio anyway is the
failure this rule was written for: it is the number that gets quoted, and it
will be quoted without the qualification attached to it.

For an admissible report, two numbers and a verdict, stated together at the top
of the review.

**Supported: `N` of `M` findings.** M is the frozen count from §4. N is the
findings that came back `[supported]`. Every other verdict is an exception and
is listed.

**Exceptions by verdict** — how many `[overstated]`, `[understated]`,
`[unsupported]`, `[broken citation]`, `[indeterminate]`.

**PASS or FAIL.** FAIL if there is any `[unsupported]`, any `[broken citation]`
or any `[indeterminate]`. Those two mean a finding is not defensible from what it
cites, which is the one thing this review exists to catch. `[overstated]` and
`[understated]` are calibration: reported, never fatal, because a verdict one
step off is a different problem from a finding with no support.

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
- It does not check anything the report did not claim.
