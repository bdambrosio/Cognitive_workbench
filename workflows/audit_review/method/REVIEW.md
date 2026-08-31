# Report review — method

## 1. Purpose

An independent review of a finished claims audit, against the materials that audit was given. The task is:

> Compare each finding with the evidence it cites, and cite what you find.

You did not perform the audit and you are not defending it. Your subject is the report; the report's subject was the target.

The audit's own method states the gap this review fills. METHOD §12 step 6b: *"This check establishes that cited material exists; it does not by itself prove that the cited line is the correct evidence."*

## 2. Scope rule

**Review what the report states, against what the materials show.**

`claim` means what METHOD §2 defines: an assertion the seller made. The report states *findings about* claims. It makes no claims of its own, and neither does this review. **"The report" always means the audit's report**, never this review.

**Do not audit the target.** A finding the report never made is not your finding, however material it appears. If the audit missed something, that is a coverage question and is reported under §7 as coverage, not as a new finding about the business.

**Do not improve the report.** Do not rewrite findings, soften verdicts, or suggest better wording. Report what does not hold.

## 3. What you have

**The report and the Gap Map** — `report.md` and `gap_map.md`, under `inspect`. These are what you review. `report.md` holds METHOD §16's REPORT, COVERAGE and LIMITATIONS blocks; the Gap Map is its GAP MAP block. An older audit run may carry no COVERAGE block and state its coverage inside the report instead; both shapes appear on the board.

**The audit's claim surface is not among these files.** METHOD §16 makes it a delivered block, but it is not written to a file of its own. It is in `working_record/reasoning_trace.jsonl`, with the rest of the auditor's legs, and §7 is when you read it.

**The working record** — also under `inspect`: `working_record/`, holding the auditor's reasoning trace, one file per evidence request, and the brief.

**The method the audit worked to** — `working_record/method_as_delivered.md`, the text that engagement actually received. Use this file rather than any current version. You are checking whether the report followed the method it was given; a method that has changed since is not the standard the report was written to.

**The materials** — the target itself, under `inspect_external`. The same tree the auditor read.

**Read the report and the materials first. Do not read the reasoning trace until §7.** A reviewer who reads the auditor's working log first adopts its framing, and then checks whether the reasoning was followed rather than whether the conclusion is supported. The trace answers "why was this not checked", which is a §7 question and comes last.

## 4.0 Admissibility — settle this before enumerating anything

Two questions. Both are about whether the report can be checked at all, and both come before any finding is enumerated.

### First question: can you establish what the report's citations refer to?

If you cannot, stop. A supported ratio computed over citations read in the wrong coordinate system is wrong, not weak.

**What you are given.** `review/citations.json` carries a `scheme` block: how many `docN:NN` references the report makes, how many resolve as line numbers, how many name a line past the end of their document, and the same figures per document. Quoted spans are resolved separately and counted there.

**The one mechanical fact.** An integer larger than its document's line count proves that reference is not a line number. Nothing mechanical can tell you what the scheme *is*.

**Distinguish an isolated anomaly from a systematic one.** Treat an isolated anomaly as a citation exception; treat a repeated pattern showing the report is using a different citation coordinate system as an admissibility problem.

- **Isolated.** `doc7:94` in a 45-line document, where the other seven references to that document resolve. Record it as a broken citation and continue.
- **Systematic.** Four of seven references to a five-line document, topping out at 13, against a coverage statement reading "doc2: 13 claims". That report is citing claim ordinals. The citations are well-formed and they are not line numbers.

**If the scheme cannot be established, stop.** State which references cannot be resolved and why, state what the report would have to say to make them resolvable, and end the review there. **Do not enumerate the findings, do not check them, and do not report a supported ratio.**

### Second question: is there anything to check?

**What you are given.** `review/conformance.json` carries `evidence fields` and `evidence fields pointing nowhere`: every Claim, Evidence or Basis field carrying neither a reference nor a quote. Under METHOD §5 a field naming only a document, or a section of one, and then writing prose is not a citation — `Evidence (doc3, CI/CD & Testing): 12 unit tests, all in test/utils/` gives a reader nothing to search for.

**Two cases that do not count as uncited.**

- **A field stating that no evidence exists** — "no source code provided to verify exact version" — has nothing to cite.
- **An absence finding.** There is no limit on how many of these a report may hold. METHOD §5 gives a claim finding whose evidence is an absence its own shape: the claim is cited normally, and the Evidence field carries the two searches, lexical and structural, that establish the absence. There is no settling line, because the point of the finding is that no such line exists. That is a complete citation in METHOD's terms. Count it as cited when both searches are stated, and `[uncited]` only when they are not.

**`evidence fields pointing nowhere` counts absence findings, and should not.** Its mechanical test is "neither a reference nor a quote", and two documented searches are neither. Treat the number as an upper bound: subtract the absence findings before reading anything from it, and say in the review that you did.

**Isolated or systematic, as above.** A few such fields make those findings `[uncited]`, listed as exceptions, and the review proceeds. If uncited evidence fields are frequent enough that the report as a whole cannot be checked reliably, that is an admissibility failure like the first question's: stop, and state how many findings it leaves unverifiable.

### The indexes are not the materials

**The materials are authoritative. `review/citations.json` and `review/conformance.json` are indexes over them.** Never assign `[broken citation]` because an index failed to resolve a reference, and never rule a finding unsupported because its evidence is missing from one. An index resolves only `docN:NN` references and quoted spans, so a report citing a document by section produces no entry, and an index that resolves a name to the wrong file marks sound references broken. In each case the gap is a property of the index, not of the document: read the document under `inspect_external` and record what is there. This rule binds wherever an index is consulted, including §5's checks and §9's retest.

## 4. The review surface, and closing it

1. **Enumerate the findings.** Every finding the report states, in the report's order.

2. **Close the surface** by stating the count after a line reading exactly:

   ```text
   === REVIEW SURFACE ===
   <N> findings
   ```

   The line immediately after the marker carries the count and nothing else: a number, then the word `findings`. List the findings after it, and close with a line reading exactly `=== END REVIEW SURFACE ===`. This is one of the four delivery blocks; §8 has the full set.

   From that point the surface is **frozen**. It is the denominator of every **review completeness** figure. Keep two measurements apart: `coverage` is METHOD §1a's quantity, resolved over identified claims, and belongs to the audit; **review completeness** is findings reviewed over findings enumerated, and belongs to this review. A report finding noticed after the surface is frozen goes into an addendum with its own count, and never into the frozen total.

3. **Check every finding.** Unlike an audit, there is no priority order and no stopping threshold. A report of fourteen findings gets fourteen checks; a report of two hundred gets two hundred.

**Do not sample.** METHOD §4 gives every resolved claim a finding, so a real target produces hundreds. Check all of them.

**Label the findings, and keep the labels.** Give each one a stable identifier as you enumerate: `F1`, `F2`, and so on, in the report's order. Every later reference uses that label. Without labels, "do not re-enumerate" cannot be checked, because nothing records what was already enumerated.

**The label is yours and identifies a finding only inside this review.** It is not the report's name for the finding. For that reason §6's format also carries the report line range.

**When you yield, state where you are and what you found.** The work may take several legs; §8 does not prescribe how many. A yield carries:

- the frozen count and the label range, and that they are not to be re-enumerated
- which labels are checked and which remain
- **for each checked label, its disposition and the citation that settles it** — not merely that it was checked

Nothing else supplies this: the client's process reports what has happened, never what remains.

## 5. What to check, in this order

Two checks per finding. Both take a different form for a derived finding, because a derived finding carries no claim verdict to check.

- **A claim finding.** Does the cited evidence support what the finding states, and does the audit's METHOD §6 claim verdict match the gap?
- **A derived finding.** Does each Basis line say what the finding quotes it as saying, and does the derivation follow from those figures as written?

In both cases, then assign a review disposition from §6.

**The index proposes; you confirm.** The client's process fetches every cited line before you start and hands you `review/citations.json`, which proposes `[broken citation]` for every reference that did not resolve. Per §4.0, open the document under `inspect_external` and check there before you assign that disposition: the index covers only some citation forms, and it can resolve a name to the wrong file. Once you have confirmed a citation is broken, do not assess support for that finding.

**A quoted citation resolves or it does not, and `review/citations.json` says which.** Its `how` field carries one of five values:

| `how` | Meaning | Treat as |
|---|---|---|
| `contiguous` | The quote appears verbatim in one document | Resolved — judge support normally |
| `segments` | One document holds every part, reordered or with gaps | Resolved — judge support normally |
| `split` | Every part was found, but no single document holds them all | `[broken citation]` |
| `partial` | Some parts were not found in the materials at all | `[broken citation]` |
| `miss` | The words are not in the materials as written | `[broken citation]` |

**Write a `split` quote out in full.** A split quote is assembled from more than one document into a sentence that reads as continuous evidence: every fragment can be verbatim while the sentence as typed is true of nothing. The disposition is `[broken citation]`, because the quote does not resolve — but the Exception block must state that the quote was assembled from more than one document, and name them. A reader who sees only `[broken citation]` will assume a typo.

The index rule binds here as everywhere: confirm against the documents before assigning the disposition.

**Fields that point nowhere are different: nothing was cited for the index to miss.** `review/conformance.json` enumerates them. Read the document each one names, look for text supporting what the finding states, and record `[uncited]` either way — the report failed to cite it, and the report is what you review.

- **Found.** Quote it in the Exception block with its line.
- **Not found, or not plainly supporting.** State where you looked.

**If support is ambiguous, record not-found rather than supplying a replacement citation.**

**Search only the document the field names.** Widening the search is auditing the target, which §2 forbids.

**Why the derived case differs.** METHOD §6 makes a finding one of two things: a resolved claim carrying one of the five claim verdicts, or a `[derived]` finding, which is a consequence computed from two or more figures the seller stated separately. A derived finding cites no claim because there is none to cite. Asking whether its cited lines support a claim it never made would rule it unsupported by construction.

The arithmetic is written out so that it can be checked. A step you cannot follow is an exception; a step you can follow and disagree with is `[overstated]` or `[understated]` as usual.

**On a claim finding's second check**, use the five **claim verdicts** from METHOD §6 — `[real]`, `[real, minor caveat]`, `[real, operational caveat]`, `[partial]`, `[delta]`. A gap described as a corner case but carrying `[delta]` is overstated. A claim shown to be false but carrying `[partial]` is understated. Both are exceptions.

**You are not asked whether a consequence matters to the buyer.** That is materiality. It is METHOD §4's judgement and the auditor's, and §10 reserves it.

**`[unverifiable]` and `[unclaimed]` are not findings and do not belong on your surface.** METHOD §6 sends both to the report's coverage statement: the first is a claim the materials could not settle, the second is something the seller never claimed. If the report states either as a numbered finding, record one conformance exception and do not enumerate them into §4's frozen count, which would inflate the denominator of every ratio.

## 6. Finding format

```text
**Exception N: F<M> (report:<lines>) — [disposition]**

Report says (report:lines): <what the finding states, verbatim>

Materials show (<document:lines>): <what the cited lines actually say, verbatim>

Exception: <the specific mismatch>
```

**The report line range appears twice deliberately.** The second reviewer does not see this review (§9), so the first line of an Exception block must identify its own subject.

**Quote both sides verbatim.** If the finding says "no alerting configured" and the line says "Alerting: none configured for backup failures", quote both and let the reader see the difference, or the absence of one.

**One exception per finding.** If a finding is wrong in two ways, the more serious one is the exception and the other is a note inside it.

**Review dispositions.** `verdict` is METHOD's word for what an audit concluded about a claim. These are what *you* conclude about a finding, and the two sets are kept apart.

| Disposition | Meaning |
|---|---|
| `[supported]` | The citation resolves and the lines say what the finding says they say |
| `[overstated]` | The cited material supports something weaker than the finding states |
| `[understated]` | The lines support something stronger — a `[partial]` that is a `[delta]` |
| `[unsupported]` | The cited material does not support the finding, or says something else |
| `[broken citation]` | The reference does not resolve in the materials |
| `[indeterminate]` | The citation is well-formed and resolves to text, and the referent it indexes cannot be established from the report |
| `[uncited]` | The finding's evidence names no location — no reference, no quote, and not an absence finding's two searches — so there is nothing to check |

**An exception is every disposition other than `[supported]`.** `[supported]` needs no Exception block; it enters the count and nothing else. The other six are written out.

**Three of them are retest-eligible exceptions: `[unsupported]`, `[indeterminate]` and `[broken citation]`.** §9 retests exactly these and no others.

Choose between the four citation states by what a reader can do with the reference:

- `[unsupported]` — the reference resolves, you read the material, and it does not support the finding.
- `[broken citation]` — a stated reference cannot be found in the materials.
- `[indeterminate]` — the reference resolves to text, and the report does not establish what it refers to. Not a hedge on a citation you did read.
- `[uncited]` — no checkable reference or quote was supplied.

**`[supported]` here does not mean METHOD §1a's `supported`.** METHOD counts a claim as `supported` when the materials bore the claim out, which is a property of the target. This disposition says a finding's citation bears out what the finding says, which is a property of the report. One audit can be `supported 37 of 39` in METHOD's sense and reviewed `supported 12 of 40` in this one, and both be correct. Each word is right in its own document, so both are kept — but the two numbers get quoted side by side, and whenever either leaves this review, say which is meant.

## 7. Coverage, and reading the record

**Once every finding has been checked** — not after each one — read the working record and report **two** things, once, about the audit as a whole. Nothing in this section is written per finding.

**What the report says it covered, against what the record shows it did.** The audit froze a claim surface; §3 names the file that holds it, since it is not in the report. METHOD §16's COVERAGE block gives one verdict per claim, and the coverage figures are computed from that ledger by the client's process rather than written by the auditor — so the figures are not yours to check. What is yours to check is whether the record bears out the work the ledger claims. The trace and the evidence requests show what was actually read. A coverage statement the record does not bear out is an exception like any other.

**What the audit did not attempt.** Not a list of everything it could have done: the claim sources are finite and the record names what was opened. Where the report claims coverage of a document the record never shows it reading, say so.

**Do not report a finding the audit should have made.** "Doc6 was never read" is a coverage exception. "Doc6 shows a churn problem they missed" is auditing the target, which §2 forbids.

## 8. The deliverable

Four blocks, each opened and closed by a line of its own:

```text
=== REVIEW SURFACE ===    …    === END REVIEW SURFACE ===
=== REVIEW ===            …    === END REVIEW ===
=== LIMITATIONS ===       …    === END LIMITATIONS ===
=== SUMMARY ===           …    === END SUMMARY ===
```

**Emit them in that order.** Take as many legs as the work needs; the blocks are what the review is judged on, not how the work was divided.

**`=== SUMMARY ===` comes after you are told the retest result** (§9), and not before. The other three may arrive in any number of legs, together or apart. The summary states an outcome you do not know until the client's process hands it to you.

**Markers are how a block is delivered.** The client's process reads them. Until a block's opening marker appears the review is still running, and you will be told which block is missing. A closing marker states that you finished writing rather than ran out of room.

### The blocks

**`=== REVIEW SURFACE ===`** — as §4 specifies: the count, then the findings listed with their labels.

**`=== REVIEW ===`** — the review proper.

**If the report is inadmissible under §4.0, this block is the admissibility statement and nothing else:** the §9 result line, which references cannot be resolved and why, and what the report would have to state to make them resolvable. No exceptions, no completeness figure, no ratio.

**An inadmissible report has three blocks, not four.** §4.0 forbids enumerating the findings, so there is no `REVIEW SURFACE` to emit and none is expected. Open the `REVIEW` block with `INADMISSIBLE` and the client's process will stop asking for the surface. `LIMITATIONS` and `SUMMARY` still follow.

Otherwise the `REVIEW` block contains, in this order:

1. **The §9 result line, the ratio, and the exceptions by disposition**, in the form §9 specifies — but **not the standings**. The retest runs after this block is delivered, so at the moment you write it no standing is known. Standings go in `=== SUMMARY ===`.
2. **Exceptions, worst first**, each in §6's format.
3. **Review completeness** — findings checked out of §4's frozen count — and, separately, what the record showed about the audit's **coverage** (§7). The addendum too, if §4 produced one.

**`=== LIMITATIONS ===`** — three lines, always present: the report and materials you read, with the materials' as-of date; that the auditor was not consulted and has not responded to these exceptions; and that this review checks findings against cited evidence and does not re-audit the target.

**`=== SUMMARY ===`** — one page, for whoever decides whether the report ships:

- The report reviewed, and the engagement it came from
- The §9 result: admissibility, then supported by their citations N of M and the standing exceptions, if the report was admissible
- Exceptions that would change a reader's understanding, worst first
- Review completeness: findings checked out of §4's frozen count
- A footer in small type: "report review · checks findings against cited evidence · not a re-audit of the target"

Do not restate the review here, and do not introduce it.

## 9. The result

**The result line is exactly one token: `ADMISSIBLE` or `INADMISSIBLE`**, per §4.0. There is no third value and nothing qualifies it. Whether the retest ran, and what it found, belongs in the standings below and never on this line.

**An inadmissible report's result is that word and nothing after it.** No supported ratio and no exceptions: both are readings of the findings, and §4.0 fired because the findings could not be read.

For an admissible report, state the ratio and the exceptions by disposition together, immediately after the result line.

**Supported by their citations: `N` of `M` findings.** M is §4's frozen count. N is the findings that came back `[supported]`. Every other disposition is an exception and is listed.

Use that full form, never a bare "supported N of M" — §6 gives the reason.

**Exceptions by disposition** — how many `[overstated]`, `[understated]`, `[unsupported]`, `[broken citation]`, `[indeterminate]`, `[uncited]`.

**The result is the exceptions and, once the retest has run, whether each one stands. There is no grade.** A finding is not defensible from what it cites when it is `[unsupported]`, `[broken citation]`, `[indeterminate]` or `[uncited]` — whether the citation is wrong, unplaceable, or absent.

**Standings are reported in `=== SUMMARY ===`, not in the `REVIEW` block.** The retest runs after `REVIEW` is delivered; until then no exception has a standing, and the `REVIEW` block should not guess one. Give each exception one of three standings in the summary:

- **stands** — the retest reached the same disposition. The finding is not defensible from what it cites.
- **does not stand** — the retest disagreed. The finding is borderline; report the disagreement rather than resolving it.
- **not retested** — the disposition does not qualify (`[uncited]`), or the retest could not be obtained. State which.

**Every retest-eligible exception is retested once**, by a second reviewer — §6 names the three, and they qualify for a derived finding as readily as a claim one. For each, the client's process has a second reviewer check that finding. That reviewer is not told your disposition and does not see your review. You are told what it found before you write the summary.

`[uncited]` is not retested: by §6's definition there is no reference for a second reviewer to open.

**If the retest could not be run at all**, mark every exception in the summary **not retested**, naming the reason. A retest that did not happen is not a retest that disagreed, and an exception whose standing is unknown must not be reported as one that does not stand. The result line still reads `ADMISSIBLE` or `INADMISSIBLE` exactly as you found it: a review whose retest did not run is not a third kind of result.

**Settle a retested `[broken citation]` against the materials**, per §4.0's index rule. An index will agree with itself.

**`[overstated]` and `[understated]` are not retested.** They are calibration: reported, and not retest-eligible, because a verdict one step off is a different problem from a finding with no support.

You do not withdraw a report, ask the auditor to redo it, or contact the client. You state which findings do not hold, and how.

## 10. What this review does not do

- It does not audit the target.
- It does not judge whether a finding is material to the buyer.
- It does not assess whether the audit's method is right, only whether the report followed it and its findings hold.
- It does not check anything the report did not state as a finding.
