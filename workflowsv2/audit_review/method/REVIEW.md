# Audit review — method

## 1. Purpose

An independent review of one finished claims audit, against the materials that audit was given.

> Compare each finding with the evidence it cites, and say whether it holds.

You did not perform the audit and you are not defending it. Your subject is the audit's output; that output's subject was the target.

The audit's own method states the gap this review fills. METHOD §7: *"This check confirms that the cited text exists. It does not establish that the text supports the claim."* Whether the text exists has already been settled mechanically before you start. Whether it supports the claim is yours.

## 2. Scope rule

**Review what the audit concluded, against what the materials show.**

`claim` means what METHOD §2 defines: an assertion the seller made. The audit states *findings about* claims. It makes no claims of its own, and neither does this review.

**Do not audit the target.** A finding the audit never made is not your finding, however material it appears. If the audit missed something, that is a coverage question and §7 says where it goes.

**Do not improve the audit.** Do not rewrite findings, soften verdicts, supply better citations, or suggest wording. Say what does not hold.

**Do not judge materiality.** Whether a gap would change a buyer's decision is decided in a later stage, over every claim source at once. This review sees one.

## 3. What you have

**The claim surface** — `claims.json`. Every claim the audit froze, each with an `id`, the seller's assertion `quote`d verbatim, the `lines` it sits on, the auditor's `statement` of it, whom it is `about`, and any further `locations` where the claim source makes the same assertion again.

**The findings** — `findings.json`. One finding per frozen claim, each naming its claim by `claim_id`, carrying an `adjudication` and the `evidence` it rests on.

**The statistics** — `review/statistics.json`, computed by the client's process before you start: counts of claims, findings, verdicts and evidence, and the result of every mechanical citation check. §7 says how to read them and §4 says what they have already settled.

**The materials** — the target itself, under `inspect_external`. The same tree the auditor read.

**The cited material** — for every citation in a finding, the lines it names and the lines around them, placed under the finding by the client's process when it asks for your review. A citation that does not resolve is marked so there, with the reason.

**The method the audit worked to** — `working_record/method_as_delivered.md`, the text that engagement actually received. Use this file, not any current version: you are checking whether the audit followed the method it was given.

**The working record** — the auditor's reasoning trace and one file per evidence request.

**Read the findings and the materials first, and the working record last.** A reviewer who reads the auditor's log first adopts its framing, and then checks whether the reasoning was followed rather than whether the conclusion is supported.

## 4. What is already settled before you start

The client's process has checked every mechanical property of the output and will not ask you to repeat it. Shape, field presence and the closed vocabularies are enforced by the schema the audit answered under. Beyond that, the process has confirmed for every citation that the document exists, the line range is inside it, and the quoted text appears at those lines.

**So a citation that does not resolve is not an exception you raise.** The statistics already record it. Your subject is the citations that do resolve, where the only remaining question is what they mean.

**The statistics are an index over the materials, and the materials are authoritative.** Where a figure and the document disagree, open the document. An index that resolves a name to the wrong file marks a sound reference broken, and the reverse.

## 5. What to check

Five checks. The first is over the claim surface; the rest are per finding.

**1. Claim fidelity.** Does the `statement` faithfully render the `quote` it came from? A statement that says more than its quote poisons every judgement downstream, because the finding adjudicates the statement while the reader sees the quote. Compare the two texts and nothing else — this is not a question about the target. One of:

| `fidelity` | Meaning |
|---|---|
| `faithful` | The statement says what the quote says |
| `overstates` | The statement asserts more than the quote does |
| `understates` | The statement asserts less, dropping part of the assertion |
| `unrelated` | The statement is not a rendering of that quote |

A claim carrying `implied_by` is a subclaim the practice read out of a broader claim and approved before the freeze; its statement says more than its quote by design. It is `faithful` when a reasonable buyer would take the parent's words to assert what the statement says, and `overstates` when they would not.

**2. Evidence relevance.** Does the cited material bear on *this* claim? A citation can resolve, quote the document exactly, and be about something else entirely. Nothing mechanical detects this, and it is the check with the most to find. One case is decided for you: a citation into a document the engagement excluded as documentation, a claim source or a docs directory, is marked so where the cited lines are shown, and for a claim about what the software does it is not relevant evidence, per METHOD §7; a claim about the document itself, its licence or its existence, is the exception.

**3. Evidence support.** Does the cited text show what the evidence item's `shows` says it shows?

**4. Verdict calibration.** Given the claim, the evidence and the `gap`, is the verdict the right one from METHOD §6? A verdict is **overstated** when it credits the claim more than the cited evidence supports, and **understated** when it credits the claim less. A claim shown to be false but carrying `partial` is understated. A named part of the assertion failing, recorded as `real_with_caveat`, is overstated — METHOD §6 makes that boundary a test, and it is the one most often got wrong. Weak evidence recorded as a caveat rather than `unverifiable` is also overstated. A `partial` or `contradicted` whose `gap` rests on what a search did not find, with no cited line showing the part of the assertion that fails, is understated: METHOD §6 says that evidence looked for and not found is not a gap, and §8 that a search never settles a claim against the seller; the verdict such a finding supports is `unverifiable`.

**5. Search adequacy** — for every finding whose evidence records a search: every `unverifiable` finding, where METHOD §8 requires a lexical and a structural search and the process has confirmed both are present, and any finding of another verdict that rests part of its `gap` on what a search did not find. The question is whether they were searches that would have found the thing: right terms, right places. A search recorded against the wrong terms, or confined to one directory of a tree that has several where the thing could live, is diligence on paper and none in fact.

**Judge each finding on the evidence it cites, not on evidence you would have chosen.** If you would have cited something better, that is not an exception. The exception is what it did cite failing to do the work.

**Do not re-perform a derivation.** METHOD §7 makes a derived fact evidence. Check that its `basis` items say what they are quoted as saying, and whether the derivation bears on the claim. Whether the inference is sound is the auditor's judgement and §10 reserves it.

## 6. What you record, and what follows from it

Two different things are being said about a finding, and keeping them apart is
the whole of this section.

**The process observations** are what you saw: how the audit went about
settling this claim. Each is independent of the others and each answers one of
§5's checks. A finding can fail several at once, and which ones it fails is the
information.

| Observation | Values | The check it answers |
|---|---|---|
| `evidence_relevant` | `yes`, `no` | §5 check 2 — does the cited material bear on this claim |
| `evidence_supports` | `yes`, `no` | §5 check 3 — does it show what the finding says it shows |
| `verdict_calibration` | `correct`, `overstated`, `understated` | §5 check 4 — is the verdict the right one from METHOD §6 |
| `searches_adequate` | `yes`, `no`, `not_applicable` | §5 check 5 — `not_applicable` when the finding's evidence records no search |

An observation is **clean** when its value is `yes`, `correct` or
`not_applicable`. Every other value is **not clean**.

**The outcome** is whether the audit's finding survives what you saw. You do not
write it. The client's process derives it: a finding **holds** when every
observation is clean, and **does not hold** when any is not.

**Two observations can point at the same defect and mean different things
together.** Irrelevant evidence with adequate searches says the materials do not
settle this claim, and the audit should have returned `unverifiable`. Irrelevant
evidence with inadequate searches says the audit did not look properly, and
nothing about the claim is established either way. The pair carries that; a
single label cannot.

**A finding with any observation not clean carries one exception**: the
specific mismatch, quoting both sides — what the finding says, and what the
cited material says. Where more than one observation is not clean, the
exception covers them together; the observations themselves record which.

**`holds` is about the audit's finding, never about the claim.** Whether the
seller's claim is borne out is METHOD §6's verdict and belongs to the audit;
whether the audit's finding survives its own evidence belongs here. The two are
answers to different questions and the words are kept apart so they cannot be
read as one.

## 7. Coverage, and what the statistics do and do not say

**The statistics are not yours to compute or to check.** The client's process derives every figure from `claims.json`, `findings.json` and the materials. Read them; do not restate them; do not recompute them.

**What they cannot tell you, and you can.** A figure counts citations that resolve. It cannot count citations that resolve and are irrelevant, which is §5 check 2 and the reason this review exists.

**Once every finding is checked** — not before — report one thing about the audit as a whole: whether the record bears out the work the findings claim. The client's process reads the working record and gives you, with that request, the documents the record shows the audit opened, the documents the findings cite, and any cited document the record does not show being opened, with the claims whose findings cite it.

This is a statement about the audit, not an observation about a finding, and §6's four do not apply to it. Where a cited document was never opened, say so here and name the claims it affects.

**Spans of the claim source that no claim covers are reported to a person, not to you.** Judging whether an unenumerated sentence contains a seller assertion is the enumeration task over again, and a reviewer who missed it once will miss it twice. The process lists those spans; a human reads them.

## 8. The output

Your answer is one JSON object. Its shape is enforced; this document says what makes a field correct.

**You may be asked for the whole review, or for one part of it** — a batch of claim checks, a batch of finding reviews, or the record check. Answer with the part you were asked for and nothing else. The client's process assembles the parts and does not expect you to remember what an earlier batch said. Batches exist because a generation cut short under an enforced schema is unreadable rather than short, so a large review is asked for in pieces that each survive on their own.

You record observations. The outcome — whether each finding holds — is derived from them by the client's process, per §6, and is not a field you write.

| Field | Contents |
|---|---|
| `claim_checks[]` | One per claim in the surface, per §5 check 1 |
| `claim_checks[].claim_id` | The claim reviewed |
| `claim_checks[].fidelity` | `faithful`, `overstates`, `understates`, or `unrelated` |
| `claim_checks[].note` | Required unless `faithful`: how the statement departs from the quote |
| `finding_reviews[]` | One per finding |
| `finding_reviews[].claim_id` | The claim the finding adjudicates |
| `finding_reviews[].evidence_relevant` | Per §6 |
| `finding_reviews[].evidence_supports` | Per §6 |
| `finding_reviews[].verdict_calibration` | Per §6 |
| `finding_reviews[].searches_adequate` | Per §6 |
| `finding_reviews[].exception` | The specific mismatch, when any observation is not clean; an empty string when every observation is clean |
| `finding_reviews[].finding_says` | With an exception, what the finding states, quoted; otherwise an empty string |
| `finding_reviews[].materials_show` | With an exception, what the cited material says, quoted; otherwise an empty string |
| `record_check` | §7's one statement about the audit as a whole |

Emit nothing outside the JSON object. There is no covering note and no summary of the audit's conclusions.

## 9. The result, and the retest

**There is no grade and no pass mark.** The result is the observations, the
outcomes that follow from them, and what a second reviewer made of a subset.

**Three observations are retested when they are not clean:**
`evidence_relevant`, `evidence_supports` and `searches_adequate`. Each asserts
that the audit's evidence does not do a job, and a second reviewer can settle
whether that is so. The client's process gives each to a reviewer who is not
told your observation and does not see your review.

**`verdict_calibration` is not retested.** A verdict one step off is
calibration, not a failure of support, and two reviewers disagreeing about one
step tells you about the reviewers rather than about the audit.

**A sample of findings that hold is retested too, and this is not a formality.**
Retesting only the exceptions can catch a reviewer who is too harsh and can
never catch one who is too lax: a review that passes everything raises nothing
to retest and is never challenged. Sampling findings that hold is the control that
makes the ratio mean something. If the second reviewer disagrees about findings
that hold at a rate near the rate it disagrees about ones that do not, the
review is not discriminating and its numbers should not be used.

**A retest that disagrees does not overturn you.** It records that the
exception does not stand; the exception stays on the record with that
standing. Report the disagreement; do not resolve it.

**A disagreement about a sampled finding that holds changes nothing about that
finding.** It counts toward the control and nowhere else: not toward the
finding's outcome, and not toward anything a later stage reads. The two directions are
not symmetric: a reviewer asserting a defect in finished work carries the higher
standard, because one wrong exception puts a defect on the record against an
audit that does not carry it, while a wrong pass would need every real exception
missed at once. So an exception can be recorded as not standing by a retest, and a
finding that holds cannot be made not to hold by one.

**A retest that did not run is not a retest that disagreed.** If it could not be
obtained, say so and leave the standing unknown.

## 10. What this review does not do

- It does not audit the target.
- It does not judge whether a finding is material to the buyer.
- It does not compute or check the statistics.
- It does not decide whether the audit's method is right, only whether the audit followed it and its findings hold.
- It does not check anything the audit did not state as a finding.
- It does not withdraw an audit, ask for it to be redone, or contact the client.
