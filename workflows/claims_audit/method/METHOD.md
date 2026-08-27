# Technical claims audit — method

## 1. What this is

A one-shot technical due-diligence engagement for a buyer evaluating a target
they do not own, in acquisitions under $5M. The operation is:

> stated claims vs. observed evidence, with citations.

What this audit does not do is in §11.

## 1a. Level of assurance

This is a **limited assurance** engagement. The audit examines part of the
target's claims, not all of them: §4 sets the order of work, §12 sets where it
stops, and §11 lists what is out of scope.

The report states its conclusion positively — of the claims it resolved, these
are supported and these are not — but only about those. It says nothing about
the rest.

Two rules follow.

1. **State the coverage wherever you state the conclusion.** The §9
   conclusion and the number of claims it rests on appear together, in the
   report and in the Gap Map. A conclusion without that number gives the
   reader nothing to rely on.

**The four claim-state quantities used in coverage reporting.** Wherever the
report states a count or a fraction of claims, use these words and no others —
"checked", "examined" and "verified" each blur `attempted` and `resolved`, and
a coverage figure whose denominator is unstated is the failure §1a exists to
prevent. Ordinary English is fine everywhere else; this binds the numbers.

| term | meaning |
|---|---|
| **identified** | claims enumerated from the claim sources and frozen in the claim surface (§12 step 2) |
| **attempted** | identified claims you tried to settle |
| **resolved** | attempted claims that reached one of §6's five claim verdicts |
| **supported** | resolved claims whose verdict is `[real]` or a caveat form |

An attempted claim that did not resolve is `[unverifiable]`. An identified
claim never attempted is *unattempted*, and is accounted for by number in the
coverage statement.

**Coverage is resolved over identified. The consistency rate is supported over
resolved.** Two rates, two denominators: state both, and never let one stand in
for the other. `attempted` sits in neither rate and is still required — it is
what separates "we could not settle it" from "we never looked", and
`[unverifiable]` is exactly `attempted` minus `resolved`.

The canonical form: **"Coverage: 39 of 43 identified claims resolved; 37 of 39
resolved claims supported."**

2. **Never write a sentence that implies you resolved more than you did.**
   "The system does what it says" is a claim about everything. "Of the 43
   claims identified, 39 resolved and 37 supported" is a claim about the work
   done. Write the second.

## 2. The scope rule

**Audit what the seller asserts about the target, against what the materials
show.**

A claim is an assertion the seller makes to the buyer — in the listing, the
technical description, the specifications, the marketing. The engagement names
which documents carry those assertions (§12 step 1). Everything else provided,
including source code and its comments, is evidence: it is what a claim is
tested against, not a claim in its own right.

Claims about the business count as much as claims about the software. Revenue,
customer counts, contracts and dependencies are claims, as much as backups,
uptime and test coverage.

**Do not opine on what the target should do.** How the system ought to be
built, or the deal structured, is the buyer's judgement and is not what was
bought. This applies to the §9 conclusion as much as to any finding:
report the state of the claims, not the action the buyer should take.

## 3. Scope adapts to the target; the method does not

Which claims exist differs by target: a hardware specification asserts
different things from a SaaS listing. The operation in §2 and the priority
order in §4 do not change.

Where a target has little to report on some subject — no meaningful security
surface, no external dependencies — **say so as a property of the target
rather than padding the report, and state it precisely.** "No telemetry path;
one user-initiated outbound call sends user-typed text off-network" is
precise. "No data exfiltration path" is broader than the evidence supports,
and a statement like it had to be retracted mid-audit once.

## 4. Materiality and priority order

1. **Enumerate the claims first** (§12 step 2).
2. **Prioritise** (order below).
3. **Report coverage explicitly** — identified, resolved, supported (§1a),
   what was not attempted, and why the gap matters.

**What makes a gap worth reporting.** A gap is material if a reasonable buyer,
knowing it, **would change the price, the structure of the deal, or the
decision to close.** The threshold is relative to the transaction: the same
defect is material in a $400k acquisition and noise in a $40m one.

Three things follow. **Report every material gap whichever way it pushes the
price** — one that favours the seller is still a gap. **A gap below the
threshold does not make the claim fail**: record it under `[real, minor
caveat]` — the verdict exists for a discrepancy that does not change the
decision — rather than raising a `[partial]`. Reserve bare `[real]` for a claim
the materials bear out with nothing to note. Materiality decides whether a
discrepancy is worth the buyer's attention; it does not decide whether the
claim was accurate, and the verdict must not say it was.

**It still gets a finding.** Every claim you *resolve* produces one, the ones
that hold included — §1a requires the report to say which claims hold and which
do not, and a rate alone does not say which. A claim you attempted and could
not settle is `[unverifiable]`, which is not a finding (§6) and belongs in the
coverage statement. **The consistency rate is `supported` over `resolved`
(§1a)** — a ratio over claims, so `[derived]` findings are outside it because
they test no claim. It summarises the findings; it never replaces them. And
**materiality is judged against the buyer's decision, not your own taste**:
"this code is poorly organised" is not material unless it bears on what was
claimed.

**Priority order.** Within what is material, verify in this order:

1. **Claims whose failure ends the business** — data that cannot be recovered,
   money that cannot be collected, a dependency that can withdraw. In a
   physical product this is safety-critical mechanism; in a SaaS business it
   is recoverability, payment integrity and single points of control.
2. **Architectural invariants** — do the stated boundaries hold? Isolation,
   redundancy, failover, tier separation, shared state.
3. **Operational parameters** — retry intervals, timeouts, monitoring,
   retention windows, thresholds affecting day-to-day operation.
4. **Low-impact claims** — individually minor; collectively they set the
   consistency rate (§1a), which is the best available signal about the claims
   nobody had time to check.

**Those four levels are the tiers**, and a finding's tier is the tier of the
claim it tests. The tiers are set by consequence to the buyer, not by
subsystem. If a tier is empty for a target, say so in the coverage statement
rather than silently running a three-tier order. The order exists so that an audit cut short by
budget, time or access has resolved the most expensive unknowns first.

## 5. Finding format

```
**Finding N: <short title> — [verdict]**

Claim (<document:lines>): <the stated claim>

Evidence: <document or file:lines> — <what the materials show>

Gap: <None, or the specific gap>
```

**Cite both halves**: the document making the claim, and the document or file
that settles it, each with line numbers. Without both, a reader cannot check
the finding and the practice cannot defend it.

**That is the obligation for a claim finding.** A derived finding has no claim
to cite, so its obligation takes the other shape below: every Basis line cited,
and the derivation written out. The requirement underneath both is the same — a
reader must be able to reach every fact the finding rests on — and neither
shape is an exemption from it.

**Quote figures from the source line, never from a summary.** Tools that read
on your behalf answer in prose, and prose carries qualitative facts reliably
and numbers unreliably. A summary that has dropped the second of two figures
reads exactly like one that has not. For any claim resting on a number, date
or threshold, get the source line and quote it.

**An absence needs the search, not the conclusion.** "This is not implemented"
cannot cite a line, because the point is that no line exists. Its evidence is
the search, and you must show two, because each fails where the other holds:

- **Lexical, in the claim's own words.** The claim supplies the terms and the
  implementer probably used them too. A claim about detecting frustration is
  searched as `frustrat`, not as whatever you would have named the module.
- **Structural.** List the directory or document the thing would be in. A
  listing cannot be defeated by a synonym; a search can.

A documented search proves diligence, not absence. Searching for the wrong
word and recording it carefully is worse than not searching, because it reads
as thorough. Where the two searches do not settle it, the verdict is
`[unverifiable]`, not `[delta]`.

### The second shape: a derived finding

Some of the most material things in a data room are stated by nobody. They
follow by arithmetic from two figures the seller supplied separately — a
retention period in one document, a last-good-backup date in another, each
innocuous alone, together fixing a date after which the business cannot be
restored.

Such a finding has no claim to cite, so it takes a different shape:

```
**Finding N: <short title> — [derived]**

Basis: <document:lines> — <the first stated figure, verbatim>
       <document:lines> — <the second, verbatim>

Derivation: <the arithmetic, written out so a reader can check it>

Consequence: <what follows, and why a buyer cares>

Escalates: <Finding N, or None>
```

Four rules:

1. **Use only the target's own figures**, quoted verbatim. Deriving an expiry
   date from the seller's stated retention terms is still auditing what they
   told you. Deriving that the market will turn against them is not.
2. **Write the arithmetic out.** "30-day retention from a 2026-07-30 last-good
   backup exhausts on 2026-08-29" is checkable in one line. Arithmetic left
   implicit is an opinion with a citation attached.
3. **No conclusion inside the finding.** State the consequence and stop.
   What the buyer should do is the buyer's judgement (§2); the report's
   conclusion is §9's job.
4. **Name what it escalates.** A derived finding often sharpens an ordinary
   one — the same backup failure, now with a date on it. Say which, so the
   report reads as one finding intensified rather than two counted. If it
   escalates nothing, say None.

**A derived finding that resolves to a date carries that date as its own
expiry.** "The last recoverable backup ages out on 2026-08-29" is true when
written, serious a week later, and meaningless a month after that. State the
date and what happens on it. The report as a whole carries an **as-of date for
the materials**: a data room is a snapshot, and every conclusion in the report
is a conclusion about that snapshot.

**Priority.** A derived finding takes the §4 priority of the facts it rests
on, and rises above them when the consequence is more severe than either
input.

## 6. Verdict vocabulary

Three vocabularies, and they answer different questions. Reading them as one
table is what made `[derived]` look like a verdict on a claim and
`[unverifiable]` look like a finding.

**Claim verdicts — how a stated claim stood up.** These five, and only these,
are verdicts. Every one produces a finding.

| Verdict | Meaning | Buyer impact |
|---|---|---|
| `[real]` | Claim holds; the materials bear it out, with nothing to note | None |
| `[real, minor caveat]` | Holds; a discrepancy that does not change the decision | Awareness |
| `[real, operational caveat]` | Holds today; operational context qualifies it | Awareness + operational planning |
| `[partial]` | Mostly true, with a specific citable gap | **Material** — a gap to price |
| `[delta]` | Claim is false; the materials show otherwise | **Material** — a broken promise, and possibly a pattern |

**Examination status — what happened when you tried.** Not a verdict, because
no verdict was reached. Not a finding.

| Status | Meaning | Where it goes |
|---|---|---|
| `[unverifiable]` | Attempted; the materials could not settle it | Coverage statement (§16), with the reason |

A claim you never attempted is neither of these. It is `unattempted` (§1a),
and the coverage statement accounts for it by number.

**Neither a claim verdict nor an examination status.** Both label something
the seller did not claim. One produces a finding and one does not, and that is
the distinction to keep.

| Type | Meaning | Finding? |
|---|---|---|
| `[derived]` | A consequence computed from two or more stated figures (§5's second shape) | **Yes** — as severe as the consequence, and frequently material |
| `[unclaimed]` | The materials cover something the seller made no claim about | **No** — noted in the coverage statement (§16); rarely material |

**So a finding is one of two things**: a resolved claim carrying one of the five
claim verdicts, or a `[derived]` finding. Nothing else is a finding, and the
report contains nothing else in §5's format.

**`[partial]` vs `[delta]` is a real distinction.** `[partial]` is "the claim
is 80% true and here is the 20% that is not, with a citation to the corner
case." `[delta]` is "the claim is false; the seller says X, the
materials show Y, here are both citations." A buyer treats them differently: `[partial]` is a
known limitation to price in; `[delta]` is a broken promise that may indicate
a pattern.

**`[derived]` is not a verdict on a claim, and that is the point.** The five
claim verdicts say how a stated claim stood up. `[derived]` says the seller's
own figures, taken together, entail something neither document states. It
carries no claim source because there is no claim — which is why it needs §5's
second shape rather than the format above.

**`[unclaimed]` was called `[non-delta]` until 2026-08-27, and the name was
actively misleading**: it reads as the negation of `[delta]` — "the claim is
not false" — when it means there was no claim in the first place.

**`[unverifiable]` must never be reported as `[delta]`.** "I could not find
it" is not the same as "it is not there." The first belongs in the coverage
statement; only the second is a finding.

Use the narrowest verdict that fits.

## 7. Correction protocol

**The auditor corrects their own findings when subsequent evidence
contradicts them. Corrections are noted in the report with a one-line
explanation of what changed and why.**

An auditor who never corrects has a citation trail that proves nothing: if no
evidence could change a finding, the citations were decoration.

## 8. The reader

**Write for the person who decides.** They read the Gap Map in about thirty
seconds and open the report only if it earns the time. They will not read
forty cited findings: they read the conclusion, then drill into one or two
if it makes them curious.

So the report opens with the §9 conclusion and its coverage, and closes
with a paragraph a non-specialist can read and explain back to a colleague.
Tier everything in between to that reader.

## 9. Report-level conclusion

§1a already calls this the report's **conclusion**, and that is the word this
document uses. It was headed "recommendation" until 2026-08-27, which put an
advisory name on a statement about evidence — and the definitions had drifted
to match.

| Conclusion | Meaning |
|---|---|
| **Clear** | No `[delta]` and no `[partial]` findings, and no caveats of note. Every resolved claim is supported. |
| **Clear with caveats** | No `[delta]` or `[partial]` findings. Documentation drift, maintenance debt or operational notes a buyer should know, which do not bear on valuation. |
| **Conditional** | Material findings, each localised and remediable — a fix of known shape, or a claim about a non-critical feature. |
| **Material** | Material findings that bear on valuation or risk profile. |
| **Systemically inconsistent** | The claims do not describe the target: material findings across independent claims, or a claim source contradicted throughout. |

**The audit concludes; the buyer decides.** The audit says "Conditional: the
seller claims 30-day retention, the materials show 7." Whether 7 days is
acceptable is the buyer's call. The audit does not say "you should walk."

**Keep the two vocabularies apart, and note that this one did not.** The five
terms above describe the state of the claims. *Proceed, negotiate, walk* is the
buyer's action vocabulary, and using it would put the audit in the business of
advising on the deal (§2). Until 2026-08-27 **Walk** was one of the five terms
— the forbidden word used as a conclusion — while **Conditional** read "Proceed
if the seller will fix them" and **Material** read "the buyer must price this in
explicitly". Three uses of the register these very lines forbid.

**And `Clear` stated a conclusion §1a forbids.** It read "The system does what
it says", which §1a names as the example of a sentence claiming more than the
work performed: *"'The system does what it says' is a claim about everything.
'Of the 43 claims identified, 39 resolved and 37 supported' is a claim about the
work done. Write the second."* Now it says the second.
The Gap Map's closing line may offer a judgement — "the gaps are addressable
within the existing integration timeline" — if it is labelled as professional
judgement rather than written as an instruction.

## 10. Liability posture

<!-- audience: practice -->

**Category: expert due-diligence, not legal advice.** The distinction is
functional, not semantic. Legal advice is "the law requires X, your rights
are Y, you should do Z." Expert due-diligence is "here is what I observed,
here is the gap between claim and implementation, here is my professional
assessment of what that gap means for the value of the asset." The legal and
business judgement stays with the client.

Same category as a financial auditor reporting material misstatements, a
structural engineer certifying an undersized wall, or a pen-test firm
reporting an exploitable endpoint.

**Independence, and which way the incentive runs.** The buyer engages the
auditor, and the buyer benefits from findings: every delta is leverage on
price. That is a self-interest threat in the direction of **over-reporting**,
and naming it is the first safeguard. The others are already load-bearing
elsewhere: §5's citation requirement, which an invented delta cannot satisfy;
§6's rule that `[unverifiable]` is never reported as `[delta]`; §4's
materiality threshold, applied **regardless of which way a gap pushes the
price**; and §12 step 6b, which verifies the finished report against its own
citations.

A finding that survives all four is not advocacy. A finding that would not
have been written for a seller-side client should not be written for a
buyer-side one.

**Vector 1 — client sues for sloppy work.** The real risk, and the same one
every audit firm carries. Managed by four things:

1. **Scope definition in the engagement letter** — "based on the materials
   provided; we did not have access to production systems, live databases, or
   the source repository." If the defect was in production but not in the
   materials, it is not in scope.
2. **Professional standard, not perfection** — what a reasonable auditor
   would find from the same materials. The citation trail demonstrates the
   work was systematic; the correction protocol (§7) demonstrates it was
   honest.
3. **Limitation of liability clause** — caps damages, excludes
   consequential. This makes the risk finite rather than existential.
4. **E&O / professional liability insurance**, in place *before* the first
   engagement, covering professional negligence in technical due-diligence.

**Vector 2 — the target sues for defamation.** Structurally weak, and the
report format is what defeats it:

- **Truth is an absolute defence.** Every finding cites its source.
- **Not a public statement.** A confidential deliverable to one client under
  NDA, for a specific transaction. Many jurisdictions recognise a qualified
  privilege for exactly this. The target is not the audience.
- **Opinion vs. fact.** "The architecture is a single point of failure" is a
  technical opinion resting on observable facts. "The seller is a fraud" is an
  accusation. Stay in the first register.
- **Tortious interference** requires a reasonable expectation of a concluded
  contract; in early diligence the target is in negotiations and the client is
  free to walk for any reason.

**Jurisdiction is unresolved and outside what anyone here can settle.**
Defamation law, qualified privilege and the standard for reckless disregard
vary by state and country. **This section is a working posture, not legal
advice, and needs a lawyer's review before the first paying engagement.**

## 11. What this audit does not do

- It does not opine on what the target *should* do.
- It is not a penetration test.
- It is not a code-quality review.
- It does not claim coverage it has not earned (§4).
- It does not give legal advice.
- It is not acceptance testing of contracted work. The mechanism is similar,
  but this method is for a buyer evaluating a target they do not own.

## 12. Running an audit: sequence

Launched with `workflows/claims_audit/scenario.yaml` — one scenario for every engagement,
with two per-target lines (`world_name`, `external_repo`). Its header carries
the operational detail; this section is the method.

1. **Receive materials** (data room, repo access, docs). **Confirm scope in
   writing, including the claim sources.**

   The **claim sources** are the documents in which the seller asserts things
   about the target. The engagement names them. Every other document provided
   is evidence (§2). Enumerate claims from the claim sources only.

   If the engagement names no claim sources, say which documents you treated
   as claim sources and why, before the marker in step 2. A claim found later
   in a document that was not a claim source goes in an addendum (step 2),
   never into the claim surface.

2. **Enumerate claims, then CLOSE the surface.** Read the claim sources named
   in step 1, and end enumeration by stating the count after a line reading
   exactly:

   ```
   === CLAIM SURFACE ===
   <N> claims
   ```

   The line immediately after the marker carries the count and nothing else:
   a number, then the word `claims`. Close the enumeration with a line reading
   exactly `=== END CLAIM SURFACE ===`. It is one of the four delivery blocks;
   §16 has the full set and the rules they share.

   **What counts as one claim.** A claim is what §2 defines: an assertion the
   seller makes to the buyer, and one that could be checked and found true or
   false. Count one claim per such assertion. Do not merge two assertions
   about the same subject. Do not split one assertion into parts. Count every
   one whatever its priority. Below the total, give a per-document count.

   From that point the **claim surface** is frozen: every coverage figure in
   the report divides by this count, and it is the only name this document
   uses for it. A claim met later that was missed here goes in an
   **addendum** — a separately counted list, stated after the coverage
   statement — and never into the claim surface itself.
3. **Prioritise** in §4's order.
4. **Work the priority order straight through.** Do not pause for
   confirmation; there is no channel to the client mid-engagement. Findings
   are reported in the deliverable, where the client can act on them.
5. **Stop when what remains is low-risk** — when the identified claims still
   unattempted are low-priority ones, and those resolved so far have been
   supported consistently.
   Say in the coverage statement where you stopped and why.
6. **Write the report.** Apply §7 to any findings that were revised.
6b. **Check the report against its own citations.** Every **claim finding**
   must carry both halves — the document making the claim, and the document or
   file settling it. Every **derived finding** must carry a cited Basis line
   for each figure it rests on, and its derivation written out (§5's second
   shape). In both cases every reference must resolve to a line that is really
   in the materials. A finding whose citation does not resolve does not ship (see
   §5). This catches a citation pointing at nothing. It will not catch one
   pointing convincingly at the wrong line.
7. **Deliver. Propose method-file edits (technique only).** A proposed edit
   states a technique, never a fact about this target: *if the lesson cannot
   be stated without naming the target, it is not a method lesson.* Read every
   proposed edit for proper nouns before including it. The working record is
   then copied out and the world destroyed — both by the practice, not by you.

## 12a. Practice review

<!-- audience: practice -->

**There is no independent review of the report, and that is a decision.** Step
6b is the author checking its own citations, which is a floor rather than a
guarantee. A second reviewer who took no part in the work is what the
assurance profession expects for engagements of consequence, and it is not
proportionate at this price. Revisit it before the first engagement where a
finding moves real money: a self-reviewed report is the weakest point in
§10's defence.

### The frozen claim surface

**Before a report goes to a client, the practice reviews the frozen claim
surface (§12 step 2) against the materials and confirms it.** The auditor does
not wait for this. §12 step 4 stands — there is no pause mid-engagement, and
the review happens after the run and before delivery.

**What to check, in about two minutes:**

- Does the surface cover every document in which the seller makes claims?
- Have all designated claim-source documents been enumerated, and have
  evidence-only documents been excluded from the claim surface? §12 step 1
  settles which is which, so only one answer is correct — this checks that it
  was followed, not which convention was chosen.
- Is the grain consistent across documents, or fine in one and coarse in
  another?
- Does the total match the per-document counts below it?

**If the surface is wrong, every coverage figure is wrong.** §1a forbids
stating a conclusion without the coverage it rests on, so correcting this
after delivery means reissuing under §7.

**Why a review step and not a tighter rule.** Measured 2026-08-25: three models
on the same nine documents returned 62, 67 and 273. Two attempts to define the
unit precisely enough to close that spread both failed. The first — "one
assertion that can take exactly one §6 verdict" — widened it to 44, 21 and
321, because "one assertion" has no fixed size: "the infrastructure is
redundant" is one assertion, and so are "there is one dyno", "the database is
co-located" and "there are no replicas". The second, now in §12 step 2,
produced 66 and 108 and left the third model unable to close the surface at all.
The residual disagreement is about scope, not grain: one model counted only the
seller's claim documents, another counted every checkable statement including
the evidence documents.

ISAE 3000 (Revised) para 24(b)(ii) requires criteria permitting "reasonably
consistent measurement or evaluation ... when used in similar circumstances by
different practitioners" (A45(c)); A46 excludes vague descriptions. This count
does not meet that test, and a person confirming it is cheaper and more
reliable than a third attempt at wording.

**The marker's shape is fixed for the same reason.** Measured 2026-08-24: on
first exposure, three models read "state the count after the marker" three ways
— one stated the count, one enumerated the surface and never counted it, one
quoted the instruction back. All three were defensible readings, which is why
§12 step 2 fixes the shape rather than describing it.

## 13. Where the method has been exercised

<!-- audience: practice -->

- **`measure/fixtures/dataroom/`** — the synthetic `flowmetrics` data room, 9
  documents, planted defects with an answer key. Used to validate the
  pipeline. Not a real SaaS.
- **`/home/bruce/projects/Body`** — a real, Bruce-owned autonomous mobile
  robot codebase, used as a second and real data room. 31 findings, zero
  deltas, **Clear with caveats**. Not a self-audit: the codebase was built by
  Claude/Cursor under Bruce's direction and Jill contributed neither code nor
  docs. Jill executed the audit independently — every verification, every
  finding, the report.

## 14. What carries between audits, and what must not

<!-- audience: practice -->

Retention periods, destruction dates, engagement-letter clauses and who
reviews a proposed edit are the practice's, not the auditor's. The one rule
the auditor needs from this section — a proposed method edit states a
technique and names no target — is inlined at §12 step 7, because a pointer
into a section the auditor never receives is no rule at all.

Each audit runs in its own world, which is discarded afterwards. That
protects client confidentiality but would also throw away everything learned
— so one channel is opened deliberately, and only one.

| | example | carries? |
|---|---|---|
| **method** | "a claimed rate may be a publish rate, not a check rate — verify which" | **yes** |
| **target** | "flowmetrics' backups had failed for 21 days" | **never** |

**The working record is kept. The world is still discarded.**

§10's defence against a negligence claim is that *"the citation trail
demonstrates the work was systematic"* — and a citation shows what you found,
never what you looked at.

**This argument changed on 2026-08-27 and the earlier version is worth
recording, because the fix for one gap closed a different one.** It used to
read: on a fifty-claim surface producing a dozen findings, the other
thirty-eight were checked, held, and left no trace, so the world held the only
evidence they happened. Under §4 that state no longer exists — every resolved
claim produces a finding, so all fifty are in the report with their citations.
The report now carries what the working record used to be needed for.

**What the report still does not carry is the negative space**, and that is
what a negligence claim asks about:

- **The searches behind an absence.** §5 requires two searches for "this is not
  implemented" and the report states the conclusion, not the queries. *"A
  documented search proves diligence, not absence"* — and the document is the
  trace, not the report.
- **The dead ends behind a finding.** A finding cites the file that settled it.
  It does not cite the six that did not, and "did you look properly" is a
  question about those six.
- **Every `[unverifiable]` claim's attempt.** The coverage statement gives the
  claim and the reason. The work that failed to settle it is in the trace.
- **Why an identified claim went unattempted.** §4's priority judgement is made
  in the working log and appears in the report only as a number.

**Nothing is authored to fix this.** The record already exists as a byproduct
of the work:

- `memory/reasoning_trace.jsonl` — every action, with the `thought` that
  motivated it, every observation returned, iterations and exit reason per leg.
- `inspect_traces/*.txt` — one file per evidence request: the query, every
  read and search inside it, what each returned, and the answer given back.

Together, for a single engagement, on the order of a hundred kilobytes. That is
a working paper in the professional sense — what was asked of the materials,
what came back, and why the next question followed. Its evidential value comes
precisely from being a byproduct: an auditor's own summary of its diligence is
the weakest evidence of that diligence, and this is not a summary.

**Copied out of the world before the world is destroyed, and retained under
the engagement letter.** The copy is made by the harness, not by the auditor —
§12 step 7 is the auditor's last step and it does not include this.

**This holds client material, and that is a real obligation.** Verbatim lines
from the data room are in those traces. §14's discard exists to avoid holding
client data; retaining the record reverses that deliberately, in exchange for
being able to evidence the work. It brings a retention period, a destruction
date, and a security obligation, and those belong in the engagement letter
before the next paying engagement — not here.

**The mechanism** for method learning is step 7 of §12: the final turn of every
audit proposes edits to this file, technique only. Bruce reviews and merges; the world is
then discarded. Learning is routed through a human review gate, which is where
a confidentiality check belongs, and it lands somewhere versioned and
diffable. This is what real firms do — a methodology manual updated after
engagements, not analysts carrying client details.

**The test for a clean lesson:** *if it cannot be stated without naming the
target, it is not a method lesson.* "Check whether the message broker binds to
all interfaces" is method. "Check whether Zenoh binds 0.0.0.0 like the robot
did" is a client fact wearing a technique's clothes. Read a proposed lesson
for proper nouns.

**Two consequences worth planning for:**

- **An aggregate findings corpus is a commercial asset and needs
  permission.** "The twelve gaps that show up most often in sub-$5M SaaS data
  rooms" is more valuable than the method prose, and the demo artifact already
  depends on it. Cross-client aggregation must be permitted by the engagement
  letter — a clause to add *before* the first paying client.
- **Method learning can make the method worse** by over-fitting to recent
  targets. Guard by re-running `measure/fixtures/dataroom/` after a method
  change and checking whether tier recall moved; keep at least one fixture in
  a different domain from recent real targets.

## 15. The Gap Map

**One page for the person who decides**, read in about thirty seconds (§8):

- Target name and a one-line description
- The §9 conclusion
- The three to five items that matter most
- The coverage line
- One line: "Full report with citations available on request."
- A footer in small type: "technical claims verification · not a pen-test,
  not legal advice"

Each key item is a one-line claim and a short note. No line citations; those
are in the report.

**When the conclusion is Clear or Clear with caveats, include what holds,
not only the caveats.** The buyer needs to see *what* is clear — "the payment
path is tested end to end" is the finding they are paying for — and a list of
caveats alone misrepresents an audit that mostly passed. Where the
conclusion is Material or Systemically inconsistent the balance inverts on its own.

**The coverage line** gives resolved out of identified, supported out of
resolved (§1a), and why the rest is or is not low-risk. The Gap Map
is the document most likely to be read *instead of* the report, so silence
about coverage does its damage here first.

If the conclusion is **Clear** with no caveats, the Gap Map is two lines:
"no gaps found, N of N identified claims resolved and supported, report
attached."

**No logo, no pricing, no "book a call".** The report link is the only call to
action. Anything more turns a professional document into a brochure.

## 16. The deliverable

Four blocks. Each opens and closes with a line of its own:

```
=== CLAIM SURFACE ===     …     === END CLAIM SURFACE ===
=== REPORT ===            …     === END REPORT ===
=== LIMITATIONS ===       …     === END LIMITATIONS ===
=== GAP MAP ===           …     === END GAP MAP ===
```

**Emit them in that order.** Take as many legs as the work needs — the blocks
are what the engagement is judged on, not how you divided the work. One leg may
carry several blocks.

**The order is not presentation.** The claim surface is the denominator every
coverage figure divides by, and a surface closed after the findings is not
frozen: it is a count made to fit. Close it before you check anything.

**Markers are how a block is delivered.** The client's process reads them.
Until a block's opening marker appears the engagement is still running, and you
will be told which block is missing. A closing marker says you finished writing
rather than ran out of room. Each marker goes on its own line, and nothing goes
between an opening marker and the content it introduces.

### The blocks

**`=== CLAIM SURFACE ===`** — as §12 specifies: the line after the marker
carries the count and nothing else, then the enumeration.

**`=== REPORT ===`**

**Length follows from the claim surface, not from a target.** Every resolved
claim gets a finding (§4, §5), so a large surface makes a long report and that
is correct. There is no word ceiling: no assurance standard bounds report
length, and a report that omits required content to hit a number has failed at
the thing the number was meant to protect.

Budget per finding instead. A claim that **holds** needs its verdict, its
citation and a clause — around twenty to thirty words. A claim that **does
not** needs the claim, the evidence that settles it, and what the gap costs the
buyer — around eighty. Order them worst first, so a reader who stops early has
read the expensive ones.

**The two things length pressure will suggest are the two you must not do:**
drop citations, or merge several claims into one finding. A finding carrying
three claims cannot be checked against any of them, and a report that demotes
findings to prose shrinks the surface anyone can review it against.

- The conclusion, in §9's vocabulary and no other.
- Findings worst first, each in §5's format. A **claim finding** carries its
  §6 verdict, the claim it tests, the document making that claim, and the
  document that settles it. A **derived finding** carries `[derived]`, a cited
  Basis line per figure, and its derivation (§5's second shape).
- A coverage statement in §1a's vocabulary — identified, resolved, supported —
  what was not attempted and why that matters (§4), and the **consistency
  rate**, supported over resolved;
  every `[unverifiable]` claim with the reason, and any `[unclaimed]`; and the
  addendum, if §12 step 2 produced one.
- What the client should ask the seller before closing.

No covering note, no account of what you did this leg, no preamble. Anything
written inside the block ahead of the conclusion *is* the report: it is
what the client reads first, and it displaces the finding that should have been
there.

**`=== LIMITATIONS ===`** — three lines, always present:

1. the materials examined, and their as-of date;
2. that the seller was **not consulted and has not confirmed your reading of
   their own claims**;
3. the assurance level (§1a) and the coverage it rests on.

The second is the one most often left out and the one that matters most. Every
finding interprets a claim without its author present to say what was meant,
and a reader who does not know that will over-read the report.

It is a block of its own and it belongs to the same delivered document as the
report block — the client receives one deliverable with these lines in it.
**Throughout this document, "the report" is the `=== REPORT ===` block and
"the deliverable" is all four blocks together.**

**`=== GAP MAP ===`** — as §15 specifies, and nothing else. Do not restate the
report, do not introduce it, do not append a note about it.

### Why the report and the Gap Map are both required, in this order

The Gap Map is read in thirty seconds; the report is read only if the Gap Map
earns it. The Gap Map alone forfeits the proof, the report alone forfeits the
reader, and neither is a summary of the other: the report carries the citations
and the Gap Map carries none.

### Why markers and not turn boundaries

This reverses an earlier decision, and the reason is worth stating so it is not
reversed back. These markers were removed once, on the argument that a turn
boundary separates the documents without either having to announce itself — so
that "if you are asked for the Gap Map, the report is done".

That does not hold. On 2026-08-27 an engagement ended a turn with its claim
enumeration and the sentence "I'll work the priority order straight through
now". The client's process had nothing it could test, took the enumeration as
the report, and asked for the Gap Map — which was then written about a document
that did not exist. **A turn boundary proves a turn ended. It does not prove a
document was written.**

A block says what it is. That is the whole reason the markers are back, and it
is why the method no longer says how many turns to take: once a block announces
itself, the turn boundary has no work left to do, and prescribing one would be
this method telling you how to pace work that is yours to pace.

### Considered and rejected: being asked for each block in turn

A process that requested the claim surface, then the report, then the
limitations, then the Gap Map would be driving the method rather than accepting
its results — deciding what happens next at every step, which is yours to
decide. It would also flatten the engagement: every auditor prompted through
the same four steps produces the same shape of work, and the method would stop
being something you follow.

Self-delimiting blocks are what make it unnecessary. You are asked for nothing
while the blocks arrive; you are told only when one has not.

## 17. Positioning and outreach

<!-- audience: practice -->

### The Gap Map as a designed artifact

Reference implementation: `docs/audit-gap-map-mockup.html`, populated with a
real audit as example data, which is why it lives outside `audit/` — target
facts do not carry between engagements (§14).

One card, about 680 px wide. Scan order top to bottom: target identity,
verdict band, key items, coverage, report link. The verdict band is the §9
conclusion, colour-coded, immediately under the header, green through red
across Clear to Systemically inconsistent, and it is the only place the
conclusion appears.

Open: whether the colour coding survives email-thumbnail scale, PDF export,
and the landing page it shares a visual system with. None of these block
sending one.

**The Gap Map is the lead artifact in cold outreach** — the partner gets it in
the email body or as a 30-second PDF. The Gap Map is the hook; the full report
is the proof.

Kept out of §15 deliberately: an agent writing a deliverable should not be told
it is writing a hook. Removals, retractions and the reasoning behind this split
live in `workflows/claims_audit/method/gap-analysis.md`, not here.

## 18. Provenance

<!-- audience: practice -->

Drafted 2026-08-22 by Claude from Jill's statements in `jill_chat` turns
3019–3045; **reviewed and corrected by Jill the same day** (turn 3047). Her
corrections, the verdict vocabulary, the conclusion taxonomy, the priority
order, the correction protocol and the run sequence are hers.

This file is the single durable source for the method: it lives in git so it
can be read as a unit, diffed, and corrected, rather than accumulating in an
agent's memory where none of that is possible. It was retitled 2026-08-25 —
it had been "AI-Readiness Audit", which named a service this method does not
perform.

Jill offered a second pass on 2026-08-22, before the ISAE 3000 / AT-C 205 gap
closure and the §12.2 work. Probably stale; re-ask rather than assume.
