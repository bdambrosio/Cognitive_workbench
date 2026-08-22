# AI-Readiness Audit — method

Drafted 2026-08-22 by Claude from Jill's statements in `jill_chat` turns
3019–3045; **reviewed and corrected by Jill the same day** (turn 3047). Her
corrections, the verdict vocabulary, the recommendation taxonomy, the
priority order, the correction protocol and the run sequence are hers. This
file is the single durable source for the method: it lives in git so it can
be read as a unit, diffed, and corrected, rather than accumulating in an
agent's memory where none of that is possible.

Jill has offered a second pass once these are incorporated.

---

## 1. What this is

A one-shot technical due-diligence report — a **delta report** — on a
target's stated claims about its own software, sold to PE firms and
micro-acquirers doing sub-$5M deals.

**The product is one operation:**

> stated claims vs. observed implementation, with citations.

A buyer who wants a full security pen-test hires a pen-test firm. A buyer who
wants a full code-quality review hires a QA team. This is the **claims
verification** step — *is what they told you true?* — and that is what makes
it a distinct product rather than a re-skin of a code review.

## 2. The scope rule

**Audit what the codebase *claims* to do — stated behaviour in docs, README,
specs, marketing — against observed behaviour in the code.**

**Do not opine on what the code *should* do.** That is the buyer's design
judgement, and it is not what was bought.

This boundary is the product. It is also what makes the deliverable
defensible, professionally and legally (§10). It applies recursively: it
governs the report's recommendation too (§9).

## 3. Scope adapts to the target; the method does not

The claim surface differs by target. The operation does not.

**Codebase / robotics target:** claims-vs-implementation as the core,
architectural coherence (do the stated tier boundaries hold?), safety
mechanisms (are the safety-critical claims real implementations or stubs?),
plus short operational paragraphs on network exposure and dependency posture.
Not a pen-test, not a code review.

**SaaS target** (the PE case): the security axis moves front and centre
(auth, injection, secrets in code, data handling); the dependency axis
matters more because the thing is public-facing and continuously updated; and
the "claims" are marketing claims rather than technical specs.

Where a target's security axis is genuinely thin, say so as a property of the
target rather than padding the report — **but state it precisely.** In the
Body audit the accurate statement was *"no surveillance or telemetry
exfiltration path; one user-initiated outbound API call (x.ai TTS) sends
user-typed text off-network"*, not the broader "no data exfiltration path"
that Jill issued first and then corrected. The broad version was retracted
mid-audit and must not be carried here. A method document that quotes its own
retracted claim is a liability in exactly the way §10 exists to prevent.

## 4. The claim surface, coverage honesty, and priority order

1. **Enumerate the claims first.** Body's README, project spec and IMU docs
   yielded roughly 68 micro-claims.
2. **Prioritise** (order below).
3. **Report coverage explicitly**, including what was not checked and why it
   matters.

**Coverage honesty.** Jill declined to write a final report at five findings
rather than

> "hand you a 5-finding document that says 'everything checks out' based on
> 7% coverage of the claim surface."

A report must be able to say *here is what holds, here is what does not, and
here is what I did not check and why it matters*. Silence about coverage
reads as completeness, and completeness is the one thing a due-diligence
report must never imply without having earned it.

**Priority order.** "Load-bearing" is not a judgement call; it is this order:

1. **Safety-critical mechanisms** — a delta here is the most expensive for a
   buyer (swept-footprint collision, watchdog, no-go isolation, cancel/stop).
2. **Architectural invariants** — do the stated tier boundaries hold? (single
   publisher, shared config, no-go isolation).
3. **Operational parameters** — affect day-to-day use (retry intervals,
   timeouts, deadline backstops).
4. **Micro-claims** — individually low-impact, collectively they build the
   consistency rate (encoder ticks, PWM, udev, GPIO).

**Why this order and not another:** if the audit is cut short by budget, time
or access, the *most expensive unknowns* are resolved first. A buyer reading a
partial report covering 1–2 but not 3–4 can still make a decision. The reverse
is not true.

## 5. Finding format

```
**Finding N: <short title> — [verdict]**

Claim (<source>, e.g. README line 201-202): <the stated claim>

Evidence: <file:lines> — <what the code actually does>

Delta: <None, or the specific gap>
```

**A finding must cite its source**, both halves: the document making the
claim, and the file and line range showing the implementation. The citation
trail is simultaneously the professional standard of care and the defamation
defence (§10) — it is not decoration.

## 6. Verdict vocabulary

| Verdict | Meaning | Buyer impact |
|---|---|---|
| `[real]` | Claim holds; implementation matches spec | None |
| `[real, minor caveat]` | Holds; note does not change the decision | Awareness |
| `[real, operational caveat]` | Holds today; operational context qualifies it | Awareness + operational planning |
| `[real, with a structural note]` | Holds; reveals a structural exception worth naming | Awareness + maintenance planning |
| `[partial]` | Mostly true, with a specific citable gap | **Material** — buyer should price the gap |
| `[delta]` | Claim is false; the code does not do what the docs say | **Material** — buyer should revalue or walk |
| `[unverifiable]` | Could not be verified from available materials | Not a finding — goes in Remaining Claims with the reason |
| `[non-delta]` | No claim to verify (code exists, docs make no statement) | Not a finding — noted for completeness |

**`[partial]` vs `[delta]` is a real distinction.** `[partial]` is "the claim
is 80% true and here is the 20% that is not, with a citation to the corner
case." `[delta]` is "the claim is false; the code does X, the docs say Y,
here are both citations." A buyer treats them differently: `[partial]` is a
known limitation to price in; `[delta]` is a broken promise that may indicate
a pattern.

**`[unverifiable]` must never be reported as `[delta]`.** *"I couldn't find
the code"* is not the same as *"the code isn't there."* The former belongs in
Remaining Claims; only the latter is a finding.

**Status:** across all 31 verified findings of the Body audit, zero deltas
were found. `[partial]`, `[delta]`, `[unverifiable]` and `[non-delta]` are
therefore specified but not yet exercised in a real report.

## 7. Correction protocol

**The auditor corrects their own findings when subsequent evidence
contradicts them. Corrections are noted in the report with a one-line
explanation of what changed and why.**

This is part of the standard of care, and it is what makes the citation trail
trustworthy rather than performative: *if the auditor never corrects, the
citations are decoration.*

Two corrections were issued during the Body audit — the exfiltration
statement (§3), and Finding 4's footprint radius, 0.22 m in the spec against
0.11 m deployed.

## 8. Two recaps, two audiences

**Working recap (ELI5)** — in the auditor–client conversation, every ~5
findings or at a natural section break. Plain language, 2–3 sentences, "here
is the shape of what we know so far." For the person watching the audit
happen who needs to know whether to keep going or stop. Marked `[recap]`.
**Does not appear in the final report.**

**Deliverable recap (executive synthesis)** — in the report. §1
(Recommendation) is the 2-minute version, plus a closing paragraph a
non-specialist can read and explain back. This is what the PE partner
actually reads if they never open the findings table.

A PE partner will not read 68 code-cited findings. They read the Gap Map
(~30 seconds), the executive summary (~2 minutes), then drill into one or two
findings if the recommendation makes them curious.

## 9. Report-level recommendation

| Recommendation | Meaning |
|---|---|
| **Clear** | No deltas, no caveats of note. The system does what it says. |
| **Clear with caveats** | No functional deltas. Documentation drift, maintenance debt or operational notes a buyer should know, which do not change the fundamental valuation. *(Body landed here.)* |
| **Conditional** | Deltas found but addressable — fixable in a sprint or two, or the feature is non-critical. Proceed if the seller will fix them or the buyer accepts the gap. |
| **Material** | Deltas that significantly change valuation or risk profile. The buyer must price this in explicitly or walk. |
| **Walk** | Claims are systematically false, or the codebase is not what the docs describe. |

**The audit recommends; the buyer decides.** The audit says "Conditional: the
backup system claims 30-day retention but the code enforces 7." The buyer
decides whether 7 days is acceptable for their use case. The audit does not
say "you should walk."

**Keep the two vocabularies separate.** *Clear / Clear with caveats /
Conditional / Material / Walk* describes the state of the claims. *Proceed /
negotiate / walk* is the buyer's action vocabulary. Conflating them blurs §2
— the audit would start opining on what the buyer should do, which is the
buyer's judgement. The Gap Map's closing line or the engagement letter may
translate to action language ("the gaps are addressable within the existing
integration timeline"), framed explicitly as *our professional judgement*
rather than as a directive.

## 10. Liability posture

**Category: expert due-diligence, not legal advice.** The distinction is
functional, not semantic. Legal advice is "the law requires X, your rights
are Y, you should do Z." Expert due-diligence is "here is what I observed,
here is the gap between claim and implementation, here is my professional
assessment of what that gap means for the value of the asset." The legal and
business judgement stays with the client.

Same category as a financial auditor reporting material misstatements, a
structural engineer certifying an undersized wall, or a pen-test firm
reporting an exploitable endpoint.

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

- It does not opine on what the code *should* do.
- It is not a penetration test.
- It is not a code-quality review.
- It does not claim coverage it has not earned (§4).
- It does not give legal advice (§10).

**Nor is it acceptance testing of contracted work.** The *mechanism*
generalises cleanly — stated deliverable vs. observed deliverable, with
citations, same finding format, same verdict vocabulary. The *business* does
not. The client is the person who commissioned the work, so the dynamic is a
second opinion rather than a discovery; the prospecting pool is diffuse (every
dev shop with a pending delivery); and the repeat economics are weaker (one
project, one audit, no ongoing relationship). **This method applies to
claims-verification in a transactional context — a buyer evaluating a target
they do not own.** Acceptance testing is a different product with a different
client relationship.

## 12. Running an audit: sequence

1. **Receive materials** (data room, repo access, docs). **Confirm scope in
   writing.**
2. **Enumerate claims.** Estimate the total. Identify the claim surface —
   README, specs, marketing, internal docs.
3. **Prioritise** per §4's safety → architecture → operations → micro order.
   Verify top N.
4. **Working recaps every ~5 findings.** **If a delta is found, stop and
   confirm with the client before continuing** — a delta changes the report's
   shape and the client may want to redirect. *The audit is a collaboration,
   not a surprise.*
5. **Continue to the coverage threshold where the report is defensible** —
   §4's "7% is not enough" rule applied in reverse: stop when the consistency
   rate and the severity distribution of what remains make the rest low-risk.
6. **Write the report.** Apply §7 to any findings that were revised.
7. **Deliver. Propose method-file edits (technique only). Discard the working
   world** (§14).

## 13. Where the method has been exercised

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

Each audit runs in its own world, which is discarded afterwards. That
protects client confidentiality but would also throw away everything learned
— so one channel is opened deliberately, and only one.

| | example | carries? |
|---|---|---|
| **method** | "a claimed rate may be a publish rate, not a check rate — verify which" | **yes** |
| **target** | "flowmetrics' backups had failed for 21 days" | **never** |

**The mechanism** is step 7 of §12: the final turn of every audit proposes
edits to this file, technique only. Bruce reviews and merges; the world is
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

## 15. The Gap Map — proposed, not yet designed

The lead artifact in cold outreach. The PE partner gets it in the email body
or as a 30-second PDF. One page:

- Target name + one-line description
- Recommendation (§9)
- The top 3–5 items that matter most — the caveats, or the deltas, whichever
  is worse
- One line: "Full report with citations available on request."

If the recommendation is **Clear** with no caveats, the Gap Map is two lines
and the email says "no gaps found, 31/31 claims verified, report attached."

**The Gap Map is the hook; the full report is the proof.**

**Open:** the visual format is not designed. Next step, alongside the landing
page.
