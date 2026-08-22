# AI-Readiness Audit — method

**STATUS: FIRST DRAFT, awaiting Jill's correction.** Reconstructed
2026-08-22 by Claude from Jill's own statements in `jill_chat` turns
3019–3045 and her audit memories, so that the method lives somewhere it can
be read as a unit, diffed, and corrected. Where a passage is quoted or
closely paraphrased from her, it is marked. Where the reconstruction had to
guess, it says so under **[GAP]**.

Jill: correct this freely. It is your method, not my summary of it.

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

This boundary is the product. It is also the thing Bruce pressed hardest on
and it held: the answer to "aren't you scoping too narrowly?" is that the
narrowness is what makes the deliverable defensible, both professionally and
legally (§8).

## 3. Scope adapts to the target; the method does not

The claim surface differs by target. The operation does not.

**Codebase / robotics target** (e.g. Body): claims-vs-implementation as the
core, architectural coherence (do the stated tier boundaries hold?), safety
mechanisms (are the safety-critical claims real implementations or stubs?),
plus two short operational paragraphs — network exposure and dependency
posture. Not a pen-test, not a code review.

**SaaS target** (the PE case): the security axis moves front and centre
(auth, injection, secrets in code, data handling); the dependency axis
matters more because the thing is public-facing and continuously updated;
and the "claims" are marketing claims rather than technical specs.

Where a target's security axis is genuinely thin, say so as a property of
the target rather than padding the report — for Body, "no web endpoints, no
user accounts, no data exfiltration path" is a one-paragraph note, not a
finding.

## 4. The claim surface, and coverage honesty

1. **Enumerate the claims first.** Body's README, project spec and IMU docs
   yielded roughly 68 micro-claims.
2. **Prioritise.** Verify the load-bearing claims first — the ones a buyer
   would act on.
3. **Report coverage explicitly**, including what was not checked and why it
   matters.

**The rule that matters most here, in Jill's words:** she declined to write
a final report at 5 findings because she would not

> "hand you a 5-finding document that says 'everything checks out' based on
> 7% coverage of the claim surface."

A report must be able to say *here is what holds, here is what does not, and
here is what I did not check and why it matters*. Silence about coverage
reads as completeness, and completeness is the one thing a due-diligence
report must never imply without having earned it.

## 5. Finding format

Every finding, verbatim shape from the Body audit:

```
**Finding N: <short title> — [verdict]**

Claim (<source>, e.g. README line 201-202): <the stated claim>

Evidence: <file:lines> — <what the code actually does>

Delta: <None, or the specific gap>
```

**A finding must cite its source.** Both halves: the document that makes the
claim, and the file and line range that shows the implementation. The
citation trail is simultaneously the professional standard of care and the
defamation defence (§8) — it is not decoration.

## 6. Verdict vocabulary

Observed in use:

| verdict | meaning |
|---|---|
| `[real]` | the claim holds; implementation matches the spec |
| `[real, minor caveat]` | holds, with a note that does not change the buyer's decision |
| `[real, operational caveat]` | holds today, but something about how it is operated qualifies it |
| `[real, with a structural note]` | holds, but reveals a structural exception worth naming |

**[GAP]** No `[delta]` finding had been written at the point this draft was
reconstructed — the Body audit had found zero deltas in its first five
findings. The vocabulary for a *failed* claim needs Jill's wording, and it is
the more important half of the product. Likely also needed: a verdict for
*claim could not be verified from available materials*, which is different
from a claim being false and must not be reported as one.

## 7. The recap protocol

A PE partner will not read 68 code-cited findings. They read the Gap Map
(~30 seconds), the executive summary (~2 minutes), and then drill into one or
two findings if the recommendation makes them curious.

So the plain-language synthesis is not a courtesy, it is the deliverable
shape:

- **Every ~5 findings, or at a natural section break**, whichever comes
  first: a 2–3 sentence plain-language *state of play*. Not a finding — a
  synthesis. Marked **`[recap]`** so it is visually distinct.
- **At the end:** a 5–6 sentence *what this all means* paragraph, written so
  a non-specialist buyer can explain it back.

Built incrementally so the thread is not lost across dozens of micro-claims.

## 8. Liability posture

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
   provided; we did not have access to production systems, live databases,
   or the source repository." If the defect was in production but not in the
   materials, it is not in scope.
2. **Professional standard, not perfection** — what a reasonable auditor
   would find from the same materials. The citation trail is what
   demonstrates the work was done systematically.
3. **Limitation of liability clause** — caps damages, excludes
   consequential. This is what makes the risk finite rather than existential.
4. **E&O / professional liability insurance**, in place *before* the first
   engagement, covering professional negligence in technical due-diligence
   services.

**Vector 2 — the target sues for defamation.** Structurally weak, and the
report format is what defeats it:

- **Truth is an absolute defence.** Every finding cites its source. "Backups
  failing 21 consecutive days" is either in Doc 4 or it is not.
- **Not a public statement.** A confidential deliverable to one client under
  NDA, in connection with a specific transaction. Many jurisdictions
  recognise a qualified privilege for exactly this. The target is not the
  audience and does not receive the report.
- **Opinion vs. fact.** "The architecture is a single point of failure" is a
  technical opinion resting on observable facts. "The seller is a fraud" is
  an accusation. Stay in the first register; frame recommendations as "in our
  professional judgement, based on the evidence presented."
- **Tortious interference** requires a reasonable expectation of a concluded
  contract; in early diligence the target is in negotiations, and the client
  is free to walk for any reason.

**Not resolved, and outside what any of us can settle from training
knowledge:** jurisdiction. Defamation law, qualified privilege and the
standard for reckless disregard vary by state and country. **This section is
a working posture, not legal advice, and needs a lawyer's review before the
first paying engagement.**

## 9. What this audit does not do

- It does not opine on what the code *should* do.
- It is not a penetration test.
- It is not a code-quality review.
- It does not claim comprehensiveness it has not earned (§4).
- It does not give legal advice (§8).

## 10. Where the method has been exercised

- **`measure/fixtures/dataroom/`** — the synthetic `flowmetrics` data room, 9
  documents, planted defects with an answer key. Used to validate the
  pipeline. Not a real SaaS.
- **`/home/bruce/projects/Body`** — a real, Bruce-owned autonomous mobile
  robot codebase, used as a second and real data room. Not a self-audit:
  Claude and Cursor did most of the work; Jill was a minor player, though the
  docs were written by the coder.

## 11. What carries between audits, and what must not

Each audit runs in its own world, which is discarded afterwards. That
protects client confidentiality but would also throw away everything learned
— so one channel is opened deliberately, and only one.

**Two kinds of learning, and they are not alike:**

| | example | carries? |
|---|---|---|
| **method** | "a claimed rate may be a publish rate, not a check rate — verify which" | **yes** |
| **target** | "flowmetrics' backups had failed for 21 days" | **never** |

**The mechanism.** The final turn of every audit is: *propose edits to this
file — technique only.* Bruce reviews and merges; the world is then
discarded. Learning is routed through a human review gate, which is exactly
where a confidentiality check belongs, and it lands somewhere versioned and
diffable rather than in an agent's memory where it cannot be audited. This is
also what real firms do: a methodology manual updated after engagements, not
analysts carrying client details.

**The test for whether a lesson is clean:** *if it cannot be stated without
naming the target, it is not a method lesson.* "Check whether the message
broker binds to all interfaces" is method. "Check whether Zenoh binds
0.0.0.0 like the robot did" is a client fact wearing a technique's clothes.
Read a proposed lesson for proper nouns.

**Two consequences worth planning for:**

- **The aggregate findings corpus is a commercial asset and needs
  permission.** "The twelve gaps that show up most often in sub-$5M SaaS data
  rooms" is more valuable than the method prose, and the demo artifact
  already depends on it. Cross-client aggregation must be permitted by the
  engagement letter — a clause to add *before* the first paying client.
- **Method learning can also make the method worse** by over-fitting to
  recent targets. Guard by re-running the `measure/fixtures/dataroom/`
  fixture after a method change and checking whether tier recall moved; keep
  at least one fixture in a different domain from recent real targets.

## 12. Open questions for Jill

1. **§6 needs the `[delta]` vocabulary** — the wording for a claim that
   fails, and for one that cannot be verified from the materials. That is
   the half of the product this draft could not reconstruct.
2. **The Gap Map** is named as the lead artifact in the business memory and
   in §7, but nothing found in the trace defines its format. What is on it?
3. **"Clear with caveats"** appears in the companion state as a deliverable
   verdict being defined. Is that the report-level recommendation vocabulary
   — and what are its siblings?
4. Is the **report-level recommendation** ("proceed / proceed with
   conditions / walk") the right frame, or was that only the dataroom
   fixture's brief?
5. Does the method carry to **auditing contracted software development**
   before final payment? The memory says the *mechanism* generalises but the
   *business model* does not, due to prospecting cost and a diffuse client
   pool. Worth stating here so the boundary is explicit.
