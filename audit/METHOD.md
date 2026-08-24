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

**A figure must be obtained verbatim, never from a summary.** Tools that
read on the auditor's behalf answer in prose, and prose summaries carry
qualitative facts reliably and numbers unreliably — observed 2026-08-22: a
document read returned "backup failures for 21 days" while silently dropping
the last-good-backup date on the very next line of the same document, and
with it the arithmetic that made the finding material. Nothing marks the
omission; the summary reads complete.

**AN ABSENCE NEEDS THE SEARCH, NOT THE CONCLUSION.** "This is not
implemented" cannot cite a file and line — the point is that there isn't one.
Its evidence is the search. Show two, because each fails where the other
holds:

- **Lexical, in the claim's own words.** The claim supplies the terms, and
  the implementer probably used them too. A claim about detecting frustration
  is searched as `frustrat`, not as whatever you would have named the module.
- **Structural.** List the directory the thing would live in. A listing
  cannot be defeated by a synonym; a grep can.

Observed 2026-08-23. A finding that a documented feature was unimplemented
grepped the term, showed all ten matches meant something else, and ruled out
the event path that could have carried it — correct, and it held up. A second
finding asserted no sentiment scorer existed. One did: 136 lines, wired into
every customer message. It had been searched for as a module *driving the
escalation*, found nothing, and generalised. Different claims; the first true,
the second false.

An absence is the most valuable thing an audit reports and the easiest to get
wrong, because nothing contradicts it until the seller does. And a documented
search proves diligence, not absence — a well-recorded search for the wrong
word is worse than none, because it reads as rigour.

So for any claim resting on a number, date, or threshold: ask for the source
line and quote it. A finding built on a remembered figure has the same defect
as a finding built on a remembered file — it is the provenance failure §5
exists to prevent, wearing a citation it did not earn.

### The second shape: a derived finding

The format above tests a stated claim. Some of the most material things in a
data room are stated by nobody — they follow by arithmetic from two figures
the seller supplied separately, in documents that never meet. A retention
window and a last-good-backup date sit in different sections of the same
file; each is innocuous, and together they carry a date after which the
business cannot be restored.

That finding has no claim to cite, so the format above has no room for it. It
gets its own:

```
**Finding N: <short title> — [derived]**

Basis: <file:lines> — <the first stated figure, verbatim>
       <file:lines> — <the second, verbatim>

Derivation: <the arithmetic, written out so a reader can check it>

Consequence: <what follows, and why a buyer cares>

Escalates: <Finding N, or None>
```

**Four rules, and they are what keep this inside §2.**

1. **Only the target's own figures.** Every input is a number, date or
   threshold the seller stated, quoted verbatim per the rule above. Deriving
   an expiry date from the seller's own retention terms is still auditing
   what they told you. Deriving that the market will turn against them is
   not, and is not a finding.

2. **The derivation is written out, not asserted.** "30-day retention from a
   2026-07-30 last-good backup exhausts on 2026-08-29" is checkable in one
   line. A consequence whose arithmetic is left implicit is an opinion
   wearing a citation, and §5 exists to prevent exactly that.

3. **No recommendation inside the finding.** State the consequence and stop.
   What the buyer should do about it is the buyer's judgement (§2), and the
   report's own recommendation is §9's job.

4. **Name what it escalates.** A derived finding frequently sharpens an
   ordinary one — the same backup failure, now with a date on it. Say which,
   so the report reads as one finding intensified rather than two findings
   counted. Where it escalates nothing, say None.

**Priority.** A derived finding inherits the §4 priority of the facts it
rests on, and rises above them when the consequence is more severe than
either input. The backup case is the pattern: two operational facts, one
consequence a buyer would treat as material.

**Why this was missing.** Measured 2026-08-23: three independent arms found
every figure needed for a derived finding, and all three declined to state
it. One wrote out the setup and stopped before the arithmetic; one said the
lines "do not establish" the conclusion; one filed it as `(duplicate)` of the
claim it escalated. That is not three models failing. That is three models
obeying a format with one shape in it.

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
| `[derived]` | Not a claim test — a consequence computed from two or more stated figures (§5's second shape) | **As severe as the consequence.** Frequently material |

**`[partial]` vs `[delta]` is a real distinction.** `[partial]` is "the claim
is 80% true and here is the 20% that is not, with a citation to the corner
case." `[delta]` is "the claim is false; the code does X, the docs say Y,
here are both citations." A buyer treats them differently: `[partial]` is a
known limitation to price in; `[delta]` is a broken promise that may indicate
a pattern.

**`[derived]` is not a verdict on a claim, and that is the point.** The other
eight say how a stated claim stood up. `[derived]` says the seller's own
figures, taken together, entail something neither document states. It carries
no claim source because there is no claim — which is why it needs §5's second
shape rather than a row in the format above.

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

Launched with `scenarios/audit.yaml` — one scenario for every engagement,
with two per-target lines (`world_name`, `external_repo`). Its header carries
the operational detail; this section is the method.

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
6b. **Verify the report against its own citations.** Every claim resolves to
   a line in the materials, or it does not ship — §5 applied to the finished
   deliverable rather than trusted to the author. It catches a claim citing
   nothing, not one citing the wrong line convincingly: a floor, not a
   guarantee.
7. **Deliver. Propose method-file edits (technique only). Discard the working
   world** (§14).

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

## 15. The Gap Map

The lead artifact in cold outreach. The PE partner gets it in the email body
or as a 30-second PDF. One page:

- Target name + one-line description
- Recommendation (§9)
- The top 3–5 items that matter most
- Coverage line
- One line: "Full report with citations available on request."

If the recommendation is **Clear** with no caveats, the Gap Map is two lines
and the email says "no gaps found, N/N claims verified, report attached."

**The Gap Map is the hook; the full report is the proof.**

### The format

Designed 2026-08-22. Reference implementation:
`docs/audit-gap-map-mockup.html` — populated with a real audit as example
data, which is why it lives outside `audit/` (§14: target facts do not carry
between engagements).

**One card, ~680 px wide.** Scan order top to bottom is deliberate: target
identity → verdict band → key items → coverage → full-report link. A partner
glancing at it in an email gets the verdict in about three seconds, then
enough substance to decide whether to open the report.

**Verdict band.** The §9 recommendation, colour-coded, immediately under the
header. All five levels have a treatment — the band is the first thing that
registers, and it is the only place the recommendation appears. Green through
red across Clear → Walk.

**Key items — three to five, and MIX THEM.** Each item is an icon, a
one-line claim, and a two-line note. No `file:line` citations; those live in
the report.

The non-obvious rule: **when the recommendation is Clear or Clear with
caveats, include the positive confirmations, not only the caveats.** The
buyer needs to see *what* is clear, not merely that it is — "the
safety-critical mechanisms are real implementations, not stubs" is the
finding they are paying for, and a list of only caveats misrepresents an
audit that mostly passed. Where the recommendation is Material or Walk the
balance inverts on its own.

**Coverage line.** How many claims were individually verified out of how many
identified, and why the remainder is or is not low-risk. This is §4's
coverage honesty at artifact scale: the Gap Map is the document most likely
to be read *instead of* the report, so silence about coverage does its damage
here first.

**Footer.** The report link, and the scope disclaimer in small type —
"technical claims verification · not a pen-test, not legal advice". That is
§2's boundary and §10's liability posture enforced at the artifact level,
where a recipient who reads nothing else will still see it.

**Deliberately absent: no logo, no pricing, no "book a call".** A lead
artifact's job is to make the recipient want the full report. The link is the
call to action; anything more converts a professional document into a
brochure and forfeits the register the disclaimer just established.

**Open:** whether the colour coding survives email-thumbnail scale, PDF
export, and the landing page it shares a visual system with. None of these
block sending one.

## 16. The deliverable

Two documents, produced together in the final turn, in this order.

**1. The report.** Aim for 2,000 words or under.

- The recommendation, from §9's taxonomy and no other vocabulary.
- Findings worst first, each in §5's format, carrying its §6 verdict, the
  claim it tests, the document making that claim, and the document that
  settles it.
- A coverage statement: what was not checked, and why that matters (§4).
- What the client should ask the seller before closing.

**2. The Gap Map** (§15), after a line reading exactly:

```
=== GAP MAP ===
```

The marker is load-bearing and must appear on its own line. It is how a
reader — and any harness driving the engagement — separates the two
documents, and its absence is the signal that the work is unfinished. An
engagement that ends without it has not delivered, whatever the closing
paragraph says.

**Why both, and why together.** The Gap Map is read in thirty seconds and
the report is read only if the Gap Map earns it (§8). Producing the Gap Map
alone forfeits the proof; producing the report alone forfeits the reader.
Neither is a summary of the other: the report carries citations and the Gap
Map carries none.
