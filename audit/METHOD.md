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

## 1a. Level of assurance

**This is limited assurance over a disclosed subset of the claim surface, and
the conclusion is expressed positively over that subset only.**

The distinction matters because it fixes what the report means. An engagement
offering *reasonable* assurance examines enough to reduce risk to acceptably
low and concludes positively over the whole subject matter. This engagement
does not: §4 prioritises, §12 stops at a coverage threshold, and §11 lists
what is out of scope. So the conclusion in §9 says something narrower than it
appears to — *of the claims examined, here is how they stood up* — and the
coverage statement is not a footnote to it but part of it.

**Two consequences that bind.**

1. **A §9 recommendation may not be stated without the coverage it rests on.**
   The recommendation and the coverage line travel together, in the report and
   in the Gap Map. A verdict without a denominator is not a conclusion.
2. **Never write a sentence that implies the unexamined was examined.** "The
   system does what it says" claims the whole surface. "Of the 43 claims
   verified, 39 hold" claims what was done. §4's coverage honesty is this rule
   applied to prose.

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

**Materiality — what makes a gap worth reporting at all.**

A gap is material if a reasonable buyer, knowing it, **would change the price,
the structure of the deal, or the decision to close.** That is the threshold,
and it is transaction-relative rather than absolute: the same defect is
material in a $400k acquisition and noise in a $40m one.

Three things follow. **Report every material gap regardless of which way it
pushes the price** — §10's independence rule. **A gap below the threshold is
not a finding**; it belongs in the consistency rate, not the findings table.
And **materiality is judged against the buyer's decision, not the auditor's
taste** — "this code is poorly organised" is not material unless it bears on
what was claimed.

Without this, "load-bearing" is whatever the reporter thinks it is. Measured
2026-08-24: three arms on one fixture reported 6, 3 and 2 unplanted findings.
Part of that spread is three reporters placing an undefined threshold in three
different places.

**Priority order.** Within what is material, verify in this order:

1. **Claims whose failure ends the business** — data that cannot be recovered,
   money that cannot be collected, a dependency that can withdraw. In a
   physical product this tier is safety-critical mechanism; in a SaaS target
   it is recoverability, payment integrity and single points of control.
2. **Architectural invariants** — do the stated boundaries hold? Isolation,
   redundancy, failover, tier separation, shared state.
3. **Operational parameters** — retry intervals, timeouts, monitoring,
   retention windows, thresholds affecting day-to-day operation.
4. **Micro-claims** — individually low-impact; collectively they establish the
   consistency rate, which is the strongest available signal about the claims
   nobody had time to check.

**The tiers are defined by consequence to the buyer, not by subsystem.** They
were first written from a robotics engagement and read as robotics — swept
footprints, watchdogs, encoder ticks, PWM — which left tier 1 empty for every
SaaS target the product is actually sold to (§1). Restated 2026-08-24 against
three engagements: a robot, a chat product, and a SaaS data room. If a tier is
empty for a target, say so in the coverage statement rather than silently
running a three-tier order.

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

**A derived finding that resolves to a date carries that date as its own
expiry.** The backup case is the pattern: *"the last recoverable backup ages
out on 2026-08-29"* is true when written, catastrophic five days later, and
meaningless a month after that. State the date, and state what happens on it.
The report as a whole carries an **as-of date for the materials**, because a
data room is a snapshot and every conclusion in the report is a conclusion
about that snapshot.

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

**Every verdict above is exercised.** Measured across three engagements —
`[delta]` 35, `[real]` 29, `[partial]` 20, `[derived]` 11, `[real, minor
caveat]` 11, `[unverifiable]` 8, `[non-delta]` 5, `[real, operational
caveat]` 4. Use the narrowest one that fits.

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

**There is no working recap, and there was no channel for one.** This section
specified one — plain language, every ~5 findings, for the person watching the
audit happen. It was never performable: the audit runs as one engagement with
no channel back to the client mid-flight, so every run silently failed a
requirement nobody could meet. An unperformable procedure is worse than an
absent one; it reads as rigour and produces nothing. Removed 2026-08-24.

If a mid-engagement channel is ever built, this comes back — with §12's
delta-confirmation step, which was removed for the same reason.

**Deliverable recap (executive synthesis)** — in the report. §1
(Recommendation) is the 2-minute version, plus a closing paragraph a
non-specialist can read and explain back. This is what the PE partner
actually reads if they never open the findings table.

A PE partner will not read 68 code-cited findings. They read the Gap Map
(~30 seconds), the executive summary (~2 minutes), then drill into one or two
findings if the recommendation makes them curious.

**One audience, then: the reader of the finished document.** Tier the
deliverable to that reader.

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
2. **Enumerate claims, then CLOSE the surface.** Estimate the total and
   identify the claim surface — README, specs, marketing, internal docs. When
   enumeration ends the surface is **frozen**: it is the denominator every
   coverage figure in the report is measured against, and a denominator that
   moves makes every percentage before it mean something else. A claim met
   later that was missed here goes in an **addendum** with its own count, not
   into the original population.
3. **Prioritise** per §4's safety → architecture → operations → micro order.
   Verify top N.
4. **Work the priority order straight through.** Do not pause for
   confirmation; there is no channel to the client mid-engagement. Deltas are
   reported in the deliverable, where the client can act on them.
5. **Continue to the coverage threshold where the report is defensible** —
   §4's "7% is not enough" rule applied in reverse: stop when the consistency
   rate and the severity distribution of what remains make the rest low-risk.
6. **Write the report.** Apply §7 to any findings that were revised.
6b. **Verify the report against its own citations.** Every claim resolves to
   a line in the materials, or it does not ship — §5 applied to the finished
   deliverable rather than trusted to the author. It catches a claim citing
   nothing, not one citing the wrong line convincingly: a floor, not a
   guarantee.
7. **Deliver. Propose method-file edits (technique only).** The working record
   is then copied out and the world destroyed (§14) — both by the practice,
   not by you.

**There is no independent review, and that is a decision rather than an
oversight.** Step 6b is self-review — the author checking its own citations —
and the method says plainly it is a floor, not a guarantee. A second reviewer
who took no part in the work is what the assurance profession requires for
engagements of consequence, and it is not proportionate at this price point.
Revisit it before the first engagement where a finding moves real money; a
self-reviewed report is the weakest link in §10's defence.

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

**The working record is kept. The world is still discarded.**

§10's defence against a negligence claim is that *"the citation trail
demonstrates the work was systematic"* — and the citation trail in the report
covers only what was reported. On a fifty-claim surface producing a dozen
findings, the other thirty-eight were examined, held, and left no trace. A
negligence claim asks about those. Discarding the world destroyed the only
evidence they happened.

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

**A one-page summary for the decision-maker**, who reads it in about thirty
seconds and opens the report only if it earns the time (§8). One page:

- Target name + one-line description
- Recommendation (§9)
- The top 3–5 items that matter most
- Coverage line
- One line: "Full report with citations available on request."

If the recommendation is **Clear** with no caveats, the Gap Map is two lines
and the email says "no gaps found, N/N claims verified, report attached."

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

**The final turn carries these two documents and nothing else.** No covering
note, no "here is what I did this leg", no continuation prose ahead of the
report. Anything written before the report *is* the report: it counts against
the word budget and it sets the ordering, because it is what the client reads
first. A leg that opens by narrating the previous leg has buried its own lede
before the first finding.

**1. The report.** Aim for 2,000 words or under.

- The recommendation, from §9's taxonomy and no other vocabulary.
- Findings worst first, each in §5's format, carrying its §6 verdict, the
  claim it tests, the document making that claim, and the document that
  settles it.
- A coverage statement: what was not checked, and why that matters (§4).
- What the client should ask the seller before closing.

- **The inherent-limitations statement.** Three lines, always present:
  the materials examined and their as-of date; that the seller was **not
  consulted and has not confirmed the auditor's reading of their own claims**;
  and the assurance level (§1a) with the coverage it rests on. The second is
  the one most easily forgotten and the most load-bearing — every finding
  interprets a claim without its author present to say what was meant, and a
  reader who does not know that will over-read the report.

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

## 17. Positioning and outreach

<!-- audience: practice -->

**The Gap Map is the lead artifact in cold outreach** — the partner gets it in
the email body or as a 30-second PDF. The Gap Map is the hook; the full report
is the proof.

Kept out of §15 deliberately: an agent writing a deliverable should not be told
it is writing a hook. Removals, retractions and the reasoning behind this split
live in `audit/METHOD-gap-analysis.md`, not here.
