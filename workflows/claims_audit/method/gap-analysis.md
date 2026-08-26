# METHOD.md against the assurance standards — a gap analysis

Written 2026-08-24. **Purpose: break a circularity.** The fixture, the answer
key and the method were authored by the same process — `answer_key.md` was
recovered from an agent exchange, METHOD.md is amended by agents through §14,
and `score.py` grades against tiers derived from that key. When every model
passes, nothing currently in the loop can distinguish "the method is good"
from "the fixture rewards what the method happens to specify."

An external reference breaks that. The operation §1 describes —
*stated claims vs. observed implementation, with citations* — has a formal
name in the assurance profession: **an attestation engagement on an
assertion**. It is codified in **ISAE 3000 (Revised)** internationally and
**AICPA AT-C 205** (examination engagements) in the US, with **SSAE 18 /
SOC 2** as a worked instance.

**This is not a conformance exercise.** Conforming METHOD.md to ISAE 3000
would make it longer, more defensive and worse at what it does — those
standards are written for human firms with partners, engagement letters and
file review. The question asked here is narrower and more useful:

> **What do these standards require that METHOD.md has no slot for?**

**Confidence note.** The requirements below are stated from working knowledge
of the standards' structure, not from the paragraph text, which is largely
paywalled. Directionally I am confident; before anything here is relied on
commercially, check it against the actual standard.

## What METHOD.md already has, arrived at independently

Worth stating first, because it is the reason the comparison is worth making
at all. These were not copied — the constraints are forced by the problem, and
two parties reasoning about it separately reached the same places.

| METHOD.md | Assurance analogue |
|---|---|
| §2 scope rule — audit stated claims, never opine on what the code *should* do | Subject matter and criteria identified in advance; report against criteria, not preference |
| §9's five-value taxonomy; "the audit recommends, the buyer decides" | Closed opinion vocabulary; the practitioner does not direct the client's action |
| §4 coverage honesty — silence reads as completeness | Scope limitations must be disclosed; an undisclosed one is itself reportable |
| §5 mandatory `file:lines` citation | Sufficient appropriate evidence, documented |
| §6 `[unverifiable]` must never be reported as `[delta]` | Inability to obtain evidence is a scope limitation, not an adverse finding |
| §7 correction protocol | Revision when subsequent evidence contradicts a conclusion |
| §10 expert due-diligence, not legal advice | The practitioner/adviser boundary |
| §11 explicit non-scope list | Engagement boundary definition |
| §14 method carries, target facts never | Firm-level quality management vs. engagement data |

## The gaps

Ranked by how much damage each could do, not by how easy they are to fix.

### 1. The assurance level is never stated — and the conclusion form contradicts the procedure

**The strongest finding here.** The standards draw a hard line between two
levels, and the *wording of the conclusion differs* between them:

- **Reasonable assurance** — positive form. *"In our opinion, X is fairly
  stated."* Requires evidence sufficient to reduce risk to acceptably low.
- **Limited assurance** — negative form. *"Nothing has come to our attention
  that causes us to believe X is materially misstated."* Fewer procedures,
  and the conclusion is explicitly weaker.

§9's taxonomy is **positive form**. "Material" asserts a state of the claims
directly. But §12 step 5 stops *"at the coverage threshold where the report is
defensible"*, and §4 explicitly accepts partial coverage of the claim surface.
That is a **limited-assurance procedure wearing a reasonable-assurance
conclusion.**

A reader cannot tell what confidence "Material" carries. Neither can a court.

This is also the gap most entangled with the fixture: `score.py` checks that
the §9 taxonomy was *used*, never that the conclusion's strength matches the
evidence actually obtained. An model that verifies 12 of 50 claims and returns
"Material" scores identically to one that verifies 45.

**Slot needed:** a statement of assurance level, and either a conclusion form
that matches it or a coverage floor below which the positive form is not
available.

### 2. The working papers are destroyed — and §10's own defence depends on them

The standards require engagement documentation sufficient for *an experienced
practitioner with no prior connection to the engagement* to understand the
nature, timing and extent of procedures, the results, and the conclusions
reached. Firms retain it for years, because it is the primary defence when
work is challenged.

§14: *"Each audit runs in its own world, which is discarded afterwards."*

§10, Vector 1, defence 2: *"the citation trail demonstrates the work was
systematic; the correction protocol (§7) demonstrates it was honest."*

**These contradict each other.** The report's citations survive; the working
papers do not. The reads that found nothing, the hypotheses tested and
dropped, the corrections made mid-engagement, the claims examined and judged
`[real]` — all of it is in the discarded world. Under a negligence claim,
"we examined roughly fifty claims and reported twelve" would be an
unevidenced assertion, and the twelve reported findings cannot demonstrate
the thirty-eight that were checked and held.

The confidentiality reasoning behind §14 is sound. The remedy is not to keep
the world but to **extract a retainable engagement file** — the procedure
log, coverage record and correction history, target facts included, retained
under the engagement letter rather than carried between engagements.

**Slot needed:** an engagement file distinct from both the deliverable and the
discarded world.

### 3. No materiality basis

The standards require materiality to be determined for the subject matter and
applied consistently — it is what separates a finding from an observation.

§4 gives a priority *order*. §6 marks `[partial]` and `[delta]` as "Material —
buyer should price the gap." But nothing states **what magnitude of gap is
worth reporting at all**. "Material" is used as a verdict label without a
threshold behind it.

Consequence, visible in our own measurements: across three models on one
fixture, Tier 3 counts came in at 6, 3 and 2. Some of that is grader noise —
but some of it is three reporters drawing the reporting threshold in three
different places, because the method does not tell them where it is.

**Slot needed:** a materiality basis. For this product it is probably
transaction-relative — a gap is material if it would change the price, the
structure or the decision to close.

### 4. Criteria are not frozen

The standards require criteria to be identified *in advance* and to remain
fixed, so the conclusion means the same thing at the end as at the start.

§12 step 2 enumerates the claim surface and estimates a total. Nothing says
the surface is **closed** at that point. An agent that meets a claim at step 5
and quietly folds it into the population has changed the criteria
mid-engagement, and both the coverage percentage and the conclusion silently
change meaning. Nothing in the method or the scorer would detect it.

**Slot needed:** the claim surface is frozen at the end of enumeration;
claims discovered later are recorded as an addendum with their own coverage
line, not merged into the original population.

### 5. The responsible party is never consulted, and that is not disclosed as a limitation

In a standard attestation the responsible party provides a written assertion
and representations to the practitioner. Here the structure is different by
design: the **seller** makes the claims, the **buyer** engages the auditor,
and the seller is never asked to confirm anything, may not know the audit is
happening, and has no right of reply.

That is legitimate for transaction diligence. It is also a **significant
inherent limitation** and the report never says so. Every claim is
interpreted without the author of the claim present to say what it meant.

§16's "what the client should ask the seller before closing" is post-audit
advice to the buyer. It is not a statement that the conclusions rest on an
uncorroborated reading.

**Slot needed:** one line in the report's inherent-limitations statement —
the party making these claims was not consulted and has not confirmed our
interpretation of them.

### 6. Conclusions with a shelf life have no expiry mechanism

The standards require consideration of events between fieldwork and report
date that would affect the conclusion.

The method has no such slot, and the fixture demonstrates exactly why it needs
one. F2 — *the last recoverable backup expires 2026-08-29* — is a conclusion
that **becomes false, or becomes catastrophic, on a specific date.** A report
delivered on 08-24 and read on 09-02 is describing a different world.

**Slot needed:** a dated finding carries its own validity window, and the
report carries an as-of date for the materials.

### 7. Independence is not addressed

The auditor is engaged by the buyer, whose interest is a lower price. An audit
that finds deltas strengthens the buyer's negotiating position. In assurance
terms that is a **self-interest threat**, and firms are required to identify
and document such threats and their safeguards.

§10 handles liability thoroughly and independence not at all. The direction of
the incentive is never named.

Partial mitigation already exists: unsupported claims are penalised by the
scorer, and §2 forbids advocacy. But the method never acknowledges that its
own incentive runs toward finding problems.

**Slot needed:** an independence statement, and a safeguard — the strongest
available being that findings must survive §12 step 6b citation verification
regardless of which direction they push the price.

### 8. No independent review

Quality management standards require an engagement quality review — a second
practitioner, uninvolved in the work — for certain engagements.

§12 step 6b is **self-review**: the author verifying its own citations. Useful,
and explicitly described in the method as "a floor, not a guarantee." There is
no independent pass.

Possibly disproportionate for a $5K product. Worth naming as a deliberate
omission rather than an oversight — and worth revisiting before the first
engagement where a finding moves real money.

## What this exercise says about the fixture

Three of these gaps are **invisible to the current scorer by construction**:

- Assurance level (§1 above) — `score.py` checks the taxonomy was used, never
  whether the conclusion's strength is earned by the coverage.
- Materiality (§3) — the Tier 3 spread across models is partly a measurement of
  a threshold the method never sets.
- Frozen criteria (§4) — a mid-engagement change to the claim population would
  not register anywhere.

That is the value of an external reference, and it is exactly what
self-authored criteria cannot provide: **the fixture cannot fail an model for a
thing the method never asked for.** Every gap above is a way an audit could be
professionally deficient while scoring a clean PASS.

## Status — all eight closed 2026-08-24

| # | gap | where it now lives |
|---|---|---|
| 1 | assurance level unstated | **§1a** — limited assurance over a disclosed subset; recommendation and coverage travel together |
| 2 | working papers destroyed | **§14** — engagement file extracted and retained before the world is discarded |
| 3 | no materiality basis | **§4** — a gap is material if it would change price, structure, or the decision to close |
| 4 | criteria not frozen | **§12.2** — the claim surface closes at enumeration; late claims go in an addendum |
| 5 | responsible party not consulted, undisclosed | **§16** — inherent-limitations statement, three lines, always present |
| 6 | no expiry on dated conclusions | **§5** — a dated derived finding carries its own expiry; the report carries an as-of date |
| 7 | independence unaddressed | **§10** — the incentive runs toward over-reporting; four existing safeguards named |
| 8 | no independent review | **§12.7** — named as a deliberate, price-point decision, with the trigger to revisit |

Plus the reverse-pass finding that was an addition rather than a removal:
**§4's priority order was a robotics artifact** whose top tier was empty for
every SaaS target. Restated against all three engagements, with tiers defined
by consequence to the buyer rather than by subsystem.

Executing text 3,963 -> 4,926 words. It grew, and should have: eight of these
were things the method did not say.

## What I would do

1. **Gaps 1 and 3 first** — assurance level and materiality. They are cheap to
   write, they change what a conclusion *means*, and both are currently
   invisible to the scorer.
2. **Gap 2 next** — the working-papers conflict is the one with real liability
   exposure, and it is a contradiction internal to the document rather than a
   matter of taste.
3. **Gaps 4–8 as a batch**, once there is a second real engagement to test
   them against.

**Do not fix these by amending METHOD.md through §14.** Every one of them was
found by an external reference precisely because the internal loop could not
see them; routing the repair back through the same loop that missed them
forfeits the point.
