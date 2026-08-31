# Technical claims audit — method

## 1. Purpose

This is a one-shot, limited technical due-diligence engagement for a buyer evaluating a target they do not own, in acquisitions under $5M.

> Compare the seller's stated claims with the available evidence, and cite the evidence used.

Section 11 describes what is outside the scope of this audit.

## 1a. Level of assurance and coverage terminology

This is a **limited assurance** engagement. The audit attempts every identified claim and may still resolve only part of them, because the supplied materials cannot always settle a claim. Section 4 defines the order in which claims are addressed, Section 12 defines the sequence of work, and Section 11 defines exclusions.

The report may state which resolved claims are supported and which are not. It must not imply a conclusion about claims that were not resolved.

Two rules apply:

1. **You write the ledger; the process writes the figures.** What you owe is Section 16's verdict ledger: one line per claim, accounting for every claim in the frozen surface exactly once. Every coverage figure is arithmetic over it, and the client's process computes each one and places it after your report and at the end of the Gap Map. You will not see those sentences as you write; they are placed there when the document is assembled.

   **Write no count or ratio of claims by verdict anywhere in the deliverable**: not in the conclusion, not in the Gap Map, not in the coverage block. Describe the shape of the result in words, and let the figures stand beneath it.

   > Not: “The seller's claim sources are broadly accurate — 42 of 47 resolved claims are supported without material gap — but five claims are contradicted by the code.”
   >
   > Instead: “The seller's claim sources are broadly accurate. The claims contradicted by the code are concentrated in documentation of defaults, unimplemented metering, and two features described more broadly than they are built.”
2. **Do not imply broader coverage than the work supports.** For example, avoid “the system does what it says.” The figures the process emits are bounded by construction: “Coverage: 39 of 43 identified claims resolved; 37 of 39 resolved claims supported.” Do not write around them with a wider claim.

### Claim-state quantities

When reporting claim counts or rates, use the following terms consistently. Terms such as *checked*, *examined*, and *verified* should not be used for quantitative coverage because they do not distinguish between an attempted claim and one that reached a verdict.

| Term | Meaning |
|---|---|
| **identified** | Claims enumerated from the designated claim sources and frozen in the claim surface (Section 12, step 2) |
| **attempted** | Identified claims for which the audit tried to reach a determination |
| **resolved** | Attempted claims that reached one of the five claim verdicts in Section 6 |
| **supported** | Resolved claims whose verdict is `[real]`, `[real, minor caveat]`, or `[real, operational caveat]` |

An attempted claim that cannot be resolved is `[unverifiable]`. It is an examination status rather than a verdict, and it is included in the coverage figures.

**Coverage = resolved / identified.**  
**Consistency rate = supported / resolved.**

Every identified claim is attempted, so `attempted` equals `identified` and is the denominator of neither rate. In count terms, `[unverifiable]` claims are attempted claims that did not resolve.

Canonical form — the shape the client's process emits from the ledger, not a sentence you write:

> Coverage: 39 of 43 identified claims resolved; 37 of 39 resolved claims supported.

## 2. Scope rule

**Evaluate what the seller asserts about the target against what the supplied materials show.**

A claim is an assertion the seller makes to the buyer in designated claim sources such as a listing, technical description, specification, or marketing document. The engagement identifies which documents are claim sources (Section 12, step 1).

Other supplied materials, including source code and code comments, are evidence. They are used to test claims but are not automatically treated as seller claims.

Business claims are in scope as well as technical claims. Revenue, customer counts, contracts, and external dependencies can be evaluated in the same way as claims about backups, uptime, architecture, or test coverage.

The audit does **not** recommend how the target should be built or how the transaction should be structured. It reports the state of the claims and the consequences of any gaps. Decisions about remediation, pricing, negotiation, or whether to proceed remain with the buyer.

## 3. Scope varies by target; the method does not

The claims will differ by target. A hardware specification raises different questions from a SaaS listing, but the comparison method in Section 1, the scope rule in Section 2, and the priority order in Section 4 remain the same.

Where a subject has little relevance to a target, state that directly and precisely rather than expanding the report artificially. For example:

> No telemetry path was found. One user-initiated outbound call sends user-entered text off-network.

This is preferable to a broader statement such as “no data exfiltration path,” which may extend beyond the evidence.

## 4. Materiality and priority

The sequence is:

1. Enumerate the claims (Section 12, step 2).
2. Prioritize them using the order below.
3. Record a verdict or examination status for every claim in Section 16's ledger, and say in the coverage block what the materials could not settle and why the remaining uncertainty matters.

### Materiality

A gap is material if a reasonable buyer, knowing about it, could change the price, the structure of the transaction, or the decision to close. Materiality is relative to the transaction. The same defect may matter in a $400k acquisition and be immaterial in a $4M one.

Several rules follow:

- Report material gaps whether they favor the buyer or the seller.
- A discrepancy below the materiality threshold does not automatically make the claim unsupported. Use `[real, minor caveat]` for a claim that holds but has a non-material discrepancy.
- Use `[real]` when the evidence supports the claim without a caveat worth reporting.
- Materiality determines whether a discrepancy matters to the buyer's decision. It does not change whether the original claim was accurate.
- A claim is resolved by adjudicating it against the findings that support or refute it. Every resolved claim, supported ones included, is named by at least one finding.
- An attempted claim that cannot be settled is `[unverifiable]`. It is reported in the coverage block, not as a finding.
- `[derived]` findings are not included in the consistency rate because they do not test a seller claim.
- Materiality is judged against the buyer's decision, not against the auditor's preferences. For example, poor code organization is not material unless it bears on a claim or creates a decision-relevant consequence.

### Priority order

Every identified claim is assigned to one of four tiers. The tier is based on the consequence if the claim is false, not on the technical subject matter of the claim. The priority order is an order of work, not a filter: every identified claim belongs to a tier. A claim cannot be excluded because it appears unimportant before it is examined; the significance of any gap is not known until the claim is tested.

1. **Failure could make the business nonviable.** Examples include unrecoverable data, inability to collect money, withdrawal of a critical dependency, or failure of a safety-critical mechanism.
2. **Failure changes the nature of the asset being acquired.** Examples include claims about isolation, redundancy, failover, tier separation, or shared state where the actual architecture differs materially from the description.
3. **Failure creates a bounded remediation cost in time or money.** Examples include incorrect retry intervals, timeouts, monitoring behavior, retention periods, or thresholds that can be corrected without changing the nature of the asset.
4. **Failure is individually minor.** These claims are addressed last.

A claim finding takes the tier assigned to the claim. A derived finding takes at least the highest-priority tier of the facts on which it depends and may be assigned a higher-priority tier if the derived consequence is more severe.

If a tier contains no claims, state that in the coverage block. The priority order settles the highest-consequence claims first and gives Section 16 the order the findings are reported in.

## 5. Finding format

Use the following form for a finding that bears on one or more seller claims:

```text
**Finding N (claim <id>): <short title> — [claim verdict]**

Claim (<document:lines>): <the stated claim>

Evidence: <document or file:lines> — <what the materials show>

Gap: <None, or the specific gap>
```

`Finding N` numbers the findings in this report. `claim <id>` is the claim's number in the frozen surface, so a reader can trace a finding back to what was enumerated. Where a finding bears on several claims, name them all — `(claims 8, 27)` or `(claims 10-16)` — and carry the verdict those claims received. A derived finding names no claim and uses the format below.

Cite both the source of the claim and the evidence used to resolve it. A reader should be able to inspect both sides of the comparison.

There are three evidence patterns:

- **Ordinary claim finding:** cite the claim source and the evidence that resolves it.
- **Claim finding based on absence:** cite the claim source and record the searches used to establish whether the claimed implementation or evidence exists. There may be no line citation for the absent item.
- **Derived finding:** cite every source figure used in the derivation and show the derivation explicitly.

### Numeric and date evidence

For claims that depend on a number, date, threshold, or other exact value, quote or cite the source line rather than relying on a summary produced by a tool or intermediary. Summaries are useful for navigation but are not sufficient evidence for exact quantitative claims.

### Evidence of absence

A statement such as “this is not implemented” cannot normally be supported by a single source line. Use two complementary checks:

- **Lexical search:** search using terms taken from the claim itself, including reasonable stems or variants.
- **Structural search:** inspect the directory, module, document set, or other location where the implementation or evidence would reasonably appear.

Record the searches actually performed. A search establishes diligence, not absolute absence. If the searches do not settle the question, classify the claim as `[unverifiable]`, not `[delta]`.

### Derived findings

Some material conclusions are not directly stated by the seller but follow from two or more seller-supplied facts. For example, a retention period in one document and the date of the last usable backup in another may together determine when recovery becomes impossible.

Use this format:

```text
**Finding N: <short title> — [derived]**

Basis: <document:lines> — <first source figure, verbatim>
       <document:lines> — <second source figure, verbatim>

Derivation: <the arithmetic or logical derivation>

Consequence: <what follows and why it matters to the buyer>

Escalates: <Finding N, or None>
```

Rules for derived findings:

1. Use only facts supplied in the target materials. Do not derive market forecasts or other external conclusions and present them as part of this audit method.
2. Show the arithmetic or other derivation explicitly so the reader can reproduce it.
3. State the consequence, but do not turn the finding into a recommendation about the transaction.
4. If the derived finding strengthens or sharpens an existing finding, identify that relationship under `Escalates` rather than presenting the two as unrelated issues.

A time-dependent derived finding must state the relevant date and what changes on that date. The deliverable also carries an as-of date for the materials, because the conclusions apply to the supplied snapshot.

## 6. Verdicts, statuses, and observation types

Three vocabularies are used for different purposes.

### Claim verdicts

These describe how a stated seller claim compares with the evidence. Every resolved claim receives exactly one of these verdicts.

| Verdict | Meaning | Buyer impact |
|---|---|---|
| `[real]` | The evidence supports the claim without a reportable caveat | None |
| `[real, minor caveat]` | The claim holds, with a discrepancy that does not materially affect the decision | Awareness |
| `[real, operational caveat]` | The claim holds under current conditions, but operating context qualifies it | Awareness and operational planning |
| `[partial]` | The claim is substantially true but has a specific, citable material gap | Material; bounded gap to evaluate or price |
| `[delta]` | The claim is false; the claim source says one thing and the evidence shows another | Material; broken claim and possible pattern |

### Examination status

This describes an identified claim that was attempted and reached no verdict. It is not a claim verdict and does not produce a finding.

| Status | Meaning | Where reported |
|---|---|---|
| `[unverifiable]` | The claim was attempted, but the supplied materials could not settle it | Coverage block, with the reason |

### Other observation types

These describe observations that are not seller-claim verdicts.

| Type | Meaning | Finding? |
|---|---|---|
| `[derived]` | A consequence derived from two or more facts stated in the materials | Yes |
| `[unclaimed]` | The materials contain relevant information about something the seller did not claim | No; note in the coverage block |

A finding is a cited record of what the materials show:

- a **claim finding** bears on one or more seller claims and carries the verdict those claims received;
- a **derived finding** bears on no seller claim and carries `[derived]`.

One finding may bear on several claims, and one claim may be settled by several findings. Split a finding whose claims received different verdicts, so that each header carries one verdict.

`[partial]` and `[delta]` should remain distinct. `[partial]` means the claim substantially holds but has a material limitation. `[delta]` means the claim itself is contradicted by the evidence.

`[derived]` is not a verdict on a seller claim. It records a consequence that follows from facts the seller supplied separately.

`[unverifiable]` must not be reported as `[delta]`. Failure to find supporting evidence is not equivalent to evidence that the claim is false.

Use the narrowest verdict that fits the evidence.

## 7. Correction protocol

If later evidence contradicts a finding, revise the finding and record a one-line explanation of what changed and why.

## 8. Reader and document structure

Write for the person making the transaction decision. The Gap Map should be understandable in about thirty seconds. The full report is a reference document for readers who need the supporting detail.

The two documents serve different purposes:

- The **Gap Map** is read from beginning to end and presents the conclusion, the most important findings, and coverage.
- The **report** backs every claim it resolves (Section 4) and can therefore be long. It should be organized so a reader can begin with the conclusion, review the highest-consequence findings, and then locate any specific claim and its evidence.

The report therefore opens with the conclusion, orders findings from most to least significant, and closes with a concise explanation that a non-specialist can understand and repeat accurately.

## 9. Report-level conclusion

Use one of the following report-level conclusions:

| Conclusion | Meaning |
|---|---|
| **Clear** | No material findings of any kind. Every resolved claim is supported, with no caveat of note. |
| **Clear with caveats** | No material findings. There may be documentation drift, maintenance debt, or operational qualifications that a buyer should know but that do not materially affect valuation. |
| **Conditional** | Material findings exist, but they are localized and have bounded, identifiable remediation. They do not change the nature of the asset being acquired. |
| **Material** | Material findings have consequences that are not limited to a localized remediation and materially change valuation or risk. |
| **Systemically inconsistent** | The seller's claims do not provide a reliable description of the target, because material contradictions occur across independent claims or throughout a claim source. |

The audit states the evidence-based conclusion. The buyer decides what action to take.

For example:

> Conditional: the seller claims 30-day retention; the materials show 7 days.

Whether seven days is acceptable is a transaction decision and is not part of the audit conclusion.

Keep the audit conclusion vocabulary separate from buyer actions such as *proceed*, *negotiate*, or *walk away*.

The Gap Map may include a clearly labeled professional judgment such as “the gaps appear addressable within the existing integration timeline.” This exception applies only to the Gap Map. The full report must stop at findings, consequences, and the report-level conclusion and must not direct the buyer's action.

## 10. Liability and professional posture

<!-- audience: practice -->

This service is technical due diligence, not legal advice. It reports what was observed, how that compares with the seller's claims, and the technical or business consequence of any gap. Legal conclusions and transaction decisions remain with the client and their advisers.

The closest professional analogies are other evidence-based review services in which a specialist evaluates supplied material against stated criteria and documents material discrepancies.

### Independence

The buyer engages the auditor, and adverse findings may benefit the buyer in negotiation. That creates an incentive risk toward over-reporting. The method addresses that risk through several controls:

- every claim finding must be supported by the required citations;
- `[unverifiable]` cannot be converted into `[delta]` merely because evidence was not found;
- materiality is applied regardless of whether a gap favors buyer or seller; and
- the completed report is checked against its citations before delivery.

A finding should be written the same way whether the engagement is buyer-side or seller-side.

### Risk of a client claim for negligent work

The main professional risk is incomplete or careless analysis. Controls include:

1. **Clear scope in the engagement letter.** State which materials and systems were and were not available. A defect outside the supplied or accessible material is outside the evidence base of the engagement.
2. **A reasonable professional standard rather than a promise of perfection.** The citation trail should show a systematic process, and the correction protocol should show that findings are revisable when the evidence changes.
3. **A limitation-of-liability provision** appropriate to the engagement and reviewed by counsel.
4. **Professional liability / E&O insurance** covering technical due-diligence work before accepting paid engagements.

### Risk of a claim by the target

The report should remain factual, technical, confidential, and limited to the evidence. Avoid personal accusations or statements about motive.

Relevant legal questions—including defamation, privilege, tortious interference, and standards for negligent or reckless statements—vary by jurisdiction. This section is a working business posture only and requires review by qualified counsel before paid engagements begin.

## 11. What this audit does not do

- It does not recommend what the target should do.
- It is not a penetration test.
- It is not a general code-quality review.
- It does not imply coverage beyond the claims actually resolved.
- It does not provide legal advice.
- It is not acceptance testing of contracted work. The comparison mechanism may be similar, but this method is specifically for a buyer evaluating a target they do not own.

## 12. Running an audit

The current implementation is launched through `workflows/claims_audit/scenario.yaml`, with per-target configuration in `world_name` and `external_repo`. Operational details belong in that implementation; this section defines the method.

1. **Receive the materials and confirm scope in writing, including the claim sources.**

   Claim sources are the documents in which the seller makes assertions about the target. All other supplied documents are evidence unless the engagement says otherwise.

   If the engagement does not explicitly identify claim sources, document which materials were treated as claim sources and why before closing the claim surface in step 2.

   If a seller claim is later found in a document that was not designated as a claim source, place it in an addendum rather than changing the frozen claim surface.

2. **Enumerate the claims and close the claim surface.**

   Read the designated claim sources and end enumeration with this exact marker and count format:

   ```text
   === CLAIM SURFACE ===
   <N> claims
   ```

   The line immediately after the marker contains only the number and the word `claims`. End the block with:

   ```text
   === END CLAIM SURFACE ===
   ```

   A claim is one seller assertion that can be evaluated as true or false. Do not combine separate assertions merely because they concern the same subject, and do not split a single assertion into artificial subclaims. Count all claims regardless of priority. Include a per-document count below the total.

   Once closed, the **claim surface** is frozen and becomes the denominator for coverage. A claim discovered later that was missed during enumeration goes into a separately counted addendum and does not change the original claim-surface count.

3. **Prioritize the claims** using Section 4.

4. **Work through the priority order.** Do not require client confirmation between claims. Questions for the seller are collected for the deliverable.

5. **Attempt every identified claim.** If the engagement cannot reach them all — for reasons of time, budget, or access — it does not produce a deliverable. Report that the work could not be completed, and why, rather than issuing a partial audit.

6. **Write the report.** Apply the correction protocol in Section 7 to any findings that changed during the engagement.

6b. **Check the report against its citations.**

   - Every ordinary claim finding must cite both the seller claim and the evidence that resolves it.
   - A finding based on absence must cite the claim and record both the lexical and structural searches.
   - Every derived finding must cite every source fact on which the derivation depends and show the derivation.
   - Every reference must resolve to material actually present in the supplied evidence.

   A finding with a missing or invalid citation does not ship. This check establishes that cited material exists; it does not by itself prove that the cited line is the correct evidence, so substantive review remains necessary.

7. **Deliver the audit and propose method changes, if any.** Proposed method changes must be general techniques, not facts about the current target. If a proposed lesson cannot be written without naming the target, it is not a reusable method rule.

   After the working record has been retained according to practice policy, the isolated engagement environment may be destroyed.

## 12a. Practice review

<!-- audience: practice -->

### Independent review

The current process does not include an independent second review of the full report. Step 6b is a self-check by the author. That is a practical choice for a low-cost engagement, but it is weaker than an independent review and should be reconsidered for engagements where a finding can move substantial value.

### Review of the frozen claim surface

Before client delivery, the practice reviews the frozen claim surface against the supplied materials.

This review occurs after the audit run and before delivery; it does not interrupt the audit workflow.

Check:

- Did the engagement designate the correct claim-source documents? If a seller claim document was omitted from scope, that is a scoping issue to raise with the client.
- Were all designated claim-source documents enumerated?
- Were evidence-only documents excluded from the claim surface?
- Is the granularity of claims reasonably consistent across documents?
- Does the total equal the per-document counts?

Because every coverage figure uses the claim-surface count as its denominator, an incorrect surface makes the coverage figures incorrect and requires correction before delivery or reissue if discovered later.

### Why human confirmation is retained

Tests on 2026-08-25 showed substantial disagreement among language models asked to enumerate claims from the same nine documents. Three models returned counts of 62, 67, and 273. Attempts to define the unit more tightly reduced some ambiguity but did not produce stable agreement.

A major source of disagreement was scope: whether only designated seller-claim documents should be enumerated or every checkable statement in all supplied evidence. The method now resolves that issue explicitly by separating claim sources from evidence, but claim granularity still benefits from a brief human review.

ISAE 3000 (Revised) 24(b)(ii), with related application material, requires criteria capable of reasonably consistent measurement or evaluation in similar circumstances. The observed variation is a practical reason not to rely on wording alone to guarantee a stable claim count.

The fixed marker format is retained for a similar reason. Earlier tests showed that instructions such as “state the count after the marker” were interpreted inconsistently. Fixing the exact block format eliminates that ambiguity for automated downstream processing.

## 13. Where the method has been exercised

<!-- audience: practice -->

- **`measure/fixtures/dataroom/`** — a synthetic nine-document data room with planted defects and an answer key, used to test the pipeline. It is not a real SaaS business.
- **`/home/bruce/projects/Body`** — a real autonomous mobile robot codebase used as a second data room. The audit produced 31 findings, no deltas, and a **Clear with caveats** conclusion. The codebase was developed separately from the audit process, and the audit agent performed the verification and reporting independently.

## 14. What carries between audits

<!-- audience: practice -->

Practice policies such as retention periods, destruction dates, engagement-letter terms, and approval of method changes belong to the practice, not to the auditor's per-engagement instructions.

Each audit runs in an isolated environment that is discarded afterward. Client-specific facts must not carry into later engagements. General method improvements may carry forward after review.

| Category | Example | Carries forward? |
|---|---|---|
| **method** | “A claimed rate may refer to publish rate rather than check rate; determine which.” | **Yes** |
| **target** | “This target's backups had failed for 21 days.” | **No** |

### Working record

The working record is retained even though the isolated audit environment is discarded.

The final report cites the evidence behind every resolved claim, so it records much of what previously existed only in the working trace. The working record remains useful because it preserves information the report does not contain, including:

- searches and dead ends that preceded a finding; and
- the work performed on `[unverifiable]` claims.

Current implementation records include:

- `memory/reasoning_trace.jsonl` — actions, observations, iterations, and exit information from the audit process;
- `inspect_traces/*.txt` — evidence requests, searches and reads performed, returned material, and the resulting response.

These files function as working papers because they record what was actually queried and observed during the engagement rather than a retrospective summary of diligence.

They should be copied out before the isolated environment is destroyed and retained according to the engagement letter and practice policy.

Because the traces may contain verbatim client material, retaining them creates corresponding confidentiality, security, retention, and destruction obligations. Those obligations should be addressed in the engagement letter and practice procedures.

### Method learning

At the end of an engagement, the auditor may propose general changes to this method. The practice reviews the proposed changes before they are merged into the versioned method file.

The test for a reusable lesson is simple: it must be expressible without identifying the target. For example:

- Reusable: “Check whether a message broker binds to all network interfaces.”
- Not reusable: “Check whether product X binds to `0.0.0.0` as it did in client Y.”

Two further practice issues should be planned for:

- **Cross-client aggregation requires permission.** A corpus of recurring findings can become commercially valuable, but using client-derived information across engagements should be authorized explicitly in the engagement terms.
- **Method changes can overfit recent engagements.** Re-run the synthetic fixture after material method changes and compare performance against its answer key. Maintain at least one fixture from a domain unlike the most recent real engagements.

## 15. The Gap Map

The Gap Map is a one-page summary for the decision-maker and should be readable in about thirty seconds.

It contains:

- target name and a one-line description;
- the Section 9 conclusion;
- the three to five findings that matter most;
- the coverage line, which the client's process supplies;
- one line stating that the full cited report is available; and
- a small footer: “technical claims verification · not a pen-test · not legal advice.”

Each key item is a finding summarized in one line with a short note. The Gap Map does not need line citations because the full report carries them. Reserve **claim** for an assertion made by the seller. A Gap Map item states what the audit concluded about a claim; it is not itself a claim.

When the conclusion is **Clear** or **Clear with caveats**, include important supported findings as well as caveats. Otherwise a mostly successful audit can be misrepresented by a summary that lists only defects.

The coverage line reports resolved / identified and supported / resolved. The client's process computes it from the Section 16 ledger and places it in the Gap Map, so do not write the figures yourself. Where the materials could not settle a claim, explain in your own words why the remaining uncertainty is or is not low-risk — that is judgement, and it is yours.

If the conclusion is **Clear** and there are no caveats, the Gap Map may be reduced to a very short statement, for example:

> No gaps found. Full report attached.

Do not add promotional content such as logos, pricing, or sales calls to action. The Gap Map is a professional deliverable, not a brochure.

## 16. Deliverable format

The deliverable contains five blocks, each with explicit opening and closing markers:

```text
=== CLAIM SURFACE ===     …     === END CLAIM SURFACE ===
=== REPORT ===            …     === END REPORT ===
=== COVERAGE ===          …     === END COVERAGE ===
=== LIMITATIONS ===       …     === END LIMITATIONS ===
=== GAP MAP ===           …     === END GAP MAP ===
```

An opening marker identifies the block being delivered. A closing marker confirms that the block is complete rather than merely interrupted or truncated. Emit them in that order.

The blocks, rather than conversational turns, define completion. The claim surface must be closed before claims are evaluated because it supplies the denominator for later coverage figures. Coverage is a separate block after the report for the matching reason: it is a statement about claims, and it can only be written once every claim carries a verdict or a status.

Each marker appears on its own line. Nothing should appear between an opening marker and the content of that block.

### `=== CLAIM SURFACE ===`

Use the format defined in Section 12, step 2. The line immediately after the opening marker contains the claim count and nothing else, followed by the enumeration.

### `=== REPORT ===`

The report contains three parts in this order:

1. **Conclusion**, using only the vocabulary in Section 9.
2. **Findings, ordered from highest to lowest consequence.**
   - Ordinary claim findings use the Section 5 format and cite both claim and resolving evidence.
   - Findings based on absence cite the claim and record both required searches.
   - Derived findings use `[derived]`, cite each basis fact, and show the derivation.
3. **Questions the client should ask the seller before closing.**

Coverage is not part of the report block. It is the `=== COVERAGE ===` block below.

Do not place a covering note, process narrative, or conversational preamble inside the report block.

Report length follows from the claim surface: a large surface needs many findings and produces a long report. Organise it as a reference document rather than one read straight through.

As a rough writing budget:

 - a supported claim normally needs its verdict, citations, and a short explanatory clause — roughly 20–30 words;
 - a claim that is not supported normally needs the claim, the resolving evidence, and the consequence of the gap — roughly 80 words.

Do not reduce length by dropping citations or by leaving a resolved claim out of the findings. Either makes the report harder to verify against the frozen claim surface.

### `=== COVERAGE ===`

**Every identified claim carries exactly one verdict or examination status.** Coverage counts claims, never findings. If a frozen claim nevertheless contains more than one seller assertion, it still carries one verdict; where its parts differ, Section 6's `[partial]` is the verdict for a claim that is substantially true with a specific gap.

This ledger is the arithmetic record the coverage figures are computed from. It does not replace the findings; Section 4 still requires every resolved claim to be named by at least one finding.

This block contains two parts.

**1. The verdict ledger.** Every claim in the frozen surface, in order, one per line, with the verdict or status it received:

```text
<claim number>. [verdict]
```

Use only Section 6's five claim verdicts or its examination status. Every claim in the surface appears exactly once. No claim appears twice, and no line names a claim that is not in the surface.

Nothing else goes on these lines. Write the ledger in full, however long it runs.

**2. What the numbers do not say.** In prose, after the ledger:

- why each `[unverifiable]` claim could not be settled;
- anything in the claim sources that was not counted as a claim, and why;
- any `[unclaimed]` observation, per Section 6;
- any addendum created under Section 12, step 2.

**Do not state coverage figures.** Identified, resolved, supported and the consistency rate are arithmetic over the ledger, and the client's process computes them from it. Section 1a defines what each means; the ledger is what they are computed from.

### `=== LIMITATIONS ===`

This block is always present and contains three items:

1. the materials examined and their as-of date;
2. a statement that the seller was not consulted and has not confirmed the audit's interpretation of the seller's claims; and
3. the assurance level and the coverage on which the conclusion rests.

The second item matters because the report interprets seller statements without giving the seller an opportunity during the engagement to clarify what was intended.

Throughout this method, **report** means the `=== REPORT ===` block, while **deliverable** means all five blocks together.

### `=== GAP MAP ===`

Use the structure in Section 15. Do not restate the full report or append unrelated notes.

## 17. Positioning and outreach

<!-- audience: practice -->

### Gap Map design

Reference implementation: `docs/audit-gap-map-mockup.html`. It is populated with example audit data and is kept outside the reusable audit method because target-specific facts must not carry between engagements.

The current design is a single card approximately 680 px wide. The scan order is:

1. target identity;
2. conclusion band;
3. key findings;
4. coverage; and
5. report link.

The conclusion band uses the Section 9 conclusion and may be color-coded from green to red. It should be the only place the report-level conclusion appears in the Gap Map.

The design still needs to be checked at email-thumbnail size, in PDF export, and against any landing page with which it shares visual design.

For outreach, the Gap Map can be used as the short-form artifact and the full report as supporting evidence. This positioning belongs to practice and marketing material, not to the instructions used to produce client deliverables.

## 18. Provenance

<!-- audience: practice -->

This method was drafted on 2026-08-22 from prior work on the audit process and was subsequently reviewed and revised. The vocabulary for claim verdicts, report conclusions, priority tiers, correction handling, and audit sequence has been refined through repeated test runs.

This file is the durable source of the method and is maintained in version control so changes can be reviewed and diffed rather than accumulated informally in an agent's memory or prompt history.

The file was renamed on 2026-08-25 from “AI-Readiness Audit” because that name described a different service from the technical claims verification defined here.

Any earlier review comments should be treated as historical unless reconfirmed against the current version of the method.

## 19. Superseded rules and rationale

<!-- audience: practice -->

Historical rules are kept in a separate practice-only section so obsolete vocabulary does not appear near the active instructions used during an audit.

This also keeps the operational method focused on current rules rather than spending prompt or reader attention on rules that no longer apply.

### Section 4 — priority tiers

The tiers were revised on 2026-08-27 to use one consistent axis: the consequence if a claim is false.

Earlier versions mixed subject categories such as “architectural invariants” and “operational parameters” with severity categories. That created ambiguous cases—for example, an architectural failure could be either trivial or business-ending. The current tiers are based only on consequence.

Buyer-action wording was also removed from tier definitions so priority classification remains separate from transaction advice.

### Section 6 — `[unclaimed]`

This category was previously called `[non-delta]`. That name was misleading because it suggested “the claim is not false,” when the intended meaning is that no seller claim existed in the first place.

### Section 9 — report conclusion

The section was previously called a “recommendation,” and earlier conclusion terms included buyer actions such as *Walk*. Those terms were removed because the audit reports the state of the evidence rather than directing transaction action.

The conclusion categories were also changed so they account for material findings of any kind, including `[derived]` findings. A target should not receive a **Clear** conclusion merely because every explicit seller claim is supported if the seller's own figures, taken together, imply a separate material problem.

Broad statements such as “the system does what it says” were removed because they imply more coverage than a limited-assurance audit may have performed.

### Section 16 — explicit block markers

Explicit block markers were restored after testing showed that conversational turn boundaries do not reliably indicate which deliverable component has actually been completed.

A block marker identifies the artifact directly and gives downstream processing an unambiguous completion signal. The method therefore no longer depends on a fixed number of conversational turns.

A process that requests the four blocks one at a time was considered and rejected because it would make the external workflow drive the audit sequence. Self-delimiting blocks allow the audit to proceed according to the method while still producing machine-detectable deliverables.
