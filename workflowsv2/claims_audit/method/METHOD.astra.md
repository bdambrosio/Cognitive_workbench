# Technical claims audit — method

## 1. Purpose

Compare the assertions in one designated claim source against the supplied materials. Identify each claim, collect evidence, assign a verdict, and record the evidence supporting that verdict.

The engagement identifies the target, the claim source, the supplied materials, and any limits on access or scope.

A run has two phases:

1. **Enumeration:** identify claims section by section. The client's process assembles them into the claim surface and freezes it.
2. **Adjudication:** gather evidence and evaluate the frozen claims in batches. The client's process assembles the findings.

Enumeration must finish before evidence gathering or adjudication begins. This prevents evidence or expected verdicts from influencing which assertions are selected as claims.

This stage produces structured records. Later stages combine runs and prepare the client's report.

Emit only the JSON object requested for the current section or batch. Use plain language within its fields. Do not add a covering note or summary.

## 2. Scope and interpretation

A **claim** is an assertion in the designated claim source about the target, the seller, or a document. Technical and business assertions are both in scope.

Only the designated document supplies claims for this run. Other supplied documents may help interpret a claim or provide evidence under §7; their assertions are not separately enumerated.

Apply these rules:

* **Navigation and courtesy:** omit text that only directs or greets, such as "see LICENSE" or "we welcome contributions." A heading, link, badge, or image is not automatically a claim or automatically excluded; consider what it asserts.
* **Hosted metrics and status badges:** omit badges reporting stars, downloads, pulls, image size, or hosted build status. This is an explicit scope exclusion. The same fact asserted in ordinary text remains in scope. Enumerate factual licence and version badges.
* **Instructions:** enumerate the capabilities, requirements, or effects asserted by an instruction. Do not make a separate claim for every command line in a procedure.
* **Intent:** enumerate a stated plan as a claim about the seller's intent, not as a claim that the planned feature exists. A statement that a feature is currently absent is a claim about the target.
* **Puffery:** omit vague praise with no stated or contextually defined test, such as "blazing fast" or "enterprise-grade." Enumerate a specific assertion such as "handles 10,000 requests per second." Do not invent a benchmark to make vague praise testable.

Interpret a claim in its ordinary meaning within the supplied transaction context. Preserve qualifications supplied by headings, table headers, nearby text, or explicit references. These may identify the subject, product edition, operating conditions, date, or distinction between present behavior and future intent.

Use supplied seller documentation to clarify what the seller means. Do not use it to add capabilities the claim does not assert. Do not use documentation discovered during adjudication to change a frozen claim.

A category assertion such as "a URL shortener" is one claim that the category's core function exists. Do not expand it into every property commonly associated with that category.

If the available context leaves more than one materially different interpretation, state the ambiguity in `statement` during enumeration. Do not silently choose the interpretation easiest to verify. During adjudication, assign a settled verdict only if the ambiguity does not affect that verdict; otherwise use `unverifiable` under §8.

## 3. Materials and execution

Distinguish these three things:

* **Supplied materials:** the files and records made available to the engagement.
* **Examined material:** content successfully read or inspected during evidence gathering.
* **Delivered evidence:** the evidence records available in the current adjudication call.

A missing file in the supplied materials is an evidence limitation. A supplied file not examined is unfinished work. Examined material omitted from an adjudication call is an evidence-delivery failure. Do not report these as the same condition.

The client's process must:

1. Provide each enumeration call with the relevant source context and prior claims, including their IDs, quotes, statements, and `about` values.
2. Separate enumeration from evidence gathering.
3. Deliver relevant source excerpts, search records, and recorded access failures to adjudication calls.
4. Preserve evidence needed to assess a claim when reducing or dividing context. Identify any omitted material.
5. Provide the previous finding when requesting a correction.
6. Preserve incompletion and surface issues when assembling responses.

The adjudicating agent must use only evidence actually delivered to it. It must not reconstruct missing quotes or search results from memory or assume that a file was examined merely because it is named.

If essential gathered evidence is missing from the current call, use `not_completed` under §4. Do not classify the underlying claim as unverifiable because of a delivery failure.

## 4. Claims, findings, and completion

The output uses these terms:

* **claim:** an assertion identified and frozen under §5.
* **verdict:** the classification assigned under §6.
* **adjudication:** the verdict and the explanatory field required by that verdict.
* **finding:** one record containing a frozen `claim_id`, its adjudication, and its evidence.
* **evidence:** cited material, recorded searches, or derived facts supporting a finding or unclaimed observation.

Every completed adjudication batch has exactly one finding for each claim assigned to it. A finding refers to the claim by `claim_id`; it does not provide a replacement claim statement.

When evidence bears on several claims, cite it in each relevant finding.

**Attempt every assigned claim.** An attempt requires examining relevant material or conducting the searches needed to determine why the materials cannot settle the claim. Merely receiving a claim or filing an unanswered evidence request is not an attempt.

If the assigned section or batch cannot be completed:

* Set `not_completed` to a plain-language reason identifying the affected work.
* Emit an empty `claims` list for enumeration or an empty `findings` list for adjudication.
* Do not assign `unverifiable` to a claim that was never attempted.

`not_completed` applies to the current response. The client's process must retain every such reason and mark the assembled run incomplete. It may retain successful batches as working records, but must not present them as a completed audit.

The provisional `unverifiable` / `not_examined` finding in §8 is an exception for attempted claims awaiting identified examination work. A run containing any such finding remains incomplete.

A run is complete only when every source section has been enumerated, every frozen claim has a final finding, and no unresolved incompletion or surface issue remains.

## 5. The claim surface

Enumerate every in-scope claim in the section provided, without gathering evidence or predicting verdicts.

Each emitted claim has:

* **`quote`:** a verbatim, contiguous excerpt from the claim source.
* **`lines`:** its start and end line numbers, inclusive, numbered from 1 in the original file.
* **`statement`:** the assertion in plain words, preserving its meaning and qualifications.
* **`about`:** `target`, `seller`, or `document`.
* **`restates`:** an earlier claim's ID, only when this assertion repeats that claim.
* **`additional_locations`:** additional occurrences in the current section, if any, each with its own `quote` and `lines`.

Use the smallest contiguous quote that expresses the assertion clearly. When the subject or qualification comes from a heading or table header, preserve it in `statement`. The quote may include surrounding text when needed. Never join noncontiguous text.

Different claims may use overlapping or identical quotes when the same sentence asserts more than one thing. Their statements must identify the different assertions.

### What the claim concerns

| `about`    | Meaning                                                                                                  |
| ---------- | -------------------------------------------------------------------------------------------------------- |
| `target`   | The product, code, infrastructure, business, or terms included in the proposed acquisition or evaluation |
| `seller`   | The seller's activity, intent, or offering outside that target                                           |
| `document` | The existence or contents of a supplied document                                                         |

Use the engagement's scope to distinguish the target from the seller. A hosted service is `target` if it is part of the business being evaluated; hosting alone does not make it `seller`.

"The software is MIT licensed" concerns the target. "The repository contains a LICENSE file" concerns a document. These are different claims.

### Divide claims by what can be tested separately

Separate independently testable properties or capabilities, even when they share a subject or sentence.

* "Backups run daily with 30-day retention" is two claims: the schedule and the retention period. Do not add "by default" unless the source says so.
* "Platform-level redundancy and automatic failover" is two claims. Redundancy does not by itself assert automatic failover.
* "The count is stored, but message bodies are never stored" is two claims: storing the count and excluding message-body storage.

There is one grouping exception: keep an explicitly listed set of alternatives for the same capability as one claim. Examples include supported export formats, supported operating systems, or the subcommands a tool provides. Preserve the complete list in `statement`.

Do not use this exception to combine different behaviors. If individual subcommands have separately described effects, those effects are separate claims.

A numerical limit, schedule, default, or boundary is tested as stated. Do not weaken "30-day retention" to "retention exists," or "never stores message bodies" to "sometimes avoids storing them."

### Repeated assertions

Assertions are restatements only when they have the same meaning, including subject, capability, conditions, quantity, and time period. A broader, narrower, or conflicting assertion is a separate claim.

For a restatement of a claim from an earlier section, emit `restates` with that claim's ID.

For repeated occurrences within the current section, emit one claim using its first occurrence and put the later occurrences in `additional_locations`. If that claim also restates an earlier claim, include both fields.

The client's process assigns IDs to new claims after each section and attaches all repeated locations to the corresponding claim.

### Freezing and discovered errors

After the final section is enumerated, the client's process freezes the claim surface. Do not add, remove, split, merge, or reinterpret claims during adjudication.

If you discover a missed assertion or a material error in a frozen statement, record it in `surface_issues` under §13. Do not adjudicate a missed assertion or disguise it as an unclaimed observation.

A surface issue requires review and correction of the surface before a completed audit can be produced. Any affected adjudication must then be rerun against the corrected surface.

## 6. Verdicts

Each finding has exactly one verdict.

| Verdict            | Meaning                                                                                                                                   |
| ------------------ | ----------------------------------------------------------------------------------------------------------------------------------------- |
| `real`             | The evidence supports the whole frozen claim, with no necessary qualification beyond those already in the claim                           |
| `real_with_caveat` | The evidence supports the whole frozen claim, and an additional evidenced fact is needed to interpret that support correctly              |
| `partial`          | For a listed-set claim retained under §5, some listed members are supported and others are contradicted; every member's status is settled |
| `contradicted`     | The evidence establishes that the frozen assertion is false                                                                               |
| `unverifiable`     | The claim was attempted, but the materials do not settle it, or examination is provisionally incomplete under §8                          |

Apply these rules in order:

1. Identify exactly what the frozen claim asserts.
2. Decide whether the evidence is sufficient for that assertion under §7.
3. For a listed-set claim, use `partial` only when every member is settled and the results include both support and contradiction. If all members are supported, use `real` or `real_with_caveat`; if all are contradicted, use `contradicted`.
4. For other claims, use `contradicted` when the asserted property, quantity, condition, or boundary is false. Support for a weaker assertion does not make the claim `partial`.
5. If the whole claim is supported, use `real_with_caveat` only when an additional evidenced fact is necessary to interpret it; otherwise use `real`.
6. If the claim remains unsettled, use `unverifiable` and complete §8's requirements.

For a listed-set claim with an unsettled member, use `unverifiable`. Record any supported or contradicted members in the evidence; do not describe the whole list as settled.

Examples:

* "Backups are retained for 30 days," against evidence establishing a seven-day retention policy: `contradicted`.
* "Exports support CSV and JSON," against evidence establishing CSV support and an explicit rejection of JSON exports: `partial`.
* "Message bodies are never stored," against a reachable logging path that stores them within the claimed operating conditions: `contradicted`.
* "An automated backup runs daily," against evidence establishing the daily backup and that it contains database records but excludes separately stored uploaded files: `real_with_caveat`, provided the original claim does not assert that all data is backed up. If it does, test that broader assertion as stated.

A caveat must not excuse a false assertion or weak evidence. Do not add observations merely because they might interest a buyer; unrelated observations belong in §9.

For unresolved conflicting evidence, use `unverifiable`. For resolved conflicts, cite the relevant sources and explain why one applies to the claim while the other does not.

## 7. Evidence

Evidence must match the claim's subject, scope, conditions, version, and time period. State only what it establishes.

### Admissibility and sufficiency

For software behavior, use implementation, configuration, build and deployment files, dependency manifests, database migrations, tests, or supplied execution records, as appropriate to the claim.

Distinguish what those sources establish:

* Implementation can establish a behavior when its relevant execution path and conditions are shown.
* A test file establishes what is tested, not that the test passed.
* A configuration file establishes the specified configuration, not necessarily the configuration of a live deployment.
* Performance, uptime, usage, and production-state claims require records appropriate to those assertions.

For business claims, use supplied accounting records, customer records, agreements, or other relevant records. Establish the relevant period and meaning of quantities before combining them. Payments alone do not establish monthly recurring revenue.

A seller's description of software behavior does not establish that behavior. Documentation may define the assertion being tested, but implementation or other appropriate evidence must settle it.

A document can establish its own existence or contents. Presence and headings can support a narrow existence claim; claims about substantive contents require reading the relevant text. A LICENSE file can establish the licence terms it states, but not automatically the licensing of every dependency or asset.

The engagement's access restrictions remain binding. If needed document content is inaccessible, report the limitation under §8. Do not treat inaccessible content as inspected.

Use executable statements rather than comments as evidence of implementation. Comments may explain intent but do not establish execution.

Do not obtain outside facts unless the engagement expressly supplies them as materials. Ordinary arithmetic and technical reasoning may be used, but identify any target-specific premise on which a conclusion depends.

### Evidence forms

Each evidence item declares one `form`.

**`citation`**

* `document`: path from the target root.
* `lines`: inclusive start and end line numbers.
* `quote`: a verbatim, contiguous excerpt within those lines.
* `shows`: what the excerpt establishes about the claim.

Use separate citation items for separate excerpts. For exact numbers or dates, quote the source text rather than a tool's paraphrase.

**`derived`**

A conclusion requiring two or more supplied facts.

* `basis`: every source fact used, each with `document`, `lines`, and `quote`.
* `derivation`: the arithmetic or reasoning needed to reproduce the conclusion.
* `consequence`: the result and its relevance to the claim or observation.

Include every necessary factual premise. For a time-dependent derivation, identify the relevant dates. Do not infer future behavior merely from a current configuration unless the claimed conclusion follows from the supplied evidence.

**`search`**

A record of examination under §8.

* `kind`: `lexical` or `structural`.
* `performed`: the actual terms, paths, records, or structures examined, including relevant limits.
* `result`: what was found, what was examined, and any access failures or incomplete coverage.
* `candidates`: paths of files identified as potentially relevant.

A candidate is a file with a plausible role in settling the claim, not every incidental keyword match. Explain exclusions when dismissing a plausible candidate.

### Citation verification

Before emission, verify each citation and each derived item's basis:

1. The document exists in the supplied materials.
2. The line range is valid.
3. The quoted text appears verbatim within that range.
4. The text supports the stated interpretation.

Use delivered source excerpts for these checks. Do not infer a quote from a summary.

Correct or remove invalid citations and reconsider the verdict. If the remaining evidence is insufficient, follow §8. If required examination or evidence delivery is incomplete, follow §4.

## 8. Searches and unresolved claims

Searches serve two purposes:

* establishing support for a claim of absence or exclusion;
* establishing why an attempted claim cannot be settled.

For either purpose, record at least one search of each kind:

* **`lexical`:** search claim terms, synonyms, stems, identifiers, and likely implementation terms.
* **`structural`:** inspect the directories, modules, record sets, or execution paths where relevant evidence would reasonably occur.

Record only searches actually performed. A request to search is not a completed search.

### Candidate examination

For every candidate, either:

* examine enough content to assess its relevance to the claim; or
* record an actual failed attempt and explain why its relevant content cannot be examined.

A filename, heading, keyword match, or partial read is not sufficient when relevant content remains unexamined.

If relevant candidates remain unexamined and no other evidence settles the claim, emit a provisional `unverifiable` finding with `unresolved_because` set to `not_examined`. Identify the outstanding files and work in the search results.

This provisional finding may precede completion of the two searches. It does not count as a final adjudication. The client's process must request the outstanding examination and then readjudicate the claim.

If no meaningful attempt was made, or a required search was simply omitted without identified follow-up work, use `not_completed` under §4 instead.

### Claims of absence or exclusion

Examples include "no telemetry" and "message bodies are never stored."

To support such a claim:

1. Define the claimed scope.
2. Examine the direct mechanism and other relevant paths, including handlers, middleware, logging, configuration, and dependencies where applicable.
3. Complete both search kinds and examine every relevant candidate.
4. Explain why the supplied materials cover the places in which the excluded behavior could occur within that scope.

A clean search result alone does not establish adequate coverage. If relevant deployment, dependency, or configuration material is missing, the broad absence claim remains `unverifiable`.

A counterexample within the claimed scope can establish `contradicted` without completing absence searches.

For an existence claim, unsuccessful searches normally establish `unverifiable`. Exception: direct inspection of a demonstrably complete, bounded structure can establish that a specifically claimed member is absent. Cite the structure and explain why it is complete for that claim. Failure to find a name across a repository is not such proof.

### Reasons for unresolved claims

Every `unverifiable` finding has one `unresolved_because` value:

| Value                      | Meaning                                                                                                                                      |
| -------------------------- | -------------------------------------------------------------------------------------------------------------------------------------------- |
| `not_in_the_materials`     | Relevant material was supplied and examined, but it does not settle the claim; this includes unresolved ambiguity or conflicting records     |
| `present_but_not_readable` | Necessary material is supplied, but a recorded attempt could not expose or interpret the relevant content with the permitted tools or access |
| `outside_the_materials`    | A necessary kind of material was not supplied                                                                                                |
| `not_examined`             | Necessary examination of identified supplied material remains unfinished; this is provisional                                                |

Choose the reason that most directly prevents adjudication. Explain other limitations in the search results.

Before using any of the first three values, finish the required searches and account for every candidate through successful examination or a recorded failed attempt.

## 9. Unclaimed observations

An unclaimed observation is an evidenced fact relevant to the target that does not adjudicate a claim in this source and is not itself an assertion missed during enumeration.

Record:

* `note`: what the observation shows and how it relates to the target.
* `evidence`: one `citation` or `derived` item under §7.

Do not conduct a general code review to populate this list. Record relevant observations encountered during the authorized work.

Evidence requests devoted to unclaimed observations use an empty claim-ID list and explicitly identify their purpose. The client's process must retain and deliver them for an observation-only emission, using the phase-two output shape with an empty `findings` list.

Do not attach unrelated evidence to a claim merely to make it reach adjudication.

## 10. Corrections

When new evidence changes a finding, replace the finding for that `claim_id`. Do not append a second finding.

The correction call must receive the previous finding and the relevant old and new evidence.

Include `correction`: one sentence stating what changed and why. If the previous finding was not supplied, do not invent its contents or a change history.

The client's process must also reconcile questions and observations affected by the correction. Preserve unrelated records, and remove or replace records made obsolete by the new evidence.

A correction changes the finding, not the frozen claim. Errors in the frozen claim follow §5.

## 11. Limits

This stage does not:

* judge the offering or whether the buyer should proceed;
* assess the transaction significance of a gap;
* recommend technical or transaction changes;
* perform a penetration test or general code-quality review;
* provide legal advice;
* imply support for unresolved or unexamined claims.

This method concerns evaluation of a target. It does not constitute acceptance testing of contracted work.

## 12. Procedure

### Phase one: enumerate

1. Read the assigned source section and supplied context.
2. Identify every in-scope claim under §§2 and 5.
3. Separate independent assertions and identify restatements.
4. Check quotes, locations, statements, and qualifications.
5. Emit the section's claim object under §13.

Do not gather evidence or form verdicts during this phase.

### Phase two: gather and adjudicate

1. Gather relevant evidence for every frozen claim. On each request, name all claim IDs it is intended to support.
2. Complete the searches and candidate examination required by §8.
3. Record relevant unclaimed observations separately under §9.
4. Adjudicate each assigned claim against the delivered evidence.
5. Record any seller question needed to resolve a claim. A question supplements a finding and never replaces it.
6. Check citations and derived reasoning.
7. Emit the requested findings object under §13.

Claim IDs on evidence requests control retrieval, not admissibility. Evidence delivered for another claim may be used if it bears directly on the current claim; cite it explicitly. Do not assume evidence from another batch is available.

The client's process must deliver relevant shared evidence to every affected adjudication and revisit affected findings when later evidence changes them.

Use `not_completed` when the assigned work cannot be completed under §4. Use provisional `not_examined` only under §8.

## 13. Output

Emit one JSON object per assigned section, claim batch, or observation-only call. Include the required fields and omit optional fields when they do not apply. Use empty arrays for required lists with no entries.

All `lines` values are two-element arrays: inclusive start and end line numbers. All document paths are relative to the target root.

### Phase one: claim surface

| Field                             | Contents                                                                            |
| --------------------------------- | ----------------------------------------------------------------------------------- |
| `claim_source`                    | Path of the designated claim source                                                 |
| `claims[]`                        | Claims enumerated from the assigned section                                         |
| `claims[].quote`                  | Verbatim source excerpt                                                             |
| `claims[].lines`                  | Source location                                                                     |
| `claims[].statement`              | Claim in plain words, including relevant qualifications                             |
| `claims[].about`                  | `target`, `seller`, or `document`                                                   |
| `claims[].restates`               | Optional earlier claim ID                                                           |
| `claims[].additional_locations[]` | Optional additional occurrences in this section, each with `quote` and `lines`      |
| `not_completed`                   | Optional reason the section could not be completed; when present, `claims` is empty |

The client's process assigns `id` values and preserves every occurrence when assembling the frozen surface.

### Phase two: findings

| Field                                        | Contents                                                                                            |
| -------------------------------------------- | --------------------------------------------------------------------------------------------------- |
| `claim_source`                               | Path of the designated claim source                                                                 |
| `findings[]`                                 | One finding per assigned frozen claim; empty for an observation-only call or an incomplete response |
| `findings[].claim_id`                        | Frozen claim ID                                                                                     |
| `findings[].adjudication.verdict`            | One verdict from §6                                                                                 |
| `findings[].adjudication.gap`                | Required for `real_with_caveat`, `partial`, and `contradicted`; absent otherwise                    |
| `findings[].adjudication.unresolved_because` | Required only for `unverifiable`, using §8                                                          |
| `findings[].evidence[]`                      | Evidence items under §7                                                                             |
| `findings[].correction`                      | Optional change explanation under §10                                                               |
| `unclaimed[]`                                | Observations, each with `note` and one `evidence` item                                              |
| `questions[]`                                | Seller questions, each with `claim_id` and `question`                                               |
| `surface_issues[]`                           | Discovered enumeration errors, as defined below                                                     |
| `not_completed`                              | Optional reason the assigned work could not be completed; when present, `findings` is empty         |

The `gap` field has a verdict-specific meaning:

* For `real_with_caveat`, state the necessary qualification and why it helps interpret the supported claim.
* For `partial`, identify the supported and contradicted members.
* For `contradicted`, state the difference between the assertion and the evidence.

Do not write "None" in an optional field. Omit it.

Explain support in a citation's `shows`, a search's `result`, or a derived item's `consequence`. Explain unresolved limitations in the evidence records.

Each `surface_issues` item has:

* `quote` and `lines`: the relevant source text;
* `issue`: the missed assertion or material error;
* optional `claim_id`: the affected frozen claim, if one exists.

A surface issue is a process defect, not a finding or an unclaimed observation.

### Evidence items

| `form`     | Required fields                                                                           |
| ---------- | ----------------------------------------------------------------------------------------- |
| `citation` | `document`, `lines`, `quote`, `shows`                                                     |
| `derived`  | `basis[]`, `derivation`, `consequence`; every basis item has `document`, `lines`, `quote` |
| `search`   | `kind`, `performed`, `result`, `candidates`                                               |

Each item must contain the fields required by its form, whether enforced during decoding or checked afterward.

The client's process must preserve all incompletion reasons and surface issues across batches. Successful output from another batch does not clear them. Duplicate findings, unresolved provisional findings, missing required fields, or invalid citations prevent completion until corrected.

## 14. Liability and professional posture

<!-- audience: practice -->

This service is technical due diligence, not legal advice. It reports what was observed, how that compares with the seller's claims, and the consequence of any gap. Legal conclusions and transaction decisions remain with the client and their advisers.

### Independence

The buyer engages the auditor, and adverse findings may benefit the buyer in negotiation. That creates an incentive toward over-reporting. The controls are in the method: every finding carries its citations; an unsettled claim cannot become a contradicted one; and every citation is checked before the run ships. A finding should be written the same way whether the engagement is buyer-side or seller-side.

### Risk of a claim for negligent work

The main professional risk is incomplete or careless analysis. Controls: a clear scope in the engagement letter, stating which materials were and were not available; a reasonable professional standard rather than a promise of perfection, evidenced by the citation trail; a limitation-of-liability provision reviewed by counsel; and professional liability insurance before accepting paid engagements.

### Risk of a claim by the target

Reports remain factual, technical, confidential and limited to the evidence. No personal accusations, no statements about motive. Defamation, privilege and tortious interference vary by jurisdiction; this is a working posture only and requires review by counsel before paid engagements.

## 15. Practice review

<!-- audience: practice -->

Before delivery, the practice reviews what each run identified against the claim source it was given. Did the engagement designate the right documents? Were evidence-only documents kept out of the claim role? Is claim granularity reasonably consistent across runs?

Human confirmation is retained because models disagree substantially about claim granularity. Three models given the same nine documents returned counts of 62, 67 and 273 before claim sources were named per engagement. Naming the source per run removed the scope disagreement; granularity still benefits from review.

ISAE 3000 (Revised) 24(b)(ii) requires criteria capable of reasonably consistent measurement in similar circumstances, which is the standard that variation of that size fails.

## 16. Where the method has been exercised

<!-- audience: practice -->

* `measure/fixtures/dataroom/` — a synthetic nine-document data room with planted defects and an answer key. A pipeline test, not a real business.

## 17. What carries between audits

<!-- audience: practice -->

Each audit runs in an isolated environment that is discarded afterwards. Client facts must not carry into later engagements; general method improvements may, after review.

| Category | Example                                                                             | Carries forward? |
| -------- | ----------------------------------------------------------------------------------- | ---------------- |
| method   | "A claimed rate may refer to publish rate rather than check rate; determine which." | Yes              |
| target   | "This target's backups had failed for three weeks."                                 | No               |

The working record is retained even though the environment is discarded. It preserves what the output does not: the searches and dead ends that preceded a finding, and the work performed on claims the materials could not settle. `memory/reasoning_trace.jsonl` holds actions, observations and exits; `inspect_traces/*.txt` holds each evidence request and what it returned. They are working papers because they record what was queried, not a retrospective account of diligence.

Because traces may contain verbatim client material, retaining them creates confidentiality, retention and destruction obligations that belong in the engagement letter.

The test for a reusable lesson: it must be expressible without identifying the target. "Check whether a message broker binds to all network interfaces" carries; "check whether product X binds to 0.0.0.0 as it did in client Y" does not.

Cross-client aggregation requires explicit authorization in the engagement terms. Method changes can overfit recent engagements: re-run the synthetic fixture after a material change and compare against its answer key.

## 18. Provenance

<!-- audience: practice -->

This file is the durable source of the method and is maintained in version control so changes are reviewed and diffed rather than accumulated in an agent's prompt history.

Earlier review comments are historical unless reconfirmed against this version.

## 19. Superseded rules and rationale

<!-- audience: practice -->

Moved to `docs/method-superseded-rules.md` on 2026-09-05, so obsolete vocabulary does not sit near the active instructions and the file the editor reads is the executable text plus the practice's posture.
