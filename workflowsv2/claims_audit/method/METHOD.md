# Technical claims audit — method

## 1. What this stage does

One claim-source document is audited per run. The engagement names it. Every other supplied document is available to you; §7 says which of them count as evidence. *Evidence* means the material actually cited in a finding, per §7.

> Compare what the seller asserts in this claim source against what the supplied materials show, and cite the material used.

The run has two phases and produces two JSON documents, each assembled by the client's process from the objects you emit. First you enumerate the claims, section by section, and the surface is **frozen**; then you adjudicate the frozen claims against the evidence, batch by batch. Both documents are read by a machine, not by the buyer. A later stage combines the runs over every claim source, and a stage after that writes the client's report.

**The surface is frozen before any claim is adjudicated, and this is a control rather than a convenience.** Enumerating while adjudicating means choosing what counts as a claim with the verdicts already in view, which is choosing the sample after seeing the result. You will be given the frozen list back; you do not revise it.

**The schema enforces the shape. This document says what makes a field correct.** A response can satisfy the schema and still be wrong: a quote that is not verbatim, evidence that does not bear on the claim, an `unverifiable` that should have been `contradicted`. Those are what this method governs.

Two things follow from being one stage of several:

- **Do not judge the offering.** No overall conclusion, no summary of what the target is worth, no statement about whether the buyer should proceed. Those depend on every claim source, and this run has seen one.
- **Do not write for a reader.** No covering note, no narrative, no recommendations. Fill the fields.

## 2. Scope rule

**Evaluate what the seller asserts about the target against what the supplied materials show.**

A claim is an assertion the seller makes to the buyer in the designated claim source — a listing, a technical description, a specification, a marketing document. The engagement names which document that is for this run.

**Not every sentence of a claim source is a claim.** Four kinds are not, or not as written:

- **Navigation and courtesy.** A heading, a link, a badge, an image, a table of contents, "see LICENSE", "we welcome contributions", a sign-off. These point or greet and assert nothing about the target, even where the link points at the seller's own site, documentation or demo. A badge or link that states a fact — a licence badge, a version badge — is the fact it states, quoted as it appears.
- **Instructions.** A command, an installation step, a prerequisite, a configuration example. Enumerate what the block asserts about the target: that a tool has the subcommands shown, that the stack runs on the named database, that a step has the stated effect. One claim per capability asserted, not one per line.
- **Intent.** A roadmap item, a feature marked as coming, a plan. It concerns what the seller means to do, not the target's present state, and is enumerated as about the seller, per §5. Intent concerns a future state: a statement that the target does not have or do something is about its present state, and is a claim about the target whatever heading it sits under.
- **Puffery.** A vague general statement of quality that a reasonable buyer would not rely on and that no evidence could settle: "blazing fast", "rock solid", "modern", "enterprise-grade". It is not enumerated. A statement that is specific and testable is a claim however it is dressed: "handles 10,000 requests a second" is a claim, "fast" is not.

**Read each assertion as a reasonable buyer in this transaction would read it:** in its ordinary sense, as one statement, using its heading only to resolve a referent, with the seller's own documentation as the specification of what the software is said to do. A statement that the target belongs to a category, "a URL shortener", "a CRM", is one claim, that the core function of the category exists; it is not expanded into the properties a member of the category might have.

**An assertion about the seller rather than the target is still a claim.** A sign-up page, a directory listing said to be in progress, a hosted plan and its price, a stated plan: the seller asserts them to the buyer, and the supplied materials are not expected to reach them. §5 marks such a claim so the reader knows why it went unsettled.

Other supplied documents are not claim sources for this run, even when they assert something; §7 says which of them are evidence.

Business claims are in scope as well as technical ones. Revenue, customer counts, contracts and external dependencies are evaluated the same way as backups, uptime, architecture or test coverage.

The audit does not recommend how the target should be built or how the transaction should be structured. It reports the state of each claim and the consequence of any gap.

## 3. Scope varies by target; the method does not

The claims differ by target. A hardware specification raises different questions from a SaaS listing. The comparison in §1 and the scope rule in §2 do not change.

Where a subject has little bearing on the target, say so precisely rather than expanding to fill space:

> No telemetry path was found. One user-initiated outbound call sends user-entered text off-network.

That is preferable to "no data exfiltration path", which claims more than the evidence carries.

## 4. What a finding is

Four terms, and they nest:

- **claim** — one seller assertion that can be evaluated as true or false, enumerated and frozen in §5.
- **verdict** — the classification assigned after comparing the claim with the evidence, from §6.
- **adjudication** — the verdict, together with the `gap` or `unresolved_because` field that verdict requires, per §13.
- **finding** — one output record: the `claim_id` of a frozen claim, its adjudication, and the evidence used.

Every frozen claim produces exactly one finding, and every finding adjudicates exactly one claim. A claim carries one verdict, so two findings on one claim are either redundant or in conflict.

**A finding names its claim by `claim_id` and does not restate it.** The frozen surface is what the claim says; the finding is what you concluded about it. Two copies of one assertion can disagree, and a reader would have no way to tell which was authoritative.

Where one piece of evidence settles several claims, write a finding for each and cite that evidence in each. Do not join claims into one finding to avoid repeating a citation.

**Attempt every claim.** You may not skip one because it looks unimportant — the significance of a gap is not known until the claim is tested.

If you cannot reach them all, the run does not produce an audit. Say so in `not_completed`, give the reason, and emit an empty list for the phase you are in. Do not mark the claims you did not reach as `unverifiable`: that verdict states that the materials cannot settle a claim, and what is true here is that the claim was never attempted.

A derived fact (§7) is evidence. It is not a finding of its own.

## 5. The claim surface

The claim source is given to you one section at a time, in document order, with the claims already enumerated from the sections before it. Enumerate every assertion in the section you are given. Each one carries:

- **`quote`** — the assertion as the claim source states it, **verbatim**. Copy it; do not paraphrase, tidy, or join text that is not contiguous.
- **`lines`** — where that quote sits in the claim source, as a start and end line number.
- **`statement`** — the claim in your own plain words, so a reader knows what is being tested.
- **`about`** — whom the assertion concerns, per §2:

| `about` | Meaning |
|---|---|
| `target` | The product, code, infrastructure, business or terms the buyer would acquire |
| `seller` | The seller's own activity, hosted service or stated intent, which the supplied materials are not expected to reach |
| `document` | A document in the materials itself, rather than what the software does: its licence, its existence, what it contains. The document settles such a claim, and a citation into it is evidence for this claim alone, per §7 |

- **`restates`** — present only when the assertion is one already enumerated, made again: the `id` of that claim.

**`id`** is assigned by the client's process, from 1, in document order, once a section's claims are in. Claims from earlier sections reach you with their ids, and that is how findings refer to them.

**An assertion made twice is one claim.** A feature table, a comparison table and a FAQ routinely repeat the headline. Where the section you are given asserts something a listed claim already asserts, emit it with `restates` naming that claim, and the client's process records the second place on the one claim. A claim is adjudicated once, and its finding covers every place it is made. Two assertions that share a subject and differ in what they assert are two claims, per the rule below.

**Where to divide the text into claims.** Split assertions apart where different evidence could give them different verdicts. Keep them together where they stand or fall on the same evidence. **Do not split one assertion into parts that the same evidence settles**, and do not combine separate assertions because they share a subject. Both are faults, and the second is not worse than the first.

> "Backups run daily with 30-day retention" is **two** claims. The schedule and the retention period are settled by different evidence, and one can hold while the other fails. Sharing a sentence is not a reason to combine.
>
> "Platform-level redundancy and automatic failover" is **one** claim. Both stand or fall on whether replicas exist, and one line of the infrastructure config settles both. Sharing a subject is not, by itself, a reason to combine — but being settled by the same evidence is.

An instruction block is divided by what it asserts, not by its lines: six subcommands listed in one block assert one thing, that the tool has them, and are one claim.

**Enumerate before you adjudicate, and enumerate everything.** At this point you have not tested any claim, so you cannot know which will hold. A claim that looks obviously true, or obviously false, or impossible to check, is enumerated exactly like the rest.

**Once the last section is enumerated the surface is frozen.** It is handed back to you for the adjudication phase and it does not change. An assertion you notice after the surface has closed is not added to it and is not adjudicated. Enumerate each section carefully the first time; that is what this phase is for.

## 6. Verdicts

Every finding carries exactly one.

| Verdict | Meaning |
|---|---|
| `real` | The evidence supports the claim without a reportable caveat |
| `real_with_caveat` | Every part of the claim is borne out, and the evidence shows something a reader must know to read the claim correctly |
| `partial` | The claim is substantially true and has a specific, citable gap |
| `contradicted` | The claim is false — the claim source says one thing and the evidence shows another |
| `unverifiable` | The claim was attempted and the supplied materials cannot settle it |

**A gap must be citable, not merely unfound.** `partial` requires evidence you can point at showing the gap. Evidence you looked for and did not find is not a gap — that is `unverifiable`, and §8 governs it.

**Choosing between `partial` and `real_with_caveat`.** Ask whether you can cite a part of the assertion the evidence shows is not borne out. If you can, the verdict is `partial`. If every part holds and there is still something to say, it is `real_with_caveat`.

> "Blended MRR is $40,000", against $16,000 from the payment processor and three wire transfers of $8,000. Every part of the assertion holds; the total is $40,000. That 60% of it arrives by manual wire from three customers is not a failure of the claim, and a reader must know it. `real_with_caveat`.
>
> "The technology stack is scalable", against a single dyno with the database co-located on it. The platform supports scaling; this deployment does not. A part of the assertion fails and can be named. `partial`.

**A caveat is not for weak evidence.** If what you have does not settle the claim, the verdict is `unverifiable` and §8 governs it. A framework version inferred from directory names, where no manifest or lockfile was supplied, is `unverifiable` — not a claim that holds with a caveat. The verdict says how the claim fared against the evidence; it does not say how confident you are.

**Keep `partial` and `contradicted` apart.** `partial` means the claim is substantially true and a named part is not borne out. `contradicted` means the evidence says otherwise.

**Do not judge whether a gap matters to the buyer.** That judgement depends on the transaction and on what every other claim source shows, and a later stage makes it. Record what the claim says, what the evidence shows, and the difference.

## 7. Evidence

Every finding cites the material that settles it. A reader must be able to inspect both sides of the comparison: the frozen claim §5 enumerated, and the evidence here.

A finding's evidence is **one list**. Each item in it declares its `form`, and the form decides which other fields the item carries. A finding may hold any number of items in any mix — two citations and a derived statement, or a citation and two searches.

**`citation`** — material that bears directly on the claim.

- **`document`**, **`lines`** — where it is;
- **`quote`** — the text, verbatim: one contiguous span of the cited lines, copied as it appears. Do not join text that is not contiguous. Where a second span is needed, cite it as a second item;
- **`shows`** — what it demonstrates about the claim.

**`derived`** — a fact that follows from two or more supplied facts. A retention period in one document and the date of the last usable backup in another together determine when recovery becomes impossible; the seller stated neither consequence.

- **`basis`** — every source fact it rests on, each with `document`, `lines` and `quote` under the same rules as a `citation`;
- **`derivation`** — the arithmetic or reasoning, shown so a reader can reproduce it;
- **`consequence`** — what follows, and why it bears on the claim.

Derive only from facts supplied in the materials; do not import market forecasts or other outside conclusions. State the consequence and stop — do not turn it into advice about the transaction. Where the derivation depends on time, give the relevant date and what changes on it. A derivation that bears on no claim is not evidence about a claim; report it under §9.

**`search`** — a record of a search performed to determine whether the supplied materials hold anything that could settle the claim. A statement that something is not implemented cannot rest on a line, because there is no line to cite; it rests on searches. §8 governs when they are required and what they settle.

- **`kind`** — `lexical` or `structural`;
- **`performed`** — what you actually searched;
- **`result`** — what came back;
- **`candidates`** — every file the search found where material that could settle the claim would appear, each by its path from the target root. An empty list when the search found none. A file listed here and not opened makes the claim `not_examined`, per §8.

**For numbers and dates, quote the source line.** A summary produced by a tool is useful for navigation and is not sufficient evidence for an exact quantitative claim.

**Documentation is not evidence for a claim about the software.** A claim about what the software does is settled by files that take part in building or running it: source, configuration, build and deployment files, dependency manifests, database migrations, tests. A README, a page under a docs directory, a website export or any claim source, named for this run or not, restates a claim and does not settle it. The engagement lists such files as excluded; the evidence request will not quote their text, and shows only that they exist, their length and their headings, which is all a claim about a document needs: "MIT licensed" is settled by the LICENSE file, "the API is documented" by the presence and headings of the documentation. Inside a code file, cite the statement, not the comment above it.

**Every citation must resolve.** Before you emit, check that each document named exists, each line range is real, and each quoted span appears in the document as you have written it.

Correct any citation that does not resolve. If it cannot be corrected — the material you remembered is not there — remove that item. If no remaining evidence settles the claim, the verdict is `unverifiable` under §8. Do not drop the finding: §4 gives every claim exactly one.

This check confirms that the cited text exists. It does not establish that the text supports the claim, which remains your judgement.

## 8. Claims the materials cannot settle

An `unverifiable` finding takes the same shape as any other. Its evidence carries `search` items, per §7, and it must **record** the searches rather than assert them.

Two are required, and they are complementary:

- **`lexical`** — search using terms taken from the claim itself, including reasonable stems and variants;
- **`structural`** — inspect the directory, module or document set where the material would reasonably appear.

For each, state what you actually searched, what came back, and in `candidates` the files it found where the material would appear. A search establishes diligence, not absolute absence.

**Every candidate is opened before the claim is recorded as unsettled.** A file the searches named and nobody read does not show that the materials cannot settle the claim; it shows that the engagement has not looked. *Opened* means its contents were read and appear in the evidence requests. If a candidate of any search on the claim was not opened, the disposition is `not_examined`; the client's process then asks you to open those files and to adjudicate the claim again. The other three dispositions are recorded only when every candidate has been opened. `not_examined` requires a named file: where the searches named no file, one of the other three applies.

The adjudication also records what the searches established, as one of:

| `unresolved_because` | Meaning |
|---|---|
| `not_in_the_materials` | Material of the right kind was supplied, and none of it settles the claim — an accounting export that carries no customer count |
| `present_but_not_readable` | The material is supplied and cannot be interrogated as given — a compiled archive with no source |
| `outside_the_materials` | The kind of material needed to test the claim was not supplied at all — no customer records of any sort |
| `not_examined` | The searches named one or more files where the material would appear, and at least one of them was not opened in this engagement |

If the searches do not settle the question, the verdict is `unverifiable`, per §6.

## 9. Unclaimed observations

The materials may show something relevant that the seller never asserted. It is not a finding — there is no claim for it to adjudicate — and it is not discarded. Record it as an unclaimed observation with a short note saying what it shows.

**This is the only thing an unclaimed observation is.** It is not a place for an assertion the frozen surface missed: §5 freezes the surface, and nothing adds to it afterwards by any route.

What the observation rests on is written as one §7 evidence item: a `citation` where a single place shows it, or a `derived` item where it follows from two or more facts and no single quote holds it.

## 10. Correction protocol

If evidence found later contradicts a finding you have already formed, revise the finding before emitting and record in one line what changed and why.

## 11. What this stage does not do

- It does not judge the offering, or reach a conclusion about the target as a whole.
- It does not recommend what the target or the buyer should do.
- It is not a penetration test.
- It is not a general code-quality review.
- It does not imply anything about claims it did not resolve.
- It does not provide legal advice.
- It is not acceptance testing of contracted work. The comparison is similar; this method is for a buyer evaluating a target they do not own.

## 12. Running the audit

**Phase one — enumerate.**

1. **Read the section of the claim source you are given**, with the claims already enumerated before it. If the section cannot be read, say so in `not_completed`, per §4.

2. **Enumerate every claim in the section**, naming any restatement, and emit them per §5 and §13. Do not gather evidence yet and do not form verdicts. The surface freezes when the last section's claims are in.

**Phase two — adjudicate.** You are given the frozen surface back.

3. **Gather evidence** across the materials §7 admits as evidence. The evidence that settles a claim usually sits far from the claim source. On every evidence request, name the claims it gathers evidence for: the record of the request is filed under those claims, and a claim is adjudicated on the requests filed under it and nothing else.

4. **Adjudicate each frozen claim** and assign one verdict, per §6. Where a claim raises something only the seller can answer, record it in `questions`.

   `questions` supplements the findings and never replaces one. A claim that needs seller information still gets its finding, and still gets `unverifiable` if the supplied materials cannot settle it.

   If it becomes clear that every frozen claim cannot be attempted, stop here: emit `not_completed` with the reason and an empty `findings`, per §4.

5. **Check every citation resolves**, per §7.

6. **Emit the findings document**, per §13.

## 13. The output

Two kinds of JSON object, one per phase. Phase one is emitted once per section of the claim source; phase two once per batch of claims. Each is complete for what it was asked, and nothing goes outside it.

**Phase one — the claim surface.**

| Field | Contents |
|---|---|
| `claim_source` | The document named by the engagement |
| `claims[]` | Every assertion the section you were given makes, per §5 |
| `claims[].quote` | The assertion verbatim, per §5 |
| `claims[].lines` | Where that quote sits in the claim source |
| `claims[].statement` | The claim in plain words |
| `claims[].about` | `target`, `seller` or `document`, per §5 |
| `claims[].restates` | Present only for an assertion already enumerated: the `id` of that claim, per §5 |
| `not_completed` | Present only when the section could not be read, per §4. When it is present, `claims` is empty |

**Phase two — the findings.** The fields follow §4's shape.

| Field | Contents |
|---|---|
| `claim_source` | The document named by the engagement |
| `findings[]` | One per frozen claim, per §4 |
| `findings[].claim_id` | The `id` of the frozen claim this finding adjudicates |
| `findings[].adjudication.verdict` | One value from §6 |
| `findings[].adjudication.gap` | What the verdict rests on: the caveat a reader needs, the part of the assertion that fails, or the contradiction. Required for `real_with_caveat`, `partial` and `contradicted`. Absent for `real`, which has nothing beside it, and for `unverifiable`, which uses the field below. **Where there is no gap, leave the field out.** Do not write "None" — an absent field is how the output says there is nothing, and the word is a value like any other. Why a claim holds belongs in its evidence, under `shows` |
| `findings[].adjudication.unresolved_because` | Per §8, required when the verdict is `unverifiable` |
| `findings[].evidence[]` | One list, each item declaring its `form` — per §7 |
| `findings[].correction` | Per §10, where a finding changed |
| `unclaimed[]` | `note` and one §7 evidence item — per §9 |
| `questions[]` | Questions only the seller can answer, raised by a claim — per §12 |
| `not_completed` | Present only when the run could not attempt every claim, per §4. When it is present, `findings` is empty |

An evidence item's `form` decides its fields, per §7:

| `form` | Fields |
|---|---|
| `citation` | `document`, `lines`, `quote`, `shows` |
| `derived` | `basis[]` (each `document`, `lines`, `quote`), `derivation`, `consequence` |
| `search` | `kind`, `performed`, `result`, `candidates` |

The schema requires `form` and leaves the rest of each item open, because a form-by-form requirement is checked after the response is parsed rather than during decoding. An item missing a field its form requires fails that check, and the output is invalid.

Emit nothing outside the JSON object. There is no covering note and no summary.

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

- `measure/fixtures/dataroom/` — a synthetic nine-document data room with planted defects and an answer key. A pipeline test, not a real business.

## 17. What carries between audits

<!-- audience: practice -->

Each audit runs in an isolated environment that is discarded afterwards. Client facts must not carry into later engagements; general method improvements may, after review.

| Category | Example | Carries forward? |
|---|---|---|
| method | "A claimed rate may refer to publish rate rather than check rate; determine which." | Yes |
| target | "This target's backups had failed for three weeks." | No |

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
