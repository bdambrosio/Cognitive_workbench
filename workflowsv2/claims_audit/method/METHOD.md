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

- **Navigation and courtesy.** A heading, a link, a badge, an image, a table of contents, "see LICENSE", "we welcome contributions", a sign-off. These point or greet and assert nothing about the target, even where the link points at the seller's own site, documentation or demo. A badge that states a fact about the target — a licence badge, a version badge — is the fact it states, quoted as it appears. A badge that reports a hosted count or measure — pulls, stars, downloads, image size, a build status — is out of scope and is not enumerated: it reports a figure the host computes, which the supplied materials cannot reach.
- **Instructions.** A command, an installation step, a prerequisite, a configuration example. Enumerate what the block asserts about the target: that a tool has the subcommands shown, that the stack runs on the named database, that a step has the stated effect. One claim per property asserted, per §5, not one per line.
- **Intent.** A roadmap item, a feature marked as coming, a plan. It concerns what the seller means to do, not the target's present state, and is enumerated as about the seller, per §5. Intent concerns a future state: a statement that the target does not have or do something is about its present state, and is a claim about the target whatever heading it sits under.
- **Puffery.** A vague general statement of quality that a reasonable buyer would not rely on and that no evidence could settle: "blazing fast", "rock solid", "modern", "enterprise-grade". It is not enumerated. A statement that is specific and testable is a claim however it is dressed: "handles 10,000 requests a second" is a claim, "fast" is not.

**Read each assertion as a reasonable buyer in this transaction would read it:** in its ordinary sense, as one statement, using its heading only to resolve a referent, with the claim source, and whatever other documentation of the seller's is in front of you, as the specification of what the software is said to do. A statement that the target belongs to a category, "a URL shortener", "a CRM", is one claim, that the core function of the category exists; it is not expanded into the properties a member of the category might have.

**An assertion about the seller rather than the target is still a claim**, enumerated with `about` set to `seller`, per §5.

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

If you cannot attempt every claim you were given, say so in `not_completed`, give the reason, and emit an empty list for the phase you are in. The client's process carries that into the run, and a run with any claim not attempted does not produce an audit. Do not mark the claims you did not reach as `unverifiable`: that verdict states that the materials cannot settle a claim, and what is true here is that the claim was never attempted.

A derived fact (§7) is evidence. It is not a finding of its own.

## 5. The claim surface

The claim source is given to you one section at a time, in document order, with the claims already enumerated from the sections before it. Enumerate every claim in the section you are given: every assertion §2 admits, and none it excludes. Each one carries:

- **`quote`** — the assertion as the claim source states it, **verbatim**. Copy it; do not paraphrase, tidy, or join text that is not contiguous.
- **`lines`** — where that quote sits in the claim source, as a start and end line number.
- **`statement`** — the claim in your own plain words, so a reader knows what is being tested.
- **`about`** — whom the assertion concerns, per §2:

| `about` | Meaning |
|---|---|
| `target` | The product, code, infrastructure, business or terms the buyer would acquire |
| `seller` | The seller's own activity, hosted service or stated intent: a sign-up page, a hosted plan and its price, a roadmap item. The supplied materials usually cannot reach it; where they do, the claim is adjudicated on them like any other |
| `document` | A document in the materials itself, rather than what the software does: its licence, its existence, what it contains. The document settles such a claim, and a citation into it is evidence for this claim alone, per §7 |

- **`restates`** — present only when the assertion is one already enumerated, made again: the `id` of that claim.

**`id`** is assigned by the client's process, from 1, in document order, once a section's claims are in. Claims from earlier sections reach you with their ids, and that is how findings refer to them.

**An assertion made twice is one claim.** A feature table, a comparison table and a FAQ routinely repeat the headline. Where the section you are given asserts something a listed claim already asserts, emit it with `restates` naming that claim, and the client's process records the second place on the one claim. A claim is adjudicated once, and its finding covers every place it is made. Two assertions that share a subject and differ in what they assert are two claims, per the rule below.

**Divide by what is asserted.** One claim per property asserted about one subject. The properties are few: that something exists or is done; a default; a quantity or limit; a boundary — "only", "no", "all", "never"; a provenance fact — licence, author, origin. A sentence asserting two properties is two claims. The same property asserted in two places is one claim with two locations. The quote is the smallest contiguous span that carries the property and its subject; a lead-in that carries no property — a bullet marker, "i.e.", "and" — is left out of it.

> "Backups run daily with 30-day retention" is **two** claims: a quantity (how often backups run) and a quantity (how long they are kept). Sharing a sentence is not a reason to combine.
>
> "Platform-level redundancy and automatic failover" is **two** claims: that replicas exist, and that failover happens without intervention. Sharing a subject is not a reason to combine.
>
> "Only the message count is stored, never the message" is **two** claims: that the count is stored, and a boundary, that nothing else is. A citation settles the first; the searches §8 requires settle the second.

**One property that names several things is one claim.** Six subcommands listed in one block, three services the stack is said to run on, four export formats: the property is that the subject has the things listed, and it is one claim however many are listed. This is the only grouping the rule allows, and §6 says how such a claim fares when some of the things listed hold and some do not.

**Enumerate before you adjudicate, and enumerate everything.** At this point you have not tested any claim, so you cannot know which will hold. A claim that looks obviously true, or obviously false, or impossible to check, is enumerated exactly like the rest.

**Once the last section is enumerated the surface is frozen.** It is handed back to you for the adjudication phase and it does not change. An assertion you notice after the surface has closed is not added to it and is not adjudicated. Enumerate each section carefully the first time; that is what this phase is for.

## 6. Verdicts

Every finding carries exactly one.

| Verdict | Meaning |
|---|---|
| `real` | The evidence supports the whole claim and nothing in it needs a caveat. For a claim of absence, the evidence is the searches §8 requires |
| `real_with_caveat` | Every part of the claim is borne out, and the evidence shows something a reader must know to read the claim correctly |
| `partial` | The subject has the property the claim asserts, but not to the extent claimed: a quantity or limit differs, or some of the things the claim names are borne out and the others are shown not to be |
| `contradicted` | The subject does not have the property the claim asserts, or has its opposite: the thing does not exist, the boundary is crossed, the default is otherwise, the provenance is different |
| `unverifiable` | The claim was attempted and the supplied materials cannot settle it |

**`partial` is a difference of extent; `contradicted` is a difference of kind.** A claim asserts one property, per §5. Ask whether the subject has that property at all. Where it does, and the evidence shows a different measure or a missing member — retention claimed at 30 days and configured at 7; six subcommands claimed and five registered — the verdict is `partial`, and `gap` states the difference. Where it does not — no retention configured at all; a boundary the claim draws and the evidence crosses; a database other than the one named — the verdict is `contradicted`. A quantity that differs is `partial` whatever the size of the difference: `gap` records the size, and whether it matters is judged later, per the last rule of this section.

**A gap must be citable, not merely unfound.** `partial` and `contradicted` require evidence you can point at showing the gap. Evidence you looked for and did not find is not a gap: for a claim that something exists, that is `unverifiable`, and §8 governs it. For a claim that names several things, where some are found and the rest are neither found nor shown absent, the claim is `unverifiable`, and the citations record what was found.

**Choosing between `partial` and `real_with_caveat`.** Ask whether you can cite a part of the claim — a measure, or one of the things it names — that the evidence shows is not as claimed. If you can, the verdict is `partial`. If every part holds and there is still something to say, it is `real_with_caveat`.

> "Blended MRR is $40,000", against a processor export showing $16,000 of subscription charges in the month and three wire transfers of $8,000 in the same month from customers on annual invoices, with no customer appearing in both. Every part of the assertion holds; the month's recurring total is $40,000. That 60% of it arrives by manual wire from three customers is not a failure of the claim, and a reader must know it. `real_with_caveat`.
>
> "The CLI has `init`, `build`, `deploy`, `status`, `logs` and `rollback`", against a command table registering the first five and no `rollback` anywhere the two §8 searches looked. Five of the things named are borne out; the table that registers commands shows the sixth is not there, and can be cited. `partial`.

**A caveat is not for weak evidence.** If what you have does not settle the claim, the verdict is `unverifiable` and §8 governs it. A framework version inferred from directory names, where no manifest or lockfile was supplied, is `unverifiable` — not a claim that holds with a caveat. The verdict says how the claim fared against the evidence; it does not say how confident you are.

**A claim of absence is settled by searches, not by a citation.** "No tracking", "only the count is stored", "never writes to disk": a line showing the mechanism the seller names shows what that mechanism does, not that nothing else does more. Such a claim is `real` only on the two searches §8 requires, made over every place the excluded thing could occur, with every candidate opened; a citation alone does not settle it. §8 says how.

> "No telemetry", against a lexical search for analytics, telemetry, tracking and metrics terms and a structural search over the network calls and the dependency manifest, every candidate opened, nothing found. The searches are the evidence. `real`.
>
> "Never the message", the boundary claim §5 splits from "only the message count is stored", against a debug middleware that writes each message body to the log. The thing excluded is found, and the log line can be cited. `contradicted`. The claim that the count is stored is a separate claim, settled by the counter column.

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

**`search`** — a record of a search performed to determine whether the supplied materials hold anything that could settle the claim. There is no line to cite for something that is absent, so searches are the evidence for a claim of absence, and the record of diligence behind an `unverifiable` verdict. §8 governs both.

- **`kind`** — `lexical` or `structural`;
- **`performed`** — what you actually searched;
- **`result`** — what came back;
- **`candidates`** — every file the search found where material that could settle the claim would appear, each by its path from the target root. An empty list when the search found none. A file listed here and not opened makes the claim `not_examined`, per §8.

**For numbers and dates, quote the source line.** A summary produced by a tool is useful for navigation and is not sufficient evidence for an exact quantitative claim.

**Documentation is not evidence for a claim about the software.** A claim about what the software does is settled by files that take part in building or running it: source, configuration, build and deployment files, dependency manifests, database migrations, tests. A README, a page under a docs directory, a website export or any claim source, named for this run or not, restates a claim and does not settle it. The engagement lists such files as excluded; the evidence request will not quote their text, and shows only that they exist, their length and their headings. Inside a code file, cite the statement, not the comment above it.

**A claim about a document, `about` set to `document` per §5, is settled by the document.** A file the engagement does not exclude is cited like any other: "MIT licensed" is settled by the text of the LICENSE file, and what it settles is that the file is the MIT licence, not that the software may lawfully be distributed under it, which is a legal question §11 leaves alone. For an excluded file, what the evidence request shows settles what it can: that the document exists, its length, and the topics its headings name. "The API is documented" is settled by a documentation file whose headings name the API. A claim about what such a document says beyond its headings — that it describes every endpoint, that a procedure in it is complete — is `unverifiable` with the disposition `present_but_not_readable`, per §8: the document is supplied and cannot be read as given.

**A business claim is settled by business records.** Revenue, customer counts, contracts and dependencies on outside parties are settled by the records supplied for them: processor exports, invoices, ledgers, signed agreements, the manifest that names a dependency. A number the seller states in prose is the claim, not the record.

**Evidence settles a claim only at the scope the claim states.** A claim names a subject, and often a condition, a version, a quantity or a period; the evidence must reach the same ones. A test file shows that a test is written, not that it passes. A deployment file shows what a deployment is configured to be, not what is running. A code path shows that a capability exists, not the throughput, uptime or usage claimed for it. Where the evidence reaches less than the claim states, it does not settle the claim, and §8 governs. Where two admissible sources disagree, cite both and say in `shows` which you relied on and why.

**Every citation must resolve.** Before you emit, check that each document named exists, each line range is real, and each quoted span appears in the document as you have written it.

Correct any citation that does not resolve. If it cannot be corrected — the material you remembered is not there — remove that item. If no remaining evidence settles the claim, the verdict is `unverifiable` under §8. Do not drop the finding: §4 gives every claim exactly one.

This check confirms that the cited text exists. It does not establish that the text supports the claim, which remains your judgement.

## 8. Searches: claims of absence, and claims the materials cannot settle

Two kinds of claim rest on searches rather than on a citation: a claim that the target lacks something or does one thing and nothing else, and a claim the supplied materials cannot settle. A finding of either kind takes the same shape as any other. Its evidence carries `search` items, per §7, and it must **record** the searches rather than assert them.

Two are required, and they are complementary:

- **`lexical`** — search using terms taken from the claim itself, including reasonable stems and variants;
- **`structural`** — inspect the directory, module or document set where the material would reasonably appear.

For each, state what you actually searched, what came back, and in `candidates` the files it found where the material would appear. A search shows what the materials hold where you looked and nothing about where you did not, which is why both kinds are required and every candidate is opened.

**Every candidate is opened before a claim resting on searches gets its verdict.** A file the searches named and nobody read does not show that the thing is absent, or that the materials cannot settle the claim; it shows that the engagement has not looked. *Opened* means a read of the file was requested and what came back is in the evidence requests: its contents, or the failure to read them. A candidate that was opened and could not be read — a binary, a compiled archive, a truncated or encrypted file — settles nothing, and where no other evidence settles the claim, the disposition — the `unresolved_because` field, from the table below — is `present_but_not_readable`. If a candidate of any search on the claim was not opened, and no citation settles the claim, the verdict is `unverifiable` with the disposition `not_examined`; the client's process then asks you to open those files and to adjudicate the claim again, per §10. The other three dispositions are recorded only when every candidate has been opened. `not_examined` requires a named file: where the searches named no file, one of the other three applies.

**A claim of absence.** "No telemetry", "only the count is stored", "never writes to disk". The two searches are its evidence, made over every place the excluded thing could occur: the code path the claim concerns, and the paths around it that could do the same thing another way — handlers, middleware, logging, configuration, dependencies. A citation showing the mechanism the seller names — a schema without the column, a handler that records only the count — shows what that mechanism does and by itself settles nothing about the rest. Where the searches find the thing, the verdict is `contradicted`, per §6, citing what was found. Where both searches were made, every candidate opened and nothing found, the verdict is `real`, with the searches as its evidence. `unverifiable` is for the case where the places the thing could live were not supplied: the deployment that would show a logging sidecar, the configuration that would show an outbound endpoint.

When the verdict is `unverifiable`, the adjudication also records what the searches established, as one of:

| `unresolved_because` | Meaning |
|---|---|
| `not_in_the_materials` | Material of the right kind was supplied, and none of it settles the claim — an accounting export that carries no customer count |
| `present_but_not_readable` | The material is supplied and cannot be interrogated as given — a compiled archive with no source |
| `outside_the_materials` | The kind of material needed to test the claim was not supplied at all — no customer records of any sort |
| `not_examined` | The searches named one or more files where the material would appear, and at least one of them was not opened in this engagement |

For a claim that something exists, searches that find nothing settle nothing, and the verdict is `unverifiable`, per §6. The one exception is a place that registers every member of a kind — a command table, a route list, a dependency manifest, a migration directory. Opened, it shows which of the things a claim names are absent, and it is cited as the evidence for `partial` or `contradicted`.

## 9. Unclaimed observations

The materials may show something relevant that the seller never asserted. It is not a finding — there is no claim for it to adjudicate — and it is not discarded. Record it as an unclaimed observation with a short note saying what it shows.

**This is the only thing an unclaimed observation is.** It is not a place for an assertion the frozen surface missed: §5 freezes the surface, and nothing adds to it afterwards by any route.

What the observation rests on is written as one §7 evidence item: a `citation` where a single place shows it, or a `derived` item where it follows from two or more facts and no single quote holds it.

## 10. Correction protocol

If evidence found later contradicts a finding you have already formed, revise the finding before emitting and record in `correction`, in one line, what changed and why.

The client's process may hand a claim back to you with the adjudication you gave it and further evidence, and ask you to adjudicate it again, per §8. Emit a whole finding. Where the verdict or the disposition changes, `correction` says what changed and why; where neither changes, leave it out.

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

3. **Gather evidence** across the materials §7 admits as evidence. The evidence that settles a claim usually sits far from the claim source. On every evidence request, name the claims it gathers evidence for: the record of the request is filed under those claims, and when you adjudicate a claim the client's process hands you the requests filed under it, and fetches nothing else for it. What is handed to you is admissible for any claim it bears on, whichever claim it was filed under. Material that bears on no claim, per §9, reaches the adjudication only through a request filed under some claim: file it under the claim whose evidence led you to it.

4. **Adjudicate each frozen claim** and assign one verdict, per §6. Where a claim raises something only the seller can answer, record it in `questions`.

   `questions` supplements the findings and never replaces one. A claim that needs seller information still gets its finding, and still gets `unverifiable` if the supplied materials cannot settle it.

   If a claim you were given cannot be attempted, stop here: emit `not_completed` with the reason and an empty `findings`, per §4.

5. **Check every citation resolves**, per §7.

6. **Emit the findings document**, per §13.

## 13. The output

Two kinds of JSON object, one per phase. Phase one is emitted once per section of the claim source; phase two once per batch of claims. Each is complete for what it was asked, and nothing goes outside it.

**Phase one — the claim surface.**

| Field | Contents |
|---|---|
| `claim_source` | The document named by the engagement |
| `claims[]` | Every claim in the section you were given, per §2 and §5 |
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
| `findings[].adjudication.gap` | What the verdict rests on: the caveat a reader needs, the part of the assertion that fails, or the contradiction. Required for `real_with_caveat`, `partial` and `contradicted`. Absent for `real`, which has nothing beside it, and for `unverifiable`, which uses the field below. **Where there is no gap, leave the field out.** Do not write "None" — an absent field is how the output says there is nothing, and the word is a value like any other. Why a claim holds belongs in its evidence, under a citation's `shows` or a search's `result` |
| `findings[].adjudication.unresolved_because` | Per §8, required when the verdict is `unverifiable` |
| `findings[].evidence[]` | One list, each item declaring its `form` — per §7 |
| `findings[].correction` | Per §10, where a finding changed |
| `unclaimed[]` | `note` and one §7 evidence item — per §9 |
| `questions[]` | Questions only the seller can answer, raised by a claim — per §12 |
| `not_completed` | Present only when a claim you were given could not be attempted, per §4. When it is present, `findings` is empty. The client's process carries it into the assembled run, beside the findings of the batches that completed |

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
