# Technical claims audit — method

## 1. What this stage does

One claim-source document is audited per run. The engagement names it. Every other supplied document is an **evidence source**, and all of them are available to you. *Evidence* means the material actually cited in a finding, per §7.

> Compare what the seller asserts in this claim source against what the supplied materials show, and cite the material used.

The run has two phases and produces two JSON documents. First you enumerate the claims and the surface is **frozen**; then you adjudicate the frozen claims against the evidence. Both are read by a machine, not by the buyer. A later stage combines the runs over every claim source, and a stage after that writes the client's report.

**The surface is frozen before any claim is adjudicated, and this is a control rather than a convenience.** Enumerating while adjudicating means choosing what counts as a claim with the verdicts already in view, which is choosing the sample after seeing the result. You will be given the frozen list back; you do not revise it.

**The schema enforces the shape. This document says what makes a field correct.** A response can satisfy the schema and still be wrong: a quote that is not verbatim, evidence that does not bear on the claim, an `unverifiable` that should have been `contradicted`. Those are what this method governs.

Two things follow from being one stage of several:

- **Do not judge the offering.** No overall conclusion, no summary of what the target is worth, no statement about whether the buyer should proceed. Those depend on every claim source, and this run has seen one.
- **Do not write for a reader.** No covering note, no narrative, no recommendations. Fill the fields.

## 2. Scope rule

**Evaluate what the seller asserts about the target against what the supplied materials show.**

A claim is an assertion the seller makes to the buyer in the designated claim source — a listing, a technical description, a specification, a marketing document. The engagement names which document that is for this run.

Other supplied documents, including source code and code comments, are evidence sources. They test claims. They are not themselves seller claims, even when they assert something.

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
- **adjudication** — the verdict, together with the gap or unresolved status that verdict requires.
- **finding** — one output record: the `claim_id` of a frozen claim, its adjudication, and the evidence used.

Every frozen claim produces exactly one finding, and every finding adjudicates exactly one claim. A claim carries one verdict, so two findings on one claim are either redundant or in conflict.

**A finding names its claim by `claim_id` and does not restate it.** The frozen surface is what the claim says; the finding is what you concluded about it. Two copies of one assertion can disagree, and a reader would have no way to tell which was authoritative.

Where one piece of evidence settles several claims, write a finding for each and cite that evidence in each. Do not join claims into one finding to avoid repeating a citation.

**Attempt every claim.** You may not skip one because it looks unimportant — the significance of a gap is not known until the claim is tested.

If you cannot reach them all, the run does not produce an audit. Say so in `not_completed`, give the reason, and emit an empty list for the phase you are in. Do not mark the claims you did not reach as `unverifiable`: that verdict states that the materials cannot settle a claim, and what is true here is that the claim was never attempted.

A derived fact (§7) is evidence. It is not a finding of its own.

## 5. The claim surface

Read the claim source and enumerate every assertion in it the seller makes about the target. Each one carries:

- **`id`** — a number, from 1, counting up in the order the claims appear in the document. This is how findings refer to it.
- **`quote`** — the assertion as the claim source states it, **verbatim**. Copy it; do not paraphrase, tidy, or join text that is not contiguous.
- **`lines`** — where that quote sits in the claim source, as a start and end line number.
- **`statement`** — the claim in your own plain words, so a reader knows what is being tested.

**Where to divide the text into claims.** Split assertions apart where different evidence could give them different verdicts. Keep them together where they stand or fall on the same evidence.

Sharing a subject is not a reason to combine: "backups run daily" and "backups are retained 30 days" are settled by different evidence and are two claims.

**Enumerate before you adjudicate, and enumerate everything.** At this point you have not tested any claim, so you cannot know which will hold. A claim that looks obviously true, or obviously false, or impossible to check, is enumerated exactly like the rest.

**Once emitted the surface is frozen.** It is handed back to you for the adjudication phase and it does not change. An assertion you notice after the surface has closed is not added to it and is not adjudicated. Enumerate carefully the first time; that is what this phase is for.

## 6. Verdicts

Every finding carries exactly one.

| Verdict | Meaning |
|---|---|
| `real` | The evidence bears the claim out, and there is nothing a reader needs beside it |
| `real_with_caveat` | Every part of the claim is borne out, and the evidence shows something a reader must know to read the claim correctly |
| `partial` | A part of the assertion is not borne out, and you can name which part |
| `contradicted` | The claim is false — the claim source says one thing and the evidence shows another |
| `unverifiable` | The claim was attempted and the supplied materials cannot settle it |

**Choosing between `partial` and `real_with_caveat`.** Ask whether you can point at a part of the assertion the evidence does not bear out. If you can, the verdict is `partial`. If every part holds and there is still something to say, it is `real_with_caveat`.

> "Blended MRR is $40,000", against $16,000 from the payment processor and three wire transfers of $8,000. Every part of the assertion holds; the total is $40,000. That 60% of it arrives by manual wire from three customers is not a failure of the claim, and a reader must know it. `real_with_caveat`.
>
> "The technology stack is scalable", against a single dyno with the database co-located on it. The platform supports scaling; this deployment does not. A part of the assertion fails and can be named. `partial`.

**A caveat is not for weak evidence.** If what you have does not settle the claim, the verdict is `unverifiable` and §8 governs it. A framework version inferred from directory names, where no manifest or lockfile was supplied, is `unverifiable` — not a claim that holds with a caveat. The verdict says how the claim fared against the evidence; it does not say how confident you are.

**Keep `partial` and `contradicted` apart.** `partial` means most of the claim is borne out and a named part is not. `contradicted` means the evidence says otherwise.

**Do not judge whether a gap matters to the buyer.** That judgement depends on the transaction and on what every other claim source shows, and a later stage makes it. Record what the claim says, what the evidence shows, and the difference.

**`unverifiable` is not `contradicted`.** Failing to find supporting evidence is not evidence that the claim is false. §8 governs it.

## 7. Evidence

Every finding cites the material that settles it. A reader must be able to inspect both sides of the comparison: the frozen claim §5 enumerated, and the evidence here.

A finding's evidence is **one list**. Each item in it declares its `form`, and the form decides which other fields the item carries. A finding may hold any number of items in any mix — two citations and a derived statement, or a citation and two searches.

**`citation`** — material that bears directly on the claim.

- **`document`**, **`lines`** — where it is;
- **`quote`** — the text, verbatim;
- **`shows`** — what it demonstrates about the claim.

**`derived`** — a fact that follows from two or more supplied facts. A retention period in one document and the date of the last usable backup in another together determine when recovery becomes impossible; the seller stated neither consequence.

- **`basis`** — every source fact it rests on, each with its document, lines and verbatim quote;
- **`derivation`** — the arithmetic or reasoning, shown so a reader can reproduce it;
- **`consequence`** — what follows, and why it bears on the claim.

Derive only from facts supplied in the materials; do not import market forecasts or other outside conclusions. State the consequence and stop — do not turn it into advice about the transaction. Where the derivation depends on time, give the relevant date and what changes on it. A derivation that bears on no claim is not evidence about a claim; report it under §9.

**`search`** — a record of a search performed to determine whether the supplied materials hold anything that could settle the claim. A statement that something is not implemented cannot rest on a line, because there is no line to cite; it rests on searches. §8 governs when they are required and what they settle.

- **`kind`** — `lexical` or `structural`;
- **`performed`** — what you actually searched;
- **`result`** — what came back.

**For numbers and dates, quote the source line.** A summary produced by a tool is useful for navigation and is not sufficient evidence for an exact quantitative claim.

**Every citation must resolve.** Before you emit, check that each document named exists, each line range is real, and each quoted span appears in the document as you have written it.

Correct any citation that does not resolve. If it cannot be corrected — the material you remembered is not there — the claim is `unverifiable` under §8, and the finding says so. Do not drop the finding: §4 gives every claim exactly one.

This check confirms that the cited text exists. It does not establish that the text supports the claim, which remains your judgement.

## 8. Claims the materials cannot settle

An `unverifiable` finding takes the same shape as any other. Its evidence carries `search` items, per §7, and it must **record** the searches rather than assert them.

Two are required, and they are complementary:

- **`lexical`** — search using terms taken from the claim itself, including reasonable stems and variants;
- **`structural`** — inspect the directory, module or document set where the material would reasonably appear.

For each, state what you actually searched and what came back. A search establishes diligence, not absolute absence.

The adjudication also records what the searches established, as one of:

| `unresolved_because` | Meaning |
|---|---|
| `not_in_the_materials` | Material of the right kind was supplied, and none of it settles the claim — an accounting export that carries no customer count |
| `present_but_not_readable` | The material is supplied and cannot be interrogated as given — a compiled archive with no source |
| `outside_the_materials` | The kind of material needed to test the claim was not supplied at all — no customer records of any sort |

If the searches do not settle the question, the verdict is `unverifiable`, never `contradicted`.

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

1. **Read the claim source named by the engagement.** If the engagement names none, the run cannot proceed: say so in `not_completed`, per §4.

2. **Enumerate every claim in it** and emit the claim surface, per §5 and §13. Do not gather evidence yet and do not form verdicts. The surface freezes when you emit it.

**Phase two — adjudicate.** You are given the frozen surface back.

3. **Gather evidence** across all the supplied materials, not only the claim source. The evidence that settles a claim made in one document usually sits in another.

4. **Adjudicate each frozen claim** and assign one verdict, per §6. Do not ask the client between claims. Where a claim raises something only the seller can answer, record it in `questions`.

   `questions` supplements the findings and never replaces one. A claim that needs seller information still gets its finding, and still gets `unverifiable` if the supplied materials cannot settle it.

   If it becomes clear that every frozen claim cannot be attempted, stop here: emit `not_completed` with the reason and an empty `findings`, per §4.

5. **Check every citation resolves**, per §7.

6. **Emit the findings document**, per §13.

## 13. The output

Two JSON objects, one per phase. Each is emitted once, complete, and nothing goes outside it.

**Phase one — the claim surface.**

| Field | Contents |
|---|---|
| `claim_source` | The document named by the engagement |
| `claims[]` | Every assertion the claim source makes, per §5 |
| `claims[].id` | A number, from 1, in document order |
| `claims[].quote` | The assertion verbatim, per §5 |
| `claims[].lines` | Where that quote sits in the claim source |
| `claims[].statement` | The claim in plain words |
| `not_completed` | Present only when the claim source could not be read, per §4. When it is present, `claims` is empty |

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
| `search` | `kind`, `performed`, `result` |

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
- `/home/bruce/projects/Body` — a real robot codebase used as a second data room, audited independently of its development.

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

Kept separate so obsolete vocabulary does not sit near the active instructions.

### The deliverable was five text blocks

The audit emitted `CLAIM SURFACE`, `REPORT`, `COVERAGE`, `LIMITATIONS` and `GAP MAP` as marker-delimited prose. Markers replaced conversational turn boundaries, which did not reliably indicate what had been completed; a schema replaces the markers for the same reason and more strongly, because a marker can be named in a status line without the block being written, and a schema field cannot.

### The report carried a conclusion, a Gap Map and a limitations statement

All three judged the offering as a whole. A run that sees one claim source cannot make that judgement, and a per-run version of it is not merely redundant but false once runs are combined — a limitations statement naming the documents a run did not use is wrong the moment its findings sit beside theirs. They moved to the stage that has every run in front of it.

### Claims were enumerated into a frozen surface, and coverage divided by it

The surface was the denominator for coverage figures. It was also self-declared: nothing validated that a run's own count was the number of claims the document contains, so a run that identified twenty and adjudicated five could not be told from one that identified five. Bounding each run to one claim source, and estimating the surface from agreement across runs, replaces an unvalidatable denominator with a measurable one.

### Claims were assigned one of four priority tiers

The tiers ordered the work so the highest-consequence claims were settled first, and ordered the findings in the report. Neither job remains: every claim is attempted, so there is nothing to triage, and the report is assembled elsewhere. Materiality — whether a gap would change a buyer's decision — moved to the stage that can weigh it, because it depends on the transaction and on what every other run found.

### The finding was defined ontologically

§4 once read: "A finding is the statement of the verdict of adjudicating a set of cited evidence with respect to a claim." That is the formulation the ontology was settled on, and it is why a finding contains its evidence rather than pointing at it. It was replaced in the executable text by four nested definitions, because it introduces `finding`, `verdict` and `adjudication` at once and a model has to unpick the recursion before it can act.

### There were two caveat verdicts, and a claim could hold with one

`real_minor_caveat` and `real_operational_caveat` both meant "the claim holds,
however —". Across three runs they took 18 of 64 findings, and **five of the
nine adverse findings two stronger models independently produced on the same
corpus were filed under `real_operational_caveat`**, recorded as claims that
hold. Minimal operational overhead and horizontal scaling both went that way.

The cause was structural rather than a model failing. The verdicts encode one
dimension — how the claim fared against the evidence — and a model that is
confident but not certain has nowhere to put that, so uncertainty drained into
the slot that sounded like it. A framework version inferred from directory
names was recorded as holding with a caveat when §8 makes it `unverifiable`.

One caveat verdict remains because one case needs it and nothing else fits:
a claim every part of which is true, where the evidence shows something a
reader must know. Blended MRR of $40,000 that is 60% manual wire transfers
from three customers is the case, and it is a finding three earlier runs
missed entirely. `partial` would be wrong — no part of the assertion fails —
and `real` would be misleading. What changed is that the boundary is now a
test rather than a magnitude: can you name a part of the assertion that fails?

### `delta` was the word for a contradicted claim

`delta` is engineering vocabulary for a difference. It is not the register the
rest of this method borrows — assurance work says misstatement, exception or
deviation — and it names only that something differs, which is equally true of
`partial` and of a caveat. `contradicted` says the direction, which is the
thing that separates it from the others.

### The verdict vocabulary was bracketed

Verdicts were written `[real]`, `[partial]`, `[delta]` because they had to be parsed out of prose. They are now field values.

### `[unclaimed]` was once named for what it was not

The category for material the seller did not claim was previously named as the negation of a contradicted claim, which read as "this claim is not false" rather than "no claim was made".

### The conclusion vocabulary once contained buyer actions

Report conclusions once included instructions to the buyer. They were removed because the audit reports the state of the evidence; the vocabulary itself has since moved downstream with the conclusion.
