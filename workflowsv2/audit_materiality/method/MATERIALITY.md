# Materiality — method

## 1. Purpose

For each finding of a finished claims audit, say whether it changes what a buyer would assume if every claim in the claim sources held.

> Rate each finding by what it would change: nothing, the price or the terms, or the decision to close.

A finding with a verdict about the claim is rated for **materiality**: what the gap the audit showed would change. A finding the audit could not settle is rated for **exposure**: what it would change if the claim turned out to be false. The two are the same judgement on the same scale, and they are never counted together, because only the first is something the audit showed.

**A gap**, in this document, is what the audit showed against or beside a claim: the caveat of a `real_with_caveat` finding, the part that fails in a `partial` one, or what is so instead in a `contradicted` one, as the finding's `gap` field states it. The fact an observation records is rated as a gap too (§4). An unsettled claim has no gap (§6).

You did not perform the audit and you do not revise it. The findings are settled. Your subject is their consequence for the transaction as the engagement states it.

More is asked of this stage. For each gap the audit showed, say in words the buyer can read on their own what the claim said and what is actually so (§5b), and what change to the target would close the gap and how much of the target that change touches (§5a). And rate the observations the audit recorded without a claim, as gaps it showed (§4).

## 2. What you have

**The findings** — every finding of every claim source in the engagement, merged. Each carries the frozen claim (`quote`, `lines`, `statement`), its `adjudication` (`verdict`, and `gap` or `unresolved_because`), its `evidence`, the review's outcome (`holds`, `does_not_hold` with the observations that failed, or `unreviewed`), and any citation problem the client's process recorded.

**The observations** — what the audit recorded that no claim in the claim sources asserts, numbered, each with a note and one evidence item. One fact may be recorded several times, once by each claim source's run that met it.

**The transaction** — what the engagement states about the buyer, the purpose of the acquisition, the price basis and the intended structure. Where the engagement states none of this, rate against a buyer paying a price that assumes every claim holds.

**The buyer's thresholds** — what the engagement records of the buyer's own view: what they are paying for, what would change the price or the terms, and what would make them walk away. Where the engagement records them, a rating is read against them. A gap of a kind the buyer has said would change the price is `material` where the gap alone is large enough that the buyer would go back to the seller over it; where it is too small for that it is `not_material`, and `basis` says that the threshold reaches it and why the gap is too small. A gap they have said would end the deal is `decisive`. Where the engagement records none, rate as §3 defines the values.

**The reliance statement**, where the engagement has one — the practice's account, corrected by a person, of what this buyer relies on the offering for: for each function, property or commitment, whether the buyer's plan `depends` on it, `uses` it or `does_not_use` it, what the buyer would have to do if it failed, and whether that rests on the buyer's words or the practice's inference. Find the item each finding falls under, and use it to decide how large the gap is for this buyer: a gap that defeats an item the buyer `depends` on is one the buyer would go back to the seller over; a gap under an item the buyer `does_not_use` is `not_material` unless a stated threshold names it. `basis` names the item. Do not add facts about the buyer that neither the engagement nor the reliance statement states.

You have no tools and no access to the target. Rate on what the finding shows.

## 3. The scale

Every rated finding carries exactly one value, as `materiality` or as `exposure`.

| `materiality` | Meaning |
|---|---|
| `not_material` | The finding would change neither the price, nor the terms of the transaction, nor the decision to close |
| `material` | The finding would change the price or the terms materially, and a buyer would still close. Materially means: the buyer would go back to the seller to reopen the price or the terms over this finding alone |
| `decisive` | The finding on its own would change the decision to close |

`exposure` takes the same three values with the same meanings, read for a claim assumed false rather than for a gap the audit showed.

**Terms** means the structure of the transaction: conditions, warranties, escrow, earn-out, and what is included.

## 4. What is rated, and how

- `contradicted` and `partial` — rate `materiality` on what the `gap` field and the evidence show.
- `real_with_caveat` — rate `materiality` on the caveat: what a buyer must know to read the claim correctly.
- `unverifiable` — rate `exposure`: what it would change for the buyer if the claim were false. The audit showed nothing against this claim; the rating says how much rests on it, and `basis` says so. Do not rate its `materiality`.
- `real` — not rated. A `real` finding changes nothing a buyer assumed.
- An observation — rate `materiality`, as for a gap the audit showed, on what its note and evidence show. First put the observations that record the same fact in the target into one group; each group is rated once, and every observation is in exactly one group. Two observations about the same file that record different facts are two groups.

A finding whose review outcome is `does_not_hold`, or which carries a citation problem, is rated the same way. The client's process reports the rating beside that outcome.

Rate each finding on its own. Do not combine findings, and do not rate a finding by its effect on another. Observations are grouped only where they record the same fact.

## 5. `basis`

One to three sentences, starting from the claim (for an observation, from what the review found): what the buyer was told, what the review showed instead, and why that difference does or does not reach the transaction and the buyer's thresholds as stated. Write for the buyer: no file names, code identifiers or library names unless the buyer would know them. Do not restate the `gap` field; the reader has it beside the rating. For `exposure`, say what the buyer would lose if the claim were false.

**Say what the rating rests on.** The last sentence of `basis` is one of these, exactly:

- *Rated against the buyer's stated threshold: "<the buyer's words>".* — where a stated threshold reaches the finding. The quotation is the threshold the rating rests on, in the buyer's words.
- *Rated on the practice's reading of the buyer's plan.* — where no stated threshold reaches the finding and the rating rests on an item of the reliance statement marked as the practice's inference.
- *Rated on the scale alone.* — where neither applies.

A buyer reading the rating must be able to tell their own words reflected back from the practice's judgement.

**Words for the buyer.** `basis`, `remedy` and `in_short` are read by the buyer, and the buyer's document does not use the words *audit* or *auditor*. In all three, the engagement is *the review*, the party that performed it is *the practice*, and the independent second pass over the findings is *the check*. This document and the record use *audit* for the engagement and *review* for the second pass; that is their vocabulary, not the buyer's.

## 5a. `remedy`

For every finding rated for `materiality`, and every group of observations, say what would close the gap.

- **`change`** — one sentence: what in the target would have to change for the gap to close, where the evidence locates it. For example: "set the backup job to keep backups for 30 days", "check the caller's permission in the handler that deletes accounts". Say what changes, not how to make the change, and not whether to make it.
- **`band`** — how much of the target that change touches. A change that touches more of the code carries more risk of breaking something, whoever makes it. The band is not a time or an effort estimate.

| `band` | Meaning |
|---|---|
| `none` | Nothing in the target needs to change: the caveat is about how the claim is read, not a defect. `change` says so in a few words |
| `setting` | A configuration value or a default, changed in one place |
| `one_place` | A code change inside one file or module |
| `several_places` | Changes across several files or components |
| `design` | A change to the data model, the architecture, or an interface other code relies on |

Judge the band from the gap and the files the evidence cites. Where the evidence does not show how far a change would reach, choose the wider of the bands you are deciding between.

## 5b. `in_short`

For every finding rated for `materiality`, and every group of observations: one or two sentences the buyer can read on their own, with no other part of the document in front of them. Say what the claim told the buyer and what is actually so. For a `real_with_caveat` finding, say what the caveat is; for `partial`, which part fails; for `contradicted`, what is so instead. For an observation, say what the review found.

Write in words the buyer understands: no file names, code identifiers or library names unless the buyer would know them. State nothing the `gap` field, or the observation's note and evidence, does not state. Each finding says who made the statement and whether it rests on knowledge of a library. A statement the buyer or the practice asked is not something the seller said: write "the buyer asked whether…", not "the listing says…". Where the finding rests in part on what a library does, from knowledge of it rather than from the materials, say so in words: "from the library's documented behaviour".

> The listing says backups are kept for 30 days. The backup job as shipped keeps them for 7.

## 6. What this stage does not do

- It does not value the target or state a price.
- It does not advise the buyer. A `remedy` says what would close a gap and how much of the target that touches; it does not say whether the gap should be closed, by whom, or at what cost.
- It does not change a verdict, add a finding, or rate a claim the audit did not make. The observations it rates are the audit's own.
- It does not rank findings beyond the three values of §3.
- It does not judge whether the audit was performed well.
- It does not treat an unsettled claim as a gap: `exposure` is never counted as `materiality`.

## 7. The output

Your answer is one JSON object. Its shape is enforced; this document says what makes a field correct.

You may be asked for the whole set of findings or for one batch. A batch holds findings of one kind: those rated for `materiality`, or those rated for `exposure`. Answer with the findings you were asked for, in the array named for them, and leave the other array empty. The client's process assembles the batches. When you are asked for the observations, answer with `observations` alone; when you are asked to check `in_short`, with `checks` alone; when you are asked to write `in_short` again, with `rewrites` alone (§8).

| Field | Contents |
|---|---|
| `ratings[]` | One per finding you were given that is rated for `materiality` |
| `ratings[].claim_source` | The finding's claim source, as given |
| `ratings[].claim_id` | The finding's claim id, as given |
| `ratings[].materiality` | One value from §3 |
| `ratings[].basis` | Per §5 |
| `ratings[].in_short` | Per §5b |
| `ratings[].remedy.change` | Per §5a |
| `ratings[].remedy.band` | One value from §5a |
| `exposures[]` | One per finding you were given that is rated for `exposure` |
| `exposures[].claim_source` | The finding's claim source, as given |
| `exposures[].claim_id` | The finding's claim id, as given |
| `exposures[].exposure` | One value from §3 |
| `exposures[].basis` | Per §5 |
| `observations[]` | One per group of observations, when you are asked for the observations, per §4 |
| `observations[].members` | The numbers of the observations in the group, as given |
| `observations[].materiality` | One value from §3 |
| `observations[].basis` | Per §5 |
| `observations[].in_short` | Per §5b |
| `observations[].remedy` | Per §5a |
| `checks[]` | One per `in_short` you were asked to check, per §8 |
| `checks[].key` | The key the `in_short` was given under |
| `checks[].verdict` | One value from §8 |
| `checks[].words` | The words at fault, quoted from the `in_short`; empty for `holds` |
| `checks[].why` | One sentence, per §8; empty for `holds` |
| `rewrites[]` | One per `in_short` you were asked to write again, per §8 |
| `rewrites[].key` | The key it was given under |
| `rewrites[].in_short` | The new `in_short`, per §5b |

Emit nothing outside the JSON object.

## 8. Checking `in_short`

The `in_short` of §5b is what a buyer reads first, and it carries no citation, so each one is checked against what it was written from before it is shown.

**When you are asked to check.** You are given, for each rated finding and each group of observations, the text the `in_short` was written from (the finding's `gap` field, or the observations' notes; below, *the source*), who made the statement, whether the finding rests on knowledge of a library, and the `in_short` itself. You did not write the `in_short`. Give each one verdict:

| `verdict` | Meaning |
|---|---|
| `holds` | It states nothing the source does not state |
| `adds` | It states a fact or a consequence the source does not |
| `contradicts` | It states something the source rules out |
| `misattributes` | It says the seller asserted what the buyer or the practice asked, or the reverse |
| `unmarked_knowledge` | The finding rests on knowledge of a library, and the `in_short` states that behaviour as fact without saying so |

A plain rewording of what the source says, a simpler word for a technical one, or a detail left out, is `holds`. For any other verdict, quote the words at fault and say why in one sentence.

**When you are asked to write again.** You are given the same, with the check's verdict and reason. Write the `in_short` again per §5b, without what the check found.
