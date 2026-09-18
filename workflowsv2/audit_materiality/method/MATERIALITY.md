# Materiality — method

## 1. Purpose

For each finding of a finished claims audit, say whether it changes what a buyer would assume if every claim in the claim sources held.

> Rate each finding by what it would change: nothing, the price or the terms, or the decision to close.

A finding with a verdict about the claim is rated for **materiality**: what the gap the audit showed would change. A finding the audit could not settle is rated for **exposure**: what it would change if the claim turned out to be false. The two are the same judgement on the same scale, and they are never counted together, because only the first is something the audit showed.

You did not perform the audit and you do not revise it. The findings are settled. Your subject is their consequence for the transaction as the engagement states it.

## 2. What you have

**The findings** — every finding of every claim source in the engagement, merged. Each carries the frozen claim (`quote`, `lines`, `statement`), its `adjudication` (`verdict`, and `gap` or `unresolved_because`), its `evidence`, the review's outcome (`holds`, `does_not_hold` with the observations that failed, or `unreviewed`), and any citation problem the client's process recorded.

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

- `contradicted` and `partial` — rate `materiality` on what the `gap` and the evidence show.
- `real_with_caveat` — rate `materiality` on the caveat: what a buyer must know to read the claim correctly.
- `unverifiable` — rate `exposure`: what it would change for the buyer if the claim were false. The audit showed nothing against this claim; the rating says how much rests on it, and `basis` says so. Do not rate its `materiality`.
- `real` — not rated. A `real` finding changes nothing a buyer assumed.

A finding whose review outcome is `does_not_hold`, or which carries a citation problem, is rated the same way. The client's process reports the rating beside that outcome.

Rate each finding on its own. Do not combine findings, and do not rate a finding by its effect on another.

## 5. `basis`

One to three sentences: what in the finding's `gap` or evidence drives the rating, and how it bears on the transaction and the buyer's thresholds as stated. Quote the figure or the fact from the finding. Do not restate the claim. For `exposure`, say what the buyer would lose if the claim were false.

**Say what the rating rests on.** The last sentence of `basis` is one of these, exactly:

- *Rated against the buyer's stated threshold: "<the buyer's words>".* — where a stated threshold reaches the finding. The quotation is the threshold the rating rests on, in the buyer's words.
- *Rated on the practice's reading of the buyer's plan.* — where no stated threshold reaches the finding and the rating rests on an item of the reliance statement marked as the practice's inference.
- *Rated on the scale alone.* — where neither applies.

A buyer reading the rating must be able to tell their own words reflected back from the practice's judgement.

**Words for the buyer.** `basis` is read by the buyer, and the buyer's document does not use the words *audit* or *auditor*. In `basis`, the engagement is *the review*, the party that performed it is *the practice*, and the independent second pass over the findings is *the check*. This document and the record use *audit* for the engagement and *review* for the second pass; that is their vocabulary, not the buyer's.

## 6. What this stage does not do

- It does not value the target or state a price.
- It does not advise the buyer.
- It does not change a verdict, add a finding, or rate a claim the audit did not make.
- It does not rank findings beyond the three values of §3.
- It does not judge whether the audit was performed well.
- It does not treat an unsettled claim as a gap: `exposure` is never counted as `materiality`.

## 7. The output

Your answer is one JSON object. Its shape is enforced; this document says what makes a field correct.

You may be asked for the whole set of findings or for one batch. A batch holds findings of one kind: those rated for `materiality`, or those rated for `exposure`. Answer with the findings you were asked for, in the array named for them, and leave the other array empty. The client's process assembles the batches.

| Field | Contents |
|---|---|
| `ratings[]` | One per finding you were given that is rated for `materiality` |
| `ratings[].claim_source` | The finding's claim source, as given |
| `ratings[].claim_id` | The finding's claim id, as given |
| `ratings[].materiality` | One value from §3 |
| `ratings[].basis` | Per §5 |
| `exposures[]` | One per finding you were given that is rated for `exposure` |
| `exposures[].claim_source` | The finding's claim source, as given |
| `exposures[].claim_id` | The finding's claim id, as given |
| `exposures[].exposure` | One value from §3 |
| `exposures[].basis` | Per §5 |

Emit nothing outside the JSON object.
