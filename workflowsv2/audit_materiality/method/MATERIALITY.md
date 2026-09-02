# Materiality — method

## 1. Purpose

For each finding of a finished claims audit, say whether it changes what a buyer would assume if every claim in the claim sources held.

> Rate each finding by what it would change: nothing, the price or the terms, or the decision to close.

You did not perform the audit and you do not revise it. The findings are settled. Your subject is their consequence for the transaction as the engagement states it.

## 2. What you have

**The findings** — every finding of every claim source in the engagement, merged. Each carries the frozen claim (`quote`, `lines`, `statement`), its `adjudication` (`verdict`, and `gap` or `unresolved_because`), its `evidence`, the review's outcome (`holds`, `does_not_hold` with the observations that failed, or `unreviewed`), and any citation problem the client's process recorded.

**The transaction** — what the engagement states about the buyer, the purpose of the acquisition, the price basis and the intended structure. Where the engagement states none of this, rate against a buyer paying a price that assumes every claim holds.

You have no tools and no access to the target. Rate on what the finding shows.

## 3. The scale

Every rated finding carries exactly one.

| `materiality` | Meaning |
|---|---|
| `not_material` | The finding would change neither the price, nor the terms of the transaction, nor the decision to close |
| `material` | The finding would change the price or the terms, and a buyer would still close |
| `decisive` | The finding on its own would change the decision to close |

**Terms** means the structure of the transaction: conditions, warranties, escrow, earn-out, and what is included.

## 4. What is rated, and how

Every finding whose verdict is not `real` is rated. A `real` finding changes nothing a buyer assumed and is not rated.

- `contradicted` and `partial` — rate what the `gap` and the evidence show.
- `real_with_caveat` — rate the caveat: what a buyer must know to read the claim correctly.
- `unverifiable` — rate as if the claim were false. The rating is the exposure, and `basis` says so.

A finding whose review outcome is `does_not_hold`, or which carries a citation problem, is rated the same way. The client's process reports the rating beside that outcome.

Rate each finding on its own. Do not combine findings, and do not rate a finding by its effect on another.

## 5. `basis`

One to three sentences: what in the finding's `gap` or evidence drives the rating, and how it bears on the transaction as stated. Quote the figure or the fact from the finding. Do not restate the claim.

## 6. What this stage does not do

- It does not value the target or state a price.
- It does not advise the buyer.
- It does not change a verdict, add a finding, or rate a claim the audit did not make.
- It does not rank findings beyond the three values of §3.
- It does not judge whether the audit was performed well.

## 7. The output

Your answer is one JSON object. Its shape is enforced; this document says what makes a field correct.

You may be asked for the whole set of findings or for one batch. Answer with the findings you were asked for and nothing else. The client's process assembles the batches.

| Field | Contents |
|---|---|
| `ratings[]` | One per finding you were given |
| `ratings[].claim_source` | The finding's claim source, as given |
| `ratings[].claim_id` | The finding's claim id, as given |
| `ratings[].materiality` | One value from §3 |
| `ratings[].basis` | Per §5 |

Emit nothing outside the JSON object.
