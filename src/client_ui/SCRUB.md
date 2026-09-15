# Scrubbing the claim surface

The claim surface is the list of claims the review will test. Enumeration drafts it from the documents the client named; a person settles it; the review tests exactly what was frozen and nothing else. The scrub decides what counts as a claim. It does not decide whether a claim is true.

## Before you start

Read the claim source whole, then the client's comments beside each claim. A comment is a reason to look, not an instruction to change.

## For each claim

- **The quote is verbatim and at the lines given.** Do not correct the seller's spelling or grammar in the quote. If the enumeration mangled it, restore the seller's words.
- **The statement says what the quote says.** Not more, not less. A statement that asserts more than the quote poisons every finding downstream, because the finding tests the statement and the reader sees the quote.
- **One assertion per claim.** Where a sentence joins two assertions that could get different verdicts, split it into two claims on the same quote. Where two claims restate one assertion, keep one.
- **`about` is right.** `target` for what the software is or does. `seller` for the seller's business, pricing, delivery, market position or competitors. `document` for a claim about the document itself, such as its date or licence. A seller claim is exempt from the search rule, so the mark changes how it is tested.
- **Puffery is not a claim.** "The most rigorous", "best in class", "enterprise-grade" with nothing testable behind it: drop. A comparative with a testable part keeps the testable part.
- **Restatements across documents stay.** Each document is reviewed on its own run. A claim the one-pager restates from the technical report is tested twice, and a stronger restatement is tested as stated.

## Claims about behaviour

A statement that says what the software does when it runs is two claims in one: the mechanism the code provides, and the behaviour in operation. "The assistant answers common customer questions on its own" asserts that a question is sent to a model and its answer returned, and that the answers resolve common questions. The code settles the first. Nothing in the code settles the second.

Enumeration splits these itself. The claim's statement is rewritten to the mechanism, and a second claim marked "implied by" the first, with the property "behaviour in operation", carries the behaviour. Both rows stay unless you drop them. Read each split: drop the behaviour row where it only repeats the mechanism in other words, and correct a statement that says more or less than the quote. Where enumeration left a behaviour claim whole, use the decompose button and edit the proposed statements into the mechanism and the behaviour. Expect the behaviour row to come back unverifiable: that is the correct outcome, it is rated for exposure, and it becomes a question to the seller or a request for records. Leaving the sentence whole gets a verdict on the mechanism that reads as a verdict on the behaviour.

Words that mark a behaviour claim: answers, understands, learns, handles, detects, in real time, under normal use, at scale, accurately, reliably, automatically. The list is a prompt to look, not a rule.

## Measured claims

A figure with a condition, such as memory under regular use, image size, response time, or a rate of anything, stays as a claim with the figure and the condition in the statement. Expect unverifiable from code alone. The seller can supply the measurement through the materials page.

## Claims of absence

"No tracking", "only these logs", "never stores". Keep them. The review settles them by searching the whole tree in two ways, so the statement must name precisely what is said to be absent. "No analytics" and "no cookies" are two claims.

## Implied claims

Where the ordinary sense of a claim implies a property the seller did not state, add it marked as implied, with the parent named. Only where a reasonable buyer would read the words that way. Do not add a checklist of properties the category usually has.

## What not to do

- Do not add a claim from your own knowledge of the code or the seller.
- Do not drop a claim because you think it is false. The review says whether it holds.
- Do not soften or sharpen the seller's words in the statement.
- Do not rate. Materiality is a later stage with the thresholds in view.

## Freezing

Save the draft as you go. When a source is settled, freeze it. When every source is frozen the client is told and the review can run. If the surface must change before a rerun, unfreeze it: the frozen file is archived, the claims become the draft again, and you freeze once more when done.
