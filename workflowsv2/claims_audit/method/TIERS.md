# Tiers — method

## 1. Purpose

Before any claim is tested, say what it would change for this buyer if it turned out to be false. The answer places the claim in a tier, and the tier decides how the buyer's time and the practice's are spent: a claim whose falsity would move the price is tested; a claim whose falsity would change nothing is listed, with your reason, and tested only if the buyer asks.

> Rate each claim by what its being false would change: nothing, the price or the terms, or the decision to close.

This is the question the materiality stage asks of a finding after the review, asked here of a claim before it. The scale is the same (MATERIALITY.md §3) and the thresholds are the same. What differs is that you have no finding: no verdict, no evidence, nothing about whether the claim is true. You rate what rests on the claim, not whether it holds.

You propose. A person sees every claim with its tier and your reason on the surface page before the freeze and changes any tier they disagree with; a tier left alone stands. You may be asked to rate the same claims more than once, independently; where the readings differ, the claim is marked for the person to decide.

## 2. What you have

**The claims** — a batch of claims from one claim source, each with its `id`, its `quote` as the source states it, its `statement`, and `about` (`target`, `seller` or `document`). A claim found to repeat an earlier one is not given to you. A claim found to be within a wider one is given, and is rated on its own: a narrow claim can carry the figure that matters.

**The transaction** — what the engagement states about the buyer, the purpose of the acquisition, the price basis and the intended structure. Where the engagement states none of this, rate against a buyer paying a price that assumes every claim holds.

**The buyer's thresholds** — what the engagement records of the buyer's own view: what they are paying for, what would change the price or the terms, and what would make them walk away. Where the engagement records them, a tier is read against them. Where it records none, rate as §3 defines the tiers.

You have no tools, no access to the target and no evidence. You are not asked whether a claim is true, and nothing you write may say or suggest that it is or is not.

## 3. The tiers

Every claim gets exactly one tier.

| `tier` | Meaning |
|---|---|
| `1` | If the claim were false, the price or the terms would change, or the buyer would not close. On the materiality scale: `material` or `decisive` |
| `2` | On its own, nothing would change if the claim were false; but it is one of the details a buyer who operates the software from its documentation relies on, and with others of its kind its falsity would change whether that documentation can be relied on. A documented default, a parameter's range or format, a field name, a file's location, a command's option |
| `3` | Nothing would change if the claim were false, on its own or with others: `not_material` |

**Terms** means the structure of the transaction: conditions, warranties, escrow, earn-out, and what is included.

## 4. How to decide

- **Assume the claim is false and ask what the buyer loses.** A claim of no tracking, for a buyer who named privacy as a threshold, is tier 1: its falsity is what the buyer said would change the price. A claim that the container image is under 5 MB, for the same buyer, is tier 1 only if a stated threshold reaches footprint; otherwise it is tier 3 unless the transaction says the buyer pays for a small footprint.
- **Read the thresholds as the buyer wrote them, and no wider.** A threshold about "any way visitors are logged or tracked" reaches a claim about what is logged; it does not reach a claim about which database file the logs go to.
- **Tier 2 is for detail a reader of the documentation would act on.** The test: would a person setting the software up, or writing against its API, do something differently if this claim were false? A default value, a supported range, a field name, a route, a configuration key. It is not for claims about what the software is or does at all, which are tier 1 or tier 3 on their own weight.
- **A claim about the seller** (`about` = `seller`: a promise to maintain, a hosted service, a commercial term) is rated the same way: what the buyer loses if the promise is not kept.
- **A claim about a document** (`about` = `document`) is tier 3 unless the document is the fact the buyer relies on: the licence, the notice of ownership.
- **A boundary claim** — "only", "no", "never", "all" — is rated on what its failure would admit. "No end-user information is ever logged" failing admits logging of personal data; that is tier 1 for any buyer whose thresholds name privacy, and tier 1 for a buyer who stated none, because a price that assumes every claim holds assumes it.
- **Rate each claim on its own.** Do not combine claims, and do not rate a claim by its effect on another. Two claims that read alike can differ in tier by their figure or their scope.
- **When in doubt between 1 and a lower tier, choose 1.** A tier-1 claim that did not need testing costs the review one claim's work. A tier-3 claim that did need testing is a claim nobody tests.

## 5. `basis`

One or two sentences: what the buyer would lose if the claim were false, and how that bears on the transaction and the thresholds as stated. Do not restate the claim, and do not say whether it is likely to be true.

**Say what the tier rests on.** Where the engagement records thresholds, the last sentence says one of two things, in these words: *rated against the buyer's stated threshold*, quoting the threshold it rests on; or *rated on the scale alone*, where no stated threshold reaches the claim. The person reading the tier must be able to tell the buyer's own words from the practice's judgement.

**Words for the buyer.** `basis` is read on the surface page and in the report. The engagement is *the review*, the party performing it is *the practice*. Do not write *audit* or *auditor*.

## 6. What this stage does not do

- It does not judge whether a claim is true, likely, or well supported.
- It does not remove a claim. A tier-3 claim stays on the surface and in the report.
- It does not value the target or state a price, and it does not advise the buyer.
- It does not rank claims beyond the three tiers.
- It does not change a claim's wording, its `about`, or any mark on it.

## 7. The output

One JSON object. Its shape is enforced; this document says what makes a field correct.

| Field | Contents |
|---|---|
| `tiers[]` | One entry for every claim you were given |
| `tiers[].claim_id` | The `id` of the claim, as given to you |
| `tiers[].tier` | `1`, `2` or `3`, per §3 |
| `tiers[].basis` | Per §5 |

Emit nothing outside the JSON object.
