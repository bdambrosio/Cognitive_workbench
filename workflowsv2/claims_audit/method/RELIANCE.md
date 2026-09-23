# Reliance statement — method

## 1. Purpose

Before the claims are rated, write down what this buyer relies on the offering for. The buyer said at intake what they are paying for and what would change the price, in business language. The claims are in technical language. The reliance statement connects the two: it says which functions, properties and commitments of the offering the buyer's plan depends on, which it uses without depending on, and which it does not use. Every later rating is read against this one account.

> Say what the buyer relies on the offering for, item by item, and what the buyer would have to do if each item failed.

A person reads the statement and corrects it before it is used. Write it so that the buyer could read it and answer "yes, that is our plan" or "no, we never use that".

## 2. What you have

**What the buyer said** — the intake form where the engagement has one: who the buyer is, the transaction, what they will do with the offering, what they know, what they are paying for, what would change the price or end the deal, the scope, and the practice's notes of the conversation. Where there is no intake form, the engagement's statement of the transaction and of the buyer's thresholds.

**The claims** — every claim enumerated from the seller's documents, by claim source, as statements. Use them as an inventory of what the offering says it is and does. You are not asked whether any claim is true, and nothing you write may say or suggest that one is or is not.

You have no tools and no access to the offering.

## 3. What to write

**`use`** — one paragraph: what the buyer will do with the offering, who will operate it, and what the buyer's own product or plan needs from it. Use the buyer's words where they exist. The paragraph describes the buyer's plan and nothing else; it does not say how the statement will be used.

**`items[]`** — between 10 and 30 items. An item is a function, a component, a property or a commitment of the offering, named as a buyer would name it: "creating and expiring links through the API", "memory used by each instance", "the Helm chart". An item is not a single claim; many claims fall under one item. Cover the whole inventory: every claim should fall under some item.

For each item:

| Field | Contents |
|---|---|
| `item` | The name of the function, component, property or commitment |
| `reliance` | `depends`: the buyer's plan does not work, or the price is not justified, without it. `uses`: the buyer will use it, and would work around its failure at a cost small beside the price. `does_not_use`: the buyer's plan does not touch it |
| `if_it_failed` | One sentence: what the buyer would have to do if this item did not work as the seller's documents describe. Where the buyer could repair or replace it themselves, say so, and say whether that is cheap or costly for this buyer, given who they said will operate the offering. Use what is generally known about this kind of software: which parts are its core, and which are commodity that a competent engineer replaces in a day. Do not estimate amounts of money |
| `source` | `buyer` only where the buyer's quoted words themselves say what the item says. Where the item goes further than the words — the buyer named tracking, and the item also covers donation messages — the part the words say is one item with `source` `buyer`, and each part beyond them is a separate item with `source` `inference`. `inference` where you reasoned from the buyer's plan and from what is generally known about this kind of software |
| `buyer_words` | For `buyer`: the buyer's words that say what the item says, quoted exactly. For `inference`: the words of the buyer's plan you reasoned from, quoted exactly. Quote only words that bear on this item; where none do, leave the field empty |

## 4. Rules

- **Do not invent facts about the buyer.** An inference says what follows from the plan the buyer stated. It does not add staff, advisers, systems, customers or intentions the buyer did not mention.
- **Where the buyer said they do not use something, or do not care about it, the item is `does_not_use` and `source` is `buyer`.**
- **Where the buyer said nothing about an item and their plan does not decide it, choose `uses`, and end `if_it_failed` with: "The buyer has not said; a person should ask."**
- **A broad statement by the buyer is recorded as broad.** Where the buyer said that everything of a kind matters — every documented setting, every feature — make one item for it with the buyer's words, and separate items only for parts the buyer or their plan singles out.
- **Do not rate, rank or count claims.** That is the next stage's work.
- **Words for the buyer.** The engagement is *the review*, and the party performing it is *the practice*. Do not write *audit* or *auditor*.

## 5. The output

One JSON object with `use` and `items[]` as §3 defines them. Its shape is enforced; this document says what makes a field correct. Emit nothing outside the JSON object.
