---
name: assess
description: Check whether text matches a natural-language condition. Returns "true" or "false". Auto-chunks long texts and short-circuits on first match.
args:
  source: required string — text to evaluate (a `$stepN` binding or a literal)
  predicate: required string — natural-language condition (e.g. "is critical of the author?", "mentions specific dates?")
---

# assess

Boolean test of text content against a natural-language predicate. Returns the literal lowercase string `"true"` or `"false"`.

## Behavior

- Texts longer than 16k characters are split into chunks at sentence boundaries.
- Returns `"true"` on the first matching chunk (short-circuit). Returns `"false"` only if all chunks fail.
- Returns `"false"` on ambiguous LLM responses (no half-credit).

## Planning notes

Phrase predicates to detect *presence* rather than global summary, because chunks are evaluated in isolation:

- Good: `"contains mention of inflation?"`
- Risky: `"is the main topic inflation?"` — a chunk that briefly mentions inflation might match even if it isn't the main topic of the whole document.

Every chunk requires an LLM call, so very long inputs are expensive.

## Example

```json
{"thought": "check if the article is critical of the proposal", "tool": "assess", "source": "$step1", "predicate": "is critical of the proposed policy?"}
```
