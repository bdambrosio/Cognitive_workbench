---
name: assess
description: Boolean test of text content against a natural language predicate. Features auto-chunking for long texts (returns "true" if ANY chunk matches).
type: python
flattens_collections: true
---

# assess

Semantic boolean testing. Evaluates natural language predicates against text content using LLM.

## Input

- `target`: String content to test (empty inputs return "false")
- `predicate`: Natural language question (e.g., "mentions specific dates?", "is critical of the author?")

## Output

Returns string `"true"` or `"false"` (lowercase string, not JSON boolean).

## Behavior

- **Auto-Chunking**: Texts >16k chars are split into boundary-aware chunks
- **OR Aggregation**: Returns `"true"` on first matching chunk (short-circuit), `"false"` only if all chunks fail
- **Fallback**: Returns `"false"` on ambiguous LLM responses

## Planning Notes

- Phrase predicates to detect *presence* rather than global summary (chunks are evaluated in isolation)
  - Good: "Contains mention of inflation?"
  - Risky: "Is the main topic inflation?"
- Every chunk requires an LLM call

## Example

```json
{"type":"assess","target":"$my_note","predicate":"is urgent?","out":"$urgency"}
```
