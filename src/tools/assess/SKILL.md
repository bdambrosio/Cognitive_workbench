---
name: assess
description: Boolean test of text content against a natural language predicate. Features auto-chunking for long texts (returns "true" if ANY chunk matches).
type: python
flattens_collections: true
---

# Test Note

Evaluates natural language predicates against text content using an LLM.

## Purpose
Semantic boolean testing. Handles large documents automatically via chunking.
Returns "true" if any part of the content meets the criteria.

## Input
- `predicate`: Natural language question (e.g., "mentions specific dates?", "is critical of the author?").
- `target`: The string content to test. *Empty inputs return "false".*

## Output
Returns string `"true"` or `"false"`.
Note: Output is a lowercase string, not a JSON boolean.

## Behavior & Performance
1. Auto-Chunking: Texts >16k characters are split into boundary-aware chunks.
2. Aggregation (OR Logic): Evaluates chunks sequentially. Returns `"true"` immediately upon the **first** chunk that satisfies the predicate (short-circuit). Returns `"false"` only if *all* chunks fail.
3. Latency: Fast for short texts. Increases linearly with length for long texts until a match is found.

## Guidelines
- Predicate Phrasing: Since the tool checks chunks in isolation, phrase predicates to detect *presence* rather than *global summary*.
    - Good: "Contains mention of inflation?"
    - Risky: "Is the main topic inflation?" (Might be a topic in one smaller chunk of input).
- Cost: Every chunk requires an LLM call.
- Fallbacks: Returns `"false"` on ambiguous LLM responses.

## Usage Examples

Example:
```json
// Because flattens_collections=true, passing a list to target returns a list of "true"/"false" strings
{"type":"assess","target":"$my_note","predicate":"is urgent?","out":"$urgency_flags"}
```