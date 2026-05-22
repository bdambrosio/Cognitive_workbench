---
name: text-find
description: Find a substring or pattern in a text document. Returns position with surrounding context. Deterministic — use this instead of asking the LLM to find substrings.
args:
  source: required string — text to search (a `$stepN` binding or a literal)
  pattern: required string — substring or regex pattern to find
  context_lines: optional int (default 1) — lines of surrounding context per match
---

# text-find

Locate patterns or substrings in text and return all matches with context.

## Behavior

- Case-sensitive by default.
- Supports standard Python regex syntax in `pattern`.
- Returns every match (not just the first), with line/character position and the surrounding lines.
- Returns an empty result if no matches — not an error.

## When to use vs `assess` / `process_text`

- `text-find` — exact substring or regex match. Deterministic. No LLM cost.
- `assess` — natural-language predicate (semantic match). LLM-backed.
- `process_text` — open-ended transformation with an instruction.

If you want "does this contain the literal string X?" → `text-find`. If you want "does this contain mention of X?" (paraphrases count) → `assess`.

## Examples

```json
{"thought": "find every TODO in the document", "tool": "text-find", "source": "$step1", "pattern": "TODO"}
```

```json
{"thought": "locate error lines mentioning a timeout", "tool": "text-find", "source": "$step1", "pattern": "ERROR.*timeout", "context_lines": 3}
```
