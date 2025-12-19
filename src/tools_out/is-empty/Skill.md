---
name: is-empty
description: Check if text is null, empty, or only whitespace
type: python
schema_hint:
  target: "$variable or Note ID"
  out: "$variable (optional)"
parameters: none
examples:
  - '{"type":"if","condition":{"type":"tool_condition","tool":"is-empty","target":"$text"},"then":[...]}'
---

# Is Empty

Check whether text contains meaningful content or is effectively empty.

## Purpose

- Validate note content exists
- Enable conditional logic in plans
- Filter out empty results
- Guard against null/missing data

## Input Format

Accepts:
- Plain text string
- Note content

## Parameters

None.

## Output Format

Returns boolean:
- `True` if text is null, empty string, or only whitespace
- `False` if text contains any non-whitespace content

## Usage Examples

**Basic validation:**
```json
{"type":"is-empty","target":"$result","out":"$is_empty"}
```

**Used in conditional:**
```json
{"type":"is-empty","target":"$search_results","out":"$no_results"}
```

## Guidelines

- Treats null, empty string, whitespace-only as empty
- Newlines, tabs, spaces count as whitespace
- Any visible character makes content non-empty

## Examples

**Empty cases:**
```
null → true
"" → true
"   " → true
"\n\t  \n" → true
```

**Non-empty cases:**
```
"Hello" → false
" x " → false
"0" → false
```

