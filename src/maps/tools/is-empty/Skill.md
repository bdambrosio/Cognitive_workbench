---
name: is-empty
description: Check if text is null, empty, or only whitespace
type: prompt_augmentation
parameters: none
---

# Is Empty

Check whether a note contains meaningful content or is effectively empty.

## Purpose

- Validate note content exists
- Enable conditional logic in plans
- Filter out empty results
- Guard against null/missing data

## Input Format

Accepts a single Note.

## Parameters

None.

## Output Format

Returns a Note containing:
- "true" if note is null, empty string, or only whitespace
- "false" if note contains any non-whitespace content

## Usage Examples

**Basic validation:**
```json
{"type":"is-empty","target":"$result","out":"is_empty"}
```

**Used in conditional:**
```json
{"type":"is-empty","target":"$search_results","out":"no_results"}
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

