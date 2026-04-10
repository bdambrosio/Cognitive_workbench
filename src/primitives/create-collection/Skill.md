---
name: create-collection
type: primitive
description: Create a Collection object and bind to variable
---

# Create-Collection

## INPUT CONTRACT

- `value` (required): One of:
  - Array of `$variables` (e.g. `["$note1", "$note2"]`)
  - Array of raw resource IDs (e.g. `["Note_42", "Note_43"]`)
  - A single `$variable` (e.g. `"$note"`)
  - Empty array `[]` (creates an empty Collection)
- `out` (required): `$variable` name for resulting Collection
- `name` (optional): Stable name for the Collection, enabling later `load` by name

**REQUIREMENTS:**
- `value` must be provided (can be empty array)
- `out` must be a `$variable`
- All elements of `value` (whether `$variables` or raw IDs) must resolve to existing Notes

**IMPORTANT:** The `name` parameter registers a persistent name for later retrieval via `load(target="my-name")`. Variable names like `$my_collection` in `out` are temporary bindings during plan execution — not persistent names.

**Code-block usage tip:** When filtering an existing Collection in a code block, you typically end up with a list of raw Note IDs from `get_items("$source_collection")`. Pass that list directly as `value` — there is no need to round-trip each ID through a `$variable`. Example:

```python
items = get_items("$titles_collection")
filtered = [tid for tid in items if not get_text(tid).startswith("[")]
tool("create-collection", value=filtered, out="$filtered_titles")
```

## EXAMPLES

```json
{"type": "create-collection", "value": ["$note1", "$note2"], "out": "$my_collection"}
```
```json
{"type": "create-collection", "value": ["Note_42", "Note_43"], "out": "$by_id"}
```
```json
{"type": "create-collection", "name": "research", "value": [], "out": "$papers"}
```
```json
{"type": "create-collection", "value": "$note", "out": "$single_item"}
```

## OUTPUT

Returns `success` with the created Collection's resource ID. The Collection is immediately available via the `$variable` bound in `out`.

## FAILURE SEMANTICS

- Missing `value` or `out` parameter
- Referenced `$variables` in the array do not exist

## ANTI-PATTERNS

- Confusing `name` (persistent identifier) with `out` (temporary binding)
- Using `create-collection` to inspect or read existing Collections — use `load` instead
