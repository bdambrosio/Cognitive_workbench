---
name: create-collection
type: primitive
description: Create a Collection object and bind to variable
---

# Create-Collection

## INPUT CONTRACT

- `value` (required): Array of `$variables`, a single `$variable`, or empty array `[]`
- `out` (required): `$variable` name for resulting Collection
- `name` (optional): Stable name for the Collection, enabling later `load` by name

**REQUIREMENTS:**
- `value` must be provided (can be empty array)
- `out` must be a `$variable`

**IMPORTANT:** The `name` parameter registers a persistent name for later retrieval via `load(target="my-name")`. Variable names like `$my_collection` in `out` are temporary bindings during plan execution — not persistent names.

## EXAMPLES

```json
{"type": "create-collection", "value": ["$note1", "$note2"], "out": "$my_collection"}
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
