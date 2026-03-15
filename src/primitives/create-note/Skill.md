---
name: create-note
type: primitive
description: Create a persistent Note object and bind to variable
---

# Create-Note

## INPUT CONTRACT

- `value` (required): Literal value (string, number, boolean, array, object) or `$variable` referencing content
- `out` (required): `$variable` name for resulting Note
- `name` (optional): Stable name for the Note, enabling later `load` by name

**REQUIREMENTS:**
- `value` must be provided (can be any type)
- `out` must be a `$variable`

**IMPORTANT:** The `name` parameter registers a persistent name for later retrieval via `load(target="my-name")`. The `out` parameter (e.g., `$my_note`) is a temporary variable binding during plan execution — these are different concepts.

## VALUE TYPES

Pass values directly as their native types — do NOT pre-serialize with `json.dumps()`:

| Type | Example value |
|---|---|
| String | `"some text data"` |
| Number | `42` or `3.14` |
| Boolean | `true` or `false` |
| Array | `["apple", "banana", "cherry"]` |
| Object | `{"key": "value", "count": 5}` |
| Variable | `"$existing_var"` |

## EXAMPLES

```json
{"type": "create-note", "value": "some data", "out": "$my_note"}
```
```json
{"type": "create-note", "value": ["apple", "banana", "cherry"], "out": "$array_note"}
```
```json
{"type": "create-note", "value": {"key": "value"}, "out": "$object_note"}
```
```json
{"type": "create-note", "value": "$variable", "out": "$new_note"}
```
```json
{"type": "create-note", "value": "important data", "name": "important-note", "out": "$my_note"}
```

## OUTPUT

Returns `success` with the created Note's resource ID. The Note is immediately available via the `$variable` bound in `out`.

## FAILURE SEMANTICS

- Missing `value` or `out` parameter
- Invalid `out` variable syntax

## ANTI-PATTERNS

- Wrapping JSON in `json.dumps()` before passing as `value` — pass native types directly
- Confusing `name` (persistent identifier) with `out` (temporary binding)
