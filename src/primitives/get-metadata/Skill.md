---
name: get-metadata
type: primitive
description: Retrieve metadata attached to a Note or Collection
---

# Get-Metadata

## INPUT CONTRACT

- `target` (required): `$variable`, Note/Collection ID, or name
- `out` (optional): `$variable` — bound to the metadata Note ID if metadata exists

**REQUIREMENTS:**
- Target resource must exist

## BEHAVIOR

Returns the metadata text associated with the target Note or Collection. Metadata is stored transparently by the system (e.g., search-web attaches source URLs as metadata).

- If metadata exists: returns the metadata content as text
- If no metadata exists: returns empty string
- If `out` is provided: binds the metadata Note ID to the variable

## EXAMPLES

```json
{"type": "get-metadata", "target": "$search_result", "out": "$meta"}
```
```json
{"type": "get-metadata", "target": "Note_123", "out": "$note_meta"}
```

## OUTPUT

Returns `success` with metadata text as value. Use `get_text("$meta")` in code blocks to read the content.

## FAILURE SEMANTICS

- Target resource not found
- Missing `target` parameter
