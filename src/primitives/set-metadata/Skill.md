---
name: set-metadata
type: primitive
description: Attach or update metadata on a Note or Collection
---

# Set-Metadata

## INPUT CONTRACT

- `target` (required): `$variable`, Note/Collection ID, or name
- `value` (required): Metadata text (string, may be JSON)
- `out` (optional): `$variable` — bound to the metadata Note ID

**REQUIREMENTS:**
- Target resource must exist
- Value must be provided

## BEHAVIOR

Associates a metadata text string with the target Note or Collection. Idempotent: updates existing metadata if already present. If `out` is provided, the variable is bound to the metadata Note ID.

## EXAMPLES

```json
{"type": "set-metadata", "target": "$report", "value": "{\"source\": \"weather.gov\", \"retrieved\": \"2026-02-21\"}"}
```
```json
{"type": "set-metadata", "target": "$report", "value": "$meta_text", "out": "$meta_note"}
```

## OUTPUT

Returns `success` with the metadata Note ID.

## FAILURE SEMANTICS

- Target resource not found
- Missing `target` or `value` parameter
