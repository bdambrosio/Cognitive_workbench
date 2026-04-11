---
name: persist
type: primitive
description: Mark a Note or Collection as persistent so it survives goal-completion cleanup
---

# Persist

## INPUT CONTRACT

- `target` (required): `$variable`, resource ID (`Note_123`, `Collection_456`, `Relation_7`), or named resource
- `name` (optional): Stable name to register for later `load(target="my-name")`
- `out` (optional): `$variable` aliased to the persisted resource

**REQUIREMENTS:**
- Target must resolve to an existing Note, Collection, or Relation

## BEHAVIOR

Marks the resource as persistent on disk so it is not swept by the transient-resource cleanup that runs after goal completion. Persistence is in-place — the same resource ID before and after. If `name` is given, the resource is also registered for load-by-name. If `out` is given, the variable is aliased to the same resource ID (equivalent to `bind` after `persist`).

Use persist for any output a subsequent goal will need to read.

## EXAMPLES

```json
{"type": "persist", "target": "$matches", "name": "transformer_attention_matches"}
```
```json
{"type": "persist", "target": "Collection_17", "name": "weather_report", "out": "$report"}
```

## OUTPUT

Returns `success` with the persisted resource's ID in `resource_id`.

## FAILURE SEMANTICS

- Missing `target`
- Target does not resolve to a Note, Collection, or Relation
- Underlying `mark_persistent` failed (e.g. disk write error)

## ANTI-PATTERNS

- Persisting transient scratch resources — only persist deliverables.
- Assuming `persist` copies or clones — it flips a flag on the existing resource. Both pre- and post-persist variables point to the same ID.
- Passing `out=` expecting a new resource. The binding points to the same resource; use `bind` explicitly if that is clearer at the call site.
