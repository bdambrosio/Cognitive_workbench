---
name: map
type: primitive
description: Apply operation to each item in Collection
---

# Map

## INPUT CONTRACT

- `target`: Collection (variable or ID)
- `operation`: Tool/primitive name (string) or dict with `tool` field
- `out`: Variable name
- Additional fields: Tool-specific parameters

**REQUIREMENTS:**
- `target` MUST be Collection
- `operation` MUST be valid tool/primitive name
- Operation applied to each Note in Collection

**NOT SUPPORTED IN MAP:**
- Collection-only primitives: `size`, `union`, `intersection`, `difference`, `join`, `filter-structured`, `sort`, `head`, `flatten`, `split`
- Control flow: `if`, `while`, `wait`
- Search primitives: `search-notes`, `search-collections`, `search-within-collection`
- Persistence: `persist`, `load`, `index`

## OUTPUT

Returns Collection of Notes, each containing result from applying operation. Failed/null results excluded if `filter_null=true` (default).

## FAILURE SEMANTICS

**Empty Collection = expected when:**
- All operations fail or return null
- Type contract violated

**Empty ≠ error** — indicates no successful results, not failure.

**Actual failures:** Invalid target type, unknown operation, or missing parameters.

## REPRESENTATION INVARIANTS

- `map(load)` on search-web results returns empty (results already materialized Notes)
- Empty result from `map(load)` = expected behavior, not diagnostic

## ANTI-PATTERNS

❌ `map(target=$note, operation="refine")` → Must be Collection
❌ `map(target=$results, operation="load")` → search-web results already Notes
❌ `map(target=$coll, operation="split")` → `split` operates on Notes, not Collections
❌ Treating empty result as error → Empty = no successful operations
