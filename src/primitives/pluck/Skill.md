---
name: pluck
type: primitive
description: Extract single field value from each Note in Collection
---

# Pluck

## INPUT CONTRACT

- `target`: Collection (variable or ID)
- `field`: String path (supports dot notation like `metadata.uri`)
- `out`: Variable name

**REQUIREMENTS:**
- Collection MUST contain Notes (not Collections)
- Each Note MUST be dict/JSON object
- Field MUST exist as key in each Note

**NOT SUPPORTED:**
- ❌ Note containing array (use `split` first)
- ❌ Collection of arrays (must be dict Notes)

## OUTPUT

Returns Collection of Notes, each containing extracted scalar value. Notes missing field are excluded.

## FAILURE SEMANTICS

**Empty Collection = expected when:**
- Field missing in all Notes
- Type contract violated

**Empty ≠ error** — indicates no matches, not failure.

**Actual failures:** Invalid target type or missing parameters.

## REPRESENTATION INVARIANTS

- Note containing JSON array ≠ Collection
- Use `split` to convert array → Collection
- `flatten` performs inverse (Collection → Note)

## ANTI-PATTERNS

❌ `pluck(target=$array_note)` → Use `split` first
❌ `pluck(target=$coll_of_arrays)` → Elements must be dicts
❌ Treating empty result as error → Empty = no matches
