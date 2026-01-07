---
name: flatten
type: primitive
description: Flatten Collection to single Note
---

# Flatten

## INPUT CONTRACT

- `target`: Collection (variable or ID)
- `out`: Variable name

**REQUIREMENTS:**
- `target` MUST be Collection

**NOT SUPPORTED:**
- ❌ Note (use `split` to convert Note → Collection first)

## OUTPUT

Returns single Note containing merged content from all Collection items.

## FAILURE SEMANTICS

**Returns `failed` when:**
- Target is Note (not Collection)
- Missing parameters

**Empty Note ≠ error** — indicates empty Collection, not failure.

## REPRESENTATION INVARIANTS

- Collection ≠ Note containing array
- `flatten` performs inverse of `split` (Collection → Note)
- Use `split` to convert array → Collection

## ANTI-PATTERNS

❌ `flatten(target=$note)` → Must be Collection
❌ Expecting structured output → Returns merged content, structure may be lost
