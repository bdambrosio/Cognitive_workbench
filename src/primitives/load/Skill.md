---
name: load
type: primitive
description: Load persistent Note or Collection by ID or name
---

# Load

## INPUT CONTRACT

- `target`: Note/Collection ID (e.g., `Note_123`), name (e.g., `"my-note"`), or variable
- `out`: Variable name

**REQUIREMENTS:**
- Resource MUST exist (persisted or named)
- Can load by ID (`Note_123`) or name (`"my-note"`)

**NOT SUPPORTED:**
- ❌ Loading non-existent resources
- ❌ Loading from Collection (load individual Notes/Collections only)

## OUTPUT

Returns prefixed content: `"Note Content: <text>"` or `"Collection Content: <ids>"` (truncated to 1024 chars). Prefix clarifies result is Note/Collection content, not domain output.

## FAILURE SEMANTICS

**Returns `failed` when:**
- Resource not found
- Invalid resource ID/name
- Missing parameters

**Empty content ≠ error** — null Notes return content, not failure.

## REPRESENTATION INVARIANTS

- `load` returns Note/Collection content, not the resource itself
- Prefix distinguishes content from domain-specific output
- search-web/semantic-scholar return Collections directly — NO load needed

## ANTI-PATTERNS

❌ `map(load)` on search-web results → Results already materialized Notes
❌ `load(target=$collection)` → Load individual Notes/Collections, not from Collection
❌ Expecting domain output → Returns prefixed Note/Collection content
