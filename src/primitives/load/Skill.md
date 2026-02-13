---
name: load
type: primitive
description: Load persistent Note or Collection by ID or name, with optional slice
---

# Load

## INPUT CONTRACT

- `target`: Note/Collection ID (e.g., `Note_123`), name (e.g., `"my-note"`), or variable
- `out`: Variable name
- `slice` (optional): Python-style slice string controlling how much content to return

**REQUIREMENTS:**
- Resource MUST exist (persisted or named)
- Can load by ID (`Note_123`) or name (`"my-note"`)

**NOT SUPPORTED:**
- ❌ Loading non-existent resources
- ❌ Loading from Collection (load individual Notes/Collections only)

## SLICE PARAMETER

| Target type | Units | Default | Ceiling |
|---|---|---|---|
| Note | characters | `"0:4096"` | 4096 chars |
| Collection | items | `"0:5"` | 16 items |

Syntax follows Python slice notation: `"start:stop"`, `":stop"`, `"start:"`, `":"` (all up to ceiling).

Examples:
- `slice: ":"` — full content up to ceiling
- `slice: "0:1000"` — first 1000 chars of a Note
- `slice: "-500:"` — last 500 chars of a Note
- `slice: "0:10"` — first 10 items of a Collection

## OUTPUT

**Notes:** Returns `"Note Content: <sliced text>"` with character count metadata.

**Collections:** The `out` binding is a **new Collection** containing the sliced items. The planner-visible value is a content preview showing each item's Note ID and first 200 chars.

## FAILURE SEMANTICS

**Returns `failed` when:**
- Resource not found
- Invalid resource ID/name
- Missing parameters

**Empty content ≠ error** — null Notes return content, not failure.

## REPRESENTATION INVARIANTS

- `load` returns Note/Collection content, not the resource itself
- Prefix distinguishes content from domain-specific output
- For Collections, `out` binds a new Collection (real resource), value string is a preview
- search-web/semantic-scholar return Collections directly — NO load needed

## ANTI-PATTERNS

❌ `map(load)` on search-web results → Results already materialized Notes
❌ `load(target=$collection)` → Load individual Notes/Collections, not from Collection
❌ Expecting domain output → Returns prefixed Note/Collection content
❌ Omitting `slice` when you need full content → Use `slice: ":"` to get up to ceiling
