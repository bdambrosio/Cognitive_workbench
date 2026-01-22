---
name: filter-structured
type: primitive
description: Filter Collection by field conditions (SQL WHERE)
---

# Filter-Structured

## INPUT CONTRACT

- `target`: Collection (variable or ID)
- `where`: SQL-like predicate string (e.g., `"year > 2020"`, `"score >= 0.5 AND year < 2025"`)
- `out`: Variable name

**REQUIREMENTS:**
- Collection MUST contain dict/JSON Notes
- WHERE clause uses field names (dot notation supported)
- Operators: `>`, `<`, `>=`, `<=`, `==`, `!=`, `AND`, `OR`

**NOT SUPPORTED:**
- ❌ Note (must be Collection)
- ❌ Collection of arrays (must be dict Notes)
- ❌ Text parsing (use `filter-semantic` tool for semantic filtering)

## OUTPUT

Returns Collection of Notes matching WHERE clause. Notes missing fields are excluded.

## FAILURE SEMANTICS

**Empty Collection = expected when:**
- No Notes match predicate
- Type contract violated

**Empty ≠ error** — indicates no matches, not failure.

**Actual failures:** Invalid target type, malformed WHERE clause, or missing parameters.

## REPRESENTATION INVARIANTS

- Note containing JSON array ≠ Collection
- Use `split` to convert array → Collection before filtering

## ANTI-PATTERNS

❌ `filter-structured(target=$note)` → Must be Collection
❌ `filter-structured(target=$coll_of_arrays)` → Elements must be dicts
❌ `filter-structured(target=$text_note)` → Use `filter-semantic` for semantic filtering
❌ Treating empty result as error → Empty = no matches
