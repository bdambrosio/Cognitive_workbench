---
name: project
type: primitive
description: Extract metadata/structured fields from each Note in Collection (SQL SELECT)
---

# Project

## INPUT CONTRACT

- `target`: Collection (variable or ID)
- `fields`: List of field paths (strings, supports dot notation like `metadata.uri`)
- `out`: Variable name

**REQUIREMENTS:**
- Collection MUST contain Notes (not Collections)
- Each Note MUST be dict/JSON object
- Fields MUST exist as keys in each Note (missing fields cause Note to be excluded)

**NOT SUPPORTED:**
- ❌ Note (must be Collection)
- ❌ Collection of arrays (must be dict Notes)
- ❌ Text parsing (use `refine` tool for LLM-based extraction from text)

## OUTPUT

Returns Collection of Notes, each containing only the requested fields. Notes missing any requested field are excluded.

## CONTENT STRUCTURE

**For JSON Notes, content is a dict with fields:**
- Top-level fields: `text`, `format`, `char_count`
- Nested fields: `metadata.*` (e.g., `metadata.uri`, `metadata.title`, `metadata.year`)

**Example Note content structure (from semantic-scholar/search-web):**
```json
{
  "text": "Full text content...",
  "format": "paper",
  "metadata": {
    "title": "Paper Title",
    "authors": ["Author 1", "Author 2"],
    "year": 2023,
    "uri": "https://example.com/paper.pdf",
    "score": 0.95
  },
  "char_count": 5000
}
```

## FIELD ACCESS EXAMPLES

**Extract single field:**
```json
{"type":"project","target":"$papers","fields":["metadata.title"],"out":"$titles"}
```

**Extract multiple fields:**
```json
{"type":"project","target":"$papers","fields":["metadata.title","metadata.year"],"out":"$paper_info"}
```

**Extract nested metadata fields:**
```json
{"type":"project","target":"$search_results","fields":["metadata.uri","metadata.score"],"out":"$urls"}
```

**Extract top-level and nested fields:**
```json
{"type":"project","target":"$results","fields":["text","metadata.uri","char_count"],"out":"$filtered"}
```

## FAILURE SEMANTICS

**Empty Collection = expected when:**
- No Notes have all requested fields
- Type contract violated (non-dict Notes)

**Empty ≠ error** — indicates no matches, not failure.

**Actual failures:** Invalid target type, missing parameters, or malformed fields list.

## REPRESENTATION INVARIANTS

- Note containing JSON array ≠ Collection
- Use `split` to convert array → Collection before projecting
- Projected Notes preserve nested structure (e.g., `metadata.uri` stays as `metadata.uri`)

## ANTI-PATTERNS

❌ `project(target=$note)` → Must be Collection
❌ `project(target=$coll_of_arrays)` → Elements must be dicts
❌ `project(target=$results, fields=["extract the author"])` → Use `refine` for text extraction
❌ Treating empty result as error → Empty = no matches

## USE CASES

- Extract `metadata.uri` from search results for `fetch-text`
- Extract `metadata.title` and `metadata.year` from papers for filtering
- Extract `metadata.source_id` and `metadata.score` from search results for analysis
- Project specific fields before `join` operations
