---
name: extract
type: python
flattens_collections: false
description: "Derive content from a single Note via LLM-guided extraction, compression, or transformation. Output is grounded entirely in the input — no new information is introduced."
schema_hint:
  target: "$variable (single Note, required)"
  instruction: "string (what to extract or how to transform, required)"
  out: "$variable (optional)"
---

# extract

LLM-guided extraction or transformation from a single Note. Output is grounded
entirely in the input — no new information introduced.

## Input

- `target`: Single Note (variable, ID, or name) — NOT Collections
- `instruction`: What to extract or how to transform (required)
- `target_tokens`: Optional output length in tokens
- `out`: Variable name for resulting Note

## Planning Notes

Prefer Python for small, predictable structure (YAML frontmatter, key:value
blocks). Use `extract` when layout is unknown or you need semantic judgment.

**Output length is unreliable.** Word-count instructions ("output a 1-3 word
phrase") are frequently ignored — expect verbose output. When output will be
used as a filename, directory name, or identifier, **always truncate and
sanitize in Python**:
```python
tokens = re.split(r'[\s_]+', raw)[:3]  # split on spaces AND underscores
name = re.sub(r'[^a-z0-9_]', '', '_'.join(tokens).lower())
```

**Do NOT use when:**
- Bibliography/references → `extract-references` (GROBID, structured, deterministic)
- Multiple documents → `synthesize`
- New content from scratch → `generate-note`
- Filtering Collection items → `filter-structured` or `filter-semantic`
- Structured metadata fields → `project` or `pluck`

**Prefer over `synthesize` when** there is only ONE source document.

## Anti-Patterns

- ❌ `extract(target=$collection)` — Use `map(extract)` for Collections.
- ❌ `extract(target=$paper, instruction="extract citations")` — Use `extract-references`.
- ❌ Raw extract output as directory/file name — Always truncate + sanitize in Python first.

## Examples

```json
{"type":"extract","target":"$paper","instruction":"Extract the key architectural innovation as one sentence.","out":"$innovation"}
{"type":"extract","target":"$abstract","instruction":"Compress to 2-3 sentences retaining methodology and results.","out":"$compressed"}
{"type":"map","target":"$papers","operation":"extract","instruction":"State the main contribution in one sentence.","out":"$contributions"}
```
